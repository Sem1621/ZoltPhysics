//! Port of: Jolt/Physics/Constraints/PathConstraint.h, Jolt/Physics/Constraints/PathConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PathConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint): TODO(debug_renderer).
//!
//! - The settings own a reference to their path (`path: RefConst(PathConstraintPath)`, Jolt's
//!   `RefConst<PathConstraintPath> mPath`): `destruct` releases it, `initCopy` adds one. The constraint holds its own
//!   reference (`SetPath`), also released in `destruct`.
//! - `EPathRotationConstraintType` is an `enum class` with the underlying type int, which the binary state of the
//!   settings writes: `PathRotationConstraintType` is a non-exhaustive `enum(i32)`, a restored value outside the named
//!   ones is valid, and the switches on it have a `_` prong that does what Jolt's switch does when no case matches
//!   (nothing). The same holds for `EMotorState` (MotorSettings.zig).
//! - SaveState writes `mPositionMotorSettings` with `inStream.Write(T)`, the raw bytes of the C++ struct (Jolt's only
//!   raw write of a MotorSettings): the mode byte, three bytes of padding, the frequency / stiffness, the damping and the
//!   four limits, 28 bytes (`MotorSettingsBytes`). Zolt writes zeros as padding, which is what the C++ build writes
//!   (the parity compares the bytes); a restored stream with other padding bytes is accepted, its padding is not kept.
//! - `GetConstraintSettings` is not implemented in Jolt (`JPH_ASSERT(false); return nullptr;`): with asserts enabled it
//!   asserts, without it returns an empty `Ref` (PhysicsScene.fromPhysicsSystem then adds the constraint without
//!   settings, like Jolt's release build). Where Jolt dereferences a null path (SaveBinaryState of settings
//!   without a path, the solver functions called directly on a constraint without a path, which the ConstraintManager
//!   never does: it is not active), Zolt panics with a message.
//! - The asserts on user input (a motor state other than Off with invalid motor settings, a target path fraction
//!   outside the path, a path fraction inside the path while the limits are active after a SetPath or RestoreState
//!   between the solver calls) only run with `Core.enable_asserts`, so ReleaseFast keeps Jolt's release behavior.
//! - The const and non-const overloads `GetPositionMotorSettings()` are `getPositionMotorSettings` (`*const`) and
//!   `getPositionMotorSettingsMut` (`*`). `GetTotalLambdaPosition` / `GetTotalLambdaRotationHinge` return Jolt's
//!   `Vector<2>` by value (`Vector(2)`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vector = @import("../../Math/Vector.zig").Vector;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const ConstraintFile = @import("Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const ConstraintSubType = ConstraintFile.ConstraintSubType;
const TwoBodyConstraintFile = @import("TwoBodyConstraint.zig");
const TwoBodyConstraint = TwoBodyConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = TwoBodyConstraintFile.TwoBodyConstraintSettings;
const PathConstraintPath = @import("PathConstraintPath.zig").PathConstraintPath;
const MotorSettingsFile = @import("MotorSettings.zig");
const MotorSettings = MotorSettingsFile.MotorSettings;
const MotorState = MotorSettingsFile.MotorState;
const SpringSettings = @import("SpringSettings.zig").SpringSettings;
const AxisConstraintPart = @import("ConstraintPart/AxisConstraintPart.zig").AxisConstraintPart;
const DualAxisConstraintPart = @import("ConstraintPart/DualAxisConstraintPart.zig").DualAxisConstraintPart;
const HingeRotationConstraintPart = @import("ConstraintPart/HingeRotationConstraintPart.zig").HingeRotationConstraintPart;
const RotationEulerConstraintPart = @import("ConstraintPart/RotationEulerConstraintPart.zig").RotationEulerConstraintPart;

/// How to constrain the rotation of the body to a PathConstraint
/// (EPathRotationConstraintType: Jolt's enum class has the underlying type int, which the binary state of the settings
/// writes; like MotorState and ConstraintSpace every int value is valid, a restored value outside the named ones is
/// not illegal behavior: a switch on it needs a `_` prong that does what Jolt's switch does when no case matches)
pub const PathRotationConstraintType = enum(i32) {
    /// Do not constrain the rotation of the body at all
    free,
    /// Only allow rotation around the tangent vector (following the path)
    constrain_around_tangent,
    /// Only allow rotation around the normal vector (perpendicular to the path)
    constrain_around_normal,
    /// Only allow rotation around the binormal vector (perpendicular to the path)
    constrain_around_binormal,
    /// Fully constrain the rotation of body 2 to the path (following the tangent and normal of the path)
    constrain_to_path,
    /// Fully constrain the rotation of the body 2 to the rotation of body 1
    fully_constrained,
    /// Any other int value
    _,
};

/// RMat44 * Mat44 (Jolt's `operator *` of Mat44 / DMat44 with a Mat44)
fn mulRMat44Mat44(a: RMat44, b: Mat44) RMat44 {
    return if (Core.double_precision) a.mulMat44(b) else a.mul(b);
}

/// Path constraint settings, used to constrain the degrees of freedom between two bodies to a path
///
/// The requirements of the path are that:
/// * Tangent, normal and bi-normal form an orthonormal basis with: tangent cross bi-normal = normal
/// * The path points along the tangent vector
/// * The path is continuous so doesn't contain any sharp corners
///
/// The reason for all this is that the constraint acts like a slider constraint with the sliding axis being the tangent vector (the assumption here is that delta time will be small enough so that the path is linear for that delta time).
pub const PathConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PathConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "PathConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .attr("mPath", .path),
            .attr("mPathPosition", .path_position),
            .attr("mPathRotation", .path_rotation),
            .attr("mPathFraction", .path_fraction),
            .attr("mMaxFrictionForce", .max_friction_force),
            .attr("mPositionMotorSettings", .position_motor_settings),
            .enumAttr("mRotationConstraintType", .rotation_constraint_type),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// The path that constrains the two bodies
    path: RefConst(PathConstraintPath) = .empty,

    /// The position of the path start relative to world transform of body 1
    path_position: Vec3 = Vec3.zero(),

    /// The rotation of the path start relative to world transform of body 1
    path_rotation: Quat = Quat.identity(),

    /// The fraction along the path that corresponds to the initial position of body 2. Usually this is 0, the beginning of the path. But if you want to start an object halfway the path you can calculate this with mPath->GetClosestPoint(point on path to attach body to).
    path_fraction: f32 = 0.0,

    /// Maximum amount of friction force to apply (N) when not driven by a motor.
    max_friction_force: f32 = 0.0,

    /// In case the constraint is powered, this determines the motor settings along the path
    position_motor_settings: MotorSettings = .{},

    /// How to constrain the rotation of the body to the path
    rotation_constraint_type: PathRotationConstraintType = .free,

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) PathConstraintSettings {
        return .{ .base = .init(PathConstraintSettings, allocator) };
    }

    /// new PathConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*PathConstraintSettings {
        const self = try allocator.create(PathConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other` (and adds a reference to its path), not its
    /// reference count
    pub fn initCopy(allocator: Allocator, other: *const PathConstraintSettings) PathConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        result.path = other.path.clone();
        return result;
    }

    /// ~PathConstraintSettings: releases the path
    pub fn destruct(self: *PathConstraintSettings) void {
        self.path.deinit();
    }

    pub fn asSettings(self: *PathConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const PathConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *PathConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const PathConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const PathConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        const path = self.path.get() orelse @panic("PathConstraintSettings::SaveBinaryState: mPath is null (Jolt dereferences a null pointer)");
        path.saveBinaryState(stream);
        stream.write(self.path_position);
        stream.write(self.path_rotation);
        stream.write(self.path_fraction);
        stream.write(self.max_friction_force);
        stream.write(self.rotation_constraint_type);
        self.position_motor_settings.saveBinaryState(stream);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const PathConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try PathConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *PathConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        var result = try PathConstraintPath.restoreFromBinaryState(self.base.base.allocator, stream);
        defer result.deinit();
        if (!result.hasError())
            self.path.set(result.getPtr());
        stream.read(&self.path_position);
        stream.read(&self.path_rotation);
        stream.read(&self.path_fraction);
        stream.read(&self.max_friction_force);
        stream.read(&self.rotation_constraint_type);
        self.position_motor_settings.restoreBinaryState(stream);
    }
};

/// The bytes of a MotorSettings as `StateRecorder::Write(mPositionMotorSettings)` writes them: the C++ struct with the
/// padding after the mode byte of its SpringSettings (sizeof(MotorSettings) == 28, see the file comment)
const MotorSettingsBytes = extern struct {
    mode: u8,
    padding: [3]u8 = @splat(0),
    frequency: f32,
    damping: f32,
    min_force_limit: f32,
    max_force_limit: f32,
    min_torque_limit: f32,
    max_torque_limit: f32,

    comptime {
        std.debug.assert(@sizeOf(MotorSettingsBytes) == 28);
    }

    fn init(m: *const MotorSettings) MotorSettingsBytes {
        return .{
            .mode = @intFromEnum(m.spring_settings.mode),
            .frequency = m.spring_settings.frequency,
            .damping = m.spring_settings.damping,
            .min_force_limit = m.min_force_limit,
            .max_force_limit = m.max_force_limit,
            .min_torque_limit = m.min_torque_limit,
            .max_torque_limit = m.max_torque_limit,
        };
    }

    fn toMotorSettings(self: *const MotorSettingsBytes) MotorSettings {
        return .{
            .spring_settings = .{ .mode = @enumFromInt(self.mode), .frequency = self.frequency, .damping = self.damping },
            .min_force_limit = self.min_force_limit,
            .max_force_limit = self.max_force_limit,
            .min_torque_limit = self.min_torque_limit,
            .max_torque_limit = self.max_torque_limit,
        };
    }
};

/// Path constraint, used to constrain the degrees of freedom between two bodies to a path
pub const PathConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .isActive, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    pub const Vec2 = Vector(2);

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    /// The path that attaches the two bodies
    path: RefConst(PathConstraintPath) = .empty,
    /// Transform that takes a quantity from path space to body 1 center of mass space
    path_to_body1: Mat44 = undefined,
    /// Transform that takes a quantity from path space to body 2 center of mass space
    path_to_body2: Mat44 = undefined,
    /// How to constrain the rotation of the path
    rotation_constraint_type: PathRotationConstraintType,

    // Friction
    max_friction_force: f32,

    // Motor controls
    position_motor_settings: MotorSettings,
    position_motor_state: MotorState = .off,
    target_velocity: f32 = 0.0,
    target_path_fraction: f32 = 0.0,

    // RUN TIME PROPERTIES FOLLOW

    // Positions where the point constraint acts on in world space
    r1: Vec3 = undefined,
    r2: Vec3 = undefined,

    // X2 + R2 - X1 - R1
    u: Vec3 = undefined,

    // World space path tangent
    path_tangent: Vec3 = undefined,

    // Normals to the path tangent
    path_normal: Vec3 = undefined,
    path_binormal: Vec3 = undefined,

    // Inverse of initial rotation from body 1 to body 2 in body 1 space (only used when rotation constraint type is FullyConstrained)
    inv_initial_orientation: Quat = undefined,

    // Current fraction along the path where body 2 is attached
    path_fraction: f32 = 0.0,

    // Translation constraint parts
    /// Constraint part that keeps the movement along the tangent of the path
    position_constraint_part: DualAxisConstraintPart = .{},
    /// Constraint part that prevents movement beyond the beginning and end of the path
    position_limits_constraint_part: AxisConstraintPart = .{},
    /// Constraint to drive the object along the path or to apply friction
    position_motor_constraint_part: AxisConstraintPart = .{},

    // Rotation constraint parts
    /// Constraint part that removes 2 degrees of rotation freedom
    hinge_constraint_part: HingeRotationConstraintPart = .{},
    /// Constraint part that removes all rotational freedom
    rotation_constraint_part: RotationEulerConstraintPart = .{},

    /// Construct point constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const PathConstraintSettings) PathConstraint {
        var self: PathConstraint = .{
            .base = .init(PathConstraint, allocator, body1, body2, &settings.base),
            .rotation_constraint_type = settings.rotation_constraint_type,
            .max_friction_force = settings.max_friction_force,
            .position_motor_settings = settings.position_motor_settings,
        };

        // Calculate transform that takes us from the path start to center of mass space of body 1
        self.path_to_body1 = Mat44.rotationTranslation(settings.path_rotation, settings.path_position.sub(body1.getShape().getCenterOfMass()));

        self.setPath(settings.path.get(), settings.path_fraction);

        return self;
    }

    /// new PathConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const PathConstraintSettings) Allocator.Error!*PathConstraint {
        const self = try allocator.create(PathConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    /// ~PathConstraint: releases the path
    pub fn destruct(self: *PathConstraint) void {
        self.path.deinit();
    }

    pub fn asConstraint(self: *PathConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const PathConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *PathConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const PathConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const PathConstraint) ConstraintSubType {
        _ = self;
        return .path;
    }

    pub fn notifyShapeChanged(self: *PathConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.path_to_body1.setTranslation(self.path_to_body1.getTranslation().sub(delta_com));
        } else if (self.base.body2.getID().eql(body_id)) {
            self.path_to_body2.setTranslation(self.path_to_body2.getTranslation().sub(delta_com));
        }
    }

    pub fn setupVelocityConstraint(self: *PathConstraint, delta_time: f32) void {
        self.calculateConstraintProperties(delta_time);
    }

    pub fn resetWarmStart(self: *PathConstraint) void {
        self.position_motor_constraint_part.deactivate();
        self.position_constraint_part.deactivate();
        self.position_limits_constraint_part.deactivate();
        self.hinge_constraint_part.deactivate();
        self.rotation_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *PathConstraint, warm_start_impulse_ratio: f32) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Warm starting: Apply previous frame impulse
        self.position_motor_constraint_part.warmStart(body1, body2, self.path_tangent, warm_start_impulse_ratio);
        self.position_constraint_part.warmStart(body1, body2, self.path_normal, self.path_binormal, warm_start_impulse_ratio);
        self.position_limits_constraint_part.warmStart(body1, body2, self.path_tangent, warm_start_impulse_ratio);

        switch (self.rotation_constraint_type) {
            .free => {
                // No rotational limits
            },

            .constrain_around_tangent, .constrain_around_normal, .constrain_around_binormal => self.hinge_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio),

            .constrain_to_path, .fully_constrained => self.rotation_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio),

            // A restored int that is no EPathRotationConstraintType: no case of Jolt's switch matches
            _ => {},
        }
    }

    pub fn solveVelocityConstraint(self: *PathConstraint, delta_time: f32) bool {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Solve motor
        var motor = false;
        if (self.position_motor_constraint_part.isActive()) {
            switch (self.position_motor_state) {
                .off => {
                    const max_lambda = self.max_friction_force * delta_time;
                    motor = self.position_motor_constraint_part.solveVelocityConstraint(body1, body2, self.path_tangent, -max_lambda, max_lambda);
                },

                .velocity, .position, .position_and_velocity => motor = self.position_motor_constraint_part.solveVelocityConstraint(body1, body2, self.path_tangent, delta_time * self.position_motor_settings.min_force_limit, delta_time * self.position_motor_settings.max_force_limit),

                // A restored int that is no EMotorState: no case of Jolt's switch matches
                _ => {},
            }
        }

        // Solve position constraint along 2 axis
        const pos = self.position_constraint_part.solveVelocityConstraint(body1, body2, self.path_normal, self.path_binormal);

        // Solve limits along path axis
        var limit = false;
        if (self.position_limits_constraint_part.isActive()) {
            if (self.path_fraction <= 0.0) {
                limit = self.position_limits_constraint_part.solveVelocityConstraint(body1, body2, self.path_tangent, 0, math.flt_max);
            } else {
                // A SetPath or RestoreState between the solver calls can violate this: Jolt's release build continues
                if (Core.enable_asserts) std.debug.assert(self.path_fraction >= self.path.get().?.getPathMaxFraction());
                limit = self.position_limits_constraint_part.solveVelocityConstraint(body1, body2, self.path_tangent, -math.flt_max, 0);
            }
        }

        // Solve rotational constraint
        // Note, this is not entirely correct, we should apply a velocity constraint so that the body will actually follow the path
        // by looking at the derivative of the tangent, normal or binormal but we don't. This means the position constraint solver
        // will need to correct the orientation error that builds up, which in turn means that the simulation is not physically correct.
        var rot = false;
        switch (self.rotation_constraint_type) {
            .free => {
                // No rotational limits
            },

            .constrain_around_tangent, .constrain_around_normal, .constrain_around_binormal => rot = self.hinge_constraint_part.solveVelocityConstraint(body1, body2),

            .constrain_to_path, .fully_constrained => rot = self.rotation_constraint_part.solveVelocityConstraint(body1, body2),

            // A restored int that is no EPathRotationConstraintType: no case of Jolt's switch matches
            _ => {},
        }

        return motor or pos or limit or rot;
    }

    pub fn solvePositionConstraint(self: *PathConstraint, delta_time: f32, baumgarte: f32) bool {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Update constraint properties (bodies may have moved)
        self.calculateConstraintProperties(delta_time);

        // Solve position constraint along 2 axis
        const pos = self.position_constraint_part.solvePositionConstraint(body1, body2, self.u, self.path_normal, self.path_binormal, baumgarte);

        // Solve limits along path axis
        var limit = false;
        if (self.position_limits_constraint_part.isActive()) {
            if (self.path_fraction <= 0.0) {
                limit = self.position_limits_constraint_part.solvePositionConstraint(body1, body2, self.path_tangent, self.u.dot(self.path_tangent), baumgarte);
            } else {
                // Always true here: CalculateConstraintProperties just activated the limits for this fraction
                if (Core.enable_asserts) std.debug.assert(self.path_fraction >= self.path.get().?.getPathMaxFraction());
                limit = self.position_limits_constraint_part.solvePositionConstraint(body1, body2, self.path_tangent, self.u.dot(self.path_tangent), baumgarte);
            }
        }

        // Solve rotational constraint
        var rot = false;
        switch (self.rotation_constraint_type) {
            .free => {
                // No rotational limits
            },

            .constrain_around_tangent, .constrain_around_normal, .constrain_around_binormal => rot = self.hinge_constraint_part.solvePositionConstraint(body1, body2, baumgarte),

            .constrain_to_path, .fully_constrained => rot = self.rotation_constraint_part.solvePositionConstraint(body1, body2, self.inv_initial_orientation, baumgarte),

            // A restored int that is no EPathRotationConstraintType: no case of Jolt's switch matches
            _ => {},
        }

        return pos or limit or rot;
    }

    // TODO(debug_renderer): DrawConstraint

    pub fn saveState(self: *const PathConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.position_constraint_part.saveState(stream);
        self.position_limits_constraint_part.saveState(stream);
        self.position_motor_constraint_part.saveState(stream);
        self.hinge_constraint_part.saveState(stream);
        self.rotation_constraint_part.saveState(stream);

        stream.write(self.max_friction_force);
        stream.write(MotorSettingsBytes.init(&self.position_motor_settings));
        stream.write(self.position_motor_state);
        stream.write(self.target_velocity);
        stream.write(self.target_path_fraction);
        stream.write(self.path_fraction);
    }

    pub fn restoreState(self: *PathConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.position_constraint_part.restoreState(stream);
        self.position_limits_constraint_part.restoreState(stream);
        self.position_motor_constraint_part.restoreState(stream);
        self.hinge_constraint_part.restoreState(stream);
        self.rotation_constraint_part.restoreState(stream);

        stream.read(&self.max_friction_force);
        var motor_settings = MotorSettingsBytes.init(&self.position_motor_settings); // The current value, for a validating stream
        stream.read(&motor_settings);
        self.position_motor_settings = motor_settings.toMotorSettings();
        stream.read(&self.position_motor_state);
        stream.read(&self.target_velocity);
        stream.read(&self.target_path_fraction);
        stream.read(&self.path_fraction);
    }

    pub fn isActive(self: *const PathConstraint) bool {
        return TwoBodyConstraint.impl.isActive(&self.base) and self.path.get() != null;
    }

    pub fn getConstraintSettings(self: *const PathConstraint) Allocator.Error!Ref(ConstraintSettings) {
        _ = self;
        if (Core.enable_asserts) Core.assertFailed("Not implemented yet", @src());
        return .empty;
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const PathConstraint) Mat44 {
        return self.path_to_body1;
    }

    pub fn getConstraintToBody2Matrix(self: *const PathConstraint) Mat44 {
        return self.path_to_body2;
    }

    /// Update the path for this constraint
    pub fn setPath(self: *PathConstraint, path: ?*const PathConstraintPath, path_fraction: f32) void {
        self.path.set(path);
        self.path_fraction = path_fraction;

        if (self.path.get()) |p| {
            const body1 = self.base.body1;
            const body2 = self.base.body2;

            // Get the point on the path for this fraction
            const point = p.getPointOnPath(self.path_fraction);

            // Construct the matrix that takes us from the closest point on the path to body 2 center of mass space
            const closest_point_to_path = Mat44.init(Vec4.fromVec3W(point.tangent, 0), Vec4.fromVec3W(point.binormal, 0), Vec4.fromVec3W(point.normal, 0), Vec4.fromVec3W(point.position, 1));
            const cp_to_body1 = self.path_to_body1.mul(closest_point_to_path);
            self.path_to_body2 = body2.getInverseCenterOfMassTransform().mul(body1.getCenterOfMassTransform()).toMat44().mul(cp_to_body1);

            // Calculate initial orientation
            if (self.rotation_constraint_type == .fully_constrained)
                self.inv_initial_orientation = RotationEulerConstraintPart.getInvInitialOrientation(body1, body2);
        }
    }

    /// Access to the current path
    pub fn getPath(self: *const PathConstraint) ?*const PathConstraintPath {
        return self.path.get();
    }

    /// Access to the current fraction along the path e [0, GetPath()->GetMaxPathFraction()]
    pub fn getPathFraction(self: *const PathConstraint) f32 {
        return self.path_fraction;
    }

    /// Friction control
    pub fn setMaxFrictionForce(self: *PathConstraint, friction_force: f32) void {
        self.max_friction_force = friction_force;
    }

    pub fn getMaxFrictionForce(self: *const PathConstraint) f32 {
        return self.max_friction_force;
    }

    /// Position motor settings
    pub fn getPositionMotorSettingsMut(self: *PathConstraint) *MotorSettings {
        return &self.position_motor_settings;
    }

    pub fn getPositionMotorSettings(self: *const PathConstraint) *const MotorSettings {
        return &self.position_motor_settings;
    }

    // Position motor controls (drives body 2 along the path)
    pub fn setPositionMotorState(self: *PathConstraint, state: MotorState) void {
        // Settings of the user: Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(state == .off or self.position_motor_settings.isValid());
        self.position_motor_state = state;
    }

    pub fn getPositionMotorState(self: *const PathConstraint) MotorState {
        return self.position_motor_state;
    }

    pub fn setTargetVelocity(self: *PathConstraint, velocity: f32) void {
        self.target_velocity = velocity;
    }

    pub fn getTargetVelocity(self: *const PathConstraint) f32 {
        return self.target_velocity;
    }

    pub fn setTargetPathFraction(self: *PathConstraint, fraction: f32) void {
        // Input of the user: Jolt's release build continues
        if (Core.enable_asserts) {
            const path = self.path.get().?;
            std.debug.assert(path.isLooping() or (fraction >= 0.0 and fraction <= path.getPathMaxFraction()));
        }
        self.target_path_fraction = fraction;
    }

    pub fn getTargetPathFraction(self: *const PathConstraint) f32 {
        return self.target_path_fraction;
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const PathConstraint) Vec2 {
        return self.position_constraint_part.getTotalLambda().*;
    }

    pub fn getTotalLambdaPositionLimits(self: *const PathConstraint) f32 {
        return self.position_limits_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaMotor(self: *const PathConstraint) f32 {
        return self.position_motor_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaRotationHinge(self: *const PathConstraint) Vec2 {
        return self.hinge_constraint_part.getTotalLambda().*;
    }

    pub fn getTotalLambdaRotation(self: *const PathConstraint) Vec3 {
        return self.rotation_constraint_part.getTotalLambda();
    }

    /// `mPath->` in the solver functions (Zolt: Jolt dereferences a null pointer when the solver runs on a constraint
    /// without a path, which the ConstraintManager never does: IsActive is false without a path)
    fn getPathChecked(self: *const PathConstraint) *const PathConstraintPath {
        return self.path.get() orelse @panic("PathConstraint: the solver runs without a path (Jolt dereferences a null pointer)");
    }

    // Internal helper function to calculate the values below
    fn calculateConstraintProperties(self: *PathConstraint, delta_time: f32) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;
        const path = self.getPathChecked();

        // Get transforms of body 1 and 2
        const transform1 = body1.getCenterOfMassTransform();
        const transform2 = body2.getCenterOfMassTransform();

        // Get the transform of the path transform as seen from body 1 in world space
        const path_to_world_1 = mulRMat44Mat44(transform1, self.path_to_body1);

        // Get the transform of from the point on path that body 2 is attached to in world space
        const path_to_world_2 = mulRMat44Mat44(transform2, self.path_to_body2);

        // Calculate new closest point on path
        const position2 = path_to_world_2.getTranslation();
        const position2_local_to_path = path_to_world_1.inversedRotationTranslation().mulRVec3(position2).toVec3();
        self.path_fraction = path.getClosestPoint(position2_local_to_path, self.path_fraction);

        // Get the point on the path for this fraction
        const point = path.getPointOnPath(self.path_fraction);
        const path_point = point.position;
        const path_tangent = point.tangent;
        const path_normal = point.normal;
        const path_binormal = point.binormal;

        // Calculate R1 and R2
        const path_point_ws: RVec3 = path_to_world_1.mulVec3(path_point);
        self.r1 = path_point_ws.sub(body1.getCenterOfMassPosition()).toVec3();
        self.r2 = position2.sub(body2.getCenterOfMassPosition()).toVec3();

        // Calculate U = X2 + R2 - X1 - R1
        self.u = position2.sub(path_point_ws).toVec3();

        // Calculate world space normals
        self.path_normal = path_to_world_1.multiply3x3(path_normal);
        self.path_binormal = path_to_world_1.multiply3x3(path_binormal);

        // Calculate slide axis
        self.path_tangent = path_to_world_1.multiply3x3(path_tangent);

        // Prepare constraint part for position constraint to slide along the path
        self.position_constraint_part.calculateConstraintProperties(body1, transform1.getRotation(), self.r1.add(self.u), body2, transform2.getRotation(), self.r2, self.path_normal, self.path_binormal);

        // Check if closest point is on the boundary of the path and if so apply limit
        if (!path.isLooping() and (self.path_fraction <= 0.0 or self.path_fraction >= path.getPathMaxFraction()))
            self.position_limits_constraint_part.calculateConstraintProperties(body1, self.r1.add(self.u), body2, self.r2, self.path_tangent, .{})
        else
            self.position_limits_constraint_part.deactivate();

        // Prepare rotation constraint part
        switch (self.rotation_constraint_type) {
            .free => {
                // No rotational limits
            },

            .constrain_around_tangent => self.hinge_constraint_part.calculateConstraintProperties(body1, transform1.getRotation(), self.path_tangent, body2, transform2.getRotation(), path_to_world_2.getAxisX()),

            .constrain_around_normal => self.hinge_constraint_part.calculateConstraintProperties(body1, transform1.getRotation(), self.path_normal, body2, transform2.getRotation(), path_to_world_2.getAxisZ()),

            .constrain_around_binormal => self.hinge_constraint_part.calculateConstraintProperties(body1, transform1.getRotation(), self.path_binormal, body2, transform2.getRotation(), path_to_world_2.getAxisY()),

            .constrain_to_path, .fully_constrained => |rotation_type| {
                if (rotation_type == .constrain_to_path) {
                    // We need to calculate the inverse of the rotation from body 1 to body 2 for the current path position (see: RotationEulerConstraintPart::sGetInvInitialOrientation)
                    // RotationBody2 = RotationBody1 * InitialOrientation <=> InitialOrientation^-1 = RotationBody2^-1 * RotationBody1
                    // We can express RotationBody2 in terms of RotationBody1: RotationBody2 = RotationBody1 * PathToBody1 * RotationClosestPointOnPath * PathToBody2^-1
                    // Combining these two: InitialOrientation^-1 = PathToBody2 * (PathToBody1 * RotationClosestPointOnPath)^-1
                    self.inv_initial_orientation = self.path_to_body2.multiply3x3RightTransposed(self.path_to_body1.multiply3x3Mat44(Mat44.init(Vec4.fromVec3W(path_tangent, 0), Vec4.fromVec3W(path_binormal, 0), Vec4.fromVec3W(path_normal, 0), Vec4.zero()))).getQuaternion();
                    // [[fallthrough]]
                }

                self.rotation_constraint_part.calculateConstraintProperties(body1, transform1.getRotation(), body2, transform2.getRotation());
            },

            // A restored int that is no EPathRotationConstraintType: no case of Jolt's switch matches
            _ => {},
        }

        // Motor properties
        switch (self.position_motor_state) {
            .off => {
                if (self.max_friction_force > 0.0)
                    self.position_motor_constraint_part.calculateConstraintProperties(body1, self.r1.add(self.u), body2, self.r2, self.path_tangent, .{})
                else
                    self.position_motor_constraint_part.deactivate();
            },

            .velocity => self.position_motor_constraint_part.calculateConstraintProperties(body1, self.r1.add(self.u), body2, self.r2, self.path_tangent, .{ .bias = -self.target_velocity }),

            .position => {
                if (self.position_motor_settings.spring_settings.hasStiffness())
                    self.position_motor_constraint_part.calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, self.r1.add(self.u), body2, self.r2, self.path_tangent, 0.0, self.calculateConstraintValue(), &self.position_motor_settings.spring_settings)
                else
                    self.position_motor_constraint_part.deactivate();
            },

            .position_and_velocity => {
                if (self.position_motor_settings.spring_settings.hasStiffnessOrDamping())
                    self.position_motor_constraint_part.calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, self.r1.add(self.u), body2, self.r2, self.path_tangent, -self.target_velocity, self.calculateConstraintValue(), &self.position_motor_settings.spring_settings)
                else
                    self.position_motor_constraint_part.deactivate();
            },

            // A restored int that is no EMotorState: no case of Jolt's switch matches
            _ => {},
        }
    }

    // Internal helper function to calculate the current constraint value C
    fn calculateConstraintValue(self: *const PathConstraint) f32 {
        const path = self.getPathChecked();
        var c: f32 = undefined;
        if (path.isLooping()) {
            const max_fraction = path.getPathMaxFraction();
            c = @rem(self.path_fraction - self.target_path_fraction, max_fraction); // fmod
            const half_max_fraction = 0.5 * max_fraction;
            if (c > half_max_fraction)
                c -= max_fraction
            else if (c < -half_max_fraction)
                c += max_fraction;
        } else c = self.path_fraction - self.target_path_fraction;
        return c;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/PathConstraintTests.zig)

const testing = std.testing;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const TestContext = @import("ConstraintPart/ConstraintPartTestContext.zig");
const virtual = @import("../../Core/Virtual.zig");
const PathConstraintPathHermite = @import("PathConstraintPathHermite.zig").PathConstraintPathHermite;

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

/// A Hermite path on the heap (reference count 0): an arc in the XZ plane with normal Y, `num` points, looping or not
fn testPath(allocator: Allocator, num: u32, looping: bool) Allocator.Error!*PathConstraintPathHermite {
    const path = try PathConstraintPathHermite.create(allocator);
    errdefer {
        path.base.addRef();
        path.base.release();
    }
    path.base.setIsLooping(looping);
    const arc: f32 = if (looping) 2.0 * math.pi else 0.5 * math.pi;
    const segments: f32 = @floatFromInt(if (looping) num else num - 1);
    const radius: f32 = 3.0;
    for (0..num) |p| {
        const a = arc * @as(f32, @floatFromInt(p)) / segments;
        const sin_cos = Vec4.replicate(a).sinCos();
        const s = sin_cos.sin.getX();
        const c = sin_cos.cos.getX();
        try path.addPoint(Vec3.init(radius * c, 0, radius * s), Vec3.init(-s, 0, c).mulScalar(radius * arc / segments), Vec3.axisY());
    }
    return path;
}

test "PathRotationConstraintType has Jolt's values" {
    try testing.expectEqual(@as(i32, 0), @intFromEnum(PathRotationConstraintType.free));
    try testing.expectEqual(@as(i32, 1), @intFromEnum(PathRotationConstraintType.constrain_around_tangent));
    try testing.expectEqual(@as(i32, 2), @intFromEnum(PathRotationConstraintType.constrain_around_normal));
    try testing.expectEqual(@as(i32, 3), @intFromEnum(PathRotationConstraintType.constrain_around_binormal));
    try testing.expectEqual(@as(i32, 4), @intFromEnum(PathRotationConstraintType.constrain_to_path));
    try testing.expectEqual(@as(i32, 5), @intFromEnum(PathRotationConstraintType.fully_constrained));
    try testing.expectEqual(@as(usize, 4), @sizeOf(PathRotationConstraintType)); // Written as an int by the binary state
}

test "PathConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expect(settings.path.get() == null);
    try testing.expect(settings.path_position.eql(Vec3.zero()));
    try testing.expect(settings.path_rotation.eql(Quat.identity()));
    try testing.expectEqual(@as(f32, 0.0), settings.path_fraction);
    try testing.expectEqual(@as(f32, 0.0), settings.max_friction_force);
    try testing.expectEqual(SpringSettings.init(.frequency_and_damping, 2.0, 1.0), settings.position_motor_settings.spring_settings);
    try testing.expectEqual(-math.flt_max, settings.position_motor_settings.min_force_limit);
    try testing.expectEqual(math.flt_max, settings.position_motor_settings.max_torque_limit);
    try testing.expectEqual(PathRotationConstraintType.free, settings.rotation_constraint_type);
    try testing.expectEqualStrings("PathConstraintSettings", settings.asSettings().vtable.rtti.name);

    // Non default values in every field
    const path = try testPath(allocator, 4, true);
    settings.path.set(path.asPath());
    try testing.expectEqual(@as(u32, 1), path.base.getRefCount());
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 9;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 4;
    settings.base.base.draw_constraint_size = 0.5;
    settings.path_position = Vec3.init(1, 2, 3);
    settings.path_rotation = Quat.rotation(Vec3.axisZ(), 0.25);
    settings.path_fraction = 1.5;
    settings.max_friction_force = 7.0;
    settings.position_motor_settings = .initModeLimits(.stiffness_and_damping, 100.0, 3.0, 50.0, 60.0);
    settings.rotation_constraint_type = .constrain_to_path;

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), the path (hash, looping, the points),
    // the path position (3 floats), the rotation (4 floats), fraction, friction, rotation type (int), the motor
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    const path_size = 4 + 1 + 4 + 4 * 48;
    try testing.expectEqual(@as(usize, 21 + path_size + 12 + 16 + 4 + 4 + 4 + 25), bytes.len);
    try testing.expectEqual(RTTI.hashName("PathConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(RTTI.hashName("PathConstraintPathHermite"), std.mem.readInt(u32, bytes[21..25], .little));
    try testing.expectEqual(@as(i32, 4), std.mem.readInt(i32, bytes[21 + path_size + 36 ..][0..4], .little)); // ConstrainToPath

    // Restore through RegisterTypes.factory (the settings and the path)
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(PathConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 9), restored.base.base.constraint_priority);
        const restored_path = virtual.downcast(PathConstraintPathHermite, restored.path.get().?);
        try testing.expectEqual(@as(u32, 1), restored_path.base.getRefCount()); // Only the settings hold it
        try testing.expect(restored_path.base.isLooping());
        try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(path.points.items), std.mem.sliceAsBytes(restored_path.points.items));
        try testing.expect(restored.path_position.eql(settings.path_position) and restored.path_rotation.eql(settings.path_rotation));
        try testing.expectEqual(@as(f32, 1.5), restored.path_fraction);
        try testing.expectEqual(@as(f32, 7.0), restored.max_friction_force);
        try testing.expectEqual(settings.position_motor_settings, restored.position_motor_settings);
        try testing.expectEqual(PathRotationConstraintType.constrain_to_path, restored.rotation_constraint_type);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated anywhere: Jolt's errors (a truncated path is ignored by the settings, which then fail themselves)
    for (0..bytes.len) |len| {
        var result = try testRestoreSettings(allocator, bytes[0..len]);
        defer result.deinit();
        try testing.expectEqualStrings(if (len < 4) "Failed to read type hash" else "Failed to restore object", result.getError());
    }

    // A path of an unknown type: Jolt keeps going without a path (mPath stays null) and reads the path data as the
    // following members
    {
        const corrupt = try allocator.dupe(u8, bytes);
        defer allocator.free(corrupt);
        std.mem.writeInt(u32, corrupt[21..25], 0x12345678, .little);
        var result = try testRestoreSettings(allocator, corrupt);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(PathConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(restored.path.get() == null);
    }

    // An unknown rotation type is a valid int
    {
        const other = try allocator.dupe(u8, bytes);
        defer allocator.free(other);
        std.mem.writeInt(i32, other[21 + path_size + 36 ..][0..4], 77, .little);
        var result = try testRestoreSettings(allocator, other);
        defer result.deinit();
        const restored = virtual.downcast(PathConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expectEqual(@as(i32, 77), @intFromEnum(restored.rotation_constraint_type));
    }

    // Out of memory while restoring: the error arrives, nothing leaks
    const Ctx = struct {
        fn run(a: Allocator, b: []const u8) !void {
            var result = try testRestoreSettings(a, b);
            result.deinit();
        }
    };
    try testing.checkAllAllocationFailures(allocator, Ctx.run, .{bytes});

    // The copy constructor copies the settings and adds a reference to the path, not the reference count
    settings.asSettings().setEmbedded();
    settings.asSettings().addRef();
    defer settings.asSettings().release();
    var copy = PathConstraintSettings.initCopy(allocator, &settings);
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expectEqual(@as(u32, 2), path.base.getRefCount());
    try testing.expect(copy.path.get() == path.asPathConst());
    try testing.expectEqual(@as(f32, 7.0), copy.max_friction_force);
    try testing.expectEqual(@as(u32, 9), copy.base.base.constraint_priority);
    copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 1), path.base.getRefCount());
}

test "PathConstraint: virtual dispatch, the path frames, setters, solver, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // An open arc, body 2 attached halfway, rotation constrained around the tangent, with friction
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    const path = try testPath(allocator, 5, false);
    settings.path.set(path.asPath());
    settings.path_position = Vec3.init(0.5, -1.0, 0.25);
    settings.path_rotation = Quat.rotation(Vec3.init(1, 1, 0).normalized(), 0.4);
    settings.path_fraction = 2.25;
    settings.max_friction_force = 2.0;
    settings.rotation_constraint_type = .constrain_around_tangent;
    settings.base.base.user_data = 77;

    // TwoBodyConstraintSettings::Create through the vtable
    var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
    defer constraint.deinit();
    const base = constraint.get().?;
    const two_body = virtual.downcast(TwoBodyConstraint, base);
    const c = virtual.downcast(PathConstraint, two_body);
    try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
    try testing.expectEqual(ConstraintSubType.path, base.getSubType());
    try testing.expectEqual(@as(u64, 77), base.getUserData());
    try testing.expectEqual(@as(u32, 2), path.base.getRefCount()); // The settings and the constraint
    try testing.expect(c.getPath() == path.asPathConst());
    try testing.expectEqual(@as(f32, 2.25), c.getPathFraction());
    try testing.expectEqual(@as(f32, 2.0), c.getMaxFrictionForce());

    // The path frame in body 1 (the box has its center of mass at its origin) and the attachment frame in body 2: the
    // point on the path at the start fraction, in world space, is the origin of both frames
    try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.rotationTranslation(settings.path_rotation, settings.path_position)));
    const point = path.asPathConst().getPointOnPath(2.25);
    const path_to_world = mulRMat44Mat44(body1.getCenterOfMassTransform(), two_body.getConstraintToBody1Matrix());
    const attachment = mulRMat44Mat44(body2.getCenterOfMassTransform(), two_body.getConstraintToBody2Matrix());
    try testing.expect(attachment.getTranslation().sub(path_to_world.mulVec3(point.position)).toVec3().length() < 1.0e-5);
    try testing.expect(attachment.getAxisX().isClose(path_to_world.multiply3x3(point.tangent), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(attachment.getAxisY().isClose(path_to_world.multiply3x3(point.binormal), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(attachment.getAxisZ().isClose(path_to_world.multiply3x3(point.normal), .{ .max_dist_sq = 1.0e-10 }));

    // GetConstraintSettings is not implemented in Jolt (it asserts and returns nullptr)
    {
        var expect_assert: @import("../../Core/IssueReporting.zig").ExpectAssert = undefined;
        expect_assert.init(1);
        errdefer expect_assert.restore();
        var copy = try base.getConstraintSettings();
        defer copy.deinit();
        try expect_assert.finish();
        try testing.expect(copy.get() == null);
    }

    // Setters and getters
    c.setMaxFrictionForce(3.0);
    try testing.expectEqual(@as(f32, 3.0), c.getMaxFrictionForce());
    c.getPositionMotorSettingsMut().setForceLimits(-1.0, 2.0);
    try testing.expectEqual(@as(f32, -1.0), c.getPositionMotorSettings().min_force_limit);
    try testing.expectEqual(@as(f32, 2.0), c.getPositionMotorSettings().max_force_limit);
    c.getPositionMotorSettingsMut().setForceLimit(1.0e6);
    try testing.expectEqual(MotorState.off, c.getPositionMotorState());
    c.setPositionMotorState(.velocity);
    try testing.expectEqual(MotorState.velocity, c.getPositionMotorState());
    c.setTargetVelocity(1.5);
    try testing.expectEqual(@as(f32, 1.5), c.getTargetVelocity());
    c.setTargetPathFraction(3.5);
    try testing.expectEqual(@as(f32, 3.5), c.getTargetPathFraction());

    // Solver: the velocity of the attachment point of body 2 relative to the path is along the tangent afterwards (the
    // motor drives it), the relative angular velocity is around the tangent only
    try testing.expect(base.isActive());
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expectApproxEqAbs(@as(f32, 2.25), c.getPathFraction(), 1.0e-3); // The closest point of the start position
    base.warmStartVelocityConstraint(1.0);
    for (0..500) |_| _ = base.solveVelocityConstraint(1.0 / 60.0); // Gauss-Seidel over three coupled parts converges slowly
    const anchor2 = body2.getCenterOfMassPosition().add(RVec3.fromVec3(c.r2));
    const dv = body2.getPointVelocity(anchor2).sub(body1.getPointVelocity(anchor2));
    try testing.expect(@abs(dv.dot(c.path_normal)) < 1.0e-3 and @abs(dv.dot(c.path_binormal)) < 1.0e-3);
    try testing.expectApproxEqAbs(@as(f32, 1.5), dv.dot(c.path_tangent), 1.0e-3); // The velocity motor
    const dw = body2.getAngularVelocity().sub(body1.getAngularVelocity());
    try testing.expect(@abs(dw.dot(c.path_normal)) < 1.0e-3 and @abs(dw.dot(c.path_binormal)) < 1.0e-3);
    try testing.expect(!c.getTotalLambdaPosition().isZero() and c.getTotalLambdaMotor() != 0.0 and !c.getTotalLambdaRotationHinge().isZero());
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPositionLimits()); // Not at the end of the path
    try testing.expect(c.getTotalLambdaRotation().eql(Vec3.zero())); // Not used by this rotation type
    _ = base.solvePositionConstraint(1.0 / 60.0, 0.2);

    // SaveState / RestoreState: enabled, the lambdas of the five parts, friction, the raw motor settings, the motor
    // state, the targets and the path fraction
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 1 + 8 + 4 + 4 + 8 + 12 + 4 + 28 + 4 + 4 + 4 + 4), recorder.getData().len);
    const motor_bytes = recorder.getData()[41..69];
    try testing.expectEqual(@as(u8, 0), motor_bytes[0]); // FrequencyAndDamping
    try testing.expectEqualSlices(u8, &.{ 0, 0, 0 }, motor_bytes[1..4]); // The padding of the C++ struct
    try testing.expectEqual(@as(f32, 2.0), @as(f32, @bitCast(motor_bytes[4..8].*)));
    try testing.expectEqual(@as(f32, -1.0e6), @as(f32, @bitCast(motor_bytes[12..16].*)));
    const lambda_position = c.getTotalLambdaPosition();
    const lambda_motor = c.getTotalLambdaMotor();
    const lambda_hinge = c.getTotalLambdaRotationHinge();
    const path_fraction = c.getPathFraction();
    base.resetWarmStart();
    try testing.expect(c.getTotalLambdaPosition().isZero() and c.getTotalLambdaMotor() == 0.0 and c.getTotalLambdaRotationHinge().isZero());
    c.setMaxFrictionForce(0.0);
    c.getPositionMotorSettingsMut().* = .{};
    c.setPositionMotorState(.off);
    c.setTargetVelocity(0.0);
    c.setTargetPathFraction(0.0);
    c.path_fraction = 0.0;
    base.restoreState(recorder.recorder());
    try testing.expect(c.getTotalLambdaPosition().eql(lambda_position) and c.getTotalLambdaMotor() == lambda_motor and c.getTotalLambdaRotationHinge().eql(lambda_hinge));
    try testing.expectEqual(@as(f32, 3.0), c.getMaxFrictionForce());
    try testing.expectEqual(@as(f32, -1.0e6), c.getPositionMotorSettings().min_force_limit);
    try testing.expectEqual(MotorState.velocity, c.getPositionMotorState());
    try testing.expectEqual(@as(f32, 1.5), c.getTargetVelocity());
    try testing.expectEqual(@as(f32, 3.5), c.getTargetPathFraction());
    try testing.expectEqual(path_fraction, c.getPathFraction());

    // A validating restore of the same state matches; the padding of a foreign stream is accepted (and not kept)
    {
        var validator = StateRecorderImpl.init(allocator);
        defer validator.deinit();
        base.saveState(validator.recorder());
        validator.recorder().setValidating(true);
        base.restoreState(validator.recorder());
        try testing.expectEqual(@as(u32, 0), validator.num_mismatches);

        var foreign = StateRecorderImpl.init(allocator);
        defer foreign.deinit();
        base.saveState(foreign.recorder());
        foreign.data.items[42] = 0xAB;
        base.restoreState(foreign.recorder());
        var resaved = StateRecorderImpl.init(allocator);
        defer resaved.deinit();
        base.saveState(resaved.recorder());
        try testing.expectEqual(@as(u8, 0), resaved.getData()[42]);
        try testing.expectEqualSlices(u8, recorder.getData(), resaved.getData());
    }

    // NotifyShapeChanged moves the path frame of the body that changed
    const m1 = c.path_to_body1;
    const m2 = c.path_to_body2;
    base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
    try testing.expect(c.path_to_body1.eql(m1) and c.path_to_body2.getTranslation().eql(m2.getTranslation().sub(Vec3.init(0.5, 0, 0))));
    try testing.expect(c.path_to_body2.getAxisX().eql(m2.getAxisX()));
    base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
    try testing.expect(c.path_to_body1.getTranslation().eql(m1.getTranslation().sub(Vec3.init(0, 1, 0))));
    base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
    try testing.expect(c.path_to_body1.getTranslation().eql(m1.getTranslation().sub(Vec3.init(0, 1, 0))));

    // SetPath: another path at another fraction, then no path (the constraint is not active without a path)
    const other = try testPath(allocator, 6, true);
    c.setPath(other.asPath(), 4.5);
    try testing.expectEqual(@as(u32, 1), path.base.getRefCount());
    try testing.expectEqual(@as(u32, 1), other.base.getRefCount());
    try testing.expect(c.getPath() == other.asPathConst());
    try testing.expectEqual(@as(f32, 4.5), c.getPathFraction());
    c.setTargetPathFraction(-20.0); // Any fraction on a looping path
    try testing.expect(base.isActive());
    c.setPath(null, 0.0);
    try testing.expect(c.getPath() == null);
    try testing.expect(!base.isActive());

    // A disabled constraint is not active (TwoBodyConstraint::IsActive)
    c.setPath(path.asPath(), 1.0);
    try testing.expect(base.isActive());
    base.setEnabled(false);
    try testing.expect(!base.isActive());
}

test "PathConstraint: the rotation constraint types, the motor states, the limits" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    var path = RefConst(PathConstraintPath).init((try testPath(allocator, 4, false)).asPath());
    defer path.deinit();

    for ([_]PathRotationConstraintType{ .free, .constrain_around_tangent, .constrain_around_normal, .constrain_around_binormal, .constrain_to_path, .fully_constrained, @enumFromInt(9) }) |rotation_type| {
        for ([_]MotorState{ .off, .velocity, .position, .position_and_velocity, @enumFromInt(7) }) |motor_state| {
            // Body 2 is dynamic and rotating, body 1 fixed to the world: the path is in world space, body 2 starts near the
            // middle of the path
            const start = path.get().?.getPointOnPath(1.5).position.add(Vec3.init(0.05, 0.1, -0.05));
            const body = try context.createBody(.dynamic, .{ .position = RVec3.fromVec3(start), .rotation = Quat.rotation(Vec3.axisY(), 0.2), .linear_velocity = Vec3.init(-1, 2, 3), .angular_velocity = Vec3.init(1, -2, 0.5) });
            defer context.destroyBodies(&.{body});
            var settings = PathConstraintSettings.init(allocator);
            defer settings.asSettings().deinit();
            settings.path.set(path.get());
            settings.path_fraction = 1.5;
            settings.rotation_constraint_type = rotation_type;
            settings.max_friction_force = 1.0;
            var c = PathConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
            defer c.asConstraint().deinit();
            const base = c.asConstraint();
            c.setPositionMotorState(motor_state);
            c.setTargetVelocity(-0.5);
            c.setTargetPathFraction(2.0);
            if (rotation_type == .fully_constrained)
                try testing.expect(c.inv_initial_orientation.eql(RotationEulerConstraintPart.getInvInitialOrientation(Body.fixedToWorld(), body)));

            const delta_time: f32 = 1.0 / 60.0;
            base.setupVelocityConstraint(delta_time);
            base.warmStartVelocityConstraint(1.0);
            var applied = false;
            for (0..10) |_| applied = base.solveVelocityConstraint(delta_time) or applied;
            try testing.expect(applied); // The point constraint always applies an impulse here
            _ = base.solvePositionConstraint(delta_time, 0.2);

            // The rotation parts that the type uses
            const hinge = !c.getTotalLambdaRotationHinge().isZero();
            const rotation = !c.getTotalLambdaRotation().eql(Vec3.zero());
            switch (rotation_type) {
                .constrain_around_tangent, .constrain_around_normal, .constrain_around_binormal => try testing.expect(hinge and !rotation),
                .constrain_to_path, .fully_constrained => try testing.expect(!hinge and rotation),
                else => try testing.expect(!hinge and !rotation),
            }

            // The motor part: friction when off, the motor otherwise (the spring has stiffness), nothing for an unknown
            // state (no case of Jolt's switch matches, the part stays inactive)
            const motor = c.getTotalLambdaMotor() != 0.0;
            try testing.expectEqual(@intFromEnum(motor_state) <= 3, motor);
            try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPositionLimits()); // In the middle of the path
        }
    }

    // The limits at the start of a non looping path: body 2 moves backwards, the limit stops it
    {
        const body = try context.createBody(.dynamic, .{ .position = RVec3.init(3, 0, 0), .linear_velocity = Vec3.init(0, 0, -2) });
        defer context.destroyBodies(&.{body});
        var settings = PathConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.path.set(path.get());
        var c = PathConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
        defer c.asConstraint().deinit();
        const base = c.asConstraint();
        base.setupVelocityConstraint(1.0 / 60.0);
        try testing.expectEqual(@as(f32, 0.0), c.getPathFraction());
        base.warmStartVelocityConstraint(1.0);
        try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
        try testing.expect(c.getTotalLambdaPositionLimits() > 0.0);
        try testing.expect(@abs(body.getLinearVelocity().dot(c.path_tangent)) < 1.0e-5);
    }

    // At the end: moving forward is stopped (the limit lambda is negative)
    {
        const body = try context.createBody(.dynamic, .{ .position = RVec3.init(0, 0, 3), .linear_velocity = Vec3.init(-2, 0, 0) });
        defer context.destroyBodies(&.{body});
        var settings = PathConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.path.set(path.get());
        settings.path_fraction = 3.0;
        var c = PathConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
        defer c.asConstraint().deinit();
        const base = c.asConstraint();
        base.setupVelocityConstraint(1.0 / 60.0);
        try testing.expectEqual(@as(f32, 3.0), c.getPathFraction());
        base.warmStartVelocityConstraint(1.0);
        try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
        try testing.expect(c.getTotalLambdaPositionLimits() < 0.0);
        _ = base.solvePositionConstraint(1.0 / 60.0, 0.2);
    }
}

test "PathConstraint: the position motor on a looping path takes the short way around" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    var path = RefConst(PathConstraintPath).init((try testPath(allocator, 8, true)).asPath());
    defer path.deinit();

    // Fraction 0.5, target 7.5: CalculateConstraintValue wraps the difference -7 to -7 + 8 = 1, so the motor drives body
    // 2 backwards (towards fraction 0) instead of forward through the whole circle
    const Case = struct { target: f32, expected: f32 };
    for ([_]Case{ .{ .target = 7.5, .expected = 1.0 }, .{ .target = 1.5, .expected = -1.0 }, .{ .target = -7.0, .expected = -0.5 }, .{ .target = 16.75, .expected = -0.25 } }) |case| {
        const body = try context.createBody(.dynamic, .{});
        defer context.destroyBodies(&.{body});
        var settings = PathConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.path.set(path.get());
        settings.path_fraction = 0.5;
        var c = PathConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
        defer c.asConstraint().deinit();
        c.setTargetPathFraction(case.target);
        try testing.expectEqual(case.expected, c.calculateConstraintValue());
    }

    // Non looping: the plain difference
    var open = RefConst(PathConstraintPath).init((try testPath(allocator, 4, false)).asPath());
    defer open.deinit();
    const body = try context.createBody(.dynamic, .{});
    defer context.destroyBodies(&.{body});
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.path.set(open.get());
    settings.path_fraction = 0.5;
    var c = PathConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    c.setTargetPathFraction(2.75);
    try testing.expectEqual(@as(f32, -2.25), c.calculateConstraintValue());
}

test "PathConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            const s = try PathConstraintSettings.create(allocator);
            var settings = Ref(ConstraintSettings).init(s.asSettings());
            defer settings.deinit();
            s.path.set((try testPath(allocator, 5, false)).asPath());
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, bodies[0], bodies[1])).base);
            defer constraint.deinit();
            const c = virtual.downcast(PathConstraint, virtual.downcast(TwoBodyConstraint, constraint.get().?));
            c.setPath((try testPath(allocator, 3, true)).asPath(), 1.0);
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
