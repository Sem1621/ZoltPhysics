//! Port of: Jolt/Physics/Constraints/ConeConstraint.h, Jolt/Physics/Constraints/ConeConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConeConstraintSettings): TODO(serialization) (the binary state is ported).
//! JPH_DEBUG_RENDERER (DrawConstraint, DrawConstraintLimits): TODO(debug_renderer).
//!
//! - `SetHalfConeAngle` asserts that the angle is in [0, pi] (user input): the assert only runs with
//!   `Core.enable_asserts`, ReleaseFast keeps Jolt's release behavior (it stores the cosine of any angle).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
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
const PointConstraintPart = @import("ConstraintPart/PointConstraintPart.zig").PointConstraintPart;
const AngleConstraintPart = @import("ConstraintPart/AngleConstraintPart.zig").AngleConstraintPart;

/// Cone constraint settings, used to create a cone constraint
pub const ConeConstraintSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, ConeConstraintSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "ConeConstraintSettings";
    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint reference frame (space determined by mSpace)
    point1: RVec3 = RVec3.zero(),
    twist_axis1: Vec3 = Vec3.axisX(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    point2: RVec3 = RVec3.zero(),
    twist_axis2: Vec3 = Vec3.axisX(),

    /// Half of maximum angle between twist axis of body 1 and 2
    half_cone_angle: f32 = 0.0,

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) ConeConstraintSettings {
        return .{ .base = .init(ConeConstraintSettings, allocator) };
    }

    /// new ConeConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*ConeConstraintSettings {
        const self = try allocator.create(ConeConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const ConeConstraintSettings) ConeConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *ConeConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const ConeConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *ConeConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const ConeConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const ConeConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.point1);
        stream.write(self.twist_axis1);
        stream.write(self.point2);
        stream.write(self.twist_axis2);
        stream.write(self.half_cone_angle);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const ConeConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try ConeConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *ConeConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.point1);
        stream.read(&self.twist_axis1);
        stream.read(&self.point2);
        stream.read(&self.twist_axis2);
        stream.read(&self.half_cone_angle);
    }
};

/// A cone constraint constraints 2 bodies to a single point and limits the swing between the twist axis within a cone:
///
/// t1 . t2 <= cos(theta)
///
/// Where:
///
/// t1 = twist axis of body 1.
/// t2 = twist axis of body 2.
/// theta = half cone angle (angle from the principal axis of the cone to the edge).
///
/// Calculating the Jacobian:
///
/// Constraint equation:
///
/// C = t1 . t2 - cos(theta)
///
/// Derivative:
///
/// d/dt C = d/dt (t1 . t2) = (d/dt t1) . t2 + t1 . (d/dt t2) = (w1 x t1) . t2 + t1 . (w2 x t2) = (t1 x t2) . w1 + (t2 x t1) . w2
///
/// d/dt C = J v = [0, -t2 x t1, 0, t2 x t1] [v1, w1, v2, w2]
///
/// Where J is the Jacobian.
///
/// Note that this is the exact same equation as used in AngleConstraintPart if we use t2 x t1 as the world space axis
pub const ConeConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Local space constraint axis
    local_space_twist_axis1: Vec3 = undefined,
    local_space_twist_axis2: Vec3 = undefined,

    // Angular limits
    cos_half_cone_angle: f32 = undefined,

    // RUN TIME PROPERTIES FOLLOW

    // Axis and angle of rotation between the two bodies
    world_space_rotation_axis: Vec3 = undefined,
    cos_theta: f32 = undefined,

    // The constraint parts
    point_constraint_part: PointConstraintPart = .{},
    angle_constraint_part: AngleConstraintPart = .{},

    /// Construct cone constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const ConeConstraintSettings) ConeConstraint {
        var self: ConeConstraint = .{ .base = .init(ConeConstraint, allocator, body1, body2, &settings.base) };

        // Store limits
        self.setHalfConeAngle(settings.half_cone_angle);

        // Initialize rotation axis to perpendicular of twist axis in case the angle between the twist axis is 0 in the first frame
        self.world_space_rotation_axis = settings.twist_axis1.getNormalizedPerpendicular();

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            const inv_transform1 = body1.getInverseCenterOfMassTransform();
            self.local_space_position1 = inv_transform1.mulRVec3(settings.point1).toVec3();
            self.local_space_twist_axis1 = inv_transform1.multiply3x3(settings.twist_axis1);

            const inv_transform2 = body2.getInverseCenterOfMassTransform();
            self.local_space_position2 = inv_transform2.mulRVec3(settings.point2).toVec3();
            self.local_space_twist_axis2 = inv_transform2.multiply3x3(settings.twist_axis2);
        } else {
            // Properties already in local space
            self.local_space_position1 = settings.point1.toVec3();
            self.local_space_position2 = settings.point2.toVec3();
            self.local_space_twist_axis1 = settings.twist_axis1;
            self.local_space_twist_axis2 = settings.twist_axis2;

            // If they were in local space, we need to take the initial rotation axis to world space
            self.world_space_rotation_axis = body1.getRotation().mulVec3(self.world_space_rotation_axis);
        }

        return self;
    }

    /// new ConeConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const ConeConstraintSettings) Allocator.Error!*ConeConstraint {
        const self = try allocator.create(ConeConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *ConeConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const ConeConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *ConeConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const ConeConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const ConeConstraint) ConstraintSubType {
        _ = self;
        return .cone;
    }

    pub fn notifyShapeChanged(self: *ConeConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    // Internal helper function to calculate the values below
    fn calculateRotationConstraintProperties(self: *ConeConstraint, rotation1: Mat44, rotation2: Mat44) void {
        // Rotation is along the cross product of both twist axis
        const twist1 = rotation1.multiply3x3(self.local_space_twist_axis1);
        const twist2 = rotation2.multiply3x3(self.local_space_twist_axis2);

        // Calculate dot product between twist axis, if it's smaller than the cone angle we need to correct
        self.cos_theta = twist1.dot(twist2);
        if (self.cos_theta < self.cos_half_cone_angle) {
            // Rotation axis is defined by the two twist axis
            const rot_axis = twist2.cross(twist1);

            // If we can't find a rotation axis because the twist is too small, we'll use last frame's rotation axis
            const len = rot_axis.length();
            if (len > 0.0)
                self.world_space_rotation_axis = rot_axis.divScalar(len);

            self.angle_constraint_part.calculateConstraintProperties(self.base.body1, self.base.body2, self.world_space_rotation_axis, .{});
        } else self.angle_constraint_part.deactivate();
    }

    pub fn setupVelocityConstraint(self: *ConeConstraint, delta_time: f32) void {
        _ = delta_time;

        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.point_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, self.local_space_position1, self.base.body2, rotation2, self.local_space_position2);
        self.calculateRotationConstraintProperties(rotation1, rotation2);
    }

    pub fn resetWarmStart(self: *ConeConstraint) void {
        self.point_constraint_part.deactivate();
        self.angle_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *ConeConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.point_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.angle_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *ConeConstraint, delta_time: f32) bool {
        _ = delta_time;

        const pos = self.point_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);

        var rot = false;
        if (self.angle_constraint_part.isActive())
            rot = self.angle_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.world_space_rotation_axis, 0, math.flt_max);

        return pos or rot;
    }

    pub fn solvePositionConstraint(self: *ConeConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        self.point_constraint_part.calculateConstraintProperties(self.base.body1, Mat44.rotationQuat(self.base.body1.getRotation()), self.local_space_position1, self.base.body2, Mat44.rotationQuat(self.base.body2.getRotation()), self.local_space_position2);
        const pos = self.point_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, baumgarte);

        var rot = false;
        self.calculateRotationConstraintProperties(Mat44.rotationQuat(self.base.body1.getRotation()), Mat44.rotationQuat(self.base.body2.getRotation()));
        if (self.angle_constraint_part.isActive())
            rot = self.angle_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.cos_theta - self.cos_half_cone_angle, baumgarte);

        return pos or rot;
    }

    // TODO(debug_renderer): DrawConstraint, DrawConstraintLimits

    pub fn saveState(self: *const ConeConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.point_constraint_part.saveState(stream);
        self.angle_constraint_part.saveState(stream);
        stream.write(self.world_space_rotation_axis); // When twist is too small, the rotation is used from last frame so we need to store it
    }

    pub fn restoreState(self: *ConeConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.point_constraint_part.restoreState(stream);
        self.angle_constraint_part.restoreState(stream);
        stream.read(&self.world_space_rotation_axis);
    }

    pub fn getConstraintSettings(self: *const ConeConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try ConeConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.fromVec3(self.local_space_position1);
        settings.twist_axis1 = self.local_space_twist_axis1;
        settings.point2 = RVec3.fromVec3(self.local_space_position2);
        settings.twist_axis2 = self.local_space_twist_axis2;
        settings.half_cone_angle = trigonometry.acos(self.cos_half_cone_angle);
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const ConeConstraint) Mat44 {
        const perp = self.local_space_twist_axis1.getNormalizedPerpendicular();
        const perp2 = self.local_space_twist_axis1.cross(perp);
        return Mat44.init(Vec4.fromVec3W(self.local_space_twist_axis1, 0), Vec4.fromVec3W(perp, 0), Vec4.fromVec3W(perp2, 0), Vec4.fromVec3W(self.local_space_position1, 1));
    }

    pub fn getConstraintToBody2Matrix(self: *const ConeConstraint) Mat44 {
        // Note: Incorrect in rotation around the twist axis (the perpendicular does not match that of body 1),
        // this should not matter as we're not limiting rotation around the twist axis.
        const perp = self.local_space_twist_axis2.getNormalizedPerpendicular();
        const perp2 = self.local_space_twist_axis2.cross(perp);
        return Mat44.init(Vec4.fromVec3W(self.local_space_twist_axis2, 0), Vec4.fromVec3W(perp, 0), Vec4.fromVec3W(perp2, 0), Vec4.fromVec3W(self.local_space_position2, 1));
    }

    /// Update maximum angle between body 1 and 2 (see ConeConstraintSettings)
    pub fn setHalfConeAngle(self: *ConeConstraint, half_cone_angle: f32) void {
        // User input: Jolt's release build stores the cosine of any angle
        if (Core.enable_asserts) std.debug.assert(half_cone_angle >= 0.0 and half_cone_angle <= math.pi);
        self.cos_half_cone_angle = trigonometry.cos(half_cone_angle);
    }

    pub fn getCosHalfConeAngle(self: *const ConeConstraint) f32 {
        return self.cos_half_cone_angle;
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const ConeConstraint) Vec3 {
        return self.point_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaRotation(self: *const ConeConstraint) f32 {
        return self.angle_constraint_part.getTotalLambda();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/ConeConstraintTests.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
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
    return data.toOwnedSlice();
}

/// ConstraintSettings.restoreFromBinaryState of bytes
fn testRestoreSettings(allocator: Allocator, bytes: []const u8) Allocator.Error!ConstraintSettings.ConstraintResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
}

test "ConeConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.point1.eql(RVec3.zero()) and settings.point2.eql(RVec3.zero()));
    try testing.expect(settings.twist_axis1.eql(Vec3.axisX()) and settings.twist_axis2.eql(Vec3.axisX()));
    try testing.expectEqual(@as(f32, 0.0), settings.half_cone_angle);
    try testing.expectEqualStrings("ConeConstraintSettings", settings.asSettings().vtable.rtti_name);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 7;
    settings.base.base.num_velocity_steps_override = 2;
    settings.base.base.num_position_steps_override = 5;
    settings.base.base.draw_constraint_size = 0.25;
    settings.base.base.user_data = 99; // Not part of the binary state (Jolt)
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(1, -2, 3);
    settings.twist_axis1 = Vec3.axisY();
    settings.point2 = RVec3.init(-0.5, 0.25, 4);
    settings.twist_axis2 = Vec3.axisZ().negate();
    settings.half_cone_angle = 0.75;

    // ConstraintSettings (21 bytes), space (int), point 1 (Real), twist axis 1 (3 floats), point 2, twist axis 2, the
    // half cone angle
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 2 * 3 * @sizeOf(Real) + 2 * 12 + 4), bytes.len);
    try testing.expectEqual(ConstraintSettings.rttiHash("ConeConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.75))), std.mem.readInt(u32, bytes[bytes.len - 4 ..][0..4], .little));

    // Restore through RegisterTypes.constraint_settings_types
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(ConeConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 7), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u32, 2), restored.base.base.num_velocity_steps_override);
        try testing.expectEqual(@as(u32, 5), restored.base.base.num_position_steps_override);
        try testing.expectEqual(@as(f32, 0.25), restored.base.base.draw_constraint_size);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.point1.eql(settings.point1) and restored.point2.eql(settings.point2));
        try testing.expect(restored.twist_axis1.eql(settings.twist_axis1) and restored.twist_axis2.eql(settings.twist_axis2));
        try testing.expectEqual(settings.half_cone_angle, restored.half_cone_angle);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated anywhere: Jolt's error
    for ([_]usize{ 4, 21, 30, bytes.len - 1 }) |len| {
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
    var copy = ConeConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.point2.eql(settings.point2) and copy.twist_axis2.eql(settings.twist_axis2));
    try testing.expectEqual(@as(f32, 0.75), copy.half_cone_angle);
    try testing.expectEqual(@as(u64, 99), copy.base.base.user_data);
}

test "ConeConstraint: virtual dispatch, space conversions, setters, solver, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];

        // World space: the same point, twist axes 90 degrees apart (outside the cone)
        var settings = ConeConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.point1 = RVec3.init(1.5, 1.75, 3.5);
        settings.point2 = RVec3.init(1.5, 1.75, 3.5);
        settings.twist_axis1 = Vec3.axisX();
        settings.twist_axis2 = Vec3.axisY();
        settings.half_cone_angle = 0.5;
        settings.base.base.user_data = 42;

        // TwoBodyConstraintSettings::Create through the vtable
        var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit();
        const base = constraint.get().?;
        const two_body = virtual.downcast(TwoBodyConstraint, base);
        const c = virtual.downcast(ConeConstraint, two_body);
        try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
        try testing.expectEqual(ConstraintSubType.cone, base.getSubType());
        try testing.expectEqual(@as(u64, 42), base.getUserData());
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, base.isActive());

        // The constructor took the points and axes to the local space of the bodies, the initial rotation axis is the
        // world space perpendicular of twist axis 1
        try testing.expect(c.local_space_position1.eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3()));
        try testing.expect(c.local_space_position2.eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3()));
        try testing.expect(c.local_space_twist_axis1.eql(body1.getInverseCenterOfMassTransform().multiply3x3(Vec3.axisX())));
        try testing.expect(c.local_space_twist_axis2.eql(body2.getInverseCenterOfMassTransform().multiply3x3(Vec3.axisY())));
        try testing.expect(c.world_space_rotation_axis.eql(Vec3.axisX().getNormalizedPerpendicular()));
        try testing.expectEqual(trigonometry.cos(0.5), c.getCosHalfConeAngle());

        // The matrices: the twist axis, two perpendiculars and the point
        const m1 = two_body.getConstraintToBody1Matrix();
        try testing.expect(m1.getAxisX().eql(c.local_space_twist_axis1));
        try testing.expect(m1.getAxisY().eql(c.local_space_twist_axis1.getNormalizedPerpendicular()));
        try testing.expect(m1.getAxisZ().eql(c.local_space_twist_axis1.cross(c.local_space_twist_axis1.getNormalizedPerpendicular())));
        try testing.expect(m1.getTranslation().eql(c.local_space_position1));
        const m2 = two_body.getConstraintToBody2Matrix();
        try testing.expect(m2.getAxisX().eql(c.local_space_twist_axis2));
        try testing.expect(m2.getTranslation().eql(c.local_space_position2));
        try testing.expect(m1.getAxisX().isNormalized(.{}) and m1.getAxisY().isNormalized(.{}) and m1.getAxisZ().isNormalized(.{}));

        // SetHalfConeAngle
        c.setHalfConeAngle(0.0);
        try testing.expectEqual(@as(f32, 1.0), c.getCosHalfConeAngle());
        c.setHalfConeAngle(math.pi);
        try testing.expectEqual(trigonometry.cos(math.pi), c.getCosHalfConeAngle());
        c.setHalfConeAngle(0.5);

        // GetConstraintSettings: local space, the half cone angle from its cosine; a constraint made from them is the same
        {
            var copy = try base.getConstraintSettings();
            defer copy.deinit();
            const s = virtual.downcast(ConeConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
            try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
            try testing.expectEqual(@as(u64, 42), s.base.base.user_data);
            try testing.expect(s.point1.eql(RVec3.fromVec3(c.local_space_position1)) and s.point2.eql(RVec3.fromVec3(c.local_space_position2)));
            try testing.expect(s.twist_axis1.eql(c.local_space_twist_axis1) and s.twist_axis2.eql(c.local_space_twist_axis2));
            try testing.expectEqual(trigonometry.acos(trigonometry.cos(0.5)), s.half_cone_angle);
            var again = ConeConstraint.init(allocator, body1, body2, s);
            defer again.asConstraint().deinit();
            try testing.expect(again.local_space_position1.eql(c.local_space_position1) and again.local_space_twist_axis2.eql(c.local_space_twist_axis2));
            try testing.expect(@abs(again.getCosHalfConeAngle() - c.getCosHalfConeAngle()) < 1.0e-6);
            // Local space: the initial rotation axis is taken to world space with the rotation of body 1
            try testing.expect(again.world_space_rotation_axis.eql(body1.getRotation().mulVec3(c.local_space_twist_axis1.getNormalizedPerpendicular())));
        }

        // Solver: the twist axes are 90 degrees apart, the cone limit is active when a body is dynamic
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);
        base.setupVelocityConstraint(1.0 / 60.0);
        const twist1 = body1.getRotation().mulVec3(c.local_space_twist_axis1);
        const twist2 = body2.getRotation().mulVec3(c.local_space_twist_axis2);
        try testing.expect(@abs(c.cos_theta - twist1.dot(twist2)) < 1.0e-6);
        try testing.expect(c.world_space_rotation_axis.isClose(twist2.cross(twist1).normalized(), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, c.angle_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        const applied = base.solveVelocityConstraint(1.0 / 60.0);
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, applied);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);
        if (applied) {
            for (0..5) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
            try testing.expect(!c.getTotalLambdaPosition().eql(Vec3.zero()));
            try testing.expect(c.getTotalLambdaRotation() >= 0.0); // The impulse only pushes back into the cone
            try testing.expect(base.solvePositionConstraint(1.0 / 60.0, 0.2));
        }

        // SaveState / RestoreState: enabled, the lambdas of both parts and the world space rotation axis
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        base.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 1 + 12 + 4 + 12), recorder.getData().len);
        const lambda_position = c.getTotalLambdaPosition();
        const lambda_rotation = c.getTotalLambdaRotation();
        const axis = c.world_space_rotation_axis;
        base.resetWarmStart();
        try testing.expect(c.getTotalLambdaPosition().eql(Vec3.zero()) and c.getTotalLambdaRotation() == 0.0);
        c.world_space_rotation_axis = Vec3.axisZ();
        base.restoreState(recorder.recorder());
        try testing.expect(c.getTotalLambdaPosition().eql(lambda_position) and c.getTotalLambdaRotation() == lambda_rotation);
        try testing.expect(c.world_space_rotation_axis.eql(axis));

        // NotifyShapeChanged moves the attachment point of the body that changed
        const p1 = c.local_space_position1;
        const p2 = c.local_space_position2;
        base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
        try testing.expect(c.local_space_position1.eql(p1) and c.local_space_position2.eql(p2.sub(Vec3.init(0.5, 0, 0))));
        base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
        try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0, 1, 0))));
        base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
        try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0, 1, 0))) and c.local_space_position2.eql(p2.sub(Vec3.init(0.5, 0, 0))));
    }
}

test "ConeConstraint: inside the cone the angle part is off, the last rotation axis is kept" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // Local space: twist axes that are the same in world space (inside any cone)
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(0.25, 0.5, 0.75);
    settings.point2 = RVec3.init(-1, -0.5, 0.25);
    settings.twist_axis1 = body1.getRotation().inverseRotate(Vec3.axisZ());
    settings.twist_axis2 = body2.getRotation().inverseRotate(Vec3.axisZ());
    settings.half_cone_angle = 0.1;
    var c = ConeConstraint.init(allocator, body1, body2, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position1.eql(Vec3.init(0.25, 0.5, 0.75)) and c.local_space_position2.eql(Vec3.init(-1, -0.5, 0.25)));
    try testing.expect(c.local_space_twist_axis1.eql(settings.twist_axis1) and c.local_space_twist_axis2.eql(settings.twist_axis2));
    const initial_axis = body1.getRotation().mulVec3(settings.twist_axis1.getNormalizedPerpendicular());
    try testing.expect(c.world_space_rotation_axis.eql(initial_axis));

    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(!c.angle_constraint_part.isActive());
    try testing.expect(c.world_space_rotation_axis.eql(initial_axis)); // Not touched
    base.warmStartVelocityConstraint(1.0);
    _ = base.solveVelocityConstraint(1.0 / 60.0);
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaRotation());

    // A cone of half angle 0 with equal twist axes: at the limit, no rotation axis from the cross product (the last one
    // is used when cos(theta) is below 1)
    c.setHalfConeAngle(0.0);
    try testing.expectEqual(@as(f32, 1.0), c.getCosHalfConeAngle());
    _ = base.solvePositionConstraint(1.0 / 60.0, 0.2);
    try testing.expect(c.cos_theta <= 1.0 + 1.0e-6);
}

test "ConeConstraint: body 1 fixed to the world" {
    // Body 1 is Body::sFixedToWorld: the cone is fixed in the world, only body 2 moves
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .rotation = Quat.rotation(Vec3.axisZ(), 1.2), .angular_velocity = Vec3.init(0, 0, 2) });
    defer context.destroyBodies(&.{body});
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = RVec3.init(1, 3, 3);
    settings.point2 = RVec3.init(1, 3, 3);
    settings.twist_axis1 = Vec3.axisY().negate();
    settings.twist_axis2 = body.getRotation().mulVec3(Vec3.axisY().negate()); // The body's -Y axis
    settings.half_cone_angle = 0.2;
    var c = ConeConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position1.eql(Vec3.init(1, 3, 3))); // The world body has an identity transform
    try testing.expect(c.local_space_twist_axis1.eql(Vec3.axisY().negate()));
    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.angle_constraint_part.isActive()); // 1.2 rad apart, the cone is 0.2
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
    for (0..10) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);

    // The angular velocity that opens the cone further is removed
    try testing.expect(body.getAngularVelocity().dot(c.world_space_rotation_axis) >= -1.0e-5);
    try testing.expect(c.getTotalLambdaRotation() > 0.0);
}

test "ConeConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try ConeConstraintSettings.create(allocator)).asSettings());
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
