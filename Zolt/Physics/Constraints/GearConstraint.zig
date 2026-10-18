//! Port of: Jolt/Physics/Constraints/GearConstraint.h, Jolt/Physics/Constraints/GearConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(GearConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint): TODO(debug_renderer).
//!
//! - `mGear1Constraint` / `mGear2Constraint` (`RefConst<Constraint>`) are `RefConst(Constraint)` members that the
//!   destructor (`destruct`, run by the generated destructor chain) releases. `SetConstraints(const Constraint *, const
//!   Constraint *)` is `setConstraints(?*const Constraint, ?*const Constraint)` (null is Jolt's nullptr).
//! - SolvePositionConstraint reads the angles of the hinges through `HingeConstraint.getCurrentAngle` after checking
//!   `getSubType() == .hinge` (Jolt's StaticCast). Another constraint type is Jolt's `JPH_ASSERT(false, "Unsupported")`
//!   (a panic with asserts on, user input can reach it) followed by `return false`.
//! - `fmod(x, 2.0f * JPH_PI)` is `@rem(x, 2.0 * math.pi)`: both are the exact IEEE remainder with the sign of x
//!   (truncated division), so the result is bit identical (Zig lowers @rem on f32 to compiler-rt's fmodf, a port of
//!   musl's, Jolt calls the C library's fmodf).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
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
const HingeConstraint = @import("HingeConstraint.zig").HingeConstraint;
const GearConstraintPart = @import("ConstraintPart/GearConstraintPart.zig").GearConstraintPart;

/// Gear constraint settings
pub const GearConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(GearConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "GearConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .enumAttr("mSpace", .space),
            .attr("mHingeAxis1", .hinge_axis1),
            .attr("mHingeAxis2", .hinge_axis2),
            .attr("mRatio", .ratio),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint reference frame (space determined by mSpace).
    hinge_axis1: Vec3 = Vec3.axisX(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    hinge_axis2: Vec3 = Vec3.axisX(),

    /// Ratio between both gears, see SetRatio.
    ratio: f32 = 1.0,

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) GearConstraintSettings {
        return .{ .base = .init(GearConstraintSettings, allocator) };
    }

    /// new GearConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*GearConstraintSettings {
        const self = try allocator.create(GearConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const GearConstraintSettings) GearConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *GearConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const GearConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *GearConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const GearConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const GearConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.hinge_axis1);
        stream.write(self.hinge_axis2);
        stream.write(self.ratio);
    }

    /// Create an instance of this constraint.
    pub fn createConstraint(self: *const GearConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try GearConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    /// Defines the ratio between the rotation of both gears
    /// The ratio is defined as: Gear1Rotation(t) = -ratio * Gear2Rotation(t)
    /// @param inNumTeethGear1 Number of teeth that body 1 has
    /// @param inNumTeethGear2 Number of teeth that body 2 has
    pub fn setRatio(self: *GearConstraintSettings, num_teeth_gear1: i32, num_teeth_gear2: i32) void {
        self.ratio = @as(f32, @floatFromInt(num_teeth_gear2)) / @as(f32, @floatFromInt(num_teeth_gear1));
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *GearConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.hinge_axis1);
        stream.read(&self.hinge_axis2);
        stream.read(&self.ratio);
    }
};

/// A gear constraint constrains the rotation of body1 to the rotation of body 2 using a gear.
/// Note that this constraint needs to be used in conjunction with a two hinge constraints.
pub const GearConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space hinge axis for body 1
    local_space_hinge_axis1: Vec3,

    // Local space hinge axis for body 2
    local_space_hinge_axis2: Vec3,

    // Ratio between gear 1 and 2
    ratio: f32,

    // The constraints that constrain both gears (2 hinges), optional and used to calculate the rotation error and fix numerical drift.
    gear1_constraint: RefConst(Constraint) = .empty,
    gear2_constraint: RefConst(Constraint) = .empty,

    // RUN TIME PROPERTIES FOLLOW

    // World space hinge axis for body 1
    world_space_hinge_axis1: Vec3 = undefined,

    // World space hinge axis for body 2
    world_space_hinge_axis2: Vec3 = undefined,

    // The constraint parts
    gear_constraint_part: GearConstraintPart = .{},

    /// Construct gear constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const GearConstraintSettings) GearConstraint {
        var self: GearConstraint = .{
            .base = .init(GearConstraint, allocator, body1, body2, &settings.base),
            .local_space_hinge_axis1 = settings.hinge_axis1,
            .local_space_hinge_axis2 = settings.hinge_axis2,
            .ratio = settings.ratio,
        };

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            self.local_space_hinge_axis1 = body1.getInverseCenterOfMassTransform().multiply3x3(self.local_space_hinge_axis1).normalized();
            self.local_space_hinge_axis2 = body2.getInverseCenterOfMassTransform().multiply3x3(self.local_space_hinge_axis2).normalized();
        }

        return self;
    }

    /// new GearConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const GearConstraintSettings) Allocator.Error!*GearConstraint {
        const self = try allocator.create(GearConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    /// Destructor: releases the references to the hinges (RefConst members)
    pub fn destruct(self: *GearConstraint) void {
        self.gear1_constraint.deinit();
        self.gear2_constraint.deinit();
    }

    pub fn asConstraint(self: *GearConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const GearConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *GearConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const GearConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const GearConstraint) ConstraintSubType {
        _ = self;
        return .gear;
    }

    pub fn notifyShapeChanged(self: *GearConstraint, body_id: BodyID, delta_com: Vec3) void {
        // Do nothing
        _ = self;
        _ = body_id;
        _ = delta_com;
    }

    pub fn setupVelocityConstraint(self: *GearConstraint, delta_time: f32) void {
        _ = delta_time;

        // Calculate constraint properties that are constant while bodies don't move
        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.calculateConstraintProperties(rotation1, rotation2);
    }

    pub fn resetWarmStart(self: *GearConstraint) void {
        self.gear_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *GearConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.gear_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *GearConstraint, delta_time: f32) bool {
        _ = delta_time;

        return self.gear_constraint_part.solveVelocityConstraint(self.base.body1, self.world_space_hinge_axis1, self.base.body2, self.world_space_hinge_axis2, self.ratio);
    }

    pub fn solvePositionConstraint(self: *GearConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        if (self.gear1_constraint.get() == null or self.gear2_constraint.get() == null)
            return false;
        const gear1_constraint = self.gear1_constraint.get().?;
        const gear2_constraint = self.gear2_constraint.get().?;

        var gear1rot: f32 = undefined;
        if (gear1_constraint.getSubType() == .hinge) {
            gear1rot = virtual.downcast(HingeConstraint, virtual.downcast(TwoBodyConstraint, gear1_constraint)).getCurrentAngle();
        } else {
            if (Core.enable_asserts) @panic("Unsupported");
            return false;
        }

        var gear2rot: f32 = undefined;
        if (gear2_constraint.getSubType() == .hinge) {
            gear2rot = virtual.downcast(HingeConstraint, virtual.downcast(TwoBodyConstraint, gear2_constraint)).getCurrentAngle();
        } else {
            if (Core.enable_asserts) @panic("Unsupported");
            return false;
        }

        const err = math.centerAngleAroundZero(@rem(gear1rot + self.ratio * gear2rot, 2.0 * math.pi));
        if (err == 0.0)
            return false;

        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.calculateConstraintProperties(rotation1, rotation2);
        return self.gear_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, err, baumgarte);
    }

    // TODO(debug_renderer): DrawConstraint

    pub fn saveState(self: *const GearConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.gear_constraint_part.saveState(stream);
    }

    pub fn restoreState(self: *GearConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.gear_constraint_part.restoreState(stream);
    }

    pub fn getConstraintSettings(self: *const GearConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try GearConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.hinge_axis1 = self.local_space_hinge_axis1;
        settings.hinge_axis2 = self.local_space_hinge_axis2;
        settings.ratio = self.ratio;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const GearConstraint) Mat44 {
        const perp = self.local_space_hinge_axis1.getNormalizedPerpendicular();
        return Mat44.init(Vec4.fromVec3W(self.local_space_hinge_axis1, 0), Vec4.fromVec3W(perp, 0), Vec4.fromVec3W(self.local_space_hinge_axis1.cross(perp), 0), Vec4.init(0, 0, 0, 1));
    }

    pub fn getConstraintToBody2Matrix(self: *const GearConstraint) Mat44 {
        const perp = self.local_space_hinge_axis2.getNormalizedPerpendicular();
        return Mat44.init(Vec4.fromVec3W(self.local_space_hinge_axis2, 0), Vec4.fromVec3W(perp, 0), Vec4.fromVec3W(self.local_space_hinge_axis2.cross(perp), 0), Vec4.init(0, 0, 0, 1));
    }

    /// The constraints that constrain both gears (2 hinges), optional and used to calculate the rotation error and fix numerical drift.
    pub fn setConstraints(self: *GearConstraint, gear1: ?*const Constraint, gear2: ?*const Constraint) void {
        self.gear1_constraint.set(gear1);
        self.gear2_constraint.set(gear2);
    }

    ///@name Get Lagrange multiplier from last physics update (the angular impulse applied to satisfy the constraint)
    pub fn getTotalLambda(self: *const GearConstraint) f32 {
        return self.gear_constraint_part.getTotalLambda();
    }

    // Internal helper function to calculate the values below
    fn calculateConstraintProperties(self: *GearConstraint, rotation1: Mat44, rotation2: Mat44) void {
        // Calculate world space normals
        self.world_space_hinge_axis1 = rotation1.mulVec3(self.local_space_hinge_axis1);
        self.world_space_hinge_axis2 = rotation2.mulVec3(self.local_space_hinge_axis2);

        self.gear_constraint_part.calculateConstraintProperties(self.base.body1, self.world_space_hinge_axis1, self.base.body2, self.world_space_hinge_axis2, self.ratio);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig and SimulationParity.zig, simulations in
// ZoltTests/Physics/GearConstraintTests.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const TestContext = @import("ConstraintPart/ConstraintPartTestContext.zig");
const HingeConstraintSettings = @import("HingeConstraint.zig").HingeConstraintSettings;
const PointConstraintSettings = @import("PointConstraint.zig").PointConstraintSettings;

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

/// A hinge from the world to `body` around `axis` at its center of mass (world space, the current rotation of the body
/// is angle 0), one reference owned by the caller
fn testCreateHinge(allocator: Allocator, body: *Body, axis: Vec3) Allocator.Error!Ref(Constraint) {
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = body.getCenterOfMassPosition();
    settings.point2 = settings.point1;
    settings.hinge_axis1 = axis;
    settings.hinge_axis2 = axis;
    settings.normal_axis1 = axis.getNormalizedPerpendicular();
    settings.normal_axis2 = settings.normal_axis1;
    return .init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, Body.fixedToWorld(), body)).base);
}

/// The current angle of a hinge made by testCreateHinge
fn testHingeAngle(hinge: Ref(Constraint)) f32 {
    return virtual.downcast(HingeConstraint, virtual.downcast(TwoBodyConstraint, hinge.get().?)).getCurrentAngle();
}

test "GearConstraintSettings: defaults, SetRatio, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = GearConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.hinge_axis1.eql(Vec3.axisX()) and settings.hinge_axis2.eql(Vec3.axisX()));
    try testing.expectEqual(@as(f32, 1.0), settings.ratio);
    try testing.expectEqualStrings("GearConstraintSettings", settings.asSettings().vtable.rtti.name);

    // SetRatio: the number of teeth of gear 2 divided by the number of teeth of gear 1 (in float)
    settings.setRatio(20, 10);
    try testing.expectEqual(@as(f32, 0.5), settings.ratio);
    var seven: f32 = 7.0;
    var three: f32 = 3.0;
    _ = .{ &seven, &three };
    settings.setRatio(3, 7);
    try testing.expectEqual(seven / three, settings.ratio);
    settings.setRatio(-4, 8); // Jolt does not check the numbers
    try testing.expectEqual(@as(f32, -2.0), settings.ratio);
    settings.setRatio(16777217, 1); // int -> float rounds to nearest even like C++: 2^24
    try testing.expectEqual(@as(f32, 1.0 / 16777216.0), settings.ratio);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 9;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 4;
    settings.base.base.draw_constraint_size = 0.5;
    settings.space = .local_to_body_com;
    settings.hinge_axis1 = Vec3.axisY();
    settings.hinge_axis2 = Vec3.init(0, 0.6, -0.8);
    settings.ratio = -2.5;

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), space (int), the axes (3 floats each),
    // the ratio
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 2 * 12 + 4), bytes.len);
    try testing.expectEqual(RTTI.hashName("GearConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, -2.5))), std.mem.readInt(u32, bytes[49..53], .little));

    // Restore through RegisterTypes.factory
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(GearConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 9), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u32, 3), restored.base.base.num_velocity_steps_override);
        try testing.expectEqual(@as(u32, 4), restored.base.base.num_position_steps_override);
        try testing.expectEqual(@as(f32, 0.5), restored.base.base.draw_constraint_size);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.hinge_axis1.eql(settings.hinge_axis1) and restored.hinge_axis2.eql(settings.hinge_axis2));
        try testing.expectEqual(@as(f32, -2.5), restored.ratio);
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
    var copy = GearConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.hinge_axis2.eql(settings.hinge_axis2));
    try testing.expectEqual(@as(f32, -2.5), copy.ratio);
    try testing.expectEqual(@as(u32, 9), copy.base.base.constraint_priority);
}

test "GearConstraint: virtual dispatch, space conversions, solver, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];
    const dt: f32 = 1.0 / 60.0;

    // World space axes (not normalized: the constructor normalizes the local axes)
    var settings = GearConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.hinge_axis1 = Vec3.init(1, 2, 3);
    settings.hinge_axis2 = Vec3.axisZ();
    settings.setRatio(10, 25);
    settings.base.base.user_data = 77;

    // TwoBodyConstraintSettings::Create through the vtable
    var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
    defer constraint.deinit();
    const base = constraint.get().?;
    const two_body = virtual.downcast(TwoBodyConstraint, base);
    const c = virtual.downcast(GearConstraint, two_body);
    try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
    try testing.expectEqual(ConstraintSubType.gear, base.getSubType());
    try testing.expectEqual(@as(u64, 77), base.getUserData());
    try testing.expect(two_body.getBody1() == body1 and two_body.getBody2() == body2);
    try testing.expectEqual(@as(f32, 2.5), c.ratio);

    // The constructor took the axes to the local space of the bodies
    try testing.expect(c.local_space_hinge_axis1.eql(body1.getInverseCenterOfMassTransform().multiply3x3(settings.hinge_axis1).normalized()));
    try testing.expect(c.local_space_hinge_axis2.eql(body2.getInverseCenterOfMassTransform().multiply3x3(settings.hinge_axis2).normalized()));
    try testing.expect(body1.getRotation().mulVec3(c.local_space_hinge_axis1).isClose(settings.hinge_axis1.normalized(), .{ .max_dist_sq = 1.0e-10 }));

    // The constraint frames: the hinge axis, a perpendicular and their cross product, no translation
    const matrices = [_]Mat44{ two_body.getConstraintToBody1Matrix(), two_body.getConstraintToBody2Matrix() };
    const axes = [_]Vec3{ c.local_space_hinge_axis1, c.local_space_hinge_axis2 };
    for (matrices, axes) |m, axis| {
        const perp = axis.getNormalizedPerpendicular();
        try testing.expect(m.getAxisX().eql(axis) and m.getAxisY().eql(perp) and m.getAxisZ().eql(axis.cross(perp)));
        try testing.expect(m.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));
        for (0..3) |k| try testing.expectEqual(@as(f32, 0.0), m.getColumn4(@intCast(k)).getW());
    }

    // GetConstraintSettings: local space, the local axes and the ratio; a constraint made from them is the same
    {
        var copy = try base.getConstraintSettings();
        defer copy.deinit();
        const s = virtual.downcast(GearConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
        try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
        try testing.expectEqual(@as(u64, 77), s.base.base.user_data);
        try testing.expect(s.hinge_axis1.eql(c.local_space_hinge_axis1) and s.hinge_axis2.eql(c.local_space_hinge_axis2));
        try testing.expectEqual(@as(f32, 2.5), s.ratio);
        var again = Ref(Constraint).init(&(try s.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer again.deinit();
        const c2 = virtual.downcast(GearConstraint, virtual.downcast(TwoBodyConstraint, again.get().?));
        try testing.expect(c2.local_space_hinge_axis1.eql(c.local_space_hinge_axis1) and c2.local_space_hinge_axis2.eql(c.local_space_hinge_axis2));
        try testing.expectEqual(c.ratio, c2.ratio);
    }

    // Solver: the angular velocities around the hinge axes satisfy a1 . w1 + ratio a2 . w2 = 0
    try testing.expect(base.isActive());
    base.setupVelocityConstraint(dt);
    try testing.expect(c.gear_constraint_part.isActive());
    try testing.expect(c.world_space_hinge_axis1.eql(Mat44.rotationQuat(body1.getRotation()).mulVec3(c.local_space_hinge_axis1)));
    try testing.expect(c.world_space_hinge_axis2.eql(Mat44.rotationQuat(body2.getRotation()).mulVec3(c.local_space_hinge_axis2)));
    base.warmStartVelocityConstraint(1.0);

    // Jolt's GearConstraintPart applies lambda I2^-1 b to body 2 where its Jacobian (r b) asks for r lambda I2^-1 b, so
    // with a ratio other than 1 an iteration scales the relative velocity by 1 - (A + r B) / (A + r^2 B) (A = a . I1^-1
    // a, B = b . I2^-1 b) instead of removing it; for r > 0 the iterations converge
    const part = &c.gear_constraint_part;
    const inv_a = c.world_space_hinge_axis1.dot(part.inv_i1_a);
    const inv_b = c.world_space_hinge_axis2.dot(part.inv_i2_b);
    const relative_before = c.world_space_hinge_axis1.dot(body1.getAngularVelocity()) + c.ratio * c.world_space_hinge_axis2.dot(body2.getAngularVelocity());
    try testing.expect(@abs(relative_before) > 0.1);
    try testing.expect(base.solveVelocityConstraint(dt));
    const relative_after = c.world_space_hinge_axis1.dot(body1.getAngularVelocity()) + c.ratio * c.world_space_hinge_axis2.dot(body2.getAngularVelocity());
    const expected_after = relative_before * (1.0 - (inv_a + c.ratio * inv_b) / (inv_a + c.ratio * c.ratio * inv_b));
    try testing.expect(@abs(relative_after - expected_after) < 1.0e-5);
    for (0..200) |_| _ = base.solveVelocityConstraint(dt);
    const relative = c.world_space_hinge_axis1.dot(body1.getAngularVelocity()) + c.ratio * c.world_space_hinge_axis2.dot(body2.getAngularVelocity());
    try testing.expect(@abs(relative) < 1.0e-5);
    try testing.expect(c.getTotalLambda() != 0.0);

    // No hinges set: no position correction (Jolt returns false before reading anything)
    const before1 = TestContext.BodySnapshot.take(body1);
    const before2 = TestContext.BodySnapshot.take(body2);
    try testing.expect(!base.solvePositionConstraint(dt, 0.2));
    try TestContext.checkUnmoved(body1, before1);
    try TestContext.checkUnmoved(body2, before2);

    // SaveState / RestoreState: enabled + the lambda of the gear part
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 1 + 4), recorder.getData().len);
    const lambda = c.getTotalLambda();
    base.resetWarmStart();
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambda());
    try testing.expect(!c.gear_constraint_part.isActive());
    base.restoreState(recorder.recorder());
    try testing.expectEqual(lambda, c.getTotalLambda());

    // Warm start applies the scaled lambda of the previous frame
    base.setupVelocityConstraint(dt);
    const w1 = body1.getAngularVelocity();
    base.warmStartVelocityConstraint(0.5);
    try testing.expectEqual(lambda * 0.5, c.getTotalLambda());
    try testing.expect(!body1.getAngularVelocity().eql(w1));

    // NotifyShapeChanged does nothing
    const a1 = c.local_space_hinge_axis1;
    const a2 = c.local_space_hinge_axis2;
    base.notifyShapeChanged(body1.getID(), Vec3.init(0.5, 0, 0));
    base.notifyShapeChanged(body2.getID(), Vec3.init(0, 1, 0));
    try testing.expect(c.local_space_hinge_axis1.eql(a1) and c.local_space_hinge_axis2.eql(a2));

    // A disabled constraint is not active (TwoBodyConstraint::IsActive)
    base.setEnabled(false);
    try testing.expect(!base.isActive());
}

test "GearConstraint: local space settings" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);

    // Local space: the axes are used as they are; any space value other than world space is local space
    for ([_]ConstraintSpace{ .local_to_body_com, @enumFromInt(7) }) |space| {
        var settings = GearConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.space = space;
        settings.hinge_axis1 = Vec3.axisY();
        settings.hinge_axis2 = Vec3.init(0, 0.6, -0.8);
        settings.ratio = -0.75;
        var c = GearConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        try testing.expect(c.local_space_hinge_axis1.eql(Vec3.axisY()) and c.local_space_hinge_axis2.eql(Vec3.init(0, 0.6, -0.8)));
        try testing.expectEqual(@as(f32, -0.75), c.ratio);
        try testing.expectEqual(@as(f32, 0.0), c.getTotalLambda());
    }
}

test "GearConstraint: position correction through the hinges" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const dt: f32 = 1.0 / 60.0;

    // Two gears on hinges around Z to the world
    const gear1 = try context.createBody(.dynamic, .{ .position = RVec3.init(0, 2, 0) });
    const gear2 = try context.createBody(.dynamic, .{ .position = RVec3.init(3, 2, 0), .half_extent = Vec3.init(1, 1, 0.25) });
    defer context.destroyBodies(&.{ gear1, gear2 });
    var hinge1 = try testCreateHinge(allocator, gear1, Vec3.axisZ());
    defer hinge1.deinit();
    var hinge2 = try testCreateHinge(allocator, gear2, Vec3.axisZ());
    defer hinge2.deinit();

    var settings = GearConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.hinge_axis1 = Vec3.axisZ();
    settings.hinge_axis2 = Vec3.axisZ();
    settings.setRatio(10, 20); // Gear 1 turns twice for every turn of gear 2
    var gear = GearConstraint.init(allocator, gear1, gear2, &settings);
    defer gear.asConstraint().deinit();
    const base = gear.asConstraint();

    // Without both hinges there is no position correction
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));
    gear.setConstraints(hinge1.get(), null);
    try testing.expectEqual(@as(u32, 2), hinge1.get().?.getRefCount()); // The gear holds a reference
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));
    gear.setConstraints(null, hinge2.get());
    try testing.expectEqual(@as(u32, 1), hinge1.get().?.getRefCount());
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));
    gear.setConstraints(hinge1.get(), hinge2.get());
    try testing.expectEqual(@as(u32, 2), hinge1.get().?.getRefCount());
    try testing.expectEqual(@as(u32, 2), hinge2.get().?.getRefCount());

    // The gears at the angles the hinges were created with: no error, no correction
    try testing.expectEqual(@as(f32, 0.0), testHingeAngle(hinge1));
    try testing.expectEqual(@as(f32, 0.0), testHingeAngle(hinge2));
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));

    // Rotate gear 1: the error gear1rot + ratio * gear2rot is corrected (both gears turn towards the relation)
    gear1.setPositionAndRotationInternal(gear1.getPosition(), Quat.rotation(Vec3.axisZ(), 0.3), .{});
    const error_before = testHingeAngle(hinge1) + gear.ratio * testHingeAngle(hinge2);
    try testing.expect(@abs(error_before - 0.3) < 1.0e-6);
    try testing.expect(base.solvePositionConstraint(dt, 0.5));
    const error_after = testHingeAngle(hinge1) + gear.ratio * testHingeAngle(hinge2);
    try testing.expect(testHingeAngle(hinge1) < 0.3 and testHingeAngle(hinge2) < 0.0);

    // Baumgarte 0.5 removes half of the error weighted like the velocity step (see the solver test above: body 2 gets
    // lambda I2^-1 b, not r lambda I2^-1 b)
    {
        const part = &gear.gear_constraint_part;
        const inv_a = Vec3.axisZ().dot(part.inv_i1_a);
        const inv_b = Vec3.axisZ().dot(part.inv_i2_b);
        const expected_error = error_before * (1.0 - 0.5 * (inv_a + gear.ratio * inv_b) / (inv_a + gear.ratio * gear.ratio * inv_b));
        try testing.expect(@abs(error_after - expected_error) < 1.0e-5);
    }

    // The error is taken modulo 2 pi and centered around zero: gear 1 at 1, gear 2 at 3 is an error of 7 - 2 pi; the
    // correction is the one of the gear part for that error
    for ([_]f32{ 1.0, -1.0 }) |sign| {
        gear1.setPositionAndRotationInternal(gear1.getPosition(), Quat.rotation(Vec3.axisZ(), sign * 1.0), .{});
        gear2.setPositionAndRotationInternal(gear2.getPosition(), Quat.rotation(Vec3.axisZ(), sign * 3.0), .{});
        const expected_error = sign * (7.0 - 2.0 * math.pi);
        var part: GearConstraintPart = .{};
        part.calculateConstraintProperties(gear1, Vec3.axisZ(), gear2, Vec3.axisZ(), gear.ratio);
        const expected_step = -part.effective_mass * 0.5 * expected_error * part.inv_i1_a.getZ();
        const angle1 = testHingeAngle(hinge1);
        try testing.expect(base.solvePositionConstraint(dt, 0.5));
        try testing.expect(@abs(testHingeAngle(hinge1) - angle1 - expected_step) < 1.0e-5);
    }

    // A constraint that is not a hinge: Jolt asserts ("Unsupported", user input) and returns false in its release build
    if (!Core.enable_asserts) {
        var point_settings = PointConstraintSettings.init(allocator);
        defer point_settings.asSettings().deinit();
        var point = Ref(Constraint).init(&(try point_settings.asTwoBodySettingsConst().createConstraint(allocator, Body.fixedToWorld(), gear1)).base);
        defer point.deinit();
        gear.setConstraints(point.get(), hinge2.get());
        try testing.expect(!base.solvePositionConstraint(dt, 0.5));
        gear.setConstraints(hinge1.get(), point.get());
        try testing.expect(!base.solvePositionConstraint(dt, 0.5));
        gear.setConstraints(hinge1.get(), hinge2.get());
    }
}

test "GearConstraint: the destructor releases the hinges" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);

    var hinge1 = try testCreateHinge(allocator, bodies[0], Vec3.axisX());
    defer hinge1.deinit();
    var settings = GearConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    {
        var gear = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, bodies[0], bodies[1])).base);
        defer gear.deinit();
        const c = virtual.downcast(GearConstraint, virtual.downcast(TwoBodyConstraint, gear.get().?));

        // The gear may hold the only reference to a hinge (Jolt's RefConst)
        var hinge2 = try testCreateHinge(allocator, bodies[1], Vec3.axisX());
        c.setConstraints(hinge1.get(), hinge2.get());
        hinge2.deinit();
        try testing.expectEqual(@as(u32, 1), c.gear2_constraint.get().?.getRefCount());

        // Setting the same constraints again keeps one reference each
        c.setConstraints(hinge1.get(), c.gear2_constraint.get());
        try testing.expectEqual(@as(u32, 2), hinge1.get().?.getRefCount());
        try testing.expectEqual(@as(u32, 1), c.gear2_constraint.get().?.getRefCount());
    }
    try testing.expectEqual(@as(u32, 1), hinge1.get().?.getRefCount());
}

test "GearConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try GearConstraintSettings.create(allocator)).asSettings());
            defer settings.deinit();
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, bodies[0], bodies[1])).base);
            defer constraint.deinit();
            var hinge = try testCreateHinge(allocator, bodies[0], Vec3.axisX());
            defer hinge.deinit();
            virtual.downcast(GearConstraint, virtual.downcast(TwoBodyConstraint, constraint.get().?)).setConstraints(hinge.get(), hinge.get());
            var copy = try constraint.get().?.getConstraintSettings();
            defer copy.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
