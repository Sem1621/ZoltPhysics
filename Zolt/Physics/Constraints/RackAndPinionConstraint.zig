//! Port of: Jolt/Physics/Constraints/RackAndPinionConstraint.h, Jolt/Physics/Constraints/RackAndPinionConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(RackAndPinionConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint): a gated declaration (RemainingFeaturesArchitecture.md R2).
//!
//! - `mPinionConstraint` / `mRackConstraint` (`RefConst<Constraint>`) are `RefConst(Constraint)` members that the
//!   destructor (`destruct`, run by the generated destructor chain) releases. `SetConstraints(const Constraint *, const
//!   Constraint *)` is `setConstraints(?*const Constraint, ?*const Constraint)` (null is Jolt's nullptr).
//! - SolvePositionConstraint reads the pinion angle through `HingeConstraint.getCurrentAngle` and the rack position
//!   through `SliderConstraint.getCurrentPosition` after checking the sub types (Jolt's StaticCast). Another constraint
//!   type is Jolt's `JPH_ASSERT(false, "Unsupported")` (a panic with asserts on, user input can reach it) followed by
//!   `return false`.
//! - `fmod(x, 2.0f * JPH_PI)` is `@rem(x, 2.0 * math.pi)`, the exact remainder with the sign of x (see GearConstraint.zig).

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
const Color = @import("../../Core/Color.zig").Color;
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
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
const SliderConstraint = @import("SliderConstraint.zig").SliderConstraint;
const RackAndPinionConstraintPart = @import("ConstraintPart/RackAndPinionConstraintPart.zig").RackAndPinionConstraintPart;

/// Rack and pinion constraint (slider & gear) settings
pub const RackAndPinionConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(RackAndPinionConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "RackAndPinionConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .enumAttr("mSpace", .space),
            .attr("mHingeAxis", .hinge_axis),
            .attr("mSliderAxis", .slider_axis),
            .attr("mRatio", .ratio),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 (pinion) constraint reference frame (space determined by mSpace).
    hinge_axis: Vec3 = Vec3.axisX(),

    /// Body 2 (rack) constraint reference frame (space determined by mSpace)
    slider_axis: Vec3 = Vec3.axisX(),

    /// Ratio between the rack and pinion, see SetRatio.
    ratio: f32 = 1.0,

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) RackAndPinionConstraintSettings {
        return .{ .base = .init(RackAndPinionConstraintSettings, allocator) };
    }

    /// new RackAndPinionConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*RackAndPinionConstraintSettings {
        const self = try allocator.create(RackAndPinionConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const RackAndPinionConstraintSettings) RackAndPinionConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *RackAndPinionConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const RackAndPinionConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *RackAndPinionConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const RackAndPinionConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const RackAndPinionConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.hinge_axis);
        stream.write(self.slider_axis);
        stream.write(self.ratio);
    }

    /// Create an instance of this constraint.
    /// Body1 should be the pinion (gear) and body 2 the rack (slider).
    pub fn createConstraint(self: *const RackAndPinionConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) TwoBodyConstraintSettings.CreateError!*TwoBodyConstraint {
        const constraint = try RackAndPinionConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    /// Defines the ratio between the rotation of the pinion and the translation of the rack.
    /// The ratio is defined as: PinionRotation(t) = ratio * RackTranslation(t)
    /// @param inNumTeethRack Number of teeth that the rack has
    /// @param inRackLength Length of the rack
    /// @param inNumTeethPinion Number of teeth the pinion has
    pub fn setRatio(self: *RackAndPinionConstraintSettings, num_teeth_rack: i32, rack_length: f32, num_teeth_pinion: i32) void {
        self.ratio = 2.0 * math.pi * @as(f32, @floatFromInt(num_teeth_rack)) / (rack_length * @as(f32, @floatFromInt(num_teeth_pinion)));
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *RackAndPinionConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.hinge_axis);
        stream.read(&self.slider_axis);
        stream.read(&self.ratio);
    }
};

/// A rack and pinion constraint constrains the rotation of body1 to the translation of body 2.
/// Note that this constraint needs to be used in conjunction with a hinge constraint for body 1 and a slider constraint for body 2.
pub const RackAndPinionConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .drawConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space hinge axis
    local_space_hinge_axis: Vec3,

    // Local space sliding direction
    local_space_slider_axis: Vec3,

    // Ratio between rack and pinion
    ratio: f32,

    // The constraints that constrain the rack and pinion (a slider and a hinge), optional and used to calculate the position error and fix numerical drift.
    pinion_constraint: RefConst(Constraint) = .empty,
    rack_constraint: RefConst(Constraint) = .empty,

    // RUN TIME PROPERTIES FOLLOW

    // World space hinge axis
    world_space_hinge_axis: Vec3 = undefined,

    // World space sliding direction
    world_space_slider_axis: Vec3 = undefined,

    // The constraint parts
    rack_and_pinion_constraint_part: RackAndPinionConstraintPart = .{},

    /// Construct gear constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const RackAndPinionConstraintSettings) RackAndPinionConstraint {
        var self: RackAndPinionConstraint = .{
            .base = .init(RackAndPinionConstraint, allocator, body1, body2, &settings.base),
            .local_space_hinge_axis = settings.hinge_axis,
            .local_space_slider_axis = settings.slider_axis,
            .ratio = settings.ratio,
        };

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            self.local_space_hinge_axis = body1.getInverseCenterOfMassTransform().multiply3x3(self.local_space_hinge_axis).normalized();
            self.local_space_slider_axis = body2.getInverseCenterOfMassTransform().multiply3x3(self.local_space_slider_axis).normalized();
        }

        return self;
    }

    /// new RackAndPinionConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const RackAndPinionConstraintSettings) Allocator.Error!*RackAndPinionConstraint {
        const self = try allocator.create(RackAndPinionConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    /// Destructor: releases the references to the hinge and the slider (RefConst members)
    pub fn destruct(self: *RackAndPinionConstraint) void {
        self.pinion_constraint.deinit();
        self.rack_constraint.deinit();
    }

    pub fn asConstraint(self: *RackAndPinionConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const RackAndPinionConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *RackAndPinionConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const RackAndPinionConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const RackAndPinionConstraint) ConstraintSubType {
        _ = self;
        return .rack_and_pinion;
    }

    pub fn notifyShapeChanged(self: *RackAndPinionConstraint, body_id: BodyID, delta_com: Vec3) void {
        // Nothing
        _ = self;
        _ = body_id;
        _ = delta_com;
    }

    pub fn setupVelocityConstraint(self: *RackAndPinionConstraint, delta_time: f32) void {
        _ = delta_time;

        // Calculate constraint properties that are constant while bodies don't move
        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.calculateConstraintProperties(rotation1, rotation2);
    }

    pub fn resetWarmStart(self: *RackAndPinionConstraint) void {
        self.rack_and_pinion_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *RackAndPinionConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.rack_and_pinion_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *RackAndPinionConstraint, delta_time: f32) bool {
        _ = delta_time;

        return self.rack_and_pinion_constraint_part.solveVelocityConstraint(self.base.body1, self.world_space_hinge_axis, self.base.body2, self.world_space_slider_axis, self.ratio);
    }

    pub fn solvePositionConstraint(self: *RackAndPinionConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        if (self.rack_constraint.get() == null or self.pinion_constraint.get() == null)
            return false;
        const pinion_constraint = self.pinion_constraint.get().?;
        const rack_constraint = self.rack_constraint.get().?;

        var rotation: f32 = undefined;
        if (pinion_constraint.getSubType() == .hinge) {
            rotation = virtual.downcast(HingeConstraint, virtual.downcast(TwoBodyConstraint, pinion_constraint)).getCurrentAngle();
        } else {
            if (Core.enable_asserts) @panic("Unsupported");
            return false;
        }

        var translation: f32 = undefined;
        if (rack_constraint.getSubType() == .slider) {
            translation = virtual.downcast(SliderConstraint, virtual.downcast(TwoBodyConstraint, rack_constraint)).getCurrentPosition();
        } else {
            if (Core.enable_asserts) @panic("Unsupported");
            return false;
        }

        const err = math.centerAngleAroundZero(@rem(rotation - self.ratio * translation, 2.0 * math.pi));
        if (err == 0.0)
            return false;

        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.calculateConstraintProperties(rotation1, rotation2);
        return self.rack_and_pinion_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, err, baumgarte);
    }

    // See Constraint::DrawConstraint (JPH_DEBUG_RENDERER)
    pub const drawConstraint = if (Core.debug_renderer) drawConstraintImpl else {};
    fn drawConstraintImpl(self: *const RackAndPinionConstraint, renderer: *DebugRenderer) void {
        const transform1 = self.base.body1.getCenterOfMassTransform();
        const transform2 = self.base.body2.getCenterOfMassTransform();

        // Draw constraint axis
        renderer.drawArrow(transform1.getTranslation(), transform1.mulVec3(self.local_space_hinge_axis), Color.green, 0.01);
        renderer.drawArrow(transform2.getTranslation(), transform2.mulVec3(self.local_space_slider_axis), Color.blue, 0.01);
    }

    pub fn saveState(self: *const RackAndPinionConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.rack_and_pinion_constraint_part.saveState(stream);
    }

    pub fn restoreState(self: *RackAndPinionConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.rack_and_pinion_constraint_part.restoreState(stream);
    }

    pub fn getConstraintSettings(self: *const RackAndPinionConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try RackAndPinionConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.hinge_axis = self.local_space_hinge_axis;
        settings.slider_axis = self.local_space_slider_axis;
        settings.ratio = self.ratio;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const RackAndPinionConstraint) Mat44 {
        const perp = self.local_space_hinge_axis.getNormalizedPerpendicular();
        return Mat44.init(Vec4.fromVec3W(self.local_space_hinge_axis, 0), Vec4.fromVec3W(perp, 0), Vec4.fromVec3W(self.local_space_hinge_axis.cross(perp), 0), Vec4.init(0, 0, 0, 1));
    }

    pub fn getConstraintToBody2Matrix(self: *const RackAndPinionConstraint) Mat44 {
        const perp = self.local_space_slider_axis.getNormalizedPerpendicular();
        return Mat44.init(Vec4.fromVec3W(self.local_space_slider_axis, 0), Vec4.fromVec3W(perp, 0), Vec4.fromVec3W(self.local_space_slider_axis.cross(perp), 0), Vec4.init(0, 0, 0, 1));
    }

    /// The constraints that constrain the rack and pinion (a slider and a hinge), optional and used to calculate the position error and fix numerical drift.
    pub fn setConstraints(self: *RackAndPinionConstraint, pinion: ?*const Constraint, rack: ?*const Constraint) void {
        self.pinion_constraint.set(pinion);
        self.rack_constraint.set(rack);
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambda(self: *const RackAndPinionConstraint) f32 {
        return self.rack_and_pinion_constraint_part.getTotalLambda();
    }

    // Internal helper function to calculate the values below
    fn calculateConstraintProperties(self: *RackAndPinionConstraint, rotation1: Mat44, rotation2: Mat44) void {
        // Calculate world space normals
        self.world_space_hinge_axis = rotation1.mulVec3(self.local_space_hinge_axis);
        self.world_space_slider_axis = rotation2.mulVec3(self.local_space_slider_axis);

        self.rack_and_pinion_constraint_part.calculateConstraintProperties(self.base.body1, self.world_space_hinge_axis, self.base.body2, self.world_space_slider_axis, self.ratio);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig and SimulationParity.zig, simulations in
// ZoltTests/Physics/RackAndPinionConstraintTests.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const TestContext = @import("ConstraintPart/ConstraintPartTestContext.zig");
const HingeConstraintSettings = @import("HingeConstraint.zig").HingeConstraintSettings;
const SliderConstraintSettings = @import("SliderConstraint.zig").SliderConstraintSettings;
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
fn testCreateHinge(allocator: Allocator, body: *Body, axis: Vec3) TwoBodyConstraintSettings.CreateError!Ref(Constraint) {
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

/// A slider from the world to `body` along `axis` at its center of mass (world space, the current position of the body
/// is position 0), one reference owned by the caller
fn testCreateSlider(allocator: Allocator, body: *Body, axis: Vec3) TwoBodyConstraintSettings.CreateError!Ref(Constraint) {
    var settings = SliderConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = body.getCenterOfMassPosition();
    settings.point2 = settings.point1;
    settings.slider_axis1 = axis;
    settings.slider_axis2 = axis;
    settings.normal_axis1 = axis.getNormalizedPerpendicular();
    settings.normal_axis2 = settings.normal_axis1;
    return .init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, Body.fixedToWorld(), body)).base);
}

/// The current angle of a hinge made by testCreateHinge
fn testHingeAngle(hinge: Ref(Constraint)) f32 {
    return virtual.downcast(HingeConstraint, virtual.downcast(TwoBodyConstraint, hinge.get().?)).getCurrentAngle();
}

/// The current position of a slider made by testCreateSlider
fn testSliderPosition(slider: Ref(Constraint)) f32 {
    return virtual.downcast(SliderConstraint, virtual.downcast(TwoBodyConstraint, slider.get().?)).getCurrentPosition();
}

test "RackAndPinionConstraintSettings: defaults, SetRatio, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = RackAndPinionConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.hinge_axis.eql(Vec3.axisX()) and settings.slider_axis.eql(Vec3.axisX()));
    try testing.expectEqual(@as(f32, 1.0), settings.ratio);
    try testing.expectEqualStrings("RackAndPinionConstraintSettings", settings.asSettings().vtable.rtti.name);

    // SetRatio: 2 pi * (teeth of the rack) / (rack length * teeth of the pinion), in Jolt's order of operations
    var two_pi: f32 = 2.0 * math.pi;
    var thirty: f32 = 30.0;
    var length: f32 = 4.0;
    var ten: f32 = 10.0;
    _ = .{ &two_pi, &thirty, &length, &ten };
    settings.setRatio(30, 4.0, 10);
    try testing.expectEqual(two_pi * thirty / (length * ten), settings.ratio);
    try testing.expect(@abs(settings.ratio - 0.75 * 2.0 * math.pi) < 1.0e-6);
    settings.setRatio(7, 0.3, 13);
    length = 0.3;
    const seven: f32 = 7.0;
    const thirteen: f32 = 13.0;
    try testing.expectEqual(two_pi * seven / (length * thirteen), settings.ratio);
    settings.setRatio(-5, 2.0, 5); // Jolt does not check the numbers
    try testing.expectEqual(-math.pi, settings.ratio);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 9;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 4;
    settings.base.base.draw_constraint_size = 0.5;
    settings.space = .local_to_body_com;
    settings.hinge_axis = Vec3.axisZ();
    settings.slider_axis = Vec3.init(0.6, 0, 0.8);
    settings.ratio = 12.5;

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), space (int), the axes (3 floats each),
    // the ratio
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 2 * 12 + 4), bytes.len);
    try testing.expectEqual(RTTI.hashName("RackAndPinionConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 12.5))), std.mem.readInt(u32, bytes[49..53], .little));

    // Restore through RegisterTypes.factory
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(RackAndPinionConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 9), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u32, 3), restored.base.base.num_velocity_steps_override);
        try testing.expectEqual(@as(u32, 4), restored.base.base.num_position_steps_override);
        try testing.expectEqual(@as(f32, 0.5), restored.base.base.draw_constraint_size);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.hinge_axis.eql(settings.hinge_axis) and restored.slider_axis.eql(settings.slider_axis));
        try testing.expectEqual(@as(f32, 12.5), restored.ratio);
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
    var copy = RackAndPinionConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.slider_axis.eql(settings.slider_axis));
    try testing.expectEqual(@as(f32, 12.5), copy.ratio);
    try testing.expectEqual(@as(u32, 9), copy.base.base.constraint_priority);
}

/// a . w1 - ratio b . v2: the velocity error of the constraint
fn testRelativeVelocity(c: *const RackAndPinionConstraint) f32 {
    return c.world_space_hinge_axis.dot(c.base.body1.getAngularVelocity()) - c.ratio * c.world_space_slider_axis.dot(c.base.body2.getLinearVelocity());
}

test "RackAndPinionConstraint: virtual dispatch, space conversions, solver, settings and state" {
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
    var settings = RackAndPinionConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.hinge_axis = Vec3.init(1, 2, 3);
    settings.slider_axis = Vec3.init(0, 0, -2);
    settings.setRatio(20, 2.0, 10);
    settings.base.base.user_data = 77;

    // TwoBodyConstraintSettings::Create through the vtable
    var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
    defer constraint.deinit();
    const base = constraint.get().?;
    const two_body = virtual.downcast(TwoBodyConstraint, base);
    const c = virtual.downcast(RackAndPinionConstraint, two_body);
    try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
    try testing.expectEqual(ConstraintSubType.rack_and_pinion, base.getSubType());
    try testing.expectEqual(@as(u64, 77), base.getUserData());
    try testing.expect(two_body.getBody1() == body1 and two_body.getBody2() == body2);
    try testing.expectEqual(settings.ratio, c.ratio);

    // The constructor took the axes to the local space of the bodies
    try testing.expect(c.local_space_hinge_axis.eql(body1.getInverseCenterOfMassTransform().multiply3x3(settings.hinge_axis).normalized()));
    try testing.expect(c.local_space_slider_axis.eql(body2.getInverseCenterOfMassTransform().multiply3x3(settings.slider_axis).normalized()));
    try testing.expect(body2.getRotation().mulVec3(c.local_space_slider_axis).isClose(Vec3.init(0, 0, -1), .{ .max_dist_sq = 1.0e-10 }));

    // The constraint frames: the hinge / slider axis, a perpendicular and their cross product, no translation
    const matrices = [_]Mat44{ two_body.getConstraintToBody1Matrix(), two_body.getConstraintToBody2Matrix() };
    const axes = [_]Vec3{ c.local_space_hinge_axis, c.local_space_slider_axis };
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
        const s = virtual.downcast(RackAndPinionConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
        try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
        try testing.expectEqual(@as(u64, 77), s.base.base.user_data);
        try testing.expect(s.hinge_axis.eql(c.local_space_hinge_axis) and s.slider_axis.eql(c.local_space_slider_axis));
        try testing.expectEqual(c.ratio, s.ratio);
        var again = Ref(Constraint).init(&(try s.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer again.deinit();
        const c2 = virtual.downcast(RackAndPinionConstraint, virtual.downcast(TwoBodyConstraint, again.get().?));
        try testing.expect(c2.local_space_hinge_axis.eql(c.local_space_hinge_axis) and c2.local_space_slider_axis.eql(c.local_space_slider_axis));
        try testing.expectEqual(c.ratio, c2.ratio);
    }

    // Solver: a . w1 = ratio b . v2 (the impulse is consistent with the Jacobian: one iteration solves it)
    try testing.expect(base.isActive());
    base.setupVelocityConstraint(dt);
    try testing.expect(c.rack_and_pinion_constraint_part.isActive());
    try testing.expect(c.world_space_hinge_axis.eql(Mat44.rotationQuat(body1.getRotation()).mulVec3(c.local_space_hinge_axis)));
    try testing.expect(c.world_space_slider_axis.eql(Mat44.rotationQuat(body2.getRotation()).mulVec3(c.local_space_slider_axis)));
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(@abs(testRelativeVelocity(c)) > 0.1);
    try testing.expect(base.solveVelocityConstraint(dt));
    try testing.expect(@abs(testRelativeVelocity(c)) < 1.0e-4);
    try testing.expect(c.getTotalLambda() != 0.0);

    // No hinge / slider set: no position correction (Jolt returns false before reading anything)
    const before1 = TestContext.BodySnapshot.take(body1);
    const before2 = TestContext.BodySnapshot.take(body2);
    try testing.expect(!base.solvePositionConstraint(dt, 0.2));
    try TestContext.checkUnmoved(body1, before1);
    try TestContext.checkUnmoved(body2, before2);

    // SaveState / RestoreState: enabled + the lambda of the rack and pinion part
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 1 + 4), recorder.getData().len);
    const lambda = c.getTotalLambda();
    base.resetWarmStart();
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambda());
    try testing.expect(!c.rack_and_pinion_constraint_part.isActive());
    base.restoreState(recorder.recorder());
    try testing.expectEqual(lambda, c.getTotalLambda());

    // Warm start applies the scaled lambda of the previous frame
    base.setupVelocityConstraint(dt);
    const v2 = body2.getLinearVelocity();
    base.warmStartVelocityConstraint(0.5);
    try testing.expectEqual(lambda * 0.5, c.getTotalLambda());
    try testing.expect(!body2.getLinearVelocity().eql(v2));

    // NotifyShapeChanged does nothing
    const a1 = c.local_space_hinge_axis;
    const a2 = c.local_space_slider_axis;
    base.notifyShapeChanged(body1.getID(), Vec3.init(0.5, 0, 0));
    base.notifyShapeChanged(body2.getID(), Vec3.init(0, 1, 0));
    try testing.expect(c.local_space_hinge_axis.eql(a1) and c.local_space_slider_axis.eql(a2));

    // A disabled constraint is not active (TwoBodyConstraint::IsActive)
    base.setEnabled(false);
    try testing.expect(!base.isActive());
}

test "RackAndPinionConstraint: local space settings" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);

    // Local space: the axes are used as they are; any space value other than world space is local space
    for ([_]ConstraintSpace{ .local_to_body_com, @enumFromInt(-3) }) |space| {
        var settings = RackAndPinionConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.space = space;
        settings.hinge_axis = Vec3.axisY();
        settings.slider_axis = Vec3.init(0, 0.6, -0.8);
        settings.ratio = -0.75;
        var c = RackAndPinionConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        try testing.expect(c.local_space_hinge_axis.eql(Vec3.axisY()) and c.local_space_slider_axis.eql(Vec3.init(0, 0.6, -0.8)));
        try testing.expectEqual(@as(f32, -0.75), c.ratio);
        try testing.expectEqual(@as(f32, 0.0), c.getTotalLambda());
    }
}

test "RackAndPinionConstraint: position correction through the hinge and the slider" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const dt: f32 = 1.0 / 60.0;

    // A pinion on a hinge around Z and a rack on a slider along X, both to the world
    const pinion_body = try context.createBody(.dynamic, .{ .position = RVec3.init(0, 2, 0), .half_extent = Vec3.init(1, 1, 0.25) });
    const rack_body = try context.createBody(.dynamic, .{ .position = RVec3.init(0, 0.5, 0), .half_extent = Vec3.init(3, 0.25, 0.25) });
    defer context.destroyBodies(&.{ pinion_body, rack_body });
    var pinion = try testCreateHinge(allocator, pinion_body, Vec3.axisZ());
    defer pinion.deinit();
    var rack = try testCreateSlider(allocator, rack_body, Vec3.axisX());
    defer rack.deinit();

    var settings = RackAndPinionConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.hinge_axis = Vec3.axisZ();
    settings.slider_axis = Vec3.axisX();
    settings.setRatio(10, 6.0, 10); // One turn of the pinion moves the rack by 6 / 10 * 10 = 6
    var rp = RackAndPinionConstraint.init(allocator, pinion_body, rack_body, &settings);
    defer rp.asConstraint().deinit();
    const base = rp.asConstraint();

    // Without both the hinge and the slider there is no position correction
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));
    rp.setConstraints(pinion.get(), null);
    try testing.expectEqual(@as(u32, 2), pinion.get().?.getRefCount()); // The constraint holds a reference
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));
    rp.setConstraints(null, rack.get());
    try testing.expectEqual(@as(u32, 1), pinion.get().?.getRefCount());
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));
    rp.setConstraints(pinion.get(), rack.get());
    try testing.expectEqual(@as(u32, 2), pinion.get().?.getRefCount());
    try testing.expectEqual(@as(u32, 2), rack.get().?.getRefCount());

    // At the angle and position the constraints were created with: no error, no correction
    try testing.expectEqual(@as(f32, 0.0), testHingeAngle(pinion));
    try testing.expectEqual(@as(f32, 0.0), testSliderPosition(rack));
    try testing.expect(!base.solvePositionConstraint(dt, 0.5));

    // Move the rack: the error rotation - ratio * translation is corrected by both bodies, the impulse is consistent
    // with the Jacobian so Baumgarte 0.5 removes half of the error (linearized)
    rack_body.setPositionAndRotationInternal(rack_body.getPosition().add(RVec3.init(0.1, 0, 0)), Quat.identity(), .{});
    const error_before = testHingeAngle(pinion) - rp.ratio * testSliderPosition(rack);
    try testing.expect(@abs(error_before + 0.1 * rp.ratio) < 1.0e-5);
    try testing.expect(base.solvePositionConstraint(dt, 0.5));
    const error_after = testHingeAngle(pinion) - rp.ratio * testSliderPosition(rack);
    try testing.expect(@abs(error_after - 0.5 * error_before) < 1.0e-4);
    try testing.expect(testHingeAngle(pinion) > 0.0 and testSliderPosition(rack) < 0.1);

    // The error is taken modulo 2 pi and centered around zero: pinion at 1, rack at -3 with ratio 2 pi / 6 is an error
    // of 1 + pi - 2 pi; the correction is the one of the part for that error
    for ([_]f32{ 1.0, -1.0 }) |sign| {
        pinion_body.setPositionAndRotationInternal(pinion_body.getPosition(), Quat.rotation(Vec3.axisZ(), sign * 1.0), .{});
        rack_body.setPositionAndRotationInternal(RVec3.init(-sign * 3.0, 0.5, 0), Quat.identity(), .{});
        const expected_error = sign * (1.0 + 3.0 * rp.ratio - 2.0 * math.pi);
        try testing.expect(@abs(expected_error - sign * (1.0 - math.pi)) < 1.0e-5);
        try testing.expect(@abs(expected_error) < math.pi);
        var part: RackAndPinionConstraintPart = .{};
        part.calculateConstraintProperties(pinion_body, Vec3.axisZ(), rack_body, Vec3.axisX(), rp.ratio);
        const expected_step = -part.effective_mass * 0.5 * expected_error * part.inv_i1_a.getZ();
        const angle = testHingeAngle(pinion);
        try testing.expect(base.solvePositionConstraint(dt, 0.5));
        try testing.expect(@abs(testHingeAngle(pinion) - angle - expected_step) < 1.0e-5);
    }

    // A constraint of another type: Jolt asserts ("Unsupported", user input) and returns false in its release build
    if (!Core.enable_asserts) {
        var point_settings = PointConstraintSettings.init(allocator);
        defer point_settings.asSettings().deinit();
        var point = Ref(Constraint).init(&(try point_settings.asTwoBodySettingsConst().createConstraint(allocator, Body.fixedToWorld(), pinion_body)).base);
        defer point.deinit();
        rp.setConstraints(point.get(), rack.get());
        try testing.expect(!base.solvePositionConstraint(dt, 0.5));
        rp.setConstraints(pinion.get(), point.get());
        try testing.expect(!base.solvePositionConstraint(dt, 0.5));
        rp.setConstraints(rack.get(), pinion.get()); // Swapped: a slider as pinion
        try testing.expect(!base.solvePositionConstraint(dt, 0.5));
        rp.setConstraints(pinion.get(), rack.get());
    }
}

test "RackAndPinionConstraint: the destructor releases the hinge and the slider" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);

    var pinion = try testCreateHinge(allocator, bodies[0], Vec3.axisX());
    defer pinion.deinit();
    var settings = RackAndPinionConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    {
        var rp = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, bodies[0], bodies[1])).base);
        defer rp.deinit();
        const c = virtual.downcast(RackAndPinionConstraint, virtual.downcast(TwoBodyConstraint, rp.get().?));

        // The constraint may hold the only reference to the slider (Jolt's RefConst)
        var rack = try testCreateSlider(allocator, bodies[1], Vec3.axisY());
        c.setConstraints(pinion.get(), rack.get());
        rack.deinit();
        try testing.expectEqual(@as(u32, 1), c.rack_constraint.get().?.getRefCount());

        // Setting the same constraints again keeps one reference each
        c.setConstraints(pinion.get(), c.rack_constraint.get());
        try testing.expectEqual(@as(u32, 2), pinion.get().?.getRefCount());
        try testing.expectEqual(@as(u32, 1), c.rack_constraint.get().?.getRefCount());
    }
    try testing.expectEqual(@as(u32, 1), pinion.get().?.getRefCount());
}

test "RackAndPinionConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try RackAndPinionConstraintSettings.create(allocator)).asSettings());
            defer settings.deinit();
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, bodies[0], bodies[1])).base);
            defer constraint.deinit();
            var pinion = try testCreateHinge(allocator, bodies[0], Vec3.axisX());
            defer pinion.deinit();
            var rack = try testCreateSlider(allocator, bodies[1], Vec3.axisX());
            defer rack.deinit();
            virtual.downcast(RackAndPinionConstraint, virtual.downcast(TwoBodyConstraint, constraint.get().?)).setConstraints(pinion.get(), rack.get());
            var copy = try constraint.get().?.getConstraintSettings();
            defer copy.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
