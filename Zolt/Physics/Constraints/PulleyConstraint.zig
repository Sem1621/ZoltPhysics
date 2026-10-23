//! Port of: Jolt/Physics/Constraints/PulleyConstraint.h, Jolt/Physics/Constraints/PulleyConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PulleyConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint): a gated declaration (RemainingFeaturesArchitecture.md R2), the text through
//! StringTools.cppFixed (R7).
//!
//! - `SetLength` asserts `inMinLength >= 0.0f && inMinLength <= inMaxLength`, which user input can violate: the
//!   assert only runs with `Core.enable_asserts`, ReleaseFast keeps Jolt's release behavior. The constructor does not
//!   call it (Jolt sets the members directly), so settings with min > max or a negative min that is not -1 are taken
//!   as they are.
//! - Like Jolt, a kinematic body reaches IndependentAxisConstraintPart, which asserts that it is dynamic (see the file
//!   comment of IndependentAxisConstraintPart.zig): a pulley connects static or dynamic bodies.
//! - `mMinLambda` / `mMaxLambda` are uninitialized in Jolt until SetupVelocityConstraint sets them (`undefined`): the
//!   velocity solver only reads them while the part is active, which only the branch that sets them makes it.

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
const StringTools = @import("../../Core/StringTools.zig");
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
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
const IndependentAxisConstraintPart = @import("ConstraintPart/IndependentAxisConstraintPart.zig").IndependentAxisConstraintPart;

/// Pulley constraint settings, used to create a pulley constraint.
/// A pulley connects two bodies via two fixed world points to each other similar to a distance constraint.
/// We define Length1 = |BodyPoint1 - FixedPoint1| where Body1 is a point on body 1 in world space and FixedPoint1 a fixed point in world space
/// Length2 = |BodyPoint2 - FixedPoint2|
/// The constraint keeps the two line segments constrained so that
/// MinDistance <= Length1 + Ratio * Length2 <= MaxDistance
pub const PulleyConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PulleyConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "PulleyConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .enumAttr("mSpace", .space),
            .attr("mBodyPoint1", .body_point1),
            .attr("mFixedPoint1", .fixed_point1),
            .attr("mBodyPoint2", .body_point2),
            .attr("mFixedPoint2", .fixed_point2),
            .attr("mRatio", .ratio),
            .attr("mMinLength", .min_length),
            .attr("mMaxLength", .max_length),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, specified properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint attachment point (space determined by mSpace).
    body_point1: RVec3 = RVec3.zero(),

    /// Fixed world point to which body 1 is connected (always world space)
    fixed_point1: RVec3 = RVec3.zero(),

    /// Body 2 constraint attachment point (space determined by mSpace)
    body_point2: RVec3 = RVec3.zero(),

    /// Fixed world point to which body 2 is connected (always world space)
    fixed_point2: RVec3 = RVec3.zero(),

    /// Ratio between the two line segments (see formula above), can be used to create a block and tackle
    ratio: f32 = 1.0,

    /// The minimum length of the line segments (see formula above), use -1 to calculate the length based on the positions of the objects when the constraint is created.
    min_length: f32 = 0.0,

    /// The maximum length of the line segments (see formula above), use -1 to calculate the length based on the positions of the objects when the constraint is created.
    max_length: f32 = -1.0,

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) PulleyConstraintSettings {
        return .{ .base = .init(PulleyConstraintSettings, allocator) };
    }

    /// new PulleyConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*PulleyConstraintSettings {
        const self = try allocator.create(PulleyConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const PulleyConstraintSettings) PulleyConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *PulleyConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const PulleyConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *PulleyConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const PulleyConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const PulleyConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.body_point1);
        stream.write(self.fixed_point1);
        stream.write(self.body_point2);
        stream.write(self.fixed_point2);
        stream.write(self.ratio);
        stream.write(self.min_length);
        stream.write(self.max_length);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const PulleyConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) TwoBodyConstraintSettings.CreateError!*TwoBodyConstraint {
        const constraint = try PulleyConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *PulleyConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.body_point1);
        stream.read(&self.fixed_point1);
        stream.read(&self.body_point2);
        stream.read(&self.fixed_point2);
        stream.read(&self.ratio);
        stream.read(&self.min_length);
        stream.read(&self.max_length);
    }
};

/// A pulley constraint.
pub const PulleyConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .drawConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions on the bodies
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // World space fixed positions
    fixed_position1: RVec3,
    fixed_position2: RVec3,

    /// Ratio between the two line segments
    ratio: f32,

    // The minimum/maximum length of the line segments
    min_length: f32,
    max_length: f32,

    // RUN TIME PROPERTIES FOLLOW

    // World space positions and normal
    world_space_position1: RVec3 = undefined,
    world_space_position2: RVec3 = undefined,
    world_space_normal1: Vec3 = undefined,
    world_space_normal2: Vec3 = undefined,

    // Depending on if the length < min or length > max we can apply forces to prevent further violations
    min_lambda: f32 = undefined,
    max_lambda: f32 = undefined,

    // The constraint part
    independent_axis_constraint_part: IndependentAxisConstraintPart = .{},

    /// Construct pulley constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const PulleyConstraintSettings) PulleyConstraint {
        var self: PulleyConstraint = .{
            .base = .init(PulleyConstraint, allocator, body1, body2, &settings.base),
            .fixed_position1 = settings.fixed_point1,
            .fixed_position2 = settings.fixed_point2,
            .ratio = settings.ratio,
            .min_length = settings.min_length,
            .max_length = settings.max_length,
        };

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            self.local_space_position1 = body1.getInverseCenterOfMassTransform().mulRVec3(settings.body_point1).toVec3();
            self.local_space_position2 = body2.getInverseCenterOfMassTransform().mulRVec3(settings.body_point2).toVec3();
            self.world_space_position1 = settings.body_point1;
            self.world_space_position2 = settings.body_point2;
        } else {
            // If properties were specified in local space, we need to calculate world space positions
            self.local_space_position1 = settings.body_point1.toVec3();
            self.local_space_position2 = settings.body_point2.toVec3();
            self.world_space_position1 = body1.getCenterOfMassTransform().mulRVec3(settings.body_point1);
            self.world_space_position2 = body2.getCenterOfMassTransform().mulRVec3(settings.body_point2);
        }

        // Calculate min/max length if it was not provided
        const current_length = self.getCurrentLength();
        if (self.min_length < 0.0)
            self.min_length = current_length;
        if (self.max_length < 0.0)
            self.max_length = current_length;

        // Initialize the normals to a likely valid axis in case the fixed points overlap with the attachment points (most likely the fixed points are above both bodies)
        self.world_space_normal1 = Vec3.axisY().negate();
        self.world_space_normal2 = self.world_space_normal1;

        return self;
    }

    /// new PulleyConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const PulleyConstraintSettings) Allocator.Error!*PulleyConstraint {
        const self = try allocator.create(PulleyConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *PulleyConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const PulleyConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *PulleyConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const PulleyConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const PulleyConstraint) ConstraintSubType {
        _ = self;
        return .pulley;
    }

    pub fn notifyShapeChanged(self: *PulleyConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    // Calculates world positions and normals and returns current length
    fn calculatePositionsNormalsAndLength(self: *PulleyConstraint) f32 {
        // Update world space positions (the bodies may have moved)
        self.world_space_position1 = self.base.body1.getCenterOfMassTransform().mulVec3(self.local_space_position1);
        self.world_space_position2 = self.base.body2.getCenterOfMassTransform().mulVec3(self.local_space_position2);

        // Calculate world space normals
        const delta1 = self.world_space_position1.sub(self.fixed_position1).toVec3();
        const delta1_len = delta1.length();
        if (delta1_len > 0.0)
            self.world_space_normal1 = delta1.divScalar(delta1_len);

        const delta2 = self.world_space_position2.sub(self.fixed_position2).toVec3();
        const delta2_len = delta2.length();
        if (delta2_len > 0.0)
            self.world_space_normal2 = delta2.divScalar(delta2_len);

        // Calculate length
        return delta1_len + self.ratio * delta2_len;
    }

    // Internal helper function to calculate the values below
    fn calculateConstraintProperties(self: *PulleyConstraint) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Calculate attachment points relative to COM
        const r1 = self.world_space_position1.sub(body1.getCenterOfMassPosition()).toVec3();
        const r2 = self.world_space_position2.sub(body2.getCenterOfMassPosition()).toVec3();

        self.independent_axis_constraint_part.calculateConstraintProperties(body1, body2, r1, self.world_space_normal1, r2, self.world_space_normal2, self.ratio);
    }

    pub fn setupVelocityConstraint(self: *PulleyConstraint, delta_time: f32) void {
        _ = delta_time;

        // Determine if the constraint is active
        const current_length = self.calculatePositionsNormalsAndLength();
        const min_length_violation = current_length <= self.min_length;
        const max_length_violation = current_length >= self.max_length;
        if (min_length_violation or max_length_violation) {
            // Determine max lambda based on if the length is too big or small
            self.min_lambda = if (max_length_violation) -math.flt_max else 0.0;
            self.max_lambda = if (min_length_violation) math.flt_max else 0.0;

            self.calculateConstraintProperties();
        } else self.independent_axis_constraint_part.deactivate();
    }

    pub fn resetWarmStart(self: *PulleyConstraint) void {
        self.independent_axis_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *PulleyConstraint, warm_start_impulse_ratio: f32) void {
        self.independent_axis_constraint_part.warmStart(self.base.body1, self.base.body2, self.world_space_normal1, self.world_space_normal2, self.ratio, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *PulleyConstraint, delta_time: f32) bool {
        _ = delta_time;
        if (self.independent_axis_constraint_part.isActive())
            return self.independent_axis_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.world_space_normal1, self.world_space_normal2, self.ratio, self.min_lambda, self.max_lambda)
        else
            return false;
    }

    pub fn solvePositionConstraint(self: *PulleyConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        // Calculate new length (bodies may have changed)
        const current_length = self.calculatePositionsNormalsAndLength();

        var position_error: f32 = 0.0;
        if (current_length < self.min_length)
            position_error = current_length - self.min_length
        else if (current_length > self.max_length)
            position_error = current_length - self.max_length;

        if (position_error != 0.0) {
            // Update constraint properties (bodies may have moved)
            self.calculateConstraintProperties();

            return self.independent_axis_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.world_space_normal1, self.world_space_normal2, self.ratio, position_error, baumgarte);
        }

        return false;
    }

    // See Constraint::DrawConstraint (JPH_DEBUG_RENDERER)
    pub const drawConstraint = if (Core.debug_renderer) drawConstraintImpl else {};
    fn drawConstraintImpl(self: *const PulleyConstraint, renderer: *DebugRenderer) void {
        // Color according to length vs min/max length
        const current_length = self.getCurrentLength();
        var color = Color.green;
        if (current_length < self.min_length) {
            color = Color.yellow;
        } else if (current_length > self.max_length) {
            color = Color.red;
        }

        // Draw constraint
        renderer.drawLine(self.world_space_position1, self.fixed_position1, color);
        renderer.drawLine(self.fixed_position1, self.fixed_position2, color);
        renderer.drawLine(self.fixed_position2, self.world_space_position2, color);

        // Draw current length
        // StringFormat("%.2f", ...): its buffer holds 1023 characters
        var text_buffer: [1023]u8 = undefined;
        var writer: std.Io.Writer = .fixed(&text_buffer);
        writer.print("{f}", .{StringTools.cppFixed(current_length, 2)}) catch {};
        renderer.drawText3D(self.fixed_position1.add(self.fixed_position2).mulScalar(0.5), writer.buffered(), .{});
    }

    pub fn saveState(self: *const PulleyConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.independent_axis_constraint_part.saveState(stream);
        stream.write(self.world_space_normal1); // When distance to fixed point = 0, the normal is used from last frame so we need to store it
        stream.write(self.world_space_normal2);
    }

    pub fn restoreState(self: *PulleyConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.independent_axis_constraint_part.restoreState(stream);
        stream.read(&self.world_space_normal1);
        stream.read(&self.world_space_normal2);
    }

    pub fn getConstraintSettings(self: *const PulleyConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try PulleyConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.body_point1 = RVec3.fromVec3(self.local_space_position1);
        settings.fixed_point1 = self.fixed_position1;
        settings.body_point2 = RVec3.fromVec3(self.local_space_position2);
        settings.fixed_point2 = self.fixed_position2;
        settings.ratio = self.ratio;
        settings.min_length = self.min_length;
        settings.max_length = self.max_length;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const PulleyConstraint) Mat44 {
        return Mat44.translation(self.local_space_position1);
    }

    pub fn getConstraintToBody2Matrix(self: *const PulleyConstraint) Mat44 {
        return Mat44.translation(self.local_space_position2); // Note: Incorrect rotation as we don't track the original rotation difference, should not matter though as the constraint is not limiting rotation.
    }

    /// Update the minimum and maximum length for the constraint
    pub fn setLength(self: *PulleyConstraint, min_length: f32, max_length: f32) void {
        if (Core.enable_asserts) std.debug.assert(min_length >= 0.0 and min_length <= max_length); // User input
        self.min_length = min_length;
        self.max_length = max_length;
    }

    pub fn getMinLength(self: *const PulleyConstraint) f32 {
        return self.min_length;
    }

    pub fn getMaxLength(self: *const PulleyConstraint) f32 {
        return self.max_length;
    }

    /// Get the current length of both segments (multiplied by the ratio for segment 2)
    pub fn getCurrentLength(self: *const PulleyConstraint) f32 {
        return self.world_space_position1.sub(self.fixed_position1).toVec3().length() + self.ratio * self.world_space_position2.sub(self.fixed_position2).toVec3().length();
    }

    ///@name Get Lagrange multiplier from last physics update (the linear impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const PulleyConstraint) f32 {
        return self.independent_axis_constraint_part.getTotalLambda();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/PulleyConstraintTests.zig)

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
    return data.toOwnedSlice();
}

/// ConstraintSettings.restoreFromBinaryState of bytes
fn testRestoreSettings(allocator: Allocator, bytes: []const u8) Allocator.Error!ConstraintSettings.ConstraintResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
}

/// |p1 - f1| + ratio * |p2 - f2| of world space points
fn testLength(p1: RVec3, f1: RVec3, p2: RVec3, f2: RVec3, ratio: f32) f32 {
    return p1.sub(f1).toVec3().length() + ratio * p2.sub(f2).toVec3().length();
}

test "PulleyConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = PulleyConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.body_point1.eql(RVec3.zero()) and settings.fixed_point1.eql(RVec3.zero()));
    try testing.expect(settings.body_point2.eql(RVec3.zero()) and settings.fixed_point2.eql(RVec3.zero()));
    try testing.expectEqual(@as(f32, 1.0), settings.ratio);
    try testing.expectEqual(@as(f32, 0.0), settings.min_length);
    try testing.expectEqual(@as(f32, -1.0), settings.max_length);
    try testing.expectEqualStrings("PulleyConstraintSettings", settings.asSettings().vtable.rtti.name);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 3;
    settings.base.base.num_velocity_steps_override = 6;
    settings.base.base.num_position_steps_override = 1;
    settings.base.base.draw_constraint_size = 2.0;
    settings.base.base.user_data = 42; // Not part of the binary state (Jolt)
    settings.space = .local_to_body_com;
    settings.body_point1 = RVec3.init(1, 2, 3);
    settings.fixed_point1 = RVec3.init(1, 7, 3);
    settings.body_point2 = RVec3.init(-1, 0.5, 0.25);
    settings.fixed_point2 = RVec3.init(-1, 8, 0.25);
    settings.ratio = 2.5;
    settings.min_length = 1.5;
    settings.max_length = 9.0;

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), space (int), the four points (Real),
    // ratio, min and max length
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 4 * 3 * @sizeOf(Real) + 3 * 4), bytes.len);
    try testing.expectEqual(RTTI.hashName("PulleyConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    const after_points = 25 + 4 * 3 * @sizeOf(Real);
    try testing.expectEqual(@as(f32, 2.5), @as(f32, @bitCast(bytes[after_points..][0..4].*)));
    try testing.expectEqual(@as(f32, 1.5), @as(f32, @bitCast(bytes[after_points + 4 ..][0..4].*)));
    try testing.expectEqual(@as(f32, 9.0), @as(f32, @bitCast(bytes[after_points + 8 ..][0..4].*)));
    try testing.expectEqual(@as(Real, 7), @as(Real, @bitCast(bytes[25 + 4 * @sizeOf(Real) ..][0..@sizeOf(Real)].*))); // Fixed point 1 Y

    // Restore through RegisterTypes.factory
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(PulleyConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 3), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.body_point1.eql(settings.body_point1) and restored.fixed_point1.eql(settings.fixed_point1));
        try testing.expect(restored.body_point2.eql(settings.body_point2) and restored.fixed_point2.eql(settings.fixed_point2));
        try testing.expectEqual(@as(f32, 2.5), restored.ratio);
        try testing.expectEqual(@as(f32, 1.5), restored.min_length);
        try testing.expectEqual(@as(f32, 9.0), restored.max_length);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated: Jolt's error (every length)
    for (0..bytes.len) |len| {
        var result = try testRestoreSettings(allocator, bytes[0..len]);
        defer result.deinit();
        try testing.expect(result.hasError());
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
    var copy = PulleyConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.fixed_point2.eql(settings.fixed_point2) and copy.ratio == settings.ratio and copy.max_length == settings.max_length);
    try testing.expectEqual(@as(u64, 42), copy.base.base.user_data);
}

test "PulleyConstraint: virtual dispatch, space conversions, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    // Static or dynamic bodies (IndependentAxisConstraintPart asserts that a body that is not static is dynamic)
    const pairs = [_]TestContext.MotionTypePair{ .{ .static, .static }, .{ .static, .dynamic }, .{ .dynamic, .static }, .{ .dynamic, .dynamic } };
    for (pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];

        var settings = PulleyConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.body_point1 = RVec3.init(1.25, 2.5, 3.0);
        settings.fixed_point1 = RVec3.init(1.0, 6.0, 3.0);
        settings.body_point2 = RVec3.init(2.0, 1.75, 4.5);
        settings.fixed_point2 = RVec3.init(2.5, 7.0, 4.0);
        settings.ratio = 1.5;
        settings.min_length = -1.0;
        settings.base.base.user_data = 5678;

        // TwoBodyConstraintSettings::Create through the vtable
        var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit();
        const base = constraint.get().?;
        const two_body = virtual.downcast(TwoBodyConstraint, base);
        const c = virtual.downcast(PulleyConstraint, two_body);
        try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
        try testing.expectEqual(ConstraintSubType.pulley, base.getSubType());
        try testing.expectEqual(@as(u64, 5678), base.getUserData());
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, base.isActive());

        // World space points in local space, the world space points kept, the fixed points, min and max length -1: the
        // current length, the normals -Y until the first update, the matrices are translations
        try testing.expect(c.local_space_position1.eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.body_point1).toVec3()));
        try testing.expect(c.local_space_position2.eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.body_point2).toVec3()));
        try testing.expect(c.world_space_position1.eql(settings.body_point1) and c.world_space_position2.eql(settings.body_point2));
        try testing.expect(c.fixed_position1.eql(settings.fixed_point1) and c.fixed_position2.eql(settings.fixed_point2));
        const length = testLength(settings.body_point1, settings.fixed_point1, settings.body_point2, settings.fixed_point2, 1.5);
        try testing.expectEqual(length, c.getCurrentLength());
        try testing.expectEqual(length, c.getMinLength());
        try testing.expectEqual(length, c.getMaxLength());
        try testing.expect(c.world_space_normal1.eql(Vec3.axisY().negate()) and c.world_space_normal2.eql(Vec3.axisY().negate()));
        try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.translation(c.local_space_position1)));
        try testing.expect(two_body.getConstraintToBody2Matrix().eql(Mat44.translation(c.local_space_position2)));

        // GetConstraintSettings: local space body points, world space fixed points, the ratio and the lengths; a
        // constraint made from them is the same
        {
            var copy = try base.getConstraintSettings();
            defer copy.deinit();
            const s = virtual.downcast(PulleyConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
            try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
            try testing.expectEqual(@as(u64, 5678), s.base.base.user_data);
            try testing.expect(s.body_point1.eql(RVec3.fromVec3(c.local_space_position1)) and s.body_point2.eql(RVec3.fromVec3(c.local_space_position2)));
            try testing.expect(s.fixed_point1.eql(settings.fixed_point1) and s.fixed_point2.eql(settings.fixed_point2));
            try testing.expectEqual(@as(f32, 1.5), s.ratio);
            try testing.expectEqual(length, s.min_length);
            try testing.expectEqual(length, s.max_length);
            var again = PulleyConstraint.init(allocator, body1, body2, s);
            defer again.asConstraint().deinit();
            try testing.expect(again.local_space_position1.eql(c.local_space_position1) and again.local_space_position2.eql(c.local_space_position2));
            try testing.expect(again.fixed_position1.eql(c.fixed_position1) and again.fixed_position2.eql(c.fixed_position2));
            try testing.expectEqual(c.getMinLength(), again.getMinLength());
            try testing.expectEqual(c.getMaxLength(), again.getMaxLength());
        }

        // Solver: min = max = the length, both directions; the velocity along the rope is removed
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);
        base.setupVelocityConstraint(1.0 / 60.0);
        const delta1 = c.world_space_position1.sub(c.fixed_position1).toVec3();
        const delta2 = c.world_space_position2.sub(c.fixed_position2).toVec3();
        try testing.expect(c.world_space_normal1.isClose(delta1.normalized(), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(c.world_space_normal2.isClose(delta2.normalized(), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expectEqual(-math.flt_max, c.min_lambda);
        try testing.expectEqual(math.flt_max, c.max_lambda);
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, c.independent_axis_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        const applied = base.solveVelocityConstraint(1.0 / 60.0);
        try testing.expectEqual(c.independent_axis_constraint_part.isActive(), applied);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);
        if (applied) {
            for (0..5) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
            const r1 = c.world_space_position1.sub(body1.getCenterOfMassPosition()).toVec3();
            const r2 = c.world_space_position2.sub(body2.getCenterOfMassPosition()).toVec3();
            const v1 = body1.getLinearVelocity().add(body1.getAngularVelocity().cross(r1));
            const v2 = body2.getLinearVelocity().add(body2.getAngularVelocity().cross(r2));
            try testing.expect(@abs(v1.dot(c.world_space_normal1) + 1.5 * v2.dot(c.world_space_normal2)) < 1.0e-3); // d/dt (length) = 0
            try testing.expect(c.getTotalLambdaPosition() != 0.0);
        }

        // SaveState / RestoreState: enabled + the lambda of the part + the two normals
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        base.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 1 + 4 + 12 + 12), recorder.getData().len);
        const lambda = c.getTotalLambdaPosition();
        const normal1 = c.world_space_normal1;
        const normal2 = c.world_space_normal2;
        base.resetWarmStart();
        try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPosition());
        try testing.expect(!c.independent_axis_constraint_part.isActive());
        c.world_space_normal1 = Vec3.axisZ();
        c.world_space_normal2 = Vec3.axisX();
        base.restoreState(recorder.recorder());
        try testing.expectEqual(lambda, c.getTotalLambdaPosition());
        try testing.expect(c.world_space_normal1.eql(normal1) and c.world_space_normal2.eql(normal2));

        // NotifyShapeChanged moves the attachment point of the body that changed
        const p1 = c.local_space_position1;
        const p2 = c.local_space_position2;
        base.notifyShapeChanged(body1.getID(), Vec3.init(0.25, 0, 0));
        try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0.25, 0, 0))) and c.local_space_position2.eql(p2));
        base.notifyShapeChanged(body2.getID(), Vec3.init(0, -0.5, 0));
        try testing.expect(c.local_space_position2.eql(p2.sub(Vec3.init(0, -0.5, 0))));
        base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
        try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0.25, 0, 0))) and c.local_space_position2.eql(p2.sub(Vec3.init(0, -0.5, 0))));

        // A disabled constraint is not active (TwoBodyConstraint::IsActive)
        base.setEnabled(false);
        try testing.expect(!base.isActive());
    }
}

test "PulleyConstraint: lengths of the settings, setters, local space" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);

    // World space: segment 1 has length 3, segment 2 length 4, ratio 0.5: current length 5
    var settings = PulleyConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.body_point1 = RVec3.init(1, 2, 3);
    settings.fixed_point1 = RVec3.init(1, 5, 3);
    settings.body_point2 = RVec3.init(2, 1.5, 4);
    settings.fixed_point2 = RVec3.init(2, 5.5, 4);
    settings.ratio = 0.5;
    const Case = struct { min: f32, max: f32, expected_min: f32, expected_max: f32 };
    const cases = [_]Case{
        .{ .min = 0, .max = -1, .expected_min = 0, .expected_max = 5 }, // Jolt's defaults: a rope that can go slack
        .{ .min = -1, .max = -1, .expected_min = 5, .expected_max = 5 },
        .{ .min = -1, .max = 7, .expected_min = 5, .expected_max = 7 },
        .{ .min = 2, .max = 3, .expected_min = 2, .expected_max = 3 }, // As given, even when the length is outside
        .{ .min = 6, .max = 4, .expected_min = 6, .expected_max = 4 }, // The constructor does not check (no SetLength)
        .{ .min = -0.5, .max = 8, .expected_min = 5, .expected_max = 8 }, // Any negative value is the current length
    };
    for (cases) |case| {
        settings.min_length = case.min;
        settings.max_length = case.max;
        var c = PulleyConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        try testing.expectEqual(@as(f32, 5.0), c.getCurrentLength());
        try testing.expectEqual(case.expected_min, c.getMinLength());
        try testing.expectEqual(case.expected_max, c.getMaxLength());
    }

    // SetLength
    var c = PulleyConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();
    c.setLength(0.0, 2.25);
    try testing.expectEqual(@as(f32, 0.0), c.getMinLength());
    try testing.expectEqual(@as(f32, 2.25), c.getMaxLength());
    c.setLength(4.5, 4.5);
    try testing.expectEqual(@as(f32, 4.5), c.getMinLength());
    try testing.expectEqual(@as(f32, 4.5), c.getMaxLength());

    // GetCurrentLength uses the world space points of the last update
    c.asConstraint().setupVelocityConstraint(1.0 / 60.0);
    try testing.expectEqual(testLength(c.world_space_position1, c.fixed_position1, c.world_space_position2, c.fixed_position2, 0.5), c.getCurrentLength());

    // Local space: the body points are relative to the centers of mass, the fixed points stay in world space
    settings.space = .local_to_body_com;
    settings.body_point1 = RVec3.init(0, 0.5, 0);
    settings.body_point2 = RVec3.init(0.25, 0, -0.25);
    settings.min_length = -1;
    settings.max_length = -1;
    var local = PulleyConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer local.asConstraint().deinit();
    try testing.expect(local.local_space_position1.eql(Vec3.init(0, 0.5, 0)) and local.local_space_position2.eql(Vec3.init(0.25, 0, -0.25)));
    try testing.expect(local.world_space_position1.eql(bodies[0].getCenterOfMassTransform().mulRVec3(settings.body_point1)));
    try testing.expect(local.world_space_position2.eql(bodies[1].getCenterOfMassTransform().mulRVec3(settings.body_point2)));
    try testing.expect(local.fixed_position1.eql(settings.fixed_point1) and local.fixed_position2.eql(settings.fixed_point2));
    const length = testLength(local.world_space_position1, settings.fixed_point1, local.world_space_position2, settings.fixed_point2, 0.5);
    try testing.expectEqual(length, local.getMinLength());
    try testing.expectEqual(length, local.getMaxLength());
}

test "PulleyConstraint: solver branches" {
    // Two bodies hang below their fixed points (segment 1 length 2, segment 2 length 3, ratio 2: length 8); body 1
    // moves down, body 2 up with `speed`, which changes the length by -speed * (1 - 2) per second... d/dt (length) =
    // n1 . v1 + 2 n2 . v2 with n = -Y: -v1y - 2 v2y
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const dt: f32 = 1.0 / 60.0;
    const flt_max = math.flt_max;

    const Case = struct {
        min: f32,
        max: f32,
        /// Y velocity of body 1 and body 2
        v1: f32,
        v2: f32,
        active: bool,
        min_lambda: f32 = 0,
        max_lambda: f32 = 0,
        applied: bool = false,
        position: bool = false,
    };
    const cases = [_]Case{
        // Exactly the length: both directions, no position error
        .{ .min = 8, .max = 8, .v1 = -1, .v2 = 0, .active = true, .min_lambda = -flt_max, .max_lambda = flt_max, .applied = true },
        // Longer than the max length (8 > 7): only shortens (a rope)
        .{ .min = 0, .max = 7, .v1 = -1, .v2 = 0, .active = true, .min_lambda = -flt_max, .max_lambda = 0, .applied = true, .position = true },
        .{ .min = 0, .max = 7, .v1 = 1, .v2 = 0, .active = true, .min_lambda = -flt_max, .max_lambda = 0, .applied = false, .position = true },
        // Shorter than the min length (8 < 9): only lengthens
        .{ .min = 9, .max = 10, .v1 = 0, .v2 = 1, .active = true, .min_lambda = 0, .max_lambda = flt_max, .applied = true, .position = true },
        .{ .min = 9, .max = 10, .v1 = 0, .v2 = -1, .active = true, .min_lambda = 0, .max_lambda = flt_max, .applied = false, .position = true },
        // Between: not active
        .{ .min = 6, .max = 9, .v1 = -1, .v2 = -1, .active = false },
    };
    for (cases) |case| {
        const body1 = try context.createBody(.dynamic, .{ .position = RVec3.init(-1, 3, 0), .linear_velocity = Vec3.init(0, case.v1, 0) });
        const body2 = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 0), .linear_velocity = Vec3.init(0, case.v2, 0) });
        defer context.destroyBodies(&.{ body1, body2 });
        var settings = PulleyConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.body_point1 = RVec3.init(-1, 3, 0);
        settings.fixed_point1 = RVec3.init(-1, 5, 0);
        settings.body_point2 = RVec3.init(1, 2, 0);
        settings.fixed_point2 = RVec3.init(1, 5, 0);
        settings.ratio = 2.0;
        settings.min_length = case.min;
        settings.max_length = case.max;
        var c = PulleyConstraint.init(allocator, body1, body2, &settings);
        defer c.asConstraint().deinit();
        const base = c.asConstraint();
        try testing.expectEqual(@as(f32, 8.0), c.getCurrentLength());

        base.setupVelocityConstraint(dt);
        try testing.expect(c.world_space_normal1.eql(Vec3.axisY().negate()) and c.world_space_normal2.eql(Vec3.axisY().negate()));
        try testing.expectEqual(case.active, c.independent_axis_constraint_part.isActive());
        if (case.active) {
            try testing.expectEqual(case.min_lambda, c.min_lambda);
            try testing.expectEqual(case.max_lambda, c.max_lambda);
        }

        base.warmStartVelocityConstraint(1.0);
        const applied = base.solveVelocityConstraint(dt);
        try testing.expectEqual(case.applied, applied);
        const rate = -body1.getLinearVelocity().getY() - 2.0 * body2.getLinearVelocity().getY();
        if (case.applied) {
            try testing.expect(@abs(rate) < 1.0e-5); // The length no longer changes
        } else if (!case.active) {
            try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPosition());
            try testing.expectEqual(case.v1, body1.getLinearVelocity().getY());
            try testing.expectEqual(case.v2, body2.getLinearVelocity().getY());
        }

        // Position: the length moves towards the violated limit
        const corrected = base.solvePositionConstraint(dt, 0.2);
        try testing.expectEqual(case.position, corrected);
        const new_length = testLength(body1.getCenterOfMassPosition(), settings.fixed_point1, body2.getCenterOfMassPosition(), settings.fixed_point2, 2.0);
        if (corrected) {
            const limit: f32 = if (case.min > 8) case.min else case.max;
            try testing.expect(@abs(new_length - limit) < @abs(8.0 - limit));
        } else {
            try testing.expectEqual(@as(f32, 8.0), new_length);
        }
    }
}

test "PulleyConstraint: body points on the fixed points keep the last normals, static bodies" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(2, 1, 0), .linear_velocity = Vec3.init(0, -1, 0) });
    defer context.destroyBodies(&.{body});

    // Body 1 is the world with its point on fixed point 1 (segment 1 has length 0): its normal stays -Y
    var settings = PulleyConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.body_point1 = RVec3.init(0, 4, 0);
    settings.fixed_point1 = RVec3.init(0, 4, 0);
    settings.body_point2 = RVec3.init(2, 1, 0);
    settings.fixed_point2 = RVec3.init(2, 4, 0);
    var c = PulleyConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expectEqual(@as(f32, 3.0), c.getMaxLength()); // Max -1: the current length
    try testing.expectEqual(@as(f32, 0.0), c.getMinLength());
    const base = c.asConstraint();
    try testing.expect(base.isActive());
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.world_space_normal1.eql(Vec3.axisY().negate()));
    try testing.expect(c.world_space_normal2.isClose(Vec3.axisY().negate(), .{ .max_dist_sq = 1.0e-12 }));
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0)); // Falling: the rope holds the body
    try testing.expect(@abs(body.getLinearVelocity().getY()) < 1.0e-6);
    try testing.expect(c.getTotalLambdaPosition() < 0.0);

    // A normal of an earlier update is kept while a segment has length 0 (and is part of the state)
    c.world_space_normal1 = Vec3.axisX();
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.world_space_normal1.eql(Vec3.axisX()));
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    c.world_space_normal1 = Vec3.axisZ();
    base.restoreState(recorder.recorder());
    try testing.expect(c.world_space_normal1.eql(Vec3.axisX()));

    // Two static bodies: never active, nothing moves
    const statics = try context.createBodyPair(.{ .static, .static });
    defer context.destroyBodies(&statics);
    var s = PulleyConstraint.init(allocator, statics[0], statics[1], &settings);
    defer s.asConstraint().deinit();
    try testing.expect(!s.asConstraint().isActive());
    s.asConstraint().setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(!s.independent_axis_constraint_part.isActive());
    try testing.expect(!s.asConstraint().solveVelocityConstraint(1.0 / 60.0));
}

test "PulleyConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try PulleyConstraintSettings.create(allocator)).asSettings());
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
