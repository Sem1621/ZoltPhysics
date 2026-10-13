//! Port of: Jolt/Physics/Constraints/DistanceConstraint.h, Jolt/Physics/Constraints/DistanceConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(DistanceConstraintSettings): TODO(serialization) (the binary state is ported).
//! JPH_DEBUG_RENDERER (DrawConstraint): TODO(debug_renderer).
//!
//! - The const and non-const overloads of `GetLimitsSpringSettings` are `getLimitsSpringSettings` (`*const`) and
//!   `getLimitsSpringSettingsMut` (`*`, guide section 4), like HingeConstraint / SliderConstraint.
//! - `SetDistance` asserts `inMinDistance <= inMaxDistance`, which user input can violate (also through the
//!   constructor: a min distance above the max distance in the settings): the assert only runs with
//!   `Core.enable_asserts`, ReleaseFast keeps Jolt's release behavior.
//! - `mMinLambda` / `mMaxLambda` are uninitialized in Jolt until CalculateConstraintProperties sets them (`undefined`):
//!   SolveVelocityConstraint only reads them while the axis constraint is active, which only the branches that set
//!   them make it.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
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
const SpringSettings = @import("SpringSettings.zig").SpringSettings;
const AxisConstraintPart = @import("ConstraintPart/AxisConstraintPart.zig").AxisConstraintPart;

/// Distance constraint settings, used to create a distance constraint
pub const DistanceConstraintSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, DistanceConstraintSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "DistanceConstraintSettings";
    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint reference frame (space determined by mSpace).
    /// Constraint will keep mPoint1 (a point on body 1) and mPoint2 (a point on body 2) at the same distance.
    /// Note that this constraint can be used as a cheap PointConstraint by setting mPoint1 = mPoint2 (but this removes only 1 degree of freedom instead of 3).
    point1: RVec3 = RVec3.zero(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    point2: RVec3 = RVec3.zero(),

    /// Ability to override the distance range at which the two points are kept apart. If the value is negative, it will be replaced by the distance between mPoint1 and mPoint2 (works only if mSpace is world space).
    min_distance: f32 = -1.0,
    max_distance: f32 = -1.0,

    /// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
    limits_spring_settings: SpringSettings = .{},

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) DistanceConstraintSettings {
        return .{ .base = .init(DistanceConstraintSettings, allocator) };
    }

    /// new DistanceConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*DistanceConstraintSettings {
        const self = try allocator.create(DistanceConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const DistanceConstraintSettings) DistanceConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *DistanceConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const DistanceConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *DistanceConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const DistanceConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const DistanceConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.point1);
        stream.write(self.point2);
        stream.write(self.min_distance);
        stream.write(self.max_distance);
        self.limits_spring_settings.saveBinaryState(stream);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const DistanceConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try DistanceConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *DistanceConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.point1);
        stream.read(&self.point2);
        stream.read(&self.min_distance);
        stream.read(&self.max_distance);
        self.limits_spring_settings.restoreBinaryState(stream);
    }
};

/// This constraint is a stiff spring that holds 2 points at a fixed distance from each other
pub const DistanceConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Min/max distance that must be kept between the world space points
    min_distance: f32,
    max_distance: f32,

    // Soft constraint limits
    limits_spring_settings: SpringSettings = .{},

    // RUN TIME PROPERTIES FOLLOW

    // World space positions and normal
    world_space_position1: RVec3 = undefined,
    world_space_position2: RVec3 = undefined,
    world_space_normal: Vec3 = undefined,

    // Depending on if the distance < min or distance > max we can apply forces to prevent further violations
    min_lambda: f32 = undefined,
    max_lambda: f32 = undefined,

    // The constraint part
    axis_constraint: AxisConstraintPart = .{},

    /// Construct distance constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const DistanceConstraintSettings) DistanceConstraint {
        var self: DistanceConstraint = .{
            .base = .init(DistanceConstraint, allocator, body1, body2, &settings.base),
            .min_distance = settings.min_distance,
            .max_distance = settings.max_distance,
        };

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            self.local_space_position1 = body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3();
            self.local_space_position2 = body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3();
            self.world_space_position1 = settings.point1;
            self.world_space_position2 = settings.point2;
        } else {
            // If properties were specified in local space, we need to calculate world space positions
            self.local_space_position1 = settings.point1.toVec3();
            self.local_space_position2 = settings.point2.toVec3();
            self.world_space_position1 = body1.getCenterOfMassTransform().mulRVec3(settings.point1);
            self.world_space_position2 = body2.getCenterOfMassTransform().mulRVec3(settings.point2);
        }

        // Store distance we want to keep between the world space points
        const distance = self.world_space_position2.sub(self.world_space_position1).toVec3().length();
        var min_distance: f32 = undefined;
        var max_distance: f32 = undefined;
        if (self.min_distance < 0.0 and self.max_distance < 0.0) {
            min_distance = distance;
            max_distance = distance;
        } else {
            min_distance = if (self.min_distance < 0.0) math.min(distance, self.max_distance) else self.min_distance;
            max_distance = if (self.max_distance < 0.0) math.max(distance, self.min_distance) else self.max_distance;
        }
        self.setDistance(min_distance, max_distance);

        // Most likely gravity is going to tear us apart (this is only used when the distance between the points = 0)
        self.world_space_normal = Vec3.axisY();

        // Store spring settings
        self.setLimitsSpringSettings(settings.limits_spring_settings);

        return self;
    }

    /// new DistanceConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const DistanceConstraintSettings) Allocator.Error!*DistanceConstraint {
        const self = try allocator.create(DistanceConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *DistanceConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const DistanceConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *DistanceConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const DistanceConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const DistanceConstraint) ConstraintSubType {
        _ = self;
        return .distance;
    }

    pub fn notifyShapeChanged(self: *DistanceConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    // Internal helper function to calculate the values below
    fn calculateConstraintProperties(self: *DistanceConstraint, delta_time: f32) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Update world space positions (the bodies may have moved)
        self.world_space_position1 = body1.getCenterOfMassTransform().mulVec3(self.local_space_position1);
        self.world_space_position2 = body2.getCenterOfMassTransform().mulVec3(self.local_space_position2);

        // Calculate world space normal
        const delta = self.world_space_position2.sub(self.world_space_position1).toVec3();
        const delta_len = delta.length();
        if (delta_len > 0.0)
            self.world_space_normal = delta.divScalar(delta_len);

        // Calculate points relative to body
        // r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
        const r1_plus_u = self.world_space_position2.sub(body1.getCenterOfMassPosition()).toVec3();
        const r2 = self.world_space_position2.sub(body2.getCenterOfMassPosition()).toVec3();

        if (self.min_distance == self.max_distance) {
            self.axis_constraint.calculateConstraintPropertiesWithSettingsForLimit(delta_time, body1, r1_plus_u, body2, r2, self.world_space_normal, 0.0, delta_len - self.min_distance, &self.limits_spring_settings);

            // Single distance, allow constraint forces in both directions
            self.min_lambda = -math.flt_max;
            self.max_lambda = math.flt_max;
        } else if (delta_len <= self.min_distance) {
            self.axis_constraint.calculateConstraintPropertiesWithSettingsForLimit(delta_time, body1, r1_plus_u, body2, r2, self.world_space_normal, 0.0, delta_len - self.min_distance, &self.limits_spring_settings);

            // Allow constraint forces to make distance bigger only
            self.min_lambda = 0;
            self.max_lambda = math.flt_max;
        } else if (delta_len >= self.max_distance) {
            self.axis_constraint.calculateConstraintPropertiesWithSettingsForLimit(delta_time, body1, r1_plus_u, body2, r2, self.world_space_normal, 0.0, delta_len - self.max_distance, &self.limits_spring_settings);

            // Allow constraint forces to make distance smaller only
            self.min_lambda = -math.flt_max;
            self.max_lambda = 0;
        } else self.axis_constraint.deactivate();
    }

    pub fn setupVelocityConstraint(self: *DistanceConstraint, delta_time: f32) void {
        self.calculateConstraintProperties(delta_time);
    }

    pub fn resetWarmStart(self: *DistanceConstraint) void {
        self.axis_constraint.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *DistanceConstraint, warm_start_impulse_ratio: f32) void {
        self.axis_constraint.warmStart(self.base.body1, self.base.body2, self.world_space_normal, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *DistanceConstraint, delta_time: f32) bool {
        _ = delta_time;
        if (self.axis_constraint.isActive())
            return self.axis_constraint.solveVelocityConstraint(self.base.body1, self.base.body2, self.world_space_normal, self.min_lambda, self.max_lambda)
        else
            return false;
    }

    pub fn solvePositionConstraint(self: *DistanceConstraint, delta_time: f32, baumgarte: f32) bool {
        if (self.limits_spring_settings.frequency <= 0.0) { // When the spring is active, we don't need to solve the position constraint
            const distance = self.world_space_position2.sub(self.world_space_position1).toVec3().dot(self.world_space_normal);

            // Calculate position error
            var position_error: f32 = 0.0;
            if (distance < self.min_distance)
                position_error = distance - self.min_distance
            else if (distance > self.max_distance)
                position_error = distance - self.max_distance;

            if (position_error != 0.0) {
                // Update constraint properties (bodies may have moved)
                self.calculateConstraintProperties(delta_time);

                return self.axis_constraint.solvePositionConstraint(self.base.body1, self.base.body2, self.world_space_normal, position_error, baumgarte);
            }
        }

        return false;
    }

    // TODO(debug_renderer): DrawConstraint

    pub fn saveState(self: *const DistanceConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.axis_constraint.saveState(stream);
        stream.write(self.world_space_normal); // When distance = 0, the normal is used from last frame so we need to store it
    }

    pub fn restoreState(self: *DistanceConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.axis_constraint.restoreState(stream);
        stream.read(&self.world_space_normal);
    }

    pub fn getConstraintSettings(self: *const DistanceConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try DistanceConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.fromVec3(self.local_space_position1);
        settings.point2 = RVec3.fromVec3(self.local_space_position2);
        settings.min_distance = self.min_distance;
        settings.max_distance = self.max_distance;
        settings.limits_spring_settings = self.limits_spring_settings;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const DistanceConstraint) Mat44 {
        return Mat44.translation(self.local_space_position1);
    }

    pub fn getConstraintToBody2Matrix(self: *const DistanceConstraint) Mat44 {
        return Mat44.translation(self.local_space_position2); // Note: Incorrect rotation as we don't track the original rotation difference, should not matter though as the constraint is not limiting rotation.
    }

    /// Update the minimum and maximum distance for the constraint
    pub fn setDistance(self: *DistanceConstraint, min_distance: f32, max_distance: f32) void {
        if (Core.enable_asserts) std.debug.assert(min_distance <= max_distance); // User input (also the settings through the constructor)
        self.min_distance = min_distance;
        self.max_distance = max_distance;
    }

    pub fn getMinDistance(self: *const DistanceConstraint) f32 {
        return self.min_distance;
    }

    pub fn getMaxDistance(self: *const DistanceConstraint) f32 {
        return self.max_distance;
    }

    /// Update the limits spring settings
    pub fn getLimitsSpringSettings(self: *const DistanceConstraint) *const SpringSettings {
        return &self.limits_spring_settings;
    }

    /// The non-const overload of GetLimitsSpringSettings (a reference that can be written)
    pub fn getLimitsSpringSettingsMut(self: *DistanceConstraint) *SpringSettings {
        return &self.limits_spring_settings;
    }

    pub fn setLimitsSpringSettings(self: *DistanceConstraint, limits_spring_settings: SpringSettings) void {
        self.limits_spring_settings = limits_spring_settings;
    }

    ///@name Get Lagrange multiplier from last physics update (the linear impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const DistanceConstraint) f32 {
        return self.axis_constraint.getTotalLambda();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/DistanceConstraintTests.zig)

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

/// The distance between the world space points of the constraint, computed like the constructor does
fn testWorldDistance(c: *const DistanceConstraint) f32 {
    return c.world_space_position2.sub(c.world_space_position1).toVec3().length();
}

test "DistanceConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = DistanceConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.point1.eql(RVec3.zero()) and settings.point2.eql(RVec3.zero()));
    try testing.expectEqual(@as(f32, -1.0), settings.min_distance);
    try testing.expectEqual(@as(f32, -1.0), settings.max_distance);
    try testing.expectEqual(SpringSettings{}, settings.limits_spring_settings);
    try testing.expectEqualStrings("DistanceConstraintSettings", settings.asSettings().vtable.rtti_name);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 7;
    settings.base.base.num_velocity_steps_override = 2;
    settings.base.base.num_position_steps_override = 5;
    settings.base.base.draw_constraint_size = 0.25;
    settings.base.base.user_data = 99; // Not part of the binary state (Jolt)
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(1, -2, 3);
    settings.point2 = RVec3.init(-0.5, 0.25, 4);
    settings.min_distance = 0.5;
    settings.max_distance = 2.5;
    settings.limits_spring_settings = .init(.stiffness_and_damping, 120.0, 3.5);

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), space (int), the points (Real), min and
    // max distance, the spring (mode byte, frequency / stiffness, damping)
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 2 * 3 * @sizeOf(Real) + 2 * 4 + 1 + 2 * 4), bytes.len);
    try testing.expectEqual(ConstraintSettings.rttiHash("DistanceConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    const after_points = 25 + 2 * 3 * @sizeOf(Real);
    try testing.expectEqual(@as(f32, 0.5), @as(f32, @bitCast(bytes[after_points..][0..4].*)));
    try testing.expectEqual(@as(f32, 2.5), @as(f32, @bitCast(bytes[after_points + 4 ..][0..4].*)));
    try testing.expectEqual(@as(u8, 1), bytes[after_points + 8]); // StiffnessAndDamping

    // Restore through RegisterTypes.constraint_settings_types
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(DistanceConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 7), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.point1.eql(settings.point1) and restored.point2.eql(settings.point2));
        try testing.expectEqual(@as(f32, 0.5), restored.min_distance);
        try testing.expectEqual(@as(f32, 2.5), restored.max_distance);
        try testing.expectEqual(settings.limits_spring_settings, restored.limits_spring_settings);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // A spring mode byte outside the named ones is restored as it is (ESpringMode is a uint8 enum)
    {
        var odd = try allocator.dupe(u8, bytes);
        defer allocator.free(odd);
        odd[after_points + 8] = 200;
        var result = try testRestoreSettings(allocator, odd);
        defer result.deinit();
        const restored = virtual.downcast(DistanceConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expectEqual(@as(u8, 200), @intFromEnum(restored.limits_spring_settings.mode));
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, odd, resaved);
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
    var copy = DistanceConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.point2.eql(settings.point2) and copy.max_distance == settings.max_distance);
    try testing.expectEqual(settings.limits_spring_settings, copy.limits_spring_settings);
    try testing.expectEqual(@as(u64, 99), copy.base.base.user_data);
}

test "DistanceConstraint: virtual dispatch, space conversions, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];

        var settings = DistanceConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.point1 = RVec3.init(1.25, 2.5, 3.0);
        settings.point2 = RVec3.init(2.0, 1.0, 4.5);
        settings.limits_spring_settings = .init(.frequency_and_damping, 0.0, 0.5);
        settings.base.base.user_data = 1234;

        // TwoBodyConstraintSettings::Create through the vtable
        var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit();
        const base = constraint.get().?;
        const two_body = virtual.downcast(TwoBodyConstraint, base);
        const c = virtual.downcast(DistanceConstraint, two_body);
        try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
        try testing.expectEqual(ConstraintSubType.distance, base.getSubType());
        try testing.expectEqual(@as(u64, 1234), base.getUserData());
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, base.isActive());

        // World space points in local space, the world space points kept, the distance of the points as min and max,
        // the normal Y until the first update, the matrices are translations
        try testing.expect(c.local_space_position1.eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3()));
        try testing.expect(c.local_space_position2.eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3()));
        try testing.expect(c.world_space_position1.eql(settings.point1) and c.world_space_position2.eql(settings.point2));
        const distance = settings.point2.sub(settings.point1).toVec3().length();
        try testing.expectEqual(distance, c.getMinDistance());
        try testing.expectEqual(distance, c.getMaxDistance());
        try testing.expect(c.world_space_normal.eql(Vec3.axisY()));
        try testing.expectEqual(settings.limits_spring_settings, c.getLimitsSpringSettings().*);
        try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.translation(c.local_space_position1)));
        try testing.expect(two_body.getConstraintToBody2Matrix().eql(Mat44.translation(c.local_space_position2)));

        // GetConstraintSettings: local space points, the distances and the spring; a constraint made from them is the same
        {
            var copy = try base.getConstraintSettings();
            defer copy.deinit();
            const s = virtual.downcast(DistanceConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
            try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
            try testing.expectEqual(@as(u64, 1234), s.base.base.user_data);
            try testing.expect(s.point1.eql(RVec3.fromVec3(c.local_space_position1)) and s.point2.eql(RVec3.fromVec3(c.local_space_position2)));
            try testing.expectEqual(distance, s.min_distance);
            try testing.expectEqual(distance, s.max_distance);
            try testing.expectEqual(settings.limits_spring_settings, s.limits_spring_settings);
            var again = DistanceConstraint.init(allocator, body1, body2, s);
            defer again.asConstraint().deinit();
            try testing.expect(again.local_space_position1.eql(c.local_space_position1) and again.local_space_position2.eql(c.local_space_position2));
            try testing.expectEqual(c.getMinDistance(), again.getMinDistance());
            try testing.expectEqual(c.getMaxDistance(), again.getMaxDistance());
        }

        // Solver: the relative velocity of the points along the normal is removed (single distance: both directions)
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);
        base.setupVelocityConstraint(1.0 / 60.0);
        const delta = c.world_space_position2.sub(c.world_space_position1).toVec3();
        try testing.expect(c.world_space_normal.isClose(delta.normalized(), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expectEqual(-math.flt_max, c.min_lambda);
        try testing.expectEqual(math.flt_max, c.max_lambda);
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, c.axis_constraint.isActive());
        base.warmStartVelocityConstraint(1.0);
        const applied = base.solveVelocityConstraint(1.0 / 60.0);
        try testing.expectEqual(c.axis_constraint.isActive(), applied);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);
        if (applied) {
            // The velocity of body 1 is taken at point 2 (r1 + u, AxisConstraintPart's Jacobian)
            for (0..5) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
            const r1_plus_u = c.world_space_position2.sub(body1.getCenterOfMassPosition()).toVec3();
            const r2 = c.world_space_position2.sub(body2.getCenterOfMassPosition()).toVec3();
            const v1 = body1.getLinearVelocity().add(body1.getAngularVelocity().cross(r1_plus_u));
            const v2 = body2.getLinearVelocity().add(body2.getAngularVelocity().cross(r2));
            try testing.expect(@abs(v2.sub(v1).dot(c.world_space_normal)) < 1.0e-3);
            try testing.expect(c.getTotalLambdaPosition() != 0.0);
        }

        // SaveState / RestoreState: enabled + the lambda of the part + the normal
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        base.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 1 + 4 + 12), recorder.getData().len);
        const lambda = c.getTotalLambdaPosition();
        const normal = c.world_space_normal;
        base.resetWarmStart();
        try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPosition());
        try testing.expect(!c.axis_constraint.isActive());
        c.world_space_normal = Vec3.axisZ();
        base.restoreState(recorder.recorder());
        try testing.expectEqual(lambda, c.getTotalLambdaPosition());
        try testing.expect(c.world_space_normal.eql(normal));

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

test "DistanceConstraint: the distance range of the settings, setters" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);

    var settings = DistanceConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = RVec3.init(1, 2, 3);
    settings.point2 = RVec3.init(1, 4, 3); // Distance 2
    const Case = struct { min: f32, max: f32, expected_min: f32, expected_max: f32 };
    const cases = [_]Case{
        .{ .min = -1, .max = -1, .expected_min = 2, .expected_max = 2 }, // Both negative: the distance
        .{ .min = -1, .max = 3, .expected_min = 2, .expected_max = 3 }, // min(distance, max)
        .{ .min = -1, .max = 1.5, .expected_min = 1.5, .expected_max = 1.5 },
        .{ .min = 0.5, .max = -1, .expected_min = 0.5, .expected_max = 2 }, // max(distance, min)
        .{ .min = 2.5, .max = -1, .expected_min = 2.5, .expected_max = 2.5 },
        .{ .min = 0.5, .max = 4, .expected_min = 0.5, .expected_max = 4 }, // As given
        .{ .min = 0, .max = 0, .expected_min = 0, .expected_max = 0 },
    };
    for (cases) |case| {
        settings.min_distance = case.min;
        settings.max_distance = case.max;
        var c = DistanceConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        try testing.expectEqual(case.expected_min, c.getMinDistance());
        try testing.expectEqual(case.expected_max, c.getMaxDistance());
    }

    // SetDistance, the limits spring setters (the non-const reference writes the constraint's settings)
    var c = DistanceConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();
    c.setDistance(1.0, 1.75);
    try testing.expectEqual(@as(f32, 1.0), c.getMinDistance());
    try testing.expectEqual(@as(f32, 1.75), c.getMaxDistance());
    c.setDistance(3.0, 3.0);
    try testing.expectEqual(@as(f32, 3.0), c.getMinDistance());
    try testing.expectEqual(@as(f32, 3.0), c.getMaxDistance());
    c.setLimitsSpringSettings(.init(.mass_normalized_stiffness_and_damping, 25.0, 1.5));
    try testing.expectEqual(SpringSettings.init(.mass_normalized_stiffness_and_damping, 25.0, 1.5), c.getLimitsSpringSettings().*);
    c.getLimitsSpringSettingsMut().damping = 0.75;
    try testing.expectEqual(@as(f32, 0.75), c.getLimitsSpringSettings().damping);
    c.getLimitsSpringSettingsMut().setStiffness(30.0);
    try testing.expectEqual(@as(f32, 30.0), c.limits_spring_settings.frequency);
}

test "DistanceConstraint: local space settings" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
    defer context.destroyBodies(&bodies);

    // The points are used as they are, the world space points are calculated from them
    var settings = DistanceConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(0.25, 0.5, 0.75);
    settings.point2 = RVec3.init(-1, -2, -3);
    var c = DistanceConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position1.eql(Vec3.init(0.25, 0.5, 0.75)) and c.local_space_position2.eql(Vec3.init(-1, -2, -3)));
    try testing.expect(c.world_space_position1.eql(bodies[0].getCenterOfMassTransform().mulRVec3(settings.point1)));
    try testing.expect(c.world_space_position2.eql(bodies[1].getCenterOfMassTransform().mulRVec3(settings.point2)));
    try testing.expectEqual(testWorldDistance(&c), c.getMinDistance());
    try testing.expectEqual(testWorldDistance(&c), c.getMaxDistance());

    // A kinematic and a dynamic body: active, the kinematic body does not move
    const before = TestContext.BodySnapshot.take(bodies[0]);
    c.asConstraint().setupVelocityConstraint(1.0 / 60.0);
    c.asConstraint().warmStartVelocityConstraint(1.0);
    try testing.expect(c.asConstraint().solveVelocityConstraint(1.0 / 60.0));
    try TestContext.checkUnmoved(bodies[0], before);
}

test "DistanceConstraint: solver branches, limits and springs" {
    // Body 1 is the world: point 1 at the origin, body 2 hangs at distance 2 below it and moves along the rope
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const dt: f32 = 1.0 / 60.0;

    const Case = struct {
        min: f32,
        max: f32,
        velocity_y: f32,
        /// Active after SetupVelocityConstraint
        active: bool,
        min_lambda: f32 = 0,
        max_lambda: f32 = 0,
        /// SolveVelocityConstraint applies an impulse (hard limit)
        applied: bool = false,
        /// SolvePositionConstraint applies an impulse (hard limit)
        position: bool = false,
    };
    const flt_max = math.flt_max;
    const cases = [_]Case{
        // Single distance: forces in both directions
        .{ .min = 2, .max = 2, .velocity_y = -1, .active = true, .min_lambda = -flt_max, .max_lambda = flt_max, .applied = true },
        .{ .min = 2, .max = 2, .velocity_y = 1, .active = true, .min_lambda = -flt_max, .max_lambda = flt_max, .applied = true },
        // Too close: only pushes apart (moving apart needs no impulse, moving closer does)
        .{ .min = 2.5, .max = 3, .velocity_y = -1, .active = true, .min_lambda = 0, .max_lambda = flt_max, .applied = false, .position = true },
        .{ .min = 2.5, .max = 3, .velocity_y = 1, .active = true, .min_lambda = 0, .max_lambda = flt_max, .applied = true, .position = true },
        // Too far: only pulls together
        .{ .min = 0.5, .max = 1.5, .velocity_y = -1, .active = true, .min_lambda = -flt_max, .max_lambda = 0, .applied = true, .position = true },
        .{ .min = 0.5, .max = 1.5, .velocity_y = 1, .active = true, .min_lambda = -flt_max, .max_lambda = 0, .applied = false, .position = true },
        // Between the limits: not active
        .{ .min = 1, .max = 3, .velocity_y = -1, .active = false },
    };
    for (cases) |case| {
        for ([_]bool{ false, true }) |soft| {
            const body = try context.createBody(.dynamic, .{ .position = RVec3.init(0, -2, 0), .linear_velocity = Vec3.init(0, case.velocity_y, 0) });
            defer context.destroyBodies(&.{body});
            var settings = DistanceConstraintSettings.init(allocator);
            defer settings.asSettings().deinit();
            settings.point1 = RVec3.zero();
            settings.point2 = RVec3.init(0, -2, 0);
            settings.min_distance = case.min;
            settings.max_distance = case.max;
            if (soft) settings.limits_spring_settings = .init(.frequency_and_damping, 5.0, 0.5);
            var c = DistanceConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
            defer c.asConstraint().deinit();
            const base = c.asConstraint();
            try testing.expect(base.isActive());

            // Setup: the normal points from point 1 to point 2, the lambda range of the branch
            base.setupVelocityConstraint(dt);
            try testing.expect(c.world_space_normal.isClose(Vec3.axisY().negate(), .{ .max_dist_sq = 1.0e-12 }));
            try testing.expectEqual(case.active, c.axis_constraint.isActive());
            if (case.active) {
                try testing.expectEqual(case.min_lambda, c.min_lambda);
                try testing.expectEqual(case.max_lambda, c.max_lambda);
                try testing.expectEqual(soft, c.axis_constraint.spring_part.isActive());
            }

            // Velocity: the impulse is clamped to the lambda range
            base.warmStartVelocityConstraint(1.0);
            const applied = base.solveVelocityConstraint(dt);
            if (!soft) try testing.expectEqual(case.applied, applied);
            if (case.active and !soft) {
                const lambda = c.getTotalLambdaPosition();
                try testing.expect(lambda >= case.min_lambda and lambda <= case.max_lambda);
                if (case.applied) try testing.expect(@abs(body.getLinearVelocity().getY()) < 1.0e-5); // Hard: no velocity along the rope
            } else if (!case.active) {
                try testing.expect(!applied);
                try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPosition());
                try testing.expectEqual(case.velocity_y, body.getLinearVelocity().getY());
            }

            // Position: only a hard limit that is violated is corrected (the spring does it through the velocity bias)
            const position_before = body.getCenterOfMassPosition();
            const corrected = base.solvePositionConstraint(dt, 0.2);
            try testing.expectEqual(case.position and !soft, corrected);
            if (corrected) {
                // The distance moves towards the violated limit
                const limit: f32 = if (case.min > 2) case.min else case.max;
                const new_distance: f32 = @floatCast(body.getCenterOfMassPosition().length());
                try testing.expect(@abs(new_distance - limit) < @abs(2.0 - limit));
            } else {
                try testing.expect(body.getCenterOfMassPosition().eql(position_before));
            }
        }
    }
}

test "DistanceConstraint: coinciding points keep the last normal" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .linear_velocity = Vec3.init(0, -1, 0) });
    defer context.destroyBodies(&.{body});

    // Point 1 = point 2 (a cheap point constraint along one axis): the normal stays Y (the constructor's guess)
    var settings = DistanceConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = RVec3.init(1, 2, 3);
    settings.point2 = RVec3.init(1, 2, 3);
    var c = DistanceConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expectEqual(@as(f32, 0.0), c.getMinDistance());
    try testing.expectEqual(@as(f32, 0.0), c.getMaxDistance());
    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.world_space_normal.eql(Vec3.axisY()));
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
    try testing.expect(@abs(body.getLinearVelocity().getY()) < 1.0e-6); // The velocity along Y is removed

    // A normal from an earlier frame is kept while the points coincide (and is part of the state)
    c.world_space_normal = Vec3.axisX();
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.world_space_normal.eql(Vec3.axisX()));
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    c.world_space_normal = Vec3.axisZ();
    base.restoreState(recorder.recorder());
    try testing.expect(c.world_space_normal.eql(Vec3.axisX()));

    // A point that moved away from the other one: the normal follows it
    c.local_space_position2 = Vec3.init(0, 0, 2);
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.world_space_normal.isClose(Vec3.axisZ(), .{ .max_dist_sq = 1.0e-12 }));
}

test "DistanceConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try DistanceConstraintSettings.create(allocator)).asSettings());
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
