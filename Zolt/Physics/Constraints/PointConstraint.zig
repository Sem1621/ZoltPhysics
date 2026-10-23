//! Port of: Jolt/Physics/Constraints/PointConstraint.h, Jolt/Physics/Constraints/PointConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PointConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint): a gated declaration (RemainingFeaturesArchitecture.md R2).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Core = @import("../../Core/Core.zig");
const Color = @import("../../Core/Color.zig").Color;
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
const PointConstraintPart = @import("ConstraintPart/PointConstraintPart.zig").PointConstraintPart;

/// Point constraint settings, used to create a point constraint
pub const PointConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PointConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "PointConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .enumAttr("mSpace", .space),
            .attr("mPoint1", .point1),
            .attr("mPoint2", .point2),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint position (space determined by mSpace).
    point1: RVec3 = RVec3.zero(),

    /// Body 2 constraint position (space determined by mSpace).
    /// Note: Normally you would set mPoint1 = mPoint2 if the bodies are already placed how you want to constrain them (if mSpace = world space).
    point2: RVec3 = RVec3.zero(),

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) PointConstraintSettings {
        return .{ .base = .init(PointConstraintSettings, allocator) };
    }

    /// new PointConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*PointConstraintSettings {
        const self = try allocator.create(PointConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const PointConstraintSettings) PointConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *PointConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const PointConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *PointConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const PointConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const PointConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.point1);
        stream.write(self.point2);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const PointConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) TwoBodyConstraintSettings.CreateError!*TwoBodyConstraint {
        const constraint = try PointConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *PointConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.point1);
        stream.read(&self.point2);
    }
};

/// A point constraint constrains 2 bodies on a single point (removing 3 degrees of freedom)
pub const PointConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .drawConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // The constraint part
    point_constraint_part: PointConstraintPart = .{},

    /// Construct point constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const PointConstraintSettings) PointConstraint {
        var self: PointConstraint = .{ .base = .init(PointConstraint, allocator, body1, body2, &settings.base) };

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            self.local_space_position1 = body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3();
            self.local_space_position2 = body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3();
        } else {
            self.local_space_position1 = settings.point1.toVec3();
            self.local_space_position2 = settings.point2.toVec3();
        }

        return self;
    }

    /// new PointConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const PointConstraintSettings) Allocator.Error!*PointConstraint {
        const self = try allocator.create(PointConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *PointConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const PointConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *PointConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const PointConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const PointConstraint) ConstraintSubType {
        _ = self;
        return .point;
    }

    pub fn notifyShapeChanged(self: *PointConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    /// Update the attachment point for body 1
    pub fn setPoint1(self: *PointConstraint, space: ConstraintSpace, point1: RVec3) void {
        if (space == .world_space) {
            self.local_space_position1 = self.base.body1.getInverseCenterOfMassTransform().mulRVec3(point1).toVec3();
        } else {
            self.local_space_position1 = point1.toVec3();
        }
    }

    /// Update the attachment point for body 2
    pub fn setPoint2(self: *PointConstraint, space: ConstraintSpace, point2: RVec3) void {
        if (space == .world_space) {
            self.local_space_position2 = self.base.body2.getInverseCenterOfMassTransform().mulRVec3(point2).toVec3();
        } else {
            self.local_space_position2 = point2.toVec3();
        }
    }

    /// Get the attachment point for body 1 relative to body 1 COM (transform by Body::GetCenterOfMassTransform to take to world space)
    pub fn getLocalSpacePoint1(self: *const PointConstraint) Vec3 {
        return self.local_space_position1;
    }

    /// Get the attachment point for body 2 relative to body 2 COM (transform by Body::GetCenterOfMassTransform to take to world space)
    pub fn getLocalSpacePoint2(self: *const PointConstraint) Vec3 {
        return self.local_space_position2;
    }

    // Internal helper function to calculate the values below
    fn calculateConstraintProperties(self: *PointConstraint) void {
        self.point_constraint_part.calculateConstraintProperties(self.base.body1, Mat44.rotationQuat(self.base.body1.getRotation()), self.local_space_position1, self.base.body2, Mat44.rotationQuat(self.base.body2.getRotation()), self.local_space_position2);
    }

    pub fn setupVelocityConstraint(self: *PointConstraint, delta_time: f32) void {
        _ = delta_time;
        self.calculateConstraintProperties();
    }

    pub fn resetWarmStart(self: *PointConstraint) void {
        self.point_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *PointConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.point_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *PointConstraint, delta_time: f32) bool {
        _ = delta_time;
        return self.point_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);
    }

    pub fn solvePositionConstraint(self: *PointConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        // Update constraint properties (bodies may have moved)
        self.calculateConstraintProperties();

        return self.point_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, baumgarte);
    }

    // See Constraint::DrawConstraint (JPH_DEBUG_RENDERER)
    pub const drawConstraint = if (Core.debug_renderer) drawConstraintImpl else {};
    fn drawConstraintImpl(self: *const PointConstraint, renderer: *DebugRenderer) void {
        // Draw constraint
        renderer.drawMarker(self.base.body1.getCenterOfMassTransform().mulVec3(self.local_space_position1), Color.red, 0.1);
        renderer.drawMarker(self.base.body2.getCenterOfMassTransform().mulVec3(self.local_space_position2), Color.green, 0.1);
    }

    pub fn saveState(self: *const PointConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.point_constraint_part.saveState(stream);
    }

    pub fn restoreState(self: *PointConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.point_constraint_part.restoreState(stream);
    }

    pub fn getConstraintSettings(self: *const PointConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try PointConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.fromVec3(self.local_space_position1);
        settings.point2 = RVec3.fromVec3(self.local_space_position2);
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const PointConstraint) Mat44 {
        return Mat44.translation(self.local_space_position1);
    }

    pub fn getConstraintToBody2Matrix(self: *const PointConstraint) Mat44 {
        return Mat44.translation(self.local_space_position2); // Note: Incorrect rotation as we don't track the original rotation difference, should not matter though as the constraint is not limiting rotation.
    }

    ///@name Get Lagrange multiplier from last physics update (the linear impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const PointConstraint) Vec3 {
        return self.point_constraint_part.getTotalLambda();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/PointConstraintTests.zig)

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

test "PointConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = PointConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.point1.eql(RVec3.zero()) and settings.point2.eql(RVec3.zero()));
    try testing.expectEqualStrings("PointConstraintSettings", settings.asSettings().vtable.rtti.name);

    settings.base.base.constraint_priority = 5;
    settings.base.base.user_data = 1234; // Not part of the binary state (Jolt)
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(1, -2, 3);
    settings.point2 = RVec3.init(0.5, 0.25, -0.125);

    // ConstraintSettings (21 bytes), space (int), the points (Real)
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 2 * 3 * @sizeOf(Real)), bytes.len);
    try testing.expectEqual(RTTI.hashName("PointConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(PointConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expectEqual(@as(u32, 5), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.point1.eql(settings.point1) and restored.point2.eql(settings.point2));
    }

    // A space value outside the named ones is restored as it is (an int, like Jolt's enum) and means local space
    {
        var odd = try allocator.dupe(u8, bytes);
        defer allocator.free(odd);
        std.mem.writeInt(i32, odd[21..25], 7, .little);
        var result = try testRestoreSettings(allocator, odd);
        defer result.deinit();
        const restored = virtual.downcast(PointConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expectEqual(@as(i32, 7), @intFromEnum(restored.space));
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, odd, resaved);
    }

    // An unknown type is Jolt's "Failed to create instance of type". Zolt (D8): so are an abstract class and a class of
    // another hierarchy, which RegisterTypes.factory knows.
    for ([_][]const u8{ "UnknownConstraintSettings", "TwoBodyConstraintSettings", "PathConstraintPathHermite" }) |name| {
        var odd = try allocator.dupe(u8, bytes);
        defer allocator.free(odd);
        std.mem.writeInt(u32, odd[0..4], RTTI.hashName(name), .little);
        var result = try testRestoreSettings(allocator, odd);
        defer result.deinit();
        try testing.expectEqualStrings("Failed to create instance of type", result.getError());
    }

    // The copy constructor
    var copy = PointConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expect(copy.point1.eql(settings.point1) and copy.space == settings.space);
    try testing.expectEqual(@as(u64, 1234), copy.base.base.user_data);

    const Ctx = struct {
        fn run(a: Allocator, b: []const u8) !void {
            var result = try testRestoreSettings(a, b);
            result.deinit();
        }
    };
    try testing.checkAllAllocationFailures(allocator, Ctx.run, .{bytes});
}

test "PointConstraint: virtual dispatch, space conversions, setters, solver, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];

        var settings = PointConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.point1 = RVec3.init(1.5, 1.75, 3.5);
        settings.point2 = RVec3.init(1.75, 1.5, 3.75);

        var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit();
        const base = constraint.get().?;
        const two_body = virtual.downcast(TwoBodyConstraint, base);
        const c = virtual.downcast(PointConstraint, two_body);
        try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
        try testing.expectEqual(ConstraintSubType.point, base.getSubType());
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, base.isActive());

        // World space points in local space, the matrices are translations
        try testing.expect(c.getLocalSpacePoint1().eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3()));
        try testing.expect(c.getLocalSpacePoint2().eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3()));
        try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.translation(c.getLocalSpacePoint1())));
        try testing.expect(two_body.getConstraintToBody2Matrix().eql(Mat44.translation(c.getLocalSpacePoint2())));

        // SetPoint1 / SetPoint2 in both spaces
        c.setPoint1(.local_to_body_com, RVec3.init(0.1, 0.2, 0.3));
        try testing.expect(c.getLocalSpacePoint1().eql(Vec3.init(0.1, 0.2, 0.3)));
        c.setPoint2(.local_to_body_com, RVec3.init(-0.1, 0.2, -0.3));
        try testing.expect(c.getLocalSpacePoint2().eql(Vec3.init(-0.1, 0.2, -0.3)));
        c.setPoint1(.world_space, settings.point1);
        try testing.expect(c.getLocalSpacePoint1().eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3()));
        c.setPoint2(.world_space, settings.point2);
        try testing.expect(c.getLocalSpacePoint2().eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3()));

        // GetConstraintSettings: local space points, a constraint made from them is the same
        {
            var copy = try base.getConstraintSettings();
            defer copy.deinit();
            const s = virtual.downcast(PointConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
            try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
            var again = PointConstraint.init(allocator, body1, body2, s);
            defer again.asConstraint().deinit();
            try testing.expect(again.getLocalSpacePoint1().eql(c.getLocalSpacePoint1()) and again.getLocalSpacePoint2().eql(c.getLocalSpacePoint2()));
        }

        // Solver: the velocity of the attachment points becomes equal, a static or kinematic body does not move
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);
        base.setupVelocityConstraint(1.0 / 60.0);
        base.warmStartVelocityConstraint(1.0);
        const applied = base.solveVelocityConstraint(1.0 / 60.0);
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, applied);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);
        if (applied) {
            for (0..5) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
            const r1 = body1.getRotation().mulVec3(c.getLocalSpacePoint1());
            const r2 = body2.getRotation().mulVec3(c.getLocalSpacePoint2());
            const v1 = body1.getLinearVelocity().add(body1.getAngularVelocity().cross(r1));
            const v2 = body2.getLinearVelocity().add(body2.getAngularVelocity().cross(r2));
            try testing.expect(v1.isClose(v2, .{ .max_dist_sq = 1.0e-6 }));
            try testing.expect(!c.getTotalLambdaPosition().eql(Vec3.zero()));
            try testing.expect(base.solvePositionConstraint(1.0 / 60.0, 0.2));
        }

        // SaveState / RestoreState: enabled + the lambda of the part
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        base.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 1 + 12), recorder.getData().len);
        const lambda = c.getTotalLambdaPosition();
        base.resetWarmStart();
        try testing.expect(c.getTotalLambdaPosition().eql(Vec3.zero()));
        base.restoreState(recorder.recorder());
        try testing.expect(c.getTotalLambdaPosition().eql(lambda));

        // NotifyShapeChanged
        const p1 = c.getLocalSpacePoint1();
        base.notifyShapeChanged(body1.getID(), Vec3.init(0.25, 0, 0));
        try testing.expect(c.getLocalSpacePoint1().eql(p1.sub(Vec3.init(0.25, 0, 0))));
    }
}

test "PointConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const body = try context.createBody(.dynamic, .{});
            defer context.destroyBodies(&.{body});
            var settings = Ref(ConstraintSettings).init((try PointConstraintSettings.create(allocator)).asSettings());
            defer settings.deinit();
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, Body.fixedToWorld(), body)).base);
            defer constraint.deinit();
            var copy = try constraint.get().?.getConstraintSettings();
            defer copy.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
