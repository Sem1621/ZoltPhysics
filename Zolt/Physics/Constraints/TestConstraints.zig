//! Zolt addition, no Jolt file: a two body test constraint for the tests of Constraint, ConstraintManager and
//! PhysicsSystem
//! Status: complete
//!
//! Only used by tests: `zolt.test_constraints` is this file in test builds (`builtin.is_test`), and build.zig gives the
//! inline tests a `zolt_user_types` module that registers `serializable_types` of this file, so that
//! ConstraintSettings.restoreFromBinaryState can create `TestConstraintSettings` (the unit tests use the default, empty
//! module). The F2 tests (written before Jolt's constraint types of Phase 6) need a concrete TwoBodyConstraint with
//! observable solver calls to instantiate the virtual function machinery (SimulationArchitecture D8): `TestConstraintSettings` (a TwoBodyConstraintSettings
//! with a binary state) creates a `TestConstraint` that counts the solver calls and stores one float of state. The
//! solver functions return a configurable "impulse applied" result so that the OR semantics of the ConstraintManager
//! can be tested.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Ref = @import("../../Core/Reference.zig").Ref;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Core = @import("../../Core/Core.zig");
const Color = @import("../../Core/Color.zig").Color;
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
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

/// The serializable classes that the inline tests register through `zolt_user_types` (see build.zig)
pub const serializable_types = .{TestConstraintSettings};

/// Settings of TestConstraint
pub const TestConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{ .name = "TestConstraintSettings", .kind = .virtual, .bases = &.{TwoBodyConstraintSettings} });
    pub const overrides = .{ .createConstraint, .saveBinaryState, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// Initial value of TestConstraint.value
    value: f32 = 0.0,

    /// new TestConstraintSettings
    pub fn create(allocator: Allocator) Allocator.Error!*TestConstraintSettings {
        const self = try allocator.create(TestConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// Default constructor on the heap (RTTI create, used by ConstraintSettings.restoreFromBinaryState)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*TestConstraintSettings {
        return create(allocator);
    }

    /// Constructor of a settings object that is not on the heap (the caller calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) TestConstraintSettings {
        return .{ .base = .init(TestConstraintSettings, allocator) };
    }

    pub fn asSettings(self: *TestConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn createConstraint(self: *const TestConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try allocator.create(TestConstraint);
        constraint.* = .{ .base = .init(TestConstraint, allocator, body1, body2, &self.base), .value = self.value };
        return &constraint.base;
    }

    pub fn saveBinaryState(self: *const TestConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(&self.base.base, stream);
        stream.write(self.value);
    }

    pub fn restoreBinaryState(self: *TestConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(&self.base.base, stream);
        stream.read(&self.value);
    }
};

/// A two body constraint that records the calls of the solver
pub const TestConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .resetWarmStart, .setupVelocityConstraint, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .drawConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    /// State that is saved and restored
    value: f32 = 0.0,

    /// Results of the solver functions ("an impulse was applied")
    velocity_result: bool = false,
    position_result: bool = false,

    /// Call counters (atomics: the solver jobs run on worker threads)
    num_setup: std.atomic.Value(u32) = .init(0),
    num_warm_start: std.atomic.Value(u32) = .init(0),
    num_solve_velocity: std.atomic.Value(u32) = .init(0),
    num_solve_position: std.atomic.Value(u32) = .init(0),
    num_reset_warm_start: u32 = 0,
    last_warm_start_impulse_ratio: f32 = 0.0,
    last_delta_com: Vec3 = Vec3.zero(),

    pub fn asConstraint(self: *TestConstraint) *Constraint {
        return &self.base.base;
    }

    /// Draws a line between the centers of mass of the two bodies (JPH_DEBUG_RENDERER: Constraint::DrawConstraint is
    /// pure virtual)
    pub const drawConstraint = if (Core.debug_renderer) drawConstraintImpl else {};
    fn drawConstraintImpl(self: *const TestConstraint, renderer: *DebugRenderer) void {
        renderer.drawLine(self.base.body1.getCenterOfMassPosition(), self.base.body2.getCenterOfMassPosition(), Color.white);
    }

    pub fn getSubType(self: *const TestConstraint) ConstraintSubType {
        _ = self;
        return .user1;
    }

    pub fn notifyShapeChanged(self: *TestConstraint, body_id: BodyID, delta_com: Vec3) void {
        _ = body_id;
        self.last_delta_com = delta_com;
    }

    pub fn resetWarmStart(self: *TestConstraint) void {
        self.num_reset_warm_start += 1;
    }

    pub fn setupVelocityConstraint(self: *TestConstraint, delta_time: f32) void {
        _ = delta_time;
        _ = self.num_setup.fetchAdd(1, .monotonic);
    }

    pub fn warmStartVelocityConstraint(self: *TestConstraint, warm_start_impulse_ratio: f32) void {
        self.last_warm_start_impulse_ratio = warm_start_impulse_ratio;
        _ = self.num_warm_start.fetchAdd(1, .monotonic);
    }

    pub fn solveVelocityConstraint(self: *TestConstraint, delta_time: f32) bool {
        _ = delta_time;
        _ = self.num_solve_velocity.fetchAdd(1, .monotonic);
        return self.velocity_result;
    }

    pub fn solvePositionConstraint(self: *TestConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;
        _ = baumgarte;
        _ = self.num_solve_position.fetchAdd(1, .monotonic);
        return self.position_result;
    }

    pub fn saveState(self: *const TestConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(&self.base.base, stream);
        stream.write(self.value);
    }

    pub fn restoreState(self: *TestConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(&self.base.base, stream);
        stream.read(&self.value);
    }

    pub fn getConstraintSettings(self: *const TestConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try TestConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(&settings.base.base);
        settings.value = self.value;
        return .init(&settings.base.base);
    }

    pub fn getConstraintToBody1Matrix(self: *const TestConstraint) Mat44 {
        _ = self;
        return Mat44.identity();
    }

    pub fn getConstraintToBody2Matrix(self: *const TestConstraint) Mat44 {
        _ = self;
        return Mat44.identity();
    }
};

const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

/// Restores constraint settings from `bytes`
fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!ConstraintSettings.ConstraintResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    return ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
}

test "ConstraintSettings: binary state, restoreFromBinaryState and the copy constructor" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // Settings with non default values
    var settings = TestConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 7;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 4;
    settings.base.base.draw_constraint_size = 2.5;
    settings.value = 1.25;

    // SaveBinaryState writes the RTTI hash and Jolt's fields in Jolt's order
    var buffer: std.ArrayList(u8) = .empty;
    defer buffer.deinit(allocator);
    {
        var writer: std.Io.Writer.Allocating = .fromArrayList(allocator, &buffer);
        defer buffer = writer.toArrayList();
        var out = StreamOutWrapper.init(&writer.writer);
        settings.asSettings().saveBinaryState(out.streamOut());
    }
    try std.testing.expectEqual(@as(usize, 4 + 1 + 4 + 4 + 4 + 4 + 4), buffer.items.len);
    try std.testing.expectEqual(RTTI.hashName("TestConstraintSettings"), std.mem.readInt(u32, buffer.items[0..4], .little));
    try std.testing.expectEqual(settings.asSettings().getRTTIHash(), RTTI.hashName("TestConstraintSettings"));

    // Round trip through the registered type list
    {
        var result = try restoreFromBuffer(allocator, buffer.items);
        defer result.deinit();
        try expect(result.isValid());
        const restored: *TestConstraintSettings = @fieldParentPtr("base", @as(*TwoBodyConstraintSettings, @fieldParentPtr("base", result.get().get().?)));
        try expect(!restored.base.base.enabled);
        try std.testing.expectEqual(@as(u32, 7), restored.base.base.constraint_priority);
        try std.testing.expectEqual(@as(u32, 3), restored.base.base.num_velocity_steps_override);
        try std.testing.expectEqual(@as(u32, 4), restored.base.base.num_position_steps_override);
        try std.testing.expectEqual(@as(f32, 2.5), restored.base.base.draw_constraint_size);
        try std.testing.expectEqual(@as(f32, 1.25), restored.value);
    }

    // Jolt's errors: an empty stream, an unknown type, a truncated object
    {
        var result = try restoreFromBuffer(allocator, &.{});
        defer result.deinit();
        try std.testing.expectEqualStrings("Failed to read type hash", result.getError());
    }
    {
        const unknown = [_]u8{ 0x78, 0x56, 0x34, 0x12 } ++ [_]u8{0} ** 21;
        var result = try restoreFromBuffer(allocator, &unknown);
        defer result.deinit();
        try std.testing.expectEqualStrings("Failed to create instance of type", result.getError());
    }
    {
        var result = try restoreFromBuffer(allocator, buffer.items[0 .. buffer.items.len - 2]);
        defer result.deinit();
        try std.testing.expectEqualStrings("Failed to restore object", result.getError());
    }

    // Out of memory while creating the object: the error arrives, nothing leaks
    const Ctx = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            var result = try restoreFromBuffer(a, bytes);
            result.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(allocator, Ctx.run, .{buffer.items});

    // The copy constructor copies the settings but not the reference count
    settings.asSettings().setEmbedded();
    settings.asSettings().addRef();
    defer settings.asSettings().release();
    var copy = TestConstraintSettings{ .base = .{ .base = .initCopy(settings.asSettings().vtable, allocator, settings.asSettings()) }, .value = settings.value };
    defer copy.asSettings().deinit();
    try std.testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try std.testing.expectEqual(@as(u32, 7), copy.base.base.constraint_priority);
}
