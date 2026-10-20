//! Zolt addition, no Jolt file: Zolt-only tests of the debug renderer (Docs/Zolt/RemainingFeaturesArchitecture.md R2 to
//! R5). UnitTests/ has no debug renderer test.
//! Status: complete
//!
//! - The debug renderer hooks are option-gated vtable entries that compile away without `-Ddebug_renderer=true`: the
//!   shape, constraint and vehicle controller vtables keep their size, PhysicsSystem.debug_renderer is `void`.
//! - Zolt's recorder writes the bytes that Jolt's DebugRendererRecorder writes for the same calls. The golden files in
//!   `golden/` were written by Jolt (built like the parity reference plus JPH_DEBUG_RENDERER, single precision) in the
//!   Zig design prototype of Phase 9. The debug renderer parity (R1, group D0) compares with the C++ library directly;
//!   these files stay as small permanent pins.
//! - A user renderer is a pattern A class (embed `base: DebugRenderer`, list the overrides).
//! - An allocation failure while drawing is latched and reported by `endFrame` (R5).

const std = @import("std");
const zolt = @import("zolt");

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualSlices = std.testing.expectEqualSlices;

const AABox = zolt.AABox;
const Color = zolt.Color;
const Quat = zolt.Quat;
const RMat44 = zolt.RMat44;
const RVec3 = zolt.RVec3;
const Vec3 = zolt.Vec3;
const DebugRenderer = zolt.DebugRenderer;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const Layers = @import("../Layers.zig").Layers;

test "DebugRenderer: the hooks compile away without -Ddebug_renderer" {
    const on = zolt.Core.debug_renderer;
    inline for (.{ "draw", "drawGetSupportFunction", "drawGetSupportingFace" }) |name| {
        // A vtable slot and a dispatcher only with the option
        try expect((@FieldType(zolt.Shape.VTable, name) == void) == !on);
        try expect((@TypeOf(@field(zolt.Shape, name)) == void) == !on);
    }
    try expect((@TypeOf(zolt.BoxShape.draw) == void) == !on);

    // Without the option the void entries occupy no space: every shape vtable is exactly as large as before Phase 9
    comptime var fn_entries: usize = 0;
    inline for (@typeInfo(zolt.Shape.VTable).@"struct".fields) |f| {
        if (f.type != void) fn_entries += 1;
    }
    try expectEqual(fn_entries * @sizeOf(usize), @sizeOf(zolt.Shape.VTable));

    // A debug-only member is void without the option (Constraint::mDrawConstraintSize)
    try expect((@FieldType(zolt.Constraint, "draw_constraint_size") == void) == !on);
}

/// The number of function pointer slots of a vtable (its void entries have none, the base table counts)
fn numSlots(comptime VTable: type) comptime_int {
    var n: comptime_int = 0;
    for (@typeInfo(VTable).@"struct".fields) |f| {
        if (@typeInfo(f.type) == .@"struct") {
            n += numSlots(f.type);
        } else if (f.type != void) {
            n += 1;
        }
    }
    return n;
}

test "DebugRenderer: the constraint and vehicle controller draw hooks compile away without -Ddebug_renderer" {
    const on = zolt.Core.debug_renderer;

    // Constraint: DrawConstraint, DrawConstraintLimits, DrawConstraintReferenceFrame
    inline for (.{ "drawConstraint", "drawConstraintLimits", "drawConstraintReferenceFrame" }) |name| {
        try expect((@FieldType(zolt.Constraint.VTable, name) == void) == !on);
        try expect((@TypeOf(@field(zolt.Constraint, name)) == void) == !on);
        try expect((@TypeOf(@field(zolt.Constraint.impl, name)) == void) == !on);
    }
    try expect((@TypeOf(zolt.TwoBodyConstraint.impl.drawConstraintReferenceFrame) == void) == !on);

    // VehicleController: Draw
    try expect((@FieldType(zolt.VehicleController.VTable, "draw") == void) == !on);
    try expect((@TypeOf(zolt.VehicleController.draw) == void) == !on);
    try expect((@TypeOf(zolt.VehicleController.impl.draw) == void) == !on);

    // Without the option the vtables keep their size from before Phase 9 (16 slots for Constraint, 2 more for
    // TwoBodyConstraint, 10 for VehicleController), the option adds one slot per draw function
    const extra_constraint = if (on) 3 else 0;
    try expectEqual(@as(usize, 16 + extra_constraint), numSlots(zolt.Constraint.VTable));
    try expectEqual(numSlots(zolt.Constraint.VTable) * @sizeOf(usize), @sizeOf(zolt.Constraint.VTable));
    try expectEqual(@as(usize, 18 + extra_constraint), numSlots(zolt.TwoBodyConstraint.VTable));
    try expectEqual(numSlots(zolt.TwoBodyConstraint.VTable) * @sizeOf(usize), @sizeOf(zolt.TwoBodyConstraint.VTable));
    const extra_controller = if (on) 1 else 0;
    try expectEqual(@as(usize, 10 + extra_controller), numSlots(zolt.VehicleController.VTable));
    try expectEqual(numSlots(zolt.VehicleController.VTable) * @sizeOf(usize), @sizeOf(zolt.VehicleController.VTable));

    // The renderer of the physics system (Jolt's DebugRenderer::sInstance, R4)
    try expect((@FieldType(zolt.PhysicsSystem, "debug_renderer") == void) == !on);
    try expect((@TypeOf(zolt.PhysicsSystem.setDebugRenderer) == void) == !on);
}

test "DebugRenderer: PhysicsSystem.setDebugRenderer" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest; // The field only exists with the option

    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    try expect(c.system.debug_renderer == null);

    var renderer: LoggingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    c.system.setDebugRenderer(&renderer.base);
    try expect(c.system.debug_renderer == &renderer.base);
    c.system.setDebugRenderer(null);
    try expect(c.system.debug_renderer == null);
}

test "DebugRenderer: GetConstraintSettings copies the draw size only with the option (R1)" {
    // Jolt's Constraint::ToConstraintSettings copies mDrawConstraintSize under JPH_IF_DEBUG_RENDERER. This is the result
    // that makes the parity reference follow -Ddebug_renderer.
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body = try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{});

    var settings = zolt.PointConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.base.base.draw_constraint_size = 0.1;
    var constraint = zolt.Ref(zolt.Constraint).init((try settings.createConstraint(allocator, body, zolt.Body.fixedToWorld())).asConstraint());
    defer constraint.deinit();
    if (zolt.Core.debug_renderer) {
        try expectEqual(@as(f32, 0.1), constraint.get().?.getDrawConstraintSize());
        constraint.get().?.setDrawConstraintSize(0.25);
    }

    var settings2 = try constraint.get().?.getConstraintSettings();
    defer settings2.deinit();
    try expectEqual(@as(f32, if (zolt.Core.debug_renderer) 0.25 else 1.0), settings2.get().?.draw_constraint_size);
}

/// Jolt's DebugRendererRecorder writes these bytes for the calls of the tests below
/// (box_batch.jor: the first command that Jolt's Initialize writes, the box; Initialize then writes 24 more batches, the
/// debug renderer parity compares all of them)
const golden_box_batch = @embedFile("golden/box_batch.jor");
const golden_frames = @embedFile("golden/box_draw_frames.jor");

test "DebugRenderer: the recorder writes Jolt's box batch of Initialize" {
    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: zolt.StreamOutWrapper = .init(&out.writer);

    var recorder: zolt.DebugRendererRecorder = undefined;
    try recorder.init(allocator, std.testing.io, wrapper.streamOut());
    defer recorder.deinit();

    // Jolt's Initialize writes 25 batches, the box first. The vertices are Float3, so this holds in both precisions.
    try expectEqualSlices(u8, golden_box_batch, out.written()[0..golden_box_batch.len]);
    try expectEqual(@as(u32, 26), recorder.next_batch_id);
}

test "DebugRenderer: BoxShape.draw and the wire helpers write Jolt's .jor bytes" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest; // Shape.draw only exists with the option

    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: zolt.StreamOutWrapper = .init(&out.writer);

    var recorder: zolt.DebugRendererRecorder = undefined;
    try recorder.init(allocator, std.testing.io, wrapper.streamOut());
    defer recorder.deinit();
    out.clearRetainingCapacity();

    const r = recorder.debugRenderer();
    var box = zolt.BoxShape.init(allocator, Vec3.init(1, 2, 3), .{ .convex_radius = 0.1 });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const com = RMat44.rotationTranslation(Quat.rotation(Vec3.init(0.3, 1, -0.2).normalized(), 0.3), RVec3.init(1, 2, 3));
    box.asShape().draw(r, com, Vec3.init(1, -2, 0.5), Color.red, false, false);
    box.asShape().draw(r, com, Vec3.init(2, 2, 2), Color.green, true, true);
    r.drawWireBox(AABox.init(Vec3.init(-1, -2, -3), Vec3.init(4, 5, 6)), Color.yellow);
    r.drawWireBoxMatrix(com, AABox.init(Vec3.init(-1, -2, -3), Vec3.init(4, 5, 6)), Color.blue);
    r.drawMarker(RVec3.init(1, 2, 3), Color.cyan, 0.5);
    r.drawArrow(RVec3.init(1, 2, 3), RVec3.init(4, -5, 6), Color.orange, 0.25);
    r.drawArrow(RVec3.init(1, 2, 3), RVec3.init(1, 2, 3), Color.orange, 0.25);
    try recorder.endFrame();
    box.asShape().draw(r, com, Vec3.init(1, 1, 1), Color.white, false, false); // The geometry is cached: no CreateGeometry
    try recorder.endFrame();

    // The frames hold RVec3 / RMat44, the golden file is single precision
    if (!zolt.Core.double_precision)
        try expectEqualSlices(u8, golden_frames, out.written());
}

/// A user implemented renderer (pattern A): logs what the recorder does not record
const LoggingRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const Batch = struct {
        pub const overrides = .{};
        base: DebugRenderer.BatchTarget,
    };

    base: DebugRenderer,
    num_lines: u32 = 0,
    num_batches: u32 = 0,
    last_world_space_bounds: AABox = .empty,
    last_lod_scale_sq: f32 = 0,

    fn init(self: *LoggingRenderer, allocator: std.mem.Allocator) !void {
        self.* = .{ .base = .init(LoggingRenderer, allocator, std.testing.io) };
        errdefer self.base.deinit();
        try self.base.initialize();
    }

    pub fn drawLine(self: *LoggingRenderer, from: RVec3, to: RVec3, color: Color) void {
        _ = .{ from, to, color };
        self.num_lines += 1;
    }
    pub fn drawTriangle(self: *LoggingRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow) void {
        _ = .{ self, v1, v2, v3, color, cast_shadow };
    }
    pub fn createTriangleBatch(self: *LoggingRenderer, triangles: []const DebugRenderer.Triangle) DebugRenderer.Batch {
        _ = triangles;
        return self.newBatch();
    }
    pub fn createTriangleBatchIndexed(self: *LoggingRenderer, vertices: []const DebugRenderer.Vertex, indices: []const u32) DebugRenderer.Batch {
        _ = .{ vertices, indices };
        return self.newBatch();
    }
    pub fn drawGeometry(self: *LoggingRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const DebugRenderer.Geometry, cull_mode: DebugRenderer.CullMode, cast_shadow: DebugRenderer.CastShadow, draw_mode: DebugRenderer.DrawMode) void {
        _ = .{ model_matrix, model_color, geometry, cull_mode, cast_shadow, draw_mode };
        self.last_world_space_bounds = world_space_bounds;
        self.last_lod_scale_sq = lod_scale_sq;
    }
    pub fn drawText3D(self: *LoggingRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        _ = .{ self, position, string, color, height };
    }

    fn newBatch(self: *LoggingRenderer) DebugRenderer.Batch {
        const batch = self.base.allocator.create(Batch) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(Batch, self.base.allocator) };
        self.num_batches += 1;
        return .init(&batch.base);
    }
};

test "DebugRenderer: a user renderer receives the draw calls (bounds and LOD scale that .jor files drop)" {
    var renderer: LoggingRenderer = undefined;
    try renderer.init(std.testing.allocator);
    defer renderer.base.deinit();
    try expectEqual(@as(u32, 25), renderer.num_batches); // The predefined geometry of initialize

    renderer.base.drawWireBox(AABox.init(Vec3.zero(), Vec3.one()), Color.white);
    try expectEqual(@as(u32, 12), renderer.num_lines);

    renderer.base.drawBox(AABox.init(Vec3.zero(), Vec3.init(2, 4, 6)), Color.white, .{});
    try expect(renderer.last_world_space_bounds.min.isClose(Vec3.zero(), .{}));
    try expect(renderer.last_world_space_bounds.max.isClose(Vec3.init(2, 4, 6), .{}));
    try expectEqual(@as(f32, 9), renderer.last_lod_scale_sq); // The largest squared axis length of the model matrix

    // Jolt's static draw flags are fields of the renderer (R4)
    try expect(!renderer.base.flags.mesh_shape.draw_triangle_groups);
}

test "DebugRenderer: an allocation failure while drawing is latched and reported by endFrame" {
    var sink: std.Io.Writer.Allocating = .init(std.testing.allocator);
    defer sink.deinit();
    var wrapper: zolt.StreamOutWrapper = .init(&sink.writer);
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{});
    var recorder: zolt.DebugRendererRecorder = undefined;
    try recorder.init(failing.allocator(), std.testing.io, wrapper.streamOut());
    defer recorder.deinit();

    failing.fail_index = failing.alloc_index; // Every further allocation fails
    recorder.debugRenderer().drawLine(RVec3.zero(), RVec3.init(1, 1, 1), Color.red); // Dropped, latched
    try std.testing.expectError(error.OutOfMemory, recorder.endFrame());
}
