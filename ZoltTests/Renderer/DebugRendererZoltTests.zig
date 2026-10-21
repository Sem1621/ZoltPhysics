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
//! - The shape draws (group D2): the draw caches (R6), allocation failures in every draw path that allocates (R5), the
//!   buoyancy draw of a body, and Zolt's defined behavior for a null entry of a mesh's material list (Jolt dereferences
//!   null there).

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

// ---------------------------------------------------------------------------------------------------------------------
// The shape draws (group D2): the draw caches of the shapes (R6), allocation failures while drawing (R5), the debug
// options of GetSubmergedVolume and the buoyancy draw of a body

test "DebugRenderer: the draw caches and debug options of the shapes compile away without -Ddebug_renderer" {
    const on = zolt.Core.debug_renderer;
    inline for (.{ zolt.ConvexHullShape, zolt.TaperedCapsuleShape, zolt.MeshShape, zolt.HeightFieldShape }) |T| {
        try expect((@FieldType(T, "draw_cache") == void) == !on);
    }

    // The sizes of the cached shapes (pins, the same in both precisions). The option adds one pointer to the draw cache;
    // only MeshShape grows, the other three have room for it in their padding.
    try expectEqual(@as(usize, if (on) @sizeOf(usize) else 0), @sizeOf(@FieldType(zolt.MeshShape, "draw_cache")));
    try expectEqual(@as(usize, 288), @sizeOf(zolt.ConvexHullShape));
    try expectEqual(@as(usize, 112), @sizeOf(zolt.TaperedCapsuleShape));
    try expectEqual(@as(usize, if (on) 96 else 88), @sizeOf(zolt.MeshShape));
    try expectEqual(@as(usize, 224), @sizeOf(zolt.HeightFieldShape));

    // The debug parameters of GetSubmergedVolume and Body::ApplyBuoyancyImpulse are void options without the option
    try expect((@FieldType(zolt.Shape.SubmergedVolumeOptions, "debug") == void) == !on);
    if (!on) try expectEqual(@as(usize, 0), @sizeOf(zolt.Shape.SubmergedVolumeOptions));
    try expect((@FieldType(zolt.Body.BuoyancyOptions, "debug_renderer") == void) == !on);
    if (!on) try expectEqual(@as(usize, 0), @sizeOf(zolt.Body.BuoyancyOptions));
}

/// A user renderer that counts what it receives (thread safe) and keeps the last text
const CountingRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const Batch = struct {
        pub const overrides = .{};
        base: DebugRenderer.BatchTarget,
    };

    base: DebugRenderer,
    num_batches: std.atomic.Value(u32) = .init(0),
    num_geometry_draws: std.atomic.Value(u32) = .init(0),
    num_lines: std.atomic.Value(u32) = .init(0),
    num_triangles: std.atomic.Value(u32) = .init(0),
    num_texts: std.atomic.Value(u32) = .init(0),
    /// The last text (only for single threaded tests)
    last_text: [64]u8 = undefined,
    last_text_len: usize = 0,
    last_text_position: RVec3 = RVec3.zero(),

    fn init(self: *CountingRenderer, allocator: std.mem.Allocator) !void {
        self.* = .{ .base = .init(CountingRenderer, allocator, std.testing.io) };
        errdefer self.base.deinit();
        try self.base.initialize();
    }

    fn batches(self: *const CountingRenderer) u32 {
        return self.num_batches.load(.monotonic);
    }

    fn lastText(self: *const CountingRenderer) []const u8 {
        return self.last_text[0..self.last_text_len];
    }

    pub fn drawLine(self: *CountingRenderer, from: RVec3, to: RVec3, color: Color) void {
        _ = .{ from, to, color };
        _ = self.num_lines.fetchAdd(1, .monotonic);
    }
    pub fn drawTriangle(self: *CountingRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow) void {
        _ = .{ v1, v2, v3, color, cast_shadow };
        _ = self.num_triangles.fetchAdd(1, .monotonic);
    }
    pub fn createTriangleBatch(self: *CountingRenderer, triangles: []const DebugRenderer.Triangle) DebugRenderer.Batch {
        _ = triangles;
        return self.newBatch();
    }
    pub fn createTriangleBatchIndexed(self: *CountingRenderer, vertices: []const DebugRenderer.Vertex, indices: []const u32) DebugRenderer.Batch {
        _ = .{ vertices, indices };
        return self.newBatch();
    }
    pub fn drawGeometry(self: *CountingRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const DebugRenderer.Geometry, cull_mode: DebugRenderer.CullMode, cast_shadow: DebugRenderer.CastShadow, draw_mode: DebugRenderer.DrawMode) void {
        _ = .{ model_matrix, world_space_bounds, lod_scale_sq, model_color, geometry, cull_mode, cast_shadow, draw_mode };
        _ = self.num_geometry_draws.fetchAdd(1, .monotonic);
    }
    pub fn drawText3D(self: *CountingRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        _ = .{ color, height };
        _ = self.num_texts.fetchAdd(1, .monotonic);
        self.last_text_len = @min(string.len, self.last_text.len);
        @memcpy(self.last_text[0..self.last_text_len], string[0..self.last_text_len]);
        self.last_text_position = position;
    }

    fn newBatch(self: *CountingRenderer) DebugRenderer.Batch {
        const batch = self.base.allocator.create(Batch) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(Batch, self.base.allocator) };
        _ = self.num_batches.fetchAdd(1, .monotonic);
        return .init(&batch.base);
    }
};

/// A user renderer that keeps every line, triangle and text it receives and the triangles of the batches created after
/// `init` (single threaded tests)
const RecordingRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const Batch = struct {
        pub const overrides = .{};
        base: DebugRenderer.BatchTarget,
    };

    const Line = struct { from: RVec3, to: RVec3, color: Color };
    const Tri = struct { v: [3]RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow };
    const Text = struct { position: RVec3, string: []u8, color: Color, height: f32 };

    base: DebugRenderer,
    lines: std.ArrayList(Line) = .empty,
    triangles: std.ArrayList(Tri) = .empty,
    texts: std.ArrayList(Text) = .empty,
    batch_triangles: std.ArrayList(DebugRenderer.Triangle) = .empty,
    num_geometry_draws: u32 = 0,

    fn init(self: *RecordingRenderer, allocator: std.mem.Allocator) !void {
        self.* = .{ .base = .init(RecordingRenderer, allocator, std.testing.io) };
        errdefer self.base.deinit();
        try self.base.initialize();
        self.clear(); // Forget the predefined geometry of initialize
    }

    pub fn destruct(self: *RecordingRenderer) void {
        const allocator = self.base.allocator;
        self.clear();
        self.lines.deinit(allocator);
        self.triangles.deinit(allocator);
        self.texts.deinit(allocator);
        self.batch_triangles.deinit(allocator);
    }

    fn clear(self: *RecordingRenderer) void {
        for (self.texts.items) |t| self.base.allocator.free(t.string);
        self.lines.clearRetainingCapacity();
        self.triangles.clearRetainingCapacity();
        self.texts.clearRetainingCapacity();
        self.batch_triangles.clearRetainingCapacity();
        self.num_geometry_draws = 0;
    }

    pub fn drawLine(self: *RecordingRenderer, from: RVec3, to: RVec3, color: Color) void {
        self.lines.append(self.base.allocator, .{ .from = from, .to = to, .color = color }) catch @panic("out of memory");
    }
    pub fn drawTriangle(self: *RecordingRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow) void {
        self.triangles.append(self.base.allocator, .{ .v = .{ v1, v2, v3 }, .color = color, .cast_shadow = cast_shadow }) catch @panic("out of memory");
    }
    pub fn createTriangleBatch(self: *RecordingRenderer, triangles: []const DebugRenderer.Triangle) DebugRenderer.Batch {
        self.batch_triangles.appendSlice(self.base.allocator, triangles) catch @panic("out of memory");
        return self.newBatch();
    }
    pub fn createTriangleBatchIndexed(self: *RecordingRenderer, vertices: []const DebugRenderer.Vertex, indices: []const u32) DebugRenderer.Batch {
        _ = .{ vertices, indices };
        return self.newBatch();
    }
    pub fn drawGeometry(self: *RecordingRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const DebugRenderer.Geometry, cull_mode: DebugRenderer.CullMode, cast_shadow: DebugRenderer.CastShadow, draw_mode: DebugRenderer.DrawMode) void {
        _ = .{ model_matrix, world_space_bounds, lod_scale_sq, model_color, geometry, cull_mode, cast_shadow, draw_mode };
        self.num_geometry_draws += 1;
    }
    pub fn drawText3D(self: *RecordingRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        const copy = self.base.allocator.dupe(u8, string) catch @panic("out of memory");
        self.texts.append(self.base.allocator, .{ .position = position, .string = copy, .color = color, .height = height }) catch @panic("out of memory");
    }

    fn newBatch(self: *RecordingRenderer) DebugRenderer.Batch {
        const batch = self.base.allocator.create(Batch) catch @panic("out of memory");
        batch.* = .{ .base = .init(Batch, self.base.allocator) };
        return .init(&batch.base);
    }
};

/// A height field of 80 x 80 samples: the draw cuts it in 4 blocks of at most 64 x 64 samples, one batch each
fn createHeightField(allocator: std.mem.Allocator) !zolt.ShapeResult {
    var settings = zolt.HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.sample_count = 80;
    settings.block_size = 4;
    try settings.height_samples.resize(allocator, 80 * 80);
    for (settings.height_samples.items, 0..) |*h, i| h.* = @floatFromInt((i * 7) % 13);
    return settings.asShapeSettings().createShape(allocator);
}

/// A mesh of two triangles (one batch)
fn createMesh(allocator: std.mem.Allocator) !zolt.ShapeResult {
    const triangles = [_]zolt.Triangle{
        .fromFloat3(zolt.Float3.init(0, 0, 0), zolt.Float3.init(0, 0, 1), zolt.Float3.init(1, 0, 0), .{ .material_index = 0 }),
        .fromFloat3(zolt.Float3.init(1, 0, 0), zolt.Float3.init(0, 0, 1), zolt.Float3.init(1, 0, 1), .{ .material_index = 0 }),
    };
    var settings = try zolt.MeshShapeSettings.init(allocator, &triangles, .{});
    defer settings.deinit();
    return settings.asShapeSettings().createShape(allocator);
}

/// A convex hull: a cube with a tip on top
fn createConvexHull(allocator: std.mem.Allocator) !zolt.ShapeResult {
    var settings = zolt.ConvexHullShapeSettings.initDefault(allocator);
    defer settings.deinit();
    for (0..8) |i| try settings.points.append(allocator, Vec3.init(if (i & 1 != 0) 1 else -1, if (i & 2 != 0) 1 else -1, if (i & 4 != 0) 1 else -1));
    try settings.points.append(allocator, Vec3.init(0, 1.5, 0));
    return settings.asShapeSettings().createShape(allocator);
}

fn createTaperedCapsule(allocator: std.mem.Allocator) !zolt.ShapeResult {
    var settings = zolt.TaperedCapsuleShapeSettings.init(allocator, 1.0, 0.5, 0.75, .{});
    defer settings.deinit();
    return settings.asShapeSettings().createShape(allocator);
}

fn createCapsule(allocator: std.mem.Allocator) !zolt.ShapeResult {
    var settings = zolt.CapsuleShapeSettings.init(allocator, 1.0, 0.5, .{});
    defer settings.deinit();
    return settings.asShapeSettings().createShape(allocator);
}

const ShapeCreator = *const fn (std.mem.Allocator) anyerror!zolt.ShapeResult;

/// The batches of the tapered capsule's geometry: CreateTriangleGeometryForConvex makes one per LOD
const tapered_capsule_batches = 4;

test "DebugRenderer: the shapes cache their draw geometry and build it again when the drawing settings change (R6)" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest; // Shape.draw only exists with the option

    const allocator = std.testing.allocator;
    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    const r = &renderer.base;
    const transform = RMat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.3), RVec3.init(1, 2, 3));

    // Convex hull and tapered capsule: one geometry for every draw (the color is the model color)
    for ([_]ShapeCreator{ createConvexHull, createTaperedCapsule }, [_]u32{ 1, tapered_capsule_batches }) |create, num_batches| {
        var result = try create(allocator);
        defer result.deinit();
        const shape = result.getPtr().?;
        const before = renderer.batches();
        shape.draw(r, transform, Vec3.one(), Color.red, false, false);
        try expectEqual(before + num_batches, renderer.batches());
        shape.draw(r, transform, Vec3.init(2, 2, 2), Color.green, true, true);
        try expectEqual(before + num_batches, renderer.batches());
    }

    // Mesh: a new geometry when the material colors or the triangle groups change
    {
        var result = try createMesh(allocator);
        defer result.deinit();
        const mesh = result.getPtr().?;
        const before = renderer.batches();
        const draws = renderer.num_geometry_draws.load(.monotonic);
        mesh.draw(r, transform, Vec3.one(), Color.white, false, false);
        mesh.draw(r, transform, Vec3.one(), Color.white, false, true);
        try expectEqual(before + 1, renderer.batches());
        mesh.draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 2, renderer.batches());
        r.flags.mesh_shape.draw_triangle_groups = true;
        mesh.draw(r, transform, Vec3.one(), Color.white, true, false);
        mesh.draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 3, renderer.batches());
        r.flags.mesh_shape.draw_triangle_groups = false;
        try expectEqual(draws + 5, renderer.num_geometry_draws.load(.monotonic));
    }

    // Height field: 4 blocks, a new geometry when the material colors change. A clone shares the geometry until
    // setHeights clears it (setMaterials keeps it, like Jolt).
    {
        var result = try createHeightField(allocator);
        defer result.deinit();
        const height_field = result.getPtr().?;
        const before = renderer.batches();
        height_field.draw(r, transform, Vec3.one(), Color.white, false, false);
        height_field.draw(r, transform, Vec3.one(), Color.white, false, false);
        try expectEqual(before + 4, renderer.batches());
        height_field.draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 8, renderer.batches());

        const cloned = try height_field.cast(zolt.HeightFieldShape).clone(allocator);
        var clone = zolt.Ref(zolt.Shape).init(cloned.asShapeMut());
        defer clone.deinit();
        cloned.asShape().draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 8, renderer.batches());

        var temp_allocator = zolt.TempAllocatorMalloc.init(allocator);
        const material_indices = [_]u8{0};
        const material_list = [_]zolt.PhysicsMaterialRefC{.init(zolt.PhysicsMaterial.default)};
        try expect(try cloned.setMaterials(0, 0, 1, 1, &material_indices, 1, &material_list, temp_allocator.tempAllocator()));
        cloned.asShape().draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 8, renderer.batches());
        const heights: [16]f32 = @splat(2.5); // One block (the block size is 4)
        try cloned.setHeights(0, 0, 4, 4, &heights, 4, temp_allocator.tempAllocator(), .{});
        cloned.asShape().draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 12, renderer.batches());
        height_field.draw(r, transform, Vec3.one(), Color.white, true, false);
        try expectEqual(before + 12, renderer.batches());
    }
    try expect(!r.isOutOfMemory());
}

test "DebugRenderer: two threads draw the same uncached shapes (one geometry each, UB10)" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    const before = renderer.batches();

    var results: [4]zolt.ShapeResult = .{ try createConvexHull(allocator), try createTaperedCapsule(allocator), try createMesh(allocator), try createHeightField(allocator) };
    defer for (&results) |*result| result.deinit();
    var shapes: [4]*const zolt.Shape = undefined;
    for (&shapes, &results) |*s, *result| s.* = result.getPtr().?;

    const Worker = struct {
        fn run(r: *DebugRenderer, s: []const *const zolt.Shape) void {
            for (0..20) |i| {
                for (s) |shape| shape.draw(r, RMat44.identity(), Vec3.one(), Color.white, false, i % 2 == 0);
            }
        }
    };
    var threads: [2]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, Worker.run, .{ &renderer.base, @as([]const *const zolt.Shape, &shapes) });
    for (threads) |t| t.join();

    // 1 batch for the hull and the mesh, 4 for the tapered capsule (LODs) and the height field (blocks)
    try expectEqual(before + 2 + tapered_capsule_batches + 4, renderer.batches());
    try expectEqual(@as(u32, 2 * 20 * 7), renderer.num_geometry_draws.load(.monotonic));
}

test "DebugRenderer: a shape draw that runs out of memory caches nothing and builds the geometry in a later draw (R5)" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    const creators = [_]ShapeCreator{ createConvexHull, createTaperedCapsule, createMesh, createHeightField };
    const num_batches = [_]u32{ 1, tapered_capsule_batches, 1, 4 };
    const num_geometries = [_]u32{ 1, 1, 1, 4 };
    // The face outlines of the hull and the triangle outlines of the mesh and the height field need no memory
    const draws_outlines = [_]bool{ true, false, true, true };
    for (creators, num_batches, num_geometries, draws_outlines) |create, expected_batches, expected_geometries, expected_outlines| {
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            // The shape and the renderer share the failing allocator: the list of geometries of the height field (the
            // shape's allocator) fails too
            var failing = std.testing.FailingAllocator.init(allocator, .{});
            var result = try create(failing.allocator());
            defer result.deinit();
            const shape = result.getPtr().?;

            var renderer: CountingRenderer = undefined;
            try renderer.init(failing.allocator());
            defer renderer.base.deinit();
            const before = renderer.batches();
            const draws = renderer.num_geometry_draws.load(.monotonic);
            renderer.base.flags.convex_hull_shape.draw_face_outlines = true;
            renderer.base.flags.mesh_shape.draw_triangle_outlines = true;
            renderer.base.flags.height_field_shape.draw_triangle_outlines = true;

            failing.fail_index = failing.alloc_index + fail_index;
            shape.draw(&renderer.base, RMat44.identity(), Vec3.one(), Color.white, false, false);
            const lines = renderer.num_lines.load(.monotonic);
            try expectEqual(expected_outlines, lines > 0); // The outlines are drawn in every round
            if (!renderer.base.isOutOfMemory()) {
                // The success round: every allocation before it failed once
                try expectEqual(before + expected_batches, renderer.batches());
                try expectEqual(draws + expected_geometries, renderer.num_geometry_draws.load(.monotonic));
                try expect(fail_index > 0);
                break;
            }

            // Nothing was cached or drawn; with memory the next draw builds the geometry and draws it
            try expectEqual(draws, renderer.num_geometry_draws.load(.monotonic));
            failing.fail_index = std.math.maxInt(usize);
            const batches_before_retry = renderer.batches();
            shape.draw(&renderer.base, RMat44.identity(), Vec3.one(), Color.white, false, false);
            try expectEqual(batches_before_retry + expected_batches, renderer.batches());
            try expectEqual(draws + expected_geometries, renderer.num_geometry_draws.load(.monotonic));
            try expectEqual(2 * lines, renderer.num_lines.load(.monotonic));
        }
    }
}

test "DebugRenderer: DrawGetSupportFunction and DrawGetSupportingFace that run out of memory drop their drawing and draw in a later call (R5)" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    for ([_]ShapeCreator{ createConvexHull, createCapsule }) |create| {
        var result = try create(allocator);
        defer result.deinit();
        const shape = result.getPtr().?;

        // DrawGetSupportFunction: without the geometry it only draws the support points and directions (no memory)
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var failing = std.testing.FailingAllocator.init(allocator, .{});
            var renderer: CountingRenderer = undefined;
            try renderer.init(failing.allocator());
            defer renderer.base.deinit();

            failing.fail_index = failing.alloc_index + fail_index;
            shape.drawGetSupportFunction(&renderer.base, RMat44.identity(), Vec3.one(), Color.white, true);
            const lines = renderer.num_lines.load(.monotonic);
            try expect(lines > 0);
            if (!renderer.base.isOutOfMemory()) {
                try expectEqual(@as(u32, 1), renderer.num_geometry_draws.load(.monotonic));
                try expect(fail_index > 0);
                break;
            }
            try expectEqual(@as(u32, 0), renderer.num_geometry_draws.load(.monotonic));
            failing.fail_index = std.math.maxInt(usize);
            shape.drawGetSupportFunction(&renderer.base, RMat44.identity(), Vec3.one(), Color.white, true);
            try expectEqual(@as(u32, 1), renderer.num_geometry_draws.load(.monotonic));
            try expectEqual(2 * lines, renderer.num_lines.load(.monotonic));
        }

        // DrawGetSupportingFace: when the map of faces cannot grow nothing is drawn
        fail_index = 0;
        while (true) : (fail_index += 1) {
            var failing = std.testing.FailingAllocator.init(allocator, .{});
            var renderer: CountingRenderer = undefined;
            try renderer.init(failing.allocator());
            defer renderer.base.deinit();

            failing.fail_index = failing.alloc_index + fail_index;
            shape.drawGetSupportingFace(&renderer.base, RMat44.identity(), Vec3.one());
            if (!renderer.base.isOutOfMemory()) {
                try expect(renderer.num_lines.load(.monotonic) > 0);
                try expect(fail_index > 0);
                break;
            }
            try expectEqual(@as(u32, 0), renderer.num_lines.load(.monotonic));
            failing.fail_index = std.math.maxInt(usize);
            shape.drawGetSupportingFace(&renderer.base, RMat44.identity(), Vec3.one());
            try expect(renderer.num_lines.load(.monotonic) > 0);
        }
    }
}

test "DebugRenderer: a clone of a drawn height field shares its geometry, every allocation of the clone can fail (R5, R6)" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var result = try createHeightField(allocator);
    defer result.deinit();
    const height_field = result.getPtr().?.cast(zolt.HeightFieldShape);

    // The number of allocations of a clone of a height field that was never drawn
    const undrawn_allocations = blk: {
        var counting = std.testing.FailingAllocator.init(allocator, .{});
        var clone = zolt.Ref(zolt.Shape).init((try height_field.clone(counting.allocator())).asShapeMut());
        clone.deinit();
        break :blk counting.alloc_index;
    };

    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    const r = &renderer.base;
    height_field.asShape().draw(r, RMat44.identity(), Vec3.one(), Color.white, false, false);
    const batches = renderer.batches();

    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const cloned = height_field.clone(failing.allocator()) catch |err| {
            try expectEqual(error.OutOfMemory, err);
            continue;
        };
        var clone = zolt.Ref(zolt.Shape).init(cloned.asShapeMut());
        defer clone.deinit();

        // The success round: one allocation more than for the height field that was never drawn (the list of the
        // shared geometries). The clone draws the shared geometry, it creates no batch.
        try expectEqual(undrawn_allocations + 1, fail_index);
        cloned.asShape().draw(r, RMat44.identity(), Vec3.one(), Color.white, false, false);
        try expectEqual(batches, renderer.batches());
        break;
    }
}

test "DebugRenderer: a null entry of the mesh's material list draws in the default material's color (Zolt, Jolt dereferences null)" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    const material = try zolt.PhysicsMaterialSimple.create(allocator, "Red", Color.red);
    var material_ref = zolt.RefConst(zolt.PhysicsMaterial).init(material.material());
    defer material_ref.deinit();

    // 2 triangles with material 0 (red) and 2 with material 1 (a null entry)
    const F3 = zolt.Float3;
    const triangles = [_]zolt.Triangle{
        .fromFloat3(F3.init(0, 0, 0), F3.init(0, 0, 1), F3.init(1, 0, 0), .{ .material_index = 0 }),
        .fromFloat3(F3.init(1, 0, 0), F3.init(0, 0, 1), F3.init(1, 0, 1), .{ .material_index = 1 }),
        .fromFloat3(F3.init(1, 0, 0), F3.init(1, 0, 1), F3.init(2, 0, 0), .{ .material_index = 0 }),
        .fromFloat3(F3.init(2, 0, 0), F3.init(1, 0, 1), F3.init(2, 0, 1), .{ .material_index = 1 }),
    };
    var settings = try zolt.MeshShapeSettings.init(allocator, &triangles, .{});
    defer settings.deinit();
    try settings.materials.append(allocator, .init(material.material()));
    try settings.materials.append(allocator, .init(null));
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const mesh = result.getPtr().?;
    try expect(mesh.cast(zolt.MeshShape).getMaterialList()[1].get() == null);

    var renderer: RecordingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();

    // Material colors, no triangle groups: one color per triangle
    mesh.draw(&renderer.base, RMat44.identity(), Vec3.one(), Color.white, true, false);
    try expectEqual(@as(usize, 4), renderer.batch_triangles.items.len);
    var num_red: u32 = 0;
    var num_default: u32 = 0;
    for (renderer.batch_triangles.items) |t| {
        for (t.v) |v| try expect(v.color.eql(t.v[0].color));
        if (t.v[0].color.eql(Color.red)) {
            num_red += 1;
        } else if (t.v[0].color.eql(zolt.PhysicsMaterial.default.getDebugColor())) {
            num_default += 1;
        }
    }
    try expectEqual(@as(u32, 2), num_red);
    try expectEqual(@as(u32, 2), num_default);
    try expectEqual(@as(u32, 1), renderer.num_geometry_draws);
}

test "DebugRenderer: Body.applyBuoyancyImpulse draws the submerged volume, the center of buoyancy and the volumes" {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Far from the origin and rotated: the base offset of the submerged volume draw is the body's position
    const position = RVec3.init(100.5, 3, -7);
    const rotation = Quat.rotation(Vec3.init(1, 2, 0.5).normalized(), 0.6);
    const body = try c.createBox(position, rotation, .dynamic, .discrete, Layers.moving, Vec3.one(), .{});
    const surface_position = position.addVec3(Vec3.init(0.1, 0.2, -0.1));
    const surface_normal = Vec3.init(0.1, 1, 0.2).normalized();
    const gravity = Vec3.init(0, -10, 0);

    var renderer: RecordingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    const r = &renderer.base;

    // Without Shape::sDrawSubmergedVolumes (or without a renderer) nothing is drawn
    try expect(body.applyBuoyancyImpulse(surface_position, surface_normal, 1.5, 0.5, 0.01, Vec3.zero(), gravity, 1.0 / 60.0, .{ .debug_renderer = r }));
    r.flags.shape.draw_submerged_volumes = true;
    try expect(body.applyBuoyancyImpulse(surface_position, surface_normal, 1.5, 0.5, 0.01, Vec3.zero(), gravity, 1.0 / 60.0, .{}));
    try expectEqual(@as(usize, 0), renderer.lines.items.len + renderer.triangles.items.len + renderer.texts.items.len);

    // What the shape draws for its submerged part at the body's center of mass (the impulses do not move the body)
    var expected: RecordingRenderer = undefined;
    try expected.init(allocator);
    defer expected.base.deinit();
    expected.base.flags.shape.draw_submerged_volumes = true;
    const com = body.getCenterOfMassPosition();
    const surface = zolt.Plane.fromPointAndNormalRVec3(surface_position.sub(com), surface_normal);
    const volume = body.getShape().getSubmergedVolume(zolt.Mat44.rotationQuat(body.getRotation()), Vec3.one(), surface, .{ .debug = .{ .base_offset = com, .renderer = &expected.base } });
    try expect(volume.submerged_volume > 0.0 and volume.submerged_volume < volume.total_volume);
    try expect(expected.triangles.items.len > 0);

    try expect(body.applyBuoyancyImpulse(surface_position, surface_normal, 1.5, 0.5, 0.01, Vec3.zero(), gravity, 1.0 / 60.0, .{ .debug_renderer = r }));

    // The submerged triangles and their outlines, at the body (far from the origin)
    try expectEqual(expected.triangles.items.len, renderer.triangles.items.len);
    for (expected.triangles.items, renderer.triangles.items) |e, a| {
        for (e.v, a.v) |ev, av| {
            try expect(ev.eql(av));
            try expect(av.sub(com).toVec3().length() < 2.0);
        }
        try expect(e.color.eql(a.color));
        try expectEqual(e.cast_shadow, a.cast_shadow);
    }
    const num_outlines = expected.lines.items.len;
    try expectEqual(num_outlines + 3, renderer.lines.items.len);
    for (expected.lines.items, renderer.lines.items[0..num_outlines]) |e, a|
        try expect(e.from.eql(a.from) and e.to.eql(a.to) and e.color.eql(a.color));

    // Then a white marker of size 2 at the center of buoyancy (Jolt's DrawMarker: the y, x and z lines)
    const center_of_buoyancy = com.addVec3(volume.center_of_buoyancy);
    const axes = [_]Vec3{ Vec3.init(0, 2, 0), Vec3.init(2, 0, 0), Vec3.init(0, 0, 2) };
    for (axes, renderer.lines.items[num_outlines..]) |axis, line| {
        try expect(line.from.eql(center_of_buoyancy.subVec3(axis)));
        try expect(line.to.eql(center_of_buoyancy.addVec3(axis)));
        try expect(line.color.eql(Color.white));
    }

    // And "submerged / total" (%.3f) in the default color and height at the center of buoyancy
    try expectEqual(@as(usize, 1), renderer.texts.items.len);
    const text = renderer.texts.items[0];
    var buffer: [64]u8 = undefined;
    const expected_text = try std.fmt.bufPrint(&buffer, "{f} / {f}", .{ zolt.cppFixed(volume.submerged_volume, 3), zolt.cppFixed(volume.total_volume, 3) });
    try std.testing.expectEqualStrings(expected_text, text.string);
    try expect(std.mem.endsWith(u8, text.string, " / 8.000"));
    try expect(text.position.eql(center_of_buoyancy));
    try expect(text.color.eql(Color.white));
    try expectEqual(@as(f32, 0.5), text.height);
}
