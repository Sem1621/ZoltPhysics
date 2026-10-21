//! Parity tests of the shape draws (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, "Draw call log" (shapes), group
//! D2). C ABI wrapper: `jolt_dr_run_shape_ops` in ZoltParity/Renderer/DebugRendererReference.cpp; the shape ops (64 to
//! 69) are in ParityRenderer.zig and ParityRenderer.h.
//!
//! Both sides build the catalogue of the pairwise parity (`PairwiseParity.makeCatalogue`: every shape kind, decorated,
//! scaled, rotated, compounds, meshes and height fields, with materials). The op stream then draws every catalogue shape
//! over several frames:
//! - Shape::Draw with every combination of use material colors and draw wireframe, with random rotations, positions (far
//!   from the origin too) and the scales that IsValidScale accepts (inside out too);
//! - the static draw flags that the shapes read (Shape::sDrawSubmergedVolumes, ConvexHullShape::sDrawFaceOutlines,
//!   MeshShape::sDrawTriangleGroups and sDrawTriangleOutlines, HeightFieldShape::sDrawTriangleOutlines): none in the
//!   first frame, all in the second, random sets later and between random ops;
//! - DrawGetSupportFunction (with and without the support direction), DrawGetSupportingFace, ConvexHullShape's
//!   DrawShrunkShape;
//! - GetSubmergedVolume with a base offset, which draws the submerged volume (PolyhedronSubmergedVolumeCalculator, the
//!   center of buoyancy of compounds) when Shape::sDrawSubmergedVolumes is set. Its results are compared too. Meshes,
//!   height fields and planes are left out: Jolt asserts "Not supported".
//! The frames reuse the cached geometry of ConvexHullShape, TaperedCapsuleShape, MeshShape and HeightFieldShape (R6), and
//! switching the material colors or the triangle groups builds it again.
//!
//! The layers (they exist only with -Ddebug_renderer=true, without the option they skip):
//! - call log: the ops on a ParityRenderer of each side, the logs must match call for call (batch contents included);
//! - DebugRendererSimple: the same on a DebugRendererSimple of each side that logs its lines;
//! - recorder: a few shape frames on Jolt's and Zolt's DebugRendererRecorder, the .jor bytes must match.
//! Each layer builds its own catalogue: the cached geometry of a shape belongs to the renderer that created it.

const std = @import("std");
const zolt = @import("zolt");
const pr = @import("ParityRenderer.zig");
const pw = @import("../Physics/PairwiseParity.zig");

const Color = zolt.Color;
const DebugRendererRecorder = zolt.DebugRendererRecorder;
const Quat = zolt.Quat;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeSubType = zolt.ShapeSubType;
const Vec3 = zolt.Vec3;
const JoltResult = pr.JoltResult;

const allocator = std.testing.allocator;

const jolt = struct {
    extern fn jolt_dr_available() c_int;
    extern fn jolt_dr_run_shape_ops(catalogue: *anyopaque, num_shapes: u32, ops: [*]const u8, size: u64, kind: c_int) ?*anyopaque;
};

/// Skip without the option (the C++ reference has no DebugRenderer then)
fn requireDebugRenderer() !void {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;
    try std.testing.expectEqual(@as(c_int, 1), jolt.jolt_dr_available()); // Both sides follow -Ddebug_renderer
}

/// The renderer that runs the ops (the `kind` of jolt_dr_run_shape_ops)
const Kind = enum(c_int) { parity_renderer = 0, simple = 1, recorder = 2 };

/// The catalogue on both sides and the Zolt shapes for the ShapeContext
const Catalogue = struct {
    buffer: []u32,
    built: *pw.Built,
    shapes: []*const Shape,

    fn init() !Catalogue {
        const buffer = try allocator.alloc(u32, pw.stream_capacity);
        errdefer allocator.free(buffer);
        const built = try pw.Built.init(allocator, try pw.makeCatalogue(allocator), buffer);
        errdefer built.deinit();
        const shapes = try allocator.alloc(*const Shape, built.shapes.items.len);
        for (shapes, 0..) |*s, i| s.* = built.shape(@intCast(i));
        return .{ .buffer = buffer, .built = built, .shapes = shapes };
    }

    fn deinit(self: *Catalogue) void {
        allocator.free(self.shapes);
        self.built.deinit();
        allocator.free(self.buffer);
    }

    fn numShapes(self: *const Catalogue) u32 {
        return @intCast(self.shapes.len);
    }

    /// GetSubmergedVolume is supported: no mesh, height field or plane in the shape (Jolt asserts "Not supported")
    fn submergedVolumeSupported(self: *const Catalogue, node: u32) bool {
        const unsupported = [_]ShapeSubType{ .mesh, .height_field, .plane };
        for (unsupported) |t| {
            if (self.built.leaf_types.items[node] & (@as(u64, 1) << @intCast(@intFromEnum(t))) != 0) return false;
        }
        return true;
    }
};

/// Options of the op generator
const GenOptions = struct {
    /// Number of frames
    frames: u32 = 4,
    /// Random ops per frame after the systematic ones
    random_ops: u32 = 150,
    /// Every combination of the draw arguments for every shape in every frame (false: one random combination)
    all_combinations: bool = true,
    /// EndFrame at the end of each frame (the recorder)
    end_frames: bool = false,
};

/// Writes the shape ops
const Generator = struct {
    gen: pw.Gen,
    cat: *const Catalogue,
    w: *pr.OpWriter,

    fn color(self: *Generator) Color {
        return Color.fromUInt32(self.gen.rng.next());
    }

    fn real(self: *Generator, min: f32, max: f32) Real {
        return self.gen.float(min, max);
    }

    /// A position, now and then far away and fractional (where double precision differs)
    fn rvec3(self: *Generator) RVec3 {
        if (self.gen.chance(15))
            return RVec3.init(@as(Real, self.gen.float(-1.0e5, 1.0e5)) + @as(Real, self.gen.float(0, 1)) / 7, self.real(-1.0e3, 1.0e3), @as(Real, self.gen.float(-1.0e5, 1.0e5)));
        return RVec3.init(self.real(-20, 20), self.real(-20, 20), self.real(-20, 20));
    }

    /// A center of mass transform: a rotation and a position
    fn transform(self: *Generator) pr.Affine {
        const q = self.gen.rotation();
        return .{ .columns = .{ q.rotateAxisX(), q.rotateAxisY(), q.rotateAxisZ() }, .translation = self.rvec3() };
    }

    fn scale(self: *Generator, node: u32) Vec3 {
        return pw.randomScale(&self.gen, self.cat.shapes[node]);
    }

    fn randomNode(self: *Generator) u32 {
        return self.gen.index(self.cat.numShapes());
    }

    fn draw(self: *Generator, n: u32, use_material_colors: bool, draw_wireframe: bool) !void {
        try self.w.drawShape(n, self.transform(), self.scale(n), self.color(), use_material_colors, draw_wireframe);
    }

    fn supportFunction(self: *Generator, n: u32) !void {
        try self.w.drawGetSupportFunction(n, self.transform(), self.scale(n), self.color(), self.gen.chance(50));
    }

    fn supportingFace(self: *Generator, n: u32) !void {
        try self.w.drawGetSupportingFace(n, self.transform(), self.scale(n));
    }

    fn isHull(self: *Generator, n: u32) bool {
        return self.cat.shapes[n].getSubType() == .convex_hull;
    }

    /// GetSubmergedVolume with a surface through the shape (or a random one)
    fn submergedVolume(self: *Generator, n: u32) !void {
        const q = self.gen.rotation();
        const translation = self.gen.vec(-10, 10);
        const normal = self.gen.direction();
        const point = if (self.gen.chance(80)) translation.add(self.gen.vec(-1, 1)) else self.gen.vec(-15, 15);
        try self.w.getSubmergedVolume(n, .{ q.rotateAxisX(), q.rotateAxisY(), q.rotateAxisZ() }, translation, self.scale(n), normal, -normal.dot(point), self.rvec3());
    }

    fn randomFlags(self: *Generator) u8 {
        return @intCast(self.gen.rng.next() & pr.ShapeDrawFlags.all);
    }

    /// One random op
    fn randomOp(self: *Generator) !void {
        const n = self.randomNode();
        switch (self.gen.index(10)) {
            0...3 => try self.draw(n, self.gen.chance(50), self.gen.chance(50)),
            4 => try self.supportFunction(n),
            5 => try self.supportingFace(n),
            6 => if (self.isHull(n)) try self.w.drawShrunkShape(n, self.transform(), self.scale(n)) else try self.draw(n, true, false),
            7 => if (self.cat.submergedVolumeSupported(n)) try self.submergedVolume(n) else try self.draw(n, false, true),
            8 => try self.w.setShapeDrawFlags(self.randomFlags()),
            9 => if (self.gen.chance(30)) try self.w.nextFrame() else try self.draw(n, false, false),
            else => unreachable,
        }
    }

    /// The frames: every shape with every combination of the draw arguments, then random ops
    fn frames(self: *Generator, options: GenOptions) !void {
        for (0..options.frames) |frame| {
            const flags: u8 = switch (frame) {
                0 => 0,
                1 => pr.ShapeDrawFlags.all,
                else => self.randomFlags(),
            };
            try self.w.setShapeDrawFlags(flags);
            for (0..self.cat.numShapes()) |i| {
                const n: u32 = @intCast(i);
                if (options.all_combinations) {
                    for ([_]bool{ false, true }) |use_material_colors| {
                        for ([_]bool{ false, true }) |draw_wireframe| try self.draw(n, use_material_colors, draw_wireframe);
                    }
                } else {
                    try self.draw(n, self.gen.chance(50), self.gen.chance(50));
                }
                try self.supportFunction(n);
                try self.supportingFace(n);
                if (self.isHull(n)) try self.w.drawShrunkShape(n, self.transform(), self.scale(n));
                if (self.cat.submergedVolumeSupported(n)) {
                    try self.submergedVolume(n);
                    try self.submergedVolume(n);
                }
            }
            for (0..options.random_ops) |_| try self.randomOp();
            if (options.end_frames) try self.w.endFrame();
            try self.w.nextFrame();
        }
    }
};

/// The op stream of a layer
fn makeOps(cat: *const Catalogue, w: *pr.OpWriter, seed: u32, options: GenOptions) !void {
    var g: Generator = .{ .gen = .{ .rng = .{ .state = seed } }, .cat = cat, .w = w };
    try g.frames(options);
}

/// Run the ops on Jolt's side
fn runJolt(cat: *const Catalogue, ops: []const u8, kind: Kind) JoltResult {
    return .{ .handle = jolt.jolt_dr_run_shape_ops(cat.built.handle, cat.numShapes(), ops.ptr, ops.len, @intFromEnum(kind)).? };
}

/// Compare the results of the shape ops (5 f32 per GetSubmergedVolume)
fn compareResults(name: []const u8, zolt_results: []const u8, jolt_results: []const u8) !void {
    if (std.mem.eql(u8, zolt_results, jolt_results))
        return;
    if (zolt_results.len != jolt_results.len) {
        std.debug.print("{s}: {d} result bytes (zolt), {d} (jolt)\n", .{ name, zolt_results.len, jolt_results.len });
        return error.ParityMismatch;
    }
    const index = std.mem.indexOfDiff(u8, zolt_results, jolt_results).? / 4;
    const z = std.mem.bytesAsValue(f32, zolt_results[4 * index ..][0..4]).*;
    const j = std.mem.bytesAsValue(f32, jolt_results[4 * index ..][0..4]).*;
    std.debug.print("{s}: GetSubmergedVolume {d}, value {d} differs: zolt {e} jolt {e}\n", .{ name, index / 5, index % 5, z, j });
    return error.ParityMismatch;
}

/// Compare two .jor byte streams, print the first difference
fn compareBytes(name: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes))
        return;
    const index = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
    std.debug.print("{s}: the .jor bytes differ at byte {d} of {d} (zolt) / {d} (jolt)\n  zolt: {x}\n  jolt: {x}\n", .{ name, index, zolt_bytes.len, jolt_bytes.len, zolt_bytes[index..@min(index + 64, zolt_bytes.len)], jolt_bytes[index..@min(index + 64, jolt_bytes.len)] });
    return error.ParityMismatch;
}

/// Run one layer: the ops on Zolt's and Jolt's renderer of `kind`, compare the logs (.jor bytes) and the results
fn runLayer(name: []const u8, seed: u32, kind: Kind, options: GenOptions) !void {
    var cat = try Catalogue.init();
    defer cat.deinit();
    var w: pr.OpWriter = .{ .allocator = allocator };
    defer w.deinit();
    try makeOps(&cat, &w, seed, options);

    const result = runJolt(&cat, w.bytes.items, kind);
    defer result.deinit();

    var context: pr.ShapeContext = .{ .allocator = allocator, .shapes = cat.shapes };
    defer context.deinit();
    switch (kind) {
        .parity_renderer => {
            var renderer: pr.ParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOpsWithShapes(allocator, &renderer.base, w.bytes.items, null, null, &context);
            try pr.compareLogs(name, renderer.log.bytes.items, result.part(0));
        },
        .simple => {
            var renderer: pr.SimpleParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOpsWithShapes(allocator, renderer.base.debugRenderer(), w.bytes.items, &renderer.base, null, &context);
            try pr.compareLogs(name, renderer.log.bytes.items, result.part(0));
        },
        .recorder => {
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var wrapper: zolt.StreamOutWrapper = .init(&out.writer);
            var recorder: DebugRendererRecorder = undefined;
            try recorder.init(allocator, std.testing.io, wrapper.streamOut());
            defer recorder.deinit();
            const construction_len = out.written().len;
            try pr.runOpsWithShapes(allocator, recorder.debugRenderer(), w.bytes.items, null, &recorder, &context);
            try compareBytes(name, out.written()[construction_len..], result.part(0));
        },
    }
    try compareResults(name, context.results.items, result.part(1));
}

test "Shape draw parity: every catalogue shape, every argument and flag, several frames (call log)" {
    try requireDebugRenderer();
    try runLayer("shape draws", 0xd2000001, .parity_renderer, .{});
}

test "Shape draw parity: DebugRendererSimple (call log)" {
    try requireDebugRenderer();
    try runLayer("shape draws: DebugRendererSimple", 0xd2000002, .simple, .{ .frames = 3, .all_combinations = false });
}

test "Shape draw parity: recorder (.jor bytes of shape frames)" {
    try requireDebugRenderer();
    try runLayer("shape draws: recorder", 0xd2000003, .recorder, .{ .frames = 3, .random_ops = 60, .end_frames = true });
}
