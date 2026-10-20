//! Parity tests of the debug renderer (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, "Draw call log", group
//! D0). C ABI wrappers: ZoltParity/Renderer/DebugRendererReference.cpp; the log and the op stream:
//! ZoltParity/Renderer/ParityRenderer.zig and ParityRenderer.h.
//!
//! The layers (they exist only with -Ddebug_renderer=true, which gives the C++ reference JPH_DEBUG_RENDERER; without the
//! option they skip):
//! - helpers: random calls of the virtual functions and of the helpers that D0 found ported (DrawWireBox x2,
//!   DrawMarker, DrawArrow, DrawBox x2, both DrawGeometry) on a ParityRenderer of each side; the call logs must match;
//! - DebugRendererSimple: the same calls and SetCameraPos on a DebugRendererSimple of each side that logs its lines: the
//!   triangle lists, the LOD selection and the transforms of Simple's DrawGeometry;
//! - recorder: the same calls on Jolt's and Zolt's DebugRendererRecorder; the .jor bytes must match;
//! - playback: Jolt's .jor stream played back by Jolt's DebugRendererPlayback and by Zolt's into a ParityRenderer; the
//!   call logs must match.
//! The construction differs until group D1: Zolt's initialize creates only the box, the first of Jolt's 25 predefined
//! batches. So Zolt's construction log (and the recorder's construction bytes) must be the start of Jolt's, and the
//! harness continues Zolt's batch ids where Jolt's construction left them.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const pr = @import("ParityRenderer.zig");

const Allocator = std.mem.Allocator;
const Rng = fw.Rng;
const AABox = zolt.AABox;
const Color = zolt.Color;
const DebugRenderer = zolt.DebugRenderer;
const DebugRendererPlayback = zolt.DebugRendererPlayback;
const DebugRendererRecorder = zolt.DebugRendererRecorder;
const Float2 = zolt.Float2;
const Float3 = zolt.Float3;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const Vec3 = zolt.Vec3;
const Triangle = DebugRenderer.Triangle;
const Vertex = DebugRenderer.Vertex;
const JoltResult = pr.JoltResult;

const allocator = std.testing.allocator;

const jolt = struct {
    extern fn jolt_dr_available() c_int;
    extern fn jolt_dr_run_ops(ops: [*]const u8, size: u64, kind: c_int) ?*anyopaque;
    extern fn jolt_dr_record_ops(ops: [*]const u8, size: u64) ?*anyopaque;
    extern fn jolt_dr_playback(stream: [*]const u8, size: u64) ?*anyopaque;
};

/// Skip without the option (the C++ reference has no DebugRenderer then)
fn requireDebugRenderer() !void {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;
    try std.testing.expectEqual(@as(c_int, 1), jolt.jolt_dr_available()); // Both sides follow -Ddebug_renderer
}

/// Options of the op generator
const GenOptions = struct {
    /// Batches without triangles. Jolt's recorder writes ID 0 for them, which Jolt's playback cannot read (UB7).
    allow_empty_batches: bool = true,
    /// SetCameraPos (DebugRendererSimple)
    camera: bool = false,
    /// EndFrame now and then and at the end (the recorder)
    frames: bool = false,
};

/// Random calls for the op stream
const Generator = struct {
    rng: Rng,
    arena: std.heap.ArenaAllocator,
    options: GenOptions,
    /// Geometry slots that hold a geometry
    filled_slots: u16 = 0,

    fn init(seed: u32, options: GenOptions) Generator {
        return .{ .rng = .{ .state = seed }, .arena = .init(allocator), .options = options };
    }

    fn deinit(self: *Generator) void {
        self.arena.deinit();
    }

    fn chance(self: *Generator, percent: u32) bool {
        return self.rng.next() % 100 < percent;
    }

    fn real(self: *Generator, min: f32, max: f32) Real {
        return self.rng.float(min, max);
    }

    fn rvec3(self: *Generator) RVec3 {
        if (self.chance(10)) {
            // Far away and fractional, where double precision differs from single precision
            return RVec3.init(@as(Real, self.rng.float(-1.0e5, 1.0e5)) + @as(Real, self.rng.float(0, 1)) / 7, self.real(-1.0e3, 1.0e3), @as(Real, self.rng.float(-1.0e5, 1.0e5)));
        }
        return RVec3.init(self.real(-100, 100), self.real(-100, 100), self.real(-100, 100));
    }

    fn vec3(self: *Generator, min: f32, max: f32) Vec3 {
        return Vec3.init(self.rng.float(min, max), self.rng.float(min, max), self.rng.float(min, max));
    }

    fn color(self: *Generator) Color {
        return Color.fromUInt32(self.rng.next());
    }

    fn aabox(self: *Generator) AABox {
        const min = self.vec3(-50, 50);
        var extent = self.vec3(0, 10);
        if (self.chance(10)) extent = Vec3.init(0, extent.getY(), 0); // Flat in two axes
        if (self.chance(10)) extent = extent.negate(); // Inverted
        return AABox.init(min, min.add(extent));
    }

    fn affine(self: *Generator) pr.Affine {
        var m: pr.Affine = .{ .columns = .{ self.vec3(-2, 2), self.vec3(-2, 2), self.vec3(-2, 2) }, .translation = self.rvec3() };
        if (self.chance(15)) {
            // A rotation times a scale
            const rotation = zolt.Quat.rotation(self.vec3(-1, 1).normalizedOr(Vec3.axisY()), self.rng.float(-3, 3));
            const scale = self.rng.float(0.1, 3);
            m.columns = .{ rotation.rotateAxisX().mulScalar(scale), rotation.rotateAxisY().mulScalar(scale), rotation.rotateAxisZ().mulScalar(scale) };
        }
        return m;
    }

    /// A cast shadow / cull / draw mode value, now and then one that Jolt's enum does not name (passed on as int)
    fn mode(self: *Generator, num_values: u32) i32 {
        if (self.chance(3)) return @intCast(num_values + self.rng.next() % 5);
        return @intCast(self.rng.next() % num_values);
    }

    fn vertex(self: *Generator) Vertex {
        var v: Vertex = undefined;
        self.vec3(-2, 2).storeFloat3(&v.position);
        self.vec3(-1, 1).storeFloat3(&v.normal);
        v.uv = Float2.init(self.rng.float(-2, 2), self.rng.float(-2, 2));
        v.color = self.color();
        return v;
    }

    fn batch(self: *Generator) !pr.BatchData {
        const a = self.arena.allocator();
        const min_count: u32 = if (self.options.allow_empty_batches and self.chance(10)) 0 else 1;
        if (self.chance(50)) {
            const triangles = try a.alloc(Triangle, min_count + self.rng.next() % (7 - min_count));
            for (triangles) |*t| {
                if (self.chance(50)) {
                    t.* = .init(self.vec3(-2, 2), self.vec3(-2, 2), self.vec3(-2, 2), self.color());
                } else {
                    for (&t.v) |*v| v.* = self.vertex();
                }
            }
            return .{ .triangles = triangles };
        } else {
            const vertices = try a.alloc(Vertex, min_count + self.rng.next() % (9 - min_count));
            for (vertices) |*v| v.* = self.vertex();
            var num_indices = if (vertices.len == 0) 0 else 3 * (min_count + self.rng.next() % 6);
            if (num_indices > 0 and self.chance(10)) num_indices += 1 + self.rng.next() % 2; // Not a multiple of 3
            const indices = try a.alloc(u32, num_indices);
            for (indices) |*i| i.* = self.rng.next() % @as(u32, @intCast(vertices.len));
            return .{ .indexed = .{ .vertices = vertices, .indices = indices } };
        }
    }

    fn text(self: *Generator) ![]const u8 {
        const string = try self.arena.allocator().alloc(u8, self.rng.next() % 25);
        for (string) |*c| c.* = @intCast(32 + self.rng.next() % 95);
        return string;
    }

    fn filledSlot(self: *Generator) ?u32 {
        if (self.filled_slots == 0) return null;
        while (true) {
            const slot = self.rng.next() % pr.num_geometry_slots;
            if (self.filled_slots & (@as(u16, 1) << @intCast(slot)) != 0) return slot;
        }
    }

    fn createGeometry(self: *Generator, w: *pr.OpWriter) !void {
        const slot = self.rng.next() % pr.num_geometry_slots;
        const lods = try self.arena.allocator().alloc(pr.LODData, 1 + self.rng.next() % 4);
        var distance: f32 = 0;
        for (lods, 0..) |*lod, i| {
            distance += self.rng.float(0.5, 20);
            lod.* = .{ .distance = if (i == lods.len - 1 and self.chance(50)) zolt.math.large_float else distance, .batch = try self.batch() };
        }
        try w.createGeometry(slot, self.aabox(), lods);
        self.filled_slots |= @as(u16, 1) << @intCast(slot);
    }

    /// One random op
    fn op(self: *Generator, w: *pr.OpWriter) !void {
        switch (self.rng.next() % 15) {
            0 => try w.drawLine(self.rvec3(), self.rvec3(), self.color()),
            1 => try w.drawTriangle(self.rvec3(), self.rvec3(), self.rvec3(), self.color(), self.mode(2)),
            2 => try w.drawText3D(self.rvec3(), try self.text(), self.color(), self.rng.float(0, 2)),
            3 => try w.drawWireBox(self.aabox(), self.color()),
            4 => try w.drawWireBoxMatrix(self.affine(), self.aabox(), self.color()),
            5 => try w.drawMarker(self.rvec3(), self.color(), if (self.chance(10)) 0 else self.rng.float(-1, 3)),
            6 => {
                const from = self.rvec3();
                const to = if (self.chance(10)) from else self.rvec3();
                try w.drawArrow(from, to, self.color(), if (self.chance(15)) 0 else self.rng.float(-1, 2));
            },
            7 => try w.drawBox(self.aabox(), self.color(), self.mode(2), self.mode(2)),
            8 => try w.drawBoxMatrix(self.affine(), self.aabox(), self.color(), self.mode(2), self.mode(2)),
            9 => try self.createGeometry(w),
            10, 11 => {
                const slot = self.filledSlot() orelse return self.createGeometry(w);
                try w.drawGeometry(self.affine(), self.aabox(), if (self.chance(10)) 0 else self.rng.float(0, 10), self.color(), slot, self.mode(3), self.mode(2), self.mode(2));
            },
            12 => {
                const slot = self.filledSlot() orelse return self.createGeometry(w);
                try w.drawGeometryAutoBounds(self.affine(), self.color(), slot, self.mode(3), self.mode(2), self.mode(2));
            },
            13 => try w.createBatch(try self.batch()),
            14 => if (self.options.camera) try w.setCameraPos(self.rvec3()) else try w.drawLine(self.rvec3(), self.rvec3(), self.color()),
            else => unreachable,
        }
        if (self.options.frames and self.chance(3))
            try w.endFrame();
    }

    /// `num_ops` random ops (and a final EndFrame with `frames`)
    fn ops(self: *Generator, w: *pr.OpWriter, num_ops: usize) !void {
        for (0..num_ops) |_| try self.op(w);
        if (self.options.frames) try w.endFrame();
    }
};

/// The next batch id of a recorder after the commands in `bytes` (Initialize writes only batches)
fn nextRecorderBatchID(bytes: []const u8) u32 {
    var pos: usize = 0;
    var max_id: u32 = 0;
    while (pos < bytes.len) {
        const command = bytes[pos];
        max_id = @max(max_id, std.mem.readInt(u32, bytes[pos + 1 ..][0..4], .little));
        const count: usize = std.mem.readInt(u32, bytes[pos + 5 ..][0..4], .little);
        switch (command) {
            @intFromEnum(DebugRendererRecorder.Command.create_batch) => pos += 9 + @sizeOf(Triangle) * count,
            @intFromEnum(DebugRendererRecorder.Command.create_batch_indexed) => {
                const num_indices: usize = std.mem.readInt(u32, bytes[pos + 9 + @sizeOf(Vertex) * count ..][0..4], .little);
                pos += 9 + @sizeOf(Vertex) * count + 4 + 4 * num_indices;
            },
            else => std.debug.panic("unexpected command {d} in the construction of a recorder", .{command}),
        }
    }
    return max_id + 1;
}

/// Zolt's construction log must be the start of Jolt's (Zolt's initialize creates only the box until group D1)
fn checkConstruction(name: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (!pr.isPrefix(zolt_bytes, jolt_bytes)) {
        try pr.compareLogs(name, zolt_bytes, jolt_bytes[0..@min(zolt_bytes.len, jolt_bytes.len)]);
        return error.ParityMismatch;
    }
    try std.testing.expect(zolt_bytes.len > 0);
}

test "DebugRenderer parity: helpers and virtual functions (call log)" {
    try requireDebugRenderer();

    for (0..40) |round| {
        var gen = Generator.init(0x1000 + @as(u32, @intCast(round)), .{});
        defer gen.deinit();
        var w: pr.OpWriter = .{ .allocator = allocator };
        defer w.deinit();
        try gen.ops(&w, 400);

        const result = JoltResult{ .handle = jolt.jolt_dr_run_ops(w.bytes.items.ptr, w.bytes.items.len, 0).? };
        defer result.deinit();

        var renderer: pr.ParityRenderer = undefined;
        try renderer.init(allocator);
        defer renderer.deinit();
        try checkConstruction("helpers: construction", renderer.log.bytes.items, result.part(0));
        renderer.next_batch_id = result.partU32(2);
        renderer.log.bytes.clearRetainingCapacity();

        try pr.runOps(allocator, &renderer.base, w.bytes.items, null, null);
        try pr.compareLogs("helpers", renderer.log.bytes.items, result.part(1));
    }
}

test "DebugRenderer parity: DebugRendererSimple (triangle lists, LOD selection)" {
    try requireDebugRenderer();

    for (0..40) |round| {
        var gen = Generator.init(0x2000 + @as(u32, @intCast(round)), .{ .camera = true });
        defer gen.deinit();
        var w: pr.OpWriter = .{ .allocator = allocator };
        defer w.deinit();
        try gen.ops(&w, 400);

        const result = JoltResult{ .handle = jolt.jolt_dr_run_ops(w.bytes.items.ptr, w.bytes.items.len, 1).? };
        defer result.deinit();

        var renderer: pr.SimpleParityRenderer = undefined;
        try renderer.init(allocator);
        defer renderer.deinit();
        try pr.compareLogs("DebugRendererSimple: construction", renderer.log.bytes.items, result.part(0));

        try pr.runOps(allocator, renderer.base.debugRenderer(), w.bytes.items, &renderer.base, null);
        try pr.compareLogs("DebugRendererSimple", renderer.log.bytes.items, result.part(1));
    }
}

/// Jolt's recorder runs the ops of a round: the .jor bytes of its construction and of the ops
fn recordJolt(ops: []const u8) JoltResult {
    return .{ .handle = jolt.jolt_dr_record_ops(ops.ptr, ops.len).? };
}

test "DebugRenderer parity: recorder (.jor bytes)" {
    try requireDebugRenderer();

    for (0..30) |round| {
        var gen = Generator.init(0x3000 + @as(u32, @intCast(round)), .{ .allow_empty_batches = round % 2 == 0, .frames = true });
        defer gen.deinit();
        var w: pr.OpWriter = .{ .allocator = allocator };
        defer w.deinit();
        try gen.ops(&w, 300);

        const result = recordJolt(w.bytes.items);
        defer result.deinit();

        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        var wrapper: zolt.StreamOutWrapper = .init(&out.writer);
        var recorder: DebugRendererRecorder = undefined;
        try recorder.init(allocator, std.testing.io, wrapper.streamOut());
        defer recorder.deinit();
        if (!pr.isPrefix(out.written(), result.part(0))) {
            std.debug.print("recorder: the construction bytes are not the start of Jolt's\n", .{});
            return error.ParityMismatch;
        }
        recorder.next_batch_id = nextRecorderBatchID(result.part(0)); // Group D1 makes the constructions equal
        const construction_len = out.written().len;

        try pr.runOps(allocator, recorder.debugRenderer(), w.bytes.items, null, &recorder);
        const zolt_bytes = out.written()[construction_len..];
        const jolt_bytes = result.part(1);
        if (!std.mem.eql(u8, zolt_bytes, jolt_bytes)) {
            const index = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
            std.debug.print("recorder (round {d}): the .jor bytes differ at byte {d} of {d} (zolt) / {d} (jolt)\n  zolt: {x}\n  jolt: {x}\n", .{ round, index, zolt_bytes.len, jolt_bytes.len, zolt_bytes[index..@min(index + 64, zolt_bytes.len)], jolt_bytes[index..@min(index + 64, jolt_bytes.len)] });
            return error.ParityMismatch;
        }
    }
}

test "DebugRenderer parity: playback of Jolt's .jor stream (call log)" {
    try requireDebugRenderer();

    for (0..30) |round| {
        var gen = Generator.init(0x4000 + @as(u32, @intCast(round)), .{ .allow_empty_batches = false, .frames = true });
        defer gen.deinit();
        var w: pr.OpWriter = .{ .allocator = allocator };
        defer w.deinit();
        try gen.ops(&w, 300);

        // Jolt's recorder writes the stream
        const recorded = recordJolt(w.bytes.items);
        defer recorded.deinit();
        const stream = try std.mem.concat(allocator, u8, &.{ recorded.part(0), recorded.part(1) });
        defer allocator.free(stream);

        // Jolt's playback into Jolt's ParityRenderer
        const result = JoltResult{ .handle = jolt.jolt_dr_playback(stream.ptr, stream.len).? };
        defer result.deinit();

        // Zolt's playback into Zolt's ParityRenderer
        var renderer: pr.ParityRenderer = undefined;
        try renderer.init(allocator);
        defer renderer.deinit();
        try checkConstruction("playback: construction", renderer.log.bytes.items, result.part(0));
        renderer.next_batch_id = result.partU32(2);
        renderer.log.bytes.clearRetainingCapacity();
        {
            var playback = DebugRendererPlayback.init(allocator, &renderer.base);
            defer playback.deinit();
            var reader: std.Io.Reader = .fixed(stream);
            var in_wrapper: zolt.StreamInWrapper = .init(&reader);
            try playback.parse(in_wrapper.streamIn());
            try std.testing.expectEqual(result.partU32(3), playback.getNumFrames());
            try std.testing.expect(playback.getNumFrames() > 1);
            for (0..playback.getNumFrames()) |f| {
                renderer.log.writeMarker(@intCast(f));
                playback.drawFrame(@intCast(f));
            }
        }
        try pr.compareLogs("playback", renderer.log.bytes.items, result.part(1));
    }
}
