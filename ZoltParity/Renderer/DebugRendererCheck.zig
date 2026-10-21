//! Zolt helper of `zig build debug-renderer-check -Ddebug_renderer=true` (no Jolt counterpart,
//! Docs/Zolt/RemainingFeaturesArchitecture.md section 4, ".jor bytes"): the C++ PerformanceTest and ZoltPerformanceTest
//! record the 5 scenes of the determinism check with `-r -i=20`, and their .jor files must be equal byte for byte.
//!
//! Usage: DebugRendererCheck <C++ PerformanceTest> <ZoltPerformanceTest> <output directory>
//!
//! Each program runs once per scene in its own directory below the output directory (PerformanceTest writes
//! `performance_test_<quality>_th1.jor` into its working directory and finds Assets/ from its own path). DrawBodies runs
//! on the main thread, so the thread count does not change the bytes; the check uses one thread.
//!
//! Zolt's DebugRendererPlayback also reads every .jor that Jolt wrote (all frames must parse). Group D0 built the step
//! before ZoltPerformanceTest could record (`zolt_records == false`: only the C++ side ran). Group D3 ported `-r` and
//! BodyManager.draw and set `zolt_records`.

const std = @import("std");
const zolt = @import("zolt");

/// True once ZoltPerformanceTest records with -r (group D3)
const zolt_records = true;

/// The number of updates of each recording
const num_updates = 20;

/// The scenes of the determinism check (.github/workflows/determinism_check.yml) and their motion quality
const scenes = [_]struct { name: []const u8, quality: []const u8 }{
    .{ .name = "ConvexVsMesh", .quality = "LinearCast" },
    .{ .name = "HighSpeed", .quality = "LinearCast" },
    .{ .name = "Ragdoll", .quality = "LinearCast" },
    .{ .name = "Pyramid", .quality = "LinearCast" },
    .{ .name = "CharacterVirtual", .quality = "Discrete" },
};

pub fn main(init: std.process.Init) !u8 {
    const arena = init.arena.allocator();
    const io = init.io;
    const cwd = std.Io.Dir.cwd();

    const args = try init.minimal.args.toSlice(arena);
    if (args.len != 4) {
        std.debug.print("usage: {s} <C++ PerformanceTest> <ZoltPerformanceTest> <output directory>\n", .{if (args.len > 0) args[0] else "DebugRendererCheck"});
        return 2;
    }
    const jolt_exe = try cwd.realPathFileAlloc(io, args[1], arena);
    const zolt_exe = try cwd.realPathFileAlloc(io, args[2], arena);
    const out_dir = args[3];

    if (!zolt.Core.debug_renderer) {
        std.debug.print("debug-renderer-check: build with -Ddebug_renderer=true (the C++ PerformanceTest only records with JPH_DEBUG_RENDERER)\n", .{});
        return 1;
    }

    var failed = false;
    for (scenes) |scene| {
        const jor_name = try std.fmt.allocPrint(arena, "performance_test_{s}_th1.jor", .{if (std.mem.eql(u8, scene.quality, "Discrete")) "discrete" else "linearcast"});
        const jolt_bytes = try record(arena, io, jolt_exe, out_dir, scene.name, "jolt", scene.quality, jor_name) orelse {
            std.debug.print("debug-renderer-check: {s}: the C++ PerformanceTest wrote no {s}\n", .{ scene.name, jor_name });
            failed = true;
            continue;
        };

        // Zolt's playback reads Jolt's stream: every frame must parse
        const num_frames = countFrames(arena, io, jolt_bytes) catch |err| {
            std.debug.print("debug-renderer-check: {s}: Zolt's DebugRendererPlayback cannot read Jolt's .jor ({s})\n", .{ scene.name, @errorName(err) });
            failed = true;
            continue;
        };
        if (num_frames != num_updates) {
            std.debug.print("debug-renderer-check: {s}: Jolt's .jor has {d} frames, expected {d}\n", .{ scene.name, num_frames, num_updates });
            failed = true;
            continue;
        }

        const zolt_bytes = try record(arena, io, zolt_exe, out_dir, scene.name, "zolt", scene.quality, jor_name);
        if (!zolt_records) {
            if (zolt_bytes != null) {
                std.debug.print("debug-renderer-check: {s}: ZoltPerformanceTest records now, set zolt_records in ZoltParity/Renderer/DebugRendererCheck.zig\n", .{scene.name});
                failed = true;
            }
            continue;
        }
        const z = zolt_bytes orelse {
            std.debug.print("debug-renderer-check: {s}: ZoltPerformanceTest wrote no {s}\n", .{ scene.name, jor_name });
            failed = true;
            continue;
        };
        if (std.mem.indexOfDiff(u8, jolt_bytes, z)) |index| {
            std.debug.print("debug-renderer-check: {s}: the .jor bytes differ at byte {d} (C++: {d} bytes, Zig: {d} bytes)\n", .{ scene.name, index, jolt_bytes.len, z.len });
            failed = true;
        }
    }
    if (failed) return 1;

    if (zolt_records) {
        std.debug.print("debug-renderer-check: the .jor bytes of the {d} scenes are equal\n", .{scenes.len});
    } else {
        std.debug.print("debug-renderer-check: the C++ PerformanceTest recorded the {d} scenes and Zolt's DebugRendererPlayback read them. The Zolt side is not ported yet: ZoltPerformanceTest -r and BodyManager.draw come with group D3.\n", .{scenes.len});
    }
    return 0;
}

/// Run `exe` for a scene in `<out_dir>/<scene>/<side>` with `-r -i=20` and return the .jor it wrote (null if none)
fn record(arena: std.mem.Allocator, io: std.Io, exe: []const u8, out_dir: []const u8, scene: []const u8, side: []const u8, quality: []const u8, jor_name: []const u8) !?[]u8 {
    const dir = try std.fs.path.join(arena, &.{ out_dir, scene, side });
    const cwd = std.Io.Dir.cwd();
    cwd.deleteTree(io, dir) catch {};
    try cwd.createDirPath(io, dir);

    const argv = [_][]const u8{
        exe,
        try std.fmt.allocPrint(arena, "-s={s}", .{scene}),
        try std.fmt.allocPrint(arena, "-q={s}", .{quality}),
        "-t=1",
        "-r",
        try std.fmt.allocPrint(arena, "-i={d}", .{num_updates}),
    };
    const result = try std.process.run(arena, io, .{ .argv = &argv, .cwd = .{ .path = dir } });
    const ok = switch (result.term) {
        .exited => |code| code == 0,
        else => false,
    };
    if (!ok) {
        std.debug.print("debug-renderer-check: {s} failed for {s} ({any})\nstdout:\n{s}\nstderr:\n{s}\n", .{ exe, scene, result.term, result.stdout, result.stderr });
        return error.ProgramFailed;
    }

    const path = try std.fs.path.join(arena, &.{ dir, jor_name });
    return cwd.readFileAlloc(io, path, arena, .unlimited) catch |err| switch (err) {
        error.FileNotFound => null,
        else => err,
    };
}

/// A renderer that only keeps the batches alive (the playback needs one)
const NullRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const BatchImpl = struct {
        pub const overrides = .{};
        base: zolt.DebugRenderer.BatchTarget,
    };

    base: zolt.DebugRenderer,

    pub fn drawLine(self: *NullRenderer, from: zolt.RVec3, to: zolt.RVec3, color: zolt.Color) void {
        _ = .{ self, from, to, color };
    }
    pub fn drawTriangle(self: *NullRenderer, v1: zolt.RVec3, v2: zolt.RVec3, v3: zolt.RVec3, color: zolt.Color, cast_shadow: zolt.DebugRenderer.CastShadow) void {
        _ = .{ self, v1, v2, v3, color, cast_shadow };
    }
    pub fn createTriangleBatch(self: *NullRenderer, triangles: []const zolt.DebugRenderer.Triangle) zolt.DebugRenderer.Batch {
        _ = triangles;
        return self.newBatch();
    }
    pub fn createTriangleBatchIndexed(self: *NullRenderer, vertices: []const zolt.DebugRenderer.Vertex, indices: []const u32) zolt.DebugRenderer.Batch {
        _ = .{ vertices, indices };
        return self.newBatch();
    }
    pub fn drawGeometry(self: *NullRenderer, model_matrix: zolt.RMat44, world_space_bounds: zolt.AABox, lod_scale_sq: f32, model_color: zolt.Color, geometry: *const zolt.DebugRenderer.Geometry, cull_mode: zolt.DebugRenderer.CullMode, cast_shadow: zolt.DebugRenderer.CastShadow, draw_mode: zolt.DebugRenderer.DrawMode) void {
        _ = .{ self, model_matrix, world_space_bounds, lod_scale_sq, model_color, geometry, cull_mode, cast_shadow, draw_mode };
    }
    pub fn drawText3D(self: *NullRenderer, position: zolt.RVec3, string: []const u8, color: zolt.Color, height: f32) void {
        _ = .{ self, position, string, color, height };
    }

    fn newBatch(self: *NullRenderer) zolt.DebugRenderer.Batch {
        const batch = self.base.allocator.create(BatchImpl) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(BatchImpl, self.base.allocator) };
        return .init(&batch.base);
    }
};

/// Parse a .jor stream with Zolt's playback and draw every frame, return the number of frames
fn countFrames(allocator: std.mem.Allocator, io: std.Io, bytes: []const u8) !u32 {
    var renderer: NullRenderer = .{ .base = .init(NullRenderer, allocator, io) };
    try renderer.base.initialize();
    defer renderer.base.deinit();

    var playback = zolt.DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback.deinit();
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: zolt.StreamInWrapper = .init(&reader);
    try playback.parse(wrapper.streamIn());
    for (0..playback.getNumFrames()) |f| playback.drawFrame(@intCast(f));
    if (renderer.base.isOutOfMemory()) return error.OutOfMemory;
    return playback.getNumFrames();
}
