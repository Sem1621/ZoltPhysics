//! Parity tests of the debug renderer (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, "Draw call log", groups
//! D0 and D1). C ABI wrappers: ZoltParity/Renderer/DebugRendererReference.cpp; the log and the op stream:
//! ZoltParity/Renderer/ParityRenderer.zig and ParityRenderer.h.
//!
//! The layers (they exist only with -Ddebug_renderer=true, which gives the C++ reference JPH_DEBUG_RENDERER; without the
//! option they skip):
//! - helpers: random calls of the virtual functions and of every helper of DebugRenderer (D0's ops and D1's: the wire
//!   helpers, the solid helpers with the predefined geometry, the four cached helpers, NextFrame, the convex and vertex
//!   list batches, both Triangle constructors) on a ParityRenderer of each side; the call logs must match;
//! - DebugRendererSimple: the same calls and SetCameraPos on a DebugRendererSimple of each side that logs its lines: the
//!   triangle lists, the LOD selection and the transforms of Simple's DrawGeometry;
//! - recorder: the same calls on Jolt's and Zolt's DebugRendererRecorder, with EndFrame and NextFrame; the .jor bytes
//!   must match;
//! - playback: Jolt's .jor stream played back by Jolt's DebugRendererPlayback and by Zolt's into a ParityRenderer; the
//!   call logs must match;
//! - caches: the cached helpers (swing cone, swing pyramid, pie, tapered cylinder) over many frames with keys from small
//!   pools, so that keys hit in the frame, hit in the previous frame and miss after two frames. The pools hold
//!   degenerate keys that Jolt accepts: zero angles and radii, equal angles, zero height, -0 next to +0 (the same key)
//!   and NaN keys of the tapered cylinder (a new geometry every time).
//! The construction logs (the 25 predefined batches of Initialize) and the recorder's construction bytes must be equal.
//! The generators keep to the input that Jolt's asserts accept (normalized axes, cone angles in [0, pi], ...).

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
    /// Only the cached helpers, NextFrame and draws of the geometry slots (the caches layer)
    caches_only: bool = false,
};

/// The number of keys in each cache key pool of the generator
const num_keys = 8;

/// Random calls for the op stream
const Generator = struct {
    rng: Rng,
    arena: std.heap.ArenaAllocator,
    options: GenOptions,
    /// Geometry slots that hold a geometry
    filled_slots: u16 = 0,
    /// Key pools of the four caches: a key from a pool hits the cache in the frame and in the next one
    cone_keys: [num_keys][2]f32 = undefined,
    pyramid_keys: [num_keys][4]f32 = undefined,
    pie_keys: [num_keys][2]f32 = undefined,
    tapered_keys: [num_keys][4]f32 = undefined,

    fn init(seed: u32, options: GenOptions) Generator {
        var gen: Generator = .{ .rng = .{ .state = seed }, .arena = .init(allocator), .options = options };
        gen.initKeys();
        return gen;
    }

    /// The key pools, with the degenerate keys that Jolt accepts
    fn initKeys(self: *Generator) void {
        const pi = zolt.math.pi;
        const nan = std.math.nan(f32);

        // Swing cone (y and z half angle in [0, pi]): no swing (nothing drawn), swing around one axis only, -0, equal
        // angles. Both half angles pi draw nothing when Sin(pi / 2) is 1 (Jolt's early return). Otherwise they are left
        // out: a vertex beyond the unit sphere gives a NaN quaternion, which Jolt and Zolt assert on.
        for (&self.cone_keys) |*k| k.* = .{ self.rng.float(0, 0.9 * pi), self.rng.float(0, 0.9 * pi) };
        self.cone_keys[0] = .{ 0, 0 };
        self.cone_keys[1][0] = 0;
        self.cone_keys[2][1] = -0.0;
        self.cone_keys[3][1] = self.cone_keys[3][0];
        if (zolt.trigonometry.sin(0.5 * pi) >= 1.0) self.cone_keys[4] = .{ pi, pi };

        // Swing pyramid (min <= max): all zero, the same key with -0, equal angles
        for (&self.pyramid_keys) |*k| {
            const y = sorted(self.rng.float(-0.9 * pi, 0.9 * pi), self.rng.float(-0.9 * pi, 0.9 * pi));
            const z = sorted(self.rng.float(-0.9 * pi, 0.9 * pi), self.rng.float(-0.9 * pi, 0.9 * pi));
            k.* = .{ y[0], y[1], z[0], z[1] };
        }
        self.pyramid_keys[0] = .{ 0, 0, 0, 0 };
        self.pyramid_keys[1] = .{ -0.0, 0, -0.0, 0 };
        self.pyramid_keys[2][1] = self.pyramid_keys[2][0];
        self.pyramid_keys[3][3] = self.pyramid_keys[3][2];

        // Pie (min angle, max angle; the key is max - min): a full circle, equal angles (nothing drawn), the key of
        // another entry from another start angle, one part
        for (&self.pie_keys) |*k| {
            const min = self.rng.float(-2 * pi, 2 * pi);
            k.* = .{ min, min + self.rng.float(0.001, 4 * pi) };
        }
        self.pie_keys[0] = .{ 0, 2 * pi };
        self.pie_keys[1] = .{ 1, 1 };
        self.pie_keys[2] = .{ 0, self.pie_keys[3][1] - self.pie_keys[3][0] };
        self.pie_keys[4] = .{ 0, 1.0e-6 };

        // Tapered cylinder (top, bottom, top radius, bottom radius): a zero radius and the same key with -0, zero
        // height, NaN keys (a new geometry for every draw), zero radii
        for (&self.tapered_keys) |*k| k.* = .{ self.rng.float(0, 2), self.rng.float(-2, 0), self.rng.float(0, 2), self.rng.float(0, 2) };
        self.tapered_keys[0] = .{ 1, -1, 0, 0.5 };
        self.tapered_keys[1] = .{ 1, -1, -0.0, 0.5 };
        self.tapered_keys[2] = .{ 0.5, 0.5, 1, 1 };
        self.tapered_keys[3] = .{ nan, -1, 0.5, 0.5 };
        self.tapered_keys[4] = .{ 1, -1, nan, 0.25 };
        self.tapered_keys[5] = .{ 1, -1, 0, 0 };
    }

    fn sorted(a: f32, b: f32) [2]f32 {
        return if (a <= b) .{ a, b } else .{ b, a };
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

    /// A unit vector
    fn unitVec3(self: *Generator) Vec3 {
        while (true) {
            const v = self.vec3(-1, 1).normalizedOr(Vec3.axisY());
            if (v.isNormalized(.{})) return v;
        }
    }

    /// Two unit vectors that are perpendicular, as Jolt's asserts of DrawOpenCone and DrawPie want them
    fn perpendicularPair(self: *Generator) [2]Vec3 {
        while (true) {
            const a = self.unitVec3();
            const b = zolt.Quat.rotation(a, self.rng.float(-3, 3)).mulVec3(a.getNormalizedPerpendicular());
            if (b.isNormalized(.{ .tolerance = 1.0e-4 }) and @abs(a.dot(b)) < 1.0e-4) return .{ a, b };
        }
    }

    /// A size, now and then zero or negative (Jolt accepts both)
    fn size(self: *Generator, max: f32) f32 {
        if (self.chance(10)) return 0;
        if (self.chance(5)) return -self.rng.float(0, max);
        return self.rng.float(0, max);
    }

    fn support(self: *Generator) pr.Support {
        return .{ .half_extent = self.vec3(0, 2), .radius = if (self.chance(20)) 0 else self.rng.float(0, 1) };
    }

    fn pick(self: *Generator, comptime n: usize, comptime T: type, pool: *const [num_keys][n]T) [n]T {
        return pool[self.rng.next() % num_keys];
    }

    /// One random op of the cached helpers (or NextFrame)
    fn cacheOp(self: *Generator, w: *pr.OpWriter) !void {
        switch (self.rng.next() % 5) {
            0 => {
                const k = self.pick(2, f32, &self.cone_keys);
                try w.drawSwingConeLimits(self.affine(), k[0], k[1], self.rng.float(0.1, 3), self.color(), self.mode(2), self.mode(2));
            },
            1 => try w.drawSwingPyramidLimits(self.affine(), self.pick(4, f32, &self.pyramid_keys), self.rng.float(0.1, 3), self.color(), self.mode(2), self.mode(2)),
            2 => {
                const k = self.pick(2, f32, &self.pie_keys);
                const normal_axis = self.perpendicularPair();
                try w.drawPie(self.rvec3(), self.size(3), normal_axis[0], normal_axis[1], k[0], k[1], self.color(), self.mode(2), self.mode(2));
            },
            3 => try w.drawTaperedCylinder(self.affine(), self.pick(4, f32, &self.tapered_keys), self.color(), self.mode(2), self.mode(2)),
            4 => if (self.chance(25)) try w.nextFrame(),
            else => unreachable,
        }
    }

    /// One random op of D1's helpers
    fn helperOp(self: *Generator, w: *pr.OpWriter) !void {
        const a = self.arena.allocator();
        switch (self.rng.next() % 21) {
            0 => try w.drawWireBoxOriented(self.affine().columns, self.vec3(-50, 50), self.vec3(0, 5), self.color()),
            1 => try w.drawCoordinateSystem(self.affine(), self.size(3)),
            2 => {
                const normal = if (self.chance(5)) Vec3.zero() else if (self.chance(50)) self.unitVec3() else self.vec3(-2, 2);
                try w.drawPlane(self.rvec3(), normal, self.color(), self.size(3));
            },
            3 => try w.drawWireTriangle(self.rvec3(), self.rvec3(), self.rvec3(), self.color()),
            4 => {
                const vertices = try a.alloc(Vec3, self.rng.next() % 7);
                for (vertices) |*v| v.* = self.vec3(-2, 2);
                try w.drawWirePolygon(self.affine(), vertices, self.color(), if (self.chance(40)) 0 else self.rng.float(-0.5, 1));
            },
            5 => try w.drawWireSphere(self.rvec3(), self.size(3), self.color(), self.level()),
            6 => try w.drawWireUnitSphere(self.affine(), self.color(), self.level()),
            7 => try w.drawSphere(self.rvec3(), self.size(3), self.color(), self.mode(2), self.mode(2)),
            8 => try w.drawUnitSphere(self.affine(), self.color(), self.mode(2), self.mode(2)),
            9, 10 => try w.drawCapsuleOrCylinder(if (self.rng.next() % 2 == 0) .draw_capsule else .draw_cylinder, self.affine(), self.size(3), self.size(2), self.color(), self.mode(2), self.mode(2)),
            11 => {
                const axis_perpendicular = self.perpendicularPair();
                const half_angle = if (self.chance(10)) 0 else self.rng.float(-1.5, 1.5);
                try w.drawOpenCone(self.rvec3(), axis_perpendicular[0], axis_perpendicular[1], half_angle, self.size(3), self.color(), self.mode(2), self.mode(2));
            },
            12, 13, 14 => try self.cacheOp(w),
            15 => try w.nextFrame(),
            16 => {
                const min_count: u32 = if (self.options.allow_empty_batches) 0 else 1;
                const vertices = try a.alloc(Float3, 1 + self.rng.next() % 8);
                for (vertices) |*v| self.vec3(-2, 2).storeFloat3(v);
                const triangles = try a.alloc(zolt.IndexedTriangleNoMaterial, min_count + self.rng.next() % (7 - min_count));
                for (triangles) |*t| {
                    for (&t.idx) |*i| i.* = self.rng.next() % @as(u32, @intCast(vertices.len));
                }
                try w.createBatchVertexList(vertices, triangles);
            },
            17 => {
                const with_bounds = self.chance(50);
                const slot = self.rng.next() % pr.num_geometry_slots;
                try w.createBatchForConvex(self.support(), @intCast(self.rng.next() % 5), with_bounds, slot);
                if (with_bounds) self.filled_slots |= @as(u16, 1) << @intCast(slot);
            },
            18 => {
                const slot = self.rng.next() % pr.num_geometry_slots;
                try w.createGeometryForConvex(self.support(), slot);
                self.filled_slots |= @as(u16, 1) << @intCast(slot);
            },
            19 => {
                const min_count: u32 = if (self.options.allow_empty_batches) 0 else 1;
                const triangles = try a.alloc(pr.TriangleData, min_count + self.rng.next() % (6 - min_count));
                for (triangles) |*t| {
                    t.* = .{ .v = .{ self.vec3(-2, 2), self.vec3(-2, 2), self.vec3(-2, 2) }, .color = self.color() };
                    if (self.chance(10)) t.v[2] = t.v[1]; // Degenerate: Triangle without UVs keeps a zero normal
                    if (self.chance(50)) t.uv = .{ .origin = self.vec3(-2, 2), .direction = self.vec3(-1, 1) };
                }
                try w.createBatchTriangles(triangles);
            },
            20 => {
                const slot = self.filledSlot() orelse return self.createGeometry(w);
                try w.drawGeometryAutoBounds(self.affine(), self.color(), slot, self.mode(3), self.mode(2), self.mode(2));
            },
            else => unreachable,
        }
    }

    /// A subdivision level of the wire spheres (level 4 now and then: 6144 lines)
    fn level(self: *Generator) i32 {
        return if (self.chance(5)) 4 else @intCast(self.rng.next() % 4);
    }

    /// One random op, then now and then the end of a frame (EndFrame, mostly followed by NextFrame like in an application)
    fn op(self: *Generator, w: *pr.OpWriter) !void {
        if (self.options.caches_only) {
            if (self.chance(15)) {
                const slot = self.filledSlot() orelse return self.createGeometry(w);
                try w.drawGeometryAutoBounds(self.affine(), self.color(), slot, self.mode(3), self.mode(2), self.mode(2));
            } else {
                try self.cacheOp(w);
            }
        } else {
            // D0's 15 ops and D1's 21
            const choice = self.rng.next() % 36;
            if (choice >= 15) try self.helperOp(w) else try self.coreOp(w, choice);
        }
        if (self.options.frames and self.chance(if (self.options.caches_only) 5 else 3)) {
            try w.endFrame();
            if (self.chance(70)) try w.nextFrame();
        }
    }

    /// One op of D0: the virtual functions and the box, marker and arrow helpers
    fn coreOp(self: *Generator, w: *pr.OpWriter, choice: u32) !void {
        switch (choice) {
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
    }

    /// `num_ops` random ops (and a final EndFrame with `frames`)
    fn ops(self: *Generator, w: *pr.OpWriter, num_ops: usize) !void {
        for (0..num_ops) |_| try self.op(w);
        if (self.options.frames) try w.endFrame();
    }
};

/// The construction log: the 25 predefined batches of Initialize (D1)
fn checkConstruction(name: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8, expected_next_batch_id: u32) !void {
    try pr.compareLogs(name, zolt_bytes, jolt_bytes);
    try std.testing.expectEqual(@as(u32, 26), expected_next_batch_id);
}

/// Compare two byte streams, print the first difference
fn compareBytes(name: []const u8, round: usize, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes))
        return;
    const index = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
    std.debug.print("{s} (round {d}): the .jor bytes differ at byte {d} of {d} (zolt) / {d} (jolt)\n  zolt: {x}\n  jolt: {x}\n", .{ name, round, index, zolt_bytes.len, jolt_bytes.len, zolt_bytes[index..@min(index + 64, zolt_bytes.len)], jolt_bytes[index..@min(index + 64, jolt_bytes.len)] });
    return error.ParityMismatch;
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
        try checkConstruction("helpers: construction", renderer.log.bytes.items, result.part(0), result.partU32(2));
        try std.testing.expectEqual(result.partU32(2), renderer.next_batch_id);
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
        try compareBytes("recorder: construction", round, out.written(), result.part(0)); // The 25 batches of Initialize
        const construction_len = out.written().len;

        try pr.runOps(allocator, recorder.debugRenderer(), w.bytes.items, null, &recorder);
        try compareBytes("recorder", round, out.written()[construction_len..], result.part(1));
    }
}

test "DebugRenderer parity: the caches of the helpers over many frames (call log and .jor bytes)" {
    try requireDebugRenderer();

    for (0..20) |round| {
        var gen = Generator.init(0x5000 + @as(u32, @intCast(round)), .{ .allow_empty_batches = false, .frames = true, .caches_only = true });
        defer gen.deinit();
        var w: pr.OpWriter = .{ .allocator = allocator };
        defer w.deinit();
        try gen.ops(&w, 400);

        // The call log of a ParityRenderer
        {
            const result = JoltResult{ .handle = jolt.jolt_dr_run_ops(w.bytes.items.ptr, w.bytes.items.len, 0).? };
            defer result.deinit();
            var renderer: pr.ParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            try checkConstruction("caches: construction", renderer.log.bytes.items, result.part(0), result.partU32(2));
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOps(allocator, &renderer.base, w.bytes.items, null, null);
            try pr.compareLogs("caches", renderer.log.bytes.items, result.part(1));
        }

        // The .jor bytes of a recorder
        {
            const result = recordJolt(w.bytes.items);
            defer result.deinit();
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var wrapper: zolt.StreamOutWrapper = .init(&out.writer);
            var recorder: DebugRendererRecorder = undefined;
            try recorder.init(allocator, std.testing.io, wrapper.streamOut());
            defer recorder.deinit();
            const construction_len = out.written().len;
            try pr.runOps(allocator, recorder.debugRenderer(), w.bytes.items, null, &recorder);
            try compareBytes("caches: recorder", round, out.written()[construction_len..], result.part(1));
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
        try checkConstruction("playback: construction", renderer.log.bytes.items, result.part(0), result.partU32(2));
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
