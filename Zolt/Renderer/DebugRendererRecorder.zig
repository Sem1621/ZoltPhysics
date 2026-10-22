//! Port of: Jolt/Renderer/DebugRendererRecorder.h, Jolt/Renderer/DebugRendererRecorder.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R3: writes Jolt's `.jor` byte stream, the
//! vehicle of the debug renderer parity (Jolt's recorder writes the same bytes for the same draw calls).
//! - `lock_guard lock(mMutex)` is the Zolt `Mutex` with the renderer's `io`.
//! - Jolt aborts when it runs out of memory. Here a draw call that cannot store its primitive drops it and latches
//!   `out_of_memory` in the base class; `endFrame` writes what it has and then returns `error.OutOfMemory` (R5).
//! - The geometry map is keyed by the geometry (Jolt: `UnorderedMap<GeometryRef, uint32>`, it holds a reference); it is
//!   only searched, never iterated, so a std hash map gives Jolt's results.
//! - Zolt (UB34 of R20): a LOD with an empty `Batch` records batch ID 0, which DebugRendererPlayback reads as an empty
//!   batch (UB7). Jolt dereferences the null batch. An empty batch comes from the playback of ID 0, from
//!   `Geometry.createWithBatch(allocator, .empty, ...)` or from an allocation failure (R5).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../Core/Color.zig").Color;
const Mutex = @import("../Core/Mutex.zig").Mutex;
const StreamOut = @import("../Core/StreamOut.zig").StreamOut;
const AABox = @import("../Geometry/AABox.zig").AABox;
const RealFile = @import("../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const DebugRenderer = @import("DebugRenderer.zig").DebugRenderer;
const Batch = DebugRenderer.Batch;
const BatchTarget = DebugRenderer.BatchTarget;
const CastShadow = DebugRenderer.CastShadow;
const CullMode = DebugRenderer.CullMode;
const DrawMode = DebugRenderer.DrawMode;
const Geometry = DebugRenderer.Geometry;
const Triangle = DebugRenderer.Triangle;
const Vertex = DebugRenderer.Vertex;

/// Implementation of DebugRenderer that records the API invocations to be played back later
pub const DebugRendererRecorder = struct {
    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    /// Control commands written into the stream
    pub const Command = enum(u8) {
        create_batch,
        create_batch_indexed,
        create_geometry,
        end_frame,
    };

    /// Holds a single line segment
    pub const LineBlob = struct {
        from: RVec3,
        to: RVec3,
        color: Color,
    };

    /// Holds a single triangle
    pub const TriangleBlob = struct {
        v1: RVec3,
        v2: RVec3,
        v3: RVec3,
        color: Color,
        cast_shadow: CastShadow,
    };

    /// Holds a single text entry (owns its string)
    pub const TextBlob = struct {
        position: RVec3,
        string: []u8,
        color: Color,
        height: f32,
    };

    /// Holds a single geometry draw call
    pub const GeometryBlob = struct {
        model_matrix: RMat44,
        model_color: Color,
        geometry_id: u32,
        cull_mode: CullMode,
        cast_shadow: CastShadow,
        draw_mode: DrawMode,
    };

    /// All information for a single frame
    pub const Frame = struct {
        lines: std.ArrayList(LineBlob) = .empty,
        triangles: std.ArrayList(TriangleBlob) = .empty,
        texts: std.ArrayList(TextBlob) = .empty,
        geometries: std.ArrayList(GeometryBlob) = .empty,
    };

    /// Implementation specific batch object
    const BatchImpl = struct {
        pub const overrides = .{};
        base: BatchTarget,
        id: u32,
    };

    base: DebugRenderer,
    /// Lock that prevents concurrent access to the internal structures
    mutex: Mutex = .{},
    /// Stream that recorded data will be sent to
    stream: StreamOut,
    /// Next available ID
    next_batch_id: u32 = 1,
    next_geometry_id: u32 = 1,
    /// Cached geometries and their IDs (each key holds a reference)
    geometries: std.AutoHashMapUnmanaged(*const Geometry, u32) = .empty,
    /// Data that is being accumulated for the current frame
    current_frame: Frame = .{},

    /// Constructor (DebugRendererRecorder(StreamOut &)): writes the predefined geometry to the stream (Initialize).
    /// Must not be moved afterwards (the base class is handed out as `*DebugRenderer`).
    pub fn init(self: *DebugRendererRecorder, allocator: Allocator, io: std.Io, stream: StreamOut) Allocator.Error!void {
        self.* = .{ .base = .init(DebugRendererRecorder, allocator, io), .stream = stream };
        errdefer self.deinit();
        try self.base.initialize();
    }

    /// Destructor
    pub fn deinit(self: *DebugRendererRecorder) void {
        self.base.deinit();
    }

    /// Destructor of this level (called by the generated destructor chain)
    pub fn destruct(self: *DebugRendererRecorder) void {
        const allocator = self.base.allocator;
        var it = self.geometries.keyIterator();
        while (it.next()) |geometry| geometry.*.release();
        self.geometries.deinit(allocator);
        self.clearFrame();
        self.current_frame.lines.deinit(allocator);
        self.current_frame.triangles.deinit(allocator);
        self.current_frame.texts.deinit(allocator);
        self.current_frame.geometries.deinit(allocator);
    }

    /// The recorder as a DebugRenderer
    pub fn debugRenderer(self: *DebugRendererRecorder) *DebugRenderer {
        return &self.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Implementation of DebugRenderer interface

    pub fn drawLine(self: *DebugRendererRecorder, from: RVec3, to: RVec3, color: Color) void {
        self.mutex.lock(self.base.io);
        defer self.mutex.unlock(self.base.io);

        self.current_frame.lines.append(self.base.allocator, .{ .from = from, .to = to, .color = color }) catch self.base.setOutOfMemory();
    }

    pub fn drawTriangle(self: *DebugRendererRecorder, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: CastShadow) void {
        self.mutex.lock(self.base.io);
        defer self.mutex.unlock(self.base.io);

        self.current_frame.triangles.append(self.base.allocator, .{ .v1 = v1, .v2 = v2, .v3 = v3, .color = color, .cast_shadow = cast_shadow }) catch self.base.setOutOfMemory();
    }

    pub fn createTriangleBatch(self: *DebugRendererRecorder, triangles: []const Triangle) Batch {
        if (triangles.len == 0)
            return self.newBatch(0);

        self.mutex.lock(self.base.io);
        defer self.mutex.unlock(self.base.io);

        self.stream.write(@intFromEnum(Command.create_batch));

        const batch_id = self.next_batch_id;
        self.next_batch_id += 1;
        std.debug.assert(batch_id != 0);
        self.stream.write(batch_id);
        self.stream.write(@as(u32, @intCast(triangles.len)));
        self.stream.writeBytes(std.mem.sliceAsBytes(triangles));

        return self.newBatch(batch_id);
    }

    pub fn createTriangleBatchIndexed(self: *DebugRendererRecorder, vertices: []const Vertex, indices: []const u32) Batch {
        if (vertices.len == 0 or indices.len == 0)
            return self.newBatch(0);

        self.mutex.lock(self.base.io);
        defer self.mutex.unlock(self.base.io);

        self.stream.write(@intFromEnum(Command.create_batch_indexed));

        const batch_id = self.next_batch_id;
        self.next_batch_id += 1;
        std.debug.assert(batch_id != 0);
        self.stream.write(batch_id);
        self.stream.write(@as(u32, @intCast(vertices.len)));
        self.stream.writeBytes(std.mem.sliceAsBytes(vertices));
        self.stream.write(@as(u32, @intCast(indices.len)));
        self.stream.writeBytes(std.mem.sliceAsBytes(indices));

        return self.newBatch(batch_id);
    }

    pub fn drawGeometry(self: *DebugRendererRecorder, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, cull_mode: CullMode, cast_shadow: CastShadow, draw_mode: DrawMode) void {
        _ = .{ world_space_bounds, lod_scale_sq }; // Not recorded (Jolt)

        self.mutex.lock(self.base.io);
        defer self.mutex.unlock(self.base.io);

        // See if this geometry was used before
        const entry = self.geometries.getOrPut(self.base.allocator, geometry) catch {
            self.base.setOutOfMemory();
            return;
        };
        if (!entry.found_existing) {
            geometry.addRef(); // The map holds a GeometryRef
            self.stream.write(@intFromEnum(Command.create_geometry));

            // Create a new ID
            const geometry_id = self.next_geometry_id;
            self.next_geometry_id += 1;
            std.debug.assert(geometry_id != 0);
            entry.value_ptr.* = geometry_id;
            self.stream.write(geometry_id);

            // Save bounds
            self.stream.write(geometry.bounds.min);
            self.stream.write(geometry.bounds.max);

            // Save the LODs
            self.stream.write(@as(u32, @intCast(geometry.lods.items.len)));
            for (geometry.lods.items) |lod| {
                self.stream.write(lod.distance);
                // Zolt (UB34): an empty batch (playback ID 0, a user geometry, an allocation failure) records ID 0.
                // Jolt dereferences the null batch.
                const id: u32 = if (lod.triangle_batch.get()) |b| @as(*const BatchImpl, @alignCast(@fieldParentPtr("base", b))).id else 0;
                self.stream.write(id);
            }
        }

        self.current_frame.geometries.append(self.base.allocator, .{ .model_matrix = model_matrix, .model_color = model_color, .geometry_id = entry.value_ptr.*, .cull_mode = cull_mode, .cast_shadow = cast_shadow, .draw_mode = draw_mode }) catch self.base.setOutOfMemory();
    }

    pub fn drawText3D(self: *DebugRendererRecorder, position: RVec3, string: []const u8, color: Color, height: f32) void {
        self.mutex.lock(self.base.io);
        defer self.mutex.unlock(self.base.io);

        const copy = self.base.allocator.dupe(u8, string) catch {
            self.base.setOutOfMemory();
            return;
        };
        self.current_frame.texts.append(self.base.allocator, .{ .position = position, .string = copy, .color = color, .height = height }) catch {
            self.base.allocator.free(copy);
            self.base.setOutOfMemory();
        };
    }

    /// Mark the end of a frame. Zolt: `error.OutOfMemory` when a draw call dropped a primitive (the frame is written
    /// without it; the latch stays set).
    pub fn endFrame(self: *DebugRendererRecorder) Allocator.Error!void {
        {
            self.mutex.lock(self.base.io);
            defer self.mutex.unlock(self.base.io);

            self.stream.write(@intFromEnum(Command.end_frame));

            // Write all lines
            self.stream.write(@as(u32, @intCast(self.current_frame.lines.items.len)));
            for (self.current_frame.lines.items) |line| {
                self.stream.write(line.from);
                self.stream.write(line.to);
                self.stream.write(line.color);
            }

            // Write all triangles
            self.stream.write(@as(u32, @intCast(self.current_frame.triangles.items.len)));
            for (self.current_frame.triangles.items) |triangle| {
                self.stream.write(triangle.v1);
                self.stream.write(triangle.v2);
                self.stream.write(triangle.v3);
                self.stream.write(triangle.color);
                self.stream.write(@intFromEnum(triangle.cast_shadow));
            }

            // Write all texts
            self.stream.write(@as(u32, @intCast(self.current_frame.texts.items.len)));
            for (self.current_frame.texts.items) |text| {
                self.stream.write(text.position);
                self.stream.writeString(text.string);
                self.stream.write(text.color);
                self.stream.write(text.height);
            }

            // Write all geometries
            self.stream.write(@as(u32, @intCast(self.current_frame.geometries.items.len)));
            for (self.current_frame.geometries.items) |geom| {
                self.stream.write(geom.model_matrix);
                self.stream.write(geom.model_color);
                self.stream.write(geom.geometry_id);
                self.stream.write(@intFromEnum(geom.cull_mode));
                self.stream.write(@intFromEnum(geom.cast_shadow));
                self.stream.write(@intFromEnum(geom.draw_mode));
            }

            self.clearFrame();
        }

        if (self.base.isOutOfMemory())
            return error.OutOfMemory;
    }

    fn clearFrame(self: *DebugRendererRecorder) void {
        self.current_frame.lines.clearRetainingCapacity();
        self.current_frame.triangles.clearRetainingCapacity();
        for (self.current_frame.texts.items) |text| self.base.allocator.free(text.string);
        self.current_frame.texts.clearRetainingCapacity();
        self.current_frame.geometries.clearRetainingCapacity();
    }

    /// new BatchImpl(inID), an empty Batch when out of memory (latched)
    fn newBatch(self: *DebugRendererRecorder, id: u32) Batch {
        const batch = self.base.allocator.create(BatchImpl) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(BatchImpl, self.base.allocator), .id = id };
        return .init(&batch.base);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Zolt-only tests: the empty batch (UB34) and the allocation failures (R5)

const Vec3 = @import("../Math/Vec3.zig").Vec3;
const StreamWrapper = @import("../Core/StreamWrapper.zig");
const DebugRendererPlayback = @import("DebugRendererPlayback.zig").DebugRendererPlayback;
const GeometryRef = DebugRenderer.GeometryRef;

/// A renderer that only keeps the batches alive (a playback needs one)
const NullRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const NullBatch = struct {
        pub const overrides = .{};
        base: BatchTarget,
    };

    base: DebugRenderer,

    pub fn drawLine(self: *NullRenderer, from: RVec3, to: RVec3, color: Color) void {
        _ = .{ self, from, to, color };
    }
    pub fn drawTriangle(self: *NullRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: CastShadow) void {
        _ = .{ self, v1, v2, v3, color, cast_shadow };
    }
    pub fn createTriangleBatch(self: *NullRenderer, triangles: []const Triangle) Batch {
        _ = triangles;
        return self.newBatch();
    }
    pub fn createTriangleBatchIndexed(self: *NullRenderer, vertices: []const Vertex, indices: []const u32) Batch {
        _ = .{ vertices, indices };
        return self.newBatch();
    }
    pub fn drawGeometry(self: *NullRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, cull_mode: CullMode, cast_shadow: CastShadow, draw_mode: DrawMode) void {
        _ = .{ self, model_matrix, world_space_bounds, lod_scale_sq, model_color, geometry, cull_mode, cast_shadow, draw_mode };
    }
    pub fn drawText3D(self: *NullRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        _ = .{ self, position, string, color, height };
    }

    fn newBatch(self: *NullRenderer) Batch {
        const batch = self.base.allocator.create(NullBatch) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(NullBatch, self.base.allocator) };
        return .init(&batch.base);
    }
};

/// Parse `bytes` into `playback`
fn parseBytes(playback: *DebugRendererPlayback, bytes: []const u8) DebugRendererPlayback.Error!void {
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try playback.parse(wrapper.streamIn());
}

test "DebugRendererRecorder: a LOD with an empty batch records batch ID 0 (UB34)" {
    // Jolt dereferences the null batch. Zolt writes ID 0, which the playback reads as an empty batch (UB7). A geometry
    // without LODs records 0 LODs, like Jolt.
    const allocator = std.testing.allocator;
    const bounds = AABox.init(Vec3.zero(), Vec3.one());

    // A user geometry with an empty batch and a geometry without LODs
    var out1: std.Io.Writer.Allocating = .init(allocator);
    defer out1.deinit();
    {
        var wrapper: StreamWrapper.StreamOutWrapper = .init(&out1.writer);
        var recorder: DebugRendererRecorder = undefined;
        try recorder.init(allocator, std.testing.io, wrapper.streamOut());
        defer recorder.deinit();
        var empty_batch = GeometryRef.init(try Geometry.createWithBatch(allocator, .empty, bounds));
        defer empty_batch.deinit();
        var no_lods = GeometryRef.init(try Geometry.create(allocator, bounds));
        defer no_lods.deinit();
        recorder.debugRenderer().drawGeometryAutoBounds(RMat44.identity(), Color.white, empty_batch.get().?, .{});
        recorder.debugRenderer().drawGeometryAutoBounds(RMat44.identity(), Color.white, no_lods.get().?, .{});
        try recorder.endFrame();
    }

    // Play the stream back into a second recorder: the playback makes an empty batch of ID 0, which the recorder
    // records as ID 0 again
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    {
        var wrapper: StreamWrapper.StreamOutWrapper = .init(&out2.writer);
        var recorder: DebugRendererRecorder = undefined;
        try recorder.init(allocator, std.testing.io, wrapper.streamOut());
        defer recorder.deinit();
        var playback = DebugRendererPlayback.init(allocator, recorder.debugRenderer());
        defer playback.deinit();
        try parseBytes(&playback, out1.written());
        try std.testing.expectEqual(@as(u32, 1), playback.getNumFrames());
        playback.drawFrame(0);
        try recorder.endFrame();
    }

    // Both streams hold geometry 1 with one LOD of batch ID 0 (an empty batch) and geometry 2 without LODs
    for ([_][]const u8{ out1.written(), out2.written() }) |bytes| {
        var renderer: NullRenderer = .{ .base = .init(NullRenderer, allocator, std.testing.io) };
        try renderer.base.initialize();
        defer renderer.base.deinit();
        var playback = DebugRendererPlayback.init(allocator, &renderer.base);
        defer playback.deinit();
        try parseBytes(&playback, bytes);
        const geometry1 = playback.geometries.find(1).?.value.get().?;
        try std.testing.expectEqual(@as(usize, 1), geometry1.lods.items.len);
        try std.testing.expect(geometry1.lods.items[0].triangle_batch.get() == null);
        try std.testing.expectEqual(@as(usize, 0), playback.geometries.find(2).?.value.get().?.lods.items.len);
        try std.testing.expectEqual(@as(usize, 2), playback.frames.items[0].geometries.items.len);
    }
}

/// The calls of the allocation failure sweep: the batches of `init`, a batch of each kind, a geometry, every draw call
/// and two frames. Returns true when `endFrame` reported an allocation failure.
fn recordWithFailures(allocator: Allocator, recorder: *DebugRendererRecorder) Allocator.Error!bool {
    const r = recorder.debugRenderer();
    const t = Triangle.init(Vec3.zero(), Vec3.axisX(), Vec3.axisY(), Color.white);
    var batch = r.createTriangleBatch(&.{ t, t });
    defer batch.deinit();
    var indexed = r.createTriangleBatchIndexed(&.{ t.v[0], t.v[1], t.v[2] }, &.{ 0, 1, 2 });
    defer indexed.deinit();

    // The geometry belongs to the test (std.testing.allocator), its batches to the recorder
    var geometry = GeometryRef.init(try Geometry.create(allocator, AABox.init(Vec3.zero(), Vec3.one())));
    defer geometry.deinit();
    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = batch.clone(), .distance = 5 });
    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = indexed.clone(), .distance = 10 });

    var failed = false;
    r.drawLine(RVec3.zero(), RVec3.init(1, 2, 3), Color.red);
    r.drawTriangle(RVec3.zero(), RVec3.init(1, 0, 0), RVec3.init(0, 1, 0), Color.green, .{ .cast_shadow = .on });
    r.drawText3D(RVec3.init(1, 1, 1), "Hello", .{});
    r.drawBox(AABox.init(Vec3.zero(), Vec3.one()), Color.blue, .{ .draw_mode = .wireframe });
    recorder.endFrame() catch {
        failed = true;
    };
    r.drawGeometryAutoBounds(RMat44.identity(), Color.white, geometry.get().?, .{});
    r.drawText3D(RVec3.zero(), "World", .{});
    recorder.endFrame() catch {
        failed = true;
    };
    return failed;
}

test "DebugRendererRecorder: allocation failures" {
    // Fail each allocation in turn: init returns error.OutOfMemory, or the draw call drops its primitive and endFrame
    // reports the latch (R5). Nothing leaks, and the stream that the recorder wrote stays readable.
    const allocator = std.testing.allocator;
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        var wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var recorder: DebugRendererRecorder = undefined;
        recorder.init(failing.allocator(), std.testing.io, wrapper.streamOut()) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        const failed = recordWithFailures(allocator, &recorder) catch |err| {
            recorder.deinit();
            return err;
        };
        recorder.deinit();
        try std.testing.expectEqual(failing.has_induced_failure, failed);

        var renderer: NullRenderer = .{ .base = .init(NullRenderer, allocator, std.testing.io) };
        try renderer.base.initialize();
        defer renderer.base.deinit();
        var playback = DebugRendererPlayback.init(allocator, &renderer.base);
        defer playback.deinit();
        try parseBytes(&playback, out.written());
        try std.testing.expectEqual(@as(u32, 2), playback.getNumFrames());
        for (0..playback.getNumFrames()) |f| playback.drawFrame(@intCast(f));
        if (!failing.has_induced_failure) break;
    }
    try std.testing.expect(fail_index > 0);
}
