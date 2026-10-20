//! Port of: Jolt/Renderer/DebugRendererPlayback.h, Jolt/Renderer/DebugRendererPlayback.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R3: a plain struct (Jolt's class has no virtual functions). It reads a
//! `.jor` stream of DebugRendererRecorder and replays it into any DebugRenderer: lines, then triangles, then texts,
//! then geometries per frame, like Jolt.
//! - `init(allocator, renderer)` is Jolt's constructor, `deinit` its destructor. The maps are Zolt's port of Jolt's
//!   UnorderedMap (only searched; `deinit` releases in Jolt's bucket order).
//! - `parse` returns an error where Jolt's behavior is undefined (UB7 of R20). Jolt's well-defined cases are kept:
//!   - a batch ID 0 in a geometry LOD is an empty `Batch` (Jolt's recorder writes 0 for a batch without triangles,
//!     Jolt's playback then reads `find(0)->second` of `end()`); an unknown batch ID is `error.InvalidStream`;
//!   - a frame that draws a geometry ID that the stream never defines is `error.InvalidStream`. Jolt looks the ID up
//!     when it draws the frame, so the check runs at the end of `parse`, over the frames of that call;
//!   - a stream that ends between two commands is complete (Jolt's loop ends there). A stream that ends inside a
//!     command is `error.InvalidStream` (Jolt reads uninitialized locals there);
//!   - on an error, the frames of that `parse` call are removed again. The batches and geometries it created stay in
//!     the maps (they are released by `deinit`);
//!   - an unknown command is Jolt's `JPH_ASSERT(false)`: a panic with asserts, then Jolt's release behavior (the next
//!     byte is read as a command).
//! - A second batch with an ID that exists already is released (Jolt: `insert` keeps the first one). A second
//!   geometry with an ID that exists already replaces the first one (Jolt: `operator []`).
//! - The cast shadow, cull and draw modes are passed on as read, like Jolt (the enums accept every value).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const StreamIn = @import("../Core/StreamIn.zig").StreamIn;
const UnorderedMap = @import("../Core/UnorderedMap.zig").UnorderedMap;
const AABox = @import("../Geometry/AABox.zig").AABox;
const DebugRenderer = @import("DebugRenderer.zig").DebugRenderer;
const DebugRendererRecorder = @import("DebugRendererRecorder.zig").DebugRendererRecorder;
const Batch = DebugRenderer.Batch;
const Geometry = DebugRenderer.Geometry;
const GeometryRef = DebugRenderer.GeometryRef;
const Triangle = DebugRenderer.Triangle;
const Vertex = DebugRenderer.Vertex;
const Command = DebugRendererRecorder.Command;

/// Class that can read a recorded stream from DebugRendererRecorder and plays it back trough a DebugRenderer
pub const DebugRendererPlayback = struct {
    /// The errors of `parse`: out of memory, or a stream that Jolt's playback cannot read (UB7)
    pub const Error = Allocator.Error || error{InvalidStream};

    /// The list of parsed frames
    pub const Frame = DebugRendererRecorder.Frame;

    /// Allocates the frames and the maps (the renderer allocates the batches)
    allocator: Allocator,

    /// The debug renderer we're using to do the actual rendering
    renderer: *DebugRenderer,

    /// Mapping of ID to batch
    batches: UnorderedMap(u32, Batch, .{}) = .empty,

    /// Mapping of ID to geometry
    geometries: UnorderedMap(u32, GeometryRef, .{}) = .empty,

    /// The list of parsed frames
    frames: std.ArrayList(Frame) = .empty,

    /// Constructor
    pub fn init(allocator: Allocator, renderer: *DebugRenderer) DebugRendererPlayback {
        return .{ .allocator = allocator, .renderer = renderer };
    }

    /// Destructor (members in Jolt's reverse declaration order: frames, geometries, batches)
    pub fn deinit(self: *DebugRendererPlayback) void {
        for (self.frames.items) |*frame| self.freeFrame(frame);
        self.frames.deinit(self.allocator);

        var geometry_it = self.geometries.iterator();
        while (geometry_it.next()) |kv| kv.value.deinit();
        self.geometries.deinit(self.allocator);

        var batch_it = self.batches.iterator();
        while (batch_it.next()) |kv| kv.value.deinit();
        self.batches.deinit(self.allocator);
    }

    /// Parse a stream of frames
    pub fn parse(self: *DebugRendererPlayback, stream: StreamIn) Error!void {
        // Zolt: an error removes the frames of this call again (see the file header)
        const first_frame = self.frames.items.len;
        errdefer self.removeFrames(first_frame);

        while (true) {
            // Read the next command
            var command: u8 = 0;
            stream.read(&command);

            if (stream.isEOF() or stream.isFailed())
                break;

            if (command == @intFromEnum(Command.create_batch)) {
                var id: u32 = 0;
                stream.read(&id);

                var triangle_count: u32 = 0;
                stream.read(&triangle_count);
                try checkStream(stream);

                const triangles = try self.allocator.alloc(Triangle, triangle_count);
                defer self.allocator.free(triangles);
                stream.readBytes(std.mem.sliceAsBytes(triangles));
                try checkStream(stream);

                try self.insertBatch(id, self.renderer.createTriangleBatch(triangles));
            } else if (command == @intFromEnum(Command.create_batch_indexed)) {
                var id: u32 = 0;
                stream.read(&id);

                var vertex_count: u32 = 0;
                stream.read(&vertex_count);
                try checkStream(stream);

                const vertices = try self.allocator.alloc(Vertex, vertex_count);
                defer self.allocator.free(vertices);
                stream.readBytes(std.mem.sliceAsBytes(vertices));

                var index_count: u32 = 0;
                stream.read(&index_count);
                try checkStream(stream);

                const indices = try self.allocator.alloc(u32, index_count);
                defer self.allocator.free(indices);
                stream.readBytes(std.mem.sliceAsBytes(indices));
                try checkStream(stream);

                try self.insertBatch(id, self.renderer.createTriangleBatchIndexed(vertices, indices));
            } else if (command == @intFromEnum(Command.create_geometry)) {
                var geometry_id: u32 = 0;
                stream.read(&geometry_id);

                var bounds: AABox = .empty;
                stream.read(&bounds.min);
                stream.read(&bounds.max);
                try checkStream(stream);

                const geometry = try Geometry.create(self.allocator, bounds);
                var geometry_ref = GeometryRef.init(geometry);
                defer geometry_ref.deinit();
                (try self.geometries.getOrPutValue(self.allocator, geometry_id, .empty)).set(geometry);

                var num_lods: u32 = 0;
                stream.read(&num_lods);
                try checkStream(stream);
                for (0..num_lods) |_| {
                    var distance: f32 = 0;
                    stream.read(&distance);

                    var batch_id: u32 = 0;
                    stream.read(&batch_id);
                    try checkStream(stream);

                    // Zolt: ID 0 is an empty batch, an unknown ID is an invalid stream (UB7)
                    var batch: Batch = .empty;
                    if (batch_id != 0) {
                        const kv = self.batches.find(batch_id) orelse return error.InvalidStream;
                        batch = kv.value.clone();
                    }
                    errdefer batch.deinit();

                    try geometry.lods.append(self.allocator, .{ .triangle_batch = batch, .distance = distance });
                }
            } else if (command == @intFromEnum(Command.end_frame)) {
                const frame = try self.frames.addOne(self.allocator);
                frame.* = .{};

                // Read all lines
                var num_lines: u32 = 0;
                stream.read(&num_lines);
                try checkStream(stream);
                for (try frame.lines.addManyAsSlice(self.allocator, num_lines)) |*line| {
                    stream.read(&line.from);
                    stream.read(&line.to);
                    stream.read(&line.color);
                }

                // Read all triangles
                var num_triangles: u32 = 0;
                stream.read(&num_triangles);
                try checkStream(stream);
                for (try frame.triangles.addManyAsSlice(self.allocator, num_triangles)) |*triangle| {
                    stream.read(&triangle.v1);
                    stream.read(&triangle.v2);
                    stream.read(&triangle.v3);
                    stream.read(&triangle.color);
                    triangle.cast_shadow = @enumFromInt(readInt(stream));
                }

                // Read all texts
                var num_texts: u32 = 0;
                stream.read(&num_texts);
                try checkStream(stream);
                try frame.texts.ensureUnusedCapacity(self.allocator, num_texts);
                for (0..num_texts) |_| {
                    const text = frame.texts.addOneAssumeCapacity();
                    text.* = .{ .position = undefined, .string = &.{}, .color = undefined, .height = 0 };
                    stream.read(&text.position);
                    try stream.readString(self.allocator, &text.string);
                    stream.read(&text.color);
                    stream.read(&text.height);
                }

                // Read all geometries
                var num_geometries: u32 = 0;
                stream.read(&num_geometries);
                try checkStream(stream);
                for (try frame.geometries.addManyAsSlice(self.allocator, num_geometries)) |*geom| {
                    stream.read(&geom.model_matrix);
                    stream.read(&geom.model_color);
                    stream.read(&geom.geometry_id);
                    geom.cull_mode = @enumFromInt(readInt(stream));
                    geom.cast_shadow = @enumFromInt(readInt(stream));
                    geom.draw_mode = @enumFromInt(readInt(stream));
                }
                try checkStream(stream);
            } else {
                // JPH_ASSERT(false): an unknown command. Jolt's release build reads the next byte as a command.
                if (Core.enable_asserts) @panic("DebugRendererPlayback: unknown command in the stream");
            }
        }

        // Zolt: every geometry that the new frames draw must exist (Jolt looks them up in DrawFrame)
        for (self.frames.items[first_frame..]) |*frame| {
            for (frame.geometries.items) |*geom| {
                if (self.geometries.find(geom.geometry_id) == null)
                    return error.InvalidStream;
            }
        }
    }

    /// Get the number of parsed frames
    pub fn getNumFrames(self: *const DebugRendererPlayback) u32 {
        return @intCast(self.frames.items.len);
    }

    /// Draw a frame
    pub fn drawFrame(self: *const DebugRendererPlayback, frame_number: u32) void {
        const frame = &self.frames.items[frame_number];

        for (frame.lines.items) |*line|
            self.renderer.drawLine(line.from, line.to, line.color);

        for (frame.triangles.items) |*triangle|
            self.renderer.drawTriangle(triangle.v1, triangle.v2, triangle.v3, triangle.color, .{ .cast_shadow = triangle.cast_shadow });

        for (frame.texts.items) |*text|
            self.renderer.drawText3D(text.position, text.string, .{ .color = text.color, .height = text.height });

        for (frame.geometries.items) |*geom| {
            const geometry = self.geometries.find(geom.geometry_id).?.value.get().?; // Checked by parse
            self.renderer.drawGeometryAutoBounds(geom.model_matrix, geom.model_color, geometry, .{ .cull_mode = geom.cull_mode, .cast_shadow = geom.cast_shadow, .draw_mode = geom.draw_mode });
        }
    }

    /// mBatches.insert({ id, batch }): keeps the first batch of an ID, the new one is released then
    fn insertBatch(self: *DebugRendererPlayback, id: u32, batch: Batch) Allocator.Error!void {
        var b = batch;
        const result = self.batches.tryEmplace(self.allocator, id, b) catch |err| {
            b.deinit();
            return err;
        };
        if (!result.inserted)
            b.deinit();
    }

    /// Zolt: the stream ended or failed inside a command
    fn checkStream(stream: StreamIn) error{InvalidStream}!void {
        if (stream.isEOF() or stream.isFailed())
            return error.InvalidStream;
    }

    /// An enum that Jolt writes as `int`
    fn readInt(stream: StreamIn) i32 {
        var value: i32 = 0;
        stream.read(&value);
        return value;
    }

    fn freeFrame(self: *DebugRendererPlayback, frame: *Frame) void {
        frame.lines.deinit(self.allocator);
        frame.triangles.deinit(self.allocator);
        for (frame.texts.items) |text| self.allocator.free(text.string);
        frame.texts.deinit(self.allocator);
        frame.geometries.deinit(self.allocator);
    }

    /// Remove the frames from `first` on (the frames of a parse call that failed)
    fn removeFrames(self: *DebugRendererPlayback, first: usize) void {
        for (self.frames.items[first..]) |*frame| self.freeFrame(frame);
        self.frames.shrinkRetainingCapacity(first);
    }
};

const Color = @import("../Core/Color.zig").Color;
const StreamOut = @import("../Core/StreamOut.zig").StreamOut;
const StreamWrapper = @import("../Core/StreamWrapper.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const RealFile = @import("../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;

/// A renderer that counts what the playback draws
const CountingRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const BatchImpl = struct {
        pub const overrides = .{};
        base: DebugRenderer.BatchTarget,
        num_triangles: usize,
    };

    base: DebugRenderer,
    num_lines: u32 = 0,
    num_triangles: u32 = 0,
    num_batches: u32 = 0,
    num_geometries: u32 = 0,
    num_lod_triangles: usize = 0,
    texts: [4][]const u8 = undefined,
    num_texts: u32 = 0,
    last_cast_shadow: DebugRenderer.CastShadow = .on,
    last_draw_mode: DebugRenderer.DrawMode = .solid,

    fn init(self: *CountingRenderer, allocator: Allocator) Allocator.Error!void {
        self.* = .{ .base = .init(CountingRenderer, allocator, std.testing.io) };
        errdefer self.base.deinit();
        try self.base.initialize();
    }

    pub fn drawLine(self: *CountingRenderer, from: RVec3, to: RVec3, color: Color) void {
        _ = .{ from, to, color };
        self.num_lines += 1;
    }
    pub fn drawTriangle(self: *CountingRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow) void {
        _ = .{ v1, v2, v3, color };
        self.num_triangles += 1;
        self.last_cast_shadow = cast_shadow;
    }
    pub fn createTriangleBatch(self: *CountingRenderer, triangles: []const Triangle) Batch {
        return self.newBatch(triangles.len);
    }
    pub fn createTriangleBatchIndexed(self: *CountingRenderer, vertices: []const Vertex, indices: []const u32) Batch {
        _ = vertices;
        return self.newBatch(indices.len / 3);
    }
    pub fn drawGeometry(self: *CountingRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, cull_mode: DebugRenderer.CullMode, cast_shadow: DebugRenderer.CastShadow, draw_mode: DebugRenderer.DrawMode) void {
        _ = .{ model_matrix, world_space_bounds, lod_scale_sq, model_color, cull_mode, cast_shadow };
        self.num_geometries += 1;
        self.last_draw_mode = draw_mode;
        for (geometry.lods.items) |lod| {
            if (lod.triangle_batch.get()) |b| self.num_lod_triangles += @as(*const BatchImpl, @alignCast(@fieldParentPtr("base", b))).num_triangles;
        }
    }
    pub fn drawText3D(self: *CountingRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        _ = .{ position, color, height };
        self.texts[self.num_texts % self.texts.len] = string;
        self.num_texts += 1;
    }

    fn newBatch(self: *CountingRenderer, num_triangles: usize) Batch {
        const batch = self.base.allocator.create(BatchImpl) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(BatchImpl, self.base.allocator), .num_triangles = num_triangles };
        self.num_batches += 1;
        return .init(&batch.base);
    }
};

/// Two frames written by Zolt's recorder (the recorder's bytes equal Jolt's, see the debug renderer parity)
fn recordTestStream(allocator: Allocator, out: *std.Io.Writer.Allocating) !void {
    var wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    var recorder: DebugRendererRecorder = undefined;
    try recorder.init(allocator, std.testing.io, wrapper.streamOut());
    defer recorder.deinit();
    const r = recorder.debugRenderer();

    // A geometry with two LODs of a triangle list
    const t = Triangle.init(Vec3.zero(), Vec3.axisX(), Vec3.axisY(), Color.white);
    var lod0 = r.createTriangleBatch(&.{ t, t });
    defer lod0.deinit();
    var geometry = GeometryRef.init(try Geometry.create(allocator, AABox.init(Vec3.zero(), Vec3.one())));
    defer geometry.deinit();
    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = lod0.clone(), .distance = 5 });
    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = lod0.clone(), .distance = 10 });

    r.drawLine(RVec3.zero(), RVec3.init(1, 2, 3), Color.red);
    r.drawTriangle(RVec3.zero(), RVec3.init(1, 0, 0), RVec3.init(0, 1, 0), Color.green, .{ .cast_shadow = .on });
    r.drawText3D(RVec3.init(1, 1, 1), "Hello", .{});
    r.drawBox(AABox.init(Vec3.zero(), Vec3.one()), Color.blue, .{ .draw_mode = .wireframe });
    try recorder.endFrame();
    r.drawGeometryAutoBounds(RMat44.identity(), Color.white, geometry.get().?, .{});
    r.drawText3D(RVec3.zero(), "World", .{});
    try recorder.endFrame();
}

/// Parse `bytes` into `playback`
fn parseBytes(playback: *DebugRendererPlayback, bytes: []const u8) DebugRendererPlayback.Error!void {
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try playback.parse(wrapper.streamIn());
}

test "DebugRendererPlayback: replays the frames of a recorder" {
    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    try recordTestStream(allocator, &out);

    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    try std.testing.expectEqual(@as(u32, 1), renderer.num_batches); // The box of initialize

    var playback = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback.deinit();
    try parseBytes(&playback, out.written());
    try std.testing.expectEqual(@as(u32, 2), playback.getNumFrames());
    try std.testing.expectEqual(@as(u32, 3), renderer.num_batches); // The box of the recorder and the LOD batch

    playback.drawFrame(0);
    try std.testing.expectEqual(@as(u32, 1), renderer.num_lines);
    try std.testing.expectEqual(@as(u32, 1), renderer.num_triangles);
    try std.testing.expectEqual(DebugRenderer.CastShadow.on, renderer.last_cast_shadow);
    try std.testing.expectEqualStrings("Hello", renderer.texts[0]);
    try std.testing.expectEqual(@as(u32, 1), renderer.num_geometries);
    try std.testing.expectEqual(DebugRenderer.DrawMode.wireframe, renderer.last_draw_mode);
    try std.testing.expectEqual(@as(usize, 12), renderer.num_lod_triangles); // The box batch, as indexed triangles

    playback.drawFrame(1);
    try std.testing.expectEqual(@as(u32, 2), renderer.num_geometries);
    try std.testing.expectEqual(@as(usize, 12 + 2 * 2), renderer.num_lod_triangles); // 2 LODs of the same batch
    try std.testing.expectEqualStrings("World", renderer.texts[1]);

    // A second parse appends its frames
    try parseBytes(&playback, out.written());
    try std.testing.expectEqual(@as(u32, 4), playback.getNumFrames());
}

test "DebugRendererPlayback: a truncated stream is invalid unless it ends between two commands" {
    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    try recordTestStream(allocator, &out);
    const bytes = out.written();

    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();

    var num_complete: u32 = 0;
    for (0..bytes.len + 1) |len| {
        var playback = DebugRendererPlayback.init(allocator, &renderer.base);
        defer playback.deinit();
        if (parseBytes(&playback, bytes[0..len])) {
            num_complete += 1;
            for (0..playback.getNumFrames()) |i| playback.drawFrame(@intCast(i));
        } else |err| {
            try std.testing.expectEqual(error.InvalidStream, err);
            try std.testing.expectEqual(@as(u32, 0), playback.getNumFrames()); // The frames of the call are removed
        }
    }

    // Empty, after the box batch, after the LOD batch, after each geometry, after each frame
    try std.testing.expectEqual(@as(u32, 7), num_complete);
}

fn writeBatch(stream: StreamOut, id: u32, triangles: []const Triangle) void {
    stream.write(@intFromEnum(Command.create_batch));
    stream.write(id);
    stream.write(@as(u32, @intCast(triangles.len)));
    stream.writeBytes(std.mem.sliceAsBytes(triangles));
}

fn writeGeometry(stream: StreamOut, id: u32, batch_ids: []const u32) void {
    stream.write(@intFromEnum(Command.create_geometry));
    stream.write(id);
    stream.write(Vec3.zero());
    stream.write(Vec3.one());
    stream.write(@as(u32, @intCast(batch_ids.len)));
    for (batch_ids) |b| {
        stream.write(@as(f32, 1));
        stream.write(b);
    }
}

fn writeFrame(stream: StreamOut, geometry_id: u32, draw_mode: i32) void {
    stream.write(@intFromEnum(Command.end_frame));
    stream.write(@as(u32, 0)); // Lines
    stream.write(@as(u32, 0)); // Triangles
    stream.write(@as(u32, 0)); // Texts
    stream.write(@as(u32, 1)); // Geometries
    stream.write(RMat44.identity());
    stream.write(Color.white);
    stream.write(geometry_id);
    stream.write(@as(i32, 0));
    stream.write(@as(i32, 0));
    stream.write(draw_mode);
}

test "DebugRendererPlayback: UB7, unknown IDs and ID 0; Jolt's insert and operator [] for duplicate IDs" {
    const allocator = std.testing.allocator;
    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();

    // A stream written by hand: batch 7 twice (Jolt's insert keeps the first one), geometry 1 twice (operator []
    // replaces it) with LODs of batch 0 (an empty batch) and 7, and a frame that draws geometry 1
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    const s = wrapper.streamOut();
    const t = Triangle.init(Vec3.zero(), Vec3.axisX(), Vec3.axisY(), Color.white);
    writeBatch(s, 7, &.{t});
    writeBatch(s, 7, &.{ t, t, t });
    writeGeometry(s, 1, &.{7});
    writeGeometry(s, 1, &.{ 0, 7 });
    writeFrame(s, 1, 5); // A draw mode that Jolt's enum does not name is passed on
    const valid_len = out.written().len;

    var playback = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback.deinit();
    try parseBytes(&playback, out.written());
    playback.drawFrame(0);
    try std.testing.expectEqual(@as(usize, 1), renderer.num_lod_triangles); // LOD 0 is empty, LOD 1 is the first batch 7
    try std.testing.expectEqual(@as(i32, 5), @intFromEnum(renderer.last_draw_mode));

    // A frame that draws a geometry that the stream never defines
    writeFrame(s, 2, 0);
    var playback2 = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback2.deinit();
    try std.testing.expectError(error.InvalidStream, parseBytes(&playback2, out.written()));
    try std.testing.expectEqual(@as(u32, 0), playback2.getNumFrames());

    // A frame that draws a geometry that the stream defines later is valid (Jolt looks it up in DrawFrame)
    out.shrinkRetainingCapacity(valid_len);
    writeFrame(s, 2, 0);
    writeGeometry(s, 2, &.{7});
    var playback3 = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback3.deinit();
    try parseBytes(&playback3, out.written());
    try std.testing.expectEqual(@as(u32, 2), playback3.getNumFrames());
    playback3.drawFrame(1);

    // A LOD of an unknown batch
    out.shrinkRetainingCapacity(valid_len);
    writeGeometry(s, 3, &.{8});
    var playback4 = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback4.deinit();
    try std.testing.expectError(error.InvalidStream, parseBytes(&playback4, out.written()));
}

test "DebugRendererPlayback: an unknown command in Jolt's release build" {
    if (Core.enable_asserts) return error.SkipZigTest; // JPH_ASSERT(false) panics with asserts

    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    try out.writer.writeByte(42); // Not a command: the next byte is read as a command
    try recordTestStream(allocator, &out);

    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();
    var playback = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback.deinit();
    try parseBytes(&playback, out.written());
    try std.testing.expectEqual(@as(u32, 2), playback.getNumFrames());
}

test "DebugRendererPlayback: allocation failures" {
    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    try recordTestStream(allocator, &out);

    const Test = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            var renderer: CountingRenderer = undefined;
            try renderer.init(a);
            defer renderer.base.deinit();
            var playback = DebugRendererPlayback.init(a, &renderer.base);
            defer playback.deinit();
            try parseBytes(&playback, bytes);
            if (renderer.base.isOutOfMemory()) return error.OutOfMemory; // A batch was dropped
            for (0..playback.getNumFrames()) |i| playback.drawFrame(@intCast(i));
        }
    };
    try std.testing.checkAllAllocationFailures(allocator, Test.run, .{out.written()});
}
