//! Port of: Jolt/Renderer/DebugRendererPlayback.h, Jolt/Renderer/DebugRendererPlayback.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R3: a plain struct (Jolt's class has no virtual functions). It reads a
//! `.jor` stream of DebugRendererRecorder and replays it into any DebugRenderer: lines, then triangles, then texts,
//! then geometries per frame, like Jolt.
//! - `init(allocator, renderer)` is Jolt's constructor, `deinit` its destructor. The maps are Zolt's port of Jolt's
//!   UnorderedMap (only searched; `deinit` releases in Jolt's bucket order).
//! - Where Jolt's behavior is undefined, Zolt has one defined behavior (UB7 of R20). Jolt's defined cases are kept:
//!   - a LOD looks its batch ID up like Jolt. Jolt reads `find()->second` of `end()` for an ID that the stream does
//!     not define. Zolt makes that `error.InvalidStream`, except for ID 0, which is an empty `Batch` (Jolt's recorder
//!     writes 0 for a batch without triangles);
//!   - a stream that ends between two commands is complete (Jolt's loop ends there). A stream that ends inside a
//!     command is `error.InvalidStream`. Inside a create command Jolt reads uninitialized locals. Inside an EndFrame
//!     Jolt keeps the frame, but the elements it did not read stay uninitialized. Zolt keeps the frame too, with the
//!     lines, triangles, texts and geometries read in full;
//!   - on an error, what was read before stays, like in Jolt: the frames, batches and geometries;
//!   - `drawFrame` skips a geometry ID that is not defined (Jolt reads `find()->second` of `end()` there). Jolt looks
//!     the ID up only when it draws, so a later `parse` call can still define it;
//!   - an unknown command is Jolt's `JPH_ASSERT(false)`, a recoverable assert (R8): it panics with asserts unless an
//!     assert handler accepts it, then Jolt's release behavior follows (the next byte is read as a command).
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
    /// The errors of `parse`: out of memory, or a stream that Jolt's playback cannot read (UB7). What was read before
    /// the error stays.
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

                    // Zolt: an ID that the stream does not define is an invalid stream, except ID 0, which is an
                    // empty batch (UB7)
                    var batch: Batch = .empty;
                    if (self.batches.find(batch_id)) |kv|
                        batch = kv.value.clone()
                    else if (batch_id != 0)
                        return error.InvalidStream;
                    errdefer batch.deinit();

                    try geometry.lods.append(self.allocator, .{ .triangle_batch = batch, .distance = distance });
                }
            } else if (command == @intFromEnum(Command.end_frame)) {
                const frame = try self.frames.addOne(self.allocator);
                frame.* = .{};

                // Read all lines (Zolt: a stream that ends inside an element keeps the elements before it)
                var num_lines: u32 = 0;
                stream.read(&num_lines);
                try checkStream(stream);
                for (try frame.lines.addManyAsSlice(self.allocator, num_lines), 0..) |*line, i| {
                    stream.read(&line.from);
                    stream.read(&line.to);
                    stream.read(&line.color);
                    if (streamEnded(stream)) return cutElements(&frame.lines, i);
                }

                // Read all triangles
                var num_triangles: u32 = 0;
                stream.read(&num_triangles);
                try checkStream(stream);
                for (try frame.triangles.addManyAsSlice(self.allocator, num_triangles), 0..) |*triangle, i| {
                    stream.read(&triangle.v1);
                    stream.read(&triangle.v2);
                    stream.read(&triangle.v3);
                    stream.read(&triangle.color);
                    triangle.cast_shadow = @enumFromInt(readInt(stream));
                    if (streamEnded(stream)) return cutElements(&frame.triangles, i);
                }

                // Read all texts
                var num_texts: u32 = 0;
                stream.read(&num_texts);
                try checkStream(stream);
                try frame.texts.ensureUnusedCapacity(self.allocator, num_texts);
                for (0..num_texts) |_| {
                    var text: DebugRendererRecorder.TextBlob = .{ .position = undefined, .string = &.{}, .color = undefined, .height = 0 };
                    stream.read(&text.position);
                    try stream.readString(self.allocator, &text.string);
                    stream.read(&text.color);
                    stream.read(&text.height);
                    if (streamEnded(stream)) {
                        self.allocator.free(text.string);
                        return error.InvalidStream;
                    }
                    frame.texts.appendAssumeCapacity(text);
                }

                // Read all geometries
                var num_geometries: u32 = 0;
                stream.read(&num_geometries);
                try checkStream(stream);
                for (try frame.geometries.addManyAsSlice(self.allocator, num_geometries), 0..) |*geom, i| {
                    stream.read(&geom.model_matrix);
                    stream.read(&geom.model_color);
                    stream.read(&geom.geometry_id);
                    geom.cull_mode = @enumFromInt(readInt(stream));
                    geom.cast_shadow = @enumFromInt(readInt(stream));
                    geom.draw_mode = @enumFromInt(readInt(stream));
                    if (streamEnded(stream)) return cutElements(&frame.geometries, i);
                }
            } else {
                // JPH_ASSERT(false): an unknown command. Jolt's release build reads the next byte as a command. The assert
                // is recoverable (R8): the test of Jolt's release behavior expects it.
                if (Core.enable_asserts) Core.assertFailed("false", @src());
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
            // Zolt: an unknown geometry ID is skipped (UB7, see the file header)
            const kv = self.geometries.find(geom.geometry_id) orelse continue;
            self.renderer.drawGeometryAutoBounds(geom.model_matrix, geom.model_color, kv.value.get().?, .{ .cull_mode = geom.cull_mode, .cast_shadow = geom.cast_shadow, .draw_mode = geom.draw_mode });
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
        if (streamEnded(stream))
            return error.InvalidStream;
    }

    fn streamEnded(stream: StreamIn) bool {
        return stream.isEOF() or stream.isFailed();
    }

    /// Zolt: the stream ended inside element `num_read` of a frame. The frame keeps the elements before it.
    fn cutElements(list: anytype, num_read: usize) error{InvalidStream} {
        list.shrinkRetainingCapacity(num_read);
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
};

const Color = @import("../Core/Color.zig").Color;
const IssueReporting = @import("../Core/IssueReporting.zig");
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

/// Two frames written by Zolt's recorder (the recorder's bytes equal Jolt's, see the debug renderer parity). Of the 25
/// predefined batches that the recorder writes first, the stream keeps only the box (the first one, the frames draw
/// it), so that the tests below stay small.
fn recordTestStream(allocator: Allocator, out: *std.Io.Writer.Allocating) !void {
    const start = out.written().len;
    var wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    var recorder: DebugRendererRecorder = undefined;
    try recorder.init(allocator, std.testing.io, wrapper.streamOut());
    defer recorder.deinit();
    const r = recorder.debugRenderer();

    // Drop the predefined batches after the box: create batch command, ID, vertex count, vertices, index count, indices
    const box_len = 1 + 4 + 4 + 24 * @sizeOf(Vertex) + 4 + 36 * 4;
    out.shrinkRetainingCapacity(start + box_len);

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
    try std.testing.expectEqual(@as(u32, 25), renderer.num_batches); // The predefined geometry of initialize

    var playback = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback.deinit();
    try parseBytes(&playback, out.written());
    try std.testing.expectEqual(@as(u32, 2), playback.getNumFrames());
    try std.testing.expectEqual(@as(u32, 27), renderer.num_batches); // The box of the recorder and the LOD batch

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
    var prev_num_frames: u32 = 0;
    for (0..bytes.len + 1) |len| {
        var playback = DebugRendererPlayback.init(allocator, &renderer.base);
        defer playback.deinit();
        if (parseBytes(&playback, bytes[0..len])) {
            num_complete += 1;
        } else |err| {
            try std.testing.expectEqual(error.InvalidStream, err);
        }

        // The frames read before the end of the stream stay, like in Jolt
        try std.testing.expect(playback.getNumFrames() >= prev_num_frames);
        prev_num_frames = playback.getNumFrames();
        for (0..playback.getNumFrames()) |i| playback.drawFrame(@intCast(i));
    }

    // Empty, after the box batch, after the LOD batch, after each geometry, after each frame
    try std.testing.expectEqual(@as(u32, 7), num_complete);
    try std.testing.expectEqual(@as(u32, 2), prev_num_frames);
}

test "DebugRendererPlayback: a stream cut short keeps the frames and the frame elements read in full" {
    const allocator = std.testing.allocator;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    const s = wrapper.streamOut();

    // A stream written by hand: a batch, a geometry and two frames. `boundaries` holds the offsets between two
    // commands, `frame_starts` the offset of each EndFrame command and `ends` the offset after each line, triangle,
    // text and geometry.
    var boundaries: std.ArrayList(usize) = .empty;
    defer boundaries.deinit(allocator);
    var frame_starts: std.ArrayList(usize) = .empty;
    defer frame_starts.deinit(allocator);
    var ends: [4]std.ArrayList(usize) = @splat(.empty);
    defer for (&ends) |*e| e.deinit(allocator);

    const t = Triangle.init(Vec3.zero(), Vec3.axisX(), Vec3.axisY(), Color.white);
    try boundaries.append(allocator, 0);
    writeBatch(s, 7, &.{t});
    try boundaries.append(allocator, out.written().len);
    writeGeometry(s, 1, &.{7});
    try boundaries.append(allocator, out.written().len);
    for (0..2) |_| {
        try frame_starts.append(allocator, out.written().len);
        s.write(@intFromEnum(Command.end_frame));
        s.write(@as(u32, 2));
        for (0..2) |_| {
            s.write(RVec3.zero());
            s.write(RVec3.init(1, 2, 3));
            s.write(Color.red);
            try ends[0].append(allocator, out.written().len);
        }
        s.write(@as(u32, 1));
        s.write(RVec3.zero());
        s.write(RVec3.init(1, 0, 0));
        s.write(RVec3.init(0, 1, 0));
        s.write(Color.green);
        s.write(@as(i32, 1));
        try ends[1].append(allocator, out.written().len);
        s.write(@as(u32, 1));
        s.write(RVec3.zero());
        s.writeString("Text");
        s.write(Color.white);
        s.write(@as(f32, 0.5));
        try ends[2].append(allocator, out.written().len);
        s.write(@as(u32, 1));
        s.write(RMat44.identity());
        s.write(Color.white);
        s.write(@as(u32, 1));
        for (0..3) |_| s.write(@as(i32, 0));
        try ends[3].append(allocator, out.written().len);
        try boundaries.append(allocator, out.written().len);
    }
    const bytes = out.written();

    var renderer: CountingRenderer = undefined;
    try renderer.init(allocator);
    defer renderer.base.deinit();

    for (0..bytes.len + 1) |len| {
        var playback = DebugRendererPlayback.init(allocator, &renderer.base);
        defer playback.deinit();
        const result = parseBytes(&playback, bytes[0..len]);
        if (std.mem.indexOfScalar(usize, boundaries.items, len) != null)
            try result
        else
            try std.testing.expectError(error.InvalidStream, result);

        // Like Jolt, every EndFrame command that was read adds a frame
        var num_frames: u32 = 0;
        for (frame_starts.items) |start| num_frames += @intFromBool(start < len);
        try std.testing.expectEqual(num_frames, playback.getNumFrames());

        // A frame cut short draws the elements read in full (Jolt's other elements are uninitialized)
        var expected: [4]u32 = @splat(0);
        for (&ends, &expected) |e, *n| {
            for (e.items) |end| n.* += @intFromBool(end <= len);
        }
        renderer.num_lines = 0;
        renderer.num_triangles = 0;
        renderer.num_texts = 0;
        renderer.num_geometries = 0;
        for (0..playback.getNumFrames()) |i| playback.drawFrame(@intCast(i));
        try std.testing.expectEqual(expected, [4]u32{ renderer.num_lines, renderer.num_triangles, renderer.num_texts, renderer.num_geometries });
    }
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

    // A frame that draws a geometry that the stream never defines: drawFrame skips it (Jolt looks the ID up there, UB7)
    writeFrame(s, 2, 0);
    var playback2 = DebugRendererPlayback.init(allocator, &renderer.base);
    defer playback2.deinit();
    try parseBytes(&playback2, out.written());
    try std.testing.expectEqual(@as(u32, 2), playback2.getNumFrames());
    const num_geometries = renderer.num_geometries;
    playback2.drawFrame(1);
    try std.testing.expectEqual(num_geometries, renderer.num_geometries);

    // A later parse call defines the geometry: now the frame draws it (Jolt's behavior is defined here)
    {
        var out2: std.Io.Writer.Allocating = .init(allocator);
        defer out2.deinit();
        var wrapper2: StreamWrapper.StreamOutWrapper = .init(&out2.writer);
        writeGeometry(wrapper2.streamOut(), 2, &.{});
        try parseBytes(&playback2, out2.written());
        playback2.drawFrame(1);
        try std.testing.expectEqual(num_geometries + 1, renderer.num_geometries);
    }

    // A batch ID 0 that the stream defines is used, like in Jolt (Jolt's recorder never writes it, a hand-made stream
    // can)
    {
        var out2: std.Io.Writer.Allocating = .init(allocator);
        defer out2.deinit();
        var wrapper2: StreamWrapper.StreamOutWrapper = .init(&out2.writer);
        const s2 = wrapper2.streamOut();
        writeBatch(s2, 0, &.{ t, t });
        writeGeometry(s2, 1, &.{0});
        writeFrame(s2, 1, 0);
        var playback0 = DebugRendererPlayback.init(allocator, &renderer.base);
        defer playback0.deinit();
        try parseBytes(&playback0, out2.written());
        const num_lod_triangles = renderer.num_lod_triangles;
        playback0.drawFrame(0);
        try std.testing.expectEqual(num_lod_triangles + 2, renderer.num_lod_triangles);
    }

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
    // JPH_ASSERT(false) once, then the next byte is read as a command
    var expect_assert: IssueReporting.ExpectAssert = undefined;
    expect_assert.init(1);
    errdefer expect_assert.restore();

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
    try expect_assert.finish();
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
