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
                // An empty batch only exists after an allocation failure (latched), Zolt writes ID 0 for it
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
