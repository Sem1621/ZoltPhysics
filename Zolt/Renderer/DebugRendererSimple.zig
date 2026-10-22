//! Port of: Jolt/Renderer/DebugRendererSimple.h, Jolt/Renderer/DebugRendererSimple.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R3:
//! - `DebugRendererSimple` is an abstract level of the pattern A root DebugRenderer (`base: DebugRenderer`, the bodies
//!   of its overrides in `impl`). It implements the batches as triangle lists and `drawGeometry` with Jolt's LOD
//!   selection (LOD 0 until `setCameraPos`). The implementation embeds it as `base` and lists its own overrides,
//!   at least `drawLine` and `drawText3D`.
//! - It takes no lock, like Jolt.
//! - `init` is Jolt's constructor (`DebugRendererSimple()` calls Initialize). It initializes in place, because the
//!   predefined geometry is created through the vtable of the implementation. A difference from C++: during `init` an
//!   implementation's own override of `createTriangleBatch` is called already (C++ calls DebugRendererSimple's while its
//!   constructor runs).
//! - Allocation failure (R5): a batch that cannot be allocated or whose triangles cannot be copied is dropped: the
//!   function returns an empty `Batch` and latches `out_of_memory`.
//! - Zolt (UB34 of R20): `drawGeometry` draws nothing for an empty `Batch` (the batch of an allocation failure, ID 0 of
//!   DebugRendererPlayback, or `Geometry.createWithBatch(allocator, .empty, ...)`) and for a geometry without LODs. Jolt
//!   dereferences the null batch, or `mLODs.data()` of an empty list.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../Core/Color.zig").Color;
const AABox = @import("../Geometry/AABox.zig").AABox;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
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
const GeometryRef = DebugRenderer.GeometryRef;
const Triangle = DebugRenderer.Triangle;
const Vertex = DebugRenderer.Vertex;

/// Inherit from this class to simplify implementing a debug renderer, start with this implementation:
///
///     const MyDebugRenderer = struct {
///         pub const overrides = .{ .drawLine, .drawTriangle, .drawText3D };
///         base: zolt.DebugRendererSimple,
///
///         pub fn drawLine(self: *MyDebugRenderer, from: RVec3, to: RVec3, color: Color) void {
///             // Implement
///         }
///
///         pub fn drawTriangle(self: *MyDebugRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow) void {
///             // Implement
///         }
///
///         pub fn drawText3D(self: *MyDebugRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
///             // Implement
///         }
///     };
///
///     var renderer: MyDebugRenderer = undefined;
///     try renderer.base.init(MyDebugRenderer, allocator, io);
///     defer renderer.base.base.deinit();
///
/// Note that this class is meant to be a quick start for implementing a debug renderer, it is not the most efficient way to implement a debug renderer.
pub const DebugRendererSimple = struct {
    base: DebugRenderer,

    /// Last provided camera position (Jolt leaves it uninitialized until SetCameraPos)
    camera_pos: RVec3 = RVec3.zero(),
    camera_pos_set: bool = false,

    /// Implementation specific batch object
    const BatchImpl = struct {
        pub const overrides = .{};

        base: BatchTarget,
        triangles: std.ArrayList(Triangle) = .empty,

        pub fn destruct(self: *BatchImpl) void {
            self.triangles.deinit(self.base.allocator);
        }
    };

    /// Constructor (DebugRendererSimple()), called by the implementation `T` with its own type: creates the predefined
    /// geometry (Initialize). In place: `self` must be the `base` of a `T` at its final address, and it must not be
    /// moved afterwards (the renderer is handed out as `*DebugRenderer`). On error nothing is left allocated.
    pub fn init(self: *DebugRendererSimple, comptime T: type, allocator: Allocator, io: std.Io) Allocator.Error!void {
        self.* = .{ .base = .init(T, allocator, io) };
        errdefer self.base.destruct(); // Only what this level created: the implementation is not constructed yet
        try self.base.initialize();
    }

    /// The renderer as a DebugRenderer
    pub fn debugRenderer(self: *DebugRendererSimple) *DebugRenderer {
        return &self.base;
    }

    /// Should be called every frame by the application to provide the camera position.
    /// This is used to determine the correct LOD for rendering.
    pub fn setCameraPos(self: *DebugRendererSimple, camera_pos: RVec3) void {
        self.camera_pos = camera_pos;
        self.camera_pos_set = true;
    }

    /// New BatchImpl, null when out of memory (latched)
    fn newBatch(self: *DebugRendererSimple) ?*BatchImpl {
        const batch = self.base.allocator.create(BatchImpl) catch {
            self.base.setOutOfMemory();
            return null;
        };
        batch.* = .{ .base = .init(BatchImpl, self.base.allocator) };
        return batch;
    }

    /// Zolt: the triangles of `batch` could not be stored, drop it (empty Batch, latched)
    fn dropBatch(self: *DebugRendererSimple, batch: *BatchImpl) Batch {
        batch.base.vtable.destroy(&batch.base);
        self.base.setOutOfMemory();
        return .empty;
    }

    /// The implementations of the DebugRenderer interface (DebugRendererSimple is abstract: DrawLine and DrawText3D stay
    /// pure virtual)
    pub const impl = struct {
        /// Fallback implementation that uses DrawLine to draw a triangle (override this if you have a version that renders solid triangles)
        pub fn drawTriangle(self: *DebugRendererSimple, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: CastShadow) void {
            _ = cast_shadow;
            self.base.drawLine(v1, v2, color);
            self.base.drawLine(v2, v3, color);
            self.base.drawLine(v3, v1, color);
        }

        /// Implementation of DebugRenderer interface
        pub fn createTriangleBatch(self: *DebugRendererSimple, triangles: []const Triangle) Batch {
            const batch = self.newBatch() orelse return .empty;
            if (triangles.len == 0)
                return .init(&batch.base);

            batch.triangles.appendSlice(self.base.allocator, triangles) catch return self.dropBatch(batch);
            return .init(&batch.base);
        }

        pub fn createTriangleBatchIndexed(self: *DebugRendererSimple, vertices: []const Vertex, indices: []const u32) Batch {
            const batch = self.newBatch() orelse return .empty;
            if (vertices.len == 0 or indices.len == 0)
                return .init(&batch.base);

            // Convert indexed triangle list to triangle list
            const triangles = batch.triangles.addManyAsSlice(self.base.allocator, indices.len / 3) catch return self.dropBatch(batch);
            for (triangles, 0..) |*triangle, t| {
                triangle.v[0] = vertices[indices[t * 3 + 0]];
                triangle.v[1] = vertices[indices[t * 3 + 1]];
                triangle.v[2] = vertices[indices[t * 3 + 2]];
            }

            return .init(&batch.base);
        }

        pub fn drawGeometry(self: *DebugRendererSimple, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, cull_mode: CullMode, cast_shadow: CastShadow, draw_mode: DrawMode) void {
            _ = cull_mode;

            // Zolt (UB34): a geometry without LODs draws nothing (Jolt reads mLODs.data())
            if (geometry.lods.items.len == 0)
                return;

            // Figure out which LOD to use
            var lod: *const DebugRenderer.LOD = &geometry.lods.items[0];
            if (self.camera_pos_set)
                lod = geometry.getLOD(self.camera_pos.toVec3(), world_space_bounds, lod_scale_sq);

            // Draw the batch (Zolt, UB34: an empty batch draws nothing)
            const target = lod.triangle_batch.get() orelse return;
            const batch: *const BatchImpl = @alignCast(@fieldParentPtr("base", target));
            for (batch.triangles.items) |*triangle| {
                const v0 = model_matrix.mulVec3(Vec3.fromFloat3(triangle.v[0].position));
                const v1 = model_matrix.mulVec3(Vec3.fromFloat3(triangle.v[1].position));
                const v2 = model_matrix.mulVec3(Vec3.fromFloat3(triangle.v[2].position));
                const color = model_color.mul(triangle.v[0].color);

                switch (draw_mode) {
                    .wireframe => {
                        self.base.drawLine(v0, v1, color);
                        self.base.drawLine(v1, v2, color);
                        self.base.drawLine(v2, v0, color);
                    },
                    .solid => self.base.drawTriangle(v0, v1, v2, color, .{ .cast_shadow = cast_shadow }),
                    _ => {}, // Jolt's switch has no default
                }
            }
        }
    };
};

/// A renderer built on DebugRendererSimple that logs its lines (DrawTriangle is DebugRendererSimple's fallback)
const LineRenderer = struct {
    pub const overrides = .{ .drawLine, .drawText3D };

    base: DebugRendererSimple,
    lines: std.ArrayList([2]RVec3) = .empty,
    line_colors: std.ArrayList(Color) = .empty,

    fn init(self: *LineRenderer, allocator: Allocator) Allocator.Error!void {
        self.* = .{ .base = undefined };
        try self.base.init(LineRenderer, allocator, std.testing.io);
    }

    fn deinit(self: *LineRenderer) void {
        self.base.base.deinit();
    }

    pub fn destruct(self: *LineRenderer) void {
        self.lines.deinit(self.base.base.allocator);
        self.line_colors.deinit(self.base.base.allocator);
    }

    pub fn drawLine(self: *LineRenderer, from: RVec3, to: RVec3, color: Color) void {
        self.lines.append(self.base.base.allocator, .{ from, to }) catch unreachable;
        self.line_colors.append(self.base.base.allocator, color) catch unreachable;
    }

    pub fn drawText3D(self: *LineRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        _ = .{ self, position, string, color, height };
    }
};

/// A renderer that also overrides DrawTriangle (it derives from the concrete LineRenderer)
const TriangleRenderer = struct {
    pub const overrides = .{.drawTriangle};

    base: LineRenderer,
    num_triangles: u32 = 0,
    last_cast_shadow: CastShadow = .on,

    fn init(self: *TriangleRenderer, allocator: Allocator) Allocator.Error!void {
        self.* = .{ .base = .{ .base = undefined } };
        try self.base.base.init(TriangleRenderer, allocator, std.testing.io);
    }

    pub fn drawTriangle(self: *TriangleRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: CastShadow) void {
        _ = .{ v1, v2, v3, color };
        self.num_triangles += 1;
        self.last_cast_shadow = cast_shadow;
    }
};

test "DebugRendererSimple: triangles and wireframes of a batch, the fallback DrawTriangle" {
    var r: LineRenderer = undefined;
    try r.init(std.testing.allocator);
    defer r.deinit();

    // The box of Initialize: 6 quads, 12 triangles, each drawn as 3 lines by the fallback DrawTriangle
    r.base.base.drawBox(AABox.init(Vec3.zero(), Vec3.init(2, 4, 6)), Color.white, .{});
    try std.testing.expectEqual(@as(usize, 36), r.lines.items.len);

    // Model matrix: scale (1, 2, 3), translation (1, 2, 3). The first triangle of the top quad is (v0, v3, v2).
    try std.testing.expect(r.lines.items[0][0].eql(RVec3.init(0, 4, 0)));
    try std.testing.expect(r.lines.items[0][1].eql(RVec3.init(0, 4, 6)));
    try std.testing.expect(r.lines.items[1][1].eql(RVec3.init(2, 4, 6)));
    try std.testing.expect(r.lines.items[2][1].eql(RVec3.init(0, 4, 0)));
    try std.testing.expect(r.line_colors.items[0].eql(Color.white.mul(Color.white))); // Model color * vertex color

    // Wireframe: 3 lines per triangle too
    r.base.base.drawBox(AABox.init(Vec3.zero(), Vec3.one()), Color.red, .{ .draw_mode = .wireframe });
    try std.testing.expectEqual(@as(usize, 72), r.lines.items.len);

    // An unknown draw mode draws nothing (Jolt's switch without default)
    r.base.base.drawBox(AABox.init(Vec3.zero(), Vec3.one()), Color.red, .{ .draw_mode = @enumFromInt(7) });
    try std.testing.expectEqual(@as(usize, 72), r.lines.items.len);
}

test "DebugRendererSimple: an overridden DrawTriangle receives the solid triangles" {
    var r: TriangleRenderer = undefined;
    try r.init(std.testing.allocator);
    defer r.base.deinit();
    const renderer = r.base.base.debugRenderer();

    renderer.drawBox(AABox.init(Vec3.zero(), Vec3.one()), Color.white, .{ .cast_shadow = .off });
    try std.testing.expectEqual(@as(u32, 12), r.num_triangles);
    try std.testing.expectEqual(CastShadow.off, r.last_cast_shadow);
    try std.testing.expectEqual(@as(usize, 0), r.base.lines.items.len);

    renderer.drawBox(AABox.init(Vec3.zero(), Vec3.one()), Color.white, .{ .draw_mode = .wireframe });
    try std.testing.expectEqual(@as(u32, 12), r.num_triangles);
    try std.testing.expectEqual(@as(usize, 36), r.base.lines.items.len);
}

test "DebugRendererSimple: LOD 0 until SetCameraPos, then Jolt's LOD selection" {
    const allocator = std.testing.allocator;
    var r: LineRenderer = undefined;
    try r.init(allocator);
    defer r.deinit();
    const renderer = r.base.debugRenderer();

    // LOD 0: 1 triangle up to distance 1, LOD 1: 2 triangles beyond
    const t = Triangle.init(Vec3.zero(), Vec3.axisX(), Vec3.axisY(), Color.white);
    var lod0 = renderer.createTriangleBatch(&.{t});
    defer lod0.deinit();
    var lod1 = renderer.createTriangleBatch(&.{ t, t });
    defer lod1.deinit();
    var geometry = GeometryRef.init(try Geometry.create(allocator, AABox.init(Vec3.zero(), Vec3.one())));
    defer geometry.deinit();
    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = lod0.clone(), .distance = 1 });
    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = lod1.clone(), .distance = 10 });

    const far_bounds = AABox.init(Vec3.replicate(100), Vec3.replicate(101));
    renderer.drawGeometry(RMat44.identity(), far_bounds, 1, Color.white, geometry.get().?, .{});
    try std.testing.expectEqual(@as(usize, 3), r.lines.items.len); // No camera: LOD 0

    r.base.setCameraPos(RVec3.zero());
    renderer.drawGeometry(RMat44.identity(), AABox.init(Vec3.zero(), Vec3.one()), 1, Color.white, geometry.get().?, .{});
    try std.testing.expectEqual(@as(usize, 6), r.lines.items.len); // Inside the bounds: LOD 0
    renderer.drawGeometry(RMat44.identity(), AABox.init(Vec3.replicate(2), Vec3.replicate(3)), 1, Color.white, geometry.get().?, .{});
    try std.testing.expectEqual(@as(usize, 12), r.lines.items.len); // Distance 2 * sqrt(3): LOD 1
    renderer.drawGeometry(RMat44.identity(), AABox.init(Vec3.replicate(2), Vec3.replicate(3)), 16, Color.white, geometry.get().?, .{});
    try std.testing.expectEqual(@as(usize, 15), r.lines.items.len); // The LOD scale stretches the distances: LOD 0
    renderer.drawGeometry(RMat44.identity(), far_bounds, 1, Color.white, geometry.get().?, .{});
    try std.testing.expectEqual(@as(usize, 21), r.lines.items.len); // Beyond the last LOD: the last LOD

    // Zolt: an empty batch and a geometry without LODs draw nothing
    var empty = GeometryRef.init(try Geometry.createWithBatch(allocator, .empty, AABox.init(Vec3.zero(), Vec3.one())));
    defer empty.deinit();
    renderer.drawGeometryAutoBounds(RMat44.identity(), Color.white, empty.get().?, .{});
    empty.get().?.lods.items[0].triangle_batch.deinit();
    empty.get().?.lods.clearRetainingCapacity();
    renderer.drawGeometryAutoBounds(RMat44.identity(), Color.white, empty.get().?, .{});
    try std.testing.expectEqual(@as(usize, 21), r.lines.items.len);

    // Indexed batches are converted to triangle lists, an empty input gives an empty batch
    var empty_batch = renderer.createTriangleBatchIndexed(&.{}, &.{});
    defer empty_batch.deinit();
    try std.testing.expect(empty_batch.get() != null);
}

test "DebugRendererSimple: allocation failures" {
    // Construction: an error and no leak, or success
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = fail_index });
        var r: LineRenderer = undefined;
        r.init(failing.allocator()) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        defer r.deinit();
        // The sweep ends only when no failure was injected: init must not swallow one (the renderer's latch included,
        // R5)
        try std.testing.expect(!failing.has_induced_failure);
        try std.testing.expect(!r.base.base.isOutOfMemory());
        break;
    }

    // Drawing: the batch is dropped and the failure latched. Each call gets its own fail index: a failed allocation
    // does not advance FailingAllocator's count, so both calls fail at the batch, then both at the triangles.
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{});
    var r: LineRenderer = undefined;
    try r.init(failing.allocator());
    defer r.deinit();
    const renderer = r.base.debugRenderer();
    const t = Triangle.init(Vec3.zero(), Vec3.axisX(), Vec3.axisY(), Color.white);
    for (0..2) |i| {
        failing.fail_index = failing.alloc_index + i; // The batch, then its triangles
        const batch = renderer.createTriangleBatch(&.{t});
        try std.testing.expect(batch.get() == null);
        try std.testing.expect(renderer.isOutOfMemory());
        renderer.out_of_memory.store(false, .monotonic);
        failing.fail_index = failing.alloc_index + i; // The batch, then its triangle buffer
        const indexed = renderer.createTriangleBatchIndexed(&.{ t.v[0], t.v[1], t.v[2] }, &.{ 0, 1, 2 });
        try std.testing.expect(indexed.get() == null);
        try std.testing.expect(renderer.isOutOfMemory());
        renderer.out_of_memory.store(false, .monotonic);
    }
}
