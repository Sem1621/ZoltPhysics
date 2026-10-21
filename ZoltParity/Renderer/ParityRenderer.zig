//! The Zig side of the draw call log of the debug renderer parity (Docs/Zolt/RemainingFeaturesArchitecture.md section
//! 4, "Draw call log"). `ParityRenderer.h` is the C++ side: both write the same bytes for the same calls, so the logs
//! of Zolt and Jolt compare byte for byte. Not a test file of its own: the parity tests of the debug renderer groups
//! import it (D0's layers are in DebugRendererParity.zig).
//!
//! - `ParityRenderer` is a DebugRenderer that logs every virtual call with all its arguments, including what a `.jor`
//!   file drops (the world space bounds and the LOD scale of drawGeometry, the contents of every batch). The format is
//!   in ParityRenderer.h.
//! - `SimpleParityRenderer` is a DebugRendererSimple that logs drawLine and drawText3D. Simple's own drawTriangle,
//!   batches and drawGeometry produce the lines.
//! - The op stream: `OpWriter` encodes the calls to make, `runOps` (and RunParityOps in ParityRenderer.h) executes
//!   them on any renderer. Each group adds its ops in its own range of `Op`:
//!     1..15     D0: the virtual functions and the helpers that D0 found ported
//!     16..63    D1: the remaining helpers of DebugRenderer (16..36 used)
//!     64..95    D2: shapes (64..69 used; the shapes come from a `ShapeContext`, see `runOpsWithShapes`)
//!     96..127   D3: constraints, bodies and the physics system
//!     128..191  D4: characters, vehicles, skeletons, soft bodies
//!     192..223  H3: hair
//! - `compareLogs` reports the first record that differs, with its op and its bytes.
//! - The renderers allocate with the allocator they get; a failed allocation of the log panics (test harness only).

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const Color = zolt.Color;
const Float3 = zolt.Float3;
const IndexedTriangleNoMaterial = zolt.IndexedTriangleNoMaterial;
const OrientedBox = zolt.OrientedBox;
const DebugRenderer = zolt.DebugRenderer;
const DebugRendererRecorder = zolt.DebugRendererRecorder;
const DebugRendererSimple = zolt.DebugRendererSimple;
const DMat44 = zolt.DMat44;
const Mat44 = zolt.Mat44;
const Real = zolt.Real;
const RMat44 = zolt.RMat44;
const RVec3 = zolt.RVec3;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Batch = DebugRenderer.Batch;
const BatchTarget = DebugRenderer.BatchTarget;
const CastShadow = DebugRenderer.CastShadow;
const CullMode = DebugRenderer.CullMode;
const DrawMode = DebugRenderer.DrawMode;
const Geometry = DebugRenderer.Geometry;
const GeometryRef = DebugRenderer.GeometryRef;
const Triangle = DebugRenderer.Triangle;
const Vertex = DebugRenderer.Vertex;

/// The op codes of the log (ELogOp in ParityRenderer.h)
pub const LogOp = enum(u8) {
    draw_line = 1,
    draw_triangle = 2,
    create_batch = 3,
    create_batch_indexed = 4,
    draw_geometry = 5,
    draw_text3d = 6,
    marker = 7,
    _,
};

/// The bytes of a log (ParityLog in ParityRenderer.h)
pub const Log = struct {
    allocator: Allocator,
    bytes: std.ArrayList(u8) = .empty,

    pub fn deinit(self: *Log) void {
        self.bytes.deinit(self.allocator);
    }

    pub fn writeBytes(self: *Log, data: []const u8) void {
        self.bytes.appendSlice(self.allocator, data) catch @panic("ParityRenderer: out of memory");
    }

    pub fn writeRaw(self: *Log, value: anytype) void {
        self.writeBytes(std.mem.asBytes(&value));
    }

    pub fn writeOp(self: *Log, op: LogOp) void {
        self.writeRaw(@intFromEnum(op));
    }

    pub fn writeRVec3(self: *Log, v: RVec3) void {
        self.writeRaw(@as(Real, v.getX()));
        self.writeRaw(@as(Real, v.getY()));
        self.writeRaw(@as(Real, v.getZ()));
    }

    pub fn writeVec3(self: *Log, v: Vec3) void {
        self.writeRaw(v.getX());
        self.writeRaw(v.getY());
        self.writeRaw(v.getZ());
    }

    pub fn writeVec4(self: *Log, v: Vec4) void {
        self.writeRaw(v.getX());
        self.writeRaw(v.getY());
        self.writeRaw(v.getZ());
        self.writeRaw(v.getW());
    }

    pub fn writeColor(self: *Log, color: Color) void {
        self.writeRaw(color.getUInt32());
    }

    pub fn writeAABox(self: *Log, box: AABox) void {
        self.writeVec3(box.min);
        self.writeVec3(box.max);
    }

    pub fn writeRMat44(self: *Log, m: RMat44) void {
        if (zolt.Core.double_precision) {
            for (0..3) |i| self.writeVec4(m.getColumn4(@intCast(i)));
            self.writeRVec3(m.getTranslation());
        } else {
            for (0..4) |i| self.writeVec4(m.getColumn4(@intCast(i)));
        }
    }

    pub fn writeMarker(self: *Log, value: u32) void {
        self.writeOp(.marker);
        self.writeRaw(value);
    }

    pub fn logLine(self: *Log, from: RVec3, to: RVec3, color: Color) void {
        self.writeOp(.draw_line);
        self.writeRVec3(from);
        self.writeRVec3(to);
        self.writeColor(color);
    }

    pub fn logText(self: *Log, position: RVec3, string: []const u8, color: Color, height: f32) void {
        self.writeOp(.draw_text3d);
        self.writeRVec3(position);
        self.writeRaw(@as(u32, @intCast(string.len)));
        self.writeBytes(string);
        self.writeColor(color);
        self.writeRaw(height);
    }
};

/// A DebugRenderer that logs every call (ParityRenderer in ParityRenderer.h)
pub const ParityRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    /// A batch that knows its id
    const BatchImpl = struct {
        pub const overrides = .{};
        base: BatchTarget,
        id: u32,
    };

    base: DebugRenderer,
    /// The log of all calls so far (initialize logs the predefined batches: the construction log)
    log: Log,
    /// The id of the next batch
    next_batch_id: u32 = 1,
    next_geometry_id: u32 = 1,
    /// Geometry ids in the order of the first draw (each key holds a reference, so an address is never reused)
    geometry_ids: std.AutoHashMapUnmanaged(*const Geometry, u32) = .empty,

    /// Constructor, in place (the renderer is handed out as *DebugRenderer)
    pub fn init(self: *ParityRenderer, allocator: Allocator) Allocator.Error!void {
        self.* = .{ .base = .init(ParityRenderer, allocator, std.testing.io), .log = .{ .allocator = allocator } };
        errdefer self.base.deinit();
        try self.base.initialize();
    }

    pub fn deinit(self: *ParityRenderer) void {
        self.base.deinit();
    }

    pub fn destruct(self: *ParityRenderer) void {
        var it = self.geometry_ids.keyIterator();
        while (it.next()) |geometry| geometry.*.release();
        self.geometry_ids.deinit(self.base.allocator);
        self.log.deinit();
    }

    pub fn drawLine(self: *ParityRenderer, from: RVec3, to: RVec3, color: Color) void {
        self.log.logLine(from, to, color);
    }

    pub fn drawTriangle(self: *ParityRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: CastShadow) void {
        self.log.writeOp(.draw_triangle);
        self.log.writeRVec3(v1);
        self.log.writeRVec3(v2);
        self.log.writeRVec3(v3);
        self.log.writeColor(color);
        self.log.writeRaw(@intFromEnum(cast_shadow));
    }

    pub fn createTriangleBatch(self: *ParityRenderer, triangles: []const Triangle) Batch {
        const id = self.next_batch_id;
        self.next_batch_id += 1;
        self.log.writeOp(.create_batch);
        self.log.writeRaw(id);
        self.log.writeRaw(@as(u32, @intCast(triangles.len)));
        self.log.writeBytes(std.mem.sliceAsBytes(triangles));
        return self.newBatch(id);
    }

    pub fn createTriangleBatchIndexed(self: *ParityRenderer, vertices: []const Vertex, indices: []const u32) Batch {
        const id = self.next_batch_id;
        self.next_batch_id += 1;
        self.log.writeOp(.create_batch_indexed);
        self.log.writeRaw(id);
        self.log.writeRaw(@as(u32, @intCast(vertices.len)));
        self.log.writeBytes(std.mem.sliceAsBytes(vertices));
        self.log.writeRaw(@as(u32, @intCast(indices.len)));
        self.log.writeBytes(std.mem.sliceAsBytes(indices));
        return self.newBatch(id);
    }

    pub fn drawGeometry(self: *ParityRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, cull_mode: CullMode, cast_shadow: CastShadow, draw_mode: DrawMode) void {
        const entry = self.geometry_ids.getOrPut(self.base.allocator, geometry) catch @panic("ParityRenderer: out of memory");
        if (!entry.found_existing) {
            geometry.addRef();
            entry.value_ptr.* = self.next_geometry_id;
            self.next_geometry_id += 1;
        }

        self.log.writeOp(.draw_geometry);
        self.log.writeRMat44(model_matrix);
        self.log.writeAABox(world_space_bounds);
        self.log.writeRaw(lod_scale_sq);
        self.log.writeColor(model_color);
        self.log.writeRaw(entry.value_ptr.*);
        self.log.writeAABox(geometry.bounds);
        self.log.writeRaw(@as(u32, @intCast(geometry.lods.items.len)));
        for (geometry.lods.items) |lod| {
            self.log.writeRaw(lod.distance);
            const id: u32 = if (lod.triangle_batch.get()) |b| @as(*const BatchImpl, @alignCast(@fieldParentPtr("base", b))).id else 0;
            self.log.writeRaw(id);
        }
        self.log.writeRaw(@intFromEnum(cull_mode));
        self.log.writeRaw(@intFromEnum(cast_shadow));
        self.log.writeRaw(@intFromEnum(draw_mode));
    }

    pub fn drawText3D(self: *ParityRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        self.log.logText(position, string, color, height);
    }

    fn newBatch(self: *ParityRenderer, id: u32) Batch {
        const batch = self.base.allocator.create(BatchImpl) catch @panic("ParityRenderer: out of memory");
        batch.* = .{ .base = .init(BatchImpl, self.base.allocator), .id = id };
        return .init(&batch.base);
    }
};

/// A DebugRendererSimple that logs its lines and texts (SimpleParityRenderer in ParityRenderer.h)
pub const SimpleParityRenderer = struct {
    pub const overrides = .{ .drawLine, .drawText3D };

    base: DebugRendererSimple,
    log: Log,

    /// Constructor, in place
    pub fn init(self: *SimpleParityRenderer, allocator: Allocator) Allocator.Error!void {
        self.* = .{ .base = undefined, .log = .{ .allocator = allocator } };
        try self.base.init(SimpleParityRenderer, allocator, std.testing.io);
    }

    pub fn deinit(self: *SimpleParityRenderer) void {
        self.base.base.deinit();
    }

    pub fn destruct(self: *SimpleParityRenderer) void {
        self.log.deinit();
    }

    pub fn drawLine(self: *SimpleParityRenderer, from: RVec3, to: RVec3, color: Color) void {
        self.log.logLine(from, to, color);
    }

    pub fn drawText3D(self: *SimpleParityRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        self.log.logText(position, string, color, height);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The op stream
// ---------------------------------------------------------------------------------------------------------------------

/// The ops of the stream (the cases of RunParityOps in ParityRenderer.h). The file header lists the range of each group.
pub const Op = enum(u8) {
    /// DrawLine: RVec3 from, RVec3 to, Color
    draw_line = 1,
    /// DrawTriangle: RVec3 v1, v2, v3, Color, i32 cast shadow
    draw_triangle = 2,
    /// DrawText3D: RVec3, string, Color, f32 height
    draw_text3d = 3,
    /// DrawWireBox(AABox): AABox, Color
    draw_wire_box = 4,
    /// DrawWireBox(RMat44, AABox): Affine, AABox, Color
    draw_wire_box_matrix = 5,
    /// DrawMarker: RVec3, Color, f32 size
    draw_marker = 6,
    /// DrawArrow: RVec3 from, RVec3 to, Color, f32 size
    draw_arrow = 7,
    /// DrawBox(AABox): AABox, Color, i32 cast shadow, i32 draw mode
    draw_box = 8,
    /// DrawBox(RMat44, AABox): Affine, AABox, Color, i32 cast shadow, i32 draw mode
    draw_box_matrix = 9,
    /// A geometry in a slot: u32 slot, AABox bounds, u32 LOD count, LOD count * (f32 distance, batch)
    create_geometry = 10,
    /// The virtual DrawGeometry: Affine, AABox world space bounds, f32 LOD scale, Color, u32 slot, i32 cull mode, i32 cast shadow, i32 draw mode
    draw_geometry = 11,
    /// The non virtual DrawGeometry: Affine, Color, u32 slot, i32 cull mode, i32 cast shadow, i32 draw mode
    draw_geometry_auto_bounds = 12,
    /// EndFrame of a recorder (other renderers ignore it)
    end_frame = 13,
    /// SetCameraPos of DebugRendererSimple (other renderers ignore it): RVec3
    set_camera_pos = 14,
    /// Create a batch and release it again: batch
    create_batch = 15,

    // D1: the other helpers of DebugRenderer

    /// DrawWireBox(OrientedBox): Vec3 columns 0..2, Vec3 translation, Vec3 half extents, Color
    draw_wire_box_oriented = 16,
    /// DrawCoordinateSystem: Affine, f32 size
    draw_coordinate_system = 17,
    /// DrawPlane: RVec3 point, Vec3 normal, Color, f32 size
    draw_plane = 18,
    /// DrawWireTriangle: RVec3 v1, v2, v3, Color
    draw_wire_triangle = 19,
    /// DrawWirePolygon: Affine, u32 count, count * Vec3, Color, f32 arrow size
    draw_wire_polygon = 20,
    /// DrawWireSphere: RVec3 center, f32 radius, Color, i32 level
    draw_wire_sphere = 21,
    /// DrawWireUnitSphere: Affine, Color, i32 level
    draw_wire_unit_sphere = 22,
    /// DrawSphere: RVec3 center, f32 radius, Color, i32 cast shadow, i32 draw mode
    draw_sphere = 23,
    /// DrawUnitSphere: Affine, Color, i32 cast shadow, i32 draw mode
    draw_unit_sphere = 24,
    /// DrawCapsule: Affine, f32 half height of the cylinder, f32 radius, Color, i32 cast shadow, i32 draw mode
    draw_capsule = 25,
    /// DrawCylinder: Affine, f32 half height, f32 radius, Color, i32 cast shadow, i32 draw mode
    draw_cylinder = 26,
    /// DrawOpenCone: RVec3 top, Vec3 axis, Vec3 perpendicular, f32 half angle, f32 length, Color, i32 cast shadow, i32 draw mode
    draw_open_cone = 27,
    /// DrawSwingConeLimits: Affine, f32 swing y half angle, f32 swing z half angle, f32 edge length, Color, i32 cast shadow, i32 draw mode
    draw_swing_cone_limits = 28,
    /// DrawSwingPyramidLimits: Affine, f32 min y, max y, min z, max z angle, f32 edge length, Color, i32 cast shadow, i32 draw mode
    draw_swing_pyramid_limits = 29,
    /// DrawPie: RVec3 center, f32 radius, Vec3 normal, Vec3 axis, f32 min angle, f32 max angle, Color, i32 cast shadow, i32 draw mode
    draw_pie = 30,
    /// DrawTaperedCylinder: Affine, f32 top, bottom, top radius, bottom radius, Color, i32 cast shadow, i32 draw mode
    draw_tapered_cylinder = 31,
    /// NextFrame
    next_frame = 32,
    /// CreateTriangleBatch(VertexList, IndexedTriangleNoMaterialList), then release it: u32 count, count * Float3, u32 count, count * 3 u32
    create_batch_vertex_list = 33,
    /// CreateTriangleBatchForConvex: support, i32 level, u8 with bounds, u32 slot (with bounds the slot gets
    /// Geometry(batch, bounds), else the batch is released)
    create_batch_for_convex = 34,
    /// CreateTriangleGeometryForConvex: support, u32 slot
    create_geometry_for_convex = 35,
    /// Triangles made with both Triangle constructors, then CreateTriangleBatch and release: u32 count, per triangle u8 kind
    /// (1: with UVs), Vec3 v1, v2, v3, Color [, Vec3 UV origin, Vec3 UV direction]
    create_batch_triangles = 36,

    // D2: shapes (u32 shape is an index into the shapes of the ShapeContext)

    /// Shape::Draw: u32 shape, Affine, Vec3 scale, Color, u8 use material colors, u8 draw wireframe
    draw_shape = 64,
    /// Shape::DrawGetSupportFunction: u32 shape, Affine, Vec3 scale, Color, u8 draw support direction
    draw_get_support_function = 65,
    /// Shape::DrawGetSupportingFace: u32 shape, Affine, Vec3 scale
    draw_get_supporting_face = 66,
    /// The static draw flags that the shapes read (`ShapeDrawFlags`): u8 bits
    set_shape_draw_flags = 67,
    /// Shape::GetSubmergedVolume, which draws into the renderer: u32 shape, Vec3 columns 0..2, Vec3 translation (a Mat44),
    /// Vec3 scale, Vec3 plane normal, f32 plane constant, RVec3 base offset. The results (total volume, submerged volume,
    /// center of buoyancy: 5 f32) are appended to the results of the ShapeContext.
    get_submerged_volume = 68,
    /// ConvexHullShape::DrawShrunkShape: u32 shape (a convex hull), Affine, Vec3 scale
    draw_shrunk_shape = 69,
    _,
};

/// The bits of the op `set_shape_draw_flags`: Jolt's static draw flags that shapes read, Zolt's `renderer.flags`
pub const ShapeDrawFlags = struct {
    /// Shape::sDrawSubmergedVolumes
    pub const draw_submerged_volumes: u8 = 1;
    /// ConvexHullShape::sDrawFaceOutlines
    pub const convex_hull_face_outlines: u8 = 2;
    /// MeshShape::sDrawTriangleGroups
    pub const mesh_triangle_groups: u8 = 4;
    /// MeshShape::sDrawTriangleOutlines
    pub const mesh_triangle_outlines: u8 = 8;
    /// HeightFieldShape::sDrawTriangleOutlines
    pub const height_field_triangle_outlines: u8 = 16;
    /// Every flag
    pub const all: u8 = 31;
};

/// The shapes of the D2 ops (ParityShapeContext in ParityRenderer.h)
pub const ShapeContext = struct {
    allocator: Allocator,
    /// The shapes that the ops index
    shapes: []const *const zolt.Shape,
    /// The results of the ops that compute numbers (get_submerged_volume)
    results: std.ArrayList(u8) = .empty,

    pub fn deinit(self: *ShapeContext) void {
        self.results.deinit(self.allocator);
    }

    fn writeResult(self: *ShapeContext, value: anytype) Allocator.Error!void {
        try self.results.appendSlice(self.allocator, std.mem.asBytes(&value));
    }
};

/// The support function of the convex batch ops (ParitySupport in ParityRenderer.h): a box with rounded edges
pub const Support = struct {
    half_extent: Vec3,
    radius: f32,

    pub fn getSupport(self: *const Support, direction: Vec3) Vec3 {
        const box = Vec3.init(
            if (direction.getX() < 0.0) -self.half_extent.getX() else self.half_extent.getX(),
            if (direction.getY() < 0.0) -self.half_extent.getY() else self.half_extent.getY(),
            if (direction.getZ() < 0.0) -self.half_extent.getZ() else self.half_extent.getZ(),
        );
        return box.add(direction.mulScalar(self.radius));
    }
};

/// A triangle of the op `create_batch_triangles`: made with Triangle.init, or Triangle.initUV when `uv` is set
pub const TriangleData = struct {
    v: [3]Vec3,
    color: Color,
    uv: ?struct { origin: Vec3, direction: Vec3 } = null,
};

/// The number of geometry slots of the op stream
pub const num_geometry_slots = 16;

/// An affine matrix as the op stream holds it: 3 columns and a translation
pub const Affine = struct {
    columns: [3]Vec3,
    translation: RVec3,

    pub fn toRMat44(self: Affine) RMat44 {
        const c0 = Vec4.fromVec3W(self.columns[0], 0);
        const c1 = Vec4.fromVec3W(self.columns[1], 0);
        const c2 = Vec4.fromVec3W(self.columns[2], 0);
        return if (zolt.Core.double_precision) DMat44.init(c0, c1, c2, self.translation) else Mat44.fromColumnsTranslation(c0, c1, c2, self.translation);
    }
};

/// A batch of the op stream: a triangle list or indexed triangles
pub const BatchData = union(enum) {
    triangles: []const Triangle,
    indexed: struct { vertices: []const Vertex, indices: []const u32 },
};

/// One LOD of a geometry of the op stream
pub const LODData = struct {
    distance: f32,
    batch: BatchData,
};

/// Encodes the op stream (ParityOpReader in ParityRenderer.h reads it)
pub const OpWriter = struct {
    allocator: Allocator,
    bytes: std.ArrayList(u8) = .empty,

    pub fn deinit(self: *OpWriter) void {
        self.bytes.deinit(self.allocator);
    }

    pub fn writeBytes(self: *OpWriter, data: []const u8) Allocator.Error!void {
        try self.bytes.appendSlice(self.allocator, data);
    }

    pub fn writeRaw(self: *OpWriter, value: anytype) Allocator.Error!void {
        try self.writeBytes(std.mem.asBytes(&value));
    }

    pub fn writeOp(self: *OpWriter, op: Op) Allocator.Error!void {
        try self.writeRaw(@intFromEnum(op));
    }

    pub fn writeRVec3(self: *OpWriter, v: RVec3) Allocator.Error!void {
        try self.writeRaw(@as(Real, v.getX()));
        try self.writeRaw(@as(Real, v.getY()));
        try self.writeRaw(@as(Real, v.getZ()));
    }

    pub fn writeVec3(self: *OpWriter, v: Vec3) Allocator.Error!void {
        try self.writeRaw(v.getX());
        try self.writeRaw(v.getY());
        try self.writeRaw(v.getZ());
    }

    pub fn writeColor(self: *OpWriter, color: Color) Allocator.Error!void {
        try self.writeRaw(color.getUInt32());
    }

    pub fn writeAABox(self: *OpWriter, box: AABox) Allocator.Error!void {
        try self.writeVec3(box.min);
        try self.writeVec3(box.max);
    }

    pub fn writeAffine(self: *OpWriter, m: Affine) Allocator.Error!void {
        for (m.columns) |c| try self.writeVec3(c);
        try self.writeRVec3(m.translation);
    }

    pub fn writeString(self: *OpWriter, string: []const u8) Allocator.Error!void {
        try self.writeRaw(@as(u32, @intCast(string.len)));
        try self.writeBytes(string);
    }

    pub fn writeBatch(self: *OpWriter, batch: BatchData) Allocator.Error!void {
        switch (batch) {
            .triangles => |triangles| {
                try self.writeRaw(@as(u8, 0));
                try self.writeRaw(@as(u32, @intCast(triangles.len)));
                try self.writeBytes(std.mem.sliceAsBytes(triangles));
            },
            .indexed => |indexed| {
                try self.writeRaw(@as(u8, 1));
                try self.writeRaw(@as(u32, @intCast(indexed.vertices.len)));
                try self.writeBytes(std.mem.sliceAsBytes(indexed.vertices));
                try self.writeRaw(@as(u32, @intCast(indexed.indices.len)));
                try self.writeBytes(std.mem.sliceAsBytes(indexed.indices));
            },
        }
    }

    pub fn drawLine(self: *OpWriter, from: RVec3, to: RVec3, color: Color) Allocator.Error!void {
        try self.writeOp(.draw_line);
        try self.writeRVec3(from);
        try self.writeRVec3(to);
        try self.writeColor(color);
    }

    pub fn drawTriangle(self: *OpWriter, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: i32) Allocator.Error!void {
        try self.writeOp(.draw_triangle);
        try self.writeRVec3(v1);
        try self.writeRVec3(v2);
        try self.writeRVec3(v3);
        try self.writeColor(color);
        try self.writeRaw(cast_shadow);
    }

    pub fn drawText3D(self: *OpWriter, position: RVec3, string: []const u8, color: Color, height: f32) Allocator.Error!void {
        try self.writeOp(.draw_text3d);
        try self.writeRVec3(position);
        try self.writeString(string);
        try self.writeColor(color);
        try self.writeRaw(height);
    }

    pub fn drawWireBox(self: *OpWriter, box: AABox, color: Color) Allocator.Error!void {
        try self.writeOp(.draw_wire_box);
        try self.writeAABox(box);
        try self.writeColor(color);
    }

    pub fn drawWireBoxMatrix(self: *OpWriter, matrix: Affine, box: AABox, color: Color) Allocator.Error!void {
        try self.writeOp(.draw_wire_box_matrix);
        try self.writeAffine(matrix);
        try self.writeAABox(box);
        try self.writeColor(color);
    }

    pub fn drawMarker(self: *OpWriter, position: RVec3, color: Color, size: f32) Allocator.Error!void {
        try self.writeOp(.draw_marker);
        try self.writeRVec3(position);
        try self.writeColor(color);
        try self.writeRaw(size);
    }

    pub fn drawArrow(self: *OpWriter, from: RVec3, to: RVec3, color: Color, size: f32) Allocator.Error!void {
        try self.writeOp(.draw_arrow);
        try self.writeRVec3(from);
        try self.writeRVec3(to);
        try self.writeColor(color);
        try self.writeRaw(size);
    }

    pub fn drawBox(self: *OpWriter, box: AABox, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_box);
        try self.writeAABox(box);
        try self.writeColor(color);
        try self.writeRaw(cast_shadow);
        try self.writeRaw(draw_mode);
    }

    pub fn drawBoxMatrix(self: *OpWriter, matrix: Affine, box: AABox, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_box_matrix);
        try self.writeAffine(matrix);
        try self.writeAABox(box);
        try self.writeColor(color);
        try self.writeRaw(cast_shadow);
        try self.writeRaw(draw_mode);
    }

    pub fn createGeometry(self: *OpWriter, slot: u32, bounds: AABox, lods: []const LODData) Allocator.Error!void {
        std.debug.assert(slot < num_geometry_slots);
        try self.writeOp(.create_geometry);
        try self.writeRaw(slot);
        try self.writeAABox(bounds);
        try self.writeRaw(@as(u32, @intCast(lods.len)));
        for (lods) |lod| {
            try self.writeRaw(lod.distance);
            try self.writeBatch(lod.batch);
        }
    }

    pub fn drawGeometry(self: *OpWriter, matrix: Affine, world_space_bounds: AABox, lod_scale_sq: f32, color: Color, slot: u32, cull_mode: i32, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_geometry);
        try self.writeAffine(matrix);
        try self.writeAABox(world_space_bounds);
        try self.writeRaw(lod_scale_sq);
        try self.writeColor(color);
        try self.writeRaw(slot);
        try self.writeRaw(cull_mode);
        try self.writeRaw(cast_shadow);
        try self.writeRaw(draw_mode);
    }

    pub fn drawGeometryAutoBounds(self: *OpWriter, matrix: Affine, color: Color, slot: u32, cull_mode: i32, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_geometry_auto_bounds);
        try self.writeAffine(matrix);
        try self.writeColor(color);
        try self.writeRaw(slot);
        try self.writeRaw(cull_mode);
        try self.writeRaw(cast_shadow);
        try self.writeRaw(draw_mode);
    }

    pub fn endFrame(self: *OpWriter) Allocator.Error!void {
        try self.writeOp(.end_frame);
    }

    pub fn setCameraPos(self: *OpWriter, position: RVec3) Allocator.Error!void {
        try self.writeOp(.set_camera_pos);
        try self.writeRVec3(position);
    }

    pub fn createBatch(self: *OpWriter, batch: BatchData) Allocator.Error!void {
        try self.writeOp(.create_batch);
        try self.writeBatch(batch);
    }

    // D1

    fn writeModes(self: *OpWriter, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeRaw(cast_shadow);
        try self.writeRaw(draw_mode);
    }

    fn writeSupport(self: *OpWriter, support: Support) Allocator.Error!void {
        try self.writeVec3(support.half_extent);
        try self.writeRaw(support.radius);
    }

    pub fn drawWireBoxOriented(self: *OpWriter, columns: [3]Vec3, translation: Vec3, half_extents: Vec3, color: Color) Allocator.Error!void {
        try self.writeOp(.draw_wire_box_oriented);
        for (columns) |c| try self.writeVec3(c);
        try self.writeVec3(translation);
        try self.writeVec3(half_extents);
        try self.writeColor(color);
    }

    pub fn drawCoordinateSystem(self: *OpWriter, transform: Affine, size: f32) Allocator.Error!void {
        try self.writeOp(.draw_coordinate_system);
        try self.writeAffine(transform);
        try self.writeRaw(size);
    }

    pub fn drawPlane(self: *OpWriter, point: RVec3, normal: Vec3, color: Color, size: f32) Allocator.Error!void {
        try self.writeOp(.draw_plane);
        try self.writeRVec3(point);
        try self.writeVec3(normal);
        try self.writeColor(color);
        try self.writeRaw(size);
    }

    pub fn drawWireTriangle(self: *OpWriter, v1: RVec3, v2: RVec3, v3: RVec3, color: Color) Allocator.Error!void {
        try self.writeOp(.draw_wire_triangle);
        try self.writeRVec3(v1);
        try self.writeRVec3(v2);
        try self.writeRVec3(v3);
        try self.writeColor(color);
    }

    pub fn drawWirePolygon(self: *OpWriter, transform: Affine, vertices: []const Vec3, color: Color, arrow_size: f32) Allocator.Error!void {
        try self.writeOp(.draw_wire_polygon);
        try self.writeAffine(transform);
        try self.writeRaw(@as(u32, @intCast(vertices.len)));
        for (vertices) |v| try self.writeVec3(v);
        try self.writeColor(color);
        try self.writeRaw(arrow_size);
    }

    pub fn drawWireSphere(self: *OpWriter, center: RVec3, radius: f32, color: Color, level: i32) Allocator.Error!void {
        try self.writeOp(.draw_wire_sphere);
        try self.writeRVec3(center);
        try self.writeRaw(radius);
        try self.writeColor(color);
        try self.writeRaw(level);
    }

    pub fn drawWireUnitSphere(self: *OpWriter, matrix: Affine, color: Color, level: i32) Allocator.Error!void {
        try self.writeOp(.draw_wire_unit_sphere);
        try self.writeAffine(matrix);
        try self.writeColor(color);
        try self.writeRaw(level);
    }

    pub fn drawSphere(self: *OpWriter, center: RVec3, radius: f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_sphere);
        try self.writeRVec3(center);
        try self.writeRaw(radius);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn drawUnitSphere(self: *OpWriter, matrix: Affine, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_unit_sphere);
        try self.writeAffine(matrix);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    /// DrawCapsule (op .draw_capsule) or DrawCylinder (op .draw_cylinder)
    pub fn drawCapsuleOrCylinder(self: *OpWriter, op: Op, matrix: Affine, half_height: f32, radius: f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        std.debug.assert(op == .draw_capsule or op == .draw_cylinder);
        try self.writeOp(op);
        try self.writeAffine(matrix);
        try self.writeRaw(half_height);
        try self.writeRaw(radius);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn drawOpenCone(self: *OpWriter, top: RVec3, axis: Vec3, perpendicular: Vec3, half_angle: f32, length: f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_open_cone);
        try self.writeRVec3(top);
        try self.writeVec3(axis);
        try self.writeVec3(perpendicular);
        try self.writeRaw(half_angle);
        try self.writeRaw(length);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn drawSwingConeLimits(self: *OpWriter, matrix: Affine, swing_y_half_angle: f32, swing_z_half_angle: f32, edge_length: f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_swing_cone_limits);
        try self.writeAffine(matrix);
        try self.writeRaw(swing_y_half_angle);
        try self.writeRaw(swing_z_half_angle);
        try self.writeRaw(edge_length);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn drawSwingPyramidLimits(self: *OpWriter, matrix: Affine, angles: [4]f32, edge_length: f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_swing_pyramid_limits);
        try self.writeAffine(matrix);
        for (angles) |a| try self.writeRaw(a);
        try self.writeRaw(edge_length);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn drawPie(self: *OpWriter, center: RVec3, radius: f32, normal: Vec3, axis: Vec3, min_angle: f32, max_angle: f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_pie);
        try self.writeRVec3(center);
        try self.writeRaw(radius);
        try self.writeVec3(normal);
        try self.writeVec3(axis);
        try self.writeRaw(min_angle);
        try self.writeRaw(max_angle);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn drawTaperedCylinder(self: *OpWriter, matrix: Affine, key: [4]f32, color: Color, cast_shadow: i32, draw_mode: i32) Allocator.Error!void {
        try self.writeOp(.draw_tapered_cylinder);
        try self.writeAffine(matrix);
        for (key) |k| try self.writeRaw(k);
        try self.writeColor(color);
        try self.writeModes(cast_shadow, draw_mode);
    }

    pub fn nextFrame(self: *OpWriter) Allocator.Error!void {
        try self.writeOp(.next_frame);
    }

    pub fn createBatchVertexList(self: *OpWriter, vertices: []const Float3, triangles: []const IndexedTriangleNoMaterial) Allocator.Error!void {
        try self.writeOp(.create_batch_vertex_list);
        try self.writeRaw(@as(u32, @intCast(vertices.len)));
        for (vertices) |v| {
            try self.writeRaw(v.x);
            try self.writeRaw(v.y);
            try self.writeRaw(v.z);
        }
        try self.writeRaw(@as(u32, @intCast(triangles.len)));
        for (triangles) |t| {
            for (t.idx) |i| try self.writeRaw(i);
        }
    }

    pub fn createBatchForConvex(self: *OpWriter, support: Support, level: i32, with_bounds: bool, slot: u32) Allocator.Error!void {
        std.debug.assert(slot < num_geometry_slots);
        try self.writeOp(.create_batch_for_convex);
        try self.writeSupport(support);
        try self.writeRaw(level);
        try self.writeRaw(@as(u8, @intFromBool(with_bounds)));
        try self.writeRaw(slot);
    }

    pub fn createGeometryForConvex(self: *OpWriter, support: Support, slot: u32) Allocator.Error!void {
        std.debug.assert(slot < num_geometry_slots);
        try self.writeOp(.create_geometry_for_convex);
        try self.writeSupport(support);
        try self.writeRaw(slot);
    }

    pub fn drawShape(self: *OpWriter, shape: u32, transform: Affine, scale: Vec3, color: Color, use_material_colors: bool, draw_wireframe: bool) Allocator.Error!void {
        try self.writeOp(.draw_shape);
        try self.writeRaw(shape);
        try self.writeAffine(transform);
        try self.writeVec3(scale);
        try self.writeColor(color);
        try self.writeRaw(@as(u8, @intFromBool(use_material_colors)));
        try self.writeRaw(@as(u8, @intFromBool(draw_wireframe)));
    }

    pub fn drawGetSupportFunction(self: *OpWriter, shape: u32, transform: Affine, scale: Vec3, color: Color, draw_support_direction: bool) Allocator.Error!void {
        try self.writeOp(.draw_get_support_function);
        try self.writeRaw(shape);
        try self.writeAffine(transform);
        try self.writeVec3(scale);
        try self.writeColor(color);
        try self.writeRaw(@as(u8, @intFromBool(draw_support_direction)));
    }

    pub fn drawGetSupportingFace(self: *OpWriter, shape: u32, transform: Affine, scale: Vec3) Allocator.Error!void {
        try self.writeOp(.draw_get_supporting_face);
        try self.writeRaw(shape);
        try self.writeAffine(transform);
        try self.writeVec3(scale);
    }

    pub fn setShapeDrawFlags(self: *OpWriter, flags: u8) Allocator.Error!void {
        try self.writeOp(.set_shape_draw_flags);
        try self.writeRaw(flags);
    }

    pub fn getSubmergedVolume(self: *OpWriter, shape: u32, columns: [3]Vec3, translation: Vec3, scale: Vec3, plane_normal: Vec3, plane_constant: f32, base_offset: RVec3) Allocator.Error!void {
        try self.writeOp(.get_submerged_volume);
        try self.writeRaw(shape);
        for (columns) |c| try self.writeVec3(c);
        try self.writeVec3(translation);
        try self.writeVec3(scale);
        try self.writeVec3(plane_normal);
        try self.writeRaw(plane_constant);
        try self.writeRVec3(base_offset);
    }

    pub fn drawShrunkShape(self: *OpWriter, shape: u32, transform: Affine, scale: Vec3) Allocator.Error!void {
        try self.writeOp(.draw_shrunk_shape);
        try self.writeRaw(shape);
        try self.writeAffine(transform);
        try self.writeVec3(scale);
    }

    pub fn createBatchTriangles(self: *OpWriter, triangles: []const TriangleData) Allocator.Error!void {
        try self.writeOp(.create_batch_triangles);
        try self.writeRaw(@as(u32, @intCast(triangles.len)));
        for (triangles) |t| {
            try self.writeRaw(@as(u8, @intFromBool(t.uv != null)));
            for (t.v) |v| try self.writeVec3(v);
            try self.writeColor(t.color);
            if (t.uv) |uv| {
                try self.writeVec3(uv.origin);
                try self.writeVec3(uv.direction);
            }
        }
    }
};

/// Reads the op stream (the Zig twin of ParityOpReader)
const OpReader = struct {
    bytes: []const u8,
    pos: usize = 0,

    fn read(self: *OpReader, comptime T: type) T {
        var value: T = undefined;
        @memcpy(std.mem.asBytes(&value), self.bytes[self.pos..][0..@sizeOf(T)]);
        self.pos += @sizeOf(T);
        return value;
    }

    fn readBytes(self: *OpReader, size: usize) []const u8 {
        const data = self.bytes[self.pos..][0..size];
        self.pos += size;
        return data;
    }

    fn readVec3(self: *OpReader) Vec3 {
        const x = self.read(f32);
        const y = self.read(f32);
        const z = self.read(f32);
        return Vec3.init(x, y, z);
    }

    fn readRVec3(self: *OpReader) RVec3 {
        const x = self.read(Real);
        const y = self.read(Real);
        const z = self.read(Real);
        return RVec3.init(x, y, z);
    }

    fn readColor(self: *OpReader) Color {
        return Color.fromUInt32(self.read(u32));
    }

    fn readAABox(self: *OpReader) AABox {
        const min = self.readVec3();
        const max = self.readVec3();
        return AABox.init(min, max);
    }

    fn readSupport(self: *OpReader) Support {
        const half_extent = self.readVec3();
        return .{ .half_extent = half_extent, .radius = self.read(f32) };
    }

    fn readModes(self: *OpReader) DebugRenderer.SolidOptions {
        const cast_shadow: CastShadow = @enumFromInt(self.read(i32));
        const draw_mode: DrawMode = @enumFromInt(self.read(i32));
        return .{ .cast_shadow = cast_shadow, .draw_mode = draw_mode };
    }

    fn readRMat44(self: *OpReader) RMat44 {
        var m: Affine = undefined;
        for (&m.columns) |*c| c.* = self.readVec3();
        m.translation = self.readRVec3();
        return m.toRMat44();
    }

    fn readString(self: *OpReader) []const u8 {
        const length = self.read(u32);
        return self.readBytes(length);
    }

    /// Copy `count` elements of T out of the stream (the stream is not aligned)
    fn readArray(self: *OpReader, allocator: Allocator, comptime T: type, count: usize) Allocator.Error![]T {
        const items = try allocator.alloc(T, count);
        @memcpy(std.mem.sliceAsBytes(items), self.readBytes(count * @sizeOf(T)));
        return items;
    }

    fn readAndCreateBatch(self: *OpReader, allocator: Allocator, renderer: *DebugRenderer) Allocator.Error!Batch {
        const kind = self.read(u8);
        if (kind == 0) {
            const triangles = try self.readArray(allocator, Triangle, self.read(u32));
            defer allocator.free(triangles);
            return renderer.createTriangleBatch(triangles);
        } else {
            const vertices = try self.readArray(allocator, Vertex, self.read(u32));
            defer allocator.free(vertices);
            const indices = try self.readArray(allocator, u32, self.read(u32));
            defer allocator.free(indices);
            return renderer.createTriangleBatchIndexed(vertices, indices);
        }
    }
};

/// Executes the op stream on `renderer` (RunParityOps in ParityRenderer.h). `simple` and `recorder` are the renderer as
/// a DebugRendererSimple / DebugRendererRecorder (or null) for the ops that only they have.
pub fn runOps(allocator: Allocator, renderer: *DebugRenderer, ops: []const u8, simple: ?*DebugRendererSimple, recorder: ?*DebugRendererRecorder) !void {
    return runOpsWithShapes(allocator, renderer, ops, simple, recorder, null);
}

/// runOps with the shapes of the D2 ops (`shapes` must be set when the stream has them)
pub fn runOpsWithShapes(allocator: Allocator, renderer: *DebugRenderer, ops: []const u8, simple: ?*DebugRendererSimple, recorder: ?*DebugRendererRecorder, shapes: ?*ShapeContext) !void {
    var r: OpReader = .{ .bytes = ops };
    var slots = [_]GeometryRef{.empty} ** num_geometry_slots;
    defer for (&slots) |*slot| slot.deinit();

    while (r.pos < ops.len) {
        const op: Op = @enumFromInt(r.read(u8));
        switch (op) {
            .draw_line => {
                const from = r.readRVec3();
                const to = r.readRVec3();
                renderer.drawLine(from, to, r.readColor());
            },
            .draw_triangle => {
                const v1 = r.readRVec3();
                const v2 = r.readRVec3();
                const v3 = r.readRVec3();
                const color = r.readColor();
                renderer.drawTriangle(v1, v2, v3, color, .{ .cast_shadow = @enumFromInt(r.read(i32)) });
            },
            .draw_text3d => {
                const position = r.readRVec3();
                const string = r.readString();
                const color = r.readColor();
                renderer.drawText3D(position, string, .{ .color = color, .height = r.read(f32) });
            },
            .draw_wire_box => {
                const box = r.readAABox();
                renderer.drawWireBox(box, r.readColor());
            },
            .draw_wire_box_matrix => {
                const matrix = r.readRMat44();
                const box = r.readAABox();
                renderer.drawWireBoxMatrix(matrix, box, r.readColor());
            },
            .draw_marker => {
                const position = r.readRVec3();
                const color = r.readColor();
                renderer.drawMarker(position, color, r.read(f32));
            },
            .draw_arrow => {
                const from = r.readRVec3();
                const to = r.readRVec3();
                const color = r.readColor();
                renderer.drawArrow(from, to, color, r.read(f32));
            },
            .draw_box => {
                const box = r.readAABox();
                const color = r.readColor();
                const cast_shadow: CastShadow = @enumFromInt(r.read(i32));
                const draw_mode: DrawMode = @enumFromInt(r.read(i32));
                renderer.drawBox(box, color, .{ .cast_shadow = cast_shadow, .draw_mode = draw_mode });
            },
            .draw_box_matrix => {
                const matrix = r.readRMat44();
                const box = r.readAABox();
                const color = r.readColor();
                const cast_shadow: CastShadow = @enumFromInt(r.read(i32));
                const draw_mode: DrawMode = @enumFromInt(r.read(i32));
                renderer.drawBoxMatrix(matrix, box, color, .{ .cast_shadow = cast_shadow, .draw_mode = draw_mode });
            },
            .create_geometry => {
                const slot = r.read(u32);
                const bounds = r.readAABox();
                var geometry = GeometryRef.init(try Geometry.create(allocator, bounds));
                defer geometry.deinit();
                const num_lods = r.read(u32);
                for (0..num_lods) |_| {
                    const distance = r.read(f32);
                    var batch = try r.readAndCreateBatch(allocator, renderer);
                    errdefer batch.deinit();
                    try geometry.get().?.lods.append(allocator, .{ .triangle_batch = batch, .distance = distance });
                }
                slots[slot].set(geometry.get());
            },
            .draw_geometry => {
                const matrix = r.readRMat44();
                const world_space_bounds = r.readAABox();
                const lod_scale_sq = r.read(f32);
                const color = r.readColor();
                const slot = r.read(u32);
                const cull_mode: CullMode = @enumFromInt(r.read(i32));
                const cast_shadow: CastShadow = @enumFromInt(r.read(i32));
                const draw_mode: DrawMode = @enumFromInt(r.read(i32));
                renderer.drawGeometry(matrix, world_space_bounds, lod_scale_sq, color, slots[slot].get().?, .{ .cull_mode = cull_mode, .cast_shadow = cast_shadow, .draw_mode = draw_mode });
            },
            .draw_geometry_auto_bounds => {
                const matrix = r.readRMat44();
                const color = r.readColor();
                const slot = r.read(u32);
                const cull_mode: CullMode = @enumFromInt(r.read(i32));
                const cast_shadow: CastShadow = @enumFromInt(r.read(i32));
                const draw_mode: DrawMode = @enumFromInt(r.read(i32));
                renderer.drawGeometryAutoBounds(matrix, color, slots[slot].get().?, .{ .cull_mode = cull_mode, .cast_shadow = cast_shadow, .draw_mode = draw_mode });
            },
            .end_frame => if (recorder) |rec| try rec.endFrame(),
            .set_camera_pos => {
                const position = r.readRVec3();
                if (simple) |s| s.setCameraPos(position);
            },
            .create_batch => {
                var batch = try r.readAndCreateBatch(allocator, renderer);
                batch.deinit();
            },

            // D1: the other helpers of DebugRenderer

            .draw_wire_box_oriented => {
                var columns: [3]Vec3 = undefined;
                for (&columns) |*c| c.* = r.readVec3();
                const t = r.readVec3();
                const half_extents = r.readVec3();
                const orientation = Mat44.init(Vec4.fromVec3W(columns[0], 0), Vec4.fromVec3W(columns[1], 0), Vec4.fromVec3W(columns[2], 0), Vec4.fromVec3W(t, 1));
                renderer.drawWireBoxOriented(OrientedBox.init(orientation, half_extents), r.readColor());
            },
            .draw_coordinate_system => {
                const transform = r.readRMat44();
                renderer.drawCoordinateSystem(transform, .{ .size = r.read(f32) });
            },
            .draw_plane => {
                const point = r.readRVec3();
                const normal = r.readVec3();
                const color = r.readColor();
                renderer.drawPlane(point, normal, color, r.read(f32));
            },
            .draw_wire_triangle => {
                const v1 = r.readRVec3();
                const v2 = r.readRVec3();
                const v3 = r.readRVec3();
                renderer.drawWireTriangle(v1, v2, v3, r.readColor());
            },
            .draw_wire_polygon => {
                const transform = r.readRMat44();
                const count = r.read(u32);
                const vertices = try allocator.alloc(Vec3, count);
                defer allocator.free(vertices);
                for (vertices) |*v| v.* = r.readVec3();
                const color = r.readColor();
                renderer.drawWirePolygon(transform, vertices, color, .{ .arrow_size = r.read(f32) });
            },
            .draw_wire_sphere => {
                const center = r.readRVec3();
                const radius = r.read(f32);
                const color = r.readColor();
                renderer.drawWireSphere(center, radius, color, .{ .level = @intCast(r.read(i32)) });
            },
            .draw_wire_unit_sphere => {
                const matrix = r.readRMat44();
                const color = r.readColor();
                renderer.drawWireUnitSphere(matrix, color, .{ .level = @intCast(r.read(i32)) });
            },
            .draw_sphere => {
                const center = r.readRVec3();
                const radius = r.read(f32);
                const color = r.readColor();
                renderer.drawSphere(center, radius, color, r.readModes());
            },
            .draw_unit_sphere => {
                const matrix = r.readRMat44();
                const color = r.readColor();
                renderer.drawUnitSphere(matrix, color, r.readModes());
            },
            .draw_capsule, .draw_cylinder => {
                const matrix = r.readRMat44();
                const half_height = r.read(f32);
                const radius = r.read(f32);
                const color = r.readColor();
                const modes = r.readModes();
                if (op == .draw_capsule)
                    renderer.drawCapsule(matrix, half_height, radius, color, modes)
                else
                    renderer.drawCylinder(matrix, half_height, radius, color, modes);
            },
            .draw_open_cone => {
                const top = r.readRVec3();
                const axis = r.readVec3();
                const perpendicular = r.readVec3();
                const half_angle = r.read(f32);
                const length = r.read(f32);
                const color = r.readColor();
                renderer.drawOpenCone(top, axis, perpendicular, half_angle, length, color, r.readModes());
            },
            .draw_swing_cone_limits => {
                const matrix = r.readRMat44();
                const swing_y_half_angle = r.read(f32);
                const swing_z_half_angle = r.read(f32);
                const edge_length = r.read(f32);
                const color = r.readColor();
                renderer.drawSwingConeLimits(matrix, swing_y_half_angle, swing_z_half_angle, edge_length, color, r.readModes());
            },
            .draw_swing_pyramid_limits => {
                const matrix = r.readRMat44();
                var angles: [4]f32 = undefined;
                for (&angles) |*a| a.* = r.read(f32);
                const edge_length = r.read(f32);
                const color = r.readColor();
                renderer.drawSwingPyramidLimits(matrix, angles[0], angles[1], angles[2], angles[3], edge_length, color, r.readModes());
            },
            .draw_pie => {
                const center = r.readRVec3();
                const radius = r.read(f32);
                const normal = r.readVec3();
                const axis = r.readVec3();
                const min_angle = r.read(f32);
                const max_angle = r.read(f32);
                const color = r.readColor();
                renderer.drawPie(center, radius, normal, axis, min_angle, max_angle, color, r.readModes());
            },
            .draw_tapered_cylinder => {
                const matrix = r.readRMat44();
                var key: [4]f32 = undefined;
                for (&key) |*k| k.* = r.read(f32);
                const color = r.readColor();
                renderer.drawTaperedCylinder(matrix, key[0], key[1], key[2], key[3], color, r.readModes());
            },
            .next_frame => renderer.nextFrame(),
            .create_batch_vertex_list => {
                const vertices = try allocator.alloc(Float3, r.read(u32));
                defer allocator.free(vertices);
                for (vertices) |*v| {
                    const x = r.read(f32);
                    const y = r.read(f32);
                    v.* = Float3.init(x, y, r.read(f32));
                }
                const triangles = try allocator.alloc(IndexedTriangleNoMaterial, r.read(u32));
                defer allocator.free(triangles);
                for (triangles) |*t| {
                    for (&t.idx) |*i| i.* = r.read(u32);
                }
                var batch = renderer.createTriangleBatchVertexList(vertices, triangles);
                batch.deinit();
            },
            .create_batch_for_convex => {
                const support = r.readSupport();
                const level = r.read(i32);
                const with_bounds = r.read(u8) != 0;
                const slot = r.read(u32);
                var bounds: AABox = .empty;
                const batch = renderer.createTriangleBatchForConvex(.init(&support, Support.getSupport), @intCast(level), .{ .out_bounds = if (with_bounds) &bounds else null });
                if (with_bounds) {
                    slots[slot].set(try Geometry.createWithBatch(allocator, batch, bounds));
                } else {
                    var b = batch;
                    b.deinit();
                }
            },
            .create_geometry_for_convex => {
                const support = r.readSupport();
                const slot = r.read(u32);
                var geometry = renderer.createTriangleGeometryForConvex(.init(&support, Support.getSupport));
                defer geometry.deinit();
                slots[slot].set(geometry.get());
            },
            .draw_shape, .draw_get_support_function, .draw_get_supporting_face, .set_shape_draw_flags, .get_submerged_volume, .draw_shrunk_shape => {
                if (comptime !zolt.Core.debug_renderer) unreachable; // The shapes only draw with the option
                try runShapeOp(op, &r, renderer, shapes orelse @panic("runOps: a shape op without a ShapeContext"));
            },
            .create_batch_triangles => {
                const triangles = try allocator.alloc(Triangle, r.read(u32));
                defer allocator.free(triangles);
                for (triangles) |*t| {
                    const kind = r.read(u8);
                    const v1 = r.readVec3();
                    const v2 = r.readVec3();
                    const v3 = r.readVec3();
                    const color = r.readColor();
                    if (kind == 0) {
                        t.* = .init(v1, v2, v3, color);
                    } else {
                        const uv_origin = r.readVec3();
                        t.* = .initUV(v1, v2, v3, color, uv_origin, r.readVec3());
                    }
                }
                var batch = renderer.createTriangleBatch(triangles);
                batch.deinit();
            },
            _ => std.debug.panic("runOps: unknown op {d}", .{@intFromEnum(op)}),
        }
    }
}

/// One op of D2 (only compiled with the option)
fn runShapeOp(op: Op, r: *OpReader, renderer: *DebugRenderer, context: *ShapeContext) !void {
    switch (op) {
        .draw_shape => {
            const shape = context.shapes[r.read(u32)];
            const transform = r.readRMat44();
            const scale = r.readVec3();
            const color = r.readColor();
            const use_material_colors = r.read(u8) != 0;
            shape.draw(renderer, transform, scale, color, use_material_colors, r.read(u8) != 0);
        },
        .draw_get_support_function => {
            const shape = context.shapes[r.read(u32)];
            const transform = r.readRMat44();
            const scale = r.readVec3();
            const color = r.readColor();
            shape.drawGetSupportFunction(renderer, transform, scale, color, r.read(u8) != 0);
        },
        .draw_get_supporting_face => {
            const shape = context.shapes[r.read(u32)];
            const transform = r.readRMat44();
            shape.drawGetSupportingFace(renderer, transform, r.readVec3());
        },
        .set_shape_draw_flags => {
            const flags = r.read(u8);
            renderer.flags.shape.draw_submerged_volumes = flags & ShapeDrawFlags.draw_submerged_volumes != 0;
            renderer.flags.convex_hull_shape.draw_face_outlines = flags & ShapeDrawFlags.convex_hull_face_outlines != 0;
            renderer.flags.mesh_shape.draw_triangle_groups = flags & ShapeDrawFlags.mesh_triangle_groups != 0;
            renderer.flags.mesh_shape.draw_triangle_outlines = flags & ShapeDrawFlags.mesh_triangle_outlines != 0;
            renderer.flags.height_field_shape.draw_triangle_outlines = flags & ShapeDrawFlags.height_field_triangle_outlines != 0;
        },
        .get_submerged_volume => {
            const shape = context.shapes[r.read(u32)];
            var columns: [3]Vec3 = undefined;
            for (&columns) |*c| c.* = r.readVec3();
            const t = r.readVec3();
            const transform = Mat44.init(Vec4.fromVec3W(columns[0], 0), Vec4.fromVec3W(columns[1], 0), Vec4.fromVec3W(columns[2], 0), Vec4.fromVec3W(t, 1));
            const scale = r.readVec3();
            const normal = r.readVec3();
            const plane = zolt.Plane.init(normal, r.read(f32));
            const base_offset = r.readRVec3();
            const v = shape.getSubmergedVolume(transform, scale, plane, .{ .debug = .{ .base_offset = base_offset, .renderer = renderer } });
            try context.writeResult(v.total_volume);
            try context.writeResult(v.submerged_volume);
            try context.writeResult(v.center_of_buoyancy.getX());
            try context.writeResult(v.center_of_buoyancy.getY());
            try context.writeResult(v.center_of_buoyancy.getZ());
        },
        .draw_shrunk_shape => {
            const shape = context.shapes[r.read(u32)];
            const transform = r.readRMat44();
            shape.cast(zolt.ConvexHullShape).drawShrunkShape(renderer, transform, r.readVec3());
        },
        else => unreachable,
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Comparing logs
// ---------------------------------------------------------------------------------------------------------------------

/// The size in bytes of the log record at the start of `bytes` (null when it is cut off or unknown)
pub fn recordSize(bytes: []const u8) ?usize {
    const real = @sizeOf(Real);
    const rvec3 = 3 * real;
    const rmat44 = if (zolt.Core.double_precision) 3 * 16 + rvec3 else 64;
    const aabox = 24;
    if (bytes.len == 0) return null;
    const op: LogOp = @enumFromInt(bytes[0]);
    const Reader = struct {
        fn u32At(b: []const u8, offset: usize) ?u32 {
            if (offset + 4 > b.len) return null;
            return std.mem.readInt(u32, b[offset..][0..4], .little);
        }
    };
    const size: usize = switch (op) {
        .draw_line => 1 + 2 * rvec3 + 4,
        .draw_triangle => 1 + 3 * rvec3 + 4 + 4,
        .create_batch => 1 + 8 + 108 * @as(usize, Reader.u32At(bytes, 5) orelse return null),
        .create_batch_indexed => blk: {
            const nv: usize = Reader.u32At(bytes, 5) orelse return null;
            const ni: usize = Reader.u32At(bytes, 9 + 36 * nv) orelse return null;
            break :blk 1 + 8 + 36 * nv + 4 + 4 * ni;
        },
        .draw_geometry => blk: {
            const fixed = 1 + rmat44 + aabox + 4 + 4 + 4 + aabox;
            const num_lods: usize = Reader.u32At(bytes, fixed) orelse return null;
            break :blk fixed + 4 + 8 * num_lods + 12;
        },
        .draw_text3d => 1 + rvec3 + 4 + @as(usize, Reader.u32At(bytes, 1 + rvec3) orelse return null) + 4 + 4,
        .marker => 1 + 4,
        _ => return null,
    };
    return if (size <= bytes.len) size else null;
}

/// Compare the log of Zolt with Jolt's. On a difference print the first record that differs (its index, op and bytes)
/// and return error.ParityMismatch.
pub fn compareLogs(name: []const u8, zolt_log: []const u8, jolt_log: []const u8) !void {
    if (std.mem.eql(u8, zolt_log, jolt_log))
        return;

    // Walk both logs record by record
    var offset: usize = 0;
    var index: usize = 0;
    while (true) : (index += 1) {
        const zolt_size = recordSize(zolt_log[offset..]);
        const jolt_size = recordSize(jolt_log[offset..]);
        const zolt_record = zolt_log[offset..][0 .. zolt_size orelse zolt_log.len - offset];
        const jolt_record = jolt_log[offset..][0 .. jolt_size orelse jolt_log.len - offset];
        if (zolt_size == null or jolt_size == null or !std.mem.eql(u8, zolt_record, jolt_record)) {
            std.debug.print("{s}: the logs differ at record {d} (byte {d}; zolt {d} bytes, jolt {d} bytes)\n", .{ name, index, offset, zolt_log.len, jolt_log.len });
            printRecord("zolt", zolt_record);
            printRecord("jolt", jolt_record);
            return error.ParityMismatch;
        }
        offset += zolt_record.len;
    }
}

fn printRecord(side: []const u8, record: []const u8) void {
    const op_name = if (record.len == 0) "(end of log)" else if (std.enums.tagName(LogOp, @enumFromInt(record[0]))) |n| n else "(unknown op)";
    const shown = record[0..@min(record.len, 160)];
    std.debug.print("  {s}: {s} {x}{s}\n", .{ side, op_name, shown, if (shown.len < record.len) "..." else "" });
}

// ---------------------------------------------------------------------------------------------------------------------
// The results of the C ABI (JoltDRResult in DebugRendererReference.cpp)
// ---------------------------------------------------------------------------------------------------------------------

const jolt_result = struct {
    extern fn jolt_dr_result_count(result: *anyopaque) u32;
    extern fn jolt_dr_result_data(result: *anyopaque, index: u32) ?[*]const u8;
    extern fn jolt_dr_result_size(result: *anyopaque, index: u32) u64;
    extern fn jolt_dr_result_free(result: *anyopaque) void;
};

/// The byte arrays that a C function of DebugRendererReference.cpp returns
pub const JoltResult = struct {
    handle: *anyopaque,

    pub fn deinit(self: JoltResult) void {
        jolt_result.jolt_dr_result_free(self.handle);
    }

    pub fn count(self: JoltResult) u32 {
        return jolt_result.jolt_dr_result_count(self.handle);
    }

    pub fn part(self: JoltResult, index: u32) []const u8 {
        const size: usize = @intCast(jolt_result.jolt_dr_result_size(self.handle, index));
        if (size == 0) return &.{};
        return jolt_result.jolt_dr_result_data(self.handle, index).?[0..size];
    }

    pub fn partU32(self: JoltResult, index: u32) u32 {
        return std.mem.readInt(u32, self.part(index)[0..4], .little);
    }
};
