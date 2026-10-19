//! Port of: Jolt/Renderer/DebugRenderer.h, Jolt/Renderer/DebugRenderer.cpp
//! Status: partial
//! Missing: DrawWireBox(OrientedBox), DrawCoordinateSystem, DrawPlane, DrawWireTriangle, DrawWirePolygon, DrawWireSphere, DrawWireUnitSphere, DrawSphere, DrawUnitSphere, DrawCapsule, DrawCylinder, DrawOpenCone, DrawSwingConeLimits, DrawSwingPyramidLimits, DrawPie, DrawTaperedCylinder, sCalculateBounds, CreateTriangleBatch(Float3 *, IndexedTriangleNoMaterial *), CreateTriangleBatchForConvex, CreateTriangleGeometryForConvex, NextFrame, the predefined geometry other than mBox (mSphere, mCapsuleTop / Mid / Bottom, mOpenCone, mCylinder) and the swing / pie / tapered cylinder caches (group D1 of Docs/Zolt/RemainingFeaturesArchitecture.md)
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R2 to R5:
//! - `-Ddebug_renderer=true` is JPH_DEBUG_RENDERER (`Core.debug_renderer`). This file always compiles and is tested in
//!   both configurations: it is a library of types and helpers. The option only controls the hooks in the simulation:
//!   the draw vtable entries, the draw functions, the debug-only members and parameters are `void` without it (R2).
//! - `DebugRenderer` is a pattern A root (Jolt keeps data in the base: the predefined geometry and the caches). The 6
//!   pure virtual functions are the vtable; the helpers (DrawWireBox, DrawBox, ...) are plain functions of the base.
//!   Implementations embed `base: DebugRenderer` and call `initialize` from their constructor, like Jolt.
//! - No `DebugRenderer::sInstance` and no static draw flags (R4). Whoever draws gets the renderer explicitly: a draw
//!   function receives it, the simulation reads `PhysicsSystem.debug_renderer`. Jolt's 14 static draw flags are the
//!   fields of `flags` (`MeshShape::sDrawTriangleGroups` is `flags.mesh_shape.draw_triangle_groups`).
//! - Drawing never fails for the caller (Jolt aborts on out of memory, its draw functions return void). An allocation
//!   failure inside a draw call drops that primitive and sets the atomic `out_of_memory` latch, which the recorder
//!   reports at `endFrame` (R5). Construction (`initialize`) returns `Allocator.Error`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const Color = @import("../Core/Color.zig").Color;
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const virtual = @import("../Core/Virtual.zig");
const AABox = @import("../Geometry/AABox.zig").AABox;
const math = @import("../Math/Math.zig");
const Float2 = @import("../Math/Float2.zig").Float2;
const Float3 = @import("../Math/Float3.zig").Float3;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const RealFile = @import("../Math/Real.zig");
const Real = RealFile.Real;
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;

/// Simple triangle renderer for debugging purposes.
///
/// Inherit from this class to provide your own implementation (embed it as `base` and list the overrides).
///
/// Implement the following virtual functions:
/// - drawLine
/// - drawTriangle
/// - drawText3D
/// - createTriangleBatch / createTriangleBatchIndexed
/// - drawGeometry
///
/// Make sure you call `initialize()` from the constructor of your implementation.
///
/// The createTriangleBatch is used to prepare a batch of triangles to be drawn by a single drawGeometry call,
/// which means that Jolt can render a complex scene much more efficiently than when each triangle in that scene would have been drawn through drawTriangle.
///
/// Note that an implementation that implements createTriangleBatch and drawGeometry is provided by DebugRendererSimple which can be used to start quickly.
pub const DebugRenderer = struct {
    /// Enum that determines if a shadow should be cast or not (Jolt: an `int` enum class)
    pub const CastShadow = enum(i32) {
        /// This shape should cast a shadow
        on,
        /// This shape should not cast a shadow
        off,
    };

    /// Determines how triangles are drawn
    pub const DrawMode = enum(i32) {
        /// Draw as a solid shape
        solid,
        /// Draw as wireframe
        wireframe,
    };

    /// Determines which polygons are culled
    pub const CullMode = enum(i32) {
        /// Don't draw backfacing polygons
        cull_back_face,
        /// Don't draw front facing polygons
        cull_front_face,
        /// Don't do culling and draw both sides
        off,
    };

    /// Vertex format used by the triangle renderer
    pub const Vertex = extern struct {
        position: Float3,
        normal: Float3,
        uv: Float2,
        color: Color,

        comptime {
            std.debug.assert(@sizeOf(Vertex) == 36); // The recorder writes the C++ bytes
        }
    };

    /// A single triangle
    pub const Triangle = extern struct {
        v: [3]Vertex,

        comptime {
            std.debug.assert(@sizeOf(Triangle) == 108);
        }

        /// Triangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, ColorArg inColor)
        pub fn init(v1: Vec3, v2: Vec3, v3: Vec3, color: Color) Triangle {
            var t: Triangle = undefined;

            // Set position
            v1.storeFloat3(&t.v[0].position);
            v2.storeFloat3(&t.v[1].position);
            v3.storeFloat3(&t.v[2].position);

            // Set color
            t.v[0].color = color;
            t.v[1].color = color;
            t.v[2].color = color;

            // Calculate normal
            var normal = v2.sub(v1).cross(v3.sub(v1));
            const normal_len = normal.length();
            if (normal_len > 0.0)
                normal = normal.divScalar(normal_len);
            var normal3: Float3 = undefined;
            normal.storeFloat3(&normal3);
            t.v[0].normal = normal3;
            t.v[1].normal = normal3;
            t.v[2].normal = normal3;

            // Reset UV's
            t.v[0].uv = .init(0, 0);
            t.v[1].uv = .init(0, 0);
            t.v[2].uv = .init(0, 0);
            return t;
        }
    };

    /// The reference counted object that a `Batch` points to (Jolt: RefTargetVirtual). Each renderer derives its batch
    /// class from it (pattern A, deleted through the generated `destroy`).
    pub const BatchTarget = struct {
        pub const VTable = struct {
            deinit: *const fn (self: *BatchTarget) void,
            destroy: *const fn (self: *BatchTarget) void,
        };

        // Qualified names: `VTable` alone is ambiguous with DebugRenderer.VTable
        vtable: *const BatchTarget.VTable,
        ref_count: RefCount = .{},
        allocator: Allocator,

        pub const impl = struct {};

        /// Constructor, called by the batch class `T` of a renderer
        pub fn init(comptime T: type, allocator: Allocator) BatchTarget {
            return .{ .vtable = virtual.vtablePtr(BatchTarget.VTable, T), .allocator = allocator };
        }

        pub fn addRef(self: *const BatchTarget) void {
            self.ref_count.addRef();
        }

        pub fn release(self: *const BatchTarget) void {
            if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
        }
    };

    /// Handle for a batch of triangles (Ref<RefTargetVirtual>), empty when the batch could not be created
    pub const Batch = Ref(BatchTarget);

    /// A single level of detail
    pub const LOD = struct {
        triangle_batch: Batch,
        distance: f32,
    };

    /// A geometry primitive containing triangle batches for various lods
    pub const Geometry = struct {
        ref_count: RefCount = .{},
        allocator: Allocator,
        /// All level of details for this mesh
        lods: std.ArrayList(LOD) = .empty,
        /// Bounding box that encapsulates all LODs
        bounds: AABox,

        /// new Geometry(inBounds): reference count 0
        pub fn create(allocator: Allocator, bounds: AABox) Allocator.Error!*Geometry {
            const self = try allocator.create(Geometry);
            self.* = .{ .allocator = allocator, .bounds = bounds };
            return self;
        }

        /// new Geometry(inBatch, inBounds): one LOD at cLargeFloat. Takes over the reference that `batch` holds, also when
        /// it fails.
        pub fn createWithBatch(allocator: Allocator, batch: Batch, bounds: AABox) Allocator.Error!*Geometry {
            var b = batch;
            errdefer b.deinit();
            const self = try create(allocator, bounds);
            errdefer allocator.destroy(self);
            try self.lods.append(allocator, .{ .triangle_batch = b, .distance = math.large_float });
            return self;
        }

        pub fn addRef(self: *const Geometry) void {
            self.ref_count.addRef();
        }

        pub fn release(self: *const Geometry) void {
            if (self.ref_count.release()) {
                const s: *Geometry = @constCast(self); // The object is dead afterwards (Rule M exception)
                for (s.lods.items) |*lod| lod.triangle_batch.deinit();
                s.lods.deinit(s.allocator);
                s.allocator.destroy(s);
            }
        }

        /// Determine which LOD to render
        /// @param camera_position Current position of the camera
        /// @param world_space_bounds World space bounds for this geometry (transform bounds by model space matrix)
        /// @param lod_scale_sq is the squared scale of the model matrix, it is multiplied with the LOD distances in the geometry to calculate the real LOD distance (so a number > 1 will force a higher LOD).
        /// @return The selected LOD.
        pub fn getLOD(self: *const Geometry, camera_position: Vec3, world_space_bounds: AABox, lod_scale_sq: f32) *const LOD {
            const dist_sq = world_space_bounds.getSqDistanceTo(camera_position);
            for (self.lods.items) |*lod| {
                if (dist_sq <= lod_scale_sq * math.square(lod.distance))
                    return lod;
            }
            return &self.lods.items[self.lods.items.len - 1];
        }
    };

    /// Handle for a lodded triangle batch
    pub const GeometryRef = Ref(Geometry);

    /// Jolt's static draw flags (R4): `Class::sDrawFoo` is `flags.class.draw_foo`. Set them between physics updates,
    /// the simulation reads them from its job threads.
    pub const DrawFlags = struct {
        shape: struct {
            /// Shape::sDrawSubmergedVolumes: debug helper which draws the intersection between water and the shapes, the center of buoyancy and the submerged volume
            draw_submerged_volumes: bool = false,
        } = .{},
        convex_hull_shape: struct {
            /// ConvexHullShape::sDrawFaceOutlines: draw the outlines of the faces of the convex hull when drawing the shape
            draw_face_outlines: bool = false,
        } = .{},
        mesh_shape: struct {
            /// MeshShape::sDrawTriangleGroups (Jolt's "Settings")
            draw_triangle_groups: bool = false,
            /// MeshShape::sDrawTriangleOutlines
            draw_triangle_outlines: bool = false,
        } = .{},
        height_field_shape: struct {
            /// HeightFieldShape::sDrawTriangleOutlines (Jolt's "Settings")
            draw_triangle_outlines: bool = false,
        } = .{},
        contact_constraint_manager: struct {
            /// ContactConstraintManager::sDrawContactPoint (Jolt's "Drawing properties")
            draw_contact_point: bool = false,
            /// ContactConstraintManager::sDrawSupportingFaces
            draw_supporting_faces: bool = false,
            /// ContactConstraintManager::sDrawContactPointReduction
            draw_contact_point_reduction: bool = false,
            /// ContactConstraintManager::sDrawContactManifolds
            draw_contact_manifolds: bool = false,
        } = .{},
        physics_system: struct {
            /// PhysicsSystem::sDrawMotionQualityLinearCast: draw debug info for objects that perform continuous collision detection through the linear cast motion quality
            draw_motion_quality_linear_cast: bool = false,
        } = .{},
        character_virtual: struct {
            /// CharacterVirtual::sDrawConstraints: draw the current state of the constraints for iteration 0 when creating them
            draw_constraints: bool = false,
            /// CharacterVirtual::sDrawWalkStairs: draw the state of the walk stairs algorithm
            draw_walk_stairs: bool = false,
            /// CharacterVirtual::sDrawStickToFloor: draw the state of the stick to floor algorithm
            draw_stick_to_floor: bool = false,
            /// CharacterVirtual::sDrawSupportingVolume: draw the supporting volume plane
            draw_supporting_volume: bool = false,
        } = .{},
    };

    /// One entry per C++ virtual function (all pure), plus the generated destructor chain
    pub const VTable = struct {
        /// Destructor chain (generated, virtual ~DebugRenderer)
        deinit: *const fn (self: *DebugRenderer) void,
        /// Draw line
        drawLine: *const fn (self: *DebugRenderer, from: RVec3, to: RVec3, color: Color) void,
        /// Draw a single back face culled triangle
        drawTriangle: *const fn (self: *DebugRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: CastShadow) void,
        /// Create a batch of triangles that can be drawn efficiently (empty batch when `triangles` is empty or on out of memory)
        createTriangleBatch: *const fn (self: *DebugRenderer, triangles: []const Triangle) Batch,
        /// Create a batch of indexed triangles that can be drawn efficiently (CreateTriangleBatch(const Vertex *, int, const uint32 *, int))
        createTriangleBatchIndexed: *const fn (self: *DebugRenderer, vertices: []const Vertex, indices: []const u32) Batch,
        /// Draw some geometry
        drawGeometry: *const fn (self: *DebugRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, cull_mode: CullMode, cast_shadow: CastShadow, draw_mode: DrawMode) void,
        /// Draw text
        drawText3D: *const fn (self: *DebugRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void,
    };

    /// Abstract class: all virtual functions are pure
    pub const impl = struct {};

    vtable: *const VTable,
    /// Allocates the predefined geometry and the caches
    allocator: Allocator,
    /// For the locks of the implementations (the recorder's mutex) and the shape draw caches
    io: std.Io,
    /// Zolt: an allocation failed while drawing, the primitive was dropped (R5). Atomic: draws come from job threads.
    out_of_memory: std.atomic.Value(bool) = .init(false),
    /// Jolt's static draw flags (R4)
    flags: DrawFlags = .{},

    // Predefined shapes
    box: GeometryRef = .empty,

    /// Constructor (DebugRenderer()), called by the implementation `T` with its own type
    pub fn init(comptime T: type, allocator: Allocator, io: std.Io) DebugRenderer {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator, .io = io };
    }

    /// virtual ~DebugRenderer(): the destructor chain of the implementation
    pub fn deinit(self: *DebugRenderer) void {
        self.vtable.deinit(self);
    }

    /// Destructor of this level (releases the predefined geometry)
    pub fn destruct(self: *DebugRenderer) void {
        self.box.deinit();
    }

    /// Zolt: latch an allocation failure inside a draw call (the primitive is dropped, R5)
    pub fn setOutOfMemory(self: *DebugRenderer) void {
        self.out_of_memory.store(true, .release);
    }

    /// Zolt: true when a draw call dropped a primitive because an allocation failed
    pub fn isOutOfMemory(self: *const DebugRenderer) bool {
        return self.out_of_memory.load(.acquire);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Virtual function dispatchers

    /// Draw line
    pub fn drawLine(self: *DebugRenderer, from: RVec3, to: RVec3, color: Color) void {
        self.vtable.drawLine(self, from, to, color);
    }

    /// Draw a single back face culled triangle
    pub fn drawTriangle(self: *DebugRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, opts: struct { cast_shadow: CastShadow = .off }) void {
        self.vtable.drawTriangle(self, v1, v2, v3, color, opts.cast_shadow);
    }

    /// Create a batch of triangles that can be drawn efficiently
    pub fn createTriangleBatch(self: *DebugRenderer, triangles: []const Triangle) Batch {
        return self.vtable.createTriangleBatch(self, triangles);
    }

    /// Create a batch of indexed triangles that can be drawn efficiently
    pub fn createTriangleBatchIndexed(self: *DebugRenderer, vertices: []const Vertex, indices: []const u32) Batch {
        return self.vtable.createTriangleBatchIndexed(self, vertices, indices);
    }

    /// Optional parameters of drawGeometry (Jolt's default arguments)
    pub const GeometryOptions = struct {
        cull_mode: CullMode = .cull_back_face,
        cast_shadow: CastShadow = .on,
        draw_mode: DrawMode = .solid,
    };

    /// Draw some geometry
    /// @param model_matrix is the matrix that transforms the geometry to world space.
    /// @param world_space_bounds is the bounding box of the geometry after transforming it into world space.
    /// @param lod_scale_sq is the squared scale of the model matrix, it is multiplied with the LOD distances in geometry to calculate the real LOD distance (so a number > 1 will force a higher LOD).
    /// @param model_color is the color with which to multiply the vertex colors in geometry.
    /// @param geometry The geometry to draw.
    pub fn drawGeometry(self: *DebugRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const Geometry, opts: GeometryOptions) void {
        self.vtable.drawGeometry(self, model_matrix, world_space_bounds, lod_scale_sq, model_color, geometry, opts.cull_mode, opts.cast_shadow, opts.draw_mode);
    }

    /// Draw text
    pub fn drawText3D(self: *DebugRenderer, position: RVec3, string: []const u8, opts: struct { color: Color = Color.white, height: f32 = 0.5 }) void {
        self.vtable.drawText3D(self, position, string, opts.color, opts.height);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Draw some geometry (the non virtual DrawGeometry(inModelMatrix, inModelColor, inGeometry, ...)): world space
    /// bounds and LOD scale from the model matrix
    pub fn drawGeometryAutoBounds(self: *DebugRenderer, model_matrix: RMat44, model_color: Color, geometry: *const Geometry, opts: GeometryOptions) void {
        const lod_scale_sq = math.max(math.max(model_matrix.getAxisX().lengthSq(), model_matrix.getAxisY().lengthSq()), model_matrix.getAxisZ().lengthSq());
        self.drawGeometry(model_matrix, geometry.bounds.transformedRMat44(model_matrix), lod_scale_sq, model_color, geometry, opts);
    }

    /// Draw wireframe box
    pub fn drawWireBox(self: *DebugRenderer, box: AABox, color: Color) void {
        // 8 vertices
        const v1 = RVec3.init(box.min.getX(), box.min.getY(), box.min.getZ());
        const v2 = RVec3.init(box.min.getX(), box.min.getY(), box.max.getZ());
        const v3 = RVec3.init(box.min.getX(), box.max.getY(), box.min.getZ());
        const v4 = RVec3.init(box.min.getX(), box.max.getY(), box.max.getZ());
        const v5 = RVec3.init(box.max.getX(), box.min.getY(), box.min.getZ());
        const v6 = RVec3.init(box.max.getX(), box.min.getY(), box.max.getZ());
        const v7 = RVec3.init(box.max.getX(), box.max.getY(), box.min.getZ());
        const v8 = RVec3.init(box.max.getX(), box.max.getY(), box.max.getZ());
        self.drawWireBoxEdges(.{ v1, v2, v3, v4, v5, v6, v7, v8 }, color);
    }

    /// Draw wireframe box (DrawWireBox(RMat44Arg, const AABox &, ColorArg))
    pub fn drawWireBoxMatrix(self: *DebugRenderer, matrix: RMat44, box: AABox, color: Color) void {
        // 8 vertices
        const v1 = matrix.mulVec3(Vec3.init(box.min.getX(), box.min.getY(), box.min.getZ()));
        const v2 = matrix.mulVec3(Vec3.init(box.min.getX(), box.min.getY(), box.max.getZ()));
        const v3 = matrix.mulVec3(Vec3.init(box.min.getX(), box.max.getY(), box.min.getZ()));
        const v4 = matrix.mulVec3(Vec3.init(box.min.getX(), box.max.getY(), box.max.getZ()));
        const v5 = matrix.mulVec3(Vec3.init(box.max.getX(), box.min.getY(), box.min.getZ()));
        const v6 = matrix.mulVec3(Vec3.init(box.max.getX(), box.min.getY(), box.max.getZ()));
        const v7 = matrix.mulVec3(Vec3.init(box.max.getX(), box.max.getY(), box.min.getZ()));
        const v8 = matrix.mulVec3(Vec3.init(box.max.getX(), box.max.getY(), box.max.getZ()));
        self.drawWireBoxEdges(.{ v1, v2, v3, v4, v5, v6, v7, v8 }, color);
    }

    /// The 12 edges of the DrawWireBox functions, in Jolt's order
    fn drawWireBoxEdges(self: *DebugRenderer, v: [8]RVec3, color: Color) void {
        self.drawLine(v[0], v[1], color);
        self.drawLine(v[0], v[2], color);
        self.drawLine(v[0], v[4], color);
        self.drawLine(v[1], v[3], color);
        self.drawLine(v[1], v[5], color);
        self.drawLine(v[2], v[3], color);
        self.drawLine(v[2], v[6], color);
        self.drawLine(v[3], v[7], color);
        self.drawLine(v[4], v[5], color);
        self.drawLine(v[4], v[6], color);
        self.drawLine(v[5], v[7], color);
        self.drawLine(v[6], v[7], color);
    }

    /// Draw a marker on a position
    pub fn drawMarker(self: *DebugRenderer, position: RVec3, color: Color, size: f32) void {
        const dx = Vec3.init(size, 0, 0);
        const dy = Vec3.init(0, size, 0);
        const dz = Vec3.init(0, 0, size);
        self.drawLine(position.subVec3(dy), position.addVec3(dy), color);
        self.drawLine(position.subVec3(dx), position.addVec3(dx), color);
        self.drawLine(position.subVec3(dz), position.addVec3(dz), color);
    }

    /// Draw an arrow
    pub fn drawArrow(self: *DebugRenderer, from: RVec3, to: RVec3, color: Color, size: f32) void {
        // Draw base line
        self.drawLine(from, to, color);

        if (size > 0.0) {
            // Draw arrow head
            var dir = to.sub(from).toVec3();
            const len = dir.length();
            if (len != 0.0)
                dir = dir.mulScalar(size / len)
            else
                dir = Vec3.init(size, 0, 0);
            const perp = dir.getNormalizedPerpendicular().mulScalar(size);
            self.drawLine(to.subVec3(dir).addVec3(perp), to, color);
            self.drawLine(to.subVec3(dir).subVec3(perp), to, color);
        }
    }

    /// Optional parameters of the solid draw helpers (Jolt's default arguments)
    pub const SolidOptions = struct {
        cast_shadow: CastShadow = .on,
        draw_mode: DrawMode = .solid,
    };

    /// Draw a box
    pub fn drawBox(self: *DebugRenderer, box: AABox, color: Color, opts: SolidOptions) void {
        var m = RMat44.scaleVec3(Vec3.max(box.getExtent(), Vec3.replicate(1.0e-6))); // Prevent div by zero when one of the edges has length 0
        m.setTranslation(RVec3.fromVec3(box.getCenter()));
        self.drawGeometryAutoBounds(m, color, self.box.get().?, .{ .cull_mode = .cull_back_face, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Draw a box (DrawBox(RMat44Arg, const AABox &, ...))
    pub fn drawBoxMatrix(self: *DebugRenderer, matrix: RMat44, box: AABox, color: Color, opts: SolidOptions) void {
        var m = Mat44.scaleVec3(Vec3.max(box.getExtent(), Vec3.replicate(1.0e-6))); // Prevent div by zero when one of the edges has length 0
        m.setTranslation(box.getCenter());
        self.drawGeometryAutoBounds(rmat44MulMat44(matrix, m), color, self.box.get().?, .{ .cull_mode = .cull_back_face, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Initialize the system, must be called from the constructor of the DebugRenderer implementation.
    /// Prototype: only the box (the first predefined geometry, so the batch IDs of a recorder match Jolt's).
    pub fn initialize(self: *DebugRenderer) Allocator.Error!void {
        // Box
        {
            var box_vertices: std.ArrayList(Vertex) = .empty;
            defer box_vertices.deinit(self.allocator);
            var box_indices: std.ArrayList(u32) = .empty;
            defer box_indices.deinit(self.allocator);

            // Get corner points
            const v0 = Vec3.init(-1, 1, -1);
            const v1 = Vec3.init(1, 1, -1);
            const v2 = Vec3.init(1, 1, 1);
            const v3 = Vec3.init(-1, 1, 1);
            const v4 = Vec3.init(-1, -1, -1);
            const v5 = Vec3.init(1, -1, -1);
            const v6 = Vec3.init(1, -1, 1);
            const v7 = Vec3.init(-1, -1, 1);

            // Top
            try self.createQuad(&box_indices, &box_vertices, v0, v3, v2, v1);

            // Bottom
            try self.createQuad(&box_indices, &box_vertices, v4, v5, v6, v7);

            // Left
            try self.createQuad(&box_indices, &box_vertices, v0, v4, v7, v3);

            // Right
            try self.createQuad(&box_indices, &box_vertices, v2, v6, v5, v1);

            // Front
            try self.createQuad(&box_indices, &box_vertices, v3, v7, v6, v2);

            // Back
            try self.createQuad(&box_indices, &box_vertices, v0, v1, v5, v4);

            const batch = self.createTriangleBatchIndexed(box_vertices.items, box_indices.items);
            if (self.isOutOfMemory()) return error.OutOfMemory;
            self.box.set(try Geometry.createWithBatch(self.allocator, batch, AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1))));
        }
    }

    /// Helper functions to create a box
    fn createQuad(self: *DebugRenderer, indices: *std.ArrayList(u32), vertices: *std.ArrayList(Vertex), v1: Vec3, v2: Vec3, v3: Vec3, v4: Vec3) Allocator.Error!void {
        // Make room
        const start_idx: u32 = @intCast(vertices.items.len);
        const quad = try vertices.addManyAsArray(self.allocator, 4);

        // Set position
        v1.storeFloat3(&quad[0].position);
        v2.storeFloat3(&quad[1].position);
        v3.storeFloat3(&quad[2].position);
        v4.storeFloat3(&quad[3].position);

        // Set color
        for (quad) |*v| v.color = Color.white;

        // Calculate normal
        const normal = v2.sub(v1).cross(v3.sub(v1)).normalized();
        var normal3: Float3 = undefined;
        normal.storeFloat3(&normal3);
        for (quad) |*v| v.normal = normal3;

        // Set UV's
        quad[0].uv = .init(0, 0);
        quad[1].uv = .init(2, 0);
        quad[2].uv = .init(2, 2);
        quad[3].uv = .init(0, 2);

        // Set indices
        try indices.appendSlice(self.allocator, &.{ start_idx, start_idx + 1, start_idx + 2, start_idx, start_idx + 2, start_idx + 3 });
    }
};

/// RMat44 * Mat44 in both precisions
fn rmat44MulMat44(a: RMat44, b: Mat44) RMat44 {
    return if (Core.double_precision) a.mulMat44(b) else a.mul(b);
}
