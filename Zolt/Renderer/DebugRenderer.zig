//! Port of: Jolt/Renderer/DebugRenderer.h, Jolt/Renderer/DebugRenderer.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R2 to R6:
//! - `-Ddebug_renderer=true` is JPH_DEBUG_RENDERER (`Core.debug_renderer`). This file always compiles and is tested in
//!   both configurations: it is a library of types and helpers. The option only controls the hooks in the simulation:
//!   the draw vtable entries, the draw functions, the debug-only members and parameters are `void` without it (R2).
//! - `DebugRenderer` is a pattern A root (Jolt keeps data in the base: the predefined geometry and the caches). The 6
//!   pure virtual functions are the vtable; the helpers (DrawWireBox, DrawBox, ...) are plain functions of the base.
//!   Implementations embed `base: DebugRenderer` and call `initialize` from their constructor, like Jolt.
//! - Overloads get names: `drawWireBoxMatrix` (DrawWireBox(RMat44, AABox)), `drawWireBoxOriented`
//!   (DrawWireBox(OrientedBox)), `drawBoxMatrix`, `createTriangleBatchIndexed`, `createTriangleBatchVertexList`
//!   (CreateTriangleBatch(VertexList, IndexedTriangleNoMaterialList)), `drawGeometryAutoBounds` (the non virtual
//!   DrawGeometry), `Triangle.initUV`. Jolt's default arguments are options structs. The array overloads of
//!   CreateTriangleBatch and the template DrawWirePolygon take slices.
//! - No `DebugRenderer::sInstance` and no static draw flags (R4). Whoever draws gets the renderer explicitly: a draw
//!   function receives it, the simulation reads `PhysicsSystem.debug_renderer`. Jolt's 14 static draw flags are the
//!   fields of `flags` (`MeshShape::sDrawTriangleGroups` is `flags.mesh_shape.draw_triangle_groups`).
//! - Drawing never fails for the caller (Jolt aborts on out of memory, its draw functions return void). An allocation
//!   failure inside a draw call drops that primitive and sets the atomic `out_of_memory` latch, which the recorder
//!   reports at `endFrame` (R5). A cached helper whose batch the renderer could not create (an empty Batch and the
//!   latch) drops its geometry too, so that no incomplete geometry is cached. Construction (`initialize`) returns
//!   `Allocator.Error`, also when the renderer latched a failure while it created a predefined batch.
//! - The four caches (swing cone, swing pyramid, pie, tapered cylinder) and their `prev` twins are Zolt's port of Jolt's
//!   UnorderedMap with Jolt's key hash and equality (R3). They are only searched, never iterated in a way that shows,
//!   so a cache hit and a miss are Jolt's: -0 and +0 are the same key, a NaN key misses every time (also within a
//!   frame). Like Jolt, a lookup inserts an empty entry first, and a helper that returns early leaves it.
//! - Zolt (R6, UB10): the caches are locked with `cache_mutex`, taken with the renderer's `io`, because several jobs
//!   may draw at once (Jolt races there). The cache lock is taken before the recorder's lock: a batch is created under
//!   the cache lock. The draw itself runs without the cache lock, with its own reference to the geometry.
//! - `JPH_STACK_ALLOC`: the swing limit geometry has a fixed size (arrays on the stack), the pie's size depends on the
//!   angle range (allocated with the renderer's allocator and freed before returning).
//! - Zolt's defined behavior where Jolt is undefined: DrawPie with an angle range whose part count does not fit Jolt's
//!   `int` arithmetic (a NaN or infinite range, or more than `maxInt(i32) / 3` parts) converts NaN or an out of range
//!   float to int in Jolt. Zolt draws nothing then (the empty cache entry stays, like after Jolt's early returns).
//! - DebugRenderer.cpp has no text helper, so R7 does not apply here.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const Color = @import("../Core/Color.zig").Color;
const HashCombine = @import("../Core/HashCombine.zig");
const Mutex = @import("../Core/Mutex.zig").Mutex;
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const UnorderedMap = @import("../Core/UnorderedMap.zig").UnorderedMap;
const virtual = @import("../Core/Virtual.zig");
const AABox = @import("../Geometry/AABox.zig").AABox;
const IndexedTriangleNoMaterial = @import("../Geometry/IndexedTriangle.zig").IndexedTriangleNoMaterial;
const OrientedBox = @import("../Geometry/OrientedBox.zig").OrientedBox;
const math = @import("../Math/Math.zig");
const trig = @import("../Math/Trigonometry.zig");
const Float2 = @import("../Math/Float2.zig").Float2;
const Float3 = @import("../Math/Float3.zig").Float3;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Quat = @import("../Math/Quat.zig").Quat;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const RealFile = @import("../Math/Real.zig");
const Real = RealFile.Real;
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const DMat44 = @import("../Math/DMat44.zig").DMat44;

// Number of LOD levels to create
const max_level: u32 = 4;

// Distance for each LOD level, these are tweaked for an object of approx. size 1. Use the lod scale to scale these distances.
const lod_distance_for_level = [_]f32{ 5.0, 10.0, 40.0, math.large_float };

// Zolt: the largest number of parts of DrawPie for which Jolt's int arithmetic is defined (3 * parts + 2 fits an int):
// the largest float <= maxInt(i32) / 3
const max_pie_parts: f32 = 715827840.0;

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
    // The three enums are `int` enum classes in Jolt. A `.jor` stream holds them as `int`, and DebugRendererPlayback
    // passes any value on to the renderer, so they accept every value (porting guide: enums read from a stream).

    /// Enum that determines if a shadow should be cast or not
    pub const CastShadow = enum(i32) {
        /// This shape should cast a shadow
        on,
        /// This shape should not cast a shadow
        off,
        _,
    };

    /// Determines how triangles are drawn
    pub const DrawMode = enum(i32) {
        /// Draw as a solid shape
        solid,
        /// Draw as wireframe
        wireframe,
        _,
    };

    /// Determines which polygons are culled
    pub const CullMode = enum(i32) {
        /// Don't draw backfacing polygons
        cull_back_face,
        /// Don't draw front facing polygons
        cull_front_face,
        /// Don't do culling and draw both sides
        off,
        _,
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

        /// Triangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, ColorArg inColor, Vec3Arg inUVOrigin, Vec3Arg inUVDirection)
        pub fn initUV(v1: Vec3, v2: Vec3, v3: Vec3, color: Color, uv_origin: Vec3, uv_direction: Vec3) Triangle {
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
            const normal = v2.sub(v1).cross(v3.sub(v1)).normalized();
            var normal3: Float3 = undefined;
            normal.storeFloat3(&normal3);
            t.v[0].normal = normal3;
            t.v[1].normal = normal3;
            t.v[2].normal = normal3;

            // Set UV's
            const uv1 = v1.sub(uv_origin);
            const uv2 = v2.sub(uv_origin);
            const uv3 = v3.sub(uv_origin);
            const axis2 = normal.cross(uv_direction);
            t.v[0].uv = .init(uv_direction.dot(uv1), axis2.dot(uv1));
            t.v[1].uv = .init(uv_direction.dot(uv2), axis2.dot(uv2));
            t.v[2].uv = .init(uv_direction.dot(uv3), axis2.dot(uv3));
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

        /// Zolt: mLODs.push_back({ inBatch, inDistance }). Takes over the reference that `batch` holds, also when it
        /// fails.
        pub fn addLOD(self: *Geometry, batch: Batch, distance: f32) Allocator.Error!void {
            var b = batch;
            errdefer b.deinit();
            try self.lods.append(self.allocator, .{ .triangle_batch = b, .distance = distance });
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

    /// Create a primitive for a convex shape using its support function (function<Vec3 (Vec3Arg inDirection)>: a
    /// function and its context)
    pub const SupportFunction = struct {
        /// The context that is passed to `function`
        context: ?*anyopaque = null,
        /// The function
        function: *const fn (context: ?*anyopaque, direction: Vec3) Vec3,

        /// `function(context, direction)`, `context` is a pointer (it may be const)
        pub fn init(context: anytype, comptime function: fn (@TypeOf(context), Vec3) Vec3) SupportFunction {
            const Context = @TypeOf(context);
            const gen = struct {
                fn call(ptr: ?*anyopaque, direction: Vec3) Vec3 {
                    const typed: Context = @ptrCast(@alignCast(ptr.?));
                    return function(typed, direction);
                }
            };
            return .{ .context = @ptrCast(@constCast(context)), .function = gen.call };
        }

        /// Call the function
        pub fn call(self: SupportFunction, direction: Vec3) Vec3 {
            return self.function(self.context, direction);
        }
    };

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

    // The keys of the caches, with Jolt's equality (operator ==) and hash (JPH_MAKE_HASH_STRUCT)

    const SwingConeLimits = struct {
        swing_y_half_angle: f32,
        swing_z_half_angle: f32,

        pub fn eql(self: SwingConeLimits, rhs: SwingConeLimits) bool {
            return self.swing_y_half_angle == rhs.swing_y_half_angle and
                self.swing_z_half_angle == rhs.swing_z_half_angle;
        }

        /// SwingConeLimitsHasher
        fn hash(t: SwingConeLimits) u64 {
            return HashCombine.hashCombineArgs(.{ t.swing_y_half_angle, t.swing_z_half_angle });
        }
    };

    const SwingConeBatches = UnorderedMap(SwingConeLimits, GeometryRef, .{ .hash = SwingConeLimits.hash });

    const SwingPyramidLimits = struct {
        min_swing_y_angle: f32,
        max_swing_y_angle: f32,
        min_swing_z_angle: f32,
        max_swing_z_angle: f32,

        pub fn eql(self: SwingPyramidLimits, rhs: SwingPyramidLimits) bool {
            return self.min_swing_y_angle == rhs.min_swing_y_angle and
                self.max_swing_y_angle == rhs.max_swing_y_angle and
                self.min_swing_z_angle == rhs.min_swing_z_angle and
                self.max_swing_z_angle == rhs.max_swing_z_angle;
        }

        /// SwingPyramidLimitsHasher
        fn hash(t: SwingPyramidLimits) u64 {
            return HashCombine.hashCombineArgs(.{ t.min_swing_y_angle, t.max_swing_y_angle, t.min_swing_z_angle, t.max_swing_z_angle });
        }
    };

    const SwingPyramidBatches = UnorderedMap(SwingPyramidLimits, GeometryRef, .{ .hash = SwingPyramidLimits.hash });

    /// UnorderedMap<float, GeometryRef> (Jolt's PieBatces): Hash<float> and std::equal_to<float>
    const PieBatches = UnorderedMap(f32, GeometryRef, .{});

    const TaperedCylinder = struct {
        top: f32,
        bottom: f32,
        top_radius: f32,
        bottom_radius: f32,

        pub fn eql(self: TaperedCylinder, rhs: TaperedCylinder) bool {
            return self.top == rhs.top and
                self.bottom == rhs.bottom and
                self.top_radius == rhs.top_radius and
                self.bottom_radius == rhs.bottom_radius;
        }

        /// TaperedCylinderHasher
        fn hash(t: TaperedCylinder) u64 {
            return HashCombine.hashCombineArgs(.{ t.top, t.bottom, t.top_radius, t.bottom_radius });
        }
    };

    /// Jolt's TaperedCylinderBatces
    const TaperedCylinderBatches = UnorderedMap(TaperedCylinder, GeometryRef, .{ .hash = TaperedCylinder.hash });

    vtable: *const VTable,
    /// Allocates the predefined geometry and the caches
    allocator: Allocator,
    /// For the locks of the implementations (the recorder's mutex) and of the caches
    io: std.Io,
    /// Zolt: an allocation failed while drawing, the primitive was dropped (R5). Atomic: draws come from job threads.
    out_of_memory: std.atomic.Value(bool) = .init(false),
    /// Jolt's static draw flags (R4)
    flags: DrawFlags = .{},

    // Predefined shapes
    box: GeometryRef = .empty,
    sphere: GeometryRef = .empty,
    capsule_top: GeometryRef = .empty,
    capsule_mid: GeometryRef = .empty,
    capsule_bottom: GeometryRef = .empty,
    open_cone: GeometryRef = .empty,
    cylinder: GeometryRef = .empty,

    /// Zolt (R6): the lock of the caches below (several jobs may draw at once)
    cache_mutex: Mutex = .{},

    swing_cone_limits: SwingConeBatches = .empty,
    prev_swing_cone_limits: SwingConeBatches = .empty,

    swing_pyramid_limits: SwingPyramidBatches = .empty,
    prev_swing_pyramid_limits: SwingPyramidBatches = .empty,

    pie_limits: PieBatches = .empty,
    prev_pie_limits: PieBatches = .empty,

    tapered_cylinders: TaperedCylinderBatches = .empty,
    prev_tapered_cylinders: TaperedCylinderBatches = .empty,

    /// Constructor (DebugRenderer()), called by the implementation `T` with its own type
    pub fn init(comptime T: type, allocator: Allocator, io: std.Io) DebugRenderer {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator, .io = io };
    }

    /// virtual ~DebugRenderer(): the destructor chain of the implementation
    pub fn deinit(self: *DebugRenderer) void {
        self.vtable.deinit(self);
    }

    /// Destructor of this level (releases the caches and the predefined geometry, in reverse order of declaration)
    pub fn destruct(self: *DebugRenderer) void {
        clearCache(SwingConeBatches, &self.prev_swing_cone_limits, self.allocator);
        clearCache(SwingConeBatches, &self.swing_cone_limits, self.allocator);
        clearCache(SwingPyramidBatches, &self.prev_swing_pyramid_limits, self.allocator);
        clearCache(SwingPyramidBatches, &self.swing_pyramid_limits, self.allocator);
        clearCache(PieBatches, &self.prev_pie_limits, self.allocator);
        clearCache(PieBatches, &self.pie_limits, self.allocator);
        clearCache(TaperedCylinderBatches, &self.prev_tapered_cylinders, self.allocator);
        clearCache(TaperedCylinderBatches, &self.tapered_cylinders, self.allocator);
        self.cylinder.deinit();
        self.open_cone.deinit();
        self.capsule_bottom.deinit();
        self.capsule_mid.deinit();
        self.capsule_top.deinit();
        self.sphere.deinit();
        self.box.deinit();
    }

    /// Release the geometry of a cache and free it (Jolt's clear())
    fn clearCache(comptime Map: type, map: *Map, allocator: Allocator) void {
        var it = map.iterator();
        while (it.next()) |kv| kv.value.deinit();
        map.clearAndFree(allocator);
    }

    /// Zolt: latch an allocation failure inside a draw call (the primitive is dropped, R5)
    pub fn setOutOfMemory(self: *DebugRenderer) void {
        self.out_of_memory.store(true, .release);
    }

    /// Zolt: true when a draw call dropped a primitive because an allocation failed
    pub fn isOutOfMemory(self: *const DebugRenderer) bool {
        return self.out_of_memory.load(.acquire);
    }

    /// Call once after frame is complete. Releases unused dynamically generated geometry assets.
    pub fn nextFrame(self: *DebugRenderer) void {
        self.cache_mutex.lock(self.io);
        defer self.cache_mutex.unlock(self.io);

        clearCache(SwingConeBatches, &self.prev_swing_cone_limits, self.allocator);
        self.swing_cone_limits.swap(&self.prev_swing_cone_limits);

        clearCache(SwingPyramidBatches, &self.prev_swing_pyramid_limits, self.allocator);
        self.swing_pyramid_limits.swap(&self.prev_swing_pyramid_limits);

        clearCache(PieBatches, &self.prev_pie_limits, self.allocator);
        self.pie_limits.swap(&self.prev_pie_limits);

        clearCache(TaperedCylinderBatches, &self.prev_tapered_cylinders, self.allocator);
        self.tapered_cylinders.swap(&self.prev_tapered_cylinders);
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

    /// Draw wireframe box (DrawWireBox(const OrientedBox &, ColorArg))
    pub fn drawWireBoxOriented(self: *DebugRenderer, box: OrientedBox, color: Color) void {
        // 8 vertices
        const he = box.half_extents;
        const v1 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(-he.getX(), -he.getY(), -he.getZ())));
        const v2 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(-he.getX(), -he.getY(), he.getZ())));
        const v3 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(-he.getX(), he.getY(), -he.getZ())));
        const v4 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(-he.getX(), he.getY(), he.getZ())));
        const v5 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(he.getX(), -he.getY(), -he.getZ())));
        const v6 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(he.getX(), -he.getY(), he.getZ())));
        const v7 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(he.getX(), he.getY(), -he.getZ())));
        const v8 = RVec3.fromVec3(box.orientation.mulVec3(Vec3.init(he.getX(), he.getY(), he.getZ())));
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

    /// Draw coordinate system (3 arrows, x = red, y = green, z = blue)
    pub fn drawCoordinateSystem(self: *DebugRenderer, transform: RMat44, opts: struct { size: f32 = 1.0 }) void {
        const size = opts.size;
        self.drawArrow(transform.getTranslation(), transform.mulVec3(Vec3.init(size, 0, 0)), Color.red, 0.1 * size);
        self.drawArrow(transform.getTranslation(), transform.mulVec3(Vec3.init(0, size, 0)), Color.green, 0.1 * size);
        self.drawArrow(transform.getTranslation(), transform.mulVec3(Vec3.init(0, 0, size)), Color.blue, 0.1 * size);
    }

    /// Draw a plane through point with normal normal
    pub fn drawPlane(self: *DebugRenderer, point: RVec3, normal: Vec3, color: Color, size: f32) void {
        // Create orthogonal basis
        var perp1 = normal.cross(Vec3.axisY()).normalizedOr(Vec3.axisX());
        const perp2 = perp1.cross(normal).normalized();
        perp1 = normal.cross(perp2);

        // Calculate corners
        const corner1 = point.addVec3(perp1.add(perp2).mulScalar(size));
        const corner2 = point.addVec3(perp1.sub(perp2).mulScalar(size));
        const corner3 = point.addVec3(perp1.negate().sub(perp2).mulScalar(size));
        const corner4 = point.addVec3(perp1.negate().add(perp2).mulScalar(size));

        // Draw cross
        self.drawLine(corner1, corner3, color);
        self.drawLine(corner2, corner4, color);

        // Draw square
        self.drawLine(corner1, corner2, color);
        self.drawLine(corner2, corner3, color);
        self.drawLine(corner3, corner4, color);
        self.drawLine(corner4, corner1, color);

        // Draw normal
        self.drawArrow(point, point.addVec3(normal.mulScalar(size)), color, 0.1 * size);
    }

    /// Draw wireframe triangle
    pub fn drawWireTriangle(self: *DebugRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color) void {
        self.drawLine(v1, v2, color);
        self.drawLine(v2, v3, color);
        self.drawLine(v3, v1, color);
    }

    /// Draw a wireframe polygon (the template DrawWirePolygon: any array of Vec3 as a slice)
    pub fn drawWirePolygon(self: *DebugRenderer, transform: RMat44, vertices: []const Vec3, color: Color, opts: struct { arrow_size: f32 = 0.0 }) void {
        for (0..vertices.len) |i|
            self.drawArrow(transform.mulVec3(vertices[i]), transform.mulVec3(vertices[(i + 1) % vertices.len]), color, opts.arrow_size);
    }

    /// Draw wireframe sphere
    pub fn drawWireSphere(self: *DebugRenderer, center: RVec3, radius: f32, color: Color, opts: struct { level: u32 = 3 }) void {
        const matrix = rmat44MulMat44(RMat44.translation(center), Mat44.scale(radius));

        self.drawWireUnitSphere(matrix, color, .{ .level = opts.level });
    }

    /// Draw wireframe sphere with radius 1 transformed by `matrix`
    pub fn drawWireUnitSphere(self: *DebugRenderer, matrix: RMat44, color: Color, opts: struct { level: u32 = 3 }) void {
        const level = opts.level;
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX(), Vec3.axisY(), Vec3.axisZ(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX().negate(), Vec3.axisY(), Vec3.axisZ(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX(), Vec3.axisY().negate(), Vec3.axisZ(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX().negate(), Vec3.axisY().negate(), Vec3.axisZ(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX(), Vec3.axisY(), Vec3.axisZ().negate(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX().negate(), Vec3.axisY(), Vec3.axisZ().negate(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX(), Vec3.axisY().negate(), Vec3.axisZ().negate(), level);
        self.drawWireUnitSphereRecursive(matrix, color, Vec3.axisX().negate(), Vec3.axisY().negate(), Vec3.axisZ().negate(), level);
    }

    /// Recursive helper function for drawWireUnitSphere
    fn drawWireUnitSphereRecursive(self: *DebugRenderer, matrix: RMat44, color: Color, dir1: Vec3, dir2: Vec3, dir3: Vec3, level: u32) void {
        if (level == 0) {
            const d1 = matrix.mulVec3(dir1);
            const d2 = matrix.mulVec3(dir2);
            const d3 = matrix.mulVec3(dir3);

            self.drawLine(d1, d2, color);
            self.drawLine(d2, d3, color);
            self.drawLine(d3, d1, color);
        } else {
            const center1 = dir1.add(dir2).normalized();
            const center2 = dir2.add(dir3).normalized();
            const center3 = dir3.add(dir1).normalized();

            self.drawWireUnitSphereRecursive(matrix, color, dir1, center1, center3, level - 1);
            self.drawWireUnitSphereRecursive(matrix, color, center1, center2, center3, level - 1);
            self.drawWireUnitSphereRecursive(matrix, color, center1, dir2, center2, level - 1);
            self.drawWireUnitSphereRecursive(matrix, color, center3, center2, dir3, level - 1);
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

    /// Draw a sphere
    pub fn drawSphere(self: *DebugRenderer, center: RVec3, radius: f32, color: Color, opts: SolidOptions) void {
        const matrix = rmat44MulMat44(RMat44.translation(center), Mat44.scale(radius));

        self.drawUnitSphere(matrix, color, opts);
    }

    /// Draw a sphere with radius 1 transformed by `matrix`
    pub fn drawUnitSphere(self: *DebugRenderer, matrix: RMat44, color: Color, opts: SolidOptions) void {
        self.drawGeometryAutoBounds(matrix, color, self.sphere.get().?, .{ .cull_mode = .cull_back_face, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Draw a capsule with one half sphere at (0, -half_height_of_cylinder, 0) and the other half sphere at (0, half_height_of_cylinder, 0) and radius radius.
    /// The capsule will be transformed by matrix.
    pub fn drawCapsule(self: *DebugRenderer, matrix: RMat44, half_height_of_cylinder: f32, radius: f32, color: Color, opts: SolidOptions) void {
        const scale_matrix = Mat44.scale(radius);

        // Calculate world space bounding box
        const local_bounds = AABox.init(Vec3.init(-radius, -half_height_of_cylinder - radius, -radius), Vec3.init(radius, half_height_of_cylinder + radius, radius));
        const world_bounds = local_bounds.transformedRMat44(matrix);

        const radius_sq = math.square(radius);

        const geometry_opts: GeometryOptions = .{ .cull_mode = .cull_back_face, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode };

        // Draw bottom half sphere
        const bottom_matrix = rmat44MulMat44(rmat44MulMat44(matrix, Mat44.translation(Vec3.init(0, -half_height_of_cylinder, 0))), scale_matrix);
        self.drawGeometry(bottom_matrix, world_bounds, radius_sq, color, self.capsule_bottom.get().?, geometry_opts);

        // Draw top half sphere
        const top_matrix = rmat44MulMat44(rmat44MulMat44(matrix, Mat44.translation(Vec3.init(0, half_height_of_cylinder, 0))), scale_matrix);
        self.drawGeometry(top_matrix, world_bounds, radius_sq, color, self.capsule_top.get().?, geometry_opts);

        // Draw middle part
        self.drawGeometry(rmat44MulMat44(matrix, Mat44.scaleVec3(Vec3.init(radius, half_height_of_cylinder, radius))), world_bounds, radius_sq, color, self.capsule_mid.get().?, geometry_opts);
    }

    /// Draw a cylinder with top (0, half_height, 0) and bottom (0, -half_height, 0) and radius radius.
    /// The cylinder will be transformed by matrix
    pub fn drawCylinder(self: *DebugRenderer, matrix: RMat44, half_height: f32, radius: f32, color: Color, opts: SolidOptions) void {
        const local_transform = Mat44.init(Vec4.init(radius, 0, 0, 0), Vec4.init(0, half_height, 0, 0), Vec4.init(0, 0, radius, 0), Vec4.init(0, 0, 0, 1));
        const transform = rmat44MulMat44(matrix, local_transform);

        const cylinder = self.cylinder.get().?;
        self.drawGeometry(transform, cylinder.bounds.transformedRMat44(transform), math.square(radius), color, cylinder, .{ .cull_mode = .cull_back_face, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Draw a bottomless cone.
    /// @param top Top of cone, center of base is at top + axis.
    /// @param axis Height and direction of cone
    /// @param perpendicular Perpendicular vector to axis.
    /// @param half_angle Specifies the cone angle in radians (angle measured between axis and cone surface).
    /// @param length The length of the cone.
    /// @param color Color to use for drawing the cone.
    /// @param opts.cast_shadow determines if this geometry should cast a shadow or not.
    /// @param opts.draw_mode determines if we draw the geometry solid or in wireframe.
    pub fn drawOpenCone(self: *DebugRenderer, top: RVec3, axis: Vec3, perpendicular: Vec3, half_angle: f32, length: f32, color: Color, opts: SolidOptions) void {
        // Degenerate input can violate these, Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(axis.isNormalized(.{ .tolerance = 1.0e-4 }));
        if (Core.enable_asserts) std.debug.assert(perpendicular.isNormalized(.{ .tolerance = 1.0e-4 }));
        if (Core.enable_asserts) std.debug.assert(@abs(perpendicular.dot(axis)) < 1.0e-4);

        const scaled_axis = axis.mulScalar(math.sign(half_angle) * length);
        const scale = length * trig.tan(@abs(half_angle));
        if (scale != 0.0) {
            const perp1 = perpendicular.mulScalar(scale);
            const perp2 = axis.cross(perpendicular).mulScalar(scale);
            const transform = rmat44FromColumns(Vec4.fromVec3W(perp1, 0), Vec4.fromVec3W(scaled_axis, 0), Vec4.fromVec3W(perp2, 0), top);
            self.drawGeometryAutoBounds(transform, color, self.open_cone.get().?, .{ .cull_mode = .off, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
        }
    }

    /// Helper function for drawSwingConeLimits and drawSwingPyramidLimits: a geometry with reference count 0, null when
    /// out of memory (latched). JPH_STACK_ALLOC: the number of segments is fixed, the arrays live on the stack.
    fn createSwingLimitGeometry(self: *DebugRenderer, comptime num_segments: u32, ls_vertices: *const [num_segments]Vec3) ?*Geometry {
        // Allocate space for vertices
        const num_vertices = 2 * num_segments;
        var vertices: [num_vertices]Vertex = undefined;

        for (0..num_segments) |i| {
            // Get output vertices
            const top = &vertices[2 * i];
            const bottom = &vertices[2 * i + 1];

            // Get local position
            const pos = ls_vertices[i];

            // Get local normal
            const prev_pos = ls_vertices[(i + num_segments - 1) % num_segments];
            const next_pos = ls_vertices[(i + 1) % num_segments];
            const normal = next_pos.cross(pos).normalizedOr(Vec3.zero()).add(pos.cross(prev_pos).normalizedOr(Vec3.zero())).mulScalar(0.5);

            // Store top vertex
            top.position = .init(0, 0, 0);
            normal.storeFloat3(&top.normal);
            top.color = Color.white;
            top.uv = .init(0, 0);

            // Store bottom vertex
            pos.storeFloat3(&bottom.position);
            normal.storeFloat3(&bottom.normal);
            bottom.color = Color.white;
            bottom.uv = .init(0, 0);
        }

        // Allocate space for indices
        const num_indices = 3 * num_segments;
        var indices: [num_indices]u32 = undefined;

        // Calculate indices
        for (0..num_segments) |i| {
            const first: u32 = @intCast(2 * i);
            const second: u32 = (first + 3) % num_vertices;
            const third: u32 = first + 1;

            // Triangle
            indices[3 * i + 0] = first;
            indices[3 * i + 1] = second;
            indices[3 * i + 2] = third;
        }

        // Convert to triangle batch
        return self.newGeometryWithBatch(self.createTriangleBatchIndexed(&vertices, &indices), calculateBounds(&vertices));
    }

    /// Zolt: new Geometry(inBatch, inBounds) in a draw call: null when out of memory (latched). A batch that the renderer
    /// could not create drops the geometry too (R5), so that it is not cached.
    fn newGeometryWithBatch(self: *DebugRenderer, batch: Batch, bounds: AABox) ?*Geometry {
        if (self.isDroppedBatch(batch))
            return null;
        return Geometry.createWithBatch(self.allocator, batch, bounds) catch {
            self.setOutOfMemory();
            return null;
        };
    }

    /// Zolt (R5): the renderer could not create the batch (it returned an empty Batch and latched the failure)
    fn isDroppedBatch(self: *const DebugRenderer, batch: Batch) bool {
        return batch.get() == null and self.isOutOfMemory();
    }

    /// Draws cone rotation limits as used by the SwingTwistConstraintPart.
    /// @param matrix Matrix that transforms from constraint space to world space
    /// @param swing_y_half_angle See SwingTwistConstraintPart
    /// @param swing_z_half_angle See SwingTwistConstraintPart
    /// @param edge_length Size of the edge of the cone shape
    /// @param color Color to use for drawing the cone.
    /// @param opts.cast_shadow determines if this geometry should cast a shadow or not.
    /// @param opts.draw_mode determines if we draw the geometry solid or in wireframe.
    pub fn drawSwingConeLimits(self: *DebugRenderer, matrix: RMat44, swing_y_half_angle: f32, swing_z_half_angle: f32, edge_length: f32, color: Color, opts: SolidOptions) void {
        // Assert sane input (degenerate input can violate these, Jolt's release build continues)
        if (Core.enable_asserts) std.debug.assert(swing_y_half_angle >= 0.0 and swing_y_half_angle <= math.pi);
        if (Core.enable_asserts) std.debug.assert(swing_z_half_angle >= 0.0 and swing_z_half_angle <= math.pi);
        if (Core.enable_asserts) std.debug.assert(edge_length > 0.0);

        // Zolt (R6): the cache lock, the draw below runs with its own reference to the geometry
        var draw_geometry: GeometryRef = blk: {
            self.cache_mutex.lock(self.io);
            defer self.cache_mutex.unlock(self.io);

            // Check cache
            const limits: SwingConeLimits = .{ .swing_y_half_angle = swing_y_half_angle, .swing_z_half_angle = swing_z_half_angle };
            const geometry = self.swing_cone_limits.getOrPutValue(self.allocator, limits, .empty) catch {
                self.setOutOfMemory();
                return;
            };
            if (geometry.get() == null) {
                if (self.prev_swing_cone_limits.find(limits)) |it|
                    geometry.set(it.value.get());
            }
            if (geometry.get() == null) {
                // Number of segments to draw the cone with
                const num_segments = 64;
                const half_num_segments: u32 = num_segments / 2;
                const half: f32 = @floatFromInt(half_num_segments);

                // The y and z values of the quaternion are limited to an ellipse, e1 and e2 are the radii of this ellipse
                const e1 = trig.sin(0.5 * swing_z_half_angle);
                const e2 = trig.sin(0.5 * swing_y_half_angle);

                // Check if the limits will draw something
                if ((e1 <= 0.0 and e2 <= 0.0) or (e2 >= 1.0 and e1 >= 1.0))
                    return;

                // Calculate squared values
                const e1_sq = math.square(e1);
                const e2_sq = math.square(e2);

                // Calculate local space vertices for shape
                var ls_vertices: [num_segments]Vec3 = undefined;
                var tgt_vertex: u32 = 0;
                for (0..2) |side_iter| {
                    for (0..half_num_segments) |segment_iter| {
                        const segment: f32 = @floatFromInt(segment_iter);
                        var y: f32 = undefined;
                        var z: f32 = undefined;
                        if (e2_sq > e1_sq) {
                            // Trace the y value of the quaternion
                            y = e2 - 2.0 * segment * e2 / half;

                            // Calculate the corresponding z value of the quaternion
                            const z_sq = e1_sq - e1_sq / e2_sq * math.square(y);
                            z = if (z_sq <= 0.0) 0.0 else @sqrt(z_sq);
                        } else {
                            // Trace the z value of the quaternion
                            z = -e1 + 2.0 * segment * e1 / half;

                            // Calculate the corresponding y value of the quaternion
                            const y_sq = e2_sq - e2_sq / e1_sq * math.square(z);
                            y = if (y_sq <= 0.0) 0.0 else @sqrt(y_sq);
                        }

                        // If we're tracing the opposite side, flip the values
                        if (side_iter == 1) {
                            z = -z;
                            y = -y;
                        }

                        // Create quaternion
                        const q_xyz = Vec3.init(0, y, z);
                        const w = @sqrt(1.0 - q_xyz.lengthSq());
                        const q = Quat.fromVec4(Vec4.fromVec3W(q_xyz, w));

                        // Store vertex
                        ls_vertices[tgt_vertex] = q.rotateAxisX();
                        tgt_vertex += 1;
                    }
                }

                geometry.set(self.createSwingLimitGeometry(num_segments, &ls_vertices) orelse return);
            }
            break :blk geometry.clone();
        };
        defer draw_geometry.deinit();

        self.drawGeometryAutoBounds(rmat44MulMat44(matrix, Mat44.scale(edge_length)), color, draw_geometry.get().?, .{ .cull_mode = .off, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Draws rotation limits as used by the SwingTwistConstraintPart.
    /// @param matrix Matrix that transforms from constraint space to world space
    /// @param min_swing_y_angle See SwingTwistConstraintPart
    /// @param max_swing_y_angle See SwingTwistConstraintPart
    /// @param min_swing_z_angle See SwingTwistConstraintPart
    /// @param max_swing_z_angle See SwingTwistConstraintPart
    /// @param edge_length Size of the edge of the cone shape
    /// @param color Color to use for drawing the cone.
    /// @param opts.cast_shadow determines if this geometry should cast a shadow or not.
    /// @param opts.draw_mode determines if we draw the geometry solid or in wireframe.
    pub fn drawSwingPyramidLimits(self: *DebugRenderer, matrix: RMat44, min_swing_y_angle: f32, max_swing_y_angle: f32, min_swing_z_angle: f32, max_swing_z_angle: f32, edge_length: f32, color: Color, opts: SolidOptions) void {
        // Assert sane input (degenerate input can violate these, Jolt's release build continues)
        if (Core.enable_asserts) std.debug.assert(min_swing_y_angle <= max_swing_y_angle and min_swing_z_angle <= max_swing_z_angle);
        if (Core.enable_asserts) std.debug.assert(edge_length > 0.0);

        // Zolt (R6): the cache lock, the draw below runs with its own reference to the geometry
        var draw_geometry: GeometryRef = blk: {
            self.cache_mutex.lock(self.io);
            defer self.cache_mutex.unlock(self.io);

            // Check cache
            const limits: SwingPyramidLimits = .{ .min_swing_y_angle = min_swing_y_angle, .max_swing_y_angle = max_swing_y_angle, .min_swing_z_angle = min_swing_z_angle, .max_swing_z_angle = max_swing_z_angle };
            const geometry = self.swing_pyramid_limits.getOrPutValue(self.allocator, limits, .empty) catch {
                self.setOutOfMemory();
                return;
            };
            if (geometry.get() == null) {
                if (self.prev_swing_pyramid_limits.find(limits)) |it|
                    geometry.set(it.value.get());
            }
            if (geometry.get() == null) {
                // Number of segments to draw the cone with
                const num_segments = 64;
                const quarter_num_segments: u32 = num_segments / 4;
                const quarter: f32 = @floatFromInt(quarter_num_segments);

                // Calculate local space vertices for shape
                var ls_vertices: [num_segments]Vec3 = undefined;
                var tgt_vertex: u32 = 0;
                for (0..quarter_num_segments) |segment_iter| {
                    const segment: f32 = @floatFromInt(segment_iter);
                    ls_vertices[tgt_vertex] = swingPyramidAxis(min_swing_y_angle, max_swing_z_angle - (max_swing_z_angle - min_swing_z_angle) * segment / quarter);
                    tgt_vertex += 1;
                }
                for (0..quarter_num_segments) |segment_iter| {
                    const segment: f32 = @floatFromInt(segment_iter);
                    ls_vertices[tgt_vertex] = swingPyramidAxis(min_swing_y_angle + (max_swing_y_angle - min_swing_y_angle) * segment / quarter, min_swing_z_angle);
                    tgt_vertex += 1;
                }
                for (0..quarter_num_segments) |segment_iter| {
                    const segment: f32 = @floatFromInt(segment_iter);
                    ls_vertices[tgt_vertex] = swingPyramidAxis(max_swing_y_angle, min_swing_z_angle + (max_swing_z_angle - min_swing_z_angle) * segment / quarter);
                    tgt_vertex += 1;
                }
                for (0..quarter_num_segments) |segment_iter| {
                    const segment: f32 = @floatFromInt(segment_iter);
                    ls_vertices[tgt_vertex] = swingPyramidAxis(max_swing_y_angle - (max_swing_y_angle - min_swing_y_angle) * segment / quarter, max_swing_z_angle);
                    tgt_vertex += 1;
                }

                geometry.set(self.createSwingLimitGeometry(num_segments, &ls_vertices) orelse return);
            }
            break :blk geometry.clone();
        };
        defer draw_geometry.deinit();

        self.drawGeometryAutoBounds(rmat44MulMat44(matrix, Mat44.scale(edge_length)), color, draw_geometry.get().?, .{ .cull_mode = .off, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// The get_axis lambda of drawSwingPyramidLimits.
    /// Note that this is q = Quat::sRotation(Vec3::sAxisZ(), z) * Quat::sRotation(Vec3::sAxisY(), y) with q.x set to zero so we don't introduce twist
    /// This matches the calculation in SwingTwistConstraintPart::ClampSwingTwist
    fn swingPyramidAxis(y: f32, z: f32) Vec3 {
        const hy = 0.5 * y;
        const hz = 0.5 * z;
        const cos_hy = trig.cos(hy);
        const cos_hz = trig.cos(hz);
        return Quat.init(0, trig.sin(hy) * cos_hz, cos_hy * trig.sin(hz), cos_hy * cos_hz).normalized().rotateAxisX();
    }

    /// Draw a pie (part of a circle).
    /// @param center The center of the circle.
    /// @param radius Radius of the circle.
    /// @param normal The plane normal in which the pie resides.
    /// @param axis The axis that defines an angle of 0 radians.
    /// @param min_angle The pie will be drawn between [min_angle, max_angle] (in radians).
    /// @param max_angle The pie will be drawn between [min_angle, max_angle] (in radians).
    /// @param color Color to use for drawing the pie.
    /// @param opts.cast_shadow determines if this geometry should cast a shadow or not.
    /// @param opts.draw_mode determines if we draw the geometry solid or in wireframe.
    pub fn drawPie(self: *DebugRenderer, center: RVec3, radius: f32, normal: Vec3, axis: Vec3, min_angle: f32, max_angle: f32, color: Color, opts: SolidOptions) void {
        if (min_angle >= max_angle)
            return;

        // Degenerate input can violate these, Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(axis.isNormalized(.{ .tolerance = 1.0e-4 }));
        if (Core.enable_asserts) std.debug.assert(normal.isNormalized(.{ .tolerance = 1.0e-4 }));
        if (Core.enable_asserts) std.debug.assert(@abs(normal.dot(axis)) < 1.0e-4);

        // Zolt (R6): the cache lock, the draw below runs with its own reference to the geometry
        var draw_geometry: GeometryRef = blk: {
            self.cache_mutex.lock(self.io);
            defer self.cache_mutex.unlock(self.io);

            // Pies have a unique batch based on the difference between min and max angle
            const delta_angle = max_angle - min_angle;
            const geometry = self.pie_limits.getOrPutValue(self.allocator, delta_angle, .empty) catch {
                self.setOutOfMemory();
                return;
            };
            if (geometry.get() == null) {
                if (self.prev_pie_limits.find(delta_angle)) |it|
                    geometry.set(it.value.get());
            }
            if (geometry.get() == null) {
                // Zolt: Jolt converts the part count to int, which is undefined for NaN and out of range values. Draw
                // nothing then (see the file header).
                const num_parts_float = @ceil(64.0 * delta_angle / (2.0 * math.pi));
                if (!(num_parts_float <= max_pie_parts))
                    return;
                const num_parts: u32 = @intFromFloat(num_parts_float);

                const normal3 = Float3.init(0, 1, 0);
                const center3 = Float3.init(0, 0, 0);

                // Allocate space for vertices (Zolt: JPH_STACK_ALLOC of an unbounded size, the renderer's allocator)
                const num_vertices = num_parts + 2;
                const vertices = self.allocator.alloc(Vertex, num_vertices) catch {
                    self.setOutOfMemory();
                    return;
                };
                defer self.allocator.free(vertices);

                // Center of circle
                vertices[0] = .{ .position = center3, .normal = normal3, .uv = .init(0, 0), .color = Color.white };

                // Outer edge of pie
                for (0..num_parts + 1) |i| {
                    const angle = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_parts)) * delta_angle;

                    const pos = Float3.init(trig.cos(angle), 0, trig.sin(angle));
                    vertices[i + 1] = .{ .position = pos, .normal = normal3, .uv = .init(0, 0), .color = Color.white };
                }

                // Allocate space for indices
                const num_indices = num_parts * 3;
                const indices = self.allocator.alloc(u32, num_indices) catch {
                    self.setOutOfMemory();
                    return;
                };
                defer self.allocator.free(indices);

                for (0..num_parts) |i| {
                    const index: u32 = @intCast(i);
                    indices[3 * i + 0] = 0;
                    indices[3 * i + 1] = index + 1;
                    indices[3 * i + 2] = index + 2;
                }

                // Convert to triangle batch
                geometry.set(self.newGeometryWithBatch(self.createTriangleBatchIndexed(vertices, indices), calculateBounds(vertices)) orelse return);
            }
            break :blk geometry.clone();
        };
        defer draw_geometry.deinit();

        // Construct matrix that transforms pie into world space
        const matrix = rmat44MulMat44(rmat44FromColumns(Vec4.fromVec3W(axis.mulScalar(radius), 0), Vec4.fromVec3W(normal.mulScalar(radius), 0), Vec4.fromVec3W(normal.cross(axis).mulScalar(radius), 0), center), Mat44.rotationY(-min_angle));

        self.drawGeometryAutoBounds(matrix, color, draw_geometry.get().?, .{ .cull_mode = .off, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Draw a tapered cylinder
    /// @param matrix Matrix that transforms the cylinder to world space.
    /// @param top Top of cylinder (along Y axis)
    /// @param bottom Bottom of cylinder (along Y axis)
    /// @param top_radius Radius at the top
    /// @param bottom_radius Radius at the bottom
    /// @param color Color to use for drawing the pie.
    /// @param opts.cast_shadow determines if this geometry should cast a shadow or not.
    /// @param opts.draw_mode determines if we draw the geometry solid or in wireframe.
    pub fn drawTaperedCylinder(self: *DebugRenderer, matrix: RMat44, top: f32, bottom: f32, top_radius: f32, bottom_radius: f32, color: Color, opts: SolidOptions) void {
        // Zolt (R6): the cache lock, the draw below runs with its own reference to the geometry
        var draw_geometry: GeometryRef = blk: {
            self.cache_mutex.lock(self.io);
            defer self.cache_mutex.unlock(self.io);

            const tapered_cylinder: TaperedCylinder = .{ .top = top, .bottom = bottom, .top_radius = top_radius, .bottom_radius = bottom_radius };

            const geometry = self.tapered_cylinders.getOrPutValue(self.allocator, tapered_cylinder, .empty) catch {
                self.setOutOfMemory();
                return;
            };
            if (geometry.get() == null) {
                if (self.prev_tapered_cylinders.find(tapered_cylinder)) |it|
                    geometry.set(it.value.get());
            }
            if (geometry.get() == null) {
                const max_radius = math.max(top_radius, bottom_radius);
                var new_geometry = GeometryRef.init(Geometry.create(self.allocator, AABox.init(Vec3.init(-max_radius, bottom, -max_radius), Vec3.init(max_radius, top, max_radius))) catch {
                    self.setOutOfMemory();
                    return;
                });
                defer new_geometry.deinit();

                var level: u32 = max_level;
                while (level >= 1) : (level -= 1) {
                    // Zolt: an incomplete geometry is dropped (not cached)
                    const batch = self.createCylinder(top, bottom, top_radius, bottom_radius, level) catch {
                        self.setOutOfMemory();
                        return;
                    };
                    if (self.isDroppedBatch(batch))
                        return;
                    new_geometry.get().?.addLOD(batch, lod_distance_for_level[max_level - level]) catch {
                        self.setOutOfMemory();
                        return;
                    };
                }

                geometry.set(new_geometry.get());
            }
            break :blk geometry.clone();
        };
        defer draw_geometry.deinit();

        self.drawGeometryAutoBounds(matrix, color, draw_geometry.get().?, .{ .cull_mode = .cull_back_face, .cast_shadow = opts.cast_shadow, .draw_mode = opts.draw_mode });
    }

    /// Calculate bounding box for a batch of triangles
    pub fn calculateBounds(vertices: []const Vertex) AABox {
        var bounds: AABox = .empty;
        for (vertices) |*v|
            bounds.encapsulateVec3(Vec3.fromFloat3(v.position));
        return bounds;
    }

    /// Create a batch of triangles from a vertex list and indexed triangles (CreateTriangleBatch(const Float3 *, int,
    /// const IndexedTriangleNoMaterial *, int) and its VertexList / IndexedTriangleNoMaterialList overload). The normals
    /// are the normalized sums of the face normals. Zolt: an empty Batch when out of memory (latched).
    pub fn createTriangleBatchVertexList(self: *DebugRenderer, in_vertices: []const Float3, triangles: []const IndexedTriangleNoMaterial) Batch {
        // Create render vertices
        const vertices = self.allocator.alloc(Vertex, in_vertices.len) catch {
            self.setOutOfMemory();
            return .empty;
        };
        defer self.allocator.free(vertices);
        for (vertices, in_vertices) |*v, position| {
            v.position = position;
            v.normal = .init(0, 0, 0);
            v.uv = .init(0, 0);
            v.color = Color.white;
        }

        // Calculate normals
        for (triangles) |tri| {
            // Calculate normal of face
            var vtx: [3]Vec3 = undefined;
            for (0..3) |j|
                vtx[j] = Vec3.loadFloat3Unsafe(&vertices[tri.idx[j]].position);
            const normal = vtx[1].sub(vtx[0]).cross(vtx[2].sub(vtx[0])).normalized();

            // Add normal to all vertices in face
            for (0..3) |j|
                Vec3.loadFloat3Unsafe(&vertices[tri.idx[j]].normal).add(normal).storeFloat3(&vertices[tri.idx[j]].normal);
        }

        // Renormalize vertex normals
        for (vertices) |*v|
            Vec3.loadFloat3Unsafe(&v.normal).normalized().storeFloat3(&v.normal);

        const indices: []const u32 = @as([*]const u32, @ptrCast(triangles.ptr))[0 .. 3 * triangles.len];
        return self.createTriangleBatchIndexed(vertices, indices);
    }

    /// Create a batch of triangles for a convex shape using its support function (the triangles of a sphere with
    /// `level` subdivisions, each vertex moved to the support point of its direction). `opts.out_bounds` receives the
    /// bounds of the vertices. Zolt: an empty Batch when out of memory (latched).
    pub fn createTriangleBatchForConvex(self: *DebugRenderer, get_support: SupportFunction, level: u32, opts: struct { out_bounds: ?*AABox = null }) Batch {
        return self.createTriangleBatchForConvexImpl(get_support, level, opts.out_bounds) catch {
            self.setOutOfMemory();
            return .empty;
        };
    }

    /// CreateTriangleBatchForConvex with its allocation failures as errors (for `initialize`)
    fn createTriangleBatchForConvexImpl(self: *DebugRenderer, get_support: SupportFunction, level: u32, out_bounds: ?*AABox) Allocator.Error!Batch {
        var vertices: std.ArrayList(Vertex) = .empty;
        defer vertices.deinit(self.allocator);
        var indices: std.ArrayList(u32) = .empty;
        defer indices.deinit(self.allocator);
        try self.create8thSphere(&indices, &vertices, Vec3.axisX(), Vec3.axisY(), Vec3.axisZ(), .init(0.25, 0.25), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisY(), Vec3.axisX().negate(), Vec3.axisZ(), .init(0.25, 0.75), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisY().negate(), Vec3.axisX(), Vec3.axisZ(), .init(0.25, 0.75), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisX().negate(), Vec3.axisY().negate(), Vec3.axisZ(), .init(0.25, 0.25), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisY(), Vec3.axisX(), Vec3.axisZ().negate(), .init(0.25, 0.75), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisX().negate(), Vec3.axisY(), Vec3.axisZ().negate(), .init(0.25, 0.25), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisX(), Vec3.axisY().negate(), Vec3.axisZ().negate(), .init(0.25, 0.25), get_support, level);
        try self.create8thSphere(&indices, &vertices, Vec3.axisY().negate(), Vec3.axisX().negate(), Vec3.axisZ().negate(), .init(0.25, 0.75), get_support, level);

        if (out_bounds) |bounds|
            bounds.* = calculateBounds(vertices.items);

        return self.createTriangleBatchIndexed(vertices.items, indices.items);
    }

    /// Create a geometry for a convex shape using its support function: 4 LODs. Zolt: an empty GeometryRef when out of
    /// memory (latched, also when the renderer could not create a batch; the caller draws nothing).
    pub fn createTriangleGeometryForConvex(self: *DebugRenderer, get_support: SupportFunction) GeometryRef {
        var geometry: GeometryRef = .empty;

        // Iterate over levels
        var level: u32 = max_level;
        while (level >= 1) : (level -= 1) {
            // Determine at which distance this level should be active
            const distance = lod_distance_for_level[max_level - level];

            // Create triangle batch and only calculate bounds for highest LOD level
            var bounds: AABox = .empty;
            const batch = self.createTriangleBatchForConvexImpl(get_support, level, if (geometry.get() == null) &bounds else null) catch {
                self.setOutOfMemory();
                geometry.deinit();
                return .empty;
            };
            if (self.isDroppedBatch(batch)) {
                geometry.deinit();
                return .empty;
            }

            // Construct geometry in the first iteration
            if (geometry.get() == null) {
                geometry = GeometryRef.init(Geometry.create(self.allocator, bounds) catch {
                    var b = batch;
                    b.deinit();
                    self.setOutOfMemory();
                    return .empty;
                });
            }

            // Add the LOD
            geometry.get().?.addLOD(batch, distance) catch {
                self.setOutOfMemory();
                geometry.deinit();
                return .empty;
            };
        }

        return geometry;
    }

    /// Initialize the system, must be called from the constructor of the DebugRenderer implementation.
    /// Creates the 25 predefined batches in Jolt's order. Zolt: an error when an allocation fails, also when the
    /// renderer latched one while it created a batch. What was created stays and is released by the destructor.
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

            const batch = try self.checkBatch(self.createTriangleBatchIndexed(box_vertices.items, box_indices.items));
            self.box.set(try Geometry.createWithBatch(self.allocator, batch, AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1))));
        }

        // Support function that returns a unit sphere
        const sphere_support: SupportFunction = .{ .function = unitSphereSupport };

        // Construct geometries
        self.sphere.set(try Geometry.create(self.allocator, AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1))));
        self.capsule_bottom.set(try Geometry.create(self.allocator, AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 0, 1))));
        self.capsule_top.set(try Geometry.create(self.allocator, AABox.init(Vec3.init(-1, 0, -1), Vec3.init(1, 1, 1))));
        self.capsule_mid.set(try Geometry.create(self.allocator, AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1))));
        self.open_cone.set(try Geometry.create(self.allocator, AABox.init(Vec3.init(-1, 0, -1), Vec3.init(1, 1, 1))));
        self.cylinder.set(try Geometry.create(self.allocator, AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1))));

        // Iterate over levels
        var level: u32 = max_level;
        while (level >= 1) : (level -= 1) {
            // Determine at which distance this level should be active
            const distance = lod_distance_for_level[max_level - level];

            // Sphere
            try self.sphere.get().?.addLOD(try self.checkBatch(try self.createTriangleBatchForConvexImpl(sphere_support, level, null)), distance);

            // Capsule bottom half sphere
            {
                var capsule_bottom_vertices: std.ArrayList(Vertex) = .empty;
                defer capsule_bottom_vertices.deinit(self.allocator);
                var capsule_bottom_indices: std.ArrayList(u32) = .empty;
                defer capsule_bottom_indices.deinit(self.allocator);
                try self.create8thSphere(&capsule_bottom_indices, &capsule_bottom_vertices, Vec3.axisX().negate(), Vec3.axisY().negate(), Vec3.axisZ(), .init(0.25, 0.25), sphere_support, level);
                try self.create8thSphere(&capsule_bottom_indices, &capsule_bottom_vertices, Vec3.axisY().negate(), Vec3.axisX(), Vec3.axisZ(), .init(0.25, 0.75), sphere_support, level);
                try self.create8thSphere(&capsule_bottom_indices, &capsule_bottom_vertices, Vec3.axisX(), Vec3.axisY().negate(), Vec3.axisZ().negate(), .init(0.25, 0.25), sphere_support, level);
                try self.create8thSphere(&capsule_bottom_indices, &capsule_bottom_vertices, Vec3.axisY().negate(), Vec3.axisX().negate(), Vec3.axisZ().negate(), .init(0.25, 0.75), sphere_support, level);
                try self.capsule_bottom.get().?.addLOD(try self.checkBatch(self.createTriangleBatchIndexed(capsule_bottom_vertices.items, capsule_bottom_indices.items)), distance);
            }

            // Capsule top half sphere
            {
                var capsule_top_vertices: std.ArrayList(Vertex) = .empty;
                defer capsule_top_vertices.deinit(self.allocator);
                var capsule_top_indices: std.ArrayList(u32) = .empty;
                defer capsule_top_indices.deinit(self.allocator);
                try self.create8thSphere(&capsule_top_indices, &capsule_top_vertices, Vec3.axisX(), Vec3.axisY(), Vec3.axisZ(), .init(0.25, 0.75), sphere_support, level);
                try self.create8thSphere(&capsule_top_indices, &capsule_top_vertices, Vec3.axisY(), Vec3.axisX().negate(), Vec3.axisZ(), .init(0.25, 0.25), sphere_support, level);
                try self.create8thSphere(&capsule_top_indices, &capsule_top_vertices, Vec3.axisY(), Vec3.axisX(), Vec3.axisZ().negate(), .init(0.25, 0.25), sphere_support, level);
                try self.create8thSphere(&capsule_top_indices, &capsule_top_vertices, Vec3.axisX().negate(), Vec3.axisY(), Vec3.axisZ().negate(), .init(0.25, 0.75), sphere_support, level);
                try self.capsule_top.get().?.addLOD(try self.checkBatch(self.createTriangleBatchIndexed(capsule_top_vertices.items, capsule_top_indices.items)), distance);
            }

            // Capsule middle part
            {
                var capsule_mid_vertices: std.ArrayList(Vertex) = .empty;
                defer capsule_mid_vertices.deinit(self.allocator);
                var capsule_mid_indices: std.ArrayList(u32) = .empty;
                defer capsule_mid_indices.deinit(self.allocator);
                for (0..4) |q| {
                    const uv: Float2 = if ((q & 1) == 0) .init(0.25, 0.25) else .init(0.25, 0.75);

                    const start_idx: u32 = @intCast(capsule_mid_vertices.items.len);

                    const num_parts = @as(u32, 1) << @intCast(level);
                    for (0..num_parts + 1) |i| {
                        const angle = 0.5 * math.pi * (@as(f32, @floatFromInt(q)) + @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_parts)));
                        const s = trig.sin(angle);
                        const c = trig.cos(angle);
                        const vt = Float3.init(s, 1.0, c);
                        const vb = Float3.init(s, -1.0, c);
                        const n = Float3.init(s, 0, c);

                        try capsule_mid_vertices.append(self.allocator, .{ .position = vt, .normal = n, .uv = uv, .color = Color.white });
                        try capsule_mid_vertices.append(self.allocator, .{ .position = vb, .normal = n, .uv = uv, .color = Color.white });
                    }

                    for (0..num_parts) |i| {
                        const start = start_idx + 2 * @as(u32, @intCast(i));

                        try capsule_mid_indices.append(self.allocator, start);
                        try capsule_mid_indices.append(self.allocator, start + 1);
                        try capsule_mid_indices.append(self.allocator, start + 3);

                        try capsule_mid_indices.append(self.allocator, start);
                        try capsule_mid_indices.append(self.allocator, start + 3);
                        try capsule_mid_indices.append(self.allocator, start + 2);
                    }
                }
                try self.capsule_mid.get().?.addLOD(try self.checkBatch(self.createTriangleBatchIndexed(capsule_mid_vertices.items, capsule_mid_indices.items)), distance);
            }

            // Open cone
            {
                var open_cone_vertices: std.ArrayList(Vertex) = .empty;
                defer open_cone_vertices.deinit(self.allocator);
                var open_cone_indices: std.ArrayList(u32) = .empty;
                defer open_cone_indices.deinit(self.allocator);
                for (0..4) |q| {
                    const uv: Float2 = if ((q & 1) == 0) .init(0.25, 0.25) else .init(0.25, 0.75);

                    const start_idx: u32 = @intCast(open_cone_vertices.items.len);

                    const num_parts = @as(u32, 2) << @intCast(level);
                    const vt = Float3.init(0, 0, 0);
                    for (0..num_parts + 1) |i| {
                        // Calculate bottom vertex
                        const angle = 0.5 * math.pi * (@as(f32, @floatFromInt(q)) + @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_parts)));
                        const s = trig.sin(angle);
                        const c = trig.cos(angle);
                        const vb = Float3.init(s, 1.0, c);

                        // Calculate normal
                        // perpendicular = Y cross vb (perpendicular to the plane in which 0, y and vb exists)
                        // normal = perpendicular cross vb (normal to the edge 0 vb)
                        const normal = Vec3.init(s, -math.square(s) - math.square(c), c).normalized();
                        var n: Float3 = undefined;
                        normal.storeFloat3(&n);

                        try open_cone_vertices.append(self.allocator, .{ .position = vt, .normal = n, .uv = uv, .color = Color.white });
                        try open_cone_vertices.append(self.allocator, .{ .position = vb, .normal = n, .uv = uv, .color = Color.white });
                    }

                    for (0..num_parts) |i| {
                        const start = start_idx + 2 * @as(u32, @intCast(i));

                        try open_cone_indices.append(self.allocator, start);
                        try open_cone_indices.append(self.allocator, start + 1);
                        try open_cone_indices.append(self.allocator, start + 3);
                    }
                }
                try self.open_cone.get().?.addLOD(try self.checkBatch(self.createTriangleBatchIndexed(open_cone_vertices.items, open_cone_indices.items)), distance);
            }

            // Cylinder
            try self.cylinder.get().?.addLOD(try self.checkBatch(try self.createCylinder(1.0, -1.0, 1.0, 1.0, level)), distance);
        }
    }

    /// The support function of initialize: a unit sphere
    fn unitSphereSupport(context: ?*anyopaque, direction: Vec3) Vec3 {
        _ = context;
        return direction;
    }

    /// Zolt: construction fails when the renderer could not create a batch (it returns an empty Batch or a batch, and
    /// latches the failure, R5)
    fn checkBatch(self: *DebugRenderer, batch: Batch) Allocator.Error!Batch {
        if (self.isOutOfMemory()) {
            var b = batch;
            b.deinit();
            return error.OutOfMemory;
        }
        return batch;
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

    /// Helper functions to create a vertex and index buffer for a sphere
    fn create8thSphereRecursive(self: *DebugRenderer, indices: *std.ArrayList(u32), vertices: *std.ArrayList(Vertex), dir1: Vec3, idx1: *u32, dir2: Vec3, idx2: *u32, dir3: Vec3, idx3: *u32, uv: Float2, get_support: SupportFunction, level: u32) Allocator.Error!void {
        if (level == 0) {
            if (idx1.* == 0xffffffff) {
                idx1.* = @intCast(vertices.items.len);
                var position: Float3 = undefined;
                var normal: Float3 = undefined;
                get_support.call(dir1).storeFloat3(&position);
                dir1.storeFloat3(&normal);
                try vertices.append(self.allocator, .{ .position = position, .normal = normal, .uv = uv, .color = Color.white });
            }

            if (idx2.* == 0xffffffff) {
                idx2.* = @intCast(vertices.items.len);
                var position: Float3 = undefined;
                var normal: Float3 = undefined;
                get_support.call(dir2).storeFloat3(&position);
                dir2.storeFloat3(&normal);
                try vertices.append(self.allocator, .{ .position = position, .normal = normal, .uv = uv, .color = Color.white });
            }

            if (idx3.* == 0xffffffff) {
                idx3.* = @intCast(vertices.items.len);
                var position: Float3 = undefined;
                var normal: Float3 = undefined;
                get_support.call(dir3).storeFloat3(&position);
                dir3.storeFloat3(&normal);
                try vertices.append(self.allocator, .{ .position = position, .normal = normal, .uv = uv, .color = Color.white });
            }

            try indices.append(self.allocator, idx1.*);
            try indices.append(self.allocator, idx2.*);
            try indices.append(self.allocator, idx3.*);
        } else {
            const center1 = dir1.add(dir2).normalized();
            const center2 = dir2.add(dir3).normalized();
            const center3 = dir3.add(dir1).normalized();

            var c_idx1: u32 = 0xffffffff;
            var c_idx2: u32 = 0xffffffff;
            var c_idx3: u32 = 0xffffffff;

            try self.create8thSphereRecursive(indices, vertices, dir1, idx1, center1, &c_idx1, center3, &c_idx3, uv, get_support, level - 1);
            try self.create8thSphereRecursive(indices, vertices, center1, &c_idx1, center2, &c_idx2, center3, &c_idx3, uv, get_support, level - 1);
            try self.create8thSphereRecursive(indices, vertices, center1, &c_idx1, dir2, idx2, center2, &c_idx2, uv, get_support, level - 1);
            try self.create8thSphereRecursive(indices, vertices, center3, &c_idx3, center2, &c_idx2, dir3, idx3, uv, get_support, level - 1);
        }
    }

    fn create8thSphere(self: *DebugRenderer, indices: *std.ArrayList(u32), vertices: *std.ArrayList(Vertex), dir1: Vec3, dir2: Vec3, dir3: Vec3, uv: Float2, get_support: SupportFunction, level: u32) Allocator.Error!void {
        var idx1: u32 = 0xffffffff;
        var idx2: u32 = 0xffffffff;
        var idx3: u32 = 0xffffffff;

        try self.create8thSphereRecursive(indices, vertices, dir1, &idx1, dir2, &idx2, dir3, &idx3, uv, get_support, level);
    }

    /// Helper functions to create a vertex and index buffer for a cylinder. The batch is the renderer's (it latches its
    /// own allocation failures), the error is a failure of the vertex and index buffers.
    fn createCylinder(self: *DebugRenderer, top: f32, bottom: f32, top_radius: f32, bottom_radius: f32, level: u32) Allocator.Error!Batch {
        var cylinder_vertices: std.ArrayList(Vertex) = .empty;
        defer cylinder_vertices.deinit(self.allocator);
        var cylinder_indices: std.ArrayList(u32) = .empty;
        defer cylinder_indices.deinit(self.allocator);

        for (0..4) |q| {
            const uv: Float2 = if ((q & 1) == 0) .init(0.25, 0.75) else .init(0.25, 0.25);

            const center_start_idx: u32 = @intCast(cylinder_vertices.items.len);

            const nt = Float3.init(0.0, 1.0, 0.0);
            const nb = Float3.init(0.0, -1.0, 0.0);
            try cylinder_vertices.append(self.allocator, .{ .position = .init(0.0, top, 0.0), .normal = nt, .uv = uv, .color = Color.white });
            try cylinder_vertices.append(self.allocator, .{ .position = .init(0.0, bottom, 0.0), .normal = nb, .uv = uv, .color = Color.white });

            const vtx_start_idx: u32 = @intCast(cylinder_vertices.items.len);

            const num_parts = @as(u32, 1) << @intCast(level);
            for (0..num_parts + 1) |i| {
                // Calculate top and bottom vertex
                const angle = 0.5 * math.pi * (@as(f32, @floatFromInt(q)) + @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_parts)));
                const s = trig.sin(angle);
                const c = trig.cos(angle);
                const vt = Float3.init(top_radius * s, top, top_radius * c);
                const vb = Float3.init(bottom_radius * s, bottom, bottom_radius * c);

                // Calculate normal
                const edge = Vec3.fromFloat3(vt).sub(Vec3.fromFloat3(vb));
                var n: Float3 = undefined;
                edge.cross(Vec3.init(s, 0, c).cross(edge)).normalized().storeFloat3(&n);

                try cylinder_vertices.append(self.allocator, .{ .position = vt, .normal = nt, .uv = uv, .color = Color.white });
                try cylinder_vertices.append(self.allocator, .{ .position = vb, .normal = nb, .uv = uv, .color = Color.white });
                try cylinder_vertices.append(self.allocator, .{ .position = vt, .normal = n, .uv = uv, .color = Color.white });
                try cylinder_vertices.append(self.allocator, .{ .position = vb, .normal = n, .uv = uv, .color = Color.white });
            }

            for (0..num_parts) |i| {
                const start = vtx_start_idx + 4 * @as(u32, @intCast(i));

                // Top
                try cylinder_indices.append(self.allocator, center_start_idx);
                try cylinder_indices.append(self.allocator, start);
                try cylinder_indices.append(self.allocator, start + 4);

                // Bottom
                try cylinder_indices.append(self.allocator, center_start_idx + 1);
                try cylinder_indices.append(self.allocator, start + 5);
                try cylinder_indices.append(self.allocator, start + 1);

                // Side
                try cylinder_indices.append(self.allocator, start + 2);
                try cylinder_indices.append(self.allocator, start + 3);
                try cylinder_indices.append(self.allocator, start + 7);

                try cylinder_indices.append(self.allocator, start + 2);
                try cylinder_indices.append(self.allocator, start + 7);
                try cylinder_indices.append(self.allocator, start + 6);
            }
        }

        return self.createTriangleBatchIndexed(cylinder_vertices.items, cylinder_indices.items);
    }
};

/// RMat44 * Mat44 in both precisions
fn rmat44MulMat44(a: RMat44, b: Mat44) RMat44 {
    return if (Core.double_precision) a.mulMat44(b) else a.mul(b);
}

/// RMat44(Vec4Arg inC1, Vec4Arg inC2, Vec4Arg inC3, RVec3Arg inC4) in both precisions
fn rmat44FromColumns(c1: Vec4, c2: Vec4, c3: Vec4, c4: RVec3) RMat44 {
    return if (Core.double_precision) DMat44.init(c1, c2, c3, c4) else Mat44.fromColumnsTranslation(c1, c2, c3, c4);
}

/// A renderer for the tests: counts the calls (atomics: the lock test draws from several threads) and keeps the triangle
/// count of each batch
const TestRenderer = struct {
    pub const overrides = .{ .drawLine, .drawTriangle, .createTriangleBatch, .createTriangleBatchIndexed, .drawGeometry, .drawText3D };

    const BatchImpl = struct {
        pub const overrides = .{};
        base: DebugRenderer.BatchTarget,
        num_triangles: usize,
    };

    base: DebugRenderer,
    num_lines: std.atomic.Value(u32) = .init(0),
    num_batches: std.atomic.Value(u32) = .init(0),
    num_draws: std.atomic.Value(u32) = .init(0),
    /// The last geometry drawn and its world space bounds and LOD scale (single threaded tests only)
    last_geometry: ?*const DebugRenderer.Geometry = null,
    last_world_space_bounds: AABox = .empty,
    last_lod_scale_sq: f32 = 0,
    last_cull_mode: DebugRenderer.CullMode = .cull_back_face,

    fn init(self: *TestRenderer, allocator: Allocator) Allocator.Error!void {
        self.* = .{ .base = .init(TestRenderer, allocator, std.testing.io) };
        errdefer self.base.deinit();
        try self.base.initialize();
    }

    pub fn drawLine(self: *TestRenderer, from: RVec3, to: RVec3, color: Color) void {
        _ = .{ from, to, color };
        _ = self.num_lines.fetchAdd(1, .monotonic);
    }
    pub fn drawTriangle(self: *TestRenderer, v1: RVec3, v2: RVec3, v3: RVec3, color: Color, cast_shadow: DebugRenderer.CastShadow) void {
        _ = .{ self, v1, v2, v3, color, cast_shadow };
    }
    pub fn createTriangleBatch(self: *TestRenderer, triangles: []const DebugRenderer.Triangle) DebugRenderer.Batch {
        return self.newBatch(triangles.len);
    }
    pub fn createTriangleBatchIndexed(self: *TestRenderer, vertices: []const DebugRenderer.Vertex, indices: []const u32) DebugRenderer.Batch {
        _ = vertices;
        return self.newBatch(indices.len / 3);
    }
    pub fn drawGeometry(self: *TestRenderer, model_matrix: RMat44, world_space_bounds: AABox, lod_scale_sq: f32, model_color: Color, geometry: *const DebugRenderer.Geometry, cull_mode: DebugRenderer.CullMode, cast_shadow: DebugRenderer.CastShadow, draw_mode: DebugRenderer.DrawMode) void {
        _ = .{ model_matrix, model_color, cast_shadow, draw_mode };
        _ = self.num_draws.fetchAdd(1, .monotonic);
        self.last_geometry = geometry;
        self.last_world_space_bounds = world_space_bounds;
        self.last_lod_scale_sq = lod_scale_sq;
        self.last_cull_mode = cull_mode;
    }
    pub fn drawText3D(self: *TestRenderer, position: RVec3, string: []const u8, color: Color, height: f32) void {
        _ = .{ self, position, string, color, height };
    }

    fn newBatch(self: *TestRenderer, num_triangles: usize) DebugRenderer.Batch {
        const batch = self.base.allocator.create(BatchImpl) catch {
            self.base.setOutOfMemory();
            return .empty;
        };
        batch.* = .{ .base = .init(BatchImpl, self.base.allocator), .num_triangles = num_triangles };
        _ = self.num_batches.fetchAdd(1, .monotonic);
        return .init(&batch.base);
    }

    fn numTriangles(lod: DebugRenderer.LOD) usize {
        return @as(*const BatchImpl, @alignCast(@fieldParentPtr("base", lod.triangle_batch.get().?))).num_triangles;
    }

    fn batches(self: *const TestRenderer) u32 {
        return self.num_batches.load(.monotonic);
    }
};

/// The support function of a unit sphere (for the tests)
fn testSphereSupport(context: ?*anyopaque, direction: Vec3) Vec3 {
    _ = context;
    return direction;
}

test "DebugRenderer: initialize creates Jolt's 25 predefined batches" {
    var r: TestRenderer = undefined;
    try r.init(std.testing.allocator);
    defer r.base.deinit();
    try std.testing.expectEqual(@as(u32, 25), r.batches());

    // The box: 6 quads in one LOD
    const box = r.base.box.get().?;
    try std.testing.expectEqual(@as(usize, 1), box.lods.items.len);
    try std.testing.expectEqual(@as(usize, 12), TestRenderer.numTriangles(box.lods.items[0]));
    try std.testing.expectEqual(math.large_float, box.lods.items[0].distance);

    // The other geometries: 4 LODs, from level 4 at distance 5 to level 1 at cLargeFloat
    const geometries = [_]*const DebugRenderer.Geometry{ r.base.sphere.get().?, r.base.capsule_bottom.get().?, r.base.capsule_top.get().?, r.base.capsule_mid.get().?, r.base.open_cone.get().?, r.base.cylinder.get().? };
    for (geometries, 0..) |geometry, g| {
        try std.testing.expectEqual(@as(usize, 4), geometry.lods.items.len);
        for (geometry.lods.items, 0..) |lod, i| {
            const level: u5 = @intCast(max_level - i);
            const expected: usize = switch (g) {
                0 => 8 * (@as(usize, 1) << (2 * level)), // 8 eighths of a sphere, 4^level triangles each
                1, 2 => 4 * (@as(usize, 1) << (2 * level)), // 4 eighths
                3 => 4 * 2 * (@as(usize, 1) << level), // 4 quarters of 2^level quads
                4 => 4 * (@as(usize, 2) << level), // 4 quarters of 2^(level + 1) triangles
                5 => 4 * 4 * (@as(usize, 1) << level), // top, bottom and a quad per part
                else => unreachable,
            };
            try std.testing.expectEqual(expected, TestRenderer.numTriangles(lod));
            try std.testing.expectEqual(lod_distance_for_level[i], lod.distance);
        }
    }
    try std.testing.expect(r.base.capsule_bottom.get().?.bounds.max.eql(Vec3.init(1, 0, 1)));
    try std.testing.expect(r.base.open_cone.get().?.bounds.min.eql(Vec3.init(-1, 0, -1)));
}

test "DebugRenderer: the caches hit within a frame and in the next one, -0 is +0, a NaN key always misses" {
    var r: TestRenderer = undefined;
    try r.init(std.testing.allocator);
    defer r.base.deinit();
    const d = &r.base;
    const m = RMat44.identity();

    // Tapered cylinder: 4 LODs on a miss
    var n = r.batches();
    d.drawTaperedCylinder(m, 1, -1, 0.5, 0.25, Color.white, .{});
    try std.testing.expectEqual(n + 4, r.batches());
    const first = r.last_geometry.?;
    d.drawTaperedCylinder(m, 1, -1, 0.5, 0.25, Color.white, .{}); // Hit
    try std.testing.expectEqual(first, r.last_geometry.?);
    d.nextFrame();
    d.drawTaperedCylinder(m, 1, -1, 0.5, 0.25, Color.white, .{}); // Hit in the cache of the previous frame
    try std.testing.expectEqual(n + 4, r.batches());
    try std.testing.expectEqual(first, r.last_geometry.?);
    d.nextFrame();
    d.nextFrame();
    d.drawTaperedCylinder(m, 1, -1, 0.5, 0.25, Color.white, .{}); // Released after a frame without a draw
    try std.testing.expectEqual(n + 8, r.batches());

    // -0 and +0 are the same key (Jolt's Hash<float> and ==)
    n = r.batches();
    d.drawTaperedCylinder(m, 0, -1, 0.5, 0.25, Color.white, .{});
    d.drawTaperedCylinder(m, -0.0, -1, 0.5, 0.25, Color.white, .{});
    try std.testing.expectEqual(n + 4, r.batches());

    // A NaN key misses every time, also within a frame
    const nan = std.math.nan(f32);
    n = r.batches();
    d.drawTaperedCylinder(m, nan, -1, 0.5, 0.25, Color.white, .{});
    d.drawTaperedCylinder(m, nan, -1, 0.5, 0.25, Color.white, .{});
    try std.testing.expectEqual(n + 8, r.batches());
    d.nextFrame();
    d.drawTaperedCylinder(m, nan, -1, 0.5, 0.25, Color.white, .{});
    try std.testing.expectEqual(n + 12, r.batches());

    // Swing cone, swing pyramid and pie: one batch on a miss
    n = r.batches();
    d.drawSwingConeLimits(m, 0.5, 0.25, 1, Color.white, .{});
    d.drawSwingConeLimits(m, 0.5, 0.25, 2, Color.white, .{});
    d.drawSwingPyramidLimits(m, -0.5, 0.5, -0.25, 0.25, 1, Color.white, .{});
    d.drawSwingPyramidLimits(m, -0.5, 0.5, -0.25, 0.25, 1, Color.white, .{});
    d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 0, 1, Color.white, .{});
    d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 0.5, 1.5, Color.white, .{}); // The same delta angle
    try std.testing.expectEqual(n + 3, r.batches());
    try std.testing.expectEqual(DebugRenderer.CullMode.off, r.last_cull_mode);
    d.nextFrame();
    d.drawSwingConeLimits(m, 0.5, 0.25, 1, Color.white, .{});
    d.drawSwingPyramidLimits(m, -0.5, 0.5, -0.25, 0.25, 1, Color.white, .{});
    d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 0, 1, Color.white, .{});
    try std.testing.expectEqual(n + 3, r.batches());

    // Nothing to draw: a swing cone without swing, a pie with equal angles. Zolt: a pie with a NaN or infinite range
    // (Jolt converts it to int there) draws nothing either.
    const draws = r.num_draws.load(.monotonic);
    d.drawSwingConeLimits(m, 0, 0, 1, Color.white, .{});
    d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 1, 1, Color.white, .{});
    d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 0, nan, Color.white, .{});
    d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), -std.math.inf(f32), 0, Color.white, .{});
    try std.testing.expectEqual(draws, r.num_draws.load(.monotonic));
    try std.testing.expectEqual(n + 3, r.batches());
}

test "DebugRenderer: helpers with the predefined geometry, wire helpers and batch helpers" {
    var r: TestRenderer = undefined;
    try r.init(std.testing.allocator);
    defer r.base.deinit();
    const d = &r.base;

    // DrawCapsule: three draws with the world space bounds of the whole capsule and the squared radius as LOD scale
    d.drawCapsule(RMat44.translation(RVec3.init(1, 2, 3)), 2, 0.5, Color.white, .{});
    try std.testing.expectEqual(r.base.capsule_mid.get().?, r.last_geometry.?);
    try std.testing.expect(r.last_world_space_bounds.min.isClose(Vec3.init(0.5, -0.5, 2.5), .{}));
    try std.testing.expect(r.last_world_space_bounds.max.isClose(Vec3.init(1.5, 4.5, 3.5), .{}));
    try std.testing.expectEqual(@as(f32, 0.25), r.last_lod_scale_sq);

    // DrawCylinder, DrawSphere, DrawOpenCone (nothing for a zero angle)
    d.drawCylinder(RMat44.identity(), 2, 3, Color.white, .{});
    try std.testing.expectEqual(r.base.cylinder.get().?, r.last_geometry.?);
    try std.testing.expectEqual(@as(f32, 9), r.last_lod_scale_sq);
    d.drawSphere(RVec3.zero(), 2, Color.white, .{});
    try std.testing.expectEqual(r.base.sphere.get().?, r.last_geometry.?);
    const draws = r.num_draws.load(.monotonic);
    d.drawOpenCone(RVec3.zero(), Vec3.axisY(), Vec3.axisX(), 0, 1, Color.white, .{});
    try std.testing.expectEqual(draws, r.num_draws.load(.monotonic));
    d.drawOpenCone(RVec3.zero(), Vec3.axisY(), Vec3.axisX(), -0.5, 1, Color.white, .{});
    try std.testing.expectEqual(r.base.open_cone.get().?, r.last_geometry.?);

    // The wire helpers: lines only
    var lines = r.num_lines.load(.monotonic);
    d.drawWireSphere(RVec3.zero(), 1, Color.white, .{ .level = 0 });
    try std.testing.expectEqual(lines + 8 * 3, r.num_lines.load(.monotonic));
    lines = r.num_lines.load(.monotonic);
    d.drawWireUnitSphere(RMat44.identity(), Color.white, .{});
    try std.testing.expectEqual(lines + 8 * 64 * 3, r.num_lines.load(.monotonic));
    lines = r.num_lines.load(.monotonic);
    d.drawCoordinateSystem(RMat44.identity(), .{});
    d.drawPlane(RVec3.zero(), Vec3.axisY(), Color.white, 1);
    d.drawWireTriangle(RVec3.zero(), RVec3.init(1, 0, 0), RVec3.init(0, 1, 0), Color.white);
    d.drawWirePolygon(RMat44.identity(), &.{ Vec3.zero(), Vec3.axisX(), Vec3.axisY() }, Color.white, .{});
    d.drawWirePolygon(RMat44.identity(), &.{}, Color.white, .{ .arrow_size = 1 });
    d.drawWireBoxOriented(.init(Mat44.identity(), Vec3.one()), Color.white);
    try std.testing.expectEqual(lines + 3 * 3 + (6 + 3) + 3 + 3 + 12, r.num_lines.load(.monotonic));

    // The batch helpers
    const n = r.batches();
    var convex = d.createTriangleGeometryForConvex(.{ .function = testSphereSupport });
    defer convex.deinit();
    try std.testing.expectEqual(n + 4, r.batches());
    try std.testing.expectEqual(@as(usize, 4), convex.get().?.lods.items.len);
    try std.testing.expect(convex.get().?.bounds.max.isClose(Vec3.one(), .{}));
    var bounds: AABox = .empty;
    var batch = d.createTriangleBatchForConvex(.{ .function = testSphereSupport }, 0, .{ .out_bounds = &bounds });
    defer batch.deinit();
    try std.testing.expectEqual(@as(usize, 8), @as(*const TestRenderer.BatchImpl, @alignCast(@fieldParentPtr("base", batch.get().?))).num_triangles);
    try std.testing.expect(bounds.min.isClose(Vec3.replicate(-1), .{}));
    var list = d.createTriangleBatchVertexList(&.{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0) }, &.{.init(0, 1, 2)});
    defer list.deinit();
    try std.testing.expectEqual(n + 6, r.batches());
}

test "DebugRenderer: Triangle with UVs" {
    const t = DebugRenderer.Triangle.initUV(Vec3.zero(), Vec3.init(2, 0, 0), Vec3.init(0, 0, -2), Color.red, Vec3.init(1, 0, 0), Vec3.axisX());
    try std.testing.expectEqual(Float3.init(0, 1, 0), t.v[0].normal);
    try std.testing.expectEqual(Float2.init(-1, 0), t.v[0].uv);
    try std.testing.expectEqual(Float2.init(1, 0), t.v[1].uv);
    try std.testing.expectEqual(Float2.init(-1, 2), t.v[2].uv); // axis2 = normal x direction = -Z
    try std.testing.expect(t.v[2].color.eql(Color.red));
}

test "DebugRenderer: allocation failures" {
    // Construction: an error and no leak, or success
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = fail_index });
        var r: TestRenderer = undefined;
        r.init(failing.allocator()) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        r.base.deinit();
        break;
    }

    // Drawing: each allocation of a helper fails once, the primitive is dropped, the failure latched, nothing leaks
    const Helpers = struct {
        fn draw(d: *DebugRenderer, i: usize) void {
            const m = RMat44.identity();
            switch (i) {
                0 => d.drawTaperedCylinder(m, 1, -1, 0.5, 0.25, Color.white, .{}),
                1 => d.drawSwingConeLimits(m, 0.5, 0.25, 1, Color.white, .{}),
                2 => d.drawSwingPyramidLimits(m, -0.5, 0.5, -0.25, 0.25, 1, Color.white, .{}),
                3 => d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 0, 1, Color.white, .{}),
                4 => {
                    var g = d.createTriangleGeometryForConvex(.{ .function = testSphereSupport });
                    g.deinit();
                },
                5 => {
                    var b = d.createTriangleBatchForConvex(.{ .function = testSphereSupport }, 2, .{});
                    b.deinit();
                },
                6 => {
                    var b = d.createTriangleBatchVertexList(&.{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0) }, &.{.init(0, 1, 2)});
                    b.deinit();
                },
                else => unreachable,
            }
        }
    };
    for (0..7) |helper| {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{});
        var r: TestRenderer = undefined;
        try r.init(failing.allocator());
        defer r.base.deinit();
        var offset: usize = 0;
        while (true) : (offset += 1) {
            failing.fail_index = failing.alloc_index + offset;
            const draws = r.num_draws.load(.monotonic);
            Helpers.draw(&r.base, helper);
            failing.fail_index = std.math.maxInt(usize);
            r.base.nextFrame();
            r.base.nextFrame(); // Empty the caches, so that the next round misses again
            if (!r.base.isOutOfMemory()) {
                try std.testing.expect(helper >= 4 or r.num_draws.load(.monotonic) == draws + 1);
                break;
            }
            try std.testing.expectEqual(draws, r.num_draws.load(.monotonic)); // Dropped
            r.base.out_of_memory.store(false, .monotonic);
        }
        try std.testing.expect(offset > 0);
    }
}

test "DebugRenderer: the caches are locked, threads that draw the same shape create one geometry (R6)" {
    var r: TestRenderer = undefined;
    try r.init(std.testing.allocator);
    defer r.base.deinit();
    const n = r.batches();

    const num_threads = 4;
    var threads: [num_threads]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, struct {
        fn run(d: *DebugRenderer) void {
            for (0..50) |i| {
                const k: f32 = @floatFromInt(i % 5);
                d.drawTaperedCylinder(RMat44.identity(), 1, -1, k, 0.5, Color.white, .{});
                d.drawSwingConeLimits(RMat44.identity(), 0.5, 0.1 * k, 1, Color.white, .{});
                d.drawPie(RVec3.zero(), 1, Vec3.axisY(), Vec3.axisX(), 0, 1 + k, Color.white, .{});
            }
        }
    }.run, .{&r.base});
    for (threads) |t| t.join();

    // 5 keys of each: tapered cylinders with 4 batches, swing cones and pies with 1
    try std.testing.expectEqual(n + 5 * 4 + 5 + 5, r.batches());
    try std.testing.expectEqual(@as(u32, num_threads * 50 * 3), r.num_draws.load(.monotonic));
}
