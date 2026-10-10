//! Port of: Jolt/Physics/Collision/Shape/MeshShape.h, Jolt/Physics/Collision/Shape/MeshShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2): `MeshShape` derives from `Shape`
//! directly (`base: Shape`), `overrides` lists every C++ `override` in header order.
//! - Settings: `MeshShapeSettings` owns its arrays (`triangle_vertices`, `indexed_triangles`, `materials`), allocated
//!   with the settings' allocator (`base.allocator`) and freed by the destructor chain. The C++ constructors
//!   `MeshShapeSettings(const TriangleList &, PhysicsMaterialList)` and `MeshShapeSettings(VertexList,
//!   IndexedTriangleList, PhysicsMaterialList)` are `init(allocator, triangles, .{ .materials })` and
//!   `initIndexed(allocator, vertices, triangles, .{ .materials })` (they copy the arrays instead of moving them,
//!   `materials` adds a reference to each material) and return `Allocator.Error` because they allocate (Indexify and
//!   Sanitize); `sanitize()` allocates its set of triangles and returns `Allocator.Error` too.
//! - The tree is a `ByteBuffer` built with AABBTreeBuilder + AABBTreeToBuffer(TriangleCodecIndexed8BitPackSOA4Flags,
//!   NodeCodecQuadTreeHalfFloat), owned by the shape (allocated with the shape's allocator). The splitter, builder and
//!   the other temporary arrays of the constructor use the allocator passed to `initFromSettings`.
//! - The template walkers `WalkTree` / `WalkTreePerTriangle` take the visitor as `anytype` (a pointer); Jolt's local
//!   `struct Visitor : public CollideConvexVsTriangles` (and the other triangle helpers) embed the helper as `base`.
//! - `DecodeSubShapeID(id, outTriangleBlock, outTriangleIndex)` returns `DecodedSubShapeID{ .triangle_block,
//!   .triangle_index }`; `MSGetTrianglesContext` keeps the output slices and write positions instead of Jolt's
//!   advancing `Float3 *` / `const PhysicsMaterial **` pointers.
//! - The enum `ETriangleFlags` becomes constants (`FLAGS_ACTIVE_EGDE_SHIFT`, Jolt's spelling, is
//!   `flags_active_edge_shift`), `NumTriangleBits` / `MaxTrianglesPerLeaf` are `num_triangle_bits` /
//!   `max_triangles_per_leaf`.
//! - Null entries in the material list (`RefConst.init(null)`, legal in Jolt and restored by `restoreWithChildren` /
//!   `restoreMaterialState`) are kept as they are: `getMaterialList`, `saveMaterialState` and the binary state see the
//!   null entry. Jolt's `GetMaterial` and the materials of `GetTrianglesNext` return nullptr for such an entry; the
//!   shared signatures `Shape.getMaterial` / `out_materials: ?[]*const PhysicsMaterial` cannot express a null
//!   material, so Zolt returns `PhysicsMaterial.default` instead (as `ConvexShape` does for its null material). The
//!   constructor options take non-null materials; append `.init(null)` to `MeshShapeSettings.materials` for a null entry.
//! - `RestoreMaterialState` allocates the material list (`Array::assign`), so `Shape.restoreMaterialState` returns
//!   `Allocator.Error!void` (the list uses the shape's allocator).
//! - `GetSubmergedVolume` is not supported (Jolt asserts): with asserts enabled it panics with Jolt's message, in a
//!   build without asserts it returns zeros (Jolt leaves the caller's out parameters untouched).
//! - The vertex indices of the triangles are checked before the triangles are read: Jolt reads the vertices of every
//!   triangle (ValidationContext, IsDegenerate) before it reaches its check "Vertex index %u is beyond vertex list",
//!   which is undefined behavior for an index beyond the vertex list. Zolt reports Jolt's error text in Jolt's loop
//!   order for such a mesh; for every mesh that Jolt handles defined the result is the same.
//! - JPH_DEBUG_RENDERER (Draw, sDrawTriangleGroups, sDrawTriangleOutlines and the cached debug geometry) is not ported
//!   yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const ByteBuffer = @import("../../../Core/ByteBuffer.zig").ByteBuffer;
const Color = @import("../../../Core/Color.zig").Color;
const HashCombine = @import("../../../Core/HashCombine.zig");
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const UnorderedMap = @import("../../../Core/UnorderedMap.zig").UnorderedMap;
const UnorderedSet = @import("../../../Core/UnorderedSet.zig").UnorderedSet;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const AABox4 = @import("../../../Geometry/AABox4.zig");
const IndexedTriangleFile = @import("../../../Geometry/IndexedTriangle.zig");
const IndexedTriangle = IndexedTriangleFile.IndexedTriangle;
const IndexedTriangleList = IndexedTriangleFile.IndexedTriangleList;
const indexify = @import("../../../Geometry/Indexify.zig").indexify;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RayInvDirection = RayAABox.RayInvDirection;
const rayAABox4 = RayAABox.rayAABox4;
const rayTriangle = @import("../../../Geometry/RayTriangle.zig").rayTriangle;
const Triangle = @import("../../../Geometry/Triangle.zig").Triangle;
const Float3File = @import("../../../Math/Float3.zig");
const Float3 = Float3File.Float3;
const VertexList = Float3File.VertexList;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const AABBTreeBuilderFile = @import("../../../AABBTree/AABBTreeBuilder.zig");
const AABBTreeBuilder = AABBTreeBuilderFile.AABBTreeBuilder;
const AABBTreeBuilderStats = AABBTreeBuilderFile.AABBTreeBuilderStats;
const AABBTreeToBuffer = @import("../../../AABBTree/AABBTreeToBuffer.zig").AABBTreeToBuffer;
const NodeCodecQuadTreeHalfFloat = @import("../../../AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.zig").NodeCodecQuadTreeHalfFloat;
const TriangleCodecIndexed8BitPackSOA4Flags = @import("../../../AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.zig").TriangleCodecIndexed8BitPackSOA4Flags;
const TriangleSplitter = @import("../../../TriangleSplitter/TriangleSplitter.zig").TriangleSplitter;
const TriangleSplitterBinning = @import("../../../TriangleSplitter/TriangleSplitterBinning.zig").TriangleSplitterBinning;
const TriangleSplitterMean = @import("../../../TriangleSplitter/TriangleSplitterMean.zig").TriangleSplitterMean;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const PhysicsMaterialList = ShapeFile.PhysicsMaterialList;
const PhysicsMaterialRefC = ShapeFile.PhysicsMaterialRefC;
const ConvexShape = @import("ConvexShape.zig").ConvexShape;
const SphereShape = @import("SphereShape.zig").SphereShape;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const BackFaceMode = @import("../BackFaceMode.zig").BackFaceMode;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const ActiveEdges = @import("../ActiveEdges.zig");
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const SortReverseAndStore = @import("../SortReverseAndStore.zig");
const sortReverseAndStore = SortReverseAndStore.sortReverseAndStore;
const countAndSortTrues = SortReverseAndStore.countAndSortTrues;
const CollideConvexVsTriangles = @import("../CollideConvexVsTriangles.zig").CollideConvexVsTriangles;
const CollideSphereVsTriangles = @import("../CollideSphereVsTriangles.zig").CollideSphereVsTriangles;
const CastConvexVsTriangles = @import("../CastConvexVsTriangles.zig").CastConvexVsTriangles;
const CastSphereVsTriangles = @import("../CastSphereVsTriangles.zig").CastSphereVsTriangles;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const CollideSoftBodyVerticesVsTriangles = @import("../CollideSoftBodyVerticesVsTriangles.zig").CollideSoftBodyVerticesVsTriangles;

// Codecs this mesh shape is using
const TriangleCodec = TriangleCodecIndexed8BitPackSOA4Flags;
const NodeCodec = NodeCodecQuadTreeHalfFloat;

/// The conversion of the tree to a buffer (AABBTreeToBuffer<TriangleCodec, NodeCodec>)
const TreeToBuffer = AABBTreeToBuffer(TriangleCodec, NodeCodec);

// Get header for tree
fn getNodeHeader(tree: *const ByteBuffer) *const NodeCodec.Header {
    return tree.get(NodeCodec.Header, 0);
}

// Get header for triangles
fn getTriangleHeader(tree: *const ByteBuffer) *const TriangleCodec.TriangleHeader {
    return tree.get(TriangleCodec.TriangleHeader, NodeCodec.header_size);
}

/// Class that constructs a MeshShape
pub const MeshShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, MeshShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    pub const BuildQuality = enum {
        /// Favor runtime performance, takes more time to build the MeshShape but performs better
        favor_runtime_performance,
        /// Favor build speed, build the tree faster but the MeshShape will be slower
        favor_build_speed,
    };

    /// Optional arguments of the constructors (the `PhysicsMaterialList inMaterials = PhysicsMaterialList()` argument)
    pub const Options = struct {
        /// Materials assigned to the triangles (a reference is added to each of them)
        materials: []const *const PhysicsMaterial = &.{},
    };

    base: ShapeSettings,

    /// Vertices belonging to indexed_triangles (allocated with base.allocator)
    triangle_vertices: VertexList = .empty,

    /// Original list of indexed triangles (triangles will be reordered internally in the mesh shape).
    /// Triangles must be provided in counter clockwise order.
    /// Degenerate triangles will automatically be removed during mesh creation but no other mesh simplifications are performed, use an external library if this is desired.
    /// For simulation, the triangles are considered to be single sided.
    /// For ray casts you can choose to make triangles double sided by setting RayCastSettings::mBackFaceMode to EBackFaceMode::CollideWithBackFaces.
    /// For collide shape tests you can use CollideShapeSettings::mBackFaceMode and for shape casts you can use ShapeCastSettings::mBackFaceModeTriangles.
    /// (allocated with base.allocator)
    indexed_triangles: IndexedTriangleList = .empty,

    /// Materials assigned to the triangles. Each triangle specifies which material it uses through its mMaterialIndex
    /// (allocated with base.allocator, holds a reference to each material)
    materials: PhysicsMaterialList = .empty,

    /// Maximum number of triangles in each leaf of the axis aligned box tree. This is a balance between memory and performance. Can be in the range [1, MeshShape::MaxTrianglesPerLeaf].
    /// Sensible values are between 4 (for better performance) and 8 (for less memory usage).
    max_triangles_per_leaf: u32 = 8,

    /// Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
    /// Setting this value too small can cause ghost collisions with edges, setting it too big can cause depenetration artifacts (objects not depenetrating quickly).
    /// Valid ranges are between cos(0 degrees) and cos(90 degrees). The default value is cos(5 degrees).
    /// Negative values will make all edges active and causes EActiveEdgeMode::CollideOnlyWithActive to behave as EActiveEdgeMode::CollideWithAll.
    /// This speeds up the build process but will require all bodies that can interact with the mesh to use BodyCreationSettings::mEnhancedInternalEdgeRemoval = true.
    active_edge_cos_threshold_angle: f32 = 0.996195, // cos(5 degrees)

    /// When true, we store the user data coming from Triangle::mUserData or IndexedTriangle::mUserData in the mesh shape.
    /// This can be used to store additional data like the original index of the triangle in the mesh.
    /// Can be retrieved using MeshShape::GetTriangleUserData.
    /// Turning this on increases the memory used by the MeshShape by roughly 25%.
    per_triangle_user_data: bool = false,

    /// Determines the quality of the tree building process.
    build_quality: BuildQuality = .favor_runtime_performance,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) MeshShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(MeshShapeSettings), allocator) };
    }

    /// Create a mesh shape (MeshShapeSettings(const TriangleList &inTriangles, PhysicsMaterialList inMaterials)): the
    /// triangles are indexified and sanitized. Settings on the stack: `defer settings.deinit()`.
    pub fn init(allocator: Allocator, triangles: []const Triangle, opts: Options) Allocator.Error!MeshShapeSettings {
        var self = initDefault(allocator);
        errdefer self.deinit();
        try self.appendMaterials(opts.materials);

        try indexify(allocator, triangles, &self.triangle_vertices, &self.indexed_triangles, .{});

        try self.sanitize();
        return self;
    }

    /// Create a mesh shape (MeshShapeSettings(VertexList inVertices, IndexedTriangleList inTriangles,
    /// PhysicsMaterialList inMaterials)): the vertices and triangles are copied and sanitized.
    pub fn initIndexed(allocator: Allocator, vertices: []const Float3, triangles: []const IndexedTriangle, opts: Options) Allocator.Error!MeshShapeSettings {
        var self = initDefault(allocator);
        errdefer self.deinit();
        try self.triangle_vertices.appendSlice(allocator, vertices);
        try self.indexed_triangles.appendSlice(allocator, triangles);
        try self.appendMaterials(opts.materials);

        try self.sanitize();
        return self;
    }

    /// new MeshShapeSettings(inTriangles, inMaterials): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, triangles: []const Triangle, opts: Options) Allocator.Error!*MeshShapeSettings {
        const self = try allocator.create(MeshShapeSettings);
        errdefer allocator.destroy(self);
        self.* = try init(allocator, triangles, opts);
        return self;
    }

    /// new MeshShapeSettings(inVertices, inTriangles, inMaterials): reference count 0, freed by the last release()
    pub fn createIndexed(allocator: Allocator, vertices: []const Float3, triangles: []const IndexedTriangle, opts: Options) Allocator.Error!*MeshShapeSettings {
        const self = try allocator.create(MeshShapeSettings);
        errdefer allocator.destroy(self);
        self.* = try initIndexed(allocator, vertices, triangles, opts);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *MeshShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *MeshShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    /// ~MeshShapeSettings: frees the arrays and releases the materials
    pub fn destruct(self: *MeshShapeSettings) void {
        const allocator = self.base.allocator;
        self.triangle_vertices.deinit(allocator);
        self.indexed_triangles.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
    }

    // Append references to `materials` to the material list (std::move(inMaterials))
    fn appendMaterials(self: *MeshShapeSettings, materials: []const *const PhysicsMaterial) Allocator.Error!void {
        try self.materials.ensureUnusedCapacity(self.base.allocator, materials.len);
        for (materials) |m|
            self.materials.appendAssumeCapacity(.init(m));
    }

    /// Sanitize the mesh data. Remove duplicate and degenerate triangles. This is called automatically when constructing the MeshShapeSettings with a list of (indexed-) triangles.
    /// The vertex indices must be valid (Jolt reads the vertices of every triangle).
    pub fn sanitize(self: *MeshShapeSettings) Allocator.Error!void {
        const allocator = self.base.allocator;

        // Remove degenerate and duplicate triangles
        var triangles: UnorderedSet(IndexedTriangle, .{}) = .empty;
        defer triangles.deinit(allocator);
        try triangles.ensureTotalCapacity(allocator, @intCast(self.indexed_triangles.items.len));
        const validation_ctx = TriangleCodec.ValidationContext.init(self.indexed_triangles.items, self.triangle_vertices.items);
        var t: isize = @as(isize, @intCast(self.indexed_triangles.items.len)) - 1;
        while (t >= 0) : (t -= 1) {
            const tri = self.indexed_triangles.items[@intCast(t)];

            if (tri.isDegenerate(self.triangle_vertices.items) // Degenerate triangle
            or validation_ctx.isDegenerate(tri) // Triangle is degenerate in the quantized space
            or !(try triangles.insert(allocator, tri.getLowestIndexFirst())).inserted) // Duplicate triangle
            {
                // The order of triangles doesn't matter (gets reordered while building the tree), so we can just swap the last triangle into this slot
                self.indexed_triangles.items[@intCast(t)] = self.indexed_triangles.getLast();
                _ = self.indexed_triangles.pop();
            }
        }
    }

    // See: ShapeSettings
    pub fn createShape(self: *MeshShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(MeshShape, self, allocator);
    }
};

/// A mesh shape, consisting of triangles. Mesh shapes are mostly used for static geometry.
/// They can be used by dynamic or kinematic objects but only if they don't collide with other mesh or heightfield shapes as those collisions are currently not supported.
/// Note that if you make a mesh shape a dynamic or kinematic object, you need to provide a mass yourself as mesh shapes don't need to form a closed hull so don't have a well defined volume from which the mass can be calculated.
pub const MeshShape = struct {
    /// Concrete class: `Shape.cast(MeshShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .mesh;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .mustBeStatic, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getSubmergedVolume, .saveBinaryState, .saveMaterialState, .restoreMaterialState, .getStats, .getVolume, .restoreBinaryState };

    /// How many bits to reserve to encode the triangle index
    pub const num_triangle_bits = 3;
    /// Number of triangles that are stored max per leaf aabb node
    pub const max_triangles_per_leaf = 1 << num_triangle_bits;

    // 8 bit flags stored per triangle (ETriangleFlags)

    /// Material index
    pub const flags_material_bits = 5;
    pub const flags_material_mask = (1 << flags_material_bits) - 1;

    /// Active edge bits
    pub const flags_active_edge_shift = flags_material_bits;
    pub const flags_active_edge_bits = 3;
    pub const flags_active_edge_mask = (1 << flags_active_edge_bits) - 1;

    base: Shape,

    /// Materials assigned to the triangles. Each triangle specifies which material it uses through its mMaterialIndex
    /// (allocated with the shape's allocator, holds a reference to each material)
    materials: PhysicsMaterialList = .empty,

    /// Resulting packed data structure (allocated with the shape's allocator)
    tree: ByteBuffer = .empty,

    // TODO(debug_renderer): mGeometry, mCachedTrianglesColoredPerGroup, mCachedUseMaterialColors (JPH_DEBUG_RENDERER)

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// MeshShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) MeshShape {
        return .{ .base = .init(Shape.vtableFor(MeshShape), allocator, .mesh, .mesh) };
    }

    /// MeshShape(const MeshShapeSettings &inSettings, ShapeResult &outResult): base part first, then the C++ body.
    /// The temporary data of the build (active edges, splitter, tree builder) is allocated with `allocator`, the
    /// materials and the tree (the buffer of the conversion) with the shape's allocator.
    pub fn initFromSettings(self: *MeshShape, settings: *const MeshShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base);
        const vertices = settings.triangle_vertices.items;
        const triangles = settings.indexed_triangles.items;

        // Check if there are any triangles
        if (triangles.len == 0) {
            result.setError("Need triangles to create a mesh shape!");
            return;
        }

        // Zolt: check the vertex indices before any vertex is read, see the file header (in Jolt's loop order, so the
        // first index reported is the one Jolt's check below would report)
        {
            var t: isize = @as(isize, @intCast(triangles.len)) - 1;
            while (t >= 0) : (t -= 1) {
                for (triangles[@intCast(t)].idx) |idx|
                    if (idx >= vertices.len) {
                        result.setErrorFmt("Vertex index {d} is beyond vertex list (size: {d})", .{ idx, @as(u32, @intCast(vertices.len)) });
                        return;
                    };
            }
        }

        // Check triangles
        const validation_ctx = TriangleCodec.ValidationContext.init(triangles, vertices);
        {
            var t: isize = @as(isize, @intCast(triangles.len)) - 1;
            while (t >= 0) : (t -= 1) {
                const triangle = triangles[@intCast(t)];
                if (triangle.isDegenerate(vertices) or validation_ctx.isDegenerate(triangle)) {
                    result.setErrorFmt("Triangle {d} is degenerate!", .{t});
                    return;
                } else {
                    // Check vertex indices
                    for (triangle.idx) |idx|
                        if (idx >= vertices.len) {
                            result.setErrorFmt("Vertex index {d} is beyond vertex list (size: {d})", .{ idx, @as(u32, @intCast(vertices.len)) });
                            return;
                        };
                }
            }
        }

        // Copy materials
        try copyMaterials(&self.materials, self.base.allocator, settings.materials.items);
        if (self.materials.items.len != 0) {
            // Validate materials
            if (self.materials.items.len > (1 << flags_material_bits)) {
                result.setErrorFmt("Supporting max {d} materials per mesh", .{@as(i32, 1 << flags_material_bits)});
                return;
            }
            for (triangles) |t|
                if (t.material_index >= self.materials.items.len) {
                    result.setErrorFmt("Triangle material {d} is beyond material list (size: {d})", .{ t.material_index, @as(u32, @intCast(self.materials.items.len)) });
                    return;
                };
        } else {
            // No materials assigned, validate that all triangles use material index 0
            for (triangles) |t|
                if (t.material_index != 0) {
                    result.setError("No materials present, all triangles should have material index 0");
                    return;
                };
        }

        // Check max triangles
        if (settings.max_triangles_per_leaf < 1 or settings.max_triangles_per_leaf > max_triangles_per_leaf) {
            result.setError("Invalid max triangles per leaf");
            return;
        }

        // Fill in active edge bits
        var indexed_triangles: IndexedTriangleList = .empty; // Copy indices since we're adding the 'active edge' flag
        defer indexed_triangles.deinit(allocator);
        try indexed_triangles.appendSlice(allocator, triangles);
        try findActiveEdges(allocator, settings, indexed_triangles.items);

        // Create triangle splitter (Jolt constructs one of the two in a union)
        var binning: TriangleSplitterBinning = undefined;
        var mean: TriangleSplitterMean = undefined;
        const splitter: *TriangleSplitter = switch (settings.build_quality) {
            .favor_runtime_performance => blk: {
                binning = try TriangleSplitterBinning.init(allocator, vertices, indexed_triangles.items, .{});
                break :blk binning.splitter();
            },

            .favor_build_speed => blk: {
                mean = try TriangleSplitterMean.init(allocator, vertices, indexed_triangles.items);
                break :blk mean.splitter();
            },
        };

        // Build tree
        var builder: AABBTreeBuilder = .init(splitter, .{ .max_triangles_per_leaf = settings.max_triangles_per_leaf });
        defer builder.deinit(allocator);
        var builder_stats: AABBTreeBuilderStats = .{};
        const root = builder.build(allocator, &builder_stats) catch |err| {
            splitter.deinit(allocator);
            return err;
        };
        splitter.deinit(allocator);

        // Convert to buffer (allocated with the shape's allocator: the shape takes the buffer)
        var buffer: TreeToBuffer = .empty;
        defer buffer.deinit(self.base.allocator);
        buffer.convert(self.base.allocator, builder.getTriangles(), builder.getNodes(), vertices, root, settings.per_triangle_user_data) catch |err| switch (err) {
            error.OutOfMemory => return error.OutOfMemory,
            else => {
                result.setError(TreeToBuffer.errorMessage(err));
                return;
            },
        };

        // Move data to this class
        std.mem.swap(ByteBuffer, &self.tree, buffer.getBufferMut());

        // Check if we're not exceeding the amount of sub shape id bits
        if (self.getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("Mesh is too big and exceeds the amount of available sub shape ID bits");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// ~MeshShape: releases the materials and frees the tree
    pub fn destruct(self: *MeshShape) void {
        const allocator = self.base.allocator;
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
        self.tree.deinit(allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const MeshShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *MeshShape) *Shape {
        return &self.base;
    }

    // `list = materials` (Array copy assignment): releases the old references, reserves exactly, adds references
    fn copyMaterials(list: *PhysicsMaterialList, allocator: Allocator, materials: []const PhysicsMaterialRefC) Allocator.Error!void {
        for (list.items) |*m| m.deinit();
        list.clearRetainingCapacity();
        try list.ensureTotalCapacityPrecise(allocator, materials.len);
        for (materials) |m|
            list.appendAssumeCapacity(m.clone());
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Active edges

    /// A struct to hold the two vertex indices of an edge
    const Edge = extern struct {
        idx1: i32,
        idx2: i32,

        comptime {
            std.debug.assert(@sizeOf(Edge) == 2 * @sizeOf(i32)); // No padding expected
        }

        fn init(idx1: u32, idx2: u32) Edge {
            // Jolt's constructor takes ints
            const index1: i32 = @bitCast(idx1);
            const index2: i32 = @bitCast(idx2);
            return .{ .idx1 = @min(index1, index2), .idx2 = @max(index1, index2) };
        }

        fn getIndexInTriangle(self: Edge, triangle: IndexedTriangle) u32 {
            for (0..3) |edge_idx| {
                const edge = Edge.init(triangle.idx[edge_idx], triangle.idx[(edge_idx + 1) % 3]);
                if (self.eql(edge))
                    return @intCast(edge_idx);
            }

            std.debug.assert(false); // The edge was taken from this triangle
            return ~@as(u32, 0);
        }

        pub fn eql(self: Edge, rhs: Edge) bool {
            return self.idx1 == rhs.idx1 and self.idx2 == rhs.idx2;
        }

        pub fn getHash(self: Edge) u64 {
            return HashCombine.hashBytes(std.mem.asBytes(&self));
        }
    };

    /// A struct to hold the triangles that are connected to an edge
    const TriangleIndices = struct {
        num_triangles: u32 = 0,
        triangle_indices: [2]u32 = .{ 0, 0 },
    };

    /// Find and flag active edges
    fn findActiveEdges(allocator: Allocator, settings: *const MeshShapeSettings, indices: []IndexedTriangle) Allocator.Error!void {
        // Check if we're requested to make all edges active
        if (settings.active_edge_cos_threshold_angle < 0.0) {
            for (indices) |*triangle|
                triangle.material_index |= 0b111 << flags_active_edge_shift;
            return;
        }

        // Build a list of edge to triangles
        const EdgeToTriangle = UnorderedMap(Edge, TriangleIndices, .{});
        var edge_to_triangle: EdgeToTriangle = .empty;
        defer edge_to_triangle.deinit(allocator);
        try edge_to_triangle.ensureTotalCapacity(allocator, @truncate(indices.len * 3));
        for (indices, 0..) |*triangle, triangle_idx| {
            for (0..3) |edge_idx| {
                const edge = Edge.init(triangle.idx[edge_idx], triangle.idx[(edge_idx + 1) % 3]);
                const edge_to_triangle_it = (try edge_to_triangle.tryEmplace(allocator, edge, .{})).ptr;
                const tri_indices = &edge_to_triangle_it.value;
                if (tri_indices.num_triangles < 2) {
                    // Store index of triangle that connects to this edge
                    tri_indices.triangle_indices[tri_indices.num_triangles] = @intCast(triangle_idx);
                    tri_indices.num_triangles += 1;
                } else {
                    // 3 or more triangles share an edge, mark this edge as active
                    const mask: u32 = @as(u32, 1) << @intCast(edge_idx + flags_active_edge_shift);
                    std.debug.assert((triangle.material_index & mask) == 0);
                    triangle.material_index |= mask;
                    tri_indices.num_triangles = 3; // Indicate that we have 3 or more triangles
                }
            }
        }

        // Walk over all edges and determine which ones are active
        var it = edge_to_triangle.constIterator();
        while (it.next()) |edge| {
            var num_active: u32 = 0;
            if (edge.value.num_triangles == 1) {
                // Edge is not shared, it is an active edge
                num_active = 1;
            } else if (edge.value.num_triangles == 2) {
                // Simple shared edge, determine if edge is active based on the two adjacent triangles
                const triangle1 = indices[edge.value.triangle_indices[0]];
                const triangle2 = indices[edge.value.triangle_indices[1]];

                // Find which edge this is for both triangles
                const edge_idx1 = edge.key.getIndexInTriangle(triangle1);
                const edge_idx2 = edge.key.getIndexInTriangle(triangle2);

                // Construct a plane for triangle 1 (e1 = edge vertex 1, e2 = edge vertex 2, op = opposing vertex)
                const triangle1_e1 = Vec3.fromFloat3(settings.triangle_vertices.items[triangle1.idx[edge_idx1]]);
                const triangle1_e2 = Vec3.fromFloat3(settings.triangle_vertices.items[triangle1.idx[(edge_idx1 + 1) % 3]]);
                const triangle1_op = Vec3.fromFloat3(settings.triangle_vertices.items[triangle1.idx[(edge_idx1 + 2) % 3]]);
                const triangle1_plane = Plane.fromPointsCCW(triangle1_e1, triangle1_e2, triangle1_op);

                // Construct a plane for triangle 2
                const triangle2_e1 = Vec3.fromFloat3(settings.triangle_vertices.items[triangle2.idx[edge_idx2]]);
                const triangle2_e2 = Vec3.fromFloat3(settings.triangle_vertices.items[triangle2.idx[(edge_idx2 + 1) % 3]]);
                const triangle2_op = Vec3.fromFloat3(settings.triangle_vertices.items[triangle2.idx[(edge_idx2 + 2) % 3]]);
                const triangle2_plane = Plane.fromPointsCCW(triangle2_e1, triangle2_e2, triangle2_op);

                // Determine if the edge is active
                num_active = if (ActiveEdges.isEdgeActive(triangle1_plane.getNormal(), triangle2_plane.getNormal(), triangle1_e2.sub(triangle1_e1), settings.active_edge_cos_threshold_angle)) 2 else 0;
            } else {
                // More edges incoming, we've already marked all edges beyond the 2nd as active
                num_active = 2;
            }

            // Mark edges of all original triangles active
            for (0..num_active) |i| {
                const triangle_idx = edge.value.triangle_indices[i];
                const triangle = &indices[triangle_idx];
                const edge_idx = edge.key.getIndexInTriangle(triangle.*);
                const mask: u32 = @as(u32, 1) << @intCast(edge_idx + flags_active_edge_shift);
                std.debug.assert((triangle.material_index & mask) == 0);
                triangle.material_index |= mask;
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get the list of all materials
    pub fn getMaterialList(self: *const MeshShape) []const PhysicsMaterialRefC {
        return self.materials.items;
    }

    /// The triangle block and the index of the triangle in that block that a sub shape ID refers to (DecodeSubShapeID's out parameters)
    const DecodedSubShapeID = struct {
        triangle_block: *const anyopaque,
        triangle_index: u32,
    };

    /// Decode a sub shape ID
    fn decodeSubShapeID(self: *const MeshShape, sub_shape_id: SubShapeID) DecodedSubShapeID {
        // Get block
        const block = sub_shape_id.popID(NodeCodec.DecodingContext.triangleBlockIDBits(getNodeHeader(&self.tree)));
        const triangle_block = NodeCodec.DecodingContext.getTriangleBlockStart(self.tree.vector.items.ptr, block.id);

        // Fetch the triangle index
        const triangle = block.remainder.popID(num_triangle_bits);
        std.debug.assert(triangle.remainder.isEmpty()); // Invalid subshape ID
        return .{ .triangle_block = triangle_block, .triangle_index = triangle.id };
    }

    /// Determine which material index a particular sub shape uses (note that if there are no materials this function will return 0 so check the array size)
    /// Note: This could for example be used to create a decorator shape around a mesh shape that overrides the GetMaterial call to replace a material with another material.
    pub fn getMaterialIndex(self: *const MeshShape, sub_shape_id: SubShapeID) u32 {
        // Decode ID
        const decoded = self.decodeSubShapeID(sub_shape_id);

        // Fetch the flags
        const flags = TriangleCodec.DecodingContext.getTriangleFlags(decoded.triangle_block, @intCast(decoded.triangle_index));
        return flags & flags_material_mask;
    }

    /// When MeshShape::mPerTriangleUserData is true, this function can be used to retrieve the user data that was stored in the mesh shape.
    pub fn getTriangleUserData(self: *const MeshShape, sub_shape_id: SubShapeID) u32 {
        // Decode ID
        const decoded = self.decodeSubShapeID(sub_shape_id);

        // Decode triangle
        const triangle_ctx = TriangleCodec.DecodingContext.init(getTriangleHeader(&self.tree));
        return triangle_ctx.getUserData(decoded.triangle_block, decoded.triangle_index);
    }

    /// Visit the entire tree using a visitor pattern (`visitor` is a pointer to a struct with `shouldAbort`,
    /// `shouldVisitNode`, `visitNodes` and `visitTriangles`)
    fn walkTree(self: *const MeshShape, visitor: anytype) void {
        const header = getNodeHeader(&self.tree);
        var node_ctx = NodeCodec.DecodingContext.init(header);

        const triangle_ctx = TriangleCodec.DecodingContext.init(getTriangleHeader(&self.tree));
        const buffer_start = self.tree.vector.items.ptr;
        node_ctx.walkTree(buffer_start, &triangle_ctx, visitor);
    }

    /// The visitor of WalkTreePerTriangle that calls `visitTriangle` of `Visitor` for every triangle
    fn ChainedVisitor(comptime Visitor: type) type {
        return struct {
            const Self = @This();

            visitor: *Visitor,
            sub_shape_id_creator2: SubShapeIDCreator,
            triangle_block_id_bits: u32,

            pub fn shouldAbort(self: *const Self) bool {
                return self.visitor.shouldAbort();
            }

            pub fn shouldVisitNode(self: *const Self, stack_top: i32) bool {
                return self.visitor.shouldVisitNode(stack_top);
            }

            pub fn visitNodes(self: *Self, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                return self.visitor.visitNodes(bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z, properties, stack_top);
            }

            pub fn visitTriangles(self: *Self, context: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
                // Create ID for triangle block
                const block_sub_shape_id = self.sub_shape_id_creator2.pushID(triangle_block_id, self.triangle_block_id_bits);

                // Decode vertices and flags
                std.debug.assert(num_triangles <= max_triangles_per_leaf);
                var vertices: [max_triangles_per_leaf * 3]Vec3 = undefined;
                var flags: [max_triangles_per_leaf]u8 = undefined;
                context.unpackWithFlags(triangles, num_triangles, &vertices, &flags);

                var triangle_idx: u32 = 0;
                var v: u32 = 0;
                const v_end = num_triangles * 3;
                while (v < v_end) : ({
                    v += 3;
                    triangle_idx += 1;
                }) {
                    // Determine active edges
                    const active_edges: u8 = (flags[triangle_idx] >> flags_active_edge_shift) & flags_active_edge_mask;

                    // Create ID for triangle
                    const triangle_sub_shape_id = block_sub_shape_id.pushID(triangle_idx, num_triangle_bits);

                    self.visitor.visitTriangle(vertices[v], vertices[v + 1], vertices[v + 2], active_edges, triangle_sub_shape_id.getID());

                    // Check if we should early out now
                    if (self.visitor.shouldAbort())
                        break;
                }
            }
        };
    }

    /// Same as above but with a callback per triangle instead of per block of triangles (`visitor` declares
    /// `visitTriangle(v0, v1, v2, active_edges: u8, sub_shape_id2: SubShapeID)` instead of `visitTriangles`)
    fn walkTreePerTriangle(self: *const MeshShape, sub_shape_id_creator2: SubShapeIDCreator, visitor: anytype) void {
        const Visitor = @typeInfo(@TypeOf(visitor)).pointer.child;
        var chained_visitor: ChainedVisitor(Visitor) = .{
            .visitor = visitor,
            .sub_shape_id_creator2 = sub_shape_id_creator2,
            .triangle_block_id_bits = NodeCodec.DecodingContext.triangleBlockIDBits(getNodeHeader(&self.tree)),
        };
        self.walkTree(&chained_visitor);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::MustBeStatic
    pub fn mustBeStatic(self: *const MeshShape) bool {
        _ = self;
        return true;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const MeshShape) AABox {
        const header = getNodeHeader(&self.tree);
        return .init(Vec3.loadFloat3Unsafe(&header.root_bounds_min), Vec3.loadFloat3Unsafe(&header.root_bounds_max));
    }

    // See Shape::GetSubShapeIDBitsRecursive
    pub fn getSubShapeIDBitsRecursive(self: *const MeshShape) u32 {
        return NodeCodec.DecodingContext.triangleBlockIDBits(getNodeHeader(&self.tree)) + num_triangle_bits;
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const MeshShape) f32 {
        _ = self;
        return 0.0;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const MeshShape) MassProperties {
        _ = self;

        // We cannot calculate the volume for an arbitrary mesh, so we return invalid mass properties.
        // If you want your mesh to be dynamic, then you should provide the mass properties yourself when
        // creating a Body:
        //
        // BodyCreationSettings::mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
        // BodyCreationSettings::mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(Vec3::sOne(), 1000.0f);
        //
        // Note that for a mesh shape to simulate properly, it is best if the mesh is manifold
        // (i.e. closed, all edges shared by only two triangles, consistent winding order).
        return .{};
    }

    // See Shape::GetMaterial
    pub fn getMaterial(self: *const MeshShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        // Return the default material if there are no materials on this shape
        if (self.materials.items.len == 0)
            return PhysicsMaterial.default;

        // Zolt: a null entry of the material list returns the default material (Jolt returns nullptr), see the file header
        return self.materials.items[self.getMaterialIndex(sub_shape_id)].get() orelse PhysicsMaterial.default;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const MeshShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = local_surface_position;

        // Decode ID
        const decoded = self.decodeSubShapeID(sub_shape_id);

        // Decode triangle
        const triangle_ctx = TriangleCodec.DecodingContext.init(getTriangleHeader(&self.tree));
        const t = triangle_ctx.getTriangle(decoded.triangle_block, decoded.triangle_index);

        // Calculate normal
        return t.v3.sub(t.v2).cross(t.v1.sub(t.v2)).normalized();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const MeshShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = direction;

        // Decode ID
        const decoded = self.decodeSubShapeID(sub_shape_id);

        // Decode triangle
        const triangle_ctx = TriangleCodec.DecodingContext.init(getTriangleHeader(&self.tree));
        out_vertices.resize(3);
        const t = triangle_ctx.getTriangle(decoded.triangle_block, decoded.triangle_index);
        out_vertices.buffer[0] = t.v1;
        out_vertices.buffer[1] = t.v2;
        out_vertices.buffer[2] = t.v3;

        // Flip triangle if scaled inside out
        if (ScaleHelpers.isInsideOut(scale))
            std.mem.swap(Vec3, &out_vertices.buffer[1], &out_vertices.buffer[2]);

        // Calculate transform with scale
        const transform = center_of_mass_transform.preScaled(scale);

        // Transform to world space
        for (out_vertices.slice()) |*v|
            v.* = transform.mulVec3(v.*);
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay
    pub fn castRay(self: *const MeshShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const Visitor = struct {
            hit: *RayCastResult,
            ray_origin: Vec3,
            ray_direction: Vec3,
            ray_inv_direction: RayInvDirection,
            triangle_block_id_bits: u32,
            sub_shape_id_creator: SubShapeIDCreator,
            return_value: bool = false,
            distance_stack: [NodeCodec.stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.hit.fraction <= 0.0;
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.hit.fraction;
            }

            pub fn visitNodes(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Test bounds of 4 children
                const distance = rayAABox4(v.ray_origin, v.ray_inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.hit.fraction, properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangles(v: *@This(), context: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
                // Test against triangles
                const r = context.testRay(v.ray_origin, v.ray_direction, triangles, num_triangles, v.hit.fraction);
                if (r.closest < v.hit.fraction) {
                    v.hit.fraction = r.closest;
                    v.hit.sub_shape_id2 = v.sub_shape_id_creator.pushID(triangle_block_id, v.triangle_block_id_bits).pushID(r.closest_triangle_index, num_triangle_bits).getID();
                    v.return_value = true;
                }
            }
        };

        var visitor: Visitor = .{
            .hit = hit,
            .ray_origin = ray.origin,
            .ray_direction = ray.direction,
            .ray_inv_direction = .init(ray.direction),
            .triangle_block_id_bits = NodeCodec.DecodingContext.triangleBlockIDBits(getNodeHeader(&self.tree)),
            .sub_shape_id_creator = sub_shape_id_creator,
        };
        self.walkTree(&visitor);

        return visitor.return_value;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const MeshShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = struct {
            collector: *CastRayCollector,
            ray_origin: Vec3,
            ray_direction: Vec3,
            ray_inv_direction: RayInvDirection,
            back_face_mode: BackFaceMode,
            distance_stack: [NodeCodec.stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.collector.shouldEarlyOut();
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.collector.getEarlyOutFraction();
            }

            pub fn visitNodes(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Test bounds of 4 children
                const distance = rayAABox4(v.ray_origin, v.ray_inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.collector.getEarlyOutFraction(), properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
                _ = active_edges;

                // Back facing check
                if (v.back_face_mode == .ignore_back_faces and v2.sub(v0).cross(v1.sub(v0)).dot(v.ray_direction) < 0)
                    return;

                // Check the triangle
                const fraction = rayTriangle(v.ray_origin, v.ray_direction, v0, v1, v2);
                if (fraction < v.collector.getEarlyOutFraction()) {
                    var hit: RayCastResult = .{};
                    hit.body_id = TransformedShape.getBodyID(v.collector.getContext());
                    hit.fraction = fraction;
                    hit.sub_shape_id2 = sub_shape_id2;
                    v.collector.addHit(&hit);
                }
            }
        };

        var visitor: Visitor = .{
            .collector = collector,
            .back_face_mode = ray_cast_settings.back_face_mode_triangles,
            .ray_origin = ray.origin,
            .ray_direction = ray.direction,
            .ray_inv_direction = .init(ray.direction),
        };
        self.walkTreePerTriangle(sub_shape_id_creator, &visitor);
    }

    /// See: Shape::CollidePoint
    /// Note that for CollidePoint to work for a mesh shape, the mesh needs to be closed (a manifold) or multiple non-intersecting manifolds. Triangles may be facing the interior of the manifold.
    /// Insideness is tested by counting the amount of triangles encountered when casting an infinite ray from inPoint. If the number of hits is odd we're inside, if it's even we're outside.
    pub fn collidePoint(self: *const MeshShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        Shape.collidePointUsingRayCast(self.asShape(), point, sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const MeshShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        const Visitor = struct {
            base: CollideSoftBodyVerticesVsTriangles,
            distance_stack: [NodeCodec.stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                _ = v;
                return false;
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.base.closest_distance_sq;
            }

            pub fn visitNodes(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Scale the bounding boxes of this node
                const bounds = AABox4.aabox4Scale(v.base.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Get distance to vertex
                const dist_sq = AABox4.aabox4DistanceSqToPointVec3(v.base.local_position, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(dist_sq, v.base.closest_distance_sq, properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
                _ = .{ active_edges, sub_shape_id2 };
                v.base.processTriangle(v0, v1, v2);
            }
        };

        var visitor: Visitor = .{ .base = .init(center_of_mass_transform, scale) };

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                visitor.base.startVertex(&v);
                self.walkTreePerTriangle(.{}, &visitor);
                visitor.base.finishVertex(&v, colliding_shape_index);
            }
        }
    }

    /// Context class for GetTrianglesStart/Next
    const MSGetTrianglesContext = struct {
        decode_ctx: NodeCodec.DecodingContext,
        shape: *const MeshShape,
        local_box: OrientedBox,
        mesh_scale: Vec3,
        local_to_world: Mat44,
        max_triangles_requested: u32 = 0,
        /// Jolt's advancing `Float3 *mTriangleVertices`: the output and the next position to write
        triangle_vertices: []Float3 = &.{},
        triangle_vertices_pos: usize = 0,
        num_triangles_found: u32 = 0,
        /// Jolt's advancing `const PhysicsMaterial **mMaterials`: the output and the next position to write
        materials: ?[]*const PhysicsMaterial = null,
        materials_pos: usize = 0,
        should_abort: bool = false,
        is_inside_out: bool,

        fn init(shape: *const MeshShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) MSGetTrianglesContext {
            return .{
                .decode_ctx = .init(getNodeHeader(&shape.tree)),
                .shape = shape,
                .local_box = .fromAABox(Mat44.inverseRotationTranslation(rotation, position_com), box),
                .mesh_scale = scale,
                .local_to_world = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale)),
                .is_inside_out = ScaleHelpers.isInsideOut(scale),
            };
        }

        pub fn shouldAbort(self: *const MSGetTrianglesContext) bool {
            return self.should_abort;
        }

        pub fn shouldVisitNode(self: *const MSGetTrianglesContext, stack_top: i32) bool {
            _ = .{ self, stack_top };
            return true;
        }

        pub fn visitNodes(self: *const MSGetTrianglesContext, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
            _ = stack_top;

            // Scale the bounding boxes of this node
            const bounds = AABox4.aabox4Scale(self.mesh_scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

            // Test which nodes collide
            const collides = AABox4.aabox4VsOrientedBox(self.local_box, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z, .{});
            return countAndSortTrues(collides, properties);
        }

        pub fn visitTriangles(self: *MSGetTrianglesContext, context: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
            _ = triangle_block_id;

            // When the buffer is full and we cannot process the triangles, abort the tree walk. The next time GetTrianglesNext is called we will continue here.
            if (self.num_triangles_found + num_triangles > self.max_triangles_requested) {
                self.should_abort = true;
                return;
            }

            // Decode vertices
            std.debug.assert(num_triangles <= max_triangles_per_leaf);
            var vertices: [max_triangles_per_leaf * 3]Vec3 = undefined;
            context.unpack(triangles, num_triangles, &vertices);

            // Store vertices as Float3
            const vertices_end = 3 * num_triangles;
            if (self.is_inside_out) {
                // Scaled inside out, flip the triangles
                var v: u32 = 0;
                while (v < vertices_end) : (v += 3) {
                    self.storeVertex(self.local_to_world.mulVec3(vertices[v]));
                    self.storeVertex(self.local_to_world.mulVec3(vertices[v + 2]));
                    self.storeVertex(self.local_to_world.mulVec3(vertices[v + 1]));
                }
            } else {
                // Normal scale
                for (vertices[0..vertices_end]) |v|
                    self.storeVertex(self.local_to_world.mulVec3(v));
            }

            if (self.materials) |materials| {
                if (self.shape.materials.items.len == 0) {
                    // No materials, output default
                    const default_material = PhysicsMaterial.default;
                    for (0..num_triangles) |_| {
                        materials[self.materials_pos] = default_material;
                        self.materials_pos += 1;
                    }
                } else {
                    // Decode triangle flags
                    var flags: [max_triangles_per_leaf]u8 = undefined;
                    TriangleCodec.DecodingContext.getFlags(triangles, num_triangles, &flags);

                    // Store materials (Zolt: a null entry of the material list stores the default material, see the file header)
                    for (flags[0..num_triangles]) |f| {
                        materials[self.materials_pos] = self.shape.materials.items[f & flags_material_mask].get() orelse PhysicsMaterial.default;
                        self.materials_pos += 1;
                    }
                }
            }

            // Accumulate triangles found
            self.num_triangles_found += num_triangles;
        }

        // (mLocalToWorld * v).StoreFloat3(mTriangleVertices++)
        fn storeVertex(self: *MSGetTrianglesContext, v: Vec3) void {
            v.storeFloat3(&self.triangle_vertices[self.triangle_vertices_pos]);
            self.triangle_vertices_pos += 1;
        }
    };

    // See Shape::GetTrianglesStart
    pub fn getTrianglesStart(self: *const MeshShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        context.emplace(MSGetTrianglesContext).* = .init(self, box, position_com, rotation, scale);
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const MeshShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        comptime std.debug.assert(Shape.get_triangles_min_triangles_requested >= max_triangles_per_leaf); // cGetTrianglesMinTrianglesRequested is too small
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        // Check if we're done
        const ctx = context.get(MSGetTrianglesContext);
        if (ctx.decode_ctx.isDoneWalking())
            return 0;

        // Store parameters on context
        ctx.max_triangles_requested = max_triangles_requested;
        ctx.triangle_vertices = out_triangle_vertices;
        ctx.triangle_vertices_pos = 0;
        ctx.materials = out_materials;
        ctx.materials_pos = 0;
        ctx.should_abort = false; // Reset the abort flag
        ctx.num_triangles_found = 0;

        // Continue (or start) walking the tree
        const triangle_ctx = TriangleCodec.DecodingContext.init(getTriangleHeader(&self.tree));
        const buffer_start = self.tree.vector.items.ptr;
        ctx.decode_ctx.walkTree(buffer_start, &triangle_ctx, ctx);
        return ctx.num_triangles_found;
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const MeshShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };
        if (Core.enable_asserts) @panic("Not supported");

        // Jolt leaves the out parameters untouched
        return .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    // See Shape::SaveBinaryState
    pub fn saveBinaryState(self: *const MeshShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);

        stream.writeArray(u8, self.tree.vector.items); // Make sure we use the Array<> overload
    }

    // See Shape::SaveMaterialState (outMaterials = mMaterials)
    pub fn saveMaterialState(self: *const MeshShape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        try copyMaterials(out_materials, allocator, self.materials.items);
    }

    // See Shape::RestoreMaterialState (mMaterials.assign(inMaterials, inMaterials + inNumMaterials))
    pub fn restoreMaterialState(self: *MeshShape, materials: []const PhysicsMaterialRefC) Allocator.Error!void {
        try copyMaterials(&self.materials, self.base.allocator, materials);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const MeshShape) Shape.Stats {
        // Walk the tree to count the triangles
        const Visitor = struct {
            num_triangles: u32 = 0,

            pub fn shouldAbort(v: *const @This()) bool {
                _ = v;
                return false;
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                _ = .{ v, stack_top };
                return true;
            }

            pub fn visitNodes(v: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                _ = .{ v, stack_top };

                // Visit all valid children
                const valid = UVec4.bitOr(UVec4.bitOr(Vec4.less(bounds_min_x, bounds_max_x), Vec4.less(bounds_min_y, bounds_max_y)), Vec4.less(bounds_min_z, bounds_max_z));
                return countAndSortTrues(valid, properties);
            }

            pub fn visitTriangles(v: *@This(), context: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
                _ = .{ context, triangles, triangle_block_id };
                v.num_triangles += num_triangles;
            }
        };

        var visitor: Visitor = .{};
        self.walkTree(&visitor);

        return .init(@sizeOf(MeshShape) + self.materials.items.len * @sizeOf(RefConst(PhysicsMaterial)) + self.tree.vector.items.len * @sizeOf(u8), visitor.num_triangles);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const MeshShape) f32 {
        _ = self;
        return 0;
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *MeshShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(&self.base, stream);

        // inStream.Read(static_cast<ByteBufferVector &>(mTree)) (the Array<> overload, the buffer stays cache line aligned)
        var len: u32 = @intCast(self.tree.vector.items.len); // Initialize to previous array size, this is used for validation in the StateRecorder class
        stream.read(&len);
        if (!stream.isEOF() and !stream.isFailed()) {
            try self.tree.vector.ensureTotalCapacityPrecise(self.base.allocator, len);
            self.tree.vector.items.len = len;
            stream.readBytes(self.tree.vector.items);
        } else self.tree.vector.clearRetainingCapacity();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch

    fn collideConvexVsMesh(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        // Get the shapes
        std.debug.assert(shape1.getType() == .convex);
        std.debug.assert(shape2.getType() == .mesh);
        const s1 = shape1.cast(ConvexShape);
        const s2 = shape2.cast(MeshShape);

        const Visitor = struct {
            base: CollideConvexVsTriangles,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                _ = .{ v, stack_top };
                return true;
            }

            pub fn visitNodes(v: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                _ = stack_top;

                // Scale the bounding boxes of this node
                const bounds = AABox4.aabox4Scale(v.base.scale2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Test which nodes collide
                const collides = AABox4.aabox4VsBox(v.base.bounds_of1_in_space_of2, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);
                return countAndSortTrues(collides, properties);
            }

            pub fn visitTriangle(v: *@This(), v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
                v.base.collide(v0, v1, v2, active_edges, sub_shape_id2);
            }
        };

        // In place: the helper keeps pointers into its support buffers after the first triangle
        var visitor: Visitor = .{ .base = .init(s1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), collide_shape_settings, collector) };
        s2.walkTreePerTriangle(sub_shape_id_creator2, &visitor);
    }

    fn collideSphereVsMesh(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        // Get the shapes
        std.debug.assert(shape1.getSubType() == .sphere);
        std.debug.assert(shape2.getType() == .mesh);
        const s1 = shape1.cast(SphereShape);
        const s2 = shape2.cast(MeshShape);

        const Visitor = struct {
            base: CollideSphereVsTriangles,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                _ = .{ v, stack_top };
                return true;
            }

            pub fn visitNodes(v: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                _ = stack_top;

                // Scale the bounding boxes of this node
                const bounds = AABox4.aabox4Scale(v.base.scale2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Test which nodes collide
                const collides = AABox4.aabox4VsSphereVec3(v.base.sphere_center_in2, v.base.radius_plus_max_separation_sq, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);
                return countAndSortTrues(collides, properties);
            }

            pub fn visitTriangle(v: *@This(), v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
                v.base.collide(v0, v1, v2, active_edges, sub_shape_id2);
            }
        };

        var visitor: Visitor = .{ .base = .init(s1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), collide_shape_settings, collector) };
        s2.walkTreePerTriangle(sub_shape_id_creator2, &visitor);
    }

    fn castConvexVsMesh(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        const Visitor = struct {
            base: CastConvexVsTriangles,
            inv_direction: RayInvDirection = undefined,
            box_center: Vec3 = undefined,
            box_extent: Vec3 = undefined,
            distance_stack: [NodeCodec.stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.base.collector.getPositiveEarlyOutFraction();
            }

            pub fn visitNodes(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Scale the bounding boxes of this node
                var bounds = AABox4.aabox4Scale(v.base.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Enlarge them by the casted shape's box extents
                AABox4.aabox4EnlargeWithExtent(v.box_extent, &bounds.min_x, &bounds.min_y, &bounds.min_z, &bounds.max_x, &bounds.max_y, &bounds.max_z);

                // Test bounds of 4 children
                const distance = rayAABox4(v.box_center, v.inv_direction, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.base.collector.getPositiveEarlyOutFraction(), properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
                v.base.cast(v0, v1, v2, active_edges, sub_shape_id2);
            }
        };

        std.debug.assert(shape.getSubType() == .mesh);
        const mesh = shape.cast(MeshShape);

        // In place: the helper keeps a pointer into its support buffer after the first triangle
        var visitor: Visitor = .{ .base = .init(shape_cast, shape_cast_settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector) };
        visitor.inv_direction.set(shape_cast.direction);
        visitor.box_center = shape_cast.shape_world_bounds.getCenter();
        visitor.box_extent = shape_cast.shape_world_bounds.getExtent().add(Vec3.replicate(shape_cast_settings.extra_convex_radius));
        mesh.walkTreePerTriangle(sub_shape_id_creator2, &visitor);
    }

    fn castSphereVsMesh(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        const Visitor = struct {
            base: CastSphereVsTriangles,
            inv_direction: RayInvDirection = undefined,
            distance_stack: [NodeCodec.stack_size]f32 = undefined,

            pub fn shouldAbort(v: *const @This()) bool {
                return v.base.collector.shouldEarlyOut();
            }

            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.distance_stack[@intCast(stack_top)] < v.base.collector.getPositiveEarlyOutFraction();
            }

            pub fn visitNodes(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Scale the bounding boxes of this node
                var bounds = AABox4.aabox4Scale(v.base.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Enlarge them by the radius of the sphere
                AABox4.aabox4EnlargeWithExtent(Vec3.replicate(v.base.radius), &bounds.min_x, &bounds.min_y, &bounds.min_z, &bounds.max_x, &bounds.max_y, &bounds.max_z);

                // Test bounds of 4 children
                const distance = rayAABox4(v.base.start, v.inv_direction, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return sortReverseAndStore(distance, v.base.collector.getPositiveEarlyOutFraction(), properties, v.distance_stack[@intCast(stack_top)..][0..4]);
            }

            pub fn visitTriangle(v: *@This(), v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
                v.base.cast(v0, v1, v2, active_edges, sub_shape_id2);
            }
        };

        std.debug.assert(shape.getSubType() == .mesh);
        const mesh = shape.cast(MeshShape);

        var visitor: Visitor = .{ .base = .init(shape_cast, shape_cast_settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector) };
        visitor.inv_direction.set(shape_cast.direction);
        mesh.walkTreePerTriangle(sub_shape_id_creator2, &visitor);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.mesh);
        f.construct = ShapeFunctions.constructor(MeshShape);
        f.color = Color.red;

        for (ShapeFile.convex_sub_shape_types) |s| {
            r.registerCollideShape(s, .mesh, collideConvexVsMesh);
            r.registerCastShape(s, .mesh, castConvexVsMesh);

            r.registerCastShape(.mesh, s, CollisionDispatch.reversedCastShape);
            r.registerCollideShape(.mesh, s, CollisionDispatch.reversedCollideShape);
        }

        // Specialized collision functions
        r.registerCollideShape(.sphere, .mesh, collideSphereVsMesh);
        r.registerCastShape(.sphere, .mesh, castSphereVsMesh);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (no Jolt test file tests MeshShape without a PhysicsSystem; the cases of the shared shape test files that
// MeshShape enables are ported with them, the bit exact comparison with Jolt is in ZoltParity/Physics/MeshShapeParity.zig)

const testing = std.testing;
const math = @import("../../../Math/Math.zig");
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const BoxShape = @import("BoxShape.zig").BoxShape;
const RegisterTypes = @import("../../../RegisterTypes.zig");

/// The 12 triangles of the box [-1, 1]^3, counter clockwise seen from the outside, 2 per face
fn boxTriangles(num_materials: u32) [12]Triangle {
    const v = [_]Vec3{
        Vec3.init(-1, 1, -1),  Vec3.init(-1, 1, 1),   Vec3.init(1, 1, 1),
        Vec3.init(-1, 1, -1),  Vec3.init(1, 1, 1),    Vec3.init(1, 1, -1),
        Vec3.init(-1, -1, -1), Vec3.init(1, -1, -1),  Vec3.init(1, -1, 1),
        Vec3.init(-1, -1, -1), Vec3.init(1, -1, 1),   Vec3.init(-1, -1, 1),
        Vec3.init(-1, 1, -1),  Vec3.init(-1, -1, -1), Vec3.init(-1, -1, 1),
        Vec3.init(-1, 1, -1),  Vec3.init(-1, -1, 1),  Vec3.init(-1, 1, 1),
        Vec3.init(1, 1, 1),    Vec3.init(1, -1, 1),   Vec3.init(1, -1, -1),
        Vec3.init(1, 1, 1),    Vec3.init(1, -1, -1),  Vec3.init(1, 1, -1),
        Vec3.init(-1, 1, 1),   Vec3.init(-1, -1, 1),  Vec3.init(1, -1, 1),
        Vec3.init(-1, 1, 1),   Vec3.init(1, -1, 1),   Vec3.init(1, 1, 1),
        Vec3.init(-1, 1, -1),  Vec3.init(1, 1, -1),   Vec3.init(1, -1, -1),
        Vec3.init(-1, 1, -1),  Vec3.init(1, -1, -1),  Vec3.init(-1, -1, -1),
    };
    var triangles: [12]Triangle = undefined;
    for (&triangles, 0..) |*t, i|
        t.* = .init(v[3 * i], v[3 * i + 1], v[3 * i + 2], .{ .material_index = if (num_materials == 0) 0 else @intCast((i / 2) % num_materials), .user_data = @intCast(100 + i) });
    return triangles;
}

/// A height field like grid of n x n cells (2 triangles each), user data = triangle index
const TestGrid = struct {
    vertices: VertexList = .empty,
    triangles: IndexedTriangleList = .empty,

    fn init(allocator: Allocator, n: u32) !TestGrid {
        var grid: TestGrid = .{};
        errdefer grid.deinit(allocator);
        for (0..n + 1) |z|
            for (0..n + 1) |x| {
                const fx: f32 = @floatFromInt(x);
                const fz: f32 = @floatFromInt(z);
                try grid.vertices.append(allocator, .init(fx, @as(f32, @floatFromInt((x * 7 + z * 13) % 5)) * 0.25, fz));
            };
        for (0..n) |z|
            for (0..n) |x| {
                const v: u32 = @intCast(z * (n + 1) + x);
                for ([_][3]u32{ .{ v, v + n + 1, v + 1 }, .{ v + 1, v + n + 1, v + n + 2 } }) |idx| {
                    const t: u32 = @intCast(grid.triangles.items.len);
                    try grid.triangles.append(allocator, .init(idx[0], idx[1], idx[2], .{ .user_data = t }));
                }
            };
        return grid;
    }

    fn deinit(self: *TestGrid, allocator: Allocator) void {
        self.vertices.deinit(allocator);
        self.triangles.deinit(allocator);
    }
};

/// Create the shape of `settings` and return the result (the caller deinits it)
fn createMesh(settings: *MeshShapeSettings, allocator: Allocator) !ShapeResult {
    return settings.asShapeSettings().createShape(allocator);
}

/// Collects every triangle of a mesh with its active edges and sub shape ID (walkTreePerTriangle)
const TriangleRecorder = struct {
    vertices: [64][3]Vec3 = undefined,
    active_edges: [64]u8 = undefined,
    ids: [64]SubShapeID = undefined,
    count: u32 = 0,

    pub fn shouldAbort(self: *const TriangleRecorder) bool {
        _ = self;
        return false;
    }

    pub fn shouldVisitNode(self: *const TriangleRecorder, stack_top: i32) bool {
        _ = .{ self, stack_top };
        return true;
    }

    pub fn visitNodes(self: *const TriangleRecorder, min_x: Vec4, min_y: Vec4, min_z: Vec4, max_x: Vec4, max_y: Vec4, max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
        _ = .{ self, stack_top };
        const valid = UVec4.bitOr(UVec4.bitOr(Vec4.less(min_x, max_x), Vec4.less(min_y, max_y)), Vec4.less(min_z, max_z));
        return countAndSortTrues(valid, properties);
    }

    pub fn visitTriangle(self: *TriangleRecorder, v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
        self.vertices[self.count] = .{ v0, v1, v2 };
        self.active_edges[self.count] = active_edges;
        self.ids[self.count] = sub_shape_id2;
        self.count += 1;
    }

    /// The active edges of the triangle with vertices v0, v1, v2 (in this order, up to the quantization of the vertices)
    fn activeEdgesOf(self: *const TriangleRecorder, v0: Vec3, v1: Vec3, v2: Vec3) ?u8 {
        for (0..self.count) |i|
            if (self.vertices[i][0].isClose(v0, .{ .max_dist_sq = 1.0e-10 }) and self.vertices[i][1].isClose(v1, .{ .max_dist_sq = 1.0e-10 }) and self.vertices[i][2].isClose(v2, .{ .max_dist_sq = 1.0e-10 }))
                return self.active_edges[i];
        return null;
    }
};

test "MeshShapeSettings: constructors, Sanitize (degenerate and duplicate triangles)" {
    const allocator = testing.allocator;

    // From a triangle list: Indexify welds the 36 vertices of the box to 8
    var box_triangles = boxTriangles(0);
    var from_triangles = try MeshShapeSettings.init(allocator, &box_triangles, .{});
    defer from_triangles.deinit();
    try testing.expectEqual(@as(usize, 8), from_triangles.triangle_vertices.items.len);
    try testing.expectEqual(@as(usize, 12), from_triangles.indexed_triangles.items.len);
    try testing.expectEqual(@as(u32, 8), from_triangles.max_triangles_per_leaf);
    try testing.expectEqual(@as(f32, 0.996195), from_triangles.active_edge_cos_threshold_angle);
    try testing.expect(!from_triangles.per_triangle_user_data);
    try testing.expectEqual(MeshShapeSettings.BuildQuality.favor_runtime_performance, from_triangles.build_quality);

    // Indexed: a degenerate triangle (collinear), a duplicate (rotated, same material and user data), a triangle with
    // the same vertices but other user data (not a duplicate) and a triangle that is degenerate after quantization
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0), .init(2, 0, 0), .init(100000, 100000, 100000), .init(0.01, 0, 0), .init(0, 0.01, 0) };
    const triangles = [_]IndexedTriangle{
        .init(0, 1, 2, .{}),
        .init(0, 1, 3, .{}), // Collinear
        .init(1, 2, 0, .{}), // Duplicate of triangle 0
        .init(2, 0, 1, .{ .user_data = 1 }), // Not a duplicate: different user data
        .init(0, 2, 4, .{}),
        .init(0, 5, 6, .{}), // Degenerate after quantization
    };
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    var indexed = try MeshShapeSettings.initIndexed(allocator, &vertices, &triangles, .{ .materials = &.{ material.material(), material.material() } });
    defer indexed.deinit();
    try testing.expectEqual(@as(u32, 3), material.material().getRefCount());
    try testing.expectEqual(@as(usize, 3), indexed.indexed_triangles.items.len);
    try testing.expectEqual(@as(usize, 7), indexed.triangle_vertices.items.len);
    // Triangles are tested back to front and removed by swapping the last one into the slot, so the duplicate that is
    // kept is the last one
    try testing.expect(indexed.indexed_triangles.items[0].eql(.init(2, 0, 1, .{ .user_data = 1 })));
    try testing.expect(indexed.indexed_triangles.items[1].eql(.init(0, 2, 4, .{})));
    try testing.expect(indexed.indexed_triangles.items[2].eql(.init(1, 2, 0, .{})));

    // Heap settings
    const heap = try MeshShapeSettings.createIndexed(allocator, &vertices, &triangles, .{});
    var heap_ref = Ref(ShapeSettings).init(heap.asShapeSettings());
    defer heap_ref.deinit();
    const heap2 = try MeshShapeSettings.create(allocator, &box_triangles, .{ .materials = &.{material.material()} });
    var heap2_ref = Ref(ShapeSettings).init(heap2.asShapeSettings());
    defer heap2_ref.deinit();
    try testing.expectEqual(@as(usize, 12), heap2.indexed_triangles.items.len);
    try testing.expectEqual(@as(u32, 4), material.material().getRefCount());
}

test "MeshShape: Jolt's error texts" {
    const allocator = testing.allocator;
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0), .init(2, 0, 0), .init(100000, 100000, 100000), .init(0.01, 0, 0), .init(0, 0.01, 0) };

    const Case = struct { triangles: []const IndexedTriangle, num_materials: u32 = 0, max_triangles_per_leaf: u32 = 8, expected: []const u8 };
    const cases = [_]Case{
        .{ .triangles = &.{}, .expected = "Need triangles to create a mesh shape!" },
        .{ .triangles = &.{ .init(0, 1, 3, .{}), .init(0, 1, 2, .{}) }, .expected = "Triangle 0 is degenerate!" },
        .{ .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 1, 3, .{}), .init(2, 1, 0, .{}) }, .expected = "Triangle 1 is degenerate!" },
        .{ .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 2, 4, .{}), .init(0, 5, 6, .{}) }, .expected = "Triangle 2 is degenerate!" }, // In quantized space only
        .{ .triangles = &.{ .init(0, 1, 9, .{}), .init(0, 1, 3, .{}), .init(0, 7, 8, .{}) }, .expected = "Vertex index 7 is beyond vertex list (size: 7)" },
        .{ .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 2, 4, .{}) }, .num_materials = 33, .expected = "Supporting max 32 materials per mesh" },
        .{ .triangles = &.{ .init(0, 1, 2, .{ .material_index = 2 }), .init(0, 2, 4, .{ .material_index = 1 }) }, .num_materials = 2, .expected = "Triangle material 2 is beyond material list (size: 2)" },
        .{ .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 2, 4, .{ .material_index = 1 }) }, .expected = "No materials present, all triangles should have material index 0" },
        .{ .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 2, 4, .{}) }, .max_triangles_per_leaf = 0, .expected = "Invalid max triangles per leaf" },
        .{ .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 2, 4, .{}) }, .max_triangles_per_leaf = 9, .expected = "Invalid max triangles per leaf" },
    };

    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();

    for (cases) |c| {
        // The arrays are set directly: the constructors sanitize the mesh
        var settings = MeshShapeSettings.initDefault(allocator);
        defer settings.deinit();
        try settings.triangle_vertices.appendSlice(allocator, &vertices);
        try settings.indexed_triangles.appendSlice(allocator, c.triangles);
        for (0..c.num_materials) |_| try settings.materials.append(allocator, .init(material.material()));
        settings.max_triangles_per_leaf = c.max_triangles_per_leaf;
        var result = try createMesh(&settings, allocator);
        defer result.deinit();
        try testing.expectEqualStrings(c.expected, result.getError());
    }
    try testing.expectEqual(@as(u32, 1), material.material().getRefCount()); // The failed shapes released the materials

    // The same mesh without errors
    var settings = MeshShapeSettings.initDefault(allocator);
    defer settings.deinit();
    try settings.triangle_vertices.appendSlice(allocator, &vertices);
    try settings.indexed_triangles.appendSlice(allocator, &.{ IndexedTriangle.init(0, 1, 2, .{}), IndexedTriangle.init(0, 2, 4, .{}) });
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    try testing.expect(result.isValid());
}

test "MeshShape: queries on a closed box mesh (bounds, materials, normals, faces, rays, points, triangles, stats)" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var materials: [3]*PhysicsMaterialSimple = undefined;
    var material_refs: [3]RefConst(PhysicsMaterial) = undefined;
    for (&materials, &material_refs, 0..) |*m, *r, i| {
        m.* = try PhysicsMaterialSimple.create(allocator, "Mat", Color.getDistinctColor(@intCast(i)));
        r.* = .init(m.*.material());
    }
    defer for (&material_refs) |*r| r.deinit();

    var box_triangles = boxTriangles(3);
    var settings = try MeshShapeSettings.init(allocator, &box_triangles, .{ .materials = &.{ materials[0].material(), materials[1].material(), materials[2].material() } });
    defer settings.deinit();
    settings.per_triangle_user_data = true;
    settings.max_triangles_per_leaf = 4;
    settings.asShapeSettings().user_data = 77;
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    const mesh = shape.cast(MeshShape);

    try expect(shape.getType() == .mesh and shape.getSubType() == .mesh);
    try testing.expectEqual(@as(u64, 77), shape.getUserData());
    try expect(shape.mustBeStatic());
    try expect(shape.getLocalBounds().eql(.init(Vec3.replicate(-1), Vec3.replicate(1))));
    try expect(shape.getCenterOfMass().eql(Vec3.zero()));
    try testing.expectEqual(@as(f32, 0.0), shape.getInnerRadius());
    try testing.expectEqual(@as(f32, 0.0), shape.getVolume());
    const p = shape.getMassProperties();
    try testing.expectEqual(@as(f32, 0.0), p.mass);
    try expect(p.inertia.eql(Mat44.zero()));
    try testing.expectEqual(@as(usize, 3), mesh.getMaterialList().len);
    const bits = shape.getSubShapeIDBitsRecursive();
    try expect(bits > MeshShape.num_triangle_bits and bits <= 32);

    // Stats: the triangles in the tree
    const stats = shape.getStats();
    try testing.expectEqual(@as(u32, 12), stats.num_triangles);
    try testing.expectEqual(@sizeOf(MeshShape) + 3 * @sizeOf(RefConst(PhysicsMaterial)) + mesh.tree.vector.items.len, stats.size_bytes);

    // Every triangle once, with its material, user data and sub shape ID
    var recorder: TriangleRecorder = .{};
    const creator = SubShapeIDCreator.pushID(.{}, 5, 3);
    mesh.walkTreePerTriangle(creator, &recorder);
    try testing.expectEqual(@as(u32, 12), recorder.count);
    var seen = [_]bool{false} ** 12;
    for (0..12) |i| {
        // Remove the prefix of the creator
        const id = recorder.ids[i].popID(3);
        try testing.expectEqual(@as(u32, 5), id.id);
        const user_data = mesh.getTriangleUserData(id.remainder);
        try expect(user_data >= 100 and user_data < 112 and !seen[user_data - 100]);
        seen[user_data - 100] = true;
        const t = box_triangles[user_data - 100];
        try expect(recorder.vertices[i][0].eql(Vec3.fromFloat3(t.v[0])) and recorder.vertices[i][1].eql(Vec3.fromFloat3(t.v[1])) and recorder.vertices[i][2].eql(Vec3.fromFloat3(t.v[2])));
        try testing.expectEqual(t.material_index, mesh.getMaterialIndex(id.remainder));
        try expect(shape.getMaterial(id.remainder) == materials[t.material_index].material());
        try testing.expectEqual(@as(u8, if (user_data % 2 == 0) 0b011 else 0b110), recorder.active_edges[i]); // The 90 degree edges of the box are active, the diagonal of a face is not

        // Surface normal: the outward normal of the face
        const normal = shape.getSurfaceNormal(id.remainder, Vec3.zero());
        const expected_normal = Vec3.fromFloat3(t.v[1]).sub(Vec3.fromFloat3(t.v[0])).cross(Vec3.fromFloat3(t.v[2]).sub(Vec3.fromFloat3(t.v[0]))).normalized();
        try expect(normal.isClose(expected_normal, .{ .max_dist_sq = 1.0e-12 }));

        // Supporting face: the triangle transformed, the winding flips with an inside out scale
        var face: Shape.SupportingFace = .empty;
        const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.3), Vec3.init(1, 2, 3));
        shape.getSupportingFace(id.remainder, Vec3.axisX(), Vec3.init(2, 1, 1), transform, &face);
        try testing.expectEqual(@as(u32, 3), face.len);
        try expect(face.get(1).isClose(transform.mulVec3(Vec3.fromFloat3(t.v[1]).mul(Vec3.init(2, 1, 1))), .{ .max_dist_sq = 1.0e-10 }));
        shape.getSupportingFace(id.remainder, Vec3.axisX(), Vec3.init(-2, 1, 1), transform, &face);
        try expect(face.get(1).isClose(transform.mulVec3(Vec3.fromFloat3(t.v[2]).mul(Vec3.init(-2, 1, 1))), .{ .max_dist_sq = 1.0e-10 }));
    }

    // Ray cast (single hit): the top face, then nothing closer
    var hit: RayCastResult = .{};
    const ray = RayCast.init(Vec3.init(0.25, 3, 0.5), Vec3.init(0, -4, 0));
    try expect(shape.castRay(ray, creator, &hit));
    try testing.expectEqual(@as(f32, 0.5), hit.fraction);
    {
        const id = hit.sub_shape_id2.popID(3);
        try testing.expectEqual(@as(u32, 5), id.id);
        try expect(shape.getSurfaceNormal(id.remainder, Vec3.zero()).isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-12 }));
    }
    try expect(!shape.castRay(ray, creator, &hit));
    hit.fraction = 0.0; // Early out
    try expect(!shape.castRay(ray, creator, &hit));

    // Ray cast (collector): back faces, early out fraction, the context's body ID
    var settings_ray: RayCastSettings = .{};
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(9), .{});
    hits.base.setContext(&context);
    shape.castRayCollector(ray, &settings_ray, creator, &hits.base, &.{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.5), hits.hits.items[0].fraction);
    try expect(hits.hits.items[0].body_id.eql(.init(9)));
    hits.reset();
    settings_ray.back_face_mode_triangles = .collide_with_back_faces;
    shape.castRayCollector(ray, &settings_ray, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 1.5), hits.hits.items[0].fraction + hits.hits.items[1].fraction); // 0.5 and 1.0
    hits.reset();
    hits.base.updateEarlyOutFraction(0.75);
    shape.castRayCollector(ray, &settings_ray, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);

    // Collide point (odd number of hits of a ray means inside)
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.init(0.1, 0.2, 0.3), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(1.1, 0.2, 0.3), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(0.1, -1.5, 0.3), creator, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    try testing.expectEqual(@as(u32, 5), points.hits.items[0].sub_shape_id2.popID(3).id);

    // GetTrianglesStart / Next: 12 triangles, the materials of the triangles
    {
        var ctx: Shape.GetTrianglesContext = .{};
        const transform_rotation = Quat.rotation(Vec3.axisZ(), 0.5);
        shape.getTrianglesStart(&ctx, AABox.biggest(), Vec3.init(1, 2, 3), transform_rotation, Vec3.init(1, 2, -1));
        var out_vertices: [3 * 32]Float3 = undefined;
        var out_materials: [32]*const PhysicsMaterial = undefined;
        const n = shape.getTrianglesNext(&ctx, 32, &out_vertices, &out_materials);
        try testing.expectEqual(@as(u32, 12), n);
        try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&ctx, 32, &out_vertices, &out_materials));
        const to_world = Mat44.rotationTranslation(transform_rotation, Vec3.init(1, 2, 3)).mul(Mat44.scaleVec3(Vec3.init(1, 2, -1)));
        for (0..12) |t| {
            // Inside out scale: the winding is flipped, find the triangle by its first vertex and the other two swapped
            const w = [_]Vec3{ Vec3.fromFloat3(out_vertices[3 * t]), Vec3.fromFloat3(out_vertices[3 * t + 1]), Vec3.fromFloat3(out_vertices[3 * t + 2]) };
            var found = false;
            for (box_triangles) |bt| {
                if (w[0].isClose(to_world.mulVec3(Vec3.fromFloat3(bt.v[0])), .{ .max_dist_sq = 1.0e-10 }) and w[1].isClose(to_world.mulVec3(Vec3.fromFloat3(bt.v[2])), .{ .max_dist_sq = 1.0e-10 }) and w[2].isClose(to_world.mulVec3(Vec3.fromFloat3(bt.v[1])), .{ .max_dist_sq = 1.0e-10 })) {
                    try expect(out_materials[t] == materials[bt.material_index].material());
                    found = true;
                }
            }
            try expect(found);
        }

        // A box that only touches the top of the mesh: fewer triangles
        shape.getTrianglesStart(&ctx, .init(Vec3.init(-5, 4.5, -5), Vec3.init(5, 6, 5)), Vec3.init(0, 4, 0), Quat.identity(), Vec3.one());
        const top = shape.getTrianglesNext(&ctx, 32, &out_vertices, null);
        try expect(top > 0 and top < 12);
    }
}

test "MeshShape: GetTrianglesNext continues where the buffer was full, the default material" {
    const allocator = testing.allocator;

    var grid = try TestGrid.init(allocator, 6); // 72 triangles
    defer grid.deinit(allocator);
    var settings = try MeshShapeSettings.initIndexed(allocator, grid.vertices.items, grid.triangles.items, .{});
    defer settings.deinit();
    settings.build_quality = .favor_build_speed;
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    try testing.expectEqual(@as(u32, 72), shape.getStats().num_triangles);

    var ctx: Shape.GetTrianglesContext = .{};
    shape.getTrianglesStart(&ctx, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
    var out_vertices: [3 * 32]Float3 = undefined;
    var out_materials: [32]*const PhysicsMaterial = undefined;
    var total: u32 = 0;
    var calls: u32 = 0;
    while (true) {
        const n = shape.getTrianglesNext(&ctx, 32, &out_vertices, &out_materials);
        if (n == 0) break;
        try testing.expect(n <= 32);
        for (out_materials[0..n]) |m| try testing.expect(m == PhysicsMaterial.default);
        total += n;
        calls += 1;
    }
    try testing.expectEqual(@as(u32, 72), total);
    try testing.expect(calls >= 3);
    try testing.expect(shape.getMaterial(.{ .value = 0 }) == PhysicsMaterial.default);
}

test "MeshShape: active edges (coplanar, convex, concave, 3 triangles on an edge, negative threshold)" {
    const allocator = testing.allocator;

    const Case = struct { vertices: []const Float3, triangles: []const IndexedTriangle, threshold: f32 = 0.996195, expected: []const u8 };
    const cases = [_]Case{
        // A quad of 2 coplanar triangles: the shared edge (edge 2 of the first, edge 0 of the second) is inactive
        .{ .vertices = &.{ .init(0, 0, 0), .init(0, 0, 1), .init(1, 0, 1), .init(1, 0, 0) }, .triangles = &.{ .init(0, 1, 2, .{}), .init(2, 3, 0, .{}) }, .expected = &.{ 0b011, 0b011 } },
        // A convex fold of 90 degrees (a floor facing up and a wall below it facing away): active
        .{ .vertices = &.{ .init(0, 0, 0), .init(0, 0, 1), .init(1, 0, 1), .init(0, -1, 0), .init(0, -1, 1) }, .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 3, 4, .{}), .init(0, 4, 1, .{}) }, .expected = &.{ 0b111, 0b011, 0b110 } },
        // A concave fold of 90 degrees (a floor facing up and a wall above it facing the floor): inactive
        .{ .vertices = &.{ .init(0, 0, 0), .init(0, 0, 1), .init(1, 0, 1), .init(0, 1, 0), .init(0, 1, 1) }, .triangles = &.{ .init(0, 1, 2, .{}), .init(0, 3, 4, .{}), .init(0, 4, 1, .{}) }, .expected = &.{ 0b110, 0b011, 0b010 } },
        // 3 triangles share the edge 0-1: active for all of them
        .{ .vertices = &.{ .init(0, 0, 0), .init(0, 0, 1), .init(1, 0, 1), .init(-1, 0, 1), .init(0, 1, 1) }, .triangles = &.{ .init(0, 1, 2, .{}), .init(1, 0, 3, .{}), .init(0, 1, 4, .{}) }, .expected = &.{ 0b111, 0b111, 0b111 } },
        // Negative threshold: all edges are active
        .{ .vertices = &.{ .init(0, 0, 0), .init(0, 0, 1), .init(1, 0, 1), .init(1, 0, 0) }, .triangles = &.{ .init(0, 1, 2, .{}), .init(2, 3, 0, .{}) }, .threshold = -1.0, .expected = &.{ 0b111, 0b111 } },
    };

    for (cases, 0..) |c, case_index| {
        errdefer std.debug.print("case {d}\n", .{case_index});
        var settings = try MeshShapeSettings.initIndexed(allocator, c.vertices, c.triangles, .{});
        defer settings.deinit();
        settings.active_edge_cos_threshold_angle = c.threshold;
        var result = try createMesh(&settings, allocator);
        defer result.deinit();
        const mesh = result.getPtr().?.cast(MeshShape);

        var recorder: TriangleRecorder = .{};
        mesh.walkTreePerTriangle(.{}, &recorder);
        try testing.expectEqual(@as(u32, @intCast(c.triangles.len)), recorder.count);
        for (c.triangles, c.expected) |t, expected| {
            const v = [_]Vec3{ Vec3.fromFloat3(c.vertices[t.idx[0]]), Vec3.fromFloat3(c.vertices[t.idx[1]]), Vec3.fromFloat3(c.vertices[t.idx[2]]) };
            try testing.expectEqual(@as(?u8, expected), recorder.activeEdgesOf(v[0], v[1], v[2]));
        }
    }
}

test "MeshShape: collide and cast convex shapes vs the mesh (CollisionDispatch), soft body vertices" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var grid = try TestGrid.init(allocator, 4);
    defer grid.deinit(allocator);
    var settings = try MeshShapeSettings.initIndexed(allocator, grid.vertices.items, grid.triangles.items, .{});
    defer settings.deinit();
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const mesh = result.getPtr().?;

    var sphere = SphereShape.init(allocator, 0.5, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.replicate(0.5), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    const mesh_transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.2), Vec3.init(-2, 0, -2));
    const shapes = [_]*const Shape{ sphere.asShape(), box.asShape() };
    for (shapes) |convex| {
        // Collide: the convex shape sinks into the grid (the highest vertex is at y = 1)
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        var collide_settings: CollideShapeSettings = .{};
        collide_settings.back_face_mode = .collide_with_back_faces;
        CollisionDispatch.collideShapeVsShape(convex, mesh, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0, 0.5, 0)), mesh_transform, .{}, .{}, &collide_settings, &collector.base, &.{});
        try collector.checkError();
        try expect(collector.hits.items.len > 0);

        // Reversed: the mesh vs the convex shape
        var reversed = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer reversed.deinit();
        CollisionDispatch.collideShapeVsShape(mesh, convex, Vec3.one(), Vec3.one(), mesh_transform, Mat44.translation(Vec3.init(0, 0.5, 0)), .{}, .{}, &collide_settings, &reversed.base, &.{});
        try reversed.checkError();
        try testing.expectEqual(collector.hits.items.len, reversed.hits.items.len);

        // Far away: nothing
        var none = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer none.deinit();
        CollisionDispatch.collideShapeVsShape(convex, mesh, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0, 10, 0)), mesh_transform, .{}, .{}, &collide_settings, &none.base, &.{});
        try testing.expectEqual(@as(usize, 0), none.hits.items.len);

        // Cast down onto the grid: the closest hit
        var cast_collector = ClosestHitCollisionCollector(CastShapeCollector).init();
        defer cast_collector.deinit();
        const cast = ShapeCast.init(convex, Vec3.one(), Mat44.translation(Vec3.init(0, 5, 0)), Vec3.init(0, -10, 0));
        var cast_settings: ShapeCastSettings = .{};
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &cast_settings, mesh, Vec3.one(), &.{}, mesh_transform, .{}, .{}, &cast_collector.base);
        try expect(cast_collector.hadHit());
        try expect(cast_collector.hit.fraction > 0.3 and cast_collector.hit.fraction < 0.5);

        // Cast up from below: back faces only
        var back_collector = ClosestHitCollisionCollector(CastShapeCollector).init();
        defer back_collector.deinit();
        const cast_up = ShapeCast.init(convex, Vec3.one(), Mat44.translation(Vec3.init(0, -5, 0)), Vec3.init(0, 10, 0));
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast_up, &cast_settings, mesh, Vec3.one(), &.{}, mesh_transform, .{}, .{}, &back_collector.base);
        try expect(!back_collector.hadHit());
        cast_settings.back_face_mode_triangles = .collide_with_back_faces;
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast_up, &cast_settings, mesh, Vec3.one(), &.{}, mesh_transform, .{}, .{}, &back_collector.base);
        try expect(back_collector.hadHit() and back_collector.hit.is_back_face_hit);
    }

    // Soft body vertices: just above the surface (at 0.625 here), far above, far outside the grid, infinite mass
    var positions = [_]Vec3{ Vec3.init(1.5, 0.7, 1.5), Vec3.init(1.5, 5, 1.5), Vec3.init(-50, 10, -50), Vec3.init(2.5, 0.0, 2.5) };
    var inv_masses = [_]f32{ 1, 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 4;
    var penetrations = [_]f32{-math.flt_max} ** 4;
    var indices = [_]i32{-1} ** 4;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    mesh.collideSoftBodyVertices(Mat44.identity(), Vec3.one(), &vertices, 4, 3);
    try testing.expectEqual(@as(i32, 3), indices[0]);
    try expect(penetrations[0] < 0.0 and penetrations[0] > -0.1);
    try testing.expectEqual(@as(i32, 3), indices[1]);
    try expect(penetrations[1] < -3.0);
    try testing.expectEqual(@as(i32, 3), indices[2]);
    try testing.expectEqual(@as(i32, -1), indices[3]);
}

test "MeshShape: binary state, material state, SaveWithChildren and the registration" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    var box_triangles = boxTriangles(1);
    var settings = try MeshShapeSettings.init(allocator, &box_triangles, .{ .materials = &.{material.material()} });
    defer settings.deinit();
    settings.per_triangle_user_data = true;
    settings.asShapeSettings().user_data = 5;
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    const mesh = shape.cast(MeshShape);

    // SaveBinaryState: sub type, user data, the tree (length + bytes)
    var buffer: std.Io.Writer.Allocating = .init(allocator);
    defer buffer.deinit();
    var out = StreamWrapper.StreamOutWrapper.init(&buffer.writer);
    shape.saveBinaryState(out.streamOut());
    const bytes = buffer.written();
    try testing.expectEqual(1 + 8 + 4 + mesh.tree.vector.items.len, bytes.len);
    try testing.expectEqual(@intFromEnum(ShapeSubType.mesh), bytes[0]);

    // Restore: the same tree, no materials until RestoreMaterialState
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var restored = try Shape.restoreFromBinaryState(allocator, in.streamIn());
    defer restored.deinit();
    const restored_mesh = restored.getPtr().?.castMut(MeshShape);
    try testing.expectEqualSlices(u8, mesh.tree.vector.items, restored_mesh.tree.vector.items);
    try testing.expect(std.mem.isAligned(@intFromPtr(restored_mesh.tree.vector.items.ptr), Core.cache_line_size));
    try testing.expectEqual(@as(u64, 5), restored_mesh.asShape().getUserData());
    try testing.expectEqual(@as(usize, 0), restored_mesh.getMaterialList().len);
    try expect(restored_mesh.asShape().getMaterial(.{ .value = 0 }) == PhysicsMaterial.default);

    // Material state: SaveMaterialState replaces the list, RestoreMaterialState assigns it
    var materials: PhysicsMaterialList = .empty;
    defer {
        for (materials.items) |*m| m.deinit();
        materials.deinit(allocator);
    }
    try materials.append(allocator, .init(PhysicsMaterial.default));
    try materials.append(allocator, .init(PhysicsMaterial.default));
    try shape.saveMaterialState(allocator, &materials);
    try testing.expectEqual(@as(usize, 1), materials.items.len);
    try expect(materials.items[0].get() == material.material());
    try restored_mesh.asShapeMut().restoreMaterialState(materials.items);
    try testing.expectEqual(@as(usize, 1), restored_mesh.getMaterialList().len);
    try testing.expectEqual(@as(u32, 5), material.material().getRefCount()); // ref, settings, shape, list, restored

    // Queries on the restored shape give the same results
    var hit1: RayCastResult = .{};
    var hit2: RayCastResult = .{};
    const ray = RayCast.init(Vec3.init(0.3, 0.2, -4), Vec3.init(0.1, 0.1, 8));
    try expect(shape.castRay(ray, .{}, &hit1) and restored_mesh.asShape().castRay(ray, .{}, &hit2));
    try testing.expectEqual(hit1.fraction, hit2.fraction);
    try expect(hit1.sub_shape_id2.eql(hit2.sub_shape_id2));
    try testing.expectEqual(mesh.getTriangleUserData(hit1.sub_shape_id2), restored_mesh.getTriangleUserData(hit2.sub_shape_id2));
    try expect(restored_mesh.asShape().getMaterial(hit2.sub_shape_id2) == material.material());

    // A truncated stream: the restore fails
    {
        var truncated: std.Io.Reader = .fixed(bytes[0..11]);
        var truncated_in = StreamWrapper.StreamInWrapper.init(&truncated);
        var r = try Shape.restoreFromBinaryState(allocator, truncated_in.streamIn());
        defer r.deinit();
        try testing.expectEqualStrings("Failed to restore shape", r.getError());
    }

    // SaveWithChildren / RestoreWithChildren (the materials are saved with the shape)
    {
        var children: std.Io.Writer.Allocating = .init(allocator);
        defer children.deinit();
        var children_out = StreamWrapper.StreamOutWrapper.init(&children.writer);
        var shape_map: Shape.ShapeToIDMap = .empty;
        defer shape_map.deinit(allocator);
        var material_map: Shape.MaterialToIDMap = .empty;
        defer material_map.deinit(allocator);
        try shape.saveWithChildren(allocator, children_out.streamOut(), &shape_map, &material_map);

        var children_reader: std.Io.Reader = .fixed(children.written());
        var children_in = StreamWrapper.StreamInWrapper.init(&children_reader);
        var id_to_shape: Shape.IDToShapeMap = .empty;
        defer {
            for (id_to_shape.items) |*s| s.deinit();
            id_to_shape.deinit(allocator);
        }
        var id_to_material: Shape.IDToMaterialMap = .empty;
        defer {
            for (id_to_material.items) |*m| m.deinit();
            id_to_material.deinit(allocator);
        }
        var r = try Shape.restoreWithChildren(allocator, children_in.streamIn(), &id_to_shape, &id_to_material);
        defer r.deinit();
        const rm = r.getPtr().?.cast(MeshShape);
        try testing.expectEqualSlices(u8, mesh.tree.vector.items, rm.tree.vector.items);
        try testing.expectEqual(@as(usize, 1), rm.getMaterialList().len);
        const restored_material = rm.getMaterialList()[0].get().?;
        try expect(restored_material != material.material()); // A new material with the same name and color
        try testing.expectEqualStrings("Mat", restored_material.getDebugName());
    }

    // Registration: the constructor, the color and the collision functions
    const functions = ShapeFunctions.get(.mesh);
    try expect(functions.construct != null);
    try expect(functions.color.eql(Color.red));
    const registry = &RegisterTypes.registry;
    try expect(registry.getCollideShape(.box, .mesh) == &MeshShape.collideConvexVsMesh);
    try expect(registry.getCollideShape(.sphere, .mesh) == &MeshShape.collideSphereVsMesh);
    try expect(registry.getCastShape(.box, .mesh) == &MeshShape.castConvexVsMesh);
    try expect(registry.getCastShape(.sphere, .mesh) == &MeshShape.castSphereVsMesh);
    try expect(registry.getCollideShape(.mesh, .user_convex8) == &CollisionDispatch.reversedCollideShape);
    try expect(registry.getCastShape(.mesh, .capsule) == &CollisionDispatch.reversedCastShape);
}

test "MeshShape: null entries in the material list" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // Materials 0 and 2 are set, 1 is null (Jolt allows nullptr in the PhysicsMaterialList)
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    var box_triangles = boxTriangles(3);
    var settings = try MeshShapeSettings.init(allocator, &box_triangles, .{});
    defer settings.deinit();
    try settings.materials.append(allocator, .init(material.material()));
    try settings.materials.append(allocator, .init(null));
    try settings.materials.append(allocator, .init(material.material()));
    settings.per_triangle_user_data = true;
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    const mesh = shape.cast(MeshShape);

    // The list keeps the null entry
    try testing.expectEqual(@as(usize, 3), mesh.getMaterialList().len);
    try expect(mesh.getMaterialList()[1].get() == null);

    // GetMaterial: the default material for the null entry (Jolt: nullptr)
    var recorder: TriangleRecorder = .{};
    mesh.walkTreePerTriangle(.{}, &recorder);
    try testing.expectEqual(@as(u32, 12), recorder.count);
    for (recorder.ids[0..recorder.count]) |id| {
        const material_index = box_triangles[mesh.getTriangleUserData(id) - 100].material_index;
        try testing.expectEqual(material_index, mesh.getMaterialIndex(id));
        try expect(shape.getMaterial(id) == if (material_index == 1) PhysicsMaterial.default else material.material());
    }

    // GetTrianglesNext: the default material for the null entry (Jolt: nullptr)
    {
        var ctx: Shape.GetTrianglesContext = .{};
        shape.getTrianglesStart(&ctx, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
        var out_vertices: [3 * 32]Float3 = undefined;
        var out_materials: [32]*const PhysicsMaterial = undefined;
        try testing.expectEqual(@as(u32, 12), shape.getTrianglesNext(&ctx, 32, &out_vertices, &out_materials));
        var num_default: u32 = 0;
        for (out_materials[0..12]) |m| {
            if (m == PhysicsMaterial.default)
                num_default += 1
            else
                try expect(m == material.material());
        }
        try testing.expectEqual(@as(u32, 4), num_default); // Triangles 2, 3, 8 and 9 use material 1
    }

    // SaveWithChildren / RestoreWithChildren: the null entry is saved as null ID and restored as null
    {
        var children: std.Io.Writer.Allocating = .init(allocator);
        defer children.deinit();
        var children_out = StreamWrapper.StreamOutWrapper.init(&children.writer);
        var shape_map: Shape.ShapeToIDMap = .empty;
        defer shape_map.deinit(allocator);
        var material_map: Shape.MaterialToIDMap = .empty;
        defer material_map.deinit(allocator);
        try shape.saveWithChildren(allocator, children_out.streamOut(), &shape_map, &material_map);

        var children_reader: std.Io.Reader = .fixed(children.written());
        var children_in = StreamWrapper.StreamInWrapper.init(&children_reader);
        var id_to_shape: Shape.IDToShapeMap = .empty;
        defer {
            for (id_to_shape.items) |*s| s.deinit();
            id_to_shape.deinit(allocator);
        }
        var id_to_material: Shape.IDToMaterialMap = .empty;
        defer {
            for (id_to_material.items) |*m| m.deinit();
            id_to_material.deinit(allocator);
        }
        var r = try Shape.restoreWithChildren(allocator, children_in.streamIn(), &id_to_shape, &id_to_material);
        defer r.deinit();
        const rm = r.getPtr().?.cast(MeshShape);
        const list = rm.getMaterialList();
        try testing.expectEqual(@as(usize, 3), list.len);
        try expect(list[0].get() != null and list[1].get() == null and list[0].get() == list[2].get());
        for (recorder.ids[0..recorder.count]) |id|
            try expect((rm.asShape().getMaterial(id) == PhysicsMaterial.default) == (mesh.getMaterialIndex(id) == 1));
    }
}

test "MeshShape: GetSubmergedVolume is not supported" {
    // Jolt asserts, Zolt panics with asserts enabled (not testable) and returns zeros otherwise
    if (Core.enable_asserts) return error.SkipZigTest;

    const allocator = testing.allocator;
    var box_triangles = boxTriangles(0);
    var settings = try MeshShapeSettings.init(allocator, &box_triangles, .{});
    defer settings.deinit();
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const v = result.getPtr().?.getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.init(Vec3.axisY(), 0.0));
    try testing.expectEqual(@as(f32, 0.0), v.total_volume);
    try testing.expectEqual(@as(f32, 0.0), v.submerged_volume);
    try testing.expect(v.center_of_buoyancy.eql(Vec3.zero()));
}

test "MeshShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;

    var grid = try TestGrid.init(allocator, 5);
    defer grid.deinit(allocator);
    var box_triangles = boxTriangles(2);
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    const materials = [_]*const PhysicsMaterial{ material.material(), material.material() };

    // The constructors of the settings (Indexify, Sanitize) and the heap settings
    for (0..4) |path| {
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            const a = failing.allocator();
            switch (path) {
                0 => {
                    var s = MeshShapeSettings.init(a, &box_triangles, .{ .materials = &materials }) catch continue;
                    s.deinit();
                },
                1 => {
                    var s = MeshShapeSettings.initIndexed(a, grid.vertices.items, grid.triangles.items, .{ .materials = &materials }) catch continue;
                    s.deinit();
                },
                2 => {
                    const s = MeshShapeSettings.create(a, &box_triangles, .{ .materials = &materials }) catch continue;
                    var ref = Ref(ShapeSettings).init(s.asShapeSettings());
                    ref.deinit();
                },
                else => {
                    const s = MeshShapeSettings.createIndexed(a, grid.vertices.items, grid.triangles.items, .{}) catch continue;
                    var ref = Ref(ShapeSettings).init(s.asShapeSettings());
                    ref.deinit();
                },
            }
            break;
        }
        try testing.expect(fail_index > 1);
    }
    try testing.expectEqual(@as(u32, 1), material.material().getRefCount());

    // createShape: the shape, the materials, the active edges, the splitter, the builder and the buffer. Out of
    // memory is not cached, a later call succeeds.
    for ([_]MeshShapeSettings.BuildQuality{ .favor_runtime_performance, .favor_build_speed }) |quality| {
        var settings = try MeshShapeSettings.init(allocator, &box_triangles, .{ .materials = &materials });
        defer settings.deinit();
        settings.build_quality = quality;
        settings.per_triangle_user_data = true;
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var r = createMesh(&settings, failing.allocator()) catch |err| {
                try testing.expectEqual(error.OutOfMemory, err);
                try testing.expect(settings.base.cached_result.isEmpty());
                continue;
            };
            defer r.deinit();
            try testing.expect(r.isValid());
            break;
        }
        try testing.expect(fail_index > 5);
    }

    // Restore (the shape and the tree), RestoreMaterialState, SaveMaterialState
    var settings = try MeshShapeSettings.init(allocator, &box_triangles, .{ .materials = &materials });
    defer settings.deinit();
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    var buffer: std.Io.Writer.Allocating = .init(allocator);
    defer buffer.deinit();
    var out = StreamWrapper.StreamOutWrapper.init(&buffer.writer);
    shape.saveBinaryState(out.streamOut());
    for (0..2) |fail_index| {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var reader: std.Io.Reader = .fixed(buffer.written());
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        try testing.expectError(error.OutOfMemory, Shape.restoreFromBinaryState(failing.allocator(), in.streamIn()));
    }
    {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 2 });
        var reader: std.Io.Reader = .fixed(buffer.written());
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var r = try Shape.restoreFromBinaryState(failing.allocator(), in.streamIn());
        defer r.deinit();
        try testing.expectError(error.OutOfMemory, r.getPtr().?.restoreMaterialState(shape.cast(MeshShape).getMaterialList()));
        try testing.expectEqual(@as(usize, 0), r.getPtr().?.cast(MeshShape).getMaterialList().len);

        var list: PhysicsMaterialList = .empty;
        try testing.expectError(error.OutOfMemory, shape.saveMaterialState(failing.allocator(), &list));
    }
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.mesh).construct.?(std.testing.failing_allocator));
}

test "MeshShape: 3 coplanar triangles that share an edge (the checks of Jolt's TestNonManifoldMesh)" {
    const allocator = testing.allocator;

    // Test 3 triangles in a plane that all share the same edge
    // Normally the shared edge would not be active, but since the mesh is non-manifold we expect all of them to be active
    const triangles = [_]Triangle{
        .fromFloat3(.init(0, 0, -1), .init(0, 0, 1), .init(1, 0, 0), .{}),
        .fromFloat3(.init(0, 0, 1), .init(0, 0, -1), .init(-1, 0, 0), .{}),
        .fromFloat3(.init(0, 0, 1), .init(0, 0, -1), .init(-0.5, 0, 0), .{}),
    };
    var settings = try MeshShapeSettings.init(allocator, &triangles, .{});
    defer settings.deinit();
    var result = try createMesh(&settings, allocator);
    defer result.deinit();
    const shape = result.getPtr().?;

    var sphere = SphereShape.init(allocator, 0.1, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    var collide_settings: CollideShapeSettings = .{};
    collide_settings.active_edge_mode = .collide_only_with_active;

    // Collide a sphere on both sides of the active edge so that a 45 degree normal will be found then the edge is active.
    // An inactive edge will return a normal that is perpendicular to the plane.
    const Case = struct { x: f32, interior: u32, shared: u32, edge_normal: Vec3 };
    const cases = [_]Case{
        // One interior hit because the sphere is above the triangle and 2 active edge hits that provide a normal pointing towards the sphere
        .{ .x = 0.05, .interior = 1, .shared = 2, .edge_normal = Vec3.init(-1, -1, 0).normalized() },
        // 2 interior hits because the sphere is above the triangle and 1 active edge hit that provide a normal pointing towards the sphere
        .{ .x = -0.05, .interior = 2, .shared = 1, .edge_normal = Vec3.init(1, -1, 0).normalized() },
    };
    for (cases) |c| {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(sphere.asShape(), shape, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(c.x, 0.05, 0)), Mat44.identity(), .{}, .{}, &collide_settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 3), collector.hits.items.len);

        var num_interior: u32 = 0;
        var num_on_shared_edge: u32 = 0;
        for (collector.hits.items) |r| {
            if (r.contact_point_on2.isClose(Vec3.init(c.x, 0.0, 0.0), .{})) {
                try testing.expect(r.penetration_axis.normalized().isClose(Vec3.init(0, -1, 0), .{}));
                num_interior += 1;
            } else if (r.contact_point_on2.isNearZero(.{})) {
                try testing.expect(r.penetration_axis.normalized().isClose(c.edge_normal, .{ .max_dist_sq = 1.0e-10 }));
                num_on_shared_edge += 1;
            }
        }
        try testing.expectEqual(c.interior, num_interior);
        try testing.expectEqual(c.shared, num_on_shared_edge);
    }
}
