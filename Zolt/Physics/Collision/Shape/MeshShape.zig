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

        return self.materials.items[self.getMaterialIndex(sub_shape_id)].get().?;
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

                    // Store materials
                    for (flags[0..num_triangles]) |f| {
                        materials[self.materials_pos] = self.shape.materials.items[f & flags_material_mask].get().?;
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
