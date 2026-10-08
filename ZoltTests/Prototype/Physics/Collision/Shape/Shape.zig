//! Port of: Jolt/Physics/Collision/Shape/Shape.h, Jolt/Physics/Collision/Shape/Shape.cpp (prototype, reduced)
//! Status: partial
//! Missing: GetSubShapeTransformedShape, GetSubmergedVolume, CollideSoftBodyVertices, GetStatsRecursive, sCollidePointUsingRayCast, GetWorldSpaceBounds(DMat44), JPH_DEBUG_RENDERER
//!
//! Architecture (Docs/Zolt/CollisionArchitecture.md):
//! - D1 `Shape` is the root of pattern A: `vtable`, the atomic `ref_count`, the `allocator` that frees the shape and its
//!   internal arrays, `user_data`, `shape_type` and `shape_sub_type`. One `Shape.VTable` entry per C++ virtual function
//!   (declaration order) plus the generated `deinit` / `destroy`, one dispatcher with the same name per entry, the
//!   default bodies in `Shape.impl`. Vtables are built by `virtual.make` (Core/Virtual.zig).
//! - Shapes are immutable after creation and shared between threads: every query takes `*const Shape`. Only the
//!   creation / restore paths take `*Shape` (setUserData, restore*State, called before the shape is shared).
//! - D2 heap shapes: `T.create(allocator, ...)` (refcount 0, like `new`), kept alive by `Ref(Shape)` / `RefConst(Shape)`,
//!   `release()` destroys them through the generated `destroy` entry. Stack / member shapes: `T.init(allocator, ...)`,
//!   `asShape().setEmbedded()` before handing out references, `asShapeMut().deinit()` at the end of their scope.
//! - D3 `ShapeSettings.createShape(allocator)` returns `Allocator.Error!ShapeResult` (ShapeResult = Result(Ref(Shape))
//!   with Jolt's error texts); the shape constructors that take settings are `initFromSettings(self, settings, result,
//!   allocator)`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const AABox = zolt.AABox;
const Color = zolt.Color;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RefCount = zolt.RefCount;
const StaticArray = zolt.StaticArray;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const Vec3 = zolt.Vec3;
const ObjectToIDMap = zolt.ObjectToIDMap;
const IDToObjectMap = zolt.IDToObjectMap;

const virtual = @import("../../../Core/Virtual.zig");
const Result = @import("../../../Core/Result.zig").Result;
const PlacementBuffer = @import("../../../Core/PlacementBuffer.zig").PlacementBuffer;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const ScaleHelpers = @import("ScaleHelpers.zig");
const PhysicsMaterialFile = @import("../PhysicsMaterial.zig");
const PhysicsMaterial = PhysicsMaterialFile.PhysicsMaterial;
const CollisionCollectorFile = @import("../CollisionCollector.zig");
const CollisionCollector = CollisionCollectorFile.CollisionCollector;
const RayCast = @import("../RayCast.zig").RayCast;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollidePointResult = @import("../CollidePointResult.zig").CollidePointResult;
const CollideShapeResult = @import("../CollideShape.zig").CollideShapeResult;
const ShapeCastResult = @import("../ShapeCast.zig").ShapeCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShapeFile = @import("../TransformedShape.zig");
const TransformedShape = TransformedShapeFile.TransformedShape;
// Cyclic imports (Shape.cpp uses ScaledShape and StaticCompoundShape in ScaleShape)
const ScaledShape = @import("ScaledShape.zig").ScaledShape;
const StaticCompoundShapeSettings = @import("StaticCompoundShape.zig").StaticCompoundShapeSettings;

pub const CastRayCollector = CollisionCollector(RayCastResult, CollisionCollectorFile.CollisionCollectorTraitsCastRay);
pub const CastShapeCollector = CollisionCollector(ShapeCastResult, CollisionCollectorFile.CollisionCollectorTraitsCastShape);
pub const CollidePointCollector = CollisionCollector(CollidePointResult, CollisionCollectorFile.CollisionCollectorTraitsCollidePoint);
pub const CollideShapeCollector = CollisionCollector(CollideShapeResult, CollisionCollectorFile.CollisionCollectorTraitsCollideShape);
pub const TransformedShapeCollector = CollisionCollector(TransformedShape, CollisionCollectorFile.CollisionCollectorTraitsCollideShape);

pub const ShapeRefC = RefConst(Shape);
pub const ShapeList = std.ArrayList(ShapeRefC);
pub const PhysicsMaterialRefC = RefConst(PhysicsMaterial);
pub const PhysicsMaterialList = PhysicsMaterialFile.PhysicsMaterialList;
pub const ShapeToIDMap = ObjectToIDMap(Shape);
pub const IDToShapeMap = IDToObjectMap(Shape);
pub const MaterialToIDMap = ObjectToIDMap(PhysicsMaterial);
pub const IDToMaterialMap = IDToObjectMap(PhysicsMaterial);

/// Shapes are categorized in groups, each shape can return which group it belongs to through its getType function.
pub const ShapeType = enum(u8) {
    /// Used by ConvexShape, all shapes that use the generic convex vs convex collision detection system (box, sphere, capsule, tapered capsule, cylinder, triangle)
    convex,
    /// Used by CompoundShape
    compound,
    /// Used by DecoratedShape
    decorated,
    /// Used by MeshShape
    mesh,
    /// Used by HeightFieldShape
    height_field,
    /// Used by SoftBodyShape
    soft_body,
    // User defined shapes
    user1,
    user2,
    user3,
    user4,
    /// Used by PlaneShape
    plane,
    /// Used by EmptyShape
    empty,
};

/// This enumerates all shape types, each shape can return its type through getSubType
pub const ShapeSubType = enum(u8) {
    // Convex shapes
    sphere,
    box,
    triangle,
    capsule,
    tapered_capsule,
    cylinder,
    convex_hull,
    // Compound shapes
    static_compound,
    mutable_compound,
    // Decorated shapes
    rotated_translated,
    scaled,
    offset_center_of_mass,
    // Other shapes
    mesh,
    height_field,
    soft_body,
    // User defined shapes
    user1,
    user2,
    user3,
    user4,
    user5,
    user6,
    user7,
    user8,
    // User defined convex shapes
    user_convex1,
    user_convex2,
    user_convex3,
    user_convex4,
    user_convex5,
    user_convex6,
    user_convex7,
    user_convex8,
    // Other shapes
    plane,
    tapered_cylinder,
    empty,
};

// Sets of shape sub types (same element order as Jolt: registration loops iterate them)
pub const all_sub_shape_types = [_]ShapeSubType{ .sphere, .box, .triangle, .capsule, .tapered_capsule, .cylinder, .convex_hull, .static_compound, .mutable_compound, .rotated_translated, .scaled, .offset_center_of_mass, .mesh, .height_field, .soft_body, .user1, .user2, .user3, .user4, .user5, .user6, .user7, .user8, .user_convex1, .user_convex2, .user_convex3, .user_convex4, .user_convex5, .user_convex6, .user_convex7, .user_convex8, .plane, .tapered_cylinder, .empty };
pub const convex_sub_shape_types = [_]ShapeSubType{ .sphere, .box, .triangle, .capsule, .tapered_capsule, .cylinder, .convex_hull, .tapered_cylinder, .user_convex1, .user_convex2, .user_convex3, .user_convex4, .user_convex5, .user_convex6, .user_convex7, .user_convex8 };
pub const compound_sub_shape_types = [_]ShapeSubType{ .static_compound, .mutable_compound };
pub const decorator_sub_shape_types = [_]ShapeSubType{ .rotated_translated, .scaled, .offset_center_of_mass };

/// How many shape types we support
pub const num_sub_shape_types: u32 = all_sub_shape_types.len;

/// Names of sub shape types
pub const sub_shape_type_names = [_][]const u8{ "Sphere", "Box", "Triangle", "Capsule", "TaperedCapsule", "Cylinder", "ConvexHull", "StaticCompound", "MutableCompound", "RotatedTranslated", "Scaled", "OffsetCenterOfMass", "Mesh", "HeightField", "SoftBody", "User1", "User2", "User3", "User4", "User5", "User6", "User7", "User8", "UserConvex1", "UserConvex2", "UserConvex3", "UserConvex4", "UserConvex5", "UserConvex6", "UserConvex7", "UserConvex8", "Plane", "TaperedCylinder", "Empty" };

comptime {
    std.debug.assert(sub_shape_type_names.len == num_sub_shape_types);
    for (all_sub_shape_types, 0..) |s, i| std.debug.assert(@intFromEnum(s) == i);
}

/// Result<Ref<Shape>> (ShapeSettings::ShapeResult)
pub const ShapeResult = Result(Ref(Shape));

/// Class that can construct shapes (pattern A RefTarget root). ObjectStream serialization: TODO(serialization).
/// Settings on the stack: `X.init(allocator, ...)` + `defer s.asShapeSettings().deinit()`; on the heap:
/// `X.create(allocator, ...)` (reference counted, freed by the last release()).
pub const ShapeSettings = struct {
    pub const VTable = struct {
        /// Destructor chain (generated)
        deinit: *const fn (self: *ShapeSettings) void,
        /// delete this (generated)
        destroy: *const fn (self: *ShapeSettings) void,
        /// Create a shape according to the settings specified by this object (Create() const in C++). Mutable receiver:
        /// it writes the cache (C++ `mutable`, Rule M).
        createShape: *const fn (self: *ShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult,
    };

    vtable: *const VTable,
    ref_count: RefCount = .{},
    /// Frees heap settings and the memory the settings own (e.g. the sub shape list of a compound)
    allocator: Allocator,
    /// User data (to be used freely by the application)
    user_data: u64 = 0,
    /// When creating a shape, the result is cached so that calling Create() again will return the same shape.
    /// If you make changes to the ShapeSettings you need to call clearCachedResult() to allow createShape to build a new shape.
    cached_result: ShapeResult = .empty,

    /// Constructor, called by the classes that derive from ShapeSettings with `vtableFor(T)`
    pub fn init(vtable: *const VTable, allocator: Allocator) ShapeSettings {
        return .{ .vtable = vtable, .allocator = allocator };
    }

    /// The vtable of concrete settings class `T`
    pub fn vtableFor(comptime T: type) *const VTable {
        return &struct {
            const vt = virtual.make(VTable, T);
        }.vt;
    }

    // RefTarget<ShapeSettings>
    pub fn addRef(self: *const ShapeSettings) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const ShapeSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self));
    }

    /// Mark these settings as embedded (stack / member object), release() never frees them
    pub fn setEmbedded(self: *const ShapeSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Destructor of settings that are not on the heap: runs the destructors, asserts that no references are left
    pub fn deinit(self: *ShapeSettings) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// ~ShapeSettings
    pub fn destruct(self: *ShapeSettings) void {
        self.cached_result.deinit();
    }

    /// Create a shape according to the settings specified by this object. The result is cached; the returned copy is
    /// owned by the caller (`defer result.deinit()`). Allocation failure is returned, not cached.
    pub fn createShape(self: *ShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return self.vtable.createShape(self, allocator);
    }

    /// When creating a shape, the result is cached so that calling createShape again will return the same shape.
    /// If you make changes to the ShapeSettings you need to call this function to clear the cached result to allow createShape to build a new shape.
    pub fn clearCachedResult(self: *ShapeSettings) void {
        self.cached_result.clear();
    }

    /// The body of every `XShapeSettings::Create()`:
    /// `if (mCachedResult.IsEmpty()) Ref<Shape> shape = new T(*this, mCachedResult); return mCachedResult;`
    /// `T.initDefault(allocator)` is T's default constructor, `T.initFromSettings(shape, settings, &cached_result,
    /// allocator)` the C++ constructor that takes the settings. The local Ref reproduces Jolt's reference counting: when
    /// the constructor reports an error the shape is destroyed, otherwise the cache keeps it alive.
    pub fn createCached(comptime T: type, settings: anytype, allocator: Allocator) Allocator.Error!ShapeResult {
        const base: *ShapeSettings = virtual.upcast(ShapeSettings, settings);
        if (base.cached_result.isEmpty())
            try constructShape(T, settings, allocator);
        return base.cached_result.clone();
    }

    /// `Ref<Shape> shape = new T(*this, mCachedResult);` (for settings with custom Create logic, e.g. StaticCompoundShapeSettings)
    pub fn constructShape(comptime T: type, settings: anytype, allocator: Allocator) Allocator.Error!void {
        const base: *ShapeSettings = virtual.upcast(ShapeSettings, settings);
        errdefer base.cached_result.clear(); // Out of memory is not cached, a later call can succeed

        const shape = try allocator.create(T);
        shape.* = .initDefault(allocator);
        var ref = Ref(Shape).init(virtual.upcast(Shape, shape));
        defer ref.deinit();
        try shape.initFromSettings(settings, &base.cached_result, allocator);
    }
};

/// Function table for functions on shapes (ShapeFunctions::sRegistry lives in the comptime registry, see RegisterTypes.zig)
pub const ShapeFunctions = struct {
    /// Construct a shape (default constructor, used by restoreFromBinaryState). Null: the shape cannot be restored.
    construct: ?*const fn (allocator: Allocator) Allocator.Error!*Shape = null,
    /// Color of the shape when drawing
    color: Color = Color.black,

    /// Get an entry in the registry for a particular sub type (sGet, read-only)
    pub fn get(sub_type: ShapeSubType) *const ShapeFunctions {
        return &RegisterTypes.registry.shape_functions[@intFromEnum(sub_type)];
    }

    /// `[]() -> Shape * { return new T; }` for the registry
    pub fn constructor(comptime T: type) *const fn (allocator: Allocator) Allocator.Error!*Shape {
        return &struct {
            fn f(allocator: Allocator) Allocator.Error!*Shape {
                const shape = try allocator.create(T);
                shape.* = .initDefault(allocator);
                return virtual.upcast(Shape, shape);
            }
        }.f;
    }
};

/// Base class for all shapes (collision volume of a body). Defines a virtual interface for collision detection.
pub const Shape = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Destructor chain (generated)
        deinit: *const fn (self: *Shape) void,
        /// delete this (generated)
        destroy: *const fn (self: *Shape) void,
        mustBeStatic: *const fn (self: *const Shape) bool,
        getCenterOfMass: *const fn (self: *const Shape) Vec3,
        getLocalBounds: *const fn (self: *const Shape) AABox,
        getSubShapeIDBitsRecursive: *const fn (self: *const Shape) u32,
        getWorldSpaceBounds: *const fn (self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3) AABox,
        getInnerRadius: *const fn (self: *const Shape) f32,
        getMassProperties: *const fn (self: *const Shape) MassProperties,
        getLeafShape: *const fn (self: *const Shape, sub_shape_id: SubShapeID) LeafShape,
        getMaterial: *const fn (self: *const Shape, sub_shape_id: SubShapeID) *const PhysicsMaterial,
        getSurfaceNormal: *const fn (self: *const Shape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3,
        getSupportingFace: *const fn (self: *const Shape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *SupportingFace) void,
        getSubShapeUserData: *const fn (self: *const Shape, sub_shape_id: SubShapeID) u64,
        castRay: *const fn (self: *const Shape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool,
        castRayCollector: *const fn (self: *const Shape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void,
        collidePoint: *const fn (self: *const Shape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void,
        collectTransformedShapes: *const fn (self: *const Shape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void,
        transformShape: *const fn (self: *const Shape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void,
        getTrianglesStart: *const fn (self: *const Shape, context: *GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void,
        getTrianglesNext: *const fn (self: *const Shape, context: *GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32,
        saveBinaryState: *const fn (self: *const Shape, stream: StreamOut) void,
        saveMaterialState: *const fn (self: *const Shape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void,
        restoreMaterialState: *const fn (self: *Shape, materials: []const PhysicsMaterialRefC) void,
        saveSubShapeState: *const fn (self: *const Shape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void,
        restoreSubShapeState: *const fn (self: *Shape, sub_shapes: []const ShapeRefC) void,
        getStats: *const fn (self: *const Shape) Stats,
        getVolume: *const fn (self: *const Shape) f32,
        isValidScale: *const fn (self: *const Shape, scale: Vec3) bool,
        makeScaleValid: *const fn (self: *const Shape, scale: Vec3) Vec3,
        /// Protected in Jolt: used by restoreFromBinaryState. Restores of shapes with arrays allocate (StreamIn.readArray).
        restoreBinaryState: *const fn (self: *Shape, stream: StreamIn) Allocator.Error!void,
    };

    /// Type definition for a supporting face
    pub const SupportingFace = StaticArray(Vec3, 32);

    /// An opaque buffer that holds shape specific information during getTrianglesStart/Next (placement new, D10)
    pub const GetTrianglesContext = PlacementBuffer(4288, 16, .{ .type_check = true });

    /// This is the minimum amount of triangles that should be requested through getTrianglesNext.
    pub const get_triangles_min_triangles_requested: u32 = 32;

    /// Result of getLeafShape (the C++ returns the shape and writes outRemainder)
    pub const LeafShape = struct { shape: ?*const Shape, remainder: SubShapeID };

    /// Class that holds information about the shape that can be used for logging / data collection purposes
    pub const Stats = struct {
        /// Amount of memory used by this shape (size in bytes)
        size_bytes: usize,
        /// Number of triangles in this shape (when applicable)
        num_triangles: u32,
    };

    vtable: *const VTable,
    ref_count: RefCount = .{},
    /// Frees this shape (heap shapes) and its internal arrays
    allocator: Allocator,
    user_data: u64 = 0,
    shape_type: ShapeType,
    shape_sub_type: ShapeSubType,

    /// Constructor (Shape(EShapeType, EShapeSubType)), called by the classes that derive from Shape directly
    pub fn init(vtable: *const VTable, allocator: Allocator, shape_type: ShapeType, shape_sub_type: ShapeSubType) Shape {
        return .{ .vtable = vtable, .allocator = allocator, .shape_type = shape_type, .shape_sub_type = shape_sub_type };
    }

    /// The part of Shape(EShapeType, EShapeSubType, const ShapeSettings &, ShapeResult &) that reads the settings
    pub fn initFromSettings(self: *Shape, settings: *const ShapeSettings) void {
        self.user_data = settings.user_data;
    }

    /// The vtable of concrete class `T` (for the classes that derive from Shape directly)
    pub fn vtableFor(comptime T: type) *const VTable {
        return &struct {
            const vt = virtual.make(VTable, T);
        }.vt;
    }

    // RefTarget<Shape>
    pub fn addRef(self: *const Shape) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const Shape) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this shape as embedded (stack / member object): release() never frees it
    pub fn setEmbedded(self: *const Shape) void {
        self.ref_count.setEmbedded();
    }

    /// Destructor of a shape that is not on the heap (embedded): runs the destructor chain, asserts that no references are left
    pub fn deinit(self: *Shape) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// `delete` of a heap shape that was never referenced (e.g. on an error path after `create`)
    pub fn destroy(self: *Shape) void {
        self.vtable.destroy(self);
    }

    /// Checked downcast (static_cast<const T *>): asserts the dynamic type in safe builds
    pub fn cast(self: *const Shape, comptime T: type) *const T {
        std.debug.assert(self.isKindOf(T));
        return virtual.downcast(T, self);
    }

    /// Checked downcast of a mutable shape
    pub fn castMut(self: *Shape, comptime T: type) *T {
        std.debug.assert(self.isKindOf(T));
        return virtual.downcast(T, self);
    }

    /// True if this shape is a `T`: concrete classes declare `shape_sub_type`, abstract classes `shape_type`
    pub fn isKindOf(self: *const Shape, comptime T: type) bool {
        if (@hasDecl(T, "shape_sub_type")) return self.shape_sub_type == T.shape_sub_type;
        if (@hasDecl(T, "shape_type")) return self.shape_type == T.shape_type;
        return T == Shape;
    }

    /// Get type
    pub fn getType(self: *const Shape) ShapeType {
        return self.shape_type;
    }

    pub fn getSubType(self: *const Shape) ShapeSubType {
        return self.shape_sub_type;
    }

    /// User data (to be used freely by the application)
    pub fn getUserData(self: *const Shape) u64 {
        return self.user_data;
    }

    pub fn setUserData(self: *Shape, user_data: u64) void {
        self.user_data = user_data;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Virtual dispatchers (one per VTable entry, Jolt's doc comments)

    /// Check if this shape can only be used to create a static body or if it can also be dynamic/kinematic
    pub fn mustBeStatic(self: *const Shape) bool {
        return self.vtable.mustBeStatic(self);
    }

    /// All shapes are centered around their center of mass. This function returns the center of mass position that needs to be applied to transform the shape to where it was created.
    pub fn getCenterOfMass(self: *const Shape) Vec3 {
        return self.vtable.getCenterOfMass(self);
    }

    /// Get local bounding box including convex radius, this box is centered around the center of mass rather than the world transform
    pub fn getLocalBounds(self: *const Shape) AABox {
        return self.vtable.getLocalBounds(self);
    }

    /// Get the max number of sub shape ID bits that are needed to be able to address any leaf shape in this shape. Used mainly for checking that it is smaller or equal than SubShapeID::MaxBits.
    pub fn getSubShapeIDBitsRecursive(self: *const Shape) u32 {
        return self.vtable.getSubShapeIDBitsRecursive(self);
    }

    /// Get world space bounds including convex radius.
    pub fn getWorldSpaceBounds(self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        return self.vtable.getWorldSpaceBounds(self, center_of_mass_transform, scale);
    }

    /// Returns the radius of the biggest sphere that fits entirely in the shape.
    pub fn getInnerRadius(self: *const Shape) f32 {
        return self.vtable.getInnerRadius(self);
    }

    /// Calculate the mass and inertia of this shape
    pub fn getMassProperties(self: *const Shape) MassProperties {
        return self.vtable.getMassProperties(self);
    }

    /// Get the leaf shape for a particular sub shape ID.
    pub fn getLeafShape(self: *const Shape, sub_shape_id: SubShapeID) LeafShape {
        return self.vtable.getLeafShape(self, sub_shape_id);
    }

    /// Get the material assigned to a particular sub shape ID
    pub fn getMaterial(self: *const Shape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        return self.vtable.getMaterial(self, sub_shape_id);
    }

    /// Get the surface normal of a particular sub shape ID and point on surface (all vectors are relative to center of mass for this shape).
    pub fn getSurfaceNormal(self: *const Shape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        return self.vtable.getSurfaceNormal(self, sub_shape_id, local_surface_position);
    }

    /// Get the vertices of the face that faces direction the most (includes any convex radius). Note that this function can only return faces of
    /// convex shapes or triangles, which is why a sub shape ID to get to that leaf must be provided.
    pub fn getSupportingFace(self: *const Shape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *SupportingFace) void {
        self.vtable.getSupportingFace(self, sub_shape_id, direction, scale, center_of_mass_transform, out_vertices);
    }

    /// Get the user data of a particular sub shape ID. Corresponds with the value stored in Shape::GetUserData of the leaf shape pointed to by inSubShapeID.
    pub fn getSubShapeUserData(self: *const Shape, sub_shape_id: SubShapeID) u64 {
        return self.vtable.getSubShapeUserData(self, sub_shape_id);
    }

    /// Cast a ray against this shape, returns true if it finds a hit closer than hit.fraction and updates that fraction. Otherwise hit is left untouched and the function returns false.
    pub fn castRay(self: *const Shape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        return self.vtable.castRay(self, ray, sub_shape_id_creator, hit);
    }

    /// Cast a ray against this shape. Allows returning multiple hits through collector. Note that this version is more flexible but also slightly slower than the castRay function that returns only a single hit.
    pub fn castRayCollector(self: *const Shape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        self.vtable.castRayCollector(self, ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    /// Check if point is inside this shape. For this tests all shapes are treated as if they were solid.
    pub fn collidePoint(self: *const Shape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        self.vtable.collidePoint(self, point, sub_shape_id_creator, collector, shape_filter);
    }

    /// Collect the leaf transformed shapes of all leaf shapes of this shape. box is specified in the local space of this shape.
    pub fn collectTransformedShapes(self: *const Shape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        self.vtable.collectTransformedShapes(self, box, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
    }

    /// Transforms this shape and all of its children with transform, resulting shape(s) are passed to collector.
    pub fn transformShape(self: *const Shape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        self.vtable.transformShape(self, center_of_mass_transform, collector);
    }

    /// To start iterating over triangles, call this function first. context is a temporary buffer and should remain untouched until the last call to getTrianglesNext.
    pub fn getTrianglesStart(self: *const Shape, context: *GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        self.vtable.getTrianglesStart(self, context, box, position_com, rotation, scale);
    }

    /// Call this repeatedly to get all triangles in the box. out_triangle_vertices must hold 3 * max_triangles_requested
    /// vertices, out_materials (optional) max_triangles_requested materials (slices instead of Jolt's raw pointers).
    pub fn getTrianglesNext(self: *const Shape, context: *GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        std.debug.assert(out_triangle_vertices.len >= 3 * max_triangles_requested);
        if (out_materials) |m| std.debug.assert(m.len >= max_triangles_requested);
        return self.vtable.getTrianglesNext(self, context, max_triangles_requested, out_triangle_vertices, out_materials);
    }

    /// Saves the contents of the shape in binary form to stream.
    pub fn saveBinaryState(self: *const Shape, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// Outputs the material references that this shape has to out_materials.
    pub fn saveMaterialState(self: *const Shape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        return self.vtable.saveMaterialState(self, allocator, out_materials);
    }

    /// Restore the material references after calling restoreFromBinaryState. Note that the exact same materials need to be provided in the same order as returned by saveMaterialState.
    pub fn restoreMaterialState(self: *Shape, materials: []const PhysicsMaterialRefC) void {
        self.vtable.restoreMaterialState(self, materials);
    }

    /// Outputs the shape references that this shape has to out_sub_shapes.
    pub fn saveSubShapeState(self: *const Shape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
        return self.vtable.saveSubShapeState(self, allocator, out_sub_shapes);
    }

    /// Restore the shape references after calling restoreFromBinaryState. Note that the exact same shapes need to be provided in the same order as returned by saveSubShapeState.
    pub fn restoreSubShapeState(self: *Shape, sub_shapes: []const ShapeRefC) void {
        self.vtable.restoreSubShapeState(self, sub_shapes);
    }

    /// Get stats of this shape. Use for logging / data collection purposes only. Does not add values from child shapes, use GetStatsRecursive for this.
    pub fn getStats(self: *const Shape) Stats {
        return self.vtable.getStats(self);
    }

    /// Volume of this shape (m^3). Note that for compound shapes the volume may be incorrect since child shapes can overlap which is not accounted for.
    pub fn getVolume(self: *const Shape) f32 {
        return self.vtable.getVolume(self);
    }

    /// Test if inScale is a valid scale for this shape. Some shapes can only be scaled uniformly, compound shapes cannot handle shapes
    /// being rotated and scaled (this would cause shearing), scale can never be zero.
    pub fn isValidScale(self: *const Shape, scale: Vec3) bool {
        return self.vtable.isValidScale(self, scale);
    }

    /// This function will make sure that if you wrap this shape in a ScaledShape that the scale is valid.
    pub fn makeScaleValid(self: *const Shape, scale: Vec3) Vec3 {
        return self.vtable.makeScaleValid(self, scale);
    }

    /// This function should not be called directly, it is used by restoreFromBinaryState.
    fn restoreBinaryState(self: *Shape, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Creates a Shape of the correct type and restores its contents from the binary stream.
    /// The sub type is validated (Jolt indexes the table without checking), the shape is constructed with the
    /// ShapeFunctions of the comptime registry.
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!ShapeResult {
        var result: ShapeResult = .empty;

        // Read the type of the shape
        var raw_sub_type: u8 = 0;
        stream.read(&raw_sub_type);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read type id");
            return result;
        }
        const construct = if (std.enums.fromInt(ShapeSubType, raw_sub_type)) |sub_type| ShapeFunctions.get(sub_type).construct else null;
        if (construct == null) {
            result.setError("Failed to read type id");
            return result;
        }

        // Construct and read the data of the shape
        var shape = Ref(Shape).init(try construct.?(allocator));
        defer shape.deinit();
        try shape.get().?.restoreBinaryState(stream);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to restore shape");
            return result;
        }

        result.set(shape.clone());
        return result;
    }

    /// Save this shape, all its children and its materials. Pass in an empty map in shape_map / material_map or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
    /// (Prototype: materials are written with the RTTI hash and PhysicsMaterial.saveBinaryState, StreamUtils::SaveObjectArray)
    pub fn saveWithChildren(self: *const Shape, allocator: Allocator, stream: StreamOut, shape_map: *ShapeToIDMap, material_map: *MaterialToIDMap) Allocator.Error!void {
        if (shape_map.find(self)) |kv| {
            // Known shape, just write the ID
            stream.write(kv.value);
            return;
        }

        // Write shape ID of this shape
        const shape_id: u32 = @intCast(shape_map.count());
        _ = try shape_map.tryEmplace(allocator, self, shape_id);
        stream.write(shape_id);

        // Write the shape itself
        self.saveBinaryState(stream);

        // Write the ID's of all sub shapes
        var sub_shapes: ShapeList = .empty;
        defer {
            for (sub_shapes.items) |*s| s.deinit();
            sub_shapes.deinit(allocator);
        }
        try self.saveSubShapeState(allocator, &sub_shapes);
        stream.write(@as(u32, @intCast(sub_shapes.items.len)));
        for (sub_shapes.items) |sub_shape| {
            if (sub_shape.get()) |s|
                try s.saveWithChildren(allocator, stream, shape_map, material_map)
            else
                stream.write(~@as(u32, 0));
        }

        // Write the materials (StreamUtils::SaveObjectArray)
        var materials: PhysicsMaterialList = .empty;
        defer {
            for (materials.items) |*m| m.deinit();
            materials.deinit(allocator);
        }
        try self.saveMaterialState(allocator, &materials);
        stream.write(@as(u32, @intCast(materials.items.len)));
        for (materials.items) |material| {
            const m = material.get() orelse {
                stream.write(~@as(u32, 0));
                continue;
            };
            if (material_map.find(m)) |kv| {
                stream.write(kv.value);
            } else {
                const material_id: u32 = @intCast(material_map.count());
                _ = try material_map.tryEmplace(allocator, m, material_id);
                stream.write(material_id);
                m.saveBinaryState(stream);
            }
        }
    }

    /// Restore a shape, all its children and materials. Pass in an empty map in shape_map / material_map or reuse the same map while reading multiple shapes from the same stream in order to restore duplicates.
    pub fn restoreWithChildren(allocator: Allocator, stream: StreamIn, shape_map: *IDToShapeMap, material_map: *IDToMaterialMap) Allocator.Error!ShapeResult {
        var result: ShapeResult = .empty;
        errdefer result.deinit();

        // Read ID of this shape
        var shape_id: u32 = 0;
        stream.read(&shape_id);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read shape id");
            return result;
        }

        // Check nullptr shape
        if (shape_id == ~@as(u32, 0)) {
            result.set(.empty);
            return result;
        }

        // Check if we already read this shape
        if (shape_id < shape_map.items.len) {
            result.set(shape_map.items[shape_id].clone());
            return result;
        }

        // Read the shape
        result.deinit();
        result = try restoreFromBinaryState(allocator, stream);
        if (result.hasError())
            return result;
        std.debug.assert(shape_map.items.len == shape_id); // Assert that this is the next ID in the map
        try shape_map.append(allocator, result.get().clone());

        // Read the sub shapes
        var len: u32 = 0;
        stream.read(&len);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read stream");
            return result;
        }
        var sub_shapes: ShapeList = .empty;
        defer {
            for (sub_shapes.items) |*s| s.deinit();
            sub_shapes.deinit(allocator);
        }
        try sub_shapes.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            var sub_shape_result = try restoreWithChildren(allocator, stream, shape_map, material_map);
            defer sub_shape_result.deinit();
            if (sub_shape_result.hasError()) {
                result.assign(&sub_shape_result);
                return result;
            }
            sub_shapes.appendAssumeCapacity(.init(sub_shape_result.getPtr()));
        }
        result.getPtr().?.restoreSubShapeState(sub_shapes.items);

        // Read the materials (StreamUtils::RestoreObjectArray)
        stream.read(&len);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read stream");
            return result;
        }
        var materials: PhysicsMaterialList = .empty;
        defer {
            for (materials.items) |*m| m.deinit();
            materials.deinit(allocator);
        }
        try materials.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            // StreamUtils::RestoreObjectReference
            var material_id: u32 = ~@as(u32, 0);
            stream.read(&material_id);
            if (material_id == ~@as(u32, 0)) {
                materials.appendAssumeCapacity(.empty);
            } else if (material_id < material_map.items.len) {
                materials.appendAssumeCapacity(.init(material_map.items[material_id].get()));
            } else {
                var material_result = try PhysicsMaterial.restoreFromBinaryState(allocator, stream);
                defer material_result.deinit();
                if (material_result.hasError()) {
                    result.setError(material_result.getError());
                    return result;
                }
                std.debug.assert(material_map.items.len == material_id);
                try material_map.append(allocator, material_result.get().clone());
                materials.appendAssumeCapacity(.init(material_result.getPtr()));
            }
        }
        result.getPtr().?.restoreMaterialState(materials.items);

        return result;
    }

    /// Scale this shape. Note that not all shapes support all scales, this will create a new shape that best mimics the scaled shape.
    /// Returns a ShapeResult (Jolt's errors) or error.OutOfMemory.
    pub fn scaleShape(self: *const Shape, allocator: Allocator, scale: Vec3) Allocator.Error!ShapeResult {
        const unit_scale = Vec3.one();

        if (scale.isNearZero(.{})) {
            var result: ShapeResult = .empty;
            result.setError("Can't use zero scale!");
            return result;
        }

        // First test if we can just wrap this shape in a scaled shape
        if (self.isValidScale(scale)) {
            // Test if the scale is near unit
            var result: ShapeResult = .empty;
            if (scale.isClose(unit_scale, .{}))
                result.set(.init(@constCast(self))) // const_cast<Shape *>(this): Ref<Shape> of a shared shape, the shape is never written through it
            else
                result.set(.init(virtual.upcast(Shape, try ScaledShape.create(allocator, self, scale))));
            return result;
        }

        // Collect the leaf shapes and their transforms
        const Collector = struct {
            pub const overrides = .{.addHit};

            base: TransformedShapeCollector,
            allocator: Allocator,
            shapes: std.ArrayList(TransformedShape) = .empty,
            alloc_error: @import("../CollisionCollectorImpl.zig").AllocationErrorLatch = .{},

            pub fn addHit(c: *@This(), result: *const TransformedShape) void {
                var ts = result.clone();
                c.shapes.append(c.allocator, ts) catch |err| {
                    ts.deinit();
                    c.alloc_error.set(err);
                    c.base.forceEarlyOut();
                };
            }
        };
        var collector: Collector = .{ .base = .init(Collector), .allocator = allocator };
        defer {
            for (collector.shapes.items) |*ts| ts.deinit();
            collector.shapes.deinit(allocator);
        }
        self.transformShape(Mat44.scaleVec3(scale).mul(Mat44.translation(self.getCenterOfMass())), &collector.base);
        try collector.alloc_error.check();

        // Construct a compound shape
        var compound: StaticCompoundShapeSettings = .init(allocator);
        defer compound.asShapeSettings().deinit();
        try compound.base.sub_shapes.ensureTotalCapacity(allocator, collector.shapes.items.len);
        for (collector.shapes.items) |*ts| {
            var shape: *const Shape = ts.shape.get().?;

            // Construct a scaled shape if scale is not unit
            const shape_scale = ts.getShapeScale();
            if (!shape_scale.isClose(unit_scale, .{}))
                shape = virtual.upcast(Shape, try ScaledShape.create(allocator, shape, shape_scale));

            // Add the shape
            try compound.base.addShapePtr(TransformedShapeFile.vec3FromRVec3(ts.shape_position_com).sub(ts.shape_rotation.mulVec3(shape.getCenterOfMass())), ts.shape_rotation, shape, .{});
        }

        return compound.asShapeSettings().createShape(allocator);
    }

    /// Default implementations of the virtual functions (C++ Shape::Foo bodies). C++'s explicit base calls
    /// (`Shape::SaveBinaryState(inStream)`) are `Shape.impl.saveBinaryState(self.asShape(), stream)`.
    pub const impl = struct {
        pub fn mustBeStatic(self: *const Shape) bool {
            _ = self;
            return false;
        }

        pub fn getCenterOfMass(self: *const Shape) Vec3 {
            _ = self;
            return Vec3.zero();
        }

        pub fn getWorldSpaceBounds(self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
            return self.getLocalBounds().scaled(scale).transformed(center_of_mass_transform);
        }

        pub fn getLeafShape(self: *const Shape, sub_shape_id: SubShapeID) LeafShape {
            return .{ .shape = self, .remainder = sub_shape_id };
        }

        pub fn getSupportingFace(self: *const Shape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *SupportingFace) void {
            // Nothing
            _ = .{ self, sub_shape_id, direction, scale, center_of_mass_transform, out_vertices };
        }

        pub fn getSubShapeUserData(self: *const Shape, sub_shape_id: SubShapeID) u64 {
            _ = sub_shape_id;
            return self.user_data;
        }

        pub fn collectTransformedShapes(self: *const Shape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
            _ = box;

            // Test shape filter
            if (!shape_filter.shouldCollide(self, sub_shape_id_creator.getID()))
                return;

            var ts = TransformedShape.init(TransformedShapeFile.rvec3FromVec3(position_com), rotation, self, TransformedShape.getBodyID(collector.getContext()), sub_shape_id_creator);
            defer ts.deinit();
            ts.setShapeScale(scale);
            collector.addHit(&ts);
        }

        pub fn transformShape(self: *const Shape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
            const decomposed = center_of_mass_transform.decompose();
            const transform = decomposed.rotation_translation;
            var ts = TransformedShape.init(TransformedShapeFile.rvec3FromVec3(transform.getTranslation()), transform.getQuaternion(), self, .invalid, .{});
            defer ts.deinit();
            ts.setShapeScale(self.makeScaleValid(decomposed.scale));
            collector.addHit(&ts);
        }

        pub fn saveBinaryState(self: *const Shape, stream: StreamOut) void {
            stream.write(self.shape_sub_type);
            stream.write(self.user_data);
        }

        pub fn restoreBinaryState(self: *Shape, stream: StreamIn) Allocator.Error!void {
            // Type hash read by restoreFromBinaryState
            stream.read(&self.user_data);
        }

        pub fn saveMaterialState(self: *const Shape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
            // By default do nothing
            _ = .{ self, allocator, out_materials };
        }

        pub fn restoreMaterialState(self: *Shape, materials: []const PhysicsMaterialRefC) void {
            _ = self;
            std.debug.assert(materials.len == 0);
        }

        pub fn saveSubShapeState(self: *const Shape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
            // By default do nothing
            _ = .{ self, allocator, out_sub_shapes };
        }

        pub fn restoreSubShapeState(self: *Shape, sub_shapes: []const ShapeRefC) void {
            _ = self;
            std.debug.assert(sub_shapes.len == 0);
        }

        pub fn isValidScale(self: *const Shape, scale: Vec3) bool {
            _ = self;
            return !ScaleHelpers.isZeroScale(scale);
        }

        pub fn makeScaleValid(self: *const Shape, scale: Vec3) Vec3 {
            _ = self;
            return ScaleHelpers.makeNonZeroScale(scale);
        }
    };
};
