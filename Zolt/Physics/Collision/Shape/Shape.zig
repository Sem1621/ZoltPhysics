//! Port of: Jolt/Physics/Collision/Shape/Shape.h, Jolt/Physics/Collision/Shape/Shape.cpp
//! Status: partial
//! Missing: scaleShape (needs ScaledShape / StaticCompoundShape, Wave A/B)
//!
//! Architecture (Docs/Zolt/CollisionArchitecture.md):
//! - D1 `Shape` is the root of pattern A: `vtable`, the atomic `ref_count`, the `allocator` that frees the shape and its
//!   internal arrays, `user_data`, `shape_type` and `shape_sub_type`. One `Shape.VTable` entry per C++ virtual function
//!   (declaration order) plus the generated `deinit` / `destroy`, one dispatcher with the same name per entry, the
//!   default bodies in `Shape.impl` (`Shape::Foo()` qualified calls of derived classes are `Shape.impl.foo(...)`).
//!   Vtables are built by `virtual.make` (Core/Virtual.zig) through `Shape.vtableFor(T)`.
//! - Shapes are immutable after creation and shared between threads: every query takes `*const Shape`. Only the
//!   creation / restore paths take `*Shape` (setUserData, restore*State, called before the shape is shared).
//! - D2 heap shapes: `T.create(allocator, ...)` (refcount 0, like `new`), kept alive by `Ref(Shape)` / `RefConst(Shape)`,
//!   `release()` destroys them through the generated `destroy` entry. Stack / member shapes: `T.init(allocator, ...)`,
//!   `asShape().setEmbedded()` before handing out references, `asShapeMut().deinit()` at the end of their scope.
//! - D3 `ShapeSettings.createShape(allocator)` returns `Allocator.Error!ShapeResult` (ShapeResult = Result(Ref(Shape))
//!   with Jolt's error texts); the shape constructors that take settings are `initFromSettings(self, settings, result,
//!   allocator)`, run in place by `ShapeSettings.createCached` / `constructShape`.
//! - D4 `ShapeFunctions::sRegistry` is part of the comptime registry (RegisterTypes.zig), `ShapeFunctions.get` is
//!   read-only.
//! - D11 binary state: `saveBinaryState(self: *const, stream)`, `restoreBinaryState(self: *, stream)
//!   Allocator.Error!void`; the material and sub shape state functions take an allocator on the save side.
//!
//! Signatures that differ from C++ (out parameters, overloads, allocation):
//! - `GetLeafShape(id, outRemainder)` -> `getLeafShape(id) LeafShape{ shape, remainder }`,
//!   `GetSubShapeTransformedShape(..., outRemainder)` -> `SubShapeTransformedShape{ transformed_shape, remainder }`
//!   (the caller owns the TransformedShape: `deinit` it), `GetSubmergedVolume(..., outTotalVolume,
//!   outSubmergedVolume, outCenterOfBuoyancy)` -> `SubmergedVolume{ total_volume, submerged_volume,
//!   center_of_buoyancy }`.
//! - `CastRay` overloads: `castRay` (single hit) / `castRayCollector`. The non virtual
//!   `GetWorldSpaceBounds(DMat44Arg, Vec3Arg)` is `getWorldSpaceBoundsDMat44` (`getWorldSpaceBoundsRMat44` picks the
//!   overload that C++ picks for an RMat44).
//! - `GetTrianglesNext` takes slices: `out_triangle_vertices: []Float3` (at least 3 * max) and
//!   `out_materials: ?[]*const PhysicsMaterial` (D10). Counts are u32.
//! - `GetStatsRecursive(VisitedShapes &)` inserts into a hash set, so it takes an allocator and returns
//!   `Allocator.Error!Stats`. `SaveMaterialState` / `SaveSubShapeState` append to lists: allocator + error union.
//! - `sRestoreFromBinaryState` validates the sub shape type read from the stream (Jolt indexes the table with it and
//!   calls a null `mConstruct`): an invalid value or a type without constructor is "Failed to read type id".
//! - JPH_DEBUG_RENDERER (Draw, DrawGetSupportFunction, DrawGetSupportingFace, sDrawSubmergedVolumes and the
//!   inBaseOffset parameter of GetSubmergedVolume) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Color = @import("../../../Core/Color.zig").Color;
const PlacementBuffer = @import("../../../Core/PlacementBuffer.zig").PlacementBuffer;
const ReferenceFile = @import("../../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const RefCount = ReferenceFile.RefCount;
const Result = @import("../../../Core/Result.zig").Result;
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const ObjectToIDMapFile = @import("../../../Core/ObjectToIDMap.zig");
const ObjectToIDMap = ObjectToIDMapFile.ObjectToIDMap;
const IDToObjectMap = ObjectToIDMapFile.IDToObjectMap;
const UnorderedSet = @import("../../../Core/UnorderedSet.zig").UnorderedSet;
const virtual = @import("../../../Core/Virtual.zig");
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const DMat44 = @import("../../../Math/DMat44.zig").DMat44;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const ScaleHelpers = @import("ScaleHelpers.zig");
const PhysicsMaterialFile = @import("../PhysicsMaterial.zig");
const PhysicsMaterial = PhysicsMaterialFile.PhysicsMaterial;
const CollisionCollectorFile = @import("../CollisionCollector.zig");
const CollisionCollector = CollisionCollectorFile.CollisionCollector;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollidePointResult = @import("../CollidePointResult.zig").CollidePointResult;
const CollideShapeResult = @import("../CollideShape.zig").CollideShapeResult;
const ShapeCastResult = @import("../ShapeCast.zig").ShapeCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;

pub const CastRayCollector = CollisionCollector(RayCastResult, CollisionCollectorFile.CollisionCollectorTraitsCastRay);
pub const CastShapeCollector = CollisionCollector(ShapeCastResult, CollisionCollectorFile.CollisionCollectorTraitsCastShape);
pub const CollidePointCollector = CollisionCollector(CollidePointResult, CollisionCollectorFile.CollisionCollectorTraitsCollidePoint);
pub const CollideShapeCollector = CollisionCollector(CollideShapeResult, CollisionCollectorFile.CollisionCollectorTraitsCollideShape);
pub const TransformedShapeCollector = CollisionCollector(TransformedShape, CollisionCollectorFile.CollisionCollectorTraitsCollideShape);

pub const ShapeRefC = RefConst(Shape);
pub const ShapeList = std.ArrayList(ShapeRefC);
pub const PhysicsMaterialRefC = RefConst(PhysicsMaterial);
pub const PhysicsMaterialList = PhysicsMaterialFile.PhysicsMaterialList;

/// Shapes are categorized in groups, each shape can return which group it belongs to through its Shape::GetType function.
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

/// This enumerates all shape types, each shape can return its type through Shape::GetSubType
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
    std.debug.assert(@typeInfo(ShapeSubType).@"enum".fields.len == num_sub_shape_types);
    for (all_sub_shape_types, 0..) |s, i| std.debug.assert(@intFromEnum(s) == i);
}

/// Result<Ref<Shape>> (ShapeSettings::ShapeResult and Shape::ShapeResult)
pub const ShapeResult = Result(Ref(Shape));

/// Class that can construct shapes and that is serializable using the ObjectStream system.
/// Can be used to store shape data in 'uncooked' form (i.e. in a form that is still human readable and authorable).
/// Once the shape has been created using the Create() function, the data will be moved into the Shape class
/// in a form that is optimized for collision detection. After this, the ShapeSettings object is no longer needed
/// and can be destroyed. Each shape class has a derived class of the ShapeSettings object to store shape specific
/// data.
///
/// Pattern A RefTarget root (D3). Settings on the stack: `X.init(allocator, ...)` + `defer s.deinit()`; on the heap:
/// `X.create(allocator, ...)` (reference count 0, put it in a Ref, the last release() frees it).
pub const ShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT(JPH_EXPORT, ShapeSettings)

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Destructor chain (generated)
        deinit: *const fn (self: *ShapeSettings) void,
        /// delete this (generated)
        destroy: *const fn (self: *ShapeSettings) void,
        /// Create a shape according to the settings specified by this object (Create() const in C++). Mutable
        /// receiver: it writes the cache (C++ `mutable`, Rule M).
        createShape: *const fn (self: *ShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<ShapeSettings>)
    ref_count: RefCount = .{},
    /// Frees heap settings and the memory the settings own (e.g. the sub shape list of a compound)
    allocator: Allocator,

    /// User data (to be used freely by the application)
    user_data: u64 = 0,

    /// When creating a shape, the result is cached so that calling Create() again will return the same shape (C++ `mutable mCachedResult`)
    cached_result: ShapeResult = .empty,

    /// Constructor, called by the classes that derive from ShapeSettings with `vtableFor(T)` of the most derived class
    pub fn init(vtable: *const VTable, allocator: Allocator) ShapeSettings {
        return .{ .vtable = vtable, .allocator = allocator };
    }

    /// The vtable of concrete settings class `T`
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<ShapeSettings>

    /// Add a reference to this object
    pub fn addRef(self: *const ShapeSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const ShapeSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this object as embedded (stack / member object): release() never frees it
    pub fn setEmbedded(self: *const ShapeSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const ShapeSettings) u32 {
        return self.ref_count.get();
    }

    /// Destructor of settings that are not on the heap: runs the destructor chain, asserts that no references are left
    pub fn deinit(self: *ShapeSettings) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// ~ShapeSettings
    pub fn destruct(self: *ShapeSettings) void {
        self.cached_result.deinit();
    }

    /// Create a shape according to the settings specified by this object.
    /// The result is cached; the returned copy is owned by the caller (`defer result.deinit()`). Allocation failure is
    /// returned, not cached.
    pub fn createShape(self: *ShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return self.vtable.createShape(self, allocator);
    }

    /// When creating a shape, the result is cached so that calling Create() again will return the same shape.
    /// If you make changes to the ShapeSettings you need to call this function to clear the cached result to allow Create() to build a new shape.
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

    /// `Ref<Shape> shape = new T(*this, mCachedResult);` (also used directly by settings with custom Create logic,
    /// e.g. StaticCompoundShapeSettings)
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

/// Function table for functions on shapes
pub const ShapeFunctions = struct {
    /// Construct a shape (`new T` with the default constructor, used by restoreFromBinaryState). Null: the shape cannot be constructed.
    construct: ?*const fn (allocator: Allocator) Allocator.Error!*Shape = null,

    /// Color of the shape when drawing
    color: Color = Color.black,

    /// Get an entry in the registry for a particular sub type (ShapeFunctions::sRegistry is part of the comptime
    /// registry, see RegisterTypes.zig; registration code writes through `Registry.shapeFunctions`)
    pub fn get(sub_type: ShapeSubType) *const ShapeFunctions {
        return &RegisterTypes.registry.shape_functions[@intFromEnum(sub_type)];
    }

    /// `[]() -> Shape * { return new T; }`: the `construct` function of shape class `T` (uses `T.initDefault(allocator)`)
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
    /// One entry per C++ virtual function, in declaration order (the dispatchers below have the full documentation)
    pub const VTable = struct {
        /// Destructor chain (generated, virtual ~Shape)
        deinit: *const fn (self: *Shape) void,
        /// delete this (generated)
        destroy: *const fn (self: *Shape) void,
        /// Check if this shape can only be used to create a static body or if it can also be dynamic/kinematic
        mustBeStatic: *const fn (self: *const Shape) bool,
        /// All shapes are centered around their center of mass. This function returns the center of mass position that needs to be applied to transform the shape to where it was created.
        getCenterOfMass: *const fn (self: *const Shape) Vec3,
        /// Get local bounding box including convex radius, this box is centered around the center of mass rather than the world transform
        getLocalBounds: *const fn (self: *const Shape) AABox,
        /// Get the max number of sub shape ID bits that are needed to be able to address any leaf shape in this shape. Used mainly for checking that it is smaller or equal than SubShapeID::MaxBits.
        getSubShapeIDBitsRecursive: *const fn (self: *const Shape) u32,
        /// Get world space bounds including convex radius.
        getWorldSpaceBounds: *const fn (self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3) AABox,
        /// Returns the radius of the biggest sphere that fits entirely in the shape. In case this shape consists of multiple sub shapes, it returns the smallest sphere of the parts.
        getInnerRadius: *const fn (self: *const Shape) f32,
        /// Calculate the mass and inertia of this shape
        getMassProperties: *const fn (self: *const Shape) MassProperties,
        /// Get the leaf shape for a particular sub shape ID.
        getLeafShape: *const fn (self: *const Shape, sub_shape_id: SubShapeID) LeafShape,
        /// Get the material assigned to a particular sub shape ID
        getMaterial: *const fn (self: *const Shape, sub_shape_id: SubShapeID) *const PhysicsMaterial,
        /// Get the surface normal of a particular sub shape ID and point on surface (all vectors are relative to center of mass for this shape).
        getSurfaceNormal: *const fn (self: *const Shape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3,
        /// Get the vertices of the face that faces direction the most (includes any convex radius).
        getSupportingFace: *const fn (self: *const Shape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *SupportingFace) void,
        /// Get the user data of a particular sub shape ID. Corresponds with the value stored in Shape::GetUserData of the leaf shape pointed to by sub_shape_id.
        getSubShapeUserData: *const fn (self: *const Shape, sub_shape_id: SubShapeID) u64,
        /// Get the direct child sub shape and its transform for a sub shape ID.
        getSubShapeTransformedShape: *const fn (self: *const Shape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) SubShapeTransformedShape,
        /// Gets the properties needed to do buoyancy calculations for a body using this shape
        getSubmergedVolume: *const fn (self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) SubmergedVolume,
        // TODO(debug_renderer): Draw, DrawGetSupportFunction, DrawGetSupportingFace (JPH_DEBUG_RENDERER)
        /// Cast a ray against this shape, returns true if it finds a hit closer than hit.fraction and updates that fraction. Otherwise hit is left untouched and the function returns false.
        castRay: *const fn (self: *const Shape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool,
        /// Cast a ray against this shape. Allows returning multiple hits through collector.
        castRayCollector: *const fn (self: *const Shape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void,
        /// Check if point is inside this shape. For this tests all shapes are treated as if they were solid.
        collidePoint: *const fn (self: *const Shape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void,
        /// Collides all vertices of a soft body with this shape and updates SoftBodyVertex::mCollisionPlane, SoftBodyVertex::mCollidingShapeIndex and SoftBodyVertex::mLargestPenetration if a collision with more penetration was found.
        collideSoftBodyVertices: *const fn (self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void,
        /// Collect the leaf transformed shapes of all leaf shapes of this shape.
        collectTransformedShapes: *const fn (self: *const Shape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void,
        /// Transforms this shape and all of its children with transform, resulting shape(s) are passed to collector.
        transformShape: *const fn (self: *const Shape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void,
        /// To start iterating over triangles, call this function first.
        getTrianglesStart: *const fn (self: *const Shape, context: *GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void,
        /// Call this repeatedly to get all triangles in the box.
        getTrianglesNext: *const fn (self: *const Shape, context: *GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32,
        /// Saves the contents of the shape in binary form to stream.
        saveBinaryState: *const fn (self: *const Shape, stream: StreamOut) void,
        /// Outputs the material references that this shape has to out_materials.
        saveMaterialState: *const fn (self: *const Shape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void,
        /// Restore the material references after calling sRestoreFromBinaryState.
        restoreMaterialState: *const fn (self: *Shape, materials: []const PhysicsMaterialRefC) void,
        /// Outputs the shape references that this shape has to out_sub_shapes.
        saveSubShapeState: *const fn (self: *const Shape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void,
        /// Restore the shape references after calling sRestoreFromBinaryState.
        restoreSubShapeState: *const fn (self: *Shape, sub_shapes: []const ShapeRefC) void,
        /// Get stats of this shape. Use for logging / data collection purposes only. Does not add values from child shapes, use GetStatsRecursive for this.
        getStats: *const fn (self: *const Shape) Stats,
        /// Get the combined stats of this shape and its children.
        getStatsRecursive: *const fn (self: *const Shape, allocator: Allocator, visited_shapes: *VisitedShapes) Allocator.Error!Stats,
        /// Volume of this shape (m^3). Note that for compound shapes the volume may be incorrect since child shapes can overlap which is not accounted for.
        getVolume: *const fn (self: *const Shape) f32,
        /// Test if scale is a valid scale for this shape.
        isValidScale: *const fn (self: *const Shape, scale: Vec3) bool,
        /// This function will make sure that if you wrap this shape in a ScaledShape that the scale is valid.
        makeScaleValid: *const fn (self: *const Shape, scale: Vec3) Vec3,
        /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
        /// Restores of shapes with arrays allocate (StreamIn.readArray).
        restoreBinaryState: *const fn (self: *Shape, stream: StreamIn) Allocator.Error!void,
    };

    /// Type definition for a supporting face
    pub const SupportingFace = StaticArray(Vec3, 32);

    /// An opaque buffer that holds shape specific information during GetTrianglesStart/Next (placement new, D10)
    pub const GetTrianglesContext = PlacementBuffer(4288, 16, .{ .type_check = true });

    /// This is the minimum amount of triangles that should be requested through GetTrianglesNext.
    pub const get_triangles_min_triangles_requested: u32 = 32;

    /// Result of getLeafShape (Jolt returns the shape and writes outRemainder)
    pub const LeafShape = struct {
        /// The shape or null if the sub shape ID is invalid
        shape: ?*const Shape,
        /// What remains of the sub shape ID after removing the path to the leaf shape (could e.g. refer to a triangle within a MeshShape)
        remainder: SubShapeID,
    };

    /// Result of getSubShapeTransformedShape (Jolt returns the TransformedShape and writes outRemainder)
    pub const SubShapeTransformedShape = struct {
        /// Direct child sub shape and its transform, note that the body ID and sub shape ID will be invalid. Holds a
        /// reference to the shape: the caller deinits it.
        transformed_shape: TransformedShape,
        /// The remainder of the sub shape ID after removing the sub shape
        remainder: SubShapeID,
    };

    /// Result of getSubmergedVolume (Jolt's out parameters)
    pub const SubmergedVolume = struct {
        /// The total volume of the shape
        total_volume: f32,
        /// The submerged volume of the shape
        submerged_volume: f32,
        /// The world space center of mass of the submerged volume
        center_of_buoyancy: Vec3,
    };

    /// Class that holds information about the shape that can be used for logging / data collection purposes
    pub const Stats = struct {
        /// Amount of memory used by this shape (size in bytes)
        size_bytes: usize,
        /// Number of triangles in this shape (when applicable)
        num_triangles: u32,

        /// Constructor
        pub fn init(size_bytes: usize, num_triangles: u32) Stats {
            return .{ .size_bytes = size_bytes, .num_triangles = num_triangles };
        }
    };

    /// Set of shapes that getStatsRecursive already visited
    pub const VisitedShapes = UnorderedSet(*const Shape, .{});

    pub const ShapeToIDMap = ObjectToIDMap(Shape);
    pub const IDToShapeMap = IDToObjectMap(Shape);
    pub const MaterialToIDMap = ObjectToIDMap(PhysicsMaterial);
    pub const IDToMaterialMap = IDToObjectMap(PhysicsMaterial);

    // TODO(debug_renderer): static bool sDrawSubmergedVolumes (Debug helper which draws the intersection between water and the shapes, the center of buoyancy and the submerged volume)

    vtable: *const VTable,
    /// Reference count (RefTarget<Shape>)
    ref_count: RefCount = .{},
    /// Frees this shape (heap shapes) and its internal arrays
    allocator: Allocator,
    user_data: u64 = 0,
    shape_type: ShapeType,
    shape_sub_type: ShapeSubType,

    /// Constructor (Shape(EShapeType, EShapeSubType)), called by the classes that derive from Shape directly with
    /// `vtableFor(T)` of the most derived class
    pub fn init(vtable: *const VTable, allocator: Allocator, shape_type: ShapeType, shape_sub_type: ShapeSubType) Shape {
        return .{ .vtable = vtable, .allocator = allocator, .shape_type = shape_type, .shape_sub_type = shape_sub_type };
    }

    /// The part of Shape(EShapeType, EShapeSubType, const ShapeSettings &, ShapeResult &) that reads the settings
    /// (the shape was constructed with `init` before, the result is not used by the base class)
    pub fn initFromSettings(self: *Shape, settings: *const ShapeSettings) void {
        self.user_data = settings.user_data;
    }

    /// The vtable of concrete shape class `T`
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<Shape>

    /// Add a reference to this object
    pub fn addRef(self: *const Shape) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const Shape) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this shape as embedded (stack / member object): release() never frees it
    pub fn setEmbedded(self: *const Shape) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const Shape) u32 {
        return self.ref_count.get();
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
    /// This shape is scaled by scale in local space first.
    /// This function can be overridden to return a closer fitting world space bounding box, by default it will just transform what GetLocalBounds() returns.
    pub fn getWorldSpaceBounds(self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        return self.vtable.getWorldSpaceBounds(self, center_of_mass_transform, scale);
    }

    /// Get world space bounds including convex radius (GetWorldSpaceBounds(DMat44Arg, Vec3Arg), not virtual).
    pub fn getWorldSpaceBoundsDMat44(self: *const Shape, center_of_mass_transform: DMat44, scale: Vec3) AABox {
        // Use single precision version using the rotation only
        var bounds = self.getWorldSpaceBounds(center_of_mass_transform.getRotation(), scale);

        // Apply translation
        bounds.translateDVec3(center_of_mass_transform.getTranslation());

        return bounds;
    }

    /// The GetWorldSpaceBounds overload that C++ calls with an RMat44: the DMat44 version in double precision, the virtual one otherwise
    pub const getWorldSpaceBoundsRMat44 = if (Core.double_precision) getWorldSpaceBoundsDMat44 else getWorldSpaceBounds;

    /// Returns the radius of the biggest sphere that fits entirely in the shape. In case this shape consists of multiple sub shapes, it returns the smallest sphere of the parts.
    /// This can be used as a measure of how far the shape can be moved without risking going through geometry.
    pub fn getInnerRadius(self: *const Shape) f32 {
        return self.vtable.getInnerRadius(self);
    }

    /// Calculate the mass and inertia of this shape
    pub fn getMassProperties(self: *const Shape) MassProperties {
        return self.vtable.getMassProperties(self);
    }

    /// Get the leaf shape for a particular sub shape ID.
    /// @param sub_shape_id The full sub shape ID that indicates the path to the leaf shape
    /// @return The shape or null if the sub shape ID is invalid, and what remains of the sub shape ID after removing the path to the leaf shape (could e.g. refer to a triangle within a MeshShape)
    pub fn getLeafShape(self: *const Shape, sub_shape_id: SubShapeID) LeafShape {
        return self.vtable.getLeafShape(self, sub_shape_id);
    }

    /// Get the material assigned to a particular sub shape ID
    pub fn getMaterial(self: *const Shape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        return self.vtable.getMaterial(self, sub_shape_id);
    }

    /// Get the surface normal of a particular sub shape ID and point on surface (all vectors are relative to center of mass for this shape).
    /// Note: When you have a CollideShapeResult or ShapeCastResult you should use -mPenetrationAxis.Normalized() as contact normal as GetSurfaceNormal will only return face normals (and not vertex or edge normals).
    pub fn getSurfaceNormal(self: *const Shape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        return self.vtable.getSurfaceNormal(self, sub_shape_id, local_surface_position);
    }

    /// Get the vertices of the face that faces direction the most (includes any convex radius). Note that this function can only return faces of
    /// convex shapes or triangles, which is why a sub shape ID to get to that leaf must be provided.
    /// @param sub_shape_id Sub shape ID of target shape
    /// @param direction Direction that the face should be facing (in local space to this shape)
    /// @param center_of_mass_transform Transform to transform out_vertices with
    /// @param scale Scale in local space of the shape (scales relative to its center of mass)
    /// @param out_vertices Resulting face. The returned face can be empty if the shape doesn't have polygons to return (e.g. because it's a sphere). The face will be returned in world space.
    pub fn getSupportingFace(self: *const Shape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *SupportingFace) void {
        self.vtable.getSupportingFace(self, sub_shape_id, direction, scale, center_of_mass_transform, out_vertices);
    }

    /// Get the user data of a particular sub shape ID. Corresponds with the value stored in Shape::GetUserData of the leaf shape pointed to by sub_shape_id.
    pub fn getSubShapeUserData(self: *const Shape, sub_shape_id: SubShapeID) u64 {
        return self.vtable.getSubShapeUserData(self, sub_shape_id);
    }

    /// Get the direct child sub shape and its transform for a sub shape ID.
    /// @param sub_shape_id Sub shape ID that indicates the path to the leaf shape
    /// @param position_com The position of the center of mass of this shape
    /// @param rotation The orientation of this shape
    /// @param scale Scale in local space of the shape (scales relative to its center of mass)
    /// @return Direct child sub shape and its transform (note that the body ID and sub shape ID will be invalid) and the remainder of the sub shape ID after removing the sub shape
    pub fn getSubShapeTransformedShape(self: *const Shape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) SubShapeTransformedShape {
        return self.vtable.getSubShapeTransformedShape(self, sub_shape_id, position_com, rotation, scale);
    }

    /// Gets the properties needed to do buoyancy calculations for a body using this shape
    /// @param center_of_mass_transform Transform that takes this shape (centered around center of mass) to world space (or a desired other space)
    /// @param scale Scale in local space of the shape (scales relative to its center of mass)
    /// @param surface The surface plane of the liquid relative to center_of_mass_transform
    /// @return The total volume of the shape, the submerged volume of the shape and the world space center of mass of the submerged volume
    pub fn getSubmergedVolume(self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) SubmergedVolume {
        return self.vtable.getSubmergedVolume(self, center_of_mass_transform, scale, surface);
    }

    /// Cast a ray against this shape, returns true if it finds a hit closer than hit.fraction and updates that fraction. Otherwise hit is left untouched and the function returns false.
    /// Note that the ray should be relative to the center of mass of this shape (i.e. subtract Shape::GetCenterOfMass() from RayCast::mOrigin if you want to cast against the shape in the space it was created).
    /// Convex objects will be treated as solid (meaning if the ray starts inside, you'll get a hit fraction of 0) and back face hits against triangles are returned.
    /// If you want the surface normal of the hit use GetSurfaceNormal(hit.sub_shape_id2, ray.getPointOnRay(hit.fraction)).
    pub fn castRay(self: *const Shape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        return self.vtable.castRay(self, ray, sub_shape_id_creator, hit);
    }

    /// Cast a ray against this shape. Allows returning multiple hits through collector. Note that this version is more flexible but also slightly slower than the CastRay function that returns only a single hit.
    /// If you want the surface normal of the hit use GetSurfaceNormal(collected sub shape ID, ray.getPointOnRay(collected faction)).
    pub fn castRayCollector(self: *const Shape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        self.vtable.castRayCollector(self, ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    /// Check if point is inside this shape. For this tests all shapes are treated as if they were solid.
    /// Note that point should be relative to the center of mass of this shape (i.e. subtract Shape::GetCenterOfMass() from point if you want to test against the shape in the space it was created).
    /// For a mesh shape, this test will only provide sensible information if the mesh is a closed manifold.
    /// For each shape that collides, collector will receive a hit.
    pub fn collidePoint(self: *const Shape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        self.vtable.collidePoint(self, point, sub_shape_id_creator, collector, shape_filter);
    }

    /// Collides all vertices of a soft body with this shape and updates SoftBodyVertex::mCollisionPlane, SoftBodyVertex::mCollidingShapeIndex and SoftBodyVertex::mLargestPenetration if a collision with more penetration was found.
    /// @param center_of_mass_transform Center of mass transform for this shape relative to the vertices.
    /// @param scale Scale in local space of the shape (scales relative to its center of mass)
    /// @param vertices The vertices of the soft body
    /// @param num_vertices The number of vertices in vertices
    /// @param colliding_shape_index Value to store in CollideSoftBodyVertexIterator::mCollidingShapeIndex when a collision was found
    pub fn collideSoftBodyVertices(self: *const Shape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        self.vtable.collideSoftBodyVertices(self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index);
    }

    /// Collect the leaf transformed shapes of all leaf shapes of this shape.
    /// box is the world space axis aligned box which leaf shapes should collide with.
    /// position_com/rotation/scale describes the transform of this shape.
    /// sub_shape_id_creator represents the current sub shape ID of this shape.
    pub fn collectTransformedShapes(self: *const Shape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        self.vtable.collectTransformedShapes(self, box, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
    }

    /// Transforms this shape and all of its children with transform, resulting shape(s) are passed to collector.
    /// Note that not all shapes support all transforms (especially true for scaling), the resulting shape will try to match the transform as accurately as possible.
    /// @param center_of_mass_transform The transform (rotation, translation, scale) that the center of mass of the shape should get
    /// @param collector The transformed shapes will be passed to this collector
    pub fn transformShape(self: *const Shape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        self.vtable.transformShape(self, center_of_mass_transform, collector);
    }

    // ScaleShape: not ported yet (needs ScaledShape / StaticCompoundShape), see the file header

    /// To start iterating over triangles, call this function first.
    /// context is a temporary buffer and should remain untouched until the last call to GetTrianglesNext.
    /// box is the world space bounding in which you want to get the triangles.
    /// position_com/rotation/scale describes the transform of this shape.
    /// To get the actual triangles call GetTrianglesNext.
    pub fn getTrianglesStart(self: *const Shape, context: *GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        self.vtable.getTrianglesStart(self, context, box, position_com, rotation, scale);
    }

    /// Call this repeatedly to get all triangles in the box.
    /// out_triangle_vertices should be large enough to hold 3 * max_triangles_requested entries.
    /// out_materials (if it is not null) should contain max_triangles_requested entries.
    /// The function returns the amount of triangles that it found (which will be <= max_triangles_requested), or 0 if there are no more triangles.
    /// Note that the function can return a value < max_triangles_requested and still have more triangles to process (triangles can be returned in blocks).
    /// Note that the function may return triangles outside of the requested box, only coarse culling is performed on the returned triangles.
    pub fn getTrianglesNext(self: *const Shape, context: *GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        std.debug.assert(out_triangle_vertices.len >= 3 * @as(usize, max_triangles_requested));
        if (out_materials) |m| std.debug.assert(m.len >= max_triangles_requested);
        return self.vtable.getTrianglesNext(self, context, max_triangles_requested, out_triangle_vertices, out_materials);
    }

    // Binary serialization of the shape. Note that this saves the 'cooked' shape in a format which will not be backwards compatible for newer library versions.
    // In this case you need to recreate the shape from the ShapeSettings object and save it again. The user is expected to call SaveBinaryState followed by SaveMaterialState and SaveSubShapeState.
    // The stream should be stored as is and the material and shape list should be saved using the applications own serialization system (e.g. by assigning an ID to each pointer).
    // When restoring data, call sRestoreFromBinaryState to get the shape and then call RestoreMaterialState and RestoreSubShapeState to restore the pointers to the external objects.
    // Alternatively you can use SaveWithChildren and sRestoreWithChildren to save and restore the shape and all its child shapes and materials in a single stream.

    /// Saves the contents of the shape in binary form to stream.
    pub fn saveBinaryState(self: *const Shape, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// Creates a Shape of the correct type and restores its contents from the binary stream.
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!ShapeResult {
        var result: ShapeResult = .empty;

        // Read the type of the shape
        var raw_sub_type: u8 = 0;
        stream.read(&raw_sub_type);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read type id");
            return result;
        }

        // Jolt indexes the registry without checking (an invalid value or a type without constructor is undefined
        // behavior there), Zolt reports it like a failed read
        const sub_type = std.enums.fromInt(ShapeSubType, raw_sub_type) orelse {
            result.setError("Failed to read type id");
            return result;
        };
        const construct = ShapeFunctions.get(sub_type).construct orelse {
            result.setError("Failed to read type id");
            return result;
        };

        // Construct and read the data of the shape
        var shape = Ref(Shape).init(try construct(allocator));
        defer shape.deinit();
        try shape.get().?.restoreBinaryState(stream);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to restore shape");
            return result;
        }

        result.set(shape.clone());
        return result;
    }

    /// Outputs the material references that this shape has to out_materials.
    pub fn saveMaterialState(self: *const Shape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        return self.vtable.saveMaterialState(self, allocator, out_materials);
    }

    /// Restore the material references after calling sRestoreFromBinaryState. Note that the exact same materials need to be provided in the same order as returned by SaveMaterialState.
    pub fn restoreMaterialState(self: *Shape, materials: []const PhysicsMaterialRefC) void {
        self.vtable.restoreMaterialState(self, materials);
    }

    /// Outputs the shape references that this shape has to out_sub_shapes.
    pub fn saveSubShapeState(self: *const Shape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
        return self.vtable.saveSubShapeState(self, allocator, out_sub_shapes);
    }

    /// Restore the shape references after calling sRestoreFromBinaryState. Note that the exact same shapes need to be provided in the same order as returned by SaveSubShapeState.
    pub fn restoreSubShapeState(self: *Shape, sub_shapes: []const ShapeRefC) void {
        self.vtable.restoreSubShapeState(self, sub_shapes);
    }

    /// Save this shape, all its children and its materials. Pass in an empty map in shape_map / material_map or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
    pub fn saveWithChildren(self: *const Shape, allocator: Allocator, stream: StreamOut, shape_map: *ShapeToIDMap, material_map: *MaterialToIDMap) Allocator.Error!void {
        if (shape_map.find(self)) |shape_id_iter| {
            // Known shape, just write the ID
            stream.write(shape_id_iter.value);
            return;
        }

        // Write shape ID of this shape
        const shape_id: u32 = shape_map.count();
        (try shape_map.getOrPutValue(allocator, self, 0)).* = shape_id;
        stream.write(shape_id);

        // Write the shape itself
        self.saveBinaryState(stream);

        // Write the ID's of all sub shapes
        var sub_shapes: ShapeList = .empty;
        defer releaseList(Shape, allocator, &sub_shapes);
        try self.saveSubShapeState(allocator, &sub_shapes);
        stream.write(@as(u32, @intCast(sub_shapes.items.len)));
        for (sub_shapes.items) |sub_shape| {
            if (sub_shape.get()) |shape|
                try shape.saveWithChildren(allocator, stream, shape_map, material_map)
            else
                stream.write(~@as(u32, 0));
        }

        // Write the materials
        var materials: PhysicsMaterialList = .empty;
        defer releaseList(PhysicsMaterial, allocator, &materials);
        try self.saveMaterialState(allocator, &materials);
        try saveObjectArray(PhysicsMaterial, allocator, stream, materials.items, material_map);
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
        result.assignMove(try restoreFromBinaryState(allocator, stream));
        if (result.hasError())
            return result;
        if (Core.enable_asserts) std.debug.assert(shape_map.items.len == shape_id); // Assert that this is the next ID in the map (a corrupt stream can violate this)
        try shape_map.ensureUnusedCapacity(allocator, 1);
        shape_map.appendAssumeCapacity(result.get().clone());

        // Read the sub shapes
        var len: u32 = 0;
        stream.read(&len);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read stream");
            return result;
        }
        var sub_shapes: ShapeList = .empty;
        defer releaseList(Shape, allocator, &sub_shapes);
        try sub_shapes.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            var sub_shape_result = try restoreWithChildren(allocator, stream, shape_map, material_map);
            if (sub_shape_result.hasError()) {
                result.assignMove(sub_shape_result);
                return result;
            }
            defer sub_shape_result.deinit();
            try sub_shapes.append(allocator, .init(sub_shape_result.getPtr()));
        }
        result.getPtr().?.restoreSubShapeState(sub_shapes.items);

        // Read the materials
        var materials: PhysicsMaterialList = .empty;
        defer releaseList(PhysicsMaterial, allocator, &materials);
        var mlresult = try restoreObjectArray(PhysicsMaterial, allocator, stream, material_map, &materials);
        defer mlresult.deinit();
        if (mlresult.hasError()) {
            result.setError(mlresult.getError());
            return result;
        }
        result.getPtr().?.restoreMaterialState(materials.items);

        return result;
    }

    /// Get stats of this shape. Use for logging / data collection purposes only. Does not add values from child shapes, use GetStatsRecursive for this.
    pub fn getStats(self: *const Shape) Stats {
        return self.vtable.getStats(self);
    }

    /// Get the combined stats of this shape and its children.
    /// @param visited_shapes is used to track which shapes have already been visited, to avoid calculating the wrong memory size.
    pub fn getStatsRecursive(self: *const Shape, allocator: Allocator, visited_shapes: *VisitedShapes) Allocator.Error!Stats {
        return self.vtable.getStatsRecursive(self, allocator, visited_shapes);
    }

    /// Volume of this shape (m^3). Note that for compound shapes the volume may be incorrect since child shapes can overlap which is not accounted for.
    pub fn getVolume(self: *const Shape) f32 {
        return self.vtable.getVolume(self);
    }

    /// Test if scale is a valid scale for this shape. Some shapes can only be scaled uniformly, compound shapes cannot handle shapes
    /// being rotated and scaled (this would cause shearing), scale can never be zero. When the scale is invalid, the function will return false.
    ///
    /// Here's a list of supported scales:
    /// * SphereShape: Scale must be uniform (signs of scale are ignored).
    /// * BoxShape: Any scale supported (signs of scale are ignored).
    /// * TriangleShape: Any scale supported when convex radius is zero, otherwise only uniform scale supported.
    /// * CapsuleShape: Scale must be uniform (signs of scale are ignored).
    /// * TaperedCapsuleShape: Scale must be uniform (sign of Y scale can be used to flip the capsule).
    /// * CylinderShape: Scale must be uniform in XZ plane, Y can scale independently (signs of scale are ignored).
    /// * RotatedTranslatedShape: Scale must not cause shear in the child shape.
    /// * CompoundShape: Scale must not cause shear in any of the child shapes.
    pub fn isValidScale(self: *const Shape, scale: Vec3) bool {
        return self.vtable.isValidScale(self, scale);
    }

    /// This function will make sure that if you wrap this shape in a ScaledShape that the scale is valid.
    /// Note that this involves discarding components of the scale that are invalid, so the resulting scaled shape may be different than the requested scale.
    /// Compare the return value of this function with the scale you passed in to detect major inconsistencies and possibly warn the user.
    /// @param scale Local space scale for this shape.
    /// @return Scale that can be used to wrap this shape in a ScaledShape. IsValidScale will return true for this scale.
    pub fn makeScaleValid(self: *const Shape, scale: Vec3) Vec3 {
        return self.vtable.makeScaleValid(self, scale);
    }

    /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
    pub fn restoreBinaryState(self: *Shape, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// A fallback version of CollidePoint that uses a ray cast and counts the number of hits to determine if the point is inside the shape. Odd number of hits means inside, even number of hits means outside.
    /// (protected static in Jolt: sCollidePointUsingRayCast)
    pub fn collidePointUsingRayCast(shape: *const Shape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // First test if we're inside our bounding box
        const bounds = shape.getLocalBounds();
        if (bounds.containsVec3(point)) {
            // A collector that just counts the number of hits
            const HitCountCollector = struct {
                pub const overrides = .{.addHit};

                base: CastRayCollector = .init(@This()),
                hit_count: i32 = 0,
                sub_shape_id: SubShapeID = .empty,

                pub fn addHit(self: *@This(), result: *const RayCastResult) void {
                    // Store the last sub shape ID so that we can provide something to our outer hit collector
                    self.sub_shape_id = result.sub_shape_id2;

                    self.hit_count += 1;
                }
            };
            var hit_count_collector: HitCountCollector = .{};

            // Configure the raycast
            var settings: RayCastSettings = .{};
            settings.setBackFaceMode(.collide_with_back_faces);

            // Cast a ray that's 10% longer than the height of our bounding box
            shape.castRayCollector(.init(point, Vec3.axisY().mulScalar(1.1 * bounds.getSize().getY())), &settings, sub_shape_id_creator, &hit_count_collector.base, shape_filter);

            // Odd amount of hits means inside
            if ((hit_count_collector.hit_count & 1) == 1)
                collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = hit_count_collector.sub_shape_id });
        }
    }

    /// Implementations of the virtual functions in Shape (C++ Shape::Foo bodies). C++'s explicit base calls
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

        pub fn getSubShapeTransformedShape(self: *const Shape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) SubShapeTransformedShape {
            _ = sub_shape_id;

            // We have reached the leaf shape so there is no remainder
            const remainder: SubShapeID = .empty;

            // Just return the transformed shape for this shape
            var ts = TransformedShape.init(RVec3.fromVec3(position_com), rotation, self, .invalid, .{});
            ts.setShapeScale(scale);
            return .{ .transformed_shape = ts, .remainder = remainder };
        }

        pub fn collectTransformedShapes(self: *const Shape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
            _ = box;

            // Test shape filter
            if (!shape_filter.shouldCollide(self, sub_shape_id_creator.getID()))
                return;

            var ts = TransformedShape.init(RVec3.fromVec3(position_com), rotation, self, TransformedShape.getBodyID(collector.getContext()), .{ .sub_shape_id_creator = sub_shape_id_creator });
            defer ts.deinit();
            ts.setShapeScale(scale);
            collector.addHit(&ts);
        }

        pub fn transformShape(self: *const Shape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
            const decomposed = center_of_mass_transform.decompose();
            const transform = decomposed.rotation_translation;
            var ts = TransformedShape.init(RVec3.fromVec3(transform.getTranslation()), transform.getQuaternion(), self, .invalid, .{});
            defer ts.deinit();
            ts.setShapeScale(self.makeScaleValid(decomposed.scale));
            collector.addHit(&ts);
        }

        pub fn saveBinaryState(self: *const Shape, stream: StreamOut) void {
            stream.write(self.shape_sub_type);
            stream.write(self.user_data);
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

        pub fn getStatsRecursive(self: *const Shape, allocator: Allocator, visited_shapes: *VisitedShapes) Allocator.Error!Stats {
            var stats = self.getStats();

            // If shape is already visited, don't count its size again
            if (!(try visited_shapes.insert(allocator, self)).inserted)
                stats.size_bytes = 0;

            return stats;
        }

        pub fn isValidScale(self: *const Shape, scale: Vec3) bool {
            _ = self;
            return !ScaleHelpers.isZeroScale(scale);
        }

        pub fn makeScaleValid(self: *const Shape, scale: Vec3) Vec3 {
            _ = self;
            return ScaleHelpers.makeNonZeroScale(scale);
        }

        pub fn restoreBinaryState(self: *Shape, stream: StreamIn) Allocator.Error!void {
            // Type hash read by sRestoreFromBinaryState
            stream.read(&self.user_data);
        }
    };
};

/// Release the references in `list` and free it
fn releaseList(comptime T: type, allocator: Allocator, list: *std.ArrayList(RefConst(T))) void {
    for (list.items) |*item| item.deinit();
    list.deinit(allocator);
}

/// StreamUtils::SaveObjectReference: save an object reference to a stream. Uses a map to map objects to IDs which is also used to prevent writing duplicates.
/// (Zolt: Jolt/Core/StreamUtils.h is deferred to the ObjectStream phase, the helpers that Shape uses live here)
fn saveObjectReference(comptime T: type, allocator: Allocator, stream: StreamOut, object: ?*const T, object_to_id_map: ?*ObjectToIDMap(T)) Allocator.Error!void {
    if (object_to_id_map == null or object == null) {
        // Write null ID
        stream.write(~@as(u32, 0));
    } else if (object_to_id_map.?.find(object.?)) |id| {
        // Existing object, write ID
        stream.write(id.value);
    } else {
        // New object, write the ID
        const new_id: u32 = object_to_id_map.?.count();
        (try object_to_id_map.?.getOrPutValue(allocator, object.?, 0)).* = new_id;
        stream.write(new_id);

        // Write the object
        object.?.saveBinaryState(stream);
    }
}

/// StreamUtils::RestoreObjectReference: restore an object reference from stream.
fn restoreObjectReference(comptime T: type, allocator: Allocator, stream: StreamIn, id_to_object_map: *IDToObjectMap(T)) Allocator.Error!Result(Ref(T)) {
    var result: Result(Ref(T)) = .empty;
    errdefer result.deinit();

    // Read id
    var id: u32 = ~@as(u32, 0);
    stream.read(&id);

    // Check null
    if (id == ~@as(u32, 0)) {
        result.set(.empty);
        return result;
    }

    // Check if it already exists
    if (id >= id_to_object_map.items.len) {
        // New object, restore it
        result.assignMove(try T.restoreFromBinaryState(allocator, stream));
        if (result.hasError())
            return result;
        if (Core.enable_asserts) std.debug.assert(id == id_to_object_map.items.len); // A corrupt stream can violate this
        try id_to_object_map.ensureUnusedCapacity(allocator, 1);
        id_to_object_map.appendAssumeCapacity(result.get().clone());
    } else {
        // Existing object filter
        result.set(id_to_object_map.items[id].clone());
    }

    return result;
}

/// StreamUtils::SaveObjectArray: save an array of objects to a stream.
fn saveObjectArray(comptime T: type, allocator: Allocator, stream: StreamOut, array: []const RefConst(T), object_to_id_map: ?*ObjectToIDMap(T)) Allocator.Error!void {
    const len: u32 = @intCast(array.len);
    stream.write(len);
    for (array) |value|
        try saveObjectReference(T, allocator, stream, value.get(), object_to_id_map);
}

/// StreamUtils::RestoreObjectArray: restore an array of objects from a stream. The objects are appended to the empty
/// list `out_values` (Jolt returns Result<ArrayType>), the returned result holds Jolt's error text on failure.
fn restoreObjectArray(comptime T: type, allocator: Allocator, stream: StreamIn, id_to_object_map: *IDToObjectMap(T), out_values: *std.ArrayList(RefConst(T))) Allocator.Error!Result(void) {
    var result: Result(void) = .empty;

    var len: u32 = 0;
    stream.read(&len);
    if (stream.isEOF() or stream.isFailed()) {
        result.setError("Failed to read stream");
        return result;
    }

    try out_values.ensureTotalCapacity(allocator, len);
    for (0..len) |_| {
        var value = try restoreObjectReference(T, allocator, stream, id_to_object_map);
        defer value.deinit();
        if (value.hasError()) {
            result.setError(value.getError());
            return result;
        }
        try out_values.append(allocator, .init(value.getPtr()));
    }

    result.set({});
    return result;
}
