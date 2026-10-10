//! Port of: Jolt/Physics/Collision/Shape/CompoundShape.h, Jolt/Physics/Collision/Shape/CompoundShape.cpp
//! Status: complete
//!
//! Architecture (Docs/Zolt/CollisionArchitecture.md):
//! - D1: CompoundShape is an abstract class that adds virtual functions, so it has its own vtable with Shape's as
//!   prefix: `CompoundShape.VTable = struct { base: Shape.VTable, getIntersectingSubShapes,
//!   getIntersectingSubShapesOrientedBox }`, built by `CompoundShape.vtableFor(T)` with the most derived type T (the
//!   constructors take T as their first parameter). Its implementations of Shape's virtual functions are in
//!   `CompoundShape.impl`; C++'s qualified `CompoundShape::Foo()` in a derived class is
//!   `CompoundShape.impl.foo(&self.base, ...)`.
//! - D2: the compound owns its children (`SubShape.shape: RefConst(Shape)`) and the sub shape array (allocated with the
//!   shape's allocator `base.allocator`), both released in `destruct` (~CompoundShape).
//! - D3 / D13: CompoundShapeSettings holds the child settings as `Ref(ShapeSettings)` (Jolt: RefConst, creating the
//!   child writes its cache, Rule M) and its sub shape array is allocated with the settings' allocator
//!   (`base.allocator`), so `addShape` / `addShapePtr` return `Allocator.Error`.
//! - D11: the binary state reads the sub shape array, so restoreBinaryState returns `Allocator.Error!void`.
//! - The visitors of CompoundShapeVisitors.h (protected nested structs in Jolt) are in CompoundShapeVisitors.zig and
//!   re-exported here (`CompoundShape.CastRayVisitor`, ...).
//!
//! Signatures that differ from C++:
//! - The two `GetIntersectingSubShapes` overloads are `getIntersectingSubShapes` (AABox) and
//!   `getIntersectingSubShapesOrientedBox`; `uint *outSubShapeIndices, int inMaxSubShapeIndices` is the slice
//!   `out_sub_shape_indices: []u32` and the result count is a u32.
//! - `CompoundShapeSettings::AddShape(..., const ShapeSettings *, ...)` / `AddShape(..., const Shape *, ...)` are
//!   `addShape` / `addShapePtr`, the defaulted `inUserData` is `opts.user_data`.
//! - `GetSubShapeIndexFromID(id, outRemainder)` returns `SubShapeIndex{ index, remainder }`.
//! - `SubShape::FromSettings(settings, outResult)` takes the allocator that creates the child shape and returns
//!   `Allocator.Error!bool`.
//! - `CompoundShape(EShapeSubType)` is `init(T, allocator, sub_type)`, the constructor that takes the settings is
//!   `initFromSettings(self, settings, result)` (the base class part, the derived class does the rest).
//! - `sCastCompoundVsShape` (private in Jolt) is `castCompoundVsShape`, pub so that tests can compare the dispatch
//!   table with it.
//! - The asserts that queries with a scale the compound cannot represent (a non uniform scale on a rotated sub shape)
//!   or a stale sub shape ID can violate (GetLocalTransformNoScale, GetSubShapeIndexFromID) are guarded with
//!   `Core.enable_asserts`: Jolt's release build continues (porting guide section 7).
//! - JPH_DEBUG_RENDERER (Draw, DrawGetSupportFunction, DrawGetSupportingFace and the draw of the center of buoyancy in
//!   GetSubmergedVolume) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const ReferenceFile = @import("../../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const virtual = @import("../../../Core/Virtual.zig");
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const math = @import("../../../Math/Math.zig");
const Quat = @import("../../../Math/Quat.zig").Quat;
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeList = ShapeFile.ShapeList;
const ShapeRefC = ShapeFile.ShapeRefC;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeType = ShapeFile.ShapeType;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const Visitors = @import("CompoundShapeVisitors.zig");

/// Base class settings to construct a compound shape
pub const CompoundShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT(JPH_EXPORT, CompoundShapeSettings)

    pub const SubShapeSettings = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, SubShapeSettings)

        /// Sub shape (either this or shape_ptr needs to be filled up). Jolt: RefConst<ShapeSettings>, a Ref here
        /// because creating the child writes its cache (Rule M).
        shape: Ref(ShapeSettings) = .empty,
        /// Sub shape (either this or shape needs to be filled up)
        shape_ptr: RefConst(Shape) = .empty,
        /// Position of the sub shape (uninitialized in Jolt)
        position: Vec3 = Vec3.zero(),
        /// Rotation of the sub shape (uninitialized in Jolt)
        rotation: Quat = Quat.identity(),

        /// User data value (can be used by the application for any purpose).
        /// Note this value can be retrieved through GetSubShape(...).user_data, not through GetSubShapeUserData(...) as that returns Shape::GetUserData() of the leaf shape.
        /// Use GetSubShapeIndexFromID get a shape index from a SubShapeID to pass to GetSubShape.
        user_data: u32 = 0,

        /// Copy (the copy constructor), adds references to the shapes
        pub fn clone(self: *const SubShapeSettings) SubShapeSettings {
            var copy = self.*;
            copy.shape = self.shape.clone();
            copy.shape_ptr = self.shape_ptr.clone();
            return copy;
        }

        /// Destructor (releases the shapes)
        pub fn deinit(self: *SubShapeSettings) void {
            self.shape.deinit();
            self.shape_ptr.deinit();
        }
    };

    pub const SubShapes = std.ArrayList(SubShapeSettings);

    base: ShapeSettings,

    /// The sub shapes (allocated with base.allocator)
    sub_shapes: SubShapes = .empty,

    /// Constructor. Use AddShape to add the parts. Called by the derived settings with their most derived type `T`.
    pub fn init(comptime T: type, allocator: Allocator) CompoundShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator) };
    }

    /// ~CompoundShapeSettings
    pub fn destruct(self: *CompoundShapeSettings) void {
        for (self.sub_shapes.items) |*s| s.deinit();
        self.sub_shapes.deinit(self.base.allocator);
    }

    /// Upcast to the root class
    pub fn asShapeSettings(self: *CompoundShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Add a shape to the compound (adds a reference to `shape`).
    pub fn addShape(self: *CompoundShapeSettings, position: Vec3, rotation: Quat, shape: ?*ShapeSettings, opts: struct { user_data: u32 = 0 }) Allocator.Error!void {
        try self.sub_shapes.ensureUnusedCapacity(self.base.allocator, 1);

        // Add shape
        self.sub_shapes.appendAssumeCapacity(.{
            .position = position,
            .rotation = rotation,
            .shape = .init(shape),
            .user_data = opts.user_data,
        });
    }

    /// Add a shape to the compound. Variant that uses a concrete shape, which means this object cannot be serialized
    /// (adds a reference to `shape`).
    pub fn addShapePtr(self: *CompoundShapeSettings, position: Vec3, rotation: Quat, shape: ?*const Shape, opts: struct { user_data: u32 = 0 }) Allocator.Error!void {
        try self.sub_shapes.ensureUnusedCapacity(self.base.allocator, 1);

        // Add shape
        self.sub_shapes.appendAssumeCapacity(.{
            .position = position,
            .rotation = rotation,
            .shape_ptr = .init(shape),
            .user_data = opts.user_data,
        });
    }
};

/// Base class for a compound shape
pub const CompoundShape = struct {
    /// Abstract class: `Shape.isKindOf(CompoundShape)` / `Shape.cast(CompoundShape)` check the shape type
    pub const shape_type: ShapeType = .compound;

    /// Shape's vtable followed by the virtual functions that CompoundShape adds
    pub const VTable = struct {
        base: Shape.VTable,
        /// Get which sub shape's bounding boxes overlap with an axis aligned box
        getIntersectingSubShapes: *const fn (self: *const CompoundShape, box: AABox, out_sub_shape_indices: []u32) u32,
        /// Get which sub shape's bounding boxes overlap with an axis aligned box (oriented box version)
        getIntersectingSubShapesOrientedBox: *const fn (self: *const CompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32,
    };

    /// Class that holds a sub shape
    pub const SubShape = struct {
        shape: RefConst(Shape) = .empty,
        /// Note: Position of center of mass of sub shape!
        position_com: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// Note: X, Y, Z of rotation quaternion (Jolt reads 4 bytes beyond this, Zolt only reads the 3 floats)
        rotation: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// User data value (put here because it falls in padding bytes)
        user_data: u32 = 0,
        /// If rotation is close to identity (put here because it falls in padding bytes)
        is_rotation_identity: bool = false,
        // 3 padding bytes left

        comptime {
            std.debug.assert(@sizeOf(SubShape) == if (@sizeOf(usize) == 8) 40 else 36); // Compiler added unexpected padding
        }

        /// Copy (the copy constructor), adds a reference to the shape
        pub fn clone(self: *const SubShape) SubShape {
            var copy = self.*;
            copy.shape = self.shape.clone();
            return copy;
        }

        /// Destructor (releases the shape)
        pub fn deinit(self: *SubShape) void {
            self.shape.deinit();
        }

        /// Initialize sub shape from sub shape settings
        /// @param settings Settings object
        /// @param result Result object, only used in case of error
        /// @param allocator Allocator for the child shape (when it is created from its settings)
        /// @return True on success, false on failure
        pub fn fromSettings(self: *SubShape, settings: *const CompoundShapeSettings.SubShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!bool {
            if (settings.shape_ptr.get()) |shape_ptr| {
                // Use provided shape
                self.shape.set(shape_ptr);
            } else {
                // Create child shape
                var child_result = try settings.shape.get().?.createShape(allocator);
                defer child_result.deinit();
                if (!child_result.isValid()) {
                    result.assign(&child_result);
                    return false;
                }
                self.shape.set(child_result.getPtr());
            }

            // Copy user data
            self.user_data = settings.user_data;

            self.setTransform(settings.position, settings.rotation, Vec3.zero()); // Center of mass not yet calculated
            return true;
        }

        /// Update the transform of this sub shape
        /// @param position New position
        /// @param rotation New orientation
        /// @param center_of_mass The center of mass of the compound shape
        pub fn setTransform(self: *SubShape, position: Vec3, rotation: Quat, center_of_mass: Vec3) void {
            self.setPositionCOM(position.sub(center_of_mass).add(rotation.mulVec3(self.shape.get().?.getCenterOfMass())));

            self.is_rotation_identity = rotation.isClose(Quat.identity(), .{}) or rotation.isClose(Quat.identity().negate(), .{});
            self.setRotation(if (self.is_rotation_identity) Quat.identity() else rotation);
        }

        /// Get the local transform for this shape given the scale of the child shape
        /// The total transform of the child shape will be GetLocalTransformNoScale(scale) * Mat44::sScaling(TransformScale(scale))
        /// @param scale The scale of the child shape (in local space of this shape)
        pub fn getLocalTransformNoScale(self: *const SubShape, scale: Vec3) Mat44 {
            if (Core.enable_asserts) std.debug.assert(self.isValidScale(scale)); // An invalid scale can violate this, Jolt's release build continues
            return Mat44.rotationTranslation(self.getRotation(), scale.mul(self.getPositionCOM()));
        }

        /// Test if scale is valid for this sub shape
        pub fn isValidScale(self: *const SubShape, scale: Vec3) bool {
            // We can always handle uniform scale or identity rotations
            if (self.is_rotation_identity or ScaleHelpers.isUniformScale(scale))
                return true;

            return ScaleHelpers.canScaleBeRotated(self.getRotation(), scale);
        }

        /// Transform the scale to the local space of the child shape
        pub fn transformScale(self: *const SubShape, scale: Vec3) Vec3 {
            // We don't need to transform uniform scale or if the rotation is identity
            if (self.is_rotation_identity or ScaleHelpers.isUniformScale(scale))
                return scale;

            return ScaleHelpers.rotateScale(self.getRotation(), scale);
        }

        /// Compress the center of mass position
        pub fn setPositionCOM(self: *SubShape, position_com: Vec3) void {
            position_com.storeFloat3(&self.position_com);
        }

        /// Uncompress the center of mass position
        pub fn getPositionCOM(self: *const SubShape) Vec3 {
            return Vec3.loadFloat3Unsafe(&self.position_com);
        }

        /// Compress the rotation
        pub fn setRotation(self: *SubShape, rotation: Quat) void {
            rotation.storeFloat3(&self.rotation);
        }

        /// Uncompress the rotation
        pub fn getRotation(self: *const SubShape) Quat {
            return if (self.is_rotation_identity) Quat.identity() else Quat.loadFloat3Unsafe(&self.rotation);
        }
    };

    pub const SubShapes = std.ArrayList(SubShape);

    /// Result of getSubShapeIndexFromID (Jolt returns the index and writes outRemainder)
    pub const SubShapeIndex = struct {
        /// The index of the sub shape of this compound
        index: u32,
        /// The sub shape ID for the sub shape of the compound after popping off the index
        remainder: SubShapeID,
    };

    // Visitors for collision detection (CompoundShapeVisitors.zig)
    pub const CastRayVisitor = Visitors.CastRayVisitor;
    pub const CastRayVisitorCollector = Visitors.CastRayVisitorCollector;
    pub const CollidePointVisitor = Visitors.CollidePointVisitor;
    pub const CastShapeVisitor = Visitors.CastShapeVisitor;
    pub const CollectTransformedShapesVisitor = Visitors.CollectTransformedShapesVisitor;
    pub const CollideCompoundVsShapeVisitor = Visitors.CollideCompoundVsShapeVisitor;
    pub const CollideShapeVsCompoundVisitor = Visitors.CollideShapeVsCompoundVisitor;
    pub const GetIntersectingSubShapesVisitor = Visitors.GetIntersectingSubShapesVisitor;

    base: Shape,
    /// Center of mass of the compound
    center_of_mass: Vec3 = Vec3.zero(),
    local_bounds: AABox = .init(Vec3.zero(), Vec3.zero()),
    /// The sub shapes (allocated with base.allocator)
    sub_shapes: SubShapes = .empty,
    /// Smallest radius of GetInnerRadius() of child shapes
    inner_radius: f32 = math.flt_max,

    /// Constructor (CompoundShape(EShapeSubType)), called by the concrete classes with their most derived type `T`
    pub fn init(comptime T: type, allocator: Allocator, sub_type: ShapeSubType) CompoundShape {
        return .{ .base = .init(&vtableFor(T).base, allocator, .compound, sub_type) };
    }

    /// The part of CompoundShape(EShapeSubType, const ShapeSettings &, ShapeResult &) that reads the settings (the shape
    /// was constructed with `init` before, the result is not used by the base class)
    pub fn initFromSettings(self: *CompoundShape, settings: *const ShapeSettings, result: *ShapeResult) void {
        _ = result;
        self.base.initFromSettings(settings);
    }

    /// The vtable of the concrete compound class `T`
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// ~CompoundShape
    pub fn destruct(self: *CompoundShape) void {
        for (self.sub_shapes.items) |*s| s.deinit();
        self.sub_shapes.deinit(self.base.allocator);
    }

    fn getVTable(self: *const CompoundShape) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// Upcast to the root class
    pub fn asShape(self: *const CompoundShape) *const Shape {
        return &self.base;
    }

    /// Upcast to the root class (creation and restore only)
    pub fn asShapeMut(self: *CompoundShape) *Shape {
        return &self.base;
    }

    /// Get which sub shape's bounding boxes overlap with an axis aligned box
    /// @param box The axis aligned box to test against (relative to the center of mass of this shape)
    /// @param out_sub_shape_indices Buffer where to place the indices of the sub shapes that intersect (its length is how many indices will fit in the buffer, normally you'd provide a buffer of GetNumSubShapes() indices)
    /// @return How many indices were placed in out_sub_shape_indices
    pub fn getIntersectingSubShapes(self: *const CompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        return self.getVTable().getIntersectingSubShapes(self, box, out_sub_shape_indices);
    }

    /// Get which sub shape's bounding boxes overlap with an axis aligned box
    /// @param box The oriented box to test against (relative to the center of mass of this shape)
    /// @param out_sub_shape_indices Buffer where to place the indices of the sub shapes that intersect (its length is how many indices will fit in the buffer, normally you'd provide a buffer of GetNumSubShapes() indices)
    /// @return How many indices were placed in out_sub_shape_indices
    pub fn getIntersectingSubShapesOrientedBox(self: *const CompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32 {
        return self.getVTable().getIntersectingSubShapesOrientedBox(self, box, out_sub_shape_indices);
    }

    /// Access to the sub shapes of this compound
    pub fn getSubShapes(self: *const CompoundShape) []const SubShape {
        return self.sub_shapes.items;
    }

    /// Get the total number of sub shapes
    pub fn getNumSubShapes(self: *const CompoundShape) u32 {
        return @intCast(self.sub_shapes.items.len);
    }

    /// Access to a particular sub shape
    pub fn getSubShape(self: *const CompoundShape, idx: u32) *const SubShape {
        return &self.sub_shapes.items[idx];
    }

    /// Get the user data associated with a shape in this compound
    pub fn getCompoundUserData(self: *const CompoundShape, idx: u32) u32 {
        return self.sub_shapes.items[idx].user_data;
    }

    /// Set the user data associated with a shape in this compound
    pub fn setCompoundUserData(self: *CompoundShape, idx: u32, user_data: u32) void {
        self.sub_shapes.items[idx].user_data = user_data;
    }

    /// Check if a sub shape ID is still valid for this shape
    /// @param sub_shape_id Sub shape id that indicates the leaf shape relative to this shape
    /// @return True if the ID is valid, false if not
    pub fn isSubShapeIDValid(self: *const CompoundShape, sub_shape_id: SubShapeID) bool {
        return sub_shape_id.popID(self.getSubShapeIDBits()).id < self.sub_shapes.items.len;
    }

    /// Convert SubShapeID to sub shape index
    /// @param sub_shape_id Sub shape id that indicates the leaf shape relative to this shape
    /// @return The index of the sub shape of this compound and the sub shape ID for the sub shape of the compound after popping off the index
    pub fn getSubShapeIndexFromID(self: *const CompoundShape, sub_shape_id: SubShapeID) SubShapeIndex {
        const popped = sub_shape_id.popID(self.getSubShapeIDBits());
        if (Core.enable_asserts) std.debug.assert(popped.id < self.sub_shapes.items.len); // Invalid SubShapeID (a stale ID can violate this, see getLeafShape)
        return .{ .index = popped.id, .remainder = popped.remainder };
    }

    /// Convert a sub shape index to a sub shape ID
    /// @param idx Index of the sub shape of this compound
    /// @param parent_sub_shape_id Parent SubShapeID (describing the path to the compound shape)
    /// @return A sub shape ID creator that contains the full path to the sub shape with index idx
    pub fn getSubShapeIDFromIndex(self: *const CompoundShape, idx: u32, parent_sub_shape_id: SubShapeIDCreator) SubShapeIDCreator {
        return parent_sub_shape_id.pushID(idx, self.getSubShapeIDBits());
    }

    /// Determine amount of bits needed to encode sub shape id (protected in Jolt)
    pub fn getSubShapeIDBits(self: *const CompoundShape) u32 {
        // Ensure we have enough bits to encode our shape [0, n - 1]
        const n: u32 = @as(u32, @intCast(self.sub_shapes.items.len)) -% 1;
        return 32 - math.countLeadingZeros(n);
    }

    /// Determine the inner radius of this shape (protected in Jolt)
    pub fn calculateInnerRadius(self: *CompoundShape) void {
        self.inner_radius = math.flt_max;
        for (self.sub_shapes.items) |*s|
            self.inner_radius = math.min(self.inner_radius, s.shape.get().?.getInnerRadius());
    }

    /// CompoundShape's implementations of Shape's virtual functions (C++ `CompoundShape::Foo`)
    pub const impl = struct {
        // See Shape::GetCenterOfMass
        pub fn getCenterOfMass(self: *const CompoundShape) Vec3 {
            return self.center_of_mass;
        }

        // See Shape::MustBeStatic
        pub fn mustBeStatic(self: *const CompoundShape) bool {
            for (self.sub_shapes.items) |*shape|
                if (shape.shape.get().?.mustBeStatic())
                    return true;

            return false;
        }

        // See Shape::GetLocalBounds
        pub fn getLocalBounds(self: *const CompoundShape) AABox {
            return self.local_bounds;
        }

        // See Shape::GetSubShapeIDBitsRecursive
        pub fn getSubShapeIDBitsRecursive(self: *const CompoundShape) u32 {
            // Add max of child bits to our bits
            var child_bits: u32 = 0;
            for (self.sub_shapes.items) |*shape|
                child_bits = @max(child_bits, shape.shape.get().?.getSubShapeIDBitsRecursive());
            return child_bits + self.getSubShapeIDBits();
        }

        // See Shape::GetWorldSpaceBounds
        pub fn getWorldSpaceBounds(self: *const CompoundShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
            if (self.sub_shapes.items.len == 0) {
                // If there are no sub-shapes, we must return an empty box to avoid overflows in the broadphase
                return .init(center_of_mass_transform.getTranslation(), center_of_mass_transform.getTranslation());
            } else if (self.sub_shapes.items.len <= 10) {
                var bounds: AABox = .empty;
                for (self.sub_shapes.items) |*shape| {
                    const transform = center_of_mass_transform.mul(shape.getLocalTransformNoScale(scale));
                    bounds.encapsulate(shape.shape.get().?.getWorldSpaceBounds(transform, shape.transformScale(scale)));
                }
                return bounds;
            } else {
                // If there are too many shapes, use the base class function (this will result in a slightly wider bounding box)
                return Shape.impl.getWorldSpaceBounds(&self.base, center_of_mass_transform, scale);
            }
        }

        // See Shape::GetInnerRadius
        pub fn getInnerRadius(self: *const CompoundShape) f32 {
            return self.inner_radius;
        }

        // See Shape::GetMassProperties
        pub fn getMassProperties(self: *const CompoundShape) MassProperties {
            var p: MassProperties = .{};

            // Calculate mass and inertia
            p.mass = 0.0;
            p.inertia = Mat44.zero();
            for (self.sub_shapes.items) |*shape| {
                // Rotate and translate inertia of child into place
                var child = shape.shape.get().?.getMassProperties();
                child.rotate(Mat44.rotationQuat(shape.getRotation()));
                child.translate(shape.getPositionCOM());

                // Accumulate mass and inertia
                p.mass += child.mass;
                p.inertia = p.inertia.add(child.inertia);
            }

            // Ensure that inertia is a 3x3 matrix, adding inertias causes the bottom right element to change
            p.inertia.setColumn4(3, Vec4.init(0, 0, 0, 1));

            return p;
        }

        // See Shape::GetMaterial
        pub fn getMaterial(self: *const CompoundShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);

            // Pass call on
            return self.sub_shapes.items[index.index].shape.get().?.getMaterial(index.remainder);
        }

        // See Shape::GetLeafShape
        pub fn getLeafShape(self: *const CompoundShape, sub_shape_id: SubShapeID) Shape.LeafShape {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);
            if (index.index >= self.sub_shapes.items.len) {
                // No longer valid index
                return .{ .shape = null, .remainder = .empty };
            }

            // Pass call on
            return self.sub_shapes.items[index.index].shape.get().?.getLeafShape(index.remainder);
        }

        // See Shape::GetSubShapeUserData
        pub fn getSubShapeUserData(self: *const CompoundShape, sub_shape_id: SubShapeID) u64 {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);
            if (index.index >= self.sub_shapes.items.len)
                return 0; // No longer valid index

            // Pass call on
            return self.sub_shapes.items[index.index].shape.get().?.getSubShapeUserData(index.remainder);
        }

        // See Shape::GetSubShapeTransformedShape
        pub fn getSubShapeTransformedShape(self: *const CompoundShape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) Shape.SubShapeTransformedShape {
            // Get the sub shape
            const index = self.getSubShapeIndexFromID(sub_shape_id);
            const sub_shape = &self.sub_shapes.items[index.index];

            // Calculate transform for sub shape
            const position = position_com.add(rotation.mulVec3(scale.mul(sub_shape.getPositionCOM())));
            const sub_shape_rotation = rotation.mul(sub_shape.getRotation());
            const sub_shape_scale = sub_shape.transformScale(scale);

            // Return transformed shape
            var ts = TransformedShape.init(RVec3.fromVec3(position), sub_shape_rotation, sub_shape.shape.get(), .invalid, .{});
            ts.setShapeScale(sub_shape_scale);
            return .{ .transformed_shape = ts, .remainder = index.remainder };
        }

        // See Shape::GetSurfaceNormal
        pub fn getSurfaceNormal(self: *const CompoundShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);

            // Transform surface position to local space and pass call on
            const shape = &self.sub_shapes.items[index.index];
            const transform = Mat44.inverseRotationTranslation(shape.getRotation(), shape.getPositionCOM());
            const normal = shape.shape.get().?.getSurfaceNormal(index.remainder, transform.mulVec3(local_surface_position));

            // Transform normal to this shape's space
            return transform.multiply3x3Transposed(normal);
        }

        // See Shape::GetSupportingFace
        pub fn getSupportingFace(self: *const CompoundShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);

            // Apply transform and pass on to sub shape
            const shape = &self.sub_shapes.items[index.index];
            const transform = shape.getLocalTransformNoScale(scale);
            shape.shape.get().?.getSupportingFace(index.remainder, transform.multiply3x3Transposed(direction), shape.transformScale(scale), center_of_mass_transform.mul(transform), out_vertices);
        }

        // See Shape::GetSubmergedVolume
        pub fn getSubmergedVolume(self: *const CompoundShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
            var out: Shape.SubmergedVolume = .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };

            for (self.sub_shapes.items) |*shape| {
                // Get center of mass transform of child
                const transform = center_of_mass_transform.mul(shape.getLocalTransformNoScale(scale));

                // Recurse to child
                const child = shape.shape.get().?.getSubmergedVolume(transform, shape.transformScale(scale), surface);

                // Accumulate volumes
                out.total_volume += child.total_volume;
                out.submerged_volume += child.submerged_volume;

                // The center of buoyancy is the weighted average of the center of buoyancy of our child shapes
                out.center_of_buoyancy = out.center_of_buoyancy.add(child.center_of_buoyancy.mulScalar(child.submerged_volume));
            }

            if (out.submerged_volume > 0.0)
                out.center_of_buoyancy = out.center_of_buoyancy.divScalar(out.submerged_volume);

            // TODO(debug_renderer): draw the center of buoyancy when sDrawSubmergedVolumes is set

            return out;
        }

        // TODO(debug_renderer): Draw, DrawGetSupportFunction, DrawGetSupportingFace

        // See: Shape::CollideSoftBodyVertices
        pub fn collideSoftBodyVertices(self: *const CompoundShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
            for (self.sub_shapes.items) |*shape| {
                const transform = shape.getLocalTransformNoScale(scale);
                shape.shape.get().?.collideSoftBodyVertices(center_of_mass_transform.mul(transform), shape.transformScale(scale), vertices, num_vertices, colliding_shape_index);
            }
        }

        // See Shape::TransformShape
        pub fn transformShape(self: *const CompoundShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
            for (self.sub_shapes.items) |*shape|
                shape.shape.get().?.transformShape(center_of_mass_transform.mul(Mat44.rotationTranslation(shape.getRotation(), shape.getPositionCOM())), collector);
        }

        // See Shape::GetTrianglesStart
        pub fn getTrianglesStart(self: *const CompoundShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
            _ = .{ self, context, box, position_com, rotation, scale };
            if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
        }

        // See Shape::GetTrianglesNext
        pub fn getTrianglesNext(self: *const CompoundShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
            _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
            if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
            return 0;
        }

        // See Shape::SaveBinaryState
        pub fn saveBinaryState(self: *const CompoundShape, stream: StreamOut) void {
            Shape.impl.saveBinaryState(&self.base, stream);

            stream.write(self.center_of_mass);
            stream.write(self.local_bounds.min);
            stream.write(self.local_bounds.max);
            stream.write(self.inner_radius);

            // Write sub shapes
            stream.writeArrayWith(SubShape, self.sub_shapes.items, {}, struct {
                fn f(_: void, element: *const SubShape, s: StreamOut) void {
                    s.write(element.user_data);
                    s.write(element.position_com);
                    s.write(element.rotation);
                }
            }.f);
        }

        // See Shape::SaveSubShapeState
        pub fn saveSubShapeState(self: *const CompoundShape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
            // outSubShapes.clear() (releases the references)
            for (out_sub_shapes.items) |*s| s.deinit();
            out_sub_shapes.clearRetainingCapacity();
            try out_sub_shapes.ensureTotalCapacity(allocator, self.sub_shapes.items.len);
            for (self.sub_shapes.items) |*shape|
                out_sub_shapes.appendAssumeCapacity(shape.shape.clone());
        }

        // See Shape::RestoreSubShapeState
        pub fn restoreSubShapeState(self: *CompoundShape, sub_shapes: []const ShapeRefC) void {
            if (Core.enable_asserts) std.debug.assert(self.sub_shapes.items.len == sub_shapes.len); // A corrupt stream can violate this (Jolt's release build then reads inNumShapes elements regardless)
            for (sub_shapes, 0..) |sub_shape, i|
                self.sub_shapes.items[i].shape.set(sub_shape.get());
        }

        // See Shape::GetStatsRecursive
        pub fn getStatsRecursive(self: *const CompoundShape, allocator: Allocator, visited_shapes: *Shape.VisitedShapes) Allocator.Error!Shape.Stats {
            // Get own stats
            var stats = try Shape.impl.getStatsRecursive(&self.base, allocator, visited_shapes);

            // Add child stats
            for (self.sub_shapes.items) |*shape| {
                const child_stats = try shape.shape.get().?.getStatsRecursive(allocator, visited_shapes);
                stats.size_bytes +%= child_stats.size_bytes;
                stats.num_triangles +%= child_stats.num_triangles;
            }

            return stats;
        }

        // See Shape::GetVolume
        pub fn getVolume(self: *const CompoundShape) f32 {
            var volume: f32 = 0.0;
            for (self.sub_shapes.items) |*shape|
                volume += shape.shape.get().?.getVolume();
            return volume;
        }

        // See Shape::IsValidScale
        pub fn isValidScale(self: *const CompoundShape, scale: Vec3) bool {
            if (!Shape.impl.isValidScale(&self.base, scale))
                return false;

            for (self.sub_shapes.items) |*shape| {
                // Test if the scale is non-uniform and the shape is rotated
                if (!shape.isValidScale(scale))
                    return false;

                // Test the child shape
                if (!shape.shape.get().?.isValidScale(shape.transformScale(scale)))
                    return false;
            }

            return true;
        }

        // See Shape::MakeScaleValid
        pub fn makeScaleValid(self: *const CompoundShape, scale_in: Vec3) Vec3 {
            const scale = ScaleHelpers.makeNonZeroScale(scale_in);
            if (impl.isValidScale(self, scale)) // C++ qualified call CompoundShape::IsValidScale (not virtual)
                return scale;

            const abs_uniform_scale = ScaleHelpers.makeUniformScale(scale.abs());
            const uniform_scale = scale.getSign().mul(abs_uniform_scale);
            if (impl.isValidScale(self, uniform_scale))
                return uniform_scale;

            return abs_uniform_scale.mulScalar(math.sign(scale.getX()));
        }

        // See: Shape::RestoreBinaryState
        pub fn restoreBinaryState(self: *CompoundShape, stream: StreamIn) Allocator.Error!void {
            try Shape.impl.restoreBinaryState(&self.base, stream);

            stream.read(&self.center_of_mass);
            stream.read(&self.local_bounds.min);
            stream.read(&self.local_bounds.max);
            stream.read(&self.inner_radius);

            // Read sub shapes
            try stream.readArrayWith(SubShape, self.base.allocator, &self.sub_shapes, {}, struct {
                fn f(_: void, s: StreamIn, element: *SubShape) Allocator.Error!void {
                    s.read(&element.user_data);
                    s.read(&element.position_com);
                    s.read(&element.rotation);
                    element.is_rotation_identity = element.rotation.x == 0 and element.rotation.y == 0 and element.rotation.z == 0;
                }
            }.f);
        }
    };

    /// Helper function called by CollisionDispatch (sCastCompoundVsShape, private in Jolt)
    pub fn castCompoundVsShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        // Fetch compound shape from cast shape
        const compound = shape_cast.shape.cast(CompoundShape); // Asserts that the type is EShapeType::Compound

        // Number of sub shapes
        const n: u32 = @intCast(compound.sub_shapes.items.len);

        // Determine amount of bits for sub shape
        const sub_shape_bits = compound.getSubShapeIDBits();

        // Recurse to sub shapes
        var i: u32 = 0;
        while (i < n) : (i += 1) {
            const sub_shape = &compound.sub_shapes.items[i];

            // Create ID for sub shape
            const shape1_sub_shape_id = sub_shape_id_creator1.pushID(i, sub_shape_bits);

            // Transform the shape cast and update the shape
            const transform = shape_cast.center_of_mass_start.mul(sub_shape.getLocalTransformNoScale(shape_cast.scale));
            const sub_scale = sub_shape.transformScale(shape_cast.scale);
            const sub_shape_cast = ShapeCast.init(sub_shape.shape.get().?, sub_scale, transform, shape_cast.direction);

            CollisionDispatch.castShapeVsShapeLocalSpace(&sub_shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, shape1_sub_shape_id, sub_shape_id_creator2, collector);

            if (collector.shouldEarlyOut())
                break;
        }
    }

    /// Register shape functions with the registry (sRegister)
    pub fn register(comptime r: *Registry) void {
        for (ShapeFile.compound_sub_shape_types) |s1|
            for (ShapeFile.all_sub_shape_types) |s2|
                r.registerCastShape(s1, s2, castCompoundVsShape);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the test shapes of TestShapes.zig are registered as User1..User3 through zolt_user_types in the inline tests)

const testing = std.testing;
const AABox4 = @import("../../../Geometry/AABox4.zig");
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RefCount = ReferenceFile.RefCount;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const CastRayCollector = ShapeFile.CastRayCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const AnyHitCollisionCollector = CollisionCollectorImpl.AnyHitCollisionCollector;
const TestShapes = @import("TestShapes.zig");
const TestBoxShape = TestShapes.TestBoxShape;
const TestBoxShapeSettings = TestShapes.TestBoxShapeSettings;
const TestSphereShape = TestShapes.TestSphereShape;
const TestCompoundShape = TestShapes.TestCompoundShape;
const TestMaterial = TestShapes.TestMaterial;

/// Settings of LinearCompoundShape (a concrete class derived from CompoundShapeSettings)
const LinearCompoundShapeSettings = struct {
    pub const overrides = .{.createShape};

    base: CompoundShapeSettings,

    fn init(allocator: Allocator) LinearCompoundShapeSettings {
        return .{ .base = .init(LinearCompoundShapeSettings, allocator) };
    }

    fn asShapeSettings(self: *LinearCompoundShapeSettings) *ShapeSettings {
        return self.base.asShapeSettings();
    }

    fn deinit(self: *LinearCompoundShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    pub fn createShape(self: *LinearCompoundShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(LinearCompoundShape, self, allocator);
    }
};

/// A minimal concrete compound shape (User4) that walks its sub shapes linearly with the visitors of
/// CompoundShapeVisitors.zig: it tests the bounds of every sub shape with `testBounds` (the box in lane 0, the other
/// lanes repeat it) and visits the ones that pass, like MutableCompoundShape::WalkSubShapes does for blocks of 4. The
/// constructor is StaticCompoundShape's without the tree (it accepts any number of sub shapes).
const LinearCompoundShape = struct {
    pub const shape_sub_type: ShapeSubType = .user4;
    pub const overrides = .{ .castRay, .castRayCollector, .collidePoint, .collectTransformedShapes, .getStats, .getIntersectingSubShapes, .getIntersectingSubShapesOrientedBox };

    base: CompoundShape,

    pub fn initDefault(allocator: Allocator) LinearCompoundShape {
        return .{ .base = .init(LinearCompoundShape, allocator, shape_sub_type) };
    }

    pub fn initFromSettings(self: *LinearCompoundShape, settings: *const LinearCompoundShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base.base, result);
        const c = &self.base;

        // Keep track of total mass to calculate center of mass
        var mass: f32 = 0.0;

        try c.sub_shapes.appendNTimes(c.base.allocator, .{}, settings.base.sub_shapes.items.len);
        for (settings.base.sub_shapes.items, c.sub_shapes.items) |*shape, *out_shape| {
            // Start constructing the runtime sub shape
            if (!try out_shape.fromSettings(shape, result, allocator))
                return;

            // Calculate mass properties of child
            const child = out_shape.shape.get().?.getMassProperties();

            // Accumulate center of mass
            mass += child.mass;
            c.center_of_mass = c.center_of_mass.add(out_shape.getPositionCOM().mulScalar(child.mass));
        }

        if (mass > 0.0)
            c.center_of_mass = c.center_of_mass.divScalar(mass);

        // Cache the inner radius as it can take a while to recursively iterate over all sub shapes
        c.calculateInnerRadius();

        // Shift all shapes so that the center of mass is now at the origin and calculate bounds
        for (c.sub_shapes.items) |*shape| {
            shape.setPositionCOM(shape.getPositionCOM().sub(c.center_of_mass));
            c.local_bounds.encapsulate(subShapeBounds(shape));
        }

        // Check if we're not exceeding the amount of sub shape id bits
        if (self.asShape().getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    fn asShape(self: *const LinearCompoundShape) *const Shape {
        return self.base.asShape();
    }

    fn asShapeMut(self: *LinearCompoundShape) *Shape {
        return self.base.asShapeMut();
    }

    /// The bounds of a sub shape in the space of the compound
    fn subShapeBounds(sub_shape: *const CompoundShape.SubShape) AABox {
        return sub_shape.shape.get().?.getWorldSpaceBounds(Mat44.rotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM()), Vec3.one());
    }

    /// The visitor protocol: testBounds, then visitShape when the result passes, then shouldAbort
    fn walkSubShapes(self: *const LinearCompoundShape, visitor: anytype) void {
        for (self.base.sub_shapes.items, 0..) |*sub_shape, i| {
            const b = subShapeBounds(sub_shape);
            const result = visitor.testBounds(b.min.splatX(), b.min.splatY(), b.min.splatZ(), b.max.splatX(), b.max.splatY(), b.max.splatZ());
            if (shouldVisit(visitor, result)) {
                visitor.visitShape(sub_shape, @intCast(i));
                if (visitor.shouldAbort())
                    break;
            }
        }
    }

    /// MutableCompoundShape's ShouldVisitSubShape of the visitors (lane 0)
    fn shouldVisit(visitor: anytype, result: anytype) bool {
        if (@TypeOf(result) == UVec4)
            return result.getX() != 0;
        const fraction = if (@TypeOf(visitor.*) == CompoundShape.CastRayVisitor) visitor.hit.fraction else visitor.collector.getEarlyOutFraction();
        return result.getX() < fraction;
    }

    pub fn castRay(self: *const LinearCompoundShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        var visitor: CompoundShape.CastRayVisitor = .init(&ray, &self.base, sub_shape_id_creator, hit);
        self.walkSubShapes(&visitor);
        return visitor.return_value;
    }

    pub fn castRayCollector(self: *const LinearCompoundShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: CompoundShape.CastRayVisitorCollector = .init(&ray, ray_cast_settings, &self.base, sub_shape_id_creator, collector, shape_filter);
        self.walkSubShapes(&visitor);
    }

    pub fn collidePoint(self: *const LinearCompoundShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: CompoundShape.CollidePointVisitor = .init(point, &self.base, sub_shape_id_creator, collector, shape_filter);
        self.walkSubShapes(&visitor);
    }

    pub fn collectTransformedShapes(self: *const LinearCompoundShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: CompoundShape.CollectTransformedShapesVisitor = .init(box, &self.base, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
        self.walkSubShapes(&visitor);
    }

    pub fn getIntersectingSubShapes(self: *const LinearCompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        var visitor: CompoundShape.GetIntersectingSubShapesVisitor(AABox) = .init(box, out_sub_shape_indices);
        if (!visitor.shouldAbort()) self.walkSubShapes(&visitor);
        return visitor.getNumResults();
    }

    pub fn getIntersectingSubShapesOrientedBox(self: *const LinearCompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32 {
        var visitor: CompoundShape.GetIntersectingSubShapesVisitor(OrientedBox) = .init(box, out_sub_shape_indices);
        if (!visitor.shouldAbort()) self.walkSubShapes(&visitor);
        return visitor.getNumResults();
    }

    pub fn getStats(self: *const LinearCompoundShape) Shape.Stats {
        return .init(@sizeOf(LinearCompoundShape) + self.base.sub_shapes.items.len * @sizeOf(CompoundShape.SubShape), 0);
    }

    // The collision functions that a compound registers in CollisionDispatch (called directly by the tests)

    fn collideCompoundVsShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const compound = shape1.cast(LinearCompoundShape);
        var visitor: CompoundShape.CollideCompoundVsShapeVisitor = .init(&compound.base, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
        compound.walkSubShapes(&visitor);
    }

    fn collideShapeVsCompound(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const compound = shape2.cast(LinearCompoundShape);
        var visitor: CompoundShape.CollideShapeVsCompoundVisitor = .init(shape1, &compound.base, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
        compound.walkSubShapes(&visitor);
    }

    fn castShapeVsCompound(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const compound = shape.cast(LinearCompoundShape);
        var visitor: CompoundShape.CastShapeVisitor = .init(shape_cast, shape_cast_settings, &compound.base, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
        compound.walkSubShapes(&visitor);
    }
};

/// A compound with 4 test children (see makeTestCompound)
const TestCompound = struct {
    material: *TestMaterial,
    box_a: *TestBoxShape,
    box_b: *TestBoxShape,
    sphere: *TestSphereShape,
    box_settings: *TestBoxShapeSettings,
    settings: LinearCompoundShapeSettings,
    result: ShapeResult,

    fn shape(self: *const TestCompound) *const Shape {
        return self.result.getPtr().?;
    }

    fn compound(self: *const TestCompound) *const CompoundShape {
        return self.shape().cast(CompoundShape);
    }

    fn deinit(self: *TestCompound) void {
        self.result.deinit();
        self.settings.deinit();
        self.material.base.release();
    }
};

/// Sub shape 0: box_a (a material, user data 11) at (-2, 0, 0); 1: box_b (a center of mass offset) rotated 90 degrees
/// around Y at (2, 0, 0) (compound user data 5); 2: a box from settings at (0, 3, 0) (compound user data 6); 3: the
/// sphere at (0, -3, 0)
fn makeTestCompound(allocator: Allocator) !TestCompound {
    const material = try TestMaterial.create(allocator, 1);
    material.base.addRef();
    errdefer material.base.release();
    const box_a = try TestBoxShape.create(allocator, Vec3.one(), .{ .material = &material.base });
    box_a.asShapeMut().setUserData(11);
    var box_a_ref = RefConst(Shape).init(box_a.asShape());
    defer box_a_ref.deinit();
    const box_b = try TestBoxShape.create(allocator, Vec3.init(0.5, 1, 1), .{ .center_of_mass = Vec3.init(0.5, 0, 0) });
    var box_b_ref = RefConst(Shape).init(box_b.asShape());
    defer box_b_ref.deinit();
    const sphere = try TestSphereShape.create(allocator, 0.5);
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    const box_settings = try TestBoxShapeSettings.create(allocator, Vec3.one(), null);
    var box_settings_ref = Ref(ShapeSettings).init(box_settings.asShapeSettings());
    defer box_settings_ref.deinit();

    var settings = LinearCompoundShapeSettings.init(allocator);
    errdefer settings.deinit();
    try settings.base.addShapePtr(Vec3.init(-2, 0, 0), Quat.identity(), box_a.asShape(), .{});
    try settings.base.addShapePtr(Vec3.init(2, 0, 0), Quat.rotation(Vec3.axisY(), 0.5 * math.pi), box_b.asShape(), .{ .user_data = 5 });
    try settings.base.addShape(Vec3.init(0, 3, 0), Quat.identity(), box_settings.asShapeSettings(), .{ .user_data = 6 });
    try settings.base.addShapePtr(Vec3.init(0, -3, 0), Quat.identity(), sphere.asShape(), .{});

    const result = try settings.asShapeSettings().createShape(allocator);
    return .{ .material = material, .box_a = box_a, .box_b = box_b, .sphere = sphere, .box_settings = box_settings, .settings = settings, .result = result };
}

fn saveToBuffer(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

/// The sub shape ID of sub shape `idx` of a compound with 3 or 4 sub shapes (2 bits)
fn id2(idx: u32) SubShapeID {
    return SubShapeIDCreator.pushID(.{}, idx, 2).getID();
}

test "CompoundShape: CompoundShape.SubShape transforms, the compressed position and rotation and the scale helpers" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = TestBoxShape.init(allocator, Vec3.one(), .{ .center_of_mass = Vec3.init(1, 0, 0) });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    var s: CompoundShape.SubShape = .{ .shape = .init(box.asShape()) };
    defer s.deinit();
    try testing.expectEqual(RefCount.embedded + 1, box.asShape().getRefCount());

    // Rotated: the center of mass of the child is rotated into place, the rotation is stored as X, Y, Z (W positive)
    const rotation = Quat.rotation(Vec3.axisZ(), 0.5 * math.pi);
    s.setTransform(Vec3.init(1, 2, 3), rotation, Vec3.init(0.5, 0.5, 0.5));
    try expect(!s.is_rotation_identity);
    try expect(s.getPositionCOM().eql(Vec3.init(1, 2, 3).sub(Vec3.init(0.5, 0.5, 0.5)).add(rotation.mulVec3(Vec3.init(1, 0, 0)))));
    try expect(s.getPositionCOM().isClose(Vec3.init(0.5, 2.5, 2.5), .{}));
    try expect(s.getRotation().isClose(rotation, .{}));
    try testing.expectEqual(rotation.getX(), s.rotation.x);
    try testing.expectEqual(rotation.getZ(), s.rotation.z);
    s.setRotation(rotation.negate()); // Same rotation, W negative: stored with W positive
    try expect(s.getRotation().isClose(rotation, .{}));

    // Identity (and -identity, and close to it) is stored as zeros and returned exactly
    for ([_]Quat{ Quat.identity(), Quat.identity().negate(), Quat.rotation(Vec3.axisX(), 1.0e-7) }) |q| {
        s.setTransform(Vec3.init(1, 2, 3), q, Vec3.zero());
        try expect(s.is_rotation_identity);
        try expect(s.rotation.x == 0 and s.rotation.y == 0 and s.rotation.z == 0);
        try expect(s.getRotation().eql(Quat.identity()));
        try expect(s.getPositionCOM().eql(Vec3.init(1, 2, 3).add(q.mulVec3(Vec3.init(1, 0, 0)))));
    }

    // Identity rotation: any scale is valid and passed on as is
    const non_uniform = Vec3.init(1, 2, 3);
    try expect(s.isValidScale(non_uniform));
    try expect(s.transformScale(non_uniform).eql(non_uniform));
    try expect(s.getLocalTransformNoScale(non_uniform).eql(Mat44.rotationTranslation(Quat.identity(), non_uniform.mul(s.getPositionCOM()))));

    // 90 degree rotation: a non uniform scale can be rotated (the axes are permuted)
    s.setTransform(Vec3.zero(), rotation, Vec3.zero());
    try expect(s.isValidScale(non_uniform));
    try expect(s.transformScale(non_uniform).isClose(Vec3.init(2, 1, 3), .{}));
    try expect(s.transformScale(non_uniform).eql(ScaleHelpers.rotateScale(s.getRotation(), non_uniform)));
    try expect(s.transformScale(Vec3.replicate(-2)).eql(Vec3.replicate(-2))); // Uniform: not transformed
    try expect(s.getLocalTransformNoScale(non_uniform).eql(Mat44.rotationTranslation(s.getRotation(), non_uniform.mul(s.getPositionCOM()))));

    // 45 degree rotation: a non uniform scale would shear the child, a uniform one is fine
    s.setTransform(Vec3.zero(), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), Vec3.zero());
    try expect(!s.isValidScale(non_uniform));
    try expect(s.isValidScale(Vec3.replicate(3)));

    // clone adds a reference
    var copy = s.clone();
    try testing.expectEqual(RefCount.embedded + 2, box.asShape().getRefCount());
    copy.deinit();
    try testing.expectEqual(RefCount.embedded + 1, box.asShape().getRefCount());
}

test "CompoundShape: CompoundShape.SubShape.fromSettings with a shape, with settings and with a child error" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = TestBoxShape.init(allocator, Vec3.one(), .{ .center_of_mass = Vec3.init(0, 1, 0) });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    var settings = LinearCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    const good = try TestBoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), null);
    const bad = try TestBoxShapeSettings.create(allocator, Vec3.zero(), null);
    try settings.base.addShapePtr(Vec3.init(1, 0, 0), Quat.identity(), box.asShape(), .{ .user_data = 7 });
    try settings.base.addShape(Vec3.init(0, 1, 0), Quat.rotation(Vec3.axisX(), 0.3), good.asShapeSettings(), .{});
    try settings.base.addShape(Vec3.zero(), Quat.identity(), bad.asShapeSettings(), .{ .user_data = 9 });
    try testing.expectEqual(@as(usize, 3), settings.base.sub_shapes.items.len);
    try testing.expectEqual(@as(u32, 1), good.base.getRefCount());
    try testing.expectEqual(RefCount.embedded + 1, box.asShape().getRefCount());

    var result: ShapeResult = .empty;
    defer result.deinit();

    var s0: CompoundShape.SubShape = .{};
    defer s0.deinit();
    try expect(try s0.fromSettings(&settings.base.sub_shapes.items[0], &result, allocator));
    try expect(s0.shape.get() == box.asShape());
    try testing.expectEqual(@as(u32, 7), s0.user_data);
    try expect(s0.getPositionCOM().eql(Vec3.init(1, 1, 0)) and s0.is_rotation_identity);

    var s1: CompoundShape.SubShape = .{};
    defer s1.deinit();
    try expect(try s1.fromSettings(&settings.base.sub_shapes.items[1], &result, allocator));
    try expect(s1.shape.get() == good.base.cached_result.getPtr().?);
    try expect(!s1.is_rotation_identity);
    try expect(result.isEmpty());

    var s2: CompoundShape.SubShape = .{};
    defer s2.deinit();
    try expect(!try s2.fromSettings(&settings.base.sub_shapes.items[2], &result, allocator));
    try testing.expectEqualStrings("Invalid half extent", result.getError());
    try expect(s2.shape.get() == null);

    // SubShapeSettings.clone adds references
    var copy = settings.base.sub_shapes.items[1].clone();
    try testing.expectEqual(@as(u32, 2), good.base.getRefCount());
    copy.deinit();
    try testing.expectEqual(@as(u32, 1), good.base.getRefCount());
}

test "CompoundShapeSettings: addShape / addShapePtr use the settings' allocator, out of memory takes no references" {
    const allocator = testing.allocator;

    var box = TestBoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const child = try TestBoxShapeSettings.create(allocator, Vec3.one(), null);
    var child_ref = Ref(ShapeSettings).init(child.asShapeSettings());
    defer child_ref.deinit();

    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 1, .resize_fail_index = 0 });
    var settings = LinearCompoundShapeSettings.init(failing.allocator());
    defer settings.deinit();
    try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), box.asShape(), .{ .user_data = 1 });
    while (settings.base.sub_shapes.items.len < settings.base.sub_shapes.capacity) // Fill the first allocation
        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), box.asShape(), .{ .user_data = 1 });
    const num_added: u32 = @intCast(settings.base.sub_shapes.items.len);
    try testing.expectError(error.OutOfMemory, settings.base.addShape(Vec3.zero(), Quat.identity(), child.asShapeSettings(), .{}));
    try testing.expectError(error.OutOfMemory, settings.base.addShapePtr(Vec3.zero(), Quat.identity(), box.asShape(), .{}));
    try testing.expectEqual(@as(u32, 1), child.base.getRefCount());
    try testing.expectEqual(RefCount.embedded + num_added, box.asShape().getRefCount());
    try testing.expectEqual(@as(usize, num_added), settings.base.sub_shapes.items.len);

    // Null shapes can be added (Jolt only fails when creating the compound)
    var null_settings = LinearCompoundShapeSettings.init(allocator);
    defer null_settings.deinit();
    try null_settings.base.addShape(Vec3.zero(), Quat.identity(), null, .{});
    try null_settings.base.addShapePtr(Vec3.zero(), Quat.identity(), null, .{});
    try testing.expect(null_settings.base.sub_shapes.items[0].shape.get() == null and null_settings.base.sub_shapes.items[1].shape_ptr.get() == null);
}

test "CompoundShape: construction, center of mass, inner radius, bounds and mass properties" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var tc = try makeTestCompound(allocator);
    defer tc.deinit();
    const shape = tc.shape();
    const c = tc.compound();

    try expect(shape.getType() == .compound and shape.getSubType() == .user4);
    try expect(shape.isKindOf(CompoundShape) and shape.isKindOf(LinearCompoundShape));
    try testing.expectEqual(@as(u32, 4), c.getNumSubShapes());
    try testing.expectEqual(@as(usize, 4), c.getSubShapes().len);
    try expect(c.getSubShape(0).shape.get() == tc.box_a.asShape());
    try expect(c.getSubShape(2).shape.get() == tc.box_settings.base.cached_result.getPtr().?);
    try testing.expectEqual(@as(u32, 5), c.getCompoundUserData(1));
    try testing.expectEqual(@as(u32, 6), c.getCompoundUserData(2));

    // Center of mass: the mass weighted average of the child centers of mass (box_b's is rotated into place)
    const mass_a = tc.box_a.asShape().getMassProperties().mass;
    const mass_b = tc.box_b.asShape().getMassProperties().mass;
    const mass_s = tc.sphere.asShape().getMassProperties().mass;
    const com_b = Vec3.init(2, 0, 0).add(Quat.rotation(Vec3.axisY(), 0.5 * math.pi).mulVec3(Vec3.init(0.5, 0, 0)));
    var expected_com = Vec3.zero();
    expected_com = expected_com.add(Vec3.init(-2, 0, 0).mulScalar(mass_a));
    expected_com = expected_com.add(com_b.mulScalar(mass_b));
    expected_com = expected_com.add(Vec3.init(0, 3, 0).mulScalar(mass_a));
    expected_com = expected_com.add(Vec3.init(0, -3, 0).mulScalar(mass_s));
    var total_mass: f32 = 0.0;
    for ([_]f32{ mass_a, mass_b, mass_a, mass_s }) |m| total_mass += m;
    expected_com = expected_com.divScalar(total_mass);
    try expect(shape.getCenterOfMass().eql(expected_com));
    try expect(c.getSubShape(0).getPositionCOM().eql(Vec3.init(-2, 0, 0).sub(expected_com)));
    try expect(c.getSubShape(1).getPositionCOM().eql(com_b.sub(expected_com)));

    // Inner radius: the smallest child
    try testing.expectEqual(@as(f32, 0.5), shape.getInnerRadius());

    // Local bounds: the children's bounds and the origin (the bounds start as a zero box)
    var expected_bounds: AABox = .init(Vec3.zero(), Vec3.zero());
    for (c.getSubShapes()) |*s| expected_bounds.encapsulate(LinearCompoundShape.subShapeBounds(s));
    try expect(shape.getLocalBounds().eql(expected_bounds));
    try expect(shape.getLocalBounds().min.isClose(Vec3.init(-3, -3.5, -1).sub(expected_com), .{ .max_dist_sq = 1.0e-10 }));
    try expect(shape.getLocalBounds().max.isClose(Vec3.init(3, 4, 1).sub(expected_com), .{ .max_dist_sq = 1.0e-10 }));

    // Mass properties: the children rotated and translated into place
    const p = shape.getMassProperties();
    try testing.expectEqual(total_mass, p.mass);
    var expected_inertia = Mat44.zero();
    for (c.getSubShapes()) |*s| {
        var child = s.shape.get().?.getMassProperties();
        child.rotate(Mat44.rotationQuat(s.getRotation()));
        child.translate(s.getPositionCOM());
        expected_inertia = expected_inertia.add(child.inertia);
    }
    expected_inertia.setColumn4(3, Vec4.init(0, 0, 0, 1));
    try expect(p.inertia.eql(expected_inertia));
    try expect(p.inertia.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));

    // Volume: the sum of the children
    var expected_volume: f32 = 0.0;
    for ([_]f32{ 8.0, 4.0, 8.0, tc.sphere.asShape().getVolume() }) |v| expected_volume += v;
    try testing.expectEqual(expected_volume, shape.getVolume());

    // Sub shape ID bits: 2 bits for 4 sub shapes
    try testing.expectEqual(@as(u32, 2), shape.getSubShapeIDBitsRecursive());
}

/// A shape that needs 31 sub shape ID bits (User5): the compound that holds 4 of them is too deep
const DeepShape = struct {
    pub const overrides = .{ .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume };

    base: Shape,

    fn init(allocator: Allocator) DeepShape {
        return .{ .base = .init(Shape.vtableFor(DeepShape), allocator, .user1, .user5) };
    }

    pub fn getLocalBounds(_: *const DeepShape) AABox {
        return .init(Vec3.replicate(-1), Vec3.one());
    }

    pub fn getSubShapeIDBitsRecursive(_: *const DeepShape) u32 {
        return 31;
    }

    pub fn getInnerRadius(_: *const DeepShape) f32 {
        return 1.0;
    }

    pub fn getMassProperties(_: *const DeepShape) MassProperties {
        return .{};
    }

    pub fn getMaterial(_: *const DeepShape, _: SubShapeID) *const PhysicsMaterial {
        return PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(_: *const DeepShape, _: SubShapeID, _: Vec3) Vec3 {
        return Vec3.axisY();
    }

    pub fn getSubmergedVolume(_: *const DeepShape, _: Mat44, _: Vec3, _: Plane) Shape.SubmergedVolume {
        return .{ .total_volume = 0, .submerged_volume = 0, .center_of_buoyancy = Vec3.zero() };
    }

    pub fn castRay(_: *const DeepShape, _: RayCast, _: SubShapeIDCreator, _: *RayCastResult) bool {
        return false;
    }

    pub fn castRayCollector(_: *const DeepShape, _: RayCast, _: *const RayCastSettings, _: SubShapeIDCreator, _: *CastRayCollector, _: *const ShapeFilter) void {}

    pub fn collidePoint(_: *const DeepShape, _: Vec3, _: SubShapeIDCreator, _: *CollidePointCollector, _: *const ShapeFilter) void {}

    pub fn collideSoftBodyVertices(_: *const DeepShape, _: Mat44, _: Vec3, _: *const CollideSoftBodyVertexIterator, _: u32, _: i32) void {}

    pub fn getTrianglesStart(_: *const DeepShape, _: *Shape.GetTrianglesContext, _: AABox, _: Vec3, _: Quat, _: Vec3) void {}

    pub fn getTrianglesNext(_: *const DeepShape, _: *Shape.GetTrianglesContext, _: u32, _: []Float3, _: ?[]*const PhysicsMaterial) u32 {
        return 0;
    }

    pub fn getStats(_: *const DeepShape) Shape.Stats {
        return .init(0, 0);
    }

    pub fn getVolume(_: *const DeepShape) f32 {
        return 0.0;
    }
};

test "CompoundShape: construction errors and out of memory" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // A child error is forwarded
    {
        var settings = LinearCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        const bad = try TestBoxShapeSettings.create(allocator, Vec3.init(1, 1, -1), null);
        try settings.base.addShape(Vec3.zero(), Quat.identity(), bad.asShapeSettings(), .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid half extent", result.getError());
    }

    // Too many sub shape ID bits
    {
        var deep = DeepShape.init(allocator);
        deep.base.setEmbedded();
        defer deep.base.deinit();

        var settings = LinearCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), &deep.base, .{});
        var ok = try settings.asShapeSettings().createShape(allocator); // 31 + 0 bits
        defer ok.deinit();
        try expect(ok.isValid());
        try testing.expectEqual(@as(u32, 31), ok.getPtr().?.getSubShapeIDBitsRecursive());

        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), &deep.base, .{});
        settings.asShapeSettings().clearCachedResult();
        var result = try settings.asShapeSettings().createShape(allocator); // 31 + 1 bits
        defer result.deinit();
        try expect(result.isValid());

        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), &deep.base, .{});
        settings.asShapeSettings().clearCachedResult();
        var too_deep = try settings.asShapeSettings().createShape(allocator); // 31 + 2 bits
        defer too_deep.deinit();
        try testing.expectEqualStrings("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits", too_deep.getError());
    }

    // Out of memory at every allocation is returned and not cached, nothing leaks
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var tc = makeTestCompound(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        defer tc.deinit();
        try expect(tc.result.isValid() and !failing.has_induced_failure);
        break;
    }
    try expect(fail_index >= 9); // Material, 3 shapes, child settings, the settings array, compound, sub shapes, child shape
}

test "CompoundShape: the overrides of Shape pass the calls on to the sub shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var tc = try makeTestCompound(allocator);
    defer tc.deinit();
    const shape = tc.shape();
    const c = tc.compound();

    // mustBeStatic: any child
    try expect(!shape.mustBeStatic());

    // Material, leaf shape, user data: decoded from the sub shape ID
    try expect(shape.getMaterial(id2(0)) == &tc.material.base);
    try expect(shape.getMaterial(id2(1)) == PhysicsMaterial.default);
    try expect(shape.getLeafShape(id2(1)).shape == tc.box_b.asShape());
    try expect(shape.getLeafShape(id2(3)).shape == tc.sphere.asShape());
    try expect(shape.getLeafShape(id2(3)).remainder.isEmpty());
    try testing.expectEqual(@as(u64, 11), shape.getSubShapeUserData(id2(0)));
    try testing.expectEqual(@as(u64, 0), shape.getSubShapeUserData(id2(2)));
    if (!Core.enable_asserts) {
        // A stale ID (index beyond the sub shapes) gives null / 0 when asserts are disabled (Jolt asserts in debug)
        var three = LinearCompoundShapeSettings.init(allocator);
        defer three.deinit();
        for (0..3) |_| try three.base.addShapePtr(Vec3.zero(), Quat.identity(), tc.sphere.asShape(), .{});
        var r = try three.asShapeSettings().createShape(allocator);
        defer r.deinit();
        try expect(r.getPtr().?.getLeafShape(id2(3)).shape == null);
        try expect(r.getPtr().?.getLeafShape(id2(3)).remainder.isEmpty());
        try testing.expectEqual(@as(u64, 0), r.getPtr().?.getSubShapeUserData(id2(3)));
    }

    // getSubShapeTransformedShape: the transform of the sub shape, the remainder
    const position = Vec3.init(1, 2, 3);
    const rotation = Quat.rotation(Vec3.axisX(), 0.5 * math.pi);
    {
        var sub = shape.getSubShapeTransformedShape(id2(1), position, rotation, Vec3.replicate(2));
        defer sub.transformed_shape.deinit();
        const s = c.getSubShape(1);
        try expect(sub.transformed_shape.shape.get() == tc.box_b.asShape());
        try expect(sub.transformed_shape.shape_position_com.eql(RVec3.fromVec3(position.add(rotation.mulVec3(Vec3.replicate(2).mul(s.getPositionCOM()))))));
        try expect(sub.transformed_shape.shape_rotation.eql(rotation.mul(s.getRotation())));
        try expect(sub.transformed_shape.getShapeScale().eql(Vec3.replicate(2)));
        try expect(sub.transformed_shape.body_id.isInvalid() and sub.remainder.isEmpty());
    }

    // getSurfaceNormal: the position goes to the space of the child, the normal back
    {
        const s = c.getSubShape(1);
        const local = s.getPositionCOM().add(s.getRotation().mulVec3(Vec3.init(0.4, 0, 0))); // On the +X face of box_b (rotated: -Z)
        const normal = shape.getSurfaceNormal(id2(1), local);
        try expect(normal.isClose(Vec3.init(0, 0, -1), .{ .max_dist_sq = 1.0e-10 }));
    }

    // getSupportingFace: transformed by the local transform of the sub shape
    {
        var face: Shape.SupportingFace = .empty;
        const transform = Mat44.rotationTranslation(rotation, position);
        shape.getSupportingFace(id2(0), Vec3.init(-1, 0, 0), Vec3.one(), transform, &face); // AABox: the face opposite to the direction
        try testing.expectEqual(@as(u32, 4), face.len);
        const s = c.getSubShape(0);
        for (face.constSlice()) |v| try expect(transform.inversedRotationTranslation().mulVec3(v).sub(s.getPositionCOM()).getX() > 0.99); // The +X face of box_a
    }

    // getSubmergedVolume: the sum of the children, the center of buoyancy weighted by the submerged volume
    {
        const surface = Plane.init(Vec3.axisY(), 0.0);
        const v = shape.getSubmergedVolume(Mat44.identity(), Vec3.one(), surface);
        var expected_total: f32 = 0.0;
        var expected_submerged: f32 = 0.0;
        var expected_center = Vec3.zero();
        for (c.getSubShapes()) |*s| {
            const child = s.shape.get().?.getSubmergedVolume(s.getLocalTransformNoScale(Vec3.one()), Vec3.one(), surface);
            expected_total += child.total_volume;
            expected_submerged += child.submerged_volume;
            expected_center = expected_center.add(child.center_of_buoyancy.mulScalar(child.submerged_volume));
        }
        try testing.expectEqual(expected_total, v.total_volume);
        try testing.expectEqual(expected_submerged, v.submerged_volume);
        try expect(v.submerged_volume > 0.0 and v.submerged_volume < v.total_volume);
        try expect(v.center_of_buoyancy.eql(expected_center.divScalar(expected_submerged)));
        const above = shape.getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.init(Vec3.axisY(), 100.0));
        try testing.expectEqual(@as(f32, 0.0), above.submerged_volume);
        try expect(above.center_of_buoyancy.eql(Vec3.zero()));
    }

    // A nested compound (the test compound is sub shape 1 of an outer compound)
    {
        var outer = LinearCompoundShapeSettings.init(allocator);
        defer outer.deinit();
        try outer.base.addShapePtr(Vec3.init(0, -10, 0), Quat.identity(), tc.sphere.asShape(), .{});
        try outer.base.addShapePtr(Vec3.zero(), Quat.identity(), shape, .{});
        var r = try outer.asShapeSettings().createShape(allocator);
        defer r.deinit();
        const nested = r.getPtr().?;
        const oc = nested.cast(CompoundShape);

        // getSubShapeUserData passes the remainder on: box_a (sub shape 0 of the inner compound), not the sub shape that
        // the full ID would select in the inner compound (box_b, user data 0)
        try testing.expectEqual(@as(u64, 11), nested.getSubShapeUserData(SubShapeIDCreator.pushID(.{}, 1, 1).pushID(0, 2).getID()));
        try testing.expectEqual(@as(u64, 0), nested.getSubShapeUserData(SubShapeIDCreator.pushID(.{}, 1, 1).pushID(1, 2).getID()));

        // getSubmergedVolume: the surface goes through the center of mass of the inner compound, so that child is
        // partially submerged and its center of buoyancy is weighted by its submerged volume (not its total volume)
        const surface = Plane.fromPointAndNormal(oc.getSubShape(1).getPositionCOM(), Vec3.axisY());
        const v = nested.getSubmergedVolume(Mat44.identity(), Vec3.one(), surface);
        var expected_submerged: f32 = 0.0;
        var expected_center = Vec3.zero();
        var total_weighted_center = Vec3.zero();
        for (oc.getSubShapes()) |*s| {
            const child = s.shape.get().?.getSubmergedVolume(s.getLocalTransformNoScale(Vec3.one()), Vec3.one(), surface);
            expected_submerged += child.submerged_volume;
            expected_center = expected_center.add(child.center_of_buoyancy.mulScalar(child.submerged_volume));
            total_weighted_center = total_weighted_center.add(child.center_of_buoyancy.mulScalar(child.total_volume));
        }
        const inner = shape.getSubmergedVolume(oc.getSubShape(1).getLocalTransformNoScale(Vec3.one()), Vec3.one(), surface);
        try expect(inner.submerged_volume > 0.0 and inner.submerged_volume < inner.total_volume);
        try testing.expectEqual(expected_submerged, v.submerged_volume);
        try expect(v.center_of_buoyancy.eql(expected_center.divScalar(expected_submerged)));
        try expect(!v.center_of_buoyancy.isClose(total_weighted_center.divScalar(expected_submerged), .{ .max_dist_sq = 1.0e-2 }));
    }

    // collideSoftBodyVertices: every child (box_a records its calls)
    {
        var record: TestShapes.SoftBodyRecord = .{};
        tc.box_a.soft_body_record = &record;
        defer tc.box_a.soft_body_record = null;
        const vertices: CollideSoftBodyVertexIterator = .{};
        shape.collideSoftBodyVertices(Mat44.identity(), Vec3.one(), &vertices, 5, 2);
        try testing.expectEqual(@as(u32, 1), record.calls);
        try testing.expectEqual(@as(u32, 5), record.num_vertices);
        try testing.expectEqual(@as(i32, 2), record.colliding_shape_index);
    }

    // transformShape: every child with the transform of its sub shape (Shape.impl.transformShape of the children)
    {
        var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collector.deinit();
        const transform = Mat44.rotationTranslation(rotation, position);
        shape.transformShape(transform, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 4), collector.hits.items.len);
        for (collector.hits.items, c.getSubShapes()) |*ts, *s| {
            const t = transform.mul(Mat44.rotationTranslation(s.getRotation(), s.getPositionCOM()));
            try expect(ts.shape.get() == s.shape.get());
            try expect(ts.shape_position_com.eql(RVec3.fromVec3(t.decompose().rotation_translation.getTranslation())));
        }
    }

    // getWorldSpaceBounds: the children's bounds (<= 10 sub shapes)
    {
        const transform = Mat44.rotationTranslation(rotation, position);
        const bounds = shape.getWorldSpaceBounds(transform, Vec3.replicate(2));
        var expected: AABox = .empty;
        for (c.getSubShapes()) |*s|
            expected.encapsulate(s.shape.get().?.getWorldSpaceBounds(transform.mul(s.getLocalTransformNoScale(Vec3.replicate(2))), s.transformScale(Vec3.replicate(2))));
        try expect(bounds.eql(expected));
        try expect(!bounds.eql(Shape.impl.getWorldSpaceBounds(shape, transform, Vec3.replicate(2)))); // Tighter than the transformed local bounds
    }

    // Stats
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    const stats = try shape.getStatsRecursive(allocator, &visited);
    try testing.expectEqual(@as(usize, @sizeOf(LinearCompoundShape) + 4 * @sizeOf(CompoundShape.SubShape) + 3 * @sizeOf(TestBoxShape) + @sizeOf(TestSphereShape)), stats.size_bytes);
    try testing.expectEqual(@as(u32, 3 * 12), stats.num_triangles);
    try testing.expectEqual(@as(u32, 5), visited.count());
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var visited2: Shape.VisitedShapes = .empty;
    defer visited2.deinit(failing.allocator());
    try testing.expectError(error.OutOfMemory, shape.getStatsRecursive(failing.allocator(), &visited2));

    // getTrianglesStart / Next assert (Jolt: "Cannot call on non-leaf shapes"), without asserts they return nothing
    if (!Core.enable_asserts) {
        var context: Shape.GetTrianglesContext = .{};
        shape.getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
        var vertices: [3 * Shape.get_triangles_min_triangles_requested]Float3 = undefined;
        try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&context, Shape.get_triangles_min_triangles_requested, &vertices, null));
    }
}

test "CompoundShape: world space bounds of empty compounds and of more than 10 sub shapes, mustBeStatic, nested sub shape ID bits" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = TestSphereShape.init(allocator, 1.0);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.1), Vec3.init(5, 6, 7));

    // No sub shapes: a point at the translation, 32 bits (n - 1 wraps around), mass 0
    var empty_settings = LinearCompoundShapeSettings.init(allocator);
    defer empty_settings.deinit();
    var empty = try empty_settings.asShapeSettings().createShape(allocator);
    defer empty.deinit();
    const e = empty.getPtr().?;
    try expect(e.getWorldSpaceBounds(transform, Vec3.one()).eql(.init(Vec3.init(5, 6, 7), Vec3.init(5, 6, 7))));
    try testing.expectEqual(@as(u32, 32), e.cast(CompoundShape).getSubShapeIDBits());
    try testing.expectEqual(math.flt_max, e.getInnerRadius());
    try testing.expectEqual(@as(f32, 0.0), e.getMassProperties().mass);
    try expect(e.getCenterOfMass().eql(Vec3.zero()) and e.getLocalBounds().eql(.init(Vec3.zero(), Vec3.zero())));

    // 10 sub shapes: the union of the children, 11: Shape.impl.getWorldSpaceBounds
    var settings = LinearCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    for (0..11) |i| {
        try settings.base.addShapePtr(Vec3.init(@floatFromInt(i), 0, 0), Quat.identity(), sphere.asShape(), .{});
        if (i == 9 or i == 10) {
            settings.asShapeSettings().clearCachedResult();
            var r = try settings.asShapeSettings().createShape(allocator);
            defer r.deinit();
            const s = r.getPtr().?;
            const bounds = s.getWorldSpaceBounds(transform, Vec3.one());
            if (i == 9) {
                var expected: AABox = .empty;
                for (s.cast(CompoundShape).getSubShapes()) |*sub|
                    expected.encapsulate(sub.shape.get().?.getWorldSpaceBounds(transform.mul(sub.getLocalTransformNoScale(Vec3.one())), Vec3.one()));
                try expect(bounds.eql(expected));
            } else {
                try expect(bounds.eql(s.getLocalBounds().scaled(Vec3.one()).transformed(transform)));
            }
            try testing.expectEqual(@as(u32, 4), s.getSubShapeIDBitsRecursive());
        }
    }

    // mustBeStatic and the bits of a nested compound (TestCompoundShape: 1 bit)
    const box = try TestBoxShape.create(allocator, Vec3.one(), .{});
    const nested = try TestCompoundShape.create(allocator, box.asShape(), Vec3.zero(), sphere.asShape(), Vec3.init(3, 0, 0));
    nested.must_be_static = true;
    var with_nested = LinearCompoundShapeSettings.init(allocator);
    defer with_nested.deinit();
    try with_nested.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
    try with_nested.base.addShapePtr(Vec3.init(0, 5, 0), Quat.identity(), nested.asShape(), .{});
    try with_nested.base.addShapePtr(Vec3.init(0, -5, 0), Quat.identity(), sphere.asShape(), .{});
    var r = try with_nested.asShapeSettings().createShape(allocator);
    defer r.deinit();
    try expect(r.getPtr().?.mustBeStatic());
    try testing.expectEqual(@as(u32, 2 + 1), r.getPtr().?.getSubShapeIDBitsRecursive());
    const leaf_id = SubShapeIDCreator.pushID(.{}, 1, 2).pushID(0, 1).getID();
    try expect(r.getPtr().?.getLeafShape(leaf_id).shape == box.asShape());
}

test "CompoundShape: scales (isValidScale, makeScaleValid)" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = TestBoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var sphere = TestSphereShape.init(allocator, 1.0);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    // Boxes, one rotated by 90 degrees: any non zero scale
    var settings = LinearCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), box.asShape(), .{});
    try settings.base.addShapePtr(Vec3.init(3, 0, 0), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), box.asShape(), .{});
    var boxes = try settings.asShapeSettings().createShape(allocator);
    defer boxes.deinit();
    const b = boxes.getPtr().?;
    try expect(b.isValidScale(Vec3.init(1, 2, 3)));
    try expect(!b.isValidScale(Vec3.init(1, 0, 3)));
    try expect(b.makeScaleValid(Vec3.init(1, 2, 3)).eql(Vec3.init(1, 2, 3)));

    // A single box without rotation: the first branch, only zero components are changed
    var single_settings = LinearCompoundShapeSettings.init(allocator);
    defer single_settings.deinit();
    try single_settings.base.addShapePtr(Vec3.init(3, 0, 0), Quat.identity(), box.asShape(), .{});
    var single = try single_settings.asShapeSettings().createShape(allocator);
    defer single.deinit();
    try expect(single.getPtr().?.makeScaleValid(Vec3.init(0, 2, -3)).eql(ScaleHelpers.makeNonZeroScale(Vec3.init(0, 2, -3))));

    // A box rotated by 45 degrees: non uniform scales shear it, makeScaleValid makes the scale uniform (keeps the signs)
    try settings.base.addShapePtr(Vec3.init(-3, 0, 0), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), box.asShape(), .{});
    settings.asShapeSettings().clearCachedResult();
    var rotated = try settings.asShapeSettings().createShape(allocator);
    defer rotated.deinit();
    const r = rotated.getPtr().?;
    try expect(!r.isValidScale(Vec3.init(1, 2, 3)));
    try expect(r.isValidScale(Vec3.replicate(-2)));
    const abs_uniform = ScaleHelpers.makeUniformScale(Vec3.init(1, 2, 3));
    try expect(r.makeScaleValid(Vec3.init(1, 2, 3)).eql(Vec3.one().mul(abs_uniform))); // The second branch (sign * uniform)
    try expect(r.makeScaleValid(Vec3.init(-1, 2, 3)).eql(abs_uniform.mulScalar(-1.0))); // Mixed signs are not uniform: the last branch

    // A sphere child only supports uniform scales (TestSphereShape accepts mixed signs)
    var sphere_settings = LinearCompoundShapeSettings.init(allocator);
    defer sphere_settings.deinit();
    try sphere_settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
    var spheres = try sphere_settings.asShapeSettings().createShape(allocator);
    defer spheres.deinit();
    const s = spheres.getPtr().?;
    try expect(s.isValidScale(Vec3.replicate(2)) and !s.isValidScale(Vec3.init(1, 2, 3)));
    try expect(s.isValidScale(Vec3.init(2, -2, 2)));
    try expect(s.makeScaleValid(Vec3.init(1, 2, 3)).eql(ScaleHelpers.makeUniformScale(Vec3.init(1, 2, 3))));
    try expect(s.makeScaleValid(Vec3.init(-1, 2, 3)).eql(Vec3.init(-1, 1, 1).mul(ScaleHelpers.makeUniformScale(Vec3.init(1, 2, 3)))));

    // TestCompoundShape child: only uniform scales without sign changes, so the last branch is taken
    const nested = try TestCompoundShape.create(allocator, box.asShape(), Vec3.zero(), sphere.asShape(), Vec3.init(3, 0, 0));
    var nested_settings = LinearCompoundShapeSettings.init(allocator);
    defer nested_settings.deinit();
    try nested_settings.base.addShapePtr(Vec3.zero(), Quat.identity(), nested.asShape(), .{});
    var with_nested = try nested_settings.asShapeSettings().createShape(allocator);
    defer with_nested.deinit();
    const n = with_nested.getPtr().?;
    try expect(!n.isValidScale(Vec3.init(-2, 2, 2)));
    const mixed = Vec3.init(-1, 2, 3);
    try expect(n.makeScaleValid(mixed).eql(ScaleHelpers.makeUniformScale(mixed.abs()).mulScalar(-1.0)));
}

test "CompoundShape: sub shape IDs and compound user data" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = TestSphereShape.init(allocator, 1.0);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    var settings = LinearCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    const expected_bits = [_]u32{ 0, 1, 2, 2, 3, 3, 3, 3, 4 };
    for (expected_bits, 1..) |bits, n| {
        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{ .user_data = @intCast(100 + n) });
        settings.asShapeSettings().clearCachedResult();
        var r = try settings.asShapeSettings().createShape(allocator);
        defer r.deinit();
        const c = r.getPtr().?.cast(CompoundShape);
        try testing.expectEqual(bits, c.getSubShapeIDBits());

        const parent = SubShapeIDCreator.pushID(.{}, 5, 3);
        for (0..n) |i| {
            const creator = c.getSubShapeIDFromIndex(@intCast(i), parent);
            try testing.expectEqual(3 + bits, creator.getNumBitsWritten());
            const own = creator.getID().popID(3).remainder; // Remove the parent's bits
            try expect(c.isSubShapeIDValid(own));
            const index = c.getSubShapeIndexFromID(own);
            try testing.expectEqual(@as(u32, @intCast(i)), index.index);
            try expect(index.remainder.isEmpty());
            try testing.expectEqual(@as(u32, @intCast(101 + i)), c.getCompoundUserData(@intCast(i)));
        }

        // An index beyond the sub shapes (when it fits in the bits)
        if (n < (@as(usize, 1) << @intCast(bits)))
            try expect(!c.isSubShapeIDValid(SubShapeIDCreator.pushID(.{}, @intCast(n), bits).getID()));
    }

    // The ID of the last sub shape of a compound can equal the empty ID (index 1 of 2: all bits set)
    {
        var two = LinearCompoundShapeSettings.init(allocator);
        defer two.deinit();
        for (0..2) |_| try two.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
        var r = try two.asShapeSettings().createShape(allocator);
        defer r.deinit();
        try expect(r.getPtr().?.cast(CompoundShape).getSubShapeIDFromIndex(1, .{}).getID().isEmpty());
        try expect(r.getPtr().?.getLeafShape(.empty).shape == sphere.asShape());
    }

    // setCompoundUserData (mutable shape)
    var r = try settings.asShapeSettings().createShape(allocator);
    defer r.deinit();
    r.getPtr().?.castMut(CompoundShape).setCompoundUserData(2, 77);
    try testing.expectEqual(@as(u32, 77), r.getPtr().?.cast(CompoundShape).getCompoundUserData(2));
}

test "CompoundShape: binary state and sub shape state" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var tc = try makeTestCompound(allocator);
    defer tc.deinit();
    const shape = tc.shape();
    const c = tc.compound();
    tc.result.getPtr().?.setUserData(42);

    var buffer: [512]u8 = undefined;
    const bytes = saveToBuffer(shape, &buffer);
    try testing.expectEqual(@as(usize, 1 + 8 + 12 + 12 + 12 + 4 + 4 + 4 * (4 + 12 + 12)), bytes.len);
    try testing.expectEqual(@intFromEnum(ShapeSubType.user4), bytes[0]);

    // Restore into a default constructed compound (Shape.restoreFromBinaryState reads the sub type)
    var restored = LinearCompoundShape.initDefault(allocator);
    restored.asShape().setEmbedded();
    defer restored.asShapeMut().deinit();
    {
        var reader: std.Io.Reader = .fixed(bytes[1..]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        try restored.asShapeMut().restoreBinaryState(in.streamIn());
        try expect(!in.streamIn().isFailed());
    }
    const rc = &restored.base;
    try testing.expectEqual(@as(u64, 42), restored.asShape().getUserData());
    try expect(rc.center_of_mass.eql(c.center_of_mass));
    try expect(rc.local_bounds.eql(c.local_bounds));
    try testing.expectEqual(c.inner_radius, rc.inner_radius);
    try testing.expectEqual(@as(usize, 4), rc.sub_shapes.items.len);
    for (rc.sub_shapes.items, c.getSubShapes()) |*a, *b| {
        try expect(a.shape.get() == null);
        try testing.expectEqual(b.user_data, a.user_data);
        try expect(std.meta.eql(a.position_com, b.position_com) and std.meta.eql(a.rotation, b.rotation));
        try testing.expectEqual(b.is_rotation_identity, a.is_rotation_identity); // Recomputed from the zero rotation
    }

    // Sub shape state: the list is cleared first, restoring sets the shapes
    var sub_shapes: ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try sub_shapes.append(allocator, .init(tc.box_a.asShape()));
    try shape.saveSubShapeState(allocator, &sub_shapes);
    try testing.expectEqual(@as(usize, 4), sub_shapes.items.len);
    for (sub_shapes.items, c.getSubShapes()) |a, *b| try expect(a.get() == b.shape.get());
    restored.asShapeMut().restoreSubShapeState(sub_shapes.items);
    for (rc.sub_shapes.items, c.getSubShapes()) |*a, *b| try expect(a.shape.get() == b.shape.get());
    try testing.expectEqual(@as(u32, 4), tc.box_a.asShape().getRefCount()); // Settings, compound, list, restored

    // The restored compound saves the same bytes
    var buffer2: [512]u8 = undefined;
    try testing.expectEqualSlices(u8, bytes, saveToBuffer(restored.asShape(), &buffer2));

    // Restoring a shorter array releases the sub shapes that are removed; truncated data reports a failed stream
    {
        var two = LinearCompoundShapeSettings.init(allocator);
        defer two.deinit();
        for (0..2) |_| try two.base.addShapePtr(Vec3.zero(), Quat.identity(), tc.sphere.asShape(), .{});
        var r = try two.asShapeSettings().createShape(allocator);
        defer r.deinit();
        var short_buffer: [512]u8 = undefined;
        const short_bytes = saveToBuffer(r.getPtr().?, &short_buffer);
        const sphere_refs = tc.sphere.asShape().getRefCount();
        var reader: std.Io.Reader = .fixed(short_bytes[1..]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        try restored.asShapeMut().restoreBinaryState(in.streamIn());
        try testing.expectEqual(@as(usize, 2), rc.sub_shapes.items.len);
        try testing.expectEqual(@as(u32, 4), tc.box_a.asShape().getRefCount()); // Element 0 is kept (Array::resize)
        try testing.expectEqual(sphere_refs - 1, tc.sphere.asShape().getRefCount()); // Element 3 is released

        var truncated: std.Io.Reader = .fixed(short_bytes[1 .. short_bytes.len - 1]);
        var truncated_in = StreamWrapper.StreamInWrapper.init(&truncated);
        try restored.asShapeMut().restoreBinaryState(truncated_in.streamIn());
        try expect(truncated_in.streamIn().isEOF() or truncated_in.streamIn().isFailed());
    }

    // Out of memory while reading the sub shape array / saving the sub shape state
    {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
        var oom = LinearCompoundShape.initDefault(failing.allocator());
        oom.asShape().setEmbedded();
        defer oom.asShapeMut().deinit();
        var reader: std.Io.Reader = .fixed(bytes[1..]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        try testing.expectError(error.OutOfMemory, oom.asShapeMut().restoreBinaryState(in.streamIn()));
        var list: ShapeList = .empty;
        try testing.expectError(error.OutOfMemory, shape.saveSubShapeState(failing.allocator(), &list));
    }
}

/// A filter that rejects one sub shape ID (as shape 2, or as either shape of a pair)
const RejectIDFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    reject: SubShapeID,

    pub fn shouldCollide(self: *const RejectIDFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = shape2;
        return !sub_shape_id_of_shape2.eql(self.reject);
    }

    pub fn shouldCollidePair(self: *const RejectIDFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ shape1, shape2 };
        return !sub_shape_id_of_shape1.eql(self.reject) and !sub_shape_id_of_shape2.eql(self.reject);
    }
};

test "CompoundShape visitors: ray casts, collide point, collect transformed shapes, intersecting sub shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var tc = try makeTestCompound(allocator);
    defer tc.deinit();
    const shape = tc.shape();
    const c = tc.compound();
    const com = shape.getCenterOfMass();

    // A ray along -X through the center of mass of box_b (the boxes are centered around their center of mass), it also
    // goes through box_a
    const box_b_com = c.getSubShape(1).getPositionCOM();
    const ray = RayCast.init(Vec3.init(10 - com.getX(), box_b_com.getY(), box_b_com.getZ()), Vec3.init(-20, 0, 0));
    var hit: RayCastResult = .{};
    try expect(shape.castRay(ray, .{}, &hit));
    try expect(hit.sub_shape_id2.eql(id2(1)));
    try testing.expectApproxEqAbs(@as(f32, (10.0 - 3.0) / 20.0), hit.fraction, 1.0e-6); // box_b reaches x = 2 + 1 (its Z half extent is along X)
    const first_fraction = hit.fraction;
    try expect(!shape.castRay(ray, .{}, &hit)); // Not closer
    try testing.expectEqual(first_fraction, hit.fraction);
    var miss: RayCastResult = .{};
    try expect(!shape.castRay(.init(Vec3.init(10, 10, 10), Vec3.init(1, 0, 0)), .{}, &miss));
    var inside: RayCastResult = .{};
    try expect(shape.castRay(.init(c.getSubShape(0).getPositionCOM(), Vec3.init(-20, 0, 0)), .{}, &inside)); // Starts in box_a: fraction 0, aborts
    try testing.expectEqual(@as(f32, 0.0), inside.fraction);

    // Collector version: all hits, back faces
    {
        var settings: RayCastSettings = .{};
        settings.setBackFaceMode(.collide_with_back_faces);
        var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
        defer collector.deinit();
        shape.castRayCollector(ray, &settings, .{}, &collector.base, &.{});
        try collector.checkError();
        collector.sort();
        try testing.expectEqual(@as(usize, 4), collector.hits.items.len);
        try expect(collector.hits.items[0].sub_shape_id2.eql(id2(1)) and collector.hits.items[3].sub_shape_id2.eql(id2(0)));

        // A filter that rejects the compound itself
        const RejectAll = struct {
            pub const overrides = .{.shouldCollide};
            base: ShapeFilter = .init(@This()),
            pub fn shouldCollide(_: *const @This(), _: *const Shape, _: SubShapeID) bool {
                return false;
            }
        };
        const reject_all: RejectAll = .{};
        collector.reset();
        shape.castRayCollector(ray, &settings, .{}, &collector.base, &reject_all.base);
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);

        // A filter that rejects sub shape 1 (box_b) only: the visitor passes the filter on to the sub shapes
        const reject_box_b: RejectIDFilter = .{ .reject = id2(1) };
        collector.reset();
        shape.castRayCollector(ray, &settings, .{}, &collector.base, &reject_box_b.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 2), collector.hits.items.len); // The front and back face of box_a
        for (collector.hits.items) |*h| try expect(h.sub_shape_id2.eql(id2(0)));

        // An any hit collector stops after the first hit
        var any = AnyHitCollisionCollector(CastRayCollector).init();
        defer any.deinit();
        shape.castRayCollector(ray, &settings, .{}, &any.base, &.{});
        try expect(any.hadHit() and any.hit.sub_shape_id2.eql(id2(0))); // The first sub shape that the walk visits
    }

    // Collide point: the point is transformed into the space of the child
    {
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        const parent = SubShapeIDCreator.pushID(.{}, 1, 1);
        shape.collidePoint(c.getSubShape(1).getPositionCOM().add(Vec3.init(0.9, 0, 0)), parent, &collector.base, &.{}); // Inside rotated box_b only (its local Z is along X)
        shape.collidePoint(Vec3.init(100, 0, 0), parent, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].sub_shape_id2.eql(c.getSubShapeIDFromIndex(1, parent).getID()));

        // The filter reaches the sub shape (box_b rejected, the compound itself accepted)
        collector.reset();
        const reject_box_b: RejectIDFilter = .{ .reject = c.getSubShapeIDFromIndex(1, parent).getID() };
        shape.collidePoint(c.getSubShape(1).getPositionCOM().add(Vec3.init(0.9, 0, 0)), parent, &collector.base, &reject_box_b.base);
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);
    }

    // Collect transformed shapes: every sub shape in the box, with the transform of the sub shape
    {
        var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collector.deinit();
        const position = Vec3.init(1, 2, 3);
        const rotation = Quat.rotation(Vec3.axisZ(), 0.5 * math.pi);
        const scale = Vec3.replicate(2);
        shape.collectTransformedShapes(AABox.biggest(), position, rotation, scale, .{}, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 4), collector.hits.items.len);
        for (collector.hits.items, c.getSubShapes(), 0..) |*ts, *s, i| {
            try expect(ts.shape.get() == s.shape.get());
            try expect(ts.sub_shape_id_creator.getID().eql(id2(@intCast(i))));
            try expect(ts.shape_position_com.eql(RVec3.fromVec3(position.add(rotation.mulVec3(scale.mul(s.getPositionCOM()))))));
            try expect(ts.shape_rotation.eql(rotation.mul(s.getRotation())));
            try expect(ts.getShapeScale().eql(scale));
        }

        // The filter reaches the sub shapes: every sub shape except box_b (sub shape 1)
        collector.reset();
        const reject_box_b: RejectIDFilter = .{ .reject = id2(1) };
        shape.collectTransformedShapes(AABox.biggest(), position, rotation, scale, .{}, &collector.base, &reject_box_b.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 3), collector.hits.items.len);
        for (collector.hits.items) |*ts| try expect(ts.shape.get() != tc.box_b.asShape());

        // A small world space box around box_a (sub shape 0) only
        collector.reset();
        const box_a_world = position.add(rotation.mulVec3(scale.mul(c.getSubShape(0).getPositionCOM())));
        shape.collectTransformedShapes(.fromCenterAndRadius(box_a_world, 0.1), position, rotation, scale, .{}, &collector.base, &.{});
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].shape.get() == tc.box_a.asShape());
    }

    // getIntersectingSubShapes: AABox and OrientedBox, the buffer limits the results
    {
        var indices: [4]u32 = undefined;
        try testing.expectEqual(@as(u32, 4), c.getIntersectingSubShapes(AABox.biggest(), &indices));
        try testing.expectEqualSlices(u32, &.{ 0, 1, 2, 3 }, &indices);
        try testing.expectEqual(@as(u32, 2), c.getIntersectingSubShapes(AABox.biggest(), indices[0..2]));
        try testing.expectEqual(@as(u32, 0), c.getIntersectingSubShapes(AABox.biggest(), indices[0..0]));
        const around_box_b = AABox.fromCenterAndRadius(c.getSubShape(1).getPositionCOM(), 0.1);
        try testing.expectEqual(@as(u32, 1), c.getIntersectingSubShapes(around_box_b, &indices));
        try testing.expectEqual(@as(u32, 1), indices[0]);
        const obox = OrientedBox.init(Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), c.getSubShape(3).getPositionCOM()), Vec3.replicate(0.1));
        try testing.expectEqual(@as(u32, 1), c.getIntersectingSubShapesOrientedBox(obox, &indices));
        try testing.expectEqual(@as(u32, 3), indices[0]);
        try testing.expectEqual(@as(u32, 4), c.getIntersectingSubShapesOrientedBox(.init(Mat44.identity(), Vec3.replicate(100)), &indices));
    }
}

test "CompoundShape visitors: collide compound vs shape, shape vs compound, cast shape vs compound, castCompoundVsShape" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var tc = try makeTestCompound(allocator);
    defer tc.deinit();
    const shape = tc.shape();
    const c = tc.compound();

    // A box that touches box_a (sub shape 0) and the box from settings (sub shape 2) in the space of the compound
    var probe = TestBoxShape.init(allocator, Vec3.one(), .{});
    probe.asShape().setEmbedded();
    defer probe.asShapeMut().deinit();
    const between = c.getSubShape(0).getPositionCOM().add(c.getSubShape(2).getPositionCOM()).mulScalar(0.5);
    const probe_transform = Mat44.translation(between);
    const settings: CollideShapeSettings = .{};
    const parent = SubShapeIDCreator.pushID(.{}, 3, 2);

    // Compound vs shape: the sub shape ID of shape 1 gets the index
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        LinearCompoundShape.collideCompoundVsShape(shape, probe.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), probe_transform, parent, .{}, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 2), collector.hits.items.len);
        try expect(collector.hits.items[0].sub_shape_id1.eql(c.getSubShapeIDFromIndex(0, parent).getID()));
        try expect(collector.hits.items[1].sub_shape_id1.eql(c.getSubShapeIDFromIndex(2, parent).getID()));
        try expect(collector.hits.items[0].sub_shape_id2.isEmpty());

        // The filter reaches CollisionDispatch: rejecting the pair with box_a leaves the box from settings
        collector.reset();
        const reject_box_a: RejectIDFilter = .{ .reject = c.getSubShapeIDFromIndex(0, parent).getID() };
        LinearCompoundShape.collideCompoundVsShape(shape, probe.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), probe_transform, parent, .{}, &settings, &collector.base, &reject_box_a.base);
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].sub_shape_id1.eql(c.getSubShapeIDFromIndex(2, parent).getID()));

        // With the probe far away nothing collides (the bounds test rejects every sub shape)
        collector.reset();
        LinearCompoundShape.collideCompoundVsShape(shape, probe.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.replicate(50)), .{}, .{}, &settings, &collector.base, &.{});
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);

        // max_separation_distance enlarges the bounds of shape 2 in the visitor
        var far_settings: CollideShapeSettings = .{};
        far_settings.max_separation_distance = 100.0;
        const visitor = CompoundShape.CollideCompoundVsShapeVisitor.init(c, probe.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.replicate(50)), .{}, .{}, &far_settings, &collector.base, &.{});
        try expect(visitor.bounds_of2_in_space_of1.contains(c.local_bounds));
    }

    // Shape vs compound: the sub shape ID of shape 2 gets the index, an any hit collector stops early
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        LinearCompoundShape.collideShapeVsCompound(probe.asShape(), shape, Vec3.one(), Vec3.one(), probe_transform, Mat44.identity(), .{}, parent, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 2), collector.hits.items.len);
        try expect(collector.hits.items[0].sub_shape_id2.eql(c.getSubShapeIDFromIndex(0, parent).getID()));
        try expect(collector.hits.items[1].sub_shape_id2.eql(c.getSubShapeIDFromIndex(2, parent).getID()));

        // The filter reaches CollisionDispatch: rejecting the pair with the box from settings leaves box_a
        collector.reset();
        const reject_box_c: RejectIDFilter = .{ .reject = c.getSubShapeIDFromIndex(2, parent).getID() };
        LinearCompoundShape.collideShapeVsCompound(probe.asShape(), shape, Vec3.one(), Vec3.one(), probe_transform, Mat44.identity(), .{}, parent, &settings, &collector.base, &reject_box_c.base);
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].sub_shape_id2.eql(c.getSubShapeIDFromIndex(0, parent).getID()));

        var any = AnyHitCollisionCollector(CollideShapeCollector).init();
        defer any.deinit();
        LinearCompoundShape.collideShapeVsCompound(probe.asShape(), shape, Vec3.one(), Vec3.one(), probe_transform, Mat44.identity(), .{}, .{}, &settings, &any.base, &.{});
        try expect(any.hadHit() and any.hit.sub_shape_id2.eql(id2(0)));
    }

    // Cast a sphere against the compound (CompoundShape.CastShapeVisitor), the cast is in the space of the compound
    var sphere = TestSphereShape.init(allocator, 0.25);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    {
        const start = Mat44.translation(c.getSubShape(1).getPositionCOM().add(Vec3.init(0, 0, -5)));
        const shape_cast = ShapeCast.init(sphere.asShape(), Vec3.one(), start, Vec3.init(0, 0, 10));
        const cast_settings: ShapeCastSettings = .{};
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        LinearCompoundShape.castShapeVsCompound(&shape_cast, &cast_settings, shape, Vec3.one(), &.{}, Mat44.identity(), .{}, parent, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const h = &collector.hits.items[0];
        try expect(h.base.sub_shape_id2.eql(c.getSubShapeIDFromIndex(1, parent).getID()));
        try testing.expectApproxEqAbs(@as(f32, (5.0 - 0.5 - 0.25) / 10.0), h.fraction, 1.0e-5); // box_b is rotated: its X half extent 0.5 is along Z

        // The filter reaches CollisionDispatch: rejecting box_b leaves nothing
        collector.reset();
        const reject_box_b: RejectIDFilter = .{ .reject = c.getSubShapeIDFromIndex(1, parent).getID() };
        LinearCompoundShape.castShapeVsCompound(&shape_cast, &cast_settings, shape, Vec3.one(), &reject_box_b.base, Mat44.identity(), .{}, parent, &collector.base);
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);
    }

    // Cast the compound (spheres) against a box: castCompoundVsShape
    {
        var sphere_settings = LinearCompoundShapeSettings.init(allocator);
        defer sphere_settings.deinit();
        try sphere_settings.base.addShapePtr(Vec3.init(-2, 0, 0), Quat.identity(), sphere.asShape(), .{});
        try sphere_settings.base.addShapePtr(Vec3.init(2, 0, 0), Quat.identity(), sphere.asShape(), .{});
        try sphere_settings.base.addShapePtr(Vec3.init(0, 0, 3), Quat.identity(), sphere.asShape(), .{});
        var r = try sphere_settings.asShapeSettings().createShape(allocator);
        defer r.deinit();
        const spheres = r.getPtr().?;
        const sc = spheres.cast(CompoundShape);
        const target_transform = Mat44.translation(Vec3.init(0, 0, 10));
        const world_cast = ShapeCast.init(spheres, Vec3.one(), Mat44.identity(), Vec3.init(0, 0, 20));
        const shape_cast = world_cast.postTransformed(target_transform.inversedRotationTranslation());
        const cast_settings: ShapeCastSettings = .{};
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CompoundShape.castCompoundVsShape(&shape_cast, &cast_settings, probe.asShape(), Vec3.one(), &.{}, target_transform, parent, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len); // Only the sphere at z = 3 is in line with the box
        try expect(collector.hits.items[0].base.sub_shape_id1.eql(sc.getSubShapeIDFromIndex(2, parent).getID()));
        try testing.expectApproxEqAbs((10.0 - 1.0 - 0.25 - sc.getSubShape(2).getPositionCOM().getZ()) / 20.0, collector.hits.items[0].fraction, 1.0e-5);

        // The filter reaches CollisionDispatch: rejecting the sphere at z = 3 leaves nothing
        collector.reset();
        const reject_sphere: RejectIDFilter = .{ .reject = sc.getSubShapeIDFromIndex(2, parent).getID() };
        CompoundShape.castCompoundVsShape(&shape_cast, &cast_settings, probe.asShape(), Vec3.one(), &reject_sphere.base, target_transform, parent, .{}, &collector.base);
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);

        // An any hit collector stops after the first sub shape that hits
        var all_in_line = LinearCompoundShapeSettings.init(allocator);
        defer all_in_line.deinit();
        for (0..3) |_| try all_in_line.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
        var in_line = try all_in_line.asShapeSettings().createShape(allocator);
        defer in_line.deinit();
        const in_line_cast = ShapeCast.init(in_line.getPtr().?, Vec3.one(), Mat44.identity(), Vec3.init(0, 0, 20)).postTransformed(target_transform.inversedRotationTranslation());
        var counting = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer counting.deinit();
        CompoundShape.castCompoundVsShape(&in_line_cast, &cast_settings, probe.asShape(), Vec3.one(), &.{}, target_transform, .{}, .{}, &counting.base);
        try testing.expectEqual(@as(usize, 3), counting.hits.items.len);
        var any = AnyHitCollisionCollector(CastShapeCollector).init();
        defer any.deinit();
        CompoundShape.castCompoundVsShape(&in_line_cast, &cast_settings, probe.asShape(), Vec3.one(), &.{}, target_transform, .{}, .{}, &any.base);
        try expect(any.hadHit() and any.hit.base.sub_shape_id1.eql(id2(0)));
    }
}

test "CompoundShape visitors: testBounds of every visitor" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var tc = try makeTestCompound(allocator);
    defer tc.deinit();
    const c = tc.compound();

    const min_x = Vec4.init(-1, 0, 2, -5);
    const min_y = Vec4.init(-1, -2, 0, -5);
    const min_z = Vec4.init(-1, 1, -3, -5);
    const max_x = Vec4.init(1, 1, 3, -4);
    const max_y = Vec4.init(1, 2, 1, -4);
    const max_z = Vec4.init(1, 3, -2, -4);

    // Ray casts: RayAABox4 with the inverse direction
    const ray = RayCast.init(Vec3.init(-10, 0.5, 0.5), Vec3.init(20, 0, 0));
    var hit: RayCastResult = .{};
    const ray_visitor = CompoundShape.CastRayVisitor.init(&ray, c, .{}, &hit);
    const expected_ray = RayAABox.rayAABox4(ray.origin, .init(ray.direction), min_x, min_y, min_z, max_x, max_y, max_z);
    try expect(ray_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(expected_ray));
    try testing.expectApproxEqAbs(@as(f32, 9.0 / 20.0), expected_ray.getX(), 1.0e-6);
    try testing.expectEqual(@as(u32, 2), ray_visitor.sub_shape_bits);
    try expect(!ray_visitor.shouldAbort());
    var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer collector.deinit();
    const ray_settings: RayCastSettings = .{};
    const ray_collector_visitor = CompoundShape.CastRayVisitorCollector.init(&ray, &ray_settings, c, .{}, &collector.base, &.{});
    try expect(ray_collector_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(expected_ray));
    try expect(!ray_collector_visitor.shouldAbort());

    // Collide point: AABox4VsPoint
    var point_collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer point_collector.deinit();
    const point_visitor = CompoundShape.CollidePointVisitor.init(Vec3.init(0.5, 0.5, 1), c, .{}, &point_collector.base, &.{});
    try expect(point_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(UVec4.init(0xffffffff, 0xffffffff, 0, 0)));

    // Cast shape: the boxes are scaled, enlarged by the extent of the cast shape (+ extra convex radius) and tested
    // against a ray from the center of the cast shape
    var sphere = TestSphereShape.init(allocator, 0.5);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    const shape_cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(-10, 0, 0)), Vec3.init(20, 0, 0));
    var cast_settings: ShapeCastSettings = .{};
    cast_settings.extra_convex_radius = 0.25;
    var cast_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer cast_collector.deinit();
    const scale = Vec3.init(2, 1, -1);
    const cast_visitor = CompoundShape.CastShapeVisitor.init(&shape_cast, &cast_settings, c, scale, &.{}, Mat44.identity(), .{}, .{}, &cast_collector.base);
    try expect(cast_visitor.box_center.eql(Vec3.init(-10, 0, 0)));
    try expect(cast_visitor.box_extent.eql(Vec3.replicate(0.75)));
    var scaled = AABox4.aabox4Scale(scale, min_x, min_y, min_z, max_x, max_y, max_z);
    AABox4.aabox4EnlargeWithExtent(Vec3.replicate(0.75), &scaled.min_x, &scaled.min_y, &scaled.min_z, &scaled.max_x, &scaled.max_y, &scaled.max_z);
    const expected_cast = RayAABox.rayAABox4(Vec3.init(-10, 0, 0), .init(shape_cast.direction), scaled.min_x, scaled.min_y, scaled.min_z, scaled.max_x, scaled.max_y, scaled.max_z);
    try expect(cast_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(expected_cast));
    try testing.expectApproxEqAbs(@as(f32, (10.0 - 2.0 - 0.75) / 20.0), expected_cast.getX(), 1.0e-6);

    // Collect transformed shapes: the query box in the local space of the compound (an oriented box) vs the scaled boxes
    var ts_collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer ts_collector.deinit();
    const position = Vec3.init(1, 2, 3);
    const rotation = Quat.rotation(Vec3.axisZ(), 0.3);
    const query = AABox.fromCenterAndRadius(position, 1.0);
    const collect_visitor = CompoundShape.CollectTransformedShapesVisitor.init(query, c, position, rotation, scale, .{}, &ts_collector.base, &.{});
    const local_box = OrientedBox.fromAABox(Mat44.inverseRotationTranslation(rotation, position), query);
    try expect(collect_visitor.local_box.orientation.eql(local_box.orientation) and collect_visitor.local_box.half_extents.eql(local_box.half_extents));
    const scaled_plain = AABox4.aabox4Scale(scale, min_x, min_y, min_z, max_x, max_y, max_z);
    try expect(collect_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(AABox4.aabox4VsOrientedBox(local_box, scaled_plain.min_x, scaled_plain.min_y, scaled_plain.min_z, scaled_plain.max_x, scaled_plain.max_y, scaled_plain.max_z, .{})));

    // Collide compound vs shape / shape vs compound: the bounds of the other shape in the space of the compound
    var probe = TestBoxShape.init(allocator, Vec3.replicate(0.5), .{});
    probe.asShape().setEmbedded();
    defer probe.asShapeMut().deinit();
    var collide_collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collide_collector.deinit();
    var collide_settings: CollideShapeSettings = .{};
    collide_settings.max_separation_distance = 0.1;
    const transform1 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.2), Vec3.init(1, 0, 0));
    const transform2 = Mat44.translation(Vec3.init(0, 3, 0));
    const vs_shape = CompoundShape.CollideCompoundVsShapeVisitor.init(c, probe.asShape(), scale, Vec3.replicate(2), transform1, transform2, .{}, .{}, &collide_settings, &collide_collector.base, &.{});
    var bounds2 = probe.asShape().getLocalBounds().scaled(Vec3.replicate(2)).transformed(transform1.inversedRotationTranslation().mul(transform2));
    bounds2.expandBy(Vec3.replicate(0.1));
    try expect(vs_shape.bounds_of2_in_space_of1.eql(bounds2));
    try expect(vs_shape.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(AABox4.aabox4VsBox(bounds2, scaled_plain.min_x, scaled_plain.min_y, scaled_plain.min_z, scaled_plain.max_x, scaled_plain.max_y, scaled_plain.max_z)));
    const vs_compound = CompoundShape.CollideShapeVsCompoundVisitor.init(probe.asShape(), c, Vec3.replicate(2), scale, transform2, transform1, .{}, .{}, &collide_settings, &collide_collector.base, &.{});
    try expect(vs_compound.bounds_of1_in_space_of2.eql(bounds2));
    try expect(vs_compound.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(AABox4.aabox4VsBox(bounds2, scaled_plain.min_x, scaled_plain.min_y, scaled_plain.min_z, scaled_plain.max_x, scaled_plain.max_y, scaled_plain.max_z)));

    // Get intersecting sub shapes: AABox4VsBox / the oriented box version, unscaled
    var indices: [2]u32 = undefined;
    const aabox_visitor = CompoundShape.GetIntersectingSubShapesVisitor(AABox).init(.init(Vec3.zero(), Vec3.one()), &indices);
    try expect(aabox_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(AABox4.aabox4VsBox(.init(Vec3.zero(), Vec3.one()), min_x, min_y, min_z, max_x, max_y, max_z)));
    const obox = OrientedBox.init(Mat44.rotation(Vec3.axisZ(), 0.7), Vec3.one());
    var obox_visitor = CompoundShape.GetIntersectingSubShapesVisitor(OrientedBox).init(obox, &indices);
    try expect(obox_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).eql(AABox4.aabox4VsOrientedBox(obox, min_x, min_y, min_z, max_x, max_y, max_z, .{})));
    try expect(!obox_visitor.shouldAbort());
    obox_visitor.visitShape(c.getSubShape(0), 7);
    obox_visitor.visitShape(c.getSubShape(0), 3);
    try expect(obox_visitor.shouldAbort());
    try testing.expectEqual(@as(u32, 2), obox_visitor.getNumResults());
    try testing.expectEqualSlices(u32, &.{ 7, 3 }, &indices);
}

test "CompoundShape: register (cast compound vs any shape)" {
    const registry = &RegisterTypes.registry;
    for (ShapeFile.compound_sub_shape_types) |s1| {
        for ([_]ShapeSubType{ .sphere, .box, .convex_hull, .height_field, .mesh, .user1, .empty }) |s2| // The decorated shapes register later and override (compound, decorator)
            try testing.expect(registry.getCastShape(s1, s2) == &CompoundShape.castCompoundVsShape);
    }
    try testing.expect(registry.getCastShape(.user4, .user1) != &CompoundShape.castCompoundVsShape); // Only the compound sub types
}
