//! Port of: Jolt/Physics/Collision/Shape/CompoundShape.h, Jolt/Physics/Collision/Shape/CompoundShape.cpp (prototype)
//! Status: partial
//! Missing: GetSubShapeTransformedShape, GetSubmergedVolume, CollideSoftBodyVertices, GetStatsRecursive, JPH_DEBUG_RENDERER
//!
//! An abstract class that adds virtual functions (D1): `CompoundShape.VTable = struct { base: Shape.VTable,
//! getIntersectingSubShapes, getIntersectingSubShapesOrientedBox }` (the two C++ overloads get distinct names, the
//! `uint *outSubShapeIndices, int inMaxSubShapeIndices` pair becomes a slice). Its implementations of Shape's virtual
//! functions are in `impl`. It owns its children (`SubShape.shape: RefConst(Shape)`) and the array that holds them
//! (allocated with `base.allocator`), released in `destruct`.
//!
//! RestoreBinaryState reads the sub shape array (StreamIn.readArrayWith), so it returns `Allocator.Error!void` (D11).

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const Core = zolt.Core;
const math = zolt.math;
const AABox = zolt.AABox;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const OrientedBox = zolt.OrientedBox;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

const virtual = @import("../../../Core/Virtual.zig");
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeType = ShapeFile.ShapeType;
const ShapeList = ShapeFile.ShapeList;
const ShapeRefC = ShapeFile.ShapeRefC;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// Base class settings to construct a compound shape (abstract)
pub const CompoundShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT
    base: ShapeSettings,
    /// The sub shapes (allocated with base.allocator)
    sub_shapes: std.ArrayList(SubShapeSettings) = .empty,

    pub const SubShapeSettings = struct {
        /// Sub shape (either this or shape_ptr needs to be filled up). Ref instead of RefConst: creating it writes its cache.
        shape: Ref(ShapeSettings) = .empty,
        /// Sub shape (either this or shape needs to be filled up)
        shape_ptr: RefConst(Shape) = .empty,
        /// Position of the sub shape
        position: Vec3 = Vec3.zero(),
        /// Rotation of the sub shape
        rotation: Quat = Quat.identity(),
        /// User data value (can be used by the application for any purpose)
        user_data: u32 = 0,

        /// Destructor
        pub fn deinit(self: *SubShapeSettings) void {
            self.shape.deinit();
            self.shape_ptr.deinit();
        }
    };

    /// Constructor, called by the derived settings with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) CompoundShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator) };
    }

    /// ~CompoundShapeSettings
    pub fn destruct(self: *CompoundShapeSettings) void {
        for (self.sub_shapes.items) |*s| s.deinit();
        self.sub_shapes.deinit(self.base.allocator);
    }

    /// Add a shape to the compound (AddShape(Vec3Arg, QuatArg, const ShapeSettings *, uint32 = 0))
    pub fn addShape(self: *CompoundShapeSettings, position: Vec3, rotation: Quat, shape: *ShapeSettings, opts: struct { user_data: u32 = 0 }) Allocator.Error!void {
        try self.sub_shapes.append(self.base.allocator, .{ .shape = .init(shape), .position = position, .rotation = rotation, .user_data = opts.user_data });
    }

    /// Add a shape to the compound. Variant that uses a concrete shape (AddShape(Vec3Arg, QuatArg, const Shape *, uint32 = 0))
    pub fn addShapePtr(self: *CompoundShapeSettings, position: Vec3, rotation: Quat, shape: *const Shape, opts: struct { user_data: u32 = 0 }) Allocator.Error!void {
        try self.sub_shapes.append(self.base.allocator, .{ .shape_ptr = .init(shape), .position = position, .rotation = rotation, .user_data = opts.user_data });
    }
};

/// Base class for a compound shape
pub const CompoundShape = struct {
    /// Abstract class: `Shape.cast(CompoundShape)` checks the shape type
    pub const shape_type: ShapeType = .compound;

    pub const VTable = struct {
        base: Shape.VTable,
        /// Get which sub shape's bounding boxes overlap with an axis aligned box
        /// @param box The axis aligned box to test against (relative to the center of mass of this shape)
        /// @param out_sub_shape_indices Buffer where to place the indices of the sub shapes that intersect (its length is Jolt's inMaxSubShapeIndices)
        /// @return How many indices were placed in out_sub_shape_indices
        getIntersectingSubShapes: *const fn (self: *const CompoundShape, box: AABox, out_sub_shape_indices: []u32) u32,
        /// Get which sub shape's bounding boxes overlap with an oriented box
        getIntersectingSubShapesOrientedBox: *const fn (self: *const CompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32,
    };

    /// Class that holds a sub shape
    pub const SubShape = struct {
        shape: RefConst(Shape) = .empty,
        /// Note: Position of center of mass of sub shape!
        position_com: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// Note: X, Y, Z of rotation quaternion
        rotation: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// User data value (put here because it falls in padding bytes)
        user_data: u32 = 0,
        /// If rotation is close to identity (put here because it falls in padding bytes)
        is_rotation_identity: bool = false,

        comptime {
            if (@sizeOf(usize) == 8) std.debug.assert(@sizeOf(SubShape) == 40); // Compiler added unexpected padding
        }

        /// Destructor (releases the shape)
        pub fn deinit(self: *SubShape) void {
            self.shape.deinit();
        }

        /// Initialize sub shape from sub shape settings. Returns false (and sets the error in `result`) when the child
        /// shape could not be created.
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
        pub fn setTransform(self: *SubShape, position: Vec3, rotation: Quat, center_of_mass: Vec3) void {
            self.setPositionCOM(position.sub(center_of_mass).add(rotation.mulVec3(self.shape.get().?.getCenterOfMass())));

            self.is_rotation_identity = rotation.isClose(Quat.identity(), .{}) or rotation.isClose(Quat.identity().negate(), .{});
            self.setRotation(if (self.is_rotation_identity) Quat.identity() else rotation);
        }

        /// Get the local transform for this shape given the scale of the child shape
        pub fn getLocalTransformNoScale(self: *const SubShape, scale: Vec3) Mat44 {
            std.debug.assert(self.isValidScale(scale));
            return Mat44.rotationTranslation(self.getRotation(), scale.mul(self.getPositionCOM()));
        }

        /// Test if inScale is valid for this sub shape
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

    /// Result of getSubShapeIndexFromID (the C++ returns the index and writes outRemainder)
    pub const SubShapeIndex = struct { index: u32, remainder: SubShapeID };

    base: Shape,
    /// Center of mass of the compound
    center_of_mass: Vec3 = Vec3.zero(),
    local_bounds: AABox = .init(Vec3.zero(), Vec3.zero()),
    /// Sub shapes (allocated with base.allocator)
    sub_shapes: std.ArrayList(SubShape) = .empty,
    /// Smallest radius of GetInnerRadius() of child shapes
    inner_radius: f32 = math.flt_max,

    /// CompoundShape(EShapeSubType): called by the concrete classes with their own type
    pub fn init(comptime T: type, allocator: Allocator, shape_sub_type: ShapeSubType) CompoundShape {
        return .{ .base = .init(&vtableFor(T).base, allocator, .compound, shape_sub_type) };
    }

    /// The part of CompoundShape(EShapeSubType, const ShapeSettings &, ShapeResult &) that reads the settings
    pub fn initFromSettings(self: *CompoundShape, settings: *const CompoundShapeSettings, result: *ShapeResult) void {
        _ = result;
        self.base.initFromSettings(&settings.base);
    }

    /// The vtable of concrete compound class `T`
    pub fn vtableFor(comptime T: type) *const VTable {
        return &struct {
            const vt = virtual.make(VTable, T);
        }.vt;
    }

    /// ~CompoundShape
    pub fn destruct(self: *CompoundShape) void {
        for (self.sub_shapes.items) |*s| s.deinit();
        self.sub_shapes.deinit(self.base.allocator);
    }

    fn getVTable(self: *const CompoundShape) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    // Virtual dispatchers of the functions this class adds
    pub fn getIntersectingSubShapes(self: *const CompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        return self.getVTable().getIntersectingSubShapes(self, box, out_sub_shape_indices);
    }

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

    /// Check if a sub shape ID is still valid for this shape
    pub fn isSubShapeIDValid(self: *const CompoundShape, sub_shape_id: SubShapeID) bool {
        return sub_shape_id.popID(self.getSubShapeIDBits()).id < self.sub_shapes.items.len;
    }

    /// Convert SubShapeID to sub shape index
    pub fn getSubShapeIndexFromID(self: *const CompoundShape, sub_shape_id: SubShapeID) SubShapeIndex {
        const r = sub_shape_id.popID(self.getSubShapeIDBits());
        std.debug.assert(r.id < self.sub_shapes.items.len); // Invalid SubShapeID
        return .{ .index = r.id, .remainder = r.remainder };
    }

    /// Convert a sub shape index to a sub shape ID
    pub fn getSubShapeIDFromIndex(self: *const CompoundShape, idx: u32, parent_sub_shape_id: SubShapeIDCreator) SubShapeIDCreator {
        return parent_sub_shape_id.pushID(idx, self.getSubShapeIDBits());
    }

    /// Determine amount of bits needed to encode sub shape id
    pub fn getSubShapeIDBits(self: *const CompoundShape) u32 {
        // Ensure we have enough bits to encode our shape [0, n - 1]
        const n: u32 = @as(u32, @intCast(self.sub_shapes.items.len)) -% 1;
        return 32 - math.countLeadingZeros(n);
    }

    /// Determine the inner radius of this shape
    pub fn calculateInnerRadius(self: *CompoundShape) void {
        self.inner_radius = math.flt_max;
        for (self.sub_shapes.items) |*s|
            self.inner_radius = math.min(self.inner_radius, s.shape.get().?.getInnerRadius());
    }

    /// CompoundShape's implementations of Shape's virtual functions
    pub const impl = struct {
        pub fn getCenterOfMass(self: *const CompoundShape) Vec3 {
            return self.center_of_mass;
        }

        pub fn mustBeStatic(self: *const CompoundShape) bool {
            for (self.sub_shapes.items) |*shape|
                if (shape.shape.get().?.mustBeStatic())
                    return true;

            return false;
        }

        pub fn getLocalBounds(self: *const CompoundShape) AABox {
            return self.local_bounds;
        }

        pub fn getSubShapeIDBitsRecursive(self: *const CompoundShape) u32 {
            // Add max of child bits to our bits
            var child_bits: u32 = 0;
            for (self.sub_shapes.items) |*shape|
                child_bits = @max(child_bits, shape.shape.get().?.getSubShapeIDBitsRecursive());
            return child_bits + self.getSubShapeIDBits();
        }

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

        pub fn getInnerRadius(self: *const CompoundShape) f32 {
            return self.inner_radius;
        }

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

        pub fn getMaterial(self: *const CompoundShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);

            // Pass call on
            return self.sub_shapes.items[index.index].shape.get().?.getMaterial(index.remainder);
        }

        pub fn getLeafShape(self: *const CompoundShape, sub_shape_id: SubShapeID) Shape.LeafShape {
            // Decode sub shape index
            const r = sub_shape_id.popID(self.getSubShapeIDBits());
            if (r.id >= self.sub_shapes.items.len) {
                // No longer valid index
                return .{ .shape = null, .remainder = .empty };
            }

            // Pass call on
            return self.sub_shapes.items[r.id].shape.get().?.getLeafShape(r.remainder);
        }

        pub fn getSubShapeUserData(self: *const CompoundShape, sub_shape_id: SubShapeID) u64 {
            // Decode sub shape index
            const r = sub_shape_id.popID(self.getSubShapeIDBits());
            if (r.id >= self.sub_shapes.items.len)
                return 0; // No longer valid index

            // Pass call on
            return self.sub_shapes.items[r.id].shape.get().?.getSubShapeUserData(r.remainder);
        }

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

        pub fn getSupportingFace(self: *const CompoundShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
            // Decode sub shape index
            const index = self.getSubShapeIndexFromID(sub_shape_id);

            // Apply transform and pass on to sub shape
            const shape = &self.sub_shapes.items[index.index];
            const transform = shape.getLocalTransformNoScale(scale);
            shape.shape.get().?.getSupportingFace(index.remainder, transform.multiply3x3Transposed(direction), shape.transformScale(scale), center_of_mass_transform.mul(transform), out_vertices);
        }

        pub fn transformShape(self: *const CompoundShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
            for (self.sub_shapes.items) |*shape|
                shape.shape.get().?.transformShape(center_of_mass_transform.mul(Mat44.rotationTranslation(shape.getRotation(), shape.getPositionCOM())), collector);
        }

        pub fn getTrianglesStart(self: *const CompoundShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
            _ = .{ self, context, box, position_com, rotation, scale };
            if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
        }

        pub fn getTrianglesNext(self: *const CompoundShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
            _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
            if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
            return 0;
        }

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

        pub fn saveSubShapeState(self: *const CompoundShape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
            for (out_sub_shapes.items) |*s| s.deinit();
            out_sub_shapes.clearRetainingCapacity();
            try out_sub_shapes.ensureTotalCapacity(allocator, self.sub_shapes.items.len);
            for (self.sub_shapes.items) |*shape|
                out_sub_shapes.appendAssumeCapacity(shape.shape.clone());
        }

        pub fn restoreSubShapeState(self: *CompoundShape, sub_shapes: []const ShapeRefC) void {
            std.debug.assert(self.sub_shapes.items.len == sub_shapes.len);
            for (self.sub_shapes.items, sub_shapes) |*shape, sub_shape|
                shape.shape.set(sub_shape.get());
        }

        pub fn getVolume(self: *const CompoundShape) f32 {
            var volume: f32 = 0.0;
            for (self.sub_shapes.items) |*shape|
                volume += shape.shape.get().?.getVolume();
            return volume;
        }

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

        pub fn makeScaleValid(self: *const CompoundShape, scale_in: Vec3) Vec3 {
            const scale = ScaleHelpers.makeNonZeroScale(scale_in);
            if (impl.isValidScale(self, scale)) // C++ qualified call CompoundShape::IsValidScale
                return scale;

            const abs_uniform_scale = ScaleHelpers.makeUniformScale(scale.abs());
            const uniform_scale = scale.getSign().mul(abs_uniform_scale);
            if (impl.isValidScale(self, uniform_scale))
                return uniform_scale;

            return abs_uniform_scale.mulScalar(math.sign(scale.getX()));
        }
    };

    /// Helper function called by CollisionDispatch (sCastCompoundVsShape)
    pub fn castCompoundVsShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        // Fetch compound shape from cast shape
        const compound = shape_cast.shape.cast(CompoundShape);

        // Determine amount of bits for sub shape
        const sub_shape_bits = compound.getSubShapeIDBits();

        // Recurse to sub shapes
        for (compound.sub_shapes.items, 0..) |*sub_shape, i| {
            // Create ID for sub shape
            const shape1_sub_shape_id = sub_shape_id_creator1.pushID(@intCast(i), sub_shape_bits);

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
