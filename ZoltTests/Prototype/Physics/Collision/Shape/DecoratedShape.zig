//! Port of: Jolt/Physics/Collision/Shape/DecoratedShape.h, Jolt/Physics/Collision/Shape/DecoratedShape.cpp (prototype)
//! Status: partial
//! Missing: GetStatsRecursive
//!
//! Abstract class without new virtual functions: it uses `Shape.VTable` and keeps its implementations of Shape's
//! virtual functions in `impl` (D1). It owns its child: `inner_shape: RefConst(Shape)`, released in `destruct` (D2).
//!
//! DecoratedShapeSettings holds the child settings as `Ref(ShapeSettings)` (Jolt: RefConst): creating the child writes
//! the child's cache, so the child must be reachable through a mutable pointer (Rule M, D3).

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const Mat44 = zolt.Mat44;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Vec3 = zolt.Vec3;

const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeType = ShapeFile.ShapeType;
const ShapeList = ShapeFile.ShapeList;
const ShapeRefC = ShapeFile.ShapeRefC;
const SubShapeID = @import("SubShapeID.zig").SubShapeID;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;

/// Class that constructs a DecoratedShape (abstract)
pub const DecoratedShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL
    base: ShapeSettings,
    /// Sub shape (either this or inner_shape_ptr needs to be filled up)
    inner_shape: Ref(ShapeSettings) = .empty,
    /// Sub shape (either this or inner_shape needs to be filled up)
    inner_shape_ptr: RefConst(Shape) = .empty,

    /// Constructor that decorates another shape (DecoratedShapeSettings(const ShapeSettings *)), called with the most derived type
    pub fn init(comptime T: type, allocator: Allocator, inner_shape: ?*ShapeSettings) DecoratedShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator), .inner_shape = .init(inner_shape) };
    }

    /// Constructor that decorates another shape (DecoratedShapeSettings(const Shape *))
    pub fn initPtr(comptime T: type, allocator: Allocator, inner_shape_ptr: ?*const Shape) DecoratedShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator), .inner_shape_ptr = .init(inner_shape_ptr) };
    }

    /// ~DecoratedShapeSettings
    pub fn destruct(self: *DecoratedShapeSettings) void {
        self.inner_shape.deinit();
        self.inner_shape_ptr.deinit();
    }
};

/// Base class for shapes that decorate another shape with extra functionality (e.g. scale, translation etc.)
pub const DecoratedShape = struct {
    /// Abstract class: `Shape.cast(DecoratedShape)` checks the shape type
    pub const shape_type: ShapeType = .decorated;

    base: Shape,
    inner_shape: RefConst(Shape) = .empty,

    /// DecoratedShape(EShapeSubType) / DecoratedShape(EShapeSubType, const Shape *), called with the most derived type
    pub fn init(comptime T: type, allocator: Allocator, shape_sub_type: ShapeSubType, inner_shape: ?*const Shape) DecoratedShape {
        return .{ .base = .init(Shape.vtableFor(T), allocator, .decorated, shape_sub_type), .inner_shape = .init(inner_shape) };
    }

    /// DecoratedShape(EShapeSubType, const DecoratedShapeSettings &, ShapeResult &): creates the child shape, a child
    /// error is copied to `result` (C++ `outResult = child_result`)
    pub fn initFromSettings(self: *DecoratedShape, settings: *const DecoratedShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base);

        // Check that there's a shape
        if (settings.inner_shape.get() == null and settings.inner_shape_ptr.get() == null) {
            result.setError("Inner shape is null!");
            return;
        }

        if (settings.inner_shape_ptr.get()) |inner_shape_ptr| {
            // Use provided shape
            self.inner_shape.set(inner_shape_ptr);
        } else {
            // Create child shape
            var child_result = try settings.inner_shape.get().?.createShape(allocator);
            defer child_result.deinit();
            if (!child_result.isValid()) {
                result.assign(&child_result);
                return;
            }
            self.inner_shape.set(child_result.getPtr());
        }
    }

    /// ~DecoratedShape
    pub fn destruct(self: *DecoratedShape) void {
        self.inner_shape.deinit();
    }

    /// Access to the decorated inner shape
    pub fn getInnerShape(self: *const DecoratedShape) *const Shape {
        return self.inner_shape.get().?;
    }

    /// DecoratedShape's implementations of Shape's virtual functions
    pub const impl = struct {
        // See Shape::MustBeStatic
        pub fn mustBeStatic(self: *const DecoratedShape) bool {
            return self.getInnerShape().mustBeStatic();
        }

        // See Shape::GetCenterOfMass
        pub fn getCenterOfMass(self: *const DecoratedShape) Vec3 {
            return self.getInnerShape().getCenterOfMass();
        }

        // See Shape::GetSubShapeIDBitsRecursive
        pub fn getSubShapeIDBitsRecursive(self: *const DecoratedShape) u32 {
            return self.getInnerShape().getSubShapeIDBitsRecursive();
        }

        // See Shape::GetLeafShape
        pub fn getLeafShape(self: *const DecoratedShape, sub_shape_id: SubShapeID) Shape.LeafShape {
            return self.getInnerShape().getLeafShape(sub_shape_id);
        }

        // See Shape::GetMaterial
        pub fn getMaterial(self: *const DecoratedShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
            return self.getInnerShape().getMaterial(sub_shape_id);
        }

        // See Shape::GetSupportingFace
        pub fn getSupportingFace(self: *const DecoratedShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
            self.getInnerShape().getSupportingFace(sub_shape_id, direction, scale, center_of_mass_transform, out_vertices);
        }

        // See Shape::GetSubShapeUserData
        pub fn getSubShapeUserData(self: *const DecoratedShape, sub_shape_id: SubShapeID) u64 {
            return self.getInnerShape().getSubShapeUserData(sub_shape_id);
        }

        // See Shape::SaveSubShapeState
        pub fn saveSubShapeState(self: *const DecoratedShape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
            for (out_sub_shapes.items) |*s| s.deinit();
            out_sub_shapes.clearRetainingCapacity();
            try out_sub_shapes.append(allocator, self.inner_shape.clone());
        }

        // See Shape::RestoreSubShapeState
        pub fn restoreSubShapeState(self: *DecoratedShape, sub_shapes: []const ShapeRefC) void {
            std.debug.assert(sub_shapes.len == 1);
            self.inner_shape.set(sub_shapes[0].get());
        }

        // See Shape::IsValidScale
        pub fn isValidScale(self: *const DecoratedShape, scale: Vec3) bool {
            return self.getInnerShape().isValidScale(scale);
        }

        // See Shape::MakeScaleValid
        pub fn makeScaleValid(self: *const DecoratedShape, scale: Vec3) Vec3 {
            return self.getInnerShape().makeScaleValid(scale);
        }
    };
};
