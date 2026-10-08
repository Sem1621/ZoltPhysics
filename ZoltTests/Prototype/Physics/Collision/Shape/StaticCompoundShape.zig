//! Port of: Jolt/Physics/Collision/Shape/StaticCompoundShape.h, Jolt/Physics/Collision/Shape/StaticCompoundShape.cpp (prototype, reduced)
//! Status: partial
//! Missing: the bounding volume hierarchy (Node, sPartition, sPartition4, the tree build with the TempAllocator and
//! WalkTree): the prototype's `walkTree` visits every sub shape in index order and the binary state has no nodes, so
//! visiting order and stream format differ from Jolt. The 1 sub shape case with a transform (RotatedTranslatedShape) is
//! not ported.
//!
//! Settings with custom Create logic (D3): `createShape` writes the cache itself and uses
//! `ShapeSettings.constructShape` for the regular case.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const AABox = zolt.AABox;
const Color = zolt.Color;
const Mat44 = zolt.Mat44;
const OrientedBox = zolt.OrientedBox;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;

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
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const CompoundShapeFile = @import("CompoundShape.zig");
const CompoundShape = CompoundShapeFile.CompoundShape;
const CompoundShapeSettings = CompoundShapeFile.CompoundShapeSettings;
const Visitors = @import("CompoundShapeVisitors.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const RayCast = @import("../RayCast.zig").RayCast;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Class that constructs a StaticCompoundShape. Note that if you only want a compound of 1 shape, use a RotatedTranslatedShape instead.
pub const StaticCompoundShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL
    pub const overrides = .{.createShape};

    base: CompoundShapeSettings,

    /// StaticCompoundShapeSettings() on the stack (`defer s.deinit()`)
    pub fn init(allocator: Allocator) StaticCompoundShapeSettings {
        return .{ .base = .init(StaticCompoundShapeSettings, allocator) };
    }

    /// new StaticCompoundShapeSettings()
    pub fn create(allocator: Allocator) Allocator.Error!*StaticCompoundShapeSettings {
        const self = try allocator.create(StaticCompoundShapeSettings);
        self.* = .init(allocator);
        return self;
    }

    pub fn asShapeSettings(self: *StaticCompoundShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    pub fn deinit(self: *StaticCompoundShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings (Create(TempAllocator &) becomes createShapeWithTempAllocator in the real port)
    pub fn createShape(self: *StaticCompoundShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        const cached_result = &self.base.base.cached_result;
        if (cached_result.isEmpty()) {
            errdefer cached_result.clear(); // Out of memory is not cached

            const sub_shapes = self.base.sub_shapes.items;
            if (sub_shapes.len == 0) {
                // It's an error to create a compound with no subshapes (the compound cannot encode this)
                cached_result.setError("Compound needs a sub shape!");
            } else if (sub_shapes.len == 1) {
                // If there's only 1 part we don't need a StaticCompoundShape
                const s = &sub_shapes[0];
                if (s.position.eql(Vec3.zero()) and s.rotation.eql(Quat.identity())) {
                    // No rotation or translation, we can use the shape directly
                    if (s.shape_ptr.get()) |shape_ptr| {
                        cached_result.set(.init(@constCast(shape_ptr))); // const_cast<Shape *>: only the (atomic) reference count is written through it
                    } else if (s.shape.get()) |shape| {
                        var child_result = try shape.createShape(allocator);
                        defer child_result.deinit();
                        cached_result.assign(&child_result);
                    } else {
                        cached_result.setError("Sub shape is null!");
                    }
                } else {
                    // We can use a RotatedTranslatedShape instead (not part of the prototype)
                    cached_result.setError("Prototype: RotatedTranslatedShape is not ported");
                }
            } else {
                // Build a regular compound shape
                try ShapeSettings.constructShape(StaticCompoundShape, self, allocator);
            }
        }
        return cached_result.clone();
    }
};

/// A compound shape, sub shapes can be rotated and translated.
/// Sub shapes cannot be modified once the compound is created, so this shape is suitable for static bodies.
pub const StaticCompoundShape = struct {
    pub const shape_sub_type: ShapeSubType = .static_compound;

    pub const overrides = .{ .castRay, .castRayCollector, .collidePoint, .collectTransformedShapes, .getIntersectingSubShapes, .getIntersectingSubShapesOrientedBox, .getStats };

    base: CompoundShape,

    /// StaticCompoundShape()
    pub fn initDefault(allocator: Allocator) StaticCompoundShape {
        return .{ .base = .init(StaticCompoundShape, allocator, shape_sub_type) };
    }

    /// StaticCompoundShape(const StaticCompoundShapeSettings &, TempAllocator &, ShapeResult &) (without the tree)
    pub fn initFromSettings(self: *StaticCompoundShape, settings: *const StaticCompoundShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base, result);
        const shape_allocator = self.base.base.allocator;

        // Check that there's at least 1 shape
        const num_subshapes = settings.base.sub_shapes.items.len;
        if (num_subshapes < 2) {
            result.setError("Compound needs at least 2 sub shapes, otherwise you should use a RotatedTranslatedShape!");
            return;
        }

        // Keep track of total mass to calculate center of mass
        var mass: f32 = 0.0;

        try self.base.sub_shapes.appendNTimes(shape_allocator, .{}, num_subshapes);
        for (settings.base.sub_shapes.items, self.base.sub_shapes.items) |*shape, *out_shape| {
            // Start constructing the runtime sub shape
            if (!try out_shape.fromSettings(shape, result, allocator))
                return;

            // Calculate mass properties of child
            const child = out_shape.shape.get().?.getMassProperties();

            // Accumulate center of mass
            mass += child.mass;
            self.base.center_of_mass = self.base.center_of_mass.add(out_shape.getPositionCOM().mulScalar(child.mass));
        }

        if (mass > 0.0)
            self.base.center_of_mass = self.base.center_of_mass.divScalar(mass);

        // Cache the inner radius as it can take a while to recursively iterate over all sub shapes
        self.base.calculateInnerRadius();

        // Shift all shapes so that the center of mass is now at the origin and calculate bounds
        for (self.base.sub_shapes.items) |*shape| {
            // Shift the shape so it's centered around our center of mass
            shape.setPositionCOM(shape.getPositionCOM().sub(self.base.center_of_mass));

            // Transform the shape's bounds into our local space
            const transform = Mat44.rotationTranslation(shape.getRotation(), shape.getPositionCOM());
            const shape_bounds = shape.shape.get().?.getWorldSpaceBounds(transform, Vec3.one());

            // Update our local bounds
            self.base.local_bounds.encapsulate(shape_bounds);
        }

        // (prototype: the bounding volume hierarchy is not built)

        // Check if we're not exceeding the amount of sub shape id bits
        if (self.asShape().getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    pub fn asShape(self: *const StaticCompoundShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *StaticCompoundShape) *Shape {
        return &self.base.base;
    }

    /// Visit the sub shapes (prototype: all of them in index order, Jolt walks its tree and tests the bounds)
    fn walkTree(self: *const StaticCompoundShape, visitor: anytype) void {
        for (self.base.sub_shapes.items, 0..) |*sub_shape, i| {
            if (visitor.shouldAbort())
                break;
            visitor.visitShape(sub_shape, @intCast(i));
        }
    }

    // See Shape::CastRay
    pub fn castRay(self: *const StaticCompoundShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        var visitor: Visitors.CastRayVisitor = .init(ray, &self.base, sub_shape_id_creator, hit);
        self.walkTree(&visitor);
        return visitor.return_value;
    }

    // See Shape::CastRay (collector version)
    pub fn castRayCollector(self: *const StaticCompoundShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: Visitors.CastRayVisitorCollector = .init(ray, ray_cast_settings, &self.base, sub_shape_id_creator, collector, shape_filter);
        self.walkTree(&visitor);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const StaticCompoundShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        var visitor: Visitors.CollidePointVisitor = .init(point, &self.base, sub_shape_id_creator, collector, shape_filter);
        self.walkTree(&visitor);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const StaticCompoundShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: Visitors.CollectTransformedShapesVisitor = .init(box, &self.base, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
        self.walkTree(&visitor);
    }

    // See: CompoundShape::GetIntersectingSubShapes (prototype: bounds test per sub shape instead of the tree)
    pub fn getIntersectingSubShapes(self: *const StaticCompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        var num_results: u32 = 0;
        for (self.base.sub_shapes.items, 0..) |*shape, i| {
            if (num_results >= out_sub_shape_indices.len)
                break;
            const bounds = shape.shape.get().?.getWorldSpaceBounds(Mat44.rotationTranslation(shape.getRotation(), shape.getPositionCOM()), Vec3.one());
            if (bounds.overlaps(box)) {
                out_sub_shape_indices[num_results] = @intCast(i);
                num_results += 1;
            }
        }
        return num_results;
    }

    // See: CompoundShape::GetIntersectingSubShapes (oriented box)
    pub fn getIntersectingSubShapesOrientedBox(self: *const StaticCompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32 {
        var num_results: u32 = 0;
        for (self.base.sub_shapes.items, 0..) |*shape, i| {
            if (num_results >= out_sub_shape_indices.len)
                break;
            const bounds = shape.shape.get().?.getWorldSpaceBounds(Mat44.rotationTranslation(shape.getRotation(), shape.getPositionCOM()), Vec3.one());
            if (box.overlapsAABox(bounds, .{})) {
                out_sub_shape_indices[num_results] = @intCast(i);
                num_results += 1;
            }
        }
        return num_results;
    }

    // See Shape::GetStats
    pub fn getStats(self: *const StaticCompoundShape) Shape.Stats {
        return .{ .size_bytes = @sizeOf(StaticCompoundShape) + self.base.sub_shapes.items.len * @sizeOf(CompoundShape.SubShape), .num_triangles = 0 };
    }

    // Helper functions called by CollisionDispatch
    pub fn collideCompoundVsShape(shape1_in: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(StaticCompoundShape);

        var visitor: Visitors.CollideCompoundVsShapeVisitor = .init(&shape1.base, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
        shape1.walkTree(&visitor);
    }

    pub fn collideShapeVsCompound(shape1: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape2 = shape2_in.cast(StaticCompoundShape);

        var visitor: Visitors.CollideShapeVsCompoundVisitor = .init(shape1, &shape2.base, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
        shape2.walkTree(&visitor);
    }

    pub fn castShapeVsCompound(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape = shape_in.cast(StaticCompoundShape);

        var visitor: Visitors.CastShapeVisitor = .init(shape_cast, shape_cast_settings, &shape.base, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
        shape.walkTree(&visitor);
    }

    // Register shape functions with the registry (sRegister)
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.static_compound);
        f.construct = ShapeFunctions.constructor(StaticCompoundShape);
        f.color = Color.orange;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.static_compound, s, collideCompoundVsShape);
            r.registerCollideShape(s, .static_compound, collideShapeVsCompound);
            r.registerCastShape(s, .static_compound, castShapeVsCompound);
        }
    }
};
