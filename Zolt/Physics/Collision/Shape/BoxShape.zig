//! Port of: Jolt/Physics/Collision/Shape/BoxShape.h, Jolt/Physics/Collision/Shape/BoxShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2, SphereShape.zig is the reference):
//! `overrides` lists every C++ `override` in header order, the support class `Box` is constructed in the caller's
//! SupportBuffer (D9), the static `sUnitBoxTriangles` is a comptime table (D10). GetSubmergedVolume is ConvexShape's.
//! JPH_DEBUG_RENDERER (Draw) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RayInvDirection = RayAABox.RayInvDirection;
const math = @import("../../../Math/Math.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const PhysicsSettings = @import("../../PhysicsSettings.zig");
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const ConvexShapeFile = @import("ConvexShape.zig");
const ConvexShape = ConvexShapeFile.ConvexShape;
const ConvexShapeSettings = ConvexShapeFile.ConvexShapeSettings;
const GetTrianglesContextVertexList = @import("GetTrianglesContext.zig").GetTrianglesContextVertexList;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Class that constructs a BoxShape
pub const BoxShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, BoxShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    /// Half the size of the box (including convex radius)
    half_extent: Vec3 = Vec3.zero(),
    convex_radius: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) BoxShapeSettings {
        return .{ .base = .init(BoxShapeSettings, allocator, null) };
    }

    /// Create a box with half edge length half_extent and convex radius convex_radius.
    /// (internally the convex radius will be subtracted from the half extent so the total box will not grow with the convex radius).
    pub fn init(allocator: Allocator, half_extent: Vec3, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) BoxShapeSettings {
        return .{ .base = .init(BoxShapeSettings, allocator, opts.material), .half_extent = half_extent, .convex_radius = opts.convex_radius };
    }

    /// new BoxShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, half_extent: Vec3, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!*BoxShapeSettings {
        const self = try allocator.create(BoxShapeSettings);
        self.* = .init(allocator, half_extent, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *BoxShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *BoxShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *BoxShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(BoxShape, self, allocator);
    }
};

/// Triangles that make up a box (static sUnitBoxTriangles)
const unit_box_triangles = [_]Vec3{
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

/// A box, centered around the origin
pub const BoxShape = struct {
    /// Concrete class: `Shape.cast(BoxShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .box;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .restoreBinaryState };

    base: ConvexShape,
    /// Half the size of the box (including convex radius)
    half_extent: Vec3 = Vec3.zero(),
    convex_radius: f32 = 0.0,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// BoxShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) BoxShape {
        return .{ .base = .init(BoxShape, allocator, shape_sub_type, null) };
    }

    /// BoxShape(const BoxShapeSettings &inSettings, ShapeResult &outResult): C++ member initializers first, then the body
    pub fn initFromSettings(self: *BoxShape, settings: *const BoxShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.half_extent = settings.half_extent;
        self.convex_radius = math.min(settings.convex_radius, settings.half_extent.reduceMin());

        // Check half extents
        if (settings.half_extent.reduceMin() < 0.0) {
            result.setError("Invalid half extent");
            return;
        }

        // Check convex radius
        if (settings.convex_radius < 0.0) {
            result.setError("Invalid convex radius");
            return;
        }

        // Result is valid
        result.set(.init(self.asShapeMut()));
    }

    /// Create a box with half edge length half_extent and convex radius convex_radius.
    /// (internally the convex radius will be subtracted from the half extent so the total box will not grow with the convex radius).
    /// On the stack / as a member: `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end.
    pub fn init(allocator: Allocator, half_extent: Vec3, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) BoxShape {
        std.debug.assert(half_extent.reduceMin() >= 0.0);
        std.debug.assert(opts.convex_radius >= 0.0);
        return .{ .base = .init(BoxShape, allocator, shape_sub_type, opts.material), .half_extent = half_extent, .convex_radius = math.min(opts.convex_radius, half_extent.reduceMin()) };
    }

    /// new BoxShape(...): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, half_extent: Vec3, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!*BoxShape {
        const self = try allocator.create(BoxShape);
        self.* = .init(allocator, half_extent, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const BoxShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *BoxShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get half extent of box
    pub fn getHalfExtent(self: *const BoxShape) Vec3 {
        return self.half_extent;
    }

    /// Get the convex radius of this box
    pub fn getConvexRadius(self: *const BoxShape) f32 {
        return self.convex_radius;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const BoxShape) AABox {
        return .init(self.half_extent.negate(), self.half_extent);
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const BoxShape) f32 {
        return self.half_extent.reduceMin();
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const BoxShape) MassProperties {
        var p: MassProperties = .{};
        p.setMassAndInertiaOfSolidBox(self.half_extent.mulScalar(2.0), self.base.getDensity());
        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const BoxShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        // Get component that is closest to the surface of the box
        const index = local_surface_position.abs().sub(self.half_extent).abs().getLowestComponentIndex();

        // Calculate normal
        var normal = Vec3.zero();
        normal.setComponent(index, if (local_surface_position.getComponent(index) > 0.0) 1.0 else -1.0);
        return normal;
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const BoxShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        const scaled_half_extent = scale.abs().mul(self.half_extent);
        const box = AABox.init(scaled_half_extent.negate(), scaled_half_extent);
        box.getSupportingFace(direction, out_vertices) catch unreachable; // A StaticArray has an empty error set

        // Transform to world space
        for (out_vertices.slice()) |*v|
            v.* = center_of_mass_transform.mulVec3(v.*);
    }

    // See ConvexShape::GetSupportFunction
    pub fn getSupportFunction(self: *const BoxShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        // Scale our half extents
        const scaled_half_extent = scale.abs().mul(self.half_extent);

        switch (mode) {
            .include_convex_radius, .default => {
                // Make box out of our half extents
                const box = AABox.init(scaled_half_extent.negate(), scaled_half_extent);
                std.debug.assert(box.isValid());
                const support = buffer.emplace(Box);
                support.* = .init(box, 0.0);
                return &support.base;
            },

            .exclude_convex_radius => {
                // Reduce the box by our convex radius
                const convex_radius = ScaleHelpers.scaleConvexRadius(self.convex_radius, scale);
                const convex_radius3 = Vec3.replicate(convex_radius);
                const reduced_half_extent = scaled_half_extent.sub(convex_radius3);
                const box = AABox.init(reduced_half_extent.negate(), reduced_half_extent);
                std.debug.assert(box.isValid());
                const support = buffer.emplace(Box);
                support.* = .init(box, convex_radius);
                return &support.base;
            },
        }
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay
    pub fn castRay(self: *const BoxShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Test hit against box
        const fraction = math.max(RayAABox.rayAABox(ray.origin, RayInvDirection.init(ray.direction), self.half_extent.negate(), self.half_extent), @as(f32, 0.0));
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const BoxShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const r = RayAABox.rayAABoxMinMax(ray.origin, RayInvDirection.init(ray.direction), self.half_extent.negate(), self.half_extent);
        const min_fraction = r.min;
        const max_fraction = r.max;
        if (min_fraction <= max_fraction // Ray should intersect
        and max_fraction >= 0.0 // End of ray should be inside box
        and min_fraction < collector.getEarlyOutFraction()) // Start of ray should be before early out fraction
        {
            // Better hit than the current hit
            var hit: RayCastResult = .{};
            hit.body_id = TransformedShape.getBodyID(collector.getContext());
            hit.sub_shape_id2 = sub_shape_id_creator.getID();

            // Check front side
            if (ray_cast_settings.treat_convex_as_solid or min_fraction > 0.0) {
                hit.fraction = math.max(@as(f32, 0.0), min_fraction);
                collector.addHit(&hit);
            }

            // Check back side hit
            if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and max_fraction < collector.getEarlyOutFraction()) {
                hit.fraction = max_fraction;
                collector.addHit(&hit);
            }
        }
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const BoxShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        if (Vec3.lessOrEqual(point.abs(), self.half_extent).testAllXYZTrue())
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const BoxShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        const inverse_transform = center_of_mass_transform.inversedRotationTranslation();
        const half_extent = scale.abs().mul(self.half_extent);

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                // Convert to local space
                const local_pos = inverse_transform.mulVec3(v.getPosition());

                // Clamp point to inside box
                const clamped_point = Vec3.max(Vec3.min(local_pos, half_extent), half_extent.negate());

                // Test if point was inside
                if (clamped_point.eql(local_pos)) {
                    // Calculate closest distance to surface
                    const delta = half_extent.sub(local_pos.abs());
                    const index = delta.getLowestComponentIndex();
                    const penetration = delta.getComponent(index);
                    if (v.updatePenetration(penetration)) {
                        // Calculate contact point and normal
                        const possible_normals = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() };
                        const normal = local_pos.getSign().mul(possible_normals[index]);
                        const point = normal.mul(half_extent);

                        // Store collision
                        v.setCollision(Plane.fromPointAndNormal(point, normal).getTransformed(center_of_mass_transform), colliding_shape_index);
                    }
                } else {
                    // Calculate normal
                    var normal = local_pos.sub(clamped_point);
                    const normal_length = normal.length();

                    // Penetration will be negative since we're not penetrating
                    const penetration = -normal_length;
                    if (v.updatePenetration(penetration)) {
                        normal = normal.divScalar(normal_length);

                        // Store collision
                        v.setCollision(Plane.fromPointAndNormal(clamped_point, normal).getTransformed(center_of_mass_transform), colliding_shape_index);
                    }
                }
            }
        }
    }

    // See Shape::GetTrianglesStart
    pub fn getTrianglesStart(self: *const BoxShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        context.emplace(GetTrianglesContextVertexList).* = .init(position_com, rotation, scale, Mat44.scaleVec3(self.half_extent), &unit_box_triangles, self.base.getConvexMaterial());
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const BoxShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    // See Shape::SaveBinaryState
    pub fn saveBinaryState(self: *const BoxShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.half_extent);
        stream.write(self.convex_radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const BoxShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(BoxShape), 12);
    }

    // See Shape::GetVolume (C++ unqualified GetLocalBounds(): a static call is equivalent in a final class)
    pub fn getVolume(self: *const BoxShape) f32 {
        return self.getLocalBounds().getVolume();
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *BoxShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.half_extent);
        stream.read(&self.convex_radius);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.box);
        f.construct = ShapeFunctions.constructor(BoxShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Class for GetSupportFunction (`class Box final : public Support`)

    const Box = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        box: AABox,
        convex_radius: f32,

        fn init(box: AABox, convex_radius: f32) Box {
            return .{ .base = .init(Box), .box = box, .convex_radius = convex_radius };
        }

        pub fn getSupport(self: *const Box, direction: Vec3) Vec3 {
            return self.box.getSupport(direction);
        }

        pub fn getConvexRadius(self: *const Box) f32 {
            return self.convex_radius;
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's own box tests are in ZoltTests/Physics, the bit exact comparison with Jolt in
// ZoltParity/Physics/ConvexParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;

test "BoxShape: settings, Jolt's error texts, convex radius and out of memory (the settings part of TestBoxShape)" {
    const allocator = testing.allocator;

    {
        // Check half extents must be positive
        var box_settings = BoxShapeSettings.init(allocator, Vec3.init(-1, 1, 1), .{});
        defer box_settings.deinit();
        var result = try box_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid half extent", result.getError());
    }

    {
        // Check convex radius must be positive
        var box_settings = BoxShapeSettings.init(allocator, Vec3.replicate(1.0), .{ .convex_radius = -1.0 });
        defer box_settings.deinit();
        var result = try box_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid convex radius", result.getError());
    }

    {
        // Create zero sized box
        var box_settings = BoxShapeSettings.init(allocator, Vec3.zero(), .{ .convex_radius = 1.0 });
        defer box_settings.deinit();
        var result = try box_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const box = result.getPtr().?.cast(BoxShape);

        // Create another box by using a different constructor
        var box2 = try BoxShape.create(allocator, Vec3.zero(), .{ .convex_radius = 1.0 });
        var box2_ref = Ref(Shape).init(box2.asShapeMut());
        defer box2_ref.deinit();

        // Check convex radius is adjusted to zero
        try testing.expectEqual(@as(f32, 0.0), box.getConvexRadius());
        try testing.expectEqual(@as(f32, 0.0), box2.getConvexRadius());
    }

    // Defaults: the default convex radius, the default constructor has a zero box
    var defaults = BoxShapeSettings.init(allocator, Vec3.one(), .{});
    defer defaults.deinit();
    try testing.expectEqual(PhysicsSettings.default_convex_radius, defaults.convex_radius);
    var empty = BoxShapeSettings.initDefault(allocator);
    defer empty.deinit();
    try testing.expect(empty.half_extent.eql(Vec3.zero()) and empty.convex_radius == 0.0);

    // Heap settings with a material
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    const settings = try BoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), .{ .convex_radius = 0.1, .material = material.material() });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    var result = try settings.createShape(allocator);
    defer result.deinit();
    const box = result.getPtr().?.cast(BoxShape);
    try testing.expect(box.getHalfExtent().eql(Vec3.init(1, 2, 3)));
    try testing.expectEqual(@as(f32, 0.1), box.getConvexRadius());
    try testing.expect(box.asShape().getMaterial(.empty) == material.material());

    // Out of memory while creating the shape is returned and not cached
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var oom_settings = BoxShapeSettings.init(allocator, Vec3.one(), .{});
        defer oom_settings.deinit();
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var r = oom_settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(oom_settings.base.base.cached_result.isEmpty());
            continue;
        };
        defer r.deinit();
        try testing.expect(r.isValid());
        try testing.expectEqual(@as(usize, 1), fail_index); // Only the shape is allocated
        break;
    }
}

test "BoxShape: bounds, inner radius, mass properties, volume, stats, surface normal, supporting face" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const shape = box.asShape();

    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.init(-1, -2, -3), Vec3.init(1, 2, 3))));
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.3), Vec3.init(1, 2, 3));
    try testing.expect(shape.getWorldSpaceBounds(transform, Vec3.init(2, -1, 1)).eql(shape.getLocalBounds().scaled(Vec3.init(2, -1, 1)).transformed(transform))); // Shape's version
    try testing.expectEqual(@as(f32, 1.0), shape.getInnerRadius());
    try testing.expectEqual(@as(f32, 48.0), shape.getVolume());
    try testing.expectEqual(@as(usize, @sizeOf(BoxShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 12), shape.getStats().num_triangles);
    try testing.expect(shape.isValidScale(Vec3.init(1, -2, 3)) and !shape.isValidScale(Vec3.init(1, 0, 3))); // Any non zero scale

    const p = shape.getMassProperties();
    try testing.expectEqual(@as(f32, 48000.0), p.mass);
    try testing.expectApproxEqRel(@as(f32, 48000.0 / 12.0 * (16.0 + 36.0)), p.inertia.get(0, 0), 1.0e-6);

    // Surface normals: the closest face
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0.9, 0.5, 0.5)).eql(Vec3.axisX()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0.1, -1.95, 0.5)).eql(Vec3.axisY().negate()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, 0, 3)).eql(Vec3.axisZ()));

    // Supporting face: scaled, transformed to world space
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.axisX(), Vec3.init(-2, 1, 1), Mat44.translation(Vec3.init(10, 0, 0)), &face);
    try testing.expectEqual(@as(u32, 4), face.len);
    for (face.constSlice()) |v| try testing.expectEqual(@as(f32, 8.0), v.getX()); // The face at -X of the scaled box faces +X the most
}

test "BoxShape: support functions" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{ .convex_radius = 0.5 });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    var buffer: ConvexShape.SupportBuffer = .{};
    const scale = Vec3.init(-2, 1, 1);

    // Include convex radius and default: the full scaled box, no convex radius
    for ([_]ConvexShape.SupportMode{ .include_convex_radius, .default }) |mode| {
        const support = box.base.getSupportFunction(mode, &buffer, scale);
        try testing.expectEqual(@as(f32, 0.0), support.getConvexRadius());
        try testing.expect(support.getSupport(Vec3.init(1, -1, 1)).eql(Vec3.init(2, -2, 3)));
    }

    // Exclude convex radius: shrunk by the scaled convex radius (limited to cDefaultConvexRadius)
    const support = box.base.getSupportFunction(.exclude_convex_radius, &buffer, scale);
    try testing.expectEqual(PhysicsSettings.default_convex_radius, support.getConvexRadius());
    const reduced = Vec3.init(2, 2, 3).sub(Vec3.replicate(PhysicsSettings.default_convex_radius));
    try testing.expect(support.getSupport(Vec3.init(1, -1, 1)).eql(Vec3.init(reduced.getX(), -reduced.getY(), reduced.getZ())));
}

test "BoxShape: ray casts, collide point, filters and the collector context" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const shape = box.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 2, 3);

    // Single hit
    var hit: RayCastResult = .{};
    try testing.expect(shape.castRay(.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0)), creator, &hit));
    try testing.expectEqual(@as(f32, 0.25), hit.fraction);
    try testing.expect(hit.sub_shape_id2.eql(creator.getID()));
    try testing.expect(!shape.castRay(.init(Vec3.init(-2, 5, 0), Vec3.init(4, 0, 0)), creator, &hit));
    try testing.expect(shape.castRay(.init(Vec3.zero(), Vec3.init(4, 0, 0)), creator, &hit)); // Starts inside
    try testing.expectEqual(@as(f32, 0.0), hit.fraction);

    // Collector: front and back face hits
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(8), .{});
    hits.base.setContext(&context);
    shape.castRayCollector(.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0)), &settings, creator, &hits.base, &.{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.25), hits.hits.items[0].fraction);
    try testing.expectEqual(@as(f32, 0.75), hits.hits.items[1].fraction);
    try testing.expect(hits.hits.items[0].body_id.eql(.init(8)));

    // Collide point (the surface counts as inside)
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.init(1, -2, 3), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(1.01, 0, 0), creator, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    try testing.expect(points.hits.items[0].sub_shape_id2.eql(creator.getID()));
}

test "BoxShape: CollideSoftBodyVertices" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5 * math.pi), Vec3.init(10, 0, 0));
    var positions = [_]Vec3{ transform.mulVec3(Vec3.init(0.5, 0.1, 0.2)), transform.mulVec3(Vec3.init(3, 0, 0)), transform.mulVec3(Vec3.init(0, 0, 0)) };
    var inv_masses = [_]f32{ 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 3;
    var penetrations = [_]f32{-math.flt_max} ** 3;
    var indices = [_]i32{-1} ** 3;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    box.asShape().collideSoftBodyVertices(transform, Vec3.one(), &vertices, 3, 4);

    // Inside: the closest face is +X (0.5 away)
    try testing.expectApproxEqAbs(@as(f32, 0.5), penetrations[0], 1.0e-6);
    try testing.expectEqual(@as(i32, 4), indices[0]);
    try testing.expect(planes[0].getNormal().isClose(transform.multiply3x3(Vec3.axisX()), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectApproxEqAbs(@as(f32, 0.0), planes[0].signedDistance(transform.mulVec3(Vec3.init(1, 0, 0))), 1.0e-5);

    // Outside: negative penetration, normal from the closest point
    try testing.expectApproxEqAbs(@as(f32, -2.0), penetrations[1], 1.0e-5);
    try testing.expect(planes[1].getNormal().isClose(transform.multiply3x3(Vec3.axisX()), .{ .max_dist_sq = 1.0e-10 }));

    // Infinite mass: skipped
    try testing.expectEqual(-math.flt_max, penetrations[2]);
    try testing.expectEqual(@as(i32, -1), indices[2]);
}

test "BoxShape: GetTrianglesStart / Next, GetSubmergedVolume" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    // 12 triangles on the faces of the scaled box, inside out scales flip the winding
    for ([_]Vec3{ Vec3.one(), Vec3.init(1, -1, 1) }) |scale| {
        var context: Shape.GetTrianglesContext = .{};
        box.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), scale);
        var vertices: [3 * 32]Float3 = undefined;
        try testing.expectEqual(@as(u32, 12), box.asShape().getTrianglesNext(&context, 32, &vertices, null));
        const flipped_order = [_]usize{ 0, 2, 1 };
        for (vertices[0..36], 0..) |v, i| {
            const index = if (scale.getY() < 0.0) i / 3 * 3 + flipped_order[i % 3] else i;
            const expected = unit_box_triangles[index].mul(Vec3.init(1, 2, 3)).mul(scale);
            try testing.expect(Vec3.fromFloat3(v).eql(expected));
        }
        try testing.expectEqual(@as(u32, 0), box.asShape().getTrianglesNext(&context, 32, &vertices, null));
    }

    // GetSubmergedVolume is ConvexShape's
    const half = box.asShape().getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.fromPointAndNormal(Vec3.zero(), Vec3.axisY()));
    try testing.expectEqual(@as(f32, 48.0), half.total_volume);
    try testing.expectApproxEqAbs(@as(f32, 24.0), half.submerged_volume, 1.0e-5);
}

test "BoxShape: binary state, restoreFromBinaryState and the registration" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{ .convex_radius = 0.25 });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    box.base.setDensity(321.0);

    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    box.asShape().saveBinaryState(out.streamOut());
    try testing.expectEqual(@as(usize, 1 + 8 + 4 + 12 + 4), writer.buffered().len); // Sub type, user data, density, half extent, convex radius

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    const restored = result.getPtr().?.cast(BoxShape);
    try testing.expect(restored.getHalfExtent().eql(Vec3.init(1, 2, 3)));
    try testing.expectEqual(@as(f32, 0.25), restored.getConvexRadius());
    try testing.expectEqual(@as(f32, 321.0), restored.base.getDensity());

    // Truncated: Jolt's error text
    var short_reader: std.Io.Reader = .fixed(writer.buffered()[0 .. writer.buffered().len - 1]);
    var short_in = StreamWrapper.StreamInWrapper.init(&short_reader);
    var short_result = try Shape.restoreFromBinaryState(allocator, short_in.streamIn());
    defer short_result.deinit();
    try testing.expectEqualStrings("Failed to restore shape", short_result.getError());

    // ShapeFunctions
    try testing.expect(ShapeFunctions.get(.box).construct != null);
    try testing.expect(ShapeFunctions.get(.box).color.eql(Color.green));
}
