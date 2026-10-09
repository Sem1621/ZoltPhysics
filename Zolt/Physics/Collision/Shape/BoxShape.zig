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
