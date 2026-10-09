//! Port of: Jolt/Physics/Collision/Shape/SphereShape.h, Jolt/Physics/Collision/Shape/SphereShape.cpp
//! Status: complete
//!
//! The reference implementation of a concrete convex shape (the porter template in Docs/Zolt/CollisionArchitecture.md,
//! section 2, follows this file):
//! - `SphereShapeSettings` / `SphereShape` embed their parents as `base` (D1), `overrides` lists every C++ `override`
//!   in header order, the C++ constructors are `initDefault` (default constructor), `initFromSettings` (D3, Jolt's
//!   error texts), `init` (stack / member shapes) and `create` (`new SphereShape`).
//! - The support classes `SphereNoConvex` / `SphereWithConvex` are constructed in the caller's SupportBuffer (D9).
//! - JPH_DEBUG_RENDERER (Draw, the drawing of the submerged volume and the `inBaseOffset` parameter of
//!   GetSubmergedVolume) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RaySphere = @import("../../../Geometry/RaySphere.zig");
const math = @import("../../../Math/Math.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
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

/// Class that constructs a SphereShape
pub const SphereShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, SphereShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    radius: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) SphereShapeSettings {
        return .{ .base = .init(SphereShapeSettings, allocator, null) };
    }

    /// Create a sphere with radius `radius` (settings on the stack: `defer settings.deinit()`)
    pub fn init(allocator: Allocator, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) SphereShapeSettings {
        return .{ .base = .init(SphereShapeSettings, allocator, opts.material), .radius = radius };
    }

    /// new SphereShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) Allocator.Error!*SphereShapeSettings {
        const self = try allocator.create(SphereShapeSettings);
        self.* = .init(allocator, radius, .{ .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *SphereShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *SphereShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *SphereShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(SphereShape, self, allocator);
    }
};

/// A sphere, centered around the origin.
/// Note that it is implemented as a point with convex radius.
pub const SphereShape = struct {
    /// Concrete class: `Shape.cast(SphereShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .sphere;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: ConvexShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    radius: f32 = 0.0,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// SphereShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) SphereShape {
        return .{ .base = .init(SphereShape, allocator, shape_sub_type, null) };
    }

    /// SphereShape(const SphereShapeSettings &inSettings, ShapeResult &outResult): base part first, then the C++ body
    pub fn initFromSettings(self: *SphereShape, settings: *const SphereShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.radius = settings.radius;

        if (settings.radius <= 0.0) {
            result.setError("Invalid radius");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// Create a sphere with radius `radius` (on the stack / as a member: `asShape().setEmbedded()` before taking
    /// references, `asShapeMut().deinit()` at the end)
    pub fn init(allocator: Allocator, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) SphereShape {
        std.debug.assert(radius > 0.0);
        return .{ .base = .init(SphereShape, allocator, shape_sub_type, opts.material), .radius = radius };
    }

    /// new SphereShape(inRadius, inMaterial): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) Allocator.Error!*SphereShape {
        const self = try allocator.create(SphereShape);
        self.* = .init(allocator, radius, .{ .material = opts.material });
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const SphereShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *SphereShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Radius of the sphere
    pub fn getRadius(self: *const SphereShape) f32 {
        return self.radius;
    }

    // Get the radius of this sphere scaled by scale
    fn getScaledRadius(self: *const SphereShape, scale: Vec3) f32 {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        const abs_scale = scale.abs();
        return abs_scale.getX() * self.radius;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const SphereShape) AABox {
        const half_extent = Vec3.replicate(self.radius);
        return .init(half_extent.negate(), half_extent);
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const SphereShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        const scaled_radius = self.getScaledRadius(scale);
        const half_extent = Vec3.replicate(scaled_radius);
        var bounds = AABox.init(half_extent.negate(), half_extent);
        bounds.translate(center_of_mass_transform.getTranslation());
        return bounds;
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const SphereShape) f32 {
        return self.radius;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const SphereShape) MassProperties {
        var p: MassProperties = .{};

        // Calculate mass
        const r2 = self.radius * self.radius;
        p.mass = (@as(f32, 4.0) / 3.0 * math.pi) * self.radius * r2 * self.base.getDensity();

        // Calculate inertia
        const inertia = (@as(f32, 2.0) / 5.0) * p.mass * r2;
        p.inertia = Mat44.scale(inertia);

        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const SphereShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = self;
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        const len = local_surface_position.length();
        return if (len != 0.0) local_surface_position.divScalar(len) else Vec3.axisY();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const SphereShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        // Hit is always a single point, no point in returning anything
        _ = .{ self, sub_shape_id, direction, scale, center_of_mass_transform, out_vertices };
    }

    // See ConvexShape::GetSupportFunction: placement new into the caller's buffer
    pub fn getSupportFunction(self: *const SphereShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        const scaled_radius = self.getScaledRadius(scale);

        switch (mode) {
            .include_convex_radius => {
                const support = buffer.emplace(SphereWithConvex);
                support.* = .init(scaled_radius);
                return &support.base;
            },

            .exclude_convex_radius, .default => {
                const support = buffer.emplace(SphereNoConvex);
                support.* = .init(scaled_radius);
                return &support.base;
            },
        }
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const SphereShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        const scaled_radius = self.getScaledRadius(scale);
        const total_volume = (@as(f32, 4.0) / 3.0 * math.pi) * math.cubed(scaled_radius);

        const distance_to_surface = surface.signedDistance(center_of_mass_transform.getTranslation());
        if (distance_to_surface >= scaled_radius) {
            // Above surface
            return .{ .total_volume = total_volume, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
        } else if (distance_to_surface <= -scaled_radius) {
            // Under surface
            return .{ .total_volume = total_volume, .submerged_volume = total_volume, .center_of_buoyancy = center_of_mass_transform.getTranslation() };
        } else {
            // Intersecting surface

            // Calculate submerged volume, see: https://en.wikipedia.org/wiki/Spherical_cap
            const h = scaled_radius - distance_to_surface;
            const submerged_volume = (math.pi / 3.0) * math.square(h) * (3.0 * scaled_radius - h);

            // Calculate center of buoyancy, see: http://mathworld.wolfram.com/SphericalCap.html (eq 10)
            const z = (@as(f32, 3.0) / 4.0) * math.square(2.0 * scaled_radius - h) / (3.0 * scaled_radius - h);
            const center_of_buoyancy = center_of_mass_transform.getTranslation().sub(surface.getNormal().mulScalar(z)); // Negative normal since we want the portion under the water

            // TODO(debug_renderer): Draw intersection between sphere and water plane (sDrawSubmergedVolumes)

            return .{ .total_volume = total_volume, .submerged_volume = submerged_volume, .center_of_buoyancy = center_of_buoyancy };
        }

        // TODO(debug_renderer): Draw center of buoyancy (sDrawSubmergedVolumes)
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay
    pub fn castRay(self: *const SphereShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const fraction = RaySphere.raySphere(ray.origin, ray.direction, Vec3.zero(), self.radius);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const SphereShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const r = RaySphere.raySphereMinMax(ray.origin, ray.direction, Vec3.zero(), self.radius);
        const min_fraction = r.min_fraction;
        const max_fraction = r.max_fraction;
        const num_results = r.num_intersections;
        if (num_results > 0 // Ray should intersect
        and max_fraction >= 0.0 // End of ray should be inside sphere
        and min_fraction < collector.getEarlyOutFraction()) // Start of ray should be before early out fraction
        {
            // Better hit than the current hit
            var hit: RayCastResult = .{};
            hit.body_id = TransformedShape.getBodyID(collector.getContext());
            hit.sub_shape_id2 = sub_shape_id_creator.getID();

            // Check front side hit
            if (ray_cast_settings.treat_convex_as_solid or min_fraction > 0.0) {
                hit.fraction = math.max(@as(f32, 0.0), min_fraction);
                collector.addHit(&hit);
            }

            // Check back side hit
            if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and num_results > 1 // Ray should have 2 intersections
            and max_fraction < collector.getEarlyOutFraction()) // End of ray should be before early out fraction
            {
                hit.fraction = max_fraction;
                collector.addHit(&hit);
            }
        }
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const SphereShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        if (point.lengthSq() <= math.square(self.radius))
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const SphereShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        const center = center_of_mass_transform.getTranslation();
        const radius = self.getScaledRadius(scale);

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                // Calculate penetration
                const delta = v.getPosition().sub(center);
                const distance = delta.length();
                const penetration = radius - distance;
                if (v.updatePenetration(penetration)) {
                    // Calculate contact point and normal
                    const normal = if (distance > 0.0) delta.divScalar(distance) else Vec3.axisY();
                    const point = center.add(normal.mulScalar(radius));

                    // Store collision
                    v.setCollision(Plane.fromPointAndNormal(point, normal), colliding_shape_index);
                }
            }
        }
    }

    // See Shape::GetTrianglesStart: placement new of a context (no pointers into itself: construct by value)
    pub fn getTrianglesStart(self: *const SphereShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        const scaled_radius = self.getScaledRadius(scale);
        context.emplace(GetTrianglesContextVertexList).* = .init(position_com, rotation, Vec3.one(), Mat44.scale(scaled_radius), ConvexShape.unit_sphere_triangles.constSlice(), self.base.getConvexMaterial());
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const SphereShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    // See Shape::SaveBinaryState: C++ `ConvexShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const SphereShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const SphereShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(SphereShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const SphereShape) f32 {
        return @as(f32, 4.0) / 3.0 * math.pi * math.cubed(self.radius);
    }

    // See Shape::IsValidScale: C++ `ConvexShape::IsValidScale` resolves to Shape's version (ConvexShape does not override it)
    pub fn isValidScale(self: *const SphereShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScale(scale.abs());
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const SphereShape, scale: Vec3) Vec3 {
        _ = self;
        const s = ScaleHelpers.makeNonZeroScale(scale);

        return s.getSign().mul(ScaleHelpers.makeUniformScale(s.abs()));
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *SphereShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.radius);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.sphere);
        f.construct = ShapeFunctions.constructor(SphereShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Classes for GetSupportFunction (`class SphereNoConvex final : public Support`)

    const SphereNoConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        radius: f32,

        fn init(radius: f32) SphereNoConvex {
            return .{ .base = .init(SphereNoConvex), .radius = radius };
        }

        pub fn getSupport(self: *const SphereNoConvex, direction: Vec3) Vec3 {
            _ = .{ self, direction };
            return Vec3.zero();
        }

        pub fn getConvexRadius(self: *const SphereNoConvex) f32 {
            return self.radius;
        }
    };

    const SphereWithConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        radius: f32,

        fn init(radius: f32) SphereWithConvex {
            return .{ .base = .init(SphereWithConvex), .radius = radius };
        }

        pub fn getSupport(self: *const SphereWithConvex, direction: Vec3) Vec3 {
            const len = direction.length();
            return if (len > 0.0) direction.mulScalar(self.radius / len) else Vec3.zero();
        }

        pub fn getConvexRadius(self: *const SphereWithConvex) f32 {
            _ = self;
            return 0.0;
        }
    };
};
