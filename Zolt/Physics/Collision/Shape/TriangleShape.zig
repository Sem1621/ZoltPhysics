//! Port of: Jolt/Physics/Collision/Shape/TriangleShape.h, Jolt/Physics/Collision/Shape/TriangleShape.cpp
//! Status: complete
//!
//! A concrete convex shape that follows the porter template (Docs/Zolt/CollisionArchitecture.md, section 2, and
//! SphereShape.zig):
//! - `TriangleShapeSettings` / `TriangleShape` embed their parents as `base` (D1), `overrides` lists every C++ `override`
//!   in header order, the C++ constructors are `initDefault`, `initFromSettings` (D3, Jolt's error text), `init` and
//!   `create`.
//! - The support classes `TriangleNoConvex` / `TriangleWithConvex` and the GetTriangles context `TSGetTrianglesContext`
//!   are constructed in the caller's buffers (D9, D10).
//! - The private static collision functions that sRegister puts in the dispatch table (`sCollideConvexVsTriangle`,
//!   `sCollideSphereVsTriangle`, `sCastConvexVsTriangle`, `sCastSphereVsTriangle`) keep their names without `s` and
//!   drive the triangle algorithms (CollideConvexVsTriangles, CollideSphereVsTriangles, CastConvexVsTriangles,
//!   CastSphereVsTriangles) with all edges active.
//! - JPH_DEBUG_RENDERER (Draw and the `inBaseOffset` parameter of GetSubmergedVolume) is not ported yet:
//!   TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const TriangleConvexSupport = @import("../../../Geometry/ConvexSupport.zig").TriangleConvexSupport;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RayTriangle = @import("../../../Geometry/RayTriangle.zig");
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
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const ConvexShapeFile = @import("ConvexShape.zig");
const ConvexShape = ConvexShapeFile.ConvexShape;
const ConvexShapeSettings = ConvexShapeFile.ConvexShapeSettings;
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
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CastConvexVsTriangles = @import("../CastConvexVsTriangles.zig").CastConvexVsTriangles;
const CastSphereVsTriangles = @import("../CastSphereVsTriangles.zig").CastSphereVsTriangles;
const CollideConvexVsTriangles = @import("../CollideConvexVsTriangles.zig").CollideConvexVsTriangles;
const CollideSphereVsTriangles = @import("../CollideSphereVsTriangles.zig").CollideSphereVsTriangles;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const CollideSoftBodyVerticesVsTriangles = @import("../CollideSoftBodyVerticesVsTriangles.zig").CollideSoftBodyVerticesVsTriangles;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// Class that constructs a TriangleShape
pub const TriangleShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, TriangleShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    v1: Vec3 = Vec3.zero(),
    v2: Vec3 = Vec3.zero(),
    v3: Vec3 = Vec3.zero(),
    convex_radius: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) TriangleShapeSettings {
        return .{ .base = .init(TriangleShapeSettings, allocator, null) };
    }

    /// Create a triangle with points (v1, v2, v3) (counter clockwise) and convex radius convex_radius.
    /// Note that the convex radius is currently only used for shape vs shape collision, for all other purposes the triangle is infinitely thin.
    /// Settings on the stack: `defer settings.deinit()`.
    pub fn init(allocator: Allocator, v1: Vec3, v2: Vec3, v3: Vec3, opts: struct { convex_radius: f32 = 0.0, material: ?*const PhysicsMaterial = null }) TriangleShapeSettings {
        return .{ .base = .init(TriangleShapeSettings, allocator, opts.material), .v1 = v1, .v2 = v2, .v3 = v3, .convex_radius = opts.convex_radius };
    }

    /// new TriangleShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, v1: Vec3, v2: Vec3, v3: Vec3, opts: struct { convex_radius: f32 = 0.0, material: ?*const PhysicsMaterial = null }) Allocator.Error!*TriangleShapeSettings {
        const self = try allocator.create(TriangleShapeSettings);
        self.* = .init(allocator, v1, v2, v3, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *TriangleShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *TriangleShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *TriangleShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(TriangleShape, self, allocator);
    }
};

/// A single triangle, not the most efficient way of creating a world filled with triangles but can be used as a query shape for example.
pub const TriangleShape = struct {
    /// Concrete class: `Shape.cast(TriangleShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .triangle;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: ConvexShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    v1: Vec3 = Vec3.zero(),
    v2: Vec3 = Vec3.zero(),
    v3: Vec3 = Vec3.zero(),
    convex_radius: f32 = 0.0,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// TriangleShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) TriangleShape {
        return .{ .base = .init(TriangleShape, allocator, shape_sub_type, null) };
    }

    /// TriangleShape(const TriangleShapeSettings &inSettings, ShapeResult &outResult): C++ member initializers first, then the body
    pub fn initFromSettings(self: *TriangleShape, settings: *const TriangleShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.v1 = settings.v1;
        self.v2 = settings.v2;
        self.v3 = settings.v3;
        self.convex_radius = settings.convex_radius;

        if (settings.convex_radius < 0.0) {
            result.setError("Invalid convex radius");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// Create a triangle with points (v1, v2, v3) (counter clockwise) and convex radius convex_radius.
    /// Note that the convex radius is currently only used for shape vs shape collision, for all other purposes the triangle is infinitely thin.
    /// On the stack / as a member: `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end.
    pub fn init(allocator: Allocator, v1: Vec3, v2: Vec3, v3: Vec3, opts: struct { convex_radius: f32 = 0.0, material: ?*const PhysicsMaterial = null }) TriangleShape {
        std.debug.assert(opts.convex_radius >= 0.0);
        return .{ .base = .init(TriangleShape, allocator, shape_sub_type, opts.material), .v1 = v1, .v2 = v2, .v3 = v3, .convex_radius = opts.convex_radius };
    }

    /// new TriangleShape(...): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, v1: Vec3, v2: Vec3, v3: Vec3, opts: struct { convex_radius: f32 = 0.0, material: ?*const PhysicsMaterial = null }) Allocator.Error!*TriangleShape {
        const self = try allocator.create(TriangleShape);
        self.* = .init(allocator, v1, v2, v3, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const TriangleShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *TriangleShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get the vertices of the triangle
    pub fn getVertex1(self: *const TriangleShape) Vec3 {
        return self.v1;
    }

    pub fn getVertex2(self: *const TriangleShape) Vec3 {
        return self.v2;
    }

    pub fn getVertex3(self: *const TriangleShape) Vec3 {
        return self.v3;
    }

    /// Convex radius
    pub fn getConvexRadius(self: *const TriangleShape) f32 {
        return self.convex_radius;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const TriangleShape) AABox {
        var bounds = AABox.init(self.v1, self.v1);
        bounds.encapsulateVec3(self.v2);
        bounds.encapsulateVec3(self.v3);
        bounds.expandBy(Vec3.replicate(self.convex_radius));
        return bounds;
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const TriangleShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        const v1 = center_of_mass_transform.mulVec3(scale.mul(self.v1));
        const v2 = center_of_mass_transform.mulVec3(scale.mul(self.v2));
        const v3 = center_of_mass_transform.mulVec3(scale.mul(self.v3));

        var bounds = AABox.init(v1, v1);
        bounds.encapsulateVec3(v2);
        bounds.encapsulateVec3(v3);
        bounds.expandBy(scale.mulScalar(self.convex_radius));
        return bounds;
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const TriangleShape) f32 {
        return self.convex_radius;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const TriangleShape) MassProperties {
        _ = self;

        // We cannot calculate the volume for a triangle, so we return invalid mass properties.
        // If you want your triangle to be dynamic, then you should provide the mass properties yourself when
        // creating a Body:
        //
        // BodyCreationSettings::mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
        // BodyCreationSettings::mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(Vec3::sOne(), 1000.0f);
        //
        // Note that this makes the triangle shape behave the same as a mesh shape with a single triangle.
        // In practice there is very little use for a dynamic triangle shape as back side collisions will be ignored
        // so if the triangle falls the wrong way it will sink through the floor.
        return .{};
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const TriangleShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = local_surface_position;
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        const cross = self.v2.sub(self.v1).cross(self.v3.sub(self.v1));
        const len = cross.length();
        return if (len != 0.0) cross.divScalar(len) else Vec3.axisY();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const TriangleShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = direction;
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        // Calculate transform with scale
        const transform = center_of_mass_transform.preScaled(scale);

        // Flip triangle if scaled inside out
        if (ScaleHelpers.isInsideOut(scale)) {
            out_vertices.append(transform.mulVec3(self.v1));
            out_vertices.append(transform.mulVec3(self.v3));
            out_vertices.append(transform.mulVec3(self.v2));
        } else {
            out_vertices.append(transform.mulVec3(self.v1));
            out_vertices.append(transform.mulVec3(self.v2));
            out_vertices.append(transform.mulVec3(self.v3));
        }
    }

    // See ConvexShape::GetSupportFunction: placement new into the caller's buffer
    pub fn getSupportFunction(self: *const TriangleShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        switch (mode) {
            .include_convex_radius, .default => {
                if (self.convex_radius > 0.0) {
                    const support = buffer.emplace(TriangleWithConvex);
                    support.* = .init(scale.mul(self.v1), scale.mul(self.v2), scale.mul(self.v3), self.convex_radius);
                    return &support.base;
                }
                // C++ [[fallthrough]] into ExcludeConvexRadius
                return self.getSupportFunctionNoConvex(buffer, scale);
            },

            .exclude_convex_radius => return self.getSupportFunctionNoConvex(buffer, scale),
        }
    }

    /// The ExcludeConvexRadius case of GetSupportFunction (the target of the C++ [[fallthrough]])
    fn getSupportFunctionNoConvex(self: *const TriangleShape, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        const support = buffer.emplace(TriangleNoConvex);
        support.* = .init(scale.mul(self.v1), scale.mul(self.v2), scale.mul(self.v3));
        return &support.base;
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const TriangleShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };

        // A triangle has no volume
        return .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay
    pub fn castRay(self: *const TriangleShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const fraction = RayTriangle.rayTriangle(ray.origin, ray.direction, self.v1, self.v2, self.v3);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const TriangleShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Back facing check
        if (ray_cast_settings.back_face_mode_triangles == .ignore_back_faces and self.v2.sub(self.v1).cross(self.v3.sub(self.v1)).dot(ray.direction) > 0.0)
            return;

        // Test ray against triangle
        const fraction = RayTriangle.rayTriangle(ray.origin, ray.direction, self.v1, self.v2, self.v3);
        if (fraction < collector.getEarlyOutFraction()) {
            // Better hit than the current hit
            var hit: RayCastResult = .{};
            hit.body_id = TransformedShape.getBodyID(collector.getContext());
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            collector.addHit(&hit);
        }
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const TriangleShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Can't be inside a triangle
        _ = .{ self, point, sub_shape_id_creator, collector, shape_filter };
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const TriangleShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        var collider = CollideSoftBodyVerticesVsTriangles.init(center_of_mass_transform, scale);

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                collider.startVertex(&v);
                collider.processTriangle(self.v1, self.v2, self.v3);
                collider.finishVertex(&v, colliding_shape_index);
            }
        }
    }

    // See Shape::GetTrianglesStart: placement new of a context (no pointers into itself: construct by value)
    pub fn getTrianglesStart(self: *const TriangleShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        const m = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale));

        context.emplace(TSGetTrianglesContext).* = .init(m.mulVec3(self.v1), m.mulVec3(self.v2), m.mulVec3(self.v3));
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const TriangleShape, context_in: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        comptime std.debug.assert(Shape.get_triangles_min_triangles_requested >= 3); // cGetTrianglesMinTrianglesRequested is too small
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        const context = context_in.get(TSGetTrianglesContext);

        // Only return the triangle the 1st time
        if (context.is_done)
            return 0;
        context.is_done = true;

        // Store triangle
        context.v1.storeFloat3(&out_triangle_vertices[0]);
        context.v2.storeFloat3(&out_triangle_vertices[1]);
        context.v3.storeFloat3(&out_triangle_vertices[2]);

        // Store material (C++ GetMaterial() without arguments is ConvexShape's non virtual version)
        if (out_materials) |materials|
            materials[0] = self.base.getConvexMaterial();

        return 1;
    }

    // See Shape::SaveBinaryState: C++ `ConvexShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const TriangleShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.v1);
        stream.write(self.v2);
        stream.write(self.v3);
        stream.write(self.convex_radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const TriangleShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TriangleShape), 1);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const TriangleShape) f32 {
        _ = self;
        return 0;
    }

    // See Shape::IsValidScale: C++ `ConvexShape::IsValidScale` resolves to Shape's version (ConvexShape does not override it)
    pub fn isValidScale(self: *const TriangleShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and (self.convex_radius == 0.0 or ScaleHelpers.isUniformScale(scale.abs()));
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const TriangleShape, scale_in: Vec3) Vec3 {
        const scale = ScaleHelpers.makeNonZeroScale(scale_in);

        if (self.convex_radius == 0.0)
            return scale;

        return scale.getSign().mul(ScaleHelpers.makeUniformScale(scale.abs()));
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *TriangleShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.v1);
        stream.read(&self.v2);
        stream.read(&self.v3);
        stream.read(&self.convex_radius);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.triangle);
        f.construct = ShapeFunctions.constructor(TriangleShape);
        f.color = Color.green;

        for (ShapeFile.convex_sub_shape_types) |s| {
            r.registerCollideShape(s, .triangle, collideConvexVsTriangle);
            r.registerCastShape(s, .triangle, castConvexVsTriangle);

            // Avoid registering triangle vs triangle as a reversed test to prevent infinite recursion
            if (s != .triangle) {
                r.registerCollideShape(.triangle, s, CollisionDispatch.reversedCollideShape);
                r.registerCastShape(.triangle, s, CollisionDispatch.reversedCastShape);
            }
        }

        // Specialized collision functions
        r.registerCollideShape(.sphere, .triangle, collideSphereVsTriangle);
        r.registerCastShape(.sphere, .triangle, castSphereVsTriangle);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch (private static in Jolt)

    /// sCollideConvexVsTriangle
    fn collideConvexVsTriangle(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        std.debug.assert(shape1_in.getType() == .convex);
        const shape1 = shape1_in.cast(ConvexShape);
        std.debug.assert(shape2_in.getSubType() == .triangle);
        const shape2 = shape2_in.cast(TriangleShape);

        var collider = CollideConvexVsTriangles.init(shape1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), collide_shape_settings, collector);
        collider.collide(shape2.v1, shape2.v2, shape2.v3, 0b111, sub_shape_id_creator2.getID());
    }

    /// sCollideSphereVsTriangle
    fn collideSphereVsTriangle(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        std.debug.assert(shape1_in.getSubType() == .sphere);
        const shape1 = shape1_in.cast(SphereShape);
        std.debug.assert(shape2_in.getSubType() == .triangle);
        const shape2 = shape2_in.cast(TriangleShape);

        var collider = CollideSphereVsTriangles.init(shape1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), collide_shape_settings, collector);
        collider.collide(shape2.v1, shape2.v2, shape2.v3, 0b111, sub_shape_id_creator2.getID());
    }

    /// sCastConvexVsTriangle
    fn castConvexVsTriangle(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        std.debug.assert(shape_in.getSubType() == .triangle);
        const shape = shape_in.cast(TriangleShape);

        var caster = CastConvexVsTriangles.init(shape_cast, shape_cast_settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector);
        caster.cast(shape.v1, shape.v2, shape.v3, 0b111, sub_shape_id_creator2.getID());
    }

    /// sCastSphereVsTriangle
    fn castSphereVsTriangle(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        std.debug.assert(shape_in.getSubType() == .triangle);
        const shape = shape_in.cast(TriangleShape);

        var caster = CastSphereVsTriangles.init(shape_cast, shape_cast_settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector);
        caster.cast(shape.v1, shape.v2, shape.v3, 0b111, sub_shape_id_creator2.getID());
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Context for GetTrianglesStart/Next (`class TSGetTrianglesContext`)

    const TSGetTrianglesContext = struct {
        v1: Vec3,
        v2: Vec3,
        v3: Vec3,

        is_done: bool = false,

        fn init(v1: Vec3, v2: Vec3, v3: Vec3) TSGetTrianglesContext {
            return .{ .v1 = v1, .v2 = v2, .v3 = v3 };
        }

        comptime {
            std.debug.assert(@sizeOf(TSGetTrianglesContext) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
        }
    };

    // ---------------------------------------------------------------------------------------------------------------
    // Classes for GetSupportFunction (`class TriangleNoConvex final : public Support`, `class TriangleWithConvex final : public Support`)

    const TriangleNoConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        triangle_support: TriangleConvexSupport,

        fn init(v1: Vec3, v2: Vec3, v3: Vec3) TriangleNoConvex {
            return .{ .base = .init(TriangleNoConvex), .triangle_support = .init(v1, v2, v3) };
        }

        pub fn getSupport(self: *const TriangleNoConvex, direction: Vec3) Vec3 {
            return self.triangle_support.getSupport(direction);
        }

        pub fn getConvexRadius(self: *const TriangleNoConvex) f32 {
            _ = self;
            return 0.0;
        }
    };

    const TriangleWithConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        convex_radius: f32,
        triangle_support: TriangleConvexSupport,

        fn init(v1: Vec3, v2: Vec3, v3: Vec3, convex_radius: f32) TriangleWithConvex {
            return .{ .base = .init(TriangleWithConvex), .convex_radius = convex_radius, .triangle_support = .init(v1, v2, v3) };
        }

        pub fn getSupport(self: *const TriangleWithConvex, direction: Vec3) Vec3 {
            var support = self.triangle_support.getSupport(direction);
            const len = direction.length();
            if (len > 0.0)
                support = support.add(direction.mulScalar(self.convex_radius / len));
            return support;
        }

        pub fn getConvexRadius(self: *const TriangleWithConvex) f32 {
            return self.convex_radius;
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's own triangle tests are in ZoltTests/Physics, the bit exact comparison with Jolt in
// ZoltParity/Physics/TriangleShapeParity.zig)

const testing = std.testing;
const math = @import("../../../Math/Math.zig");
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const BoxShape = @import("BoxShape.zig").BoxShape;
const SphereShapeSettings = @import("SphereShape.zig").SphereShapeSettings;

/// CHECK_APPROX_EQUAL for vectors: `IsClose(b, tolerance^2)`
fn expectClose(expected: Vec3, actual: Vec3, tolerance: f32) !void {
    if (!actual.isClose(expected, .{ .max_dist_sq = tolerance * tolerance })) {
        std.debug.print("expected {any}, got {any}\n", .{ expected.value, actual.value });
        return error.TestExpectedApproxEq;
    }
}

/// A filter that rejects everything and counts the calls (state behind a pointer, Rule M)
const RejectAllFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: ShapeFilter = .init(@This()),
    calls: *u32,

    pub fn shouldCollide(self: *const RejectAllFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ shape2, sub_shape_id_of_shape2 };
        self.calls.* += 1;
        return false;
    }
};

test "TriangleShape: settings, Jolt's error text, cached results and out of memory" {
    const allocator = testing.allocator;
    const v1 = Vec3.init(1, 2, 3);
    const v2 = Vec3.init(4, 5, 6);
    const v3 = Vec3.init(7, 8, 10);

    // Invalid convex radius
    for ([_]f32{ -1.0e-6, -1.0 }) |convex_radius| {
        var settings = TriangleShapeSettings.init(allocator, v1, v2, v3, .{ .convex_radius = convex_radius });
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid convex radius", result.getError());
    }

    // Default constructor (deserialization): a degenerate triangle at the origin without convex radius is valid
    var default_settings = TriangleShapeSettings.initDefault(allocator);
    defer default_settings.deinit();
    try testing.expect(default_settings.v1.eql(Vec3.zero()) and default_settings.v2.eql(Vec3.zero()) and default_settings.v3.eql(Vec3.zero()));
    try testing.expectEqual(@as(f32, 0.0), default_settings.convex_radius);
    try testing.expectEqual(@as(f32, 1000.0), default_settings.base.density);
    var default_result = try default_settings.asShapeSettings().createShape(allocator);
    defer default_result.deinit();
    try testing.expect(default_result.isValid());

    // Heap settings, material and user data are passed to the shape, the result is cached
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    const settings = try TriangleShapeSettings.create(allocator, v1, v2, v3, .{ .convex_radius = 0.25, .material = material.material() });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    settings.asShapeSettings().user_data = 99;
    var result = try settings.createShape(allocator);
    defer result.deinit();
    const triangle = result.getPtr().?.cast(TriangleShape);
    try testing.expect(triangle.getVertex1().eql(v1) and triangle.getVertex2().eql(v2) and triangle.getVertex3().eql(v3));
    try testing.expectEqual(@as(f32, 0.25), triangle.getConvexRadius());
    try testing.expect(triangle.base.getConvexMaterial() == material.material());
    try testing.expectEqual(@as(u64, 99), triangle.asShape().getUserData());
    try testing.expectEqual(ShapeSubType.triangle, triangle.asShape().getSubType());
    var result2 = try settings.createShape(allocator);
    defer result2.deinit();
    try testing.expect(result2.getPtr() == result.getPtr());

    // Out of memory while creating the shape is returned and not cached, a later call succeeds
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var oom_settings = TriangleShapeSettings.init(allocator, v1, v2, v3, .{ .convex_radius = 0.1 });
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

test "TriangleShape: bounds, inner radius, mass properties, volume, stats, surface normal, supporting face" {
    const allocator = testing.allocator;

    var triangle = TriangleShape.init(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 10), .{ .convex_radius = 0.5 });
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();
    const shape = triangle.asShape();

    // Bounds include the convex radius
    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.init(0.5, 1.5, 2.5), Vec3.init(7.5, 8.5, 10.5))));
    const translation = Mat44.translation(Vec3.init(1, 0, 0));
    try testing.expect(shape.getWorldSpaceBounds(translation, Vec3.replicate(2.0)).eql(.init(Vec3.init(2, 3, 5), Vec3.init(16, 17, 21))));
    // Jolt expands by scale * convex radius, so a negative scale shrinks the box by the convex radius
    try testing.expect(shape.getWorldSpaceBounds(translation, Vec3.replicate(-2.0)).eql(.init(Vec3.init(-12, -15, -19), Vec3.init(-2, -5, -7))));
    const rotation = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5 * math.pi), Vec3.init(1, 2, 3));
    const rotated = shape.getWorldSpaceBounds(rotation, Vec3.one());
    try expectClose(Vec3.init(3.5, 3.5, -4.5), rotated.min, 1.0e-5);
    try expectClose(Vec3.init(11.5, 10.5, 2.5), rotated.max, 1.0e-5);
    var bounds_d = shape.getWorldSpaceBounds(Mat44.identity(), Vec3.one());
    bounds_d.translateDVec3(.init(1, 2, 3));
    try testing.expect(shape.getWorldSpaceBoundsDMat44(.fromMat44(Mat44.translation(Vec3.init(1, 2, 3))), Vec3.one()).eql(bounds_d)); // using Shape::GetWorldSpaceBounds

    try testing.expectEqual(@as(f32, 0.5), shape.getInnerRadius());
    try testing.expect(shape.getCenterOfMass().eql(Vec3.zero()));
    try testing.expect(!shape.mustBeStatic());

    // A triangle has no mass or volume
    const p = shape.getMassProperties();
    try testing.expectEqual(@as(f32, 0.0), p.mass);
    try testing.expect(p.inertia.eql(Mat44.zero()));
    try testing.expectEqual(@as(f32, 0.0), shape.getVolume());
    try testing.expectEqual(@as(usize, @sizeOf(TriangleShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 1), shape.getStats().num_triangles);

    // Surface normal: the normalized cross product (3, 3, 3) x (6, 6, 7), the position is ignored
    try expectClose(Vec3.init(1, -1, 0).normalized(), shape.getSurfaceNormal(.empty, Vec3.init(100, 200, 300)), 1.0e-6);

    // Degenerate triangle: Y axis
    var degenerate = TriangleShape.init(allocator, Vec3.init(1, 2, 3), Vec3.init(2, 4, 6), Vec3.init(3, 6, 9), .{});
    degenerate.asShape().setEmbedded();
    defer degenerate.asShapeMut().deinit();
    try testing.expect(degenerate.asShape().getSurfaceNormal(.empty, Vec3.zero()).eql(Vec3.axisY()));

    // Supporting face: the scaled and transformed triangle, flipped when the scale is inside out
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.axisX(), Vec3.replicate(2.0), translation, &face);
    try testing.expectEqual(@as(u32, 3), face.len);
    try testing.expect(face.at(0).eql(Vec3.init(3, 4, 6)) and face.at(1).eql(Vec3.init(9, 10, 12)) and face.at(2).eql(Vec3.init(15, 16, 20)));
    face.clear();
    shape.getSupportingFace(.empty, Vec3.axisX(), Vec3.init(-2, 2, 2), translation, &face);
    try testing.expectEqual(@as(u32, 3), face.len);
    try testing.expect(face.at(0).eql(Vec3.init(-1, 4, 6)) and face.at(1).eql(Vec3.init(-13, 16, 20)) and face.at(2).eql(Vec3.init(-7, 10, 12)));
}

test "TriangleShape: TestIsValidScale, the triangle part (ShapeTests.cpp)" {
    const allocator = testing.allocator;
    const min_scale_tolerance_sq: f32 = math.square(1.0e-6 * ScaleHelpers.min_scale);

    var triangle_ref = Ref(Shape).init((try TriangleShape.create(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{})).asShapeMut());
    defer triangle_ref.deinit();
    const triangle = triangle_ref.get().?;
    try testing.expect(!triangle.isValidScale(Vec3.zero()));
    try testing.expect(!triangle.isValidScale(Vec3.axisX()));
    try testing.expect(!triangle.isValidScale(Vec3.axisY()));
    try testing.expect(!triangle.isValidScale(Vec3.axisZ()));
    try testing.expect(triangle.isValidScale(Vec3.init(2, 2, 2)));
    try testing.expect(triangle.isValidScale(Vec3.init(-1, 1, -1)));
    try testing.expect(triangle.isValidScale(Vec3.init(2, 1, 1)));
    try testing.expect(triangle.isValidScale(Vec3.init(1, 2, 1)));
    try testing.expect(triangle.isValidScale(Vec3.init(1, 1, 2)));
    try testing.expect(triangle.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try testing.expect(triangle.makeScaleValid(Vec3.init(2, 5, -4)).eql(Vec3.init(2, 5, -4)));

    var triangle2_ref = Ref(Shape).init((try TriangleShape.create(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{ .convex_radius = 0.01 })).asShapeMut()); // With convex radius
    defer triangle2_ref.deinit();
    const triangle2 = triangle2_ref.get().?;
    try testing.expect(!triangle2.isValidScale(Vec3.zero()));
    try testing.expect(!triangle2.isValidScale(Vec3.axisX()));
    try testing.expect(!triangle2.isValidScale(Vec3.axisY()));
    try testing.expect(!triangle2.isValidScale(Vec3.axisZ()));
    try testing.expect(triangle2.isValidScale(Vec3.init(2, 2, 2)));
    try testing.expect(triangle2.isValidScale(Vec3.init(-1, 1, -1)));
    try testing.expect(!triangle2.isValidScale(Vec3.init(2, 1, 1)));
    try testing.expect(!triangle2.isValidScale(Vec3.init(1, 2, 1)));
    try testing.expect(!triangle2.isValidScale(Vec3.init(1, 1, 2)));
    try testing.expect(triangle2.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try testing.expect(triangle2.makeScaleValid(Vec3.init(2, 6, -4)).eql(Vec3.init(4, 4, -4)));
}

test "TriangleShape: more valid scales (Zolt only, not in Jolt)" {
    const allocator = testing.allocator;

    // Without convex radius any non zero scale is valid
    var triangle_ref = Ref(Shape).init((try TriangleShape.create(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{})).asShapeMut());
    defer triangle_ref.deinit();
    const triangle = triangle_ref.get().?;
    try testing.expect(triangle.isValidScale(Vec3.init(1, 1, 1)));
    try testing.expect(triangle.makeScaleValid(Vec3.init(-2, 0, 4)).eql(Vec3.init(-2, ScaleHelpers.min_scale, 4)));

    // With convex radius the scale must be uniform (signs may differ)
    var triangle2_ref = Ref(Shape).init((try TriangleShape.create(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{ .convex_radius = 0.01 })).asShapeMut());
    defer triangle2_ref.deinit();
    const triangle2 = triangle2_ref.get().?;
    try testing.expect(triangle2.isValidScale(Vec3.init(1, 1, 1)));
    try testing.expect(triangle2.makeScaleValid(Vec3.init(-2, 3, 4)).eql(Vec3.init(-3, 3, 3)));
}

test "TriangleShape: support functions of every mode" {
    const allocator = testing.allocator;

    var with_radius = TriangleShape.init(allocator, Vec3.zero(), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0), .{ .convex_radius = 0.5 });
    with_radius.asShape().setEmbedded();
    defer with_radius.asShapeMut().deinit();
    var without_radius = TriangleShape.init(allocator, Vec3.zero(), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0), .{});
    without_radius.asShape().setEmbedded();
    defer without_radius.asShapeMut().deinit();

    var buffer: ConvexShape.SupportBuffer = .{};
    const scale = Vec3.init(2, -1, 1);

    // Include convex radius and default: the scaled vertices plus the (unscaled) convex radius in the direction
    for ([_]ConvexShape.SupportMode{ .include_convex_radius, .default }) |mode| {
        const support = with_radius.base.getSupportFunction(mode, &buffer, scale);
        try testing.expectEqual(@as(f32, 0.5), support.getConvexRadius());
        try testing.expect(support.getSupport(Vec3.init(0, 3, 0)).eql(Vec3.init(2, 0.5, 0))); // Ties go to the last vertex
        try testing.expect(support.getSupport(Vec3.init(-4, 0, 0)).eql(Vec3.init(-0.5, 0, 1)));
        try testing.expect(support.getSupport(Vec3.zero()).eql(Vec3.init(2, 0, 0))); // Zero direction: no radius added

        // Without convex radius the mode falls through to the version without convex radius
        const no_radius = without_radius.base.getSupportFunction(mode, &buffer, scale);
        try testing.expectEqual(@as(f32, 0.0), no_radius.getConvexRadius());
        try testing.expect(no_radius.getSupport(Vec3.init(0, 3, 0)).eql(Vec3.init(2, 0, 0)));
    }

    // Exclude convex radius: the scaled vertices
    const exclude = with_radius.base.getSupportFunction(.exclude_convex_radius, &buffer, scale);
    try testing.expectEqual(@as(f32, 0.0), exclude.getConvexRadius());
    try testing.expect(exclude.getSupport(Vec3.init(1, 0, 0)).eql(Vec3.init(2, 0, 0)));
    try testing.expect(exclude.getSupport(Vec3.init(-1, 0, 0)).eql(Vec3.init(0, 0, 1)));
    try testing.expect(exclude.getSupport(Vec3.init(-1, 0, -1)).eql(Vec3.zero()));
}

test "TriangleShape: ray casts, back faces, collide point, filters and the collector context" {
    const allocator = testing.allocator;

    // Normal pointing up (Y)
    var triangle = TriangleShape.init(allocator, Vec3.zero(), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0), .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();
    const shape = triangle.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 1, 3);
    const from_above = RayCast.init(Vec3.init(0.25, 1, 0.25), Vec3.init(0, -2, 0));
    const from_below = RayCast.init(Vec3.init(0.25, -1, 0.25), Vec3.init(0, 2, 0));

    // Single hit: closer hits only, both sides
    var hit: RayCastResult = .{};
    try testing.expect(shape.castRay(from_above, creator, &hit));
    try testing.expectEqual(@as(f32, 0.5), hit.fraction);
    try testing.expect(hit.sub_shape_id2.eql(creator.getID()));
    try testing.expect(!shape.castRay(from_above, creator, &hit)); // Hit at 0.5 is not closer
    hit = .{};
    try testing.expect(shape.castRay(from_below, creator, &hit)); // No back face check in the single hit version
    try testing.expectEqual(@as(f32, 0.5), hit.fraction);
    hit = .{};
    try testing.expect(!shape.castRay(.init(Vec3.init(2, 1, 2), Vec3.init(0, -2, 0)), creator, &hit)); // Misses
    try testing.expect(!shape.castRay(.init(Vec3.init(0.25, 1, 0.25), Vec3.init(1, 0, 0)), creator, &hit)); // Parallel

    // Collector: back faces are ignored by default
    var settings: RayCastSettings = .{};
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(12), .{});
    hits.base.setContext(&context);
    shape.castRayCollector(from_below, &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    shape.castRayCollector(from_above, &settings, creator, &hits.base, &.{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.5), hits.hits.items[0].fraction);
    try testing.expect(hits.hits.items[0].body_id.eql(.init(12)) and hits.hits.items[0].sub_shape_id2.eql(creator.getID()));

    // Collide with back faces (the convex back face mode does not matter)
    hits.reset();
    settings.back_face_mode_triangles = .collide_with_back_faces;
    shape.castRayCollector(from_below, &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.5), hits.hits.items[0].fraction);

    // The early out fraction rejects hits that are further away
    hits.reset();
    hits.base.updateEarlyOutFraction(0.5);
    shape.castRayCollector(from_above, &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);

    // The shape filter is tested first
    hits.reset();
    var calls: u32 = 0;
    const reject: RejectAllFilter = .{ .calls = &calls };
    shape.castRayCollector(from_above, &settings, creator, &hits.base, &reject.base);
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    try testing.expectEqual(@as(u32, 1), calls);

    // Collide point: a triangle has no inside (not even its vertices)
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.zero(), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(0.25, 0, 0.25), creator, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 0), points.hits.items.len);
}

test "TriangleShape: GetSubmergedVolume (a triangle has no volume)" {
    const allocator = testing.allocator;

    var triangle = TriangleShape.init(allocator, Vec3.zero(), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0), .{ .convex_radius = 0.1 });
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    const r = triangle.asShape().getSubmergedVolume(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.one(), Plane.fromPointAndNormal(Vec3.init(0, 10, 0), Vec3.axisY()));
    try testing.expectEqual(@as(f32, 0.0), r.total_volume);
    try testing.expectEqual(@as(f32, 0.0), r.submerged_volume);
    try testing.expect(r.center_of_buoyancy.eql(Vec3.zero()));
}

test "TriangleShape: CollideSoftBodyVertices" {
    const allocator = testing.allocator;

    // Normal pointing up (Y)
    var triangle = TriangleShape.init(allocator, Vec3.zero(), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0), .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    // Transform: translate by (1, 2, 3), scale 2
    var positions = [_]Vec3{ Vec3.init(1.5, 2.2, 3.5), Vec3.init(1.5, 1.95, 3.5), Vec3.init(1.5, 1.5, 3.5), Vec3.init(4, 3, 3), Vec3.init(1.5, 1.95, 3.5) };
    var inv_masses = [_]f32{ 1, 1, 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 5;
    var penetrations = [_]f32{-math.flt_max} ** 5;
    var indices = [_]i32{-1} ** 5;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    triangle.asShape().collideSoftBodyVertices(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.replicate(2.0), &vertices, 5, 7);

    // Above the interior: negative penetration, the plane of the triangle
    try testing.expectApproxEqAbs(@as(f32, -0.2), penetrations[0], 1.0e-6);
    try testing.expectEqual(@as(i32, 7), indices[0]);
    try testing.expect(planes[0].getNormal().eql(Vec3.axisY()));
    try testing.expectApproxEqAbs(@as(f32, 0.0), planes[0].signedDistance(Vec3.init(1, 2, 3)), 1.0e-6);
    // Below the interior, less than the triangle thickness: positive penetration
    try testing.expectApproxEqAbs(@as(f32, 0.05), penetrations[1], 1.0e-6);
    try testing.expectEqual(@as(i32, 7), indices[1]);
    // Below the interior, more than the triangle thickness: no collision
    try testing.expectEqual(-math.flt_max, penetrations[2]);
    try testing.expectEqual(@as(i32, -1), indices[2]);
    // Outside, nearest to vertex 3 (at (3, 2, 3)): the distance as negative penetration
    try testing.expectApproxEqAbs(-@sqrt(@as(f32, 2.0)), penetrations[3], 1.0e-6);
    try testing.expectEqual(@as(i32, 7), indices[3]);
    try expectClose(Vec3.init(1, 1, 0).normalized(), planes[3].getNormal(), 1.0e-6);
    // Infinite mass: skipped
    try testing.expectEqual(-math.flt_max, penetrations[4]);
    try testing.expectEqual(@as(i32, -1), indices[4]);
}

test "TriangleShape: GetTrianglesStart / Next" {
    const allocator = testing.allocator;

    var triangle = TriangleShape.init(allocator, Vec3.zero(), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0), .{ .convex_radius = 0.1 });
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    var context: Shape.GetTrianglesContext = .{};
    triangle.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.init(1, 2, 3), Quat.identity(), Vec3.init(2, 3, -4));
    var vertices: [3 * Shape.get_triangles_min_triangles_requested]Float3 = undefined;
    var materials: [Shape.get_triangles_min_triangles_requested]*const PhysicsMaterial = undefined;
    try testing.expectEqual(@as(u32, 1), triangle.asShape().getTrianglesNext(&context, Shape.get_triangles_min_triangles_requested, &vertices, &materials));
    try testing.expect(Vec3.fromFloat3(vertices[0]).eql(Vec3.init(1, 2, 3)));
    try testing.expect(Vec3.fromFloat3(vertices[1]).eql(Vec3.init(1, 2, -1)));
    try testing.expect(Vec3.fromFloat3(vertices[2]).eql(Vec3.init(3, 2, 3)));
    try testing.expect(materials[0] == PhysicsMaterial.default);

    // Only the first call returns the triangle
    try testing.expectEqual(@as(u32, 0), triangle.asShape().getTrianglesNext(&context, Shape.get_triangles_min_triangles_requested, &vertices, null));

    // Without materials
    triangle.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.one());
    try testing.expectEqual(@as(u32, 1), triangle.asShape().getTrianglesNext(&context, Shape.get_triangles_min_triangles_requested, &vertices, null));
    try expectClose(Vec3.init(0, 1, 0), Vec3.fromFloat3(vertices[2]), 1.0e-6);
}

test "TriangleShape: binary state, restoreFromBinaryState and the registration" {
    const allocator = testing.allocator;

    var triangle = TriangleShape.init(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{ .convex_radius = 0.125 });
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();
    triangle.base.setDensity(321.0);
    triangle.asShapeMut().setUserData(5);

    var buffer: [128]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    triangle.asShape().saveBinaryState(out.streamOut());
    try testing.expectEqual(@as(usize, 1 + 8 + 4 + 3 * 12 + 4), writer.buffered().len); // Sub type, user data, density, vertices, convex radius

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    const restored = result.getPtr().?.cast(TriangleShape);
    try testing.expect(restored.getVertex1().eql(Vec3.init(1, 2, 3)) and restored.getVertex2().eql(Vec3.init(4, 5, 6)) and restored.getVertex3().eql(Vec3.init(7, 8, 9)));
    try testing.expectEqual(@as(f32, 0.125), restored.getConvexRadius());
    try testing.expectEqual(@as(f32, 321.0), restored.base.getDensity());
    try testing.expectEqual(@as(u64, 5), restored.asShape().getUserData());

    // ShapeFunctions
    const functions = ShapeFunctions.get(.triangle);
    try testing.expect(functions.construct != null);
    try testing.expect(functions.color.eql(Color.green));

    // The dispatch table: every convex shape vs triangle, specialized sphere vs triangle, reversed triangle vs convex
    // shape except triangle vs triangle
    const registry = &RegisterTypes.registry;
    for (ShapeFile.convex_sub_shape_types) |s| {
        if (s == .sphere) {
            try testing.expect(registry.getCollideShape(s, .triangle) == &TriangleShape.collideSphereVsTriangle);
            try testing.expect(registry.getCastShape(s, .triangle) == &TriangleShape.castSphereVsTriangle);
        } else {
            try testing.expect(registry.getCollideShape(s, .triangle) == &TriangleShape.collideConvexVsTriangle);
            try testing.expect(registry.getCastShape(s, .triangle) == &TriangleShape.castConvexVsTriangle);
        }
        if (s != .triangle) {
            try testing.expect(registry.getCollideShape(.triangle, s) == &CollisionDispatch.reversedCollideShape);
            try testing.expect(registry.getCastShape(.triangle, s) == &CollisionDispatch.reversedCastShape);
        }
    }
}

test "TriangleShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, TriangleShape.create(failing.allocator(), Vec3.zero(), Vec3.axisX(), Vec3.axisZ(), .{}));
    try testing.expectError(error.OutOfMemory, TriangleShapeSettings.create(failing.allocator(), Vec3.zero(), Vec3.axisX(), Vec3.axisZ(), .{}));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.triangle).construct.?(failing.allocator()));

    // Restore: the shape is the only allocation
    var triangle = TriangleShape.init(allocator, Vec3.zero(), Vec3.axisX(), Vec3.axisZ(), .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();
    var buffer: [128]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    triangle.asShape().saveBinaryState(out.streamOut());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    try testing.expectError(error.OutOfMemory, Shape.restoreFromBinaryState(failing.allocator(), in.streamIn()));
}

test "TriangleShape: collide through CollisionDispatch (sphere, convex, reversed, back faces)" {
    const allocator = testing.allocator;

    // Normal pointing up (Y)
    var triangle_ref = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-1, 0, -1), Vec3.init(-1, 0, 1), Vec3.init(1, 0, -1), .{})).asShape());
    defer triangle_ref.deinit();
    const triangle = triangle_ref.get().?;
    var sphere_settings = SphereShapeSettings.init(allocator, 0.5, .{});
    defer sphere_settings.deinit();
    var sphere_result = try sphere_settings.asShapeSettings().createShape(allocator);
    defer sphere_result.deinit();
    const sphere = sphere_result.getPtr().?;
    var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape());
    defer box_ref.deinit();
    const box = box_ref.get().?;

    const settings: CollideShapeSettings = .{};
    const creator1 = SubShapeIDCreator.pushID(.{}, 1, 2);
    const creator2 = SubShapeIDCreator.pushID(.{}, 2, 3);
    const above = Mat44.translation(Vec3.init(-0.25, 0.4, -0.25));

    // Sphere vs triangle (CollideSphereVsTriangles)
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(sphere, triangle, Vec3.one(), Vec3.one(), above, Mat44.identity(), creator1, creator2, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try expectClose(Vec3.init(-0.25, -0.1, -0.25), hit.contact_point_on1, 1.0e-5);
        try expectClose(Vec3.init(-0.25, 0, -0.25), hit.contact_point_on2, 1.0e-5);
        try expectClose(Vec3.init(0, -1, 0), hit.penetration_axis.normalized(), 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 0.1), hit.penetration_depth, 1.0e-5);
        try testing.expect(hit.sub_shape_id1.eql(creator1.getID()) and hit.sub_shape_id2.eql(creator2.getID()));

        // Reversed: triangle vs sphere
        var reversed = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer reversed.deinit();
        CollisionDispatch.collideShapeVsShape(triangle, sphere, Vec3.one(), Vec3.one(), Mat44.identity(), above, creator2, creator1, &settings, &reversed.base, &.{});
        try reversed.checkError();
        try testing.expectEqual(@as(usize, 1), reversed.hits.items.len);
        const r = &reversed.hits.items[0];
        try testing.expect(r.contact_point_on1.eql(hit.contact_point_on2) and r.contact_point_on2.eql(hit.contact_point_on1));
        try testing.expect(r.penetration_axis.eql(hit.penetration_axis.negate()));
        try testing.expectEqual(hit.penetration_depth, r.penetration_depth);
        try testing.expect(r.sub_shape_id1.eql(creator2.getID()) and r.sub_shape_id2.eql(creator1.getID()));
    }

    // Box vs triangle (CollideConvexVsTriangles) and reversed
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(box, triangle, Vec3.one(), Vec3.one(), above, Mat44.identity(), creator1, creator2, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.1), collector.hits.items[0].penetration_depth, 1.0e-5);
        try expectClose(Vec3.init(0, -1, 0), collector.hits.items[0].penetration_axis.normalized(), 1.0e-5);

        var reversed = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer reversed.deinit();
        CollisionDispatch.collideShapeVsShape(triangle, box, Vec3.one(), Vec3.one(), Mat44.identity(), above, creator2, creator1, &settings, &reversed.base, &.{});
        try reversed.checkError();
        try testing.expectEqual(@as(usize, 1), reversed.hits.items.len);
        try testing.expectEqual(collector.hits.items[0].penetration_depth, reversed.hits.items[0].penetration_depth);
        try testing.expect(reversed.hits.items[0].penetration_axis.eql(collector.hits.items[0].penetration_axis.negate()));
    }

    // Back faces: a sphere below the triangle only collides with back faces
    {
        const below = Mat44.translation(Vec3.init(-0.25, -0.4, -0.25));
        var no_back_faces = settings;
        no_back_faces.back_face_mode = .ignore_back_faces;
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(sphere, triangle, Vec3.one(), Vec3.one(), below, Mat44.identity(), creator1, creator2, &no_back_faces, &collector.base, &.{});
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);
        var back_faces = settings;
        back_faces.back_face_mode = .collide_with_back_faces;
        CollisionDispatch.collideShapeVsShape(sphere, triangle, Vec3.one(), Vec3.one(), below, Mat44.identity(), creator1, creator2, &back_faces, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try expectClose(Vec3.init(0, 1, 0), collector.hits.items[0].penetration_axis.normalized(), 1.0e-5);
    }
}

test "TriangleShape: TestCollideTriangleVsTriangle (CollideShapeTests.cpp)" {
    const allocator = testing.allocator;
    const penetration: f32 = 0.01;

    // A triangle centered around the origin in the XZ plane
    var t1 = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-1, 0, 1), Vec3.init(1, 0, 1), Vec3.init(0, 0, -1), .{})).asShape());
    defer t1.deinit();

    // A triangle in the XY plane with its tip just pointing in the origin
    var t2 = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-1, 1, 0), Vec3.init(1, 1, 0), Vec3.init(0, -penetration, 0), .{})).asShape());
    defer t2.deinit();

    const collide_settings: CollideShapeSettings = .{};
    var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(t1.get().?, t2.get().?, Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &collide_settings, &collector.base, &.{});

    try testing.expect(collector.hadHit());
    try expectClose(Vec3.zero(), collector.hit.contact_point_on1, 1.0e-6);
    try expectClose(Vec3.init(0, -penetration, 0), collector.hit.contact_point_on2, 1.0e-6);
    try testing.expectApproxEqAbs(penetration, collector.hit.penetration_depth, 1.0e-6);
    try expectClose(Vec3.init(0, 1, 0), collector.hit.penetration_axis.normalized(), 1.0e-6);
}

test "TriangleShape: TestTriangleVsBoxLargeSeparationDistance (CollideShapeTests.cpp)" {
    const allocator = testing.allocator;
    const triangle_x: f32 = -0.1;
    const half_extent: f32 = 10.0;
    var triangle_shape = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(triangle_x, -10, 10), Vec3.init(triangle_x, -10, -10), Vec3.init(triangle_x, 10, 0), .{})).asShape());
    defer triangle_shape.deinit();
    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(half_extent), .{})).asShape());
    defer box_shape.deinit();
    const distances = [_]f32{ 0.0, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0 };
    var num_hits: u32 = 0;
    for (distances) |x| {
        for (distances) |max_separation| {
            var collide_settings: CollideShapeSettings = .{};
            collide_settings.max_separation_distance = max_separation;
            var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
            defer collector.deinit();
            CollisionDispatch.collideShapeVsShape(triangle_shape.get().?, box_shape.get().?, Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(x, 0, 0)), .{}, .{}, &collide_settings, &collector.base, &.{});

            const expected_penetration = triangle_x - (x - half_extent);
            if (collector.hadHit()) {
                num_hits += 1;
                try testing.expectApproxEqAbs(expected_penetration, collector.hit.penetration_depth, 1.0e-3);
            } else {
                try testing.expect(expected_penetration < -max_separation);
                // Not ported: Jolt also checks the penetration axis of the default constructed (uninitialized) mHit here
            }
        }
    }
    try testing.expect(num_hits > 0 and num_hits < distances.len * distances.len); // Both branches
}

/// sTestCastSphereVertexOrEdge of CastShapeTests.cpp
fn testCastSphereVertexOrEdge(sphere: *const Shape, position: Vec3, direction: Vec3, triangle: *const Shape) !void {
    const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(position.sub(direction)), direction);
    var cast_settings: ShapeCastSettings = .{};
    cast_settings.back_face_mode_triangles = .collide_with_back_faces;
    cast_settings.back_face_mode_convex = .collide_with_back_faces;
    var collector = AllHitCollisionCollector(CastShapeCollector).init(testing.allocator);
    defer collector.deinit();
    CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
    try collector.checkError();
    try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
    const result = &collector.hits.items[collector.hits.items.len - 1];
    try testing.expectApproxEqAbs(1.0 - 0.2 / direction.length(), result.fraction, 1.0e-4);
    try expectClose(direction.normalized(), result.base.penetration_axis.normalized(), 1.0e-3);
    try testing.expectApproxEqAbs(@as(f32, 0.0), result.base.penetration_depth, 1.0e-3);
    try expectClose(position, result.base.contact_point_on1, 1.0e-3);
    try expectClose(position, result.base.contact_point_on2, 1.0e-3);
}

test "TriangleShape: TestCastSphereTriangle, the triangle shape half (CastShapeTests.cpp)" {
    const allocator = testing.allocator;

    // Create triangle
    var triangle_settings = TriangleShapeSettings.init(allocator, Vec3.init(50, 25, 0), Vec3.init(-50, 25, 0), Vec3.init(0, -25, 0), .{});
    defer triangle_settings.deinit();
    var triangle_result = try triangle_settings.asShapeSettings().createShape(allocator);
    defer triangle_result.deinit();
    const triangle = triangle_result.getPtr().?;

    // Create sphere
    var sphere_settings = SphereShapeSettings.init(allocator, 0.2, .{});
    defer sphere_settings.deinit();
    var sphere_result = try sphere_settings.asShapeSettings().createShape(allocator);
    defer sphere_result.deinit();
    const sphere = sphere_result.getPtr().?;

    {
        // Hit front face
        const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(Vec3.init(0, 0, 15)), Vec3.init(0, 0, -30));
        var cast_settings: ShapeCastSettings = .{};
        cast_settings.back_face_mode_triangles = .ignore_back_faces;
        cast_settings.back_face_mode_convex = .ignore_back_faces;
        cast_settings.return_deepest_point = false;
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const result = &collector.hits.items[0];
        try testing.expectApproxEqAbs((15.0 - 0.2) / 30.0, result.fraction, 1.0e-4);
        try expectClose(Vec3.init(0, 0, -1), result.base.penetration_axis.normalized(), 1.0e-3);
        try testing.expectEqual(@as(f32, 0.0), result.base.penetration_depth);
        try expectClose(Vec3.zero(), result.base.contact_point_on1, 1.0e-3);
        try expectClose(Vec3.zero(), result.base.contact_point_on2, 1.0e-3);
        try testing.expect(!result.is_back_face_hit);
    }

    {
        // Hit back face -> ignored
        const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(Vec3.init(0, 0, -15)), Vec3.init(0, 0, 30));
        var cast_settings: ShapeCastSettings = .{};
        cast_settings.back_face_mode_triangles = .ignore_back_faces;
        cast_settings.back_face_mode_convex = .ignore_back_faces;
        cast_settings.return_deepest_point = false;
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);

        // Hit back face -> collision
        cast_settings.back_face_mode_triangles = .collide_with_back_faces;
        cast_settings.back_face_mode_convex = .collide_with_back_faces;
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const result = &collector.hits.items[collector.hits.items.len - 1];
        try testing.expectApproxEqAbs((15.0 - 0.2) / 30.0, result.fraction, 1.0e-4);
        try expectClose(Vec3.init(0, 0, 1), result.base.penetration_axis.normalized(), 1.0e-3);
        try testing.expectEqual(@as(f32, 0.0), result.base.penetration_depth);
        try expectClose(Vec3.zero(), result.base.contact_point_on1, 1.0e-3);
        try expectClose(Vec3.zero(), result.base.contact_point_on2, 1.0e-3);
        try testing.expect(result.is_back_face_hit);
    }

    {
        // Hit back face while starting in collision -> ignored
        const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(Vec3.init(0, 0, -0.1)), Vec3.init(0, 0, 15));
        var cast_settings: ShapeCastSettings = .{};
        cast_settings.back_face_mode_triangles = .ignore_back_faces;
        cast_settings.back_face_mode_convex = .ignore_back_faces;
        cast_settings.return_deepest_point = true;
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);

        // Hit back face while starting in collision -> collision
        cast_settings.back_face_mode_triangles = .collide_with_back_faces;
        cast_settings.back_face_mode_convex = .collide_with_back_faces;
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const result = &collector.hits.items[collector.hits.items.len - 1];
        try testing.expectApproxEqAbs(@as(f32, 0.0), result.fraction, 1.0e-6);
        try expectClose(Vec3.init(0, 0, 1), result.base.penetration_axis.normalized(), 1.0e-3);
        try testing.expectApproxEqAbs(@as(f32, 0.1), result.base.penetration_depth, 1.0e-3);
        try expectClose(Vec3.init(0, 0, 0.1), result.base.contact_point_on1, 1.0e-3);
        try expectClose(Vec3.zero(), result.base.contact_point_on2, 1.0e-3);
        try testing.expect(result.is_back_face_hit);
    }

    // Hit vertex 1, 2 and 3
    try testCastSphereVertexOrEdge(sphere, Vec3.init(50, 25, 0), Vec3.init(-10, -10, 0), triangle);
    try testCastSphereVertexOrEdge(sphere, Vec3.init(-50, 25, 0), Vec3.init(10, -10, 0), triangle);
    try testCastSphereVertexOrEdge(sphere, Vec3.init(0, -25, 0), Vec3.init(0, 10, 0), triangle);

    // Hit edge 1, 2 and 3
    try testCastSphereVertexOrEdge(sphere, Vec3.init(0, 25, 0), Vec3.init(0, -10, 0), triangle); // Edge: Vec3(50, 25, 0), Vec3(-50, 25, 0)
    try testCastSphereVertexOrEdge(sphere, Vec3.init(-25, 0, 0), Vec3.init(10, 10, 0), triangle); // Edge: Vec3(-50, 25, 0), Vec3(0,-25, 0)
    try testCastSphereVertexOrEdge(sphere, Vec3.init(25, 0, 0), Vec3.init(-10, 10, 0), triangle); // Edge: Float3(0,-25, 0), Float3(50, 25, 0)
}

test "TriangleShape: cast through CollisionDispatch (box vs triangle, triangle vs box, triangle vs triangle)" {
    const allocator = testing.allocator;

    // Normal pointing up (Y)
    var triangle_ref = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-1, 0, -1), Vec3.init(-1, 0, 1), Vec3.init(1, 0, -1), .{})).asShape());
    defer triangle_ref.deinit();
    const triangle = triangle_ref.get().?;
    var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape());
    defer box_ref.deinit();
    const box = box_ref.get().?;
    const cast_settings: ShapeCastSettings = .{};

    // Box vs triangle (CastConvexVsTriangles): the bottom of the box (1.5 above the triangle) hits it at 1.5 / 4
    {
        const shape_cast = ShapeCast.init(box, Vec3.one(), Mat44.translation(Vec3.init(-0.5, 2, -0.5)), Vec3.init(0, -4, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.375), collector.hits.items[0].fraction, 1.0e-4);
        try expectClose(Vec3.init(0, -1, 0), collector.hits.items[0].base.penetration_axis.normalized(), 1.0e-3);
    }

    // Triangle vs box (reversed cast): the triangle moves up into the bottom of the box
    {
        const shape_cast = ShapeCast.init(triangle, Vec3.one(), Mat44.translation(Vec3.init(0.25, -2, 0.25)), Vec3.init(0, 4, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, box, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.375), collector.hits.items[0].fraction, 1.0e-4);
        try expectClose(Vec3.init(0, 1, 0), collector.hits.items[0].base.penetration_axis.normalized(), 1.0e-3);
    }

    // Triangle vs triangle (CastConvexVsTriangles, not reversed): a vertical triangle with its tip moving down into the triangle
    {
        var vertical = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-0.5, 1, 0), Vec3.init(0.5, 1, 0), Vec3.init(0, 0, 0), .{})).asShape());
        defer vertical.deinit();
        const shape_cast = ShapeCast.init(vertical.get().?, Vec3.one(), Mat44.translation(Vec3.init(-0.5, 1, -0.5)), Vec3.init(0, -2, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.5), collector.hits.items[0].fraction, 1.0e-4);
        try expectClose(Vec3.init(-0.5, 0, -0.5), collector.hits.items[0].base.contact_point_on2, 1.0e-3);
    }
}
