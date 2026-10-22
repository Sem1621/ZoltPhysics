//! Port of: Jolt/Physics/SoftBody/SoftBodyShape.h, Jolt/Physics/SoftBody/SoftBodyShape.cpp
//! Status: complete
//!
//! RemainingFeaturesArchitecture.md R15: a final pattern A shape (`base: Shape`, built with `Shape.vtableFor`). It is
//! only created by BodyManager.allocateSoftBody, embedded in the soft body next to its motion properties
//! (`SoftBodyWithMotionPropertiesAndShape`): `init` is Jolt's constructor, then BodyManager sets `motion_properties`
//! (Jolt: `mSoftBodyMotionProperties`, BodyManager is a friend) and marks the shape embedded.
//! - The private static collision functions that `register` puts in the dispatch table (`sCollideConvexVsSoftBody`,
//!   `sCollideSphereVsSoftBody`, `sCastConvexVsSoftBody`, `sCastSphereVsSoftBody`) keep their names without `s`. They
//!   drive the triangle algorithms with every face of the soft body and all edges active.
//! - `SBSGetTrianglesContext` is constructed in the caller's buffer (CollisionArchitecture.md D10). Jolt
//!   reinterpret_casts the buffer without constructing it.
//! - A null entry in the material list gives `PhysicsMaterial.default` in `getMaterial` and `getTrianglesNext`, where
//!   Jolt gives nullptr. The shared signatures cannot hold a null material (like MeshShape).
//! - Draw (JPH_DEBUG_RENDERER) is a gated declaration (RemainingFeaturesArchitecture.md R2).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../Core/Color.zig").Color;
const Core = @import("../../Core/Core.zig");
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const RayTriangle = @import("../../Geometry/RayTriangle.zig");
const Float3 = @import("../../Math/Float3.zig").Float3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const Quat = @import("../../Math/Quat.zig").Quat;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const MassProperties = @import("../Body/MassProperties.zig").MassProperties;
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const ConvexShape = @import("../Collision/Shape/ConvexShape.zig").ConvexShape;
const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;
const SubShapeIDFile = @import("../Collision/Shape/SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../Collision/RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../Collision/CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../Collision/CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../Collision/ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../Collision/ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const CastConvexVsTriangles = @import("../Collision/CastConvexVsTriangles.zig").CastConvexVsTriangles;
const CastSphereVsTriangles = @import("../Collision/CastSphereVsTriangles.zig").CastSphereVsTriangles;
const CollideConvexVsTriangles = @import("../Collision/CollideConvexVsTriangles.zig").CollideConvexVsTriangles;
const CollideSphereVsTriangles = @import("../Collision/CollideSphereVsTriangles.zig").CollideSphereVsTriangles;
const CollideSoftBodyVertexIterator = @import("../Collision/CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const CollisionDispatch = @import("../Collision/CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const SoftBodyMotionProperties = @import("SoftBodyMotionProperties.zig").SoftBodyMotionProperties;

/// Shape used exclusively for soft bodies. Adds the ability to perform collision checks against soft bodies.
pub const SoftBodyShape = struct {
    /// Concrete class: `Shape.cast(SoftBodyShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .soft_body;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .mustBeStatic, .getCenterOfMass, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .draw, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume };

    base: Shape,
    /// The motion properties of the soft body (Jolt: mSoftBodyMotionProperties, set by BodyManager)
    motion_properties: *const SoftBodyMotionProperties = undefined,

    /// Constructor (SoftBodyShape()): BodyManager sets `motion_properties` and marks the shape embedded
    pub fn init(allocator: Allocator) SoftBodyShape {
        return .{ .base = .init(Shape.vtableFor(SoftBodyShape), allocator, .soft_body, shape_sub_type) };
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const SoftBodyShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *SoftBodyShape) *Shape {
        return &self.base;
    }

    /// Determine amount of bits needed to encode sub shape id
    pub fn getSubShapeIDBits(self: *const SoftBodyShape) u32 {
        // Ensure we have enough bits to encode our shape [0, n - 1]
        const n: u32 = @as(u32, @intCast(self.motion_properties.getFaces().len)) -% 1;
        return 32 - @as(u32, @clz(n));
    }

    /// Convert a sub shape ID back to a face index
    pub fn getFaceIndex(self: *const SoftBodyShape, sub_shape_id: SubShapeID) u32 {
        const result = sub_shape_id.popID(self.getSubShapeIDBits());
        std.debug.assert(result.remainder.isEmpty());
        return result.id;
    }

    // See Shape
    pub fn mustBeStatic(self: *const SoftBodyShape) bool {
        _ = self;
        return false;
    }

    pub fn getCenterOfMass(self: *const SoftBodyShape) Vec3 {
        _ = self;
        return Vec3.zero();
    }

    pub fn getLocalBounds(self: *const SoftBodyShape) AABox {
        return self.motion_properties.getLocalBounds();
    }

    pub fn getSubShapeIDBitsRecursive(self: *const SoftBodyShape) u32 {
        return self.getSubShapeIDBits();
    }

    pub fn getInnerRadius(self: *const SoftBodyShape) f32 {
        _ = self;
        return 0.0;
    }

    pub fn getMassProperties(self: *const SoftBodyShape) MassProperties {
        _ = self;
        return .{};
    }

    pub fn getMaterial(self: *const SoftBodyShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        const result = sub_shape_id.popID(self.getSubShapeIDBits());
        const triangle_idx = result.id;
        std.debug.assert(result.remainder.isEmpty());

        const f = self.motion_properties.getFace(triangle_idx);
        // Zolt: a null entry of the material list returns the default material (Jolt returns nullptr), see the file header
        return self.motion_properties.getMaterials()[f.material_index].get() orelse PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(self: *const SoftBodyShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = local_surface_position;
        const result = sub_shape_id.popID(self.getSubShapeIDBits());
        const triangle_idx = result.id;
        std.debug.assert(result.remainder.isEmpty());

        const f = self.motion_properties.getFace(triangle_idx);
        const vertices = self.motion_properties.getVertices();

        const x1 = vertices[f.vertex[0]].position;
        const x2 = vertices[f.vertex[1]].position;
        const x3 = vertices[f.vertex[2]].position;

        return x2.sub(x1).cross(x3.sub(x1)).normalizedOr(Vec3.axisY());
    }

    pub fn getSupportingFace(self: *const SoftBodyShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = direction;
        const result = sub_shape_id.popID(self.getSubShapeIDBits());
        const triangle_idx = result.id;
        std.debug.assert(result.remainder.isEmpty());

        const f = self.motion_properties.getFace(triangle_idx);
        const vertices = self.motion_properties.getVertices();

        for (f.vertex) |i|
            out_vertices.append(center_of_mass_transform.mulVec3(scale.mul(vertices[i].position)));
    }

    pub fn getSubmergedVolume(self: *const SoftBodyShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane, opts: Shape.SubmergedVolumeOptions) Shape.SubmergedVolume {
        _ = .{ center_of_mass_transform, scale, surface, opts };
        return .{
            .submerged_volume = 0.0,
            .total_volume = self.motion_properties.getVolume(),
            .center_of_buoyancy = Vec3.zero(),
        };
    }

    // See Shape::Draw (JPH_DEBUG_RENDERER)
    pub const draw = if (Core.debug_renderer) drawImpl else {};
    fn drawImpl(self: *const SoftBodyShape, renderer: *DebugRenderer, center_of_mass_transform: RMat44, scale: Vec3, color: Color, use_material_colors: bool, draw_wireframe: bool) void {
        _ = .{ scale, use_material_colors, draw_wireframe };
        const vertices = self.motion_properties.getVertices();
        for (self.motion_properties.getFaces()) |*f| {
            const x1 = center_of_mass_transform.mulVec3(vertices[f.vertex[0]].position);
            const x2 = center_of_mass_transform.mulVec3(vertices[f.vertex[1]].position);
            const x3 = center_of_mass_transform.mulVec3(vertices[f.vertex[2]].position);

            renderer.drawTriangle(x1, x2, x3, color, .{ .cast_shadow = .on });
        }
    }

    pub fn castRay(self: *const SoftBodyShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const num_triangle_bits = self.getSubShapeIDBits();
        var triangle_idx: u32 = std.math.maxInt(u32);

        const vertices = self.motion_properties.getVertices();
        for (self.motion_properties.getFaces(), 0..) |*f, f_idx| {
            const x1 = vertices[f.vertex[0]].position;
            const x2 = vertices[f.vertex[1]].position;
            const x3 = vertices[f.vertex[2]].position;

            const fraction = RayTriangle.rayTriangle(ray.origin, ray.direction, x1, x2, x3);
            if (fraction < hit.fraction) {
                // Store fraction
                hit.fraction = fraction;

                // Store triangle index
                triangle_idx = @intCast(f_idx);
            }
        }

        if (triangle_idx == std.math.maxInt(u32))
            return false;

        hit.sub_shape_id2 = sub_shape_id_creator.pushID(triangle_idx, num_triangle_bits).getID();
        return true;
    }

    pub fn castRayCollector(self: *const SoftBodyShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const num_triangle_bits = self.getSubShapeIDBits();
        const check_backfaces = ray_cast_settings.back_face_mode_triangles == .ignore_back_faces and !self.motion_properties.getFacesDoubleSided();

        const vertices = self.motion_properties.getVertices();
        for (self.motion_properties.getFaces(), 0..) |*f, f_idx| {
            const x1 = vertices[f.vertex[0]].position;
            const x2 = vertices[f.vertex[1]].position;
            const x3 = vertices[f.vertex[2]].position;

            // Back facing check
            if (check_backfaces and x2.sub(x1).cross(x3.sub(x1)).dot(ray.direction) > 0.0)
                continue;

            // Test ray against triangle
            const fraction = RayTriangle.rayTriangle(ray.origin, ray.direction, x1, x2, x3);
            if (fraction < collector.getEarlyOutFraction()) {
                // Better hit than the current hit
                var hit: RayCastResult = .{};
                hit.body_id = TransformedShape.getBodyID(collector.getContext());
                hit.fraction = fraction;
                hit.sub_shape_id2 = sub_shape_id_creator.pushID(@intCast(f_idx), num_triangle_bits).getID();
                collector.addHit(&hit);
            }
        }
    }

    pub fn collidePoint(self: *const SoftBodyShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        Shape.collidePointUsingRayCast(self.asShape(), point, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collideSoftBodyVertices(self: *const SoftBodyShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = .{ self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index };
        // Not implemented
    }

    pub fn getTrianglesStart(self: *const SoftBodyShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = .{ self, box };
        context.emplace(SBSGetTrianglesContext).* = .{
            .center_of_mass_transform = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale)),
            .triangle_index = 0,
        };
    }

    pub fn getTrianglesNext(self: *const SoftBodyShape, context_in: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        const context = context_in.get(SBSGetTrianglesContext);

        const faces = self.motion_properties.getFaces();
        const vertices = self.motion_properties.getVertices();
        const materials = self.motion_properties.getMaterials();

        // Jolt's `outTriangleVertices++` / `outMaterials++` are the write positions `3 * i` and `i`
        const num_triangles: i32 = @min(@as(i32, @intCast(max_triangles_requested)), @as(i32, @intCast(faces.len)) - context.triangle_index);
        for (0..@intCast(num_triangles)) |i| {
            const f = &faces[@as(usize, @intCast(context.triangle_index)) + i];

            const x1 = context.center_of_mass_transform.mulVec3(vertices[f.vertex[0]].position);
            const x2 = context.center_of_mass_transform.mulVec3(vertices[f.vertex[1]].position);
            const x3 = context.center_of_mass_transform.mulVec3(vertices[f.vertex[2]].position);

            x1.storeFloat3(&out_triangle_vertices[3 * i]);
            x2.storeFloat3(&out_triangle_vertices[3 * i + 1]);
            x3.storeFloat3(&out_triangle_vertices[3 * i + 2]);

            // Zolt: a null entry of the material list returns the default material (Jolt returns nullptr), see the file header
            if (out_materials) |m|
                m[i] = materials[f.material_index].get() orelse PhysicsMaterial.default;
        }

        context.triangle_index += num_triangles;
        return @intCast(num_triangles);
    }

    pub fn getStats(self: *const SoftBodyShape) Shape.Stats {
        return .init(@sizeOf(SoftBodyShape), @intCast(self.motion_properties.getFaces().len));
    }

    pub fn getVolume(self: *const SoftBodyShape) f32 {
        return self.motion_properties.getVolume();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.soft_body);
        f.construct = null; // Not supposed to be constructed by users!
        f.color = Color.dark_green;

        for (ShapeFile.convex_sub_shape_types) |s| {
            r.registerCollideShape(s, .soft_body, collideConvexVsSoftBody);
            r.registerCastShape(s, .soft_body, castConvexVsSoftBody);

            r.registerCollideShape(.soft_body, s, CollisionDispatch.reversedCollideShape);
            r.registerCastShape(.soft_body, s, CollisionDispatch.reversedCastShape);
        }

        // Specialized collision functions
        r.registerCollideShape(.sphere, .soft_body, collideSphereVsSoftBody);
        r.registerCastShape(.sphere, .soft_body, castSphereVsSoftBody);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch (private static in Jolt)

    /// sCollideConvexVsSoftBody
    fn collideConvexVsSoftBody(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        std.debug.assert(shape1_in.getType() == .convex);
        const shape1 = shape1_in.cast(ConvexShape);
        std.debug.assert(shape2_in.getSubType() == .soft_body);
        const shape2 = shape2_in.cast(SoftBodyShape);

        const vertices = shape2.motion_properties.getVertices();
        const faces = shape2.motion_properties.getFaces();
        const num_triangle_bits = shape2.getSubShapeIDBits();

        var settings = collide_shape_settings.*;
        if (shape2.motion_properties.getFacesDoubleSided())
            settings.back_face_mode = .collide_with_back_faces;
        var collider = CollideConvexVsTriangles.init(shape1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), &settings, collector);
        for (faces, 0..) |*f, f_idx| {
            const x1 = vertices[f.vertex[0]].position;
            const x2 = vertices[f.vertex[1]].position;
            const x3 = vertices[f.vertex[2]].position;

            collider.collide(x1, x2, x3, 0b111, sub_shape_id_creator2.pushID(@intCast(f_idx), num_triangle_bits).getID());
        }
    }

    /// sCollideSphereVsSoftBody
    fn collideSphereVsSoftBody(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        std.debug.assert(shape1_in.getSubType() == .sphere);
        const shape1 = shape1_in.cast(SphereShape);
        std.debug.assert(shape2_in.getSubType() == .soft_body);
        const shape2 = shape2_in.cast(SoftBodyShape);

        const vertices = shape2.motion_properties.getVertices();
        const faces = shape2.motion_properties.getFaces();
        const num_triangle_bits = shape2.getSubShapeIDBits();

        var settings = collide_shape_settings.*;
        if (shape2.motion_properties.getFacesDoubleSided())
            settings.back_face_mode = .collide_with_back_faces;
        var collider = CollideSphereVsTriangles.init(shape1, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1.getID(), &settings, collector);
        for (faces, 0..) |*f, f_idx| {
            const x1 = vertices[f.vertex[0]].position;
            const x2 = vertices[f.vertex[1]].position;
            const x3 = vertices[f.vertex[2]].position;

            collider.collide(x1, x2, x3, 0b111, sub_shape_id_creator2.pushID(@intCast(f_idx), num_triangle_bits).getID());
        }
    }

    /// sCastConvexVsSoftBody
    fn castConvexVsSoftBody(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        std.debug.assert(shape_in.getSubType() == .soft_body);
        const shape = shape_in.cast(SoftBodyShape);

        const vertices = shape.motion_properties.getVertices();
        const faces = shape.motion_properties.getFaces();
        const num_triangle_bits = shape.getSubShapeIDBits();

        var settings = shape_cast_settings.*;
        if (shape.motion_properties.getFacesDoubleSided())
            settings.back_face_mode_triangles = .collide_with_back_faces;
        var caster = CastConvexVsTriangles.init(shape_cast, &settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector);
        for (faces, 0..) |*f, f_idx| {
            const x1 = vertices[f.vertex[0]].position;
            const x2 = vertices[f.vertex[1]].position;
            const x3 = vertices[f.vertex[2]].position;

            caster.cast(x1, x2, x3, 0b111, sub_shape_id_creator2.pushID(@intCast(f_idx), num_triangle_bits).getID());
        }
    }

    /// sCastSphereVsSoftBody
    fn castSphereVsSoftBody(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        std.debug.assert(shape_in.getSubType() == .soft_body);
        const shape = shape_in.cast(SoftBodyShape);

        const vertices = shape.motion_properties.getVertices();
        const faces = shape.motion_properties.getFaces();
        const num_triangle_bits = shape.getSubShapeIDBits();

        var settings = shape_cast_settings.*;
        if (shape.motion_properties.getFacesDoubleSided())
            settings.back_face_mode_triangles = .collide_with_back_faces;
        var caster = CastSphereVsTriangles.init(shape_cast, &settings, scale, center_of_mass_transform2, sub_shape_id_creator1, collector);
        for (faces, 0..) |*f, f_idx| {
            const x1 = vertices[f.vertex[0]].position;
            const x2 = vertices[f.vertex[1]].position;
            const x3 = vertices[f.vertex[2]].position;

            caster.cast(x1, x2, x3, 0b111, sub_shape_id_creator2.pushID(@intCast(f_idx), num_triangle_bits).getID());
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Context for GetTrianglesStart/Next (`struct SBSGetTrianglesContext`)

    const SBSGetTrianglesContext = struct {
        center_of_mass_transform: Mat44,
        triangle_index: i32,

        comptime {
            std.debug.assert(@sizeOf(SBSGetTrianglesContext) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt tests SoftBodyShape only through a PhysicsSystem, in UnitTests/Physics/SoftBodyTests.cpp; the bit exact
// comparison with Jolt is the soft body part of ZoltParity/Physics/PairwiseParity.zig)

const testing = std.testing;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const CollisionCollectorImpl = @import("../Collision/CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const PhysicsMaterialSimple = @import("../Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const BoxShape = @import("../Collision/Shape/BoxShape.zig").BoxShape;
const SphereShapeSettings = @import("../Collision/Shape/SphereShape.zig").SphereShapeSettings;
const RegisterTypes = @import("../../RegisterTypes.zig");
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;
const SoftBodyCreationSettings = @import("SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const SoftBodyVertex = @import("SoftBodyVertex.zig").SoftBodyVertex;

/// CHECK_APPROX_EQUAL for vectors: `IsClose(b, tolerance^2)`
fn expectClose(expected: Vec3, actual: Vec3, tolerance: f32) !void {
    if (!actual.isClose(expected, .{ .max_dist_sq = tolerance * tolerance })) {
        std.debug.print("expected {any}, got {any}\n", .{ expected.value, actual.value });
        return error.TestExpectedApproxEq;
    }
}

/// A soft body for the tests: the motion properties and the embedded shape (like BodyManager's
/// SoftBodyWithMotionPropertiesAndShape). The surface of the tetrahedron (0, 0, 0), (1, 0, 0), (0, 0, 1), (0, 1, 0)
/// with outward facing faces: 0 is y = 0, 1 is z = 0, 2 is x = 0 and 3 is x + y + z = 1.
const TestSoftBody = struct {
    motion_properties: SoftBodyMotionProperties,
    shape: SoftBodyShape,

    /// Create it in place (the shape points to the motion properties). `materials` replaces the default material list,
    /// `material_indices` are the material indices of the 4 faces.
    fn init(self: *TestSoftBody, allocator: Allocator, opts: struct { double_sided: bool = false, materials: []const ?*const PhysicsMaterial = &.{}, material_indices: [4]u32 = @splat(0) }) !void {
        const shared = try SoftBodySharedSettings.create(allocator);
        var creation: SoftBodyCreationSettings = .init(shared, .zero(), Quat.identity(), 0);
        defer creation.deinit();
        creation.faces_double_sided = opts.double_sided;
        for ([_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 1 }, .{ 0, 1, 0 } }) |p|
            try shared.vertices.append(allocator, .init(.{ .x = p[0], .y = p[1], .z = p[2] }, .{}));
        for ([_][3]u32{ .{ 0, 1, 2 }, .{ 0, 3, 1 }, .{ 0, 2, 3 }, .{ 1, 3, 2 } }, opts.material_indices) |f, m|
            try shared.addFace(.init(f[0], f[1], f[2], .{ .material_index = m }));
        if (opts.materials.len > 0) {
            for (shared.materials.items) |*m| m.deinit();
            shared.materials.clearRetainingCapacity();
            for (opts.materials) |m| try shared.materials.append(allocator, .init(m));
        }
        // Zolt (UB2): initialize needs an update group, the shape does not use it (Jolt's Optimize is not needed)
        try shared.update_groups.append(allocator, std.mem.zeroes(SoftBodySharedSettings.UpdateGroup));

        self.motion_properties = .{ .allocator = allocator };
        errdefer self.motion_properties.deinit();
        try self.motion_properties.initialize(&creation);

        self.shape = .init(allocator);
        self.shape.motion_properties = &self.motion_properties;
        self.shape.asShape().setEmbedded();
    }

    fn deinit(self: *TestSoftBody) void {
        self.shape.asShapeMut().deinit();
        self.motion_properties.deinit();
    }

    /// The sub shape ID of a face
    fn faceID(self: *const TestSoftBody, creator: SubShapeIDCreator, face: u32) SubShapeID {
        return creator.pushID(face, self.shape.getSubShapeIDBits()).getID();
    }
};

test "SoftBodyShape: the shape of a soft body" {
    const allocator = testing.allocator;
    const expectEqual = testing.expectEqual;

    var sb: TestSoftBody = undefined;
    try sb.init(allocator, .{});
    defer sb.deinit();
    const shape = &sb.shape;

    const s = shape.asShape();
    try testing.expect(s.isKindOf(SoftBodyShape));
    try expectEqual(ShapeFile.ShapeType.soft_body, s.getType());
    try testing.expect(!s.mustBeStatic());
    try testing.expect(s.getCenterOfMass().eql(Vec3.zero()));
    try testing.expect(s.getLocalBounds().eql(AABox.init(Vec3.zero(), Vec3.one())));
    try expectEqual(@as(f32, 0.0), s.getInnerRadius());
    try expectEqual(@as(f32, 0.0), s.getMassProperties().mass);

    // 4 faces need 2 bits; a face index survives the sub shape ID round trip
    try expectEqual(@as(u32, 2), s.getSubShapeIDBitsRecursive());
    const creator: SubShapeIDCreator = .{};
    const id = creator.pushID(3, shape.getSubShapeIDBits()).getID();
    try expectEqual(@as(u32, 3), shape.getFaceIndex(id));
    try expectEqual(@as(u32, 4), s.getStats().num_triangles);

    // The tetrahedron has volume 1/6. Its faces point outwards, so the volume is positive. Only face (1, 3, 2) adds to
    // it: (1, 0, 0) x (0, 1, 0) . (0, 0, 1) = 1, the other faces have vertex 0 at the origin.
    try expectEqual(@as(f32, 1.0 / 6.0), s.getVolume());

    // A soft body is not submerged: the total volume is its volume, the rest is zero
    const submerged = s.getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.init(Vec3.axisY(), 0.5), .{});
    try expectEqual(@as(f32, 1.0 / 6.0), submerged.total_volume);
    try expectEqual(@as(f32, 0.0), submerged.submerged_volume);
    try testing.expect(submerged.center_of_buoyancy.eql(Vec3.zero()));

    // The bits for 1 face and for no face at all (n - 1 wraps around, like Jolt's uint32)
    const shared: *SoftBodySharedSettings = @constCast(sb.motion_properties.getSettings().?);
    shared.faces.items.len = 1;
    try expectEqual(@as(u32, 0), shape.getSubShapeIDBits());
    shared.faces.items.len = 0;
    try expectEqual(@as(u32, 32), shape.getSubShapeIDBits());
    shared.faces.items.len = 4;
}

test "SoftBodyShape: ray casts, back faces, collide point and the shape filter" {
    const allocator = testing.allocator;
    const expectEqual = testing.expectEqual;

    var single: TestSoftBody = undefined;
    try single.init(allocator, .{});
    defer single.deinit();
    var double: TestSoftBody = undefined;
    try double.init(allocator, .{ .double_sided = true });
    defer double.deinit();

    const creator = SubShapeIDCreator.pushID(.{}, 5, 3);

    // A ray upwards through face 0 (y = 0, front face) at fraction 0.5, then face 3 (back face) at 0.75
    const ray = RayCast.init(Vec3.init(0.25, -1, 0.25), Vec3.init(0, 2, 0));
    {
        var hit: RayCastResult = .{};
        try testing.expect(single.shape.castRay(ray, creator, &hit));
        try testing.expectApproxEqAbs(@as(f32, 0.5), hit.fraction, 1.0e-6);
        try testing.expect(hit.sub_shape_id2.eql(single.faceID(creator, 0)));

        // A hit that is closer already: no hit, the hit stays untouched
        var closer: RayCastResult = .{};
        closer.fraction = 0.25;
        try testing.expect(!single.shape.castRay(ray, creator, &closer));
        try expectEqual(@as(f32, 0.25), closer.fraction);
        try testing.expect(closer.sub_shape_id2.isEmpty());
    }

    // The collector version: single sided faces skip the back face unless the settings collide with back faces
    for ([_]bool{ false, true }) |double_sided| {
        for ([_]bool{ false, true }) |back_faces| {
            const shape = if (double_sided) &double.shape else &single.shape;
            var settings: RayCastSettings = .{};
            settings.back_face_mode_triangles = if (back_faces) .collide_with_back_faces else .ignore_back_faces;
            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            try collector.checkError();
            const expected: usize = if (double_sided or back_faces) 2 else 1;
            try expectEqual(expected, collector.hits.items.len);
            try testing.expectApproxEqAbs(@as(f32, 0.5), collector.hits.items[0].fraction, 1.0e-6);
            try testing.expect(collector.hits.items[0].sub_shape_id2.eql(single.faceID(creator, 0)));
            if (expected == 2) {
                try testing.expectApproxEqAbs(@as(f32, 0.75), collector.hits.items[1].fraction, 1.0e-6);
                try testing.expect(collector.hits.items[1].sub_shape_id2.eql(single.faceID(creator, 3)));
            }
        }
    }

    // A shape filter that rejects the shape: no hits and no faces tested
    {
        const RejectAll = struct {
            pub const overrides = .{.shouldCollide};
            base: ShapeFilter = .init(@This()),
            calls: *u32,
            pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
                _ = sub_shape_id_of_shape2;
                std.debug.assert(shape2.getSubType() == .soft_body);
                self.calls.* += 1;
                return false;
            }
        };
        var calls: u32 = 0;
        const filter: RejectAll = .{ .calls = &calls };
        var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
        defer collector.deinit();
        single.shape.castRayCollector(ray, &.{}, creator, &collector.base, &filter.base);
        try expectEqual(@as(usize, 0), collector.hits.items.len);
        try expectEqual(@as(u32, 1), calls);
    }

    // Collide point: a ray up from the point hits 1 face when the point is inside
    {
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        single.shape.collidePoint(Vec3.init(0.2, 0.2, 0.2), creator, &collector.base, &.{}); // Inside
        single.shape.collidePoint(Vec3.init(0.9, 0.5, 0.9), creator, &collector.base, &.{}); // In the bounds, outside the tetrahedron
        single.shape.collidePoint(Vec3.init(2, 0, 0), creator, &collector.base, &.{}); // Outside the bounds
        try collector.checkError();
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expect(collector.hits.items[0].sub_shape_id2.eql(single.faceID(creator, 3)));
    }
}

test "SoftBodyShape: materials, surface normal, supporting face and GetTrianglesStart / Next" {
    const allocator = testing.allocator;
    const expectEqual = testing.expectEqual;

    const red = try PhysicsMaterialSimple.create(allocator, "Red", Color.red);
    var red_ref = RefConst(PhysicsMaterial).init(red.material());
    defer red_ref.deinit();
    const blue = try PhysicsMaterialSimple.create(allocator, "Blue", Color.blue);
    var blue_ref = RefConst(PhysicsMaterial).init(blue.material());
    defer blue_ref.deinit();

    // Face 2 uses a null entry of the material list (Zolt: the default material, see the file header)
    var sb: TestSoftBody = undefined;
    try sb.init(allocator, .{ .materials = &.{ red.material(), blue.material(), null }, .material_indices = .{ 0, 1, 2, 1 } });
    defer sb.deinit();
    const shape = &sb.shape;
    const s = shape.asShape();

    try testing.expect(s.getMaterial(sb.faceID(.{}, 0)) == red.material());
    try testing.expect(s.getMaterial(sb.faceID(.{}, 1)) == blue.material());
    try testing.expect(s.getMaterial(sb.faceID(.{}, 2)) == PhysicsMaterial.default);
    try testing.expect(s.getMaterial(sb.faceID(.{}, 3)) == blue.material());

    // The normal of the face (unit length), the position does not matter
    try expectClose(Vec3.init(0, -1, 0), s.getSurfaceNormal(sb.faceID(.{}, 0), Vec3.zero()), 1.0e-6);
    try expectClose(Vec3.replicate(1.0 / @sqrt(3.0)), s.getSurfaceNormal(sb.faceID(.{}, 3), Vec3.init(5, 5, 5)), 1.0e-6);

    // The supporting face: the 3 vertices of the face, scaled and transformed (the direction does not matter)
    {
        var face: Shape.SupportingFace = .empty;
        const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5 * std.math.pi), Vec3.init(1, 2, 3));
        const scale = Vec3.init(2, 3, 4);
        s.getSupportingFace(sb.faceID(.{}, 3), Vec3.axisX(), scale, transform, &face);
        try expectEqual(@as(usize, 3), face.len);
        for (face.constSlice(), [_]Vec3{ Vec3.init(1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0, 0, 1) }) |actual, local|
            try testing.expect(actual.eql(transform.mulVec3(scale.mul(local))));
    }

    // All triangles in one go (with materials) and then no more
    {
        var context: Shape.GetTrianglesContext = .{};
        const rotation = Quat.rotation(Vec3.axisZ(), 0.25 * std.math.pi);
        const position = Vec3.init(-1, 0, 2);
        const scale = Vec3.init(1, -2, 1);
        s.getTrianglesStart(&context, AABox.init(Vec3.replicate(-10), Vec3.replicate(10)), position, rotation, scale);
        var vertices: [3 * 32]Float3 = undefined;
        var materials: [32]*const PhysicsMaterial = undefined;
        try expectEqual(@as(u32, 4), s.getTrianglesNext(&context, 32, &vertices, &materials));
        const transform = Mat44.rotationTranslation(rotation, position).mul(Mat44.scaleVec3(scale));
        var expected: Float3 = undefined;
        transform.mulVec3(Vec3.init(1, 0, 0)).storeFloat3(&expected); // Face 3 starts with vertex 1
        try testing.expect(std.meta.eql(expected, vertices[9]));
        try testing.expect(materials[0] == red.material() and materials[1] == blue.material() and materials[2] == PhysicsMaterial.default and materials[3] == blue.material());
        try expectEqual(@as(u32, 0), s.getTrianglesNext(&context, 32, &vertices, &materials));
    }

    // In batches of 3 (Jolt does not check cGetTrianglesMinTrianglesRequested) and without materials
    {
        var context: Shape.GetTrianglesContext = .{};
        s.getTrianglesStart(&context, .empty, Vec3.zero(), Quat.identity(), Vec3.one());
        var vertices: [3 * 3]Float3 = undefined;
        try expectEqual(@as(u32, 3), s.getTrianglesNext(&context, 3, &vertices, null));
        try expectEqual(@as(u32, 1), s.getTrianglesNext(&context, 3, &vertices, null));
        try testing.expect(std.meta.eql(Float3{ .x = 1, .y = 0, .z = 0 }, vertices[0])); // Face 3, vertex 1
        try expectEqual(@as(u32, 0), s.getTrianglesNext(&context, 3, &vertices, null));
    }

    // CollideSoftBodyVertices is not implemented: the vertices stay the same
    {
        var vertices: [2]SoftBodyVertex = .{ sb.motion_properties.getVertices()[0], sb.motion_properties.getVertices()[1] };
        const before = vertices;
        const iterator = CollideSoftBodyVertexIterator.initFromVertices(&vertices);
        s.collideSoftBodyVertices(Mat44.identity(), Vec3.one(), &iterator, 2, 0);
        try testing.expect(std.mem.eql(u8, std.mem.asBytes(&before), std.mem.asBytes(&vertices)));
    }
}

test "SoftBodyShape: the registration and the collision functions" {
    const allocator = testing.allocator;
    const expectEqual = testing.expectEqual;
    const registry = &RegisterTypes.registry;

    // sRegister: convex and sphere vs soft body, reversed the other way around, nothing against other shapes
    try testing.expect(registry.getCollideShape(.box, .soft_body) == &SoftBodyShape.collideConvexVsSoftBody);
    try testing.expect(registry.getCastShape(.convex_hull, .soft_body) == &SoftBodyShape.castConvexVsSoftBody);
    try testing.expect(registry.getCollideShape(.sphere, .soft_body) == &SoftBodyShape.collideSphereVsSoftBody);
    try testing.expect(registry.getCastShape(.sphere, .soft_body) == &SoftBodyShape.castSphereVsSoftBody);
    try testing.expect(registry.getCollideShape(.soft_body, .sphere) == &CollisionDispatch.reversedCollideShape);
    try testing.expect(registry.getCastShape(.soft_body, .triangle) == &CollisionDispatch.reversedCastShape);
    for ([_]ShapeSubType{ .soft_body, .mesh, .height_field, .plane }) |t| {
        try testing.expect(registry.getCollideShape(.soft_body, t) == &CollisionDispatch.collideUnsupported);
        try testing.expect(registry.getCastShape(t, .soft_body) == &CollisionDispatch.castUnsupported);
    }
    try testing.expect(ShapeFile.ShapeFunctions.get(.soft_body).construct == null);
    try testing.expect(ShapeFile.ShapeFunctions.get(.soft_body).color.eql(Color.dark_green));

    var single: TestSoftBody = undefined;
    try single.init(allocator, .{});
    defer single.deinit();
    var double: TestSoftBody = undefined;
    try double.init(allocator, .{ .double_sided = true });
    defer double.deinit();

    var sphere_settings = SphereShapeSettings.init(allocator, 0.15, .{});
    defer sphere_settings.deinit();
    var sphere_result = try sphere_settings.asShapeSettings().createShape(allocator);
    defer sphere_result.deinit();
    const sphere = sphere_result.getPtr().?;
    var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.15), .{})).asShape());
    defer box_ref.deinit();
    const box = box_ref.get().?;

    const settings: CollideShapeSettings = .{};
    const creator1 = SubShapeIDCreator.pushID(.{}, 1, 2);
    const creator2 = SubShapeIDCreator.pushID(.{}, 2, 3);

    // A sphere and a box that reach 0.1 through face 0 from below (front side)
    const below = Mat44.translation(Vec3.init(0.25, -0.05, 0.25));
    for ([_]*const Shape{ sphere, box }) |shape1| {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(shape1, single.shape.asShape(), Vec3.one(), Vec3.one(), below, Mat44.identity(), creator1, creator2, &settings, &collector.base, &.{});
        try collector.checkError();
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try testing.expectApproxEqAbs(@as(f32, 0.1), hit.penetration_depth, 1.0e-5);
        try expectClose(Vec3.init(0, 1, 0), hit.penetration_axis.normalized(), 1.0e-5);
        try testing.expect(hit.sub_shape_id1.eql(creator1.getID()) and hit.sub_shape_id2.eql(single.faceID(creator2, 0)));

        // Reversed: soft body vs sphere / box
        var reversed = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer reversed.deinit();
        CollisionDispatch.collideShapeVsShape(single.shape.asShape(), shape1, Vec3.one(), Vec3.one(), Mat44.identity(), below, creator2, creator1, &settings, &reversed.base, &.{});
        try reversed.checkError();
        try expectEqual(@as(usize, 1), reversed.hits.items.len);
        try expectEqual(hit.penetration_depth, reversed.hits.items[0].penetration_depth);
        try testing.expect(reversed.hits.items[0].penetration_axis.eql(hit.penetration_axis.negate()));
        try testing.expect(reversed.hits.items[0].sub_shape_id1.eql(single.faceID(creator2, 0)));
    }

    // A sphere inside the tetrahedron that touches face 0 from the back: only double sided faces (or the settings)
    // collide with it
    const inside = Mat44.translation(Vec3.init(0.25, 0.1, 0.25));
    for ([_]bool{ false, true }) |double_sided| {
        for ([_]bool{ false, true }) |back_faces| {
            const soft_body = if (double_sided) &double else &single;
            var back_face_settings = settings;
            back_face_settings.back_face_mode = if (back_faces) .collide_with_back_faces else .ignore_back_faces;
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            CollisionDispatch.collideShapeVsShape(sphere, soft_body.shape.asShape(), Vec3.one(), Vec3.one(), inside, Mat44.identity(), creator1, creator2, &back_face_settings, &collector.base, &.{});
            try collector.checkError();
            try expectEqual(@as(usize, if (double_sided or back_faces) 1 else 0), collector.hits.items.len);
        }
    }

    // Casts upwards into face 0: the sphere and the box hit it at (1 - 0.15) / 2, the back face 3 is ignored
    const cast_settings: ShapeCastSettings = .{};
    for ([_]*const Shape{ sphere, box }) |shape1| {
        const shape_cast = ShapeCast.init(shape1, Vec3.one(), Mat44.translation(Vec3.init(0.25, -1, 0.25)), Vec3.init(0, 2, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, single.shape.asShape(), Vec3.one(), &.{}, Mat44.identity(), creator1, creator2, &collector.base);
        try collector.checkError();
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.425), collector.hits.items[0].fraction, 1.0e-4);
        try testing.expect(collector.hits.items[0].base.sub_shape_id2.eql(single.faceID(creator2, 0)));

        // Double sided faces: face 3 is hit from the back as well
        var double_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer double_collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, double.shape.asShape(), Vec3.one(), &.{}, Mat44.identity(), creator1, creator2, &double_collector.base);
        try double_collector.checkError();
        try expectEqual(@as(usize, 2), double_collector.hits.items.len);
    }

    // Reversed cast (in the local space of the sphere): the soft body moves down onto the sphere, face 0 hits it at
    // (1 - 0.15) / 2
    {
        const shape_cast = ShapeCast.init(single.shape.asShape(), Vec3.one(), Mat44.translation(Vec3.init(-0.25, 1, -0.25)), Vec3.init(0, -2, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, sphere, Vec3.one(), &.{}, Mat44.identity(), creator2, creator1, &collector.base);
        try collector.checkError();
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.425), collector.hits.items[0].fraction, 1.0e-4);
        try testing.expect(collector.hits.items[0].base.sub_shape_id1.eql(single.faceID(creator2, 0)));
    }
}
