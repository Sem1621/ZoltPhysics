//! Port of: Jolt/Physics/Collision/Shape/TriangleShape.h, Jolt/Physics/Collision/Shape/TriangleShape.cpp (prototype)
//! Status: stub
//!
//! Reduced to what the prototype needs to show Jolt's registration semantics with real Jolt code: `register` is a
//! line by line port of TriangleShape::sRegister. ConvexShape.register first registers convex vs convex for the
//! triangle, then this file overrides (convex, triangle) with its own function, (triangle, convex) with
//! CollisionDispatch.reversedCollideShape (except triangle vs triangle) and (sphere, triangle) with a specialized
//! function. The collision functions themselves are stand-ins that forward to the convex vs convex algorithm (the real
//! port uses CollideConvexVsTriangles / CollideSphereVsTriangles); castRay / collidePoint use ConvexShape's fallbacks
//! instead of Jolt's analytic versions; settings, GetSupportingFace, IsValidScale and MakeScaleValid are missing.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const AABox = zolt.AABox;
const Color = zolt.Color;
const Mat44 = zolt.Mat44;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const TriangleConvexSupport = zolt.TriangleConvexSupport;
const Vec3 = zolt.Vec3;

const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeSubType = ShapeFile.ShapeSubType;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const ConvexShape = @import("ConvexShape.zig").ConvexShape;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// A single triangle, its center of mass is at the origin
pub const TriangleShape = struct {
    pub const shape_sub_type: ShapeSubType = .triangle;

    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportFunction, .saveBinaryState, .restoreBinaryState, .getStats, .getVolume };

    base: ConvexShape,
    v1: Vec3 = Vec3.zero(),
    v2: Vec3 = Vec3.zero(),
    v3: Vec3 = Vec3.zero(),
    convex_radius: f32 = 0.0,

    /// TriangleShape()
    pub fn initDefault(allocator: Allocator) TriangleShape {
        return .{ .base = .init(TriangleShape, allocator, shape_sub_type, null) };
    }

    /// Create a triangle with points (v1, v2, v3) (counter clockwise) and convex radius
    pub fn init(allocator: Allocator, v1: Vec3, v2: Vec3, v3: Vec3, opts: struct { convex_radius: f32 = 0.0, material: ?*const PhysicsMaterial = null }) TriangleShape {
        std.debug.assert(opts.convex_radius >= 0.0);
        return .{ .base = .init(TriangleShape, allocator, shape_sub_type, opts.material), .v1 = v1, .v2 = v2, .v3 = v3, .convex_radius = opts.convex_radius };
    }

    pub fn create(allocator: Allocator, v1: Vec3, v2: Vec3, v3: Vec3, opts: struct { convex_radius: f32 = 0.0, material: ?*const PhysicsMaterial = null }) Allocator.Error!*TriangleShape {
        const self = try allocator.create(TriangleShape);
        self.* = .init(allocator, v1, v2, v3, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    pub fn asShape(self: *const TriangleShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *TriangleShape) *Shape {
        return &self.base.base;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const TriangleShape) AABox {
        var bounds = AABox.init(self.v1, self.v1);
        bounds.encapsulateVec3(self.v2);
        bounds.encapsulateVec3(self.v3);
        bounds.expandBy(Vec3.replicate(self.convex_radius));
        return bounds;
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const TriangleShape) f32 {
        return self.convex_radius;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const TriangleShape) MassProperties {
        // We cannot calculate the volume for a triangle, so we return invalid mass properties.
        _ = self;
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

    // See ConvexShape::GetSupportFunction
    pub fn getSupportFunction(self: *const TriangleShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        if (mode != .exclude_convex_radius and self.convex_radius > 0.0) {
            const support = buffer.emplace(TriangleWithConvex);
            support.* = .init(scale.mul(self.v1), scale.mul(self.v2), scale.mul(self.v3), self.convex_radius);
            return &support.base;
        }
        const support = buffer.emplace(TriangleNoConvex);
        support.* = .init(scale.mul(self.v1), scale.mul(self.v2), scale.mul(self.v3));
        return &support.base;
    }

    // See Shape::SaveBinaryState
    pub fn saveBinaryState(self: *const TriangleShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.v1);
        stream.write(self.v2);
        stream.write(self.v3);
        stream.write(self.convex_radius);
    }

    // See Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *TriangleShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.v1);
        stream.read(&self.v2);
        stream.read(&self.v3);
        stream.read(&self.convex_radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const TriangleShape) Shape.Stats {
        _ = self;
        return .{ .size_bytes = @sizeOf(TriangleShape), .num_triangles = 1 };
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const TriangleShape) f32 {
        _ = self;
        return 0;
    }

    // Stand-ins for sCollideConvexVsTriangle / sCollideSphereVsTriangle / sCastConvexVsTriangle / sCastSphereVsTriangle
    // (see the top of this file)
    pub fn collideConvexVsTriangle(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        ConvexShape.collideConvexVsConvex(shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    pub fn collideSphereVsTriangle(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        ConvexShape.collideConvexVsConvex(shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    pub fn castConvexVsTriangle(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        ConvexShape.castConvexVsConvex(shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    pub fn castSphereVsTriangle(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        ConvexShape.castConvexVsConvex(shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    // Register shape functions with the registry (literal port of TriangleShape::sRegister)
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
