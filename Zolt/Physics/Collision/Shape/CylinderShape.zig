//! Port of: Jolt/Physics/Collision/Shape/CylinderShape.h, Jolt/Physics/Collision/Shape/CylinderShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2, SphereShape.zig / BoxShape.zig are the
//! reference): `overrides` lists every C++ `override` in header order, the support class `Cylinder` is constructed in the
//! caller's SupportBuffer (D9), the static `sUnitCylinderTriangles` is a comptime table (`unit_cylinder_triangles`, D10).
//! CylinderShape only overrides the single hit CastRay (`using ConvexShape::CastRay`): the collector version is
//! ConvexShape's, which calls the analytic CastRay through the vtable. GetSubmergedVolume is ConvexShape's.
//! JPH_DEBUG_RENDERER (Draw) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RayCylinder = @import("../../../Geometry/RayCylinder.zig");
const math = @import("../../../Math/Math.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const PhysicsSettings = @import("../../PhysicsSettings.zig");
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
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
const RayCast = @import("../RayCast.zig").RayCast;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Class that constructs a CylinderShape
pub const CylinderShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, CylinderShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    half_height: f32 = 0.0,
    radius: f32 = 0.0,
    convex_radius: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) CylinderShapeSettings {
        return .{ .base = .init(CylinderShapeSettings, allocator, null) };
    }

    /// Create a shape centered around the origin with one top at (0, -half_height, 0) and the other at (0, half_height, 0) and radius radius.
    /// (internally the convex radius will be subtracted from the cylinder the total cylinder will not grow with the convex radius, but the edges of the cylinder will be rounded a bit).
    pub fn init(allocator: Allocator, half_height: f32, radius: f32, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) CylinderShapeSettings {
        return .{ .base = .init(CylinderShapeSettings, allocator, opts.material), .half_height = half_height, .radius = radius, .convex_radius = opts.convex_radius };
    }

    /// new CylinderShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, half_height: f32, radius: f32, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!*CylinderShapeSettings {
        const self = try allocator.create(CylinderShapeSettings);
        self.* = .init(allocator, half_height, radius, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *CylinderShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *CylinderShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *CylinderShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(CylinderShape, self, allocator);
    }
};

/// Approximation of top face with 8 vertices (static cCylinderTopFace)
const cylinder_top_face = [_]Vec3{
    Vec3.init(0.0, 1.0, 1.0),
    Vec3.init(0.707106769, 1.0, 0.707106769),
    Vec3.init(1.0, 1.0, 0.0),
    Vec3.init(0.707106769, 1.0, -0.707106769),
    Vec3.init(-0.0, 1.0, -1.0),
    Vec3.init(-0.707106769, 1.0, -0.707106769),
    Vec3.init(-1.0, 1.0, 0.0),
    Vec3.init(-0.707106769, 1.0, 0.707106769),
};

/// Triangles of a cylinder with half height 1 and radius 1 (static sUnitCylinderTriangles; Jolt builds it with a static
/// initializer, Zolt at compile time with the same code)
const unit_cylinder_triangles: StaticArray(Vec3, 96) = buildUnitCylinderTriangles();

/// The static initializer of sUnitCylinderTriangles (also run at runtime by a test to compare the bits)
fn buildUnitCylinderTriangles() StaticArray(Vec3, 96) {
    var verts: StaticArray(Vec3, 96) = .empty;

    const bottom_offset = Vec3.init(0.0, -2.0, 0.0);

    const num_verts = cylinder_top_face.len;
    for (0..num_verts) |i| {
        const t1 = cylinder_top_face[i];
        const t2 = cylinder_top_face[(i + 1) % num_verts];
        const b1 = cylinder_top_face[i].add(bottom_offset);
        const b2 = cylinder_top_face[(i + 1) % num_verts].add(bottom_offset);

        // Top
        verts.append(Vec3.init(0.0, 1.0, 0.0));
        verts.append(t1);
        verts.append(t2);

        // Bottom
        verts.append(Vec3.init(0.0, -1.0, 0.0));
        verts.append(b2);
        verts.append(b1);

        // Side
        verts.append(t1);
        verts.append(b1);
        verts.append(t2);

        verts.append(t2);
        verts.append(b1);
        verts.append(b2);
    }

    return verts;
}

/// A cylinder
pub const CylinderShape = struct {
    /// Concrete class: `Shape.cast(CylinderShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .cylinder;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .castRay, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: ConvexShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    half_height: f32 = 0.0,
    radius: f32 = 0.0,
    convex_radius: f32 = 0.0,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// CylinderShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) CylinderShape {
        return .{ .base = .init(CylinderShape, allocator, shape_sub_type, null) };
    }

    /// CylinderShape(const CylinderShapeSettings &inSettings, ShapeResult &outResult): C++ member initializers first, then the body
    pub fn initFromSettings(self: *CylinderShape, settings: *const CylinderShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.half_height = settings.half_height;
        self.radius = settings.radius;
        self.convex_radius = math.min(settings.convex_radius, math.min(settings.half_height, settings.radius));

        if (settings.half_height < 0.0) {
            result.setError("Invalid height");
            return;
        }

        if (settings.radius < 0.0) {
            result.setError("Invalid radius");
            return;
        }

        if (settings.convex_radius < 0.0) {
            result.setError("Invalid convex radius");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// Create a shape centered around the origin with one top at (0, -half_height, 0) and the other at (0, half_height, 0) and radius radius.
    /// (internally the convex radius will be subtracted from the cylinder the total cylinder will not grow with the convex radius, but the edges of the cylinder will be rounded a bit).
    /// On the stack / as a member: `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end.
    pub fn init(allocator: Allocator, half_height: f32, radius: f32, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) CylinderShape {
        std.debug.assert(half_height >= 0.0);
        std.debug.assert(radius >= 0.0);
        std.debug.assert(opts.convex_radius >= 0.0);
        return .{ .base = .init(CylinderShape, allocator, shape_sub_type, opts.material), .half_height = half_height, .radius = radius, .convex_radius = math.min(opts.convex_radius, math.min(half_height, radius)) };
    }

    /// new CylinderShape(...): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, half_height: f32, radius: f32, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!*CylinderShape {
        const self = try allocator.create(CylinderShape);
        self.* = .init(allocator, half_height, radius, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const CylinderShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *CylinderShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get half height of cylinder
    pub fn getHalfHeight(self: *const CylinderShape) f32 {
        return self.half_height;
    }

    /// Get radius of cylinder
    pub fn getRadius(self: *const CylinderShape) f32 {
        return self.radius;
    }

    /// Get the convex radius of this cylinder
    pub fn getConvexRadius(self: *const CylinderShape) f32 {
        return self.convex_radius;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const CylinderShape) AABox {
        const extent = Vec3.init(self.radius, self.half_height, self.radius);
        return .init(extent.negate(), extent);
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const CylinderShape) f32 {
        return math.min(self.half_height, self.radius);
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const CylinderShape) MassProperties {
        var p: MassProperties = .{};

        // Mass is surface of circle * height
        const radius_sq = math.square(self.radius);
        const height = 2.0 * self.half_height;
        p.mass = math.pi * radius_sq * height * self.base.getDensity();

        // Inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia:
        const inertia_y = radius_sq * p.mass * 0.5;
        const inertia_x = inertia_y * 0.5 + p.mass * height * height / 12.0;
        const inertia_z = inertia_x;

        // Set inertia
        p.inertia = Mat44.scaleVec3(Vec3.init(inertia_x, inertia_y, inertia_z));

        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const CylinderShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        // Calculate distance to infinite cylinder surface
        const local_surface_position_xz = Vec3.init(local_surface_position.getX(), 0, local_surface_position.getZ());
        const local_surface_position_xz_len = local_surface_position_xz.length();
        const distance_to_curved_surface = @abs(local_surface_position_xz_len - self.radius);

        // Calculate distance to top or bottom plane
        const distance_to_top_or_bottom = @abs(@abs(local_surface_position.getY()) - self.half_height);

        // Return normal according to closest surface
        if (distance_to_curved_surface < distance_to_top_or_bottom)
            return local_surface_position_xz.divScalar(local_surface_position_xz_len)
        else
            return if (local_surface_position.getY() > 0.0) Vec3.axisY() else Vec3.axisY().negate();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const CylinderShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        // Get scaled cylinder
        const abs_scale = scale.abs();
        const scale_xz = abs_scale.getX();
        const scale_y = abs_scale.getY();
        const scaled_half_height = scale_y * self.half_height;
        const scaled_radius = scale_xz * self.radius;

        const x = direction.getX();
        const y = direction.getY();
        const z = direction.getZ();
        const xz_sq = math.square(x) + math.square(z);
        const y_sq = math.square(y);

        // Check which component is bigger
        if (xz_sq > y_sq) {
            // Hitting side
            const f = -scaled_radius / @sqrt(xz_sq);
            const vx = x * f;
            const vz = z * f;
            out_vertices.append(center_of_mass_transform.mulVec3(Vec3.init(vx, scaled_half_height, vz)));
            out_vertices.append(center_of_mass_transform.mulVec3(Vec3.init(vx, -scaled_half_height, vz)));
        } else {
            // Hitting top or bottom

            // When the direction is more than 5 degrees from vertical, align the vertices so that 1 of the vertices
            // points towards direction in the XZ plane. This ensures that we always have a vertex towards max penetration depth.
            var transform = center_of_mass_transform;
            if (xz_sq > 0.00765427 * y_sq) {
                const base_x = Vec4.init(x, 0, z, 0).divScalar(@sqrt(xz_sq));
                const base_z = base_x.swizzle(.z, .y, .x, .w).mul(Vec4.init(-1, 0, 1, 0));
                transform = transform.mul(Mat44.init(base_x, Vec4.init(0, 1, 0, 0), base_z, Vec4.init(0, 0, 0, 1)));
            }

            // Adjust for scale and height
            const multiplier = if (y < 0.0) Vec3.init(scaled_radius, scaled_half_height, scaled_radius) else Vec3.init(-scaled_radius, -scaled_half_height, scaled_radius);
            transform = transform.preScaled(multiplier);

            for (cylinder_top_face) |v|
                out_vertices.append(transform.mulVec3(v));
        }
    }

    // See ConvexShape::GetSupportFunction
    pub fn getSupportFunction(self: *const CylinderShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        // Get scaled cylinder
        const abs_scale = scale.abs();
        const scale_xz = abs_scale.getX();
        const scale_y = abs_scale.getY();
        const scaled_half_height = scale_y * self.half_height;
        const scaled_radius = scale_xz * self.radius;
        const scaled_convex_radius = ScaleHelpers.scaleConvexRadius(self.convex_radius, scale);

        switch (mode) {
            .include_convex_radius, .default => {
                const support = buffer.emplace(Cylinder);
                support.* = .init(scaled_half_height, scaled_radius, 0.0);
                return &support.base;
            },

            .exclude_convex_radius => {
                const support = buffer.emplace(Cylinder);
                support.* = .init(scaled_half_height - scaled_convex_radius, scaled_radius - scaled_convex_radius, scaled_convex_radius);
                return &support.base;
            },
        }
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay (the collector version is ConvexShape's, `using ConvexShape::CastRay`)
    pub fn castRay(self: *const CylinderShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Test ray against capsule
        const fraction = RayCylinder.rayCylinder(ray.origin, ray.direction, self.half_height, self.radius);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const CylinderShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Check if the point is in the cylinder
        if (@abs(point.getY()) <= self.half_height // Within the height
        and math.square(point.getX()) + math.square(point.getZ()) <= math.square(self.radius)) // Within the radius
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const CylinderShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        const inverse_transform = center_of_mass_transform.inversedRotationTranslation();

        // Get scaled cylinder
        const abs_scale = scale.abs();
        const half_height = abs_scale.getY() * self.half_height;
        const radius = abs_scale.getX() * self.radius;

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                const local_pos = inverse_transform.mulVec3(v.getPosition());

                // Calculate penetration into side surface
                var side_normal = local_pos;
                side_normal.setY(0.0);
                const side_normal_length = side_normal.length();
                const side_penetration = radius - side_normal_length;

                // Calculate penetration into top or bottom plane
                const top_penetration = half_height - @abs(local_pos.getY());

                var point: Vec3 = undefined;
                var normal: Vec3 = undefined;
                if (side_penetration < 0.0 and top_penetration < 0.0) {
                    // We're outside the cylinder height and radius
                    point = side_normal.mulScalar(radius / side_normal_length).add(Vec3.init(0, half_height * math.sign(local_pos.getY()), 0));
                    normal = local_pos.sub(point).normalizedOr(Vec3.axisY());
                } else if (side_penetration < top_penetration) {
                    // Side surface is closest
                    normal = if (side_normal_length > 0.0) side_normal.divScalar(side_normal_length) else Vec3.axisX();
                    point = normal.mulScalar(radius);
                } else {
                    // Top or bottom plane is closest
                    normal = Vec3.init(0, math.sign(local_pos.getY()), 0);
                    point = normal.mulScalar(half_height);
                }

                // Calculate penetration
                const plane = Plane.fromPointAndNormal(point, normal);
                const penetration = -plane.signedDistance(local_pos);
                if (v.updatePenetration(penetration))
                    v.setCollision(plane.getTransformed(center_of_mass_transform), colliding_shape_index);
            }
        }
    }

    // See Shape::GetTrianglesStart: placement new of a context (no pointers into itself: construct by value)
    pub fn getTrianglesStart(self: *const CylinderShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        const unit_cylinder_transform = Mat44.init(Vec4.init(self.radius, 0, 0, 0), Vec4.init(0, self.half_height, 0, 0), Vec4.init(0, 0, self.radius, 0), Vec4.init(0, 0, 0, 1));
        context.emplace(GetTrianglesContextVertexList).* = .init(position_com, rotation, scale, unit_cylinder_transform, unit_cylinder_triangles.constSlice(), self.base.getConvexMaterial());
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const CylinderShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    // See Shape::SaveBinaryState: C++ `ConvexShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const CylinderShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.half_height);
        stream.write(self.radius);
        stream.write(self.convex_radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const CylinderShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(CylinderShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const CylinderShape) f32 {
        return 2.0 * math.pi * self.half_height * math.square(self.radius);
    }

    // See Shape::IsValidScale: C++ `ConvexShape::IsValidScale` resolves to Shape's version (ConvexShape does not override it)
    pub fn isValidScale(self: *const CylinderShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScaleXZ(scale.abs());
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const CylinderShape, scale: Vec3) Vec3 {
        _ = self;
        const s = ScaleHelpers.makeNonZeroScale(scale);

        return s.getSign().mul(ScaleHelpers.makeUniformScaleXZ(s.abs()));
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *CylinderShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.half_height);
        stream.read(&self.radius);
        stream.read(&self.convex_radius);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.cylinder);
        f.construct = ShapeFunctions.constructor(CylinderShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Class for GetSupportFunction (`class CylinderShape::Cylinder final : public Support`)

    const Cylinder = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        half_height: f32,
        radius: f32,
        convex_radius: f32,

        fn init(half_height: f32, radius: f32, convex_radius: f32) Cylinder {
            return .{ .base = .init(Cylinder), .half_height = half_height, .radius = radius, .convex_radius = convex_radius };
        }

        pub fn getSupport(self: *const Cylinder, direction: Vec3) Vec3 {
            // Support mapping, taken from:
            // A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen
            // page 8
            const x = direction.getX();
            const y = direction.getY();
            const z = direction.getZ();
            const o = @sqrt(math.square(x) + math.square(z));
            if (o > 0.0)
                return Vec3.init((self.radius * x) / o, math.sign(y) * self.half_height, (self.radius * z) / o)
            else
                return Vec3.init(0, math.sign(y) * self.half_height, 0);
        }

        pub fn getConvexRadius(self: *const Cylinder) f32 {
            return self.convex_radius;
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's own cylinder tests are in ZoltTests/Physics, the bit exact comparison with Jolt in
// ZoltParity/Physics/CylindersParity.zig)

const testing = std.testing;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const CastRayCollector = ShapeFile.CastRayCollector;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RegisterTypes = @import("../../../RegisterTypes.zig");

test "CylinderShape: the unit cylinder table is Jolt's static initializer, built at compile time" {
    const runtime = buildUnitCylinderTriangles();
    try testing.expectEqual(@as(u32, 96), runtime.len);
    try testing.expectEqual(runtime.len, unit_cylinder_triangles.len);
    for (runtime.constSlice(), unit_cylinder_triangles.constSlice()) |r, t|
        try testing.expectEqual(@as(u128, @bitCast(r.value)), @as(u128, @bitCast(t.value)));

    // The first triangle is the top cap, the bottom vertices have +0 where the top face has -0 (-0 + 0 = +0)
    try testing.expect(unit_cylinder_triangles.get(0).eql(Vec3.init(0, 1, 0)));
    try testing.expect(unit_cylinder_triangles.get(1).eql(cylinder_top_face[0]));
    try testing.expect(std.math.signbit(cylinder_top_face[4].getX()));
    try testing.expect(std.math.signbit(unit_cylinder_triangles.get(4 * 12 + 1).getX())); // t1 of i = 4
    try testing.expect(!std.math.signbit(unit_cylinder_triangles.get(4 * 12 + 5).getX())); // b1 of i = 4
}

test "CylinderShape: settings, Jolt's error texts, convex radius and out of memory (the settings part of TestCylinderShape)" {
    const allocator = testing.allocator;

    const Case = struct { half_height: f32, radius: f32, convex_radius: f32, error_text: []const u8 };
    for ([_]Case{
        .{ .half_height = -1.0, .radius = 1.0, .convex_radius = 1.0, .error_text = "Invalid height" }, // Check half height must be positive
        .{ .half_height = 1.0, .radius = -1.0, .convex_radius = 1.0, .error_text = "Invalid radius" }, // Check radius must be positive
        .{ .half_height = 1.0, .radius = 1.0, .convex_radius = -1.0, .error_text = "Invalid convex radius" }, // Check convex radius must be positive
        .{ .half_height = -1.0, .radius = -1.0, .convex_radius = -1.0, .error_text = "Invalid height" }, // The first check wins
    }) |c| {
        var settings = CylinderShapeSettings.init(allocator, c.half_height, c.radius, .{ .convex_radius = c.convex_radius });
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings(c.error_text, result.getError());
    }

    {
        // Create zero sized cylinder
        var settings = CylinderShapeSettings.init(allocator, 0.0, 0.0, .{ .convex_radius = 1.0 });
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const cylinder = result.getPtr().?.cast(CylinderShape);

        // Create another cylinder by using a different constructor
        var cylinder2_ref = RefConst(Shape).init((try CylinderShape.create(allocator, 0.0, 0.0, .{ .convex_radius = 1.0 })).asShape());
        defer cylinder2_ref.deinit();
        const cylinder2 = cylinder2_ref.get().?.cast(CylinderShape);

        // Check convex radius is adjusted to zero
        try testing.expectEqual(@as(f32, 0.0), cylinder.getConvexRadius());
        try testing.expectEqual(@as(f32, 0.0), cylinder2.getConvexRadius());
    }

    // Defaults: the default convex radius, the default constructor has a zero cylinder
    var defaults = CylinderShapeSettings.init(allocator, 1.0, 1.0, .{});
    defer defaults.deinit();
    try testing.expectEqual(PhysicsSettings.default_convex_radius, defaults.convex_radius);
    var empty = CylinderShapeSettings.initDefault(allocator);
    defer empty.deinit();
    try testing.expect(empty.half_height == 0.0 and empty.radius == 0.0 and empty.convex_radius == 0.0);

    // Heap settings with a material, density and user data
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    const settings = try CylinderShapeSettings.create(allocator, 2.0, 0.5, .{ .convex_radius = 0.1, .material = material.material() });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    settings.base.setDensity(250.0);
    settings.asShapeSettings().user_data = 42;
    var result = try settings.createShape(allocator);
    defer result.deinit();
    const cylinder = result.getPtr().?.cast(CylinderShape);
    try testing.expectEqual(@as(f32, 2.0), cylinder.getHalfHeight());
    try testing.expectEqual(@as(f32, 0.5), cylinder.getRadius());
    try testing.expectEqual(@as(f32, 0.1), cylinder.getConvexRadius());
    try testing.expectEqual(@as(f32, 250.0), cylinder.base.getDensity());
    try testing.expectEqual(@as(u64, 42), cylinder.asShape().getUserData());
    try testing.expect(cylinder.asShape().getMaterial(.empty) == material.material());

    // The convex radius is limited by the half height and the radius
    var thin = CylinderShape.init(allocator, 0.02, 1.0, .{ .convex_radius = 0.05 });
    thin.asShape().setEmbedded();
    defer thin.asShapeMut().deinit();
    try testing.expectEqual(@as(f32, 0.02), thin.getConvexRadius());

    // Out of memory while creating the shape is returned and not cached
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var oom_settings = CylinderShapeSettings.init(allocator, 1.0, 1.0, .{});
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

test "CylinderShape: bounds, inner radius, mass properties, volume, stats, surface normal" {
    const allocator = testing.allocator;

    var cylinder = CylinderShape.init(allocator, 2.0, 0.5, .{});
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();
    const shape = cylinder.asShape();

    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.init(-0.5, -2, -0.5), Vec3.init(0.5, 2, 0.5))));
    try testing.expect(shape.getCenterOfMass().eql(Vec3.zero()));
    try testing.expectEqual(@as(f32, 0.5), shape.getInnerRadius());
    try testing.expectApproxEqRel(@as(f32, std.math.pi), shape.getVolume(), 1.0e-6); // 2 * pi * 2 * 0.25
    try testing.expectEqual(@as(usize, @sizeOf(CylinderShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);

    // Mass and inertia of a solid cylinder (https://en.wikipedia.org/wiki/List_of_moments_of_inertia)
    const p = shape.getMassProperties();
    const mass = std.math.pi * 0.25 * 4.0 * 1000.0;
    try testing.expectApproxEqRel(@as(f32, mass), p.mass, 1.0e-6);
    try testing.expectApproxEqRel(@as(f32, 0.5 * mass * 0.25), p.inertia.get(1, 1), 1.0e-6);
    try testing.expectApproxEqRel(@as(f32, mass * (3.0 * 0.25 + 16.0) / 12.0), p.inertia.get(0, 0), 1.0e-6);
    try testing.expectEqual(p.inertia.get(0, 0), p.inertia.get(2, 2));
    try testing.expectEqual(@as(f32, 0.0), p.inertia.get(0, 1));

    // Surface normals: the closest surface
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0.5, 0.3, 0)).eql(Vec3.axisX()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, -1, -0.49)).eql(Vec3.init(0, 0, -1)));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0.1, 1.99, 0.1)).eql(Vec3.axisY()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0.1, -1.99, 0.1)).eql(Vec3.axisY().negate()));
}

test "CylinderShape: supporting faces (side, top, bottom, aligned with the direction)" {
    const allocator = testing.allocator;

    var cylinder = CylinderShape.init(allocator, 2.0, 0.5, .{});
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();
    const shape = cylinder.asShape();
    const scale = Vec3.init(-2, 0.5, 2);
    const transform = Mat44.translation(Vec3.init(10, 0, 0));

    // Side: 2 vertices on the side opposite to the direction (scaled radius 1, scaled half height 1)
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.init(1, 0.1, 0), scale, transform, &face);
    try testing.expectEqual(@as(u32, 2), face.len);
    try testing.expect(face.get(0).eql(Vec3.init(9, 1, 0)));
    try testing.expect(face.get(1).eql(Vec3.init(9, -1, 0)));

    // Bottom (direction up): 8 vertices on the bottom cap
    face.clear();
    shape.getSupportingFace(.empty, Vec3.init(0, 1, 0), scale, Mat44.identity(), &face);
    try testing.expectEqual(@as(u32, 8), face.len);
    for (face.constSlice()) |v| {
        try testing.expectEqual(@as(f32, -1.0), v.getY());
        try testing.expectApproxEqAbs(@as(f32, 1.0), Vec3.init(v.getX(), 0, v.getZ()).length(), 1.0e-6);
    }

    // Top (direction down)
    face.clear();
    shape.getSupportingFace(.empty, Vec3.init(0, -1, 0), scale, Mat44.identity(), &face);
    try testing.expectEqual(@as(u32, 8), face.len);
    for (face.constSlice()) |v| try testing.expectEqual(@as(f32, 1.0), v.getY());

    // More than 5 degrees from vertical: one vertex points towards the direction in the XZ plane
    face.clear();
    shape.getSupportingFace(.empty, Vec3.init(0.3, -1, 0.4), scale, Mat44.identity(), &face);
    try testing.expectEqual(@as(u32, 8), face.len);
    var found = false;
    for (face.constSlice()) |v| found = found or v.isClose(Vec3.init(0.6, 1, 0.8), .{ .max_dist_sq = 1.0e-10 });
    try testing.expect(found);

    // Less than 5 degrees from vertical: the face is not rotated
    face.clear();
    shape.getSupportingFace(.empty, Vec3.init(0.03, -1, 0.04), scale, Mat44.identity(), &face);
    try testing.expect(face.get(0).isClose(Vec3.init(0, 1, 1), .{ .max_dist_sq = 1.0e-10 }));
}

test "CylinderShape: valid scales (the cylinder part of Jolt's TestIsValidScale)" {
    const allocator = testing.allocator;

    // Constant of TestIsValidScale: Square(1.0e-6f * ScaleHelpers::cMinScale)
    const min_scale_tolerance_sq: f32 = math.square(1.0e-6 * ScaleHelpers.min_scale);

    var cylinder_ref = Ref(Shape).init((try CylinderShape.create(allocator, 0.5, 2.0, .{})).asShapeMut());
    defer cylinder_ref.deinit();
    const cylinder = cylinder_ref.get().?;
    try testing.expect(!cylinder.isValidScale(Vec3.zero()));
    try testing.expect(!cylinder.isValidScale(Vec3.init(0, 1, 0)));
    try testing.expect(!cylinder.isValidScale(Vec3.init(1, 0, 1)));
    try testing.expect(cylinder.isValidScale(Vec3.init(2, 2, 2)));
    try testing.expect(cylinder.isValidScale(Vec3.init(-1, 1, -1)));
    try testing.expect(!cylinder.isValidScale(Vec3.init(2, 1, 1)));
    try testing.expect(cylinder.isValidScale(Vec3.init(1, 2, 1)));
    try testing.expect(!cylinder.isValidScale(Vec3.init(1, 1, 2)));
    try testing.expect(cylinder.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try testing.expect(cylinder.makeScaleValid(Vec3.init(-1.0e-10, 1, 1.0e-10)).eql(Vec3.init(-ScaleHelpers.min_scale, 1, ScaleHelpers.min_scale)));
    try testing.expect(cylinder.makeScaleValid(Vec3.init(2, 5, -4)).eql(Vec3.init(3, 5, -3)));
}

test "CylinderShape: support functions" {
    const allocator = testing.allocator;

    var cylinder = CylinderShape.init(allocator, 2.0, 1.0, .{ .convex_radius = 0.5 });
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();

    var buffer: ConvexShape.SupportBuffer = .{};
    const scale = Vec3.init(-2, 0.5, 2);

    // Include convex radius and default: the full scaled cylinder, no convex radius
    for ([_]ConvexShape.SupportMode{ .include_convex_radius, .default }) |mode| {
        const support = cylinder.base.getSupportFunction(mode, &buffer, scale);
        try testing.expectEqual(@as(f32, 0.0), support.getConvexRadius());
        try testing.expect(support.getSupport(Vec3.init(3, -1, 4)).isClose(Vec3.init(1.2, -1, 1.6), .{ .max_dist_sq = 1.0e-12 }));
        try testing.expect(support.getSupport(Vec3.init(0, -1, 0)).eql(Vec3.init(0, -1, 0)));
        try testing.expect(support.getSupport(Vec3.zero()).eql(Vec3.init(0, 1, 0))); // Sign(0) = 1
    }

    // Exclude convex radius: shrunk by the scaled convex radius (limited to cDefaultConvexRadius)
    const r = PhysicsSettings.default_convex_radius;
    const support = cylinder.base.getSupportFunction(.exclude_convex_radius, &buffer, scale);
    try testing.expectEqual(r, support.getConvexRadius());
    try testing.expect(support.getSupport(Vec3.init(1, 1, 0)).eql(Vec3.init(2 - r, 1 - r, 0)));
}

test "CylinderShape: ray casts (TestCylinderShapeRay), collide point (TestCollidePointVsCylinder) and filters" {
    const allocator = testing.allocator;

    var cylinder = CylinderShape.init(allocator, 4, 2, .{});
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();
    const shape = cylinder.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 2, 3);

    // Rays through the cylinder from outside (the analytic CastRay)
    for ([_][2]Vec3{
        .{ Vec3.init(-2, 0, 0), Vec3.init(2, 0, 0) },
        .{ Vec3.init(0, -4, 0), Vec3.init(0, 4, 0) },
        .{ Vec3.init(0, 0, -2), Vec3.init(0, 0, 2) },
    }) |ends| {
        const origin = ends[0].mulScalar(2);
        const direction = ends[1].sub(origin);
        var hit: RayCastResult = .{};
        try testing.expect(shape.castRay(.init(origin, direction), creator, &hit));
        try testing.expect(origin.add(direction.mulScalar(hit.fraction)).isClose(ends[0], .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(hit.sub_shape_id2.eql(creator.getID()));
    }
    var miss: RayCastResult = .{};
    try testing.expect(!shape.castRay(.init(Vec3.init(-4, 5, 0), Vec3.init(8, 0, 0)), creator, &miss));
    try testing.expect(shape.castRay(.init(Vec3.zero(), Vec3.init(8, 0, 0)), creator, &miss)); // Starts inside
    try testing.expectEqual(@as(f32, 0.0), miss.fraction);

    // The collector version is ConvexShape's and uses the analytic CastRay: front and back face
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(5), .{});
    hits.base.setContext(&context);
    shape.castRayCollector(.init(Vec3.init(-4, 0, 0), Vec3.init(8, 0, 0)), &settings, creator, &hits.base, &.{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.25), hits.hits.items[0].fraction);
    try testing.expectApproxEqAbs(@as(f32, 0.75), hits.hits.items[1].fraction, 1.0e-6); // Inverted ray of ConvexShape's fallback
    try testing.expect(hits.hits.items[0].body_id.eql(.init(5)) and hits.hits.items[1].body_id.eql(.init(5)));

    // TestCollidePointVsCylinder
    const half_height: f32 = 0.2;
    const radius: f32 = 0.1;
    var small = CylinderShape.init(allocator, half_height, radius, .{});
    small.asShape().setEmbedded();
    defer small.asShapeMut().deinit();
    const xy_and_zero_probes = [_]Vec3{ Vec3.zero(), Vec3.init(1, 0, 0), Vec3.init(-1, 0, 0), Vec3.init(0, 0, 1), Vec3.init(0, 0, -1) };
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    for (xy_and_zero_probes) |probe| {
        // Top and bottom hits
        for ([_]f32{ half_height, -half_height }) |y| {
            points.reset();
            small.asShape().collidePoint(probe.mulScalar(radius).add(Vec3.init(0, y, 0)).mulScalar(0.99), creator, &points.base, &.{});
            try testing.expectEqual(@as(usize, 1), points.hits.items.len);
        }
    }
    // Misses just outside the faces of the bounding box
    const cube_probes = [_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, -1, 0), Vec3.init(0, 1, 0), Vec3.init(0, 0, -1), Vec3.init(0, 0, 1) };
    for (cube_probes) |probe| {
        points.reset();
        small.asShape().collidePoint(Vec3.init(radius, half_height, radius).mulScalar(1.01).mul(probe), creator, &points.base, &.{});
        try testing.expectEqual(@as(usize, 0), points.hits.items.len);
    }
    try points.checkError();

    // The shape filter is tested first
    const RejectAll = struct {
        pub const overrides = .{.shouldCollide};
        base: ShapeFilter = .init(@This()),
        pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, shape2, sub_shape_id_of_shape2 };
            return false;
        }
    };
    const reject: RejectAll = .{};
    points.reset();
    small.asShape().collidePoint(Vec3.zero(), creator, &points.base, &reject.base);
    try testing.expectEqual(@as(usize, 0), points.hits.items.len);
}

test "CylinderShape: CollideSoftBodyVertices" {
    const allocator = testing.allocator;

    var cylinder = CylinderShape.init(allocator, 2.0, 1.0, .{});
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();

    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.init(10, 0, 0));
    var positions = [_]Vec3{
        transform.mulVec3(Vec3.init(0.8, 0.5, 0)), // Inside, closest to the side
        transform.mulVec3(Vec3.init(0, 1.9, 0)), // Inside, closest to the top
        transform.mulVec3(Vec3.init(0, -3, 0)), // Outside below the bottom
        transform.mulVec3(Vec3.init(4, 5, 0)), // Outside the height and the radius: the closest point is the rim
        transform.mulVec3(Vec3.init(0, 0, 0)), // Infinite mass
    };
    var inv_masses = [_]f32{ 1, 1, 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 5;
    var penetrations = [_]f32{-math.flt_max} ** 5;
    var indices = [_]i32{-1} ** 5;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    cylinder.asShape().collideSoftBodyVertices(transform, Vec3.one(), &vertices, 5, 6);

    try testing.expectApproxEqAbs(@as(f32, 0.2), penetrations[0], 1.0e-5);
    try testing.expectEqual(@as(i32, 6), indices[0]);
    try testing.expect(planes[0].getNormal().isClose(transform.multiply3x3(Vec3.axisX()), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectApproxEqAbs(@as(f32, 0.1), penetrations[1], 1.0e-5);
    try testing.expect(planes[1].getNormal().isClose(transform.multiply3x3(Vec3.axisY()), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectApproxEqAbs(@as(f32, -1.0), penetrations[2], 1.0e-5);
    try testing.expect(planes[2].getNormal().isClose(transform.multiply3x3(Vec3.axisY().negate()), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectApproxEqAbs(-@sqrt(@as(f32, 18.0)), penetrations[3], 1.0e-5); // The closest point is the rim at (1, 2, 0)
    try testing.expect(planes[3].getNormal().isClose(transform.multiply3x3(Vec3.init(1, 1, 0).normalized()), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(-math.flt_max, penetrations[4]);
    try testing.expectEqual(@as(i32, -1), indices[4]);
}

test "CylinderShape: GetTrianglesStart / Next, GetSubmergedVolume" {
    const allocator = testing.allocator;

    // A non default material: every triangle reports the material of the shape
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var cylinder = CylinderShape.init(allocator, 2.0, 0.5, .{ .material = material.material() });
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();

    // 32 triangles of the scaled cylinder, inside out scales flip the winding
    for ([_]Vec3{ Vec3.one(), Vec3.init(1, -1, 1) }) |scale| {
        var context: Shape.GetTrianglesContext = .{};
        cylinder.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), scale);
        var vertices: [3 * 32]Float3 = undefined;
        var materials: [32]*const PhysicsMaterial = undefined;
        try testing.expectEqual(@as(u32, 32), cylinder.asShape().getTrianglesNext(&context, 32, &vertices, &materials));
        const flipped_order = [_]usize{ 0, 2, 1 };
        for (vertices, 0..) |v, i| {
            const index = if (scale.getY() < 0.0) i / 3 * 3 + flipped_order[i % 3] else i;
            const expected = unit_cylinder_triangles.get(@intCast(index)).mul(Vec3.init(0.5, 2, 0.5)).mul(scale);
            try testing.expect(Vec3.fromFloat3(v).eql(expected));
        }
        for (materials) |m|
            try testing.expect(m == material.material());
        try testing.expectEqual(@as(u32, 0), cylinder.asShape().getTrianglesNext(&context, 32, &vertices, null));
    }

    // GetSubmergedVolume is ConvexShape's (bounding box based)
    const half = cylinder.asShape().getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.fromPointAndNormal(Vec3.zero(), Vec3.axisY()));
    try testing.expectEqual(@as(f32, 4.0), half.total_volume);
    try testing.expectApproxEqAbs(@as(f32, 2.0), half.submerged_volume, 1.0e-5);
}

test "CylinderShape: binary state, restoreFromBinaryState and the registration" {
    const allocator = testing.allocator;

    var cylinder = CylinderShape.init(allocator, 2.0, 0.5, .{ .convex_radius = 0.25 });
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();
    cylinder.base.setDensity(321.0);
    cylinder.asShapeMut().setUserData(7);

    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    cylinder.asShape().saveBinaryState(out.streamOut());
    try testing.expectEqual(@as(usize, 1 + 8 + 4 + 4 + 4 + 4), writer.buffered().len); // Sub type, user data, density, half height, radius, convex radius

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    const restored = result.getPtr().?.cast(CylinderShape);
    try testing.expectEqual(@as(f32, 2.0), restored.getHalfHeight());
    try testing.expectEqual(@as(f32, 0.5), restored.getRadius());
    try testing.expectEqual(@as(f32, 0.25), restored.getConvexRadius());
    try testing.expectEqual(@as(f32, 321.0), restored.base.getDensity());
    try testing.expectEqual(@as(u64, 7), restored.asShape().getUserData());

    // Truncated: Jolt's error text
    var short_reader: std.Io.Reader = .fixed(writer.buffered()[0 .. writer.buffered().len - 1]);
    var short_in = StreamWrapper.StreamInWrapper.init(&short_reader);
    var short_result = try Shape.restoreFromBinaryState(allocator, short_in.streamIn());
    defer short_result.deinit();
    try testing.expectEqualStrings("Failed to restore shape", short_result.getError());

    // ShapeFunctions
    const functions = ShapeFunctions.get(.cylinder);
    try testing.expect(functions.construct != null);
    try testing.expect(functions.color.eql(Color.green));
    try testing.expect(RegisterTypes.registry.shape_functions[@intFromEnum(ShapeSubType.cylinder)].construct == functions.construct);
}

test "CylinderShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, CylinderShape.create(failing.allocator(), 1.0, 1.0, .{}));
    try testing.expectError(error.OutOfMemory, CylinderShapeSettings.create(failing.allocator(), 1.0, 1.0, .{}));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.cylinder).construct.?(failing.allocator()));

    // Restore: the shape is the only allocation
    var cylinder = CylinderShape.init(allocator, 1.0, 1.0, .{});
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();
    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    cylinder.asShape().saveBinaryState(out.streamOut());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    try testing.expectError(error.OutOfMemory, Shape.restoreFromBinaryState(failing.allocator(), in.streamIn()));
}
