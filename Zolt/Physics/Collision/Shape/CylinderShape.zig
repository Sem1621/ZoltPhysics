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
