//! Port of: Jolt/Physics/Collision/Shape/CapsuleShape.h, Jolt/Physics/Collision/Shape/CapsuleShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2, SphereShape.zig is the reference):
//! - `overrides` lists every C++ `override` in header order. CapsuleShape only overrides the single hit `CastRay`
//!   (`using ConvexShape::CastRay`): the collector version is ConvexShape's, which calls the analytic `castRay` through
//!   the vtable. GetSubmergedVolume is ConvexShape's.
//! - `CapsuleShapeSettings.createShape` has Jolt's custom Create logic: a capsule without height becomes a SphereShape
//!   (built with `SphereShape(radius, material)`, so it keeps the default density and user data like in Jolt).
//! - The support classes `CapsuleNoConvex` / `CapsuleWithConvex` are constructed in the caller's SupportBuffer (D9).
//! - The static vertex lists `sCapsuleTopTriangles`, `sCapsuleMiddleTriangles` and `sCapsuleBottomTriangles` are comptime
//!   tables with the bits of Jolt's static initializers (D10, a test compares them with a runtime build).
//! - JPH_DEBUG_RENDERER (Draw) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const rayCapsule = @import("../../../Geometry/RayCapsule.zig").rayCapsule;
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
const GetTrianglesContextFile = @import("GetTrianglesContext.zig");
const GetTrianglesContextVertexList = GetTrianglesContextFile.GetTrianglesContextVertexList;
const GetTrianglesContextMultiVertexList = GetTrianglesContextFile.GetTrianglesContextMultiVertexList;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SphereShape = @import("SphereShape.zig").SphereShape;
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

/// Class that constructs a CapsuleShape
pub const CapsuleShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, CapsuleShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    radius: f32 = 0.0,
    half_height_of_cylinder: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) CapsuleShapeSettings {
        return .{ .base = .init(CapsuleShapeSettings, allocator, null) };
    }

    /// Create a capsule centered around the origin with one sphere cap at (0, -half_height_of_cylinder, 0) and the other at (0, half_height_of_cylinder, 0)
    /// (settings on the stack: `defer settings.deinit()`)
    pub fn init(allocator: Allocator, half_height_of_cylinder: f32, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) CapsuleShapeSettings {
        return .{ .base = .init(CapsuleShapeSettings, allocator, opts.material), .radius = radius, .half_height_of_cylinder = half_height_of_cylinder };
    }

    /// new CapsuleShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, half_height_of_cylinder: f32, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) Allocator.Error!*CapsuleShapeSettings {
        const self = try allocator.create(CapsuleShapeSettings);
        self.* = .init(allocator, half_height_of_cylinder, radius, .{ .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *CapsuleShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *CapsuleShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    /// Check if this is a valid capsule shape
    pub fn isValid(self: *const CapsuleShapeSettings) bool {
        return self.radius > 0.0 and self.half_height_of_cylinder >= 0.0;
    }

    /// Checks if the settings of this capsule make this shape a sphere
    pub fn isSphere(self: *const CapsuleShapeSettings) bool {
        return self.half_height_of_cylinder == 0.0;
    }

    /// Create a shape according to the settings specified by this object.
    /// Note when half_height_of_cylinder is 0, this will create a SphereShape instead of a CapsuleShape.
    pub fn createShape(self: *CapsuleShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        const cached_result = &self.base.base.cached_result;
        if (cached_result.isEmpty()) {
            if (self.isValid() and self.isSphere()) {
                // If the capsule has no height, use a sphere instead
                const shape = try SphereShape.create(allocator, self.radius, .{ .material = self.base.material.get() });
                cached_result.set(.init(shape.asShapeMut()));
            } else try ShapeSettings.constructShape(CapsuleShape, self, allocator);
        }
        return cached_result.clone();
    }
};

/// cCapsuleDetailLevel
const capsule_detail_level = 2;

/// sCapsuleTopTriangles (built at compile time with the code of Jolt's static initializer)
const capsule_top_triangles: StaticArray(Vec3, 192) = blk: {
    @setEvalBranchQuota(100_000);
    var verts: StaticArray(Vec3, 192) = .empty;
    GetTrianglesContextVertexList.createHalfUnitSphereTop(&verts, capsule_detail_level) catch unreachable;
    break :blk verts;
};

/// sCapsuleMiddleTriangles
const capsule_middle_triangles: StaticArray(Vec3, 96) = blk: {
    @setEvalBranchQuota(1_000_000);
    var verts: StaticArray(Vec3, 96) = .empty;
    GetTrianglesContextVertexList.createUnitOpenCylinder(&verts, capsule_detail_level) catch unreachable;
    break :blk verts;
};

/// sCapsuleBottomTriangles
const capsule_bottom_triangles: StaticArray(Vec3, 192) = blk: {
    @setEvalBranchQuota(100_000);
    var verts: StaticArray(Vec3, 192) = .empty;
    GetTrianglesContextVertexList.createHalfUnitSphereBottom(&verts, capsule_detail_level) catch unreachable;
    break :blk verts;
};

/// A capsule, implemented as a line segment with convex radius
pub const CapsuleShape = struct {
    /// Concrete class: `Shape.cast(CapsuleShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .capsule;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .castRay, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: ConvexShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    radius: f32 = 0.0,
    half_height_of_cylinder: f32 = 0.0,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// CapsuleShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by constructShape
    pub fn initDefault(allocator: Allocator) CapsuleShape {
        return .{ .base = .init(CapsuleShape, allocator, shape_sub_type, null) };
    }

    /// CapsuleShape(const CapsuleShapeSettings &inSettings, ShapeResult &outResult): C++ member initializers first, then the body
    pub fn initFromSettings(self: *CapsuleShape, settings: *const CapsuleShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.radius = settings.radius;
        self.half_height_of_cylinder = settings.half_height_of_cylinder;

        if (settings.half_height_of_cylinder <= 0.0) {
            result.setError("Invalid height");
            return;
        }

        if (settings.radius <= 0.0) {
            result.setError("Invalid radius");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// Create a capsule centered around the origin with one sphere cap at (0, -half_height_of_cylinder, 0) and the other at (0, half_height_of_cylinder, 0)
    /// (on the stack / as a member: `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end)
    pub fn init(allocator: Allocator, half_height_of_cylinder: f32, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) CapsuleShape {
        std.debug.assert(half_height_of_cylinder > 0.0);
        std.debug.assert(radius > 0.0);
        return .{ .base = .init(CapsuleShape, allocator, shape_sub_type, opts.material), .radius = radius, .half_height_of_cylinder = half_height_of_cylinder };
    }

    /// new CapsuleShape(inHalfHeightOfCylinder, inRadius, inMaterial): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, half_height_of_cylinder: f32, radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) Allocator.Error!*CapsuleShape {
        const self = try allocator.create(CapsuleShape);
        self.* = .init(allocator, half_height_of_cylinder, radius, .{ .material = opts.material });
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const CapsuleShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *CapsuleShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Radius of the cylinder
    pub fn getRadius(self: *const CapsuleShape) f32 {
        return self.radius;
    }

    /// Get half of the height of the cylinder
    pub fn getHalfHeightOfCylinder(self: *const CapsuleShape) f32 {
        return self.half_height_of_cylinder;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const CapsuleShape) AABox {
        const extent = Vec3.replicate(self.radius).add(Vec3.init(0, self.half_height_of_cylinder, 0));
        return .init(extent.negate(), extent);
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const CapsuleShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        const abs_scale = scale.abs();
        const uniform_scale = abs_scale.getX();
        const extent = Vec3.replicate(uniform_scale * self.radius);
        const height = Vec3.init(0, uniform_scale * self.half_height_of_cylinder, 0);
        const p1 = center_of_mass_transform.mulVec3(height.negate());
        const p2 = center_of_mass_transform.mulVec3(height);
        return .init(Vec3.min(p1, p2).sub(extent), Vec3.max(p1, p2).add(extent));
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const CapsuleShape) f32 {
        return self.radius;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const CapsuleShape) MassProperties {
        var p: MassProperties = .{};

        const density = self.base.getDensity();

        // Calculate inertia and mass according to:
        // https://www.gamedev.net/resources/_/technical/math-and-physics/capsule-inertia-tensor-r3856
        // Note that there is an error in eq 14, H^2/2 should be H^2/4 in Ixx and Izz, eq 12 does contain the correct value
        const radius_sq = math.square(self.radius);
        const height = 2.0 * self.half_height_of_cylinder;
        const cylinder_mass = math.pi * height * radius_sq * density;
        const hemisphere_mass = (2.0 * math.pi / 3.0) * radius_sq * self.radius * density;

        // From cylinder
        const height_sq = math.square(height);
        var inertia_y = radius_sq * cylinder_mass * 0.5;
        var inertia_xz = inertia_y * 0.5 + cylinder_mass * height_sq / 12.0;

        // From hemispheres
        const temp = hemisphere_mass * 4.0 * radius_sq / 5.0;
        inertia_y += temp;
        inertia_xz += temp + hemisphere_mass * (0.5 * height_sq + (3.0 / 4.0) * height * self.radius);

        // Mass is cylinder + hemispheres
        p.mass = cylinder_mass + hemisphere_mass * 2.0;

        // Set inertia
        p.inertia = Mat44.scaleVec3(Vec3.init(inertia_xz, inertia_y, inertia_xz));

        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const CapsuleShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        if (local_surface_position.getY() > self.half_height_of_cylinder)
            return local_surface_position.sub(Vec3.init(0, self.half_height_of_cylinder, 0)).normalized()
        else if (local_surface_position.getY() < -self.half_height_of_cylinder)
            return local_surface_position.sub(Vec3.init(0, -self.half_height_of_cylinder, 0)).normalized()
        else
            return Vec3.init(local_surface_position.getX(), 0, local_surface_position.getZ()).normalizedOr(Vec3.axisX());
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const CapsuleShape, sub_shape_id: SubShapeID, direction_in: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
        std.debug.assert(self.isValidScale(scale));

        // Get direction in horizontal plane
        var direction = direction_in;
        direction.setComponent(1, 0.0);

        // Check zero vector, in this case we're hitting from top/bottom so there's no supporting face
        const len = direction.length();
        if (len == 0.0)
            return;

        // Get scaled capsule
        const abs_scale = scale.abs();
        const uniform_scale = abs_scale.getX();
        const scaled_half_height_of_cylinder = Vec3.init(0, uniform_scale * self.half_height_of_cylinder, 0);
        const scaled_radius = uniform_scale * self.radius;

        // Get support point for top and bottom sphere in the opposite of 'direction' (including convex radius)
        const support = direction.mulScalar(scaled_radius / len);
        const support_top = scaled_half_height_of_cylinder.sub(support);
        const support_bottom = scaled_half_height_of_cylinder.negate().sub(support);

        // Get projection on direction_in
        // Note that direction_in is not normalized, so we need to divide by direction_in.Length() to get the actual projection
        // We've multiplied both sides of the if below with direction_in.Length()
        const proj_top = support_top.dot(direction_in);
        const proj_bottom = support_bottom.dot(direction_in);

        // If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
        if (@abs(proj_top - proj_bottom) < PhysicsSettings.capsule_projection_slop * direction_in.length()) {
            out_vertices.append(center_of_mass_transform.mulVec3(support_top));
            out_vertices.append(center_of_mass_transform.mulVec3(support_bottom));
        }
    }

    // See ConvexShape::GetSupportFunction: placement new into the caller's buffer
    pub fn getSupportFunction(self: *const CapsuleShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        std.debug.assert(self.isValidScale(scale));

        // Get scaled capsule
        const abs_scale = scale.abs();
        const uniform_scale = abs_scale.getX();
        const scaled_half_height_of_cylinder = Vec3.init(0, uniform_scale * self.half_height_of_cylinder, 0);
        const scaled_radius = uniform_scale * self.radius;

        switch (mode) {
            .include_convex_radius => {
                const support = buffer.emplace(CapsuleWithConvex);
                support.* = .init(scaled_half_height_of_cylinder, scaled_radius);
                return &support.base;
            },

            .exclude_convex_radius, .default => {
                const support = buffer.emplace(CapsuleNoConvex);
                support.* = .init(scaled_half_height_of_cylinder, scaled_radius);
                return &support.base;
            },
        }
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay (`using ConvexShape::CastRay`: the collector version is ConvexShape's)
    pub fn castRay(self: *const CapsuleShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Test ray against capsule
        const fraction = rayCapsule(ray.origin, ray.direction, self.half_height_of_cylinder, self.radius);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const CapsuleShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const radius_sq = math.square(self.radius);

        // Get vertical distance to the top/bottom sphere centers
        const delta_y = @abs(point.getY()) - self.half_height_of_cylinder;

        // Get distance in horizontal plane
        const xz_sq = math.square(point.getX()) + math.square(point.getZ());

        // Check if the point is in one of the two spheres
        const in_sphere = xz_sq + math.square(delta_y) <= radius_sq;

        // Check if the point is in the cylinder in the middle
        const in_cylinder = delta_y <= 0.0 and xz_sq <= radius_sq;

        if (in_sphere or in_cylinder)
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const CapsuleShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        std.debug.assert(self.isValidScale(scale));

        const inverse_transform = center_of_mass_transform.inversedRotationTranslation();

        // Get scaled capsule
        const uniform_scale = @abs(scale.getX());
        const half_height_of_cylinder = uniform_scale * self.half_height_of_cylinder;
        const radius = uniform_scale * self.radius;

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                // Calculate penetration
                const local_pos = inverse_transform.mulVec3(v.getPosition());
                if (@abs(local_pos.getY()) <= half_height_of_cylinder) {
                    // Near cylinder
                    var normal = local_pos;
                    normal.setY(0.0);
                    const normal_length = normal.length();
                    const penetration = radius - normal_length;
                    if (v.updatePenetration(penetration)) {
                        // Calculate contact point and normal
                        normal = if (normal_length > 0.0) normal.divScalar(normal_length) else Vec3.axisX();
                        const point = normal.mulScalar(radius);

                        // Store collision
                        v.setCollision(Plane.fromPointAndNormal(point, normal).getTransformed(center_of_mass_transform), colliding_shape_index);
                    }
                } else {
                    // Near cap
                    const center = Vec3.init(0, math.sign(local_pos.getY()) * half_height_of_cylinder, 0);
                    const delta = local_pos.sub(center);
                    const distance = delta.length();
                    const penetration = radius - distance;
                    if (v.updatePenetration(penetration)) {
                        // Calculate contact point and normal
                        const normal = delta.divScalar(distance);
                        const point = center.add(normal.mulScalar(radius));

                        // Store collision
                        v.setCollision(Plane.fromPointAndNormal(point, normal).getTransformed(center_of_mass_transform), colliding_shape_index);
                    }
                }
            }
        }
    }

    // See Shape::GetTrianglesStart: placement new of a context (no pointers into itself: construct by value)
    pub fn getTrianglesStart(self: *const CapsuleShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        std.debug.assert(self.isValidScale(scale));

        const abs_scale = scale.abs();
        const uniform_scale = abs_scale.getX();

        const ctx = context.emplace(GetTrianglesContextMultiVertexList);
        ctx.* = .init(false, self.base.getConvexMaterial());

        const world_matrix = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scale(uniform_scale));

        const top_matrix = world_matrix.mul(Mat44.init(Vec4.init(self.radius, 0, 0, 0), Vec4.init(0, self.radius, 0, 0), Vec4.init(0, 0, self.radius, 0), Vec4.init(0, self.half_height_of_cylinder, 0, 1)));
        ctx.addPart(top_matrix, capsule_top_triangles.constSlice());

        const middle_matrix = world_matrix.mul(Mat44.scaleVec3(Vec3.init(self.radius, self.half_height_of_cylinder, self.radius)));
        ctx.addPart(middle_matrix, capsule_middle_triangles.constSlice());

        const bottom_matrix = world_matrix.mul(Mat44.init(Vec4.init(self.radius, 0, 0, 0), Vec4.init(0, self.radius, 0, 0), Vec4.init(0, 0, self.radius, 0), Vec4.init(0, -self.half_height_of_cylinder, 0, 1)));
        ctx.addPart(bottom_matrix, capsule_bottom_triangles.constSlice());
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const CapsuleShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextMultiVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    // See Shape::SaveBinaryState: C++ `ConvexShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const CapsuleShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.radius);
        stream.write(self.half_height_of_cylinder);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const CapsuleShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(CapsuleShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const CapsuleShape) f32 {
        return @as(f32, 4.0) / 3.0 * math.pi * math.cubed(self.radius) + 2.0 * math.pi * self.half_height_of_cylinder * math.square(self.radius);
    }

    // See Shape::IsValidScale: C++ `ConvexShape::IsValidScale` resolves to Shape's version (ConvexShape does not override it)
    pub fn isValidScale(self: *const CapsuleShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScale(scale.abs());
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const CapsuleShape, scale: Vec3) Vec3 {
        _ = self;
        const s = ScaleHelpers.makeNonZeroScale(scale);

        return s.getSign().mul(ScaleHelpers.makeUniformScale(s.abs()));
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *CapsuleShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.radius);
        stream.read(&self.half_height_of_cylinder);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.capsule);
        f.construct = ShapeFunctions.constructor(CapsuleShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Classes for GetSupportFunction (`class CapsuleNoConvex final : public Support`)

    const CapsuleNoConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        half_height_of_cylinder: Vec3,
        convex_radius: f32,

        fn init(half_height_of_cylinder: Vec3, convex_radius: f32) CapsuleNoConvex {
            return .{ .base = .init(CapsuleNoConvex), .half_height_of_cylinder = half_height_of_cylinder, .convex_radius = convex_radius };
        }

        pub fn getSupport(self: *const CapsuleNoConvex, direction: Vec3) Vec3 {
            if (direction.getY() > 0)
                return self.half_height_of_cylinder
            else
                return self.half_height_of_cylinder.negate();
        }

        pub fn getConvexRadius(self: *const CapsuleNoConvex) f32 {
            return self.convex_radius;
        }
    };

    const CapsuleWithConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        half_height_of_cylinder: Vec3,
        radius: f32,

        fn init(half_height_of_cylinder: Vec3, radius: f32) CapsuleWithConvex {
            return .{ .base = .init(CapsuleWithConvex), .half_height_of_cylinder = half_height_of_cylinder, .radius = radius };
        }

        pub fn getSupport(self: *const CapsuleWithConvex, direction: Vec3) Vec3 {
            const len = direction.length();
            const radius = if (len > 0.0) direction.mulScalar(self.radius / len) else Vec3.zero();

            if (direction.getY() > 0)
                return radius.add(self.half_height_of_cylinder)
            else
                return radius.sub(self.half_height_of_cylinder);
        }

        pub fn getConvexRadius(self: *const CapsuleWithConvex) f32 {
            _ = self;
            return 0.0;
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/CapsulesParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const VertexArrayList = @import("../../../Geometry/VertexArray.zig").VertexArrayList;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const CastRayCollector = ShapeFile.CastRayCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const BoxShapeSettings = @import("BoxShape.zig").BoxShapeSettings;
const RotatedTranslatedShapeSettings = @import("RotatedTranslatedShape.zig").RotatedTranslatedShapeSettings;

/// A filter that accepts or rejects everything and records its calls (state behind a pointer, Rule M)
const RecordingShapeFilter = struct {
    pub const overrides = .{.shouldCollide};

    const Log = struct {
        calls: u32 = 0,
        shape: ?*const Shape = null,
        sub_shape_id: SubShapeID = .empty,
    };

    base: ShapeFilter = .init(@This()),
    accept: bool,
    log: *Log,

    pub fn shouldCollide(self: *const RecordingShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.calls += 1;
        self.log.shape = shape2;
        self.log.sub_shape_id = sub_shape_id_of_shape2;
        return self.accept;
    }
};

test "CapsuleShape: settings, Jolt's error texts, a sphere for a zero height, cached results and out of memory" {
    const allocator = testing.allocator;

    // Invalid settings: the height is checked first
    const invalid = [_]struct { half_height: f32, radius: f32, err: []const u8 }{
        .{ .half_height = -1.0, .radius = 1.0, .err = "Invalid height" },
        .{ .half_height = 0.0, .radius = 0.0, .err = "Invalid height" }, // Not valid, so not a sphere
        .{ .half_height = -1.0, .radius = -1.0, .err = "Invalid height" },
        .{ .half_height = 1.0, .radius = 0.0, .err = "Invalid radius" },
        .{ .half_height = 1.0, .radius = -2.0, .err = "Invalid radius" },
    };
    for (invalid) |c| {
        var settings = CapsuleShapeSettings.init(allocator, c.half_height, c.radius, .{});
        defer settings.deinit();
        try testing.expect(!settings.isValid() or !settings.isSphere());
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings(c.err, result.getError());
    }

    // Default constructor (deserialization): no radius, no height
    var default_settings = CapsuleShapeSettings.initDefault(allocator);
    defer default_settings.deinit();
    try testing.expect(default_settings.radius == 0.0 and default_settings.half_height_of_cylinder == 0.0);
    var default_result = try default_settings.asShapeSettings().createShape(allocator);
    defer default_result.deinit();
    try testing.expectEqualStrings("Invalid height", default_result.getError());

    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material()); // Used by several settings
    defer material_ref.deinit();

    // A zero height makes a sphere (also -0): SphereShape(radius, material) keeps the default density and user data
    for ([_]f32{ 0.0, -0.0 }) |half_height| {
        var settings = CapsuleShapeSettings.init(allocator, half_height, 2.0, .{ .material = material.material() });
        defer settings.deinit();
        settings.base.density = 500.0;
        settings.asShapeSettings().user_data = 7;
        try testing.expect(settings.isValid() and settings.isSphere());
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const sphere = result.getPtr().?.cast(SphereShape);
        try testing.expectEqual(@as(f32, 2.0), sphere.getRadius());
        try testing.expect(sphere.base.getConvexMaterial() == material.material());
        try testing.expectEqual(@as(f32, 1000.0), sphere.base.getDensity());
        try testing.expectEqual(@as(u64, 0), sphere.asShape().getUserData());

        // Cached: the same sphere again
        var again = try settings.asShapeSettings().createShape(allocator);
        defer again.deinit();
        try testing.expect(again.getPtr() == result.getPtr());
    }

    // Heap settings: material, density and user data are passed to the capsule
    const settings = try CapsuleShapeSettings.create(allocator, 1.5, 0.25, .{ .material = material.material() });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    settings.base.setDensity(321.0);
    settings.asShapeSettings().user_data = 99;
    var result = try settings.createShape(allocator);
    defer result.deinit();
    const capsule = result.getPtr().?.cast(CapsuleShape);
    try testing.expectEqual(@as(f32, 1.5), capsule.getHalfHeightOfCylinder());
    try testing.expectEqual(@as(f32, 0.25), capsule.getRadius());
    try testing.expect(capsule.asShape().getMaterial(.empty) == material.material());
    try testing.expectEqual(@as(f32, 321.0), capsule.base.getDensity());
    try testing.expectEqual(@as(u64, 99), capsule.asShape().getUserData());
    try testing.expectEqual(ShapeSubType.capsule, capsule.asShape().getSubType());

    // Out of memory while creating the shape (capsule or sphere) is returned and not cached, a later call succeeds
    for ([_]f32{ 1.0, 0.0 }) |half_height| {
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var oom_settings = CapsuleShapeSettings.init(allocator, half_height, 0.5, .{});
            defer oom_settings.deinit();
            var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var r = oom_settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
                try testing.expectEqual(error.OutOfMemory, err);
                try testing.expect(oom_settings.base.base.cached_result.isEmpty());
                continue;
            };
            defer r.deinit();
            try testing.expect(r.isValid());
            try testing.expectEqual(if (half_height == 0.0) ShapeSubType.sphere else ShapeSubType.capsule, r.getPtr().?.getSubType());
            try testing.expectEqual(@as(usize, 1), fail_index); // Only the shape is allocated
            break;
        }
    }
}

test "CapsuleShape: bounds, inner radius, mass properties, volume, stats, surface normals, supporting faces" {
    const allocator = testing.allocator;

    var capsule = CapsuleShape.init(allocator, 2.0, 0.5, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();
    const shape = capsule.asShape();

    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.init(-0.5, -2.5, -0.5), Vec3.init(0.5, 2.5, 0.5))));
    try testing.expect(shape.getWorldSpaceBounds(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.replicate(-2.0)).eql(.init(Vec3.init(0, -3, 2), Vec3.init(2, 7, 4))));
    const rotated = shape.getWorldSpaceBounds(Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.zero()), Vec3.one()); // Lying along X
    try testing.expect(rotated.min.isClose(Vec3.init(-2.5, -0.5, -0.5), .{ .max_dist_sq = 1.0e-10 }) and rotated.max.isClose(Vec3.init(2.5, 0.5, 0.5), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(@as(f32, 0.5), shape.getInnerRadius());
    try testing.expect(shape.getCenterOfMass().eql(Vec3.zero()));
    try testing.expectEqual(@as(u32, 0), shape.getSubShapeIDBitsRecursive());
    try testing.expectEqual(@as(usize, @sizeOf(CapsuleShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);

    // Volume and mass: a cylinder of height 4 and two half spheres
    const volume: f32 = 4.0 / 3.0 * std.math.pi * 0.125 + std.math.pi * 4.0 * 0.25;
    try testing.expectApproxEqRel(volume, shape.getVolume(), 1.0e-6);
    const p = shape.getMassProperties();
    try testing.expectApproxEqRel(1000.0 * volume, p.mass, 1.0e-6);
    const cylinder_mass: f32 = std.math.pi * 4.0 * 0.25 * 1000.0;
    const hemisphere_mass: f32 = 2.0 * std.math.pi / 3.0 * 0.125 * 1000.0;
    try testing.expectApproxEqRel(0.25 * cylinder_mass * 0.5 + hemisphere_mass * 4.0 * 0.25 / 5.0, p.inertia.get(1, 1), 1.0e-6);
    try testing.expectApproxEqRel(0.25 * cylinder_mass * 0.25 + cylinder_mass * 16.0 / 12.0 + hemisphere_mass * 4.0 * 0.25 / 5.0 + hemisphere_mass * (0.5 * 16.0 + 0.75 * 4.0 * 0.5), p.inertia.get(0, 0), 1.0e-6);
    try testing.expectEqual(p.inertia.get(0, 0), p.inertia.get(2, 2));
    try testing.expectEqual(@as(f32, 0.0), p.inertia.get(0, 1));

    // Surface normals: the caps, the cylinder (and the axis, where the normal is X)
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, 2.5, 0)).eql(Vec3.axisY()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, -2.5, 0)).eql(Vec3.axisY().negate()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, 2.0, 0.5)).eql(Vec3.axisZ())); // Exactly at the top of the cylinder
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(-0.5, 1.0, 0)).eql(Vec3.axisX().negate()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.zero()).eql(Vec3.axisX()));

    // Supporting face: a line on the side of the cylinder in the opposite direction, scaled and transformed
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.init(2, 0, 0), Vec3.replicate(2.0), Mat44.translation(Vec3.init(10, 0, 0)), &face);
    try testing.expectEqual(@as(u32, 2), face.len);
    try testing.expect(face.get(0).eql(Vec3.init(9, 4, 0)) and face.get(1).eql(Vec3.init(9, -4, 0)));

    // Slightly tilted: still a line (within cCapsuleProjectionSlop)
    face.clear();
    shape.getSupportingFace(.empty, Vec3.init(1, 0.001, 0), Vec3.one(), Mat44.identity(), &face);
    try testing.expectEqual(@as(u32, 2), face.len);

    // Hitting a cap: no face
    for ([_]Vec3{ Vec3.init(0, 1, 0), Vec3.init(1, 1, 0), Vec3.zero() }) |direction| {
        face.clear();
        shape.getSupportingFace(.empty, direction, Vec3.one(), Mat44.identity(), &face);
        try testing.expectEqual(@as(u32, 0), face.len);
    }

    // GetSubmergedVolume is ConvexShape's (bounding box based)
    const half = shape.getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.fromPointAndNormal(Vec3.zero(), Vec3.axisY()));
    try testing.expectEqual(@as(f32, 5.0), half.total_volume);
    try testing.expectApproxEqAbs(@as(f32, 2.5), half.submerged_volume, 1.0e-5);
}

test "CapsuleShape: support functions" {
    const allocator = testing.allocator;

    var capsule = CapsuleShape.init(allocator, 2.0, 0.5, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();

    var buffer: ConvexShape.SupportBuffer = .{};
    const scale = Vec3.init(-2, 2, -2);

    // Include convex radius: the scaled capsule, the convex radius is 0
    const with_convex = capsule.base.getSupportFunction(.include_convex_radius, &buffer, scale);
    try testing.expectEqual(@as(f32, 0.0), with_convex.getConvexRadius());
    try testing.expect(with_convex.getSupport(Vec3.init(2, 0, 0)).eql(Vec3.init(1, -4, 0))); // Not up: the bottom sphere
    try testing.expect(with_convex.getSupport(Vec3.init(0, 3, 0)).eql(Vec3.init(0, 5, 0)));
    try testing.expect(with_convex.getSupport(Vec3.init(0, 0, -0.5)).eql(Vec3.init(0, -4, -1)));
    try testing.expect(with_convex.getSupport(Vec3.zero()).eql(Vec3.init(0, -4, 0)));

    // Exclude convex radius and default: the line segment with the scaled radius as convex radius
    for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .default }) |mode| {
        const no_convex = capsule.base.getSupportFunction(mode, &buffer, scale);
        try testing.expectEqual(@as(f32, 1.0), no_convex.getConvexRadius());
        try testing.expect(no_convex.getSupport(Vec3.init(5, 6, 7)).eql(Vec3.init(0, 4, 0)));
        try testing.expect(no_convex.getSupport(Vec3.init(5, -6, 7)).eql(Vec3.init(0, -4, 0)));
        try testing.expect(no_convex.getSupport(Vec3.init(5, 0, 7)).eql(Vec3.init(0, -4, 0)));
    }
}

// Jolt's TestCapsuleShapeRay / TestCollidePointVsCapsule are in ZoltTests/Physics, this adds specific fractions, sub shape IDs
// and the shape filter
test "CapsuleShape: analytic ray casts, the collector version of ConvexShape, collide point with sub shape IDs and the shape filter" {
    const allocator = testing.allocator;

    var shape_ref = Ref(Shape).init((try CapsuleShape.create(allocator, 4, 2, .{})).asShapeMut());
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;
    const creator = SubShapeIDCreator.pushID(.{}, 1, 3);

    // Single hit: the analytic version
    var hit: RayCastResult = .{};
    try testing.expect(shape.castRay(.init(Vec3.init(-4, 0, 0), Vec3.init(8, 0, 0)), creator, &hit));
    try testing.expectEqual(@as(f32, 0.25), hit.fraction);
    try testing.expect(hit.sub_shape_id2.eql(creator.getID()));
    try testing.expect(shape.castRay(.init(Vec3.init(0, -8, 0), Vec3.init(0, 16, 0)), creator, &hit)); // The bottom cap
    try testing.expectEqual(@as(f32, 0.125), hit.fraction);
    try testing.expect(!shape.castRay(.init(Vec3.init(0, 0, -6), Vec3.init(0, 0, 8)), creator, &hit)); // Hit at 0.5 is not closer
    try testing.expect(shape.castRay(.init(Vec3.init(0, 5, 0), Vec3.init(0, 0, 8)), creator, &hit)); // Starts inside the top cap
    try testing.expectEqual(@as(f32, 0.0), hit.fraction);
    hit = .{};
    try testing.expect(!shape.castRay(.init(Vec3.init(-3, 0, 0), Vec3.init(0, 1, 0)), creator, &hit)); // Parallel to the axis outside the cylinder
    try testing.expect(!shape.castRay(.init(Vec3.init(-4, 0, 0), Vec3.init(1, 0, 0)), creator, &hit)); // Too short

    // Collector: ConvexShape's version calls the analytic CastRay for the front and the back face (TestRayHelper)
    const cases = [_]struct { a: Vec3, b: Vec3 }{
        .{ .a = Vec3.init(-2, 0, 0), .b = Vec3.init(2, 0, 0) },
        .{ .a = Vec3.init(0, -6, 0), .b = Vec3.init(0, 6, 0) },
        .{ .a = Vec3.init(0, 0, -2), .b = Vec3.init(0, 0, 2) },
    };
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    for (cases) |c| {
        for ([2][2]Vec3{ .{ c.a, c.b }, .{ c.b, c.a } }) |ab| {
            const delta = ab[1].sub(ab[0]);
            const l2 = ab[0].sub(delta.mulScalar(0.1));
            const r1 = ab[1].add(delta.mulScalar(0.1));
            var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer hits.deinit();
            shape.castRayCollector(.init(l2, r1.sub(l2)), &settings, creator, &hits.base, &.{});
            try hits.checkError();
            try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
            try testing.expectApproxEqAbs(@as(f32, 0.1) / 1.2, hits.hits.items[0].fraction, 1.0e-5);
            try testing.expectApproxEqAbs(@as(f32, 1.1) / 1.2, hits.hits.items[1].fraction, 1.0e-5);
        }
    }

    // The points of TestCollidePointVsCapsule with a sub shape ID creator
    const half_height: f32 = 0.2;
    const radius: f32 = 0.1;
    var point_shape_ref = Ref(Shape).init((try CapsuleShape.create(allocator, half_height, radius, .{})).asShapeMut());
    defer point_shape_ref.deinit();
    const point_shape = point_shape_ref.get().?;
    const xy_and_zero_probes = [_]Vec3{ Vec3.zero(), Vec3.init(-1, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 0, -1), Vec3.init(0, 0, 1) };
    const cube_probes = [_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, -1, 0), Vec3.init(0, 1, 0), Vec3.init(0, 0, -1), Vec3.init(0, 0, 1) };
    var hit_points: [2 * xy_and_zero_probes.len + 1]Vec3 = undefined;
    for (xy_and_zero_probes, 0..) |probe, i| {
        hit_points[i] = probe.mulScalar(0.99 * radius).add(Vec3.init(0, half_height, 0)); // Top hits
        hit_points[xy_and_zero_probes.len + i] = probe.mulScalar(0.99 * radius).add(Vec3.init(0, -half_height, 0)); // Bottom hits
    }
    hit_points[2 * xy_and_zero_probes.len] = Vec3.zero(); // Center hit
    for (hit_points) |point| {
        var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer points.deinit();
        point_shape.collidePoint(point, creator, &points.base, &.{});
        try points.checkError();
        try testing.expectEqual(@as(usize, 1), points.hits.items.len);
        try testing.expect(points.hits.items[0].sub_shape_id2.eql(creator.getID()));
    }
    for (cube_probes) |probe| {
        // Misses
        var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer points.deinit();
        point_shape.collidePoint(Vec3.init(radius, half_height + radius, radius).mul(probe).mulScalar(1.01), creator, &points.base, &.{});
        try testing.expectEqual(@as(usize, 0), points.hits.items.len);
    }

    // The shape filter is called with the capsule and the ID of the creator, rejecting it skips a point inside
    for ([_]bool{ true, false }) |accept| {
        var log: RecordingShapeFilter.Log = .{};
        const filter: RecordingShapeFilter = .{ .accept = accept, .log = &log };
        var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer points.deinit();
        point_shape.collidePoint(Vec3.zero(), creator, &points.base, &filter.base);
        try points.checkError();
        try testing.expectEqual(@as(usize, @intFromBool(accept)), points.hits.items.len);
        try testing.expectEqual(@as(u32, 1), log.calls);
        try testing.expect(log.shape.? == point_shape);
        try testing.expect(!creator.getID().eql(.empty) and log.sub_shape_id.eql(creator.getID()));
    }
}

test "CapsuleShape: CollideSoftBodyVertices" {
    const allocator = testing.allocator;

    var capsule = CapsuleShape.init(allocator, 1.0, 0.5, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();

    // Scale 2: half height 2, radius 1
    var positions = [_]Vec3{ Vec3.init(1, 2.5, 3.5), Vec3.init(1, 2, 3), Vec3.init(1, 5, 3), Vec3.init(1, -1.5, 3), Vec3.init(1, 2, 3) };
    var inv_masses = [_]f32{ 1, 1, 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 5;
    var penetrations: [5]f32 = .{ -math.flt_max, -math.flt_max, -math.flt_max, 0.0, -math.flt_max };
    var indices = [_]i32{-1} ** 5;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    capsule.asShape().collideSoftBodyVertices(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.replicate(-2.0), &vertices, 5, 7);

    // Near the cylinder: penetration 0.5, plane through the surface with the outward normal
    try testing.expectEqual(@as(f32, 0.5), penetrations[0]);
    try testing.expectEqual(@as(i32, 7), indices[0]);
    try testing.expect(planes[0].getNormal().eql(Vec3.axisZ()));
    try testing.expectEqual(@as(f32, 0.0), planes[0].signedDistance(Vec3.init(1, 2, 4)));
    // On the axis: the normal is X
    try testing.expectEqual(@as(f32, 1.0), penetrations[1]);
    try testing.expect(planes[1].getNormal().eql(Vec3.axisX()));
    // Touching the top cap
    try testing.expectEqual(@as(f32, 0.0), penetrations[2]);
    try testing.expect(planes[2].getNormal().eql(Vec3.axisY()));
    try testing.expectEqual(@as(f32, 0.0), planes[2].signedDistance(Vec3.init(1, 5, 3)));
    // Outside the bottom cap with a larger penetration already stored: untouched
    try testing.expectEqual(@as(f32, 0.0), penetrations[3]);
    try testing.expectEqual(@as(i32, -1), indices[3]);
    // Infinite mass: skipped
    try testing.expectEqual(-math.flt_max, penetrations[4]);
}

test "CapsuleShape: GetTrianglesStart / Next and the static vertex lists" {
    const allocator = testing.allocator;

    // The comptime tables have the bits of a runtime build (Jolt's static initializers)
    var level: u32 = capsule_detail_level;
    _ = &level;
    var top: std.ArrayList(Vec3) = .empty;
    defer top.deinit(allocator);
    try GetTrianglesContextVertexList.createHalfUnitSphereTop(VertexArrayList.init(allocator, &top), level);
    var middle: std.ArrayList(Vec3) = .empty;
    defer middle.deinit(allocator);
    try GetTrianglesContextVertexList.createUnitOpenCylinder(VertexArrayList.init(allocator, &middle), level);
    var bottom: std.ArrayList(Vec3) = .empty;
    defer bottom.deinit(allocator);
    try GetTrianglesContextVertexList.createHalfUnitSphereBottom(VertexArrayList.init(allocator, &bottom), level);
    const runtime_tables = [_][]const Vec3{ top.items, middle.items, bottom.items };
    const tables = [_][]const Vec3{ capsule_top_triangles.constSlice(), capsule_middle_triangles.constSlice(), capsule_bottom_triangles.constSlice() };
    for (runtime_tables, tables) |runtime_table, table| {
        try testing.expectEqual(runtime_table.len, table.len);
        try testing.expect(std.mem.eql(u8, std.mem.sliceAsBytes(runtime_table), std.mem.sliceAsBytes(table)));
    }

    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();

    // Every triangle has the material of the capsule (GetMaterial(): the default material when it has none)
    for ([_]?*const PhysicsMaterial{ null, material.material() }) |capsule_material| {
        const expected_material = capsule_material orelse PhysicsMaterial.default;
        var capsule = CapsuleShape.init(allocator, 1.0, 0.5, .{ .material = capsule_material });
        capsule.asShape().setEmbedded();
        defer capsule.asShapeMut().deinit();

        // 64 + 32 + 64 triangles on the surface of the scaled capsule (the sign of the scale does not flip the winding)
        var first: [3 * 160]Float3 = undefined;
        for ([_]Vec3{ Vec3.replicate(2.0), Vec3.init(-2, 2, -2) }, 0..) |scale, pass| {
            var context: Shape.GetTrianglesContext = .{};
            capsule.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.init(1, 2, 3), Quat.identity(), scale);
            var all: [3 * 160]Float3 = undefined;
            var count: usize = 0;
            var vertices: [3 * 32]Float3 = undefined;
            var materials: [32]*const PhysicsMaterial = undefined;
            for (0..5) |_| {
                try testing.expectEqual(@as(u32, 32), capsule.asShape().getTrianglesNext(&context, 32, &vertices, &materials));
                @memcpy(all[3 * count ..][0 .. 3 * 32], &vertices);
                count += 32;
                for (materials) |m|
                    try testing.expect(m == expected_material);
            }
            try testing.expectEqual(@as(u32, 0), capsule.asShape().getTrianglesNext(&context, 32, &vertices, null));
            for (all) |f| {
                // Distance to the line segment (1, 0, 3) - (1, 4, 3) is the scaled radius
                const v = Vec3.fromFloat3(f);
                const closest = Vec3.init(1, math.clamp(v.getY(), 0.0, 4.0), 3);
                try testing.expectApproxEqAbs(@as(f32, 1.0), v.sub(closest).length(), 1.0e-5);
            }
            if (pass == 0)
                first = all
            else
                try testing.expect(std.mem.eql(u8, std.mem.sliceAsBytes(&first), std.mem.sliceAsBytes(&all)));
        }
    }
}

test "CapsuleShape: binary state, restoreFromBinaryState and the registration" {
    const allocator = testing.allocator;

    var capsule = CapsuleShape.init(allocator, 1.5, 0.25, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();
    capsule.base.setDensity(321.0);
    capsule.asShapeMut().setUserData(5);

    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    capsule.asShape().saveBinaryState(out.streamOut());
    try testing.expectEqual(@as(usize, 1 + 8 + 4 + 4 + 4), writer.buffered().len); // Sub type, user data, density, radius, half height

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    const restored = result.getPtr().?.cast(CapsuleShape);
    try testing.expectEqual(@as(f32, 0.25), restored.getRadius());
    try testing.expectEqual(@as(f32, 1.5), restored.getHalfHeightOfCylinder());
    try testing.expectEqual(@as(f32, 321.0), restored.base.getDensity());
    try testing.expectEqual(@as(u64, 5), restored.asShape().getUserData());

    // Truncated: Jolt's error text
    var short_reader: std.Io.Reader = .fixed(writer.buffered()[0 .. writer.buffered().len - 1]);
    var short_in = StreamWrapper.StreamInWrapper.init(&short_reader);
    var short_result = try Shape.restoreFromBinaryState(allocator, short_in.streamIn());
    defer short_result.deinit();
    try testing.expectEqualStrings("Failed to restore shape", short_result.getError());

    // ShapeFunctions
    try testing.expect(ShapeFunctions.get(.capsule).construct != null);
    try testing.expect(ShapeFunctions.get(.capsule).color.eql(Color.green));
}

test "CapsuleShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, CapsuleShape.create(failing.allocator(), 1.0, 1.0, .{}));
    try testing.expectError(error.OutOfMemory, CapsuleShapeSettings.create(failing.allocator(), 1.0, 1.0, .{}));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.capsule).construct.?(failing.allocator()));

    // Restore: the shape is the only allocation
    var capsule = CapsuleShape.init(allocator, 1.0, 1.0, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();
    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    capsule.asShape().saveBinaryState(out.streamOut());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    try testing.expectError(error.OutOfMemory, Shape.restoreFromBinaryState(failing.allocator(), in.streamIn()));
}
