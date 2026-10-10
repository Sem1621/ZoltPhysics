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
