//! Port of: Jolt/Physics/Collision/Shape/PlaneShape.h, Jolt/Physics/Collision/Shape/PlaneShape.cpp
//! Status: complete
//!
//! A concrete shape that derives from Shape directly (D1: `base: Shape`, built with `Shape.vtableFor(PlaneShape)`),
//! following the porter template of Docs/Zolt/CollisionArchitecture.md (section 2):
//! - `PlaneShapeSettings` / `PlaneShape` own a `RefConst(PhysicsMaterial)` (released in `destruct`), the C++ constructors
//!   are `initDefault` (default constructor), `initFromSettings` (D3, Jolt's error text), `init` (stack / member shapes,
//!   `PlaneShape(inPlane, inMaterial, inHalfExtent)`) and `create` (`new PlaneShape(...)`).
//! - `PSGetTrianglesContext` is constructed in the caller's GetTrianglesContext (placement new, D10).
//! - sCollideConvexVsPlane / sCastConvexVsPlane are `collideConvexVsPlane` / `castConvexVsPlane` (D4), registered by
//!   `register` (PlaneShape::sRegister) for every convex sub shape type, with the reversed functions of
//!   CollisionDispatch for plane vs convex.
//!
//! Names that differ from C++:
//! - The non virtual `PlaneShape::GetMaterial()` is `getPlaneMaterial()` (the virtual `getMaterial(sub_shape_id)` has
//!   the plain name, like `ConvexShape.getConvexMaterial`).
//! - The file static helpers `sPlaneGetOrthogonalBasis(normal, outPerp1, outPerp2)` and `sGetSupportingFace(...)` are
//!   `planeGetOrthogonalBasis(normal) OrthogonalBasis` and `planeGetSupportingFace(...)` (the plain name would shadow
//!   the virtual `getSupportingFace`); `GetVertices(outVertices)` returns the 4 vertices.
//! - `GetSubmergedVolume` asserts "Not supported" like Jolt; Jolt's release build then leaves the out parameters
//!   untouched, Zolt returns zeros (the out parameters are a returned struct).
//! - The fields that Jolt leaves uninitialized (`PlaneShapeSettings() = default` / `PlaneShape()`: the plane, and the
//!   half extent of the shape) get defined values: the plane `(0, 0, 0), 0` (an invalid plane, Create() reports
//!   "Plane normal needs to be normalized!") and a half extent of 0.
//! - JPH_DEBUG_RENDERER (Draw) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Color = @import("../../../Core/Color.zig").Color;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
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
const PhysicsMaterialList = ShapeFile.PhysicsMaterialList;
const PhysicsMaterialRefC = ShapeFile.PhysicsMaterialRefC;
const ConvexShape = @import("ConvexShape.zig").ConvexShape;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeFile = @import("../CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const NarrowPhaseStats = @import("../NarrowPhaseStats.zig");
const TrackNarrowPhaseCollector = NarrowPhaseStats.TrackNarrowPhaseCollector;
const track_narrowphase_stats = NarrowPhaseStats.track_narrowphase_stats;

/// Class that constructs a PlaneShape
pub const PlaneShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, PlaneShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    /// Default half-extent of the plane (total size along 1 axis will be 2 * half-extent)
    pub const default_half_extent: f32 = 1000.0;

    base: ShapeSettings,
    /// Plane that describes the shape. The negative half space is considered solid.
    /// (Jolt leaves it uninitialized in the default constructor, Zolt uses an invalid plane, see the file comment)
    plane: Plane = Plane.init(Vec3.zero(), 0.0),
    /// Surface material of the plane
    material: RefConst(PhysicsMaterial) = .empty,
    /// The bounding box of this plane will run from [-half_extent, half_extent]. Keep this as low as possible for better broad phase performance.
    half_extent: f32 = default_half_extent,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) PlaneShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(PlaneShapeSettings), allocator) };
    }

    /// Create a plane shape (settings on the stack: `defer settings.deinit()`)
    pub fn init(allocator: Allocator, plane: Plane, opts: struct { material: ?*const PhysicsMaterial = null, half_extent: f32 = default_half_extent }) PlaneShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(PlaneShapeSettings), allocator), .plane = plane, .material = .init(opts.material), .half_extent = opts.half_extent };
    }

    /// new PlaneShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, plane: Plane, opts: struct { material: ?*const PhysicsMaterial = null, half_extent: f32 = default_half_extent }) Allocator.Error!*PlaneShapeSettings {
        const self = try allocator.create(PlaneShapeSettings);
        self.* = .init(allocator, plane, .{ .material = opts.material, .half_extent = opts.half_extent });
        return self;
    }

    /// ~PlaneShapeSettings (releases the material)
    pub fn destruct(self: *PlaneShapeSettings) void {
        self.material.deinit();
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *PlaneShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *PlaneShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *PlaneShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(PlaneShape, self, allocator);
    }
};

/// A plane shape. The negative half space is considered solid. Planes cannot be dynamic objects, only static or kinematic.
/// The plane is considered an infinite shape, but testing collision outside of its bounding box (defined by the half-extent parameter) will not return a collision result.
/// At the edge of the bounding box collision with the plane will be inconsistent. If you need something of a well defined size, a box shape may be better.
pub const PlaneShape = struct {
    /// Concrete class: `Shape.cast(PlaneShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .plane;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .mustBeStatic, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getSubmergedVolume, .saveBinaryState, .saveMaterialState, .restoreMaterialState, .getStats, .getVolume, .restoreBinaryState };

    base: Shape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the
    /// destructor chain (Jolt leaves the plane and the half extent of the default constructor uninitialized)
    plane: Plane = Plane.init(Vec3.zero(), 0.0),
    material: RefConst(PhysicsMaterial) = .empty,
    half_extent: f32 = 0.0,
    local_bounds: AABox = .empty,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// PlaneShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) PlaneShape {
        return .{ .base = .init(Shape.vtableFor(PlaneShape), allocator, .plane, shape_sub_type) };
    }

    /// PlaneShape(const PlaneShapeSettings &inSettings, ShapeResult &outResult): base part and member initializers
    /// first, then the C++ body
    pub fn initFromSettings(self: *PlaneShape, settings: *const PlaneShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base);
        self.plane = settings.plane;
        self.material.set(settings.material.get());
        self.half_extent = settings.half_extent;

        if (!self.plane.getNormal().isNormalized(.{})) {
            result.setError("Plane normal needs to be normalized!");
            return;
        }

        self.calculateLocalBounds();

        result.set(.init(self.asShapeMut()));
    }

    /// PlaneShape(inPlane, inMaterial = nullptr, inHalfExtent = PlaneShapeSettings::cDefaultHalfExtent) (on the stack /
    /// as a member: `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end). Like Jolt,
    /// the normal is not validated.
    pub fn init(allocator: Allocator, plane: Plane, opts: struct { material: ?*const PhysicsMaterial = null, half_extent: f32 = PlaneShapeSettings.default_half_extent }) PlaneShape {
        var self: PlaneShape = .{ .base = .init(Shape.vtableFor(PlaneShape), allocator, .plane, shape_sub_type), .plane = plane, .material = .init(opts.material), .half_extent = opts.half_extent };
        self.calculateLocalBounds();
        return self;
    }

    /// new PlaneShape(inPlane, inMaterial, inHalfExtent): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, plane: Plane, opts: struct { material: ?*const PhysicsMaterial = null, half_extent: f32 = PlaneShapeSettings.default_half_extent }) Allocator.Error!*PlaneShape {
        const self = try allocator.create(PlaneShape);
        self.* = .init(allocator, plane, .{ .material = opts.material, .half_extent = opts.half_extent });
        return self;
    }

    /// ~PlaneShape (releases the material)
    pub fn destruct(self: *PlaneShape) void {
        self.material.deinit();
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const PlaneShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *PlaneShape) *Shape {
        return &self.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get the plane
    pub fn getPlane(self: *const PlaneShape) Plane {
        return self.plane;
    }

    /// Get the half-extent of the bounding box of the plane
    pub fn getHalfExtent(self: *const PlaneShape) f32 {
        return self.half_extent;
    }

    /// Material of the shape (SetMaterial; like Jolt's, only for a shape that is not shared yet)
    pub fn setMaterial(self: *PlaneShape, material: ?*const PhysicsMaterial) void {
        self.material.set(material);
    }

    /// Material of the shape (Jolt's non virtual GetMaterial(), renamed: the virtual getMaterial(sub_shape_id) has the
    /// plain name)
    pub fn getPlaneMaterial(self: *const PlaneShape) *const PhysicsMaterial {
        return self.material.get() orelse PhysicsMaterial.default;
    }

    // Get 4 vertices that form the plane
    fn getVertices(self: *const PlaneShape) [4]Vec3 {
        // Create orthogonal basis
        const normal = self.plane.getNormal();
        const basis = planeGetOrthogonalBasis(normal);
        var perp1 = basis.perp1;
        var perp2 = basis.perp2;

        // Scale basis
        perp1 = perp1.mulScalar(self.half_extent);
        perp2 = perp2.mulScalar(self.half_extent);

        // Calculate corners
        const point = normal.negate().mulScalar(self.plane.getConstant());
        return .{
            point.add(perp1).add(perp2),
            point.add(perp1).sub(perp2),
            point.sub(perp1).sub(perp2),
            point.sub(perp1).add(perp2),
        };
    }

    // Cache the local bounds
    fn calculateLocalBounds(self: *PlaneShape) void {
        // Get the vertices of the plane
        const vertices = self.getVertices();

        // Encapsulate the vertices and a point mHalfExtent behind the plane
        self.local_bounds = .empty;
        const normal = self.plane.getNormal();
        for (vertices) |v| {
            self.local_bounds.encapsulateVec3(v);
            self.local_bounds.encapsulateVec3(v.sub(normal.mulScalar(self.half_extent)));
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::MustBeStatic
    pub fn mustBeStatic(self: *const PlaneShape) bool {
        _ = self;
        return true;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const PlaneShape) AABox {
        return self.local_bounds;
    }

    // See Shape::GetSubShapeIDBitsRecursive
    pub fn getSubShapeIDBitsRecursive(self: *const PlaneShape) u32 {
        _ = self;
        return 0;
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const PlaneShape) f32 {
        _ = self;
        return 0.0;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const PlaneShape) MassProperties {
        _ = self;

        // Object should always be static, return default mass properties
        return .{};
    }

    // See Shape::GetMaterial
    pub fn getMaterial(self: *const PlaneShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
        return self.getPlaneMaterial();
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const PlaneShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = local_surface_position;
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
        return self.plane.getNormal();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const PlaneShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = .{ sub_shape_id, direction };

        // Get the vertices of the plane
        var vertices = self.getVertices();

        // Reverse if scale is inside out
        if (ScaleHelpers.isInsideOut(scale)) {
            std.mem.swap(Vec3, &vertices[0], &vertices[3]);
            std.mem.swap(Vec3, &vertices[1], &vertices[2]);
        }

        // Transform them to world space
        out_vertices.clear();
        const com = center_of_mass_transform.preScaled(scale);
        for (vertices) |v|
            out_vertices.append(com.mulVec3(v));
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::CastRay
    pub fn castRay(self: *const PlaneShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Test starting inside of negative half space
        const distance = self.plane.signedDistance(ray.origin);
        if (distance <= 0.0) {
            hit.fraction = 0.0;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }

        // Test ray parallel to plane
        const dot = ray.direction.dot(self.plane.getNormal());
        if (dot == 0.0)
            return false;

        // Calculate hit fraction
        const fraction = -distance / dot;
        if (fraction >= 0.0 and fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }

        return false;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const PlaneShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Inside solid half space?
        const distance = self.plane.signedDistance(ray.origin);
        if (ray_cast_settings.treat_convex_as_solid and distance <= 0.0 // Inside plane
        and collector.getEarlyOutFraction() > 0.0) // Willing to accept hits at fraction 0
        {
            // Hit at fraction 0
            var hit: RayCastResult = .{};
            hit.body_id = TransformedShape.getBodyID(collector.getContext());
            hit.fraction = 0.0;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            collector.addHit(&hit);
        }

        const dot = ray.direction.dot(self.plane.getNormal());
        if (dot != 0.0 // Parallel ray will not hit plane
        and (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces or dot < 0.0)) // Back face culling
        {
            // Calculate hit with plane
            const fraction = -distance / dot;
            if (fraction >= 0.0 and fraction < collector.getEarlyOutFraction()) {
                var hit: RayCastResult = .{};
                hit.body_id = TransformedShape.getBodyID(collector.getContext());
                hit.fraction = fraction;
                hit.sub_shape_id2 = sub_shape_id_creator.getID();
                collector.addHit(&hit);
            }
        }
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const PlaneShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Check if the point is inside the plane
        if (self.plane.signedDistance(point) < 0.0)
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const PlaneShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        // Convert plane to world space
        const plane = self.plane.scaled(scale).getTransformed(center_of_mass_transform);

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                // Calculate penetration
                const penetration = -plane.signedDistance(v.getPosition());
                if (v.updatePenetration(penetration))
                    v.setCollision(plane, colliding_shape_index);
            }
        }
    }

    // See Shape::GetTrianglesStart: placement new of the context in the caller's buffer
    pub fn getTrianglesStart(self: *const PlaneShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        const ctx = context.emplace(PSGetTrianglesContext);
        ctx.* = .{};

        // Get the vertices of the plane
        var vertices = self.getVertices();

        // Reverse if scale is inside out
        if (ScaleHelpers.isInsideOut(scale)) {
            std.mem.swap(Vec3, &vertices[0], &vertices[3]);
            std.mem.swap(Vec3, &vertices[1], &vertices[2]);
        }

        // Transform them to world space
        const com = Mat44.rotationTranslation(rotation, position_com).preScaled(scale);
        for (0..4) |i|
            com.mulVec3(vertices[i]).storeFloat3(&ctx.vertices[i]);
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const PlaneShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        comptime {
            std.debug.assert(Shape.get_triangles_min_triangles_requested >= 2); // cGetTrianglesMinTrianglesRequested is too small
        }
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        // Check if we're done
        const ctx = context.get(PSGetTrianglesContext);
        if (ctx.done)
            return 0;
        ctx.done = true;

        // 1st triangle
        out_triangle_vertices[0] = ctx.vertices[0];
        out_triangle_vertices[1] = ctx.vertices[1];
        out_triangle_vertices[2] = ctx.vertices[2];

        // 2nd triangle
        out_triangle_vertices[3] = ctx.vertices[0];
        out_triangle_vertices[4] = ctx.vertices[2];
        out_triangle_vertices[5] = ctx.vertices[3];

        if (out_materials) |materials| {
            // Get material
            const material = self.getMaterial(.empty);
            materials[0] = material;
            materials[1] = material;
        }

        return 2;
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const PlaneShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };
        if (Core.enable_asserts) @panic("Not supported");

        // Jolt's release build leaves the out parameters untouched, Zolt returns zeros (see the file comment)
        return .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    // See Shape::SaveBinaryState: C++ `Shape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const PlaneShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.plane);
        stream.write(self.half_extent);
    }

    // See Shape::SaveMaterialState (`outMaterials = { mMaterial };`)
    pub fn saveMaterialState(self: *const PlaneShape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        for (out_materials.items) |*m| m.deinit();
        out_materials.clearRetainingCapacity();
        try out_materials.ensureUnusedCapacity(allocator, 1); // Allocate before taking the reference (out of memory leaves the list empty)
        out_materials.appendAssumeCapacity(self.material.clone());
    }

    // See Shape::RestoreMaterialState
    pub fn restoreMaterialState(self: *PlaneShape, materials: []const PhysicsMaterialRefC) Allocator.Error!void {
        if (Core.enable_asserts) std.debug.assert(materials.len == 1); // A corrupt stream can violate this
        self.material.set(materials[0].get());
    }

    // See Shape::GetStats
    pub fn getStats(self: *const PlaneShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(PlaneShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const PlaneShape) f32 {
        _ = self;
        return 0;
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *PlaneShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.plane);
        stream.read(&self.half_extent);

        self.calculateLocalBounds();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time) and the functions called by CollisionDispatch

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.plane);
        f.construct = ShapeFunctions.constructor(PlaneShape);
        f.color = Color.dark_red;

        for (ShapeFile.convex_sub_shape_types) |s| {
            r.registerCollideShape(s, .plane, collideConvexVsPlane);
            r.registerCastShape(s, .plane, castConvexVsPlane);

            r.registerCastShape(.plane, s, CollisionDispatch.reversedCastShape);
            r.registerCollideShape(.plane, s, CollisionDispatch.reversedCollideShape);
        }
    }

    /// sCollideConvexVsPlane
    fn collideConvexVsPlane(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        // Get the shapes
        std.debug.assert(shape1_in.getType() == .convex);
        std.debug.assert(shape2_in.getType() == .plane);
        const shape1 = shape1_in.cast(ConvexShape);
        const shape2 = shape2_in.cast(PlaneShape);

        // Transform the plane to the space of the convex shape
        const scaled_plane = shape2.plane.scaled(scale2);
        const plane = scaled_plane.getTransformed(center_of_mass_transform1.inversedRotationTranslation().mul(center_of_mass_transform2));
        const normal = plane.getNormal();

        // Get support function
        var shape1_support_buffer: ConvexShape.SupportBuffer = .{};
        const shape1_support = shape1.getSupportFunction(.default, &shape1_support_buffer, scale1);

        // Get the support point of the convex shape in the opposite direction of the plane normal
        const support_point = shape1_support.getSupport(normal.negate());
        const signed_distance = plane.signedDistance(support_point);
        const convex_radius = shape1_support.getConvexRadius();
        const penetration_depth = -signed_distance + convex_radius;
        if (penetration_depth > -collide_shape_settings.max_separation_distance) {
            // Get contact point
            const point1 = center_of_mass_transform1.mulVec3(support_point.sub(normal.mulScalar(convex_radius)));
            const point2 = center_of_mass_transform1.mulVec3(support_point.sub(normal.mulScalar(signed_distance)));
            const penetration_axis_world = center_of_mass_transform1.multiply3x3(normal.negate());

            // Create collision result
            var result = CollideShapeResult.init(point1, point2, penetration_axis_world, penetration_depth, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));

            // Gather faces
            if (collide_shape_settings.collect_faces_mode == .collect_faces) {
                // Get supporting face of shape 1
                shape1.base.getSupportingFace(.empty, normal, scale1, center_of_mass_transform1, &result.shape1_face);

                // Get supporting face of shape 2
                if (!result.shape1_face.isEmpty())
                    planeGetSupportingFace(shape1, center_of_mass_transform1.getTranslation(), scaled_plane, center_of_mass_transform2, &result.shape2_face);
            }

            // Notify the collector
            var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
            if (track_narrowphase_stats) track = .init();
            defer if (track_narrowphase_stats) track.deinit();
            collector.addHit(&result);
        }
    }

    /// sCastConvexVsPlane
    fn castConvexVsPlane(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        // Get the shapes
        std.debug.assert(shape_cast.shape.getType() == .convex);
        std.debug.assert(shape_in.getType() == .plane);
        const convex_shape = shape_cast.shape.cast(ConvexShape);
        const plane_shape = shape_in.cast(PlaneShape);

        // Shape cast is provided relative to COM of inShape, so all we need to do is transform our plane with inScale
        const plane = plane_shape.plane.scaled(scale);
        const normal = plane.getNormal();

        // Get support function
        var shape1_support_buffer: ConvexShape.SupportBuffer = .{};
        const shape1_support = convex_shape.getSupportFunction(.default, &shape1_support_buffer, shape_cast.scale);

        // Get the support point of the convex shape in the opposite direction of the plane normal in our local space
        const normal_in_convex_shape_space = shape_cast.center_of_mass_start.multiply3x3Transposed(normal);
        const support_point = shape_cast.center_of_mass_start.mulVec3(shape1_support.getSupport(normal_in_convex_shape_space.negate()));
        const signed_distance = plane.signedDistance(support_point);
        const convex_radius = shape1_support.getConvexRadius() + shape_cast_settings.extra_convex_radius;
        const penetration_depth = -signed_distance + convex_radius;
        const dot = shape_cast.direction.dot(normal);

        // Collision output
        var com_hit: Mat44 = undefined;
        var point1: Vec3 = undefined;
        var point2: Vec3 = undefined;
        var fraction: f32 = undefined;

        // Do we start in collision?
        if (penetration_depth > 0.0) {
            // Back face culling?
            if (shape_cast_settings.back_face_mode_convex == .ignore_back_faces and dot > 0.0)
                return;

            // Shallower hit?
            if (penetration_depth <= -collector.getEarlyOutFraction())
                return;

            // We're hitting at fraction 0
            fraction = 0.0;

            // Get contact point
            com_hit = center_of_mass_transform2;
            point1 = center_of_mass_transform2.mulVec3(support_point.sub(normal.mulScalar(convex_radius)));
            point2 = center_of_mass_transform2.mulVec3(support_point.sub(normal.mulScalar(signed_distance)));
        } else if (dot < 0.0) // Moving towards the plane?
        {
            // Calculate hit fraction
            fraction = penetration_depth / dot;
            std.debug.assert(fraction >= 0.0);

            // Further than early out fraction?
            if (fraction >= collector.getEarlyOutFraction())
                return;

            // Get contact point
            com_hit = center_of_mass_transform2.postTranslated(shape_cast.direction.mulScalar(fraction));
            point1 = com_hit.mulVec3(support_point.sub(normal.mulScalar(convex_radius)));
            point2 = point1;
        } else {
            // Moving away from the plane
            return;
        }

        // Create cast result
        const penetration_axis_world = com_hit.multiply3x3(normal.negate());
        const back_facing = dot > 0.0;
        var result = ShapeCastResult.init(fraction, point1, point2, penetration_axis_world, back_facing, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));

        // Gather faces
        if (shape_cast_settings.collect_faces_mode == .collect_faces) {
            // Get supporting face of convex shape
            const shape_to_world = com_hit.mul(shape_cast.center_of_mass_start);
            convex_shape.base.getSupportingFace(.empty, normal_in_convex_shape_space, shape_cast.scale, shape_to_world, &result.base.shape1_face);

            // Get supporting face of plane
            if (!result.base.shape1_face.isEmpty())
                planeGetSupportingFace(convex_shape, shape_to_world.getTranslation(), plane, center_of_mass_transform2, &result.base.shape2_face);
        }

        // Notify the collector
        var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
        if (track_narrowphase_stats) track = .init();
        defer if (track_narrowphase_stats) track.deinit();
        collector.addHit(&result);
    }

    /// Context class for GetTrianglesStart/Next (struct PSGetTrianglesContext)
    const PSGetTrianglesContext = struct {
        vertices: [4]Float3 = undefined,
        done: bool = false,

        comptime {
            std.debug.assert(@sizeOf(PSGetTrianglesContext) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
        }
    };
};

/// Result of planeGetOrthogonalBasis (Jolt's out parameters)
const OrthogonalBasis = struct {
    perp1: Vec3,
    perp2: Vec3,
};

/// sPlaneGetOrthogonalBasis: two vectors that are perpendicular to the normal and to each other
fn planeGetOrthogonalBasis(normal: Vec3) OrthogonalBasis {
    var perp1 = normal.cross(Vec3.axisY()).normalizedOr(Vec3.axisX());
    const perp2 = perp1.cross(normal).normalized();
    perp1 = normal.cross(perp2);
    return .{ .perp1 = perp1, .perp2 = perp2 };
}

/// sGetSupportingFace: this is a version of GetSupportingFace that returns a face that is large enough to cover the
/// shape we're colliding with but not as large as the regular GetSupportedFace to avoid numerical precision issues
fn planeGetSupportingFace(shape: *const ConvexShape, shape_com: Vec3, plane: Plane, plane_to_world: Mat44, out_plane_face: *Shape.SupportingFace) void {
    // Project COM of shape onto plane
    const world_plane = plane.getTransformed(plane_to_world);
    const center = world_plane.projectPointOnPlane(shape_com);

    // Create orthogonal basis for the plane
    const normal = world_plane.getNormal();
    const basis = planeGetOrthogonalBasis(normal);
    var perp1 = basis.perp1;
    var perp2 = basis.perp2;

    // Base the size of the face on the bounding box of the shape, ensuring that it is large enough to cover the entire shape
    const size = shape.base.getLocalBounds().getSize().length();
    perp1 = perp1.mulScalar(size);
    perp2 = perp2.mulScalar(size);

    // Emit the vertices
    out_plane_face.resize(4);
    out_plane_face.at(0).* = center.add(perp1).add(perp2);
    out_plane_face.at(1).* = center.add(perp1).sub(perp2);
    out_plane_face.at(2).* = center.sub(perp1).sub(perp2);
    out_plane_face.at(3).* = center.sub(perp1).add(perp2);
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt has no unit tests for PlaneShape, the bit exact comparison with Jolt is in
// ZoltParity/Physics/PlaneEmptyParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const math = @import("../../../Math/Math.zig");
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const SphereShape = @import("SphereShape.zig").SphereShape;
const BoxShape = @import("BoxShape.zig").BoxShape;

/// The plane y = 1 (solid below)
const test_plane = Plane.init(Vec3.axisY(), -1.0);

fn saveToBuffer(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!ShapeResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return Shape.restoreFromBinaryState(allocator, in.streamIn());
}

test "PlaneShape: settings, Jolt's error text, cached results, material and out of memory" {
    const allocator = testing.allocator;

    // A normal that is not normalized
    for ([_]Vec3{ Vec3.zero(), Vec3.init(0, 2, 0), Vec3.init(1, 1, 0) }) |normal| {
        var settings = PlaneShapeSettings.init(allocator, .init(normal, 0.0), .{});
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Plane normal needs to be normalized!", result.getError());
    }

    // Default constructor (deserialization): Jolt's half extent, an invalid plane
    var default_settings = PlaneShapeSettings.initDefault(allocator);
    defer default_settings.deinit();
    try testing.expectEqual(@as(f32, 1000.0), default_settings.half_extent);
    try testing.expectEqual(PlaneShapeSettings.default_half_extent, default_settings.half_extent);
    try testing.expect(default_settings.material.get() == null);
    var default_result = try default_settings.asShapeSettings().createShape(allocator);
    defer default_result.deinit();
    try testing.expect(default_result.hasError());

    // Heap settings: plane, material, half extent and user data are passed to the shape, the result is cached
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    const settings = try PlaneShapeSettings.create(allocator, test_plane, .{ .material = material.material(), .half_extent = 10.0 });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    settings.asShapeSettings().user_data = 99;
    var result = try settings.createShape(allocator);
    defer result.deinit();
    var result2 = try settings.createShape(allocator);
    defer result2.deinit();
    try testing.expect(result.getPtr() == result2.getPtr());
    const plane = result.getPtr().?.cast(PlaneShape);
    try testing.expect(plane.getPlane().normal_and_constant.eql(test_plane.normal_and_constant));
    try testing.expectEqual(@as(f32, 10.0), plane.getHalfExtent());
    try testing.expect(plane.getPlaneMaterial() == material.material());
    try testing.expect(plane.asShape().getMaterial(.empty) == material.material());
    try testing.expectEqual(@as(u64, 99), plane.asShape().getUserData());
    try testing.expectEqual(ShapeSubType.plane, plane.asShape().getSubType());
    try testing.expect(plane.asShape().getType() == .plane);
    try testing.expectEqual(@as(u32, 2), material.material().getRefCount()); // The settings and the shape

    // Out of memory while creating the shape is returned and not cached, a later call succeeds
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var oom_settings = PlaneShapeSettings.init(allocator, test_plane, .{ .material = material.material() });
        defer oom_settings.deinit();
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var r = oom_settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(oom_settings.base.cached_result.isEmpty());
            continue;
        };
        defer r.deinit();
        try testing.expect(r.isValid());
        try testing.expectEqual(@as(usize, 1), fail_index); // Only the shape is allocated
        break;
    }
}

test "PlaneShape: bounds, vertices, mass properties, volume, stats, material, surface normal, supporting face" {
    const allocator = testing.allocator;

    var plane = PlaneShape.init(allocator, test_plane, .{ .half_extent = 10.0 });
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();
    const shape = plane.asShape();

    // Corners of the plane and the box behind it
    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.init(-10, -9, -10), Vec3.init(10, 1, 10))));
    const transform = Mat44.translation(Vec3.init(1, 2, 3));
    try testing.expect(shape.getWorldSpaceBounds(transform, Vec3.one()).eql(.init(Vec3.init(-9, -7, -7), Vec3.init(11, 3, 13)))); // Shape's version
    try testing.expect(shape.getCenterOfMass().eql(Vec3.zero()));
    try testing.expect(shape.mustBeStatic());
    try testing.expectEqual(@as(f32, 0.0), shape.getInnerRadius());
    try testing.expectEqual(@as(u32, 0), shape.getSubShapeIDBitsRecursive());
    try testing.expect(shape.getMassProperties().eql(&.{}));
    try testing.expectEqual(@as(f32, 0.0), shape.getVolume());
    try testing.expectEqual(@as(usize, @sizeOf(PlaneShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);
    try testing.expect(shape.getMaterial(.empty) == PhysicsMaterial.default);
    try testing.expect(shape.isValidScale(Vec3.init(-1, 2, 3)) and !shape.isValidScale(Vec3.zero())); // Shape's version
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(5, 1, 5)).eql(Vec3.axisY()));

    // Supporting face: the 4 corners in world space (the direction is ignored), the order is kept for an inside out scale
    var face: Shape.SupportingFace = .empty;
    face.append(Vec3.init(9, 9, 9)); // Cleared first
    shape.getSupportingFace(.empty, Vec3.axisX(), Vec3.one(), transform, &face);
    try testing.expectEqual(@as(u32, 4), face.len);
    try testing.expect(face.get(0).eql(Vec3.init(11, 3, 13)));
    try testing.expect(face.get(1).eql(Vec3.init(11, 3, -7)));
    try testing.expect(face.get(2).eql(Vec3.init(-9, 3, -7)));
    try testing.expect(face.get(3).eql(Vec3.init(-9, 3, 13)));
    var flipped: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.axisX(), Vec3.init(-1, 1, 1), transform, &flipped);
    try testing.expect(flipped.eql(&face)); // Mirroring and reversing the corners gives the same face

    // A plane along X: the orthogonal basis uses the cross product with Y
    var plane_x = PlaneShape.init(allocator, .init(Vec3.axisX(), 2.0), .{ .half_extent = 1.0 });
    plane_x.asShape().setEmbedded();
    defer plane_x.asShapeMut().deinit();
    try testing.expect(plane_x.asShape().getLocalBounds().eql(.init(Vec3.init(-3, -1, -1), Vec3.init(-2, 1, 1))));

    // SetMaterial
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    plane.setMaterial(material.material());
    try testing.expect(shape.getMaterial(.empty) == material.material());
    plane.setMaterial(null);
    try testing.expect(shape.getMaterial(.empty) == PhysicsMaterial.default);
}

test "PlaneShape: ray casts, collide point, filters and the collector context" {
    const allocator = testing.allocator;

    var plane = PlaneShape.init(allocator, test_plane, .{});
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();
    const shape = plane.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 1, 3);

    // Single hit: from above, closer hits only
    var hit: RayCastResult = .{};
    try testing.expect(shape.castRay(.init(Vec3.init(0, 5, 0), Vec3.init(0, -10, 0)), creator, &hit));
    try testing.expectEqual(@as(f32, 0.4), hit.fraction);
    try testing.expect(hit.sub_shape_id2.eql(creator.getID()));
    try testing.expect(!shape.castRay(.init(Vec3.init(0, 6, 0), Vec3.init(0, -10, 0)), creator, &hit)); // 0.5 is not closer
    try testing.expect(!shape.castRay(.init(Vec3.init(0, 5, 0), Vec3.init(1, 0, 0)), creator, &hit)); // Parallel
    try testing.expect(!shape.castRay(.init(Vec3.init(0, 5, 0), Vec3.init(0, 10, 0)), creator, &hit)); // Pointing away
    try testing.expect(shape.castRay(.init(Vec3.init(0, 1, 0), Vec3.init(1, 0, 0)), creator, &hit)); // Starts on the surface
    try testing.expectEqual(@as(f32, 0.0), hit.fraction);

    // Collector: a ray from inside hits at 0 (solid) and the back face (only with CollideWithBackFaces)
    var settings: RayCastSettings = .{};
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(12), .{});
    hits.base.setContext(&context);
    const inside_ray = RayCast.init(Vec3.init(0, -1, 0), Vec3.init(0, 10, 0));
    shape.castRayCollector(inside_ray, &settings, creator, &hits.base, &.{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.0), hits.hits.items[0].fraction);
    try testing.expect(hits.hits.items[0].body_id.eql(.init(12)) and hits.hits.items[0].sub_shape_id2.eql(creator.getID()));
    hits.reset();
    settings.setBackFaceMode(.collide_with_back_faces);
    shape.castRayCollector(inside_ray, &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.2), hits.hits.items[1].fraction);

    // Not solid: only the back face
    hits.reset();
    settings.treat_convex_as_solid = false;
    shape.castRayCollector(inside_ray, &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.2), hits.hits.items[0].fraction);

    // Front face from above, early out fraction, parallel ray
    hits.reset();
    settings.treat_convex_as_solid = true;
    shape.castRayCollector(.init(Vec3.init(0, 5, 0), Vec3.init(0, -10, 0)), &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.4), hits.hits.items[0].fraction);
    hits.reset();
    hits.base.updateEarlyOutFraction(0.3);
    shape.castRayCollector(.init(Vec3.init(0, 5, 0), Vec3.init(0, -10, 0)), &settings, creator, &hits.base, &.{});
    shape.castRayCollector(.init(Vec3.init(0, 5, 0), Vec3.init(1, 0, 0)), &settings, creator, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);

    // A collector with early out fraction 0 does not accept the hit at fraction 0
    var closest = ClosestHitCollisionCollector(CastRayCollector).init();
    defer closest.deinit();
    closest.base.updateEarlyOutFraction(0.0);
    shape.castRayCollector(inside_ray, &settings, creator, &closest.base, &.{});
    try testing.expect(!closest.hadHit());

    // A filter that rejects everything
    const RejectAll = struct {
        pub const overrides = .{.shouldCollide};
        base: ShapeFilter = .init(@This()),
        pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, shape2, sub_shape_id_of_shape2 };
            return false;
        }
    };
    const reject_all: RejectAll = .{};
    hits.reset();
    shape.castRayCollector(inside_ray, &settings, creator, &hits.base, &reject_all.base);
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);

    // Collide point: strictly below the surface
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    points.base.setContext(&context);
    shape.collidePoint(Vec3.init(100, 0.5, -100), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(0, 1, 0), creator, &points.base, &.{}); // On the surface
    shape.collidePoint(Vec3.init(0, 2, 0), creator, &points.base, &.{}); // Outside
    shape.collidePoint(Vec3.init(0, 0, 0), creator, &points.base, &reject_all.base); // Filtered
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    try testing.expect(points.hits.items[0].body_id.eql(.init(12)) and points.hits.items[0].sub_shape_id2.eql(creator.getID()));
}

test "PlaneShape: CollideSoftBodyVertices" {
    const allocator = testing.allocator;

    var plane = PlaneShape.init(allocator, test_plane, .{});
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();

    // The plane y = 1 moved up by 2: y = 3 in world space
    var positions = [_]Vec3{ Vec3.init(1, 2.5, 3), Vec3.init(1, 4, 3), Vec3.init(1, 6, 3), Vec3.init(3, 2, 3) };
    var inv_masses = [_]f32{ 1, 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 4;
    var penetrations: [4]f32 = .{ -math.flt_max, -math.flt_max, 0.5, -math.flt_max };
    var indices = [_]i32{-1} ** 4;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    plane.asShape().collideSoftBodyVertices(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.one(), &vertices, 4, 7);

    // Inside: penetration 0.5, the world space plane
    try testing.expectEqual(@as(f32, 0.5), penetrations[0]);
    try testing.expectEqual(@as(i32, 7), indices[0]);
    try testing.expect(planes[0].normal_and_constant.eql(Plane.init(Vec3.axisY(), -3.0).normal_and_constant));
    // Above the plane: a negative penetration that is larger than the stored one
    try testing.expectEqual(@as(f32, -1.0), penetrations[1]);
    try testing.expectEqual(@as(i32, 7), indices[1]);
    // A larger penetration already stored: untouched
    try testing.expectEqual(@as(f32, 0.5), penetrations[2]);
    try testing.expectEqual(@as(i32, -1), indices[2]);
    // Infinite mass: skipped
    try testing.expectEqual(-math.flt_max, penetrations[3]);
    try testing.expectEqual(@as(i32, -1), indices[3]);

    // Scaled: the plane is scaled first (y = 1 scaled by 2 is y = 2, moved up by 2)
    penetrations[0] = -math.flt_max;
    plane.asShape().collideSoftBodyVertices(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.init(1, 2, 1), &vertices, 1, 8);
    try testing.expectEqual(@as(f32, 1.5), penetrations[0]);
    try testing.expectEqual(@as(i32, 8), indices[0]);
}

test "PlaneShape: GetTrianglesStart / Next" {
    const allocator = testing.allocator;

    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    var plane = PlaneShape.init(allocator, test_plane, .{ .material = material.material(), .half_extent = 10.0 });
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();

    // Two triangles once, then 0; an inside out scale (mirror in X) gives the same triangles with the same winding
    for ([_]Vec3{ Vec3.one(), Vec3.init(-1, 1, 1) }) |scale| {
        var context: Shape.GetTrianglesContext = .{};
        plane.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.init(1, 2, 3), Quat.identity(), scale);
        var vertices: [3 * 32]Float3 = undefined;
        var materials: [32]*const PhysicsMaterial = undefined;
        try testing.expectEqual(@as(u32, 2), plane.asShape().getTrianglesNext(&context, 32, &vertices, &materials));
        const expected = [_]Vec3{ Vec3.init(11, 3, 13), Vec3.init(11, 3, -7), Vec3.init(-9, 3, -7), Vec3.init(11, 3, 13), Vec3.init(-9, 3, -7), Vec3.init(-9, 3, 13) };
        for (expected, vertices[0..6]) |e, v| try testing.expect(Vec3.fromFloat3(v).eql(e));
        try testing.expect(materials[0] == material.material() and materials[1] == material.material());
        try testing.expectEqual(@as(u32, 0), plane.asShape().getTrianglesNext(&context, 32, &vertices, null));
    }

    // Without materials
    var context: Shape.GetTrianglesContext = .{};
    plane.asShape().getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.one());
    var vertices: [3 * 32]Float3 = undefined;
    try testing.expectEqual(@as(u32, 2), plane.asShape().getTrianglesNext(&context, 32, &vertices, null));
    try testing.expectApproxEqAbs(@as(f32, -1.0), vertices[0].x, 1.0e-5); // The surface y = 1 rotated to x = -1
}

test "PlaneShape: GetSubmergedVolume is not supported (release builds return zeros)" {
    if (Core.enable_asserts) return error.SkipZigTest; // Asserts "Not supported" like Jolt

    const allocator = testing.allocator;
    var plane = PlaneShape.init(allocator, test_plane, .{});
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();
    const r = plane.asShape().getSubmergedVolume(Mat44.identity(), Vec3.one(), test_plane);
    try testing.expectEqual(@as(f32, 0.0), r.total_volume);
    try testing.expectEqual(@as(f32, 0.0), r.submerged_volume);
    try testing.expect(r.center_of_buoyancy.eql(Vec3.zero()));
}

test "PlaneShape: collide and cast convex shapes against the plane (CollisionDispatch, reversed functions, faces)" {
    const allocator = testing.allocator;

    var plane = PlaneShape.init(allocator, test_plane, .{ .half_extent = 10.0 });
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();
    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    const creator1 = SubShapeIDCreator.pushID(.{}, 1, 2);
    const creator2 = SubShapeIDCreator.pushID(.{}, 2, 3);
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(5), .{});

    // Sphere 0.5 into the plane
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        collector.base.setContext(&context);
        var settings: CollideShapeSettings = .{};
        settings.collect_faces_mode = .collect_faces;
        CollisionDispatch.collideShapeVsShape(sphere.asShape(), plane.asShape(), Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0, 1.5, 0)), Mat44.identity(), creator1, creator2, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try testing.expectEqual(@as(f32, 0.5), hit.penetration_depth);
        try testing.expect(hit.contact_point_on1.eql(Vec3.init(0, 0.5, 0)));
        try testing.expect(hit.contact_point_on2.eql(Vec3.init(0, 1, 0)));
        try testing.expect(hit.penetration_axis.eql(Vec3.init(0, -1, 0)));
        try testing.expect(hit.sub_shape_id1.eql(creator1.getID()) and hit.sub_shape_id2.eql(creator2.getID()));
        try testing.expect(hit.body_id2.eql(.init(5)));
        try testing.expectEqual(@as(u32, 0), hit.shape1_face.len + hit.shape2_face.len); // A sphere has no face: no plane face either
    }

    // Plane vs sphere (reversed): the contact points and the axis are swapped
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(plane.asShape(), sphere.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(0, 1.5, 0)), creator2, creator1, &.{}, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try testing.expectEqual(@as(f32, 0.5), hit.penetration_depth);
        try testing.expect(hit.contact_point_on1.eql(Vec3.init(0, 1, 0)));
        try testing.expect(hit.contact_point_on2.eql(Vec3.init(0, 0.5, 0)));
        try testing.expect(hit.penetration_axis.eql(Vec3.init(0, 1, 0)));
        try testing.expect(hit.sub_shape_id1.eql(creator2.getID()) and hit.sub_shape_id2.eql(creator1.getID()));
    }

    // A box with faces: the face of the plane is centered below the box, sized by its bounds
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        var settings: CollideShapeSettings = .{};
        settings.collect_faces_mode = .collect_faces;
        CollisionDispatch.collideShapeVsShape(box.asShape(), plane.asShape(), Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(3, 1.75, 0)), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try testing.expectEqual(@as(f32, 0.25), hit.penetration_depth);
        try testing.expectEqual(@as(u32, 4), hit.shape1_face.len);
        for (hit.shape1_face.constSlice()) |v| try testing.expectEqual(@as(f32, 0.75), v.getY());
        try testing.expectEqual(@as(u32, 4), hit.shape2_face.len);
        const size = 2.0 * @sqrt(@as(f32, 3.0));
        for (hit.shape2_face.constSlice()) |v| {
            try testing.expectEqual(@as(f32, 1.0), v.getY());
            try testing.expectApproxEqAbs(size, @abs(v.getX() - 3.0), 1.0e-5);
            try testing.expectApproxEqAbs(size, @abs(v.getZ()), 1.0e-5);
        }
    }

    // Separated by 0.5: only with a max separation distance (negative penetration)
    for ([_]f32{ 0.0, 0.6 }) |max_separation| {
        var settings: CollideShapeSettings = .{};
        settings.max_separation_distance = max_separation;
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(sphere.asShape(), plane.asShape(), Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0, 2.5, 0)), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
        try collector.checkError();
        try testing.expectEqual(@as(usize, if (max_separation > 0.0) 1 else 0), collector.hits.items.len);
        if (max_separation > 0.0) try testing.expectEqual(@as(f32, -0.5), collector.hits.items[0].penetration_depth);
    }

    // Cast a sphere down onto the plane: hits at fraction 0.3
    {
        var settings: ShapeCastSettings = .{};
        settings.collect_faces_mode = .collect_faces;
        const cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(0, 5, 0)), Vec3.init(0, -10, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        collector.base.setContext(&context);
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &settings, plane.asShape(), Vec3.one(), &.{}, Mat44.identity(), creator1, creator2, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try testing.expectApproxEqAbs(@as(f32, 0.3), hit.fraction, 1.0e-6);
        try testing.expect(hit.base.contact_point_on1.isClose(Vec3.init(0, 1, 0), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(hit.base.contact_point_on1.eql(hit.base.contact_point_on2));
        try testing.expect(hit.base.penetration_axis.eql(Vec3.init(0, -1, 0)));
        try testing.expect(!hit.is_back_face_hit);
        try testing.expect(hit.base.body_id2.eql(.init(5)) and hit.base.sub_shape_id2.eql(creator2.getID()));
    }

    // Moving away from the plane: no hit
    {
        const cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(0, 5, 0)), Vec3.init(0, 10, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &.{}, plane.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);
    }

    // Starting inside and moving up (back facing): only with CollideWithBackFaces, at fraction 0; the early out rejects
    // a hit that is less deep
    for ([_]bool{ false, true }) |back_faces| {
        var settings: ShapeCastSettings = .{};
        if (back_faces) settings.back_face_mode_convex = .collide_with_back_faces;
        settings.collect_faces_mode = .collect_faces;
        const cast = ShapeCast.init(box.asShape(), Vec3.one(), Mat44.identity(), Vec3.init(0, 10, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &settings, plane.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, if (back_faces) 1 else 0), collector.hits.items.len);
        if (back_faces) {
            const hit = &collector.hits.items[0];
            try testing.expectEqual(@as(f32, 0.0), hit.fraction);
            try testing.expect(hit.is_back_face_hit);
            try testing.expectEqual(@as(f32, 2.0), hit.base.penetration_depth);
            try testing.expectEqual(@as(u32, 4), hit.base.shape1_face.len);
            try testing.expectEqual(@as(u32, 4), hit.base.shape2_face.len);

            var shallow = ClosestHitCollisionCollector(CastShapeCollector).init();
            defer shallow.deinit();
            shallow.base.updateEarlyOutFraction(-3.0);
            CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &settings, plane.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &shallow.base);
            try testing.expect(!shallow.hadHit());
        }
    }

    // Casting the plane against a sphere (reversed cast): the plane moves up 3 to touch the sphere
    {
        const cast = ShapeCast.init(plane.asShape(), Vec3.one(), Mat44.identity(), Vec3.init(0, 10, 0));
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &.{}, sphere.asShape(), Vec3.one(), &.{}, Mat44.translation(Vec3.init(0, 5, 0)), .{}, .{}, &collector.base);
        try collector.checkError();
        try testing.expectEqual(@as(usize, 1), collector.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.3), collector.hits.items[0].fraction, 1.0e-6);
    }
}

test "PlaneShape: binary state, material state, restoreFromBinaryState, saveWithChildren and the registration" {
    const allocator = testing.allocator;

    const material = try PhysicsMaterialSimple.create(allocator, "Stone", Color.grey);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    const normal = Vec3.init(1, 2, 3).normalized();
    var plane = PlaneShape.init(allocator, .init(normal, 0.5), .{ .material = material.material(), .half_extent = 7.0 });
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();
    plane.asShapeMut().setUserData(5);

    // Sub type, user data, plane, half extent
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(plane.asShape(), &buffer);
    try testing.expectEqual(@as(usize, 1 + 8 + 16 + 4), bytes.len);
    var result = try restoreFromBuffer(allocator, bytes);
    defer result.deinit();
    const restored = result.getPtr().?.cast(PlaneShape);
    try testing.expect(restored.getPlane().normal_and_constant.eql(plane.getPlane().normal_and_constant));
    try testing.expectEqual(@as(f32, 7.0), restored.getHalfExtent());
    try testing.expect(restored.asShape().getLocalBounds().eql(plane.asShape().getLocalBounds())); // Recalculated
    try testing.expectEqual(@as(u64, 5), restored.asShape().getUserData());
    try testing.expect(restored.getPlaneMaterial() == PhysicsMaterial.default); // Until the material state is restored

    // Material state: SaveMaterialState replaces the list contents with the material
    var materials: PhysicsMaterialList = .empty;
    defer {
        for (materials.items) |*m| m.deinit();
        materials.deinit(allocator);
    }
    try materials.append(allocator, .init(PhysicsMaterial.default));
    try materials.append(allocator, .init(PhysicsMaterial.default));
    try plane.asShape().saveMaterialState(allocator, &materials);
    try testing.expectEqual(@as(usize, 1), materials.items.len);
    try testing.expect(materials.items[0].get() == material.material());
    try result.getPtr().?.restoreMaterialState(materials.items);
    try testing.expect(restored.getPlaneMaterial() == material.material());

    // Without a material the list holds a null reference
    var no_material = PlaneShape.init(allocator, test_plane, .{});
    no_material.asShape().setEmbedded();
    defer no_material.asShapeMut().deinit();
    try no_material.asShape().saveMaterialState(allocator, &materials);
    try testing.expectEqual(@as(usize, 1), materials.items.len);
    try testing.expect(materials.items[0].get() == null);

    // saveWithChildren / restoreWithChildren restore the material
    var graph_buffer: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&graph_buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    var shape_map: Shape.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: Shape.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    try plane.asShape().saveWithChildren(allocator, out.streamOut(), &shape_map, &material_map);
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var id_to_shape: Shape.IDToShapeMap = .empty;
    defer {
        for (id_to_shape.items) |*s| s.deinit();
        id_to_shape.deinit(allocator);
    }
    var id_to_material: Shape.IDToMaterialMap = .empty;
    defer {
        for (id_to_material.items) |*m| m.deinit();
        id_to_material.deinit(allocator);
    }
    var graph = try Shape.restoreWithChildren(allocator, in.streamIn(), &id_to_shape, &id_to_material);
    defer graph.deinit();
    const graph_plane = graph.getPtr().?.cast(PlaneShape);
    try testing.expectEqualStrings("Stone", graph_plane.getPlaneMaterial().getDebugName());
    try testing.expectEqual(@as(f32, 7.0), graph_plane.getHalfExtent());

    // ShapeFunctions and the dispatch tables (sRegister)
    const functions = ShapeFunctions.get(.plane);
    try testing.expect(functions.construct != null);
    try testing.expect(functions.color.eql(Color.dark_red));
    const registry = &RegisterTypes.registry;
    for (ShapeFile.convex_sub_shape_types) |s| {
        try testing.expect(registry.getCollideShape(s, .plane) == &PlaneShape.collideConvexVsPlane);
        try testing.expect(registry.getCastShape(s, .plane) == &PlaneShape.castConvexVsPlane);
        try testing.expect(registry.getCollideShape(.plane, s) == &CollisionDispatch.reversedCollideShape);
        try testing.expect(registry.getCastShape(.plane, s) == &CollisionDispatch.reversedCastShape);
    }
    try testing.expect(registry.getCollideShape(.plane, .plane) == &CollisionDispatch.collideUnsupported);
}

test "PlaneShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, PlaneShape.create(failing.allocator(), test_plane, .{}));
    try testing.expectError(error.OutOfMemory, PlaneShapeSettings.create(failing.allocator(), test_plane, .{}));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.plane).construct.?(failing.allocator()));

    // A heap shape released through a reference
    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    var plane_ref = RefConst(Shape).init((try PlaneShape.create(allocator, test_plane, .{ .material = material.material() })).asShape());
    try testing.expectEqual(@as(u32, 2), material.material().getRefCount());
    plane_ref.deinit();
    try testing.expectEqual(@as(u32, 1), material.material().getRefCount());

    // Restore: the shape is the only allocation
    var plane = PlaneShape.init(allocator, test_plane, .{ .material = material.material() });
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(plane.asShape(), &buffer);
    try testing.expectError(error.OutOfMemory, restoreFromBuffer(failing.allocator(), bytes));

    // Save material state
    var materials: PhysicsMaterialList = .empty;
    try testing.expectError(error.OutOfMemory, plane.asShape().saveMaterialState(failing.allocator(), &materials));
    try testing.expectEqual(@as(usize, 0), materials.items.len);

    // saveWithChildren / restoreWithChildren at every allocation
    var graph_buffer: [256]u8 = undefined;
    var graph_bytes: []const u8 = &.{};
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var save_failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const a = save_failing.allocator();
        var writer: std.Io.Writer = .fixed(&graph_buffer);
        var out = StreamWrapper.StreamOutWrapper.init(&writer);
        var shape_map: Shape.ShapeToIDMap = .empty;
        defer shape_map.deinit(a);
        var material_map: Shape.MaterialToIDMap = .empty;
        defer material_map.deinit(a);
        plane.asShape().saveWithChildren(a, out.streamOut(), &shape_map, &material_map) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        graph_bytes = writer.buffered();
        break;
    }
    try testing.expect(fail_index > 1);
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var restore_failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const a = restore_failing.allocator();
        var reader: std.Io.Reader = .fixed(graph_bytes);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var shapes: Shape.IDToShapeMap = .empty;
        defer {
            for (shapes.items) |*s| s.deinit();
            shapes.deinit(a);
        }
        var materials_map: Shape.IDToMaterialMap = .empty;
        defer {
            for (materials_map.items) |*m| m.deinit();
            materials_map.deinit(a);
        }
        var r = Shape.restoreWithChildren(a, in.streamIn(), &shapes, &materials_map) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        defer r.deinit();
        try testing.expect(r.isValid() and !restore_failing.has_induced_failure);
        try testing.expectEqualStrings("Mat", r.getPtr().?.cast(PlaneShape).getPlaneMaterial().getDebugName());
        break;
    }
    try testing.expect(fail_index > 3);
}
