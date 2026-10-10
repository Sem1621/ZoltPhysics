//! Port of: Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h, Jolt/Physics/Collision/Shape/TaperedCapsuleShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2, SphereShape.zig is the reference):
//! - `overrides` lists every C++ `override` in header order. CastRay (both overloads), CollidePoint, GetTrianglesStart /
//!   Next and GetSubmergedVolume are ConvexShape's (GJK based) versions, like in Jolt.
//! - `TaperedCapsuleShapeSettings.createShape` has Jolt's custom Create logic: when one sphere contains the other it
//!   returns a SphereShape (built with `SphereShape(radius, material)`, so with the default density and user data), offset
//!   with a RotatedTranslatedShape when its center is not at the origin.
//! - The support class `TaperedCapsule` is constructed in the caller's SupportBuffer (D9).
//! - JPH_DEBUG_RENDERER (Draw and the `mutable mGeometry` it caches) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Color = @import("../../../Core/Color.zig").Color;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const math = @import("../../../Math/Math.zig");
const trigonometry = @import("../../../Math/Trigonometry.zig");
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
const ConvexShapeFile = @import("ConvexShape.zig");
const ConvexShape = ConvexShapeFile.ConvexShape;
const ConvexShapeSettings = ConvexShapeFile.ConvexShapeSettings;
const RotatedTranslatedShapeSettings = @import("RotatedTranslatedShape.zig").RotatedTranslatedShapeSettings;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SphereShape = @import("SphereShape.zig").SphereShape;
const SubShapeID = @import("SubShapeID.zig").SubShapeID;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Class that constructs a TaperedCapsuleShape
pub const TaperedCapsuleShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, TaperedCapsuleShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    half_height_of_tapered_cylinder: f32 = 0.0,
    top_radius: f32 = 0.0,
    bottom_radius: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) TaperedCapsuleShapeSettings {
        return .{ .base = .init(TaperedCapsuleShapeSettings, allocator, null) };
    }

    /// Create a tapered capsule centered around the origin with one sphere cap at (0, -half_height_of_tapered_cylinder, 0) with radius bottom_radius and the other at (0, half_height_of_tapered_cylinder, 0) with radius top_radius
    /// (settings on the stack: `defer settings.deinit()`)
    pub fn init(allocator: Allocator, half_height_of_tapered_cylinder: f32, top_radius: f32, bottom_radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) TaperedCapsuleShapeSettings {
        return .{
            .base = .init(TaperedCapsuleShapeSettings, allocator, opts.material),
            .half_height_of_tapered_cylinder = half_height_of_tapered_cylinder,
            .top_radius = top_radius,
            .bottom_radius = bottom_radius,
        };
    }

    /// new TaperedCapsuleShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, half_height_of_tapered_cylinder: f32, top_radius: f32, bottom_radius: f32, opts: struct { material: ?*const PhysicsMaterial = null }) Allocator.Error!*TaperedCapsuleShapeSettings {
        const self = try allocator.create(TaperedCapsuleShapeSettings);
        self.* = .init(allocator, half_height_of_tapered_cylinder, top_radius, bottom_radius, .{ .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *TaperedCapsuleShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *TaperedCapsuleShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    /// Check if the settings are valid
    pub fn isValid(self: *const TaperedCapsuleShapeSettings) bool {
        return self.top_radius > 0.0 and self.bottom_radius > 0.0 and self.half_height_of_tapered_cylinder >= 0.0;
    }

    /// Checks if the settings of this tapered capsule make this shape a sphere
    pub fn isSphere(self: *const TaperedCapsuleShapeSettings) bool {
        return math.max(self.top_radius, self.bottom_radius) >= 2.0 * self.half_height_of_tapered_cylinder + math.min(self.top_radius, self.bottom_radius);
    }

    /// Create a shape according to the settings specified by this object.
    /// Note that when one sphere fully contains the other sphere, this will return a RotatedTranslatedShape with a SphereShape, or a SphereShape if half_height_of_tapered_cylinder is 0.
    pub fn createShape(self: *TaperedCapsuleShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        const cached_result = &self.base.base.cached_result;
        if (cached_result.isEmpty()) {
            if (self.isValid() and self.isSphere()) {
                // Determine sphere center and radius
                var radius: f32 = undefined;
                var center: f32 = undefined;
                if (self.top_radius > self.bottom_radius) {
                    radius = self.top_radius;
                    center = self.half_height_of_tapered_cylinder;
                } else {
                    radius = self.bottom_radius;
                    center = -self.half_height_of_tapered_cylinder;
                }

                // Create sphere
                var shape = Ref(Shape).init((try SphereShape.create(allocator, radius, .{ .material = self.base.material.get() })).asShapeMut());
                defer shape.deinit();

                // Offset sphere if needed
                if (@abs(center) > 1.0e-6) {
                    var rot_trans = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(0, center, 0), Quat.identity(), shape.get());
                    defer rot_trans.deinit();
                    cached_result.assignMove(try rot_trans.createShape(allocator));
                } else cached_result.set(shape.clone());
            } else {
                // Normal tapered capsule shape
                try ShapeSettings.constructShape(TaperedCapsuleShape, self, allocator);
            }
        }
        return cached_result.clone();
    }
};

/// A capsule with different top and bottom radii
pub const TaperedCapsuleShape = struct {
    /// Concrete class: `Shape.cast(TaperedCapsuleShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .tapered_capsule;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .collideSoftBodyVertices, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: ConvexShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    center_of_mass: Vec3 = Vec3.zero(),
    top_radius: f32 = 0.0,
    bottom_radius: f32 = 0.0,
    top_center: f32 = 0.0,
    bottom_center: f32 = 0.0,
    convex_radius: f32 = 0.0,
    sin_alpha: f32 = 0.0,
    tan_alpha: f32 = 0.0,

    // TODO(debug_renderer): mutable DebugRenderer::GeometryRef mGeometry (JPH_DEBUG_RENDERER, a cache written by Draw:
    // needs a Rule M solution when the debug renderer is ported)

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// TaperedCapsuleShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by constructShape
    pub fn initDefault(allocator: Allocator) TaperedCapsuleShape {
        return .{ .base = .init(TaperedCapsuleShape, allocator, shape_sub_type, null) };
    }

    /// TaperedCapsuleShape(const TaperedCapsuleShapeSettings &inSettings, ShapeResult &outResult): C++ member initializers first, then the body
    pub fn initFromSettings(self: *TaperedCapsuleShape, settings: *const TaperedCapsuleShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.top_radius = settings.top_radius;
        self.bottom_radius = settings.bottom_radius;

        if (self.top_radius <= 0.0) {
            result.setError("Invalid top radius");
            return;
        }

        if (self.bottom_radius <= 0.0) {
            result.setError("Invalid bottom radius");
            return;
        }

        if (settings.half_height_of_tapered_cylinder <= 0.0) {
            result.setError("Invalid height");
            return;
        }

        // If this goes off one of the sphere ends falls totally inside the other and you should use a sphere instead
        if (settings.isSphere()) {
            result.setError("One sphere embedded in other sphere, please use sphere shape instead");
            return;
        }

        // Approximation: The center of mass is exactly half way between the top and bottom cap of the tapered capsule
        self.top_center = settings.half_height_of_tapered_cylinder + 0.5 * (self.bottom_radius - self.top_radius);
        self.bottom_center = -settings.half_height_of_tapered_cylinder + 0.5 * (self.bottom_radius - self.top_radius);

        // Calculate center of mass
        self.center_of_mass = Vec3.init(0, settings.half_height_of_tapered_cylinder - self.top_center, 0);

        // Calculate convex radius
        self.convex_radius = math.min(self.top_radius, self.bottom_radius);
        std.debug.assert(self.convex_radius > 0.0);

        // Calculate the sin and tan of the angle that the cone surface makes with the Y axis
        // See: TaperedCapsuleShape.gliffy
        self.sin_alpha = (self.bottom_radius - self.top_radius) / (self.top_center - self.bottom_center);
        if (Core.enable_asserts) std.debug.assert(self.sin_alpha >= -1.0 and self.sin_alpha <= 1.0); // Rounding can violate this for nearly embedded spheres (or NaN settings), Jolt's release build continues (ASin clamps)
        self.tan_alpha = trigonometry.tan(trigonometry.asin(self.sin_alpha));

        result.set(.init(self.asShapeMut()));
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const TaperedCapsuleShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *TaperedCapsuleShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get top radius of the tapered capsule
    pub fn getTopRadius(self: *const TaperedCapsuleShape) f32 {
        return self.top_radius;
    }

    /// Get bottom radius of the tapered capsule
    pub fn getBottomRadius(self: *const TaperedCapsuleShape) f32 {
        return self.bottom_radius;
    }

    /// Get half height between the top and bottom sphere center
    pub fn getHalfHeight(self: *const TaperedCapsuleShape) f32 {
        return 0.5 * (self.top_center - self.bottom_center);
    }

    /// Returns box that approximates the inertia
    fn getInertiaApproximation(self: *const TaperedCapsuleShape) AABox {
        // TODO: For now the mass and inertia is that of a box
        const avg_radius = 0.5 * (self.top_radius + self.bottom_radius);
        return .init(Vec3.init(-avg_radius, self.bottom_center - self.bottom_radius, -avg_radius), Vec3.init(avg_radius, self.top_center + self.top_radius, avg_radius));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const TaperedCapsuleShape) Vec3 {
        return self.center_of_mass;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const TaperedCapsuleShape) AABox {
        const max_radius = math.max(self.top_radius, self.bottom_radius);
        return .init(Vec3.init(-max_radius, self.bottom_center - self.bottom_radius, -max_radius), Vec3.init(max_radius, self.top_center + self.top_radius, max_radius));
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const TaperedCapsuleShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        const abs_scale = scale.abs();
        const scale_xz = abs_scale.getX();
        const scale_y = scale.getY(); // The sign of y is important as it flips the tapered capsule
        const bottom_extent = Vec3.replicate(scale_xz * self.bottom_radius);
        const bottom_center = center_of_mass_transform.mulVec3(Vec3.init(0, scale_y * self.bottom_center, 0));
        const top_extent = Vec3.replicate(scale_xz * self.top_radius);
        const top_center = center_of_mass_transform.mulVec3(Vec3.init(0, scale_y * self.top_center, 0));
        const p1 = Vec3.min(top_center.sub(top_extent), bottom_center.sub(bottom_extent));
        const p2 = Vec3.max(top_center.add(top_extent), bottom_center.add(bottom_extent));
        return .init(p1, p2);
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const TaperedCapsuleShape) f32 {
        return math.min(self.top_radius, self.bottom_radius);
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const TaperedCapsuleShape) MassProperties {
        const box = self.getInertiaApproximation();

        var p: MassProperties = .{};
        p.setMassAndInertiaOfSolidBox(box.getSize(), self.base.getDensity());
        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const TaperedCapsuleShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        // See: TaperedCapsuleShape.gliffy
        // We need to calculate ty and by in order to see if the position is on the top or bottom sphere
        // sin(alpha) = by / br = ty / tr
        // => by = sin(alpha) * br, ty = sin(alpha) * tr

        if (local_surface_position.getY() > self.top_center + self.sin_alpha * self.top_radius)
            return local_surface_position.sub(Vec3.init(0, self.top_center, 0)).normalized()
        else if (local_surface_position.getY() < self.bottom_center + self.sin_alpha * self.bottom_radius)
            return local_surface_position.sub(Vec3.init(0, self.bottom_center, 0)).normalized()
        else {
            // Get perpendicular vector to the surface in the xz plane
            var perpendicular = Vec3.init(local_surface_position.getX(), 0, local_surface_position.getZ()).normalizedOr(Vec3.axisX());

            // We know that the perpendicular has length 1 and that it needs a y component where tan(alpha) = y / 1 in order to align it to the surface
            perpendicular.setY(self.tan_alpha);
            return perpendicular.normalized();
        }
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const TaperedCapsuleShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
        std.debug.assert(self.isValidScale(scale));

        // Check zero vector
        const len = direction.length();
        if (len == 0.0)
            return;

        // Get scaled tapered capsule
        const abs_scale = scale.abs();
        const scale_xz = abs_scale.getX();
        const scale_y = scale.getY(); // The sign of y is important as it flips the tapered capsule
        const scaled_top_center = Vec3.init(0, scale_y * self.top_center, 0);
        const scaled_bottom_center = Vec3.init(0, scale_y * self.bottom_center, 0);
        const scaled_top_radius = scale_xz * self.top_radius;
        const scaled_bottom_radius = scale_xz * self.bottom_radius;

        // Get support point for top and bottom sphere in the opposite of direction (including convex radius)
        const support_top = scaled_top_center.sub(direction.mulScalar(scaled_top_radius / len));
        const support_bottom = scaled_bottom_center.sub(direction.mulScalar(scaled_bottom_radius / len));

        // Get projection on direction
        const proj_top = support_top.dot(direction);
        const proj_bottom = support_bottom.dot(direction);

        // If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
        if (@abs(proj_top - proj_bottom) < PhysicsSettings.capsule_projection_slop * len) {
            out_vertices.append(center_of_mass_transform.mulVec3(support_top));
            out_vertices.append(center_of_mass_transform.mulVec3(support_bottom));
        }
    }

    // See ConvexShape::GetSupportFunction: placement new into the caller's buffer
    pub fn getSupportFunction(self: *const TaperedCapsuleShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        std.debug.assert(self.isValidScale(scale));

        // Get scaled tapered capsule
        const abs_scale = scale.abs();
        const scale_xz = abs_scale.getX();
        const scale_y = scale.getY(); // The sign of y is important as it flips the tapered capsule
        const scaled_top_center = Vec3.init(0, scale_y * self.top_center, 0);
        const scaled_bottom_center = Vec3.init(0, scale_y * self.bottom_center, 0);
        const scaled_top_radius = scale_xz * self.top_radius;
        const scaled_bottom_radius = scale_xz * self.bottom_radius;
        const scaled_convex_radius = scale_xz * self.convex_radius;

        switch (mode) {
            .include_convex_radius => {
                const support = buffer.emplace(TaperedCapsule);
                support.* = .init(scaled_top_center, scaled_bottom_center, scaled_top_radius, scaled_bottom_radius, 0.0);
                return &support.base;
            },

            .exclude_convex_radius, .default => {
                // Get radii reduced by convex radius
                const tr = scaled_top_radius - scaled_convex_radius;
                const br = scaled_bottom_radius - scaled_convex_radius;
                std.debug.assert(tr >= 0.0 and br >= 0.0);
                std.debug.assert(tr == 0.0 or br == 0.0); // Convex radius should be that of the smallest sphere
                const support = buffer.emplace(TaperedCapsule);
                support.* = .init(scaled_top_center, scaled_bottom_center, tr, br, scaled_convex_radius);
                return &support.base;
            },
        }
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const TaperedCapsuleShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        std.debug.assert(self.isValidScale(scale));

        const inverse_transform = center_of_mass_transform.inversedRotationTranslation();

        // Get scaled tapered capsule
        const abs_scale = scale.abs();
        const scale_y = abs_scale.getY();
        const scale_xz = abs_scale.getX();
        const scale_y_flip = Vec3.init(1, math.sign(scale.getY()), 1);
        const scaled_top_center = Vec3.init(0, scale_y * self.top_center, 0);
        const scaled_bottom_center = Vec3.init(0, scale_y * self.bottom_center, 0);
        const scaled_top_radius = scale_xz * self.top_radius;
        const scaled_bottom_radius = scale_xz * self.bottom_radius;

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                const local_pos = scale_y_flip.mul(inverse_transform.mulVec3(v.getPosition()));

                var position: Vec3 = undefined;
                var normal: Vec3 = undefined;

                // If the vertex is inside the cone starting at the top center pointing along the y-axis with angle PI/2 - alpha then the closest point is on the top sphere
                // This corresponds to: Dot(y-axis, (local_pos - top_center) / |local_pos - top_center|) >= cos(PI/2 - alpha)
                // <=> (local_pos - top_center).y >= sin(alpha) * |local_pos - top_center|
                const top_center_to_local_pos = local_pos.sub(scaled_top_center);
                const top_center_to_local_pos_len = top_center_to_local_pos.length();
                if (top_center_to_local_pos.getY() >= self.sin_alpha * top_center_to_local_pos_len) {
                    // Top sphere
                    normal = if (top_center_to_local_pos_len != 0.0) top_center_to_local_pos.divScalar(top_center_to_local_pos_len) else Vec3.axisY();
                    position = scaled_top_center.add(normal.mulScalar(scaled_top_radius));
                } else {
                    // If the vertex is outside the cone starting at the bottom center pointing along the y-axis with angle PI/2 - alpha then the closest point is on the bottom sphere
                    // This corresponds to: Dot(y-axis, (local_pos - bottom_center) / |local_pos - bottom_center|) <= cos(PI/2 - alpha)
                    // <=> (local_pos - bottom_center).y <= sin(alpha) * |local_pos - bottom_center|
                    const bottom_center_to_local_pos = local_pos.sub(scaled_bottom_center);
                    const bottom_center_to_local_pos_len = bottom_center_to_local_pos.length();
                    if (bottom_center_to_local_pos.getY() <= self.sin_alpha * bottom_center_to_local_pos_len) {
                        // Bottom sphere
                        normal = if (bottom_center_to_local_pos_len != 0.0) bottom_center_to_local_pos.divScalar(bottom_center_to_local_pos_len) else Vec3.axisY().negate();
                    } else {
                        // Tapered cylinder
                        normal = Vec3.init(local_pos.getX(), 0, local_pos.getZ()).normalizedOr(Vec3.axisX());
                        normal.setY(self.tan_alpha);
                        normal = normal.normalizedOr(Vec3.axisX());
                    }
                    position = scaled_bottom_center.add(normal.mulScalar(scaled_bottom_radius));
                }

                var plane = Plane.fromPointAndNormal(position, normal);
                const penetration = -plane.signedDistance(local_pos);
                if (v.updatePenetration(penetration)) {
                    // Need to flip the normal's y if capsule is flipped (this corresponds to flipping both the point and the normal around y)
                    plane.setNormal(scale_y_flip.mul(plane.getNormal()));

                    // Store collision
                    v.setCollision(plane.getTransformed(center_of_mass_transform), colliding_shape_index);
                }
            }
        }
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::SaveBinaryState: C++ `ConvexShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const TaperedCapsuleShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.center_of_mass);
        stream.write(self.top_radius);
        stream.write(self.bottom_radius);
        stream.write(self.top_center);
        stream.write(self.bottom_center);
        stream.write(self.convex_radius);
        stream.write(self.sin_alpha);
        stream.write(self.tan_alpha);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const TaperedCapsuleShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TaperedCapsuleShape), 0);
    }

    // See Shape::GetVolume (C++ unqualified GetLocalBounds(): a static call is equivalent in a final class)
    pub fn getVolume(self: *const TaperedCapsuleShape) f32 {
        return self.getLocalBounds().getVolume(); // Volume is approximate!
    }

    // See Shape::IsValidScale: C++ `ConvexShape::IsValidScale` resolves to Shape's version (ConvexShape does not override it)
    pub fn isValidScale(self: *const TaperedCapsuleShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScale(scale.abs());
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const TaperedCapsuleShape, scale: Vec3) Vec3 {
        _ = self;
        const s = ScaleHelpers.makeNonZeroScale(scale);

        return s.getSign().mul(ScaleHelpers.makeUniformScale(s.abs()));
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *TaperedCapsuleShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.center_of_mass);
        stream.read(&self.top_radius);
        stream.read(&self.bottom_radius);
        stream.read(&self.top_center);
        stream.read(&self.bottom_center);
        stream.read(&self.convex_radius);
        stream.read(&self.sin_alpha);
        stream.read(&self.tan_alpha);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.tapered_capsule);
        f.construct = ShapeFunctions.constructor(TaperedCapsuleShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Class for GetSupportFunction (`class TaperedCapsule final : public Support`)

    const TaperedCapsule = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        top_center: Vec3,
        bottom_center: Vec3,
        top_radius: f32,
        bottom_radius: f32,
        convex_radius: f32,

        fn init(top_center: Vec3, bottom_center: Vec3, top_radius: f32, bottom_radius: f32, convex_radius: f32) TaperedCapsule {
            return .{ .base = .init(TaperedCapsule), .top_center = top_center, .bottom_center = bottom_center, .top_radius = top_radius, .bottom_radius = bottom_radius, .convex_radius = convex_radius };
        }

        pub fn getSupport(self: *const TaperedCapsule, direction: Vec3) Vec3 {
            // Check zero vector
            const len = direction.length();
            if (len == 0.0)
                return self.top_center.add(Vec3.init(0, self.top_radius, 0)); // Return top

            // Check if the support of the top sphere or bottom sphere is bigger
            const support_top = self.top_center.add(direction.mulScalar(self.top_radius / len));
            const support_bottom = self.bottom_center.add(direction.mulScalar(self.bottom_radius / len));
            if (support_top.dot(direction) > support_bottom.dot(direction))
                return support_top
            else
                return support_bottom;
        }

        pub fn getConvexRadius(self: *const TaperedCapsule) f32 {
            return self.convex_radius;
        }
    };
};
