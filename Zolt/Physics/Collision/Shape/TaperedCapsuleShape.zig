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

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/CapsulesParity.zig)

const testing = std.testing;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const CastRayCollector = ShapeFile.CastRayCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const RayCast = @import("../RayCast.zig").RayCast;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const PhysicsMaterialSimple = @import("../PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RotatedTranslatedShape = @import("RotatedTranslatedShape.zig").RotatedTranslatedShape;

/// Create a shape from tapered capsule settings, the caller releases it
fn createTaperedCapsule(allocator: Allocator, half_height: f32, top_radius: f32, bottom_radius: f32) !Ref(Shape) {
    var settings = TaperedCapsuleShapeSettings.init(allocator, half_height, top_radius, bottom_radius, .{});
    defer settings.deinit();
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    return result.get().clone();
}

test "TaperedCapsuleShape: settings, Jolt's error texts, spheres when one sphere contains the other, out of memory" {
    const allocator = testing.allocator;

    // Invalid settings: the radii are checked before the height, a zero radius at one end is invalid
    const invalid = [_]struct { half_height: f32, top_radius: f32, bottom_radius: f32, err: []const u8 }{
        .{ .half_height = 1.0, .top_radius = 0.0, .bottom_radius = 0.5, .err = "Invalid top radius" },
        .{ .half_height = -1.0, .top_radius = -1.0, .bottom_radius = -1.0, .err = "Invalid top radius" },
        .{ .half_height = 1.0, .top_radius = 0.5, .bottom_radius = 0.0, .err = "Invalid bottom radius" },
        .{ .half_height = -1.0, .top_radius = 0.5, .bottom_radius = -0.5, .err = "Invalid bottom radius" },
        .{ .half_height = -1.0, .top_radius = 0.5, .bottom_radius = 0.5, .err = "Invalid height" },
        .{ .half_height = -1.0, .top_radius = 5.0, .bottom_radius = 0.5, .err = "Invalid height" }, // Not valid, so not a sphere
    };
    for (invalid) |c| {
        var settings = TaperedCapsuleShapeSettings.init(allocator, c.half_height, c.top_radius, c.bottom_radius, .{});
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings(c.err, result.getError());
    }

    // Default constructor (deserialization): no radii
    var default_settings = TaperedCapsuleShapeSettings.initDefault(allocator);
    defer default_settings.deinit();
    try testing.expect(default_settings.half_height_of_tapered_cylinder == 0.0 and default_settings.top_radius == 0.0 and default_settings.bottom_radius == 0.0);
    var default_result = try default_settings.asShapeSettings().createShape(allocator);
    defer default_result.deinit();
    try testing.expectEqualStrings("Invalid top radius", default_result.getError());

    // Create() never constructs a tapered capsule from settings that make a sphere, the constructor reports it
    {
        var settings = TaperedCapsuleShapeSettings.init(allocator, 1.0, 3.0, 1.0, .{});
        defer settings.deinit();
        try ShapeSettings.constructShape(TaperedCapsuleShape, &settings, allocator);
        try testing.expectEqualStrings("One sphere embedded in other sphere, please use sphere shape instead", settings.base.base.cached_result.getError());
    }

    const material = try PhysicsMaterialSimple.create(allocator, "Mat", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material()); // Used by several settings
    defer material_ref.deinit();

    // One sphere contains the other: a sphere (offset with a RotatedTranslatedShape) with the default density and user data
    const spheres = [_]struct { half_height: f32, top_radius: f32, bottom_radius: f32, radius: f32, center: f32 }{
        .{ .half_height = 1.0, .top_radius = 3.0, .bottom_radius = 1.0, .radius = 3.0, .center = 1.0 },
        .{ .half_height = 1.0, .top_radius = 1.0, .bottom_radius = 3.0, .radius = 3.0, .center = -1.0 },
        .{ .half_height = 0.5, .top_radius = 1.0, .bottom_radius = 2.0, .radius = 2.0, .center = -0.5 }, // Touching inside
        .{ .half_height = 0.0, .top_radius = 2.0, .bottom_radius = 2.0, .radius = 2.0, .center = 0.0 }, // Equal radii, no height
        .{ .half_height = 0.0, .top_radius = 2.0, .bottom_radius = 1.0, .radius = 2.0, .center = 0.0 },
        .{ .half_height = 1.0e-7, .top_radius = 2.0, .bottom_radius = 1.0, .radius = 2.0, .center = 0.0 }, // Offset too small
    };
    for (spheres) |c| {
        var settings = TaperedCapsuleShapeSettings.init(allocator, c.half_height, c.top_radius, c.bottom_radius, .{ .material = material.material() });
        defer settings.deinit();
        settings.base.density = 500.0;
        settings.asShapeSettings().user_data = 7;
        try testing.expect(settings.isValid() and settings.isSphere());
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const shape = result.getPtr().?;
        try testing.expectEqual(@as(u64, 0), shape.getUserData());
        const sphere = if (c.center != 0.0) blk: {
            const rotated_translated = shape.cast(RotatedTranslatedShape);
            try testing.expect(rotated_translated.getPosition().eql(Vec3.init(0, c.center, 0)));
            try testing.expect(rotated_translated.getRotation().eql(Quat.identity()));
            try testing.expect(shape.getCenterOfMass().eql(Vec3.init(0, c.center, 0)));
            break :blk rotated_translated.base.inner_shape.get().?.cast(SphereShape);
        } else shape.cast(SphereShape);
        try testing.expectEqual(c.radius, sphere.getRadius());
        try testing.expect(sphere.base.getConvexMaterial() == material.material());
        try testing.expectEqual(@as(f32, 1000.0), sphere.base.getDensity());

        // Cached: the same shape again
        var again = try settings.asShapeSettings().createShape(allocator);
        defer again.deinit();
        try testing.expect(again.getPtr() == result.getPtr());
    }

    // Heap settings: material, density and user data are passed to the tapered capsule
    const settings = try TaperedCapsuleShapeSettings.create(allocator, 2.0, 0.5, 1.0, .{ .material = material.material() });
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    settings.base.setDensity(321.0);
    settings.asShapeSettings().user_data = 99;
    var result = try settings.createShape(allocator);
    defer result.deinit();
    const capsule = result.getPtr().?.cast(TaperedCapsuleShape);
    try testing.expectEqual(@as(f32, 0.5), capsule.getTopRadius());
    try testing.expectEqual(@as(f32, 1.0), capsule.getBottomRadius());
    try testing.expectEqual(@as(f32, 2.0), capsule.getHalfHeight());
    try testing.expect(capsule.asShape().getMaterial(.empty) == material.material());
    try testing.expectEqual(@as(f32, 321.0), capsule.base.getDensity());
    try testing.expectEqual(@as(u64, 99), capsule.asShape().getUserData());
    try testing.expectEqual(ShapeSubType.tapered_capsule, capsule.asShape().getSubType());

    // Out of memory while creating the shape (tapered capsule, sphere or offset sphere) is returned and not cached
    const oom_cases = [_]struct { half_height: f32, top_radius: f32, sub_type: ShapeSubType, allocations: usize }{
        .{ .half_height = 1.0, .top_radius = 0.5, .sub_type = .tapered_capsule, .allocations = 1 },
        .{ .half_height = 0.0, .top_radius = 0.5, .sub_type = .sphere, .allocations = 1 },
        .{ .half_height = 1.0, .top_radius = 5.0, .sub_type = .rotated_translated, .allocations = 2 },
    };
    for (oom_cases) |c| {
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var oom_settings = TaperedCapsuleShapeSettings.init(allocator, c.half_height, c.top_radius, 0.75, .{});
            defer oom_settings.deinit();
            var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var r = oom_settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
                try testing.expectEqual(error.OutOfMemory, err);
                try testing.expect(oom_settings.base.base.cached_result.isEmpty());
                continue;
            };
            defer r.deinit();
            try testing.expect(r.isValid());
            try testing.expectEqual(c.sub_type, r.getPtr().?.getSubType());
            try testing.expectEqual(c.allocations, fail_index);
            break;
        }
    }
}

test "TaperedCapsuleShape: center of mass, bounds, inner radius, mass properties, volume, stats, surface normals, supporting faces" {
    const allocator = testing.allocator;

    var shape_ref = try createTaperedCapsule(allocator, 2.0, 0.5, 1.0);
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;
    const capsule = shape.cast(TaperedCapsuleShape);

    // The sphere centers are shifted so that the center of mass is half way between the top and the bottom
    try testing.expectEqual(@as(f32, 2.25), capsule.top_center);
    try testing.expectEqual(@as(f32, -1.75), capsule.bottom_center);
    try testing.expectEqual(@as(f32, 0.5), capsule.convex_radius);
    try testing.expectEqual(@as(f32, 0.125), capsule.sin_alpha);
    try testing.expectApproxEqAbs(@as(f32, 0.125 / @sqrt(1.0 - 0.125 * 0.125)), capsule.tan_alpha, 1.0e-6);
    try testing.expect(shape.getCenterOfMass().eql(Vec3.init(0, -0.25, 0)));

    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.init(-1, -2.75, -1), Vec3.init(1, 2.75, 1))));
    try testing.expect(shape.getWorldSpaceBounds(Mat44.translation(Vec3.init(1, 2, 3)), Vec3.init(2, -2, 2)).eql(.init(Vec3.init(-1, -3.5, 1), Vec3.init(3, 7.5, 5)))); // Flipped
    try testing.expectEqual(@as(f32, 0.5), shape.getInnerRadius());
    try testing.expectEqual(@as(f32, 22.0), shape.getVolume()); // Approximate: the bounding box
    try testing.expectEqual(@as(usize, @sizeOf(TaperedCapsuleShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);

    // Mass properties: those of a box with the average radius
    const p = shape.getMassProperties();
    try testing.expectEqual(@as(f32, 1.5 * 5.5 * 1.5 * 1000.0), p.mass);
    try testing.expectApproxEqRel(p.mass / 12.0 * (5.5 * 5.5 + 1.5 * 1.5), p.inertia.get(0, 0), 1.0e-6);
    try testing.expectApproxEqRel(p.mass / 12.0 * (1.5 * 1.5 + 1.5 * 1.5), p.inertia.get(1, 1), 1.0e-6);

    // Surface normals: the top sphere, the bottom sphere and the tapered cylinder
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, 3, 0)).eql(Vec3.axisY()));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0, -3, 0)).eql(Vec3.axisY().negate()));
    const side_normal = Vec3.init(1, capsule.tan_alpha, 0).normalized();
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(0.75, 0.5, 0)).eql(side_normal));
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.zero()).eql(side_normal));

    // Supporting face: a line on the cone, only in the direction of its normal
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, side_normal.negate(), Vec3.one(), Mat44.translation(Vec3.init(10, 0, 0)), &face);
    try testing.expectEqual(@as(u32, 2), face.len);
    try testing.expect(face.get(0).isClose(Vec3.init(10, 2.25, 0).add(side_normal.mulScalar(0.5)), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(face.get(1).isClose(Vec3.init(10, -1.75, 0).add(side_normal), .{ .max_dist_sq = 1.0e-10 }));
    for ([_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(0, 1, 0), Vec3.zero() }) |direction| {
        face.clear();
        shape.getSupportingFace(.empty, direction, Vec3.one(), Mat44.identity(), &face);
        try testing.expectEqual(@as(u32, 0), face.len);
    }

    // Equal radii: a capsule
    var equal_ref = try createTaperedCapsule(allocator, 1.0, 0.5, 0.5);
    defer equal_ref.deinit();
    const equal = equal_ref.get().?.cast(TaperedCapsuleShape);
    try testing.expect(equal.sin_alpha == 0.0 and equal.tan_alpha == 0.0 and equal.top_center == 1.0 and equal.bottom_center == -1.0);
    try testing.expect(equal.asShape().getCenterOfMass().eql(Vec3.zero()));
    try testing.expect(equal.asShape().getSurfaceNormal(.empty, Vec3.init(0, 0.5, -0.5)).eql(Vec3.axisZ().negate()));
}

test "TaperedCapsuleShape: valid scales (the tapered capsule part of Jolt's TestIsValidScale)" {
    const allocator = testing.allocator;

    // Constant of TestIsValidScale: Square(1.0e-6f * ScaleHelpers::cMinScale)
    const min_scale_tolerance_sq: f32 = math.square(1.0e-6 * ScaleHelpers.min_scale);

    var tapered_capsule_ref = try createTaperedCapsule(allocator, 2.0, 0.5, 0.7);
    defer tapered_capsule_ref.deinit();
    const tapered_capsule = tapered_capsule_ref.get().?;
    try testing.expect(!tapered_capsule.isValidScale(Vec3.zero()));
    try testing.expect(tapered_capsule.isValidScale(Vec3.init(2, 2, 2)));
    try testing.expect(tapered_capsule.isValidScale(Vec3.init(-1, 1, -1)));
    try testing.expect(!tapered_capsule.isValidScale(Vec3.init(2, 1, 1)));
    try testing.expect(!tapered_capsule.isValidScale(Vec3.init(1, 2, 1)));
    try testing.expect(!tapered_capsule.isValidScale(Vec3.init(1, 1, 2)));
    try testing.expect(tapered_capsule.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try testing.expect(tapered_capsule.makeScaleValid(Vec3.init(2, -3, 4)).eql(Vec3.init(3, -3, 3)));
}

test "TaperedCapsuleShape: support functions (a negative Y scale flips the capsule)" {
    const allocator = testing.allocator;

    var shape_ref = try createTaperedCapsule(allocator, 2.0, 0.5, 1.0);
    defer shape_ref.deinit();
    const convex = shape_ref.get().?.cast(ConvexShape);

    var buffer: ConvexShape.SupportBuffer = .{};
    const scale = Vec3.init(2, -2, 2);

    // Include convex radius: the scaled spheres, the convex radius is 0
    const with_convex = convex.getSupportFunction(.include_convex_radius, &buffer, scale);
    try testing.expectEqual(@as(f32, 0.0), with_convex.getConvexRadius());
    try testing.expect(with_convex.getSupport(Vec3.init(0, 1, 0)).eql(Vec3.init(0, 5.5, 0))); // The bottom sphere is on top
    try testing.expect(with_convex.getSupport(Vec3.init(0, -1, 0)).eql(Vec3.init(0, -5.5, 0)));
    try testing.expect(with_convex.getSupport(Vec3.init(4, 0, 0)).eql(Vec3.init(2, 3.5, 0))); // The bigger sphere
    try testing.expect(with_convex.getSupport(Vec3.zero()).eql(Vec3.init(0, -3.5, 0))); // Zero vector: the top

    // Exclude convex radius and default: the radii reduced by the scaled convex radius
    for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .default }) |mode| {
        const no_convex = convex.getSupportFunction(mode, &buffer, scale);
        try testing.expectEqual(@as(f32, 1.0), no_convex.getConvexRadius());
        try testing.expect(no_convex.getSupport(Vec3.init(0, 1, 0)).eql(Vec3.init(0, 4.5, 0)));
        try testing.expect(no_convex.getSupport(Vec3.init(0, -1, 0)).eql(Vec3.init(0, -4.5, 0)));
        try testing.expect(no_convex.getSupport(Vec3.zero()).eql(Vec3.init(0, -4.5, 0)));
    }
}

test "TaperedCapsuleShape: ray casts (the shape part of TestTaperedCapsuleShapeRay) and collide point (TestCollidePointVsTaperedCapsule) through ConvexShape" {
    const allocator = testing.allocator;

    // TestTaperedCapsuleShapeRay: the rays go through the surface points a and b (relative to the shape's origin, rays are
    // relative to the center of mass)
    var shape_ref = try createTaperedCapsule(allocator, 3, 4, 2);
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;
    const cases = [_]struct { a: Vec3, b: Vec3 }{
        .{ .a = Vec3.init(0, 7, 0), .b = Vec3.init(0, -5, 0) }, // Top to bottom
        .{ .a = Vec3.init(-4, 3, 0), .b = Vec3.init(4, 3, 0) }, // Top sphere
        .{ .a = Vec3.init(0, 3, -4), .b = Vec3.init(0, 3, 4) }, // Top sphere
    };
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    for (cases) |c| {
        for ([2][2]Vec3{ .{ c.a, c.b }, .{ c.b, c.a } }) |ab| {
            const delta = ab[1].sub(ab[0]);
            const l1 = ab[0].sub(delta.mulScalar(2.0)).sub(shape.getCenterOfMass());
            const l2 = ab[0].sub(delta.mulScalar(0.1)).sub(shape.getCenterOfMass());
            const inner2 = ab[1].sub(delta.mulScalar(0.1)).sub(shape.getCenterOfMass());
            const r1 = ab[1].add(delta.mulScalar(0.1)).sub(shape.getCenterOfMass());

            // Through the shape: the front and the back face
            var hit: RayCastResult = .{};
            try testing.expect(shape.castRay(.init(l2, r1.sub(l2)), .{}, &hit));
            try testing.expectApproxEqAbs(@as(f32, 0.1) / 1.2, hit.fraction, 1.0e-5);
            var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer hits.deinit();
            shape.castRayCollector(.init(l2, r1.sub(l2)), &settings, .{}, &hits.base, &.{});
            try hits.checkError();
            try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
            try testing.expectApproxEqAbs(@as(f32, 0.1) / 1.2, hits.hits.items[0].fraction, 1.0e-5);
            try testing.expectApproxEqAbs(@as(f32, 1.1) / 1.2, hits.hits.items[1].fraction, 1.0e-5);

            // Starting inside: fraction 0, the back face at 0.5
            hit = .{};
            try testing.expect(shape.castRay(.init(inner2, r1.sub(inner2)), .{}, &hit));
            try testing.expectApproxEqAbs(@as(f32, 0.0), hit.fraction, 1.0e-5);
            hits.reset();
            shape.castRayCollector(.init(inner2, r1.sub(inner2)), &settings, .{}, &hits.base, &.{});
            try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
            try testing.expectApproxEqAbs(@as(f32, 0.0), hits.hits.items[0].fraction, 1.0e-5);
            try testing.expectApproxEqAbs(@as(f32, 0.5), hits.hits.items[1].fraction, 1.0e-5);

            // Stopping before the shape: no hit
            hit = .{};
            try testing.expect(!shape.castRay(.init(l1, l2.sub(l1)), .{}, &hit));
        }
    }

    // TestCollidePointVsTaperedCapsule
    const half_height: f32 = 0.4;
    const top_radius: f32 = 0.1;
    const bottom_radius: f32 = 0.2;
    var point_shape_ref = try createTaperedCapsule(allocator, half_height, top_radius, bottom_radius);
    defer point_shape_ref.deinit();
    const point_shape = point_shape_ref.get().?;
    const xy_probes = [_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 0, -1), Vec3.init(0, 0, 1) };
    const xy_and_zero_probes = [_]Vec3{Vec3.zero()} ++ xy_probes;
    var hit_points: [2 * xy_and_zero_probes.len + 1]Vec3 = undefined;
    for (xy_and_zero_probes, 0..) |probe, i| {
        hit_points[i] = probe.mulScalar(0.99 * top_radius).add(Vec3.init(0, half_height, 0)); // Top hits
        hit_points[xy_and_zero_probes.len + i] = probe.mulScalar(0.99 * bottom_radius).add(Vec3.init(0, -half_height, 0)); // Bottom hits
    }
    hit_points[2 * xy_and_zero_probes.len] = Vec3.zero(); // Center hit
    var miss_points: [2 * xy_probes.len + 2]Vec3 = undefined;
    miss_points[0] = Vec3.init(0, half_height + top_radius + 0.01, 0); // Top misses
    miss_points[1] = Vec3.init(0, -half_height - bottom_radius - 0.01, 0); // Bottom misses
    for (xy_probes, 0..) |probe, i| {
        miss_points[2 + i] = probe.mulScalar(1.01 * top_radius).add(Vec3.init(0, half_height, 0));
        miss_points[2 + xy_probes.len + i] = probe.mulScalar(1.01 * bottom_radius).add(Vec3.init(0, -half_height, 0));
    }
    for ([_][]const Vec3{ &hit_points, &miss_points }, [_]usize{ 1, 0 }) |points, expected| {
        for (points) |point| {
            var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer collector.deinit();
            point_shape.collidePoint(point.sub(point_shape.getCenterOfMass()), .{}, &collector.base, &.{});
            try collector.checkError();
            try testing.expectEqual(expected, collector.hits.items.len);
        }
    }
}

test "TaperedCapsuleShape: CollideSoftBodyVertices" {
    const allocator = testing.allocator;

    var shape_ref = try createTaperedCapsule(allocator, 2.0, 0.5, 1.0);
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;
    const capsule = shape.cast(TaperedCapsuleShape);

    var positions = [_]Vec3{ Vec3.init(0, 3, 0), Vec3.init(0, 2.25, 0), Vec3.init(0, -2, 0), Vec3.init(0.5, 0, 0), Vec3.init(0, -1.75, 0), Vec3.zero() };
    var inv_masses = [_]f32{ 1, 1, 1, 1, 1, 0 };
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** positions.len;
    var penetrations = [_]f32{-math.flt_max} ** positions.len;
    var indices = [_]i32{-1} ** positions.len;
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    shape.collideSoftBodyVertices(Mat44.identity(), Vec3.one(), &vertices, positions.len, 3);

    // Above the top sphere
    try testing.expectEqual(@as(f32, -0.25), penetrations[0]);
    try testing.expect(planes[0].getNormal().eql(Vec3.axisY()));
    try testing.expectEqual(@as(f32, 0.0), planes[0].signedDistance(Vec3.init(0, 2.75, 0)));
    // At the top center: the normal is Y
    try testing.expectEqual(@as(f32, 0.5), penetrations[1]);
    try testing.expect(planes[1].getNormal().eql(Vec3.axisY()));
    // Inside the bottom sphere
    try testing.expectEqual(@as(f32, 0.75), penetrations[2]);
    try testing.expect(planes[2].getNormal().eql(Vec3.axisY().negate()));
    // Near the tapered cylinder: the distance to the cone
    const side_normal = Vec3.init(1, capsule.tan_alpha, 0).normalized();
    try testing.expectApproxEqAbs(-side_normal.dot(Vec3.init(0.5, 1.75, 0)) + 1.0, penetrations[3], 1.0e-6);
    try testing.expect(planes[3].getNormal().isClose(side_normal, .{ .max_dist_sq = 1.0e-12 }));
    // At the bottom center: the bottom sphere with the normal -Y
    try testing.expectEqual(@as(f32, 1.0), penetrations[4]);
    try testing.expect(planes[4].getNormal().eql(Vec3.axisY().negate()));
    try testing.expectEqual(@as(i32, 3), indices[4]);
    // Infinite mass: skipped
    try testing.expectEqual(-math.flt_max, penetrations[5]);
    try testing.expectEqual(@as(i32, -1), indices[5]);

    // Flipped along Y and translated: the top sphere is at the bottom
    var flipped_positions = [_]Vec3{ Vec3.init(1, -1, 0), Vec3.init(1, 1, 0) };
    var flipped_inv_masses = [_]f32{ 1, 1 };
    var flipped_planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** 2;
    var flipped_penetrations = [_]f32{-math.flt_max} ** 2;
    var flipped_indices = [_]i32{-1} ** 2;
    const flipped_vertices = CollideSoftBodyVertexIterator.init(.init(&flipped_positions[0], .{}), .init(&flipped_inv_masses[0], .{}), .init(&flipped_planes[0], .{}), .init(&flipped_penetrations[0], .{}), .init(&flipped_indices[0], .{}));
    shape.collideSoftBodyVertices(Mat44.translation(Vec3.init(1, 2, 0)), Vec3.init(1, -1, 1), &flipped_vertices, 2, 0);
    try testing.expectEqual(@as(f32, -0.25), flipped_penetrations[0]); // 3 below the center
    try testing.expect(flipped_planes[0].getNormal().eql(Vec3.axisY().negate()));
    try testing.expectEqual(@as(f32, 0.0), flipped_planes[0].signedDistance(Vec3.init(1, -0.75, 0)));
    try testing.expectApproxEqAbs(1.0 - 2.75 * side_normal.getY(), flipped_penetrations[1], 1.0e-6); // 1 below the center: near the tapered cylinder
    try testing.expect(flipped_planes[1].getNormal().isClose(side_normal.flipSign(1, -1, 1), .{ .max_dist_sq = 1.0e-12 }));
}

test "TaperedCapsuleShape: GetTrianglesStart / Next and GetSubmergedVolume are ConvexShape's" {
    const allocator = testing.allocator;

    var shape_ref = try createTaperedCapsule(allocator, 2.0, 0.5, 1.0);
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;

    // The support function including the convex radius applied to the unit sphere (384 vertices)
    var context: Shape.GetTrianglesContext = .{};
    shape.getTrianglesStart(&context, AABox.biggest(), Vec3.init(1, 2, 3), Quat.identity(), Vec3.one());
    var vertices: [3 * 128]Float3 = undefined;
    var materials: [128]*const PhysicsMaterial = undefined;
    try testing.expectEqual(@as(u32, 128), shape.getTrianglesNext(&context, 128, &vertices, &materials));
    try testing.expect(materials[0] == PhysicsMaterial.default);
    for (vertices) |f| {
        const v = Vec3.fromFloat3(f).sub(Vec3.init(1, 2, 3));
        try testing.expect(v.getY() >= -2.75 - 1.0e-5 and v.getY() <= 2.75 + 1.0e-5 and @abs(v.getX()) <= 1.0 + 1.0e-5);
    }
    try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&context, 128, &vertices, null));

    // Bounding box based
    const half = shape.getSubmergedVolume(Mat44.identity(), Vec3.one(), Plane.fromPointAndNormal(Vec3.zero(), Vec3.axisY()));
    try testing.expectEqual(@as(f32, 22.0), half.total_volume);
    try testing.expectApproxEqAbs(@as(f32, 11.0), half.submerged_volume, 1.0e-4);
}

test "TaperedCapsuleShape: binary state, restoreFromBinaryState and the registration" {
    const allocator = testing.allocator;

    var shape_ref = try createTaperedCapsule(allocator, 2.0, 0.5, 1.0);
    defer shape_ref.deinit();
    const capsule = shape_ref.get().?.castMut(TaperedCapsuleShape);
    capsule.base.setDensity(321.0);
    capsule.asShapeMut().setUserData(5);

    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    capsule.asShape().saveBinaryState(out.streamOut());
    try testing.expectEqual(@as(usize, 1 + 8 + 4 + 12 + 7 * 4), writer.buffered().len); // Sub type, user data, density, center of mass, 7 floats

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    const restored = result.getPtr().?.cast(TaperedCapsuleShape);
    inline for (.{ "top_radius", "bottom_radius", "top_center", "bottom_center", "convex_radius", "sin_alpha", "tan_alpha" }) |field|
        try testing.expectEqual(@field(capsule, field), @field(restored, field));
    try testing.expect(restored.center_of_mass.eql(capsule.center_of_mass));
    try testing.expectEqual(@as(f32, 321.0), restored.base.getDensity());
    try testing.expectEqual(@as(u64, 5), restored.asShape().getUserData());

    // Truncated: Jolt's error text
    var short_reader: std.Io.Reader = .fixed(writer.buffered()[0 .. writer.buffered().len - 1]);
    var short_in = StreamWrapper.StreamInWrapper.init(&short_reader);
    var short_result = try Shape.restoreFromBinaryState(allocator, short_in.streamIn());
    defer short_result.deinit();
    try testing.expectEqualStrings("Failed to restore shape", short_result.getError());

    // ShapeFunctions
    try testing.expect(ShapeFunctions.get(.tapered_capsule).construct != null);
    try testing.expect(ShapeFunctions.get(.tapered_capsule).color.eql(Color.green));
}

test "TaperedCapsuleShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, TaperedCapsuleShapeSettings.create(failing.allocator(), 1.0, 1.0, 0.5, .{}));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.tapered_capsule).construct.?(failing.allocator()));

    // Restore: the shape is the only allocation
    var shape_ref = try createTaperedCapsule(allocator, 1.0, 1.0, 0.5);
    defer shape_ref.deinit();
    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    shape_ref.get().?.saveBinaryState(out.streamOut());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    try testing.expectError(error.OutOfMemory, Shape.restoreFromBinaryState(failing.allocator(), in.streamIn()));
}
