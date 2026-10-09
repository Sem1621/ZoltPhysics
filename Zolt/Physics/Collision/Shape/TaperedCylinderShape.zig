//! Port of: Jolt/Physics/Collision/Shape/TaperedCylinderShape.h, Jolt/Physics/Collision/Shape/TaperedCylinderShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2, SphereShape.zig / BoxShape.zig are the
//! reference): `overrides` lists every C++ `override` in header order, the support class `TaperedCylinder` is
//! constructed in the caller's SupportBuffer (D9), `TCSGetTrianglesContext` in the caller's GetTrianglesContext (D10).
//! TaperedCylinderShape does not override CastRay or GetSubmergedVolume: those are ConvexShape's (GJK / bounding box).
//!
//! Signatures that differ from C++:
//! - `TaperedCylinderShapeSettings::Create` builds a CylinderShape when the radii are equal, constructed from a local
//!   CylinderShapeSettings into this object's cache (`createShape`, custom Create logic as in Jolt).
//! - The private `GetScaled(inScale, outTop, outBottom, outTopRadius, outBottomRadius, outConvexRadius)` returns a
//!   `ScaledTaperedCylinder` struct (out parameters).
//! - The file static helpers sCalculateSideNormalXZ / sCalculateSideNormal are `calculateSideNormalXZ` /
//!   `calculateSideNormal`.
//! JPH_DEBUG_RENDERER (Draw) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
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
const CylinderShapeFile = @import("CylinderShape.zig");
const CylinderShape = CylinderShapeFile.CylinderShape;
const CylinderShapeSettings = CylinderShapeFile.CylinderShapeSettings;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Approximation of a face of the tapered cylinder (static cTaperedCylinderFace)
const tapered_cylinder_face = [_]Vec3{
    Vec3.init(0.0, 0.0, 1.0),
    Vec3.init(0.707106769, 0.0, 0.707106769),
    Vec3.init(1.0, 0.0, 0.0),
    Vec3.init(0.707106769, 0.0, -0.707106769),
    Vec3.init(-0.0, 0.0, -1.0),
    Vec3.init(-0.707106769, 0.0, -0.707106769),
    Vec3.init(-1.0, 0.0, 0.0),
    Vec3.init(-0.707106769, 0.0, 0.707106769),
};

/// Class that constructs a TaperedCylinderShape
pub const TaperedCylinderShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, TaperedCylinderShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    half_height: f32 = 0.0,
    top_radius: f32 = 0.0,
    bottom_radius: f32 = 0.0,
    convex_radius: f32 = 0.0,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) TaperedCylinderShapeSettings {
        return .{ .base = .init(TaperedCylinderShapeSettings, allocator, null) };
    }

    /// Create a tapered cylinder centered around the origin with bottom at (0, -half_height_of_tapered_cylinder, 0) with radius bottom_radius and top at (0, half_height_of_tapered_cylinder, 0) with radius top_radius
    pub fn init(allocator: Allocator, half_height_of_tapered_cylinder: f32, top_radius: f32, bottom_radius: f32, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) TaperedCylinderShapeSettings {
        return .{ .base = .init(TaperedCylinderShapeSettings, allocator, opts.material), .half_height = half_height_of_tapered_cylinder, .top_radius = top_radius, .bottom_radius = bottom_radius, .convex_radius = opts.convex_radius };
    }

    /// new TaperedCylinderShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, half_height_of_tapered_cylinder: f32, top_radius: f32, bottom_radius: f32, opts: struct { convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!*TaperedCylinderShapeSettings {
        const self = try allocator.create(TaperedCylinderShapeSettings);
        self.* = .init(allocator, half_height_of_tapered_cylinder, top_radius, bottom_radius, .{ .convex_radius = opts.convex_radius, .material = opts.material });
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *TaperedCylinderShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *TaperedCylinderShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    /// Create a shape according to the settings specified by this object.
    /// Note that this will return a CylinderShape if top and bottom radii are equal.
    pub fn createShape(self: *TaperedCylinderShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        const cached_result = &self.base.base.cached_result;
        if (cached_result.isEmpty()) {
            if (self.top_radius == self.bottom_radius) {
                // Convert to regular cylinder
                var settings = CylinderShapeSettings.initDefault(allocator);
                defer settings.deinit();
                settings.half_height = self.half_height;
                settings.radius = self.top_radius;
                settings.base.material.set(self.base.material.get());
                settings.convex_radius = self.convex_radius;

                // shape = new CylinderShape(settings, mCachedResult)
                errdefer cached_result.clear(); // Out of memory is not cached, a later call can succeed
                const shape = try allocator.create(CylinderShape);
                shape.* = .initDefault(allocator);
                var ref = Ref(Shape).init(shape.asShapeMut());
                defer ref.deinit();
                try shape.initFromSettings(&settings, cached_result, allocator);
            } else {
                // Normal tapered cylinder shape
                try ShapeSettings.constructShape(TaperedCylinderShape, self, allocator);
            }
        }
        return cached_result.clone();
    }
};

/// A cylinder with different top and bottom radii
pub const TaperedCylinderShape = struct {
    /// Concrete class: `Shape.cast(TaperedCylinderShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .tapered_cylinder;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: ConvexShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    top: f32 = 0.0,
    bottom: f32 = 0.0,
    top_radius: f32 = 0.0,
    bottom_radius: f32 = 0.0,
    convex_radius: f32 = 0.0,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// TaperedCylinderShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createShape
    pub fn initDefault(allocator: Allocator) TaperedCylinderShape {
        return .{ .base = .init(TaperedCylinderShape, allocator, shape_sub_type, null) };
    }

    /// TaperedCylinderShape(const TaperedCylinderShapeSettings &inSettings, ShapeResult &outResult): C++ member initializers first, then the body
    pub fn initFromSettings(self: *TaperedCylinderShape, settings: *const TaperedCylinderShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base, result);
        self.top_radius = settings.top_radius;
        self.bottom_radius = settings.bottom_radius;
        self.convex_radius = math.min(settings.convex_radius, math.min(settings.top_radius, settings.bottom_radius));

        if (self.top_radius < 0.0) {
            result.setError("Invalid top radius");
            return;
        }

        if (self.bottom_radius < 0.0) {
            result.setError("Invalid bottom radius");
            return;
        }

        if (self.convex_radius < 0.0) {
            result.setError("Invalid convex radius");
            return;
        }

        if (settings.half_height <= 0.0) {
            result.setError("Invalid height");
            return;
        }

        // Calculate the center of mass (using wxMaxima).
        // Radius of cross section for tapered cylinder from 0 to h:
        // r(x):=br+x*(tr-br)/h;
        // Area:
        // area(x):=%pi*r(x)^2;
        // Total volume of cylinder:
        // volume(h):=integrate(area(x),x,0,h);
        // Center of mass:
        // com(br,tr,h):=integrate(x*area(x),x,0,h)/volume(h);
        // Results:
        // ratsimp(com(br,tr,h),br,bt);
        // Non-tapered cylinder should have com = 0.5:
        // ratsimp(com(r,r,h));
        // Cone with tip at origin and height h should have com = 3/4 h
        // ratsimp(com(0,r,h));
        const h = 2.0 * settings.half_height;
        const tr = self.top_radius;
        const tr2 = math.square(tr);
        const br = self.bottom_radius;
        const br2 = math.square(br);
        const com = h * (3 * tr2 + 2 * br * tr + br2) / (4.0 * (tr2 + br * tr + br2));
        self.top = h - com;
        self.bottom = -com;

        result.set(.init(self.asShapeMut()));
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const TaperedCylinderShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *TaperedCylinderShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get top radius of the tapered cylinder
    pub fn getTopRadius(self: *const TaperedCylinderShape) f32 {
        return self.top_radius;
    }

    /// Get bottom radius of the tapered cylinder
    pub fn getBottomRadius(self: *const TaperedCylinderShape) f32 {
        return self.bottom_radius;
    }

    /// Get convex radius of the tapered cylinder
    pub fn getConvexRadius(self: *const TaperedCylinderShape) f32 {
        return self.convex_radius;
    }

    /// Get half height of the tapered cylinder
    pub fn getHalfHeight(self: *const TaperedCylinderShape) f32 {
        return 0.5 * (self.top - self.bottom);
    }

    /// The tapered cylinder scaled by a scale (out parameters of GetScaled)
    const ScaledTaperedCylinder = struct {
        top: f32,
        bottom: f32,
        top_radius: f32,
        bottom_radius: f32,
        convex_radius: f32,
    };

    // Scale the cylinder
    fn getScaled(self: *const TaperedCylinderShape, scale: Vec3) ScaledTaperedCylinder {
        const abs_scale = scale.abs();
        const scale_xz = abs_scale.getX();
        const scale_y = scale.getY();

        var out: ScaledTaperedCylinder = .{
            .top = scale_y * self.top,
            .bottom = scale_y * self.bottom,
            .top_radius = scale_xz * self.top_radius,
            .bottom_radius = scale_xz * self.bottom_radius,
            .convex_radius = math.min(abs_scale.getY(), scale_xz) * self.convex_radius,
        };

        // Negative Y-scale flips the top and bottom
        if (out.bottom > out.top) {
            std.mem.swap(f32, &out.top, &out.bottom);
            std.mem.swap(f32, &out.top_radius, &out.bottom_radius);
        }

        return out;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const TaperedCylinderShape) Vec3 {
        return Vec3.init(0, -0.5 * (self.top + self.bottom), 0);
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const TaperedCylinderShape) AABox {
        const max_radius = math.max(self.top_radius, self.bottom_radius);
        return .init(Vec3.init(-max_radius, self.bottom, -max_radius), Vec3.init(max_radius, self.top, max_radius));
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const TaperedCylinderShape) f32 {
        return math.min(self.top_radius, self.bottom_radius);
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const TaperedCylinderShape) MassProperties {
        var p: MassProperties = .{};

        // Calculate mass
        const density = self.base.getDensity();
        p.mass = self.getVolume() * density; // C++ unqualified GetVolume(): a static call is equivalent in a final class

        // Calculate inertia of a tapered cylinder (using wxMaxima)
        // Radius:
        // r(x):=br+(x-b)*(tr-br)/(t-b);
        // Where t=top, b=bottom, tr=top radius, br=bottom radius
        // Area of the cross section of the cylinder at x:
        // area(x):=%pi*r(x)^2;
        // Inertia x slice at x (using inertia of a solid disc, see https://en.wikipedia.org/wiki/List_of_moments_of_inertia, note needs to be multiplied by density):
        // dix(x):=area(x)*r(x)^2/4;
        // Inertia y slice at y (note needs to be multiplied by density)
        // diy(x):=area(x)*r(x)^2/2;
        // Volume:
        // volume(b,t):=integrate(area(x),x,b,t);
        // The constant density (note that we have this through GetDensity() so we'll use that instead):
        // density(b,t):=m/volume(b,t);
        // Inertia tensor element xx, note that we use the parallel axis theorem to move the inertia: Ixx' = Ixx + m translation^2, also note we multiply by density here:
        // Ixx(br,tr,b,t):=integrate(dix(x)+area(x)*x^2,x,b,t)*density(b,t);
        // Inertia tensor element yy:
        // Iyy(br,tr,b,t):=integrate(diy(x),x,b,t)*density(b,t);
        // Note that we can simplify Ixx by using:
        // Ixx_delta(br,tr,b,t):=Ixx(br,tr,b,t)-Iyy(br,tr,b,t)/2;
        // For a cylinder this formula matches what is listed on the wiki:
        // factor(Ixx(r,r,-h/2,h/2));
        // factor(Iyy(r,r,-h/2,h/2));
        // For a cone with tip at origin too:
        // factor(Ixx(0,r,0,h));
        // factor(Iyy(0,r,0,h));
        // Now for the tapered cylinder:
        // rat(Ixx(br,tr,b,t),br,bt);
        // rat(Iyy(br,tr,b,t),br,bt);
        // rat(Ixx_delta(br,tr,b,t),br,bt);
        const t = self.top;
        const t2 = math.square(t);
        const t3 = t * t2;

        const b = self.bottom;
        const b2 = math.square(b);
        const b3 = b * b2;

        const br = self.bottom_radius;
        const br2 = math.square(br);
        const br3 = br * br2;
        const br4 = math.square(br2);

        const tr = self.top_radius;
        const tr2 = math.square(tr);
        const tr3 = tr * tr2;
        const tr4 = math.square(tr2);

        const inertia_y = (math.pi / 10.0) * density * (t - b) * (br4 + tr * br3 + tr2 * br2 + tr3 * br + tr4);
        const inertia_x_delta = (math.pi / 30.0) * density * ((t3 + 2 * b * t2 + 3 * b2 * t - 6 * b3) * br2 + (3 * t3 + b * t2 - b2 * t - 3 * b3) * tr * br + (6 * t3 - 3 * b * t2 - 2 * b2 * t - b3) * tr2);
        const inertia_x = inertia_x_delta + inertia_y / 2;
        const inertia_z = inertia_x;
        p.inertia = Mat44.scaleVec3(Vec3.init(inertia_x, inertia_y, inertia_z));
        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const TaperedCylinderShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        const epsilon: f32 = 1.0e-5;

        if (local_surface_position.getY() > self.top - epsilon)
            return Vec3.init(0, 1, 0)
        else if (local_surface_position.getY() < self.bottom + epsilon)
            return Vec3.init(0, -1, 0)
        else
            return calculateSideNormal(calculateSideNormalXZ(local_surface_position), self.top, self.bottom, self.top_radius, self.bottom_radius);
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const TaperedCylinderShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        // Get scaled tapered cylinder
        const scaled = self.getScaled(scale);
        const top = scaled.top;
        const bottom = scaled.bottom;
        const top_radius = scaled.top_radius;
        const bottom_radius = scaled.bottom_radius;

        // Get the normal of the side of the cylinder
        const normal_xz = calculateSideNormalXZ(direction.negate());
        const normal = calculateSideNormal(normal_xz, top, bottom, top_radius, bottom_radius);

        const min_radius: f32 = 1.0e-3;

        // Check if the normal is closer to the side than to the top or bottom
        if (@abs(normal.dot(direction)) > @abs(direction.getY())) {
            // Return the side of the cylinder
            out_vertices.append(center_of_mass_transform.mulVec3(normal_xz.mulScalar(top_radius).add(Vec3.init(0, top, 0))));
            out_vertices.append(center_of_mass_transform.mulVec3(normal_xz.mulScalar(bottom_radius).add(Vec3.init(0, bottom, 0))));
        } else {
            // When the direction is more than 5 degrees from vertical, align the vertices so that 1 of the vertices
            // points towards direction in the XZ plane. This ensures that we always have a vertex towards max penetration depth.
            var transform = center_of_mass_transform;
            var base_x = Vec4.init(direction.getX(), 0, direction.getZ(), 0);
            const xz_sq = base_x.lengthSq();
            const y_sq = math.square(direction.getY());
            if (xz_sq > 0.00765427 * y_sq) {
                base_x = base_x.divScalar(@sqrt(xz_sq));
                const base_z = base_x.swizzle(.z, .y, .x, .w).mul(Vec4.init(-1, 0, 1, 0));
                transform = transform.mul(Mat44.init(base_x, Vec4.init(0, 1, 0, 0), base_z, Vec4.init(0, 0, 0, 1)));
            }

            if (direction.getY() < 0.0) {
                // Top of the cylinder
                if (top_radius > min_radius) {
                    const top_3d = Vec3.init(0, top, 0);
                    for (tapered_cylinder_face) |v|
                        out_vertices.append(transform.mulVec3(v.mulScalar(top_radius).add(top_3d)));
                }
            } else {
                // Bottom of the cylinder
                if (bottom_radius > min_radius) {
                    const bottom_3d = Vec3.init(0, bottom, 0);
                    var i: usize = tapered_cylinder_face.len;
                    while (i > 0) {
                        i -= 1;
                        out_vertices.append(transform.mulVec3(tapered_cylinder_face[i].mulScalar(bottom_radius).add(bottom_3d)));
                    }
                }
            }
        }
    }

    // See ConvexShape::GetSupportFunction
    pub fn getSupportFunction(self: *const TaperedCylinderShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        // Get scaled tapered cylinder
        const s = self.getScaled(scale);

        switch (mode) {
            .include_convex_radius, .default => {
                const support = buffer.emplace(TaperedCylinder);
                support.* = .init(s.top, s.bottom, s.top_radius, s.bottom_radius, 0.0);
                return &support.base;
            },

            .exclude_convex_radius => {
                const support = buffer.emplace(TaperedCylinder);
                support.* = .init(s.top - s.convex_radius, s.bottom + s.convex_radius, s.top_radius - s.convex_radius, s.bottom_radius - s.convex_radius, s.convex_radius);
                return &support.base;
            },
        }
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const TaperedCylinderShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Check if the point is in the tapered cylinder
        if (point.getY() >= self.bottom and point.getY() <= self.top // Within height
        and math.square(point.getX()) + math.square(point.getZ()) <= math.square(self.bottom_radius + (point.getY() - self.bottom) * (self.top_radius - self.bottom_radius) / (self.top - self.bottom))) // Within the radius
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const TaperedCylinderShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        std.debug.assert(self.isValidScale(scale)); // C++ unqualified IsValidScale(): a static call is equivalent in a final class

        const inverse_transform = center_of_mass_transform.inversedRotationTranslation();

        // Get scaled tapered cylinder
        const scaled = self.getScaled(scale);
        const top = scaled.top;
        const bottom = scaled.bottom;
        const top_radius = scaled.top_radius;
        const bottom_radius = scaled.bottom_radius;
        const top_3d = Vec3.init(0, top, 0);
        const bottom_3d = Vec3.init(0, bottom, 0);

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                const local_pos = inverse_transform.mulVec3(v.getPosition());

                // Calculate penetration into side surface
                const normal_xz = calculateSideNormalXZ(local_pos);
                const side_normal = calculateSideNormal(normal_xz, top, bottom, top_radius, bottom_radius);
                const side_support_top = normal_xz.mulScalar(top_radius).add(top_3d);
                const side_penetration = side_support_top.sub(local_pos).dot(side_normal);

                // Calculate penetration into top and bottom plane
                const top_penetration = top - local_pos.getY();
                const bottom_penetration = local_pos.getY() - bottom;
                const min_top_bottom_penetration = math.min(top_penetration, bottom_penetration);

                var point: Vec3 = undefined;
                var normal: Vec3 = undefined;
                if (side_penetration < 0.0 or min_top_bottom_penetration < 0.0) {
                    // We're outside the cylinder
                    // Calculate the closest point on the line segment from bottom to top support point:
                    // closest_point = bottom + fraction * (top - bottom) / |top - bottom|^2
                    const side_support_bottom = normal_xz.mulScalar(bottom_radius).add(bottom_3d);
                    const bottom_to_top = side_support_top.sub(side_support_bottom);
                    const fraction = local_pos.sub(side_support_bottom).dot(bottom_to_top);

                    // Calculate the distance to the axis of the cylinder
                    const distance_to_axis = normal_xz.dot(local_pos);
                    const inside_top_radius = distance_to_axis <= top_radius;
                    const inside_bottom_radius = distance_to_axis <= bottom_radius;

                    //  Regions of tapered cylinder (side view):
                    //
                    //      _  B |       |
                    //       --_ |   A   |
                    //           t-------+
                    //     C    /         \
                    //         /  tapered  \
                    //  _     /  cylinder   \
                    //   --_ /               \
                    //      b-----------------+
                    //   D  |        E        |
                    //      |                 |
                    //
                    //  t = side_support_top, b = side_support_bottom
                    //  Lines between B and C and C and D are at a 90 degree angle to the line between t and b
                    if (fraction >= bottom_to_top.lengthSq() // Region B: Above the line segment
                    and !inside_top_radius) // Outside the top radius
                    {
                        // Top support point is closest
                        point = side_support_top;
                        normal = local_pos.sub(point).normalizedOr(Vec3.axisY());
                    } else if (fraction < 0.0 // Region D: Below the line segment
                    and !inside_bottom_radius) // Outside the bottom radius
                    {
                        // Bottom support point is closest
                        point = side_support_bottom;
                        normal = local_pos.sub(point).normalizedOr(Vec3.axisY());
                    } else if (top_penetration < 0.0 // Region A: Above the top plane
                    and inside_top_radius) // Inside the top radius
                    {
                        // Top plane is closest
                        point = top_3d;
                        normal = Vec3.init(0, 1, 0);
                    } else if (bottom_penetration < 0.0 // Region E: Below the bottom plane
                    and inside_bottom_radius) // Inside the bottom radius
                    {
                        // Bottom plane is closest
                        point = bottom_3d;
                        normal = Vec3.init(0, -1, 0);
                    } else // Region C
                    {
                        // Side surface is closest
                        point = side_support_top;
                        normal = side_normal;
                    }
                } else if (side_penetration < min_top_bottom_penetration) {
                    // Side surface is closest
                    point = side_support_top;
                    normal = side_normal;
                } else if (top_penetration < bottom_penetration) {
                    // Top plane is closest
                    point = top_3d;
                    normal = Vec3.init(0, 1, 0);
                } else {
                    // Bottom plane is closest
                    point = bottom_3d;
                    normal = Vec3.init(0, -1, 0);
                }

                // Calculate penetration
                const plane = Plane.fromPointAndNormal(point, normal);
                const penetration = -plane.signedDistance(local_pos);
                if (v.updatePenetration(penetration))
                    v.setCollision(plane.getTransformed(center_of_mass_transform), colliding_shape_index);
            }
        }
    }

    // See Shape::GetTrianglesStart: placement new of the context (no pointers into itself: construct by value)
    pub fn getTrianglesStart(self: *const TaperedCylinderShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;

        // Make sure the scale is not inside out
        const s = if (ScaleHelpers.isInsideOut(scale)) scale.flipSign(-1, 1, 1) else scale;

        // Mark top and bottom processed if their radius is too small
        const ctx = context.emplace(TCSGetTrianglesContext);
        ctx.* = .init(Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(s)));
        const min_radius: f32 = 1.0e-3;
        if (self.top_radius < min_radius)
            ctx.processed |= 0b001;
        if (self.bottom_radius < min_radius)
            ctx.processed |= 0b010;
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const TaperedCylinderShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        const num_vertices: u32 = tapered_cylinder_face.len;

        comptime std.debug.assert(Shape.get_triangles_min_triangles_requested >= 2 * num_vertices);
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        const ctx = context.get(TCSGetTrianglesContext);

        var total_num_triangles: u32 = 0;
        var out: usize = 0; // Write position in out_triangle_vertices (C++ outTriangleVertices++)

        // Top cap
        const top_3d = Vec3.init(0, self.top, 0);
        if ((ctx.processed & 0b001) == 0) {
            const v0 = ctx.transform.mulVec3(top_3d.add(tapered_cylinder_face[0].mulScalar(self.top_radius)));
            var v1 = ctx.transform.mulVec3(top_3d.add(tapered_cylinder_face[1].mulScalar(self.top_radius)));

            for (tapered_cylinder_face[2..num_vertices]) |v| {
                const v2 = ctx.transform.mulVec3(top_3d.add(v.mulScalar(self.top_radius)));

                v0.storeFloat3(&out_triangle_vertices[out + 0]);
                v1.storeFloat3(&out_triangle_vertices[out + 1]);
                v2.storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;

                v1 = v2;
            }

            total_num_triangles = num_vertices - 2;
            ctx.processed |= 0b001;
        }

        // Bottom cap
        const bottom_3d = Vec3.init(0, self.bottom, 0);
        if ((ctx.processed & 0b010) == 0 and total_num_triangles + num_vertices - 2 < max_triangles_requested) {
            const v0 = ctx.transform.mulVec3(bottom_3d.add(tapered_cylinder_face[0].mulScalar(self.bottom_radius)));
            var v1 = ctx.transform.mulVec3(bottom_3d.add(tapered_cylinder_face[1].mulScalar(self.bottom_radius)));

            for (tapered_cylinder_face[2..num_vertices]) |v| {
                const v2 = ctx.transform.mulVec3(bottom_3d.add(v.mulScalar(self.bottom_radius)));

                v0.storeFloat3(&out_triangle_vertices[out + 0]);
                v2.storeFloat3(&out_triangle_vertices[out + 1]);
                v1.storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;

                v1 = v2;
            }

            total_num_triangles += num_vertices - 2;
            ctx.processed |= 0b010;
        }

        // Side
        if ((ctx.processed & 0b100) == 0 and total_num_triangles + 2 * num_vertices < max_triangles_requested) {
            var v0t = ctx.transform.mulVec3(top_3d.add(tapered_cylinder_face[num_vertices - 1].mulScalar(self.top_radius)));
            var v0b = ctx.transform.mulVec3(bottom_3d.add(tapered_cylinder_face[num_vertices - 1].mulScalar(self.bottom_radius)));

            for (tapered_cylinder_face[0..num_vertices]) |v| {
                const v1t = ctx.transform.mulVec3(top_3d.add(v.mulScalar(self.top_radius)));
                v0t.storeFloat3(&out_triangle_vertices[out + 0]);
                v0b.storeFloat3(&out_triangle_vertices[out + 1]);
                v1t.storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;

                const v1b = ctx.transform.mulVec3(bottom_3d.add(v.mulScalar(self.bottom_radius)));
                v1t.storeFloat3(&out_triangle_vertices[out + 0]);
                v0b.storeFloat3(&out_triangle_vertices[out + 1]);
                v1b.storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;

                v0t = v1t;
                v0b = v1b;
            }

            total_num_triangles += 2 * num_vertices;
            ctx.processed |= 0b100;
        }

        // Store materials
        if (out_materials) |materials| {
            const material = self.base.getConvexMaterial();
            for (materials[0..total_num_triangles]) |*m|
                m.* = material;
        }

        return total_num_triangles;
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER)

    // See Shape::SaveBinaryState: C++ `ConvexShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const TaperedCylinderShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.top);
        stream.write(self.bottom);
        stream.write(self.top_radius);
        stream.write(self.bottom_radius);
        stream.write(self.convex_radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const TaperedCylinderShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TaperedCylinderShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const TaperedCylinderShape) f32 {
        // Volume of a tapered cylinder is: integrate(%pi*(b+x*(t-b)/h)^2,x,0,h) where t is the top radius, b is the bottom radius and h is the height
        return (math.pi / 3.0) * (self.top - self.bottom) * (math.square(self.top_radius) + self.top_radius * self.bottom_radius + math.square(self.bottom_radius));
    }

    // See Shape::IsValidScale: C++ `ConvexShape::IsValidScale` resolves to Shape's version (ConvexShape does not override it)
    pub fn isValidScale(self: *const TaperedCylinderShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScaleXZ(scale.abs());
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const TaperedCylinderShape, scale: Vec3) Vec3 {
        _ = self;
        const s = ScaleHelpers.makeNonZeroScale(scale);

        return s.getSign().mul(ScaleHelpers.makeUniformScaleXZ(s.abs()));
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *TaperedCylinderShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.top);
        stream.read(&self.bottom);
        stream.read(&self.top_radius);
        stream.read(&self.bottom_radius);
        stream.read(&self.convex_radius);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.tapered_cylinder);
        f.construct = ShapeFunctions.constructor(TaperedCylinderShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Class for GetSupportFunction (`class TaperedCylinderShape::TaperedCylinder final : public Support`)

    const TaperedCylinder = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        top: f32,
        bottom: f32,
        top_radius: f32,
        bottom_radius: f32,
        convex_radius: f32,

        fn init(top: f32, bottom: f32, top_radius: f32, bottom_radius: f32, convex_radius: f32) TaperedCylinder {
            return .{ .base = .init(TaperedCylinder), .top = top, .bottom = bottom, .top_radius = top_radius, .bottom_radius = bottom_radius, .convex_radius = convex_radius };
        }

        pub fn getSupport(self: *const TaperedCylinder, direction: Vec3) Vec3 {
            const x = direction.getX();
            const y = direction.getY();
            const z = direction.getZ();
            const o = @sqrt(math.square(x) + math.square(z));
            if (o > 0.0) {
                const top_support = Vec3.init((self.top_radius * x) / o, self.top, (self.top_radius * z) / o);
                const bottom_support = Vec3.init((self.bottom_radius * x) / o, self.bottom, (self.bottom_radius * z) / o);
                return if (direction.dot(top_support) > direction.dot(bottom_support)) top_support else bottom_support;
            } else {
                if (y > 0.0)
                    return Vec3.init(0, self.top, 0)
                else
                    return Vec3.init(0, self.bottom, 0);
            }
        }

        pub fn getConvexRadius(self: *const TaperedCylinder) f32 {
            return self.convex_radius;
        }
    };

    // ---------------------------------------------------------------------------------------------------------------
    // Class for GetTrianglesStart / Next (`class TaperedCylinderShape::TCSGetTrianglesContext`)

    const TCSGetTrianglesContext = struct {
        transform: Mat44,
        /// Which elements we processed, bit 0 = top, bit 1 = bottom, bit 2 = side
        processed: u32 = 0,

        fn init(transform: Mat44) TCSGetTrianglesContext {
            return .{ .transform = transform };
        }
    };
};

fn calculateSideNormalXZ(surface_position: Vec3) Vec3 {
    return Vec3.init(1, 0, 1).mul(surface_position).normalizedOr(Vec3.axisX());
}

fn calculateSideNormal(normal_xz: Vec3, top: f32, bottom: f32, top_radius: f32, bottom_radius: f32) Vec3 {
    const tan_alpha = (bottom_radius - top_radius) / (top - bottom);
    return Vec3.init(normal_xz.getX(), tan_alpha, normal_xz.getZ()).normalized();
}
