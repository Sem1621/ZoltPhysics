//! Port of: Jolt/Physics/Collision/Shape/ConvexShape.h, Jolt/Physics/Collision/Shape/ConvexShape.cpp
//! Status: complete
//!
//! Architecture (Docs/Zolt/CollisionArchitecture.md):
//! - D1 an abstract class that adds a virtual function: `ConvexShape.VTable = struct { base: Shape.VTable,
//!   getSupportFunction }` (the C++ single inheritance layout). `Shape.vtable` of a convex shape points at that `base`
//!   field and `getVTable()` recovers the full table; this is safe by construction because the only constructor,
//!   `ConvexShape.init(T, ...)`, builds `ConvexShape.vtableFor(T)` itself.
//! - The bodies of the Shape virtual functions that ConvexShape overrides are in `ConvexShape.impl` (abstract class
//!   rule): the top-level names of ConvexShape are only the dispatcher of `getSupportFunction` and non virtual
//!   functions, so a call inside this file can never bind statically where C++ makes a virtual call (e.g. the collector
//!   version of CastRay calls the virtual `CastRay`, which SphereShape, BoxShape, ... override with an analytic
//!   version). A derived class calls `ConvexShape::SaveBinaryState(inStream)` as
//!   `ConvexShape.impl.saveBinaryState(&self.base, stream)`.
//! - D9 `getSupportFunction` constructs a `Support` (pattern A, never destroyed) in the caller's `SupportBuffer`
//!   (placement new: `buffer.emplace(T)`, size and alignment checked at compile time). `*const Support` is a convex
//!   object for GJK / EPA (Geometry/ConvexSupport.zig), wrapped with `TransformedConvexObject(Support)` /
//!   `AddConvexRadius(Support)` like Jolt's templates: one virtual call per support point, same operation order.
//! - D10 `CSGetTrianglesContext` holds a SupportBuffer and a pointer into it: it is constructed in place in the caller's
//!   `Shape.GetTrianglesContext` (`context.emplace(CSGetTrianglesContext).init(...)`). `sUnitSphereTriangles` is a
//!   comptime constant (`unit_sphere_triangles`) with the bits of Jolt's static initializer.
//! - D4 `sRegister` is `register(comptime r: *Registry)`; sCollideConvexVsConvex / sCastConvexVsConvex are
//!   `collideConvexVsConvex` / `castConvexVsConvex`.
//!
//! Signatures that differ from C++:
//! - Jolt's non virtual `ConvexShape::GetMaterial()` is `getConvexMaterial()` (D14: the virtual
//!   `Shape.getMaterial(sub_shape_id)` has the plain name).
//! - The constructors `ConvexShape(EShapeSubType)` / `ConvexShape(EShapeSubType, const PhysicsMaterial *)` are
//!   `init(T, allocator, sub_type, material)` (with the most derived type `T`, for the vtable), the settings constructor
//!   is `initFromSettings` (D3).
//! - JPH_DEBUG_RENDERER (DrawGetSupportFunction, DrawGetSupportingFace and the `inBaseOffset` parameter of
//!   GetSubmergedVolume) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const PlacementBuffer = @import("../../../Core/PlacementBuffer.zig").PlacementBuffer;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const virtual = @import("../../../Core/Virtual.zig");
const Core = @import("../../../Core/Core.zig");
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const ConvexSupport = @import("../../../Geometry/ConvexSupport.zig");
const AddConvexRadius = ConvexSupport.AddConvexRadius;
const PointConvexSupport = ConvexSupport.PointConvexSupport;
const TransformedConvexObject = ConvexSupport.TransformedConvexObject;
const EPAPenetrationDepth = @import("../../../Geometry/EPAPenetrationDepth.zig").EPAPenetrationDepth;
const GJKClosestPoint = @import("../../../Geometry/GJKClosestPoint.zig").GJKClosestPoint;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const math = @import("../../../Math/Math.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const PhysicsSettings = @import("../../PhysicsSettings.zig");
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeType = ShapeFile.ShapeType;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const PhysicsMaterialList = ShapeFile.PhysicsMaterialList;
const PhysicsMaterialRefC = ShapeFile.PhysicsMaterialRefC;
const ScaleHelpers = @import("ScaleHelpers.zig");
const GetTrianglesContextVertexList = @import("GetTrianglesContext.zig").GetTrianglesContextVertexList;
const PolyhedronSubmergedVolumeCalculator = @import("PolyhedronSubmergedVolumeCalculator.zig").PolyhedronSubmergedVolumeCalculator;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollidePointResult = @import("../CollidePointResult.zig").CollidePointResult;
const CollideShapeFile = @import("../CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const Registry = @import("../CollisionDispatch.zig").Registry;
const NarrowPhaseStats = @import("../NarrowPhaseStats.zig");
const TrackNarrowPhaseCollector = NarrowPhaseStats.TrackNarrowPhaseCollector;
const track_narrowphase_stats = NarrowPhaseStats.track_narrowphase_stats;

/// Class that constructs a ConvexShape (abstract)
pub const ConvexShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT(JPH_EXPORT, ConvexShapeSettings)

    base: ShapeSettings,

    // Properties
    /// Material assigned to this shape
    material: RefConst(PhysicsMaterial) = .empty,
    /// Uniform density of the interior of the convex object (kg / m^3)
    density: f32 = 1000.0,

    /// Constructor (ConvexShapeSettings() / ConvexShapeSettings(const PhysicsMaterial *inMaterial)), called by the
    /// derived settings with their most derived type `T` (for the vtable)
    pub fn init(comptime T: type, allocator: Allocator, material: ?*const PhysicsMaterial) ConvexShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator), .material = .init(material) };
    }

    /// ~ConvexShapeSettings
    pub fn destruct(self: *ConvexShapeSettings) void {
        self.material.deinit();
    }

    /// Set the density of the object in kg / m^3
    pub fn setDensity(self: *ConvexShapeSettings, density: f32) void {
        self.density = density;
    }
};

/// Base class for all convex shapes. Defines a virtual interface.
pub const ConvexShape = struct {
    /// Abstract class: `Shape.cast(ConvexShape)` checks the shape type
    pub const shape_type: ShapeType = .convex;

    /// Shape.VTable plus the virtual functions that ConvexShape introduces
    pub const VTable = struct {
        base: Shape.VTable,
        /// Returns an object that provides the GetSupport function for this shape.
        /// mode determines if this support function includes or excludes the convex radius.
        /// of the values returned by the GetSupport function. This improves numerical accuracy of the results.
        /// scale scales this shape in local space.
        getSupportFunction: *const fn (self: *const ConvexShape, mode: SupportMode, buffer: *SupportBuffer, scale: Vec3) *const Support,
    };

    /// Function that provides an interface for GJK
    pub const Support = struct {
        /// One entry per C++ virtual function. Warning: Virtual destructor will not be called on this object! (so there is
        /// no destructor entry, `init` refuses types that own resources)
        pub const VTable = struct {
            /// Calculate the support vector for this convex shape (includes / excludes the convex radius depending on how this was obtained).
            /// Support vector is relative to the center of mass of the shape.
            getSupport: *const fn (self: *const Support, direction: Vec3) Vec3,

            /// Convex radius of shape. Collision detection on penetrating shapes is much more expensive,
            /// so you can add a radius around objects to increase the shape. This makes it far less likely that they will actually penetrate.
            getConvexRadius: *const fn (self: *const Support) f32,
        };

        vtable: *const Support.VTable,

        /// Constructor, called by the concrete supports with their own type: `.{ .base = .init(@This()), ... }`
        pub fn init(comptime T: type) Support {
            comptime {
                if (@hasDecl(T, "deinit") or @hasDecl(T, "destruct"))
                    @compileError(@typeName(T) ++ ": a Support is never destroyed, it cannot own resources");
            }
            return .{ .vtable = virtual.vtablePtr(Support.VTable, T) };
        }

        /// Calculate the support vector for this convex shape (includes / excludes the convex radius depending on how this was obtained).
        /// Support vector is relative to the center of mass of the shape.
        pub fn getSupport(self: *const Support, direction: Vec3) Vec3 {
            return self.vtable.getSupport(self, direction);
        }

        /// Convex radius of shape. Collision detection on penetrating shapes is much more expensive,
        /// so you can add a radius around objects to increase the shape. This makes it far less likely that they will actually penetrate.
        pub fn getConvexRadius(self: *const Support) f32 {
            return self.vtable.getConvexRadius(self);
        }
    };

    /// Buffer to hold a Support object, used to avoid dynamic memory allocations (alignas(16), 4160 bytes)
    pub const SupportBuffer = PlacementBuffer(4160, 16, .{});

    /// How the GetSupport function should behave
    pub const SupportMode = enum {
        /// Return the shape excluding the convex radius, Support::GetConvexRadius will return the convex radius if there is one, but adding this radius may not result in the most accurate/efficient representation of shapes with sharp edges
        exclude_convex_radius,
        /// Return the shape including the convex radius, Support::GetSupport includes the convex radius if there is one, Support::GetConvexRadius will return 0
        include_convex_radius,
        /// Use both Support::GetSupport add Support::GetConvexRadius to get a support point that matches the original shape as accurately/efficiently as possible
        default,
    };

    base: Shape,

    // Properties
    /// Material assigned to this shape
    material: RefConst(PhysicsMaterial) = .empty,
    /// Uniform density of the interior of the convex object (kg / m^3)
    density: f32 = 1000.0,

    /// Vertex list that forms a unit sphere (protected static sUnitSphereTriangles; Jolt builds it with a static
    /// initializer, Zolt at compile time with the same code)
    pub const unit_sphere_triangles: StaticArray(Vec3, 384) = blk: {
        @setEvalBranchQuota(100_000);
        const level = 2;

        var verts: StaticArray(Vec3, 384) = .empty;
        GetTrianglesContextVertexList.createHalfUnitSphereTop(&verts, level) catch unreachable;
        GetTrianglesContextVertexList.createHalfUnitSphereBottom(&verts, level) catch unreachable;
        break :blk verts;
    };

    /// Constructor (ConvexShape(EShapeSubType inSubType) with a null material, ConvexShape(EShapeSubType inSubType, const
    /// PhysicsMaterial *inMaterial)), called by the concrete classes with their own type `T` (for the vtable)
    pub fn init(comptime T: type, allocator: Allocator, sub_type: ShapeSubType, material: ?*const PhysicsMaterial) ConvexShape {
        return .{ .base = .init(&vtableFor(T).base, allocator, .convex, sub_type), .material = .init(material) };
    }

    /// The part of ConvexShape(EShapeSubType, const ConvexShapeSettings &, ShapeResult &) that reads the settings (the
    /// shape was constructed with `init` before, the result is not used by the base class)
    pub fn initFromSettings(self: *ConvexShape, settings: *const ConvexShapeSettings, result: *ShapeResult) void {
        _ = result;
        self.base.initFromSettings(&settings.base);
        self.material.set(settings.material.get());
        self.density = settings.density;
    }

    /// The vtable of concrete convex class `T`
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// ~ConvexShape
    pub fn destruct(self: *ConvexShape) void {
        self.material.deinit();
    }

    fn getVTable(self: *const ConvexShape) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// Returns an object that provides the GetSupport function for this shape.
    /// mode determines if this support function includes or excludes the convex radius.
    /// of the values returned by the GetSupport function. This improves numerical accuracy of the results.
    /// scale scales this shape in local space.
    pub fn getSupportFunction(self: *const ConvexShape, mode: SupportMode, buffer: *SupportBuffer, scale: Vec3) *const Support {
        return self.getVTable().getSupportFunction(self, mode, buffer, scale);
    }

    /// Material of the shape (before the shape is shared)
    pub fn setMaterial(self: *ConvexShape, material: ?*const PhysicsMaterial) void {
        self.material.set(material);
    }

    /// Material of the shape (non virtual `GetMaterial()`, renamed: the virtual `Shape.getMaterial(sub_shape_id)` has the plain name)
    pub fn getConvexMaterial(self: *const ConvexShape) *const PhysicsMaterial {
        return self.material.get() orelse PhysicsMaterial.default;
    }

    /// Set density of the shape (kg / m^3)
    pub fn setDensity(self: *ConvexShape, density: f32) void {
        self.density = density;
    }

    /// Get density of the shape (kg / m^3)
    pub fn getDensity(self: *const ConvexShape) f32 {
        return self.density;
    }

    /// Upcast (implicit in C++)
    pub fn asShape(self: *const ConvexShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *ConvexShape) *Shape {
        return &self.base;
    }

    /// ConvexShape's implementations of Shape's virtual functions (C++ `ConvexShape::Foo` bodies)
    pub const impl = struct {
        // See Shape::GetSubShapeIDBitsRecursive
        pub fn getSubShapeIDBitsRecursive(self: *const ConvexShape) u32 {
            _ = self;
            return 0; // Convex shapes don't have sub shapes
        }

        // See Shape::GetMaterial
        pub fn getMaterial(self: *const ConvexShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
            std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID
            return self.getConvexMaterial();
        }

        // See Shape::CastRay
        pub fn castRay(self: *const ConvexShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
            // Note: This is a fallback routine, most convex shapes should implement a more performant version!

            // Create support function
            var buffer: SupportBuffer = .{};
            const support = self.getSupportFunction(.include_convex_radius, &buffer, Vec3.one());

            // Cast ray
            var gjk: GJKClosestPoint = .{};
            if (gjk.castRay(ray.origin, ray.direction, PhysicsSettings.default_collision_tolerance, support, &hit.fraction)) {
                hit.sub_shape_id2 = sub_shape_id_creator.getID();
                return true;
            }

            return false;
        }

        // See Shape::CastRay (the collector version)
        pub fn castRayCollector(self: *const ConvexShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
            // Note: This is a fallback routine, most convex shapes should implement a more performant version!

            // Test shape filter
            if (!shape_filter.shouldCollide(&self.base, sub_shape_id_creator.getID()))
                return;

            // First do a normal raycast, limited to the early out fraction
            var hit: RayCastResult = .{};
            hit.fraction = collector.getEarlyOutFraction();
            if (self.base.castRay(ray, sub_shape_id_creator, &hit)) { // Virtual call (C++ unqualified CastRay)
                // Check front side
                if (ray_cast_settings.treat_convex_as_solid or hit.fraction > 0.0) {
                    hit.body_id = TransformedShape.getBodyID(collector.getContext());
                    collector.addHit(&hit);
                }

                // Check if we want back facing hits and the collector still accepts additional hits
                if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and !collector.shouldEarlyOut()) {
                    // Invert the ray, going from the early out fraction back to the fraction where we found our forward hit
                    const start_fraction = math.min(@as(f32, 1.0), collector.getEarlyOutFraction());
                    const delta_fraction = hit.fraction - start_fraction;
                    if (delta_fraction < 0.0) {
                        const inverted_ray = RayCast.init(ray.origin.add(ray.direction.mulScalar(start_fraction)), ray.direction.mulScalar(delta_fraction));

                        // Cast another ray
                        var inverted_hit: RayCastResult = .{};
                        inverted_hit.fraction = 1.0;
                        if (self.base.castRay(inverted_ray, sub_shape_id_creator, &inverted_hit) // Virtual call
                        and inverted_hit.fraction > 0.0) // Ignore hits with fraction 0, this means the ray ends inside the object and we don't want to report it as a back facing hit
                        {
                            // Invert fraction and rescale it to the fraction of the original ray
                            inverted_hit.fraction = hit.fraction + (inverted_hit.fraction - 1.0) * delta_fraction;
                            inverted_hit.body_id = TransformedShape.getBodyID(collector.getContext());
                            collector.addHit(&inverted_hit);
                        }
                    }
                }
            }
        }

        // See: Shape::CollidePoint
        pub fn collidePoint(self: *const ConvexShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
            // Test shape filter
            if (!shape_filter.shouldCollide(&self.base, sub_shape_id_creator.getID()))
                return;

            // First test bounding box
            if (self.base.getLocalBounds().containsVec3(point)) {
                // Create support function
                var buffer: SupportBuffer = .{};
                const support = self.getSupportFunction(.include_convex_radius, &buffer, Vec3.one());

                // Create support function for point
                const point_support: PointConvexSupport = .{ .point = point };

                // Test intersection
                var gjk: GJKClosestPoint = .{};
                var v = point;
                if (gjk.intersects(support, &point_support, PhysicsSettings.default_collision_tolerance, &v))
                    collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
            }
        }

        // See Shape::GetTrianglesStart (placement new of a context that points into itself)
        pub fn getTrianglesStart(self: *const ConvexShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
            _ = box;
            context.emplace(CSGetTrianglesContext).init(self, position_com, rotation, scale);
        }

        // See Shape::GetTrianglesNext
        pub fn getTrianglesNext(self: *const ConvexShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
            std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

            const ctx = context.get(CSGetTrianglesContext);

            const total_num_vertices: usize = @min(@as(usize, max_triangles_requested) * 3, unit_sphere_triangles.len - ctx.current_vertex);
            const vertices = unit_sphere_triangles.constSlice()[ctx.current_vertex .. ctx.current_vertex + total_num_vertices];

            var out: usize = 0;
            var v: usize = 0;
            if (ctx.is_inside_out) {
                // Store triangles flipped
                while (v < vertices.len) : (v += 3) {
                    ctx.local_to_world.mulVec3(ctx.support.getSupport(vertices[v + 0])).storeFloat3(&out_triangle_vertices[out + 0]);
                    ctx.local_to_world.mulVec3(ctx.support.getSupport(vertices[v + 2])).storeFloat3(&out_triangle_vertices[out + 1]);
                    ctx.local_to_world.mulVec3(ctx.support.getSupport(vertices[v + 1])).storeFloat3(&out_triangle_vertices[out + 2]);
                    out += 3;
                }
            } else {
                // Store triangles
                while (v < vertices.len) : (v += 3) {
                    ctx.local_to_world.mulVec3(ctx.support.getSupport(vertices[v + 0])).storeFloat3(&out_triangle_vertices[out + 0]);
                    ctx.local_to_world.mulVec3(ctx.support.getSupport(vertices[v + 1])).storeFloat3(&out_triangle_vertices[out + 1]);
                    ctx.local_to_world.mulVec3(ctx.support.getSupport(vertices[v + 2])).storeFloat3(&out_triangle_vertices[out + 2]);
                    out += 3;
                }
            }

            ctx.current_vertex += total_num_vertices;
            const total_num_triangles = total_num_vertices / 3;

            // Store materials
            if (out_materials) |materials| {
                const material = self.getConvexMaterial();
                for (materials[0..total_num_triangles]) |*m|
                    m.* = material;
            }

            return @intCast(total_num_triangles);
        }

        // See Shape::GetSubmergedVolume
        pub fn getSubmergedVolume(self: *const ConvexShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
            // Calculate total volume
            const abs_scale = scale.abs();
            const extent = self.base.getLocalBounds().getExtent().mul(abs_scale); // Virtual call (C++ unqualified GetLocalBounds)
            const total_volume = 8.0 * extent.getX() * extent.getY() * extent.getZ();

            // Points of the bounding box
            const points = [_]Vec3{
                Vec3.init(-1, -1, -1),
                Vec3.init(1, -1, -1),
                Vec3.init(-1, 1, -1),
                Vec3.init(1, 1, -1),
                Vec3.init(-1, -1, 1),
                Vec3.init(1, -1, 1),
                Vec3.init(-1, 1, 1),
                Vec3.init(1, 1, 1),
            };

            // Faces of the bounding box
            const faces = [_]Face{
                makeFace(0, 2, 3, 1),
                makeFace(4, 6, 2, 0),
                makeFace(4, 5, 7, 6),
                makeFace(1, 3, 7, 5),
                makeFace(2, 6, 7, 3),
                makeFace(0, 1, 5, 4),
            };

            var buffer: [8]PolyhedronSubmergedVolumeCalculator.Point = undefined; // JPH_STACK_ALLOC(8 * sizeof(Point))
            var submerged_vol_calc = PolyhedronSubmergedVolumeCalculator.init(center_of_mass_transform.mul(Mat44.scaleVec3(extent)), .init(&points[0], .{}), 8, surface, &buffer);

            if (submerged_vol_calc.areAllAbove()) {
                // We're above the water
                return .{ .total_volume = total_volume, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
            } else if (submerged_vol_calc.areAllBelow()) {
                // We're fully submerged
                return .{ .total_volume = total_volume, .submerged_volume = total_volume, .center_of_buoyancy = center_of_mass_transform.getTranslation() };
            } else {
                // Calculate submerged volume
                const reference_point_bit = @as(u32, 1) << @intCast(submerged_vol_calc.getReferencePointIdx());
                for (&faces) |*f| {
                    // Test if this face includes the reference point
                    if ((f[4] & reference_point_bit) == 0) {
                        // Triangulate the face (a quad)
                        submerged_vol_calc.addFace(f[0], f[1], f[2]);
                        submerged_vol_calc.addFace(f[0], f[2], f[3]);
                    }
                }

                const r = submerged_vol_calc.getResult();
                return .{ .total_volume = total_volume, .submerged_volume = r.submerged_volume, .center_of_buoyancy = r.center_of_buoyancy };
            }
        }

        // TODO(debug_renderer): DrawGetSupportFunction, DrawGetSupportingFace (JPH_DEBUG_RENDERER)

        // See Shape::SaveBinaryState
        pub fn saveBinaryState(self: *const ConvexShape, stream: StreamOut) void {
            Shape.impl.saveBinaryState(&self.base, stream);

            stream.write(self.density);
        }

        // See Shape::SaveMaterialState
        pub fn saveMaterialState(self: *const ConvexShape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
            for (out_materials.items) |*m| m.deinit();
            out_materials.clearRetainingCapacity();
            try out_materials.append(allocator, self.material.clone());
        }

        // See Shape::RestoreMaterialState
        pub fn restoreMaterialState(self: *ConvexShape, materials: []const PhysicsMaterialRefC) void {
            if (Core.enable_asserts) std.debug.assert(materials.len == 1); // A corrupt stream can violate this
            self.material.set(materials[0].get());
        }

        // See: Shape::RestoreBinaryState
        pub fn restoreBinaryState(self: *ConvexShape, stream: StreamIn) Allocator.Error!void {
            try Shape.impl.restoreBinaryState(&self.base, stream);

            stream.read(&self.density);
        }
    };

    /// A face of the bounding box in GetSubmergedVolume: 4 indices and a bit mask that indicates which indices are used
    const Face = [5]u32;

    /// MAKE_FACE(a, b, c, d): { a, b, c, d, ((1 << a) | (1 << b) | (1 << c) | (1 << d)) } (last int is a bit mask that indicates which indices are used)
    fn makeFace(comptime a: u32, comptime b: u32, comptime c: u32, comptime d: u32) Face {
        return .{ a, b, c, d, ((1 << a) | (1 << b) | (1 << c) | (1 << d)) };
    }

    /// Class for GetTrianglesStart/Next: holds a SupportBuffer and a pointer into it, so it is constructed in place
    const CSGetTrianglesContext = struct {
        support_buffer: SupportBuffer,
        support: *const Support,
        local_to_world: Mat44,
        is_inside_out: bool,
        current_vertex: usize,

        /// In place constructor (CSGetTrianglesContext(inShape, inPositionCOM, inRotation, inScale))
        fn init(self: *CSGetTrianglesContext, shape: *const ConvexShape, position_com: Vec3, rotation: Quat, scale: Vec3) void {
            self.local_to_world = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale));
            self.is_inside_out = ScaleHelpers.isInsideOut(scale);
            self.current_vertex = 0;
            self.support = shape.getSupportFunction(.include_convex_radius, &self.support_buffer, Vec3.one()); // Virtual call
        }

        comptime {
            std.debug.assert(@sizeOf(CSGetTrianglesContext) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
        }
    };

    // Helper functions called by CollisionDispatch (private static in Jolt)

    /// sCollideConvexVsConvex
    fn collideConvexVsConvex(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;

        // Get the shapes
        const shape1 = shape1_in.cast(ConvexShape);
        const shape2 = shape2_in.cast(ConvexShape);

        // Get transforms
        const inverse_transform1 = center_of_mass_transform1.inversedRotationTranslation();
        const transform_2_to_1 = inverse_transform1.mul(center_of_mass_transform2);

        // Get bounding boxes
        var max_separation_distance = collide_shape_settings.max_separation_distance;
        var shape1_bbox = shape1.base.getLocalBounds().scaled(scale1);
        shape1_bbox.expandBy(Vec3.replicate(max_separation_distance));
        const shape2_bbox = shape2.base.getLocalBounds().scaled(scale2);

        // Check if they overlap
        if (!OrientedBox.fromAABox(transform_2_to_1, shape2_bbox).overlapsAABox(shape1_bbox, .{}))
            return;

        // Note: As we don't remember the penetration axis from the last iteration, and it is likely that shape2 is pushed out of
        // collision relative to shape1 by comparing their COM's, we use that as an initial penetration axis: shape2.com - shape1.com
        // This has been seen to improve performance by approx. 1% over using a fixed axis like (1, 0, 0).
        var penetration_axis = transform_2_to_1.getTranslation();

        // Ensure that we do not pass in a near zero penetration axis
        if (penetration_axis.isNearZero(.{}))
            penetration_axis = Vec3.axisX();

        var point1: Vec3 = undefined;
        var point2: Vec3 = undefined;
        var pen_depth: EPAPenetrationDepth = .{};
        var status: EPAPenetrationDepth.Status = undefined;

        // Scope to limit lifetime of SupportBuffer
        {
            // Create support function
            var buffer1_excl_cvx_radius: SupportBuffer = .{};
            var buffer2_excl_cvx_radius: SupportBuffer = .{};
            const shape1_excl_cvx_radius = shape1.getSupportFunction(.exclude_convex_radius, &buffer1_excl_cvx_radius, scale1);
            const shape2_excl_cvx_radius = shape2.getSupportFunction(.exclude_convex_radius, &buffer2_excl_cvx_radius, scale2);

            // Transform shape 2 in the space of shape 1
            const transformed2_excl_cvx_radius = TransformedConvexObject(Support).init(transform_2_to_1, shape2_excl_cvx_radius);

            // Perform GJK step
            status = pen_depth.getPenetrationDepthStepGJK(shape1_excl_cvx_radius, shape1_excl_cvx_radius.getConvexRadius() + max_separation_distance, &transformed2_excl_cvx_radius, shape2_excl_cvx_radius.getConvexRadius(), collide_shape_settings.collision_tolerance, &penetration_axis, &point1, &point2);
        }

        // Check result of collision detection
        switch (status) {
            .colliding => {},

            .not_colliding => return,

            .indeterminate => {
                // Need to run expensive EPA algorithm

                // We know we're overlapping at this point, so we can set the max separation distance to 0.
                // Numerically it is possible that GJK finds that the shapes are overlapping but EPA finds that they're separated.
                // In order to avoid this, we clamp the max separation distance to 1 so that we don't excessively inflate the shape,
                // but we still inflate it enough to avoid the case where EPA misses the collision.
                max_separation_distance = math.min(max_separation_distance, @as(f32, 1.0));

                // Create support function
                var buffer1_incl_cvx_radius: SupportBuffer = .{};
                var buffer2_incl_cvx_radius: SupportBuffer = .{};
                const shape1_incl_cvx_radius = shape1.getSupportFunction(.include_convex_radius, &buffer1_incl_cvx_radius, scale1);
                const shape2_incl_cvx_radius = shape2.getSupportFunction(.include_convex_radius, &buffer2_incl_cvx_radius, scale2);

                // Add separation distance
                const shape1_add_max_separation_distance = AddConvexRadius(Support).init(shape1_incl_cvx_radius, max_separation_distance);

                // Transform shape 2 in the space of shape 1
                const transformed2_incl_cvx_radius = TransformedConvexObject(Support).init(transform_2_to_1, shape2_incl_cvx_radius);

                // Perform EPA step
                if (!pen_depth.getPenetrationDepthStepEPA(&shape1_add_max_separation_distance, &transformed2_incl_cvx_radius, collide_shape_settings.penetration_tolerance, &penetration_axis, &point1, &point2))
                    return;
            },
        }

        // Check if the penetration is bigger than the early out fraction
        const penetration_depth = point2.sub(point1).length() - max_separation_distance;
        if (-penetration_depth >= collector.getEarlyOutFraction())
            return;

        // Correct point1 for the added separation distance
        const penetration_axis_len = penetration_axis.length();
        if (penetration_axis_len > 0.0)
            point1 = point1.sub(penetration_axis.mulScalar(max_separation_distance / penetration_axis_len));

        // Convert to world space
        point1 = center_of_mass_transform1.mulVec3(point1);
        point2 = center_of_mass_transform1.mulVec3(point2);
        const penetration_axis_world = center_of_mass_transform1.multiply3x3(penetration_axis);

        // Create collision result
        var result = CollideShapeResult.init(point1, point2, penetration_axis_world, penetration_depth, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));

        // Gather faces
        if (collide_shape_settings.collect_faces_mode == .collect_faces) {
            // Get supporting face of shape 1
            shape1.base.getSupportingFace(.empty, penetration_axis.negate(), scale1, center_of_mass_transform1, &result.shape1_face);

            // Get supporting face of shape 2
            shape2.base.getSupportingFace(.empty, transform_2_to_1.multiply3x3Transposed(penetration_axis), scale2, center_of_mass_transform2, &result.shape2_face);
        }

        // Notify the collector
        var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
        if (track_narrowphase_stats) track = .init();
        defer if (track_narrowphase_stats) track.deinit();
        collector.addHit(&result);
    }

    /// sCastConvexVsConvex
    fn castConvexVsConvex(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        _ = shape_filter;

        // Only supported for convex shapes
        const cast_shape = shape_cast.shape.cast(ConvexShape);
        const shape = shape_in.cast(ConvexShape);

        // Determine if we want to use the actual shape or a shrunken shape with convex radius
        const support_mode: SupportMode = if (shape_cast_settings.use_shrunken_shape_and_convex_radius) .exclude_convex_radius else .default;

        // Create support function for shape to cast
        var cast_buffer: SupportBuffer = .{};
        const cast_support = cast_shape.getSupportFunction(support_mode, &cast_buffer, shape_cast.scale);

        // Create support function for target shape
        var target_buffer: SupportBuffer = .{};
        const target_support = shape.getSupportFunction(support_mode, &target_buffer, scale);

        // Do a raycast against the result
        var epa: EPAPenetrationDepth = .{};
        var fraction = collector.getEarlyOutFraction();
        var contact_point_a: Vec3 = undefined;
        var contact_point_b: Vec3 = undefined;
        var contact_normal: Vec3 = undefined;
        if (epa.castShape(shape_cast.center_of_mass_start, shape_cast.direction, shape_cast_settings.collision_tolerance, shape_cast_settings.penetration_tolerance, cast_support, target_support, cast_support.getConvexRadius() + shape_cast_settings.extra_convex_radius, target_support.getConvexRadius(), shape_cast_settings.return_deepest_point, &fraction, &contact_point_a, &contact_point_b, &contact_normal) and
            (shape_cast_settings.back_face_mode_convex == .collide_with_back_faces or contact_normal.dot(shape_cast.direction) >= 0.0)) // Test if backfacing
        {
            // Convert to world space
            contact_point_a = center_of_mass_transform2.mulVec3(contact_point_a);
            contact_point_b = center_of_mass_transform2.mulVec3(contact_point_b);
            const contact_normal_world = center_of_mass_transform2.multiply3x3(contact_normal);

            var result = ShapeCastResult.init(fraction, contact_point_a, contact_point_b, contact_normal_world, false, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));

            // Early out if this hit is deeper than the collector's early out value
            if (fraction == 0.0 and -result.base.penetration_depth >= collector.getEarlyOutFraction())
                return;

            // Gather faces
            if (shape_cast_settings.collect_faces_mode == .collect_faces) {
                // Get supporting face of shape 1
                var transform_1_to_2 = shape_cast.center_of_mass_start;
                transform_1_to_2.setTranslation(transform_1_to_2.getTranslation().add(shape_cast.direction.mulScalar(fraction)));
                cast_shape.base.getSupportingFace(.empty, transform_1_to_2.multiply3x3Transposed(contact_normal.negate()), shape_cast.scale, center_of_mass_transform2.mul(transform_1_to_2), &result.base.shape1_face);

                // Get supporting face of shape 2
                shape.base.getSupportingFace(.empty, contact_normal, scale, center_of_mass_transform2, &result.base.shape2_face);
            }

            var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
            if (track_narrowphase_stats) track = .init();
            defer if (track_narrowphase_stats) track.deinit();
            collector.addHit(&result);
        }
    }

    /// Register shape functions with the registry (sRegister, runs at compile time)
    pub fn register(comptime r: *Registry) void {
        for (ShapeFile.convex_sub_shape_types) |s1| {
            for (ShapeFile.convex_sub_shape_types) |s2| {
                r.registerCollideShape(s1, s2, collideConvexVsConvex);
                r.registerCastShape(s1, s2, castConvexVsConvex);
            }
        }
    }
};
