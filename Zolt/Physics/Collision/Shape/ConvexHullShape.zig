//! Port of: Jolt/Physics/Collision/Shape/ConvexHullShape.h, Jolt/Physics/Collision/Shape/ConvexHullShape.cpp
//! Status: complete
//!
//! Follows the porter template of Docs/Zolt/CollisionArchitecture.md (section 2, SphereShape.zig / BoxShape.zig):
//! - `overrides` lists every C++ `override` in header order. The hull is built with Geometry/ConvexHullBuilder.zig in
//!   `initFromSettings` (D3, Jolt's error texts; the `%g` / `%d` / `%u` of Jolt's StringFormat messages are formatted
//!   like C's printf through `StringTools.writeFloatGeneral`, so the texts are identical).
//! - The arrays the shape owns (points, faces, planes, vertex indices) are allocated with the shape's allocator and freed
//!   in `destruct`; the settings own a copy of their points (`ConvexHullShapeSettings.points`, freed with the settings).
//! - The support classes are constructed in place in the caller's SupportBuffer (D9). `HullNoConvex` holds up to 256
//!   shrunk points (4 KB), it is initialized field by field in the buffer (`init(self: *HullNoConvex, ...)`), never
//!   built on the stack and copied.
//! - `CHSGetTrianglesContext` is constructed in the caller's GetTrianglesContext (D10).
//! - JPH_STACK_ALLOC in GetSubmergedVolume is a fixed array of `max_points_in_hull` entries (the number of points is at
//!   most that, checked at creation).
//! - JPH_DEBUG_RENDERER (Draw, DrawShrunkShape, sDrawFaceOutlines, mGeometry, the drawing of the center of buoyancy and
//!   the `inBaseOffset` parameter of GetSubmergedVolume) is not ported yet: TODO(debug_renderer).
//!
//! Signatures that differ from C++:
//! - The two settings constructors `(const Vec3 *inPoints, int inNumPoints, ...)` and `(const Array<Vec3> &, ...)`
//!   are one `init(allocator, points: []const Vec3, .{ .max_convex_radius, .material })` that copies the points (it
//!   allocates, so it returns `Allocator.Error`). `points` is an `std.ArrayList(Vec3)` owned by the settings'
//!   allocator: `try settings.points.append(settings.asShapeSettings().allocator, p)` is Jolt's `mPoints.push_back(p)`.
//! - `GetFaceVertices(inFaceIndex, inMaxVertices, outVertices)` is `getFaceVertices(face_index, out_vertices: []u32)`,
//!   `inMaxVertices` is the length of the slice.
//! - The private `CastRayHelper(inRay, outMinFraction, outMaxFraction) -> bool` returns a `CastRayHelperResult`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const Core = @import("../../../Core/Core.zig");
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const StringTools = @import("../../../Core/StringTools.zig");
const UnorderedMap = @import("../../../Core/UnorderedMap.zig").UnorderedMap;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const ClosestPoint = @import("../../../Geometry/ClosestPoint.zig");
const ConvexHullBuilder = @import("../../../Geometry/ConvexHullBuilder.zig").ConvexHullBuilder;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const math = @import("../../../Math/Math.zig");
const Trigonometry = @import("../../../Math/Trigonometry.zig");
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
const CastRayCollector = ShapeFile.CastRayCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const ConvexShapeFile = @import("ConvexShape.zig");
const ConvexShape = ConvexShapeFile.ConvexShape;
const ConvexShapeSettings = ConvexShapeFile.ConvexShapeSettings;
const PolyhedronSubmergedVolumeCalculator = @import("PolyhedronSubmergedVolumeCalculator.zig").PolyhedronSubmergedVolumeCalculator;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Class that constructs a ConvexHullShape
pub const ConvexHullShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, ConvexHullShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ConvexShapeSettings,
    /// Points to create the hull from. Note that these points don't need to be the vertices of the convex hull, they can contain interior points or points on faces/edges.
    /// (allocated with the allocator of the settings, `asShapeSettings().allocator`)
    points: std.ArrayList(Vec3) = .empty,
    /// Convex radius as supplied by the constructor. Note that during hull creation the convex radius can be made smaller if the value is too big for the hull.
    max_convex_radius: f32 = 0.0,
    /// Maximum distance between the shrunk hull + convex radius and the actual hull.
    max_error_convex_radius: f32 = 0.05,
    /// Points are allowed this far outside of the hull (increasing this yields a hull with less vertices). Note that the actual used value can be larger if the points of the hull are far apart.
    hull_tolerance: f32 = 1.0e-3,

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) ConvexHullShapeSettings {
        return .{ .base = .init(ConvexHullShapeSettings, allocator, null) };
    }

    /// Create a convex hull from points and maximum convex radius max_convex_radius, the radius is automatically lowered if the hull requires it.
    /// (internally this will be subtracted so the total size will not grow with the convex radius).
    /// The points are copied (both C++ constructors: `(const Vec3 *inPoints, int inNumPoints, ...)` and `(const Array<Vec3> &inPoints, ...)`).
    pub fn init(allocator: Allocator, points: []const Vec3, opts: struct { max_convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!ConvexHullShapeSettings {
        var point_list: std.ArrayList(Vec3) = .empty;
        try point_list.appendSlice(allocator, points);
        return .{ .base = .init(ConvexHullShapeSettings, allocator, opts.material), .points = point_list, .max_convex_radius = opts.max_convex_radius };
    }

    /// new ConvexHullShapeSettings(...): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, points: []const Vec3, opts: struct { max_convex_radius: f32 = PhysicsSettings.default_convex_radius, material: ?*const PhysicsMaterial = null }) Allocator.Error!*ConvexHullShapeSettings {
        const self = try allocator.create(ConvexHullShapeSettings);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, points, .{ .max_convex_radius = opts.max_convex_radius, .material = opts.material });
        return self;
    }

    /// ~ConvexHullShapeSettings (frees the points)
    pub fn destruct(self: *ConvexHullShapeSettings) void {
        self.points.deinit(self.base.base.allocator);
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *ConvexHullShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *ConvexHullShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *ConvexHullShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(ConvexHullShape, self, allocator);
    }
};

/// A convex hull
pub const ConvexHullShape = struct {
    /// Concrete class: `Shape.cast(ConvexHullShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .convex_hull;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .restoreBinaryState };

    /// Maximum amount of points supported in a convex hull. Note that while constructing a hull, interior points are discarded so you can provide more points.
    /// The ConvexHullShapeSettings::Create function will return an error when too many points are provided.
    pub const max_points_in_hull = 256;

    pub const Face = extern struct {
        /// First index in vertex_idx to use
        first_vertex: u16,
        /// Number of vertices in the vertex_idx to use
        num_vertices: u16 = 0,

        comptime {
            std.debug.assert(@sizeOf(Face) == 4); // Unexpected size
            std.debug.assert(@alignOf(Face) == 2); // Unexpected alignment
        }
    };

    pub const Point = extern struct {
        /// Position of vertex
        position: Vec3,
        /// Number of faces in the face array below
        num_faces: i32 = 0,
        /// Indices of 3 neighboring faces with the biggest difference in normal (used to shift vertices for convex radius)
        faces: [3]i32 = .{ -1, -1, -1 },

        comptime {
            std.debug.assert(@sizeOf(Point) == 32); // Unexpected size
            std.debug.assert(@alignOf(Point) == 16); // Unexpected alignment (JPH_VECTOR_ALIGNMENT)
        }
    };

    base: ConvexShape,
    /// Center of mass of this convex hull (uninitialized in Jolt)
    center_of_mass: Vec3 = Vec3.zero(),
    /// Inertia matrix assuming density is 1 (needs to be multiplied by density) (uninitialized in Jolt)
    inertia: Mat44 = Mat44.zero(),
    /// Local bounding box for the convex hull
    local_bounds: AABox = .empty,
    /// Points on the convex hull surface
    points: std.ArrayList(Point) = .empty,
    /// Faces of the convex hull surface
    faces: std.ArrayList(Face) = .empty,
    /// Planes for the faces (1-on-1 with faces array, separate because they need to be 16 byte aligned)
    planes: std.ArrayList(Plane) = .empty,
    /// A list of vertex indices (indexing in points) for each of the faces
    vertex_idx: std.ArrayList(u8) = .empty,
    /// Convex radius
    convex_radius: f32 = 0.0,
    /// Total volume of the convex hull (uninitialized in Jolt)
    volume: f32 = 0.0,
    /// Radius of the biggest sphere that fits entirely in the convex hull
    inner_radius: f32 = math.flt_max,

    // TODO(debug_renderer): mutable DebugRenderer::GeometryRef mGeometry

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// ConvexHullShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) ConvexHullShape {
        return .{ .base = .init(ConvexHullShape, allocator, shape_sub_type, null) };
    }

    /// ConvexHullShape(const ConvexHullShapeSettings &inSettings, ShapeResult &outResult): base part first, then the C++ body.
    /// On the stack: `var shape = ConvexHullShape.initDefault(allocator); try shape.initFromSettings(...)`, then
    /// `asShape().setEmbedded()` and finally `asShapeMut().deinit()` (frees the arrays).
    pub fn initFromSettings(self: *ConvexHullShape, settings: *const ConvexHullShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base, result);
        self.convex_radius = settings.max_convex_radius;

        const shape_allocator = self.base.base.allocator;
        const settings_points = settings.points.items;

        // Check convex radius
        if (self.convex_radius < 0.0) {
            result.setError("Invalid convex radius");
            return;
        }

        // Build convex hull
        var builder = ConvexHullBuilder.init(allocator, settings_points);
        defer builder.deinit();
        const build_result = try builder.initialize(max_points_in_hull, settings.hull_tolerance);
        if (build_result.result != .success and build_result.result != .max_vertices_reached) {
            result.setError(build_result.error_message.?);
            return;
        }
        const builder_faces = builder.getFaces();

        // Check the consistency of the resulting hull if we fully built it
        if (build_result.result == .success) {
            const max_error = builder.determineMaxError();
            if (max_error.max_error > 4.0 * math.max(max_error.coplanar_distance, settings.hull_tolerance)) { // Coplanar distance could be bigger than the allowed tolerance if the points are far apart
                result.setErrorFmt("Hull building failed, point {d} had an error of {f} (relative to tolerance: {f})", .{ max_error.max_error_position_idx, PrintfG{ .value = max_error.max_error }, PrintfG{ .value = max_error.max_error / settings.hull_tolerance } });
                return;
            }
        }

        // Calculate center of mass and volume
        const center_of_mass_and_volume = builder.getCenterOfMassAndVolume();
        self.center_of_mass = center_of_mass_and_volume.center_of_mass;
        self.volume = center_of_mass_and_volume.volume;

        // Calculate covariance matrix
        // See:
        // - Why the inertia tensor is the inertia tensor - Jonathan Blow (http://number-none.com/blow/inertia/deriving_i.html)
        // - How to find the inertia tensor (or other mass properties) of a 3D solid body represented by a triangle mesh (Draft) - Jonathan Blow, Atman J Binstock (http://number-none.com/blow/inertia/bb_inertia.doc)
        const one_60th = @as(f32, 1.0) / 60.0;
        const one_120th = @as(f32, 1.0) / 120.0;
        const covariance_canonical = Mat44.init(Vec4.init(one_60th, one_120th, one_120th, 0), Vec4.init(one_120th, one_60th, one_120th, 0), Vec4.init(one_120th, one_120th, one_60th, 0), Vec4.init(0, 0, 0, 1));
        var covariance_matrix = Mat44.zero();
        for (builder_faces) |f| {
            // Fourth point of the tetrahedron is at the center of mass, we subtract it from the other points so we get a tetrahedron with one vertex at zero
            // The first point on the face will be used to form a triangle fan
            var e = f.first_edge.?;
            const v1 = settings_points[@intCast(e.start_idx)].sub(self.center_of_mass);

            // Get the 2nd point
            e = e.next_edge.?;
            var v2 = settings_points[@intCast(e.start_idx)].sub(self.center_of_mass);

            // Loop over the triangle fan
            e = e.next_edge.?;
            while (e != f.first_edge.?) : (e = e.next_edge.?) {
                const v3 = settings_points[@intCast(e.start_idx)].sub(self.center_of_mass);

                // Affine transform that transforms a unit tetrahedron (with vertices (0, 0, 0), (1, 0, 0), (0, 1, 0) and (0, 0, 1) to this tetrahedron
                const a = Mat44.init(Vec4.fromVec3W(v1, 0), Vec4.fromVec3W(v2, 0), Vec4.fromVec3W(v3, 0), Vec4.init(0, 0, 0, 1));

                // Calculate covariance matrix for this tetrahedron
                const det_a = a.getDeterminant3x3();
                const c = a.mul(covariance_canonical).mul(a.transposed()).mulScalar(det_a);

                // Add it
                covariance_matrix = covariance_matrix.add(c);

                // Prepare for next triangle
                v2 = v3;
            }
        }

        // Calculate inertia matrix assuming density is 1, note that element (3, 3) is garbage
        self.inertia = Mat44.identity().mulScalar(covariance_matrix.get(0, 0) + covariance_matrix.get(1, 1) + covariance_matrix.get(2, 2)).sub(covariance_matrix);

        // Convert polygons from the builder to our internal representation
        const VtxMap = UnorderedMap(i32, u8, .{});
        var vertex_map: VtxMap = .empty;
        defer vertex_map.deinit(allocator);
        try vertex_map.ensureTotalCapacity(allocator, @intCast(settings_points.len));
        for (builder_faces) |builder_face| {
            // Determine where the vertices go
            std.debug.assert(self.vertex_idx.items.len <= 0xFFFF);
            const first_vertex: u16 = @truncate(self.vertex_idx.items.len);
            var num_vertices: u16 = 0;

            // Loop over vertices in face
            var edge = builder_face.first_edge.?;
            while (true) {
                // Remap to new index, not all points in the original input set are required to form the hull
                var new_idx: u8 = undefined;
                const original_idx = edge.start_idx;
                if (vertex_map.find(original_idx)) |m| {
                    // Found, reuse
                    new_idx = m.value;
                } else {
                    // This is a new point
                    // Make relative to center of mass
                    const p = settings_points[@intCast(original_idx)].sub(self.center_of_mass);

                    // Update local bounds
                    self.local_bounds.encapsulateVec3(p);

                    // Add to point list
                    if (Core.enable_asserts) std.debug.assert(self.points.items.len <= 0xff); // Jolt's release build truncates, the hull is rejected below
                    new_idx = @truncate(self.points.items.len);
                    try self.points.append(shape_allocator, .{ .position = p });
                    (try vertex_map.getOrPutValue(allocator, original_idx, 0)).* = new_idx;
                }

                // Append to vertex list
                std.debug.assert(self.vertex_idx.items.len < 0xffff);
                try self.vertex_idx.append(shape_allocator, new_idx);
                num_vertices += 1;

                edge = edge.next_edge.?;
                if (edge == builder_face.first_edge.?) break;
            }

            // Add face
            try self.faces.append(shape_allocator, .{ .first_vertex = first_vertex, .num_vertices = num_vertices });

            // Add plane
            const plane = Plane.fromPointAndNormal(builder_face.centroid.sub(self.center_of_mass), builder_face.normal.normalized());
            try self.planes.append(shape_allocator, plane);
        }

        // Test if GetSupportFunction can support this many points
        if (self.points.items.len > max_points_in_hull) {
            result.setErrorFmt("Internal error: Too many points in hull ({d}), max allowed {d}", .{ @as(u32, @intCast(self.points.items.len)), @as(i32, max_points_in_hull) });
            return;
        }

        var faces: std.ArrayList(i32) = .empty;
        defer faces.deinit(allocator);
        for (0..self.points.items.len) |p| {
            // For each point, find faces that use the point
            faces.clearRetainingCapacity();
            for (self.faces.items, 0..) |face, f| {
                for (0..face.num_vertices) |v| {
                    if (self.vertex_idx.items[face.first_vertex + v] == p) {
                        try faces.append(allocator, @intCast(f));
                        break;
                    }
                }
            }

            if (faces.items.len < 2) {
                result.setError("A point must be connected to 2 or more faces!");
                return;
            }

            // Find the 3 normals that form the largest tetrahedron
            // The largest tetrahedron we can get is ((1, 0, 0) x (0, 1, 0)) . (0, 0, 1) = 1, if the volume is only 5% of that,
            // the three vectors are too coplanar and we fall back to using only 2 plane normals
            var biggest_volume: f32 = 0.05;
            var best3 = [3]i32{ -1, -1, -1 };

            // When using 2 normals, we get the two with the biggest angle between them with a minimal difference of 1 degree
            // otherwise we fall back to just using 1 plane normal
            var smallest_dot = Trigonometry.cos(math.degreesToRadians(1.0));
            var best2 = [2]i32{ -1, -1 };

            for (0..faces.items.len) |face1| {
                const normal1 = self.planes.items[@intCast(faces.items[face1])].getNormal();
                for (face1 + 1..faces.items.len) |face2| {
                    const normal2 = self.planes.items[@intCast(faces.items[face2])].getNormal();
                    const cross = normal1.cross(normal2);

                    // Determine the 2 face normals that are most apart
                    const dot = normal1.dot(normal2);
                    if (dot < smallest_dot) {
                        smallest_dot = dot;
                        best2[0] = faces.items[face1];
                        best2[1] = faces.items[face2];
                    }

                    // Determine the 3 face normals that form the largest tetrahedron
                    for (face2 + 1..faces.items.len) |face3| {
                        const normal3 = self.planes.items[@intCast(faces.items[face3])].getNormal();
                        const volume = @abs(cross.dot(normal3));
                        if (volume > biggest_volume) {
                            biggest_volume = volume;
                            best3[0] = faces.items[face1];
                            best3[1] = faces.items[face2];
                            best3[2] = faces.items[face3];
                        }
                    }
                }
            }

            // If we didn't find 3 planes, use 2, if we didn't find 2 use 1
            var chosen: StaticArray(i32, 3) = .empty;
            if (best3[0] != -1) {
                chosen = .fromSlice(&best3);
            } else if (best2[0] != -1) {
                chosen = .fromSlice(&best2);
            } else {
                chosen.append(faces.items[0]);
            }

            // Copy the faces to the points buffer
            const point = &self.points.items[p];
            point.num_faces = @intCast(chosen.len);
            for (chosen.constSlice(), 0..) |face, i|
                point.faces[i] = face;
        }

        // If the convex radius is already zero, there's no point in further reducing it
        if (self.convex_radius > 0.0) {
            // Find out how thin the hull is by walking over all planes and checking the thickness of the hull in that direction
            var min_size: f32 = math.flt_max;
            for (self.planes.items) |plane| {
                // Take the point that is furthest away from the plane as thickness of this hull
                var max_dist: f32 = 0.0;
                for (self.points.items) |point| {
                    const dist = -plane.signedDistance(point.position); // Point is always behind plane, so we need to negate
                    if (dist > max_dist)
                        max_dist = dist;
                }
                min_size = math.min(min_size, max_dist);
            }

            // We need to fit in 2x the convex radius in min_size, so reduce the convex radius if it's bigger than that
            self.convex_radius = math.min(self.convex_radius, 0.5 * min_size);
        }

        // Now walk over all points and see if we have to further reduce the convex radius because of sharp edges
        if (self.convex_radius > 0.0) {
            for (self.points.items) |point| {
                if (point.num_faces != 1) { // If we have a single face, shifting back is easy and we don't need to reduce the convex radius
                    // Get first two planes
                    const p1 = self.planes.items[@intCast(point.faces[0])];
                    const p2 = self.planes.items[@intCast(point.faces[1])];
                    var p3: Plane = undefined;
                    var offset_mask: Vec3 = undefined;

                    if (point.num_faces == 3) {
                        // Get third plane
                        p3 = self.planes.items[@intCast(point.faces[2])];

                        // All 3 planes will be offset by the convex radius
                        offset_mask = Vec3.replicate(1);
                    } else {
                        // Third plane has normal perpendicular to the other two planes and goes through the vertex position
                        std.debug.assert(point.num_faces == 2);
                        p3 = Plane.fromPointAndNormal(point.position, p1.getNormal().cross(p2.getNormal()));

                        // Only the first and 2nd plane will be offset, the 3rd plane is only there to guide the intersection point
                        offset_mask = Vec3.init(1, 1, 0);
                    }

                    // Plane equation: point . normal + constant = 0
                    // Offsetting the plane backwards with convex radius r: point . normal + constant + r = 0
                    // To find the intersection 'point' of 3 planes we solve:
                    // |n1x n1y n1z| |x|     | r + c1 |
                    // |n2x n2y n2z| |y| = - | r + c2 | <=> n point = -r (1, 1, 1) - (c1, c2, c3)
                    // |n3x n3y n3z| |z|     | r + c3 |
                    // Where point = (x, y, z), n1x is the x component of the first plane, c1 = plane constant of plane 1, etc.
                    // The relation between how much the intersection point shifts as a function of r is: -r * n^-1 (1, 1, 1) = r * offset
                    // Where offset = -n^-1 (1, 1, 1) or -n^-1 (1, 1, 0) in case only the first 2 planes are offset
                    // The error that is introduced by a convex radius r is: error = r * |offset| - r
                    // So the max convex radius given error is: r = error / (|offset| - 1)
                    const n = Mat44.init(Vec4.fromVec3W(p1.getNormal(), 0), Vec4.fromVec3W(p2.getNormal(), 0), Vec4.fromVec3W(p3.getNormal(), 0), Vec4.init(0, 0, 0, 1)).transposed();
                    const det_n = n.getDeterminant3x3();
                    if (det_n == 0.0) {
                        // If the determinant is zero, the matrix is not invertible so no solution exists to move the point backwards and we have to choose a convex radius of zero
                        self.convex_radius = 0.0;
                        break;
                    }
                    const adj_n = n.adjointed3x3();
                    const offset = adj_n.mulVec3(offset_mask).divScalar(det_n).length();
                    if (Core.enable_asserts) std.debug.assert(offset > 1.0); // Degenerate hulls (planes with NaN normals) violate this, Jolt's release build continues
                    const max_convex_radius = settings.max_error_convex_radius / (offset - 1.0);
                    self.convex_radius = math.min(self.convex_radius, max_convex_radius);
                }
            }
        }

        // Calculate the inner radius by getting the minimum distance from the origin to the planes of the hull
        self.inner_radius = math.flt_max;
        for (self.planes.items) |p|
            self.inner_radius = math.min(self.inner_radius, -p.getConstant());
        self.inner_radius = math.max(@as(f32, 0.0), self.inner_radius); // Clamp against zero, this should do nothing as the shape is centered around the center of mass but for flat convex hulls there may be numerical round off issues

        result.set(.init(self.asShapeMut()));
    }

    /// ~ConvexHullShape (frees the arrays)
    pub fn destruct(self: *ConvexHullShape) void {
        const allocator = self.base.base.allocator;
        self.points.deinit(allocator);
        self.faces.deinit(allocator);
        self.planes.deinit(allocator);
        self.vertex_idx.deinit(allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const ConvexHullShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *ConvexHullShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get the convex radius of this convex hull
    pub fn getConvexRadius(self: *const ConvexHullShape) f32 {
        return self.convex_radius;
    }

    /// Get the planes of this convex hull
    pub fn getPlanes(self: *const ConvexHullShape) []const Plane {
        return self.planes.items;
    }

    /// Get the number of vertices in this convex hull
    pub fn getNumPoints(self: *const ConvexHullShape) u32 {
        return @intCast(self.points.items.len);
    }

    /// Get a vertex of this convex hull relative to the center of mass
    pub fn getPoint(self: *const ConvexHullShape, index: u32) Vec3 {
        return self.points.items[index].position;
    }

    /// Get the number of faces in this convex hull
    pub fn getNumFaces(self: *const ConvexHullShape) u32 {
        return @intCast(self.faces.items.len);
    }

    /// Get the number of vertices in a face
    pub fn getNumVerticesInFace(self: *const ConvexHullShape, face_index: u32) u32 {
        return self.faces.items[face_index].num_vertices;
    }

    /// Get the vertices indices of a face
    /// @param face_index Index of the face.
    /// @param out_vertices Array of vertices indices (its length is the maximum number of vertices to return), the vertices are returned in counter clockwise order and the positions can be obtained using getPoint(index).
    /// @return Number of vertices in face, if this is bigger than out_vertices.len, not all vertices were retrieved.
    pub fn getFaceVertices(self: *const ConvexHullShape, face_index: u32, out_vertices: []u32) u32 {
        const face = self.faces.items[face_index];
        const first_vertex = self.vertex_idx.items[face.first_vertex..];
        const num_vertices = @min(@as(usize, face.num_vertices), out_vertices.len);
        for (0..num_vertices) |i|
            out_vertices[i] = first_vertex[i];
        return face.num_vertices;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const ConvexHullShape) Vec3 {
        return self.center_of_mass;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const ConvexHullShape) AABox {
        return self.local_bounds;
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const ConvexHullShape) f32 {
        return self.inner_radius;
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const ConvexHullShape) MassProperties {
        var p: MassProperties = .{};

        const density = self.base.getDensity();

        // Calculate mass
        p.mass = density * self.volume;

        // Calculate inertia matrix
        p.inertia = self.inertia.mulScalar(density);
        p.inertia.set(3, 3, 1.0);

        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const ConvexHullShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        const first_plane = self.planes.items[0];
        var best_normal = first_plane.getNormal();
        var best_dist = @abs(first_plane.signedDistance(local_surface_position));

        // Find the face that has the shortest distance to the surface point
        for (1..self.faces.items.len) |i| {
            const plane = self.planes.items[i];
            const plane_normal = plane.getNormal();
            const dist = @abs(plane.signedDistance(local_surface_position));
            if (dist < best_dist) {
                best_dist = dist;
                best_normal = plane_normal;
            }
        }

        return best_normal;
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const ConvexHullShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty()); // Invalid subshape ID

        const inv_scale = scale.reciprocal();

        // Need to transform the plane normals using scale
        // Transforming a direction with matrix M is done through multiplying by (M^-1)^T
        // In this case M is a diagonal matrix with the scale vector, so we need to multiply our normal by 1 / scale and renormalize afterwards
        const plane0_normal = inv_scale.mul(self.planes.items[0].getNormal());
        var best_dot = plane0_normal.dot(direction) / plane0_normal.length();
        var best_face_idx: usize = 0;

        for (1..self.planes.items.len) |i| {
            const plane_normal = inv_scale.mul(self.planes.items[i].getNormal());
            const dot = plane_normal.dot(direction) / plane_normal.length();
            if (dot < best_dot) {
                best_dot = dot;
                best_face_idx = i;
            }
        }

        // Get vertices
        const best_face = self.faces.items[best_face_idx];
        const vertices = self.vertex_idx.items[best_face.first_vertex..][0..best_face.num_vertices];

        // If we have more than 1/2 the capacity of out_vertices worth of vertices, we start skipping vertices (note we can't fill the buffer completely since extra edges will be generated by clipping).
        // TODO: This really needs a better algorithm to determine which vertices are important!
        const max_vertices_to_return: i32 = Shape.SupportingFace.capacity / 2;
        const delta_vtx: i32 = @divTrunc(@as(i32, best_face.num_vertices) + max_vertices_to_return, max_vertices_to_return);

        // Calculate transform with scale
        const transform = center_of_mass_transform.preScaled(scale);

        if (ScaleHelpers.isInsideOut(scale)) {
            // Flip winding of supporting face
            var v: i32 = @as(i32, best_face.num_vertices) - 1;
            while (v >= 0) : (v -= delta_vtx)
                out_vertices.append(transform.mulVec3(self.points.items[vertices[@intCast(v)]].position));
        } else {
            // Normal winding of supporting face
            var v: i32 = 0;
            while (v < best_face.num_vertices) : (v += delta_vtx)
                out_vertices.append(transform.mulVec3(self.points.items[vertices[@intCast(v)]].position));
        }
    }

    // See ConvexShape::GetSupportFunction
    pub fn getSupportFunction(self: *const ConvexHullShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        // If there's no convex radius, we don't need to shrink the hull
        if (self.convex_radius == 0.0) {
            if (ScaleHelpers.isNotScaled(scale)) {
                const support = buffer.emplace(HullWithConvex);
                support.* = .init(self);
                return &support.base;
            } else {
                const support = buffer.emplace(HullWithConvexScaled);
                support.* = .init(self, scale);
                return &support.base;
            }
        }

        switch (mode) {
            .include_convex_radius, .default => {
                if (ScaleHelpers.isNotScaled(scale)) {
                    const support = buffer.emplace(HullWithConvex);
                    support.* = .init(self);
                    return &support.base;
                } else {
                    const support = buffer.emplace(HullWithConvexScaled);
                    support.* = .init(self, scale);
                    return &support.base;
                }
            },

            .exclude_convex_radius => {
                if (ScaleHelpers.isNotScaled(scale)) {
                    // Create support function
                    const hull = buffer.emplace(HullNoConvex);
                    hull.init(self.convex_radius);
                    const transformed_points = hull.getPoints();
                    std.debug.assert(self.points.items.len <= max_points_in_hull); // Not enough space, this should have been caught during shape creation!

                    for (self.points.items) |point| {
                        var new_point: Vec3 = undefined;

                        if (point.num_faces == 1) {
                            // Simply shift back by the convex radius using our 1 plane
                            new_point = point.position.sub(self.planes.items[@intCast(point.faces[0])].getNormal().mulScalar(self.convex_radius));
                        } else {
                            // Get first two planes and offset inwards by convex radius
                            const p1 = self.planes.items[@intCast(point.faces[0])].offset(-self.convex_radius);
                            const p2 = self.planes.items[@intCast(point.faces[1])].offset(-self.convex_radius);
                            var p3: Plane = undefined;

                            if (point.num_faces == 3) {
                                // Get third plane and offset inwards by convex radius
                                p3 = self.planes.items[@intCast(point.faces[2])].offset(-self.convex_radius);
                            } else {
                                // Third plane has normal perpendicular to the other two planes and goes through the vertex position
                                std.debug.assert(point.num_faces == 2);
                                p3 = Plane.fromPointAndNormal(point.position, p1.getNormal().cross(p2.getNormal()));
                            }

                            // Find intersection point between the three planes
                            new_point = Plane.intersectPlanes(p1, p2, p3) orelse
                                // Fallback: Just push point back using the first plane
                                point.position.sub(p1.getNormal().mulScalar(self.convex_radius));
                        }

                        // Add point
                        transformed_points.append(new_point);
                    }

                    return &hull.base;
                } else {
                    // Calculate scaled convex radius
                    const convex_radius = ScaleHelpers.scaleConvexRadius(self.convex_radius, scale);

                    // Create new support function
                    const hull = buffer.emplace(HullNoConvex);
                    hull.init(convex_radius);
                    const transformed_points = hull.getPoints();
                    std.debug.assert(self.points.items.len <= max_points_in_hull); // Not enough space, this should have been caught during shape creation!

                    // Precalculate inverse scale
                    const inv_scale = scale.reciprocal();

                    for (self.points.items) |point| {
                        // Calculate scaled position
                        const pos = scale.mul(point.position);

                        // Transform normals for plane 1 with scale
                        const n1 = inv_scale.mul(self.planes.items[@intCast(point.faces[0])].getNormal()).normalized();

                        var new_point: Vec3 = undefined;

                        if (point.num_faces == 1) {
                            // Simply shift back by the convex radius using our 1 plane
                            new_point = pos.sub(n1.mulScalar(convex_radius));
                        } else {
                            // Transform normals for plane 2 with scale
                            const n2 = inv_scale.mul(self.planes.items[@intCast(point.faces[1])].getNormal()).normalized();

                            // Get first two planes and offset inwards by convex radius
                            const p1 = Plane.fromPointAndNormal(pos, n1).offset(-convex_radius);
                            const p2 = Plane.fromPointAndNormal(pos, n2).offset(-convex_radius);
                            var p3: Plane = undefined;

                            if (point.num_faces == 3) {
                                // Transform last normal with scale
                                const n3 = inv_scale.mul(self.planes.items[@intCast(point.faces[2])].getNormal()).normalized();

                                // Get third plane and offset inwards by convex radius
                                p3 = Plane.fromPointAndNormal(pos, n3).offset(-convex_radius);
                            } else {
                                // Third plane has normal perpendicular to the other two planes and goes through the vertex position
                                std.debug.assert(point.num_faces == 2);
                                p3 = Plane.fromPointAndNormal(pos, n1.cross(n2));
                            }

                            // Find intersection point between the three planes
                            new_point = Plane.intersectPlanes(p1, p2, p3) orelse
                                // Fallback: Just push point back using the first plane
                                pos.sub(n1.mulScalar(convex_radius));
                        }

                        // Add point
                        transformed_points.append(new_point);
                    }

                    return &hull.base;
                }
            },
        }
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const ConvexHullShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        // Trivially calculate total volume
        const abs_scale = scale.abs();
        const total_volume = self.volume * abs_scale.getX() * abs_scale.getY() * abs_scale.getZ();

        // Check if shape has been scaled inside out
        const is_inside_out = ScaleHelpers.isInsideOut(scale);

        // Convert the points to world space and determine the distance to the surface
        const num_points: u32 = @intCast(self.points.items.len);
        var buffer: [max_points_in_hull]PolyhedronSubmergedVolumeCalculator.Point = undefined; // JPH_STACK_ALLOC(num_points * sizeof(Point)), at most max_points_in_hull points
        var submerged_vol_calc = PolyhedronSubmergedVolumeCalculator.init(center_of_mass_transform.mul(Mat44.scaleVec3(scale)), .init(&self.points.items[0].position, .{ .stride = @sizeOf(Point) }), num_points, surface, buffer[0..num_points]);

        if (submerged_vol_calc.areAllAbove()) {
            // We're above the water
            return .{ .total_volume = total_volume, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
        } else if (submerged_vol_calc.areAllBelow()) {
            // We're fully submerged
            return .{ .total_volume = total_volume, .submerged_volume = total_volume, .center_of_buoyancy = center_of_mass_transform.getTranslation() };
        } else {
            // Calculate submerged volume
            const reference_point_idx = submerged_vol_calc.getReferencePointIdx();
            for (self.faces.items) |f| {
                const vertices = self.vertex_idx.items[f.first_vertex..][0..f.num_vertices];

                // If any of the vertices of this face are the reference point, the volume will be zero so we can skip this face
                var degenerate = false;
                for (vertices) |v| {
                    if (v == reference_point_idx) {
                        degenerate = true;
                        break;
                    }
                }
                if (degenerate)
                    continue;

                // Triangulate the face
                const i_1: u32 = vertices[0];
                if (is_inside_out) {
                    // Reverse winding
                    for (2..vertices.len) |v| {
                        const i_2: u32 = vertices[v - 1];
                        const i_3: u32 = vertices[v];
                        submerged_vol_calc.addFace(i_1, i_3, i_2);
                    }
                } else {
                    // Normal winding
                    for (2..vertices.len) |v| {
                        const i_2: u32 = vertices[v - 1];
                        const i_3: u32 = vertices[v];
                        submerged_vol_calc.addFace(i_1, i_2, i_3);
                    }
                }
            }

            // Get the results
            const r = submerged_vol_calc.getResult();

            // TODO(debug_renderer): Draw center of buoyancy (sDrawSubmergedVolumes)

            return .{ .total_volume = total_volume, .submerged_volume = r.submerged_volume, .center_of_buoyancy = r.center_of_buoyancy };
        }
    }

    // TODO(debug_renderer): Draw, DrawShrunkShape and sDrawFaceOutlines (JPH_DEBUG_RENDERER)

    /// The result of castRayHelper (Jolt's return value and the outMinFraction / outMaxFraction out parameters, which Jolt
    /// leaves unset on one of the paths that return false)
    const CastRayHelperResult = struct {
        hit: bool,
        min_fraction: f32 = 0.0,
        max_fraction: f32 = 1.0 + math.flt_epsilon,
    };

    /// Helper function that returns the min and max fraction along the ray that hits the convex hull. Returns false if there is no hit.
    fn castRayHelper(self: *const ConvexHullShape, ray: RayCast) CastRayHelperResult {
        if (self.faces.items.len == 2) {
            // If we have only 2 faces, we're a flat convex hull and we need to test edges instead of planes

            // Check if plane is parallel to ray
            const p = self.planes.items[0];
            const plane_normal = p.getNormal();
            const direction_projection = ray.direction.dot(plane_normal);
            if (@abs(direction_projection) >= 1.0e-12) {
                // Calculate intersection point
                const distance_to_plane = ray.origin.dot(plane_normal) + p.getConstant();
                const fraction = -distance_to_plane / direction_projection;
                if (fraction < 0.0 or fraction > 1.0) {
                    // Does not hit plane, no hit
                    return .{ .hit = false, .min_fraction = 0.0, .max_fraction = 1.0 + math.flt_epsilon };
                }
                const intersection_point = ray.origin.add(ray.direction.mulScalar(fraction));

                // Test all edges to see if point is inside polygon
                const f = self.faces.items[0];
                const first_vtx = f.first_vertex;
                const end_vtx = first_vtx + f.num_vertices;
                var p1 = self.points.items[self.vertex_idx.items[end_vtx]].position; // Jolt reads *end_vtx: the first vertex of the second face (the last vertex of this face for a hull from the 2D builder)
                for (self.vertex_idx.items[first_vtx..end_vtx]) |v| {
                    const p2 = self.points.items[v].position;
                    if (p2.sub(p1).cross(intersection_point.sub(p1)).dot(plane_normal) < 0.0) {
                        // Outside polygon, no hit
                        return .{ .hit = false, .min_fraction = 0.0, .max_fraction = 1.0 + math.flt_epsilon };
                    }
                    p1 = p2;
                }

                // Inside polygon, a hit
                return .{ .hit = true, .min_fraction = fraction, .max_fraction = fraction };
            } else {
                // Parallel ray doesn't hit
                return .{ .hit = false, .min_fraction = 0.0, .max_fraction = 1.0 + math.flt_epsilon };
            }
        } else {
            // Clip ray against all planes
            var fractions_set: u32 = 0;
            var all_inside = true;
            var min_fraction: f32 = 0.0;
            var max_fraction: f32 = 1.0 + math.flt_epsilon;
            for (self.planes.items) |p| {
                // Check if the ray origin is behind this plane
                const plane_normal = p.getNormal();
                const distance_to_plane = ray.origin.dot(plane_normal) + p.getConstant();
                const is_outside = distance_to_plane > 0.0;
                all_inside = all_inside and !is_outside;

                // Check if plane is parallel to ray
                const direction_projection = ray.direction.dot(plane_normal);
                if (@abs(direction_projection) >= 1.0e-12) {
                    // Get intersection fraction between ray and plane
                    const fraction = -distance_to_plane / direction_projection;

                    // Update interval of ray that is inside the hull
                    if (direction_projection < 0.0) {
                        min_fraction = math.max(fraction, min_fraction);
                        fractions_set |= 1;
                    } else {
                        max_fraction = math.min(fraction, max_fraction);
                        fractions_set |= 2;
                    }
                } else if (is_outside)
                    return .{ .hit = false }; // Outside the plane and parallel, no hit! (Jolt does not write the fractions)
            }

            // Test if both min and max have been set
            if (fractions_set == 3) {
                // Output fractions
                // Test if the infinite ray intersects with the hull (the length will be checked later)
                return .{ .hit = min_fraction <= max_fraction and max_fraction >= 0.0, .min_fraction = min_fraction, .max_fraction = max_fraction };
            } else {
                // Degenerate case, either the ray is parallel to all planes or the ray has zero length
                // Return if the origin is inside the hull
                return .{ .hit = all_inside, .min_fraction = 0.0, .max_fraction = 1.0 + math.flt_epsilon };
            }
        }
    }

    // See Shape::CastRay
    pub fn castRay(self: *const ConvexHullShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Determine if ray hits the shape
        const r = self.castRayHelper(ray);
        if (r.hit and r.min_fraction < hit.fraction) { // Check if this is a closer hit
            // Better hit than the current hit
            hit.fraction = r.min_fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const ConvexHullShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Determine if ray hits the shape
        const r = self.castRayHelper(ray);
        if (r.hit and r.min_fraction < collector.getEarlyOutFraction()) { // Check if this is closer than the early out fraction
            // Better hit than the current hit
            var hit: RayCastResult = .{};
            hit.body_id = TransformedShape.getBodyID(collector.getContext());
            hit.sub_shape_id2 = sub_shape_id_creator.getID();

            // Check front side hit
            if (ray_cast_settings.treat_convex_as_solid or r.min_fraction > 0.0) {
                hit.fraction = r.min_fraction;
                collector.addHit(&hit);
            }

            // Check back side hit
            if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and r.max_fraction < collector.getEarlyOutFraction()) {
                hit.fraction = r.max_fraction;
                collector.addHit(&hit);
            }
        }
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const ConvexHullShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Check if point is behind all planes
        for (self.planes.items) |p| {
            if (p.signedDistance(point) > 0.0)
                return;
        }

        // Point is inside
        collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const ConvexHullShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        const inverse_transform = center_of_mass_transform.inversedRotationTranslation();

        const inv_scale = scale.reciprocal();
        const is_not_scaled = ScaleHelpers.isNotScaled(scale);
        const scale_flip: f32 = if (ScaleHelpers.isInsideOut(scale)) -1.0 else 1.0;

        var v = vertices.*;
        const sbv_end = vertices.add(@intCast(num_vertices));
        while (!v.eql(&sbv_end)) : (v.increment()) {
            if (v.getInvMass() > 0.0) {
                const local_pos = inverse_transform.mulVec3(v.getPosition());

                // Find most facing plane
                var max_distance: f32 = -math.flt_max;
                var max_plane_normal = Vec3.zero();
                var max_plane_idx: usize = 0;
                if (is_not_scaled) {
                    // Without scale, it is trivial to calculate the distance to the hull
                    for (self.planes.items, 0..) |p, i| {
                        const distance = p.signedDistance(local_pos);
                        if (distance > max_distance) {
                            max_distance = distance;
                            max_plane_normal = p.getNormal();
                            max_plane_idx = i;
                        }
                    }
                } else {
                    // When there's scale we need to calculate the planes first
                    for (0..self.planes.items.len) |i| {
                        // Calculate plane normal and point by scaling the original plane
                        const plane_normal = inv_scale.mul(self.planes.items[i].getNormal()).normalized();
                        const plane_point = scale.mul(self.points.items[self.vertex_idx.items[self.faces.items[i].first_vertex]].position);

                        const distance = plane_normal.dot(local_pos.sub(plane_point));
                        if (distance > max_distance) {
                            max_distance = distance;
                            max_plane_normal = plane_normal;
                            max_plane_idx = i;
                        }
                    }
                }

                // Project point onto that plane, in local space to the vertex
                var closest_point = max_plane_normal.mulScalar(-max_distance);

                // Check edges if we're outside the hull (when inside we know the closest face is also the closest point to the surface)
                const is_outside = max_distance > 0.0;
                if (is_outside) {
                    // Loop over edges
                    var closest_point_dist_sq: f32 = math.flt_max;
                    const face = self.faces.items[max_plane_idx];
                    const face_vertices = self.vertex_idx.items[face.first_vertex..][0..face.num_vertices];
                    for (0..face_vertices.len) |v1| {
                        // Find second point
                        var v2 = v1 + 1;
                        if (v2 == face_vertices.len)
                            v2 = 0;

                        // Get edge points
                        const p1 = scale.mul(self.points.items[face_vertices[v1]].position);
                        const p2 = scale.mul(self.points.items[face_vertices[v2]].position);

                        // Check if the position is outside the edge (if not, the face will be closer)
                        const edge_normal = p2.sub(p1).cross(max_plane_normal);
                        if (scale_flip * edge_normal.dot(local_pos.sub(p1)) > 0.0) {
                            // Get closest point on edge
                            const closest = ClosestPoint.getClosestPointOnLine(p1.sub(local_pos), p2.sub(local_pos)).point;
                            const distance_sq = closest.lengthSq();
                            if (distance_sq < closest_point_dist_sq) {
                                closest_point_dist_sq = distance_sq;
                                closest_point = closest;
                            }
                        }
                    }
                }

                // Check if this is the largest penetration
                var normal = closest_point.negate();
                const normal_length = normal.length();
                var penetration = normal_length;
                if (is_outside)
                    penetration = -penetration
                else
                    normal = normal.negate();
                if (v.updatePenetration(penetration)) {
                    // Calculate contact plane
                    normal = if (normal_length > 1.0e-12) normal.divScalar(normal_length) else max_plane_normal;
                    const plane = Plane.fromPointAndNormal(local_pos.add(closest_point), normal);

                    // Store collision
                    v.setCollision(plane.getTransformed(center_of_mass_transform), colliding_shape_index);
                }
            }
        }
    }

    // See Shape::GetTrianglesStart
    pub fn getTrianglesStart(self: *const ConvexHullShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = self;
        _ = box;
        context.emplace(CHSGetTrianglesContext).* = .init(Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale)), ScaleHelpers.isInsideOut(scale));
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const ConvexHullShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        comptime std.debug.assert(Shape.get_triangles_min_triangles_requested >= 12); // cGetTrianglesMinTrianglesRequested is too small
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        const ctx = context.get(CHSGetTrianglesContext);

        var triangles_left: i32 = @intCast(max_triangles_requested);
        var total_num_triangles: u32 = 0;
        var out: usize = 0;
        while (ctx.current_face < self.faces.items.len) : (ctx.current_face += 1) {
            const f = self.faces.items[ctx.current_face];

            const vertices = self.vertex_idx.items[f.first_vertex..][0..f.num_vertices];

            // Check if there is still room in the output buffer for this face
            const num_triangles: i32 = @as(i32, f.num_vertices) - 2;
            triangles_left -= num_triangles;
            if (triangles_left < 0)
                break;
            total_num_triangles += @intCast(num_triangles);

            // Get first triangle of polygon
            const v0 = ctx.transform.mulVec3(self.points.items[vertices[0]].position);
            const v1 = ctx.transform.mulVec3(self.points.items[vertices[1]].position);
            const v2 = ctx.transform.mulVec3(self.points.items[vertices[2]].position);
            v0.storeFloat3(&out_triangle_vertices[out]);
            out += 1;
            if (ctx.is_inside_out) {
                // Store first triangle in this polygon flipped
                v2.storeFloat3(&out_triangle_vertices[out]);
                v1.storeFloat3(&out_triangle_vertices[out + 1]);
                out += 2;

                // Store other triangles in this polygon flipped
                for (3..vertices.len) |v| {
                    v0.storeFloat3(&out_triangle_vertices[out]);
                    ctx.transform.mulVec3(self.points.items[vertices[v]].position).storeFloat3(&out_triangle_vertices[out + 1]);
                    ctx.transform.mulVec3(self.points.items[vertices[v - 1]].position).storeFloat3(&out_triangle_vertices[out + 2]);
                    out += 3;
                }
            } else {
                // Store first triangle in this polygon
                v1.storeFloat3(&out_triangle_vertices[out]);
                v2.storeFloat3(&out_triangle_vertices[out + 1]);
                out += 2;

                // Store other triangles in this polygon
                for (3..vertices.len) |v| {
                    v0.storeFloat3(&out_triangle_vertices[out]);
                    ctx.transform.mulVec3(self.points.items[vertices[v - 1]].position).storeFloat3(&out_triangle_vertices[out + 1]);
                    ctx.transform.mulVec3(self.points.items[vertices[v]].position).storeFloat3(&out_triangle_vertices[out + 2]);
                    out += 3;
                }
            }
        }

        // Store materials
        if (out_materials) |materials| {
            const material = self.base.getConvexMaterial();
            for (materials[0..total_num_triangles]) |*m|
                m.* = material;
        }

        return total_num_triangles;
    }

    // See Shape::SaveBinaryState
    pub fn saveBinaryState(self: *const ConvexHullShape, stream: StreamOut) void {
        ConvexShape.impl.saveBinaryState(&self.base, stream);

        stream.write(self.center_of_mass);
        stream.write(self.inertia);
        stream.write(self.local_bounds.min);
        stream.write(self.local_bounds.max);
        stream.writeArray(Point, self.points.items);
        stream.writeArray(Face, self.faces.items);
        stream.writeArray(Plane, self.planes.items);
        stream.writeArray(u8, self.vertex_idx.items);
        stream.write(self.convex_radius);
        stream.write(self.volume);
        stream.write(self.inner_radius);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const ConvexHullShape) Shape.Stats {
        // Count number of triangles
        var triangle_count: u32 = 0;
        for (self.faces.items) |f|
            triangle_count +%= @as(u32, f.num_vertices) -% 2;

        return .init(@sizeOf(ConvexHullShape) +
            self.points.items.len * @sizeOf(Point) +
            self.faces.items.len * @sizeOf(Face) +
            self.planes.items.len * @sizeOf(Plane) +
            self.vertex_idx.items.len * @sizeOf(u8), triangle_count);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const ConvexHullShape) f32 {
        return self.volume;
    }

    // See: Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *ConvexHullShape, stream: StreamIn) Allocator.Error!void {
        try ConvexShape.impl.restoreBinaryState(&self.base, stream);

        const allocator = self.base.base.allocator;
        stream.read(&self.center_of_mass);
        stream.read(&self.inertia);
        stream.read(&self.local_bounds.min);
        stream.read(&self.local_bounds.max);
        try stream.readArray(Point, allocator, &self.points);
        try stream.readArray(Face, allocator, &self.faces);
        try stream.readArray(Plane, allocator, &self.planes);
        try stream.readArray(u8, allocator, &self.vertex_idx);
        stream.read(&self.convex_radius);
        stream.read(&self.volume);
        stream.read(&self.inner_radius);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.convex_hull);
        f.construct = ShapeFunctions.constructor(ConvexHullShape);
        f.color = Color.green;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Class for GetTrianglesStart/Next

    const CHSGetTrianglesContext = struct {
        transform: Mat44,
        is_inside_out: bool,
        current_face: usize = 0,

        fn init(transform: Mat44, is_inside_out: bool) CHSGetTrianglesContext {
            return .{ .transform = transform, .is_inside_out = is_inside_out };
        }

        comptime {
            std.debug.assert(@sizeOf(CHSGetTrianglesContext) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
        }
    };

    // ---------------------------------------------------------------------------------------------------------------
    // Classes for GetSupportFunction

    /// The hull shrunk by the convex radius (`class HullNoConvex final : public Support`), up to max_points_in_hull
    /// points: constructed in place in the SupportBuffer with `init(self: *HullNoConvex, ...)`
    const HullNoConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        const PointsArray = StaticArray(Vec3, max_points_in_hull);

        base: ConvexShape.Support,
        convex_radius: f32,
        points: PointsArray,

        /// In place constructor (explicit HullNoConvex(float inConvexRadius)), the points start empty
        fn init(self: *HullNoConvex, convex_radius: f32) void {
            self.base = .init(HullNoConvex);
            self.convex_radius = convex_radius;
            self.points.len = 0;
        }

        pub fn getSupport(self: *const HullNoConvex, direction: Vec3) Vec3 {
            // Find the point with the highest projection on direction
            var best_dot: f32 = -math.flt_max;
            var best_point = Vec3.zero();

            for (self.points.constSlice()) |point| {
                // Check if its support is bigger than the current max
                const dot = point.dot(direction);
                if (dot > best_dot) {
                    best_dot = dot;
                    best_point = point;
                }
            }

            return best_point;
        }

        pub fn getConvexRadius(self: *const HullNoConvex) f32 {
            return self.convex_radius;
        }

        fn getPoints(self: *HullNoConvex) *PointsArray {
            return &self.points;
        }

        fn getPointsConst(self: *const HullNoConvex) *const PointsArray {
            return &self.points;
        }

        comptime {
            std.debug.assert(@sizeOf(HullNoConvex) <= ConvexShape.SupportBuffer.buffer_size); // Buffer size too small
        }
    };

    /// The hull including the convex radius (`class HullWithConvex final : public Support`)
    const HullWithConvex = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        shape: *const ConvexHullShape,

        fn init(shape: *const ConvexHullShape) HullWithConvex {
            return .{ .base = .init(HullWithConvex), .shape = shape };
        }

        pub fn getSupport(self: *const HullWithConvex, direction: Vec3) Vec3 {
            // Find the point with the highest projection on direction
            var best_dot: f32 = -math.flt_max;
            var best_point = Vec3.zero();

            for (self.shape.points.items) |point| {
                // Check if its support is bigger than the current max
                const dot = point.position.dot(direction);
                if (dot > best_dot) {
                    best_dot = dot;
                    best_point = point.position;
                }
            }

            return best_point;
        }

        pub fn getConvexRadius(self: *const HullWithConvex) f32 {
            _ = self;
            return 0.0;
        }
    };

    /// The scaled hull including the convex radius (`class HullWithConvexScaled final : public Support`)
    const HullWithConvexScaled = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        shape: *const ConvexHullShape,
        scale: Vec3,

        fn init(shape: *const ConvexHullShape, scale: Vec3) HullWithConvexScaled {
            return .{ .base = .init(HullWithConvexScaled), .shape = shape, .scale = scale };
        }

        pub fn getSupport(self: *const HullWithConvexScaled, direction: Vec3) Vec3 {
            // Find the point with the highest projection on direction
            var best_dot: f32 = -math.flt_max;
            var best_point = Vec3.zero();

            for (self.shape.points.items) |point| {
                // Calculate scaled position
                const pos = self.scale.mul(point.position);

                // Check if its support is bigger than the current max
                const dot = pos.dot(direction);
                if (dot > best_dot) {
                    best_dot = dot;
                    best_point = pos;
                }
            }

            return best_point;
        }

        pub fn getConvexRadius(self: *const HullWithConvexScaled) f32 {
            _ = self;
            return 0.0;
        }
    };
};

/// Formats a float like C's printf("%g") (Jolt's StringFormat with "%g" and a double argument)
const PrintfG = struct {
    value: f64,

    pub fn format(self: PrintfG, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        return StringTools.writeFloatGeneral(writer, self.value);
    }
};
