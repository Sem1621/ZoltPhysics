//! Parity tests for ConvexHullShape (Phase 4, Wave A): the hull is built from the same point cloud and settings on both
//! sides (`HullDesc`), then every query is compared bit for bit with the C++ library. C ABI wrappers:
//! ZoltParity/Physics/ConvexHullShapeReference.cpp. The shapes are created once per point cloud and passed to the
//! queries as handles (`Pair`).
//!
//! The point clouds are uniform in a box, on a sphere (also more than 256 points: the hull stops at 256 vertices),
//! boxes with interior points and points on the faces (sometimes thin), flat (2 faces, the 2D builder), nearly flat
//! (slivers and Jolt's "Hull building failed" error), prisms with up to 128 sides (points that shrink along 2 planes),
//! boxes with a nearly flat tip (points that shrink along 1 plane), tetrahedra (sharp tips that reduce the convex
//! radius), translated, tiny and huge clouds, integer grids (exact ties, coplanar faces), triangles, clouds with
//! duplicate points and degenerate clouds (too few points, colinear, all equal), with random convex radii (zero, the
//! default, too big for the hull, negative), max errors, hull tolerances and densities.
//!
//! Compared: the error texts of invalid settings; the whole hull after creation through the public accessors (points,
//! faces, planes, vertex indices, convex radius) and the binary state (which also holds the center of mass, the inertia
//! matrix, the bounds, the faces of each point used to shrink it, the volume and the inner radius); GetLocalBounds,
//! GetWorldSpaceBounds (Mat44 and DMat44), GetCenterOfMass, GetInnerRadius, GetMassProperties, GetVolume, GetStats,
//! GetSubShapeIDBitsRecursive, IsValidScale / MakeScaleValid, GetLeafShape, GetSubShapeUserData, GetMaterial,
//! MustBeStatic, GetSurfaceNormal, GetSupportingFace (scaled, inside out); the support points of every ESupportMode
//! with and without scale (HullNoConvex, HullWithConvex, HullWithConvexScaled); CastRay (both overloads, with the
//! AllHit / AnyHit / ClosestHit collectors, back faces, solid or not, early out fractions; rays through the hull, from
//! inside, along faces, grazing vertices, parallel, zero length) and CollidePoint (also on vertices and faces);
//! CollisionDispatch::sCollideShapeVsShape and sCastShapeVsShapeWorldSpace of hulls against spheres, boxes, other hulls
//! and themselves (in both orders, with scales with negative components, max separation distances, tolerances, active
//! edge / back face / collect faces modes, shrunken shapes, deepest points, extra convex radius, early out fractions);
//! GetSubmergedVolume; GetTrianglesStart / Next; CollideSoftBodyVertices; SaveBinaryState and sRestoreFromBinaryState
//! (also truncated streams).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CollisionDispatch = zolt.CollisionDispatch;
const ConvexHullBuilder = zolt.ConvexHullBuilder;
const ConvexHullShape = zolt.ConvexHullShape;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const ConvexShape = zolt.ConvexShape;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Ref = zolt.Ref;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastSettings = zolt.ShapeCastSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see ConvexHullShapeReference.cpp
const jolt = struct {
    extern fn jolt_chs_create_hull(desc: *const HullDesc, out_error: *[128]u8) ?*anyopaque;
    extern fn jolt_chs_create_sphere(radius: f32, density: f32) *anyopaque;
    extern fn jolt_chs_create_box(half_extent: *const P, convex_radius: f32, density: f32) *anyopaque;
    extern fn jolt_chs_release(shape: *anyopaque) void;
    extern fn jolt_chs_accessors(shape: *const anyopaque, output: *AccessorsOutput) void;
    extern fn jolt_chs_properties(shape: *const anyopaque, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_chs_support(shape: *const anyopaque, mode: c_int, scale: *const P, directions: [*]const f32, num_directions: c_int, out_points: [*]f32) f32;
    extern fn jolt_chs_cast_ray(shape: *const anyopaque, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_chs_collide_point(shape: *const anyopaque, point: *const P, creator: *const [2]u32, body_id: u32, out_ids: *[2]u32) u32;
    extern fn jolt_chs_collide(shape1: *const anyopaque, shape2: *const anyopaque, input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_chs_cast(shape1: *const anyopaque, shape2: *const anyopaque, input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_chs_submerged_volume(shape: *const anyopaque, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_chs_triangles(shape: *const anyopaque, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, out_counts: *[64]c_int, out_vertices: *[max_triangles * 9]f32, out_default_material: *[max_triangles]c_int) c_int;
    extern fn jolt_chs_binary_state(shape: *anyopaque, user_data: u64, truncate: u32, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32, out_error: *[128]u8) u32;
    extern fn jolt_chs_soft_body(shape: *const anyopaque, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// Print the index and kind of every cloud before it is built (to find the clouds that violate an assert, see
/// `assert_clouds`)
const debug_clouds = false;

/// The clouds of `CloudGen` that violate an assert of Zolt's hull builder in `initialize` (`IsFacing` in FindEdge or
/// `edges.size() >= 3` in AddPoint). Jolt's release build (the reference) continues, Zolt and Jolt builds with asserts
/// abort (see `ConvexHullBuilder.initialize`). With `Core.enable_asserts` (Debug, ReleaseSafe) these clouds are skipped,
/// in ReleaseFast everything is compared (`zig build parity -Doptimize=ReleaseFast`). Found with `debug_clouds` in a
/// ReleaseSafe build; update this list when the generator changes.
const assert_clouds = [_]u32{};

/// Number of clouds of the creation test; the other tests use the valid clouds among them
const num_clouds = 4000;

/// Most points in a point cloud
const max_cloud_points = 600;

/// Most triangles returned by GetTrianglesNext that are compared
const max_triangles = 512;

/// Capacity of the binary state buffers (a hull with 256 points needs about 22 KB)
const binary_state_capacity = 32 * 1024;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Hull description, must match HullDesc in ConvexHullShapeReference.cpp
const HullDesc = extern struct {
    /// 3 floats per point
    points: [*]const f32,
    num_points: u32,
    max_convex_radius: f32,
    max_error_convex_radius: f32,
    hull_tolerance: f32,
    density: f32,
};

/// Must match AccessorsOutput in ConvexHullShapeReference.cpp
const AccessorsOutput = extern struct {
    convex_radius: f32,
    num_points: u32,
    points: [256 * 3]f32,
    num_faces: u32,
    num_planes: u32,
    planes: [512 * 4]f32,
    num_vertices_in_face: [512]u32,
    /// GetFaceVertices with inMaxVertices = 3
    num_vertices_returned: [512]u32,
    first_vertices: [512 * 3]u32,
    num_vertex_indices: u32,
    /// GetFaceVertices of all faces, concatenated
    vertex_indices: [2048]u32,
};

/// Must match PropertiesInput in ConvexHullShapeReference.cpp
const PropertiesInput = extern struct {
    scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
    /// GetLeafShape, GetSubShapeUserData
    sub_shape_id: u32,
};

/// Must match PropertiesOutput in ConvexHullShapeReference.cpp
const PropertiesOutput = extern struct {
    local_bounds: [6]f32,
    world_bounds: [6]f32,
    world_bounds_d: [6]f32,
    center_of_mass: P,
    inner_radius: f32,
    mass: f32,
    inertia: [16]f32,
    volume: f32,
    num_triangles: u32,
    sub_shape_id_bits: u32,
    is_valid_scale: c_int,
    scale_valid: P,
    leaf_is_self: c_int,
    leaf_remainder: u32,
    sub_shape_user_data: u64,
    material_is_default: c_int,
    must_be_static: c_int,
    surface_normal: P,
    face_count: u32,
    face: [32 * 3]f32,
};

/// Must match RayInput in ConvexHullShapeReference.cpp
const RayInput = extern struct {
    origin: P,
    direction: P,
    /// Sub shape ID creator: value pushed, number of bits
    creator: [2]u32,
    /// Initial fraction of the single hit version
    fraction: f32,
    /// 1: collide with back faces (convex)
    back_face_mode: c_int,
    treat_convex_as_solid: c_int,
    /// 0: AllHit, 1: AnyHit, 2: ClosestHit
    collector: c_int,
    /// Early out fraction of the collector (when < the initial one)
    early_out: f32,
    /// Body ID of the collector context
    body_id: u32,
};

const RayHit = extern struct {
    fraction: f32,
    body_id: u32,
    sub_shape_id: u32,
};

/// Must match RayOutput in ConvexHullShapeReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
};

/// Must match CollideInput in ConvexHullShapeReference.cpp
const CollideInput = extern struct {
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    /// 0: collide only with active, 1: collide with all
    active_edge_mode: c_int,
    /// 1: collide with back faces
    back_face_mode: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
};

/// Must match HitOutput in ConvexHullShapeReference.cpp
const HitOutput = extern struct {
    /// Cast only
    fraction: f32,
    /// Cast only
    back_face: c_int,
    point1: P,
    point2: P,
    axis: P,
    depth: f32,
    id1: u32,
    id2: u32,
    body_id: u32,
    face1_count: u32,
    face2_count: u32,
    face1: [32 * 3]f32,
    face2: [32 * 3]f32,
};

/// Must match HitsOutput in ConvexHullShapeReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    hits: [2]HitOutput,
};

/// Must match CastInput in ConvexHullShapeReference.cpp
const CastInput = extern struct {
    scale1: P,
    start: [16]f32,
    direction: P,
    scale2: P,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    extra_convex_radius: f32,
    /// 0: collide only with active, 1: collide with all
    active_edge_mode: c_int,
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// The shapes on both sides

/// The same shape created by Jolt (a handle with one reference) and by Zolt; both are null when the settings are invalid
const Pair = struct {
    jolt: ?*anyopaque = null,
    zolt: ?Ref(Shape) = null,
    /// The error texts of both sides (zero padded)
    jolt_error: [128]u8 = @splat(0),
    zolt_error: [128]u8 = @splat(0),
    /// The cloud was not built because it violates an assert of the hull builder (see `violatesBuilderAssert`)
    skipped: bool = false,

    fn deinit(self: *Pair) void {
        if (self.jolt) |j| jolt.jolt_chs_release(j);
        if (self.zolt) |*z| z.deinit();
    }

    fn isValid(self: *const Pair) bool {
        return self.jolt != null and self.zolt != null;
    }

    fn shape(self: *const Pair) *const Shape {
        return self.zolt.?.get().?;
    }

    fn shapeMut(self: *Pair) *Shape {
        return self.zolt.?.get().?;
    }
};

/// A point cloud with its settings
const HullInput = struct {
    points: [max_cloud_points]P = undefined,
    num_points: u32 = 0,
    max_convex_radius: f32 = zolt.physics_settings.default_convex_radius,
    max_error_convex_radius: f32 = 0.05,
    hull_tolerance: f32 = 1.0e-3,
    density: f32 = 1000.0,
    /// The kind of cloud (see `Gen.hull`)
    kind: u32 = 0,
    /// Index in the sequence of clouds (`CloudGen`)
    index: u32 = 0,

    fn desc(self: *const HullInput) HullDesc {
        return .{
            .points = @ptrCast(&self.points),
            .num_points = self.num_points,
            .max_convex_radius = self.max_convex_radius,
            .max_error_convex_radius = self.max_error_convex_radius,
            .hull_tolerance = self.hull_tolerance,
            .density = self.density,
        };
    }

    fn add(self: *HullInput, p: P) void {
        if (self.num_points < max_cloud_points) {
            self.points[self.num_points] = p;
            self.num_points += 1;
        }
    }

    fn slice(self: *const HullInput) []const P {
        return self.points[0..self.num_points];
    }
};

/// The kind of cloud with near duplicate points (`Gen.hull`)
const duplicates_kind = 13;

/// Clouds with near duplicate points can leave a face with a zero normal in a fully built hull. DetermineMaxError
/// asserts that the normals are not zero; Jolt's release build (the reference) continues, Zolt and Jolt builds with
/// asserts abort (see `ConvexHullBuilder.initialize`). With `Core.enable_asserts` (Debug, ReleaseSafe) these clouds are
/// skipped, in ReleaseFast everything is compared (`zig build parity -Doptimize=ReleaseFast`).
fn violatesBuilderAssert(allocator: Allocator, input: *const HullInput, points: []const Vec3) !bool {
    if (!zolt.Core.enable_asserts) return false;
    var builder = ConvexHullBuilder.init(allocator, points);
    defer builder.deinit();
    if (input.max_convex_radius < 0.0) return false; // The hull is not built
    const result = try builder.initialize(ConvexHullShape.max_points_in_hull, input.hull_tolerance);
    if (result.result != .success) return false; // DetermineMaxError is not called
    for (builder.getFaces()) |f|
        if (!(f.normal.length() > 0.0)) return true;
    return false;
}

fn createHull(allocator: Allocator, input: *const HullInput) !Pair {
    var pair: Pair = .{};
    var points: [max_cloud_points]Vec3 = undefined;
    for (input.slice(), 0..) |p, i| points[i] = vec3(p);
    if (debug_clouds) std.debug.print("CLOUD {d} KIND {d}\n", .{ input.index, input.kind });
    if (zolt.Core.enable_asserts and std.mem.indexOfScalar(u32, &assert_clouds, input.index) != null) {
        pair.skipped = true;
        return pair;
    }
    if (input.kind == duplicates_kind and try violatesBuilderAssert(allocator, input, points[0..input.num_points])) {
        pair.skipped = true;
        return pair;
    }

    const desc = input.desc();
    pair.jolt = jolt.jolt_chs_create_hull(&desc, &pair.jolt_error);

    var settings = try ConvexHullShapeSettings.init(allocator, points[0..input.num_points], .{ .max_convex_radius = input.max_convex_radius });
    defer settings.deinit();
    settings.max_error_convex_radius = input.max_error_convex_radius;
    settings.hull_tolerance = input.hull_tolerance;
    settings.base.density = input.density;
    var result = settings.asShapeSettings().createShape(allocator) catch |err| {
        pair.deinit();
        return err;
    };
    defer result.deinit();
    if (result.hasError()) {
        const text = result.getError();
        const n = @min(text.len, 127);
        @memcpy(pair.zolt_error[0..n], text[0..n]);
    } else {
        pair.zolt = Ref(Shape).init(result.getPtr());
    }
    return pair;
}

fn createSphere(allocator: Allocator, radius: f32, density: f32) !Pair {
    var settings = SphereShapeSettings.init(allocator, radius, .{});
    defer settings.deinit();
    settings.base.density = density;
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    return .{ .jolt = jolt.jolt_chs_create_sphere(radius, density), .zolt = Ref(Shape).init(result.getPtr()) };
}

fn createBox(allocator: Allocator, half_extent: P, convex_radius: f32, density: f32) !Pair {
    var settings = BoxShapeSettings.init(allocator, vec3(half_extent), .{ .convex_radius = convex_radius });
    defer settings.deinit();
    settings.base.density = density;
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    return .{ .jolt = jolt.jolt_chs_create_box(&half_extent, convex_radius, density), .zolt = Ref(Shape).init(result.getPtr()) };
}

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn vec4(a: [4]f32) Vec4 {
    return Vec4.init(a[0], a[1], a[2], a[3]);
}

fn arr4(v: Vec4) [4]f32 {
    return .{ v.getX(), v.getY(), v.getZ(), v.getW() };
}

fn quat(a: [4]f32) Quat {
    return Quat.init(a[0], a[1], a[2], a[3]);
}

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn boxArr(b: AABox) [6]f32 {
    return arr3(b.min) ++ arr3(b.max);
}

fn planeArr(p: Plane) [4]f32 {
    return arr3(p.getNormal()) ++ [1]f32{p.getConstant()};
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn storeFace(face: *const Shape.SupportingFace, out_count: *u32, out_face: *[32 * 3]f32) void {
    out_count.* = face.len;
    for (face.constSlice(), 0..) |v, i| out_face[3 * i ..][0..3].* = arr3(v);
}

fn storeCollideHit(result: *const CollideShapeResult, out: *HitOutput) void {
    out.point1 = arr3(result.contact_point_on1);
    out.point2 = arr3(result.contact_point_on2);
    out.axis = arr3(result.penetration_axis);
    out.depth = result.penetration_depth;
    out.id1 = result.sub_shape_id1.getValue();
    out.id2 = result.sub_shape_id2.getValue();
    out.body_id = result.body_id2.getIndexAndSequenceNumber();
    storeFace(&result.shape1_face, &out.face1_count, &out.face1);
    storeFace(&result.shape2_face, &out.face2_count, &out.face2);
}

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

fn zoltAccessors(hull: *const ConvexHullShape, o: *AccessorsOutput) void {
    o.convex_radius = hull.getConvexRadius();
    o.num_points = hull.getNumPoints();
    for (0..@min(o.num_points, 256)) |i| o.points[3 * i ..][0..3].* = arr3(hull.getPoint(@intCast(i)));
    o.num_faces = hull.getNumFaces();
    const planes = hull.getPlanes();
    o.num_planes = @intCast(planes.len);
    for (planes[0..@min(planes.len, 512)], 0..) |plane, i| o.planes[4 * i ..][0..4].* = planeArr(plane);
    o.num_vertex_indices = 0;
    for (0..@min(o.num_faces, 512)) |f| {
        const face: u32 = @intCast(f);
        o.num_vertices_in_face[f] = hull.getNumVerticesInFace(face);
        o.num_vertices_returned[f] = hull.getFaceVertices(face, o.first_vertices[3 * f ..][0..3]);
        var indices: [256]u32 = undefined;
        const num = hull.getFaceVertices(face, &indices);
        for (indices[0..@min(num, 256)]) |index| {
            if (o.num_vertex_indices < 2048) {
                o.vertex_indices[o.num_vertex_indices] = index;
                o.num_vertex_indices += 1;
            }
        }
    }
}

fn zoltProperties(shape: *const Shape, input: *const PropertiesInput) PropertiesOutput {
    var o = std.mem.zeroes(PropertiesOutput);
    const scale = vec3(input.scale);
    const transform = mat44(input.transform);
    o.local_bounds = boxArr(shape.getLocalBounds());
    o.world_bounds = boxArr(shape.getWorldSpaceBounds(transform, scale));
    o.world_bounds_d = boxArr(shape.getWorldSpaceBoundsDMat44(DMat44.fromMat44Translation(transform, DVec3.init(input.translation[0], input.translation[1], input.translation[2])), scale));
    o.center_of_mass = arr3(shape.getCenterOfMass());
    o.inner_radius = shape.getInnerRadius();
    const p = shape.getMassProperties();
    o.mass = p.mass;
    o.inertia = arr16(p.inertia);
    o.volume = shape.getVolume();
    o.num_triangles = shape.getStats().num_triangles;
    o.sub_shape_id_bits = shape.getSubShapeIDBitsRecursive();
    o.is_valid_scale = @intFromBool(shape.isValidScale(scale));
    o.scale_valid = arr3(shape.makeScaleValid(scale));
    var sub_shape_id: SubShapeID = .empty;
    sub_shape_id.setValue(input.sub_shape_id);
    const leaf = shape.getLeafShape(sub_shape_id);
    o.leaf_is_self = @intFromBool(leaf.shape == shape);
    o.leaf_remainder = leaf.remainder.getValue();
    o.sub_shape_user_data = shape.getSubShapeUserData(sub_shape_id);
    o.material_is_default = @intFromBool(shape.getMaterial(.empty) == PhysicsMaterial.default);
    o.must_be_static = @intFromBool(shape.mustBeStatic());
    o.surface_normal = arr3(shape.getSurfaceNormal(.empty, vec3(input.point)));
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, vec3(input.direction), scale, transform, &face);
    storeFace(&face, &o.face_count, &o.face);
    return o;
}

fn zoltCastRay(allocator: Allocator, shape: *const Shape, input: *const RayInput) !RayOutput {
    var o = std.mem.zeroes(RayOutput);
    const ray = RayCast.init(vec3(input.origin), vec3(input.direction));
    const creator = makeCreator(input.creator);

    var hit: RayCastResult = .{};
    hit.fraction = input.fraction;
    o.hit = @intFromBool(shape.castRay(ray, creator, &hit));
    o.fraction = hit.fraction;
    o.sub_shape_id = hit.sub_shape_id2.getValue();

    var settings: RayCastSettings = .{};
    settings.back_face_mode_convex = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.treat_convex_as_solid = input.treat_convex_as_solid != 0;
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    const store = struct {
        fn f(out: *RayOutput, h: *const RayCastResult) void {
            out.hits[out.num_hits] = .{ .fraction = h.fraction, .body_id = h.body_id.getIndexAndSequenceNumber(), .sub_shape_id = h.sub_shape_id2.getValue() };
            out.num_hits += 1;
        }
    }.f;
    switch (input.collector) {
        0 => {
            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            try collector.checkError();
            for (collector.hits.items) |*h| store(&o, h);
        },
        1 => {
            var collector = AnyHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            if (collector.hadHit()) store(&o, &collector.hit);
        },
        else => {
            var collector = ClosestHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            if (collector.hadHit()) store(&o, &collector.hit);
        },
    }
    return o;
}

fn zoltCollide(allocator: Allocator, shape1: *const Shape, shape2: *const Shape, input: *const CollideInput) !HitsOutput {
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = input.max_separation_distance;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.back_face_mode = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.collideShapeVsShape(shape1, shape2, vec3(input.scale1), vec3(input.scale2), mat44(input.transform1), mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &settings, &collector.base, &.{});
    try collector.checkError();
    var o = std.mem.zeroes(HitsOutput);
    for (collector.hits.items) |*r| {
        if (o.num_hits < 2) {
            const h = &o.hits[o.num_hits];
            o.num_hits += 1;
            h.fraction = 0.0;
            h.back_face = 0;
            storeCollideHit(r, h);
        }
    }
    return o;
}

fn zoltCast(allocator: Allocator, shape1: *const Shape, shape2: *const Shape, input: *const CastInput) !HitsOutput {
    var settings: ShapeCastSettings = .{};
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.use_shrunken_shape_and_convex_radius = input.use_shrunken_shape != 0;
    settings.return_deepest_point = input.return_deepest_point != 0;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    const shape_cast = ShapeCast.init(shape1, vec3(input.scale1), mat44(input.start), vec3(input.direction));
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &settings, shape2, vec3(input.scale2), &.{}, mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &collector.base);
    try collector.checkError();
    var o = std.mem.zeroes(HitsOutput);
    for (collector.hits.items) |*r| {
        if (o.num_hits < 2) {
            const h = &o.hits[o.num_hits];
            o.num_hits += 1;
            h.fraction = r.fraction;
            h.back_face = @intFromBool(r.is_back_face_hit);
            storeCollideHit(&r.base, h);
        }
    }
    return o;
}

/// The result of the binary state wrapper
const BinaryState = struct {
    size: u32,
    bytes: [binary_state_capacity]u8,
    restored_size: u32,
    restored: [binary_state_capacity]u8,
    error_text: [128]u8,
};

fn zoltBinaryState(allocator: Allocator, shape: *Shape, user_data: u64, truncate: u32, o: *BinaryState) !void {
    shape.setUserData(user_data);
    var writer: std.Io.Writer = .fixed(&o.bytes);
    var stream_out = StreamOutWrapper.init(&writer);
    shape.saveBinaryState(stream_out.streamOut());
    o.size = @intCast(writer.buffered().len);

    // Restore the bytes and save again
    o.restored_size = 0;
    var reader: std.Io.Reader = .fixed(o.bytes[0 .. o.size - truncate]);
    var stream_in = StreamInWrapper.init(&reader);
    var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
    defer result.deinit();
    if (result.isValid()) {
        var restored_writer: std.Io.Writer = .fixed(&o.restored);
        var restored_out = StreamOutWrapper.init(&restored_writer);
        result.getPtr().?.saveBinaryState(restored_out.streamOut());
        o.restored_size = @intCast(restored_writer.buffered().len);
    } else {
        const text = result.getError();
        @memcpy(o.error_text[0..text.len], text);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Input generation

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 1.0e-20, 3.0, -3.0 };

/// Input generator: xorshift32 with helpers for the edge cases
const Gen = struct {
    rng: fw.Rng = .{},

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    fn index(self: *Gen, n: usize) usize {
        return self.next() % n;
    }

    fn oneIn(self: *Gen, n: u32) bool {
        return self.next() % n == 0;
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Gen, min: f32, max: f32) f32 {
        if (self.oneIn(10)) return special_values[self.index(special_values.len)];
        return self.rng.float(min, max);
    }

    fn plain(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) P {
        return .{ self.float(min, max), self.float(min, max), self.float(min, max) };
    }

    fn plainVec(self: *Gen, min: f32, max: f32) P {
        return .{ self.plain(min, max), self.plain(min, max), self.plain(min, max) };
    }

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    /// A direction: random, axis aligned, zero, tiny
    fn direction(self: *Gen, length: f32) P {
        return switch (self.index(8)) {
            0 => .{ 0, 0, 0 },
            1 => blk: {
                var d: P = .{ 0, 0, 0 };
                d[self.index(3)] = if (self.oneIn(2)) length else -length;
                break :blk d;
            },
            2 => .{ self.grid(1), self.grid(1), self.grid(1) },
            3 => self.plainVec(-1.0e-6, 1.0e-6),
            else => self.vec(-length, length),
        };
    }

    /// A random unit vector
    fn unitVec(self: *Gen) Vec3 {
        while (true) {
            const v = vec3(self.plainVec(-1, 1));
            const len_sq = v.lengthSq();
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return v.normalized();
        }
    }

    /// A unit quaternion, sometimes a rotation of a multiple of 90 degrees around an axis or the identity
    fn rotation(self: *Gen) Quat {
        if (self.oneIn(4)) {
            const axes = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() };
            return Quat.rotation(axes[self.index(3)], @as(f32, @floatFromInt(self.index(4))) * 0.5 * math.pi);
        }
        while (true) {
            const q = self.rng.floatArray(4, -1, 1);
            const len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return quat(q).normalized();
        }
    }

    /// A rotation + translation matrix
    fn transform(self: *Gen, range: f32) Mat44 {
        return Mat44.rotationTranslation(self.rotation(), vec3(self.vec(-range, range)));
    }

    fn creator(self: *Gen) [2]u32 {
        const bits: u32 = @intCast(self.index(9));
        return .{ if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
    }

    /// A scale: one, uniform, non uniform, with negative components (inside out)
    fn scale(self: *Gen) P {
        return switch (self.index(5)) {
            0 => .{ 1, 1, 1 },
            1 => blk: {
                const s = self.plain(0.2, 2.5);
                break :blk .{ s, s, s };
            },
            2 => .{ if (self.oneIn(2)) 1.0 else -1.0, if (self.oneIn(2)) 1.0 else -1.0, if (self.oneIn(2)) 1.0 else -1.0 },
            else => blk: {
                var r = self.plainVec(0.2, 2.5);
                for (&r) |*c| {
                    if (self.oneIn(3)) c.* = -c.*;
                }
                break :blk r;
            },
        };
    }

    /// A scale that is valid for a sphere: uniform, components with different signs
    fn uniformScale(self: *Gen) P {
        const m: f32 = if (self.oneIn(3)) 1.0 else self.plain(0.2, 2.5);
        return .{ if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m };
    }

    /// Any scale (for IsValidScale / MakeScaleValid): random, uniform, tiny / zero / negative components
    fn anyScale(self: *Gen) P {
        return switch (self.index(5)) {
            0 => self.vec(-3, 3),
            1 => blk: {
                const s = self.float(-3, 3);
                break :blk .{ s, if (self.oneIn(2)) s else -s, s };
            },
            2 => .{ 1.0 + self.plain(-1.0e-4, 1.0e-4), 1.0, 1.0 + self.plain(-1.0e-5, 1.0e-5) },
            3 => .{ self.float(-1.0e-5, 1.0e-5), self.plain(0.1, 2), self.plain(-2, -0.1) },
            else => .{ self.grid(2), self.grid(2), self.grid(2) },
        };
    }

    /// A point cloud with its settings (see the file header for the kinds of clouds)
    fn hull(self: *Gen) HullInput {
        var h: HullInput = .{};
        const kind = self.index(16);
        h.kind = @intCast(kind);
        switch (kind) {
            0 => {
                // Uniform in a box
                const n = 4 + self.index(80);
                const e = self.plainVec(0.1, 3);
                for (0..n) |_| h.add(.{ self.plain(-e[0], e[0]), self.plain(-e[1], e[1]), self.plain(-e[2], e[2]) });
            },
            1, 2 => {
                // On a sphere (more than 256 points: the hull stops at the max number of vertices)
                const n = if (kind == 1) 8 + self.index(120) else 200 + self.index(max_cloud_points - 200);
                const r = self.plain(0.2, 3);
                const noise = if (self.oneIn(2)) 0.0 else self.plain(0, 0.05);
                for (0..n) |_| h.add(arr3(self.unitVec().mulScalar(r + self.plain(-noise, noise))));
            },
            3 => {
                // A box with interior points and points on its faces, sometimes thin
                var e = self.plainVec(0.1, 3);
                if (self.oneIn(3)) e[self.index(3)] = self.plain(0.001, 0.05);
                for (0..8) |i| h.add(.{ if (i & 1 != 0) e[0] else -e[0], if (i & 2 != 0) e[1] else -e[1], if (i & 4 != 0) e[2] else -e[2] });
                for (0..self.index(20)) |_| {
                    var p: P = .{ self.plain(-e[0], e[0]), self.plain(-e[1], e[1]), self.plain(-e[2], e[2]) };
                    if (self.oneIn(2)) {
                        const axis = self.index(3);
                        p[axis] = if (self.oneIn(2)) e[axis] else -e[axis];
                    }
                    h.add(p);
                }
            },
            4, 5 => {
                // Flat (exactly coplanar after the rotation within rounding) or nearly flat
                const n = 3 + self.index(40);
                const rotation_matrix = if (self.oneIn(3)) Mat44.identity() else Mat44.rotationTranslation(self.rotation(), vec3(self.plainVec(-2, 2)));
                const thickness: f32 = if (kind == 4) 0.0 else if (self.oneIn(2)) self.plain(1.0e-6, 1.0e-4) else self.plain(1.0e-4, 1.0e-2);
                const e = self.plainVec(0.2, 3);
                for (0..n) |_| h.add(arr3(rotation_matrix.mulVec3(Vec3.init(self.plain(-e[0], e[0]), self.plain(-thickness, thickness), self.plain(-e[2], e[2])))));
            },
            6 => {
                // Prism with n sides (n up to 128: the cap vertices shrink along 2 planes)
                const n = if (self.oneIn(2)) 3 + self.index(30) else 100 + self.index(29);
                const r = self.plain(0.2, 2);
                const half_height = self.plain(0.05, 2);
                const rotation_matrix = Mat44.rotationQuat(self.rotation());
                for (0..n) |i| {
                    const sc = Vec4.replicate(2.0 * math.pi * @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(n))).sinCos();
                    h.add(arr3(rotation_matrix.mulVec3(Vec3.init(r * sc.cos.getX(), -half_height, r * sc.sin.getX()))));
                    h.add(arr3(rotation_matrix.mulVec3(Vec3.init(r * sc.cos.getX(), half_height, r * sc.sin.getX()))));
                }
            },
            7 => {
                // A box with a nearly flat tip (the tip shrinks along 1 plane)
                const e = self.plainVec(0.5, 2);
                for (0..8) |i| h.add(.{ if (i & 1 != 0) e[0] else -e[0], if (i & 2 != 0) e[1] else -e[1], if (i & 4 != 0) e[2] else -e[2] });
                h.add(.{ self.plain(-0.1, 0.1), e[1] * (1.0 + self.plain(0.002, 0.01)), self.plain(-0.1, 0.1) });
                h.hull_tolerance = 1.0e-4;
            },
            8 => {
                // Tetrahedra and other sharp shapes
                const n = 4 + self.index(3);
                for (0..n) |_| h.add(self.plainVec(-2, 2));
                if (self.oneIn(2)) h.add(.{ 0, self.plain(5, 20), 0 }); // Sharp tip
            },
            9 => {
                // Translated far from the origin
                const offset = self.plainVec(-1000, 1000);
                const n = 4 + self.index(60);
                for (0..n) |_| {
                    const p = self.plainVec(-1, 1);
                    h.add(.{ p[0] + offset[0], p[1] + offset[1], p[2] + offset[2] });
                }
            },
            10 => {
                // Tiny or huge, the hull tolerance relative to the size (a tolerance close to the size of the cloud can
                // violate the asserts of the hull builder)
                const s: f32 = if (self.oneIn(2)) self.plain(0.001, 0.01) else self.plain(50, 200);
                const n = 4 + self.index(60);
                for (0..n) |_| h.add(arr3(vec3(self.plainVec(-1, 1)).mulScalar(s)));
                h.hull_tolerance = s * ([_]f32{ 0.0, 1.0e-5, 1.0e-3, 1.0e-2 })[self.index(4)];
            },
            11 => {
                // Integer grid (exact ties, coplanar faces that are merged)
                const n = 4 + self.index(60);
                for (0..n) |_| h.add(.{ self.grid(2), self.grid(2), self.grid(2) });
            },
            12 => {
                // A triangle
                for (0..3) |_| h.add(self.plainVec(-2, 2));
            },
            13 => {
                // Duplicates and near duplicates (Jolt's TestRandomHull distribution)
                const n = 4 + self.index(40);
                for (0..n) |i| {
                    if (i > 0 and self.oneIn(3)) {
                        const p = h.points[self.index(i)];
                        const d: f32 = if (self.oneIn(2)) 0.0 else self.plain(-1.0e-5, 1.0e-5);
                        h.add(.{ p[0] + d, p[1], p[2] - d });
                    } else h.add(self.plainVec(-1, 1));
                }
            },
            14 => {
                // Degenerate: too few points, colinear, all equal
                switch (self.index(4)) {
                    0 => {
                        for (0..self.index(3)) |_| h.add(self.plainVec(-1, 1));
                    },
                    1 => {
                        // Exactly colinear (integers: nearly colinear clouds can violate the asserts of the hull builder)
                        const a = Vec3.init(self.grid(3), self.grid(3), self.grid(3));
                        const d = Vec3.init(self.grid(2), self.grid(2), self.grid(2));
                        for (0..3 + self.index(10)) |_| h.add(arr3(a.add(d.mulScalar(self.grid(4)))));
                    },
                    2 => {
                        const a = self.plainVec(-1, 1);
                        for (0..3 + self.index(10)) |_| h.add(a);
                    },
                    else => {
                        // Four points on a line through integers (exactly colinear)
                        for (0..4) |i| h.add(.{ @floatFromInt(i), @floatFromInt(2 * i), 0 });
                    },
                }
            },
            else => {
                // Points on a cylinder with random heights
                const n = 8 + self.index(60);
                const r = self.plain(0.2, 2);
                for (0..n) |_| {
                    const sc = Vec4.replicate(self.plain(0, 2.0 * math.pi)).sinCos();
                    h.add(.{ r * sc.cos.getX(), self.plain(-1, 1), r * sc.sin.getX() });
                }
            },
        }

        // Settings
        h.max_convex_radius = switch (self.index(8)) {
            0, 1 => 0.0,
            2, 3 => zolt.physics_settings.default_convex_radius,
            4, 5 => self.plain(0, 0.5),
            6 => self.plain(2, 10), // Limited by the thickness of the hull
            else => if (self.oneIn(4)) -0.01 else self.plain(0, 0.2), // Invalid
        };
        h.max_error_convex_radius = if (self.oneIn(2)) 0.05 else self.plain(0.001, 0.5);
        if (kind != 7 and kind != 10) {
            h.hull_tolerance = switch (self.index(8)) {
                0, 1, 2, 3 => 1.0e-3,
                4 => 0.0,
                5 => 1.0e-5,
                else => self.plain(0, 0.1),
            };
        }
        h.density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
        return h;
    }
};

/// The radius of a sphere around the origin that contains the point cloud
fn extentOf(input: *const HullInput) f32 {
    var max: f32 = 0.0;
    for (input.slice()) |p| max = @max(max, vec3(p).length());
    return max;
}

/// The point clouds: the same sequence in every test, a cloud is identified by its index (see `assert_clouds`)
const CloudGen = struct {
    gen: Gen = .{ .rng = .{ .state = 0x2545f491 } },
    index: u32 = 0,

    fn next(self: *CloudGen) HullInput {
        var h = self.gen.hull();
        h.index = self.index;
        self.index += 1;
        return h;
    }
};

/// Create the next valid hull (the clouds that give an error are skipped)
fn nextValidHull(allocator: Allocator, clouds: *CloudGen, out_input: *HullInput) !Pair {
    while (true) {
        std.debug.assert(clouds.index < num_clouds); // assert_clouds only covers the clouds of the creation test
        out_input.* = clouds.next();
        var pair = try createHull(allocator, out_input);
        if (pair.isValid()) return pair;
        pair.deinit();
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

/// Number of point clouds per test
const num_hulls = 1000;

test "ConvexHullShape parity: creation, Jolt's error texts and the whole hull (accessors and binary state)" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed3812 } };
    var errors: Checker = .{ .name = "creation" };
    var accessors: Checker = .{ .name = "accessors" };
    var states: Checker = .{ .name = "binary state" };
    var num_valid: usize = 0;
    var num_errors: usize = 0;
    var num_hull_failed: usize = 0;
    var num_skipped: usize = 0;

    const jolt_accessors = try allocator.create(AccessorsOutput);
    defer allocator.destroy(jolt_accessors);
    const zolt_accessors = try allocator.create(AccessorsOutput);
    defer allocator.destroy(zolt_accessors);
    const jolt_state = try allocator.create(BinaryState);
    defer allocator.destroy(jolt_state);
    const zolt_state = try allocator.create(BinaryState);
    defer allocator.destroy(zolt_state);

    for (0..num_clouds) |_| {
        const input = clouds.next();
        var pair = try createHull(allocator, &input);
        defer pair.deinit();
        if (pair.skipped) {
            num_skipped += 1;
            continue;
        }
        errors.check(.{ input.num_points, input.max_convex_radius, input.hull_tolerance }, .{ @intFromBool(pair.zolt != null), pair.zolt_error }, .{ @intFromBool(pair.jolt != null), pair.jolt_error });
        if (!pair.isValid()) {
            num_errors += 1;
            if (std.mem.startsWith(u8, &pair.jolt_error, "Hull building failed")) num_hull_failed += 1;
            continue;
        }
        num_valid += 1;

        // Everything the accessors return
        jolt_accessors.* = std.mem.zeroes(AccessorsOutput);
        zolt_accessors.* = std.mem.zeroes(AccessorsOutput);
        jolt.jolt_chs_accessors(pair.jolt.?, jolt_accessors);
        zoltAccessors(pair.shape().cast(ConvexHullShape), zolt_accessors);
        accessors.check(.{input.num_points}, zolt_accessors.*, jolt_accessors.*);

        // The binary state (all data of the hull) and a restore of it, sometimes of a truncated stream
        const user_data = (@as(u64, gen.next()) << 32) | gen.next();
        const truncate: u32 = if (gen.oneIn(8)) @intCast(1 + gen.index(30)) else 0;
        jolt_state.* = std.mem.zeroes(BinaryState);
        zolt_state.* = std.mem.zeroes(BinaryState);
        jolt_state.size = jolt.jolt_chs_binary_state(pair.jolt.?, user_data, truncate, &jolt_state.bytes, binary_state_capacity, &jolt_state.restored, &jolt_state.restored_size, &jolt_state.error_text);
        try zoltBinaryState(allocator, pair.shapeMut(), user_data, truncate, zolt_state);
        states.check(.{ input.num_points, truncate }, zolt_state.*, jolt_state.*);
    }
    try finishAll(&.{ &errors, &accessors, &states });
    try std.testing.expect(num_valid > num_clouds / 2 and num_errors > num_clouds / 40 and num_hull_failed > 10);
    try std.testing.expect(num_skipped < num_clouds / 50);
}

test "ConvexHullShape parity: bounds, mass properties, volume, scales, surface normal, supporting face" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed4923 } };
    var checker: Checker = .{ .name = "properties" };
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const extent = extentOf(&input);
        const com = pair.shape().getCenterOfMass();
        for (0..8) |q| {
            var props: PropertiesInput = .{
                .scale = if (gen.oneIn(3)) gen.anyScale() else gen.scale(),
                .transform = arr16(gen.transform(10)),
                .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
                .point = gen.vec(-1.5 * extent, 1.5 * extent),
                .direction = gen.direction(3),
                .sub_shape_id = if (gen.oneIn(2)) 0xffffffff else gen.next(),
            };
            // Surface normals of the points of the hull (on the surface)
            if (q < 2) {
                const hull = pair.shape().cast(ConvexHullShape);
                props.point = arr3(hull.getPoint(@intCast(gen.index(hull.getNumPoints()))));
            } else if (gen.oneIn(4)) props.point = arr3(vec3(input.points[gen.index(input.num_points)]).sub(com));
            var jolt_output = std.mem.zeroes(PropertiesOutput);
            jolt.jolt_chs_properties(pair.jolt.?, &props, &jolt_output);
            const zolt_output = zoltProperties(pair.shape(), &props);
            checker.check(.{ input.num_points, props }, zolt_output, jolt_output);
        }
    }
    try checker.finish();
}

test "ConvexHullShape parity: support functions of every mode (shrunk hulls, scaled)" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed5a34 } };
    var checker: Checker = .{ .name = "support" };
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const convex = pair.shape().cast(ConvexShape);
        for (0..3) |_| {
            const scale = gen.scale();
            var directions: [16 * 3]f32 = undefined;
            for (0..16) |d| directions[3 * d ..][0..3].* = gen.direction(3);
            for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .include_convex_radius, .default }) |mode| {
                var jolt_points: [16 * 3]f32 = @splat(0);
                const jolt_radius = jolt.jolt_chs_support(pair.jolt.?, @intFromEnum(mode), &scale, &directions, 16, &jolt_points);
                var buffer: ConvexShape.SupportBuffer = .{};
                const support = convex.getSupportFunction(mode, &buffer, vec3(scale));
                var zolt_points: [16 * 3]f32 = @splat(0);
                for (0..16) |d| zolt_points[3 * d ..][0..3].* = arr3(support.getSupport(vec3(directions[3 * d ..][0..3].*)));
                checker.check(.{ input.num_points, mode, scale }, .{ support.getConvexRadius(), zolt_points }, .{ jolt_radius, jolt_points });
            }
        }
    }
    try checker.finish();
}

test "ConvexHullShape parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed6b45 } };
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_hits: usize = 0;
    var num_rays: usize = 0;
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const hull = pair.shape().cast(ConvexHullShape);
        const extent = @max(extentOf(&input), 1.0e-3);
        for (0..16) |q| {
            var ray: RayInput = .{
                .origin = gen.vec(-2 * extent - 0.1, 2 * extent + 0.1),
                .direction = gen.direction(4 * extent + 0.2),
                .creator = gen.creator(),
                .fraction = if (gen.oneIn(2)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
                .back_face_mode = @intFromBool(gen.oneIn(2)),
                .treat_convex_as_solid = @intFromBool(!gen.oneIn(3)),
                .collector = @intCast(gen.index(3)),
                .early_out = if (gen.oneIn(2)) 2.0 else gen.plain(0, 1),
                .body_id = gen.next() & 0x7fffff,
            };
            switch (q % 4) {
                0 => {
                    // Aim at a point inside the hull
                    const target = vec3(gen.plainVec(-0.3 * extent, 0.3 * extent));
                    ray.direction = arr3(target.sub(vec3(ray.origin)).mulScalar(gen.plain(0.5, 3)));
                },
                1 => {
                    // Aim at a vertex of the hull (grazing, through edges)
                    const target = hull.getPoint(@intCast(gen.index(hull.getNumPoints())));
                    ray.direction = arr3(target.sub(vec3(ray.origin)).mulScalar(if (gen.oneIn(2)) 1.0 else gen.plain(0.5, 3)));
                },
                2 => {
                    // Parallel to a face, sometimes in its plane, or starting inside
                    const plane = hull.getPlanes()[gen.index(hull.getNumFaces())];
                    const in_plane = vec3(gen.vec(-extent, extent));
                    const on_plane = in_plane.sub(plane.getNormal().mulScalar(plane.signedDistance(in_plane)));
                    const tangent = plane.getNormal().getNormalizedPerpendicular();
                    if (gen.oneIn(2)) {
                        ray.origin = arr3(on_plane.add(plane.getNormal().mulScalar(if (gen.oneIn(2)) 0.0 else gen.plain(-0.1, 0.1))).sub(tangent.mulScalar(2 * extent)));
                        ray.direction = arr3(tangent.mulScalar(4 * extent));
                    } else {
                        ray.origin = gen.plainVec(-0.1 * extent, 0.1 * extent);
                    }
                },
                else => {},
            }
            var jolt_output = std.mem.zeroes(RayOutput);
            jolt.jolt_chs_cast_ray(pair.jolt.?, &ray, &jolt_output);
            const zolt_output = try zoltCastRay(allocator, pair.shape(), &ray);
            num_hits += @intCast(zolt_output.hit);
            num_rays += 1;
            rays.check(.{ input.num_points, ray }, zolt_output, jolt_output);

            // Points inside, on the vertices / faces and outside
            var point = gen.vec(-1.5 * extent, 1.5 * extent);
            if (gen.oneIn(4)) point = arr3(hull.getPoint(@intCast(gen.index(hull.getNumPoints()))));
            if (gen.oneIn(4)) {
                const plane = hull.getPlanes()[gen.index(hull.getNumFaces())];
                point = arr3(vec3(point).sub(plane.getNormal().mulScalar(plane.signedDistance(vec3(point)))));
            }
            var jolt_ids: [2]u32 = undefined;
            const jolt_count = jolt.jolt_chs_collide_point(pair.jolt.?, &point, &ray.creator, ray.body_id, &jolt_ids);
            var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer collector.deinit();
            const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(ray.body_id), .{});
            collector.base.setContext(&context);
            pair.shape().collidePoint(vec3(point), makeCreator(ray.creator), &collector.base, &.{});
            try collector.checkError();
            var zolt_ids: [2]u32 = .{ 0, 0 };
            for (collector.hits.items) |h| zolt_ids = .{ h.body_id.getIndexAndSequenceNumber(), h.sub_shape_id2.getValue() };
            points.check(.{ input.num_points, point }, .{ @as(u32, @intCast(collector.hits.items.len)), zolt_ids }, .{ jolt_count, jolt_ids });
        }
    }
    try finishAll(&.{ &rays, &points });
    try std.testing.expect(num_hits > num_rays / 5 and num_hits < 4 * num_rays / 5); // Hits and misses
}

/// The other shape of a collide / cast test: a sphere, a box, another hull or the hull itself
fn otherShape(allocator: Allocator, gen: *Gen, clouds: *CloudGen, out_extent: *f32, out_is_sphere: *bool) !?Pair {
    out_is_sphere.* = false;
    switch (gen.index(4)) {
        0 => {
            const radius = gen.plain(0.05, 2);
            out_extent.* = radius;
            out_is_sphere.* = true;
            return try createSphere(allocator, radius, 1000.0);
        },
        1 => {
            const half_extent = gen.plainVec(0.05, 2);
            out_extent.* = vec3(half_extent).length();
            return try createBox(allocator, half_extent, if (gen.oneIn(2)) 0.0 else gen.plain(0, 0.1), 1000.0);
        },
        2 => {
            var input: HullInput = undefined;
            const other = try nextValidHull(allocator, clouds, &input);
            out_extent.* = extentOf(&input);
            return other;
        },
        else => return null, // The hull itself
    }
}

test "ConvexHullShape parity: collide hull vs sphere / box / hull / itself through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed7c56 } };
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    var num_queries: usize = 0;
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const extent = extentOf(&input);
        var other_extent: f32 = extent;
        var other_is_sphere = false;
        var other = try otherShape(allocator, &gen, &clouds, &other_extent, &other_is_sphere);
        defer if (other) |*o| o.deinit();
        const other_pair = if (other) |*o| o else &pair;

        for (0..4) |_| {
            // Shape 2 near shape 1 so that about half of the pairs collide; sometimes the hull is shape 2
            const swap = gen.oneIn(2);
            const hull_scale = gen.scale();
            const other_scale = if (other_is_sphere) gen.uniformScale() else gen.scale();
            const transform1 = gen.transform(5);
            var relative = gen.transform(0.8 * (extent + other_extent));
            if (gen.oneIn(10)) relative.setTranslation(Vec3.zero());
            const input_collide: CollideInput = .{
                .scale1 = if (swap) other_scale else hull_scale,
                .scale2 = if (swap) hull_scale else other_scale,
                .transform1 = arr16(transform1),
                .transform2 = arr16(transform1.mul(relative)),
                .creator1 = gen.creator(),
                .creator2 = gen.creator(),
                .max_separation_distance = switch (gen.index(5)) {
                    0, 1 => 0.0,
                    2 => gen.plain(0, 1),
                    3 => gen.plain(1, 3),
                    else => 100.0,
                },
                .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
                .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
                .active_edge_mode = @intFromBool(gen.oneIn(2)),
                .back_face_mode = @intFromBool(gen.oneIn(2)),
                .collect_faces = @intFromBool(gen.oneIn(2)),
                .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
                .body_id = gen.next() & 0x7fffff,
            };
            const first = if (swap) other_pair else &pair;
            const second = if (swap) &pair else other_pair;
            var jolt_output = std.mem.zeroes(HitsOutput);
            jolt.jolt_chs_collide(first.jolt.?, second.jolt.?, &input_collide, &jolt_output);
            const zolt_output = try zoltCollide(allocator, first.shape(), second.shape(), &input_collide);
            num_hits += zolt_output.num_hits;
            num_queries += 1;
            checker.check(.{ input.num_points, swap, input_collide }, zolt_output, jolt_output);
        }
    }
    try checker.finish();
    try std.testing.expect(num_hits > num_queries / 4 and num_hits < 3 * num_queries / 4); // Both paths are exercised
}

test "ConvexHullShape parity: cast hull vs sphere / box / hull / itself through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed8d67 } };
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    var num_queries: usize = 0;
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const extent = extentOf(&input);
        var other_extent: f32 = extent;
        var other_is_sphere = false;
        var other = try otherShape(allocator, &gen, &clouds, &other_extent, &other_is_sphere);
        defer if (other) |*o| o.deinit();
        const other_pair = if (other) |*o| o else &pair;

        for (0..4) |_| {
            const swap = gen.oneIn(2);
            const hull_scale = gen.scale();
            const other_scale = if (other_is_sphere) gen.uniformScale() else gen.scale();
            const reach = extent + other_extent;
            const transform2 = gen.transform(5);
            // Start near shape 2 (sometimes overlapping), move towards it, past it or away from it
            var start = transform2.mul(gen.transform(2.0 * reach));
            if (gen.oneIn(8)) start.setTranslation(transform2.getTranslation().add(vec3(gen.plainVec(-0.2, 0.2))));
            const to_target = transform2.getTranslation().sub(start.getTranslation());
            const direction = switch (gen.index(4)) {
                0 => to_target.mulScalar(gen.plain(0.5, 3)),
                1 => to_target.mulScalar(gen.plain(-1, 0.2)),
                2 => vec3(gen.direction(3 * reach)),
                else => to_target.add(vec3(gen.plainVec(-reach, reach))).mulScalar(gen.plain(0.5, 2)),
            };
            const input_cast: CastInput = .{
                .scale1 = if (swap) other_scale else hull_scale,
                .start = arr16(start),
                .direction = arr3(direction),
                .scale2 = if (swap) hull_scale else other_scale,
                .transform2 = arr16(transform2),
                .creator1 = gen.creator(),
                .creator2 = gen.creator(),
                .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
                .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
                .extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0,
                .active_edge_mode = @intFromBool(gen.oneIn(2)),
                .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
                .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
                .return_deepest_point = @intFromBool(gen.oneIn(2)),
                .collect_faces = @intFromBool(gen.oneIn(2)),
                .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
                .body_id = gen.next() & 0x7fffff,
            };
            const first = if (swap) other_pair else &pair;
            const second = if (swap) &pair else other_pair;
            var jolt_output = std.mem.zeroes(HitsOutput);
            jolt.jolt_chs_cast(first.jolt.?, second.jolt.?, &input_cast, &jolt_output);
            const zolt_output = try zoltCast(allocator, first.shape(), second.shape(), &input_cast);
            num_hits += zolt_output.num_hits;
            num_queries += 1;
            checker.check(.{ input.num_points, swap, input_cast }, zolt_output, jolt_output);
        }
    }
    try checker.finish();
    try std.testing.expect(num_hits > num_queries / 5 and num_hits < 4 * num_queries / 5); // Hits and misses
}

test "ConvexHullShape parity: GetSubmergedVolume" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51ed9e78 } };
    var checker: Checker = .{ .name = "submerged volume" };
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const extent = extentOf(&input);
        for (0..8) |_| {
            const scale = gen.scale();
            const transform = arr16(gen.transform(3));
            const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
            // A surface through the shape most of the time
            const surface_point = if (gen.oneIn(4)) gen.vec(-5 - extent, 5 + extent) else arr3(mat44(transform).getTranslation().add(vec3(gen.plainVec(-extent, extent))));
            const plane = arr4(Plane.fromPointAndNormal(vec3(surface_point), normal).normal_and_constant);
            var jolt_values: [5]f32 = undefined;
            jolt.jolt_chs_submerged_volume(pair.jolt.?, &transform, &scale, &plane, &jolt_values);
            const r = pair.shape().getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(plane)));
            const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
            checker.check(.{ input.num_points, scale, transform, plane }, zolt_values, jolt_values);
        }
    }
    try checker.finish();
}

test "ConvexHullShape parity: GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51edaf89 } };
    var checker: Checker = .{ .name = "triangles" };
    const jolt_vertices = try allocator.create([max_triangles * 9]f32);
    defer allocator.destroy(jolt_vertices);
    const zolt_vertices = try allocator.create([max_triangles * 9]f32);
    defer allocator.destroy(zolt_vertices);
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        for (0..2) |_| {
            const scale = gen.scale();
            const position = gen.vec(-10, 10);
            const rotation = arr4(gen.rotation().getXYZW());
            const max_requested: c_int = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(200));

            var jolt_counts: [64]c_int = @splat(0);
            @memset(jolt_vertices, 0);
            var jolt_default: [max_triangles]c_int = @splat(0);
            const jolt_calls = jolt.jolt_chs_triangles(pair.jolt.?, &position, &rotation, &scale, max_requested, &jolt_counts, jolt_vertices, &jolt_default);

            var zolt_counts: [64]c_int = @splat(0);
            @memset(zolt_vertices, 0);
            var zolt_default: [max_triangles]c_int = @splat(0);
            var context: Shape.GetTrianglesContext = .{};
            pair.shape().getTrianglesStart(&context, AABox.biggest(), vec3(position), quat(rotation), vec3(scale));
            var triangles: [3 * 232]Float3 = undefined;
            var materials: [232]*const PhysicsMaterial = undefined;
            var zolt_calls: c_int = 0;
            var total: usize = 0;
            while (true) {
                const count = pair.shape().getTrianglesNext(&context, @intCast(max_requested), triangles[0 .. 3 * @as(usize, @intCast(max_requested))], materials[0..@intCast(max_requested)]);
                zolt_counts[@intCast(zolt_calls)] = @intCast(count);
                zolt_calls += 1;
                for (0..count) |i| {
                    if (total >= max_triangles) break;
                    for (0..3) |v| {
                        const t = triangles[3 * i + v];
                        zolt_vertices[9 * total + 3 * v ..][0..3].* = .{ t.x, t.y, t.z };
                    }
                    zolt_default[total] = @intFromBool(materials[i] == PhysicsMaterial.default);
                    total += 1;
                }
                if (count == 0 or zolt_calls == 64) break;
            }
            checker.check(.{ input.num_points, scale, position, rotation, max_requested }, .{ zolt_calls, zolt_counts, zolt_vertices.*, zolt_default }, .{ jolt_calls, jolt_counts, jolt_vertices.*, jolt_default });
        }
    }
    try checker.finish();
}

test "ConvexHullShape parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var clouds: CloudGen = .{};
    var gen: Gen = .{ .rng = .{ .state = 0x51edc09a } };
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 16;
    for (0..num_hulls) |_| {
        var input: HullInput = undefined;
        var pair = try nextValidHull(allocator, &clouds, &input);
        defer pair.deinit();
        const hull = pair.shape().cast(ConvexHullShape);
        const extent = extentOf(&input) * 1.5;
        for (0..2) |_| {
            const scale = gen.scale();
            const transform = arr16(gen.transform(3));
            var positions: [n * 3]f32 = undefined;
            var inv_masses: [n]f32 = undefined;
            var penetrations: [n]f32 = undefined;
            const planes: [n * 4]f32 = @splat(0);
            const indices: [n]c_int = @splat(-1);
            for (0..n) |v| {
                var local = vec3(gen.vec(-extent, extent));
                if (gen.oneIn(6)) local = hull.getPoint(@intCast(gen.index(hull.getNumPoints()))).mul(vec3(scale)).mul(Vec3.replicate(gen.plain(0.8, 1.2))); // Near a vertex
                if (gen.oneIn(8)) local = Vec3.zero(); // At the center
                positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(local));
                inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
                penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
            }
            var jolt_penetrations = penetrations;
            var jolt_planes = planes;
            var jolt_indices = indices;
            jolt.jolt_chs_soft_body(pair.jolt.?, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

            var zolt_positions: [n]Vec3 = undefined;
            var zolt_planes: [n]Plane = undefined;
            var zolt_penetrations = penetrations;
            var zolt_indices: [n]i32 = undefined;
            for (0..n) |v| {
                zolt_positions[v] = vec3(positions[3 * v ..][0..3].*);
                zolt_planes[v] = .fromVec4(vec4(planes[4 * v ..][0..4].*));
                zolt_indices[v] = indices[v];
            }
            const vertices = CollideSoftBodyVertexIterator.init(.init(&zolt_positions[0], .{}), .init(&inv_masses[0], .{}), .init(&zolt_planes[0], .{}), .init(&zolt_penetrations[0], .{}), .init(&zolt_indices[0], .{}));
            pair.shape().collideSoftBodyVertices(mat44(transform), vec3(scale), &vertices, n, 3);
            var zolt_plane_values: [n * 4]f32 = undefined;
            for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
            var zolt_index_values: [n]c_int = undefined;
            for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
            checker.check(.{ input.num_points, scale, transform, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
        }
    }
    try checker.finish();
}
