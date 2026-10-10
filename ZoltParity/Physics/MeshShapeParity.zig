//! Parity tests for MeshShape (Phase 4, Wave B): MeshShapeSettings (the constructors with Indexify and Sanitize) and
//! MeshShape (construction with Jolt's error texts, the active edges, the tree built with AABBTreeBuilder +
//! AABBTreeToBuffer, the properties, the per triangle queries, both CastRay overloads, CollidePoint, collide and cast of
//! spheres and boxes vs the mesh and of the mesh vs them through CollisionDispatch (and InternalEdgeRemovingCollector),
//! GetTrianglesStart / Next, CollideSoftBodyVertices, GetSubmergedVolume (only without asserts: Jolt asserts) and the
//! binary state incl. SaveWithChildren). C ABI wrappers: ZoltParity/Physics/MeshShapeReference.cpp.
//!
//! Both sides write their results into a stream of u32 (see `Stream`, the layout of every function is the same as in
//! MeshShapeReference.cpp) and the streams must be identical. The meshes are random grids (flat, terraced, noisy, with
//! random diagonals and big offsets), soups of random triangles with shared vertices, closed meshes (boxes, UV spheres)
//! and a big grid, with materials (up to 32, more for the error), user data, degenerate and duplicate triangles,
//! every construction mode (arrays set directly, the indexed constructor, the triangle list constructor with vertices
//! that Indexify welds) and random settings (max triangles per leaf, the active edge threshold incl. negative values,
//! per triangle user data, build quality). The queries use random inputs mixed with hand picked edge cases: rays
//! through vertices and along edges, parallel to faces, starting inside or on the surface, zero length rays, touching
//! and deeply penetrating shapes, scales with negative and non uniform components, back face and active edge modes,
//! max separation distances, collected faces and early out fractions.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

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
const Color = zolt.Color;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const InternalEdgeRemovingCollector = zolt.InternalEdgeRemovingCollector;
const Mat44 = zolt.Mat44;
const MeshShape = zolt.MeshShape;
const MeshShapeSettings = zolt.MeshShapeSettings;
const NodeCodec = zolt.NodeCodecQuadTreeHalfFloat;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialList = zolt.PhysicsMaterialList;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const RefConst = zolt.RefConst;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const StridedPtr = zolt.StridedPtr;
const StridedPtrConst = zolt.StridedPtrConst;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const Triangle = zolt.Triangle;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see MeshShapeReference.cpp
const jolt = struct {
    extern fn jolt_mesh_create(input: *const MeshInput, out: [*]u32, capacity: u32, out_size: *u32) ?*anyopaque;
    extern fn jolt_mesh_destroy(handle: *anyopaque) void;
    extern fn jolt_mesh_properties(handle: *anyopaque, input: *const PropertiesInput, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_sub_shapes(handle: *anyopaque, ids: [*]const u32, num_ids: u32, input: *const SubShapeInput, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_cast_ray(handle: *anyopaque, input: *const RayInput, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_collide_point(handle: *anyopaque, point: *const P, creator: *const [2]u32, body_id: u32, reject_all: c_int, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_collide(handle: *anyopaque, input: *const CollideInput, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_cast(handle: *anyopaque, input: *const CastInput, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_triangles(handle: *anyopaque, input: *const TrianglesInput, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_soft_body(handle: *anyopaque, transform: *const [16]f32, scale: *const P, num_vertices: u32, positions: [*]const f32, inv_masses: [*]const f32, planes: [*]const f32, penetrations: [*]const f32, indices: [*]const c_int, colliding_shape_index: c_int, out: [*]u32, capacity: u32) u32;
    extern fn jolt_mesh_binary_state(handle: *anyopaque, ray: *const RayInput, out: [*]u32, capacity: u32) u32;
};

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Number of materials that the tests can use, must match cNumMaterials in MeshShapeReference.cpp
const num_parity_materials = 40;

/// Capacity of the C++ output stream (in u32)
const stream_capacity = 1 << 22;

// Section markers in the streams, must match MeshShapeReference.cpp
const marker_settings: u32 = 0xB0000001;
const marker_result: u32 = 0xB0000002;
const marker_properties: u32 = 0xB0000003;
const marker_sub_shape: u32 = 0xB0000004;
const marker_ray: u32 = 0xB0000005;
const marker_ray_collector: u32 = 0xB0000006;
const marker_point: u32 = 0xB0000007;
const marker_collide: u32 = 0xB0000008;
const marker_cast: u32 = 0xB0000009;
const marker_triangles: u32 = 0xB000000A;
const marker_soft_body: u32 = 0xB000000B;
const marker_binary_state: u32 = 0xB000000C;
const marker_submerged: u32 = 0xB000000D;
const marker_hit: u32 = 0xB000000E;

/// Must match MeshInput in MeshShapeReference.cpp
const MeshInput = extern struct {
    vertices: [*]const Float3,
    triangles: [*]const IndexedTriangle,
    triangle_list: [*]const Triangle,
    user_data: u64,
    num_vertices: u32,
    num_triangles: u32,
    num_triangle_list: u32,
    /// 0: arrays set directly, 1: the indexed constructor, 2: the triangle list constructor
    mode: u32,
    /// The first num_materials of the parity materials (modulo num_parity_materials)
    num_materials: u32,
    max_triangles_per_leaf: u32,
    active_edge_cos_threshold_angle: f32,
    per_triangle_user_data: c_int,
    build_quality: u32,
    padding: u32 = 0,
};

/// Must match ConvexDesc in MeshShapeReference.cpp
const ConvexDesc = extern struct {
    /// 0: SphereShape, 1: BoxShape
    kind: u32,
    radius: f32 = 0.0,
    half_extent: P = .{ 0, 0, 0 },
    convex_radius: f32 = 0.0,
};

/// Must match PropertiesInput in MeshShapeReference.cpp
const PropertiesInput = extern struct {
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    scale: P,
    /// GetSubmergedVolume
    surface_plane: [4]f32,
    /// Call GetSubmergedVolume (only when asserts are off)
    submerged: c_int,
};

/// Must match SubShapeInput in MeshShapeReference.cpp
const SubShapeInput = extern struct {
    transform: [16]f32,
    scale: P,
    direction: P,
    position: P,
    rotation: [4]f32,
    point: P,
};

/// Must match RayInput in MeshShapeReference.cpp
const RayInput = extern struct {
    origin: P,
    direction: P,
    /// Sub shape ID creator: value pushed, number of bits
    creator: [2]u32,
    /// Initial fraction of the single hit version
    fraction: f32,
    /// 1: collide with back faces (triangles)
    back_face_mode: c_int,
    /// 0: AllHit, 1: AnyHit, 2: ClosestHit
    collector: c_int,
    /// Early out fraction of the collector (when < the initial one)
    early_out: f32,
    /// Body ID of the collector context
    body_id: u32,
    /// Use a shape filter that rejects everything
    reject_all: c_int,
};

/// Must match CollideInput in MeshShapeReference.cpp
const CollideInput = extern struct {
    convex: ConvexDesc,
    /// 1: the mesh is shape 1 (reversed)
    mesh_first: c_int,
    scale_convex: P,
    scale_mesh: P,
    transform_convex: [16]f32,
    transform_mesh: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    /// 1: CollideWithBackFaces
    back_face_mode: c_int,
    /// 1: CollideWithAll
    active_edge_mode: c_int,
    collect_faces: c_int,
    active_edge_movement_direction: P,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
    /// Use InternalEdgeRemovingCollector::sCollideShapeVsShape
    internal_edge_removal: c_int,
    /// mInternalEdgeRemovalVertexToleranceSq
    vertex_tolerance_sq: f32,
};

/// Must match CastInput in MeshShapeReference.cpp
const CastInput = extern struct {
    convex: ConvexDesc,
    /// 1: the mesh is the cast shape, the convex shape the target
    mesh_cast: c_int,
    scale_cast: P,
    start: [16]f32,
    direction: P,
    scale_target: P,
    transform_target: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    extra_convex_radius: f32,
    back_face_mode_triangles: c_int,
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// 1: CollideWithAll
    active_edge_mode: c_int,
    active_edge_movement_direction: P,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

/// Must match TrianglesInput in MeshShapeReference.cpp
const TrianglesInput = extern struct {
    box: [6]f32,
    position: P,
    rotation: [4]f32,
    scale: P,
    max_triangles_requested: c_int,
    /// Request the materials
    materials: c_int,
};

// ---------------------------------------------------------------------------------------------------------------------
// Conversions

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

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn quat(a: [4]f32) Quat {
    return Quat.init(a[0], a[1], a[2], a[3]);
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn makeID(value: u32) SubShapeID {
    var id: SubShapeID = .empty;
    id.setValue(value);
    return id;
}

// ---------------------------------------------------------------------------------------------------------------------
// The output stream (the same layout as OutStream in MeshShapeReference.cpp)

const Stream = struct {
    allocator: Allocator,
    values: std.ArrayList(u32) = .empty,

    fn deinit(s: *Stream) void {
        s.values.deinit(s.allocator);
    }

    fn u(s: *Stream, v: u32) void {
        s.values.append(s.allocator, v) catch @panic("out of memory");
    }

    fn i(s: *Stream, v: i32) void {
        s.u(@bitCast(v));
    }

    fn b(s: *Stream, v: bool) void {
        s.u(@intFromBool(v));
    }

    fn f(s: *Stream, v: f32) void {
        s.u(@bitCast(v));
    }

    fn d(s: *Stream, v: f64) void {
        const bits: u64 = @bitCast(v);
        s.u(@truncate(bits));
        s.u(@truncate(bits >> 32));
    }

    fn v3(s: *Stream, v: Vec3) void {
        s.f(v.getX());
        s.f(v.getY());
        s.f(v.getZ());
    }

    fn f3(s: *Stream, v: Float3) void {
        s.f(v.x);
        s.f(v.y);
        s.f(v.z);
    }

    fn r3(s: *Stream, v: RVec3) void {
        s.d(v.getX());
        s.d(v.getY());
        s.d(v.getZ());
    }

    fn q(s: *Stream, v: Quat) void {
        s.f(v.getX());
        s.f(v.getY());
        s.f(v.getZ());
        s.f(v.getW());
    }

    fn m(s: *Stream, v: Mat44) void {
        for (0..4) |c|
            for (0..4) |r|
                s.f(v.get(@intCast(r), @intCast(c)));
    }

    fn box(s: *Stream, v: AABox) void {
        s.v3(v.min);
        s.v3(v.max);
    }

    fn plane(s: *Stream, v: Plane) void {
        s.v3(v.getNormal());
        s.f(v.getConstant());
    }

    fn str(s: *Stream, v: []const u8) void {
        s.u(@intCast(v.len));
        for (v) |c| s.u(c);
    }

    fn bytes(s: *Stream, v: []const u8) void {
        s.str(v);
    }

    fn face(s: *Stream, v: *const Shape.SupportingFace) void {
        s.u(v.len);
        for (v.constSlice()) |p| s.v3(p);
    }
};

/// The parity materials ("Material i" with Color.getDistinctColor(i), the same as GetMaterials() in C++)
const Materials = struct {
    refs: [num_parity_materials]RefConst(PhysicsMaterial) = undefined,
    ptrs: [num_parity_materials]*const PhysicsMaterial = undefined,

    fn init(self: *Materials, allocator: Allocator) !void {
        for (0..num_parity_materials) |i| {
            var name_buffer: [32]u8 = undefined;
            const name = try std.fmt.bufPrint(&name_buffer, "Material {d}", .{i});
            const material = try PhysicsMaterialSimple.create(allocator, name, Color.getDistinctColor(@intCast(i)));
            self.ptrs[i] = material.material();
            self.refs[i] = .init(self.ptrs[i]);
        }
    }

    fn deinit(self: *Materials) void {
        for (&self.refs) |*r| r.deinit();
    }

    /// WriteMaterial in C++
    fn write(self: *const Materials, s: *Stream, material: *const PhysicsMaterial) void {
        if (material == PhysicsMaterial.default) {
            s.u(0xffffffff);
            return;
        }
        for (self.ptrs, 0..) |p, i|
            if (p == material) {
                s.u(@intCast(i));
                return;
            };
        s.u(0xfffffffe);
        s.str(material.getDebugName());
        s.u(material.getDebugColor().getUInt32());
    }
};

fn writeCollideHit(s: *Stream, r: *const CollideShapeResult) void {
    s.u(marker_hit);
    s.v3(r.contact_point_on1);
    s.v3(r.contact_point_on2);
    s.v3(r.penetration_axis);
    s.f(r.penetration_depth);
    s.u(r.sub_shape_id1.getValue());
    s.u(r.sub_shape_id2.getValue());
    s.u(r.body_id2.getIndexAndSequenceNumber());
    s.face(&r.shape1_face);
    s.face(&r.shape2_face);
}

fn createConvex(allocator: Allocator, desc: ConvexDesc) !RefConst(Shape) {
    var result = if (desc.kind == 0) blk: {
        var settings = SphereShapeSettings.init(allocator, desc.radius, .{});
        defer settings.deinit();
        break :blk try settings.asShapeSettings().createShape(allocator);
    } else blk: {
        var settings = BoxShapeSettings.init(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius });
        defer settings.deinit();
        break :blk try settings.asShapeSettings().createShape(allocator);
    };
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr().?);
}

fn saveBinaryState(allocator: Allocator, shape: *const Shape) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var stream = StreamOutWrapper.init(&out.writer);
    shape.saveBinaryState(stream.streamOut());
    return out.toOwnedSlice();
}

fn saveWithChildren(allocator: Allocator, shape: *const Shape) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var stream = StreamOutWrapper.init(&out.writer);
    var shape_map: Shape.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: Shape.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    try shape.saveWithChildren(allocator, stream.streamOut(), &shape_map, &material_map);
    return out.toOwnedSlice();
}

/// A shape filter that rejects everything
const RejectAllFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: ShapeFilter = .init(@This()),

    pub fn shouldCollide(self: *const RejectAllFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ self, shape2, sub_shape_id_of_shape2 };
        return false;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of every C++ function

/// jolt_mesh_create: returns a reference to the shape or null when the settings are invalid
fn zoltCreate(allocator: Allocator, materials: *const Materials, in: *const MeshInput, s: *Stream) !?RefConst(Shape) {
    var material_list: [64]*const PhysicsMaterial = undefined;
    for (0..in.num_materials) |i| material_list[i] = materials.ptrs[i % num_parity_materials];
    const mesh_materials = material_list[0..in.num_materials];

    var settings = switch (in.mode) {
        0 => blk: {
            var settings = MeshShapeSettings.initDefault(allocator);
            errdefer settings.deinit();
            try settings.triangle_vertices.appendSlice(allocator, in.vertices[0..in.num_vertices]);
            try settings.indexed_triangles.appendSlice(allocator, in.triangles[0..in.num_triangles]);
            for (mesh_materials) |mat| try settings.materials.append(allocator, .init(mat));
            break :blk settings;
        },
        1 => try MeshShapeSettings.initIndexed(allocator, in.vertices[0..in.num_vertices], in.triangles[0..in.num_triangles], .{ .materials = mesh_materials }),
        else => try MeshShapeSettings.init(allocator, in.triangle_list[0..in.num_triangle_list], .{ .materials = mesh_materials }),
    };
    defer settings.deinit();
    settings.max_triangles_per_leaf = in.max_triangles_per_leaf;
    settings.active_edge_cos_threshold_angle = in.active_edge_cos_threshold_angle;
    settings.per_triangle_user_data = in.per_triangle_user_data != 0;
    settings.build_quality = @enumFromInt(in.build_quality);
    settings.base.user_data = in.user_data;

    // The settings after the constructor
    s.u(marker_settings);
    s.u(@intCast(settings.triangle_vertices.items.len));
    for (settings.triangle_vertices.items) |v| s.f3(v);
    s.u(@intCast(settings.indexed_triangles.items.len));
    for (settings.indexed_triangles.items) |t| {
        s.u(t.idx[0]);
        s.u(t.idx[1]);
        s.u(t.idx[2]);
        s.u(t.material_index);
        s.u(t.user_data);
    }
    s.u(@intCast(settings.materials.items.len));

    // The result
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    s.u(marker_result);
    if (result.hasError()) {
        s.u(0);
        s.str(result.getError());
        return null;
    }
    s.u(1);
    const shape: *const Shape = result.getPtr().?;
    const mesh = shape.cast(MeshShape);

    // The properties that don't depend on an input
    s.u(marker_properties);
    s.u(@intFromEnum(shape.getType()));
    s.u(@intFromEnum(shape.getSubType()));
    s.b(shape.mustBeStatic());
    s.box(shape.getLocalBounds());
    s.v3(shape.getCenterOfMass());
    s.u(shape.getSubShapeIDBitsRecursive());
    s.f(shape.getInnerRadius());
    s.f(shape.getVolume());
    const p = shape.getMassProperties();
    s.f(p.mass);
    s.m(p.inertia);
    const stats = shape.getStats();
    s.u(stats.num_triangles);
    s.u(@intCast(stats.size_bytes - @sizeOf(MeshShape)));
    s.u(@intCast(mesh.getMaterialList().len));
    for (mesh.getMaterialList()) |mat| materials.write(s, mat.get().?);
    s.u(@truncate(shape.getUserData()));
    s.u(@truncate(shape.getUserData() >> 32));

    // The binary state contains the tree
    const state = try saveBinaryState(allocator, shape);
    defer allocator.free(state);
    s.bytes(state);

    return RefConst(Shape).init(shape);
}

/// jolt_mesh_properties
fn zoltProperties(shape: *const Shape, in: *const PropertiesInput, s: *Stream) void {
    const transform = mat44(in.transform);
    const scale = vec3(in.scale);
    s.u(marker_properties);
    s.box(shape.getWorldSpaceBounds(transform, scale));
    s.box(shape.getWorldSpaceBoundsDMat44(DMat44.fromMat44Translation(transform, DVec3.init(in.translation[0], in.translation[1], in.translation[2])), scale));
    s.b(shape.isValidScale(scale));
    s.v3(shape.makeScaleValid(scale));
    if (in.submerged != 0) {
        const v = shape.getSubmergedVolume(transform, scale, Plane.fromVec4(vec4(in.surface_plane)));
        s.u(marker_submerged);
        s.f(v.total_volume);
        s.f(v.submerged_volume);
        s.v3(v.center_of_buoyancy);
    }
}

/// jolt_mesh_sub_shapes
fn zoltSubShapes(materials: *const Materials, shape: *const Shape, ids: []const u32, in: *const SubShapeInput, s: *Stream) void {
    const mesh = shape.cast(MeshShape);
    const transform = mat44(in.transform);
    const scale = vec3(in.scale);
    for (ids) |value| {
        const id = makeID(value);
        s.u(marker_sub_shape);
        materials.write(s, shape.getMaterial(id));
        s.u(mesh.getMaterialIndex(id));
        s.u(mesh.getTriangleUserData(id));
        s.v3(shape.getSurfaceNormal(id, vec3(in.point)));
        var face: Shape.SupportingFace = .empty;
        shape.getSupportingFace(id, vec3(in.direction), scale, transform, &face);
        s.face(&face);
        const leaf = shape.getLeafShape(id);
        s.b(leaf.shape == shape);
        s.u(leaf.remainder.getValue());
        s.u(@truncate(shape.getSubShapeUserData(id)));
        var ts = shape.getSubShapeTransformedShape(id, vec3(in.position), quat(in.rotation), scale);
        defer ts.transformed_shape.deinit();
        s.b(ts.transformed_shape.shape.get() == shape);
        s.r3(ts.transformed_shape.shape_position_com);
        s.q(ts.transformed_shape.shape_rotation);
        s.v3(ts.transformed_shape.getShapeScale());
        s.u(ts.transformed_shape.body_id.getIndexAndSequenceNumber());
        s.u(ts.transformed_shape.sub_shape_id_creator.getID().getValue());
        s.u(ts.remainder.getValue());
    }
}

/// jolt_mesh_cast_ray
fn zoltCastRay(allocator: Allocator, shape: *const Shape, in: *const RayInput, s: *Stream) !void {
    const ray = RayCast.init(vec3(in.origin), vec3(in.direction));
    const creator = makeCreator(in.creator);

    var hit: RayCastResult = .{};
    hit.fraction = in.fraction;
    s.u(marker_ray);
    s.b(shape.castRay(ray, creator, &hit));
    s.f(hit.fraction);
    s.u(hit.sub_shape_id2.getValue());

    var settings: RayCastSettings = .{};
    settings.back_face_mode_triangles = if (in.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(in.body_id), .{});
    const reject_all: RejectAllFilter = .{};
    const default_filter: ShapeFilter = .{};
    const filter: *const ShapeFilter = if (in.reject_all != 0) &reject_all.base else &default_filter;
    const write = struct {
        fn write(st: *Stream, h: *const RayCastResult) void {
            st.f(h.fraction);
            st.u(h.body_id.getIndexAndSequenceNumber());
            st.u(h.sub_shape_id2.getValue());
        }
    }.write;
    s.u(marker_ray_collector);
    switch (in.collector) {
        0 => {
            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            collector.base.setContext(&context);
            if (in.early_out < collector.base.getEarlyOutFraction())
                collector.base.updateEarlyOutFraction(in.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, filter);
            try collector.checkError();
            s.u(@intCast(collector.hits.items.len));
            for (collector.hits.items) |*h| write(s, h);
            s.f(collector.base.getEarlyOutFraction());
        },
        1 => {
            var collector = AnyHitCollisionCollector(CastRayCollector).init();
            collector.base.setContext(&context);
            if (in.early_out < collector.base.getEarlyOutFraction())
                collector.base.updateEarlyOutFraction(in.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, filter);
            s.b(collector.hadHit());
            if (collector.hadHit()) write(s, &collector.hit);
            s.f(collector.base.getEarlyOutFraction());
        },
        else => {
            var collector = ClosestHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (in.early_out < collector.base.getEarlyOutFraction())
                collector.base.updateEarlyOutFraction(in.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, filter);
            s.b(collector.hadHit());
            if (collector.hadHit()) write(s, &collector.hit);
            s.f(collector.base.getEarlyOutFraction());
        },
    }
}

/// jolt_mesh_collide_point
fn zoltCollidePoint(allocator: Allocator, shape: *const Shape, point: P, creator: [2]u32, body_id: u32, reject_all: bool, s: *Stream) !void {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(body_id), .{});
    collector.base.setContext(&context);
    const reject_filter: RejectAllFilter = .{};
    const default_filter: ShapeFilter = .{};
    shape.collidePoint(vec3(point), makeCreator(creator), &collector.base, if (reject_all) &reject_filter.base else &default_filter);
    try collector.checkError();
    s.u(marker_point);
    s.u(@intCast(collector.hits.items.len));
    for (collector.hits.items) |h| {
        s.u(h.body_id.getIndexAndSequenceNumber());
        s.u(h.sub_shape_id2.getValue());
    }
}

/// jolt_mesh_collide
fn zoltCollide(allocator: Allocator, mesh: *const Shape, in: *const CollideInput, s: *Stream) !void {
    var convex = try createConvex(allocator, in.convex);
    defer convex.deinit();
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = in.max_separation_distance;
    settings.collision_tolerance = in.collision_tolerance;
    settings.penetration_tolerance = in.penetration_tolerance;
    settings.back_face_mode = if (in.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.active_edge_mode = if (in.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.collect_faces_mode = if (in.collect_faces != 0) .collect_faces else .no_faces;
    settings.active_edge_movement_direction = vec3(in.active_edge_movement_direction);
    settings.internal_edge_removal_vertex_tolerance_sq = in.vertex_tolerance_sq;
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(in.body_id), .{});
    collector.base.setContext(&context);
    if (in.early_out < collector.base.getEarlyOutFraction())
        collector.base.updateEarlyOutFraction(in.early_out);

    const mesh_first = in.mesh_first != 0;
    const shape1 = if (mesh_first) mesh else convex.get().?;
    const shape2 = if (mesh_first) convex.get().? else mesh;
    const scale1 = vec3(if (mesh_first) in.scale_mesh else in.scale_convex);
    const scale2 = vec3(if (mesh_first) in.scale_convex else in.scale_mesh);
    const transform1 = mat44(if (mesh_first) in.transform_mesh else in.transform_convex);
    const transform2 = mat44(if (mesh_first) in.transform_convex else in.transform_mesh);
    if (in.internal_edge_removal != 0)
        try InternalEdgeRemovingCollector.collideShapeVsShape(allocator, shape1, shape2, scale1, scale2, transform1, transform2, makeCreator(in.creator1), makeCreator(in.creator2), &settings, &collector.base, &.{})
    else
        CollisionDispatch.collideShapeVsShape(shape1, shape2, scale1, scale2, transform1, transform2, makeCreator(in.creator1), makeCreator(in.creator2), &settings, &collector.base, &.{});
    try collector.checkError();

    s.u(marker_collide);
    s.u(@intCast(collector.hits.items.len));
    for (collector.hits.items) |*r| writeCollideHit(s, r);
    s.f(collector.base.getEarlyOutFraction());
}

/// jolt_mesh_cast
fn zoltCast(allocator: Allocator, mesh: *const Shape, in: *const CastInput, s: *Stream) !void {
    var convex = try createConvex(allocator, in.convex);
    defer convex.deinit();
    var settings: ShapeCastSettings = .{};
    settings.collision_tolerance = in.collision_tolerance;
    settings.penetration_tolerance = in.penetration_tolerance;
    settings.extra_convex_radius = in.extra_convex_radius;
    settings.back_face_mode_triangles = if (in.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.back_face_mode_convex = if (in.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.use_shrunken_shape_and_convex_radius = in.use_shrunken_shape != 0;
    settings.return_deepest_point = in.return_deepest_point != 0;
    settings.collect_faces_mode = if (in.collect_faces != 0) .collect_faces else .no_faces;
    settings.active_edge_mode = if (in.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.active_edge_movement_direction = vec3(in.active_edge_movement_direction);
    const cast_shape = if (in.mesh_cast != 0) mesh else convex.get().?;
    const target = if (in.mesh_cast != 0) convex.get().? else mesh;
    const shape_cast = ShapeCast.init(cast_shape, vec3(in.scale_cast), mat44(in.start), vec3(in.direction));
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(in.body_id), .{});
    collector.base.setContext(&context);
    if (in.early_out < collector.base.getEarlyOutFraction())
        collector.base.updateEarlyOutFraction(in.early_out);
    CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &settings, target, vec3(in.scale_target), &.{}, mat44(in.transform_target), makeCreator(in.creator1), makeCreator(in.creator2), &collector.base);
    try collector.checkError();
    s.u(marker_cast);
    s.box(shape_cast.shape_world_bounds);
    s.u(@intCast(collector.hits.items.len));
    for (collector.hits.items) |*r| {
        s.f(r.fraction);
        s.b(r.is_back_face_hit);
        writeCollideHit(s, &r.base);
    }
    s.f(collector.base.getEarlyOutFraction());
}

/// jolt_mesh_triangles
fn zoltTriangles(allocator: Allocator, materials: *const Materials, shape: *const Shape, in: *const TrianglesInput, s: *Stream) !void {
    var context: Shape.GetTrianglesContext = .{};
    shape.getTrianglesStart(&context, .init(vec3(in.box[0..3].*), vec3(in.box[3..6].*)), vec3(in.position), quat(in.rotation), vec3(in.scale));
    const max: u32 = @intCast(in.max_triangles_requested);
    const vertices = try allocator.alloc(Float3, 3 * max);
    defer allocator.free(vertices);
    const out_materials = try allocator.alloc(*const PhysicsMaterial, max);
    defer allocator.free(out_materials);
    s.u(marker_triangles);
    for (0..1000) |_| {
        const count = shape.getTrianglesNext(&context, max, vertices, if (in.materials != 0) out_materials else null);
        s.u(count);
        for (vertices[0 .. 3 * count]) |v| s.f3(v);
        if (in.materials != 0)
            for (out_materials[0..count]) |mat| materials.write(s, mat);
        if (count == 0)
            break;
    }
}

/// jolt_mesh_soft_body
fn zoltSoftBody(shape: *const Shape, transform: [16]f32, scale: P, positions_in: []const f32, inv_masses: []const f32, planes_in: []const f32, penetrations_in: []const f32, indices_in: []const c_int, colliding_shape_index: c_int, s: *Stream) void {
    const n = inv_masses.len;
    var positions: [64]Vec3 = undefined;
    var planes: [64]Plane = undefined;
    var penetrations: [64]f32 = undefined;
    var indices: [64]i32 = undefined;
    for (0..n) |i| {
        positions[i] = Vec3.init(positions_in[3 * i], positions_in[3 * i + 1], positions_in[3 * i + 2]);
        planes[i] = Plane.fromVec4(vec4(planes_in[4 * i ..][0..4].*));
        penetrations[i] = penetrations_in[i];
        indices[i] = indices_in[i];
    }
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    shape.collideSoftBodyVertices(mat44(transform), vec3(scale), &vertices, @intCast(n), colliding_shape_index);
    s.u(marker_soft_body);
    for (0..n) |i| {
        s.plane(planes[i]);
        s.f(penetrations[i]);
        s.i(indices[i]);
    }
}

/// jolt_mesh_binary_state
fn zoltBinaryState(allocator: Allocator, materials: *const Materials, shape: *const Shape, ray: *const RayInput, s: *Stream) !void {
    s.u(marker_binary_state);

    // Binary state
    const state = try saveBinaryState(allocator, shape);
    defer allocator.free(state);
    s.bytes(state);

    // Material state
    var material_list: PhysicsMaterialList = .empty;
    defer {
        for (material_list.items) |*mat| mat.deinit();
        material_list.deinit(allocator);
    }
    try material_list.append(allocator, .init(PhysicsMaterial.default));
    try shape.saveMaterialState(allocator, &material_list);
    s.u(@intCast(material_list.items.len));
    for (material_list.items) |mat| materials.write(s, mat.get().?);

    // Restore
    {
        var reader: std.Io.Reader = .fixed(state);
        var in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        s.b(result.isValid());
        if (result.isValid()) {
            const restored = result.getPtr().?;
            try restored.restoreMaterialState(material_list.items);
            const restored_state = try saveBinaryState(allocator, restored);
            defer allocator.free(restored_state);
            s.bytes(restored_state);
            var hit: RayCastResult = .{};
            const had_hit = restored.castRay(.init(vec3(ray.origin), vec3(ray.direction)), .{}, &hit);
            s.b(had_hit);
            s.f(hit.fraction);
            s.u(hit.sub_shape_id2.getValue());
            if (hit.fraction < 1.0)
                materials.write(s, restored.getMaterial(hit.sub_shape_id2));
        }
    }

    // Truncated binary state
    {
        var reader: std.Io.Reader = .fixed(state[0 .. state.len - 1]);
        var in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        s.b(result.isValid());
        if (result.hasError())
            s.str(result.getError());
    }

    // Save with children
    const children = try saveWithChildren(allocator, shape);
    defer allocator.free(children);
    s.bytes(children);
    {
        var reader: std.Io.Reader = .fixed(children);
        var in = StreamInWrapper.init(&reader);
        var id_to_shape: Shape.IDToShapeMap = .empty;
        defer {
            for (id_to_shape.items) |*x| x.deinit();
            id_to_shape.deinit(allocator);
        }
        var id_to_material: Shape.IDToMaterialMap = .empty;
        defer {
            for (id_to_material.items) |*x| x.deinit();
            id_to_material.deinit(allocator);
        }
        var result = try Shape.restoreWithChildren(allocator, in.streamIn(), &id_to_shape, &id_to_material);
        defer result.deinit();
        s.b(result.isValid());
        if (result.isValid()) {
            const restored = result.getPtr().?;
            const restored_children = try saveWithChildren(allocator, restored);
            defer allocator.free(restored_children);
            s.bytes(restored_children);
            const list = restored.cast(MeshShape).getMaterialList();
            s.u(@intCast(list.len));
            for (list) |mat| materials.write(s, mat.get().?);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Comparison of the streams

const Comparison = struct {
    name: []const u8,
    calls: usize = 0,
    mismatches: usize = 0,

    fn check(self: *Comparison, what: []const u8, mesh_index: usize, zolt_values: []const u32, jolt_values: []const u32) void {
        self.calls += 1;
        if (std.mem.eql(u32, zolt_values, jolt_values))
            return;
        if (self.mismatches < 5) {
            // Find the first difference and the last marker before it
            var first: usize = 0;
            while (first < zolt_values.len and first < jolt_values.len and zolt_values[first] == jolt_values[first]) first += 1;
            var marker: u32 = 0;
            for (zolt_values[0..first]) |v|
                if (v >= 0xB0000001 and v <= 0xB000000E) {
                    marker = v;
                };
            std.debug.print("{s}: {s} mismatch for mesh {d} at value {d} (after marker 0x{x}), lengths zolt {d} jolt {d}\n", .{ self.name, what, mesh_index, first, marker, zolt_values.len, jolt_values.len });
            const from = first -| 4;
            std.debug.print("  zolt: {any}\n  jolt: {any}\n", .{ zolt_values[from..@min(zolt_values.len, first + 8)], jolt_values[from..@min(jolt_values.len, first + 8)] });
        }
        self.mismatches += 1;
    }

    fn finish(self: *const Comparison) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} mismatches in {d} calls\n", .{ self.name, self.mismatches, self.calls });
            return error.ParityMismatch;
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Input generation

const Rng = fw.Rng;

/// A mesh description with its arrays
const MeshDesc = struct {
    allocator: Allocator,
    vertices: std.ArrayList(Float3) = .empty,
    triangles: std.ArrayList(IndexedTriangle) = .empty,
    triangle_list: std.ArrayList(Triangle) = .empty,
    input: MeshInput = undefined,
    /// A closed mesh (CollidePoint is meaningful)
    closed: bool = false,

    fn deinit(self: *MeshDesc) void {
        self.vertices.deinit(self.allocator);
        self.triangles.deinit(self.allocator);
        self.triangle_list.deinit(self.allocator);
    }

    fn addVertex(self: *MeshDesc, v: Vec3) !u32 {
        var f: Float3 = undefined;
        v.storeFloat3(&f);
        try self.vertices.append(self.allocator, f);
        return @intCast(self.vertices.items.len - 1);
    }

    fn addTriangle(self: *MeshDesc, a: u32, b: u32, c: u32) !void {
        try self.triangles.append(self.allocator, .init(a, b, c, .{}));
    }

    fn finalize(self: *MeshDesc) void {
        const dummy_vertex = &[_]Float3{.init(0, 0, 0)};
        const dummy_triangle = &[_]IndexedTriangle{.init(0, 0, 0, .{})};
        const dummy_list = &[_]Triangle{.fromFloat3(.init(0, 0, 0), .init(0, 0, 0), .init(0, 0, 0), .{})};
        self.input.vertices = if (self.vertices.items.len > 0) self.vertices.items.ptr else dummy_vertex;
        self.input.num_vertices = @intCast(self.vertices.items.len);
        self.input.triangles = if (self.triangles.items.len > 0) self.triangles.items.ptr else dummy_triangle;
        self.input.num_triangles = @intCast(self.triangles.items.len);
        self.input.triangle_list = if (self.triangle_list.items.len > 0) self.triangle_list.items.ptr else dummy_list;
        self.input.num_triangle_list = @intCast(self.triangle_list.items.len);
    }
};

const Gen = struct {
    rng: Rng = .{},

    fn index(self: *Gen, n: u32) u32 {
        return self.rng.next() % n;
    }

    fn chance(self: *Gen, percent: u32) bool {
        return self.index(100) < percent;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.float(min, max), self.float(min, max), self.float(min, max));
    }

    fn direction(self: *Gen) Vec3 {
        while (true) {
            const v = self.vec(-1, 1);
            const len_sq = v.lengthSq();
            if (len_sq > 0.01 and len_sq <= 1.0)
                return v.divScalar(@sqrt(len_sq));
        }
    }

    fn rotation(self: *Gen) Quat {
        return switch (self.index(4)) {
            0 => Quat.identity(),
            1 => Quat.rotation(([_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() })[self.index(3)], @as(f32, @floatFromInt(self.index(4))) * 0.5 * math.pi),
            else => Quat.rotation(self.direction(), self.float(-math.pi, math.pi)),
        };
    }

    fn transform(self: *Gen, extent: f32) Mat44 {
        return Mat44.rotationTranslation(self.rotation(), self.vec(-extent, extent));
    }

    /// A random scale for the mesh: uniform, non uniform, with negative components
    fn meshScale(self: *Gen) Vec3 {
        return switch (self.index(6)) {
            0, 1 => Vec3.one(),
            2 => Vec3.replicate(self.float(0.5, 2.0)),
            3 => Vec3.init(self.float(0.5, 2.0), self.float(0.5, 2.0), self.float(0.5, 2.0)),
            4 => Vec3.init(-1, 1, 1),
            else => Vec3.init(if (self.chance(50)) -1 else 1, if (self.chance(50)) -1 else 1, if (self.chance(50)) -1 else 1).mul(Vec3.init(self.float(0.5, 2.0), self.float(0.5, 2.0), self.float(0.5, 2.0))),
        };
    }

    fn creator(self: *Gen) [2]u32 {
        return switch (self.index(3)) {
            0 => .{ 0, 0 },
            1 => .{ self.index(8), 3 },
            else => .{ self.index(1 << 6), 6 },
        };
    }

    fn convex(self: *Gen) ConvexDesc {
        if (self.chance(50))
            return .{ .kind = 0, .radius = self.float(0.05, 1.5) };
        const half_extent = self.vec(0.05, 1.5);
        const max_radius = half_extent.reduceMin();
        return .{ .kind = 1, .half_extent = arr3(half_extent), .convex_radius = if (self.chance(30)) 0.0 else self.float(0.0, max_radius) };
    }

    /// A scale that is valid for the convex shape (uniform with signs for spheres, anything for boxes)
    fn convexScale(self: *Gen, desc: ConvexDesc) Vec3 {
        if (self.chance(40))
            return Vec3.one();
        const signs = Vec3.init(if (self.chance(30)) -1 else 1, if (self.chance(30)) -1 else 1, if (self.chance(30)) -1 else 1);
        if (desc.kind == 0)
            return signs.mul(Vec3.replicate(self.float(0.5, 2.0)));
        return signs.mul(self.vec(0.5, 2.0));
    }
};

/// A grid of nx * nz cells in the XZ plane (CCW seen from above), heights by `style`
fn makeGrid(gen: *Gen, mesh: *MeshDesc, nx: u32, nz: u32, spacing: f32, offset: Vec3, style: u32) !void {
    const base: u32 = @intCast(mesh.vertices.items.len);
    for (0..nz + 1) |z|
        for (0..nx + 1) |x| {
            const fx: f32 = @floatFromInt(x);
            const fz: f32 = @floatFromInt(z);
            const h: f32 = switch (style) {
                0 => 0.0, // Flat: all inner edges are coplanar
                1 => @floor(gen.float(0, 3)) * spacing, // Terraced: steps
                2 => gen.float(-0.5, 0.5) * spacing, // Noise
                else => @sin(fx * 0.7) * @cos(fz * 0.5) * spacing, // Smooth hills
            };
            _ = try mesh.addVertex(offset.add(Vec3.init(fx * spacing, h, fz * spacing)));
        };
    for (0..nz) |z|
        for (0..nx) |x| {
            const v: u32 = base + @as(u32, @intCast(z * (nx + 1) + x));
            const row = nx + 1;
            if (gen.chance(50)) {
                try mesh.addTriangle(v, v + row, v + 1);
                try mesh.addTriangle(v + 1, v + row, v + row + 1);
            } else {
                try mesh.addTriangle(v, v + row, v + row + 1);
                try mesh.addTriangle(v, v + row + 1, v + 1);
            }
        };
}

/// A closed UV sphere (CCW seen from the outside)
fn makeSphere(gen: *Gen, mesh: *MeshDesc, rings: u32, segments: u32, radius: f32, center: Vec3, jitter: f32) !void {
    const top = try mesh.addVertex(center.add(Vec3.init(0, radius, 0)));
    const first: u32 = @intCast(mesh.vertices.items.len);
    for (1..rings) |r| {
        const theta = math.pi * @as(f32, @floatFromInt(r)) / @as(f32, @floatFromInt(rings));
        for (0..segments) |seg| {
            const phi = 2.0 * math.pi * @as(f32, @floatFromInt(seg)) / @as(f32, @floatFromInt(segments));
            const rr = radius * (1.0 + gen.float(-jitter, jitter));
            _ = try mesh.addVertex(center.add(Vec3.init(rr * @sin(theta) * @cos(phi), rr * @cos(theta), rr * @sin(theta) * @sin(phi))));
        }
    }
    const bottom = try mesh.addVertex(center.add(Vec3.init(0, -radius, 0)));
    const ring_vertex = struct {
        fn f(first_vertex: u32, segs: u32, r: u32, seg: u32) u32 {
            return first_vertex + r * segs + seg % segs;
        }
    }.f;
    for (0..segments) |seg_usize| {
        const seg: u32 = @intCast(seg_usize);
        // Top cap and bottom cap
        try mesh.addTriangle(top, ring_vertex(first, segments, 0, seg + 1), ring_vertex(first, segments, 0, seg));
        try mesh.addTriangle(bottom, ring_vertex(first, segments, rings - 2, seg), ring_vertex(first, segments, rings - 2, seg + 1));
        for (0..rings - 2) |r_usize| {
            const r: u32 = @intCast(r_usize);
            const a = ring_vertex(first, segments, r, seg);
            const b = ring_vertex(first, segments, r, seg + 1);
            const c = ring_vertex(first, segments, r + 1, seg);
            const d = ring_vertex(first, segments, r + 1, seg + 1);
            try mesh.addTriangle(a, b, d);
            try mesh.addTriangle(a, d, c);
        }
    }
}

/// A closed box made of 12 triangles
fn makeBox(mesh: *MeshDesc, center: Vec3, half_extent: Vec3) !void {
    const base: u32 = @intCast(mesh.vertices.items.len);
    for (0..8) |i| {
        const sign = Vec3.init(if (i & 1 != 0) 1 else -1, if (i & 2 != 0) 1 else -1, if (i & 4 != 0) 1 else -1);
        _ = try mesh.addVertex(center.add(sign.mul(half_extent)));
    }
    const faces = [_][4]u32{ .{ 0, 4, 6, 2 }, .{ 1, 3, 7, 5 }, .{ 0, 1, 5, 4 }, .{ 2, 6, 7, 3 }, .{ 0, 2, 3, 1 }, .{ 4, 5, 7, 6 } };
    for (faces) |q| {
        try mesh.addTriangle(base + q[0], base + q[1], base + q[2]);
        try mesh.addTriangle(base + q[0], base + q[2], base + q[3]);
    }
}

/// A soup of random triangles with vertices from a shared pool
fn makeSoup(gen: *Gen, mesh: *MeshDesc, num_triangles: u32, extent: f32) !void {
    const pool = 3 + gen.index(num_triangles * 2 + 1);
    const base: u32 = @intCast(mesh.vertices.items.len);
    for (0..pool) |_| _ = try mesh.addVertex(gen.vec(-extent, extent));
    for (0..num_triangles) |_| {
        const a = gen.index(pool);
        var b = gen.index(pool);
        var c = gen.index(pool);
        if (b == a) b = (a + 1) % pool;
        if (c == a or c == b) c = (@max(a, b) + 1) % pool;
        if (c == a or c == b) c = (c + 1) % pool;
        try mesh.addTriangle(base + a, base + b, base + c);
    }
}

/// Number of hand picked meshes (see makeSpecialMesh), the others are random
const num_special_meshes = 15;

/// Hand picked mesh number `n`: every error of the constructor and edge cases of the active edges
fn makeSpecialMesh(allocator: Allocator, n: usize) !MeshDesc {
    var mesh: MeshDesc = .{ .allocator = allocator };
    errdefer mesh.deinit();

    var mode: u32 = 1;
    var num_materials: u32 = 0;
    var max_triangles_per_leaf: u32 = 8;
    var threshold: f32 = 0.996195;
    const square = [_]Vec3{ Vec3.init(0, 0, 0), Vec3.init(0, 0, 1), Vec3.init(1, 0, 1), Vec3.init(1, 0, 0) };
    switch (n) {
        0 => mode = 0, // No triangles
        1 => {
            // A collinear triangle (not removed: mode 0)
            mode = 0;
            for (square) |v| _ = try mesh.addVertex(v);
            _ = try mesh.addVertex(Vec3.init(2, 0, 0));
            try mesh.addTriangle(0, 1, 2);
            try mesh.addTriangle(0, 3, 4);
            try mesh.addTriangle(0, 2, 3);
        },
        2, 14 => {
            // A triangle that is only degenerate in quantized space (not removed with mode 0, removed by Sanitize)
            mode = if (n == 2) 0 else 1;
            _ = try mesh.addVertex(Vec3.init(0, 0, 0));
            _ = try mesh.addVertex(Vec3.init(0.01, 0, 0));
            _ = try mesh.addVertex(Vec3.init(0, 0.01, 0));
            _ = try mesh.addVertex(Vec3.init(100000, 100000, 100000));
            _ = try mesh.addVertex(Vec3.init(0, 100000, 0));
            try mesh.addTriangle(0, 4, 3);
            try mesh.addTriangle(0, 1, 2);
            if (n == 14) try mesh.addTriangle(0, 3, 4);
        },
        3, 4, 5, 6, 7 => {
            for (square) |v| _ = try mesh.addVertex(v);
            try mesh.addTriangle(0, 1, 2);
            try mesh.addTriangle(0, 2, 3);
            switch (n) {
                3 => num_materials = 33, // Too many materials
                4 => {
                    // Material index beyond the material list
                    num_materials = 2;
                    mesh.triangles.items[1].material_index = 2;
                },
                5 => mesh.triangles.items[0].material_index = 1, // No materials but a material index
                6 => max_triangles_per_leaf = 0,
                else => max_triangles_per_leaf = 9,
            }
        },
        8 => {
            // A single triangle
            for (square[0..3]) |v| _ = try mesh.addVertex(v);
            try mesh.addTriangle(0, 1, 2);
        },
        9 => {
            // Back to back triangles (opposite normals: active), 3 and 4 triangles that share an edge (active)
            for (square) |v| _ = try mesh.addVertex(v);
            _ = try mesh.addVertex(Vec3.init(0, 1, 0.5));
            _ = try mesh.addVertex(Vec3.init(-1, 0.5, 0.5));
            _ = try mesh.addVertex(Vec3.init(5, 0, 0));
            _ = try mesh.addVertex(Vec3.init(5, 0, 1));
            _ = try mesh.addVertex(Vec3.init(6, 0, 1));
            try mesh.addTriangle(6, 7, 8);
            try mesh.addTriangle(6, 8, 7);
            try mesh.addTriangle(0, 1, 2);
            try mesh.addTriangle(1, 0, 3);
            try mesh.addTriangle(0, 1, 4);
            try mesh.addTriangle(1, 0, 5);
        },
        10 => {
            // A coplanar quad, a convex and a concave fold
            const v = [_]Vec3{ Vec3.init(0, 0, 0), Vec3.init(0, 0, 1), Vec3.init(1, 0, 1), Vec3.init(1, 0, 0), Vec3.init(0, -1, 0), Vec3.init(0, -1, 1), Vec3.init(1, 1, 0), Vec3.init(1, 1, 1) };
            for (v) |x| _ = try mesh.addVertex(x);
            try mesh.addTriangle(0, 1, 2);
            try mesh.addTriangle(0, 2, 3);
            try mesh.addTriangle(0, 4, 5); // Convex with 0-1 (wall below, facing away)
            try mesh.addTriangle(0, 5, 1);
            try mesh.addTriangle(3, 2, 7); // Concave with 2-3 (wall above, facing the floor)
            try mesh.addTriangle(3, 7, 6);
            threshold = 0.5;
        },
        11 => {
            // The triangle list constructor: Indexify welds vertices that are close
            mode = 2;
            const a = Vec3.init(0, 0, 0);
            const b = Vec3.init(0, 0, 1);
            const c = Vec3.init(1, 0, 1);
            const d = Vec3.init(1, 0, 0);
            const e = Vec3.init(1.0e-5, 0, 0);
            try mesh.triangle_list.append(allocator, .init(a, b, c, .{ .user_data = 1 }));
            try mesh.triangle_list.append(allocator, .init(e, c, d, .{ .user_data = 2 }));
            try mesh.triangle_list.append(allocator, .init(a, c, d, .{ .user_data = 2 })); // A duplicate after welding
            try mesh.triangle_list.append(allocator, .init(a, a, d, .{ .user_data = 3 })); // Degenerate

            // The vertices and triangles used to aim the queries (not passed to the constructor)
            for ([_]Vec3{ a, b, c, d }) |v| _ = try mesh.addVertex(v);
            try mesh.addTriangle(0, 1, 2);
            try mesh.addTriangle(0, 2, 3);
        },
        12 => {
            // A vertex far away that no triangle uses (not part of the quantization bounds), every edge active
            for (square) |v| _ = try mesh.addVertex(v);
            _ = try mesh.addVertex(Vec3.init(1.0e6, 1.0e6, 1.0e6));
            try mesh.addTriangle(0, 1, 2);
            try mesh.addTriangle(0, 2, 3);
            threshold = -0.5;
        },
        13 => {
            // 32 materials, all used, user data
            num_materials = 32;
            for (0..9) |i| _ = try mesh.addVertex(Vec3.init(@floatFromInt(i % 3), @floatFromInt(i / 3), @as(f32, @floatFromInt(i % 2)) * 0.25));
            for (0..2) |y|
                for (0..2) |x| {
                    const v: u32 = @intCast(y * 3 + x);
                    try mesh.addTriangle(v, v + 1, v + 4);
                    try mesh.addTriangle(v, v + 4, v + 3);
                };
            for (mesh.triangles.items, 0..) |*t, i| {
                t.material_index = @intCast(31 - i);
                t.user_data = @intCast(1000 + i);
            }
        },
        else => unreachable,
    }

    mesh.input = .{
        .vertices = undefined,
        .triangles = undefined,
        .triangle_list = undefined,
        .user_data = n,
        .num_vertices = 0,
        .num_triangles = 0,
        .num_triangle_list = 0,
        .mode = mode,
        .num_materials = num_materials,
        .max_triangles_per_leaf = max_triangles_per_leaf,
        .active_edge_cos_threshold_angle = threshold,
        .per_triangle_user_data = 1,
        .build_quality = @intCast(n % 2),
    };
    mesh.finalize();
    return mesh;
}

/// Generate mesh number `n`: the geometry, then materials, user data, injected problems and settings
fn makeMesh(allocator: Allocator, gen: *Gen, n: usize) !MeshDesc {
    if (n < num_special_meshes)
        return makeSpecialMesh(allocator, n);

    var mesh: MeshDesc = .{ .allocator = allocator };
    errdefer mesh.deinit();

    // Geometry
    const kind = n % 8;
    switch (kind) {
        0, 1 => try makeGrid(gen, &mesh, 1 + gen.index(12), 1 + gen.index(12), gen.float(0.2, 2.0), if (gen.chance(20)) gen.vec(-1000, 1000) else gen.vec(-5, 5), gen.index(4)),
        2 => try makeSoup(gen, &mesh, 1 + gen.index(200), gen.float(0.5, 10)),
        3 => {
            try makeSphere(gen, &mesh, 3 + gen.index(10), 3 + gen.index(14), gen.float(0.5, 4), gen.vec(-2, 2), if (gen.chance(50)) 0.0 else 0.2);
            mesh.closed = true;
        },
        4 => {
            try makeBox(&mesh, gen.vec(-2, 2), gen.vec(0.2, 3));
            if (gen.chance(50)) try makeBox(&mesh, gen.vec(5, 8), gen.vec(0.2, 1)); // Two non intersecting manifolds
            mesh.closed = true;
        },
        5 => {
            // A grid with a box on it (convex and concave edges)
            try makeGrid(gen, &mesh, 2 + gen.index(6), 2 + gen.index(6), 1.0, Vec3.zero(), 0);
            try makeBox(&mesh, Vec3.init(1, 0.5, 1), Vec3.replicate(0.5));
        },
        6 => {
            // Several meshes in one
            try makeGrid(gen, &mesh, 1 + gen.index(5), 1 + gen.index(5), gen.float(0.2, 2.0), gen.vec(-5, 5), gen.index(4));
            try makeSoup(gen, &mesh, 1 + gen.index(30), 3);
            try makeSphere(gen, &mesh, 3 + gen.index(4), 3 + gen.index(5), 1.0, gen.vec(-3, 3), 0.0);
        },
        else => {
            if (n % 32 == 7) {
                // A big grid: a deep tree with many triangle blocks
                try makeGrid(gen, &mesh, 50 + gen.index(30), 50 + gen.index(30), 0.5, Vec3.zero(), 2 + gen.index(2));
            } else try makeSoup(gen, &mesh, 1 + gen.index(20), gen.float(0.1, 2));
        },
    }

    // Materials and user data
    var num_materials: u32 = if (gen.chance(40)) 0 else 1 + gen.index(32);
    for (mesh.triangles.items) |*t| {
        t.material_index = if (num_materials > 0) gen.index(num_materials) else 0;
        t.user_data = if (gen.chance(20)) 0 else gen.rng.next();
    }

    // Injected problems: degenerate triangles, duplicates (rotated, with the same or another material / user data)
    if (gen.chance(30) and mesh.triangles.items.len > 0) {
        for (0..1 + gen.index(5)) |_| {
            const t = mesh.triangles.items[gen.index(@intCast(mesh.triangles.items.len))];
            switch (gen.index(4)) {
                0 => try mesh.triangles.append(allocator, .init(t.idx[0], t.idx[0], t.idx[1], .{ .material_index = t.material_index })), // Repeated index
                1 => {
                    // Collinear
                    const a = Vec3.fromFloat3(mesh.vertices.items[t.idx[0]]);
                    const b = Vec3.fromFloat3(mesh.vertices.items[t.idx[1]]);
                    const c = try mesh.addVertex(a.add(b.sub(a).mulScalar(2.0)));
                    try mesh.triangles.append(allocator, .init(t.idx[0], t.idx[1], c, .{ .material_index = t.material_index }));
                },
                2 => try mesh.triangles.append(allocator, .init(t.idx[1], t.idx[2], t.idx[0], .{ .material_index = t.material_index, .user_data = t.user_data })), // Duplicate
                else => try mesh.triangles.append(allocator, .init(t.idx[2], t.idx[0], t.idx[1], .{ .material_index = t.material_index, .user_data = t.user_data +% 1 })), // Same vertices, other user data
            }
        }
    }

    // Errors
    var mode: u32 = gen.index(3);
    if (gen.chance(8)) {
        switch (gen.index(5)) {
            0 => num_materials = 33 + gen.index(8), // Too many materials
            1 => if (mesh.triangles.items.len > 0) {
                mesh.triangles.items[0].material_index = num_materials + gen.index(3); // Material beyond the list (or no materials)
            },
            2 => mesh.triangles.clearRetainingCapacity(), // No triangles
            3 => mode = 0, // Degenerate triangles are not removed
            else => {},
        }
    }

    // The triangle list of the triangle list constructor, with vertices that Indexify welds
    if (mode == 2) {
        for (mesh.triangles.items) |t| {
            var v: [3]Float3 = undefined;
            for (0..3) |j| {
                v[j] = mesh.vertices.items[t.idx[j]];
                if (gen.chance(10)) v[j].x += 1.0e-5; // Within the weld distance
            }
            try mesh.triangle_list.append(allocator, .fromFloat3(v[0], v[1], v[2], .{ .material_index = t.material_index, .user_data = t.user_data }));
        }
    }

    const thresholds = [_]f32{ 0.996195, 0.996195, -1.0, 0.0, 0.5, 0.9999, 1.0 };
    mesh.input = .{
        .vertices = undefined,
        .triangles = undefined,
        .triangle_list = undefined,
        .user_data = if (gen.chance(50)) 0 else (@as(u64, gen.rng.next()) << 32) | gen.rng.next(),
        .num_vertices = 0,
        .num_triangles = 0,
        .num_triangle_list = 0,
        .mode = mode,
        .num_materials = num_materials,
        .max_triangles_per_leaf = if (gen.chance(5)) ([_]u32{ 0, 9, 16 })[gen.index(3)] else 1 + gen.index(8),
        .active_edge_cos_threshold_angle = if (gen.chance(20)) @cos(gen.float(0, 0.5 * math.pi)) else thresholds[gen.index(thresholds.len)],
        .per_triangle_user_data = @intFromBool(gen.chance(50)),
        .build_quality = gen.index(2),
    };
    mesh.finalize();
    return mesh;
}

/// Collects the triangle blocks of a tree (block ID and number of triangles) to build every valid sub shape ID
const BlockCollector = struct {
    allocator: Allocator,
    ids: std.ArrayList(u32) = .empty,
    block_id_bits: u32,

    pub fn shouldAbort(self: *const BlockCollector) bool {
        _ = self;
        return false;
    }

    pub fn shouldVisitNode(self: *const BlockCollector, stack_top: i32) bool {
        _ = .{ self, stack_top };
        return true;
    }

    pub fn visitNodes(self: *BlockCollector, min_x: Vec4, min_y: Vec4, min_z: Vec4, max_x: Vec4, max_y: Vec4, max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
        _ = .{ self, min_x, min_y, min_z, max_x, max_y, max_z, properties, stack_top };
        return 4; // Padding children are skipped by the walk
    }

    pub fn visitTriangles(self: *BlockCollector, context: *const u8, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
        _ = .{ context, triangles };
        for (0..num_triangles) |t| {
            const id = SubShapeIDCreator.pushID(.{}, triangle_block_id, self.block_id_bits).pushID(@intCast(t), MeshShape.num_triangle_bits).getID();
            self.ids.append(self.allocator, id.getValue()) catch @panic("out of memory");
        }
    }
};

fn collectSubShapeIDs(allocator: Allocator, mesh: *const MeshShape) !std.ArrayList(u32) {
    const items = mesh.tree.vector.items;
    const header: *const NodeCodec.Header = @ptrCast(@alignCast(items.ptr));
    var collector: BlockCollector = .{ .allocator = allocator, .block_id_bits = NodeCodec.DecodingContext.triangleBlockIDBits(header) };
    var ctx = NodeCodec.DecodingContext.init(header);
    const dummy: u8 = 0;
    ctx.walkTree(items.ptr, &dummy, &collector);
    return collector.ids;
}

// ---------------------------------------------------------------------------------------------------------------------
// The test

/// Run a C++ function that writes a stream (`call(out, capacity) u32`)
fn joltStream(buffer: []u32, call: anytype) []const u32 {
    const size = call.run(buffer.ptr, @intCast(buffer.len));
    std.debug.assert(size <= buffer.len); // The capacity is big enough for every query of the test
    return buffer[0..size];
}

test "MeshShape parity" {
    const allocator = std.testing.allocator;

    var materials: Materials = .{};
    try materials.init(allocator);
    defer materials.deinit();

    const buffer = try allocator.alloc(u32, stream_capacity);
    defer allocator.free(buffer);

    var create_cmp: Comparison = .{ .name = "MeshShape create (settings, Sanitize, errors, properties, tree)" };
    var properties_cmp: Comparison = .{ .name = "MeshShape properties (world bounds, scales, submerged volume)" };
    var sub_shapes_cmp: Comparison = .{ .name = "MeshShape sub shape queries (material, user data, normal, face, leaf, transformed shape)" };
    var ray_cmp: Comparison = .{ .name = "MeshShape CastRay" };
    var point_cmp: Comparison = .{ .name = "MeshShape CollidePoint" };
    var collide_cmp: Comparison = .{ .name = "MeshShape collide" };
    var cast_cmp: Comparison = .{ .name = "MeshShape cast" };
    var triangles_cmp: Comparison = .{ .name = "MeshShape GetTrianglesStart / Next" };
    var soft_body_cmp: Comparison = .{ .name = "MeshShape CollideSoftBodyVertices" };
    var binary_cmp: Comparison = .{ .name = "MeshShape binary state" };

    var gen: Gen = .{};
    const num_meshes = num_special_meshes + 96;
    var num_valid: usize = 0;
    var num_errors: usize = 0;
    for (0..num_meshes) |mesh_index| {
        var desc = try makeMesh(allocator, &gen, mesh_index);
        defer desc.deinit();

        // Create on both sides
        var zs: Stream = .{ .allocator = allocator };
        defer zs.deinit();
        var zolt_shape = try zoltCreate(allocator, &materials, &desc.input, &zs);
        defer if (zolt_shape) |*x| x.deinit();
        var jolt_size: u32 = 0;
        const handle = jolt.jolt_mesh_create(&desc.input, buffer.ptr, stream_capacity, &jolt_size);
        defer if (handle) |h| jolt.jolt_mesh_destroy(h);
        std.debug.assert(jolt_size <= stream_capacity);
        create_cmp.check("create", mesh_index, zs.values.items, buffer[0..jolt_size]);
        if ((zolt_shape == null) != (handle == null)) {
            create_cmp.mismatches += 1;
            continue;
        }
        if (handle == null) {
            num_errors += 1;
            continue;
        }
        num_valid += 1;
        const h = handle.?;
        const shape = zolt_shape.?.get().?;
        const mesh = shape.cast(MeshShape);
        const bounds = shape.getLocalBounds();
        const center = bounds.getCenter();
        const extent = bounds.getExtent().add(Vec3.replicate(0.1));
        const size = bounds.getSize().length() + 0.1;
        const vertices = desc.vertices.items;

        // Properties
        for (0..10) |i| {
            const surface = Plane.fromPointAndNormal(center, gen.direction());
            const input: PropertiesInput = .{
                .transform = arr16(gen.transform(10)),
                .translation = .{ gen.float(-1000, 1000), gen.float(-1000, 1000), @as(f64, gen.float(-1, 1)) * 1.0e6 },
                .scale = arr3(if (i == 0) Vec3.zero() else if (i == 1) Vec3.init(1, 0, 1) else gen.meshScale()),
                .surface_plane = arr3(surface.getNormal()) ++ [1]f32{surface.getConstant()},
                .submerged = @intFromBool(!std.debug.runtime_safety), // Jolt asserts: only without asserts
            };
            zs.values.clearRetainingCapacity();
            zoltProperties(shape, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const PropertiesInput,
                fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_properties(c.h, c.in, out, cap);
                }
            };
            properties_cmp.check("properties", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .in = &input }));
        }

        // Every sub shape ID
        var ids = try collectSubShapeIDs(allocator, mesh);
        defer ids.deinit(allocator);
        if (ids.items.len != shape.getStats().num_triangles) {
            std.debug.print("mesh {d}: {d} sub shape IDs for {d} triangles\n", .{ mesh_index, ids.items.len, shape.getStats().num_triangles });
            return error.TestUnexpectedResult;
        }
        {
            const chunk = 400;
            var start: usize = 0;
            while (start < ids.items.len) : (start += chunk) {
                const part = ids.items[start..@min(ids.items.len, start + chunk)];
                const input: SubShapeInput = .{
                    .transform = arr16(gen.transform(5)),
                    .scale = arr3(gen.meshScale()),
                    .direction = arr3(gen.direction()),
                    .position = arr3(gen.vec(-5, 5)),
                    .rotation = arr4(gen.rotation().getXYZW()),
                    .point = arr3(gen.vec(-5, 5)),
                };
                zs.values.clearRetainingCapacity();
                zoltSubShapes(&materials, shape, part, &input, &zs);
                const Call = struct {
                    h: *anyopaque,
                    ids: []const u32,
                    in: *const SubShapeInput,
                    fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                        return jolt.jolt_mesh_sub_shapes(c.h, c.ids.ptr, @intCast(c.ids.len), c.in, out, cap);
                    }
                };
                sub_shapes_cmp.check("sub shapes", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .ids = part, .in = &input }));
            }
        }

        // Rays: random, through vertices (straight down and diagonal), along edges, parallel to faces, from the inside,
        // from the center, zero length
        const num_rays: usize = if (ids.items.len > 2000) 100 else 250;
        for (0..num_rays) |i| {
            var origin: Vec3 = undefined;
            var direction: Vec3 = undefined;
            const kind = gen.index(8);
            switch (kind) {
                0, 1, 2 => {
                    origin = center.add(gen.vec(-1, 1).mul(extent).mulScalar(1.5));
                    direction = gen.direction().mulScalar(gen.float(0.1, 3) * size);
                },
                3 => {
                    // Through a vertex
                    const v = Vec3.fromFloat3(vertices[gen.index(@intCast(vertices.len))]);
                    direction = if (gen.chance(50)) Vec3.init(0, -2 * size, 0) else gen.direction().mulScalar(2 * size);
                    origin = v.sub(direction.mulScalar(0.5));
                },
                4 => {
                    // Along an edge or through the middle of an edge
                    const t = desc.triangles.items[gen.index(@intCast(desc.triangles.items.len))];
                    const a = Vec3.fromFloat3(vertices[t.idx[0]]);
                    const b = Vec3.fromFloat3(vertices[t.idx[1]]);
                    if (gen.chance(50)) {
                        direction = b.sub(a).mulScalar(3.0);
                        origin = a.sub(b.sub(a));
                    } else {
                        direction = gen.direction().mulScalar(size);
                        origin = a.add(b).mulScalar(0.5).sub(direction.mulScalar(0.5));
                    }
                },
                5 => {
                    // Parallel to a face, in its plane
                    const t = desc.triangles.items[gen.index(@intCast(desc.triangles.items.len))];
                    const a = Vec3.fromFloat3(vertices[t.idx[0]]);
                    const b = Vec3.fromFloat3(vertices[t.idx[1]]);
                    const c = Vec3.fromFloat3(vertices[t.idx[2]]);
                    const in_plane = b.sub(a).mulScalar(gen.float(-1, 1)).add(c.sub(a).mulScalar(gen.float(-1, 1)));
                    origin = a.add(in_plane);
                    direction = b.sub(a).add(c.sub(a).mulScalar(gen.float(-1, 1))).mulScalar(2.0);
                },
                6 => {
                    // From the inside / the center (closed meshes)
                    origin = if (gen.chance(50)) center else center.add(gen.vec(-0.5, 0.5).mul(extent));
                    direction = gen.direction().mulScalar(size);
                },
                else => {
                    origin = center.add(gen.vec(-1, 1).mul(extent));
                    direction = if (gen.chance(50)) Vec3.zero() else gen.direction().mulScalar(1.0e-3);
                },
            }
            const input: RayInput = .{
                .origin = arr3(origin),
                .direction = arr3(direction),
                .creator = gen.creator(),
                .fraction = if (gen.chance(80)) 1.0 + math.flt_epsilon else gen.float(0, 1),
                .back_face_mode = @intFromBool(gen.chance(50)),
                .collector = @intCast(i % 3),
                .early_out = if (gen.chance(70)) math.flt_max else gen.float(0, 1),
                .body_id = gen.index(1000),
                .reject_all = @intFromBool(gen.chance(3)),
            };
            zs.values.clearRetainingCapacity();
            try zoltCastRay(allocator, shape, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const RayInput,
                fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_cast_ray(c.h, c.in, out, cap);
                }
            };
            ray_cmp.check("ray", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .in = &input }));
        }

        // Collide point
        for (0..if (desc.closed) 120 else 30) |_| {
            const point = if (gen.chance(10)) Vec3.fromFloat3(vertices[gen.index(@intCast(vertices.len))]) else center.add(gen.vec(-1.2, 1.2).mul(extent));
            const creator = gen.creator();
            const body_id = gen.index(1000);
            const reject_all = gen.chance(3);
            zs.values.clearRetainingCapacity();
            try zoltCollidePoint(allocator, shape, arr3(point), creator, body_id, reject_all, &zs);
            const Call = struct {
                h: *anyopaque,
                point: P,
                creator: [2]u32,
                body_id: u32,
                reject_all: bool,
                fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_collide_point(c.h, &c.point, &c.creator, c.body_id, @intFromBool(c.reject_all), out, cap);
                }
            };
            point_cmp.check("point", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .point = arr3(point), .creator = creator, .body_id = body_id, .reject_all = reject_all }));
        }

        // Collide: a sphere or a box near a random triangle (touching, penetrating, separated), either order
        const num_collides: usize = if (ids.items.len > 2000) 40 else 100;
        for (0..num_collides) |_| {
            const convex = gen.convex();
            const scale_mesh = gen.meshScale();
            const transform_mesh = gen.transform(3);
            const t = desc.triangles.items[gen.index(@intCast(desc.triangles.items.len))];
            const a = Vec3.fromFloat3(vertices[t.idx[0]]);
            const b = Vec3.fromFloat3(vertices[t.idx[1]]);
            const c = Vec3.fromFloat3(vertices[t.idx[2]]);
            const w = gen.vec(0, 1);
            const on_triangle = a.mulScalar(w.getX()).add(b.mulScalar(w.getY())).add(c.mulScalar(w.getZ())).divScalar(w.getX() + w.getY() + w.getZ() + 1.0e-6);
            const normal = b.sub(a).cross(c.sub(a)).normalizedOr(Vec3.axisY());
            const radius = if (convex.kind == 0) convex.radius else vec3(convex.half_extent).length();
            const distance = switch (gen.index(4)) {
                0 => radius, // Touching (roughly)
                1 => gen.float(-radius, radius),
                2 => gen.float(radius, 2 * radius),
                else => gen.float(-2 * radius, 2 * radius),
            };
            const position_in_mesh = on_triangle.add(normal.mulScalar(distance * (if (gen.chance(20)) @as(f32, -1) else 1)));
            const position = transform_mesh.mulVec3(scale_mesh.mul(position_in_mesh));
            const transform_convex = Mat44.rotationTranslation(gen.rotation(), position);
            const internal_edge_removal = gen.chance(15);
            const input: CollideInput = .{
                .convex = convex,
                .mesh_first = @intFromBool(gen.chance(30)),
                .scale_convex = arr3(gen.convexScale(convex)),
                .scale_mesh = arr3(scale_mesh),
                .transform_convex = arr16(transform_convex),
                .transform_mesh = arr16(transform_mesh),
                .creator1 = gen.creator(),
                .creator2 = gen.creator(),
                .max_separation_distance = if (gen.chance(60)) 0.0 else gen.float(0, 1),
                .collision_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                .penetration_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                .back_face_mode = @intFromBool(gen.chance(40)),
                .active_edge_mode = @intFromBool(internal_edge_removal or gen.chance(30)),
                .collect_faces = @intFromBool(internal_edge_removal or gen.chance(40)),
                .active_edge_movement_direction = if (gen.chance(50)) .{ 0, 0, 0 } else arr3(gen.vec(-1, 1)),
                .early_out = if (gen.chance(80)) math.flt_max else gen.float(-0.5, 0.5),
                .body_id = gen.index(1000),
                .internal_edge_removal = @intFromBool(internal_edge_removal),
                .vertex_tolerance_sq = if (gen.chance(50)) 1.0e-8 else gen.float(0, 0.01),
            };
            zs.values.clearRetainingCapacity();
            try zoltCollide(allocator, shape, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const CollideInput,
                fn run(cc: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_collide(cc.h, cc.in, out, cap);
                }
            };
            collide_cmp.check("collide", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .in = &input }));
        }

        // Cast: a sphere or a box through the mesh, or the mesh against a sphere or a box. Not for the meshes with
        // coordinates around 1e5 (special meshes 2 and 14): long casts overflow in CastSphereVsTriangles::RayCylinder
        // and the NaN fraction violates Jolt's assert `fraction >= 0`.
        const num_casts: usize = if (size > 1.0e4) 0 else if (ids.items.len > 2000) 25 else 60;
        for (0..num_casts) |_| {
            const convex = gen.convex();
            const mesh_cast = gen.chance(25);
            const scale_mesh = gen.meshScale();
            const scale_convex = gen.convexScale(convex);
            const transform_mesh = gen.transform(3);
            const target_point = transform_mesh.mulVec3(scale_mesh.mul(Vec3.fromFloat3(vertices[gen.index(@intCast(vertices.len))]).add(gen.vec(-0.3, 0.3))));
            const dir = if (gen.chance(15)) Vec3.zero() else gen.direction().mulScalar(gen.float(0.5, 3) * size);
            const start_point = target_point.sub(dir.mulScalar(gen.float(0.2, 0.8)));
            const input: CastInput = .{
                .convex = convex,
                .mesh_cast = @intFromBool(mesh_cast),
                .scale_cast = arr3(if (mesh_cast) scale_mesh else scale_convex),
                .start = arr16(if (mesh_cast) Mat44.translation(target_point.sub(start_point).negate()).mul(transform_mesh) else Mat44.rotationTranslation(gen.rotation(), start_point)),
                .direction = arr3(dir),
                .scale_target = arr3(if (mesh_cast) scale_convex else scale_mesh),
                .transform_target = arr16(if (mesh_cast) Mat44.rotationTranslation(gen.rotation(), target_point) else transform_mesh),
                .creator1 = gen.creator(),
                .creator2 = gen.creator(),
                .collision_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                .penetration_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                .extra_convex_radius = if (gen.chance(70)) 0.0 else gen.float(0, 0.3),
                .back_face_mode_triangles = @intFromBool(gen.chance(40)),
                .back_face_mode_convex = @intFromBool(gen.chance(40)),
                .use_shrunken_shape = @intFromBool(gen.chance(40)),
                .return_deepest_point = @intFromBool(gen.chance(40)),
                .collect_faces = @intFromBool(gen.chance(40)),
                .active_edge_mode = @intFromBool(gen.chance(40)),
                .active_edge_movement_direction = if (gen.chance(50)) .{ 0, 0, 0 } else arr3(gen.vec(-1, 1)),
                .early_out = if (gen.chance(80)) 1.0 + math.flt_epsilon else gen.float(-0.1, 1),
                .body_id = gen.index(1000),
            };
            zs.values.clearRetainingCapacity();
            try zoltCast(allocator, shape, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const CastInput,
                fn run(cc: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_cast(cc.h, cc.in, out, cap);
                }
            };
            cast_cmp.check("cast", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .in = &input }));
        }

        // GetTrianglesStart / Next: everything, random boxes, with transforms and inside out scales
        for (0..6) |i| {
            const position = gen.vec(-5, 5);
            const rotation = gen.rotation();
            const scale = gen.meshScale();
            var box: [6]f32 = .{ -1.0e6, -1.0e6, -1.0e6, 1.0e6, 1.0e6, 1.0e6 };
            if (i > 0) {
                const world_center = Mat44.rotationTranslation(rotation, position).mulVec3(scale.mul(center.add(gen.vec(-1, 1).mul(extent))));
                const half = gen.vec(0.01, 1).mul(extent.mul(scale.abs()));
                box = arr3(world_center.sub(half)) ++ arr3(world_center.add(half));
            }
            const input: TrianglesInput = .{
                .box = box,
                .position = arr3(position),
                .rotation = arr4(rotation.getXYZW()),
                .scale = arr3(scale),
                .max_triangles_requested = @intCast(32 + gen.index(100)),
                .materials = @intFromBool(gen.chance(70)),
            };
            zs.values.clearRetainingCapacity();
            try zoltTriangles(allocator, &materials, shape, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const TrianglesInput,
                fn run(cc: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_triangles(cc.h, cc.in, out, cap);
                }
            };
            triangles_cmp.check("triangles", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .in = &input }));
        }

        // CollideSoftBodyVertices
        for (0..3) |_| {
            const n = 16;
            const transform = gen.transform(2);
            const scale = gen.meshScale();
            var positions: [3 * n]f32 = undefined;
            var inv_masses: [n]f32 = undefined;
            var planes: [4 * n]f32 = undefined;
            var penetrations: [n]f32 = undefined;
            var indices: [n]c_int = undefined;
            for (0..n) |i| {
                const local = if (gen.chance(30)) Vec3.fromFloat3(vertices[gen.index(@intCast(vertices.len))]).add(gen.vec(-0.2, 0.2)) else center.add(gen.vec(-1.3, 1.3).mul(extent));
                positions[3 * i ..][0..3].* = arr3(transform.mulVec3(scale.mul(local)));
                inv_masses[i] = if (gen.chance(15)) 0.0 else 1.0;
                planes[4 * i ..][0..4].* = .{ 0, 1, 0, 0 };
                penetrations[i] = if (gen.chance(20)) gen.float(-0.5, 0.5) else -math.flt_max;
                indices[i] = -1;
            }
            const colliding_shape_index: c_int = @intCast(gen.index(10));
            zs.values.clearRetainingCapacity();
            zoltSoftBody(shape, arr16(transform), arr3(scale), &positions, &inv_masses, &planes, &penetrations, &indices, colliding_shape_index, &zs);
            const Call = struct {
                h: *anyopaque,
                transform: [16]f32,
                scale: P,
                positions: *const [3 * n]f32,
                inv_masses: *const [n]f32,
                planes: *const [4 * n]f32,
                penetrations: *const [n]f32,
                indices: *const [n]c_int,
                index: c_int,
                fn run(cc: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_soft_body(cc.h, &cc.transform, &cc.scale, n, cc.positions, cc.inv_masses, cc.planes, cc.penetrations, cc.indices, cc.index, out, cap);
                }
            };
            soft_body_cmp.check("soft body", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .transform = arr16(transform), .scale = arr3(scale), .positions = &positions, .inv_masses = &inv_masses, .planes = &planes, .penetrations = &penetrations, .indices = &indices, .index = colliding_shape_index }));
        }

        // Binary state
        {
            const input: RayInput = .{
                .origin = arr3(center.add(Vec3.init(0, 2 * size, 0))),
                .direction = arr3(Vec3.init(0.01, -4 * size, 0.02)),
                .creator = .{ 0, 0 },
                .fraction = 0,
                .back_face_mode = 0,
                .collector = 0,
                .early_out = 0,
                .body_id = 0,
                .reject_all = 0,
            };
            zs.values.clearRetainingCapacity();
            try zoltBinaryState(allocator, &materials, shape, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const RayInput,
                fn run(cc: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.jolt_mesh_binary_state(cc.h, cc.in, out, cap);
                }
            };
            binary_cmp.check("binary state", mesh_index, zs.values.items, joltStream(buffer, Call{ .h = h, .in = &input }));
        }
    }

    // Enough meshes of both kinds were tested
    if (num_valid < 70 or num_errors < 10) {
        std.debug.print("MeshShape parity: {d} valid meshes, {d} errors\n", .{ num_valid, num_errors });
        return error.TestUnexpectedResult;
    }

    var failed = false;
    for ([_]*const Comparison{ &create_cmp, &properties_cmp, &sub_shapes_cmp, &ray_cmp, &point_cmp, &collide_cmp, &cast_cmp, &triangles_cmp, &soft_body_cmp, &binary_cmp }) |c|
        c.finish() catch {
            failed = true;
        };
    if (failed) return error.ParityMismatch;
}
