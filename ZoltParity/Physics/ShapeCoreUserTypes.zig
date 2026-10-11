//! The `zolt_user_types` module of the parity tests (see build.zig): the test shape of the shape core parity tests
//! (`ParityShape`, User1 / User2, a class that derives from Shape directly) and the collide / cast functions that the
//! parity build registers for it through the D4 user hook (Docs/Zolt/CollisionArchitecture.md), exactly like an
//! application. ShapeCoreReference.cpp registers the same functions with CollisionDispatch::sRegisterCollideShape /
//! sRegisterCastShape, so CollisionDispatch (including the reversed functions) and the TransformedShape entry points
//! run through the dispatch tables on both sides.
//!
//! The registered functions and the overrides of ParityShape record what they receive (transforms, scales, shape casts,
//! sub shape IDs, boxes, rays, the early out fraction of the collector) in a `Record` that the shapes point to (const
//! queries: the state lives behind a pointer, Rule M), and add hits computed from their inputs, so the parity test
//! (ShapeCoreParity.zig) can compare the inputs and the (reversed) results bit for bit. This is a module of its own: it
//! imports "zolt" (a module import cycle, like an application) and the parity tests import it as "parity_user_types".

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CollisionDispatch = zolt.CollisionDispatch;
const Float3 = zolt.Float3;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialList = zolt.PhysicsMaterialList;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const RefConst = zolt.RefConst;
const ScaleHelpers = zolt.ScaleHelpers;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const ShapeList = zolt.ShapeList;
const ShapeSubType = zolt.ShapeSubType;
const ShapeType = zolt.ShapeType;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// The user registrations of the parity build (RegisterTypes.user_registrations)
pub const registrations = .{ParityShapeRegistration};

/// What a registered collide function received, must match CollideRecord in ShapeCoreReference.cpp
pub const CollideRecord = extern struct {
    /// Number of calls
    calls: u32,
    /// Sub type of shape 1 / shape 2 (0: User1, 1: User2)
    sub_types: [2]u32,
    scales: [2][3]f32,
    transforms: [2][16]f32,
    /// Sub shape ID creators (ID, number of bits written)
    ids: [2][2]u32,
    /// Early out fraction of the collector after each of the 2 hits
    early_out: [2]f32,
};

/// What a registered cast function received, must match CastRecord in ShapeCoreReference.cpp
pub const CastRecord = extern struct {
    /// Number of calls
    calls: u32,
    /// Sub type of the cast shape / the shape cast against (0: User1, 1: User2)
    sub_types: [2]u32,
    /// The shape cast: center of mass start, direction, scale, world bounds
    start: [16]f32,
    direction: [3]f32,
    cast_scale: [3]f32,
    bounds: [6]f32,
    /// Scale and center of mass transform of the shape cast against
    scale: [3]f32,
    transform2: [16]f32,
    /// Sub shape ID creators (ID, number of bits written)
    ids: [2][2]u32,
    /// Early out fraction of the collector after each of the 2 hits
    early_out: [2]f32,
};

/// What the overrides of ParityShape received from the TransformedShape queries, must match QueryRecord in
/// ShapeCoreReference.cpp
pub const QueryRecord = extern struct {
    /// GetTrianglesStart
    triangles_box: [6]f32,
    triangles_position: [3]f32,
    triangles_rotation: [4]f32,
    triangles_scale: [3]f32,
    /// GetSupportingFace
    face_id: u32,
    face_direction: [3]f32,
    face_scale: [3]f32,
    face_transform: [16]f32,
    /// CollectTransformedShapes
    collect_box: [6]f32,
    /// CastRay (the collector version, used by sCollidePointUsingRayCast): number of calls, back face modes (triangles,
    /// convex: 1 = collide with back faces)
    ray_calls: u32,
    ray_back_face_modes: [2]u32,
};

/// The state of a parity test that the shapes point to: inputs of the registered functions and what they received
pub const Record = struct {
    /// The local ray / point that TransformedShape passed to CastRay / CollidePoint
    last_ray: RayCast = .init(Vec3.zero(), Vec3.zero()),
    last_point: Vec3 = Vec3.zero(),

    /// Penetration depths (collide) / fractions (cast) of the 2 hits that the registered functions add
    hit_values: [2]f32 = .{ 0.0, 0.0 },
    /// CollidePoint uses Shape.collidePointUsingRayCast, CastRay (collector version) adds this many hits
    point_using_ray_cast: bool = false,
    num_ray_hits: u32 = 0,

    collide: CollideRecord = std.mem.zeroes(CollideRecord),
    cast: CastRecord = std.mem.zeroes(CastRecord),
    queries: QueryRecord = std.mem.zeroes(QueryRecord),
};

/// 0 for User1, 1 for User2 (the sub types of ParityShape)
fn subTypeIndex(shape: *const Shape) u32 {
    return @intFromBool(shape.getSubType() == .user2);
}

fn arr3(v: Vec3) [3]f32 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn arr4(v: Vec4) [4]f32 {
    return .{ v.getX(), v.getY(), v.getZ(), v.getW() };
}

pub fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn boxArr(b: AABox) [6]f32 {
    return arr3(b.min) ++ arr3(b.max);
}

fn creatorArr(c: SubShapeIDCreator) [2]u32 {
    return .{ c.getID().getValue(), c.getNumBitsWritten() };
}

/// A shape that derives from Shape directly (User1 or User2), must match ParityShape in ShapeCoreReference.cpp: a box
/// around its center of mass, an optional uniform scale requirement, children and a material for the binary state of a
/// graph. It records what the queries pass to it in `record`.
pub const ParityShape = struct {
    /// Both sub types are this class (C++: `Shape(EShapeType::User1, inSubType)`)
    pub const shape_type: ShapeType = .user1;
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .collectTransformedShapes, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .saveMaterialState, .saveSubShapeState, .getStats, .getVolume, .makeScaleValid };

    base: Shape,
    half_extent: Vec3,
    center_of_mass: Vec3,
    uniform_scale: bool,
    material: RefConst(PhysicsMaterial) = .empty,
    children: std.ArrayList(RefConst(Shape)) = .empty,
    record: *Record,

    /// Constructor of a User1 shape
    pub fn init(allocator: Allocator, half_extent: Vec3, center_of_mass: Vec3, uniform_scale: bool, record: *Record) ParityShape {
        return initSubType(allocator, .user1, half_extent, center_of_mass, uniform_scale, record);
    }

    /// Constructor (sub_type: User1 or User2)
    pub fn initSubType(allocator: Allocator, sub_type: ShapeSubType, half_extent: Vec3, center_of_mass: Vec3, uniform_scale: bool, record: *Record) ParityShape {
        std.debug.assert(sub_type == .user1 or sub_type == .user2);
        return .{ .base = .init(Shape.vtableFor(ParityShape), allocator, .user1, sub_type), .half_extent = half_extent, .center_of_mass = center_of_mass, .uniform_scale = uniform_scale, .record = record };
    }

    pub fn destruct(self: *ParityShape) void {
        self.material.deinit();
        for (self.children.items) |*c| c.deinit();
        self.children.deinit(self.base.allocator);
    }

    pub fn asShape(self: *const ParityShape) *const Shape {
        return &self.base;
    }

    pub fn getCenterOfMass(self: *const ParityShape) Vec3 {
        return self.center_of_mass;
    }

    pub fn getLocalBounds(self: *const ParityShape) AABox {
        return .init(self.half_extent.negate(), self.half_extent);
    }

    pub fn getSubShapeIDBitsRecursive(self: *const ParityShape) u32 {
        _ = self;
        return 0;
    }

    pub fn getInnerRadius(self: *const ParityShape) f32 {
        return self.half_extent.reduceMin();
    }

    pub fn getMassProperties(self: *const ParityShape) MassProperties {
        _ = self;
        return .{};
    }

    pub fn getMaterial(self: *const ParityShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        _ = sub_shape_id;
        return self.material.get() orelse PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(self: *const ParityShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = .{ self, sub_shape_id };
        return local_surface_position.normalizedOr(Vec3.axisY());
    }

    pub fn getSupportingFace(self: *const ParityShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        const r = &self.record.queries;
        r.face_id = sub_shape_id.getValue();
        r.face_direction = arr3(direction);
        r.face_scale = arr3(scale);
        r.face_transform = arr16(center_of_mass_transform);
        out_vertices.append(center_of_mass_transform.mulVec3(scale.mul(self.half_extent)));
        out_vertices.append(center_of_mass_transform.mulVec3(direction));
    }

    pub fn getSubmergedVolume(self: *const ParityShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };
        return .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    pub fn castRay(self: *const ParityShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        self.record.last_ray = ray;
        hit.fraction = 0.5;
        hit.sub_shape_id2 = sub_shape_id_creator.getID();
        return true;
    }

    pub fn castRayCollector(self: *const ParityShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        _ = shape_filter;
        const record = self.record;
        record.last_ray = ray;
        record.queries.ray_calls += 1;
        record.queries.ray_back_face_modes = .{ @intFromBool(ray_cast_settings.back_face_mode_triangles == .collide_with_back_faces), @intFromBool(ray_cast_settings.back_face_mode_convex == .collide_with_back_faces) };
        for (0..record.num_ray_hits) |i|
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .fraction = 0.25 * @as(f32, @floatFromInt(i)), .sub_shape_id2 = sub_shape_id_creator.pushID(@intCast(i), 2).getID() });
    }

    pub fn collidePoint(self: *const ParityShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        self.record.last_point = point;
        if (self.record.point_using_ray_cast)
            Shape.collidePointUsingRayCast(self.asShape(), point, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collideSoftBodyVertices(self: *const ParityShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = .{ self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index };
    }

    pub fn collectTransformedShapes(self: *const ParityShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        self.record.queries.collect_box = boxArr(box);
        Shape.impl.collectTransformedShapes(self.asShape(), box, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn getTrianglesStart(self: *const ParityShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = context;
        const r = &self.record.queries;
        r.triangles_box = boxArr(box);
        r.triangles_position = arr3(position_com);
        r.triangles_rotation = arr4(rotation.getXYZW());
        r.triangles_scale = arr3(scale);
    }

    pub fn getTrianglesNext(self: *const ParityShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        return 0;
    }

    pub fn saveBinaryState(self: *const ParityShape, stream: zolt.StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);
        stream.write(self.half_extent);
        stream.write(self.center_of_mass);
    }

    pub fn saveMaterialState(self: *const ParityShape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        for (out_materials.items) |*m| m.deinit();
        out_materials.clearRetainingCapacity();
        try out_materials.ensureUnusedCapacity(allocator, 1);
        out_materials.appendAssumeCapacity(self.material.clone());
    }

    pub fn saveSubShapeState(self: *const ParityShape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
        try out_sub_shapes.ensureUnusedCapacity(allocator, self.children.items.len);
        for (self.children.items) |c| out_sub_shapes.appendAssumeCapacity(c.clone());
    }

    pub fn getStats(self: *const ParityShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(ParityShape), 0);
    }

    pub fn getVolume(self: *const ParityShape) f32 {
        _ = self;
        return 1.0;
    }

    pub fn makeScaleValid(self: *const ParityShape, scale: Vec3) Vec3 {
        const s = Shape.impl.makeScaleValid(self.asShape(), scale);
        return if (self.uniform_scale) ScaleHelpers.makeUniformScale(s) else s;
    }
};

/// Registration of the parity shape functions (the sRegister of a user shape), must match RegisterParityShapes in
/// ShapeCoreReference.cpp. User1 vs User2 only exists the other way around: the reversed functions of CollisionDispatch.
pub const ParityShapeRegistration = struct {
    pub fn register(comptime r: *CollisionDispatch.Registry) void {
        for ([_]ShapeSubType{ .user1, .user2 }) |s| {
            r.registerCollideShape(s, .user1, collideParity);
            r.registerCastShape(s, .user1, castParity);
        }
        r.registerCollideShape(.user2, .user2, collideParity);
        r.registerCastShape(.user2, .user2, castParity);
        r.registerCollideShape(.user1, .user2, CollisionDispatch.reversedCollideShape);
        r.registerCastShape(.user1, .user2, CollisionDispatch.reversedCastShape);
    }
};

/// Collide function of the parity shapes: records its inputs, adds 2 hits (when they pass the early out fraction) with
/// contact points, axis and faces computed from the inputs
pub fn collideParity(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    _ = .{ collide_shape_settings, shape_filter };
    const p1 = shape1.cast(ParityShape);
    const p2 = shape2.cast(ParityShape);
    const record = p1.record;
    const r = &record.collide;
    r.calls += 1;
    r.sub_types = .{ subTypeIndex(shape1), subTypeIndex(shape2) };
    r.scales = .{ arr3(scale1), arr3(scale2) };
    r.transforms = .{ arr16(center_of_mass_transform1), arr16(center_of_mass_transform2) };
    r.ids = .{ creatorArr(sub_shape_id_creator1), creatorArr(sub_shape_id_creator2) };

    for (record.hit_values, 0..) |depth, i| {
        if (-depth < collector.getEarlyOutFraction()) {
            const contact1 = center_of_mass_transform1.getTranslation();
            const contact2 = center_of_mass_transform2.mulVec3(scale2.mul(p2.half_extent));
            var result = CollideShapeResult.init(contact1, contact2, contact2.sub(contact1), depth, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));
            result.shape1_face.append(center_of_mass_transform1.mulVec3(scale1.mul(p1.half_extent)));
            result.shape1_face.append(center_of_mass_transform1.mulVec3(scale1.mul(p1.half_extent.negate())));
            result.shape2_face.append(center_of_mass_transform2.mulVec3(scale2.mul(p2.center_of_mass)));
            collector.addHit(&result);
        }
        r.early_out[i] = collector.getEarlyOutFraction();
    }
}

/// Cast function of the parity shapes: records its inputs, adds 2 hits (when they pass the early out fraction) with
/// contact points, axis and faces computed from the inputs
pub fn castParity(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    _ = .{ shape_cast_settings, shape_filter };
    const p1 = shape_cast.shape.cast(ParityShape);
    const p2 = shape.cast(ParityShape);
    const record = p1.record;
    const r = &record.cast;
    r.calls += 1;
    r.sub_types = .{ subTypeIndex(shape_cast.shape), subTypeIndex(shape) };
    r.start = arr16(shape_cast.center_of_mass_start);
    r.direction = arr3(shape_cast.direction);
    r.cast_scale = arr3(shape_cast.scale);
    r.bounds = boxArr(shape_cast.shape_world_bounds);
    r.scale = arr3(scale);
    r.transform2 = arr16(center_of_mass_transform2);
    r.ids = .{ creatorArr(sub_shape_id_creator1), creatorArr(sub_shape_id_creator2) };

    for (record.hit_values, 0..) |fraction, i| {
        if (fraction < collector.getEarlyOutFraction()) {
            const contact1 = center_of_mass_transform2.mulVec3(shape_cast.getPointOnRay(fraction));
            const contact2 = center_of_mass_transform2.mulVec3(shape_cast.center_of_mass_start.getTranslation().add(scale.mul(p2.half_extent)));
            var result = ShapeCastResult.init(fraction, contact1, contact2, center_of_mass_transform2.multiply3x3(shape_cast.direction), i == 1, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));
            result.base.shape1_face.append(contact1);
            result.base.shape1_face.append(Vec3.replicate(-0.0)); // Its reversed copy -0 - fraction * world direction shows the sign of zero components of the world direction
            result.base.shape2_face.append(center_of_mass_transform2.mulVec3(p2.center_of_mass));
            result.base.shape2_face.append(contact2);
            collector.addHit(&result);
        }
        r.early_out[i] = collector.getEarlyOutFraction();
    }
}
