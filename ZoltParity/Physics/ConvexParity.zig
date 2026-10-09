//! Parity tests for the convex shapes (Phase 4, Wave A): ConvexShape, SphereShape and BoxShape, with
//! PolyhedronSubmergedVolumeCalculator through ConvexShape::GetSubmergedVolume. The shapes are built from their settings
//! on both sides (`ShapeDesc`). `ParityConvexShape` is a convex shape (UserConvex1) that only provides a support function
//! (a box with rounded edges), the same class as in ConvexReference.cpp, so that the GJK based fallbacks of ConvexShape
//! (CastRay, CollidePoint, GetTrianglesStart / Next) and GetSubmergedVolume are compared as well as the analytic
//! versions of the sphere and the box.
//!
//! Compared bit for bit on random inputs mixed with edge cases: GetLocalBounds, GetWorldSpaceBounds (Mat44 and DMat44),
//! GetCenterOfMass, GetInnerRadius, GetMassProperties, GetVolume, GetStats (triangles), GetSubShapeIDBitsRecursive,
//! IsValidScale / MakeScaleValid, GetSurfaceNormal, GetSupportingFace, the support points of every ESupportMode with
//! scales, CastRay (both overloads, with the AllHit / AnyHit / ClosestHit collectors, back faces, solid or not, early
//! out fractions), CollidePoint, CollisionDispatch::sCollideShapeVsShape and sCastShapeVsShapeWorldSpace for every pair
//! of the three shapes (GJK, EPA, max separation distance, tolerances, faces, back faces, shrunken shapes, deepest point,
//! extra convex radius, early out fractions; all hits in order), GetSubmergedVolume, GetTrianglesStart / Next,
//! CollideSoftBodyVertices, the binary state bytes (and a restore), ConvexShape::sUnitSphereTriangles and the error
//! results of invalid settings. C ABI wrappers: ZoltParity/Physics/ConvexReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const BoxShape = zolt.BoxShape;
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
const ConvexShape = zolt.ConvexShape;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Ref = zolt.Ref;
const RVec3 = zolt.RVec3;
const ScaleHelpers = zolt.ScaleHelpers;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeSubType = zolt.ShapeSubType;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see ConvexReference.cpp
const jolt = struct {
    extern fn jolt_convex_shape_unit_sphere(out_vertices: *[384 * 3]f32) void;
    extern fn jolt_convex_shape_settings(desc: *const ShapeDesc, out_error: *[128]u8) c_int;
    extern fn jolt_convex_shape_properties(desc: *const ShapeDesc, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_convex_shape_support(desc: *const ShapeDesc, mode: c_int, scale: *const P, directions: [*]const f32, num_directions: c_int, out_points: [*]f32) f32;
    extern fn jolt_convex_shape_cast_ray(desc: *const ShapeDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_convex_shape_collide_point(desc: *const ShapeDesc, point: *const P, creator: *const [2]u32, body_id: u32, out_ids: *[2]u32) u32;
    extern fn jolt_convex_shape_collide(input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_convex_shape_cast(input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_convex_shape_submerged_volume(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_convex_shape_triangles(desc: *const ShapeDesc, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, out_counts: *[64]c_int, out_vertices: *[max_vertices * 3]f32, out_default_material: *[max_vertices / 3]c_int) c_int;
    extern fn jolt_convex_shape_binary_state(desc: *const ShapeDesc, user_data: u64, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32) u32;
    extern fn jolt_convex_shape_soft_body(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// The most triangle vertices a shape returns (the unit sphere)
const max_vertices = 384;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Shape description, must match ShapeDesc in ConvexReference.cpp
const ShapeDesc = extern struct {
    /// 0: SphereShape, 1: BoxShape, 2: ParityConvexShape
    kind: u32,
    /// SphereShape
    radius: f32 = 0.0,
    /// BoxShape, ParityConvexShape
    half_extent: P = .{ 0, 0, 0 },
    convex_radius: f32 = 0.0,
    density: f32 = 1000.0,
};

/// Must match PropertiesInput in ConvexReference.cpp
const PropertiesInput = extern struct {
    scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
};

/// Must match PropertiesOutput in ConvexReference.cpp
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
    surface_normal: P,
    face_count: u32,
    face: [32 * 3]f32,
};

/// Must match RayInput in ConvexReference.cpp
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

/// Must match RayOutput in ConvexReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
};

/// Must match CollideInput in ConvexReference.cpp
const CollideInput = extern struct {
    shape1: ShapeDesc,
    shape2: ShapeDesc,
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
};

/// Must match HitOutput in ConvexReference.cpp
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

/// Must match HitsOutput in ConvexReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    hits: [2]HitOutput,
};

/// Must match CastInput in ConvexReference.cpp
const CastInput = extern struct {
    shape1: ShapeDesc,
    shape2: ShapeDesc,
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
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// The parity test shape and the Zolt side of the wrappers

/// A convex shape that only provides a support function: a box with rounded edges (the box shrunk by the convex radius,
/// plus the convex radius). Must match ParityConvexShape in ConvexReference.cpp. CastRay, CollidePoint,
/// GetTrianglesStart / Next and GetSubmergedVolume are ConvexShape's.
const ParityConvexShape = struct {
    pub const shape_sub_type: ShapeSubType = .user_convex1;
    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .getSupportFunction, .collideSoftBodyVertices, .getStats, .getVolume };

    base: ConvexShape,
    half_extent: Vec3,
    convex_radius: f32,

    fn create(allocator: Allocator, half_extent: Vec3, convex_radius: f32) Allocator.Error!*ParityConvexShape {
        const self = try allocator.create(ParityConvexShape);
        self.* = .{ .base = .init(ParityConvexShape, allocator, shape_sub_type, null), .half_extent = half_extent, .convex_radius = convex_radius };
        return self;
    }

    pub fn getLocalBounds(self: *const ParityConvexShape) AABox {
        return .init(self.half_extent.negate(), self.half_extent);
    }

    pub fn getInnerRadius(self: *const ParityConvexShape) f32 {
        return self.half_extent.reduceMin();
    }

    pub fn getMassProperties(self: *const ParityConvexShape) MassProperties {
        var p: MassProperties = .{};
        p.setMassAndInertiaOfSolidBox(self.half_extent.mulScalar(2.0), self.base.getDensity());
        return p;
    }

    pub fn getSurfaceNormal(self: *const ParityConvexShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = .{ self, sub_shape_id };
        return local_surface_position.normalizedOr(Vec3.axisY());
    }

    pub fn getSupportingFace(self: *const ParityConvexShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = sub_shape_id;
        const scaled_half_extent = scale.abs().mul(self.half_extent);
        AABox.init(scaled_half_extent.negate(), scaled_half_extent).getSupportingFace(direction, out_vertices) catch unreachable;
        for (out_vertices.slice()) |*v|
            v.* = center_of_mass_transform.mulVec3(v.*);
    }

    pub fn getSupportFunction(self: *const ParityConvexShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        const scaled_half_extent = scale.abs().mul(self.half_extent);
        const convex_radius = ScaleHelpers.scaleConvexRadius(self.convex_radius, scale);
        const reduced_half_extent = scaled_half_extent.sub(Vec3.replicate(convex_radius));
        const box = AABox.init(reduced_half_extent.negate(), reduced_half_extent);
        switch (mode) {
            .include_convex_radius => {
                const support = buffer.emplace(RoundedBox);
                support.* = .{ .base = .init(RoundedBox), .box = box, .radius = convex_radius };
                return &support.base;
            },
            .exclude_convex_radius, .default => {
                const support = buffer.emplace(ShrunkBox);
                support.* = .{ .base = .init(ShrunkBox), .box = box, .radius = convex_radius };
                return &support.base;
            },
        }
    }

    pub fn collideSoftBodyVertices(self: *const ParityConvexShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = .{ self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index };
    }

    pub fn getStats(self: *const ParityConvexShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(ParityConvexShape), 0);
    }

    pub fn getVolume(self: *const ParityConvexShape) f32 {
        return self.getLocalBounds().getVolume();
    }

    /// The box including the convex radius (rounded edges)
    const RoundedBox = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        box: AABox,
        radius: f32,

        pub fn getSupport(self: *const RoundedBox, direction: Vec3) Vec3 {
            const len = direction.length();
            const p = self.box.getSupport(direction);
            return if (len > 0.0) p.add(direction.mulScalar(self.radius / len)) else p;
        }

        pub fn getConvexRadius(self: *const RoundedBox) f32 {
            _ = self;
            return 0.0;
        }
    };

    /// The box excluding the convex radius
    const ShrunkBox = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };

        base: ConvexShape.Support,
        box: AABox,
        radius: f32,

        pub fn getSupport(self: *const ShrunkBox, direction: Vec3) Vec3 {
            return self.box.getSupport(direction);
        }

        pub fn getConvexRadius(self: *const ShrunkBox) f32 {
            return self.radius;
        }
    };
};

/// Build the shape (spheres and boxes from their settings), null when the settings are invalid (the error text is
/// copied to `out_error` when given)
fn createShape(allocator: Allocator, desc: ShapeDesc, out_error: ?*[128]u8) !?Ref(Shape) {
    var result = switch (desc.kind) {
        0 => blk: {
            var settings = SphereShapeSettings.init(allocator, desc.radius, .{});
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        1 => blk: {
            var settings = BoxShapeSettings.init(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius });
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        else => {
            const shape = try ParityConvexShape.create(allocator, vec3(desc.half_extent), desc.convex_radius);
            shape.base.setDensity(desc.density);
            return Ref(Shape).init(&shape.base.base);
        },
    };
    defer result.deinit();
    if (result.hasError()) {
        if (out_error) |e| {
            @memset(e, 0);
            const text = result.getError();
            @memcpy(e[0..@min(text.len, 127)], text[0..@min(text.len, 127)]);
        }
        return null;
    }
    return Ref(Shape).init(result.getPtr());
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

fn zoltCollide(allocator: Allocator, input: *const CollideInput) !HitsOutput {
    var shape1 = (try createShape(allocator, input.shape1, null)).?;
    defer shape1.deinit();
    var shape2 = (try createShape(allocator, input.shape2, null)).?;
    defer shape2.deinit();
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = input.max_separation_distance;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.collideShapeVsShape(shape1.get().?, shape2.get().?, vec3(input.scale1), vec3(input.scale2), mat44(input.transform1), mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &settings, &collector.base, &.{});
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

fn zoltCast(allocator: Allocator, input: *const CastInput) !HitsOutput {
    var shape1 = (try createShape(allocator, input.shape1, null)).?;
    defer shape1.deinit();
    var shape2 = (try createShape(allocator, input.shape2, null)).?;
    defer shape2.deinit();
    var settings: ShapeCastSettings = .{};
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.use_shrunken_shape_and_convex_radius = input.use_shrunken_shape != 0;
    settings.return_deepest_point = input.return_deepest_point != 0;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    const shape_cast = ShapeCast.init(shape1.get().?, vec3(input.scale1), mat44(input.start), vec3(input.direction));
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &settings, shape2.get().?, vec3(input.scale2), &.{}, mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &collector.base);
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

    /// A shape: sphere, box (with or without convex radius, sometimes flat) or the parity shape
    fn shape(self: *Gen) ShapeDesc {
        const density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
        switch (self.index(3)) {
            0 => return .{ .kind = 0, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 3), .density = density },
            1 => {
                var half_extent = self.plainVec(0.05, 3);
                if (self.oneIn(10)) half_extent[self.index(3)] = 0.0; // Flat box
                if (self.oneIn(5)) half_extent = .{ 1, 1, 1 };
                const convex_radius: f32 = switch (self.index(4)) {
                    0 => 0.0,
                    1 => 0.05, // cDefaultConvexRadius
                    2 => self.plain(0, 5), // Bigger than the box: clamped
                    else => self.plain(0, 0.2),
                };
                return .{ .kind = 1, .half_extent = half_extent, .convex_radius = convex_radius, .density = density };
            },
            else => {
                const half_extent = self.plainVec(0.1, 2.5);
                const min_extent = @min(half_extent[0], @min(half_extent[1], half_extent[2]));
                const convex_radius: f32 = if (self.oneIn(4)) 0.0 else self.plain(0, 0.9 * min_extent);
                return .{ .kind = 2, .half_extent = half_extent, .convex_radius = convex_radius, .density = density };
            },
        }
    }

    /// A valid scale for the shape: uniform (with signs) for a sphere, anything non zero for the others
    fn scale(self: *Gen, desc: ShapeDesc) P {
        if (self.oneIn(5)) return .{ 1, 1, 1 };
        const s = if (self.oneIn(4)) self.grid(2) else self.plain(0.2, 2.5);
        const m = if (s == 0.0) 1.0 else @abs(s);
        if (desc.kind == 0)
            return .{ if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m };
        var r = self.plainVec(0.2, 2.5);
        for (&r) |*c| {
            if (self.oneIn(3)) c.* = -c.*;
        }
        return r;
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
};

/// The extent of a shape (for placing shapes near each other)
fn extentOf(desc: ShapeDesc) f32 {
    return if (desc.kind == 0) desc.radius else @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2]));
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "Convex parity: ConvexShape::sUnitSphereTriangles" {
    var jolt_vertices: [384 * 3]f32 = undefined;
    jolt.jolt_convex_shape_unit_sphere(&jolt_vertices);
    var zolt_vertices: [384 * 3]f32 = undefined;
    for (ConvexShape.unit_sphere_triangles.constSlice(), 0..) |v, i| zolt_vertices[3 * i ..][0..3].* = arr3(v);
    var checker: Checker = .{ .name = "unit sphere triangles" };
    for (0..384) |i| checker.check(.{i}, zolt_vertices[3 * i ..][0..3].*, jolt_vertices[3 * i ..][0..3].*);
    try checker.finish();
}

test "Convex parity: settings and Jolt's error texts" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "settings" };
    for (0..10_000) |i| {
        var desc: ShapeDesc = .{ .kind = @intCast(gen.index(2)) };
        desc.radius = if (i < 4) ([_]f32{ 0.0, -0.0, -1.0, 1.0e-30 })[i] else gen.float(-1, 1);
        desc.half_extent = gen.vec(-0.5, 2);
        desc.convex_radius = gen.float(-0.5, 1);
        var jolt_error: [128]u8 = undefined;
        const jolt_valid = jolt.jolt_convex_shape_settings(&desc, &jolt_error);
        var zolt_error: [128]u8 = @splat(0);
        var shape = try createShape(allocator, desc, &zolt_error);
        const zolt_valid: c_int = @intFromBool(shape != null);
        if (shape) |*s| s.deinit();
        checker.check(.{ desc.kind, desc.radius, desc.half_extent, desc.convex_radius }, .{ zolt_valid, zolt_error }, .{ jolt_valid, jolt_error });
    }
    try checker.finish();
}

test "Convex parity: bounds, mass properties, volume, scales, surface normal, supporting face" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "properties" };
    for (0..iterations) |_| {
        const desc = gen.shape();
        var input: PropertiesInput = .{
            .scale = gen.scale(desc),
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = gen.vec(-4, 4),
            .direction = gen.direction(3),
        };
        // IsValidScale / MakeScaleValid on any scale: for a sphere only where the scale is valid (GetWorldSpaceBounds asserts)
        if (desc.kind != 0 and gen.oneIn(2)) input.scale = gen.anyScale();

        var jolt_output = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_convex_shape_properties(&desc, &input, &jolt_output);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const zolt_output = zoltProperties(shape.get().?, &input);
        checker.check(.{ desc, input }, zolt_output, jolt_output);

        // IsValidScale / MakeScaleValid on any scale (also invalid ones for the sphere)
        const any_scale = gen.anyScale();
        var jolt_scale_input = input;
        jolt_scale_input.scale = any_scale;
        if (desc.kind == 0) {
            // Only the scale functions (the rest needs a valid scale), through Zolt directly and Jolt's properties with a
            // valid scale for the other values
            const s = shape.get().?;
            var jolt_any = std.mem.zeroes(PropertiesOutput);
            jolt.jolt_convex_shape_properties(&desc, &jolt_scale_input, &jolt_any);
            checker.check(.{ desc, any_scale }, .{ @as(c_int, @intFromBool(s.isValidScale(vec3(any_scale)))), arr3(s.makeScaleValid(vec3(any_scale))) }, .{ jolt_any.is_valid_scale, jolt_any.scale_valid });
        }
    }
    try checker.finish();
}

test "Convex parity: support functions of every mode" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "support" };
    for (0..iterations / 4) |_| {
        const desc = gen.shape();
        const scale = gen.scale(desc);
        var directions: [8 * 3]f32 = undefined;
        for (0..8) |d| directions[3 * d ..][0..3].* = gen.direction(3);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const convex = shape.get().?.cast(ConvexShape);
        for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .include_convex_radius, .default }) |mode| {
            var jolt_points: [8 * 3]f32 = @splat(0);
            const jolt_radius = jolt.jolt_convex_shape_support(&desc, @intFromEnum(mode), &scale, &directions, 8, &jolt_points);
            var buffer: ConvexShape.SupportBuffer = .{};
            const support = convex.getSupportFunction(mode, &buffer, vec3(scale));
            var zolt_points: [8 * 3]f32 = @splat(0);
            for (0..8) |d| zolt_points[3 * d ..][0..3].* = arr3(support.getSupport(vec3(directions[3 * d ..][0..3].*)));
            checker.check(.{ desc, mode, scale, directions }, .{ support.getConvexRadius(), zolt_points }, .{ jolt_radius, jolt_points });
        }
    }
    try checker.finish();
}

test "Convex parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    for (0..iterations) |_| {
        const desc = gen.shape();
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const extent = extentOf(desc);

        // Rays from outside through the shape, from inside, along faces, degenerate directions
        var input: RayInput = .{
            .origin = gen.vec(-2 * extent - 1, 2 * extent + 1),
            .direction = gen.direction(4 * extent + 2),
            .creator = gen.creator(),
            .fraction = if (gen.oneIn(2)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
            .back_face_mode = @intFromBool(gen.oneIn(2)),
            .treat_convex_as_solid = @intFromBool(!gen.oneIn(3)),
            .collector = @intCast(gen.index(3)),
            .early_out = if (gen.oneIn(2)) 2.0 else gen.plain(0, 1),
            .body_id = gen.next() & 0x7fffff,
        };
        if (gen.oneIn(4)) {
            // Aim at the center
            const target = gen.plainVec(-0.5 * extent, 0.5 * extent);
            input.direction = arr3(vec3(target).sub(vec3(input.origin)).mulScalar(gen.plain(0.5, 3)));
        }
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_convex_shape_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, shape.get().?, &input);
        rays.check(.{ desc, input }, zolt_output, jolt_output);

        // Points inside, on the surface (at the extents) and outside
        var point = gen.vec(-1.5 * extent, 1.5 * extent);
        if (gen.oneIn(5)) point[gen.index(3)] = if (desc.kind == 0) desc.radius else desc.half_extent[gen.index(3)];
        var jolt_ids: [2]u32 = undefined;
        const jolt_count = jolt.jolt_convex_shape_collide_point(&desc, &point, &input.creator, input.body_id, &jolt_ids);
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
        collector.base.setContext(&context);
        shape.get().?.collidePoint(vec3(point), makeCreator(input.creator), &collector.base, &.{});
        try collector.checkError();
        var zolt_ids: [2]u32 = .{ 0, 0 };
        for (collector.hits.items) |h| zolt_ids = .{ h.body_id.getIndexAndSequenceNumber(), h.sub_shape_id2.getValue() };
        points.check(.{ desc, point }, .{ @as(u32, @intCast(collector.hits.items.len)), zolt_ids }, .{ jolt_count, jolt_ids });
    }
    try finishAll(&.{ &rays, &points });
}

test "Convex parity: collide convex vs convex through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    for (0..iterations) |i| {
        const shape1 = gen.shape();
        const shape2 = gen.shape();
        const transform1 = gen.transform(5);
        // Place shape 2 near shape 1 so that about half of the pairs collide
        const reach = extentOf(shape1) + extentOf(shape2);
        var relative = gen.transform(1.2 * reach);
        if (gen.oneIn(10)) relative.setTranslation(Vec3.zero()); // Same center: near zero penetration axis
        if (gen.oneIn(10)) relative = Mat44.identity();
        var input: CollideInput = .{
            .shape1 = shape1,
            .shape2 = shape2,
            .scale1 = gen.scale(shape1),
            .scale2 = gen.scale(shape2),
            .transform1 = arr16(transform1),
            .transform2 = arr16(transform1.mul(relative)),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .max_separation_distance = switch (gen.index(5)) {
                0, 1 => 0.0,
                2 => gen.plain(0, 1),
                3 => gen.plain(1, 3), // Clamped to 1 in the EPA path
                else => 100.0,
            },
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
            .body_id = gen.next() & 0x7fffff,
        };
        if (i < 4) {
            // Exactly touching unit spheres along each axis / at the same position
            input.shape1 = .{ .kind = 0, .radius = 1.0 };
            input.shape2 = .{ .kind = 0, .radius = 1.0 };
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = .{ 1, 1, 1 };
            input.transform1 = arr16(Mat44.identity());
            var t: P = .{ 0, 0, 0 };
            if (i < 3) t[i] = 2.0;
            input.transform2 = arr16(Mat44.translation(vec3(t)));
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_convex_shape_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 4 and num_hits < 3 * iterations / 4); // Both paths are exercised
}

test "Convex parity: cast convex vs convex through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    for (0..iterations) |_| {
        const shape1 = gen.shape();
        const shape2 = gen.shape();
        const transform2 = gen.transform(5);
        const reach = extentOf(shape1) + extentOf(shape2);
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
        const input: CastInput = .{
            .shape1 = shape1,
            .shape2 = shape2,
            .scale1 = gen.scale(shape1),
            .start = arr16(start),
            .direction = arr3(direction),
            .scale2 = gen.scale(shape2),
            .transform2 = arr16(transform2),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0,
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .return_deepest_point = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
        };
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_convex_shape_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 5 and num_hits < 4 * iterations / 5); // Hits and misses
}

test "Convex parity: GetSubmergedVolume" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "submerged volume" };
    for (0..iterations) |_| {
        const desc = gen.shape();
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
        const plane = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), normal).normal_and_constant);
        var jolt_values: [5]f32 = undefined;
        jolt.jolt_convex_shape_submerged_volume(&desc, &transform, &scale, &plane, &jolt_values);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const r = shape.get().?.getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(plane)));
        const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
        checker.check(.{ desc, scale, transform, plane }, zolt_values, jolt_values);
    }
    try checker.finish();
}

test "Convex parity: GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "triangles" };
    for (0..iterations / 10) |_| {
        const desc = gen.shape();
        const scale = gen.scale(desc);
        const position = gen.vec(-10, 10);
        const rotation = arr4(gen.rotation().getXYZW());
        const max_requested: c_int = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(200));

        var jolt_counts: [64]c_int = @splat(0);
        var jolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var jolt_default: [max_vertices / 3]c_int = @splat(0);
        const jolt_calls = jolt.jolt_convex_shape_triangles(&desc, &position, &rotation, &scale, max_requested, &jolt_counts, &jolt_vertices, &jolt_default);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        var zolt_counts: [64]c_int = @splat(0);
        var zolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var zolt_default: [max_vertices / 3]c_int = @splat(0);
        var context: Shape.GetTrianglesContext = .{};
        shape.get().?.getTrianglesStart(&context, AABox.biggest(), vec3(position), quat(rotation), vec3(scale));
        var triangles: [3 * 232]Float3 = undefined;
        var materials: [232]*const PhysicsMaterial = undefined;
        var zolt_calls: c_int = 0;
        var out: usize = 0;
        var out_material: usize = 0;
        while (true) {
            const count = shape.get().?.getTrianglesNext(&context, @intCast(max_requested), triangles[0 .. 3 * @as(usize, @intCast(max_requested))], materials[0..@intCast(max_requested)]);
            zolt_counts[@intCast(zolt_calls)] = @intCast(count);
            zolt_calls += 1;
            for (triangles[0 .. 3 * count]) |t| {
                zolt_vertices[out..][0..3].* = .{ t.x, t.y, t.z };
                out += 3;
            }
            for (materials[0..count]) |m| {
                zolt_default[out_material] = @intFromBool(m == PhysicsMaterial.default);
                out_material += 1;
            }
            if (count == 0 or zolt_calls == 64) break;
        }
        checker.check(.{ desc, scale, position, rotation, max_requested }, .{ zolt_calls, zolt_counts, zolt_vertices, zolt_default }, .{ jolt_calls, jolt_counts, jolt_vertices, jolt_default });
    }
    try checker.finish();
}

test "Convex parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 4) |_| {
        var desc = gen.shape();
        while (desc.kind == 2) desc = gen.shape(); // ParityConvexShape does not implement it
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        const extent = extentOf(desc) * 2.5;
        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(vec3(gen.vec(-extent, extent))));
            if (gen.oneIn(8)) positions[3 * v ..][0..3].* = arr3(mat44(transform).getTranslation()); // At the center
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_convex_shape_soft_body(&desc, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
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
        shape.get().?.collideSoftBodyVertices(mat44(transform), vec3(scale), &vertices, n, 3);
        var zolt_plane_values: [n * 4]f32 = undefined;
        for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
        var zolt_index_values: [n]c_int = undefined;
        for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
        checker.check(.{ desc, scale, transform, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
    }
    try checker.finish();
}

test "Convex parity: binary state" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "binary state" };
    for (0..iterations / 10) |_| {
        var desc = gen.shape();
        while (desc.kind == 2) desc = gen.shape(); // Only registered shapes can be restored
        const user_data = (@as(u64, gen.next()) << 32) | gen.next();
        var jolt_bytes: [64]u8 = @splat(0);
        var jolt_restored: [64]u8 = @splat(0);
        var jolt_restored_size: u32 = 0;
        const jolt_size = jolt.jolt_convex_shape_binary_state(&desc, user_data, &jolt_bytes, 64, &jolt_restored, &jolt_restored_size);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        shape.get().?.setUserData(user_data);
        var zolt_bytes: [64]u8 = @splat(0);
        var writer: std.Io.Writer = .fixed(&zolt_bytes);
        var stream_out = StreamOutWrapper.init(&writer);
        shape.get().?.saveBinaryState(stream_out.streamOut());
        const zolt_size: u32 = @intCast(writer.buffered().len);

        // Restore Jolt's bytes and save again
        var zolt_restored: [64]u8 = @splat(0);
        var zolt_restored_size: u32 = 0;
        var reader: std.Io.Reader = .fixed(jolt_bytes[0..jolt_size]);
        var stream_in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
        defer result.deinit();
        if (result.isValid()) {
            var restored_writer: std.Io.Writer = .fixed(&zolt_restored);
            var restored_out = StreamOutWrapper.init(&restored_writer);
            result.getPtr().?.saveBinaryState(restored_out.streamOut());
            zolt_restored_size = @intCast(restored_writer.buffered().len);
        }
        checker.check(.{ desc, user_data }, .{ zolt_size, zolt_bytes, zolt_restored_size, zolt_restored }, .{ jolt_size, jolt_bytes, jolt_restored_size, jolt_restored });
    }
    try checker.finish();
}
