//! Prototype of the Phase 4 collision architecture (Docs/Zolt/CollisionArchitecture.md).
//!
//! The files next to this one mirror the final layout (Zolt/Core, Zolt/Physics/Body, Zolt/Physics/Collision[/Shape],
//! Zolt/RegisterTypes.zig) in reduced form: Shape / ConvexShape / Sphere / Box / Triangle (stub) / Decorated / Scaled /
//! Compound / StaticCompound, settings and ShapeResult, reference counting and destroy, collectors, the dispatch table
//! with Jolt's override order and reversed entries, filters, materials, GetTrianglesContext, TransformedShape and the
//! binary state. Every test uses std.testing.allocator (leak checked).

const std = @import("std");
const Allocator = std.mem.Allocator;
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualStrings = std.testing.expectEqualStrings;
const expectError = std.testing.expectError;
const zolt = @import("zolt");
const math = zolt.math;
const AABox = zolt.AABox;
const Color = zolt.Color;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RMat44 = zolt.RMat44;
const RVec3 = zolt.RVec3;
const StaticArray = zolt.StaticArray;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const Vec3 = zolt.Vec3;

const virtual = @import("Core/Virtual.zig");
const RegisterTypes = @import("RegisterTypes.zig");
const BodyID = @import("Physics/Body/BodyID.zig").BodyID;
const Body = @import("Physics/Body/Body.zig").Body;
const MassProperties = @import("Physics/Body/MassProperties.zig").MassProperties;
const ShapeFile = @import("Physics/Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const ConvexShape = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShape;
const SphereShapeFile = @import("Physics/Collision/Shape/SphereShape.zig");
const SphereShape = SphereShapeFile.SphereShape;
const SphereShapeSettings = SphereShapeFile.SphereShapeSettings;
const BoxShapeFile = @import("Physics/Collision/Shape/BoxShape.zig");
const BoxShape = BoxShapeFile.BoxShape;
const BoxShapeSettings = BoxShapeFile.BoxShapeSettings;
const TriangleShape = @import("Physics/Collision/Shape/TriangleShape.zig").TriangleShape;
const ScaledShapeFile = @import("Physics/Collision/Shape/ScaledShape.zig");
const ScaledShape = ScaledShapeFile.ScaledShape;
const ScaledShapeSettings = ScaledShapeFile.ScaledShapeSettings;
const CompoundShape = @import("Physics/Collision/Shape/CompoundShape.zig").CompoundShape;
const StaticCompoundShapeFile = @import("Physics/Collision/Shape/StaticCompoundShape.zig");
const StaticCompoundShape = StaticCompoundShapeFile.StaticCompoundShape;
const StaticCompoundShapeSettings = StaticCompoundShapeFile.StaticCompoundShapeSettings;
const GetTrianglesContextVertexList = @import("Physics/Collision/Shape/GetTrianglesContext.zig").GetTrianglesContextVertexList;
const SubShapeIDFile = @import("Physics/Collision/Shape/SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const CollisionCollectorImpl = @import("Physics/Collision/CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const ClosestHitPerBodyCollisionCollector = CollisionCollectorImpl.ClosestHitPerBodyCollisionCollector;
const AnyHitCollisionCollector = CollisionCollectorImpl.AnyHitCollisionCollector;
const CollisionDispatch = @import("Physics/Collision/CollisionDispatch.zig");
const ShapeFilter = @import("Physics/Collision/ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("Physics/Collision/TransformedShape.zig").TransformedShape;
const RayCastFile = @import("Physics/Collision/RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RRayCast = RayCastFile.RRayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("Physics/Collision/CastResult.zig").RayCastResult;
const CollidePointResult = @import("Physics/Collision/CollidePointResult.zig").CollidePointResult;
const CollideShapeFile = @import("Physics/Collision/CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const ShapeCastFile = @import("Physics/Collision/ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;

test {
    _ = @import("Core/Virtual.zig");
    _ = @import("Core/Result.zig");
}

fn rvec3(x: f32, y: f32, z: f32) RVec3 {
    return RVec3.init(x, y, z);
}

/// A user defined convex shape (UserConvex1): implements the pure virtual functions and an analytic castRay. It does
/// not override castRayCollector / collidePoint, so ConvexShape's versions are used and must call this castRay
/// virtually (C++: the unqualified call CastRay(...) inside ConvexShape::CastRay(collector version)).
const UserConvexShape = struct {
    pub const shape_sub_type: ShapeSubType = .user_convex1;
    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportFunction, .castRay, .getStats, .getVolume };

    base: ConvexShape,
    radius: f32,
    /// Mutable state of a const query lives behind a pointer (Rule M)
    cast_ray_calls: *u32,

    fn init(allocator: Allocator, radius: f32, cast_ray_calls: *u32) UserConvexShape {
        return .{ .base = .init(UserConvexShape, allocator, shape_sub_type, null), .radius = radius, .cast_ray_calls = cast_ray_calls };
    }

    fn asShape(self: *const UserConvexShape) *const Shape {
        return &self.base.base;
    }

    pub fn getLocalBounds(self: *const UserConvexShape) AABox {
        return .fromCenterAndRadius(Vec3.zero(), self.radius);
    }

    pub fn getInnerRadius(self: *const UserConvexShape) f32 {
        return self.radius;
    }

    pub fn getMassProperties(self: *const UserConvexShape) MassProperties {
        _ = self;
        return .{};
    }

    pub fn getSurfaceNormal(self: *const UserConvexShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = .{ self, sub_shape_id };
        return local_surface_position.normalized();
    }

    pub fn getSupportFunction(self: *const UserConvexShape, mode: ConvexShape.SupportMode, buffer: *ConvexShape.SupportBuffer, scale: Vec3) *const ConvexShape.Support {
        // In place construction in the caller's buffer (C++ `new (&inBuffer) PointWithRadius(...)`)
        const support = buffer.emplace(PointWithRadius);
        support.* = .{ .base = .init(PointWithRadius), .radius = self.radius * scale.getX(), .exclude_convex_radius = mode != .include_convex_radius };
        return &support.base;
    }

    pub fn castRay(self: *const UserConvexShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        self.cast_ray_calls.* += 1;
        const fraction = zolt.raySphere(ray.origin, ray.direction, Vec3.zero(), self.radius);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    pub fn getStats(self: *const UserConvexShape) Shape.Stats {
        _ = self;
        return .{ .size_bytes = @sizeOf(UserConvexShape), .num_triangles = 0 };
    }

    pub fn getVolume(self: *const UserConvexShape) f32 {
        return @as(f32, 4.0) / 3.0 * math.pi * math.cubed(self.radius);
    }

    const PointWithRadius = struct {
        pub const overrides = .{ .getSupport, .getConvexRadius };
        base: ConvexShape.Support,
        radius: f32,
        /// Default / ExcludeConvexRadius: a point with a convex radius, IncludeConvexRadius: the sphere itself
        exclude_convex_radius: bool,

        pub fn getSupport(self: *const PointWithRadius, direction: Vec3) Vec3 {
            if (self.exclude_convex_radius) return Vec3.zero();
            const len = direction.length();
            return if (len > 0.0) direction.mulScalar(self.radius / len) else Vec3.zero();
        }

        pub fn getConvexRadius(self: *const PointWithRadius) f32 {
            return if (self.exclude_convex_radius) self.radius else 0.0;
        }
    };
};

test "CollisionPrototype: virtual dispatch, inherited overrides and checked downcasts" {
    const allocator = std.testing.allocator;

    // Embedded (stack) user shape: ConvexShape's collector castRay must call the user's castRay through the vtable
    var calls: u32 = 0;
    var user = UserConvexShape.init(allocator, 1.0, &calls);
    user.asShape().setEmbedded();
    defer virtual.upcast(Shape, &user).deinit();

    const shape = user.asShape();
    var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer collector.deinit();
    const ray = RayCast.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0));
    shape.castRayCollector(ray, &.{}, .{}, &collector.base, &.{});
    try collector.checkError();
    try expectEqual(@as(u32, 1), calls); // Virtual call to UserConvexShape.castRay, not the GJK fallback
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expectEqual(@as(f32, 0.25), collector.hits.items[0].fraction);

    // Inherited from ConvexShape.impl: collidePoint (GJK on the support function), getMaterial, getSubShapeIDBitsRecursive
    var point_collector = AnyHitCollisionCollector(CollidePointCollector).init();
    defer point_collector.deinit();
    shape.collidePoint(Vec3.init(0.5, 0, 0), .{}, &point_collector.base, &.{});
    try expect(point_collector.hadHit());
    try expect(shape.getMaterial(.empty) == PhysicsMaterial.default);
    try expectEqual(@as(u32, 0), shape.getSubShapeIDBitsRecursive());

    // Checked downcasts: concrete classes by sub type, abstract classes by type
    try expect(shape.isKindOf(ConvexShape) and !shape.isKindOf(SphereShape) and !shape.isKindOf(CompoundShape));
    try expect(shape.cast(UserConvexShape) == &user);
    try expect(shape.cast(ConvexShape) == &user.base);

    // A user convex shape collides with built-in convex shapes without registering anything (ConvexShape.register
    // covers all convex sub types, including UserConvex1..8)
    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer hits.deinit();
    CollisionDispatch.collideShapeVsShape(shape, sphere.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(1.5, 0, 0)), .{}, .{}, &.{}, &hits.base, &.{});
    try hits.checkError();
    try expectEqual(@as(usize, 1), hits.hits.items.len);
    try expect(@abs(hits.hits.items[0].penetration_depth - 0.5) < 1.0e-3);
}

test "CollisionPrototype: a collector derived from ClosestHitPerBody inherits its overrides" {
    const allocator = std.testing.allocator;

    // Jolt's UnitTests derive from ClosestHitPerBodyCollisionCollector (CastShapeTests.cpp): the builder looks up
    // overrides in every level, so onBody / onBodyEnd / reset of the parent are used
    const MyCollector = struct {
        pub const overrides = .{.addHit};

        base: ClosestHitPerBodyCollisionCollector(CastRayCollector),
        num_add_hit: u32 = 0,

        fn init(a: Allocator) @This() {
            return .{ .base = .initDerived(@This(), a) };
        }

        pub fn addHit(self: *@This(), result: *const RayCastResult) void {
            self.num_add_hit += 1;
            self.base.addHit(result); // C++ ClosestHitPerBodyCollisionCollector::AddHit(inResult)
        }
    };

    var sphere = try SphereShape.create(allocator, 1.0, .{});
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();

    var collector = MyCollector.init(allocator);
    defer collector.base.deinit();
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    const bodies = [_]Body{ .{ .id = .init(1) }, .{ .id = .init(2) } };
    for (&bodies, 0..) |*body, i| {
        var ts = TransformedShape.init(rvec3(@floatFromInt(10 * i), 0, 0), Quat.identity(), sphere.asShape(), body.getID(), .{});
        defer ts.deinit();
        const root: *CastRayCollector = &collector.base.base;
        root.onBody(body); // NarrowPhaseQuery calls these through the vtable
        ts.castRayCollector(.init(rvec3(@as(f32, @floatFromInt(10 * i)) - 2.0, 0, 0), Vec3.init(4, 0, 0)), &settings, root, .{});
        root.onBodyEnd();
        try expectEqual(CastRayCollector.Traits.initial_early_out_fraction, root.getEarlyOutFraction()); // Restored by ClosestHitPerBody.onBodyEnd
    }
    try collector.base.checkError();
    try expectEqual(@as(u32, 2), collector.num_add_hit); // Front face hits only: after the first hit the early out fraction is 0.25
    try expectEqual(@as(usize, 2), collector.base.hits.items.len);
    try expect(collector.base.hits.items[0].body_id.eql(.init(1)));
    try expect(collector.base.hits.items[1].body_id.eql(.init(2)));
    try expectEqual(@as(f32, 0.25), collector.base.hits.items[1].fraction);
}

test "CollisionPrototype: settings, cached result and Jolt's error strings" {
    const allocator = std.testing.allocator;

    // Invalid settings: the error is cached
    var bad = SphereShapeSettings.init(allocator, 0.0, .{});
    defer bad.deinit();
    {
        var r1 = try bad.asShapeSettings().createShape(allocator);
        defer r1.deinit();
        try expect(r1.hasError());
        try expectEqualStrings("Invalid radius", r1.getError());
        var r2 = try bad.createShape(allocator); // static call on the concrete settings
        defer r2.deinit();
        try expectEqualStrings("Invalid radius", r2.getError());
    }

    // Change the settings: clearCachedResult builds a new shape, later calls return the cached one
    bad.radius = 2.0;
    bad.asShapeSettings().clearCachedResult();
    {
        var r1 = try bad.asShapeSettings().createShape(allocator);
        defer r1.deinit();
        var r2 = try bad.asShapeSettings().createShape(allocator);
        defer r2.deinit();
        try expect(r1.isValid() and r1.getPtr() == r2.getPtr());
        const shape = r1.getPtr().?;
        try expectEqual(@as(u32, 3), shape.ref_count.get()); // cache + r1 + r2
        try expectEqual(@as(f32, 2.0), shape.cast(SphereShape).getRadius());
    }

    // Box errors in Jolt's order
    var box1 = BoxShapeSettings.init(allocator, Vec3.init(1, -1, 1), .{});
    defer box1.deinit();
    var b1 = try box1.asShapeSettings().createShape(allocator);
    defer b1.deinit();
    try expectEqualStrings("Invalid half extent", b1.getError());
    var box2 = BoxShapeSettings.init(allocator, Vec3.one(), .{ .convex_radius = -1.0 });
    defer box2.deinit();
    var b2 = try box2.asShapeSettings().createShape(allocator);
    defer b2.deinit();
    try expectEqualStrings("Invalid convex radius", b2.getError());

    // A child error is forwarded by the decorated shape (C++ outResult = child_result); heap child settings are
    // reference counted and released when the parent settings are destroyed
    const child = try SphereShapeSettings.create(allocator, -1.0, .{});
    var scaled = ScaledShapeSettings.init(allocator, child.asShapeSettings(), Vec3.replicate(2.0));
    defer scaled.deinit();
    var s1 = try scaled.asShapeSettings().createShape(allocator);
    defer s1.deinit();
    try expectEqualStrings("Invalid radius", s1.getError());

    var zero_scale = ScaledShapeSettings.init(allocator, (try SphereShapeSettings.create(allocator, 1.0, .{})).asShapeSettings(), Vec3.zero());
    defer zero_scale.deinit();
    var s2 = try zero_scale.asShapeSettings().createShape(allocator);
    defer s2.deinit();
    try expectEqualStrings("Can't use zero scale!", s2.getError());

    var no_inner = ScaledShapeSettings.init(allocator, null, Vec3.one());
    defer no_inner.deinit();
    var s3 = try no_inner.asShapeSettings().createShape(allocator);
    defer s3.deinit();
    try expectEqualStrings("Inner shape is null!", s3.getError());

    // Compound errors and the 1 sub shape shortcut
    var empty_compound = StaticCompoundShapeSettings.init(allocator);
    defer empty_compound.deinit();
    var c1 = try empty_compound.asShapeSettings().createShape(allocator);
    defer c1.deinit();
    try expectEqualStrings("Compound needs a sub shape!", c1.getError());

    const box = try BoxShape.create(allocator, Vec3.one(), .{});
    var single = StaticCompoundShapeSettings.init(allocator);
    defer single.deinit();
    try single.base.addShapePtr(Vec3.zero(), Quat.identity(), box.asShape(), .{});
    var c2 = try single.asShapeSettings().createShape(allocator);
    defer c2.deinit();
    try expect(c2.getPtr() == box.asShapeMut()); // No compound needed: the shape itself
}

/// Calls through a function pointer so that the optimizer cannot see the write (Rule M regression test)
noinline fn createThroughVTable(settings: *ShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
    return settings.createShape(allocator);
}

test "CollisionPrototype: Rule M, mutable C++ members are written through mutable pointers" {
    const allocator = std.testing.allocator;

    // The cache (C++ `mutable mCachedResult`) is written by createShape through a *ShapeSettings: the caller sees it
    // in every optimize mode (with a *const receiver and @constCast, ReleaseFast loses the write)
    var settings = SphereShapeSettings.init(allocator, 1.0, .{});
    defer settings.deinit();
    var result = try createThroughVTable(settings.asShapeSettings(), allocator);
    defer result.deinit();
    try expect(settings.base.base.cached_result.isValid());
    try expect(settings.base.base.cached_result.getPtr() == result.getPtr());

    // ShapeFilter.body_id2 (C++ `mutable mBodyID2`) is written by the TransformedShape entry point through a
    // *ShapeFilter and read by the filter during the query; the filter's own mutable state is behind a pointer
    const RecordingFilter = struct {
        pub const overrides = .{.shouldCollide};
        base: ShapeFilter = .init(@This()),
        seen_body_id: *BodyID,

        pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ shape2, sub_shape_id_of_shape2 };
            self.seen_body_id.* = self.base.body_id2;
            return true;
        }
    };
    var seen: BodyID = .invalid;
    var filter: RecordingFilter = .{ .seen_body_id = &seen };
    var ts = TransformedShape.init(rvec3(0, 0, 0), Quat.identity(), result.getPtr(), .init(42), .{});
    defer ts.deinit();
    var collector = ClosestHitCollisionCollector(CastRayCollector).init();
    defer collector.deinit();
    ts.castRayCollector(.init(rvec3(-2, 0, 0), Vec3.init(4, 0, 0)), &.{}, &collector.base, .{ .shape_filter = &filter.base });
    try expect(seen.eql(.init(42)));
    try expect(collector.hadHit() and collector.hit.body_id.eql(.init(42)));
}

test "CollisionPrototype: out of memory" {
    const allocator = std.testing.allocator;

    // Shape creation: every allocation failure is returned, nothing leaks, nothing is cached, a retry succeeds
    var compound = StaticCompoundShapeSettings.init(allocator);
    defer compound.deinit();
    try compound.base.addShape(Vec3.init(2, 0, 0), Quat.identity(), (try SphereShapeSettings.create(allocator, 1.0, .{})).asShapeSettings(), .{});
    try compound.base.addShape(Vec3.init(-2, 0, 0), Quat.identity(), (try BoxShapeSettings.create(allocator, Vec3.one(), .{})).asShapeSettings(), .{});
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var result = compound.asShapeSettings().createShape(failing.allocator()) catch |err| {
            try expectEqual(error.OutOfMemory, err);
            try expect(compound.base.base.cached_result.isEmpty());
            // The child settings cached their shapes (allocated with the failing allocator before the failure)
            for (compound.base.sub_shapes.items) |*s| s.shape.get().?.clearCachedResult();
            continue;
        };
        try expect(result.isValid());
        try expect(fail_index >= 3); // Compound, sphere, box, sub shape array
        // The shapes were allocated with this iteration's allocator: release them before it goes out of scope
        result.deinit();
        for (compound.base.sub_shapes.items) |*s| s.shape.get().?.clearCachedResult();
        compound.asShapeSettings().clearCachedResult();
        break;
    }

    // Collectors (D6): addHit stays void, the failure is recorded, the query stops early, checkError reports it
    var sphere = try SphereShape.create(allocator, 1.0, .{});
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var all_hits = AllHitCollisionCollector(CastRayCollector).init(failing.allocator());
    defer all_hits.deinit();
    sphere.asShape().castRayCollector(.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0)), &settings, .{}, &all_hits.base, &.{});
    try expect(all_hits.base.shouldEarlyOut()); // The back face hit was not attempted
    try expect(failing.has_induced_failure);
    try expectError(error.OutOfMemory, all_hits.checkError());
    try expectEqual(@as(usize, 0), all_hits.hits.items.len);
    all_hits.reset(); // Clears the recorded error (it was observed)
    try all_hits.checkError();

    // ClosestHitPerBody: the error survives onBodyEnd (the early out fraction stays forced)
    var failing2 = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var per_body = ClosestHitPerBodyCollisionCollector(CastRayCollector).init(failing2.allocator());
    defer per_body.deinit();
    const body: Body = .{ .id = .init(3) };
    per_body.base.onBody(&body);
    sphere.asShape().castRayCollector(.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0)), &settings, .{}, &per_body.base, &.{});
    per_body.base.onBodyEnd();
    try expect(per_body.base.shouldEarlyOut());
    try expectError(error.OutOfMemory, per_body.checkError());

    // Restore (D11): restoreBinaryState of a compound reads an array and returns the allocation failure
    var result = try compound.asShapeSettings().createShape(allocator);
    defer result.deinit();
    var buffer: [4096]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    result.getPtr().?.saveBinaryState(out.streamOut());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamInWrapper.init(&reader);
    var failing3 = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 1 }); // 0: the shape, 1: its sub shape array
    try expectError(error.OutOfMemory, Shape.restoreFromBinaryState(failing3.allocator(), in.streamIn()));
}

test "CollisionPrototype: ray casts through the vtable, collectors and TransformedShape" {
    const allocator = std.testing.allocator;

    var sphere_settings = SphereShapeSettings.init(allocator, 1.0, .{});
    defer sphere_settings.deinit();
    var sphere_result = try sphere_settings.asShapeSettings().createShape(allocator);
    defer sphere_result.deinit();
    const sphere = sphere_result.getPtr().?;
    var box = try BoxShape.create(allocator, Vec3.one(), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();

    const ray = RayCast.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0));
    for ([_]*const Shape{ sphere, box.asShape() }) |shape| {
        // Closest hit
        var hit: RayCastResult = .{};
        try expect(shape.castRay(ray, .{}, &hit));
        try expectEqual(@as(f32, 0.25), hit.fraction);

        // All hits with back faces, sorted
        var settings: RayCastSettings = .{};
        settings.setBackFaceMode(.collide_with_back_faces);
        var all_hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
        defer all_hits.deinit();
        shape.castRayCollector(ray, &settings, .{}, &all_hits.base, &.{});
        try all_hits.checkError();
        all_hits.sort();
        try expectEqual(@as(usize, 2), all_hits.hits.items.len);
        try expectEqual(@as(f32, 0.25), all_hits.hits.items[0].fraction);
        try expectEqual(@as(f32, 0.75), all_hits.hits.items[1].fraction);

        // Closest hit collector, then reset (virtual) and reuse
        var closest = ClosestHitCollisionCollector(CastRayCollector).init();
        defer closest.deinit();
        shape.castRayCollector(ray, &settings, .{}, &closest.base, &.{});
        try expectEqual(@as(f32, 0.25), closest.hit.fraction);
        closest.base.reset();
        try expect(!closest.hadHit() and closest.base.getEarlyOutFraction() == CastRayCollector.Traits.initial_early_out_fraction);

        // Any hit stops after the first hit
        var any = AnyHitCollisionCollector(CastRayCollector).init();
        defer any.deinit();
        shape.castRayCollector(ray, &settings, .{}, &any.base, &.{});
        try expect(any.hadHit() and any.base.shouldEarlyOut());
    }

    // TransformedShape: world space ray, body ID on the hit and as collector context
    var ts = TransformedShape.init(rvec3(10, 0, 0), Quat.identity(), sphere, .init(5), .{});
    defer ts.deinit();
    var hit: RayCastResult = .{};
    try expect(ts.castRay(.init(rvec3(8, 0, 0), Vec3.init(4, 0, 0)), &hit));
    try expectEqual(@as(f32, 0.25), hit.fraction);
    try expect(hit.body_id.eql(.init(5)));
    var all_hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer all_hits.deinit();
    ts.castRayCollector(.init(rvec3(8, 0, 0), Vec3.init(4, 0, 0)), &.{}, &all_hits.base, .{});
    try expect(all_hits.base.getContext() == &ts);
    try expect(all_hits.hits.items[0].body_id.eql(.init(5)));

    // Collide point through TransformedShape
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    ts.collidePoint(rvec3(10.5, 0, 0), &points.base, .{});
    try expectEqual(@as(usize, 1), points.hits.items.len);
}

test "CollisionPrototype: the dispatch table reproduces Jolt's registration order" {
    const r = &RegisterTypes.registry;
    const dispatch = CollisionDispatch;

    // ConvexShape: convex x convex (registered first)
    try expect(r.getCollideShape(.sphere, .box) == &ConvexShape.collideConvexVsConvex);
    try expect(r.getCollideShape(.box, .sphere) == &ConvexShape.collideConvexVsConvex);
    try expect(r.getCastShape(.box, .user_convex1) == &ConvexShape.castConvexVsConvex);

    // TriangleShape overrides ConvexShape's entries: (convex, triangle), the reversed (triangle, convex) except
    // (triangle, triangle), then the specialized (sphere, triangle)
    try expect(r.getCollideShape(.box, .triangle) == &TriangleShape.collideConvexVsTriangle);
    try expect(r.getCollideShape(.triangle, .box) == &dispatch.reversedCollideShape);
    try expect(r.getCollideShape(.triangle, .triangle) == &TriangleShape.collideConvexVsTriangle);
    try expect(r.getCollideShape(.sphere, .triangle) == &TriangleShape.collideSphereVsTriangle);
    try expect(r.getCastShape(.triangle, .sphere) == &dispatch.reversedCastShape);
    try expect(r.getCastShape(.sphere, .triangle) == &TriangleShape.castSphereVsTriangle);

    // Compounds: CompoundShape registers cast (compound, any), StaticCompoundShape (registered later) overrides
    // (any, static compound); within one loop iteration the second write wins: (SC, SC) = ShapeVsCompound
    try expect(r.getCollideShape(.static_compound, .sphere) == &StaticCompoundShape.collideCompoundVsShape);
    try expect(r.getCollideShape(.sphere, .static_compound) == &StaticCompoundShape.collideShapeVsCompound);
    try expect(r.getCollideShape(.static_compound, .static_compound) == &StaticCompoundShape.collideShapeVsCompound);
    try expect(r.getCastShape(.static_compound, .sphere) == &CompoundShape.castCompoundVsShape);
    try expect(r.getCastShape(.static_compound, .static_compound) == &StaticCompoundShape.castShapeVsCompound);
    try expect(r.getCastShape(.mutable_compound, .box) == &CompoundShape.castCompoundVsShape);

    // ScaledShape is registered after the compounds: it wins every pair with a scaled shape, (scaled, scaled) = ShapeVsScaled
    try expect(r.getCollideShape(.scaled, .static_compound) == &ScaledShape.collideScaledVsShape);
    try expect(r.getCollideShape(.static_compound, .scaled) == &ScaledShape.collideShapeVsScaled);
    try expect(r.getCollideShape(.scaled, .scaled) == &ScaledShape.collideShapeVsScaled);
    try expect(r.getCastShape(.scaled, .scaled) == &ScaledShape.castShapeVsScaled);

    // Pairs nobody registered keep sInit's asserting function
    try expect(r.getCollideShape(.mesh, .mesh) == &dispatch.collideUnsupported);
    try expect(r.getCastShape(.mesh, .sphere) == &dispatch.castUnsupported);

    // ShapeFunctions
    try expect(ShapeFunctions.get(.sphere).construct != null);
    try expect(ShapeFunctions.get(.sphere).color.eql(Color.green));
    try expect(ShapeFunctions.get(.static_compound).color.eql(Color.orange));
    try expect(ShapeFunctions.get(.scaled).color.eql(Color.yellow));
    try expect(ShapeFunctions.get(.mesh).construct == null);

    // User registrations (the zolt_user_types module in the library) run after Jolt's order and override it
    const UserShapes = struct {
        fn collideUser(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
            _ = .{ shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter };
        }
        pub fn register(comptime reg: *CollisionDispatch.Registry) void {
            reg.shapeFunctions(.user1).color = Color.purple;
            for (ShapeFile.convex_sub_shape_types) |s|
                reg.registerCollideShape(.user1, s, collideUser);
        }
    };
    const user_registry: CollisionDispatch.Registry = comptime .build(RegisterTypes.registration_order ++ .{UserShapes});
    try expect(user_registry.getCollideShape(.user1, .sphere) == &UserShapes.collideUser);
    try expect(user_registry.getCollideShape(.sphere, .box) == &ConvexShape.collideConvexVsConvex);
    try expect(user_registry.shape_functions[@intFromEnum(ShapeSubType.user1)].color.eql(Color.purple));
}

test "CollisionPrototype: collide and cast through the dispatch table, reversed entries" {
    const allocator = std.testing.allocator;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var triangle = TriangleShape.init(allocator, Vec3.init(-2, 0, -2), Vec3.init(0, 0, 2), Vec3.init(2, 0, -2), .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    // Sphere vs box: GJK / EPA on the placement constructed Support objects
    var collide_settings: CollideShapeSettings = .{};
    collide_settings.collect_faces_mode = .collect_faces;
    var closest = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer closest.deinit();
    CollisionDispatch.collideShapeVsShape(sphere.asShape(), box.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(1.5, 0, 0)), .{}, .{}, &collide_settings, &closest.base, &.{});
    try expect(closest.hadHit());
    try expect(@abs(closest.hit.penetration_depth - 0.5) < 1.0e-3);
    try expect(closest.hit.contact_point_on1.isClose(Vec3.init(1, 0, 0), .{ .max_dist_sq = 1.0e-4 }));
    try expectEqual(@as(u32, 4), closest.hit.shape2_face.len); // Box face

    // Triangle vs sphere goes through reversedCollideShape: the result is sphere vs triangle reversed, bit for bit
    const t_sphere = Mat44.translation(Vec3.init(0.1, 0.5, 0.2));
    var forward = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer forward.deinit();
    CollisionDispatch.collideShapeVsShape(sphere.asShape(), triangle.asShape(), Vec3.one(), Vec3.one(), t_sphere, Mat44.identity(), .{}, .{}, &.{}, &forward.base, &.{});
    var reversed = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer reversed.deinit();
    CollisionDispatch.collideShapeVsShape(triangle.asShape(), sphere.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), t_sphere, .{}, .{}, &.{}, &reversed.base, &.{});
    try expect(forward.hadHit() and reversed.hadHit());
    try expect(reversed.hit.contact_point_on1.eql(forward.hit.contact_point_on2));
    try expect(reversed.hit.contact_point_on2.eql(forward.hit.contact_point_on1));
    try expect(reversed.hit.penetration_axis.eql(forward.hit.penetration_axis.negate()));
    try expectEqual(forward.hit.penetration_depth, reversed.hit.penetration_depth);

    // Shape cast: sphere moving into the box (world space cast)
    const cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(-4, 0, 0)), Vec3.init(4, 0, 0));
    var cast_hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer cast_hits.deinit();
    CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &.{}, box.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &cast_hits.base);
    try cast_hits.checkError();
    try expectEqual(@as(usize, 1), cast_hits.hits.items.len);
    try expect(@abs(cast_hits.hits.items[0].fraction - 0.5) < 1.0e-3);
    const as_collide_result: *const CollideShapeResult = &cast_hits.hits.items[0].base; // ShapeCastResult passed as its base (D12)
    try expect(as_collide_result.contact_point_on2.isClose(Vec3.init(-1, 0, 0), .{ .max_dist_sq = 1.0e-4 }));

    // Casting a triangle against a sphere goes through reversedCastShape
    const triangle_cast = ShapeCast.init(triangle.asShape(), Vec3.one(), Mat44.translation(Vec3.init(0, 3, 0)), Vec3.init(0, -4, 0));
    var reversed_cast = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer reversed_cast.deinit();
    CollisionDispatch.castShapeVsShapeWorldSpace(&triangle_cast, &.{}, sphere.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &reversed_cast.base);
    try expect(reversed_cast.hadHit());
    try expect(@abs(reversed_cast.hit.fraction - 0.5) < 1.0e-3);
}

test "CollisionPrototype: compound and scaled shapes own their children" {
    const allocator = std.testing.allocator;

    const material = try PhysicsMaterialSimple.create(allocator, "Rubber", Color.red);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();

    // Children from settings (heap, released by the compound settings) and from a concrete shape
    const box = try BoxShape.create(allocator, Vec3.one(), .{ .material = material.material() });
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShape(Vec3.init(2, 0, 0), Quat.identity(), (try SphereShapeSettings.create(allocator, 1.0, .{})).asShapeSettings(), .{ .user_data = 7 });
    try compound_settings.base.addShapePtr(Vec3.init(-2, 0, 0), Quat.identity(), box.asShape(), .{});
    var result = try compound_settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const compound_shape = result.getPtr().?;
    const compound = compound_shape.cast(CompoundShape);
    try expectEqual(@as(u32, 2), compound.getNumSubShapes());
    try expectEqual(@as(u32, 1), compound.getSubShapeIDBits());
    try expectEqual(@as(u32, 3), box.asShape().ref_count.get()); // box_ref, compound settings, compound
    try expectEqual(@as(u32, 7), compound.getCompoundUserData(0));

    // Ray along +x hits the box (sub shape 1) first; sub shape IDs lead to the leaf shape and its material
    var hit: RayCastResult = .{};
    const com = compound_shape.getCenterOfMass();
    try expect(compound_shape.castRay(.init(Vec3.init(-10, 0, 0).sub(com), Vec3.init(20, 0, 0)), .{}, &hit));
    const leaf = compound_shape.getLeafShape(hit.sub_shape_id2);
    try expect(leaf.shape == box.asShape() and leaf.remainder.isEmpty());
    try expect(compound_shape.getMaterial(hit.sub_shape_id2) == material.material());
    try expect(compound.getSubShapeIndexFromID(hit.sub_shape_id2).index == 1);

    // Compound vs sphere through the dispatch table (the sphere touches the box child only)
    var sphere = try SphereShape.create(allocator, 0.5, .{});
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    var collide_hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collide_hits.deinit();
    CollisionDispatch.collideShapeVsShape(compound_shape, sphere.asShape(), Vec3.one(), Vec3.one(), Mat44.translation(com), Mat44.translation(Vec3.init(-1.25, 0, 0)), .{}, .{}, &.{}, &collide_hits.base, &.{});
    try collide_hits.checkError();
    try expectEqual(@as(usize, 1), collide_hits.hits.items.len);
    try expect(compound.getSubShapeIndexFromID(collide_hits.hits.items[0].sub_shape_id1).index == 1);

    // Scaled shape: decorated shapes forward to their child with the scale applied
    var scaled = try ScaledShape.create(allocator, sphere.asShape(), Vec3.replicate(2.0));
    var scaled_ref = RefConst(Shape).init(scaled.asShape());
    defer scaled_ref.deinit();
    var scaled_hit: RayCastResult = .{};
    try expect(scaled.asShape().castRay(.init(Vec3.init(-2, 0, 0), Vec3.init(4, 0, 0)), .{}, &scaled_hit));
    try expectEqual(@as(f32, 0.25), scaled_hit.fraction);
    try expect(scaled.asShape().getMaterial(.empty) == PhysicsMaterial.default); // DecoratedShape.impl.getMaterial -> sphere
    try expectEqual(@as(f32, 1.0), scaled.asShape().getInnerRadius());

    // CollectTransformedShapes: every stored TransformedShape holds a reference (cloned by the collector)
    var ts = TransformedShape.init(rvec3(100, 0, 0), Quat.identity(), compound_shape, .init(9), .{});
    defer ts.deinit();
    {
        var leaves = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer leaves.deinit();
        ts.collectTransformedShapes(.init(Vec3.replicate(-1000), Vec3.replicate(1000)), &leaves.base, .{});
        try leaves.checkError();
        try expectEqual(@as(usize, 2), leaves.hits.items.len);
        try expect(leaves.hits.items[1].shape.get() == box.asShape());
        try expectEqual(@as(u32, 4), box.asShape().ref_count.get());
        try expect(leaves.hits.items[1].body_id.eql(.init(9)));
    }
    try expectEqual(@as(u32, 3), box.asShape().ref_count.get());

    // scaleShape: a valid scale wraps the shape, an invalid one (non uniform on a rotated child) builds a new compound
    var uniform = try compound_shape.scaleShape(allocator, Vec3.replicate(2.0));
    defer uniform.deinit();
    try expectEqual(ShapeSubType.scaled, uniform.getPtr().?.getSubType());
    var zero = try compound_shape.scaleShape(allocator, Vec3.zero());
    defer zero.deinit();
    try expectEqualStrings("Can't use zero scale!", zero.getError());

    var rotated_settings = StaticCompoundShapeSettings.init(allocator);
    defer rotated_settings.deinit();
    try rotated_settings.base.addShapePtr(Vec3.init(2, 0, 0), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), box.asShape(), .{});
    try rotated_settings.base.addShapePtr(Vec3.init(-2, 0, 0), Quat.identity(), sphere.asShape(), .{});
    var rotated = try rotated_settings.asShapeSettings().createShape(allocator);
    defer rotated.deinit();
    try expect(!rotated.getPtr().?.isValidScale(Vec3.init(1, 2, 3)));
    var rescaled = try rotated.getPtr().?.scaleShape(allocator, Vec3.init(1, 2, 3));
    defer rescaled.deinit();
    try expectEqual(ShapeSubType.static_compound, rescaled.getPtr().?.getSubType());
    try expectEqual(@as(u32, 2), rescaled.getPtr().?.cast(CompoundShape).getNumSubShapes());
}

test "CollisionPrototype: shape filter" {
    const allocator = std.testing.allocator;

    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShape(Vec3.init(2, 0, 0), Quat.identity(), (try SphereShapeSettings.create(allocator, 1.0, .{})).asShapeSettings(), .{});
    try compound_settings.base.addShape(Vec3.init(-2, 0, 0), Quat.identity(), (try BoxShapeSettings.create(allocator, Vec3.one(), .{})).asShapeSettings(), .{});
    var result = try compound_settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const compound = result.getPtr().?.cast(CompoundShape);

    // Reject the sphere (sub shape 0) of body 3. Note: like in Jolt, the ID of the last sub shape of a compound with
    // 2 sub shapes is all ones, i.e. equal to the empty ID that the compound itself is tested with.
    try expect(compound.getSubShapeIDFromIndex(1, .{}).getID().eql(.empty));
    const RejectFilter = struct {
        pub const overrides = .{.shouldCollide};
        base: ShapeFilter = .init(@This()),
        rejected: SubShapeID,

        pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = shape2;
            return !(self.base.body_id2.eql(.init(3)) and sub_shape_id_of_shape2.eql(self.rejected));
        }
    };
    var filter: RejectFilter = .{ .rejected = compound.getSubShapeIDFromIndex(0, .{}).getID() };
    var ts = TransformedShape.init(rvec3(0, 0, 0), Quat.identity(), result.getPtr(), .init(3), .{});
    defer ts.deinit();
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    ts.castRayCollector(.init(rvec3(-10, 0, 0), Vec3.init(20, 0, 0)), &settings, &hits.base, .{ .shape_filter = &filter.base });
    try hits.checkError();
    try expectEqual(@as(usize, 2), hits.hits.items.len); // Front and back face of the box only
    for (hits.hits.items) |h|
        try expectEqual(@as(u32, 1), compound.getSubShapeIndexFromID(h.sub_shape_id2).index);

    // The same filter on another body accepts everything
    var ts2 = TransformedShape.init(rvec3(0, 0, 0), Quat.identity(), result.getPtr(), .init(4), .{});
    defer ts2.deinit();
    var hits2 = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits2.deinit();
    ts2.castRayCollector(.init(rvec3(-10, 0, 0), Vec3.init(20, 0, 0)), &settings, &hits2.base, .{ .shape_filter = &filter.base });
    try hits2.checkError();
    try expectEqual(@as(usize, 4), hits2.hits.items.len);
}

test "CollisionPrototype: default material and material references" {
    const allocator = std.testing.allocator;

    // The default material is a compile time constant: reference counting it never writes to read-only memory
    const default = PhysicsMaterial.default;
    try expect(default.is_static);
    try expectEqualStrings("Default", default.getDebugName());
    try expect(default.getDebugColor().eql(Color.grey));
    var default_ref = RefConst(PhysicsMaterial).init(default);
    default_ref.deinit();
    try expectEqual(@as(u32, 0), default.ref_count.get());

    // A heap material owns a copy of its name
    const name = try allocator.dupe(u8, "Ice");
    const ice = try PhysicsMaterialSimple.create(allocator, name, Color.cyan);
    allocator.free(name);
    var sphere_settings = SphereShapeSettings.init(allocator, 1.0, .{ .material = ice.material() });
    defer sphere_settings.deinit();
    var result = try sphere_settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    try expect(result.getPtr().?.getMaterial(.empty) == ice.material());
    try expectEqualStrings("Ice", result.getPtr().?.getMaterial(.empty).getDebugName());
    try expectEqual(@as(u32, 2), ice.base.ref_count.get()); // settings + shape

    // Material binary state: Jolt's RTTI hash, then the fields; restored through the comptime material type list
    var buffer: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    ice.material().saveBinaryState(out.streamOut());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamInWrapper.init(&reader);
    var restored = try PhysicsMaterial.restoreFromBinaryState(allocator, in.streamIn());
    defer restored.deinit();
    try expectEqualStrings("Ice", restored.getPtr().?.getDebugName());
    try expect(restored.getPtr().?.getDebugColor().eql(Color.cyan));
    try expectEqual(PhysicsMaterial.rttiHash("PhysicsMaterialSimple"), restored.getPtr().?.getRTTIHash());
}

test "CollisionPrototype: GetTrianglesContext placement and comptime vertex tables" {
    const allocator = std.testing.allocator;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var triangle = TriangleShape.init(allocator, Vec3.init(-1, 0, -1), Vec3.init(0, 0, 1), Vec3.init(1, 0, -1), .{ .convex_radius = 0.1 });
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    const max = Shape.get_triangles_min_triangles_requested;
    var vertices: [3 * max]Float3 = undefined;
    var materials: [max]*const PhysicsMaterial = undefined;
    // Box and sphere: GetTrianglesContextVertexList, triangle: ConvexShape's context (a SupportBuffer and a pointer into it)
    const cases = [_]struct { shape: *const Shape, triangles: u32 }{
        .{ .shape = box.asShape(), .triangles = 12 },
        .{ .shape = sphere.asShape(), .triangles = 128 },
        .{ .shape = triangle.asShape(), .triangles = 128 },
    };
    for (cases) |c| {
        var context: Shape.GetTrianglesContext = .{};
        c.shape.getTrianglesStart(&context, AABox.biggest(), Vec3.init(1, 0, 0), Quat.identity(), Vec3.one());
        var total: u32 = 0;
        while (true) {
            const n = c.shape.getTrianglesNext(&context, max, &vertices, &materials);
            if (n == 0) break;
            for (materials[0..n]) |m| try expect(m == PhysicsMaterial.default);
            total += n;
        }
        try expectEqual(c.triangles, total);
    }

    // The unit sphere that Jolt builds in a static initializer is a comptime constant: same bits as at runtime
    var runtime_sphere: StaticArray(Vec3, 384) = .empty;
    var level: u32 = 2;
    _ = &level;
    try GetTrianglesContextVertexList.createHalfUnitSphereTop(&runtime_sphere, level);
    try GetTrianglesContextVertexList.createHalfUnitSphereBottom(&runtime_sphere, level);
    try expectEqual(runtime_sphere.len, ConvexShape.unit_sphere_triangles.len);
    try expect(std.mem.eql(u8, std.mem.sliceAsBytes(runtime_sphere.constSlice()), std.mem.sliceAsBytes(ConvexShape.unit_sphere_triangles.constSlice())));
}

test "CollisionPrototype: binary state" {
    const allocator = std.testing.allocator;
    var buffer: [4096]u8 = undefined;

    // Save / restore through ShapeFunctions.construct of the comptime registry
    var sphere = try SphereShape.create(allocator, 2.5, .{});
    sphere.base.setDensity(500.0);
    sphere.asShapeMut().setUserData(42);
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    sphere.asShape().saveBinaryState(out.streamOut());
    const saved = writer.buffered();
    {
        var reader: std.Io.Reader = .fixed(saved);
        var in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        const restored = result.getPtr().?.cast(SphereShape);
        try expectEqual(@as(f32, 2.5), restored.getRadius());
        try expectEqual(@as(f32, 500.0), restored.base.getDensity());
        try expectEqual(@as(u64, 42), restored.asShape().getUserData());
    }

    // Jolt's error texts; an invalid sub type is an error instead of undefined behavior
    {
        var reader: std.Io.Reader = .fixed(saved[0 .. saved.len - 1]);
        var in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        try expectEqualStrings("Failed to restore shape", result.getError());
    }
    for ([_]u8{ 200, @intFromEnum(ShapeSubType.mesh) }) |bad_sub_type| {
        var reader: std.Io.Reader = .fixed(&.{bad_sub_type});
        var in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, in.streamIn());
        defer result.deinit();
        try expectEqualStrings("Failed to read type id", result.getError());
    }

    // SaveWithChildren / RestoreWithChildren: shared children are written once, materials are restored
    const material = try PhysicsMaterialSimple.create(allocator, "Wood", Color.orange);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    const box = try BoxShape.create(allocator, Vec3.one(), .{ .material = material.material() });
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShapePtr(Vec3.init(2, 0, 0), Quat.identity(), box.asShape(), .{});
    try compound_settings.base.addShapePtr(Vec3.init(-2, 0, 0), Quat.identity(), box.asShape(), .{});
    var compound = try compound_settings.asShapeSettings().createShape(allocator);
    defer compound.deinit();

    var shape_map: ShapeFile.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: ShapeFile.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    var writer2: std.Io.Writer = .fixed(&buffer);
    var out2 = StreamOutWrapper.init(&writer2);
    try compound.getPtr().?.saveWithChildren(allocator, out2.streamOut(), &shape_map, &material_map);
    try expectEqual(@as(u32, 2), shape_map.count()); // compound + the shared box
    try expectEqual(@as(u32, 1), material_map.count());

    var reader2: std.Io.Reader = .fixed(writer2.buffered());
    var in2 = StreamInWrapper.init(&reader2);
    var id_to_shape: ShapeFile.IDToShapeMap = .empty;
    defer {
        for (id_to_shape.items) |*s| s.deinit();
        id_to_shape.deinit(allocator);
    }
    var id_to_material: ShapeFile.IDToMaterialMap = .empty;
    defer {
        for (id_to_material.items) |*m| m.deinit();
        id_to_material.deinit(allocator);
    }
    var restored = try Shape.restoreWithChildren(allocator, in2.streamIn(), &id_to_shape, &id_to_material);
    defer restored.deinit();
    const restored_compound = restored.getPtr().?.cast(CompoundShape);
    try expectEqual(@as(u32, 2), restored_compound.getNumSubShapes());
    const child0 = restored_compound.getSubShape(0).shape.get().?;
    try expect(child0 == restored_compound.getSubShape(1).shape.get().?);
    try expectEqualStrings("Wood", child0.getMaterial(.empty).getDebugName());
    try expect(restored_compound.base.getCenterOfMass().eql(compound.getPtr().?.getCenterOfMass()));
}

test "CollisionPrototype: embedded shapes and settings" {
    const allocator = std.testing.allocator;

    // Settings on the stack that are referenced by heap settings must be embedded
    var child = SphereShapeSettings.init(allocator, 1.0, .{});
    child.asShapeSettings().setEmbedded();
    defer child.deinit();
    {
        var scaled = try ScaledShapeSettings.create(allocator, child.asShapeSettings(), Vec3.replicate(3.0));
        var scaled_ref = Ref(ShapeSettings).init(scaled.asShapeSettings());
        defer scaled_ref.deinit();
        var result = try scaled_ref.get().?.createShape(allocator);
        defer result.deinit();
        try expectEqual(@as(f32, 3.0), result.getPtr().?.getInnerRadius());
    }

    // A shape on the stack: references can be taken, release never frees it, deinit checks that none are left
    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    var ref = RefConst(Shape).init(sphere.asShape());
    try expectEqual(@as(u32, 1 + zolt.RefCount.embedded), sphere.asShape().ref_count.get());
    ref.deinit();
    sphere.asShapeMut().deinit();
}

test "CollisionPrototype: concurrent queries on a shared shape" {
    const allocator = std.testing.allocator;

    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShape(Vec3.init(2, 0, 0), Quat.rotation(Vec3.axisY(), 0.3), (try BoxShapeSettings.create(allocator, Vec3.init(1, 2, 0.5), .{})).asShapeSettings(), .{});
    try compound_settings.base.addShape(Vec3.init(-2, 1, 0), Quat.identity(), (try SphereShapeSettings.create(allocator, 1.5, .{})).asShapeSettings(), .{});
    var result = try compound_settings.asShapeSettings().createShape(allocator);
    defer result.deinit();

    const Worker = struct {
        fn run(shape: *const Shape, out_sum: *f32) void {
            var ts = TransformedShape.init(rvec3(0, 0, 0), Quat.identity(), shape, .init(1), .{});
            defer ts.deinit();
            var sum: f32 = 0;
            var i: u32 = 0;
            while (i < 500) : (i += 1) {
                const y = @as(f32, @floatFromInt(i % 50)) * 0.1 - 2.5;
                var hit: RayCastResult = .{};
                if (ts.castRay(.init(rvec3(-10, y, 0), Vec3.init(20, 0.1, 0)), &hit))
                    sum += hit.fraction;
            }
            out_sum.* = sum;
        }
    };

    var expected: f32 = 0;
    Worker.run(result.getPtr().?, &expected);
    try expect(expected > 0);
    var sums: [4]f32 = @splat(0);
    var threads: [4]std.Thread = undefined;
    for (&threads, &sums) |*t, *s| t.* = try std.Thread.spawn(.{}, Worker.run, .{ result.getPtr().?, s });
    for (threads) |t| t.join();
    for (sums) |s| try expectEqual(expected, s);
}
