//! Zolt addition, no Jolt file: tests of the bounds cache of CharacterVirtual (Zolt/Physics/Character/
//! CharacterBoundsCache.zig, Roadmap Phase 12, "Character bounds caches"). Jolt has no such cache: CharacterVsCharacterCollisionSimple
//! computes the center of mass transform and the world bounds of every other character for every query.
//! Status: complete
//!
//! - T1: the replayed bounds equal Shape.getWorldSpaceBounds as real numbers, and neither has a NaN, for every replayed
//!   shape kind and nesting (sphere, box, capsule, tapered capsule, cylinder, tapered cylinder, convex hull, and
//!   RotatedTranslated / Scaled / OffsetCenterOfMass chains of up to 4 levels), identity, axis aligned and random
//!   rotations, translations from +-0 through 1e-30, 1, 5e3, 1.2e4, 1e7 to 3e38, and infinite and NaN ones. A replay
//!   returns null exactly when a translation of its chain is not finite. In double precision the characters sit around
//!   1e9 with a base offset of 0 and one near the position (the translation of the loop's transform). The unsupported
//!   chains (too deep, too many offsets, compounds, a triangle, an invalid scale) build no replay.
//! - T2: the reject decisions of the loops (ExpandBy + Overlaps, ExpandBy + RayAABoxHits) on the replayed box equal the
//!   decisions on Jolt's box at the translation where they flip and 1 to 3 ulp around it on each axis and side, with
//!   separations / paddings 0, a denormal, tiny, 0.02, huge, inf and NaN, and cast directions with zero, -0, tiny,
//!   huge and NaN components. Fact 5 of the proof directly: the decisions on two boxes that differ only in the signs of
//!   zeros are equal, with query boxes, separations, origins, directions (also infinite and NaN), extents and paddings
//!   full of zeros of both signs.
//! - T3: end to end, the hits of CharacterVsCharacterCollisionSimple equal those of an oracle that is Jolt's loop copied
//!   into this file, bit for bit (user data order, contact points, axes, depths, faces, fractions), and so do the
//!   accept decisions (every user data the loop sets, also for pairs without a hit), on random crowds of supported and
//!   unsupported shapes, near the origin and far from it, for CollideCharacter and CastCharacter. The fallback for a
//!   translation that is not finite (`non_finite`: T infinite, or the OffsetCenterOfMass addition overflowing) runs
//!   Jolt's GetWorldSpaceBounds and gives the oracle's decisions.
//! - T4: staleness. Direct writes to position, rotation, shape offset and character padding, CharacterBase.setUp and
//!   base.shape.set make the cache stale (the stale counter rises, the hits equal the oracle), the refresh points make
//!   it valid again. An attempted address reuse: the shapes come from a FixedBufferAllocator, which hands out the
//!   address of a freed last block again. The pin keeps the old shape alive, so a new shape cannot get its address.
//! - T5: a MutableCompoundShape changed in place is counted unsupported and gives the oracle's hits.
//! - Bite proofs (each one makes a test of this file fail; tried and reverted):
//!   - `up` dropped from the key: T4 setUp.
//!   - The replayed tail of AABox::Transformed re-associated, as `T + ((v0 + v1) + v2)` and as `(T + (v0 + v1)) + v2`:
//!     each fails both T1 replay tests and the T2 flip test.
//!   - The finite guard of `Replay.bounds` removed: T1 (the NaN pattern of the infinite translations, and the
//!     overflowing OffsetCenterOfMass + RotatedTranslated chain) and the T3 non_finite test (its counter).
//!   - No pin, the key keeping the shape address without a reference (a raw `?*const Shape` in place of `pin`): T4
//!     address reuse fails at the queries after the reuse, where the big box has the small box's address (the other
//!     character rejects it: its decisions and its hits differ from the oracle's). Removing only `pin.set` is a
//!     different bite: the stored shape is then null, every read is stale, and T4 fails earlier, at `Find.valid`.
//!   - `AABox.overlaps`, or the slab test of `rayAABoxHitsDirection`, ordering -0 below +0: the T2 fact 5 test fails,
//!     the T2 flip test passes (its boxes never differ in the sign of a zero).
//!   - The non_finite fallback running GetWorldSpaceBounds on the cached transform without the base offset (the
//!     decisions of the collide differ), or not counting `non_finite`: the T3 non_finite test.

const std = @import("std");
const zolt = @import("zolt");
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const Layers = @import("../Layers.zig").Layers;

const AABox = zolt.AABox;
const BoxShapeSettings = zolt.BoxShapeSettings;
const BoxShape = zolt.BoxShape;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const CastShapeCollector = zolt.CastShapeCollector;
const CharacterBoundsCache = zolt.CharacterBoundsCache;
const CharacterVirtual = zolt.CharacterVirtual;
const CharacterVirtualSettings = zolt.CharacterVirtualSettings;
const CharacterVsCharacterCollisionSimple = zolt.CharacterVsCharacterCollisionSimple;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const Core = zolt.Core;
const CylinderShapeSettings = zolt.CylinderShapeSettings;
const Mat44 = zolt.Mat44;
const Mt19937 = zolt.Mt19937;
const MutableCompoundShape = zolt.MutableCompoundShape;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const Quat = zolt.Quat;
const RMat44 = zolt.RMat44;
const RVec3 = zolt.RVec3;
const Real = zolt.Real;
const RefConst = zolt.RefConst;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeResult = zolt.ShapeResult;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const TaperedCapsuleShapeSettings = zolt.TaperedCapsuleShapeSettings;
const TaperedCylinderShapeSettings = zolt.TaperedCylinderShapeSettings;
const TriangleShapeSettings = zolt.TriangleShapeSettings;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const rayAABoxHitsDirection = zolt.rayAABoxHitsDirection;
const math = zolt.math;

const Allocator = std.mem.Allocator;
const allocator = std.testing.allocator;
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const Replay = CharacterBoundsCache.Replay;
const Stats = CharacterBoundsCache.Stats;

const inf = std.math.inf(f32);
const nan = std.math.nan(f32);

// ---------------------------------------------------------------------------------------------------------------------
// Shapes

/// The shapes of a test, one reference each
const Shapes = struct {
    list: std.ArrayList(RefConst(Shape)) = .empty,

    fn deinit(self: *Shapes) void {
        for (self.list.items) |*s| s.deinit();
        self.list.deinit(allocator);
    }

    /// Keep the shape of a creation result
    fn add(self: *Shapes, result_in: ShapeResult) !*const Shape {
        var result = result_in;
        defer result.deinit();
        if (result.hasError()) return error.TestUnexpectedResult;
        const shape = result.getPtr().?;
        try self.list.append(allocator, .init(shape));
        return shape;
    }

    fn sphere(self: *Shapes, radius: f32) !*const Shape {
        var s = SphereShapeSettings.init(allocator, radius, .{});
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn box(self: *Shapes, half_extent: Vec3, convex_radius: f32) !*const Shape {
        var s = BoxShapeSettings.init(allocator, half_extent, .{ .convex_radius = convex_radius });
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn capsule(self: *Shapes, half_height: f32, radius: f32) !*const Shape {
        var s = CapsuleShapeSettings.init(allocator, half_height, radius, .{});
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn taperedCapsule(self: *Shapes, half_height: f32, top_radius: f32, bottom_radius: f32) !*const Shape {
        var s = TaperedCapsuleShapeSettings.init(allocator, half_height, top_radius, bottom_radius, .{});
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn cylinder(self: *Shapes, half_height: f32, radius: f32, convex_radius: f32) !*const Shape {
        var s = CylinderShapeSettings.init(allocator, half_height, radius, .{ .convex_radius = convex_radius });
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn taperedCylinder(self: *Shapes, half_height: f32, top_radius: f32, bottom_radius: f32, convex_radius: f32) !*const Shape {
        var s = TaperedCylinderShapeSettings.init(allocator, half_height, top_radius, bottom_radius, .{ .convex_radius = convex_radius });
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn convexHull(self: *Shapes, points: []const Vec3) !*const Shape {
        var s = try ConvexHullShapeSettings.init(allocator, points, .{});
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn triangle(self: *Shapes) !*const Shape {
        var s = TriangleShapeSettings.init(allocator, Vec3.init(-0.5, 0, 0), Vec3.init(0.5, 0, 0), Vec3.init(0, 1, 0.2), .{});
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn rotatedTranslated(self: *Shapes, inner: *const Shape, position: Vec3, rotation: Quat) !*const Shape {
        var s = RotatedTranslatedShapeSettings.initPtr(allocator, position, rotation, inner);
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn scaled(self: *Shapes, inner: *const Shape, scale: Vec3) !*const Shape {
        var s = ScaledShapeSettings.initPtr(allocator, inner, scale);
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn offsetCenterOfMass(self: *Shapes, inner: *const Shape, offset: Vec3) !*const Shape {
        var s = OffsetCenterOfMassShapeSettings.initPtr(allocator, offset, inner);
        defer s.deinit();
        return self.add(try s.createShape(allocator));
    }

    fn staticCompound(self: *Shapes, a: *const Shape, b: *const Shape) !*const Shape {
        var s = StaticCompoundShapeSettings.init(allocator);
        defer s.deinit();
        try s.base.addShapePtr(Vec3.init(0, 0.5, 0), Quat.identity(), a, .{});
        try s.base.addShapePtr(Vec3.init(0.2, 1.4, 0), Quat.rotation(Vec3.axisY(), 0.7), b, .{});
        return self.add(try s.createShape(allocator));
    }

    /// A MutableCompoundShape (the caller changes it in place)
    fn mutableCompound(self: *Shapes, a: *const Shape, b: *const Shape) !*MutableCompoundShape {
        const compound = try MutableCompoundShape.create(allocator);
        try self.list.append(allocator, .init(compound.asShape()));
        _ = try compound.addShape(Vec3.init(0, 0.6, 0), Quat.identity(), a, .{});
        _ = try compound.addShape(Vec3.init(-0.3, 1.2, 0.1), Quat.rotation(Vec3.axisZ(), 0.4), b, .{});
        return compound;
    }
};

/// A random rotation, translation, ...
const Random = struct {
    rng: Mt19937,

    fn init(seed: u32) Random {
        return .{ .rng = .init(seed) };
    }

    /// A float in [lo, hi)
    fn float(self: *Random, lo: f32, hi: f32) f32 {
        const u = @as(f32, @floatFromInt(self.rng.next() >> 8)) / (1 << 24);
        return lo + (hi - lo) * u;
    }

    fn vec3(self: *Random, lo: f32, hi: f32) Vec3 {
        return Vec3.init(self.float(lo, hi), self.float(lo, hi), self.float(lo, hi));
    }

    fn index(self: *Random, n: usize) usize {
        return self.rng.next() % n;
    }

    fn rotation(self: *Random) Quat {
        var axis = self.vec3(-1, 1);
        if (axis.lengthSq() < 1.0e-4) axis = Vec3.axisX();
        return Quat.rotation(axis.normalized(), self.float(-3.1, 3.1));
    }
};

/// The supported shapes of T1 / T2 / T3: every replayed kind and nestings of up to 4 decorators
fn supportedShapes(shapes: *Shapes, random: *Random) !std.ArrayList(*const Shape) {
    var list: std.ArrayList(*const Shape) = .empty;
    errdefer list.deinit(allocator);
    const sphere = try shapes.sphere(0.4);
    const box = try shapes.box(Vec3.init(0.3, 0.6, 0.2), 0.05);
    const capsule = try shapes.capsule(0.5, 0.3);
    const tapered_capsule = try shapes.taperedCapsule(0.5, 0.2, 0.35);
    const cylinder = try shapes.cylinder(0.6, 0.3, 0.05);
    const tapered_cylinder = try shapes.taperedCylinder(0.5, 0.2, 0.35, 0.05);
    var points: [12]Vec3 = undefined;
    for (&points) |*p| p.* = random.vec3(-0.5, 0.5).mul(Vec3.init(1, 2, 0.8));
    const hull = try shapes.convexHull(&points);
    const leaves = [_]*const Shape{ sphere, box, capsule, tapered_capsule, cylinder, tapered_cylinder, hull };
    try list.appendSlice(allocator, &leaves);
    for (leaves) |leaf| {
        // Every leaf behind each decorator, the perf scene's character (RotatedTranslated capsule with identity rotation)
        try list.append(allocator, try shapes.rotatedTranslated(leaf, Vec3.init(0, 0.8, 0), Quat.identity()));
        try list.append(allocator, try shapes.rotatedTranslated(leaf, Vec3.init(0.1, 0.5, -0.2), random.rotation()));
        try list.append(allocator, try shapes.scaled(leaf, Vec3.replicate(1.2)));
        try list.append(allocator, try shapes.scaled(leaf, Vec3.replicate(-0.8)));
        try list.append(allocator, try shapes.offsetCenterOfMass(leaf, Vec3.init(0.1, 0.3, -0.2)));
    }
    // Non uniform scales (box, cylinder and hull accept them; the tapered capsule a uniform one with a flipped y)
    try list.append(allocator, try shapes.scaled(box, Vec3.init(1.2, 1.4, 0.8)));
    try list.append(allocator, try shapes.scaled(try shapes.rotatedTranslated(box, Vec3.init(0, 0.5, 0), random.rotation()), Vec3.init(1.2, 1.4, 0.8)));
    try list.append(allocator, try shapes.scaled(try shapes.scaled(hull, Vec3.init(1, 2, 0.5)), Vec3.replicate(0.5)));
    try list.append(allocator, try shapes.scaled(tapered_capsule, Vec3.init(1.5, -1.5, 1.5)));
    // Nestings
    try list.append(allocator, try shapes.scaled(try shapes.rotatedTranslated(capsule, Vec3.init(0, 0.75, 0), Quat.identity()), Vec3.replicate(1.2)));
    try list.append(allocator, try shapes.rotatedTranslated(try shapes.scaled(box, Vec3.init(1, 0.5, 2)), Vec3.init(0.2, 0, 0), random.rotation()));
    try list.append(allocator, try shapes.offsetCenterOfMass(try shapes.rotatedTranslated(tapered_capsule, Vec3.init(0, 0.4, 0), random.rotation()), Vec3.init(0.3, -0.1, 0.2)));
    try list.append(allocator, try shapes.rotatedTranslated(try shapes.offsetCenterOfMass(sphere, Vec3.init(0, 0.5, 0)), Vec3.init(0.2, 0, 0), random.rotation()));
    try list.append(allocator, try shapes.rotatedTranslated(try shapes.rotatedTranslated(cylinder, Vec3.init(0, 0.2, 0.1), random.rotation()), Vec3.init(-0.1, 0.3, 0), random.rotation()));
    try list.append(allocator, try shapes.offsetCenterOfMass(try shapes.offsetCenterOfMass(box, Vec3.init(0.5, 0, 0)), Vec3.init(0, -0.25, 0.75)));
    try list.append(allocator, try shapes.rotatedTranslated(try shapes.scaled(try shapes.offsetCenterOfMass(try shapes.rotatedTranslated(capsule, Vec3.init(0, 0.6, 0), random.rotation()), Vec3.init(0.1, 0.2, 0.3)), Vec3.replicate(-1.1)), Vec3.init(0, 0.1, 0), random.rotation()));
    return list;
}

/// The unsupported shapes: they never build a replay
fn unsupportedShapes(shapes: *Shapes, random: *Random) !std.ArrayList(*const Shape) {
    var list: std.ArrayList(*const Shape) = .empty;
    errdefer list.deinit(allocator);
    const sphere = try shapes.sphere(0.4);
    const capsule = try shapes.capsule(0.5, 0.3);
    // 5 decorators
    var deep = sphere;
    for (0..5) |_| deep = try shapes.rotatedTranslated(deep, Vec3.init(0, 0.1, 0), random.rotation());
    try list.append(allocator, deep);
    // 3 OffsetCenterOfMassShapes
    try list.append(allocator, try shapes.offsetCenterOfMass(try shapes.offsetCenterOfMass(try shapes.offsetCenterOfMass(capsule, Vec3.init(0.1, 0, 0)), Vec3.init(0, 0.1, 0)), Vec3.init(0, 0, 0.1)));
    // Compounds, a triangle, a non uniform scale that the capsule does not accept
    try list.append(allocator, try shapes.staticCompound(sphere, capsule));
    try list.append(allocator, (try shapes.mutableCompound(sphere, capsule)).asShape());
    try list.append(allocator, try shapes.triangle());
    try list.append(allocator, try shapes.rotatedTranslated(try shapes.staticCompound(sphere, capsule), Vec3.init(0, 0.2, 0), Quat.identity()));
    return list;
}

/// The rotations of T1 / T2: identity, axis aligned, random
fn testRotations(random: *Random) [12]Quat {
    var rotations: [12]Quat = undefined;
    rotations[0] = Quat.identity();
    rotations[1] = Quat.rotation(Vec3.axisX(), 0.5 * std.math.pi);
    rotations[2] = Quat.rotation(Vec3.axisY(), 0.5 * std.math.pi);
    rotations[3] = Quat.rotation(Vec3.axisZ(), -0.5 * std.math.pi);
    rotations[4] = Quat.rotation(Vec3.axisY(), std.math.pi);
    rotations[5] = Quat.init(0, 0, 0, -1); // The other identity
    for (rotations[6..]) |*q| q.* = random.rotation();
    return rotations;
}

/// The translations of T1: signed zeros, tiny, ordinary, large, huge, infinite and NaN components
const special_translations = [_]Vec3{
    Vec3.init(0, 0, 0),
    Vec3.init(-0.0, -0.0, -0.0),
    Vec3.init(0, -0.0, 0),
    Vec3.init(1.0e-30, -1.0e-30, -0.0),
    Vec3.init(1, 2, 3),
    Vec3.init(-1, -2, -3),
    Vec3.init(5.0e3, -5.0e3, 2.5e3),
    Vec3.init(1.2e4, 1.2e4, -1.2e4),
    Vec3.init(1.0e7, -1.0e7, 1.0e7),
    Vec3.init(3.0e38, -3.0e38, 3.0e38),
    Vec3.init(3.4e38, -3.4e38, 1),
    Vec3.init(0, 1.0e7, -0.0),
    Vec3.init(inf, 0, 0),
    Vec3.init(0, -inf, 1),
    Vec3.init(nan, 0, 0),
    Vec3.init(1, 2, nan),
};

/// a and b are equal as real numbers (+0 == -0), component by component; neither has a NaN
fn expectEqualReals(replayed: AABox, jolt: AABox) !void {
    const pairs = [_][2]Vec3{ .{ replayed.min, jolt.min }, .{ replayed.max, jolt.max } };
    for (pairs) |pair| {
        for (0..3) |i| {
            const a = pair[0].getComponent(@intCast(i));
            const b = pair[1].getComponent(@intCast(i));
            if (std.math.isNan(a) or std.math.isNan(b) or a != b) {
                std.debug.print("replayed bounds {any} differ from Jolt's {any}\n", .{ replayed, jolt });
                return error.TestUnexpectedResult;
            }
        }
    }
}

/// The bits of two boxes are equal
fn boxBitsEqual(a: AABox, b: AABox) bool {
    return std.mem.eql(u8, std.mem.asBytes(&a.min.value), std.mem.asBytes(&b.min.value)) and std.mem.eql(u8, std.mem.asBytes(&a.max.value), std.mem.asBytes(&b.max.value));
}

/// The translation after the OffsetCenterOfMassShape additions of a replay (null: one of them is not finite)
fn chainTranslationIsFinite(replay: *const Replay, t: Vec3) bool {
    var translation = t;
    if (!isFinite(translation)) return false;
    for (replay.addends[0..replay.num_addends]) |a| {
        translation = translation.add(a);
        if (!isFinite(translation)) return false;
    }
    return true;
}

fn isFinite(v: Vec3) bool {
    return std.math.isFinite(v.getX()) and std.math.isFinite(v.getY()) and std.math.isFinite(v.getZ());
}

/// Counts of a T1 run
const ReplayCounts = struct {
    compared: u32 = 0,
    null_results: u32 = 0,
    /// Equal as real numbers, but other bits (a zero of the other sign)
    other_bits: u32 = 0,
};

/// T1 for one shape, rotation and translation: Replay.bounds against Jolt's GetWorldSpaceBounds
fn checkReplay(shape: *const Shape, replay: *const Replay, rotation: Quat, t: Vec3, counts: *ReplayCounts) !void {
    const jolt = shape.getWorldSpaceBounds(Mat44.rotationTranslation(rotation, t), Vec3.one());
    if (replay.bounds(t)) |replayed| {
        try expectEqualReals(replayed, jolt);
        try expect(chainTranslationIsFinite(replay, t));
        counts.compared += 1;
        if (!boxBitsEqual(replayed, jolt))
            counts.other_bits += 1;
    } else {
        // Only a translation that is not finite takes Jolt's code
        try expect(!chainTranslationIsFinite(replay, t));
        counts.null_results += 1;
    }
}

test "CharacterBoundsCache T1: replayed bounds equal GetWorldSpaceBounds as real numbers" {
    var shapes: Shapes = .{};
    defer shapes.deinit();
    var random = Random.init(0x12345);
    var supported = try supportedShapes(&shapes, &random);
    defer supported.deinit(allocator);
    const rotations = testRotations(&random);

    var counts: ReplayCounts = .{};
    for (supported.items) |shape| {
        try expect(CharacterBoundsCache.supportedShape(shape));
        for (rotations) |rotation| {
            var replay: Replay = undefined;
            try expect(replay.build(shape, Mat44.rotationQuat(rotation)));
            for (special_translations) |t|
                try checkReplay(shape, &replay, rotation, t, &counts);
            for (0..40) |i| {
                // Random translations of every magnitude, and the neighbours of the last one
                const scale: f32 = switch (i % 5) {
                    0 => 1.0e-30,
                    1 => 2.0,
                    2 => 1.2e4,
                    3 => 1.0e7,
                    else => 3.0e38,
                };
                const t = random.vec3(-1, 1).mulScalar(scale);
                try checkReplay(shape, &replay, rotation, t, &counts);
                try checkReplay(shape, &replay, rotation, Vec3.init(std.math.nextAfter(f32, t.getX(), inf), t.getY(), std.math.nextAfter(f32, t.getZ(), -inf)), &counts);
            }
        }
    }
    try expect(counts.compared > 50000);
    try expect(counts.null_results > 1000);
    // (`counts.other_bits`: the hoisted `T * 0` of a RotatedTranslatedShape can be +0 where Jolt's is -0, and so a zero
    // of the box could have the other sign. The final additions of T absorb it: these inputs give none, and neither did
    // a search over flat boxes (one half extent 0) behind RotatedTranslatedShapes with the exact rotations of quaternions
    // with components +-1 and +-0 and translations of +-0. The comparison is as real numbers anyway, and the T2 fact 5
    // test shows that the decisions do not read the sign of a zero.)
}

test "CharacterBoundsCache T1: an overflowing OffsetCenterOfMass translation in front of a rotation takes Jolt's code" {
    // OffsetCenterOfMass(RotatedTranslated(capsule)): T + offset overflows to inf, then the rotation product multiplies
    // it by 0 and Jolt's bounds are NaN. The replay must not give the finite box of its hoisted columns.
    var shapes: Shapes = .{};
    defer shapes.deinit();
    const capsule = try shapes.capsule(0.5, 0.3);
    const shape = try shapes.offsetCenterOfMass(try shapes.rotatedTranslated(capsule, Vec3.init(0, 0.5, 0), Quat.rotation(Vec3.axisZ(), 0.3)), Vec3.init(1.0e38, 0, 0));
    var replay: Replay = undefined;
    try expect(replay.build(shape, Mat44.identity()));
    const t = Vec3.init(-3.3e38, 1, 2);
    const jolt = shape.getWorldSpaceBounds(Mat44.translation(t), Vec3.one());
    try expect(std.math.isNan(jolt.min.getX())); // T.x = -inf, so the x row of the columns became NaN
    try expect(replay.bounds(t) == null);
}

test "CharacterBoundsCache T1: unsupported shapes and values build no replay" {
    var shapes: Shapes = .{};
    defer shapes.deinit();
    var random = Random.init(77);
    var unsupported = try unsupportedShapes(&shapes, &random);
    defer unsupported.deinit(allocator);
    for (unsupported.items) |shape| {
        var replay: Replay = undefined;
        try expect(!replay.build(shape, Mat44.identity()));
    }
    // A scale that the capsule does not accept (CapsuleShape::GetWorldSpaceBounds asserts on it), a hoisted value that
    // is not finite
    const capsule = try shapes.capsule(0.5, 0.3);
    const non_uniform = try shapes.scaled(capsule, Vec3.init(1, 2, 1));
    var replay: Replay = undefined;
    try expect(CharacterBoundsCache.supportedShape(non_uniform));
    try expect(!replay.build(non_uniform, Mat44.identity()));
    const huge = try shapes.scaled(try shapes.box(Vec3.replicate(1), 0.05), Vec3.init(1.0e30, 1.0e30, 1.0e30));
    try expect(replay.build(huge, Mat44.identity()));
    const overflow = try shapes.scaled(huge, Vec3.replicate(1.0e30));
    try expect(!replay.build(overflow, Mat44.identity()));
}

// ---------------------------------------------------------------------------------------------------------------------
// Characters

/// A system with characters in one CharacterVsCharacterCollisionSimple
const Crowd = struct {
    context: *PhysicsTestContext,
    simple: CharacterVsCharacterCollisionSimple,
    characters: std.ArrayList(*CharacterVirtual) = .empty,
    stats: Stats = .{},

    fn init(self: *Crowd, context: *PhysicsTestContext) void {
        self.* = .{ .context = context, .simple = .init(allocator) };
        self.simple.stats = &self.stats;
    }

    fn deinit(self: *Crowd) void {
        for (self.characters.items) |c| {
            self.simple.remove(c);
            c.base.release();
        }
        self.characters.deinit(allocator);
        self.simple.deinit();
    }

    fn add(self: *Crowd, shape: *const Shape, position: RVec3, rotation: Quat, opts: struct { padding: f32 = 0.02, shape_offset: Vec3 = Vec3.zero(), up: Vec3 = Vec3.axisY() }) !*CharacterVirtual {
        var settings = CharacterVirtualSettings.init(allocator);
        defer settings.deinit();
        settings.base.shape.set(shape);
        settings.base.up = opts.up;
        settings.character_padding = opts.padding;
        settings.shape_offset = opts.shape_offset;
        try self.characters.ensureUnusedCapacity(allocator, 1);
        const c = try CharacterVirtual.create(allocator, &settings, position, rotation, 0, self.context.getSystem());
        c.base.addRef();
        self.characters.appendAssumeCapacity(c);
        try self.simple.add(c);
        c.setCharacterVsCharacterCollision(self.simple.characterVsCharacterCollision());
        return c;
    }
};

/// The bytes of the hits that a collector received (with the user data that was set for each), and every user data that
/// the loop set: the characters that passed the bounds test, in order (the accept decisions, also those without a hit),
/// then the reset to 0
fn HitRecorder(comptime Collector: type, comptime Result: type) type {
    return struct {
        const Self = @This();
        pub const overrides = .{ .setUserData, .addHit };

        base: Collector = .init(Self),
        user_data: u64 = 0,
        bytes: std.ArrayList(u8) = .empty,
        decisions: std.ArrayList(u64) = .empty,
        num_hits: u32 = 0,
        max_hits: u32 = std.math.maxInt(u32),
        failed: bool = false,

        fn deinit(self: *Self) void {
            self.bytes.deinit(allocator);
            self.decisions.deinit(allocator);
        }

        pub fn setUserData(self: *Self, user_data: u64) void {
            self.user_data = user_data;
            self.decisions.append(allocator, user_data) catch {
                self.failed = true;
            };
        }

        pub fn addHit(self: *Self, result: *const Result) void {
            self.record(result) catch {
                self.failed = true;
            };
            self.num_hits += 1;
        }

        fn put(self: *Self, value: anytype) !void {
            try self.bytes.appendSlice(allocator, std.mem.asBytes(&value));
        }

        fn putVec3(self: *Self, v: Vec3) !void {
            try self.put(v.getX());
            try self.put(v.getY());
            try self.put(v.getZ());
        }

        fn record(self: *Self, result: *const Result) !void {
            const base: *const CollideShapeResult = if (Result == ShapeCastResult) &result.base else result;
            try self.put(self.user_data);
            try self.putVec3(base.contact_point_on1);
            try self.putVec3(base.contact_point_on2);
            try self.putVec3(base.penetration_axis);
            try self.put(base.penetration_depth);
            try self.put(base.sub_shape_id1.getValue());
            try self.put(base.sub_shape_id2.getValue());
            try self.put(base.body_id2.getIndexAndSequenceNumber());
            for ([_]*const CollideShapeResult.Face{ &base.shape1_face, &base.shape2_face }) |face| {
                try self.put(face.len);
                for (face.constSlice()) |p| try self.putVec3(p);
            }
            if (Result == ShapeCastResult) {
                try self.put(result.fraction);
                try self.put(@intFromBool(result.is_back_face_hit));
            }
            // The collectors of CharacterVirtual update the early out fraction: do it too, so that ShouldEarlyOut matters
            if (Result == CollideShapeResult)
                self.base.updateEarlyOutFraction(math.min(self.base.getEarlyOutFraction(), -base.penetration_depth))
            else
                self.base.updateEarlyOutFraction(math.min(self.base.getEarlyOutFraction(), result.fraction));
            // An early out after `max_hits` hits: the loops must stop at the same character
            if (self.num_hits + 1 >= self.max_hits)
                self.base.forceEarlyOut();
        }
    };
}

const CollideRecorder = HitRecorder(CollideShapeCollector, CollideShapeResult);
const CastRecorder = HitRecorder(CastShapeCollector, ShapeCastResult);

/// Jolt's CharacterVsCharacterCollisionSimple::CollideCharacter (Jolt/Physics/Character/CharacterVirtual.cpp), copied:
/// the oracle of the cache
fn oracleCollideCharacter(characters: []const *CharacterVirtual, character: *const CharacterVirtual, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector) void {
    // Make shape 1 relative to inBaseOffset
    const transform1 = center_of_mass_transform.postTranslatedRVec3(base_offset.negate()).toMat44();

    const shape1 = character.getShape().?;
    var settings = collide_shape_settings.*;

    // Get bounds for character
    const bounds1 = shape1.getWorldSpaceBounds(transform1, Vec3.one());

    // Iterate over all characters
    for (characters) |c| {
        if (c != character and !collector.shouldEarlyOut()) {
            // Make shape 2 relative to inBaseOffset
            const transform2 = c.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();

            // We need to add the padding of character 2 so that we will detect collision with its outer shell
            settings.max_separation_distance = collide_shape_settings.max_separation_distance + c.getCharacterPadding();

            // Check if the bounding boxes of the characters overlap
            const shape2 = c.getShape().?;
            var bounds2 = shape2.getWorldSpaceBounds(transform2, Vec3.one());
            bounds2.expandBy(Vec3.replicate(settings.max_separation_distance));
            if (!bounds1.overlaps(bounds2))
                continue;

            // Collector needs to know which character we're colliding with
            collector.setUserData(@intFromPtr(c));

            // Note that this collides against the character's shape without padding, this will be corrected for in CharacterVirtual::GetContactsAtPosition
            CollisionDispatch.collideShapeVsShape(shape1, shape2, Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, collector, &.{});
        }
    }

    // Reset the user data
    collector.setUserData(0);
}

/// Jolt's CharacterVsCharacterCollisionSimple::CastCharacter, copied: the oracle of the cache
fn oracleCastCharacter(characters: []const *CharacterVirtual, character: *const CharacterVirtual, center_of_mass_transform: RMat44, direction: Vec3, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector) void {
    // Convert shape cast relative to inBaseOffset
    const transform1 = center_of_mass_transform.postTranslatedRVec3(base_offset.negate()).toMat44();
    const shape_cast = ShapeCast.init(character.getShape().?, Vec3.one(), transform1, direction);

    // Get world space bounds of the character in the form of center and extent
    const origin = shape_cast.shape_world_bounds.getCenter();
    const extents = shape_cast.shape_world_bounds.getExtent().add(Vec3.replicate(shape_cast_settings.extra_convex_radius));

    // Create a copy of the settings so we can modify it for each character we test against
    var cast_settings = shape_cast_settings.*;

    // Iterate over all characters
    for (characters) |c| {
        if (c != character and !collector.shouldEarlyOut()) {
            // Make shape 2 relative to inBaseOffset
            const transform2 = c.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();

            // We need to add the padding of character 2 so that we will detect collision with its outer shell
            cast_settings.extra_convex_radius = shape_cast_settings.extra_convex_radius + c.getCharacterPadding();

            // Sweep bounding box of the character against the bounding box of the other character to see if they can collide
            const shape2 = c.getShape().?;
            var bounds2 = shape2.getWorldSpaceBounds(transform2, Vec3.one());
            bounds2.expandBy(extents.add(Vec3.replicate(c.getCharacterPadding())));
            if (!rayAABoxHitsDirection(origin, direction, bounds2.min, bounds2.max))
                continue;

            // Collector needs to know which character we're colliding with
            collector.setUserData(@intFromPtr(c));

            // Note that this collides against the character's shape without padding, this will be corrected for in CharacterVirtual::ValidateMovement
            CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &cast_settings, shape2, Vec3.one(), &.{}, transform2, .{}, .{}, collector);
        }
    }

    // Reset the user data
    collector.setUserData(0);
}

/// The hits of CollideCharacter of the Simple and of the oracle are equal bit for bit; returns the number of hits
fn expectCollideEqualsOracle(crowd: *Crowd, character: *const CharacterVirtual, max_separation_distance: f32, base_offset: RVec3, max_hits: u32) !u32 {
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = max_separation_distance;
    const transform = character.getCenterOfMassTransform();
    var simple_hits: CollideRecorder = .{ .max_hits = max_hits };
    defer simple_hits.deinit();
    var oracle_hits: CollideRecorder = .{ .max_hits = max_hits };
    defer oracle_hits.deinit();
    crowd.simple.collideCharacter(character, transform, &settings, base_offset, &simple_hits.base);
    oracleCollideCharacter(crowd.simple.characters.items, character, transform, &settings, base_offset, &oracle_hits.base);
    try expectRecordersEqual(CollideRecorder, &oracle_hits, &simple_hits);
    try expectEqual(@as(u64, 0), simple_hits.user_data); // Reset to 0
    return simple_hits.num_hits;
}

/// The hits of CastCharacter of the Simple and of the oracle are equal bit for bit (the base offset of
/// ValidateMovement: the start position); returns the number of hits
fn expectCastEqualsOracle(crowd: *Crowd, character: *const CharacterVirtual, direction: Vec3, extra_convex_radius: f32, max_hits: u32) !u32 {
    return expectCastEqualsOracleAt(crowd, character, direction, extra_convex_radius, max_hits, character.getCenterOfMassTransform().getTranslation());
}

/// `expectCastEqualsOracle` with another base offset
fn expectCastEqualsOracleAt(crowd: *Crowd, character: *const CharacterVirtual, direction: Vec3, extra_convex_radius: f32, max_hits: u32, base_offset: RVec3) !u32 {
    var settings: ShapeCastSettings = .{};
    settings.extra_convex_radius = extra_convex_radius;
    settings.use_shrunken_shape_and_convex_radius = true;
    settings.return_deepest_point = false;
    const start = character.getCenterOfMassTransform();
    var simple_hits: CastRecorder = .{ .max_hits = max_hits };
    defer simple_hits.deinit();
    var oracle_hits: CastRecorder = .{ .max_hits = max_hits };
    defer oracle_hits.deinit();
    crowd.simple.castCharacter(character, start, direction, &settings, base_offset, &simple_hits.base);
    oracleCastCharacter(crowd.simple.characters.items, character, start, direction, &settings, base_offset, &oracle_hits.base);
    try expectRecordersEqual(CastRecorder, &oracle_hits, &simple_hits);
    return simple_hits.num_hits;
}

/// The Simple accepted the same characters as the oracle (in the same order) and got the same hits, bit for bit
fn expectRecordersEqual(comptime Recorder: type, oracle: *const Recorder, simple: *const Recorder) !void {
    try expect(!simple.failed and !oracle.failed);
    try expect(std.mem.eql(u64, oracle.decisions.items, simple.decisions.items));
    try expectEqual(oracle.num_hits, simple.num_hits);
    try expect(std.mem.eql(u8, oracle.bytes.items, simple.bytes.items));
}

/// The position of the crowds: the origin, and far from it (around 1e9 in double precision)
fn crowdCenter(far: bool) RVec3 {
    if (!far) return RVec3.zero();
    return if (Core.double_precision) RVec3.init(1.0e9 + 0.25, 200, -1.0e9 - 0.5) else RVec3.init(5.0e3, 200, -1.2e4);
}

/// Two transforms have the same bits (the rotation columns and the translation; a DMat44 has padding bytes)
fn expectSameTransformBits(a: RMat44, b: RMat44) !void {
    for (0..3) |i|
        try expect(std.mem.eql(u8, std.mem.asBytes(&a.col[i].value), std.mem.asBytes(&b.col[i].value)));
    const ta = a.getTranslation();
    const tb = b.getTranslation();
    for (0..3) |i| {
        const x = ta.getComponent(@intCast(i));
        const y = tb.getComponent(@intCast(i));
        try expect(std.mem.eql(u8, std.mem.asBytes(&x), std.mem.asBytes(&y)));
    }
}

fn offsetPosition(center: RVec3, offset: Vec3) RVec3 {
    return center.addVec3(offset);
}

test "CharacterBoundsCache T1: the replayed bounds of characters, with the loop's transform" {
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    var random = Random.init(0xbeef);
    var supported = try supportedShapes(&shapes, &random);
    defer supported.deinit(allocator);

    var counts: ReplayCounts = .{};
    for ([_]bool{ false, true }) |far| {
        var crowd: Crowd = undefined;
        crowd.init(&context);
        defer crowd.deinit();
        const center = crowdCenter(far);
        for (supported.items, 0..) |shape, i| {
            const rotation = if (i % 3 == 0) Quat.identity() else random.rotation();
            const c = try crowd.add(shape, offsetPosition(center, random.vec3(-20, 20)), rotation, .{ .padding = if (i % 4 == 0) 0 else 0.02, .shape_offset = if (i % 2 == 0) Vec3.zero() else random.vec3(-0.2, 0.2) });
            try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
            // 1a: the cached transform has the bits of GetCenterOfMassTransform
            const com = c.getCenterOfMassTransform();
            try expectSameTransformBits(com, c.bounds_cache.center_of_mass_transform);
            // 1b: the bounds of the loop's transform for a base offset 0, near the position and at the position
            const base_offsets = [_]RVec3{ RVec3.zero(), c.getPosition().addVec3(random.vec3(-3, 3)), c.getPosition(), center };
            for (base_offsets) |base_offset| {
                const transform2 = com.postTranslatedRVec3(base_offset.negate()).toMat44();
                const jolt = shape.getWorldSpaceBounds(transform2, Vec3.one());
                const replayed = c.bounds_cache.replay.bounds(transform2.getTranslation()) orelse return error.TestUnexpectedResult;
                try expectEqualReals(replayed, jolt);
                counts.compared += 1;
            }
        }
    }
    try expect(counts.compared == 8 * supported.items.len);
}

// ---------------------------------------------------------------------------------------------------------------------
// T2: decisions

/// The decision of CollideCharacter's loop: ExpandBy(max separation distance), then Overlaps
fn collideDecision(bounds1: AABox, bounds2_in: AABox, separation: f32) bool {
    var bounds2 = bounds2_in;
    bounds2.expandBy(Vec3.replicate(separation));
    return bounds1.overlaps(bounds2);
}

/// The decision of CastCharacter's loop: ExpandBy(extents + padding), then RayAABoxHits
fn castDecision(origin: Vec3, direction: Vec3, extents: Vec3, padding: f32, bounds2_in: AABox) bool {
    var bounds2 = bounds2_in;
    bounds2.expandBy(extents.add(Vec3.replicate(padding)));
    return rayAABoxHitsDirection(origin, direction, bounds2.min, bounds2.max);
}

/// A float `n` ulp next to `x` (n < 0: towards -inf)
fn ulps(x: f32, n: i32) f32 {
    var y = x;
    var i: i32 = 0;
    while (i < @abs(n)) : (i += 1)
        y = std.math.nextAfter(f32, y, if (n > 0) inf else -inf);
    return y;
}

fn withComponent(v: Vec3, axis: usize, value: f32) Vec3 {
    var r = v;
    r.setComponent(@intCast(axis), value);
    return r;
}

test "CharacterBoundsCache T2: reject decisions at the flip and 1 to 3 ulp around it" {
    var shapes: Shapes = .{};
    defer shapes.deinit();
    var random = Random.init(0xd1ce);
    var supported = try supportedShapes(&shapes, &random);
    defer supported.deinit(allocator);
    const rotations = testRotations(&random);
    const separations = [_]f32{ 0, -0.0, 1.0e-40, 1.0e-30, 0.02, 1.0e30, inf, nan };
    const directions = [_]Vec3{
        Vec3.init(1, 0, 0),            Vec3.init(0, 0, 0),   Vec3.init(-0.0, -0.0, -0.0), Vec3.init(1.0e-30, 0, -0.0),
        Vec3.init(1.0e30, -1.0e30, 0), Vec3.init(nan, 0, 1), Vec3.init(0.3, -0.2, 0.9),   Vec3.init(-2, 0.5, 0),
        Vec3.init(0, 1.0e-45, 1),      Vec3.init(inf, 0, 0),
    };

    var num_decisions: u32 = 0;
    var num_flips: u32 = 0;
    for (supported.items) |shape| {
        for (rotations[0..6]) |rotation| {
            var replay: Replay = undefined;
            try expect(replay.build(shape, Mat44.rotationQuat(rotation)));
            const t0 = random.vec3(-100, 100);
            const jolt0 = shape.getWorldSpaceBounds(Mat44.rotationTranslation(rotation, t0), Vec3.one());
            for (0..3) |axis| {
                for ([_]bool{ false, true }) |above| {
                    for (separations) |separation| {
                        // A query box that touches the expanded box of Jolt at t0 on one face: the decision flips there
                        var expanded = jolt0;
                        expanded.expandBy(Vec3.replicate(separation));
                        const face = if (above) expanded.max.getComponent(@intCast(axis)) else expanded.min.getComponent(@intCast(axis));
                        if (!std.math.isFinite(face)) continue;
                        const far_side: f32 = if (above) 10 else -10;
                        const lo = withComponent(Vec3.replicate(-1.0e6), axis, if (above) face else face + far_side);
                        const hi = withComponent(Vec3.replicate(1.0e6), axis, if (above) face + far_side else face);
                        const bounds1 = AABox.init(lo, hi);
                        var last: ?bool = null;
                        var n: i32 = -3;
                        while (n <= 3) : (n += 1) {
                            const t = withComponent(t0, axis, ulps(t0.getComponent(@intCast(axis)), n));
                            const jolt = shape.getWorldSpaceBounds(Mat44.rotationTranslation(rotation, t), Vec3.one());
                            const replayed = replay.bounds(t).?;
                            const decision = collideDecision(bounds1, jolt, separation);
                            try expectEqual(decision, collideDecision(bounds1, replayed, separation));
                            if (last) |l| {
                                if (l != decision) num_flips += 1;
                            }
                            last = decision;
                            num_decisions += 1;

                            // The cast decision of a ray that grazes the same face
                            for (directions) |direction| {
                                const origin = withComponent(t0.add(random.vec3(-2, 2)), axis, ulps(face, @mod(n + 3, 3) - 1));
                                const extents = Vec3.init(0.3, 0.9, 0.3);
                                for ([_]f32{ 0, 0.02, separation }) |padding| {
                                    const cast = castDecision(origin, direction, extents, padding, jolt);
                                    try expectEqual(cast, castDecision(origin, direction, extents, padding, replayed));
                                    num_decisions += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    try expect(num_decisions > 100000);
    try expect(num_flips > 1000); // The tests cross the flip of the overlap test
}

/// The components of the fact 5 test: zeros of both signs (more often), a denormal, ordinary values
const zero_rich_values = [_]f32{ 0, -0.0, 0, -0.0, 0, -0.0, 1.0e-45, -1.0e-45, 0.25, -0.25, 1, -1 };

fn zeroRichVec3(random: *Random, values: []const f32) Vec3 {
    return Vec3.init(values[random.index(values.len)], values[random.index(values.len)], values[random.index(values.len)]);
}

/// `v` with the sign of the zero components flipped where `mask` has a bit (other components unchanged)
fn flipZeroSigns(v: Vec3, mask: usize) Vec3 {
    var r = v;
    for (0..3) |i| {
        const x = v.getComponent(@intCast(i));
        if (x == 0 and (mask >> @intCast(i)) & 1 != 0)
            r.setComponent(@intCast(i), -x);
    }
    return r;
}

test "CharacterBoundsCache T2: the decisions read no sign of a zero in the box (proof fact 5)" {
    // The replayed box equals Jolt's as real numbers, but a zero in it can have the other sign. Fact 5 of the proof in
    // CharacterBoundsCache.zig: ExpandBy + Overlaps and ExpandBy + RayAABoxHits give the same answer for two boxes that
    // differ only there. The boxes, the query boxes, the separations, origins, directions, extents and paddings are full
    // of zeros of both signs (and the directions have infinite and NaN components), so that a decision that read a sign
    // bit would differ.
    var random = Random.init(0xfac75);
    const separations = [_]f32{ 0, -0.0, 0, 1.0e-45, 0.25 };
    const directions = [_]f32{ 0, -0.0, 1.0e-45, 1, -1, inf, -inf, nan };
    var num_differing_boxes: u32 = 0;
    var num_collide = [2]u32{ 0, 0 }; // Decisions false / true on boxes that differ
    var num_cast = [2]u32{ 0, 0 };
    for (0..200000) |_| {
        const box = AABox.init(zeroRichVec3(&random, &zero_rich_values), zeroRichVec3(&random, &zero_rich_values));
        const other = AABox.init(flipZeroSigns(box.min, random.index(8)), flipZeroSigns(box.max, random.index(8)));
        if (boxBitsEqual(box, other)) continue;
        num_differing_boxes += 1;

        const bounds1 = AABox.init(zeroRichVec3(&random, &zero_rich_values), zeroRichVec3(&random, &zero_rich_values));
        const separation = separations[random.index(separations.len)];
        const collide = collideDecision(bounds1, box, separation);
        try expectEqual(collide, collideDecision(bounds1, other, separation));
        num_collide[@intFromBool(collide)] += 1;

        const origin = zeroRichVec3(&random, &zero_rich_values);
        const direction = zeroRichVec3(&random, &directions);
        const extents = zeroRichVec3(&random, &[_]f32{ 0, -0.0, 0.25 });
        const padding = separations[random.index(separations.len)];
        const cast = castDecision(origin, direction, extents, padding, box);
        try expectEqual(cast, castDecision(origin, direction, extents, padding, other));
        num_cast[@intFromBool(cast)] += 1;
    }
    try expect(num_differing_boxes > 100000);
    for ([_][2]u32{ num_collide, num_cast }) |n|
        try expect(n[0] > 10000 and n[1] > 10000); // Both answers occur
}

// ---------------------------------------------------------------------------------------------------------------------
// T3: end to end

/// A random crowd of `count` characters with the shapes of `shape_list` around `center`, within a cube of `size`
fn addRandomCrowd(crowd: *Crowd, random: *Random, shape_list: []const *const Shape, count: usize, center: RVec3, size: f32) !void {
    for (0..count) |i| {
        const shape = shape_list[random.index(shape_list.len)];
        const rotation = switch (i % 3) {
            0 => Quat.identity(),
            1 => Quat.rotation(Vec3.axisY(), random.float(-3, 3)),
            else => random.rotation(),
        };
        _ = try crowd.add(shape, offsetPosition(center, random.vec3(-size, size)), rotation, .{
            .padding = switch (i % 4) {
                0 => 0.02,
                1 => 0,
                2 => 0.1,
                else => 0.5,
            },
            .shape_offset = if (i % 5 == 0) random.vec3(-0.3, 0.3) else Vec3.zero(),
            .up = if (i % 7 == 3) Vec3.axisX() else Vec3.axisY(),
        });
    }
}

test "CharacterBoundsCache T3: the hits of the Simple equal Jolt's loop on random crowds" {
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    var random = Random.init(0x5eed);
    var supported = try supportedShapes(&shapes, &random);
    defer supported.deinit(allocator);
    var unsupported = try unsupportedShapes(&shapes, &random);
    defer unsupported.deinit(allocator);
    var shape_list: std.ArrayList(*const Shape) = .empty;
    defer shape_list.deinit(allocator);
    try shape_list.appendSlice(allocator, supported.items);
    try shape_list.appendSlice(allocator, unsupported.items[1..4]); // 3 OffsetCenterOfMass levels (max_addends is 2), the two compounds

    var num_collide_hits: u32 = 0;
    var num_cast_hits: u32 = 0;
    var total: Stats = .{};
    for ([_]bool{ false, true }) |far| {
        for ([_]f32{ 2.5, 6.0 }) |size| {
            var crowd: Crowd = undefined;
            crowd.init(&context);
            defer crowd.deinit();
            const center = crowdCenter(far);
            try addRandomCrowd(&crowd, &random, shape_list.items, 40, center, size);
            for (crowd.characters.items) |c| {
                for ([_]f32{ 0, 0.1, 1.0 }) |separation| {
                    const base_offsets = [_]RVec3{ c.getPosition(), RVec3.zero(), center, c.getPosition().addVec3(random.vec3(-1, 1)) };
                    for (base_offsets, 0..) |base_offset, i|
                        num_collide_hits += try expectCollideEqualsOracle(&crowd, c, separation, base_offset, if (i == 3) 2 else std.math.maxInt(u32));
                }
                for (0..3) |i| {
                    const direction = random.vec3(-1, 1).mulScalar(random.float(0.01, 3.0));
                    num_cast_hits += try expectCastEqualsOracle(&crowd, c, direction, if (random.index(2) == 0) 0 else 0.02, if (i == 2) 1 else std.math.maxInt(u32));
                }
                num_cast_hits += try expectCastEqualsOracle(&crowd, c, Vec3.zero(), 0, std.math.maxInt(u32));
            }
            inline for (std.meta.fields(Stats)) |f| @field(total, f.name) += @field(crowd.stats, f.name);
        }
    }
    try expect(num_collide_hits > 500);
    try expect(num_cast_hits > 100);
    try expect(total.rejected > 10000);
    try expect(total.accepted > 1000);
    try expect(total.unsupported > 1000);
    try expectEqual(@as(u64, 0), total.stale); // Every character is refreshed
}

test "CharacterBoundsCache T3: a translation that is not finite runs Jolt's GetWorldSpaceBounds (the non_finite fallback)" {
    // Two ways to a translation that is not finite (Replay.bounds returns null, the loop counts `non_finite` and runs
    // Jolt's GetWorldSpaceBounds on transform2): the center of mass minus the base offset overflows (T is infinite), or
    // T is finite and the OffsetCenterOfMassShape addition overflows. Jolt's boxes are then infinite in x (a capsule) or
    // NaN in x (a RotatedTranslatedShape multiplies T by 0). Jolt's code rejects all these pairs: on x (an infinite box),
    // or on z, where they are far. The plain capsule `fa` lies where its box would overlap the query if the fallback
    // dropped the base offset; its collision would then return at the oriented box test of CollideConvexVsConvex
    // (an infinite translation), and the accepted pair shows in the decisions.
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    const capsule = try shapes.capsule(0.5, 0.3);
    const character_shape = try shapes.rotatedTranslated(capsule, Vec3.init(0, 0.8, 0), Quat.identity());
    // Its center of mass is 2e38 behind its position: at a position of 2e38 it is at the origin, and T + 2e38 overflows
    // when T > 1.4e38
    const offset_shape = try shapes.offsetCenterOfMass(capsule, Vec3.init(-2.0e38, 0, 0));
    const huge: Real = 1.75e38;

    // Collide: q and n query with the base offset (-1.75e38, 0, 0), so their transform1 is at x = 1.75e38
    {
        var crowd: Crowd = undefined;
        crowd.init(&context);
        defer crowd.deinit();
        const q = try crowd.add(character_shape, RVec3.zero(), Quat.identity(), .{});
        const n = try crowd.add(character_shape, RVec3.init(0, 0, 0.5), Quat.identity(), .{});
        // T = 3.5e38 is infinite: Jolt's box is [inf, inf] in x, rejected on x
        _ = try crowd.add(capsule, RVec3.init(huge, 0, 0), Quat.identity(), .{});
        // T = 3.5e38 is infinite: Jolt's box is NaN in x, rejected on z
        _ = try crowd.add(character_shape, RVec3.init(huge, 0, 50), Quat.identity(), .{});
        // T = 1.75e38 is finite, T + 2e38 is not: Jolt's box is [inf, inf] in x, rejected on x
        _ = try crowd.add(offset_shape, RVec3.init(2.0e38, 0, 0), Quat.identity(), .{});
        for (crowd.characters.items) |c|
            try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
        var num_hits: u32 = 0;
        for ([_]*CharacterVirtual{ q, n }) |c| {
            for ([_]f32{ 0, 0.1 }) |separation| {
                const before = crowd.stats;
                num_hits += try expectCollideEqualsOracle(&crowd, c, separation, RVec3.init(-huge, 0, 0), std.math.maxInt(u32));
                try expectEqual(before.non_finite + 3, crowd.stats.non_finite);
                try expectEqual(before.accepted + 1, crowd.stats.accepted); // The other one of q and n
                try expectEqual(before.rejected + 3, crowd.stats.rejected);
                // With the base offset 0 every translation is finite
                num_hits += try expectCollideEqualsOracle(&crowd, c, separation, RVec3.zero(), std.math.maxInt(u32));
                try expectEqual(before.non_finite + 3, crowd.stats.non_finite);
            }
        }
        try expect(num_hits > 0); // q and n touch
    }

    // Cast (and collide) with the base offset of ValidateMovement, the start position: q2 and m are at x = -1.75e38,
    // the others at 1.75e38 and 2e38 with z = 50. Every direction has z <= 0: the slab test rejects them on z.
    {
        var crowd: Crowd = undefined;
        crowd.init(&context);
        defer crowd.deinit();
        const q2 = try crowd.add(character_shape, RVec3.init(-huge, 0, 0), Quat.identity(), .{});
        const m = try crowd.add(character_shape, RVec3.init(-huge, 0, 0.5), Quat.identity(), .{});
        _ = try crowd.add(capsule, RVec3.init(huge, 0, 50), Quat.identity(), .{});
        _ = try crowd.add(character_shape, RVec3.init(huge, 0, 50), Quat.identity(), .{});
        // T = 1.75e38 (the center of mass is at the origin), T + 2e38 overflows
        _ = try crowd.add(offset_shape, RVec3.init(2.0e38, 0, 50), Quat.identity(), .{});
        var num_hits: u32 = 0;
        for ([_]*CharacterVirtual{ q2, m }) |c| {
            const start = c.getCenterOfMassTransform().getTranslation();
            for ([_]Vec3{ Vec3.init(0, 0, -0.6), Vec3.init(-0.3, 0.1, -0.8), Vec3.init(0, -1, 0), Vec3.zero() }) |direction| {
                const non_finite_before = crowd.stats.non_finite;
                num_hits += try expectCastEqualsOracleAt(&crowd, c, direction, 0, std.math.maxInt(u32), start);
                try expectEqual(non_finite_before + 3, crowd.stats.non_finite);
            }
            const non_finite_before = crowd.stats.non_finite;
            num_hits += try expectCollideEqualsOracle(&crowd, c, 0.1, start, std.math.maxInt(u32));
            try expectEqual(non_finite_before + 3, crowd.stats.non_finite);
        }
        try expect(num_hits > 0); // m casts into q2, q2 and m touch
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// T4: staleness

/// T4: the cache of `c` is stale, the queries of every character still equal the oracle and count stale pairs
fn expectStaleAndOracle(crowd: *Crowd, c: *const CharacterVirtual) !void {
    try expectEqual(CharacterBoundsCache.Find.stale, c.bounds_cache.find(&c.boundsCacheKey()));
    const stale_before = crowd.stats.stale;
    try expectQueriesEqualOracle(crowd);
    try expect(crowd.stats.stale > stale_before);
}

fn expectQueriesEqualOracle(crowd: *Crowd) !void {
    for (crowd.characters.items) |q| {
        _ = try expectCollideEqualsOracle(crowd, q, 0.1, q.getPosition(), std.math.maxInt(u32));
        _ = try expectCastEqualsOracle(crowd, q, Vec3.init(1.5, -0.2, 0.4), 0, std.math.maxInt(u32));
    }
}

test "CharacterBoundsCache T4: direct writes, setUp and base.shape.set make the cache stale, the refresh points valid" {
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    const capsule = try shapes.rotatedTranslated(try shapes.capsule(0.5, 0.3), Vec3.init(0, 0.8, 0), Quat.identity());
    const box = try shapes.box(Vec3.init(0.4, 0.8, 0.4), 0.05);
    var crowd: Crowd = undefined;
    crowd.init(&context);
    defer crowd.deinit();
    const c = try crowd.add(capsule, RVec3.init(0, 0, 0), Quat.identity(), .{});
    _ = try crowd.add(capsule, RVec3.init(0.7, 0, 0), Quat.identity(), .{});
    _ = try crowd.add(box, RVec3.init(-0.6, 0.2, 0.3), Quat.rotation(Vec3.axisY(), 0.3), .{});
    try expectQueriesEqualOracle(&crowd);
    try expectEqual(@as(u64, 0), crowd.stats.stale);

    // Direct writes to the public fields
    c.position = RVec3.init(0.35, 0, 0.1);
    try expectStaleAndOracle(&crowd, c);
    c.refreshBoundsCache();
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    c.rotation = Quat.rotation(Vec3.axisZ(), 0.4);
    try expectStaleAndOracle(&crowd, c);
    c.setPosition(c.getPosition()); // A refresh point
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    c.shape_offset = Vec3.init(0, 0.2, 0);
    try expectStaleAndOracle(&crowd, c);
    c.setShapeOffset(Vec3.init(0, 0.1, 0));
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    c.character_padding = 0.3;
    try expectStaleAndOracle(&crowd, c);
    c.setRotation(Quat.identity());
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));

    // CharacterBase.setUp (not a refresh point: the padding moves the center of mass along up)
    c.base.setUp(Vec3.init(0.6, 0.8, 0));
    try expectStaleAndOracle(&crowd, c);
    c.refreshBoundsCache();
    try expectQueriesEqualOracle(&crowd);

    // base.shape.set
    c.base.shape.set(box);
    try expectStaleAndOracle(&crowd, c);
    c.refreshBoundsCache();
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);

    // SetShape (a refresh point on every exit) to an unsupported shape and back
    const compound = try shapes.staticCompound(box, capsule);
    const filters = .{ context.getSystem().getDefaultBroadPhaseLayerFilter(Layers.moving), context.getSystem().getDefaultLayerFilter(Layers.moving) };
    const temp = context.temp_allocator.tempAllocator();
    try expect(try c.setShape(compound, std.math.floatMax(f32), &filters[0].base, &filters[1].base, &.{}, null, temp));
    try expectEqual(CharacterBoundsCache.Find.unsupported, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);
    try expect(try c.setShape(capsule, std.math.floatMax(f32), &filters[0].base, &filters[1].base, &.{}, null, temp));
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);
}

test "CharacterBoundsCache T4: a freed shape address cannot be reused while it is cached (the pin)" {
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    const capsule = try shapes.rotatedTranslated(try shapes.capsule(0.5, 0.3), Vec3.init(0, 0.8, 0), Quat.identity());
    var crowd: Crowd = undefined;
    crowd.init(&context);
    defer crowd.deinit();
    const c = try crowd.add(capsule, RVec3.init(0, 0, 0), Quat.identity(), .{});
    _ = try crowd.add(capsule, RVec3.init(1.2, 0, 0), Quat.identity(), .{});

    // The shapes of this part come from a FixedBufferAllocator: freeing its last block and allocating the same size
    // again returns the same address
    var buffer: [4096]u8 align(64) = undefined;
    var fba = std.heap.FixedBufferAllocator.init(&buffer);
    const small = (try BoxShape.create(fba.allocator(), Vec3.replicate(0.1), .{ .convex_radius = 0.05 })).asShape();
    c.base.shape.set(small);
    c.refreshBoundsCache(); // Cached for the small box: the queries of the other character reject c
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);

    // c switches back (a direct write); the small box loses its last outside reference, but the cache holds one
    c.base.shape.set(capsule);
    const big = (try BoxShape.create(fba.allocator(), Vec3.replicate(2.0), .{ .convex_radius = 0.05 })).asShape();
    var big_ref = RefConst(Shape).init(big);
    defer big_ref.deinit();
    c.base.shape.set(big); // A direct write: the key has another shape than the cache
    // Without the pin the big box gets the address of the freed small box, the key matches the cache and the other
    // character rejects the big box with the bounds of the small one: the hits differ from the oracle
    try expectQueriesEqualOracle(&crowd);
    try expect(big != small);
    try expectStaleAndOracle(&crowd, c);
    c.base.shape.set(capsule);
    c.refreshBoundsCache(); // Releases the small box
}

// ---------------------------------------------------------------------------------------------------------------------
// T5: MutableCompoundShape

test "CharacterBoundsCache T5: a MutableCompoundShape changed in place is unsupported and gives Jolt's hits" {
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    const sphere = try shapes.sphere(0.3);
    const capsule = try shapes.capsule(0.4, 0.25);
    const compound = try shapes.mutableCompound(sphere, capsule);
    const other = try shapes.rotatedTranslated(capsule, Vec3.init(0, 0.65, 0), Quat.identity());
    var crowd: Crowd = undefined;
    crowd.init(&context);
    defer crowd.deinit();
    const c = try crowd.add(compound.asShape(), RVec3.init(0, 0, 0), Quat.identity(), .{});
    _ = try crowd.add(other, RVec3.init(0.9, 0, 0), Quat.identity(), .{});
    _ = try crowd.add(other, RVec3.init(-0.4, 0.3, 0.8), Quat.rotation(Vec3.axisX(), 0.2), .{});
    try expectEqual(CharacterBoundsCache.Find.unsupported, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);
    const unsupported_before = crowd.stats.unsupported;
    try expect(unsupported_before > 0);

    // Change the compound in place (its bounds and its center of mass), the key of c does not change
    compound.modifyShape(0, Vec3.init(0.8, 0.4, 0), Quat.rotation(Vec3.axisY(), 0.5));
    compound.modifyShape(1, Vec3.init(-0.6, 1.5, 0.2), Quat.identity());
    compound.adjustCenterOfMass();
    try expectEqual(CharacterBoundsCache.Find.unsupported, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);
    try expect(crowd.stats.unsupported > unsupported_before);
    _ = try compound.addShape(Vec3.init(0, -0.5, 0.9), Quat.identity(), sphere, .{});
    c.refreshBoundsCache(); // The key did not change: still unsupported
    try expectQueriesEqualOracle(&crowd);
}

test "CharacterBoundsCache: the refresh points of Update, restore and destruction" {
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var shapes: Shapes = .{};
    defer shapes.deinit();
    const capsule = try shapes.rotatedTranslated(try shapes.capsule(0.5, 0.3), Vec3.init(0, 0.8, 0), Quat.identity());
    var crowd: Crowd = undefined;
    crowd.init(&context);
    defer crowd.deinit();
    const c = try crowd.add(capsule, RVec3.init(0, 2, 0), Quat.identity(), .{});
    _ = try crowd.add(capsule, RVec3.init(0.7, 2, 0), Quat.identity(), .{});
    try expectEqual(@as(u32, 5), capsule.getRefCount()); // Shapes, and per character base.shape and the pin

    // Update moves the character (in place in MoveShape) and refreshes in UpdateInnerBodyTransform
    var recorder: zolt.StateRecorderImpl = .init(allocator);
    defer recorder.deinit();
    c.saveState(&recorder.base);
    const filters = .{ context.getSystem().getDefaultBroadPhaseLayerFilter(Layers.moving), context.getSystem().getDefaultLayerFilter(Layers.moving) };
    c.setLinearVelocity(Vec3.init(2, -1, 0));
    try c.update(1.0 / 60.0, Vec3.init(0, -9.81, 0), &filters[0].base, &filters[1].base, &.{}, null, context.temp_allocator.tempAllocator());
    try expect(!c.getPosition().eql(RVec3.init(0, 2, 0)));
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);

    // RestoreState writes the position and refreshes
    recorder.rewind();
    try c.restoreState(&recorder.base);
    try expect(c.getPosition().eql(RVec3.init(0, 2, 0)));
    try expectEqual(CharacterBoundsCache.Find.valid, c.bounds_cache.find(&c.boundsCacheKey()));
    try expectQueriesEqualOracle(&crowd);

    // The pin holds one reference per character, the destructor releases it (the testing allocator finds a leak)
    try expectEqual(@as(u32, 5), capsule.getRefCount());
}
