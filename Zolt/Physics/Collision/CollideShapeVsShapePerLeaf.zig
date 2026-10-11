//! Port of: Jolt/Physics/Collision/CollideShapeVsShapePerLeaf.h
//! Status: complete
//!
//! `template <class LeafCollector> void CollideShapeVsShapePerLeaf(...)` is
//! `collideShapeVsShapePerLeaf(comptime LeafCollector, allocator, ...)`. LeafCollector is a collector type with
//! `init()`, `deinit()`, `base` (a CollideShapeCollector), `hadHit()` and `hit`:
//! `AnyHitCollisionCollector(CollideShapeCollector)` or `ClosestHitCollisionCollector(CollideShapeCollector)`.
//!
//! Allocation (Docs/Zolt/CollisionArchitecture.md, D6 / D13): Jolt collects the leaf shapes in an
//! `Array<LeafShape, STLLocalAllocator<LeafShape, 32>>`, which lives on the stack and falls back to the heap for more
//! than 32 leaves. Zolt does the same with an `STLLocalAllocator(LeafShape, 32)` whose heap is the `allocator`
//! parameter (a new parameter: Jolt uses its global heap), so the function returns `Allocator.Error!void`. When a leaf
//! cannot be stored the collection is stopped (forced early out) and error.OutOfMemory is returned before any pair is
//! collided.
//! Jolt's default argument `inShapeFilter = { }` is a required parameter (pass `&.{}`), like
//! CollisionDispatch.collideShapeVsShape.

const std = @import("std");
const Allocator = std.mem.Allocator;
const STLLocalAllocator = @import("../../Core/STLLocalAllocator.zig").STLLocalAllocator;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const SubShapeIDCreator = @import("Shape/SubShapeID.zig").SubShapeIDCreator;
const CollideShapeSettings = @import("CollideShape.zig").CollideShapeSettings;
const CollisionDispatch = @import("CollisionDispatch.zig");
const ShapeFilter = @import("ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;

/// Tracks information we need about a leaf shape
const LeafShape = struct {
    bounds: AABox,
    center_of_mass_transform: Mat44,
    scale: Vec3,
    shape: *const Shape,
    sub_shape_id_creator: SubShapeIDCreator,
};

const max_local_leaf_shapes = 32;

/// A collector that stores the information we need from a leaf shape in an array that is usually on the stack but can fall back to the heap if needed
const LeafShapeCollector = struct {
    pub const overrides = .{.addHit};

    base: TransformedShapeCollector,
    /// The local buffer of `hits` (STLLocalAllocator<LeafShape, cMaxLocalLeafShapes>)
    local_allocator: STLLocalAllocator(LeafShape, max_local_leaf_shapes),
    hits: std.ArrayList(LeafShape) = .empty,
    /// The first allocation failure of addHit
    alloc_error: ?Allocator.Error = null,

    /// Constructor, in place: `hits` uses the buffer inside this collector, it must not be moved
    fn init(self: *LeafShapeCollector, allocator: Allocator) void {
        self.* = .{ .base = .init(LeafShapeCollector), .local_allocator = .init(allocator) };

        // mHits.reserve(cMaxLocalLeafShapes): takes the local buffer (cannot fail, the array grows on demand otherwise)
        self.hits.ensureTotalCapacityPrecise(self.local_allocator.allocator(), max_local_leaf_shapes) catch {};
    }

    fn deinit(self: *LeafShapeCollector) void {
        self.hits.deinit(self.local_allocator.allocator());
    }

    pub fn addHit(self: *LeafShapeCollector, shape: *const TransformedShape) void {
        // Leaves after an allocation failure are dropped, the collection is stopping
        if (self.alloc_error != null)
            return;

        self.hits.append(self.local_allocator.allocator(), .{
            .bounds = shape.getWorldSpaceBounds(),
            .center_of_mass_transform = shape.getCenterOfMassTransform().toMat44(),
            .scale = shape.getShapeScale(),
            .shape = shape.shape.get().?,
            .sub_shape_id_creator = shape.sub_shape_id_creator,
        }) catch |err| {
            self.alloc_error = err;
            self.base.forceEarlyOut();
        };
    }
};

/// Collide 2 shapes and returns at most 1 hit per leaf shape pairs that overlapping. This can be used when not all contacts between the shapes are needed.
/// E.g. when testing a compound with 2 MeshShapes A and B against a compound with 2 SphereShapes C and D, then at most you'll get 4 collisions: AC, AD, BC, BD.
/// The default CollisionDispatch::sCollideShapeVsShape function would return all intersecting triangles in A against C, all in B against C etc.
/// @param LeafCollector The type of the collector that will be used to collect hits between leaf pairs. Must be either AnyHitCollisionCollector(CollideShapeCollector) to get any hit (cheapest) or ClosestHitCollisionCollector(CollideShapeCollector) to get the deepest hit (more expensive).
/// @param allocator The heap that the list of leaf shapes falls back to when there are more than 32 leaves (Zolt addition)
/// @param shape1 The first shape
/// @param shape2 The second shape
/// @param scale1 Local space scale of shape 1 (scales relative to its center of mass)
/// @param scale2 Local space scale of shape 2 (scales relative to its center of mass)
/// @param center_of_mass_transform1 Transform to transform center of mass of shape 1 into world space
/// @param center_of_mass_transform2 Transform to transform center of mass of shape 2 into world space
/// @param sub_shape_id_creator1 Class that tracks the current sub shape ID for shape 1
/// @param sub_shape_id_creator2 Class that tracks the current sub shape ID for shape 2
/// @param collide_shape_settings Options for the CollideShape test
/// @param collector The collector that receives the results.
/// @param shape_filter allows selectively disabling collisions between pairs of (sub) shapes.
pub fn collideShapeVsShapePerLeaf(comptime LeafCollector: type, allocator: Allocator, shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
    // Get bounds of both shapes
    const bounds1 = shape1.getWorldSpaceBounds(center_of_mass_transform1, scale1);
    const bounds2 = shape2.getWorldSpaceBounds(center_of_mass_transform2, scale2);

    // Get leaf shapes that overlap with the bounds of the other shape
    var leaf_shapes1: LeafShapeCollector = undefined;
    leaf_shapes1.init(allocator);
    defer leaf_shapes1.deinit();
    var leaf_shapes2: LeafShapeCollector = undefined;
    leaf_shapes2.init(allocator);
    defer leaf_shapes2.deinit();
    shape1.collectTransformedShapes(bounds2, center_of_mass_transform1.getTranslation(), center_of_mass_transform1.getQuaternion(), scale1, sub_shape_id_creator1, &leaf_shapes1.base, shape_filter);
    if (leaf_shapes1.alloc_error) |err| return err;
    shape2.collectTransformedShapes(bounds1, center_of_mass_transform2.getTranslation(), center_of_mass_transform2.getQuaternion(), scale2, sub_shape_id_creator2, &leaf_shapes2.base, shape_filter);
    if (leaf_shapes2.alloc_error) |err| return err;

    // Now test each leaf shape against each other leaf
    for (leaf_shapes1.hits.items) |*leaf1| {
        for (leaf_shapes2.hits.items) |*leaf2| {
            if (leaf1.bounds.overlaps(leaf2.bounds)) {
                // Use the leaf collector to collect max 1 hit for this pair and pass it on to collector
                var leaf_collector = LeafCollector.init();
                defer leaf_collector.deinit();
                CollisionDispatch.collideShapeVsShape(leaf1.shape, leaf2.shape, leaf1.scale, leaf2.scale, leaf1.center_of_mass_transform, leaf2.center_of_mass_transform, leaf1.sub_shape_id_creator, leaf2.sub_shape_id_creator, collide_shape_settings, &leaf_collector.base, shape_filter);
                if (leaf_collector.hadHit())
                    collector.addHit(&leaf_collector.hit);
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/TrianglesParity.zig). The compounds are the test
// shapes of Shape/TestShapes.zig (registered as User1..User3 in the inline tests).

const testing = std.testing;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const CollisionCollectorImpl = @import("CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const AnyHitCollisionCollector = CollisionCollectorImpl.AnyHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const TestShapes = @import("Shape/TestShapes.zig");
const SphereShape = @import("Shape/SphereShape.zig").SphereShape;
const BoxShape = @import("Shape/BoxShape.zig").BoxShape;

/// A binary tree of TestCompoundShapes with 2^depth TestBoxShape leaves along the X axis (leaf i at x = 2 * i)
fn createBoxTree(allocator: Allocator, depth: u32, first_leaf: u32) Allocator.Error!*const Shape {
    if (depth == 0)
        return (try TestShapes.TestBoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape();
    const half: u32 = @as(u32, 1) << @intCast(depth - 1);
    const left = try createBoxTree(allocator, depth - 1, first_leaf);
    errdefer {
        var r = RefConst(Shape).init(left);
        r.deinit();
    }
    const right = try createBoxTree(allocator, depth - 1, first_leaf + half);
    errdefer {
        var r = RefConst(Shape).init(right);
        r.deinit();
    }
    // Children are positioned relative to the center of their parent's box range: the leaves stay at 2 * i
    const center_left = 2.0 * (@as(f32, @floatFromInt(first_leaf)) + 0.5 * @as(f32, @floatFromInt(half - 1)));
    const center_right = 2.0 * (@as(f32, @floatFromInt(first_leaf + half)) + 0.5 * @as(f32, @floatFromInt(half - 1)));
    const center = 0.5 * (center_left + center_right);
    return (try TestShapes.TestCompoundShape.create(allocator, left, Vec3.init(center_left - center, 0, 0), right, Vec3.init(center_right - center, 0, 0))).asShape();
}

test "collideShapeVsShapePerLeaf: one hit per overlapping leaf pair (AnyHit and ClosestHit)" {
    const allocator = testing.allocator;

    // A compound of 2 boxes (at x = 0 and x = 2) against a big box that overlaps both: 2 leaf pairs
    const tree = try createBoxTree(allocator, 1, 0);
    var tree_ref = RefConst(Shape).init(tree);
    defer tree_ref.deinit();
    var big = TestShapes.TestBoxShape.init(allocator, Vec3.init(2, 0.5, 0.5), .{});
    big.asShape().setEmbedded();
    defer big.asShapeMut().deinit();

    const settings: CollideShapeSettings = .{};
    var all_hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer all_hits.deinit();
    const transform1 = Mat44.identity(); // The leaves are at x = -1 and x = 1
    const transform2 = Mat44.translation(Vec3.init(0, 0.25, 0));
    try collideShapeVsShapePerLeaf(AnyHitCollisionCollector(CollideShapeCollector), allocator, tree, big.asShape(), Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, &all_hits.base, &.{});
    try all_hits.checkError();
    try testing.expectEqual(@as(usize, 2), all_hits.hits.items.len);
    try testing.expectEqual(SubShapeIDCreator.pushID(.{}, 0, 1).getID().getValue(), all_hits.hits.items[0].sub_shape_id1.getValue());
    try testing.expectEqual(SubShapeIDCreator.pushID(.{}, 1, 1).getID().getValue(), all_hits.hits.items[1].sub_shape_id1.getValue());

    // The closest hit version gives the same pairs, the deepest hit of each
    all_hits.reset();
    try collideShapeVsShapePerLeaf(ClosestHitCollisionCollector(CollideShapeCollector), allocator, tree, big.asShape(), Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, &all_hits.base, &.{});
    try all_hits.checkError();
    try testing.expectEqual(@as(usize, 2), all_hits.hits.items.len);
    for (all_hits.hits.items) |hit|
        try testing.expectApproxEqAbs(@as(f32, 0.75), hit.penetration_depth, 1.0e-5);

    // Moved away: no pairs
    all_hits.reset();
    try collideShapeVsShapePerLeaf(AnyHitCollisionCollector(CollideShapeCollector), allocator, tree, big.asShape(), Vec3.one(), Vec3.one(), transform1, Mat44.translation(Vec3.init(0, 3, 0)), .{}, .{}, &settings, &all_hits.base, &.{});
    try testing.expectEqual(@as(usize, 0), all_hits.hits.items.len);

    // Two real convex shapes (a single leaf each)
    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    try collideShapeVsShapePerLeaf(ClosestHitCollisionCollector(CollideShapeCollector), allocator, sphere.asShape(), box.asShape(), Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(1.5, 0, 0)), Mat44.identity(), .{}, .{}, &settings, &all_hits.base, &.{});
    try all_hits.checkError();
    try testing.expectEqual(@as(usize, 1), all_hits.hits.items.len);
    try testing.expectApproxEqAbs(@as(f32, 0.5), all_hits.hits.items[0].penetration_depth, 1.0e-5);
}

test "collideShapeVsShapePerLeaf: more than 32 leaves (heap fallback) and out of memory" {
    const allocator = testing.allocator;

    // 64 leaves against a box that overlaps all of them
    const tree = try createBoxTree(allocator, 6, 0);
    var tree_ref = RefConst(Shape).init(tree);
    defer tree_ref.deinit();
    var big = TestShapes.TestBoxShape.init(allocator, Vec3.init(100, 0.5, 0.5), .{});
    big.asShape().setEmbedded();
    defer big.asShapeMut().deinit();
    const settings: CollideShapeSettings = .{};
    const transform1 = Mat44.identity(); // The leaves are at x = -63, -61, ..., 63
    const transform2 = Mat44.translation(Vec3.init(0, 0.5, 0));

    var all_hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer all_hits.deinit();
    try collideShapeVsShapePerLeaf(AnyHitCollisionCollector(CollideShapeCollector), allocator, tree, big.asShape(), Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, &all_hits.base, &.{});
    try all_hits.checkError();
    try testing.expectEqual(@as(usize, 64), all_hits.hits.items.len);
    for (all_hits.hits.items, 0..) |hit, i| {
        // Every leaf once, in the order of the tree (the sub shape ID is the path of child indices from the root)
        var expected = SubShapeIDCreator{};
        for (0..6) |level| expected = expected.pushID(@intCast((i >> @intCast(5 - level)) & 1), 1);
        try testing.expectEqual(expected.getID().getValue(), hit.sub_shape_id1.getValue());
    }

    // Out of memory in the heap fallback: reported, no hits
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        all_hits.reset();
        collideShapeVsShapePerLeaf(AnyHitCollisionCollector(CollideShapeCollector), failing.allocator(), tree, big.asShape(), Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, &all_hits.base, &.{}) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expectEqual(@as(usize, 0), all_hits.hits.items.len);
            continue;
        };
        try testing.expectEqual(@as(usize, 64), all_hits.hits.items.len);
        try testing.expect(fail_index > 0); // At least one heap allocation
        break;
    }
}
