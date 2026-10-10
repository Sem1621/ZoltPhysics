//! Port of: UnitTests/Physics/MutableCompoundShapeTests.cpp
//! Status: partial
//! Missing: TestEmptyMutableCompoundShape (needs PhysicsTestContext, Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const CollidePointCollector = zolt.CollidePointCollector;
const MutableCompoundShape = zolt.MutableCompoundShape;
const MutableCompoundShapeSettings = zolt.MutableCompoundShapeSettings;
const Quat = zolt.Quat;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;

/// The `check_shape_hit` lambda: the sub shape that contains `position` (at most one) or null
fn checkShapeHit(shape: *const MutableCompoundShape, position: Vec3) !?*const Shape {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    shape.asShape().collidePoint(position.sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len <= 1);
    return if (collector.hits.items.len != 0) shape.base.getSubShape(shape.base.getSubShapeIndexFromID(collector.hits.items[0].sub_shape_id2).index).shape.get() else null;
}

/// new SphereShape(radius) held by a Ref<Shape>
fn newSphere(radius: f32) !RefConst(Shape) {
    return .init((try SphereShape.create(allocator, radius, .{})).asShape());
}

test "TestMutableCompoundShapeAddRemove" {
    var settings = MutableCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    var sphere1 = try newSphere(1.0);
    defer sphere1.deinit();
    try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere1.get(), .{});
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.castMut(MutableCompoundShape);

    try fw.expect(shape.base.getNumSubShapes() == 1);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere1.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == sphere1.get());

    var sphere2 = try newSphere(2.0);
    defer sphere2.deinit();
    _ = try shape.addShape(Vec3.init(10, 0, 0), Quat.identity(), sphere2.get().?, .{ .user_data = 0, .index = 0 }); // Insert at the start
    try fw.expect(shape.base.getNumSubShapes() == 2);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere1.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-1, -2, -2), Vec3.init(12, 2, 2))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == sphere1.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());

    var sphere3 = try newSphere(3.0);
    defer sphere3.deinit();
    _ = try shape.addShape(Vec3.init(20, 0, 0), Quat.identity(), sphere3.get().?, .{ .user_data = 0, .index = 2 }); // Insert at the end
    try fw.expect(shape.base.getNumSubShapes() == 3);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere1.get());
    try fw.expect(shape.base.getSubShape(2).shape.get() == sphere3.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-1, -3, -3), Vec3.init(23, 3, 3))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == sphere1.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == sphere3.get());

    shape.removeShape(1);
    try fw.expect(shape.base.getNumSubShapes() == 2);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere3.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(8, -3, -3), Vec3.init(23, 3, 3))));
    try fw.expect(try checkShapeHit(shape, Vec3.init(0, 0, 0)) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == sphere3.get());

    var sphere4 = try newSphere(4.0);
    defer sphere4.deinit();
    _ = try shape.addShape(Vec3.init(0, 0, 0), Quat.identity(), sphere4.get().?, .{ .user_data = 0 }); // Insert at the end
    try fw.expect(shape.base.getNumSubShapes() == 3);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere3.get());
    try fw.expect(shape.base.getSubShape(2).shape.get() == sphere4.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-4, -4, -4), Vec3.init(23, 4, 4))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == sphere4.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == sphere3.get());

    var sphere5 = try newSphere(1.0);
    defer sphere5.deinit();
    _ = try shape.addShape(Vec3.init(15, 0, 0), Quat.identity(), sphere5.get().?, .{ .user_data = 0, .index = 1 }); // Insert in the middle
    try fw.expect(shape.base.getNumSubShapes() == 4);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere5.get());
    try fw.expect(shape.base.getSubShape(2).shape.get() == sphere3.get());
    try fw.expect(shape.base.getSubShape(3).shape.get() == sphere4.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-4, -4, -4), Vec3.init(23, 4, 4))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == sphere4.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(15, 0, 0)) == sphere5.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == sphere3.get());

    shape.removeShape(3);
    try fw.expect(shape.base.getNumSubShapes() == 3);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere5.get());
    try fw.expect(shape.base.getSubShape(2).shape.get() == sphere3.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(8, -3, -3), Vec3.init(23, 3, 3))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(15, 0, 0)) == sphere5.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == sphere3.get());

    shape.removeShape(1);
    try fw.expect(shape.base.getNumSubShapes() == 2);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.base.getSubShape(1).shape.get() == sphere3.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(8, -3, -3), Vec3.init(23, 3, 3))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(15, 0, 0)) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == sphere3.get());

    shape.removeShape(1);
    try fw.expect(shape.base.getNumSubShapes() == 1);
    try fw.expect(shape.base.getSubShape(0).shape.get() == sphere2.get());
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(8, -2, -2), Vec3.init(12, 2, 2))));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == sphere2.get());
    try fw.expect(try checkShapeHit(shape, Vec3.init(15, 0, 0)) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == null);

    shape.removeShape(0);
    try fw.expect(shape.base.getNumSubShapes() == 0);
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.zero(), Vec3.zero())));
    try fw.expect(try checkShapeHit(shape, Vec3.zero()) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(10, 0, 0)) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(15, 0, 0)) == null);
    try fw.expect(try checkShapeHit(shape, Vec3.init(20, 0, 0)) == null);
}

test "TestMutableCompoundShapeAdjustCenterOfMass" {
    // Start with a box at (-1 0 0)
    var settings = MutableCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    const box1 = try BoxShape.create(allocator, Vec3.one(), .{});
    var box_shape1 = RefConst(Shape).init(box1.asShape());
    defer box_shape1.deinit();
    box1.asShapeMut().setUserData(1);
    try settings.base.addShapePtr(Vec3.init(-1.0, 0.0, 0.0), Quat.identity(), box_shape1.get(), .{});
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.castMut(MutableCompoundShape);
    try fw.expect(shape.asShape().getCenterOfMass().eql(Vec3.init(-1.0, 0.0, 0.0)));
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.replicate(-1.0), Vec3.one())));

    // Check that we can hit the box
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    shape.asShape().collidePoint(Vec3.init(-0.5, 0.0, 0.0).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1 and shape.asShape().getSubShapeUserData(collector.hits.items[0].sub_shape_id2) == 1);
    collector.base.reset();
    try fw.expect(collector.hits.items.len == 0);

    // Now add another box at (1 0 0)
    const box2 = try BoxShape.create(allocator, Vec3.one(), .{});
    var box_shape2 = RefConst(Shape).init(box2.asShape());
    defer box_shape2.deinit();
    box2.asShapeMut().setUserData(2);
    _ = try shape.addShape(Vec3.init(1.0, 0.0, 0.0), Quat.identity(), box_shape2.get().?, .{});
    try fw.expect(shape.asShape().getCenterOfMass().eql(Vec3.init(-1.0, 0.0, 0.0)));
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-1.0, -1.0, -1.0), Vec3.init(3.0, 1.0, 1.0))));

    // Check that we can hit both boxes
    shape.asShape().collidePoint(Vec3.init(-0.5, 0.0, 0.0).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1 and shape.asShape().getSubShapeUserData(collector.hits.items[0].sub_shape_id2) == 1);
    collector.base.reset();
    shape.asShape().collidePoint(Vec3.init(0.5, 0.0, 0.0).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1 and shape.asShape().getSubShapeUserData(collector.hits.items[0].sub_shape_id2) == 2);
    collector.base.reset();

    // Adjust the center of mass
    shape.adjustCenterOfMass();
    try fw.expect(shape.asShape().getCenterOfMass().eql(Vec3.zero()));
    try fw.expect(shape.asShape().getLocalBounds().eql(AABox.init(Vec3.init(-2.0, -1.0, -1.0), Vec3.init(2.0, 1.0, 1.0))));

    // Check that we can hit both boxes
    shape.asShape().collidePoint(Vec3.init(-0.5, 0.0, 0.0).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1 and shape.asShape().getSubShapeUserData(collector.hits.items[0].sub_shape_id2) == 1);
    collector.base.reset();
    shape.asShape().collidePoint(Vec3.init(0.5, 0.0, 0.0).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1 and shape.asShape().getSubShapeUserData(collector.hits.items[0].sub_shape_id2) == 2);
    collector.base.reset();
}

// TestEmptyMutableCompoundShape: not ported yet, needs PhysicsTestContext (Phase 5)
