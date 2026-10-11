//! Port of: UnitTests/Physics/OffsetCenterOfMassShapeTests.cpp
//! Status: partial
//! Missing: TestAddAngularImpulseCOMZero, TestAddAngularImpulseCOMOffset, TestAddTorqueCOMZero, TestAddTorqueCOMOffset
//!   (need PhysicsTestContext, Phase 5)
//!
//! Every TEST_CASE of this file creates a body and applies an impulse or a torque, which needs PhysicsTestContext. Until
//! then the test below checks the part of them that only needs the shape: the mass and the inertia that the body
//! derives from OffsetCenterOfMassShape.GetMassProperties (same inputs and expected values as Jolt's tests).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const BoxShapeSettings = zolt.BoxShapeSettings;
const OffsetCenterOfMassShape = zolt.OffsetCenterOfMassShape;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

/// The mass and the inertia around Y of a box with half extent `half_extent` and its center of mass moved by
/// `com_offset` (the "Check mass and inertia calculated correctly" part of the TEST_CASEs)
fn checkMassAndInertia(com_offset: Vec3) !void {
    // Create box
    const half_extent = Vec3.init(0.5, 1.0, 1.5);
    var box = BoxShapeSettings.init(allocator, half_extent, .{});
    box.asShapeSettings().setEmbedded();
    defer box.deinit();

    // Create shape with COM offset
    var com = OffsetCenterOfMassShapeSettings.init(allocator, com_offset, box.asShapeSettings());
    com.asShapeSettings().setEmbedded();
    defer com.deinit();
    var result = try com.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    try fw.expect(shape.cast(OffsetCenterOfMassShape).getOffset().eql(com_offset));
    try fw.expect(shape.getCenterOfMass().eql(com_offset));

    // Check mass and inertia calculated correctly
    const p = shape.getMassProperties();
    const mass = (8.0 * half_extent.getX() * half_extent.getY() * half_extent.getZ()) * box.base.density;
    try fw.checkApproxEqual(1.0 / p.mass, 1.0 / mass, .{});
    const inertia_y = mass / 12.0 * (math.square(2.0 * half_extent.getX()) + math.square(2.0 * half_extent.getZ())) + mass * math.square(com_offset.getX()); // See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia & https://en.wikipedia.org/wiki/Parallel_axis_theorem
    try fw.checkApproxEqual(1.0 / p.inertia.get(1, 1), 1.0 / inertia_y, .{});

    // The inertia is diagonal (the offset is along X), so the body's inverse inertia around Y is 1 / inertia_y
    for (0..3) |r| {
        for (0..3) |c| {
            if (r != c) try fw.expectEqual(@as(f32, 0.0), p.inertia.get(@intCast(r), @intCast(c)));
        }
    }
}

test "OffsetCenterOfMassShapeTests: mass and inertia of the shape (the shape part of TestAddAngularImpulseCOMZero / TestAddTorqueCOMZero)" {
    // Create body with COM offset 0
    try checkMassAndInertia(Vec3.zero());
}

test "OffsetCenterOfMassShapeTests: mass and inertia of the shape (the shape part of TestAddAngularImpulseCOMOffset / TestAddTorqueCOMOffset)" {
    // Create body with COM offset
    try checkMassAndInertia(Vec3.init(5.0, 0, 0));
}
