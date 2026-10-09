//! Port of: UnitTests/Physics/ShapeTests.cpp
//! Status: partial
//! Missing: TestConvexHullShape, TestCapsuleVsConvexHullInertia, TestIsValidScale, TestEmbeddedShape,
//!   TestIsValidSubShapeID, TestOutOfSubShapeIDBits, TestEmptyMutableCompound, TestSaveMeshShape,
//!   TestMeshShapePerTriangleUserData (need ConvexHullShape, CapsuleShape, TaperedCapsuleShape, CylinderShape,
//!   TaperedCylinderShape, TriangleShape, ScaledShape, RotatedTranslatedShape, the compound shapes, MeshShape and
//!   HeightFieldShape, Phase 4 Wave A / B), TestBoxShape, TestCylinderShape, TestTaperedCylinderShape (need
//!   PhysicsTestContext, Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const BoxShape = zolt.BoxShape;
const Mat44 = zolt.Mat44;
const Plane = zolt.Plane;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;

// Test re-creating shape using the same settings object
test "TestClearCachedResult" {
    // Create a sphere and check radius
    var sphere_settings = SphereShapeSettings.init(allocator, 1.0, .{});
    defer sphere_settings.deinit();
    var result1 = try sphere_settings.asShapeSettings().createShape(allocator);
    defer result1.deinit();
    var sphere1 = RefConst(Shape).init(result1.getPtr()); // StaticCast<SphereShape>: a RefConst(Shape), cast on use
    defer sphere1.deinit();
    try fw.expect(sphere1.get().?.cast(SphereShape).getRadius() == 1.0);

    // Modify radius and check that creating the shape again returns the cached result
    sphere_settings.radius = 2.0;
    var result2 = try sphere_settings.asShapeSettings().createShape(allocator);
    defer result2.deinit();
    var sphere2 = RefConst(Shape).init(result2.getPtr()); // StaticCast<SphereShape>: a RefConst(Shape), cast on use
    defer sphere2.deinit();
    try fw.expect(sphere2.get() == sphere1.get());

    sphere_settings.asShapeSettings().clearCachedResult();
    var result3 = try sphere_settings.asShapeSettings().createShape(allocator);
    defer result3.deinit();
    var sphere3 = RefConst(Shape).init(result3.getPtr()); // StaticCast<SphereShape>: a RefConst(Shape), cast on use
    defer sphere3.deinit();
    try fw.expect(sphere3.get().?.cast(SphereShape).getRadius() == 2.0);
}

// Test submerged volume calculation
test "TestGetSubmergedVolume" {
    var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(1, 2, 3), .{})).asShape());
    defer box_ref.deinit();
    const box = box_ref.get().?;
    const scale = Vec3.init(2, -3, 4);
    const translation = Mat44.translation(Vec3.init(0, 6, 0)); // Translate so we're on the y = 0 plane

    // Plane pointing positive Y
    // Entirely above the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, -0.001, 0), Vec3.axisY()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 0.0, .{});
    }

    // Entirely below the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 12.001, 0), Vec3.axisY()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 0), .{});
    }

    // Halfway through
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 6.0, 0), Vec3.axisY()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 6.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 3, 0), .{});
    }

    // Plane pointing negative Y
    // Entirely above the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(-4, 12.001, 0), Vec3.axisY().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 0.0, .{});
    }

    // Entirely below the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, -0.001, 0), Vec3.axisY().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 0), .{});
    }

    // Halfway through
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 6.0, 0), Vec3.axisY().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 6.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 9, 0), .{});
    }

    // Plane pointing positive X
    // Entirely above the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(-2.001, 0, 0), Vec3.axisX()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 0.0, .{});
    }

    // Entirely below the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(2.001, 0, 0), Vec3.axisX()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 0), .{});
    }

    // Halfway through
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, 0), Vec3.axisX()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 2.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(-1, 6, 0), .{});
    }

    // Plane pointing negative X
    // Entirely above the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(2.001, 0, 0), Vec3.axisX().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 0.0, .{});
    }

    // Entirely below the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(-2.001, 0, 0), Vec3.axisX().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 0), .{});
    }

    // Halfway through
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, 0), Vec3.axisX().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 2.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(1, 6, 0), .{});
    }

    // Plane pointing positive Z
    // Entirely above the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, -12.001), Vec3.axisZ()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 0.0, .{});
    }

    // Entirely below the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, 12.001), Vec3.axisZ()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 0), .{});
    }

    // Halfway through
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, 0), Vec3.axisZ()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 12.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, -6), .{});
    }

    // Plane pointing negative Z
    // Entirely above the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, 12.001), Vec3.axisZ().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 0.0, .{});
    }

    // Entirely below the plane
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, -12.001), Vec3.axisZ().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 0), .{});
    }

    // Halfway through
    {
        const r = box.getSubmergedVolume(translation, scale, Plane.fromPointAndNormal(Vec3.init(0, 0, 0), Vec3.axisZ().negate()));
        try fw.checkApproxEqual(r.total_volume, 4.0 * 12.0 * 24.0, .{});
        try fw.checkApproxEqual(r.submerged_volume, 4.0 * 12.0 * 12.0, .{});
        try fw.checkApproxEqual(r.center_of_buoyancy, Vec3.init(0, 6, 6), .{});
    }
}

// Test setting user data on shapes
test "TestShapeUserData" {
    const radius: f32 = 2.0;

    // Create a sphere with user data
    var sphere_settings = SphereShapeSettings.init(allocator, radius, .{});
    defer sphere_settings.deinit();
    sphere_settings.asShapeSettings().user_data = 0x1234567887654321;
    var sphere: Ref(Shape) = .empty;
    defer sphere.deinit();
    {
        var result = try sphere_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        sphere.set(result.getPtr().?);
    }
    try fw.expect(sphere.get().?.getUserData() == 0x1234567887654321);

    // Change the user data
    sphere.get().?.setUserData(0x5678123443218765);
    try fw.expect(sphere.get().?.getUserData() == 0x5678123443218765);

    var data: [256]u8 = undefined;

    // Write sphere to a binary stream
    var writer: std.Io.Writer = .fixed(&data);
    {
        var stream_out = StreamOutWrapper.init(&writer);
        sphere.get().?.saveBinaryState(stream_out.streamOut());
    }

    // Destroy the sphere
    sphere.set(null);

    // Read sphere from binary stream
    {
        var reader: std.Io.Reader = .fixed(writer.buffered());
        var stream_in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
        defer result.deinit();
        sphere.set(result.getPtr().?);
    }

    // Check that the sphere and its user data was preserved
    try fw.expect(sphere.get().?.getType() == .convex);
    try fw.expect(sphere.get().?.getSubType() == .sphere);
    try fw.expect(sphere.get().?.getUserData() == 0x5678123443218765);
    try fw.expect(sphere.get().?.cast(SphereShape).getRadius() == radius);
}
