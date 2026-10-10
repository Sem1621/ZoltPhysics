//! Port of: UnitTests/Physics/ShapeTests.cpp
//! Status: partial
//! Missing: the PhysicsTestContext parts of TestBoxShape, TestCylinderShape and TestTaperedCylinderShape (a zero sized
//!   shape resting on a floor; need PhysicsTestContext, Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CapsuleShape = zolt.CapsuleShape;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollidePointCollector = zolt.CollidePointCollector;
const CollisionDispatch = zolt.CollisionDispatch;
const CompoundShape = zolt.CompoundShape;
const ConvexHullShape = zolt.ConvexHullShape;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const ConvexShape = zolt.ConvexShape;
const CylinderShape = zolt.CylinderShape;
const CylinderShapeSettings = zolt.CylinderShapeSettings;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const HeightFieldShapeSettings = zolt.HeightFieldShapeSettings;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const MeshShape = zolt.MeshShape;
const MeshShapeSettings = zolt.MeshShapeSettings;
const MutableCompoundShape = zolt.MutableCompoundShape;
const MutableCompoundShapeSettings = zolt.MutableCompoundShapeSettings;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const ScaledShape = zolt.ScaledShape;
const ScaleHelpers = zolt.ScaleHelpers;
const Shape = zolt.Shape;
const ShapeResult = zolt.ShapeResult;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const TaperedCapsuleShapeSettings = zolt.TaperedCapsuleShapeSettings;
const TaperedCylinderShape = zolt.TaperedCylinderShape;
const TaperedCylinderShapeSettings = zolt.TaperedCylinderShapeSettings;
const Triangle = zolt.Triangle;
const TriangleList = zolt.TriangleList;
const TriangleShape = zolt.TriangleShape;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

/// `settings.Create().Get()`: the shape of a newly created result (the caller releases the reference)
fn createShape(settings: *zolt.ShapeSettings) !RefConst(Shape) {
    var result = try settings.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr());
}

// Test convex hull shape
test "TestConvexHullShape" {
    const density: f32 = 1.5;

    // Create convex hull shape of a box
    const box = [_]Vec3{
        Vec3.init(5, 6, 7),
        Vec3.init(5, 6, 14),
        Vec3.init(5, 12, 7),
        Vec3.init(5, 12, 14),
        Vec3.init(10, 6, 7),
        Vec3.init(10, 6, 14),
        Vec3.init(10, 12, 7),
        Vec3.init(10, 12, 14),
    };
    var settings = try ConvexHullShapeSettings.init(allocator, &box, .{});
    defer settings.deinit();
    settings.base.setDensity(density);
    var shape_ref = try createShape(settings.asShapeSettings());
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;

    // Validate calculated center of mass
    const com = shape.getCenterOfMass();
    try fw.checkApproxEqual(Vec3.init(7.5, 9.0, 10.5), com, .{ .tolerance = 1.0e-5 });

    // Calculate reference value of mass and inertia of a box
    var reference: MassProperties = .{};
    reference.setMassAndInertiaOfSolidBox(Vec3.init(5, 6, 7), density);

    // Mass is easy to calculate, double check if SetMassAndInertiaOfSolidBox calculated it correctly
    try fw.checkApproxEqual(5.0 * 6.0 * 7.0 * density, reference.mass, .{ .tolerance = 1.0e-6 });

    // Get calculated inertia tensor
    const m = shape.getMassProperties();
    try fw.checkApproxEqual(reference.mass, m.mass, .{ .tolerance = 1.0e-6 });
    try fw.checkApproxEqual(reference.inertia, m.inertia, .{ .tolerance = 1.0e-4 });

    // Check inner radius
    try fw.checkApproxEqual(shape.getInnerRadius(), 2.5, .{});
}

// Test inertia calculations for a capsule vs that of a convex hull of a capsule
test "TestCapsuleVsConvexHullInertia" {
    const half_height: f32 = 5.0;
    const radius: f32 = 3.0;

    // Create a capsule
    var capsule = CapsuleShape.init(allocator, half_height, radius, .{});
    capsule.base.setDensity(7.0);
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();
    const mp_capsule = capsule.asShape().getMassProperties();

    // Verify mass
    const mass_cylinder = 2.0 * half_height * math.pi * math.square(radius) * capsule.base.getDensity();
    const mass_sphere = @as(f32, 4.0) / 3.0 * math.pi * math.cubed(radius) * capsule.base.getDensity();
    try fw.checkApproxEqual(mp_capsule.mass, mass_cylinder + mass_sphere, .{});

    // Extract support points
    var buffer: ConvexShape.SupportBuffer = .{};
    const support = capsule.base.getSupportFunction(.include_convex_radius, &buffer, Vec3.one());
    var capsule_points: std.ArrayList(Vec3) = .empty;
    defer capsule_points.deinit(allocator);
    try capsule_points.ensureTotalCapacity(allocator, Vec3.unit_sphere.len);
    for (Vec3.unit_sphere.constSlice()) |v|
        capsule_points.appendAssumeCapacity(support.getSupport(v));

    // Create a convex hull using the support points
    var capsule_hull = try ConvexHullShapeSettings.init(allocator, capsule_points.items, .{});
    defer capsule_hull.deinit();
    capsule_hull.base.setDensity(capsule.base.getDensity());
    var capsule_hull_shape = try createShape(capsule_hull.asShapeSettings());
    defer capsule_hull_shape.deinit();
    const mp_capsule_hull = capsule_hull_shape.get().?.getMassProperties();

    // Check that the mass and inertia of the convex hull match that of the capsule (within certain tolerance since the convex hull is an approximation)
    const mass_error = (mp_capsule_hull.mass - mp_capsule.mass) / mp_capsule.mass;
    try fw.expect(mass_error > -0.05);
    try fw.expect(mass_error < 0.0); // Mass is smaller since the convex hull is smaller
    for (0..3) |i| {
        for (0..3) |j| {
            const row: u32 = @intCast(i);
            const col: u32 = @intCast(j);
            if (i == j) {
                const inertia_error = (mp_capsule_hull.inertia.get(row, col) - mp_capsule.inertia.get(row, col)) / mp_capsule.inertia.get(row, col);
                try fw.expect(inertia_error > -0.05);
                try fw.expect(inertia_error < 0.0); // Inertia is smaller since the convex hull is smaller
            } else {
                try fw.expect(mp_capsule.inertia.get(row, col) == 0.0);
                const scaled_inertia = mp_capsule_hull.inertia.get(row, col) / mp_capsule_hull.mass;
                try fw.checkApproxEqual(scaled_inertia, 0.0, .{ .tolerance = 1.0e-3 });
            }
        }
    }
}

/// `compound_settings.AddShape(position, rotation, new ScaledShape(shape, scale))`: the settings take the only reference
fn addScaledShape(settings: *zolt.CompoundShapeSettings, position: Vec3, rotation: Quat, shape: *const Shape, scale: Vec3) !void {
    const scaled = try ScaledShape.create(allocator, shape, scale);
    var scaled_ref = RefConst(Shape).init(scaled.asShape());
    defer scaled_ref.deinit();
    try settings.addShapePtr(position, rotation, scaled.asShape(), .{});
}

// Test IsValidScale function
test "TestIsValidScale" {
    const min_scale_tolerance_sq: f32 = math.square(@as(f32, 1.0e-6) * ScaleHelpers.min_scale);

    // Test simple shapes
    var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 2.0, .{})).asShape());
    defer sphere_ref.deinit();
    const sphere = sphere_ref.get().?;
    try fw.expect(!sphere.isValidScale(Vec3.zero()));
    try fw.expect(sphere.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(sphere.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!sphere.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!sphere.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!sphere.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(sphere.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq })); // Averaging can cause a slight error
    try fw.expect(sphere.makeScaleValid(Vec3.init(-2, 3, 4)).eql(Vec3.init(-3, 3, 3)));

    var capsule_ref = RefConst(Shape).init((try CapsuleShape.create(allocator, 2.0, 0.5, .{})).asShape());
    defer capsule_ref.deinit();
    const capsule = capsule_ref.get().?;
    try fw.expect(!capsule.isValidScale(Vec3.zero()));
    try fw.expect(!capsule.isValidScale(Vec3.init(0, 1, 0)));
    try fw.expect(!capsule.isValidScale(Vec3.init(1, 0, 1)));
    try fw.expect(capsule.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(capsule.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!capsule.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!capsule.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!capsule.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(capsule.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try fw.expect(capsule.makeScaleValid(Vec3.init(-2, 3, 4)).eql(Vec3.init(-3, 3, 3)));

    var tapered_capsule_settings = TaperedCapsuleShapeSettings.init(allocator, 2.0, 0.5, 0.7, .{});
    defer tapered_capsule_settings.deinit();
    var tapered_capsule_ref = try createShape(tapered_capsule_settings.asShapeSettings());
    defer tapered_capsule_ref.deinit();
    const tapered_capsule = tapered_capsule_ref.get().?;
    try fw.expect(!tapered_capsule.isValidScale(Vec3.zero()));
    try fw.expect(tapered_capsule.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(tapered_capsule.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!tapered_capsule.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!tapered_capsule.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!tapered_capsule.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(tapered_capsule.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try fw.expect(tapered_capsule.makeScaleValid(Vec3.init(2, -3, 4)).eql(Vec3.init(3, -3, 3)));

    var cylinder_ref = RefConst(Shape).init((try CylinderShape.create(allocator, 0.5, 2.0, .{})).asShape());
    defer cylinder_ref.deinit();
    const cylinder = cylinder_ref.get().?;
    try fw.expect(!cylinder.isValidScale(Vec3.zero()));
    try fw.expect(!cylinder.isValidScale(Vec3.init(0, 1, 0)));
    try fw.expect(!cylinder.isValidScale(Vec3.init(1, 0, 1)));
    try fw.expect(cylinder.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(cylinder.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!cylinder.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(cylinder.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!cylinder.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(cylinder.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try fw.expect(cylinder.makeScaleValid(Vec3.init(-1.0e-10, 1, 1.0e-10)).eql(Vec3.init(-ScaleHelpers.min_scale, 1, ScaleHelpers.min_scale)));
    try fw.expect(cylinder.makeScaleValid(Vec3.init(2, 5, -4)).eql(Vec3.init(3, 5, -3)));

    var tapered_cylinder_settings = TaperedCylinderShapeSettings.init(allocator, 0.5, 2.0, 3.0, .{});
    defer tapered_cylinder_settings.deinit();
    var tapered_cylinder_ref = try createShape(tapered_cylinder_settings.asShapeSettings());
    defer tapered_cylinder_ref.deinit();
    const tapered_cylinder = tapered_cylinder_ref.get().?;
    try fw.expect(!tapered_cylinder.isValidScale(Vec3.zero()));
    try fw.expect(!tapered_cylinder.isValidScale(Vec3.init(0, 1, 0)));
    try fw.expect(!tapered_cylinder.isValidScale(Vec3.init(1, 0, 1)));
    try fw.expect(tapered_cylinder.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(tapered_cylinder.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!tapered_cylinder.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(tapered_cylinder.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!tapered_cylinder.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(tapered_cylinder.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try fw.expect(tapered_cylinder.makeScaleValid(Vec3.init(-1.0e-10, 1, 1.0e-10)).eql(Vec3.init(-ScaleHelpers.min_scale, 1, ScaleHelpers.min_scale)));
    try fw.expect(tapered_cylinder.makeScaleValid(Vec3.init(2, 5, -4)).eql(Vec3.init(3, 5, -3)));

    var triangle_ref = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{})).asShape());
    defer triangle_ref.deinit();
    const triangle = triangle_ref.get().?;
    try fw.expect(!triangle.isValidScale(Vec3.zero()));
    try fw.expect(!triangle.isValidScale(Vec3.axisX()));
    try fw.expect(!triangle.isValidScale(Vec3.axisY()));
    try fw.expect(!triangle.isValidScale(Vec3.axisZ()));
    try fw.expect(triangle.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(triangle.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(triangle.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(triangle.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(triangle.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(triangle.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try fw.expect(triangle.makeScaleValid(Vec3.init(2, 5, -4)).eql(Vec3.init(2, 5, -4)));

    var triangle2_ref = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{ .convex_radius = 0.01 })).asShape()); // With convex radius
    defer triangle2_ref.deinit();
    const triangle2 = triangle2_ref.get().?;
    try fw.expect(!triangle2.isValidScale(Vec3.zero()));
    try fw.expect(!triangle2.isValidScale(Vec3.axisX()));
    try fw.expect(!triangle2.isValidScale(Vec3.axisY()));
    try fw.expect(!triangle2.isValidScale(Vec3.axisZ()));
    try fw.expect(triangle2.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(triangle2.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!triangle2.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!triangle2.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!triangle2.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(triangle2.makeScaleValid(Vec3.zero()).isClose(Vec3.replicate(ScaleHelpers.min_scale), .{ .max_dist_sq = min_scale_tolerance_sq }));
    try fw.expect(triangle2.makeScaleValid(Vec3.init(2, 6, -4)).eql(Vec3.init(4, 4, -4)));

    var scaled_ref = RefConst(Shape).init((try ScaledShape.create(allocator, sphere, Vec3.init(1, 2, 1))).asShape());
    defer scaled_ref.deinit();
    const scaled = scaled_ref.get().?;
    try fw.expect(!scaled.isValidScale(Vec3.zero()));
    try fw.expect(!scaled.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(scaled.isValidScale(Vec3.init(1, 0.5, 1)));
    try fw.expect(scaled.isValidScale(Vec3.init(-1, 0.5, 1)));
    try fw.expect(!scaled.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!scaled.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!scaled.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(scaled.makeScaleValid(Vec3.init(3, 3, 3)).eql(Vec3.init(4, 2, 4)));
    try fw.expect(scaled.makeScaleValid(Vec3.init(4, 2, 4)).eql(Vec3.init(4, 2, 4)));

    var scaled2_ref = RefConst(Shape).init((try ScaledShape.create(allocator, scaled, Vec3.init(1, 0.5, 1))).asShape());
    defer scaled2_ref.deinit();
    const scaled2 = scaled2_ref.get().?;
    try fw.expect(!scaled2.isValidScale(Vec3.zero()));
    try fw.expect(scaled2.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(scaled2.isValidScale(Vec3.init(-1, 1, -1)));
    try fw.expect(!scaled2.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!scaled2.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!scaled2.isValidScale(Vec3.init(1, 1, 2)));
    try fw.expect(scaled2.makeScaleValid(Vec3.init(3, 3, 3)).eql(Vec3.init(3, 3, 3)));
    try fw.expect(scaled2.makeScaleValid(Vec3.init(5, 2, 5)).eql(Vec3.init(4, 4, 4)));

    // Test a compound with shapes that can only be scaled uniformly
    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.1 * math.pi), sphere, .{});
    try compound_settings.base.addShapePtr(Vec3.init(4, 5, 6), Quat.rotation(Vec3.axisY(), 0.1 * math.pi), capsule, .{});
    var compound_ref = try createShape(compound_settings.asShapeSettings());
    defer compound_ref.deinit();
    const compound = compound_ref.get().?;
    try fw.expect(!compound.isValidScale(Vec3.zero()));
    try fw.expect(compound.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(compound.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(!compound.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!compound.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!compound.isValidScale(Vec3.init(1, 1, 2)));

    // Test compound containing a triangle shape that can be scaled in any way
    var compound_settings2 = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings2.deinit();
    try compound_settings2.base.addShapePtr(Vec3.init(1, 2, 3), Quat.identity(), triangle, .{});
    try addScaledShape(&compound_settings2.base, Vec3.init(4, 5, 6), Quat.identity(), triangle, Vec3.init(10, 11, 12));
    var compound2_ref = try createShape(compound_settings2.asShapeSettings());
    defer compound2_ref.deinit();
    const compound2 = compound2_ref.get().?;
    try fw.expect(!compound2.isValidScale(Vec3.zero()));
    try fw.expect(compound2.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(compound2.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(compound2.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(compound2.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(compound2.isValidScale(Vec3.init(1, 1, 2)));

    // Test rotations inside the compound of 90 degrees
    var compound_settings3 = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings3.deinit();
    try compound_settings3.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.5 * math.pi), triangle, .{});
    try addScaledShape(&compound_settings3.base, Vec3.init(4, 5, 6), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), triangle, Vec3.init(10, 11, 12));
    var compound3_ref = try createShape(compound_settings3.asShapeSettings());
    defer compound3_ref.deinit();
    const compound3 = compound3_ref.get().?;
    try fw.expect(!compound3.isValidScale(Vec3.zero()));
    try fw.expect(compound3.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(compound3.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(compound3.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(compound3.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(compound3.isValidScale(Vec3.init(1, 1, 2)));

    // Test non-90 degree rotations, this would cause shearing so is not allowed (we can't express that by passing a diagonal scale vector)
    var compound_settings4 = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings4.deinit();
    try compound_settings4.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), triangle, .{});
    try compound_settings4.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.25 * math.pi), triangle, .{});
    var compound4_ref = try createShape(compound_settings4.asShapeSettings());
    defer compound4_ref.deinit();
    const compound4 = compound4_ref.get().?;
    try fw.expect(!compound4.isValidScale(Vec3.zero()));
    try fw.expect(compound4.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(compound4.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(!compound4.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!compound4.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(compound4.isValidScale(Vec3.init(1, 1, 2))); // We're rotation around Z, so non-uniform in the Z direction is ok

    // Test a mutable compound with shapes that can only be scaled uniformly
    var mutable_compound_settings = MutableCompoundShapeSettings.init(allocator);
    defer mutable_compound_settings.deinit();
    try mutable_compound_settings.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.1 * math.pi), sphere, .{});
    try mutable_compound_settings.base.addShapePtr(Vec3.init(4, 5, 6), Quat.rotation(Vec3.axisY(), 0.1 * math.pi), capsule, .{});
    var mutable_compound_ref = try createShape(mutable_compound_settings.asShapeSettings());
    defer mutable_compound_ref.deinit();
    const mutable_compound = mutable_compound_ref.get().?;
    try fw.expect(!mutable_compound.isValidScale(Vec3.zero()));
    try fw.expect(mutable_compound.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(mutable_compound.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(!mutable_compound.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!mutable_compound.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!mutable_compound.isValidScale(Vec3.init(1, 1, 2)));

    // Test mutable compound containing a triangle shape that can be scaled in any way
    var mutable_compound_settings2 = MutableCompoundShapeSettings.init(allocator);
    defer mutable_compound_settings2.deinit();
    try mutable_compound_settings2.base.addShapePtr(Vec3.init(1, 2, 3), Quat.identity(), triangle, .{});
    try addScaledShape(&mutable_compound_settings2.base, Vec3.init(4, 5, 6), Quat.identity(), triangle, Vec3.init(10, 11, 12));
    var mutable_compound2_ref = try createShape(mutable_compound_settings2.asShapeSettings());
    defer mutable_compound2_ref.deinit();
    const mutable_compound2 = mutable_compound2_ref.get().?;
    try fw.expect(!mutable_compound2.isValidScale(Vec3.zero()));
    try fw.expect(mutable_compound2.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(mutable_compound2.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(mutable_compound2.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(mutable_compound2.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(mutable_compound2.isValidScale(Vec3.init(1, 1, 2)));

    // Test rotations inside the mutable compound of 90 degrees
    var mutable_compound_settings3 = MutableCompoundShapeSettings.init(allocator);
    defer mutable_compound_settings3.deinit();
    try mutable_compound_settings3.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.5 * math.pi), triangle, .{});
    try addScaledShape(&mutable_compound_settings3.base, Vec3.init(4, 5, 6), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), triangle, Vec3.init(10, 11, 12));
    var mutable_compound3_ref = try createShape(mutable_compound_settings3.asShapeSettings());
    defer mutable_compound3_ref.deinit();
    const mutable_compound3 = mutable_compound3_ref.get().?;
    try fw.expect(!mutable_compound3.isValidScale(Vec3.zero()));
    try fw.expect(mutable_compound3.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(mutable_compound3.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(mutable_compound3.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(mutable_compound3.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(mutable_compound3.isValidScale(Vec3.init(1, 1, 2)));

    // Test non-90 degree rotations, this would cause shearing so is not allowed (we can't express that by passing a diagonal scale vector)
    var mutable_compound_settings4 = MutableCompoundShapeSettings.init(allocator);
    defer mutable_compound_settings4.deinit();
    try mutable_compound_settings4.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), triangle, .{});
    try mutable_compound_settings4.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.25 * math.pi), triangle, .{});
    var mutable_compound4_ref = try createShape(mutable_compound_settings4.asShapeSettings());
    defer mutable_compound4_ref.deinit();
    const mutable_compound4 = mutable_compound4_ref.get().?;
    try fw.expect(!mutable_compound4.isValidScale(Vec3.zero()));
    try fw.expect(mutable_compound4.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(mutable_compound4.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(!mutable_compound4.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!mutable_compound4.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(mutable_compound4.isValidScale(Vec3.init(1, 1, 2))); // We're rotation around Z, so non-uniform in the Z direction is ok

    // Test a cylinder rotated by 90 degrees around Z rotating Y to X, meaning that Y and Z should be scaled uniformly
    var mutable_compound_settings5 = MutableCompoundShapeSettings.init(allocator);
    defer mutable_compound_settings5.deinit();
    {
        var cylinder5 = RefConst(Shape).init((try CylinderShape.create(allocator, 1.0, 0.5, .{})).asShape());
        defer cylinder5.deinit();
        try mutable_compound_settings5.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.5 * math.pi), cylinder5.get(), .{});
    }
    var mutable_compound5_ref = try createShape(mutable_compound_settings5.asShapeSettings());
    defer mutable_compound5_ref.deinit();
    const mutable_compound5 = mutable_compound5_ref.get().?;
    try fw.expect(mutable_compound5.isValidScale(Vec3.replicate(2)));
    try fw.expect(mutable_compound5.isValidScale(Vec3.init(1, 2, 2)));
    try fw.expect(mutable_compound5.isValidScale(Vec3.init(1, 2, -2)));
    try fw.expect(!mutable_compound5.isValidScale(Vec3.init(2, 1, 2)));
    try fw.expect(!mutable_compound5.isValidScale(Vec3.init(2, 2, 1)));
    try fw.expect(mutable_compound5.makeScaleValid(Vec3.replicate(2)).isClose(Vec3.replicate(2), .{}));
    try fw.expect(mutable_compound5.makeScaleValid(Vec3.replicate(-2)).isClose(Vec3.replicate(-2), .{}));
    try fw.expect(mutable_compound5.makeScaleValid(Vec3.init(1, 2, 2)).isClose(Vec3.init(1, 2, 2), .{}));
    try fw.expect(mutable_compound5.makeScaleValid(Vec3.init(1, 2, -2)).isClose(Vec3.init(1, 2, -2), .{}));
    try fw.expect(mutable_compound5.makeScaleValid(Vec3.init(2, 1, 2)).isClose(Vec3.replicate(@as(f32, 5.0) / 3.0), .{})); // Not the best solution, but we don't have logic to average over YZ only
    try fw.expect(mutable_compound5.makeScaleValid(Vec3.init(2, 2, 1)).isClose(Vec3.replicate(@as(f32, 5.0) / 3.0), .{})); // Not the best solution, but we don't have logic to average over YZ only

    // Test a rotated translated shape that can only be scaled uniformly
    var rt_settings = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.1 * math.pi), sphere);
    defer rt_settings.deinit();
    var rt_shape_ref = try createShape(rt_settings.asShapeSettings());
    defer rt_shape_ref.deinit();
    const rt_shape = rt_shape_ref.get().?;
    try fw.expect(!rt_shape.isValidScale(Vec3.zero()));
    try fw.expect(rt_shape.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(rt_shape.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(!rt_shape.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!rt_shape.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(!rt_shape.isValidScale(Vec3.init(1, 1, 2)));

    // Test rotated translated shape containing a triangle shape that can be scaled in any way
    var scaled_triangle = RefConst(Shape).init((try ScaledShape.create(allocator, triangle, Vec3.init(10, 11, 12))).asShape());
    defer scaled_triangle.deinit();
    var rt_settings2 = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(4, 5, 6), Quat.identity(), scaled_triangle.get());
    defer rt_settings2.deinit();
    var rt_shape2_ref = try createShape(rt_settings2.asShapeSettings());
    defer rt_shape2_ref.deinit();
    const rt_shape2 = rt_shape2_ref.get().?;
    try fw.expect(!rt_shape2.isValidScale(Vec3.zero()));
    try fw.expect(rt_shape2.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(rt_shape2.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(rt_shape2.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(rt_shape2.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(rt_shape2.isValidScale(Vec3.init(1, 1, 2)));

    // Test rotations inside the rotated translated of 90 degrees
    var rt_settings3 = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.5 * math.pi), triangle);
    defer rt_settings3.deinit();
    var rt_shape3_ref = try createShape(rt_settings3.asShapeSettings());
    defer rt_shape3_ref.deinit();
    const rt_shape3 = rt_shape3_ref.get().?;
    try fw.expect(!rt_shape3.isValidScale(Vec3.zero()));
    try fw.expect(rt_shape3.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(rt_shape3.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(rt_shape3.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(rt_shape3.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(rt_shape3.isValidScale(Vec3.init(1, 1, 2)));

    // Test non-90 degree rotations, this would cause shearing so is not allowed (we can't express that by passing a diagonal scale vector)
    var rt_settings4 = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), triangle);
    defer rt_settings4.deinit();
    var rt_shape4_ref = try createShape(rt_settings4.asShapeSettings());
    defer rt_shape4_ref.deinit();
    const rt_shape4 = rt_shape4_ref.get().?;
    try fw.expect(!rt_shape4.isValidScale(Vec3.zero()));
    try fw.expect(rt_shape4.isValidScale(Vec3.init(1, 1, 1)));
    try fw.expect(rt_shape4.isValidScale(Vec3.init(2, 2, 2)));
    try fw.expect(!rt_shape4.isValidScale(Vec3.init(2, 1, 1)));
    try fw.expect(!rt_shape4.isValidScale(Vec3.init(1, 2, 1)));
    try fw.expect(rt_shape4.isValidScale(Vec3.init(1, 1, 2))); // We're rotation around Z, so non-uniform in the Z direction is ok

    // Test a cylinder rotated by 90 degrees around Z rotating Y to X, meaning that Y and Z should be scaled uniformly
    var cylinder5 = RefConst(Shape).init((try CylinderShape.create(allocator, 1.0, 0.5, .{})).asShape());
    defer cylinder5.deinit();
    var rt_settings5 = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.5 * math.pi), cylinder5.get());
    defer rt_settings5.deinit();
    var rt_shape5_ref = try createShape(rt_settings5.asShapeSettings());
    defer rt_shape5_ref.deinit();
    const rt_shape5 = rt_shape5_ref.get().?;
    try fw.expect(rt_shape5.isValidScale(Vec3.replicate(2)));
    try fw.expect(rt_shape5.isValidScale(Vec3.init(1, 2, 2)));
    try fw.expect(rt_shape5.isValidScale(Vec3.init(1, 2, -2)));
    try fw.expect(!rt_shape5.isValidScale(Vec3.init(2, 1, 2)));
    try fw.expect(!rt_shape5.isValidScale(Vec3.init(2, 2, 1)));
    try fw.expect(rt_shape5.makeScaleValid(Vec3.replicate(2)).isClose(Vec3.replicate(2), .{}));
    try fw.expect(rt_shape5.makeScaleValid(Vec3.replicate(-2)).isClose(Vec3.replicate(-2), .{}));
    try fw.expect(rt_shape5.makeScaleValid(Vec3.init(1, 2, 2)).isClose(Vec3.init(1, 2, 2), .{}));
    try fw.expect(rt_shape5.makeScaleValid(Vec3.init(1, 2, -2)).isClose(Vec3.init(1, 2, -2), .{}));
    try fw.expect(rt_shape5.makeScaleValid(Vec3.init(2, 1, 2)).isClose(Vec3.init(2, 1.5, 1.5), .{})); // YZ will be averaged here
    try fw.expect(rt_shape5.makeScaleValid(Vec3.init(2, 2, 1)).isClose(Vec3.init(2, 1.5, 1.5), .{})); // YZ will be averaged here
}

// Test embedded shape
test "TestEmbeddedShape" {
    {
        // Test shape constructed on stack, where shape construction succeeds
        var settings = ConvexHullShapeSettings.initDefault(allocator);
        defer settings.deinit();
        try settings.points.append(allocator, Vec3.init(0, 0, 0));
        try settings.points.append(allocator, Vec3.init(1, 0, 0));
        try settings.points.append(allocator, Vec3.init(0, 1, 0));
        try settings.points.append(allocator, Vec3.init(0, 0, 1));
        var result: ShapeResult = .empty;
        defer result.deinit();
        var shape = ConvexHullShape.initDefault(allocator);
        try shape.initFromSettings(&settings, &result, allocator);
        shape.asShape().setEmbedded();
        defer shape.asShapeMut().deinit();
        try fw.expect(result.isValid());
        result.clear(); // Release the reference from the result

        // Test CollidePoint for this shape
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        shape.asShape().collidePoint(Vec3.replicate(-0.1).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 0);
        shape.asShape().collidePoint(Vec3.replicate(0.1).sub(shape.asShape().getCenterOfMass()), .{}, &collector.base, &.{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
    }

    {
        // Test shape constructed on stack, where shape construction fails
        var settings = ConvexHullShapeSettings.initDefault(allocator);
        defer settings.deinit();
        var result: ShapeResult = .empty;
        defer result.deinit();
        var shape = ConvexHullShape.initDefault(allocator);
        try shape.initFromSettings(&settings, &result, allocator);
        shape.asShape().setEmbedded();
        defer shape.asShapeMut().deinit();
        try fw.expect(!result.isValid());
    }
}

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

// Test setting user data on shapes
test "TestIsValidSubShapeID" {
    var shape1_settings = MutableCompoundShapeSettings.init(allocator);
    defer shape1_settings.deinit();
    var shape1_ref = try createShape(shape1_settings.asShapeSettings()); // StaticCast<CompoundShape>: a RefConst(Shape), cast on use
    defer shape1_ref.deinit();
    const shape1 = shape1_ref.get().?.cast(CompoundShape);

    var shape2_settings = MutableCompoundShapeSettings.init(allocator);
    defer shape2_settings.deinit();
    for (0..3) |_| {
        var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 1.0, .{})).asShape());
        defer sphere.deinit();
        try shape2_settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.get(), .{});
    }
    var shape2_ref = try createShape(shape2_settings.asShapeSettings());
    defer shape2_ref.deinit();
    const shape2 = shape2_ref.get().?.cast(CompoundShape);

    // Get sub shape IDs of shape 2 and test if they're valid
    const sub_shape1 = shape2.getSubShapeIDFromIndex(0, .{}).getID();
    try fw.expect(shape2.isSubShapeIDValid(sub_shape1));
    const sub_shape2 = shape2.getSubShapeIDFromIndex(1, .{}).getID();
    try fw.expect(shape2.isSubShapeIDValid(sub_shape2));
    const sub_shape3 = shape2.getSubShapeIDFromIndex(2, .{}).getID();
    try fw.expect(shape2.isSubShapeIDValid(sub_shape3));
    const sub_shape4 = shape2.getSubShapeIDFromIndex(3, .{}).getID(); // This one doesn't exist
    try fw.expect(!shape2.isSubShapeIDValid(sub_shape4));

    // Shape 1 has no parts so these sub shape ID's should not be valid
    try fw.expect(!shape1.isSubShapeIDValid(sub_shape1));
    try fw.expect(!shape1.isSubShapeIDValid(sub_shape2));
    try fw.expect(!shape1.isSubShapeIDValid(sub_shape3));
    try fw.expect(!shape1.isSubShapeIDValid(sub_shape4));
}

// Test that an error is reported when we run out of sub shape bits
test "TestOutOfSubShapeIDBits" {
    const height_field_samples: u32 = 1024;
    const num_bits_per_compound: u32 = 4;

    // Create a heightfield
    var previous_shape: RefConst(Shape) = .empty;
    defer previous_shape.deinit();
    {
        const samples = try allocator.alloc(f32, height_field_samples * height_field_samples);
        defer allocator.free(samples);
        @memset(samples, 0);
        var height_field_settings = try HeightFieldShapeSettings.init(allocator, samples, Vec3.zero(), Vec3.one(), height_field_samples, .{});
        defer height_field_settings.deinit();
        var height_field = try createShape(height_field_settings.asShapeSettings());
        defer height_field.deinit();
        previous_shape.set(height_field.get());
    }

    // Calculate the amount of bits needed to address all triangles in the heightfield
    var num_bits: u32 = 32 - math.countLeadingZeros((height_field_samples - 1) * (height_field_samples - 1) * 2);

    while (true) {
        // Check that the total sub shape ID bits up to this point is correct
        try fw.expect(previous_shape.get().?.getSubShapeIDBitsRecursive() == num_bits);

        // Create a compound with a number of sub shapes
        var compound_settings = StaticCompoundShapeSettings.init(allocator);
        compound_settings.asShapeSettings().setEmbedded();
        defer compound_settings.deinit();
        for (0..(1 << num_bits_per_compound)) |i|
            try compound_settings.base.addShapePtr(Vec3.init(@floatFromInt(i), 0, 0), Quat.identity(), previous_shape.get(), .{});
        var result = try compound_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        num_bits += num_bits_per_compound;

        if (num_bits < SubShapeID.max_bits) {
            // Creation should have succeeded
            try fw.expect(result.isValid());
            previous_shape.set(result.getPtr());
        } else {
            // Creation should have failed because we ran out of bits
            try fw.expect(!result.isValid());
            break;
        }
    }
}

test "TestEmptyMutableCompound" {
    // Create empty shape
    const mutable_compound = try MutableCompoundShape.create(allocator);
    var mutable_compound_ref = Ref(Shape).init(mutable_compound.asShapeMut());
    defer mutable_compound_ref.deinit();

    // A non-identity rotation
    const rotation = Quat.rotation(Vec3.replicate(1.0 / @sqrt(@as(f32, 3.0))), 0.1 * math.pi);

    // Check that local bounding box is a single point
    const bounds1 = mutable_compound.asShape().getLocalBounds();
    try fw.expect(bounds1.eql(AABox.init(Vec3.zero(), Vec3.zero())));

    // Check that get world space bounds returns a single point
    const vec3_pos = Vec3.init(100, 200, 300);
    const bounds2 = mutable_compound.asShape().getWorldSpaceBounds(Mat44.rotationTranslation(rotation, vec3_pos), Vec3.init(1, 2, 3));
    try fw.expect(bounds2.eql(AABox.init(vec3_pos, vec3_pos)));

    // Check that get world space bounds returns a single point for double precision parameters
    const bounds3 = mutable_compound.asShape().getWorldSpaceBoundsDMat44(DMat44.rotationTranslation(rotation, DVec3.fromVec3(vec3_pos)), Vec3.init(1, 2, 3));
    try fw.expect(bounds3.eql(AABox.init(vec3_pos, vec3_pos)));

    // Add a shape
    {
        var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(1.0), .{})).asShape());
        defer box.deinit();
        _ = try mutable_compound.addShape(Vec3.zero(), Quat.identity(), box.get().?, .{});
    }
    const bounds4 = mutable_compound.asShape().getLocalBounds();
    try fw.expect(bounds4.eql(AABox.init(Vec3.replicate(-1.0), Vec3.replicate(1.0))));

    // Remove it again
    mutable_compound.removeShape(0);

    // Check that the bounding box has zero size again
    const bounds5 = mutable_compound.asShape().getLocalBounds();
    try fw.expect(bounds5.eql(AABox.init(Vec3.zero(), Vec3.zero())));
}

test "TestSaveMeshShape" {
    // Create an n x n grid of triangles
    const n = 10;
    const s: f32 = 0.1;
    var triangles: TriangleList = .empty;
    defer triangles.deinit(allocator);
    for (0..n) |z| {
        for (0..n) |x| {
            const fx = s * @as(f32, @floatFromInt(x)) - s * n / 2;
            const fz = s * @as(f32, @floatFromInt(z)) - s * n / 2;
            try triangles.append(allocator, .init(Vec3.init(fx, 0, fz), Vec3.init(fx, 0, fz + s), Vec3.init(fx + s, 0, fz + s), .{}));
            try triangles.append(allocator, .init(Vec3.init(fx, 0, fz), Vec3.init(fx + s, 0, fz + s), Vec3.init(fx + s, 0, fz), .{}));
        }
    }
    var mesh_settings = try MeshShapeSettings.init(allocator, triangles.items, .{});
    mesh_settings.asShapeSettings().setEmbedded();
    defer mesh_settings.deinit();
    var shape = try createShape(mesh_settings.asShapeSettings());
    defer shape.deinit();

    // Calculate expected bounds
    var expected_bounds: AABox = .empty;
    for (triangles.items) |t| {
        for (t.v) |v|
            expected_bounds.encapsulateVec3(Vec3.fromFloat3(v));
    }

    var stream: std.Io.Writer.Allocating = .init(allocator);
    defer stream.deinit();

    {
        // Write mesh to stream
        var wrapper = StreamOutWrapper.init(&stream.writer);
        shape.get().?.saveBinaryState(wrapper.streamOut());
    }

    {
        // Read back mesh
        var reader: std.Io.Reader = .fixed(stream.written());
        var iwrapper = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, iwrapper.streamIn());
        defer result.deinit();
        try fw.expect(result.isValid());
        var mesh_shape_ref = RefConst(Shape).init(result.getPtr()); // StaticCast<MeshShape>: a RefConst(Shape), cast on use
        defer mesh_shape_ref.deinit();
        const mesh_shape = mesh_shape_ref.get().?.cast(MeshShape).asShape();

        // Test if it contains the same amount of triangles
        const stats = mesh_shape.getStats();
        try fw.expect(stats.num_triangles == triangles.items.len);

        // Check bounding box
        try fw.expect(mesh_shape.getLocalBounds().eql(expected_bounds));

        // Check if we can hit it with a ray
        var hit: RayCastResult = .{};
        const ray = RayCast.init(Vec3.init(0.5 * s, 1, 0.25 * s), Vec3.init(0, -2, 0)); // Hit in the center of a triangle
        try fw.expect(mesh_shape.castRay(ray, .{}, &hit));
        try fw.expect(hit.fraction == 0.5);
        try fw.expect(mesh_shape.getSurfaceNormal(hit.sub_shape_id2, ray.getPointOnRay(hit.fraction)).eql(Vec3.axisY()));
    }
}

test "TestMeshShapePerTriangleUserData" {
    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);

    // Create regular grid of triangles
    var triangles: [2]TriangleList = .{ .empty, .empty };
    defer for (&triangles) |*t| t.deinit(allocator);
    for (0..20) |x| {
        for (0..20) |z| {
            const x1 = 10.0 * @as(f32, @floatFromInt(x));
            const z1 = 10.0 * @as(f32, @floatFromInt(z));
            const x2 = x1 + 10.0;
            const z2 = z1 + 10.0;

            const v1 = Float3.init(x1, 0, z1);
            const v2 = Float3.init(x2, 0, z1);
            const v3 = Float3.init(x1, 0, z2);
            const v4 = Float3.init(x2, 0, z2);

            const user_data: u32 = (@as(u32, @intCast(x)) << 16) + @as(u32, @intCast(z));
            try triangles[random.next() & 1].append(allocator, .fromFloat3(v1, v3, v4, .{ .material_index = 0, .user_data = user_data }));
            try triangles[random.next() & 1].append(allocator, .fromFloat3(v1, v4, v2, .{ .material_index = 0, .user_data = user_data | 0x80000000 }));
        }
    }

    // Create a compound with 2 meshes
    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    compound_settings.asShapeSettings().setEmbedded();
    defer compound_settings.deinit();
    for (&triangles) |*t| {
        // Shuffle the triangles (Fisher-Yates, std::shuffle's exact sequence is implementation defined)
        var i = t.items.len;
        while (i > 1) : (i -= 1)
            std.mem.swap(Triangle, &t.items[i - 1], &t.items[random.next() % i]);

        // Create mesh
        var mesh_settings = try MeshShapeSettings.init(allocator, t.items, .{});
        defer mesh_settings.deinit();
        mesh_settings.per_triangle_user_data = true;
        var mesh = try createShape(mesh_settings.asShapeSettings());
        defer mesh.deinit();
        try compound_settings.base.addShapePtr(Vec3.zero(), Quat.identity(), mesh.get(), .{});
    }
    var compound_ref = try createShape(compound_settings.asShapeSettings());
    defer compound_ref.deinit();
    const compound = compound_ref.get().?;

    // Collide the compound with a box to get all triangles back
    var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(100.0), .{})).asShape());
    defer box.deinit();
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    CollisionDispatch.collideShapeVsShape(box.get().?, compound, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(100.0, 0, 100.0)), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == triangles[0].items.len + triangles[1].items.len);
    for (collector.hits.items) |*r| {
        // Get average vertex
        var avg = Vec3.zero();
        for (r.shape2_face.constSlice()) |v|
            avg = avg.add(v);

        // Calculate the expected user data
        avg = avg.divScalar(30.0);
        const x: u32 = @intFromFloat(avg.getX());
        const z: u32 = @intFromFloat(avg.getZ());
        var expected_user_data: u32 = (x << 16) + z;
        if (avg.getX() - @as(f32, @floatFromInt(x)) > 0.5)
            expected_user_data |= 0x80000000;

        // Get the leaf shape (mesh shape in this case)
        const leaf = compound.getLeafShape(r.sub_shape_id2);
        const shape = leaf.shape.?;
        std.debug.assert(shape.getType() == .mesh);

        // Get user data from the triangle that was hit
        const user_data = shape.cast(MeshShape).getTriangleUserData(leaf.remainder);

        try fw.expect(user_data == expected_user_data);
    }
}

test "TestBoxShape" {
    {
        // Check half extents must be positive
        var box_settings = BoxShapeSettings.init(allocator, Vec3.init(-1, 1, 1), .{});
        defer box_settings.deinit();
        var result = try box_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Check convex radius must be positive
        var box_settings = BoxShapeSettings.init(allocator, Vec3.replicate(1.0), .{ .convex_radius = -1.0 });
        defer box_settings.deinit();
        var result = try box_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Create zero sized box
        var box_settings = BoxShapeSettings.init(allocator, Vec3.zero(), .{ .convex_radius = 1.0 });
        defer box_settings.deinit();
        var box = try createShape(box_settings.asShapeSettings()); // StaticCast<BoxShape>: a RefConst(Shape), cast on use
        defer box.deinit();

        // Create another box by using a different constructor
        var box2 = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.zero(), .{ .convex_radius = 1.0 })).asShape());
        defer box2.deinit();

        // Check convex radius is adjusted to zero
        try fw.expect(box.get().?.cast(BoxShape).getConvexRadius() == 0.0);
        try fw.expect(box2.get().?.cast(BoxShape).getConvexRadius() == 0.0);

        // Not ported: checking that it successfully rests on a floor (needs PhysicsTestContext, Phase 5)
    }
}

test "TestCylinderShape" {
    {
        // Check half height must be positive
        var cylinder_settings = CylinderShapeSettings.init(allocator, -1.0, 1.0, .{ .convex_radius = 1.0 });
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Check radius must be positive
        var cylinder_settings = CylinderShapeSettings.init(allocator, 1.0, -1.0, .{ .convex_radius = 1.0 });
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Check convex radius must be positive
        var cylinder_settings = CylinderShapeSettings.init(allocator, 1.0, 1.0, .{ .convex_radius = -1.0 });
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Create zero sized cylinder
        var cylinder_settings = CylinderShapeSettings.init(allocator, 0.0, 0.0, .{ .convex_radius = 1.0 });
        defer cylinder_settings.deinit();
        var cylinder = try createShape(cylinder_settings.asShapeSettings()); // StaticCast<CylinderShape>: a RefConst(Shape), cast on use
        defer cylinder.deinit();

        // Create another cylinder by using a different constructor
        var cylinder2 = RefConst(Shape).init((try CylinderShape.create(allocator, 0.0, 0.0, .{ .convex_radius = 1.0 })).asShape());
        defer cylinder2.deinit();

        // Check convex radius is adjusted to zero
        try fw.expect(cylinder.get().?.cast(CylinderShape).getConvexRadius() == 0.0);
        try fw.expect(cylinder2.get().?.cast(CylinderShape).getConvexRadius() == 0.0);

        // Not ported: checking that it successfully rests on a floor (needs PhysicsTestContext, Phase 5)
    }
}

test "TestTaperedCylinderShape" {
    {
        // Check half height must be positive
        var cylinder_settings = TaperedCylinderShapeSettings.init(allocator, -1.0, 1.0, 0.1, .{ .convex_radius = 1.0 }); // Top != bottom or else we'll be creating a CylinderShape instead
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Check top radius must be positive
        var cylinder_settings = TaperedCylinderShapeSettings.init(allocator, 1.0, -1.0, 0.1, .{ .convex_radius = 1.0 }); // Top != bottom or else we'll be creating a CylinderShape instead
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Check bottom radius must be positive
        var cylinder_settings = TaperedCylinderShapeSettings.init(allocator, 1.0, 1.0, -0.1, .{ .convex_radius = 1.0 }); // Top != bottom or else we'll be creating a CylinderShape instead
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Check convex radius must be positive
        var cylinder_settings = TaperedCylinderShapeSettings.init(allocator, 1.0, 1.0, 0.1, .{ .convex_radius = -1.0 }); // Top != bottom or else we'll be creating a CylinderShape instead
        defer cylinder_settings.deinit();
        var result = try cylinder_settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try fw.expect(result.hasError());
    }

    {
        // Create zero sized cylinder
        var cylinder_settings = TaperedCylinderShapeSettings.init(allocator, 1.0e-12, 0.0, 1.0e-12, .{ .convex_radius = 1.0 }); // Top != bottom or else we'll be creating a CylinderShape instead
        defer cylinder_settings.deinit();
        var cylinder = try createShape(cylinder_settings.asShapeSettings()); // StaticCast<TaperedCylinderShape>: a RefConst(Shape), cast on use
        defer cylinder.deinit();

        // Check convex radius is adjusted to zero
        try fw.expect(cylinder.get().?.cast(TaperedCylinderShape).getConvexRadius() == 0.0);

        // Not ported: checking that it successfully rests on a floor (needs PhysicsTestContext, Phase 5)
    }
}
