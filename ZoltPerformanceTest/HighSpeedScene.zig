//! Port of: PerformanceTest/HighSpeedScene.h
//! Status: complete
//!
//! `new HighSpeedScene` is `HighSpeedScene.create(allocator)`; the scene is used through the PerformanceTestScene
//! interface (`scene()`), which owns it. `Array<Ref<Shape>> mShapes` holds `RefConst(Shape)` (Jolt only passes them to
//! `SetShape(const Shape *)`). `std::mt19937` is `zolt.Mt19937` (bit identical), the `random_float` lambda is a local
//! function with the lambda's operation order.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
const Layers = @import("Layers.zig").Layers;

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShape = zolt.SphereShape;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const BodyCreationSettings = zolt.BodyCreationSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const MotionQuality = zolt.MotionQuality;
const Mt19937 = zolt.Mt19937;
const pi = zolt.math.pi;

/// A scene that contains a large number of fast moving objects
pub const HighSpeedScene = struct {
    allocator: Allocator,
    shapes: std.ArrayList(RefConst(Shape)) = .empty,

    /// new HighSpeedScene
    pub fn create(allocator: Allocator) Allocator.Error!*HighSpeedScene {
        const self = try allocator.create(HighSpeedScene);
        self.* = .{ .allocator = allocator };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *HighSpeedScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *HighSpeedScene) void {
        self.releaseShapes();
        self.shapes.deinit(self.allocator);
        self.allocator.destroy(self);
    }

    fn releaseShapes(self: *HighSpeedScene) void {
        for (self.shapes.items) |*shape| shape.deinit();
        self.shapes.clearRetainingCapacity();
    }

    pub fn getName(self: *const HighSpeedScene) [:0]const u8 {
        _ = self;
        return "HighSpeed";
    }

    /// AddShape of a `new XShapeSettings` (reference count 0): a temporary reference frees it when AddShape fails
    fn addNewShape(compound: *StaticCompoundShapeSettings, position: Vec3, rotation: Quat, shape: *ShapeSettings) Allocator.Error!void {
        var ref = Ref(ShapeSettings).init(shape);
        defer ref.deinit();
        try compound.base.addShape(position, rotation, shape, .{});
    }

    pub fn load(self: *HighSpeedScene, asset_path: []const u8) PerformanceTestScene.Error!bool {
        _ = asset_path;
        const allocator = self.allocator;

        const shape_size: f32 = 0.5;

        // Ref<StaticCompoundShapeSettings> compound = new StaticCompoundShapeSettings()
        const compound = try StaticCompoundShapeSettings.create(allocator);
        var compound_ref = Ref(ShapeSettings).init(compound.asShapeSettings());
        defer compound_ref.deinit();
        try addNewShape(compound, Vec3.init(0, shape_size, 0), Quat.identity(), (try CapsuleShapeSettings.create(allocator, shape_size, 0.25 * shape_size, .{})).asShapeSettings());
        try addNewShape(compound, Vec3.init(shape_size, 0, 0), Quat.identity(), (try BoxShapeSettings.create(allocator, Vec3.init(shape_size, 0.25 * shape_size, 0.25 * shape_size), .{})).asShapeSettings());

        self.releaseShapes();
        try self.shapes.ensureTotalCapacity(allocator, 3);
        self.shapes.appendAssumeCapacity(.init((try SphereShape.create(allocator, shape_size, .{})).asShape()));
        self.shapes.appendAssumeCapacity(.init((try BoxShape.create(allocator, Vec3.init(0.5 * shape_size, shape_size, 1.0 * shape_size), .{})).asShape()));
        {
            var result = try compound.asShapeSettings().createShape(allocator);
            defer result.deinit();
            self.shapes.appendAssumeCapacity(.init(result.getPtr()));
        }

        return true;
    }

    /// The `random_float` lambda (don't use std::uniform_real_distribution as that is not cross platform deterministic)
    fn randomFloat(rnd: *Mt19937, min: f32, max: f32) f32 {
        return min + @as(f32, @floatFromInt(rnd.next() - Mt19937.min_value)) * (max - min) / @as(f32, @floatFromInt(Mt19937.max_value - Mt19937.min_value));
    }

    pub fn startTest(self: *HighSpeedScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        _ = motion_quality;
        const bi = physics_system.getBodyInterfaceMut();

        const speed: f32 = 250.0;
        const wall_thickness: f32 = 0.2;
        const half_box_size: f32 = 50.0;
        const pos_range: f32 = 0.9 * half_box_size;

        // Create hollow box to enclose the objects
        var body_settings: BodyCreationSettings = .initWithShape((try BoxShape.create(self.allocator, Vec3.init(half_box_size, wall_thickness, half_box_size), .{})).asShape(), RVec3.init(0, -@as(Real, half_box_size), 0), Quat.identity(), .static, Layers.non_moving);
        defer body_settings.deinit();
        body_settings.restitution = 1.0;
        body_settings.friction = 1.0;
        _ = try bi.createAndAddBody(&body_settings, .dont_activate);
        body_settings.position = RVec3.init(0, @as(Real, half_box_size), 0);
        _ = try bi.createAndAddBody(&body_settings, .dont_activate);
        body_settings.position = RVec3.init(0, 0, -@as(Real, half_box_size));
        body_settings.rotation = Quat.rotation(Vec3.axisX(), 0.5 * pi);
        _ = try bi.createAndAddBody(&body_settings, .dont_activate);
        body_settings.position = RVec3.init(0, 0, @as(Real, half_box_size));
        _ = try bi.createAndAddBody(&body_settings, .dont_activate);
        body_settings.position = RVec3.init(-@as(Real, half_box_size), 0, 0);
        body_settings.rotation = Quat.rotation(Vec3.axisZ(), 0.5 * pi);
        _ = try bi.createAndAddBody(&body_settings, .dont_activate);
        body_settings.position = RVec3.init(@as(Real, half_box_size), 0, 0);
        _ = try bi.createAndAddBody(&body_settings, .dont_activate);

        // Create dynamic box
        var dynamic_body_settings: BodyCreationSettings = .initWithShape(self.shapes.items[0].get(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer dynamic_body_settings.deinit();
        dynamic_body_settings.motion_quality = .linear_cast;

        // Create many instances with high velocity (don't use std::uniform_real_distribution as that is not cross platform deterministic)
        var rnd = Mt19937.init(Mt19937.default_seed);
        for (0..5000) |i| {
            // Note that we explicitly order x, y and z. Calling this in the constructor means they get called in a different order on different platforms. This breaks cross platform determinism.
            const x: Real = randomFloat(&rnd, -pos_range, pos_range);
            const y: Real = randomFloat(&rnd, -pos_range, pos_range);
            const z: Real = randomFloat(&rnd, -pos_range, pos_range);

            dynamic_body_settings.setShape(self.shapes.items[i % self.shapes.items.len].get());
            dynamic_body_settings.position = RVec3.init(x, y, z);
            dynamic_body_settings.rotation = Quat.random(&rnd);
            dynamic_body_settings.friction = randomFloat(&rnd, 0.5, 1.0);
            dynamic_body_settings.restitution = randomFloat(&rnd, 0.9, 1.0);
            dynamic_body_settings.linear_velocity = Vec3.random(&rnd).mulScalar(speed);
            _ = try bi.createAndAddBody(&dynamic_body_settings, .activate);
        }
    }
};
