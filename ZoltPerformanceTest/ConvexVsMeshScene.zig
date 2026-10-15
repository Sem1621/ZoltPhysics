//! Port of: PerformanceTest/ConvexVsMeshScene.h
//! Status: complete
//!
//! `new ConvexVsMeshScene` is `ConvexVsMeshScene.create(allocator)`; the scene is used through the PerformanceTestScene
//! interface (`scene()`), which owns it. `Array<Ref<Shape>> mShapes` holds `RefConst(Shape)` (Jolt only passes them to
//! `SetShape(const Shape *)`). The temporary `ConvexHullShapeSettings(...).Create().Get()` is a ConvexHullShapeSettings
//! on the stack that is deinit'ed after its shape was referenced.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
const Layers = @import("Layers.zig").Layers;

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const BoxShape = zolt.BoxShape;
const SphereShape = zolt.SphereShape;
const CapsuleShape = zolt.CapsuleShape;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const MeshShapeSettings = zolt.MeshShapeSettings;
const BodyCreationSettings = zolt.BodyCreationSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsSettings = zolt.PhysicsSettings;
const MotionQuality = zolt.MotionQuality;
const sin = zolt.trigonometry.sin;
const cos = zolt.trigonometry.cos;

/// A scene that drops a number of convex shapes on a sloping terrain made out of a mesh shape
pub const ConvexVsMeshScene = struct {
    allocator: Allocator,
    mesh_settings: BodyCreationSettings = .{},
    shapes: std.ArrayList(RefConst(Shape)) = .empty,

    /// new ConvexVsMeshScene
    pub fn create(allocator: Allocator) Allocator.Error!*ConvexVsMeshScene {
        const self = try allocator.create(ConvexVsMeshScene);
        self.* = .{ .allocator = allocator };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *ConvexVsMeshScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *ConvexVsMeshScene) void {
        self.releaseShapes();
        self.shapes.deinit(self.allocator);
        self.mesh_settings.deinit();
        self.allocator.destroy(self);
    }

    fn releaseShapes(self: *ConvexVsMeshScene) void {
        for (self.shapes.items) |*shape| shape.deinit();
        self.shapes.clearRetainingCapacity();
    }

    pub fn getName(self: *const ConvexVsMeshScene) [:0]const u8 {
        _ = self;
        return "ConvexVsMesh";
    }

    pub fn load(self: *ConvexVsMeshScene, asset_path: []const u8) PerformanceTestScene.Error!bool {
        _ = asset_path;
        const allocator = self.allocator;

        const n = 100;
        const cell_size: f32 = 3.0;
        const max_height: f32 = 5.0;
        const center: f32 = @as(f32, n) * cell_size / 2;

        // Create vertices
        const num_vertices = (n + 1) * (n + 1);
        const vertices = try allocator.alloc(Float3, num_vertices);
        defer allocator.free(vertices);
        for (0..n + 1) |x| {
            for (0..n + 1) |z| {
                const fx: f32 = @floatFromInt(x);
                const fz: f32 = @floatFromInt(z);
                const height = sin(fx * 50.0 / @as(f32, n)) * cos(fz * 50.0 / @as(f32, n));
                vertices[z * (n + 1) + x] = Float3.init(cell_size * fx, max_height * height, cell_size * fz);
            }
        }

        // Create regular grid of triangles
        const num_triangles = n * n * 2;
        const indices = try allocator.alloc(IndexedTriangle, num_triangles);
        defer allocator.free(indices);
        var next: usize = 0;
        for (0..n) |x| {
            for (0..n) |z| {
                const start: u32 = @intCast((n + 1) * z + x);

                indices[next] = .{ .idx = .{ start, start + n + 1, start + 1 } };
                next += 1;

                indices[next] = .{ .idx = .{ start + 1, start + n + 1, start + n + 2 } };
                next += 1;
            }
        }

        // Create mesh shape settings
        const mesh_shape_settings = try MeshShapeSettings.createIndexed(allocator, vertices, indices, .{});
        mesh_shape_settings.max_triangles_per_leaf = 4;

        // Create mesh shape creation settings
        self.mesh_settings.motion_type = .static;
        self.mesh_settings.object_layer = Layers.non_moving;
        self.mesh_settings.position = RVec3.init(@as(Real, -center), @as(Real, max_height), @as(Real, -center));
        self.mesh_settings.friction = 0.5;
        self.mesh_settings.restitution = 0.6;
        self.mesh_settings.setShapeSettings(mesh_shape_settings.asShapeSettings());

        // Create other shapes
        self.releaseShapes();
        try self.shapes.ensureTotalCapacity(allocator, 4);
        self.shapes.appendAssumeCapacity(.init((try BoxShape.create(allocator, Vec3.init(0.5, 0.75, 1.0), .{})).asShape()));
        self.shapes.appendAssumeCapacity(.init((try SphereShape.create(allocator, 0.5, .{})).asShape()));
        self.shapes.appendAssumeCapacity(.init((try CapsuleShape.create(allocator, 0.75, 0.5, .{})).asShape()));
        {
            var hull_settings = try ConvexHullShapeSettings.init(allocator, &.{ Vec3.init(0, 1, 0), Vec3.init(1, 0, 0), Vec3.init(-1, 0, 0), Vec3.init(0, 0, 1), Vec3.init(0, 0, -1) }, .{});
            defer hull_settings.deinit();
            var result = try hull_settings.asShapeSettings().createShape(allocator);
            defer result.deinit();
            self.shapes.appendAssumeCapacity(.init(result.getPtr()));
        }

        return true;
    }

    pub fn startTest(self: *ConvexVsMeshScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        // Reduce the solver iteration count, the scene doesn't have any constraints so we don't need the default amount of iterations
        var settings: PhysicsSettings = physics_system.getPhysicsSettings().*;
        settings.num_velocity_steps = 4;
        settings.num_position_steps = 1;
        physics_system.setPhysicsSettings(&settings);

        // Create background
        const bi = physics_system.getBodyInterfaceMut();
        _ = try bi.createAndAddBody(&self.mesh_settings, .dont_activate);

        // Construct bodies
        var x: i32 = -10;
        while (x <= 10) : (x += 1) {
            for (self.shapes.items, 0..) |shape, y| {
                var z: i32 = -10;
                while (z <= 10) : (z += 1) {
                    var creation_settings: BodyCreationSettings = .{};
                    defer creation_settings.deinit();
                    creation_settings.motion_type = .dynamic;
                    creation_settings.motion_quality = motion_quality;
                    creation_settings.object_layer = Layers.moving;
                    creation_settings.position = RVec3.init(7.5 * @as(Real, @floatFromInt(x)), 15.0 + 2.0 * @as(Real, @floatFromInt(y)), 7.5 * @as(Real, @floatFromInt(z)));
                    creation_settings.friction = 0.5;
                    creation_settings.restitution = 0.6;
                    creation_settings.setShape(shape.get());
                    _ = try bi.createAndAddBody(&creation_settings, .activate);
                }
            }
        }
    }
};
