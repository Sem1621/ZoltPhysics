//! Port of: PerformanceTest/PyramidScene.h
//! Status: complete
//!
//! `new PyramidScene` is `PyramidScene.create(allocator)`; the scene is used through the PerformanceTestScene
//! interface (`scene()`), which owns it. Jolt's `RefConst<Shape>` / temporary BodyCreationSettings become a RefConst
//! released with `defer` and BodyCreationSettings that are deinit'ed after each body.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
const Layers = @import("Layers.zig").Layers;

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const BoxShape = zolt.BoxShape;
const BodyCreationSettings = zolt.BodyCreationSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const MotionQuality = zolt.MotionQuality;

/// Jolt's `RVec3(Vec3)`: identity in single precision, DVec3(Vec3) in double precision
fn toRVec3(v: Vec3) RVec3 {
    return if (Core.double_precision) RVec3.fromVec3(v) else v;
}

/// A scene that creates a pyramid of boxes to create a very large island
pub const PyramidScene = struct {
    allocator: Allocator,

    /// new PyramidScene
    pub fn create(allocator: Allocator) Allocator.Error!*PyramidScene {
        const self = try allocator.create(PyramidScene);
        self.* = .{ .allocator = allocator };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *PyramidScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *PyramidScene) void {
        self.allocator.destroy(self);
    }

    pub fn getName(self: *const PyramidScene) [:0]const u8 {
        _ = self;
        return "Pyramid";
    }

    pub fn startTest(self: *PyramidScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        _ = motion_quality;
        const bi = physics_system.getBodyInterfaceMut();

        // Floor
        {
            var floor_settings: BodyCreationSettings = .initWithShape((try BoxShape.create(self.allocator, Vec3.init(50.0, 1.0, 50.0), .{ .convex_radius = 0.0 })).asShape(), toRVec3(Vec3.init(0.0, -1.0, 0.0)), Quat.identity(), .static, Layers.non_moving);
            defer floor_settings.deinit();
            _ = try bi.createAndAddBody(&floor_settings, .dont_activate);
        }

        const box_size: f32 = 2.0;
        const box_separation: f32 = 0.5;
        const half_box_size: f32 = 0.5 * box_size;
        const pyramid_height: i32 = 15;

        var box_shape = RefConst(Shape).init((try BoxShape.create(self.allocator, Vec3.replicate(half_box_size), .{ .convex_radius = 0.0 })).asShape()); // No convex radius to force more collisions
        defer box_shape.deinit();

        // Pyramid
        var i: i32 = 0;
        while (i < pyramid_height) : (i += 1) {
            var j: i32 = @divTrunc(i, 2);
            while (j < pyramid_height - @divTrunc(i + 1, 2)) : (j += 1) {
                var k: i32 = @divTrunc(i, 2);
                while (k < pyramid_height - @divTrunc(i + 1, 2)) : (k += 1) {
                    // Jolt computes the coordinates in float (int + float) and converts them to Real
                    const offset: f32 = if (i & 1 != 0) half_box_size else 0.0;
                    const position = RVec3.init(
                        @as(Real, @as(f32, @floatFromInt(-pyramid_height)) + box_size * @as(f32, @floatFromInt(j)) + offset),
                        @as(Real, 1.0 + (box_size + box_separation) * @as(f32, @floatFromInt(i))),
                        @as(Real, @as(f32, @floatFromInt(-pyramid_height)) + box_size * @as(f32, @floatFromInt(k)) + offset),
                    );
                    var settings: BodyCreationSettings = .initWithShape(box_shape.get().?, position, Quat.identity(), .dynamic, Layers.moving);
                    defer settings.deinit();
                    settings.allow_sleeping = false; // No sleeping to force the large island to stay awake
                    _ = try bi.createAndAddBody(&settings, .activate);
                }
            }
        }
    }
};
