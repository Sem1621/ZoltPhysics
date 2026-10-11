//! Port of: PerformanceTest/LargeMeshScene.h
//! Status: complete
//!
//! `new LargeMeshScene` is `LargeMeshScene.create(allocator)`; the scene is used through the PerformanceTestScene
//! interface (`scene()`), which owns it. `Trace` is `std.log.info`: PerformanceTest.zig's `std_options.logFn` prints it
//! like Jolt's TraceImpl. The printf formats `%u` / `%llu` / `%d` / `%.1f` / `%.3f` are `{d}` / `{d:.1}` / `{d:.3}`.
//! The time is measured with the monotonic clock of `std.Io` (Jolt: `std::chrono::high_resolution_clock`).

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
const MeshShapeSettings = zolt.MeshShapeSettings;
const BodyCreationSettings = zolt.BodyCreationSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const MotionQuality = zolt.MotionQuality;
const sin = zolt.trigonometry.sin;
const cos = zolt.trigonometry.cos;

const log = std.log;

/// A scene that first finds the largest possible mesh and then simulates some objects on it
pub const LargeMeshScene = struct {
    allocator: Allocator,
    io: std.Io,
    mesh_creation_settings: BodyCreationSettings = .{},

    /// new LargeMeshScene (`io` measures the time of the mesh creation)
    pub fn create(allocator: Allocator, io: std.Io) Allocator.Error!*LargeMeshScene {
        const self = try allocator.create(LargeMeshScene);
        self.* = .{ .allocator = allocator, .io = io };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *LargeMeshScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *LargeMeshScene) void {
        self.mesh_creation_settings.deinit();
        self.allocator.destroy(self);
    }

    pub fn getName(self: *const LargeMeshScene) [:0]const u8 {
        _ = self;
        return "LargeMeshScene";
    }

    pub fn load(self: *LargeMeshScene, asset_path: []const u8) PerformanceTestScene.Error!bool {
        _ = asset_path;
        const allocator = self.allocator;

        // Create mesh shape creation settings
        self.mesh_creation_settings.motion_type = .static;
        self.mesh_creation_settings.object_layer = Layers.non_moving;
        self.mesh_creation_settings.position = RVec3.zero();
        self.mesh_creation_settings.friction = 0.5;
        self.mesh_creation_settings.restitution = 0.6;

        log.info("Finding the largest possible mesh, this will take some time!", .{});
        log.info("N, Num Triangles, Mesh Size, Size / Triangle, SubShapeID Bits, Time", .{});
        var i: u32 = 1;
        while (true) : (i += 1) {
            const n: u32 = 500 * i;
            const cell_size: f32 = 1.0;
            const max_height: f32 = 50.0;
            const fn_: f32 = @floatFromInt(n);

            // Create heights
            var settings: MeshShapeSettings = .initDefault(allocator);
            defer settings.deinit();
            const center: f32 = fn_ * cell_size / 2;
            try settings.triangle_vertices.ensureTotalCapacity(allocator, @as(usize, n + 1) * (n + 1));
            for (0..n + 1) |x| {
                for (0..n + 1) |z| {
                    const fx: f32 = @floatFromInt(x);
                    const fz: f32 = @floatFromInt(z);
                    settings.triangle_vertices.appendAssumeCapacity(Float3.init(cell_size * fx - center, max_height * sin(fx * 50.0 / fn_) * cos(fz * 50.0 / fn_), cell_size * fz - center));
                }
            }

            // Create regular grid of triangles
            try settings.indexed_triangles.ensureTotalCapacity(allocator, 2 * @as(usize, n) * n);
            for (0..n) |xi| {
                for (0..n) |zi| {
                    const x: u32 = @intCast(xi);
                    const z: u32 = @intCast(zi);
                    settings.indexed_triangles.appendAssumeCapacity(.init(x + z * (n + 1), x + 1 + z * (n + 1), x + (z + 1) * (n + 1), .{}));
                    settings.indexed_triangles.appendAssumeCapacity(.init(x + 1 + z * (n + 1), x + 1 + (z + 1) * (n + 1), x + (z + 1) * (n + 1), .{}));
                }
            }

            // Start measuring
            const clock_start = std.Io.Clock.awake.now(self.io);

            // Create the mesh shape
            var result = try settings.asShapeSettings().createShape(allocator);
            defer result.deinit();

            // Stop measuring
            const clock_end = std.Io.Clock.awake.now(self.io);
            const duration = clock_start.durationTo(clock_end);

            if (result.hasError()) {
                // Break when we get an error
                log.info("Mesh creation failed with error: {s}", .{result.getError()});
                break;
            } else {
                // Trace stats
                var shape = RefConst(Shape).init(result.getPtr());
                defer shape.deinit();
                const stats = shape.get().?.getStats();
                log.info("{d}, {d}, {d}, {d:.1}, {d}, {d:.3}", .{ n, stats.num_triangles, @as(u64, stats.size_bytes), @as(f64, @floatFromInt(stats.size_bytes)) / @as(f64, @floatFromInt(stats.num_triangles)), shape.get().?.getSubShapeIDBitsRecursive(), 1.0e-9 * @as(f64, @floatFromInt(duration.toNanoseconds())) });

                // Set this shape as the best shape so far
                self.mesh_creation_settings.setShape(shape.get());
            }
        }

        return true;
    }

    pub fn startTest(self: *LargeMeshScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        // Create background
        const bi = physics_system.getBodyInterfaceMut();
        _ = try bi.createAndAddBody(&self.mesh_creation_settings, .dont_activate);

        // Construct bodies
        var creation_settings: BodyCreationSettings = .{};
        defer creation_settings.deinit();
        creation_settings.motion_type = .dynamic;
        creation_settings.motion_quality = motion_quality;
        creation_settings.object_layer = Layers.moving;
        creation_settings.friction = 0.5;
        creation_settings.restitution = 0.6;
        creation_settings.setShape((try BoxShape.create(self.allocator, Vec3.init(0.5, 0.75, 1.0), .{})).asShape());
        var x: i32 = -10;
        while (x <= 10) : (x += 1) {
            for (0..10) |y| {
                var z: i32 = -10;
                while (z <= 10) : (z += 1) {
                    creation_settings.position = RVec3.init(7.5 * @as(Real, @floatFromInt(x)), 55.0 + 2.0 * @as(Real, @floatFromInt(y)), 7.5 * @as(Real, @floatFromInt(z)));
                    _ = try bi.createAndAddBody(&creation_settings, .activate);
                }
            }
        }
    }
};
