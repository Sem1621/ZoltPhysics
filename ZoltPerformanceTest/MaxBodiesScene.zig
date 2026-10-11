//! Port of: PerformanceTest/MaxBodiesScene.h
//! Status: complete
//!
//! `new MaxBodiesScene` is `MaxBodiesScene.create(allocator)`; the scene is used through the PerformanceTestScene
//! interface (`scene()`), which owns it. `BodyIDVector body_ids` is an `std.ArrayList(BodyID)`. `pow(float, float)` is
//! `std.math.pow(f32, ...)`: only its integer part is used (8388608^(1/3) = 203.19), which every pow implementation
//! gets right, so this does not affect determinism.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
const Layers = @import("Layers.zig").Layers;

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const BodyCreationSettings = zolt.BodyCreationSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsSettings = zolt.PhysicsSettings;
const MotionQuality = zolt.MotionQuality;

/// A scene that creates the max number of bodies that Jolt supports and simulates them
pub const MaxBodiesScene = struct {
    allocator: Allocator,

    /// new MaxBodiesScene
    pub fn create(allocator: Allocator) Allocator.Error!*MaxBodiesScene {
        const self = try allocator.create(MaxBodiesScene);
        self.* = .{ .allocator = allocator };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *MaxBodiesScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *MaxBodiesScene) void {
        self.allocator.destroy(self);
    }

    pub fn getName(self: *const MaxBodiesScene) [:0]const u8 {
        _ = self;
        return "MaxBodies";
    }

    pub fn getTempAllocatorSizeMB(self: *const MaxBodiesScene) usize {
        _ = self;
        return 8192;
    }

    pub fn getMaxBodies(self: *const MaxBodiesScene) u32 {
        _ = self;
        return PhysicsSystem.max_bodies_limit;
    }

    pub fn getMaxBodyPairs(self: *const MaxBodiesScene) u32 {
        _ = self;
        return PhysicsSystem.max_body_pairs_limit;
    }

    pub fn getMaxContactConstraints(self: *const MaxBodiesScene) u32 {
        _ = self;
        return PhysicsSystem.max_contact_constraints_limit;
    }

    pub fn startTest(self: *MaxBodiesScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        _ = motion_quality;
        const bi = physics_system.getBodyInterfaceMut();

        // Reduce the solver iteration count in the interest of performance
        var settings: PhysicsSettings = physics_system.getPhysicsSettings().*;
        settings.num_velocity_steps = 4;
        settings.num_position_steps = 1;
        physics_system.setPhysicsSettings(&settings);

        // Create the bodies
        const num_bodies: u32 = physics_system.getMaxBodies();
        var num_constraints: u32 = 0;
        var body_ids: std.ArrayList(BodyID) = .empty;
        defer body_ids.deinit(self.allocator);
        try body_ids.ensureTotalCapacity(self.allocator, num_bodies);
        const num_per_axis: u32 = @as(u32, @intFromFloat(std.math.pow(f32, @floatFromInt(num_bodies), 1.0 / 3.0))) + 1;
        const half_extent = Vec3.replicate(0.5);
        var bcs: BodyCreationSettings = .initWithShape((try BoxShape.create(self.allocator, half_extent, .{})).asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer bcs.deinit();
        bcs.override_mass_properties = .mass_and_inertia_provided;
        bcs.mass_properties_override.setMassAndInertiaOfSolidBox(half_extent.mulScalar(2.0), 1000.0);
        var z: u32 = 0;
        while (z < num_per_axis and body_ids.items.len < num_bodies) : (z += 1) {
            var y: u32 = 0;
            while (y < num_per_axis and body_ids.items.len < num_bodies) : (y += 1) {
                var x: u32 = 0;
                while (x < num_per_axis and body_ids.items.len < num_bodies) : (x += 1) {
                    // When we reach the limit of contact constraints, start placing the boxes further apart so they don't collide
                    const fx: Real = @floatFromInt(x);
                    bcs.position = RVec3.init(if (num_constraints < PhysicsSystem.max_contact_constraints_limit) fx else 2.0 * fx, 2.0 * @as(Real, @floatFromInt(y)), 2.0 * @as(Real, @floatFromInt(z)));
                    body_ids.appendAssumeCapacity((try bi.createBody(&bcs)).?.getID()); // Jolt dereferences the result: never out of bodies here

                    // From the 2nd box onwards in a row, we will get a contact constraint
                    if (x > 0)
                        num_constraints += 1;
                }
            }
        }

        // Add the bodies to the simulation
        const state = try bi.addBodiesPrepare(body_ids.items[0..num_bodies]);
        bi.addBodiesFinalize(body_ids.items[0..num_bodies], state, .activate);
    }
};
