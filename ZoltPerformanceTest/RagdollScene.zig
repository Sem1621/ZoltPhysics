//! Port of: PerformanceTest/RagdollScene.h
//! Status: complete
//!
//! Milestone M9 (SerializationArchitecture.md section 1): `zig build performance-test -Doptimize=ReleaseFast --
//! -q=LinearCast -t=max -s=Ragdoll -validate_hash=0x3be6ea76c07a2ac8` (0x8dd6bc501859a228 with
//! -Ddouble_precision=true) prints Jolt's hash for every thread count.
//! - `new RagdollScene(...)` is `RagdollScene.create(allocator, io, num_piles_per_axis, pile_size,
//!   vertical_separation)`; the scene is used through the PerformanceTestScene interface (`scene()`), which owns it.
//!   `io` reads the assets.
//! - Load reads the three assets with the file variants of ObjectStreamIn (`sReadObjectRefFromFile`). A file that cannot
//!   be read prints Jolt's message to stderr (`cerr`) and returns false. Only running out of memory is an error.
//! - `Ref<RagdollSettings>`, `Ref<SkeletalAnimation>` and `Ref<PhysicsScene>` are Refs; `SkeletonPose mPose` is a
//!   SkeletonPose that the scene deinits. `Array<Ref<Ragdoll>> mRagdolls` holds one reference per ragdoll.
//! - CreateRagdoll returns null when the system is out of bodies, where Jolt dereferences null: Zolt panics with "out of
//!   bodies" (SerializationArchitecture.md section 4, G6).
//! - StopTest returns `Error!void`, because Ragdoll.removeFromPhysicsSystem allocates. When a ragdoll cannot be removed,
//!   the ragdolls before it are released and the others stay in the list, so that a second call removes them.
//! - StartTest reserves the slot in `ragdolls` before it creates a ragdoll, so that a ragdoll in the system always has
//!   its reference. When StartTest fails, it removes the ragdolls that it added again. That removal can run out of
//!   memory too. The ragdolls then stay in the list, and a second StopTest removes them.
//! - Destroy leaks the ragdolls that are still in the list. Their release would destroy their bodies, and main destroys
//!   the physics system before the scene. This only happens when StopTest ran out of memory and was not called again.
//!   Only a non-allocating Ragdoll.removeFromPhysicsSystem could avoid this leak.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
/// The layers of the performance test (also used by the parity test perf_ragdoll, which imports this file as a module)
pub const layers = @import("Layers.zig");
const Layers = layers.Layers;
const BroadPhaseLayers = layers.BroadPhaseLayers;

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RagdollSettings = zolt.RagdollSettings;
const Ragdoll = zolt.Ragdoll;
const SkeletalAnimation = zolt.SkeletalAnimation;
const SkeletonPose = zolt.SkeletonPose;
const PhysicsScene = zolt.PhysicsScene;
const ObjectStreamIn = zolt.ObjectStreamIn;
const PhysicsSystem = zolt.PhysicsSystem;
const MotionQuality = zolt.MotionQuality;
const CollisionGroup = zolt.CollisionGroup;
const RRayCast = zolt.RRayCast;
const RayCastResult = zolt.RayCastResult;
const SpecifiedBroadPhaseLayerFilter = zolt.SpecifiedBroadPhaseLayerFilter;
const SpecifiedObjectLayerFilter = zolt.SpecifiedObjectLayerFilter;
const pi = zolt.math.pi;

/// A scene that loads a part of a Horizon Zero Dawn level and drops many ragdolls on the terrain (motors enabled)
pub const RagdollScene = struct {
    allocator: Allocator,
    /// Reads the assets (Zolt)
    io: std.Io,
    num_piles_per_axis: i32,
    pile_size: i32,
    vertical_separation: f32,
    ragdoll_settings: Ref(RagdollSettings) = .empty,
    animation: Ref(SkeletalAnimation) = .empty,
    pose: SkeletonPose,
    background: Ref(PhysicsScene) = .empty,
    /// Array<Ref<Ragdoll>>: one reference per ragdoll
    ragdolls: std.ArrayList(*Ragdoll) = .empty,

    /// new RagdollScene(inNumPilesPerAxis, inPileSize, inVerticalSeparation)
    pub fn create(allocator: Allocator, io: std.Io, num_piles_per_axis: i32, pile_size: i32, vertical_separation: f32) Allocator.Error!*RagdollScene {
        const self = try allocator.create(RagdollScene);
        self.* = .{
            .allocator = allocator,
            .io = io,
            .num_piles_per_axis = num_piles_per_axis,
            .pile_size = pile_size,
            .vertical_separation = vertical_separation,
            .pose = .init(allocator),
        };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *RagdollScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *RagdollScene) void {
        // Zolt: StopTest released the ragdolls. A ragdoll is only left when StopTest ran out of memory and was not called
        // again. It is leaked then: its release would destroy its bodies, and their physics system may be gone.
        self.ragdolls.deinit(self.allocator);
        self.background.deinit();
        self.pose.deinit();
        self.animation.deinit();
        self.ragdoll_settings.deinit();
        self.allocator.destroy(self);
    }

    pub fn getName(self: *const RagdollScene) [:0]const u8 {
        return if (self.num_piles_per_axis == 1) "RagdollSinglePile" else "Ragdoll";
    }

    /// ObjectStreamIn::sReadObject(path, outObject): the object of the file `asset_path` + `name`, null when the file
    /// cannot be read
    fn readObject(self: *RagdollScene, comptime T: type, asset_path: []const u8, name: []const u8) Allocator.Error!?Ref(T) {
        const path = try std.mem.concat(self.allocator, u8, &.{ asset_path, name });
        defer self.allocator.free(path);
        return ObjectStreamIn.sReadObjectRefFromFile(T, self.allocator, self.io, path, .{}) catch |err| switch (err) {
            error.OutOfMemory => return error.OutOfMemory,
            else => return null,
        };
    }

    /// cerr << message << endl
    fn printError(message: []const u8) void {
        std.debug.print("{s}\n", .{message});
    }

    pub fn load(self: *RagdollScene, asset_path: []const u8) PerformanceTestScene.Error!bool {
        // Load ragdoll
        self.ragdoll_settings.deinit();
        self.ragdoll_settings = try self.readObject(RagdollSettings, asset_path, "Human.tof") orelse {
            printError("Unable to load ragdoll");
            return false;
        };
        const settings = self.ragdoll_settings.get().?;
        for (settings.parts.items) |*body|
            body.base.object_layer = Layers.moving;

        // Init ragdoll
        settings.getSkeletonMut().?.calculateParentJointIndices();
        _ = try settings.stabilize();
        try settings.calculateBodyIndexToConstraintIndex();
        try settings.calculateConstraintIndexToBodyIdxPair();

        // Load animation
        self.animation.deinit();
        self.animation = try self.readObject(SkeletalAnimation, asset_path, "Human/dead_pose1.tof") orelse {
            printError("Unable to load animation");
            return false;
        };

        // Sample pose
        try self.pose.setSkeleton(settings.getSkeleton().?);
        self.animation.get().?.sample(0.0, &self.pose);

        // Read the background scene
        self.background.deinit();
        self.background = try self.readObject(PhysicsScene, asset_path, "terrain2.bof") orelse {
            printError("Unable to load terrain");
            return false;
        };
        const background = self.background.get().?;
        for (background.getBodiesMut().items) |*body|
            body.object_layer = Layers.non_moving;
        _ = try background.fixInvalidScales();

        return true;
    }

    pub fn startTest(self: *RagdollScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        // Test configuration
        const horizontal_separation: Real = 4.0;

        const settings = self.ragdoll_settings.get().?;

        // Set motion quality on ragdoll
        for (settings.parts.items) |*body|
            body.base.motion_quality = motion_quality;

        // Add background geometry
        _ = try self.background.get().?.createBodies(physics_system);

        // Zolt: when a ragdoll cannot be created or added, the ragdolls that were added leave the system again
        errdefer self.stopTest(physics_system) catch {};

        // Create ragdoll piles
        var group_id: CollisionGroup.GroupID = 1;
        var row: i32 = 0;
        while (row < self.num_piles_per_axis) : (row += 1) {
            var col: i32 = 0;
            while (col < self.num_piles_per_axis) : (col += 1) {
                // Determine start location of ray
                const half_piles = @as(Real, @floatFromInt(self.num_piles_per_axis - 1)) / 2.0;
                var start = RVec3.init(horizontal_separation * (@as(Real, @floatFromInt(col)) - half_piles), 100, horizontal_separation * (@as(Real, @floatFromInt(row)) - half_piles));

                // Cast ray down to terrain
                var hit: RayCastResult = .{};
                const ray_direction = Vec3.init(0, -200, 0);
                const ray = RRayCast.init(start, ray_direction);
                const broad_phase_layer_filter = SpecifiedBroadPhaseLayerFilter.init(BroadPhaseLayers.non_moving);
                const object_layer_filter = SpecifiedObjectLayerFilter.init(Layers.non_moving);
                if (physics_system.getNarrowPhaseQuery().castRay(ray, &hit, .{ .broad_phase_layer_filter = &broad_phase_layer_filter.base, .object_layer_filter = &object_layer_filter.base }))
                    start = ray.getPointOnRay(hit.fraction);

                var i: i32 = 0;
                while (i < self.pile_size) : (i += 1) {
                    // Zolt: the slot in mRagdolls, so that a ragdoll in the system always has its reference
                    try self.ragdolls.ensureUnusedCapacity(self.allocator, 1);

                    // Create ragdoll
                    const ragdoll = try settings.createRagdoll(group_id, 0, physics_system) orelse @panic("out of bodies"); // Jolt dereferences the null ragdoll
                    group_id += 1;
                    ragdoll.addRef();
                    errdefer ragdoll.release();

                    // Override root
                    var pose_copy = try self.pose.clone();
                    defer pose_copy.deinit();
                    pose_copy.setRootOffset(start);
                    const root = pose_copy.getJointMut(0);
                    root.translation = Vec3.init(0, self.vertical_separation * @as(f32, @floatFromInt(i + 1)), 0);
                    const angle = 2.0 * pi * @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(self.pile_size));
                    root.rotation = Quat.rotation(Vec3.axisY(), angle).mul(root.rotation);
                    pose_copy.calculateJointMatrices();

                    // Drive to pose
                    ragdoll.setPose(&pose_copy, .{});
                    ragdoll.driveToPoseUsingMotors(&pose_copy);
                    try ragdoll.addToPhysicsSystem(.activate, .{});

                    // Keep reference
                    self.ragdolls.appendAssumeCapacity(ragdoll);
                }
            }
        }
    }

    pub fn stopTest(self: *RagdollScene, physics_system: *PhysicsSystem) PerformanceTestScene.Error!void {
        _ = physics_system;

        // Remove ragdolls
        for (self.ragdolls.items, 0..) |ragdoll, i| {
            ragdoll.removeFromPhysicsSystem(.{}) catch |err| {
                // Zolt: release the ragdolls that left the system, keep the others for a second call
                for (self.ragdolls.items[0..i]) |r| r.release();
                const rest = self.ragdolls.items[i..];
                std.mem.copyForwards(*Ragdoll, self.ragdolls.items[0..rest.len], rest);
                self.ragdolls.shrinkRetainingCapacity(rest.len);
                return err;
            };
        }
        for (self.ragdolls.items) |ragdoll| ragdoll.release();
        self.ragdolls.clearRetainingCapacity();
    }
};
