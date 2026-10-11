//! Port of: UnitTests/PhysicsTestContext.h, UnitTests/PhysicsTestContext.cpp
//! Status: complete
//! Not ported: RecordDebugOutput (JPH_DEBUG_RENDERER: TODO(debug_renderer))
//!
//! Helper class used in test cases for creating and manipulating physics objects (SimulationArchitecture D12).
//! - Initialized in place (the physics system and the job system must not move): `var c: PhysicsTestContext = undefined;
//!   try c.init(allocator, .{}); defer c.deinit();`. Jolt's defaulted constructor arguments are the options.
//! - The temp allocator is a TempAllocatorImpl of 4 MB (Jolt: unless JPH_DISABLE_TEMP_ALLOCATOR), the job system a
//!   JobSystemThreadPool with `worker_threads` threads (0: the main thread runs every job in the barrier).
//! - Functions that create bodies return `!*Body` (Jolt dereferences the returned pointer: out of bodies fails the
//!   test with error.OutOfBodies). `simulateSingleStep` checks that the temp allocator is empty afterwards (Jolt's
//!   JPH_ASSERT) and returns error.TestUnexpectedResult otherwise.
//! - `CreateBody(const ShapeSettings *, ...)` is `createBodyWithShapeSettings`, `CreateConstraint<T>` is
//!   `createConstraint(T, ...)`, `Simulate(time, function<void()>)` takes `pre_step_callback: anytype` (any value with
//!   a `call()` method, or `{}` for none).

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("Layers.zig");

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Core = zolt.Core;
const Activation = zolt.Activation;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyInterface = zolt.BodyInterface;
const MotionType = zolt.MotionType;
const MotionQuality = zolt.MotionQuality;
const ObjectLayer = zolt.ObjectLayer;
const ShapeSettings = zolt.ShapeSettings;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const TwoBodyConstraintSettings = zolt.TwoBodyConstraintSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsUpdateError = zolt.PhysicsUpdateError;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const JobSystem = zolt.JobSystem;
const JobSystemThreadPool = zolt.JobSystemThreadPool;

pub const io = std.testing.io;

/// Helper class used in test cases for creating and manipulating physics objects
pub const PhysicsTestContext = struct {
    /// The constructor arguments of Jolt (with Jolt's defaults)
    pub const Options = struct {
        delta_time: f32 = 1.0 / 60.0,
        collision_steps: u32 = 1,
        worker_threads: i32 = 0,
        max_bodies: u32 = 1024,
        max_body_pairs: u32 = 4096,
        max_contact_constraints: u32 = 1024,
    };

    allocator: Allocator,
    temp_allocator: TempAllocatorImpl,
    job_system: JobSystemThreadPool,
    broad_phase_layer_interface: Layers.BPLayerInterfaceImpl,
    object_vs_broad_phase_layer_filter: Layers.ObjectVsBroadPhaseLayerFilterImpl,
    object_vs_object_layer_filter: Layers.ObjectLayerPairFilterImpl,
    system: PhysicsSystem,
    delta_time: f32,
    collision_steps: u32,

    /// Constructor (in place)
    pub fn init(self: *PhysicsTestContext, allocator: Allocator, opts: Options) !void {
        self.* = .{
            .allocator = allocator,
            .temp_allocator = try .init(allocator, 4 * 1024 * 1024),
            .job_system = .empty,
            .broad_phase_layer_interface = .init(),
            .object_vs_broad_phase_layer_filter = .{},
            .object_vs_object_layer_filter = .{},
            .system = .empty,
            .delta_time = opts.delta_time,
            .collision_steps = opts.collision_steps,
        };
        errdefer self.temp_allocator.deinit();
        try self.job_system.init(allocator, io, @intCast(zolt.physics_settings.max_physics_jobs), @intCast(zolt.physics_settings.max_physics_barriers), .{ .num_threads = opts.worker_threads });
        errdefer self.job_system.deinit();

        // Create physics system
        try self.system.init(allocator, io, opts.max_bodies, 0, opts.max_body_pairs, opts.max_contact_constraints, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_vs_object_layer_filter.base, .{});
    }

    /// Destructor
    pub fn deinit(self: *PhysicsTestContext) void {
        self.system.deinit();
        self.job_system.deinit();
        self.temp_allocator.deinit();
    }

    /// Set the gravity to zero
    pub fn zeroGravity(self: *PhysicsTestContext) void {
        self.system.setGravity(Vec3.zero());
    }

    /// Create a floor at Y = 0
    pub fn createFloor(self: *PhysicsTestContext) !*Body {
        var settings: BodyCreationSettings = .{};
        defer settings.deinit();
        settings.setShape((try BoxShape.create(self.allocator, Vec3.init(100.0, 1.0, 100.0), .{ .convex_radius = 0.0 })).asShape());
        settings.position = RVec3.init(0.0, -1.0, 0.0);
        settings.motion_type = .static;
        settings.object_layer = Layers.Layers.non_moving;
        const bi = self.getBodyInterface();
        const floor = (try bi.createBody(&settings)) orelse return error.OutOfBodies;
        try bi.addBody(floor.getID(), .dont_activate);
        return floor;
    }

    /// Create a body and add it to the world
    pub fn createBody(self: *PhysicsTestContext, settings: *const BodyCreationSettings, activation: Activation) !*Body {
        const bi = self.getBodyInterface();
        const body = (try bi.createBody(settings)) orelse return error.OutOfBodies;
        try bi.addBody(body.getID(), activation);
        return body;
    }

    /// Create a body and add it to the world (Jolt: the CreateBody overload with a ShapeSettings)
    pub fn createBodyWithShapeSettings(self: *PhysicsTestContext, shape_settings: *ShapeSettings, position: RVec3, rotation: Quat, motion_type: MotionType, motion_quality: MotionQuality, layer: ObjectLayer, activation: Activation) !*Body {
        var settings: BodyCreationSettings = .{};
        defer settings.deinit();
        settings.setShapeSettings(shape_settings);
        settings.position = position;
        settings.rotation = rotation;
        settings.motion_type = motion_type;
        settings.motion_quality = motion_quality;
        settings.object_layer = layer;
        settings.linear_damping = 0.0;
        settings.angular_damping = 0.0;
        settings.collision_group.setGroupID(0);
        return self.createBody(&settings, activation);
    }

    /// Create a box and add it to the world
    pub fn createBox(self: *PhysicsTestContext, position: RVec3, rotation: Quat, motion_type: MotionType, motion_quality: MotionQuality, layer: ObjectLayer, half_extent: Vec3, opts: struct { activation: Activation = .activate }) !*Body {
        const shape_settings = try BoxShapeSettings.create(self.allocator, half_extent, .{});
        return self.createBodyWithShapeSettings(shape_settings.asShapeSettings(), position, rotation, motion_type, motion_quality, layer, opts.activation);
    }

    /// Create a sphere and add it to the world
    pub fn createSphere(self: *PhysicsTestContext, position: RVec3, radius: f32, motion_type: MotionType, motion_quality: MotionQuality, layer: ObjectLayer, opts: struct { activation: Activation = .activate }) !*Body {
        const shape_settings = try SphereShapeSettings.create(self.allocator, radius, .{});
        return self.createBodyWithShapeSettings(shape_settings.asShapeSettings(), position, Quat.identity(), motion_type, motion_quality, layer, opts.activation);
    }

    /// Create a constraint and add it to the world (`T` is the constraint class, its `base` is a TwoBodyConstraint)
    pub fn createConstraint(self: *PhysicsTestContext, comptime T: type, body1: *Body, body2: *Body, settings: *const TwoBodyConstraintSettings) !*T {
        const constraint = try settings.createConstraint(self.allocator, body1, body2);

        // Jolt: AddConstraint takes the first reference; Zolt: a temporary one, so that the constraint is freed when
        // AddConstraint fails
        var ref = zolt.Ref(zolt.Constraint).init(&constraint.base);
        defer ref.deinit();
        try self.system.addConstraint(&constraint.base);
        return zolt.virtual.downcast(T, constraint);
    }

    /// Call the update with zero delta time
    pub fn simulateNoDeltaTime(self: *PhysicsTestContext) !PhysicsUpdateError {
        const errors = try self.system.update(0.0, self.collision_steps, self.temp_allocator.tempAllocator(), self.job_system.jobSystem());
        if (!self.temp_allocator.isEmpty()) return error.TestUnexpectedResult;
        return errors;
    }

    /// Simulate only for one delta time step
    pub fn simulateSingleStep(self: *PhysicsTestContext) !PhysicsUpdateError {
        const errors = try self.system.update(self.delta_time, self.collision_steps, self.temp_allocator.tempAllocator(), self.job_system.jobSystem());
        if (!self.temp_allocator.isEmpty()) return error.TestUnexpectedResult;
        // TODO(debug_renderer): DrawBodies / DrawConstraints / EndFrame of the debug renderer
        return errors;
    }

    /// Simulate the world for inTotalTime time (`pre_step_callback`: `{}` or a value with a `call()` method that may fail)
    pub fn simulate(self: *PhysicsTestContext, total_time: f32, pre_step_callback: anytype) !PhysicsUpdateError {
        var errors: PhysicsUpdateError = .none;
        const num_steps: i32 = @intFromFloat(@round(total_time / self.delta_time));
        var s: i32 = 0;
        while (s < num_steps) : (s += 1) {
            if (@TypeOf(pre_step_callback) != void)
                try pre_step_callback.call();
            errors = errors.bitOr(try self.simulateSingleStep());
        }
        return errors;
    }

    /// Predict position assuming ballistic motion using initial position, velocity acceleration and time
    pub fn predictPosition(self: *const PhysicsTestContext, position: RVec3, velocity: Vec3, acceleration: Vec3, total_time: f32) RVec3 {
        // Integrate position using a Symplectic Euler step (just like the PhysicsSystem)
        var pos = position;
        var vel = velocity;
        const delta_time = self.getStepDeltaTime();
        const num_steps: i32 = @intFromFloat(@round(total_time / delta_time));
        var s: i32 = 0;
        while (s < num_steps) : (s += 1) {
            vel = vel.add(acceleration.mulScalar(delta_time));
            pos = if (Core.double_precision) pos.addVec3(vel.mulScalar(delta_time)) else pos.add(vel.mulScalar(delta_time));
        }
        return pos;
    }

    /// Predict rotation assuming ballistic motion using initial orientation, angular velocity angular acceleration and time
    pub fn predictOrientation(self: *const PhysicsTestContext, rotation: Quat, angular_velocity: Vec3, angular_acceleration: Vec3, total_time: f32) Quat {
        // Integrate position using a Symplectic Euler step (just like the PhysicsSystem)
        var rot = rotation;
        var vel = angular_velocity;
        const delta_time = self.getStepDeltaTime();
        const num_steps: i32 = @intFromFloat(@round(total_time / delta_time));
        var s: i32 = 0;
        while (s < num_steps) : (s += 1) {
            vel = vel.add(angular_acceleration.mulScalar(delta_time));
            const vel_len = vel.length();
            if (vel_len != 0.0)
                rot = Quat.rotation(vel.divScalar(vel_len), vel_len * delta_time).mul(rot);
        }
        return rot;
    }

    /// Access to the physics system
    pub fn getSystem(self: *PhysicsTestContext) *PhysicsSystem {
        return &self.system;
    }

    /// Access to the body interface
    pub fn getBodyInterface(self: *PhysicsTestContext) *BodyInterface {
        return self.system.getBodyInterfaceMut();
    }

    /// Get delta time for simulation step
    pub fn getDeltaTime(self: *const PhysicsTestContext) f32 {
        return self.delta_time;
    }

    /// Get delta time for a simulation collision step
    pub fn getStepDeltaTime(self: *const PhysicsTestContext) f32 {
        return self.delta_time / @as(f32, @floatFromInt(self.collision_steps));
    }

    /// Get the temporary allocator
    pub fn getTempAllocator(self: *PhysicsTestContext) TempAllocator {
        return self.temp_allocator.tempAllocator();
    }

    /// Get the job system
    pub fn getJobSystem(self: *PhysicsTestContext) JobSystem {
        return self.job_system.jobSystem();
    }

    // TODO(debug_renderer): RecordDebugOutput
};
