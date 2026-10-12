//! Port of: HelloWorld/HelloWorld.cpp
//!
//! Milestone M5 (SimulationArchitecture section 3): Jolt's HelloWorld example, ported statement by statement with the
//! same constants, the same layers and listeners and the same output. `zig build hello-world` builds and runs it;
//! `zig build hello-world-check` (part of `zig build parity`) also builds the C++ HelloWorld against the parity step's
//! C++ Jolt library, runs both and fails unless their stdout is byte identical. ZoltParity/Physics/HelloWorldParity.zig
//! compares the scene bit for bit after every step.
//!
//! What differs from the C++ program:
//! - `RegisterDefaultAllocator`, the Factory and `RegisterTypes` / `UnregisterTypes` have nothing to do: Zolt takes an
//!   explicit allocator (`init.gpa`, which checks for leaks in Debug builds), and its type registrations (shapes,
//!   collision dispatch, materials) are comptime.
//! - `Trace = TraceImpl` is `std_options.logFn`: every log message (Zolt's `Trace` is `std.log.info`) is formatted into
//!   a 1024 byte buffer like vsnprintf and printed to stdout. Zolt's asserts panic (`std.debug.assert`), so there is no
//!   `AssertFailed` hook; `JPH_ASSERT(false)` in the layer classes is `if (Core.enable_asserts) @panic(...)`.
//! - `cout << ... << endl` is `printLine`: one line written to stdout and flushed. Floats are printed like
//!   `std::ostream` prints them (`zolt.cppFloat`, printf "%g"), so the text is Jolt's. The listeners keep the `io` that
//!   writing needs (they are called from the job system's threads).
//! - The C++ destructors at the end of `main` are `defer`s in the order of construction (destroyed in reverse order).

const std = @import("std");
const zolt = @import("zolt");

const Core = zolt.Core;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShape = zolt.SphereShape;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const ContactListener = zolt.ContactListener;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const CollideShapeResult = zolt.CollideShapeResult;
const SubShapeIDPair = zolt.SubShapeIDPair;
const ValidateResult = zolt.ValidateResult;
const BodyActivationListener = zolt.BodyActivationListener;
const PhysicsSystem = zolt.PhysicsSystem;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const physics_settings = zolt.physics_settings;
const cppFloat = zolt.cppFloat;

/// Callback for traces (Jolt's `Trace = TraceImpl`): every log message of the program and the library goes to stdout
pub const std_options: std.Options = .{
    .log_level = .info,
    .logFn = traceImpl,
};

/// Callback for traces, connect this to your own trace function if you have one
fn traceImpl(comptime level: std.log.Level, comptime scope: @EnumLiteral(), comptime format: []const u8, args: anytype) void {
    _ = level;
    _ = scope;

    // Format the message (vsnprintf into a char[1024]: at most 1023 characters)
    var buffer: [1024]u8 = undefined;
    var writer: std.Io.Writer = .fixed(buffer[0 .. buffer.len - 1]);
    writer.print(format, args) catch {}; // A fixed writer fills the buffer before it fails: the message is truncated
    const len = writer.end;

    // Print to the TTY (cout << buffer << endl)
    buffer[len] = '\n';
    std.Io.File.stdout().writeStreamingAll(std.Options.debug_io, buffer[0 .. len + 1]) catch {};
}

/// `cout << ... << endl`: print a line to stdout and flush it (errors are ignored like those of a stream)
fn printLine(io: std.Io, comptime format: []const u8, args: anytype) void {
    var buffer: [1024]u8 = undefined;
    var stdout = std.Io.File.stdout().writerStreaming(io, &buffer);
    stdout.interface.print(format ++ "\n", args) catch return;
    stdout.interface.flush() catch return;
}

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
const Layers = struct {
    const non_moving: ObjectLayer = 0;
    const moving: ObjectLayer = 1;
    const num_layers: ObjectLayer = 2;
};

/// Class that determines if two object layers can collide
const ObjectLayerPairFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerPairFilter = .init(ObjectLayerPairFilterImpl),

    pub fn shouldCollide(self: *const ObjectLayerPairFilterImpl, object1: ObjectLayer, object2: ObjectLayer) bool {
        _ = self;
        switch (object1) {
            Layers.non_moving => return object2 == Layers.moving, // Non moving only collides with moving
            Layers.moving => return true, // Moving collides with everything
            else => {
                if (Core.enable_asserts) @panic("invalid object layer"); // JPH_ASSERT(false)
                return false;
            },
        }
    }
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
const BroadPhaseLayers = struct {
    const non_moving: BroadPhaseLayer = .init(0);
    const moving: BroadPhaseLayer = .init(1);
    const num_layers: u32 = 2;
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
const BPLayerInterfaceImpl = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(BPLayerInterfaceImpl),
    object_to_broad_phase: [Layers.num_layers]BroadPhaseLayer,

    pub fn init() BPLayerInterfaceImpl {
        var self: BPLayerInterfaceImpl = .{ .object_to_broad_phase = undefined };

        // Create a mapping table from object to broad phase layer
        self.object_to_broad_phase[Layers.non_moving] = BroadPhaseLayers.non_moving;
        self.object_to_broad_phase[Layers.moving] = BroadPhaseLayers.moving;
        return self;
    }

    pub fn getNumBroadPhaseLayers(self: *const BPLayerInterfaceImpl) u32 {
        _ = self;
        return BroadPhaseLayers.num_layers;
    }

    pub fn getBroadPhaseLayer(self: *const BPLayerInterfaceImpl, layer: ObjectLayer) BroadPhaseLayer {
        if (Core.enable_asserts) std.debug.assert(layer < Layers.num_layers);
        return self.object_to_broad_phase[layer];
    }

    // GetBroadPhaseLayerName (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED): Zolt has no profiler
};

/// Class that determines if an object layer can collide with a broadphase layer
const ObjectVsBroadPhaseLayerFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectVsBroadPhaseLayerFilter = .init(ObjectVsBroadPhaseLayerFilterImpl),

    pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilterImpl, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        _ = self;
        switch (layer1) {
            Layers.non_moving => return layer2.eql(BroadPhaseLayers.moving),
            Layers.moving => return true,
            else => {
                if (Core.enable_asserts) @panic("invalid object layer"); // JPH_ASSERT(false)
                return false;
            },
        }
    }
};

// An example contact listener
const MyContactListener = struct {
    io: std.Io,

    // See: ContactListener
    pub fn onContactValidate(self: *MyContactListener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        _ = body1;
        _ = body2;
        _ = base_offset;
        _ = collision_result;
        printLine(self.io, "Contact validate callback", .{});

        // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
        return .accept_all_contacts_for_this_body_pair;
    }

    pub fn onContactAdded(self: *MyContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        _ = body1;
        _ = body2;
        _ = manifold;
        _ = settings;
        printLine(self.io, "A contact was added", .{});
    }

    pub fn onContactPersisted(self: *MyContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        _ = body1;
        _ = body2;
        _ = manifold;
        _ = settings;
        printLine(self.io, "A contact was persisted", .{});
    }

    pub fn onContactRemoved(self: *MyContactListener, sub_shape_pair: *const SubShapeIDPair) void {
        _ = sub_shape_pair;
        printLine(self.io, "A contact was removed", .{});
    }
};

// An example activation listener
const MyBodyActivationListener = struct {
    io: std.Io,

    pub fn onBodyActivated(self: *MyBodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        _ = body_id;
        _ = body_user_data;
        printLine(self.io, "A body got activated", .{});
    }

    pub fn onBodyDeactivated(self: *MyBodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        _ = body_id;
        _ = body_user_data;
        printLine(self.io, "A body went to sleep", .{});
    }
};

/// thread::hardware_concurrency(): the number of CPUs, 0 when it cannot be determined
fn hardwareConcurrency() u32 {
    const count = std.Thread.getCpuCount() catch return 0;
    return std.math.cast(u32, count) orelse std.math.maxInt(u32);
}

// Program entry point
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const io = init.io;

    // Register allocation hook: Zolt has no global allocator, the allocator is passed to every function that allocates.

    // Install trace and assert callbacks: see std_options (Zolt's asserts panic).

    // Create a factory, register all physics types with the factory and install their collision handlers with the
    // CollisionDispatch class: Zolt's registrations are comptime (Zolt/RegisterTypes.zig), there is nothing to do.

    // We need a temp allocator for temporary allocations during the physics update. We're
    // pre-allocating 10 MB to avoid having to do allocations during the physics update.
    // B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
    // If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
    // malloc / free.
    var temp_allocator: TempAllocatorImpl = try .init(allocator, 10 * 1024 * 1024);
    defer temp_allocator.deinit();

    // We need a job system that will execute physics jobs on multiple threads. Typically
    // you would implement the JobSystem interface yourself and let Jolt Physics run on top
    // of your own job scheduler. JobSystemThreadPool is an example implementation.
    var job_system: JobSystemThreadPool = .empty;
    try job_system.init(allocator, io, @intCast(physics_settings.max_physics_jobs), @intCast(physics_settings.max_physics_barriers), .{ .num_threads = @bitCast(hardwareConcurrency() -% 1) });
    defer job_system.deinit();

    // This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
    // Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
    const max_bodies: u32 = 1024;

    // This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
    const num_body_mutexes: u32 = 0;

    // This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
    // body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
    // too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
    // Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
    const max_body_pairs: u32 = 1024;

    // This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
    // number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
    // Note: This value is low because this is a simple test. For a real project use something in the order of 10240.
    const max_contact_constraints: u32 = 1024;

    // Create mapping table from object layer to broadphase layer
    // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
    // Also have a look at BroadPhaseLayerInterfaceTable or BroadPhaseLayerInterfaceMask for a simpler interface.
    const broad_phase_layer_interface: BPLayerInterfaceImpl = .init();

    // Create class that filters object vs broadphase layers
    // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
    // Also have a look at ObjectVsBroadPhaseLayerFilterTable or ObjectVsBroadPhaseLayerFilterMask for a simpler interface.
    const object_vs_broadphase_layer_filter: ObjectVsBroadPhaseLayerFilterImpl = .{};

    // Create class that filters object vs object layers
    // Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
    // Also have a look at ObjectLayerPairFilterTable or ObjectLayerPairFilterMask for a simpler interface.
    const object_vs_object_layer_filter: ObjectLayerPairFilterImpl = .{};

    // Now we can create the actual physics system.
    var physics_system: PhysicsSystem = .empty;
    try physics_system.init(allocator, io, max_bodies, num_body_mutexes, max_body_pairs, max_contact_constraints, &broad_phase_layer_interface.base, &object_vs_broadphase_layer_filter.base, &object_vs_object_layer_filter.base, .{});
    defer physics_system.deinit();

    // A body activation listener gets notified when bodies activate and go to sleep
    // Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    var body_activation_listener: MyBodyActivationListener = .{ .io = io };
    physics_system.setBodyActivationListener(.init(&body_activation_listener));

    // A contact listener gets notified when bodies (are about to) collide, and when they separate again.
    // Note that this is called from a job so whatever you do here needs to be thread safe.
    // Registering one is entirely optional.
    var contact_listener: MyContactListener = .{ .io = io };
    physics_system.setContactListener(.init(&contact_listener));

    // The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
    // variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
    const body_interface = physics_system.getBodyInterfaceMut();

    // Next we can create a rigid body to serve as the floor, we make a large box
    // Create the settings for the collision volume (the shape).
    // Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
    var floor_shape_settings: BoxShapeSettings = .init(allocator, Vec3.init(100.0, 1.0, 100.0), .{});
    floor_shape_settings.asShapeSettings().setEmbedded(); // A ref counted object on the stack (base class RefTarget) should be marked as such to prevent it from being freed when its reference count goes to 0.
    defer floor_shape_settings.deinit();

    // Create the shape
    var floor_shape_result = try floor_shape_settings.createShape(allocator);
    defer floor_shape_result.deinit();
    var floor_shape = floor_shape_result.get().clone(); // We don't expect an error here, but you can check floor_shape_result for hasError() / getError()
    defer floor_shape.deinit();

    // Create the settings for the body itself. Note that here you can also set other properties like the restitution / friction.
    var floor_settings: BodyCreationSettings = .initWithShape(floor_shape.get(), RVec3.init(0.0, -1.0, 0.0), Quat.identity(), .static, Layers.non_moving);
    defer floor_settings.deinit();

    // Create the actual rigid body
    const floor = (try body_interface.createBody(&floor_settings)).?; // Note that if we run out of bodies this can return null

    // Add it to the world
    try body_interface.addBody(floor.getID(), .dont_activate);

    // Now create a dynamic body to bounce on the floor
    // Note that this uses the shorthand version of creating and adding a body to the world
    var sphere_settings: BodyCreationSettings = .initWithShape((try SphereShape.create(allocator, 0.5, .{})).asShape(), RVec3.init(0.0, 2.0, 0.0), Quat.identity(), .dynamic, Layers.moving);
    defer sphere_settings.deinit();
    const sphere_id = try body_interface.createAndAddBody(&sphere_settings, .activate);

    // Now you can interact with the dynamic body, in this case we're going to give it a velocity.
    // (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)
    body_interface.setLinearVelocity(sphere_id, Vec3.init(0.0, -5.0, 0.0));

    // We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
    const delta_time: f32 = 1.0 / 60.0;

    // Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
    // You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
    // Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
    try physics_system.optimizeBroadPhase();

    // Now we're ready to simulate the body, keep simulating until it goes to sleep
    var step: u32 = 0;
    while (body_interface.isActive(sphere_id)) {
        // Next step
        step += 1;

        // Output current position and velocity of the sphere
        const position = body_interface.getCenterOfMassPosition(sphere_id);
        const velocity = body_interface.getLinearVelocity(sphere_id);
        printLine(io, "Step {d}: Position = ({f}, {f}, {f}), Velocity = ({f}, {f}, {f})", .{ step, cppFloat(position.getX()), cppFloat(position.getY()), cppFloat(position.getZ()), cppFloat(velocity.getX()), cppFloat(velocity.getY()), cppFloat(velocity.getZ()) });

        // If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
        const collision_steps: u32 = 1;

        // Step the world
        _ = try physics_system.update(delta_time, collision_steps, temp_allocator.tempAllocator(), job_system.jobSystem());
    }

    // Remove the sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
    body_interface.removeBody(sphere_id);

    // Destroy the sphere. After this the sphere ID is no longer valid.
    body_interface.destroyBody(sphere_id);

    // Remove and destroy the floor
    body_interface.removeBody(floor.getID());
    body_interface.destroyBody(floor.getID());

    // Unregisters all types with the factory and cleans up the default material, destroy the factory: nothing to do
    // (comptime registrations, the default material is a static constant).
}
