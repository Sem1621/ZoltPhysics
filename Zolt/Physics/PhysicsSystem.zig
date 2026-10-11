//! Port of: Jolt/Physics/PhysicsSystem.h, Jolt/Physics/PhysicsSystem.cpp
//! Status: partial
//! Missing: the job bodies that need parts of Waves A and B, each marked `// REGION Rn` (SimulationArchitecture section 2):
//! REGION R2 ProcessBodyPair (the narrow phase and the contact constraints, needs the ContactConstraintManager of A4);
//! REGION R3 the constraint and contact solving in JobSolveVelocityConstraints / JobSolvePositionConstraints (needs A4,
//! the LargeIslandSplitter of A5 and CalculateSolverSteps of A2); REGION R4 the narrow phase cast of JobFindCCDContacts,
//! sSolveCCDContact and the hit path of JobResolveCCDContacts (needs A4 and A7); the soft body jobs beyond the "no
//! active soft bodies" path of JobSoftBodyPrepare (JobSoftBodyCollide, JobSoftBodySimulate, JobSoftBodyFinalize,
//! Phase 9). Every stub panics where Jolt would do work it cannot do yet, so a result is never silently wrong: the
//! gravity-only path (no body pair reaches the narrow phase, no constraint or contact in an island, no CCD hit, no soft
//! body) runs Jolt's code end to end.
//!
//! SimulationArchitecture D4 / D5 / D6 / D7:
//! - `update` builds Jolt's complete job graph per collision step with Jolt's dependency counts, creation order and kick
//!   order. Every C++ lambda is an inline `struct { fn call(...) }.call` closure at the lambda's position whose captures
//!   are the lambda's captures (`[&context, &step]` -> `.{ context, step }`). Jobs created inside jobs
//!   (TrySpawnJobFindCollisions, the FindCCDContacts jobs) are added to the barrier, as in Jolt.
//! - Start gate (D4): the first step's root jobs get one dependency (Jolt: 0) that `kickStep` (the kick of StartNextStep)
//!   removes after the whole graph exists, so a failing createJob while building has no side effect.
//! - Out of memory (D5): `update` returns `error{OutOfMemory}!PhysicsUpdateError`. A failure while building the jobs (or
//!   in the allocations before) changes nothing. Inside a job an allocation failure sets the latch of the context; a
//!   poisoned job skips its work and allocations but always does its frees, its commits and its dependency bookkeeping,
//!   and the protocol jobs (BroadPhasePrepare, UpdateBroadPhaseFinalize, FindCollisions, PostIntegrateVelocity,
//!   ResolveCCDContacts, ContactRemovedCallbacks, StartNextStep, SoftBodyPrepare) follow D5's table. A job that cannot be
//!   created from inside a job is not an error. Temp memory is `TempArray`, freed at Jolt's positions.
//! - The PhysicsSystem is initialized in place (`var system: PhysicsSystem = .empty; try system.init(...)`) because the
//!   lock and body interfaces point into it; it must not move afterwards. `init` takes Jolt's arguments plus
//!   `allocator`, `io` and an options struct: `.broad_phase = .brute_force` (default until A1 ports the QuadTree, which
//!   Jolt's BROAD_PHASE define selects) or `.custom = *BroadPhase` (a Zolt addition: a broad phase that the system owns
//!   from a successful init on).
//! - Const / non-const overloads: `getBodyInterface` / `getBodyInterfaceMut`, `getBodyInterfaceNoLock` /
//!   `getBodyInterfaceNoLockMut`. Listeners are stored by value (pattern B, D7): `?ContactListener`,
//!   `?BodyActivationListener`, `?SoftBodyContactListener`; `AddStepListener` keeps the `PhysicsStepListener` value
//!   (Jolt: the pointer), found again by its `ptr`.
//! - `SimCollideBodyVsBody` (a std::function) is a function pointer + context, `call` passes the allocator that the
//!   InternalEdgeRemovingCollector of the default function needs.
//! - Defaulted parameters are options: `saveState(stream, .{ .state = .all, .filter = null })`,
//!   `restoreState(stream, .{ .filter = null })`. Functions that allocate return Allocator.Error (`getBodies`,
//!   `getActiveBodies`, `getConstraints`, `addConstraint(s)`, `addStepListener`, `saveState`, `restoreState`).
//! - JPH_DET_LOG: TODO(determinism_log). JPH_DEBUG_RENDERER (sDrawMotionQualityLinearCast, DrawBodies,
//!   DrawConstraints, DrawConstraintLimits, DrawConstraintReferenceFrame): TODO(debug_renderer).
//!   JPH_TRACK_SIMULATION_STATS (GatherIslandStats, the tick counters) follows MotionProperties'
//!   comptime switch `track_simulation_stats` (off). JPH_TRACK_BROADPHASE_STATS (ReportBroadphaseStats) and
//!   ReportSimulationStats are not ported with the profiler. JPH_PROFILE is dropped.

const std = @import("std");
const builtin = @import("builtin");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const math = @import("../Math/Math.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Quat = @import("../Math/Quat.zig").Quat;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const RealFile = @import("../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const AABox = @import("../Geometry/AABox.zig").AABox;
const RayAABoxFile = @import("../Geometry/RayAABox.zig");
const Color = @import("../Core/Color.zig").Color;
const Mutex = @import("../Core/Mutex.zig").Mutex;
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const quickSort = @import("../Core/QuickSort.zig").quickSort;
const JobSystemFile = @import("../Core/JobSystem.zig");
const JobSystem = JobSystemFile.JobSystem;
const JobHandle = JobSystem.JobHandle;
const TempAllocatorFile = @import("../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const getProcessorTickCount = @import("../Core/TickCounter.zig").getProcessorTickCount;
const physics_settings_file = @import("PhysicsSettings.zig");
const PhysicsSettings = physics_settings_file.PhysicsSettings;
const Body = @import("Body/Body.zig").Body;
const BodyID = @import("Body/BodyID.zig").BodyID;
const BodyPair = @import("Body/BodyPair.zig").BodyPair;
const BodyType = @import("Body/BodyType.zig").BodyType;
const BodyAccess = @import("Body/BodyAccess.zig").BodyAccess;
const BodyInterface = @import("Body/BodyInterface.zig").BodyInterface;
const BodyLockInterface = @import("Body/BodyLockInterface.zig").BodyLockInterface;
const BodyActivationListener = @import("Body/BodyActivationListener.zig").BodyActivationListener;
const BodyManagerFile = @import("Body/BodyManager.zig");
const BodyManager = BodyManagerFile.BodyManager;
const BodyIDVector = BodyManagerFile.BodyIDVector;
const MotionPropertiesFile = @import("Body/MotionProperties.zig");
const CanSleep = MotionPropertiesFile.CanSleep;
const track_simulation_stats = MotionPropertiesFile.track_simulation_stats;
const ObjectLayerFile = @import("Collision/ObjectLayer.zig");
const ObjectLayer = ObjectLayerFile.ObjectLayer;
const ObjectLayerPairFilter = ObjectLayerFile.ObjectLayerPairFilter;
const DefaultObjectLayerFilter = ObjectLayerFile.DefaultObjectLayerFilter;
const BroadPhaseLayerFile = @import("Collision/BroadPhase/BroadPhaseLayer.zig");
const BroadPhaseLayerInterface = BroadPhaseLayerFile.BroadPhaseLayerInterface;
const ObjectVsBroadPhaseLayerFilter = BroadPhaseLayerFile.ObjectVsBroadPhaseLayerFilter;
const DefaultBroadPhaseLayerFilter = BroadPhaseLayerFile.DefaultBroadPhaseLayerFilter;
const BroadPhaseFile = @import("Collision/BroadPhase/BroadPhase.zig");
const BroadPhase = BroadPhaseFile.BroadPhase;
const BodyPairCollector = BroadPhaseFile.BodyPairCollector;
const BroadPhaseQueryFile = @import("Collision/BroadPhase/BroadPhaseQuery.zig");
const BroadPhaseQuery = BroadPhaseQueryFile.BroadPhaseQuery;
const CastShapeBodyCollector = BroadPhaseQueryFile.CastShapeBodyCollector;
const BroadPhaseBruteForce = @import("Collision/BroadPhase/BroadPhaseBruteForce.zig").BroadPhaseBruteForce;
const BroadPhaseCastResult = @import("Collision/CastResult.zig").BroadPhaseCastResult;
const RShapeCast = @import("Collision/ShapeCast.zig").RShapeCast;
const ShapeFile = @import("Collision/Shape/Shape.zig");
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const SubShapeIDCreator = @import("Collision/Shape/SubShapeID.zig").SubShapeIDCreator;
const CollideShapeSettings = @import("Collision/CollideShape.zig").CollideShapeSettings;
const CollisionDispatch = @import("Collision/CollisionDispatch.zig");
const InternalEdgeRemovingCollector = @import("Collision/InternalEdgeRemovingCollector.zig").InternalEdgeRemovingCollector;
const ShapeFilter = @import("Collision/ShapeFilter.zig").ShapeFilter;
const SimShapeFilter = @import("Collision/SimShapeFilter.zig").SimShapeFilter;
const ContactListener = @import("Collision/ContactListener.zig").ContactListener;
const NarrowPhaseQuery = @import("Collision/NarrowPhaseQuery.zig").NarrowPhaseQuery;
const ConstraintFile = @import("Constraints/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintManagerFile = @import("Constraints/ConstraintManager.zig");
const ConstraintManager = ConstraintManagerFile.ConstraintManager;
const Constraints = ConstraintManagerFile.Constraints;
const ContactConstraintManager = @import("Constraints/ContactConstraintManager.zig").ContactConstraintManager;
const SoftBodyContactListener = @import("SoftBody/SoftBodyContactListener.zig").SoftBodyContactListener;
const StateRecorderFile = @import("StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderState = StateRecorderFile.StateRecorderState;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;
const PhysicsUpdateError = @import("EPhysicsUpdateError.zig").PhysicsUpdateError;
const PhysicsStepListenerFile = @import("PhysicsStepListener.zig");
const PhysicsStepListener = PhysicsStepListenerFile.PhysicsStepListener;
const PhysicsStepListenerContext = PhysicsStepListenerFile.PhysicsStepListenerContext;
const PhysicsUpdateContext = @import("PhysicsUpdateContext.zig").PhysicsUpdateContext;
const IslandBuilder = @import("IslandBuilder.zig").IslandBuilder;
const LargeIslandSplitter = @import("LargeIslandSplitter.zig").LargeIslandSplitter;

const color_update_broad_phase_finalize = Color.getDistinctColor(1);
const color_update_broad_phase_prepare = Color.getDistinctColor(2);
const color_find_collisions = Color.getDistinctColor(3);
const color_apply_gravity = Color.getDistinctColor(4);
const color_setup_velocity_constraints = Color.getDistinctColor(5);
const color_build_islands_from_constraints = Color.getDistinctColor(6);
const color_determine_active_constraints = Color.getDistinctColor(7);
const color_finalize_islands = Color.getDistinctColor(8);
const color_contact_removed_callbacks = Color.getDistinctColor(9);
const color_body_set_island_index = Color.getDistinctColor(10);
const color_start_next_step = Color.getDistinctColor(11);
const color_solve_velocity_constraints = Color.getDistinctColor(12);
const color_pre_integrate_velocity = Color.getDistinctColor(13);
const color_integrate_velocity = Color.getDistinctColor(14);
const color_post_integrate_velocity = Color.getDistinctColor(15);
const color_resolve_ccd_contacts = Color.getDistinctColor(16);
const color_solve_position_constraints = Color.getDistinctColor(17);
const color_find_ccd_contacts = Color.getDistinctColor(18);
const color_step_listeners = Color.getDistinctColor(19);
const color_soft_body_prepare = Color.getDistinctColor(20);
const color_soft_body_collide = Color.getDistinctColor(21);
const color_soft_body_simulate = Color.getDistinctColor(22);
const color_soft_body_finalize = Color.getDistinctColor(23);

/// Jolt's `Vec3(RVec3)`: identity in single precision, DVec3.toVec3 in double precision
fn toVec3(v: RVec3) Vec3 {
    return if (Core.double_precision) v.toVec3() else v;
}

/// The main class for the physics system. It contains all rigid bodies and simulates them.
///
/// The main simulation is performed by the Update() call on multiple threads (if the JobSystem is configured to use them). Please refer to the general architecture overview in the Docs folder for more information.
pub const PhysicsSystem = struct {
    const Self = @This();
    const Step = PhysicsUpdateContext.Step;
    const CCDBody = Step.CCDBody;
    const JobMask = PhysicsUpdateContext.JobMask;
    const ContactAllocator = ContactConstraintManager.ContactAllocator;

    /// The maximum value that can be passed to Init for inMaxBodies.
    pub const max_bodies_limit: u32 = BodyID.max_body_index + 1;

    /// The maximum value that can be passed to Init for inMaxBodyPairs.
    /// Note you should really use a lower value, using this value will cost a lot of memory!
    /// On a 32 bit platform, you'll run out of memory way before you reach this limit.
    pub const max_body_pairs_limit: u32 = ContactConstraintManager.max_body_pairs_limit;

    /// The maximum value that can be passed to Init for inMaxContactConstraints.
    /// Note you should really use a lower value, using this value will cost a lot of memory!
    /// On a 32 bit platform, you'll run out of memory way before you reach this limit.
    pub const max_contact_constraints_limit: u32 = ContactConstraintManager.max_contact_constraints_limit;

    /// Helper struct that counts the number of bodies of each type
    pub const BodyStats = BodyManager.BodyStats;

    /// Advanced use only: This function is similar to CollisionDispatch::sCollideShapeVsShape but only used to collide bodies during simulation.
    /// inBody1 The first body to collide.
    /// inBody2 The second body to collide.
    /// inCenterOfMassTransform1 The center of mass transform of the first body (note this will not be the actual world space position of the body, it will be made relative to some position so we can drop down to single precision).
    /// inCenterOfMassTransform2 The center of mass transform of the second body.
    /// ioCollideShapeSettings Settings that control the collision detection. Note that the implementation can freely overwrite the shape settings if needed, the caller provides a temporary that will not be used after the function returns.
    /// ioCollector The collector that will receive the contact points.
    /// inShapeFilter The shape filter that can be used to exclude shapes from colliding with each other.
    /// (std::function: a function pointer + context; `allocator` is the heap of the InternalEdgeRemovingCollector)
    pub const SimCollideBodyVsBody = struct {
        pub const Function = *const fn (context: ?*anyopaque, allocator: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void;

        function: Function,
        context: ?*anyopaque = null,

        /// PhysicsSystem::sDefaultSimCollideBodyVsBody
        pub const default: SimCollideBodyVsBody = .{ .function = defaultThunk };

        fn defaultThunk(context: ?*anyopaque, allocator: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
            _ = context;
            return defaultSimCollideBodyVsBody(allocator, body1, body2, center_of_mass_transform1, center_of_mass_transform2, collide_shape_settings, collector, shape_filter);
        }

        /// Call the function
        pub fn call(self: *const SimCollideBodyVsBody, allocator: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
            return self.function(self.context, allocator, body1, body2, center_of_mass_transform1, center_of_mass_transform2, collide_shape_settings, collector, shape_filter);
        }
    };

    /// The broad phase that `init` creates (Zolt option, Jolt: the BROAD_PHASE define)
    pub const BroadPhaseOption = union(enum) {
        /// BroadPhaseBruteForce (the default until A1 ports BroadPhaseQuadTree, Jolt's default)
        brute_force,
        /// A broad phase created by the caller; the system owns it after a successful init (it destroys it in deinit)
        custom: *BroadPhase,
    };

    /// Zolt options of `init`
    pub const InitOptions = struct {
        broad_phase: BroadPhaseOption = .brute_force,
    };

    /// Number of constraints to process at once in JobDetermineActiveConstraints
    const determine_active_constraints_batch_size = 64;

    /// Number of constraints to process at once in JobSetupVelocityConstraints, we want a low number of threads working on this so we take fairly large batches
    const setup_velocity_constraints_batch_size = 256;

    /// Number of bodies to process at once in JobApplyGravity
    const apply_gravity_batch_size = 64;

    /// Number of active bodies to test for collisions per batch
    const active_bodies_batch_size = 16;

    /// Number of active bodies to integrate velocities for
    const integrate_velocity_batch_size = 64;

    /// Number of contacts that need to be queued before another narrow phase job is started
    const narrow_phase_batch_size = 16;

    /// Number of continuous collision shape casts that need to be queued before another job is started
    const num_ccd_bodies_per_job = 4;

    /// Allocates the broad phase, the body manager, the managers and the temporary arrays of the functions that copy
    allocator: Allocator = undefined,

    /// The io of the mutexes
    io: std.Io = undefined,

    /// Broadphase layer filter that decides if two objects can collide
    object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter = undefined,

    /// Object layer filter that decides if two objects can collide
    object_layer_pair_filter: *const ObjectLayerPairFilter = undefined,

    /// The body manager keeps track which bodies are in the simulation
    body_manager: BodyManager = .empty,

    /// Body locking interfaces
    body_lock_interface_no_lock: BodyLockInterface = undefined,
    body_lock_interface_locking: BodyLockInterface = undefined,

    /// Body interfaces
    body_interface_no_lock: BodyInterface = .empty,
    body_interface_locking: BodyInterface = .empty,

    /// Narrow phase query interface
    narrow_phase_query_no_lock: NarrowPhaseQuery = .empty,
    narrow_phase_query_locking: NarrowPhaseQuery = .empty,

    /// The broadphase does quick collision detection between body pairs
    broad_phase: ?*BroadPhase = null,

    /// The soft body contact listener
    soft_body_contact_listener: ?SoftBodyContactListener = null,

    /// The shape filter that is used to filter out sub shapes during simulation
    sim_shape_filter: ?*const SimShapeFilter = null,

    /// The collision function that is used to collide two shapes during simulation
    sim_collide_body_vs_body: SimCollideBodyVsBody = .default,

    /// Simulation settings
    physics_settings: PhysicsSettings = .{},

    /// The contact manager resolves all contacts during a simulation step
    contact_manager: ContactConstraintManager = .empty,

    /// All non-contact constraints
    constraint_manager: ConstraintManager = .empty,

    /// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
    island_builder: IslandBuilder = .empty,

    /// Will split large islands into smaller groups of bodies that can be processed in parallel
    large_island_splitter: LargeIslandSplitter = .empty,

    /// Mutex protecting mStepListeners
    step_listeners_mutex: Mutex = .{},

    /// List of physics step listeners
    step_listeners: std.ArrayList(PhysicsStepListener) = .empty,

    /// This is the global gravity vector
    gravity: Vec3 = Vec3.init(0, -9.81, 0),

    /// Previous frame's delta time of one sub step to allow scaling previous frame's constraint impulses
    previous_step_delta_time: f32 = 0.0,

    /// Not initialized (Jolt's constructor), call init
    pub const empty: PhysicsSystem = .{};

    /// Initialize the system.
    /// @param max_bodies Maximum number of bodies to support.
    /// @param num_body_mutexes Number of body mutexes to use. Should be a power of 2 in the range [1, 64], use 0 to auto detect.
    /// @param max_body_pairs Maximum amount of body pairs to process (anything else will fall through the world), this number should generally be much higher than the max amount of contact points as there will be lots of bodies close that are not actually touching.
    /// @param max_contact_constraints Maximum amount of contact constraints to process (anything else will fall through the world).
    /// @param broad_phase_layer_interface Information on the mapping of object layers to broad phase layers. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
    /// @param object_vs_broad_phase_layer_filter Filter callback function that is used to determine if an object layer collides with a broad phase layer. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
    /// @param object_layer_pair_filter Filter callback function that is used to determine if two object layers collide. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
    /// (Zolt: in place, the system must not move afterwards; on error nothing is left allocated and the system is `empty`)
    pub fn init(self: *Self, allocator: Allocator, io: std.Io, max_bodies_in: u32, num_body_mutexes: u32, max_body_pairs: u32, max_contact_constraints: u32, broad_phase_layer_interface: *const BroadPhaseLayerInterface, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, opts: InitOptions) Allocator.Error!void {
        std.debug.assert(self.broad_phase == null); // Init twice
        self.* = .{ .allocator = allocator, .io = io };
        errdefer self.* = .empty;

        // Jolt's constructor: mConstraintManager(&mBodyManager) (Zolt: allocates its mutable mutex)
        try self.constraint_manager.init(allocator, io, &self.body_manager);
        errdefer self.constraint_manager.deinit();

        // Clamp max bodies
        const max_bodies = @min(max_bodies_in, max_bodies_limit);
        std.debug.assert(max_bodies == max_bodies_in); // Cannot support this many bodies!

        self.object_vs_broad_phase_layer_filter = object_vs_broad_phase_layer_filter;
        self.object_layer_pair_filter = object_layer_pair_filter;

        // Initialize body manager
        try self.body_manager.init(allocator, io, max_bodies, num_body_mutexes, broad_phase_layer_interface);
        errdefer self.body_manager.deinit();
        self.body_lock_interface_no_lock = .initNoLock(&self.body_manager);
        self.body_lock_interface_locking = .initLocking(&self.body_manager);

        // Create broadphase
        const broad_phase: *BroadPhase = switch (opts.broad_phase) {
            .brute_force => (try BroadPhaseBruteForce.create(allocator)).asBroadPhase(),
            .custom => |custom| custom,
        };
        errdefer if (opts.broad_phase == .brute_force) broad_phase.base.destroy();
        try broad_phase.initBroadPhase(&self.body_manager, broad_phase_layer_interface);

        // Init contact constraint manager
        try self.contact_manager.init(allocator, &self.physics_settings, max_body_pairs, max_contact_constraints);
        errdefer self.contact_manager.deinit();

        // Init islands builder
        try self.island_builder.init(allocator, max_bodies);

        self.broad_phase = broad_phase;

        // Initialize body interface
        self.body_interface_locking = .init(&self.body_lock_interface_locking, &self.body_manager, broad_phase);
        self.body_interface_no_lock = .init(&self.body_lock_interface_no_lock, &self.body_manager, broad_phase);

        // Initialize narrow phase query
        self.narrow_phase_query_locking = .init(&self.body_lock_interface_locking, broad_phase.asQuery());
        self.narrow_phase_query_no_lock = .init(&self.body_lock_interface_no_lock, broad_phase.asQuery());
    }

    /// Destructor
    pub fn deinit(self: *Self) void {
        const broad_phase = self.broad_phase orelse return; // Not initialized

        // Remove broadphase
        broad_phase.base.destroy();

        // The members, in reverse order of declaration
        self.step_listeners.deinit(self.allocator);
        self.large_island_splitter.deinit();
        self.island_builder.deinit();
        self.constraint_manager.deinit();
        self.contact_manager.deinit();
        self.body_manager.deinit();
        self.* = .empty;
    }

    fn getBroadPhase(self: *const Self) *BroadPhase {
        return self.broad_phase.?;
    }

    /// Listener that is notified whenever a body is activated/deactivated
    pub fn setBodyActivationListener(self: *Self, listener: ?BodyActivationListener) void {
        self.body_manager.setBodyActivationListener(listener);
    }

    pub fn getBodyActivationListener(self: *const Self) ?BodyActivationListener {
        return self.body_manager.getBodyActivationListener();
    }

    /// Listener that is notified whenever a contact point between two bodies is added/updated/removed.
    /// You can't change contact listener during PhysicsSystem::Update but it can be changed at any other time.
    pub fn setContactListener(self: *Self, listener: ?ContactListener) void {
        self.contact_manager.setContactListener(listener);
    }

    pub fn getContactListener(self: *const Self) ?ContactListener {
        return self.contact_manager.getContactListener();
    }

    /// Listener that is notified whenever a contact point between a soft body and another body
    pub fn setSoftBodyContactListener(self: *Self, listener: ?SoftBodyContactListener) void {
        self.soft_body_contact_listener = listener;
    }

    pub fn getSoftBodyContactListener(self: *const Self) ?SoftBodyContactListener {
        return self.soft_body_contact_listener;
    }

    /// Set the function that combines the friction of two bodies and returns it
    /// Default method is the geometric mean: sqrt(friction1 * friction2).
    pub fn setCombineFriction(self: *Self, combine_friction: ContactConstraintManager.CombineFunction) void {
        self.contact_manager.setCombineFriction(combine_friction);
    }

    pub fn getCombineFriction(self: *const Self) ContactConstraintManager.CombineFunction {
        return self.contact_manager.getCombineFriction();
    }

    /// Set the function that combines the restitution of two bodies and returns it
    /// Default method is max(restitution1, restitution1)
    pub fn setCombineRestitution(self: *Self, combine_restitution: ContactConstraintManager.CombineFunction) void {
        self.contact_manager.setCombineRestitution(combine_restitution);
    }

    pub fn getCombineRestitution(self: *const Self) ContactConstraintManager.CombineFunction {
        return self.contact_manager.getCombineRestitution();
    }

    /// Set/get the shape filter that will be used during simulation. This can be used to exclude shapes within a body from colliding with each other.
    /// E.g. if you have a high detail and a low detail collision model, you can attach them to the same body in a StaticCompoundShape and use the ShapeFilter
    /// to exclude the high detail collision model when simulating and exclude the low detail collision model when casting rays. Note that in this case
    /// you would need to pass the inverse of inShapeFilter to the CastRay function. Pass a nullptr to disable the shape filter.
    /// The PhysicsSystem does not own the ShapeFilter, make sure it stays alive during the lifetime of the PhysicsSystem.
    pub fn setSimShapeFilter(self: *Self, shape_filter: ?*const SimShapeFilter) void {
        self.sim_shape_filter = shape_filter;
    }

    pub fn getSimShapeFilter(self: *const Self) ?*const SimShapeFilter {
        return self.sim_shape_filter;
    }

    /// Advanced use only: Set the function that will be used to collide two bodies during simulation.
    /// This function is expected to eventually call CollideShapeCollector::AddHit all contact points between the shapes of body 1 and 2 in their given transforms.
    pub fn setSimCollideBodyVsBody(self: *Self, body_vs_body: SimCollideBodyVsBody) void {
        self.sim_collide_body_vs_body = body_vs_body;
    }

    pub fn getSimCollideBodyVsBody(self: *const Self) *const SimCollideBodyVsBody {
        return &self.sim_collide_body_vs_body;
    }

    /// Advanced use only: Default function that is used to collide two bodies during simulation.
    /// (Zolt: `allocator` is the heap of the InternalEdgeRemovingCollector, Allocator.Error when it could not store a hit)
    pub fn defaultSimCollideBodyVsBody(allocator: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
        const part1: SubShapeIDCreator = .{};
        const part2: SubShapeIDCreator = .{};

        if (body1.getEnhancedInternalEdgeRemovalWithBody(body2)) {
            // Collide with enhanced internal edge removal
            collide_shape_settings.active_edge_mode = .collide_with_all;
            try InternalEdgeRemovingCollector.collideShapeVsShape(allocator, body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, part1, part2, collide_shape_settings, collector, shape_filter);
        } else {
            // Regular collide
            CollisionDispatch.collideShapeVsShape(body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, part1, part2, collide_shape_settings, collector, shape_filter);
        }
    }

    /// Control the main constants of the physics simulation
    pub fn setPhysicsSettings(self: *Self, settings: *const PhysicsSettings) void {
        self.physics_settings = settings.*;
    }

    pub fn getPhysicsSettings(self: *const Self) *const PhysicsSettings {
        return &self.physics_settings;
    }

    /// Access to the body interface. This interface allows to to create / remove bodies and to change their properties.
    pub fn getBodyInterface(self: *const Self) *const BodyInterface {
        return &self.body_interface_locking;
    }

    pub fn getBodyInterfaceMut(self: *Self) *BodyInterface {
        return &self.body_interface_locking;
    }

    /// Version that does not lock the bodies, use with great care!
    pub fn getBodyInterfaceNoLock(self: *const Self) *const BodyInterface {
        return &self.body_interface_no_lock;
    }

    /// Version that does not lock the bodies, use with great care!
    pub fn getBodyInterfaceNoLockMut(self: *Self) *BodyInterface {
        return &self.body_interface_no_lock;
    }

    /// Access to the broadphase interface that allows coarse collision queries
    pub fn getBroadPhaseQuery(self: *const Self) *const BroadPhaseQuery {
        return self.getBroadPhase().asQuery();
    }

    /// Interface that allows fine collision queries against first the broad phase and then the narrow phase.
    pub fn getNarrowPhaseQuery(self: *const Self) *const NarrowPhaseQuery {
        return &self.narrow_phase_query_locking;
    }

    /// Version that does not lock the bodies, use with great care!
    pub fn getNarrowPhaseQueryNoLock(self: *const Self) *const NarrowPhaseQuery {
        return &self.narrow_phase_query_no_lock;
    }

    /// Add constraint to the world
    pub fn addConstraint(self: *Self, constraint: *Constraint) Allocator.Error!void {
        try self.constraint_manager.add((&constraint)[0..1]);
    }

    /// Remove constraint from the world
    pub fn removeConstraint(self: *Self, constraint: *Constraint) void {
        self.constraint_manager.remove((&constraint)[0..1]);
    }

    /// Batch add constraints.
    pub fn addConstraints(self: *Self, constraints: []const *Constraint) Allocator.Error!void {
        try self.constraint_manager.add(constraints);
    }

    /// Batch remove constraints.
    pub fn removeConstraints(self: *Self, constraints: []const *Constraint) void {
        self.constraint_manager.remove(constraints);
    }

    /// Get a list of all constraints (a copy allocated with `allocator` that holds a reference to every constraint, see
    /// ConstraintManager.getConstraints)
    pub fn getConstraints(self: *const Self, allocator: Allocator) Allocator.Error!Constraints {
        return self.constraint_manager.getConstraints(allocator);
    }

    /// Optimize the broadphase, needed only if you've added many bodies prior to calling Update() for the first time.
    /// Don't call this every frame as PhysicsSystem::Update spreads out the same work over multiple frames.
    /// If you add many bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize and if the bodies in a batch are
    /// in a roughly unoccupied space (e.g. a new level section) then a call to OptimizeBroadPhase is also not needed
    /// as batch adding creates an efficient bounding volume hierarchy.
    /// Don't call this function while bodies are being modified from another thread or use the locking BodyInterface to modify bodies.
    pub fn optimizeBroadPhase(self: *Self) void {
        self.getBroadPhase().optimize();
    }

    /// Adds a new step listener
    pub fn addStepListener(self: *Self, listener: PhysicsStepListener) Allocator.Error!void {
        self.step_listeners_mutex.lock(self.io);
        defer self.step_listeners_mutex.unlock(self.io);

        for (self.step_listeners.items) |l|
            std.debug.assert(l.ptr != listener.ptr);
        try self.step_listeners.append(self.allocator, listener);
    }

    /// Removes a step listener
    pub fn removeStepListener(self: *Self, listener: PhysicsStepListener) void {
        self.step_listeners_mutex.lock(self.io);
        defer self.step_listeners_mutex.unlock(self.io);

        for (self.step_listeners.items, 0..) |l, i| {
            if (l.ptr == listener.ptr) {
                self.step_listeners.items[i] = self.step_listeners.items[self.step_listeners.items.len - 1];
                self.step_listeners.items.len -= 1;
                return;
            }
        }
        if (Core.enable_asserts) @panic("PhysicsSystem.removeStepListener: listener not found");
    }

    /// Set gravity value
    pub fn setGravity(self: *Self, gravity: Vec3) void {
        self.gravity = gravity;
    }

    pub fn getGravity(self: *const Self) Vec3 {
        return self.gravity;
    }

    /// Returns a locking interface that won't actually lock the body. Use with great care!
    pub fn getBodyLockInterfaceNoLock(self: *const Self) *const BodyLockInterface {
        return &self.body_lock_interface_no_lock;
    }

    /// Returns a locking interface that locks the body so other threads cannot modify it.
    pub fn getBodyLockInterface(self: *const Self) *const BodyLockInterface {
        return &self.body_lock_interface_locking;
    }

    /// Return the BroadPhaseLayerInterface that this class has been initialized with
    pub fn getBroadPhaseLayerInterface(self: *const Self) *const BroadPhaseLayerInterface {
        return self.body_manager.getBroadPhaseLayerInterface();
    }

    /// Broadphase layer filter that decides if two objects can collide, this was passed to the Init function.
    pub fn getObjectVsBroadPhaseLayerFilter(self: *const Self) *const ObjectVsBroadPhaseLayerFilter {
        return self.object_vs_broad_phase_layer_filter;
    }

    /// Object layer filter that decides if two objects can collide, this was passed to the Init function.
    pub fn getObjectLayerPairFilter(self: *const Self) *const ObjectLayerPairFilter {
        return self.object_layer_pair_filter;
    }

    /// Get an broadphase layer filter that uses the default pair filter and a specified object layer to determine if broadphase layers collide
    pub fn getDefaultBroadPhaseLayerFilter(self: *const Self, layer: ObjectLayer) DefaultBroadPhaseLayerFilter {
        return .init(self.object_vs_broad_phase_layer_filter, layer);
    }

    /// Get an object layer filter that uses the default pair filter and a specified layer to determine if layers collide
    pub fn getDefaultLayerFilter(self: *const Self, layer: ObjectLayer) DefaultObjectLayerFilter {
        return .init(self.object_layer_pair_filter, layer);
    }

    /// Gets the current amount of bodies that are in the body manager
    pub fn getNumBodies(self: *const Self) u32 {
        return self.body_manager.getNumBodies();
    }

    /// Gets the current amount of active bodies that are in the body manager
    pub fn getNumActiveBodies(self: *const Self, body_type: BodyType) u32 {
        return self.body_manager.getNumActiveBodies(body_type);
    }

    /// Get the maximum amount of bodies that this physics system supports
    pub fn getMaxBodies(self: *const Self) u32 {
        return self.body_manager.getMaxBodies();
    }

    /// Get stats about the bodies in the body manager (slow, iterates through all bodies)
    pub fn getBodyStats(self: *const Self) BodyStats {
        return self.body_manager.getBodyStats();
    }

    /// Get copy of the list of all bodies under protection of a lock.
    /// @param out_body_ids On return, this will contain the list of BodyIDs (allocated with `allocator`)
    pub fn getBodies(self: *const Self, allocator: Allocator, out_body_ids: *BodyIDVector) Allocator.Error!void {
        return self.body_manager.getBodyIDs(allocator, out_body_ids);
    }

    /// Get copy of the list of active bodies under protection of a lock.
    /// @param body_type The type of bodies to get
    /// @param out_body_ids On return, this will contain the list of BodyIDs (allocated with `allocator`)
    pub fn getActiveBodies(self: *const Self, allocator: Allocator, body_type: BodyType, out_body_ids: *BodyIDVector) Allocator.Error!void {
        return self.body_manager.getActiveBodies(allocator, body_type, out_body_ids);
    }

    /// Get the list of active bodies, use GetNumActiveBodies() to find out how long the list is.
    /// Note: Not thread safe. The active bodies list can change at any moment when other threads are doing work. Use GetActiveBodies() if you need a thread safe version.
    pub fn getActiveBodiesUnsafe(self: *const Self, body_type: BodyType) [*]const BodyID {
        return self.body_manager.getActiveBodiesUnsafe(body_type);
    }

    /// Check if 2 bodies were in contact during the last simulation step. Since contacts are only detected between active bodies, so at least one of the bodies must be active in order for this function to work.
    /// It queries the state at the time of the last PhysicsSystem::Update and will return true if the bodies were in contact, even if one of the bodies was moved / removed afterwards.
    /// This function can be called from any thread when the PhysicsSystem::Update is not running. During PhysicsSystem::Update this function is only valid during contact callbacks:
    /// - During the ContactListener::OnContactAdded callback this function can be used to determine if a different contact pair between the bodies was active in the previous simulation step (function returns true) or if this is the first step that the bodies are touching (function returns false).
    /// - During the ContactListener::OnContactRemoved callback this function can be used to determine if this is the last contact pair between the bodies (function returns false) or if there are other contacts still present (function returns true).
    pub fn wereBodiesInContact(self: *const Self, body1_id: BodyID, body2_id: BodyID) bool {
        return self.contact_manager.wereBodiesInContact(body1_id, body2_id);
    }

    /// Get the bounding box of all bodies in the physics system.
    /// Deprecated: Use GetBroadPhaseQuery().GetBounds() instead.
    pub fn getBounds(self: *const Self) AABox {
        return self.getBroadPhaseQuery().getBounds();
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Update

    /// Gather stats from the islands and distribute them over the bodies (JPH_TRACK_SIMULATION_STATS)
    fn gatherIslandStats(self: *Self) void {
        var island_idx: u32 = 0;
        while (island_idx < self.island_builder.getNumIslands()) : (island_idx += 1) {
            const bodies = self.island_builder.getBodiesInIsland(island_idx);
            const num_bodies: u64 = bodies.len;

            // Calculate the number of dynamic bodies
            var num_dynamic_bodies: u64 = 0;
            for (bodies) |body_id| {
                if (self.body_manager.getBody(body_id).getMotionType() == .dynamic)
                    num_dynamic_bodies += 1;
            }
            num_dynamic_bodies = @max(num_dynamic_bodies, 1); // Ensure we don't divide by zero

            // Equally distribute the stats over all bodies
            const stats = self.island_builder.getIslandStats(island_idx);
            const num_velocity_ticks = stats.velocity_constraint_ticks.load(.seq_cst) / num_dynamic_bodies;
            const num_position_ticks = stats.position_constraint_ticks.load(.seq_cst) / num_dynamic_bodies;
            const num_update_bounds_ticks = stats.update_bounds_ticks.load(.seq_cst) / num_bodies;

            for (bodies) |body_id| {
                const body = self.body_manager.getBodyMut(body_id);
                const out_stats = body.getMotionPropertiesMut().getSimulationStatsMut();
                out_stats.num_collision_steps +%= 1;
                out_stats.num_velocity_steps = stats.num_velocity_steps;
                out_stats.num_position_steps = stats.num_position_steps;
                if (body.getMotionType() == .dynamic) {
                    out_stats.velocity_constraint_ticks += num_velocity_ticks; // In case of multiple collision steps we accumulate
                    out_stats.position_constraint_ticks += num_position_ticks;
                }
                out_stats.update_bounds_ticks += num_update_bounds_ticks;
                out_stats.is_large_island = stats.is_large_island;
            }
        }
    }

    /// Simulate the system.
    /// The world steps for a total of inDeltaTime seconds. This is divided in inCollisionSteps iterations.
    /// Each iteration consists of collision detection followed by an integration step.
    /// This function internally spawns jobs using inJobSystem and waits for them to complete, so no jobs will be running when this function returns.
    /// The temp allocator is used, for example, to store the list of bodies that are in contact, how they form islands together
    /// and data to solve the contacts between bodies. At the end of the Update call, all allocated memory will have been freed.
    /// Returns Jolt's soft errors (the simulation continued), or error.OutOfMemory when the temp allocator or the
    /// allocator ran out (D5): a failure while building the jobs changes nothing; a failure inside a job leaves the step
    /// half done (memory safe, no lock held, nothing leaked, the temp allocator is empty): restore a saved state or
    /// deinit the system.
    pub fn update(self: *Self, delta_time: f32, collision_steps: u32, temp_allocator: TempAllocator, job_system: JobSystem) error{OutOfMemory}!PhysicsUpdateError {
        // TODO(determinism_log): JPH_DET_LOG("PhysicsSystem::Update: dt: " << inDeltaTime << " steps: " << inCollisionSteps);

        std.debug.assert(collision_steps > 0);
        std.debug.assert(delta_time >= 0.0);

        const broad_phase = self.getBroadPhase();

        // Sync point for the broadphase. This will allow it to do clean up operations without having any mutexes locked yet.
        broad_phase.frameSync();

        // If there are no active bodies (and no step listener to wake them up) or there's no time delta
        const num_active_rigid_bodies = self.body_manager.getNumActiveBodies(.rigid_body);
        const num_active_soft_bodies = self.body_manager.getNumActiveBodies(.soft_body);
        if ((num_active_rigid_bodies == 0 and num_active_soft_bodies == 0 and self.step_listeners.items.len == 0) or delta_time <= 0.0) {
            self.body_manager.lockAllBodies();

            // Update broadphase
            broad_phase.lockModifications();
            const update_state = broad_phase.updatePrepare() catch |err| {
                // Zolt (D5): the broad phase is unchanged
                broad_phase.unlockModifications();
                self.body_manager.unlockAllBodies();
                return err;
            };
            broad_phase.updateFinalize(&update_state);
            broad_phase.unlockModifications();

            // If time has passed, call contact removal callbacks from contacts that existed in the previous update
            if (delta_time > 0.0)
                self.contact_manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(0, 0);

            self.body_manager.unlockAllBodies();
            return .none;
        }

        // Reset accumulated stats on the active bodies
        if (track_simulation_stats)
            self.body_manager.resetSimulationStats();

        // Calculate ratio between current and previous frame delta time to scale initial constraint forces.
        // Note that scaling of the impulses works perfectly for a system in equilibrium. When a body is falling on another body, it will receive
        // a large impulse to stop the body. In the next step, a much smaller impulse needs to be applied to counter gravity.
        // This means that warm starting will initially give the body on top a much higher impulse that it needs.
        // If you scale that impulse up because the delta time ratio >> 1 then the body will get such a large velocity that the solver
        // cannot correct it anymore. We therefore clamp the ratio to ensure that the solver can still correct the excess velocity.
        // Note that this will make a stack in equilibrium less stable when there are large delta time differences between steps.
        // See: https://github.com/jrouwe/JoltPhysics/discussions/2129#discussioncomment-18525044
        const step_delta_time = delta_time / @as(f32, @floatFromInt(collision_steps));
        const warm_start_impulse_ratio: f32 = if (self.previous_step_delta_time > 0.0) math.min(step_delta_time / self.previous_step_delta_time, @as(f32, 4.0)) else 0.0;
        const old_previous_step_delta_time = self.previous_step_delta_time; // Zolt: restored when building the jobs fails
        self.previous_step_delta_time = step_delta_time;

        // Create the context used for passing information between jobs
        var context: PhysicsUpdateContext = .{
            .physics_system = self,
            .temp_allocator = temp_allocator,
            .job_system = job_system,
            .barrier = job_system.createBarrier() orelse @panic("PhysicsSystem.update: the job system has no free barrier"), // Jolt crashes
            .body_manager = &self.body_manager,
            .island_builder = &self.island_builder,
            .step_delta_time = step_delta_time,
            .warm_start_impulse_ratio = warm_start_impulse_ratio,
        };

        // context.mSteps.resize(inCollisionSteps) (Zolt: the steps and the body pairs are allocated before the locks, like
        // in Jolt; a failure frees what was allocated and changes nothing)
        context.steps = PhysicsUpdateContext.Steps.alloc(temp_allocator, collision_steps) catch |err| {
            job_system.destroyBarrier(context.barrier);
            self.previous_step_delta_time = old_previous_step_delta_time;
            return err;
        };
        for (context.steps.items) |*s|
            s.* = .{};

        // Allocate space for body pairs
        std.debug.assert(context.body_pairs.isEmpty());
        context.body_pairs = TempArray(PhysicsUpdateContext.BodyPairSlot).alloc(temp_allocator, @intCast(self.physics_settings.max_in_flight_body_pairs)) catch |err| {
            job_system.destroyBarrier(context.barrier);
            context.deinit();
            self.previous_step_delta_time = old_previous_step_delta_time;
            return err;
        };

        // Lock all bodies for write so that we can freely touch them
        self.step_listeners_mutex.lock(self.io);
        self.body_manager.lockAllBodies();
        broad_phase.lockModifications();

        // Build the jobs. Zolt: no job runs before every job exists (the start gate), so a failure has no side effects.
        var constraints_locked = false;
        self.buildJobs(&context, collision_steps, &constraints_locked) catch |err| {
            self.finishUpdate(&context, constraints_locked);
            self.previous_step_delta_time = old_previous_step_delta_time;
            return err;
        };

        // Zolt start gate (D4): Jolt starts the first step's root jobs while building the jobs, Zolt kicks them now,
        // exactly like StartNextStep kicks the following steps (their root jobs have 1 dependency in both)
        kickStep(&context.steps.items[0]);

        // Build the list of jobs to wait for
        const barrier = context.barrier;
        {
            var handles: StaticArray(JobHandle, @intCast(physics_settings_file.max_physics_jobs)) = .empty;
            defer for (handles.slice()) |*h| h.deinit();
            for (context.steps.items) |*step| {
                if (step.broad_phase_prepare.isValid())
                    handles.append(step.broad_phase_prepare.clone());
                for (step.step_listeners.constSlice()) |*h|
                    handles.append(h.clone());
                for (step.determine_active_constraints.constSlice()) |*h|
                    handles.append(h.clone());
                for (step.apply_gravity.constSlice()) |*h|
                    handles.append(h.clone());
                for (step.find_collisions.constSlice()) |*h|
                    handles.append(h.clone());
                if (step.update_broadphase_finalize.isValid())
                    handles.append(step.update_broadphase_finalize.clone());
                for (step.setup_velocity_constraints.constSlice()) |*h|
                    handles.append(h.clone());
                handles.append(step.build_islands_from_constraints.clone());
                handles.append(step.finalize_islands.clone());
                handles.append(step.body_set_island_index.clone());
                for (step.solve_velocity_constraints.constSlice()) |*h|
                    handles.append(h.clone());
                handles.append(step.pre_integrate_velocity.clone());
                for (step.integrate_velocity.constSlice()) |*h|
                    handles.append(h.clone());
                handles.append(step.post_integrate_velocity.clone());
                handles.append(step.resolve_ccd_contacts.clone());
                for (step.solve_position_constraints.constSlice()) |*h|
                    handles.append(h.clone());
                handles.append(step.contact_removed_callbacks.clone());
                if (step.soft_body_prepare.isValid())
                    handles.append(step.soft_body_prepare.clone());
                if (step.start_next_step.isValid())
                    handles.append(step.start_next_step.clone());
            }
            barrier.addJobs(handles.constSlice());
        }

        // Wait until all jobs finish
        // Note we don't just wait for the last job. If we would and another job
        // would be scheduled in between there is the possibility of a deadlock.
        // The other job could try to e.g. add/remove a body which would try to
        // lock a body mutex while this thread has already locked the mutex
        job_system.waitForJobs(barrier);

        self.finishUpdate(&context, constraints_locked);

        // Zolt (D5): an allocation failed inside a job
        if (context.isOutOfMemory())
            return error.OutOfMemory;

        // Return any errors
        const errors = PhysicsUpdateError.fromInt(context.errors.load(.acquire));
        if (Core.enable_asserts) std.debug.assert(errors.isNone()); // An error occurred during the physics update, see EPhysicsUpdateError for more information
        return errors;
    }

    /// Kick the root jobs of a step: the kick of StartNextStep, also used for the first step by the Zolt start gate (D4)
    fn kickStep(step: *Step) void {
        // Kick the jobs of the next step (in the same order as the first step)
        step.broad_phase_prepare.removeDependency(.{});
        if (step.step_listeners.isEmpty()) {
            // Kick the gravity and active constraints jobs immediately
            JobHandle.removeDependenciesStaticArray(&step.apply_gravity, .{});
            JobHandle.removeDependenciesStaticArray(&step.determine_active_constraints, .{});
        } else {
            // Kick the step listeners job first
            JobHandle.removeDependenciesStaticArray(&step.step_listeners, .{});
        }
    }

    /// The tail of Update, also the cleanup of a failed build (D5): Jolt's frees in Jolt's order (TempArrays that were
    /// never allocated are skipped), the unlocks and the destructor of the context.
    fn finishUpdate(self: *Self, context: *PhysicsUpdateContext, constraints_locked: bool) void {
        const temp_allocator = context.temp_allocator;

        // We're done with the barrier for this update
        context.job_system.destroyBarrier(context.barrier);

        // Validate that the cached bounds are correct (JPH_DEBUG; Zolt: not after an out of memory error, the step
        // stopped halfway)
        if (builtin.mode == .Debug and !context.isOutOfMemory())
            self.body_manager.validateActiveBodyBounds();

        // Gather stats from the islands and distribute them over the bodies
        if (track_simulation_stats and !context.isOutOfMemory())
            self.gatherIslandStats();

        // Clear the large island splitter
        self.large_island_splitter.reset(temp_allocator);

        // Clear the island builder
        self.island_builder.resetIslands(temp_allocator);

        // Clear the contact manager
        self.contact_manager.finishConstraintBuffer();

        // Free active constraints
        context.active_constraints.free(temp_allocator);

        // Free body pairs
        context.body_pairs.free(temp_allocator);

        // Unlock the broadphase
        self.getBroadPhase().unlockModifications();

        // Unlock all constraints
        if (constraints_locked)
            self.constraint_manager.unlockAllConstraints();

        // Allow write operations to the active bodies list
        self.body_manager.setActiveBodiesLocked(false);

        // Unlock all bodies
        self.body_manager.unlockAllBodies();

        // Unlock step listeners
        self.step_listeners_mutex.unlock(self.io);

        // ~PhysicsUpdateContext: releases the job handles and frees the steps (the first temp allocation)
        context.deinit();
    }

    /// The "Build Jobs" block of PhysicsSystem::Update. Every C++ lambda is an inline closure whose captures are the
    /// lambda's: `[&context, &step]` -> `.{ context, step }`, `[&step, i]` -> `.{ step, i }`, `[this, next_step]` ->
    /// `.{ self, next_step }`.
    fn buildJobs(self: *Self, context: *PhysicsUpdateContext, collision_steps: u32, constraints_locked: *bool) Allocator.Error!void {
        const temp_allocator = context.temp_allocator;
        const job_system = context.job_system;
        const steps = context.steps.items;
        const num_active_rigid_bodies = self.body_manager.getNumActiveBodies(.rigid_body);

        // Get max number of concurrent jobs
        const max_concurrency: u32 = context.getMaxConcurrency();
        const max_concurrency_i: i32 = @intCast(max_concurrency);

        // Calculate how many step listener jobs we spawn
        const num_step_listeners: i32 = @intCast(self.step_listeners.items.len);
        const num_step_listener_jobs: u32 = if (self.step_listeners.items.len == 0) 0 else @intCast(@max(1, @min(@divTrunc(@divTrunc(num_step_listeners, self.physics_settings.step_listeners_batch_size), self.physics_settings.step_listener_batches_per_job), max_concurrency_i)));

        // Number of gravity jobs depends on the amount of active bodies.
        // Launch max 1 job per batch of active bodies
        // Leave 1 thread for update broadphase prepare and 1 for determine active constraints
        const num_apply_gravity_jobs: u32 = @intCast(@max(1, @min(@divTrunc(@as(i32, @intCast(num_active_rigid_bodies)) + apply_gravity_batch_size - 1, apply_gravity_batch_size), max_concurrency_i - 2)));

        // Number of determine active constraints jobs to run depends on number of constraints.
        // Leave 1 thread for update broadphase prepare and 1 for apply gravity
        const num_constraints: i32 = @intCast(self.constraint_manager.getNumConstraints());
        const num_determine_active_constraints_jobs: u32 = @intCast(@max(1, @min(@divTrunc(num_constraints + determine_active_constraints_batch_size - 1, determine_active_constraints_batch_size), max_concurrency_i - 2)));

        // Number of setup velocity constraints jobs to run depends on number of constraints.
        const num_setup_velocity_constraints_jobs: u32 = @intCast(@max(1, @min(@divTrunc(num_constraints + setup_velocity_constraints_batch_size - 1, setup_velocity_constraints_batch_size), max_concurrency_i)));

        // Number of find collisions jobs to run depends on number of active bodies.
        // Note that when we have more than 1 thread, we always spawn at least 2 find collisions jobs so that the first job can wait for build islands from constraints
        // (which may activate additional bodies that need to be processed) while the second job can start processing collision work.
        const num_find_collisions_jobs: u32 = @intCast(@max(@as(i32, if (max_concurrency == 1) 1 else 2), @min(@divTrunc(@as(i32, @intCast(num_active_rigid_bodies)) + active_bodies_batch_size - 1, active_bodies_batch_size), max_concurrency_i)));

        // Number of integrate velocity jobs depends on number of active bodies.
        const num_integrate_velocity_jobs: u32 = @intCast(@max(1, @min(@divTrunc(@as(i32, @intCast(num_active_rigid_bodies)) + integrate_velocity_batch_size - 1, integrate_velocity_batch_size), max_concurrency_i)));

        // Iterate over collision steps
        for (steps, 0..) |*step, step_idx| {
            const is_first_step = step_idx == 0;
            const is_last_step = step_idx == collision_steps - 1;

            step.context = context;
            step.is_first = is_first_step;
            step.is_last = is_last_step;

            // Create job to do broadphase finalization
            // This job must finish before integrating velocities. Until then the positions will not be updated neither will bodies be added / removed.
            step.update_broadphase_finalize = try job_system.createJob("UpdateBroadPhaseFinalize", color_update_broad_phase_finalize, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Validate that all find collision jobs have stopped
                    std.debug.assert(s.active_find_collision_jobs.load(.monotonic) == 0);

                    // Finalize the broadphase update (Zolt D5: a commit, it also runs when poisoned; a state that was not
                    // prepared is zeroed and does nothing)
                    c.physics_system.getBroadPhase().updateFinalize(&s.broad_phase_update_state);

                    // Signal that it is done
                    s.pre_integrate_velocity.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_find_collisions_jobs + 2 }); // depends on: find collisions, broadphase prepare update, finish building jobs

            // The immediate jobs below are only immediate for the first step, the all finished job will kick them for the next step
            // (Zolt start gate: 1 for the first step too, `update` kicks the first step after building the jobs, D4)
            const previous_step_dependency_count: u32 = 1;

            // Start job immediately: Start the prepare broadphase
            // Must be done under body lock protection since the order is body locks then broadphase mutex
            // If this is turned around the RemoveBody call will hang since it locks in that order
            step.broad_phase_prepare = try job_system.createJob("UpdateBroadPhasePrepare", color_update_broad_phase_prepare, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Prepare the broadphase update (Zolt D5: skipped when poisoned, on error the state stays zeroed)
                    if (!c.isOutOfMemory())
                        s.broad_phase_update_state = c.physics_system.getBroadPhase().updatePrepare() catch blk: {
                            c.setOutOfMemory();
                            break :blk .{};
                        };

                    // Now the finalize can run (if other dependencies are met too)
                    s.update_broadphase_finalize.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = previous_step_dependency_count });

            // This job will find all collisions
            step.body_pair_queues.resize(max_concurrency);
            for (step.body_pair_queues.slice()) |*q|
                q.* = .{};
            step.max_body_pairs_per_queue = @as(u32, @intCast(self.physics_settings.max_in_flight_body_pairs)) / max_concurrency;
            step.active_find_collision_jobs.store(~@as(JobMask, 0) >> @intCast(@bitSizeOf(JobMask) - num_find_collisions_jobs), .release);
            PhysicsUpdateContext.resizeHandles(&step.find_collisions, num_find_collisions_jobs);
            for (step.find_collisions.slice(), 0..) |*h, i| {
                // Build islands from constraints may activate additional bodies, so the first job will wait for this to finish in order to not miss any active bodies
                const num_dep_build_islands_from_constraints: u32 = if (i == 0) 1 else 0;
                h.* = try job_system.createJob("FindCollisions", color_find_collisions, .init(struct {
                    fn call(s: *Step, job_index: u32) void {
                        // Zolt D5: a protocol job, its body checks the latch
                        s.context.physics_system.jobFindCollisions(s, job_index);
                    }
                }.call, .{ step, @as(u32, @intCast(i)) }), .{ .num_dependencies = num_apply_gravity_jobs + num_determine_active_constraints_jobs + 1 + num_dep_build_islands_from_constraints }); // depends on: apply gravity, determine active constraints, finish building jobs, build islands from constraints
            }

            if (is_first_step) {
                // Don't allow write operations to the active bodies list
                self.body_manager.setActiveBodiesLocked(true);

                // Store the number of active bodies at the start of the step
                step.num_active_bodies_at_step_start = self.body_manager.getNumActiveBodies(.rigid_body);

                // Lock all constraints
                self.constraint_manager.lockAllConstraints();
                constraints_locked.* = true;

                // Allocate memory for storing the active constraints
                std.debug.assert(context.active_constraints.isEmpty());
                context.active_constraints = try .alloc(temp_allocator, self.constraint_manager.getNumConstraints());

                // Prepare contact buffer
                try self.contact_manager.prepareConstraintBuffer(context);

                // Setup island builder
                try self.island_builder.prepareContactConstraints(self.contact_manager.getMaxConstraints(), context.temp_allocator);
            }

            // This job applies gravity to all active bodies
            PhysicsUpdateContext.resizeHandles(&step.apply_gravity, num_apply_gravity_jobs);
            for (step.apply_gravity.slice()) |*h|
                h.* = try job_system.createJob("ApplyGravity", color_apply_gravity, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobApplyGravity(c, s);

                        JobHandle.removeDependenciesStaticArray(&s.find_collisions, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = if (num_step_listener_jobs > 0) num_step_listener_jobs else previous_step_dependency_count }); // depends on: step listeners (or previous step if no step listeners)

            // This job will setup velocity constraints for non-collision constraints
            PhysicsUpdateContext.resizeHandles(&step.setup_velocity_constraints, num_setup_velocity_constraints_jobs);
            for (step.setup_velocity_constraints.slice()) |*h|
                h.* = try job_system.createJob("SetupVelocityConstraints", color_setup_velocity_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobSetupVelocityConstraints(c.step_delta_time, s);

                        JobHandle.removeDependenciesStaticArray(&s.solve_velocity_constraints, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = num_determine_active_constraints_jobs + 1 }); // depends on: determine active constraints, finish building jobs

            // This job will build islands from constraints
            step.build_islands_from_constraints = try job_system.createJob("BuildIslandsFromConstraints", color_build_islands_from_constraints, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobBuildIslandsFromConstraints(c, s) catch c.setOutOfMemory();

                    s.find_collisions.slice()[0].removeDependency(.{}); // The first collisions job cannot start running until we've finished building islands and activated all bodies
                    s.finalize_islands.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_determine_active_constraints_jobs + 1 }); // depends on: determine active constraints, finish building jobs

            // This job determines active constraints
            PhysicsUpdateContext.resizeHandles(&step.determine_active_constraints, num_determine_active_constraints_jobs);
            for (step.determine_active_constraints.slice()) |*h|
                h.* = try job_system.createJob("DetermineActiveConstraints", color_determine_active_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobDetermineActiveConstraints(s);

                        s.build_islands_from_constraints.removeDependency(.{});

                        // Kick these jobs last as they will use up all CPU cores leaving no space for the previous job, we prefer setup velocity constraints to finish first so we kick it first
                        JobHandle.removeDependenciesStaticArray(&s.setup_velocity_constraints, .{});
                        JobHandle.removeDependenciesStaticArray(&s.find_collisions, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = if (num_step_listener_jobs > 0) num_step_listener_jobs else previous_step_dependency_count }); // depends on: step listeners (or previous step if no step listeners)

            // This job calls the step listeners
            PhysicsUpdateContext.resizeHandles(&step.step_listeners, num_step_listener_jobs);
            for (step.step_listeners.slice()) |*h|
                h.* = try job_system.createJob("StepListeners", color_step_listeners, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        // Call the step listeners
                        if (!c.isOutOfMemory())
                            c.physics_system.jobStepListeners(s);

                        // Kick apply gravity and determine active constraint jobs
                        JobHandle.removeDependenciesStaticArray(&s.apply_gravity, .{});
                        JobHandle.removeDependenciesStaticArray(&s.determine_active_constraints, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = previous_step_dependency_count });

            // Unblock the previous step
            if (!is_first_step)
                steps[step_idx - 1].start_next_step.removeDependency(.{});

            // This job will finalize the simulation islands
            step.finalize_islands = try job_system.createJob("FinalizeIslands", color_finalize_islands, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Validate that all find collision jobs have stopped
                    std.debug.assert(s.active_find_collision_jobs.load(.monotonic) == 0);

                    if (!c.isOutOfMemory())
                        c.physics_system.jobFinalizeIslands(c) catch c.setOutOfMemory();

                    JobHandle.removeDependenciesStaticArray(&s.solve_velocity_constraints, .{});
                    s.body_set_island_index.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_find_collisions_jobs + 2 }); // depends on: find collisions, build islands from constraints, finish building jobs

            // Unblock previous job
            // Note: technically we could release find collisions here but we don't want to because that could make them run before 'setup velocity constraints' which means that job won't have a thread left
            step.build_islands_from_constraints.removeDependency(.{});

            // This job will call the contact removed callbacks
            step.contact_removed_callbacks = try job_system.createJob("ContactRemovedCallbacks", color_contact_removed_callbacks, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: a protocol job, its body checks the latch
                    c.physics_system.jobContactRemovedCallbacks(c, s);

                    if (s.start_next_step.isValid())
                        s.start_next_step.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 1 }); // depends on the find ccd contacts

            // This job will set the island index on each body (only used for debug drawing purposes)
            // It will also delete any bodies that have been destroyed in the last frame
            step.body_set_island_index = try job_system.createJob("BodySetIslandIndex", color_body_set_island_index, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobBodySetIslandIndex();

                    JobHandle.removeDependenciesStaticArray(&s.solve_position_constraints, .{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 2 }); // depends on: finalize islands, finish building jobs

            // Job to start the next collision step
            if (!is_last_step) {
                const next_step = &steps[step_idx + 1];
                step.start_next_step = try job_system.createJob("StartNextStep", color_start_next_step, .init(struct {
                    fn call(ps: *Self, ns: *Step) void {
                        const c = ns.context;

                        // Validate that the cached bounds are correct (JPH_DEBUG; Zolt: not after an out of memory error)
                        if (builtin.mode == .Debug and !c.isOutOfMemory())
                            ps.body_manager.validateActiveBodyBounds();

                        // Gather stats from the islands and distribute them over the bodies
                        if (track_simulation_stats and !c.isOutOfMemory())
                            ps.gatherIslandStats();

                        // Store the number of active bodies at the start of the step
                        ns.num_active_bodies_at_step_start = ps.body_manager.getNumActiveBodies(.rigid_body);

                        // Clear the large island splitter
                        const temp_allocator_ns = c.temp_allocator;
                        ps.large_island_splitter.reset(temp_allocator_ns);

                        // Clear the island builder
                        ps.island_builder.resetIslands(temp_allocator_ns);

                        // Setup island builder (Zolt D5: no allocation once poisoned)
                        if (!c.isOutOfMemory())
                            ps.island_builder.prepareContactConstraints(ps.contact_manager.getMaxConstraints(), temp_allocator_ns) catch c.setOutOfMemory();

                        // Restart the contact manager
                        ps.contact_manager.recycleConstraintBuffer();

                        // Kick the jobs of the next step (in the same order as the first step)
                        kickStep(ns);
                    }
                }.call, .{ self, next_step }), .{ .num_dependencies = 3 }); // depends on: update soft bodies, contact removed callbacks, finish building the previous step
            }

            // This job will solve the velocity constraints
            PhysicsUpdateContext.resizeHandles(&step.solve_velocity_constraints, max_concurrency);
            for (step.solve_velocity_constraints.slice()) |*h|
                h.* = try job_system.createJob("SolveVelocityConstraints", color_solve_velocity_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobSolveVelocityConstraints(c, s);

                        s.pre_integrate_velocity.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = num_setup_velocity_constraints_jobs + 2 }); // depends on: finalize islands, setup velocity constraints, finish building jobs.

            // We prefer setup velocity constraints to finish first so we kick it first
            JobHandle.removeDependenciesStaticArray(&step.setup_velocity_constraints, .{});
            JobHandle.removeDependenciesStaticArray(&step.find_collisions, .{});

            // Finalize islands is a dependency on find collisions so it can go last
            step.finalize_islands.removeDependency(.{});

            // This job will prepare the position update of all active bodies
            step.pre_integrate_velocity = try job_system.createJob("PreIntegrateVelocity", color_pre_integrate_velocity, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobPreIntegrateVelocity(c, s) catch c.setOutOfMemory();

                    JobHandle.removeDependenciesStaticArray(&s.integrate_velocity, .{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 2 + max_concurrency }); // depends on: broadphase update finalize, solve velocity constraints, finish building jobs.

            // Unblock previous jobs
            step.update_broadphase_finalize.removeDependency(.{});
            JobHandle.removeDependenciesStaticArray(&step.solve_velocity_constraints, .{});

            // This job will update the positions of all active bodies
            PhysicsUpdateContext.resizeHandles(&step.integrate_velocity, num_integrate_velocity_jobs);
            for (step.integrate_velocity.slice()) |*h|
                h.* = try job_system.createJob("IntegrateVelocity", color_integrate_velocity, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobIntegrateVelocity(c, s);

                        s.post_integrate_velocity.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = 2 }); // depends on: pre integrate velocity, finish building jobs.

            // Unblock previous job
            step.pre_integrate_velocity.removeDependency(.{});

            // This job will finish the position update of all active bodies
            step.post_integrate_velocity = try job_system.createJob("PostIntegrateVelocity", color_post_integrate_velocity, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: a protocol job, its body checks the latch
                    c.physics_system.jobPostIntegrateVelocity(c, s);

                    s.resolve_ccd_contacts.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_integrate_velocity_jobs + 1 }); // depends on: integrate velocity, finish building jobs

            // Unblock previous jobs
            JobHandle.removeDependenciesStaticArray(&step.integrate_velocity, .{});

            // This job will update the positions and velocities for all bodies that need continuous collision detection
            step.resolve_ccd_contacts = try job_system.createJob("ResolveCCDContacts", color_resolve_ccd_contacts, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: a protocol job, its body checks the latch (and always frees)
                    c.physics_system.jobResolveCCDContacts(c, s);

                    JobHandle.removeDependenciesStaticArray(&s.solve_position_constraints, .{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 2 }); // depends on: integrate velocities, detect ccd contacts (added dynamically), finish building jobs.

            // Unblock previous job
            step.post_integrate_velocity.removeDependency(.{});

            // Fixes up drift in positions and updates the broadphase with new body positions
            PhysicsUpdateContext.resizeHandles(&step.solve_position_constraints, max_concurrency);
            for (step.solve_position_constraints.slice()) |*h|
                h.* = try job_system.createJob("SolvePositionConstraints", color_solve_position_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobSolvePositionConstraints(c, s);

                        // Kick the next step
                        if (s.soft_body_prepare.isValid())
                            s.soft_body_prepare.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = 3 }); // depends on: resolve ccd contacts, body set island index, finish building jobs.

            // Unblock previous jobs.
            step.resolve_ccd_contacts.removeDependency(.{});
            step.body_set_island_index.removeDependency(.{});

            // The soft body prepare job will create other jobs if needed
            step.soft_body_prepare = try job_system.createJob("SoftBodyPrepare", color_soft_body_prepare, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: a protocol job, its body checks the latch
                    c.physics_system.jobSoftBodyPrepare(c, s);
                }
            }.call, .{ context, step }), .{ .num_dependencies = max_concurrency }); // depends on: solve position constraints.

            // Unblock previous jobs
            JobHandle.removeDependenciesStaticArray(&step.solve_position_constraints, .{});
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Various job entry points

    fn jobStepListeners(self: *Self, step: *Step) void {
        // Read positions (broadphase updates concurrently so we can't write), read/write velocities
        var grant: BodyAccess.Grant = .init(.read_write, .read);
        defer grant.deinit();

        // Can activate bodies only (we cache the amount of active bodies at the beginning of the step in mNumActiveBodiesAtStepStart so we cannot deactivate here)
        var grant_active: BodyManager.GrantActiveBodiesAccess = .init(true, false);
        defer grant_active.deinit();

        const context: PhysicsStepListenerContext = .{
            .delta_time = step.context.step_delta_time,
            .is_first_step = step.is_first,
            .is_last_step = step.is_last,
            .physics_system = self,
        };

        const batch_size: u32 = @intCast(self.physics_settings.step_listeners_batch_size);
        while (true) {
            // Get the start of a new batch
            const batch = step.step_listener_read_idx.value.fetchAdd(batch_size, .seq_cst);
            if (batch >= self.step_listeners.items.len)
                break;

            // Call the listeners
            const i_end = @min(@as(u32, @intCast(self.step_listeners.items.len)), batch + batch_size);
            for (self.step_listeners.items[batch..i_end]) |listener|
                listener.onStep(&context);
        }
    }

    fn jobDetermineActiveConstraints(self: *const Self, step: *Step) void {
        // No body access
        var grant: BodyAccess.Grant = .init(.none, .none);
        defer grant.deinit();

        const num_constraints = self.constraint_manager.getNumConstraints();
        var active_constraints: [determine_active_constraints_batch_size]*Constraint = undefined; // JPH_STACK_ALLOC with a fixed size

        while (true) {
            // Atomically fetch a batch of constraints
            const constraint_idx = step.determine_active_constraint_read_idx.value.fetchAdd(determine_active_constraints_batch_size, .seq_cst);
            if (constraint_idx >= num_constraints)
                break;

            // Calculate the end of the batch
            const constraint_idx_end = @min(num_constraints, constraint_idx + determine_active_constraints_batch_size);

            // Store the active constraints at the start of the step (bodies get activated during the step which in turn may activate constraints leading to an inconsistent shapshot)
            const num_active_constraints = self.constraint_manager.getActiveConstraints(constraint_idx, constraint_idx_end, &active_constraints);

            // Copy the block of active constraints to the global list of active constraints
            if (num_active_constraints > 0) {
                const active_constraint_idx = step.num_active_constraints.value.fetchAdd(num_active_constraints, .seq_cst);
                @memcpy(step.context.active_constraints.items[active_constraint_idx..][0..num_active_constraints], active_constraints[0..num_active_constraints]);
            }
        }
    }

    fn jobApplyGravity(self: *Self, context: *const PhysicsUpdateContext, step: *Step) void {
        // We update velocities and need the rotation to do so
        var grant: BodyAccess.Grant = .init(.read_write, .read);
        defer grant.deinit();

        // Get list of active bodies that we had at the start of the physics update.
        // Any body that is activated as part of the simulation step does not receive gravity this frame.
        // Note that bodies may be activated during this job but not deactivated, this means that only elements
        // will be added to the array. Since the array is made to not reallocate, this is a safe operation.
        const active_bodies = self.body_manager.getActiveBodiesUnsafe(.rigid_body);
        const num_active_bodies_at_step_start = step.num_active_bodies_at_step_start;

        // Fetch delta time once outside the loop
        const delta_time = context.step_delta_time;

        // Update velocities from forces
        while (true) {
            // Atomically fetch a batch of bodies
            var active_body_idx = step.apply_gravity_read_idx.value.fetchAdd(apply_gravity_batch_size, .seq_cst);
            if (active_body_idx >= num_active_bodies_at_step_start)
                break;

            // Calculate the end of the batch
            const active_body_idx_end = @min(num_active_bodies_at_step_start, active_body_idx + apply_gravity_batch_size);

            // Process the batch
            while (active_body_idx < active_body_idx_end) {
                const body = self.body_manager.getBodyMut(active_bodies[active_body_idx]);
                if (body.isDynamic()) {
                    const mp = body.getMotionPropertiesMut();
                    const rotation = body.getRotation();

                    if (body.getApplyGyroscopicForce())
                        mp.applyGyroscopicForceInternal(rotation, delta_time);

                    mp.applyForceTorqueAndDragInternal(rotation, self.gravity, delta_time);
                }
                active_body_idx += 1;
            }
        }
    }

    fn jobSetupVelocityConstraints(self: *const Self, delta_time: f32, step: *Step) void {
        _ = self;

        // We only read positions
        var grant: BodyAccess.Grant = .init(.none, .read);
        defer grant.deinit();

        const num_constraints = step.num_active_constraints.value.load(.seq_cst);

        while (true) {
            // Atomically fetch a batch of constraints
            const constraint_idx = step.setup_velocity_constraints_read_idx.value.fetchAdd(setup_velocity_constraints_batch_size, .seq_cst);
            if (constraint_idx >= num_constraints)
                break;

            ConstraintManager.setupVelocityConstraints(step.context.active_constraints.items[constraint_idx..][0..@min(@as(u32, setup_velocity_constraints_batch_size), num_constraints - constraint_idx)], delta_time);
        }
    }

    fn jobBuildIslandsFromConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) TempAllocator.Error!void {
        // We read constraints and positions
        var grant: BodyAccess.Grant = .init(.read, .read);
        defer grant.deinit();

        // Can only activate bodies
        var grant_active: BodyManager.GrantActiveBodiesAccess = .init(true, false);
        defer grant_active.deinit();

        const num_active_constraints = step.num_active_constraints.value.load(.seq_cst);

        // Prepare the island builder
        try self.island_builder.prepareNonContactConstraints(num_active_constraints, context.temp_allocator);

        // Build the islands
        ConstraintManager.buildIslands(step.context.active_constraints.items[0..num_active_constraints], &self.island_builder, &self.body_manager);
    }

    /// Tries to spawn a new FindCollisions job if max concurrency hasn't been reached yet
    /// (Zolt D5: a job that cannot be created is not an error, the claim and the dependencies are undone and the running
    /// jobs do the work)
    fn trySpawnJobFindCollisions(self: *const Self, step: *Step) void {
        // Get how many jobs we can spawn and check if we can spawn more
        const max_jobs: u32 = step.body_pair_queues.len;
        if (@popCount(step.active_find_collision_jobs.load(.monotonic)) >= max_jobs)
            return;

        // Count how many body pairs we have waiting
        var num_body_pairs: u32 = 0;
        for (step.body_pair_queues.constSlice()) |*queue|
            num_body_pairs +%= queue.write_idx.load(.seq_cst) -% queue.read_idx.load(.seq_cst);

        // Count how many active bodies we have waiting
        const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body) -% step.active_body_read_idx.value.load(.seq_cst);

        // Calculate how many jobs we would like
        const desired_num_jobs = @min((num_body_pairs +% (narrow_phase_batch_size - 1)) / narrow_phase_batch_size +% (num_active_bodies +% (active_bodies_batch_size - 1)) / active_bodies_batch_size, max_jobs);

        while (true) {
            // Get the bit mask of active jobs and see if we can spawn more
            const current_active_jobs = step.active_find_collision_jobs.load(.monotonic);
            const job_index: u32 = @ctz(~current_active_jobs);
            if (job_index >= desired_num_jobs)
                break;

            // Try to claim the job index
            const job_mask = @as(JobMask, 1) << @intCast(job_index);
            const prev_value = step.active_find_collision_jobs.fetchOr(job_mask, .acquire);
            if ((prev_value & job_mask) == 0) {
                // Add dependencies from the find collisions job to the next jobs
                step.update_broadphase_finalize.addDependency(.{});
                step.finalize_islands.addDependency(.{});

                // Start the job
                var job = step.context.job_system.createJob("FindCollisions", color_find_collisions, .init(struct {
                    fn call(s: *Step, ji: u32) void {
                        s.context.physics_system.jobFindCollisions(s, ji);
                    }
                }.call, .{ step, job_index }), .{}) catch {
                    // Zolt (D5): not an error, undo the claim and the dependencies, the running jobs do the work
                    _ = step.active_find_collision_jobs.fetchAnd(~job_mask, .release);
                    step.update_broadphase_finalize.removeDependency(.{});
                    step.finalize_islands.removeDependency(.{});
                    return;
                };
                defer job.deinit();

                // Add the job to the job barrier so the main updating thread can execute the job too
                step.context.barrier.addJob(&job);

                // Spawn only 1 extra job at a time
                return;
            }
        }
    }

    /// Collect the information of a contact allocator in the step (sFinalizeContactAllocator)
    fn finalizeContactAllocator(step: *Step, contact_allocator: *const ContactAllocator) void {
        // Atomically accumulate the number of found manifolds and body pairs
        _ = step.num_body_pairs.fetchAdd(contact_allocator.num_body_pairs, .monotonic);
        _ = step.num_manifolds.fetchAdd(contact_allocator.num_manifolds, .monotonic);

        // Combine update errors
        _ = step.context.errors.fetchOr(contact_allocator.errors.toInt(), .monotonic);
    }

    /// Zolt (D5): a protocol job. Once poisoned it takes no new batch of active bodies and no new body pair and leaves
    /// through Jolt's "found nothing" exit, which finalizes the ContactAllocator, clears its bit in the job mask and
    /// removes the dependencies.
    /// (Jolt disables TSAN for this function: the body pair slots are `.monotonic` atomics in Zolt, see
    /// PhysicsUpdateContext.BodyPairSlot)
    fn jobFindCollisions(self: *Self, step: *Step, job_index: u32) void {
        find: {
            // We read positions and read velocities (for elastic collisions)
            var grant: BodyAccess.Grant = .init(.read, .read);
            defer grant.deinit();

            // Can only activate bodies
            var grant_active: BodyManager.GrantActiveBodiesAccess = .init(true, false);
            defer grant_active.deinit();

            const context = step.context;

            // Allocation context for allocating new contact points
            var contact_allocator = self.contact_manager.getContactAllocator();

            // Determine initial queue to read pairs from if no broadphase work can be done
            // (always start looking at results from the next job)
            var read_queue_idx = (job_index + 1) % step.body_pair_queues.len;

            // Allocate space to temporarily store a batch of active bodies
            var active_bodies: [active_bodies_batch_size]BodyID = undefined; // JPH_STACK_ALLOC with a fixed size

            while (true) {
                // Zolt (D5): no new work after an out of memory error (checked before the claims below)
                const poisoned = context.isOutOfMemory();

                // Check if there are active bodies to be processed
                const active_bodies_read_idx = step.active_body_read_idx.value.load(.seq_cst);
                const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body);
                if (!poisoned and active_bodies_read_idx < num_active_bodies) {
                    // Take a batch of active bodies
                    const active_bodies_read_idx_end = @min(num_active_bodies, active_bodies_read_idx + active_bodies_batch_size);
                    if (step.active_body_read_idx.value.cmpxchgStrong(active_bodies_read_idx, active_bodies_read_idx_end, .seq_cst, .seq_cst) == null) {
                        // Callback when a new body pair is found
                        const MyBodyPairCallback = struct {
                            pub const overrides = .{.addHit};

                            base: BodyPairCollector = .init(@This()),
                            step: *Step,
                            contact_allocator: *ContactAllocator,
                            job_index: u32,

                            // Callback function when a body pair is found
                            pub fn addHit(cb: *@This(), pair: *const BodyPair) void {
                                // Check if we have space in our write queue
                                const queue = &cb.step.body_pair_queues.slice()[cb.job_index];
                                const body_pairs_in_queue = queue.write_idx.load(.seq_cst) -% queue.read_idx.load(.seq_cst);
                                if (body_pairs_in_queue >= cb.step.max_body_pairs_per_queue) {
                                    // Buffer full, process the pair now
                                    cb.step.context.physics_system.processBodyPair(cb.contact_allocator, pair.*) catch cb.step.context.setOutOfMemory();
                                } else {
                                    // Store the pair in our own queue
                                    PhysicsUpdateContext.storeBodyPair(&cb.step.context.body_pairs.items[cb.job_index * cb.step.max_body_pairs_per_queue + queue.write_idx.load(.seq_cst) % cb.step.max_body_pairs_per_queue], pair.*);
                                    _ = queue.write_idx.fetchAdd(1, .seq_cst);
                                }
                            }
                        };
                        var add_pair: MyBodyPairCallback = .{ .step = step, .contact_allocator = &contact_allocator, .job_index = job_index };

                        // Copy active bodies to temporary array, broadphase will reorder them
                        const batch_size = active_bodies_read_idx_end - active_bodies_read_idx;
                        @memcpy(active_bodies[0..batch_size], self.body_manager.getActiveBodiesUnsafe(.rigid_body)[active_bodies_read_idx..active_bodies_read_idx_end]);

                        // Find pairs in the broadphase
                        self.getBroadPhase().findCollidingPairs(active_bodies[0..batch_size], self.physics_settings.speculative_contact_distance, self.object_vs_broad_phase_layer_filter, self.object_layer_pair_filter, &add_pair.base);

                        // Check if we have enough pairs in the buffer to start a new job
                        const queue = &step.body_pair_queues.slice()[job_index];
                        const body_pairs_in_queue = queue.write_idx.load(.seq_cst) -% queue.read_idx.load(.seq_cst);
                        if (body_pairs_in_queue >= narrow_phase_batch_size)
                            self.trySpawnJobFindCollisions(step);
                    }
                } else {
                    // Lockless loop to get the next body pair from the pairs buffer
                    const first_read_queue_idx = read_queue_idx;
                    while (true) {
                        const queue = &step.body_pair_queues.slice()[read_queue_idx];

                        // Get the next pair to process
                        const pair_idx = queue.read_idx.load(.seq_cst);

                        // If the pair hasn't been written yet (Zolt D5: a poisoned job treats every queue as empty)
                        if (poisoned or pair_idx >= queue.write_idx.load(.seq_cst)) {
                            // Go to the next queue
                            read_queue_idx = (read_queue_idx + 1) % step.body_pair_queues.len;

                            // If we're back at the first queue, we've looked at all of them and found nothing
                            if (read_queue_idx == first_read_queue_idx) {
                                // Collect information from the contact allocator and accumulate it in the step.
                                finalizeContactAllocator(step, &contact_allocator);

                                // Mark this job as inactive
                                _ = step.active_find_collision_jobs.fetchAnd(~(@as(JobMask, 1) << @intCast(job_index)), .release);

                                // Trigger the next jobs (Zolt: after the grants, see below)
                                break :find;
                            }

                            // Try again reading from the next queue
                            continue;
                        }

                        // Copy the body pair out of the buffer
                        const bp = PhysicsUpdateContext.loadBodyPair(&context.body_pairs.items[read_queue_idx * step.max_body_pairs_per_queue + pair_idx % step.max_body_pairs_per_queue]);

                        // Mark this pair as taken
                        if (queue.read_idx.cmpxchgStrong(pair_idx, pair_idx + 1, .seq_cst, .seq_cst) == null) {
                            // Process the actual body pair
                            self.processBodyPair(&contact_allocator, bp) catch context.setOutOfMemory();
                            break;
                        }
                    }
                }
            }
        }

        // Trigger the next jobs
        // (Zolt: after the scope of the grants above. JobSystemSingleThreaded runs the jobs that become ready inside this
        // call, and their Grant asserts that no other grant is active on the thread. Jolt triggers inside the scope,
        // which asserts with JobSystemSingleThreaded from the second collision step on (Jolt only tests that job system
        // with one step); with the Zolt start gate (D4) already in the first step. Same in jobSoftBodyPrepare.)
        step.update_broadphase_finalize.removeDependency(.{});
        step.finalize_islands.removeDependency(.{});
    }

    /// Process narrow phase for a single body pair
    /// (Zolt: Allocator.Error, the narrow phase allocates: InternalEdgeRemovingCollector, the collectors)
    fn processBodyPair(self: *Self, contact_allocator: *ContactAllocator, body_pair: BodyPair) Allocator.Error!void {
        _ = .{ self, contact_allocator, body_pair };

        // REGION R2: stub. The narrow phase between the bodies of a pair, the contact cache and the contact constraints
        // (GetContactsFromCache, AddBodyPair, AddContactConstraint, the contact listener calls) need the
        // ContactConstraintManager (A4). Only reached when the broad phase reports a body pair (overlapping bounds of
        // bodies whose layers collide).
        @panic("PhysicsSystem.processBodyPair: REGION R2 (needs the ContactConstraintManager, A4)");
    }

    fn jobFinalizeIslands(self: *Self, context: *PhysicsUpdateContext) TempAllocator.Error!void {
        // We only touch island data
        var grant: BodyAccess.Grant = .init(.none, .none);
        defer grant.deinit();

        // Finish collecting the islands, at this point the active body list doesn't change so it's safe to access
        const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body);
        try self.island_builder.finalize(self.body_manager.getActiveBodiesUnsafe(.rigid_body)[0..num_active_bodies], self.contact_manager.getNumConstraints(), context.temp_allocator);

        // Prepare the large island splitter
        if (self.physics_settings.use_large_island_splitter)
            try self.large_island_splitter.prepare(&self.island_builder, self.body_manager.getNumActiveBodies(.rigid_body), context.temp_allocator);
    }

    fn jobBodySetIslandIndex(self: *Self) void {
        // We only touch island data
        var grant: BodyAccess.Grant = .init(.none, .none);
        defer grant.deinit();

        // Loop through the result and tag all bodies with an island index
        var island_idx: u32 = 0;
        const n = self.island_builder.getNumIslands();
        while (island_idx < n) : (island_idx += 1) {
            for (self.island_builder.getBodiesInIsland(island_idx)) |body|
                self.body_manager.getBodyMut(body).getMotionPropertiesMut().setIslandIndexInternal(island_idx);
        }
    }

    fn jobSolveVelocityConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        // We update velocities and need to read positions to do so
        var grant: BodyAccess.Grant = .init(.read_write, .read);
        defer grant.deinit();

        var check_islands = true;
        var check_split_islands = self.physics_settings.use_large_island_splitter;
        while (true) {
            // First try to get work from large islands
            if (check_split_islands) {
                var batch: LargeIslandSplitter.Batch = undefined;
                switch (self.large_island_splitter.fetchNextBatch(&batch)) {
                    .batch_retrieved => {
                        // REGION R3: stub. Warm start (first iteration) or solve the batch, mark it as processed and store
                        // the applied impulses (needs A2's CalculateSolverSteps, A4 and A5; the splitter stub never
                        // creates a batch)
                        @panic("PhysicsSystem.jobSolveVelocityConstraints: REGION R3 (split island batch)");
                    },

                    .waiting_for_batch => {},

                    .all_batches_done => check_split_islands = false,
                }
            }

            // If that didn't succeed try to process an island
            if (check_islands) {
                // Next island
                const island_idx = step.solve_velocity_constraints_next_island.fetchAdd(1, .seq_cst);
                if (island_idx >= self.island_builder.getNumIslands()) {
                    // We processed all islands, stop checking islands
                    check_islands = false;
                    continue;
                }

                // Get iterators for this island
                const constraints = self.island_builder.getConstraintsInIsland(island_idx);
                const contacts = self.island_builder.getContactsInIsland(island_idx);
                const has_constraints = constraints.len > 0;
                const has_contacts = contacts.len > 0;

                // If we don't have any contacts or constraints, we know that none of the following islands have any contacts or constraints
                // (because they're sorted by most constraints first). This means we're done.
                if (!has_contacts and !has_constraints) {
                    if (Core.enable_asserts) {
                        // Validate our assumption that the next islands don't have any constraints or contacts
                        var i = island_idx;
                        while (i < self.island_builder.getNumIslands()) : (i += 1) {
                            std.debug.assert(self.island_builder.getConstraintsInIsland(i).len == 0);
                            std.debug.assert(self.island_builder.getContactsInIsland(i).len == 0);
                        }
                    }

                    check_islands = false;
                    continue;
                }

                // REGION R3: stub. Convert the contact indices to offsets, sort the constraints and the contacts, split
                // the island or warm start and solve it with CalculateSolverSteps, store the number of position steps
                // and the applied impulses (needs A2's CalculateSolverSteps, A4 and A5).
                _ = context;
                @panic("PhysicsSystem.jobSolveVelocityConstraints: REGION R3 (an island with constraints or contacts)");
            } else if (check_split_islands) {
                // If there are split islands, but we didn't do any work, give up a time slice
                std.Thread.yield() catch {};
            } else {
                // No more work
                break;
            }
        }
    }

    fn jobPreIntegrateVelocity(self: *Self, context: *PhysicsUpdateContext, step: *Step) TempAllocator.Error!void {
        // Reserve enough space for all bodies that may need a cast
        const temp_allocator = context.temp_allocator;
        std.debug.assert(step.ccd_bodies.isEmpty());
        step.ccd_bodies = try .alloc(temp_allocator, self.body_manager.getNumActiveCCDBodies());

        // Initialize the mapping table between active body and CCD body
        std.debug.assert(step.active_body_to_ccd_body.isEmpty());
        step.active_body_to_ccd_body = try .alloc(temp_allocator, self.body_manager.getNumActiveBodies(.rigid_body));

        // Prepare the split island builder for solving the position constraints
        self.large_island_splitter.prepareForSolvePositions();
    }

    fn jobIntegrateVelocity(self: *Self, context: *const PhysicsUpdateContext, step: *Step) void {
        // We update positions and need velocity to do so, we also clamp velocities so need to write to them
        var grant: BodyAccess.Grant = .init(.read_write, .read_write);
        defer grant.deinit();

        const delta_time = context.step_delta_time;
        const active_bodies = self.body_manager.getActiveBodiesUnsafe(.rigid_body);
        const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body);

        while (true) {
            // Atomically fetch a batch of bodies
            var active_body_idx = step.integrate_velocity_read_idx.fetchAdd(integrate_velocity_batch_size, .seq_cst);
            if (active_body_idx >= num_active_bodies)
                break;

            // Calculate the end of the batch
            const active_body_idx_end = @min(num_active_bodies, active_body_idx + integrate_velocity_batch_size);

            // Process the batch
            while (active_body_idx < active_body_idx_end) {
                // Update the positions using an Symplectic Euler step (which integrates using the updated velocity v1' rather
                // than the original velocity v1):
                // x1' = x1 + h * v1'
                // At this point the active bodies list does not change, so it is safe to access the array.
                const body_id = active_bodies[active_body_idx];
                const body = self.body_manager.getBodyMut(body_id);
                const mp = body.getMotionPropertiesMut();

                // TODO(determinism_log): JPH_DET_LOG("JobIntegrateVelocity: id: " << body_id << " v: " << body.GetLinearVelocity() << " w: " << body.GetAngularVelocity());

                // Clamp velocities (not for kinematic bodies)
                if (body.isDynamic()) {
                    mp.clampLinearVelocity();
                    mp.clampAngularVelocity();
                }

                // Update the rotation of the body according to the angular velocity
                // For motion type discrete we need to do this anyway, for motion type linear cast we have multiple choices
                // 1. Rotate the body first and then sweep
                // 2. First sweep and then rotate the body at the end
                // 3. Pick some in between rotation (e.g. half way), then sweep and finally rotate the remainder
                // (1) has some clear advantages as when a long thin body hits a surface away from the center of mass, this will result in a large angular velocity and a limited reduction in linear velocity.
                // When simulation the rotation first before doing the translation, the body will be able to rotate away from the contact point allowing the center of mass to approach the surface. When using
                // approach (2) in this case what will happen is that we will immediately detect the same collision again (the body has not rotated and the body was already colliding at the end of the previous
                // time step) resulting in a lot of stolen time and the body appearing to be frozen in an unnatural pose (like it is glued at an angle to the surface). (2) obviously has some negative side effects
                // too as simulating the rotation first may cause it to tunnel through a small object that the linear cast might have otherwise detected. In any case a linear cast is not good for detecting
                // tunneling due to angular rotation, so we don't care about that too much (you'd need a full cast to take angular effects into account).
                body.addRotationStep(body.getAngularVelocity().mulScalar(delta_time));

                // Get delta position
                const delta_pos = body.getLinearVelocity().mulScalar(delta_time);

                // If the position should be updated (or if it is delayed because of CCD)
                var update_position = true;

                switch (mp.getMotionQuality()) {
                    .discrete => {
                        // No additional collision checking to be done
                    },

                    .linear_cast => {
                        if (body.isDynamic() // Kinematic bodies cannot be stopped
                        and !body.isSensor()) // We don't support CCD sensors
                        {
                            // Determine inner radius (the smallest sphere that fits into the shape)
                            const inner_radius = body.getShape().getInnerRadius();
                            std.debug.assert(inner_radius > 0.0); // The shape has no inner radius, this makes the shape unsuitable for the linear cast motion quality as we cannot move it without risking tunneling.

                            // Measure translation in this step and check if it above the threshold to perform a linear cast
                            const linear_cast_threshold_sq = math.square(self.physics_settings.linear_cast_threshold * inner_radius);
                            if (delta_pos.lengthSq() > linear_cast_threshold_sq) {
                                // This body needs a cast
                                const ccd_body_idx = step.num_ccd_bodies.fetchAdd(1, .seq_cst);
                                std.debug.assert(active_body_idx < step.active_body_to_ccd_body.items.len);
                                step.active_body_to_ccd_body.items[active_body_idx] = @intCast(ccd_body_idx);
                                step.ccd_bodies.items[ccd_body_idx] = .init(body_id, delta_pos, linear_cast_threshold_sq, math.min(self.physics_settings.penetration_slop, self.physics_settings.linear_cast_max_penetration * inner_radius));

                                update_position = false;
                            }
                        }
                    },
                }

                if (update_position) {
                    // Move the body now
                    body.addPositionStep(delta_pos);

                    // We did not create a CCD body
                    step.active_body_to_ccd_body.items[active_body_idx] = -1;
                }

                active_body_idx += 1;
            }
        }
    }

    /// Zolt (D5): a protocol job. Once poisoned it takes the "no CCD bodies" path. A FindCCDContacts job that cannot be
    /// created is not an error: the created jobs process every CCD body (they take bodies until mNextCCDBody runs out),
    /// and when none could be created this job does the work itself.
    fn jobPostIntegrateVelocity(self: *const Self, context: *PhysicsUpdateContext, step: *Step) void {
        // Validate that our reservations were correct
        std.debug.assert(step.num_ccd_bodies.load(.monotonic) <= self.body_manager.getNumActiveCCDBodies());

        const num_ccd_bodies = step.num_ccd_bodies.load(.seq_cst);
        if (num_ccd_bodies == 0 or context.isOutOfMemory()) {
            // No continuous collision detection jobs -> kick the next job ourselves
            step.contact_removed_callbacks.removeDependency(.{});
        } else {
            // Run the continuous collision detection jobs
            const num_continuous_collision_jobs: u32 = @min((num_ccd_bodies + num_ccd_bodies_per_job - 1) / num_ccd_bodies_per_job, context.getMaxConcurrency());
            step.resolve_ccd_contacts.addDependency(.{ .count = @intCast(num_continuous_collision_jobs) });
            step.contact_removed_callbacks.addDependency(.{ .count = @intCast(num_continuous_collision_jobs - 1) }); // Already had 1 dependency
            var i: u32 = 0;
            while (i < num_continuous_collision_jobs) : (i += 1) {
                var job = context.job_system.createJob("FindCCDContacts", color_find_ccd_contacts, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobFindCCDContacts(c, s);

                        s.resolve_ccd_contacts.removeDependency(.{});
                        s.contact_removed_callbacks.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{}) catch {
                    // Zolt (D5): not an error, see the function comment
                    if (i == 0)
                        context.physics_system.jobFindCCDContacts(context, step);
                    const num_missing: i32 = @intCast(num_continuous_collision_jobs - i);
                    step.resolve_ccd_contacts.removeDependency(.{ .count = num_missing });
                    step.contact_removed_callbacks.removeDependency(.{ .count = num_missing });
                    return;
                };
                defer job.deinit();
                context.barrier.addJob(&job);
            }
        }
    }

    /// Helper function to calculate the motion of a body during this CCD step
    fn calculateBodyMotion(body: *const Body, delta_time: f32) Vec3 {
        // If the body is linear casting, the body has not yet moved so we need to calculate its motion
        if (body.isDynamic() and body.getMotionProperties().getMotionQuality() == .linear_cast)
            return body.getLinearVelocity().mulScalar(delta_time);

        // Body has already moved, so we don't need to correct for anything
        return Vec3.zero();
    }

    /// Helper function that finds the CCD body corresponding to a body (if it exists)
    fn getCCDBody(body: *const Body, step: *Step) ?*CCDBody {
        // Only rigid bodies can have a CCD body
        if (!body.isRigidBody())
            return null;

        // If the body has no motion properties it cannot have a CCD body
        const motion_properties = body.getMotionPropertiesUnchecked() orelse return null;

        // If it is not active it cannot have a CCD body
        const active_index = motion_properties.getIndexInActiveBodiesInternal();
        if (active_index == MotionPropertiesFile.MotionProperties.inactive_index)
            return null;

        // Check if the active body has a corresponding CCD body
        std.debug.assert(active_index < step.active_body_to_ccd_body.items.len); // Ensure that the body has a mapping to CCD body
        const ccd_index = step.active_body_to_ccd_body.items[active_index];
        if (ccd_index < 0)
            return null;

        const ccd_body = &step.ccd_bodies.items[@intCast(ccd_index)];
        std.debug.assert(ccd_body.body_id1.eql(body.getID())); // We found the wrong CCD body!
        return ccd_body;
    }

    /// The CCD bodies are cast through the broad phase like Jolt; the narrow phase cast against a candidate body is
    /// REGION R4 (see the panic in CCDBroadPhaseCollector.addHit).
    fn jobFindCCDContacts(self: *Self, context: *const PhysicsUpdateContext, step: *Step) void {
        // We only read positions, but the validate callback may read body positions and velocities
        var grant: BodyAccess.Grant = .init(.read, .read);
        defer grant.deinit();

        // Allocation context for allocating new contact points
        const contact_allocator = self.contact_manager.getContactAllocator();

        // REGION R4: the ShapeCastSettings of the narrow phase cast (mUseShrunkenShapeAndConvexRadius, the back face
        // modes, mReturnDeepestPoint, mCollectFacesMode, mActiveEdgeMode)

        while (true) {
            // Fetch the next body to cast
            const idx = step.next_ccd_body.fetchAdd(1, .seq_cst);
            if (idx >= step.num_ccd_bodies.load(.seq_cst))
                break;
            const ccd_body = &step.ccd_bodies.items[idx];
            const body = self.body_manager.getBody(ccd_body.body_id1);

            // Filter out layers
            const broadphase_layer_filter = self.getDefaultBroadPhaseLayerFilter(body.getObjectLayer());
            const object_layer_filter = self.getDefaultLayerFilter(body.getObjectLayer());

            // TODO(debug_renderer): Draw start and end shape of cast (sDrawMotionQualityLinearCast)

            // This collector wraps the narrowphase collector and collects the closest hit
            const CCDBroadPhaseCollector = struct {
                pub const overrides = .{.addHit};

                base: CastShapeBodyCollector = .init(@This()),
                ccd_body: *const CCDBody,
                body1: *const Body,
                body1_extent: Vec3,
                shape_cast: RShapeCast,
                body_manager: *const BodyManager,
                step: *Step,
                delta_time: f32,

                pub fn addHit(collector: *@This(), result: *const BroadPhaseCastResult) void {
                    std.debug.assert(result.fraction <= collector.base.getEarlyOutFraction()); // This hit should not have been passed on to the collector

                    // Test if we're colliding with ourselves
                    if (collector.body1.getID().eql(result.body_id))
                        return;

                    // Avoid treating duplicates, if both bodies are doing CCD then only consider collision if body ID < other body ID
                    const body2 = collector.body_manager.getBody(result.body_id);
                    const ccd_body2 = getCCDBody(body2, collector.step);
                    if (ccd_body2 != null and collector.ccd_body.body_id1.greaterThan(ccd_body2.?.body_id1))
                        return;

                    // Test group filter
                    if (!collector.body1.getCollisionGroup().canCollide(body2.getCollisionGroup()))
                        return;

                    // TODO: For now we ignore sensors
                    if (body2.isSensor())
                        return;

                    // Get relative movement of these two bodies
                    const direction = collector.shape_cast.direction.sub(calculateBodyMotion(body2, collector.delta_time));

                    // Test if the remaining movement is less than our movement threshold
                    if (direction.lengthSq() < collector.ccd_body.linear_cast_threshold_sq)
                        return;

                    // Get the bounds of 2, widen it by the extent of 1 and test a ray to see if it hits earlier than the current early out fraction
                    var bounds = body2.getWorldSpaceBounds().*;
                    bounds.min = bounds.min.sub(collector.body1_extent);
                    bounds.max = bounds.max.add(collector.body1_extent);
                    const hit_fraction = RayAABoxFile.rayAABox(toVec3(collector.shape_cast.center_of_mass_start.getTranslation()), .init(direction), bounds.min, bounds.max);
                    if (hit_fraction > collector.base.getPositiveEarlyOutFraction()) // If early out fraction <= 0, we have the possibility of finding a deeper hit so we need to clamp the early out fraction
                        return;

                    // REGION R4: stub. Reset the narrow phase collector for this body pair, set body 2 on the shape
                    // filter, cast the shape against body 2 (TransformedShape.castShape with the CCDNarrowPhaseCollector:
                    // ValidateContactPoint of the ContactConstraintManager, the earliest hit, the movement of body 2) and
                    // update the early out fraction (needs A4 and A7's SimShapeFilterWrapper).
                    @panic("PhysicsSystem.jobFindCCDContacts: REGION R4 (narrow phase cast against a candidate body)");
                }
            };

            // Check if we collide with any other body. Note that we use the non-locking interface as we know the broadphase cannot be modified at this point.
            const shape_cast: RShapeCast = .init(body.getShape(), Vec3.one(), body.getCenterOfMassTransform(), ccd_body.delta_position);
            var bp_collector: CCDBroadPhaseCollector = .{
                .ccd_body = ccd_body,
                .body1 = body,
                .body1_extent = shape_cast.shape_world_bounds.getExtent(),
                .shape_cast = shape_cast,
                .body_manager = &self.body_manager,
                .step = step,
                .delta_time = context.step_delta_time,
            };
            self.getBroadPhase().castAABoxNoLock(&.{ .box = shape_cast.shape_world_bounds, .direction = shape_cast.direction }, &bp_collector.base, &broadphase_layer_filter.base, &object_layer_filter.base);

            // Check if there was a hit
            if (ccd_body.fraction_plus_slop < 1.0) {
                // REGION R4: stub. The contact manifold of the hit (ManifoldBetweenTwoFaces), the contact point
                // callbacks (OnCCDContactAdded), sensors and the average contact point (needs A4). Not reached: the
                // broad phase collector above panics before a hit is stored.
                @panic("PhysicsSystem.jobFindCCDContacts: REGION R4 (a CCD hit)");
            }
        }

        // Collect information from the contact allocator and accumulate it in the step.
        finalizeContactAllocator(step, &contact_allocator);
    }

    /// Zolt (D5): a protocol job. The work is skipped when poisoned, the frees at the end always happen.
    fn jobResolveCCDContacts(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        // Read/write body access
        var grant: BodyAccess.Grant = .init(.read_write, .read_write);
        defer grant.deinit();

        // We activate bodies that we collide with
        var grant_active: BodyManager.GrantActiveBodiesAccess = .init(true, false);
        defer grant_active.deinit();

        const temp_allocator = context.temp_allocator;

        // Check if there's anything to do
        const num_ccd_bodies = step.num_ccd_bodies.load(.seq_cst);
        if (num_ccd_bodies > 0 and !context.isOutOfMemory())
            self.resolveCCDContacts(temp_allocator, step, num_ccd_bodies) catch context.setOutOfMemory();

        // Ensure we free the CCD bodies array now, will not call the destructor!
        step.active_body_to_ccd_body.free(temp_allocator);
        step.ccd_bodies.free(temp_allocator);
    }

    /// The work of JobResolveCCDContacts (Zolt: Allocator.Error from the sort array)
    fn resolveCCDContacts(self: *Self, temp_allocator: TempAllocator, step: *Step, num_ccd_bodies: u32) TempAllocator.Error!void {
        // Sort on fraction so that we process earliest collisions first
        // This is needed to make the simulation deterministic and also to be able to stop contact processing
        // between body pairs if an earlier hit was found involving the body by another CCD body
        // (if it's body ID < this CCD body's body ID - see filtering logic in CCDBroadPhaseCollector)
        var sorted_ccd_bodies = try TempArray(*CCDBody).alloc(temp_allocator, num_ccd_bodies);
        defer sorted_ccd_bodies.free(temp_allocator); // JPH_SCOPE_EXIT
        {
            // We don't want to copy the entire struct (it's quite big), so we create a pointer array first
            for (sorted_ccd_bodies.items, step.ccd_bodies.items[0..num_ccd_bodies]) |*dst, *src|
                dst.* = src;

            // Which we then sort
            quickSort(*CCDBody, sorted_ccd_bodies.items, {}, struct {
                fn lessThan(_: void, body1: *CCDBody, body2: *CCDBody) bool {
                    if (body1.fraction_plus_slop != body2.fraction_plus_slop)
                        return body1.fraction_plus_slop < body2.fraction_plus_slop;

                    return body1.body_id1.lessThan(body2.body_id1);
                }
            }.lessThan);
        }

        // We can collide with bodies that are not active, we track them here so we can activate them in one go at the end.
        // This is also needed because we can't modify the active body array while we iterate it.
        // (REGION R4: the batch of cBodiesBatch = 64 bodies to activate, filled by the hit path below)

        for (sorted_ccd_bodies.items) |ccd_body| {
            const body1 = self.body_manager.getBodyMut(ccd_body.body_id1);

            // If there was a hit
            if (!ccd_body.body_id2.isInvalid()) {
                // REGION R4: stub. Shorten the other CCD body, sSolveCCDContact (rigid body 2) or the soft body face
                // response, and the activation of body 2 (needs A4; the hit path of JobFindCCDContacts panics first).
                @panic("PhysicsSystem.jobResolveCCDContacts: REGION R4 (a CCD hit)");
            }

            // Update body position
            body1.addPositionStep(ccd_body.delta_position.mulScalar(ccd_body.fraction_plus_slop));
        }

        // Activate the requested bodies (REGION R4: the bodies collected by the hit path)
    }

    /// Zolt (D5): a protocol job. When poisoned the write cache is discarded instead of swapped (D10).
    fn jobContactRemovedCallbacks(self: *Self, context: *const PhysicsUpdateContext, step: *const Step) void {
        // We don't touch any bodies
        var grant: BodyAccess.Grant = .init(.none, .none);
        defer grant.deinit();

        // Reset the Body::EFlags::InvalidateContactCache flag for all bodies
        self.body_manager.validateContactCacheForAllBodies();

        // Zolt (D10): the write cache of a poisoned update is incomplete
        if (context.isOutOfMemory()) {
            self.contact_manager.finalizeContactCacheAfterOutOfMemory();
            return;
        }

        // Finalize the contact cache (this swaps the read and write versions of the contact cache)
        // Trigger all contact removed callbacks by looking at last step contact points that have not been flagged as reused
        self.contact_manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(step.num_body_pairs.load(.seq_cst), step.num_manifolds.load(.seq_cst));
    }

    /// This helper batches up bodies that need to put to sleep to avoid contention on the activation mutex
    const BodiesToSleep = struct {
        const bodies_to_sleep_size = 512;
        const max_bodies_to_put_in_buffer = 128;

        body_manager: *BodyManager,
        bodies_to_sleep_buffer: *[bodies_to_sleep_size]BodyID,
        num_bodies_in_buffer: usize = 0,

        fn init(body_manager: *BodyManager, bodies_to_sleep_buffer: *[bodies_to_sleep_size]BodyID) BodiesToSleep {
            return .{ .body_manager = body_manager, .bodies_to_sleep_buffer = bodies_to_sleep_buffer };
        }

        /// Destructor: flush the bodies to sleep buffer
        fn deinit(self: *BodiesToSleep) void {
            // Flush the bodies to sleep buffer
            if (self.num_bodies_in_buffer > 0)
                self.body_manager.deactivateBodies(self.bodies_to_sleep_buffer[0..self.num_bodies_in_buffer]);
        }

        fn putToSleep(self: *BodiesToSleep, bodies: []const BodyID) void {
            const num_bodies_to_sleep = bodies.len;
            if (num_bodies_to_sleep > max_bodies_to_put_in_buffer) {
                // Too many bodies, deactivate immediately
                self.body_manager.deactivateBodies(bodies);
            } else {
                // Check if there's enough space in the bodies to sleep buffer
                if (self.num_bodies_in_buffer + num_bodies_to_sleep > bodies_to_sleep_size) {
                    // Flush the bodies to sleep buffer
                    self.body_manager.deactivateBodies(self.bodies_to_sleep_buffer[0..self.num_bodies_in_buffer]);
                    self.num_bodies_in_buffer = 0;
                }

                // Copy the bodies in the buffer
                @memcpy(self.bodies_to_sleep_buffer[self.num_bodies_in_buffer..][0..num_bodies_to_sleep], bodies);
                self.num_bodies_in_buffer += num_bodies_to_sleep;
            }
        }
    };

    /// Called at the end of JobSolveVelocityConstraints to check if bodies need to go to sleep and to update their bounding box in the broadphase
    fn checkSleepAndUpdateBounds(self: *Self, island_index: u32, context: *const PhysicsUpdateContext, step: *const Step, bodies_to_sleep: *BodiesToSleep) void {
        // Get the bodies that belong to this island
        const bodies = self.island_builder.getBodiesInIsland(island_index);

        // Only check sleeping in the last step
        // Also resets force and torque used during the apply gravity phase
        if (step.is_last) {
            comptime std.debug.assert(@intFromEnum(CanSleep.cannot_sleep) == 0 and @intFromEnum(CanSleep.can_sleep) == 1); // Loop below makes this assumption
            var all_can_sleep: u32 = if (self.physics_settings.allow_sleeping) @intFromEnum(CanSleep.can_sleep) else @intFromEnum(CanSleep.cannot_sleep);

            const time_before_sleep = self.physics_settings.time_before_sleep;
            const max_movement = self.physics_settings.point_velocity_sleep_threshold * time_before_sleep;

            for (bodies) |body_id| {
                const body = self.body_manager.getBodyMut(body_id);

                // Update bounding box
                body.calculateWorldSpaceBoundsInternal();

                // Update sleeping
                all_can_sleep &= @intFromEnum(body.updateSleepStateInternal(context.step_delta_time, max_movement, time_before_sleep));

                // Reset force and torque
                const mp = body.getMotionPropertiesMut();
                mp.resetForce();
                mp.resetTorque();
            }

            // If all bodies indicate they can sleep we can deactivate them
            if (all_can_sleep == @intFromEnum(CanSleep.can_sleep))
                bodies_to_sleep.putToSleep(bodies);
        } else {
            // Update bounding box only for all other steps
            for (bodies) |body_id| {
                const body = self.body_manager.getBodyMut(body_id);
                body.calculateWorldSpaceBoundsInternal();
            }
        }

        // Notify broadphase of changed objects
        // Note: Shuffles the BodyID's around!!!
        self.getBroadPhase().notifyBodiesAABBChanged(bodies, .{ .take_lock = false });
    }

    fn jobSolvePositionConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        // We fix up position errors
        var grant: BodyAccess.Grant = .init(.none, .read_write);
        defer grant.deinit();

        // Can only deactivate bodies
        var grant_active: BodyManager.GrantActiveBodiesAccess = .init(false, true);
        defer grant_active.deinit();

        const delta_time = context.step_delta_time;
        const baumgarte = self.physics_settings.baumgarte;
        const active_constraints = context.active_constraints.items;

        // Keep a buffer of bodies that need to go to sleep in order to not constantly lock the active bodies mutex and create contention between all solving threads
        var bodies_to_sleep_buffer: [BodiesToSleep.bodies_to_sleep_size]BodyID = undefined; // JPH_STACK_ALLOC with a fixed size
        var bodies_to_sleep: BodiesToSleep = .init(&self.body_manager, &bodies_to_sleep_buffer);
        defer bodies_to_sleep.deinit();

        var check_islands = true;
        var check_split_islands = self.physics_settings.use_large_island_splitter;
        while (true) {
            // First try to get work from large islands
            if (check_split_islands) {
                var batch: LargeIslandSplitter.Batch = undefined;
                switch (self.large_island_splitter.fetchNextBatch(&batch)) {
                    .batch_retrieved => {
                        const start_tick = if (track_simulation_stats) getProcessorTickCount() else {};

                        // Solve the batch
                        _ = ConstraintManager.solvePositionConstraints(active_constraints, batch.constraints, delta_time, baumgarte);
                        _ = self.contact_manager.solvePositionConstraints(batch.contacts);

                        // Mark the batch as processed
                        var last_iteration: bool = undefined;
                        var final_batch: bool = undefined;
                        self.large_island_splitter.markBatchProcessed(batch.split_island_index, batch.constraints, batch.contacts, &last_iteration, &final_batch);

                        // The final batch will update all bounds and check sleeping
                        if (final_batch)
                            self.checkSleepAndUpdateBounds(self.large_island_splitter.getIslandIndex(batch.split_island_index), context, step, &bodies_to_sleep);

                        if (track_simulation_stats) {
                            const num_ticks = getProcessorTickCount() -% start_tick;
                            _ = self.island_builder.getIslandStats(self.large_island_splitter.getIslandIndex(batch.split_island_index)).position_constraint_ticks.fetchAdd(num_ticks, .monotonic);
                        }

                        // We processed work, loop again
                        continue;
                    },

                    .waiting_for_batch => {},

                    .all_batches_done => check_split_islands = false,
                }
            }

            // If that didn't succeed try to process an island
            if (check_islands) {
                // Next island
                const island_idx = step.solve_position_constraints_next_island.fetchAdd(1, .seq_cst);
                if (island_idx >= self.island_builder.getNumIslands()) {
                    // We processed all islands, stop checking islands
                    check_islands = false;
                    continue;
                }

                // Get iterators for this island
                const constraints = self.island_builder.getConstraintsInIsland(island_idx);
                const contacts = self.island_builder.getContactsInIsland(island_idx);

                // If this island is a large island, it will be picked up as a batch and we don't need to do anything here
                const num_items = constraints.len + contacts.len;
                if (self.physics_settings.use_large_island_splitter and num_items >= LargeIslandSplitter.large_island_treshold)
                    continue;

                const start_tick = if (track_simulation_stats) getProcessorTickCount() else {};

                // Check if this island needs solving
                if (num_items > 0) {
                    // Iterate
                    const num_position_steps = self.island_builder.getNumPositionSteps(island_idx);
                    var position_step: u32 = 0;
                    while (position_step < num_position_steps) : (position_step += 1) {
                        var applied_impulse = ConstraintManager.solvePositionConstraints(active_constraints, constraints, delta_time, baumgarte);
                        applied_impulse = self.contact_manager.solvePositionConstraints(contacts) or applied_impulse;
                        if (!applied_impulse)
                            break;
                    }
                }

                const solve_position_ticks = if (track_simulation_stats) getProcessorTickCount() else {};
                if (track_simulation_stats) {
                    // Accumulate time spent in solving position constraints
                    _ = self.island_builder.getIslandStats(island_idx).position_constraint_ticks.fetchAdd(solve_position_ticks -% start_tick, .monotonic);
                }

                // After solving we will update all bounds and check sleeping
                self.checkSleepAndUpdateBounds(island_idx, context, step, &bodies_to_sleep);

                if (track_simulation_stats) {
                    // Accumulate time spent in updating bounding box
                    _ = self.island_builder.getIslandStats(island_idx).update_bounds_ticks.fetchAdd(getProcessorTickCount() -% solve_position_ticks, .monotonic);
                }
            } else if (check_split_islands) {
                // If there are split islands, but we didn't do any work, give up a time slice
                std.Thread.yield() catch {};
            } else {
                // No more work
                break;
            }
        }
    }

    /// Zolt (D5): a protocol job. Its "no active soft bodies" path kicks StartNextStep; once poisoned the list of active
    /// soft bodies stays empty.
    fn jobSoftBodyPrepare(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        {
            // Reading soft body positions
            var grant: BodyAccess.Grant = .init(.none, .read);
            defer grant.deinit();

            // Get the active soft bodies (Zolt: the copy allocates; once poisoned the list stays empty)
            var active_bodies: BodyIDVector = .empty;
            defer active_bodies.deinit(self.allocator);
            if (!context.isOutOfMemory())
                self.body_manager.getActiveBodies(self.allocator, .soft_body, &active_bodies) catch context.setOutOfMemory();

            if (active_bodies.items.len != 0) {
                // Phase 9: sort the active soft bodies, allocate and initialize the SoftBodyUpdateContexts and (after
                // this scope) create the SoftBodyFinalize / SoftBodySimulate / SoftBodyCollide jobs and return (no soft
                // body can be created before Phase 9)
                @panic("PhysicsSystem.jobSoftBodyPrepare: soft bodies: Phase 9");
            }
        }

        // Quit if there are no active soft bodies
        // (Zolt: the kick happens after the scope of the grant, Jolt kicks inside it, see jobFindCollisions)
        {
            // Kick the next step
            if (step.start_next_step.isValid())
                step.start_next_step.removeDependency(.{});
            return;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // State

    /// Saving state for replay
    /// (Zolt: Allocator.Error from the sorted arrays of the managers)
    pub fn saveState(self: *const Self, stream: *StateRecorder, opts: struct { state: StateRecorderState = .all, filter: ?*const StateRecorderFilter = null }) Allocator.Error!void {
        const state = opts.state;
        stream.write(state);

        if (state.global) {
            stream.write(self.previous_step_delta_time);
            stream.write(self.gravity);
        }

        if (state.bodies)
            try self.body_manager.saveState(stream, opts.filter);

        if (state.contacts)
            try self.contact_manager.saveState(stream, opts.filter);

        if (state.constraints)
            try self.constraint_manager.saveState(stream, opts.filter);
    }

    /// Restoring state for replay. Returns false if failed.
    /// (Zolt: Allocator.Error from the arrays of the body manager and the list of bodies in the broad phase)
    pub fn restoreState(self: *Self, stream: *StateRecorder, opts: struct { filter: ?*const StateRecorderFilter = null }) Allocator.Error!bool {
        var state: StateRecorderState = .all; // Set this value for validation. If a partial state is saved, validation will not work anyway.
        stream.read(&state);

        if (state.global) {
            stream.read(&self.previous_step_delta_time);
            stream.read(&self.gravity);
        }

        if (state.bodies) {
            if (!try self.body_manager.restoreState(stream))
                return false;

            // Update bounding boxes for all bodies in the broadphase
            if (stream.isLastPart()) {
                var bodies: std.ArrayList(BodyID) = .empty;
                defer bodies.deinit(self.allocator);
                for (self.body_manager.getBodies()) |b| {
                    if (BodyManager.isValidBodyPointer(b) and b.getBody().isInBroadPhase())
                        try bodies.append(self.allocator, b.getBody().getID());
                }
                if (bodies.items.len != 0)
                    self.getBroadPhase().notifyBodiesAABBChanged(bodies.items, .{});
            }
        }

        if (state.contacts) {
            if (!self.contact_manager.restoreState(stream, opts.filter))
                return false;
        }

        if (state.constraints) {
            if (!self.constraint_manager.restoreState(stream))
                return false;
        }

        return true;
    }

    /// Saving state of a single body.
    pub fn saveBodyState(self: *const Self, body: *const Body, stream: *StateRecorder) void {
        self.body_manager.saveBodyState(body, stream);
    }

    /// Restoring state of a single body.
    pub fn restoreBodyState(self: *Self, body: *Body, stream: *StateRecorder) void {
        self.body_manager.restoreBodyState(body, stream);

        var id = body.getID();
        self.getBroadPhase().notifyBodiesAABBChanged((&id)[0..1], .{});
    }
};
