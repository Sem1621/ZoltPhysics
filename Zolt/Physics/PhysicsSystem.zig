//! Port of: Jolt/Physics/PhysicsSystem.h, Jolt/Physics/PhysicsSystem.cpp
//! Status: complete
//!
//! Soft bodies (RemainingFeaturesArchitecture.md R17, R18):
//! - JobSoftBodyPrepare, JobSoftBodyCollide, JobSoftBodySimulate and JobSoftBodyFinalize are literal ports at the lambda
//!   positions. Prepare sorts the active soft bodies by BodyID (QuickSort), keeps the contexts in
//!   `PhysicsUpdateContext.soft_body_update_contexts` (a TempArray) and creates Finalize, then N Simulate jobs, then N
//!   Collide jobs (N = GetMaxConcurrency). The Collide jobs start at creation. Jolt's memory orders.
//! - The D5 rows of R17 (SimulationArchitecture D5): an allocation failure in Prepare or in DetermineCollidingShapes is
//!   latched; a soft body job that Prepare cannot create is not an error, Prepare does its work (see jobSoftBodyPrepare).
//!   A poisoned Finalize skips its updates, but always clears the colliding shapes and sensors and frees the contexts.
//! - The response of a CCD body against a soft body face in JobResolveCCDContacts is literal: the face of the sub shape
//!   ID, the vertex masses, the barycentric velocity, the impulses on body 1 and on the 3 vertices (getVertexMut: the
//!   resolve job is serial), MarkCCDContact and RequestContactCallback with a soft body contact listener only.
//!
//! Continuous collision detection (Wave B R4):
//! - JobFindCCDContacts is a literal port: the ShapeCastSettings, the CCDNarrowPhaseCollector (OnContactValidate through
//!   ContactConstraintManager.validateContactPoint, OnSoftBodyContactValidate for a soft body 2, the earliest hit plus
//!   the penetration slop, the correction for the motion of body 2) and the CCDBroadPhaseCollector (the body filters, the
//!   ray against the widened bounds, the narrow phase cast through TransformedShape.castShape with the
//!   SimShapeFilterWrapper, the early out) as local structs with `pub const overrides = .{.addHit}`, then the hit:
//!   ManifoldBetweenTwoFaces, ContactConstraintManager.onCCDContactAdded, sensors and the average contact point.
//! - sSolveCCDContact<Type2> is `solveCCDContact(comptime type2, ...)`, Jolt's dispatch table over the motion type of
//!   body 2 in JobResolveCCDContacts an `inline else` switch. JobResolveCCDContacts sorts the CCD bodies, shortens the
//!   other CCD body, solves the contact and activates the bodies it hit in batches of 64 (JPH_STACK_ALLOC: a fixed
//!   array).
//!
//! Contacts and solving (Wave B R2 / R3):
//! - ProcessBodyPair is a literal port: the contact cache (GetContactsFromCache), AddBodyPair, the narrow phase through
//!   `sim_collide_body_vs_body` with the SimShapeFilterWrapper, the two collectors (with and without manifold
//!   reduction, OnContactValidate through ContactConstraintManager.validateContactPoint) as local structs with
//!   `pub const overrides = .{.addHit}`, PruneContactPoints and AddContactConstraint. It returns Allocator.Error when the
//!   SimCollideBodyVsBody function ran out of heap memory (the InternalEdgeRemovingCollector of the default function):
//!   with manifold reduction the manifolds of the pair are not added, without reduction the contacts added before the
//!   error stay (NonReductionCollideShapeCollector adds every hit at once); either way the FindCollisions job latches
//!   the error (D5).
//! - JobSolveVelocityConstraints solves the islands with constraints or contacts (ConstraintIdxToConstraintOffset, the
//!   two sorts, CalculateSolverSteps, SplitIsland or warm start + velocity steps + StoreAppliedImpulses) and the batches
//!   of the split islands. Nothing in it allocates; its closure checks the out of memory latch at job entry only
//!   (SimulationArchitecture section 8), so a job that entered keeps claiming and splitting islands and the batch
//!   protocol of the LargeIslandSplitter is never left halfway.
//!
//! SimulationArchitecture D4 / D5 / D6 / D7:
//! - `update` builds Jolt's complete job graph per collision step with Jolt's dependency counts, creation order and kick
//!   order. Every C++ lambda is an inline `struct { fn call(...) }.call` closure at the lambda's position whose captures
//!   are the lambda's captures (`[&context, &step]` -> `.{ context, step }`). Jobs created inside jobs
//!   (TrySpawnJobFindCollisions, the FindCCDContacts jobs, the soft body jobs of JobSoftBodyPrepare) are added to the
//!   barrier, as in Jolt.
//! - Start gate (D4): the first step's root jobs get one dependency (Jolt: 0) that `kickStep` (the kick of StartNextStep)
//!   removes after the whole graph exists, so a failing createJob while building has no side effect.
//! - Out of memory (D5): `update` returns `error{OutOfMemory}!PhysicsUpdateError`. A failure while building the jobs (or
//!   in the allocations before) changes nothing. Inside a job an allocation failure sets the latch of the context; a
//!   poisoned job skips its work and allocations but always does its frees, its commits and its dependency bookkeeping,
//!   and the protocol jobs (BroadPhasePrepare, UpdateBroadPhaseFinalize, FindCollisions, PostIntegrateVelocity,
//!   ResolveCCDContacts, ContactRemovedCallbacks, StartNextStep, SoftBodyPrepare, SoftBodyFinalize) follow D5's table. A
//!   job that cannot be created from inside a job is not an error. Temp memory is `TempArray`, freed at Jolt's positions.
//! - Debug grants (BodyAccess.Grant, GrantActiveBodiesAccess) end before a job kicks other jobs: JobFindCollisions and
//!   JobSoftBodyPrepare remove their dependencies after the scope of their grants (Jolt: inside it). With
//!   JobSystemSingleThreaded the kicked jobs run inside the call and their grants assert that no grant is active on the
//!   thread; Jolt hits this from the second collision step on, Zolt with the start gate already in the first step.
//! - The PhysicsSystem is initialized in place (`var system: PhysicsSystem = .empty; try system.init(...)`) because the
//!   lock and body interfaces point into it; it must not move afterwards. `init` takes Jolt's arguments plus
//!   `allocator`, `io` and an options struct: `.broad_phase = .quad_tree` (the default, Jolt's BROAD_PHASE define),
//!   `.brute_force` or `.custom = *BroadPhase` (a Zolt addition: a broad phase that the system owns from a successful
//!   init on).
//! - Const / non-const overloads: `getBodyInterface` / `getBodyInterfaceMut`, `getBodyInterfaceNoLock` /
//!   `getBodyInterfaceNoLockMut`. Listeners are stored by value (pattern B, D7): `?ContactListener`,
//!   `?BodyActivationListener`, `?SoftBodyContactListener`; `AddStepListener` keeps the `PhysicsStepListener` value
//!   (Jolt: the pointer), found again by its `ptr`.
//! - `SimCollideBodyVsBody` (a std::function) is a function pointer + context, `call` passes the allocator that the
//!   InternalEdgeRemovingCollector of the default function needs.
//! - Defaulted parameters are options: `saveState(stream, .{ .state = .all, .filter = null })`,
//!   `restoreState(stream, .{ .filter = null })`. Functions that allocate return Allocator.Error (`getBodies`,
//!   `getActiveBodies`, `getConstraints`, `addConstraint(s)`, `addStepListener`, `saveState`, `restoreState`,
//!   `optimizeBroadPhase`).
//! - JPH_DET_LOG: `if (DeterminismLog.enabled) DeterminismLog.log(.{ ... })` (D11). JPH_DEBUG_RENDERER
//!   (RemainingFeaturesArchitecture.md R2, R4): DrawBodies, DrawConstraints, DrawConstraintLimits and
//!   DrawConstraintReferenceFrame are gated declarations; Jolt's default argument `inBodyFilter` is an options struct.
//!   The renderer that the simulation draws into is `debug_renderer` (Jolt's `DebugRenderer::sInstance`): `void`
//!   without `-Ddebug_renderer`, set it with `setDebugRenderer` between updates. The static flag
//!   sDrawMotionQualityLinearCast is `debug_renderer.flags.physics_system.draw_motion_quality_linear_cast` (the CCD
//!   draws). The contact draws of ProcessBodyPair get the renderer through the debug options of manifoldBetweenTwoFaces
//!   and pruneContactPoints. Without a renderer nothing is drawn (Jolt dereferences a null sInstance when a flag is set).
//!   JPH_TRACK_SIMULATION_STATS (GatherIslandStats, the tick counters) follows MotionProperties'
//!   comptime switch `track_simulation_stats` (off). JPH_TRACK_BROADPHASE_STATS (ReportBroadphaseStats) and
//!   ReportSimulationStats are not ported with the profiler. JPH_PROFILE is dropped.

const std = @import("std");
const builtin = @import("builtin");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const IssueReporting = @import("../Core/IssueReporting.zig");
const DeterminismLog = @import("DeterminismLog.zig");
const math = @import("../Math/Math.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Quat = @import("../Math/Quat.zig").Quat;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const RealFile = @import("../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const AABox = @import("../Geometry/AABox.zig").AABox;
const Plane = @import("../Geometry/Plane.zig").Plane;
const ClosestPoint = @import("../Geometry/ClosestPoint.zig");
const RayAABoxFile = @import("../Geometry/RayAABox.zig");
const Color = @import("../Core/Color.zig").Color;
const DebugRenderer = @import("../Renderer/DebugRenderer.zig").DebugRenderer;
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
const MotionType = @import("Body/MotionType.zig").MotionType;
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
const BroadPhaseQuadTree = @import("Collision/BroadPhase/BroadPhaseQuadTree.zig").BroadPhaseQuadTree;
const BroadPhaseCastResult = @import("Collision/CastResult.zig").BroadPhaseCastResult;
const ShapeCastFile = @import("Collision/ShapeCast.zig");
const RShapeCast = ShapeCastFile.RShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const ShapeFile = @import("Collision/Shape/Shape.zig");
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const SubShapeIDCreator = @import("Collision/Shape/SubShapeID.zig").SubShapeIDCreator;
const CollideShapeFile = @import("Collision/CollideShape.zig");
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const ManifoldBetweenTwoFacesFile = @import("Collision/ManifoldBetweenTwoFaces.zig");
const manifoldBetweenTwoFaces = ManifoldBetweenTwoFacesFile.manifoldBetweenTwoFaces;
const pruneContactPoints = ManifoldBetweenTwoFacesFile.pruneContactPoints;
const CollisionDispatch = @import("Collision/CollisionDispatch.zig");
const InternalEdgeRemovingCollector = @import("Collision/InternalEdgeRemovingCollector.zig").InternalEdgeRemovingCollector;
const ShapeFilter = @import("Collision/ShapeFilter.zig").ShapeFilter;
const SimShapeFilter = @import("Collision/SimShapeFilter.zig").SimShapeFilter;
const SimShapeFilterWrapper = @import("Collision/SimShapeFilterWrapper.zig").SimShapeFilterWrapper;
const ContactListenerFile = @import("Collision/ContactListener.zig");
const ContactListener = ContactListenerFile.ContactListener;
const ContactManifold = ContactListenerFile.ContactManifold;
const ContactSettings = ContactListenerFile.ContactSettings;
const NarrowPhaseQuery = @import("Collision/NarrowPhaseQuery.zig").NarrowPhaseQuery;
const ConstraintFile = @import("Constraints/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintManagerFile = @import("Constraints/ConstraintManager.zig");
const ConstraintManager = ConstraintManagerFile.ConstraintManager;
const Constraints = ConstraintManagerFile.Constraints;
const ContactConstraintManager = @import("Constraints/ContactConstraintManager.zig").ContactConstraintManager;
const SoftBodyContactListenerFile = @import("SoftBody/SoftBodyContactListener.zig");
const SoftBodyContactListener = SoftBodyContactListenerFile.SoftBodyContactListener;
const SoftBodyContactSettings = SoftBodyContactListenerFile.SoftBodyContactSettings;
const SoftBodyMotionProperties = @import("SoftBody/SoftBodyMotionProperties.zig").SoftBodyMotionProperties;
const SoftBodyShape = @import("SoftBody/SoftBodyShape.zig").SoftBodyShape;
const SoftBodyUpdateContext = @import("SoftBody/SoftBodyUpdateContext.zig").SoftBodyUpdateContext;
const ContactConstraintPart = @import("Constraints/ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart;
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
const CalculateSolverStepsFile = @import("Constraints/CalculateSolverSteps.zig");
const CalculateSolverSteps = CalculateSolverStepsFile.CalculateSolverSteps;
const DummyCalculateSolverSteps = CalculateSolverStepsFile.DummyCalculateSolverSteps;

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
        /// BroadPhaseQuadTree (the default, Jolt's BROAD_PHASE define)
        quad_tree,
        /// BroadPhaseBruteForce (a reference implementation for tests)
        brute_force,
        /// A broad phase created by the caller; the system owns it after a successful init (it destroys it in deinit)
        custom: *BroadPhase,
    };

    /// Zolt options of `init`
    pub const InitOptions = struct {
        broad_phase: BroadPhaseOption = .quad_tree,
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

    /// Zolt (RemainingFeaturesArchitecture.md R4): the renderer that the simulation draws into, Jolt's
    /// `DebugRenderer::sInstance` (the CCD draws, the contact point and manifold draws). Only with -Ddebug_renderer
    /// (`void` otherwise). Null draws nothing.
    debug_renderer: if (Core.debug_renderer) ?*DebugRenderer else void = if (Core.debug_renderer) null else {},

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
            .quad_tree => (try BroadPhaseQuadTree.create(allocator)).asBroadPhase(),
            .brute_force => (try BroadPhaseBruteForce.create(allocator)).asBroadPhase(),
            .custom => |custom| custom,
        };
        errdefer if (opts.broad_phase != .custom) broad_phase.base.destroy();
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

    /// Zolt (RemainingFeaturesArchitecture.md R4): set the renderer that the simulation draws into (Jolt's
    /// `DebugRenderer::sInstance`, set by the constructor of a renderer). The draw flags are the fields of
    /// `renderer.flags`. Set the renderer and its flags between updates: the simulation reads them from its job threads.
    /// Only with -Ddebug_renderer (a `void` declaration otherwise).
    pub const setDebugRenderer = if (Core.debug_renderer) setDebugRendererImpl else {};
    fn setDebugRendererImpl(self: *Self, renderer: ?*DebugRenderer) void {
        self.debug_renderer = renderer;
    }

    /// Draw the state of the bodies (debugging purposes) (JPH_DEBUG_RENDERER)
    pub const drawBodies = if (Core.debug_renderer) drawBodiesImpl else {};
    fn drawBodiesImpl(self: *Self, settings: *const BodyManager.DrawSettings, renderer: *DebugRenderer, opts: BodyManager.DrawOptions) void {
        self.body_manager.draw(settings, &self.physics_settings, renderer, opts);
    }

    /// Draw the constraints only (debugging purposes) (JPH_DEBUG_RENDERER)
    pub const drawConstraints = if (Core.debug_renderer) drawConstraintsImpl else {};
    fn drawConstraintsImpl(self: *Self, renderer: *DebugRenderer) void {
        self.constraint_manager.drawConstraints(renderer);
    }

    /// Draw the constraint limits only (debugging purposes) (JPH_DEBUG_RENDERER)
    pub const drawConstraintLimits = if (Core.debug_renderer) drawConstraintLimitsImpl else {};
    fn drawConstraintLimitsImpl(self: *Self, renderer: *DebugRenderer) void {
        self.constraint_manager.drawConstraintLimits(renderer);
    }

    /// Draw the constraint reference frames only (debugging purposes) (JPH_DEBUG_RENDERER)
    pub const drawConstraintReferenceFrame = if (Core.debug_renderer) drawConstraintReferenceFrameImpl else {};
    fn drawConstraintReferenceFrameImpl(self: *Self, renderer: *DebugRenderer) void {
        self.constraint_manager.drawConstraintReferenceFrame(renderer);
    }

    /// The options of manifoldBetweenTwoFaces and pruneContactPoints in the physics update: Jolt's `inCenterOfMass`
    /// and DebugRenderer::sInstance with -Ddebug_renderer, nothing without it
    fn manifoldOptions(self: *const Self, center_of_mass: RVec3) ManifoldBetweenTwoFacesFile.Options {
        if (!Core.debug_renderer) return .{};
        return .{ .debug = .{ .center_of_mass = center_of_mass, .renderer = self.debug_renderer } };
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
    /// (Zolt: Allocator.Error from rebuilding the trees, the broad phase stays valid)
    pub fn optimizeBroadPhase(self: *Self) Allocator.Error!void {
        return self.getBroadPhase().optimize();
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
        if (DeterminismLog.enabled) DeterminismLog.log(.{ "PhysicsSystem::Update: dt: ", delta_time, " steps: ", @as(i32, @bitCast(collision_steps)) }); // inCollisionSteps is an int in Jolt

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

        // Zolt: Jolt resets the accumulated stats on the active bodies here (JPH_TRACK_SIMULATION_STATS), Zolt does it
        // after building the jobs, before the first job starts, so that a failure while building them changes nothing

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

        // Zolt (D5): the jobs reset the InvalidateContactCache flags, an update that runs out of memory puts them back
        self.body_manager.saveContactCacheInvalidation();

        // Build the jobs. Zolt: no job runs before every job exists (the start gate), so a failure has no side effects.
        var constraints_locked = false;
        self.buildJobs(&context, collision_steps, &constraints_locked) catch |err| {
            self.finishUpdate(&context, constraints_locked);
            self.previous_step_delta_time = old_previous_step_delta_time;
            return err;
        };

        // Reset accumulated stats on the active bodies (Zolt: moved here, see above; no job has run yet)
        if (track_simulation_stats)
            self.body_manager.resetSimulationStats();

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

        // Zolt (D5): an allocation failed inside a job: flag the contact caches that were invalid at the start of the
        // update again (the caller restores a saved state, which does not hold the flags)
        if (context.isOutOfMemory())
            self.body_manager.restoreContactCacheInvalidation();

        self.finishUpdate(&context, constraints_locked);

        // Zolt (D5): an allocation failed inside a job
        if (context.isOutOfMemory())
            return error.OutOfMemory;

        // Return any errors
        const errors = PhysicsUpdateError.fromInt(context.errors.load(.acquire));
        if (Core.enable_asserts) IssueReporting.assertRecoverable(errors.isNone(), "An error occurred during the physics update, see EPhysicsUpdateError for more information", @src());
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

        // Clear the large island splitter (Zolt D5: after an out of memory error not every large island was split)
        if (context.isOutOfMemory())
            self.large_island_splitter.resetAfterOutOfMemory(temp_allocator)
        else
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

                        // Clear the large island splitter (Zolt D5: after an out of memory error not every large
                        // island was split)
                        const temp_allocator_ns = c.temp_allocator;
                        if (c.isOutOfMemory())
                            ps.large_island_splitter.resetAfterOutOfMemory(temp_allocator_ns)
                        else
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

                        // Find pairs in the broadphase (Zolt D5: the broad phase ran out of heap memory, latched)
                        self.getBroadPhase().findCollidingPairs(active_bodies[0..batch_size], self.physics_settings.speculative_contact_distance, self.object_vs_broad_phase_layer_filter, self.object_layer_pair_filter, &add_pair.base) catch context.setOutOfMemory();

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
    /// (Zolt: Allocator.Error when the SimCollideBodyVsBody function ran out of heap memory (the default one in the
    /// InternalEdgeRemovingCollector); the contacts of the pair are then incomplete, the caller latches the error, D5)
    fn processBodyPair(self: *Self, contact_allocator: *ContactAllocator, body_pair: BodyPair) Allocator.Error!void {
        // Fetch body pair
        var body1 = self.body_manager.getBodyMut(body_pair.body_a);
        var body2 = self.body_manager.getBodyMut(body_pair.body_b);
        std.debug.assert(body1.isActive());

        if (DeterminismLog.enabled) DeterminismLog.log(.{ "ProcessBodyPair: id1: ", body_pair.body_a, " id2: ", body_pair.body_b, " p1: ", body1.getCenterOfMassPosition(), " p2: ", body2.getCenterOfMassPosition(), " r1: ", body1.getRotation(), " r2: ", body2.getRotation() });

        // Validates that a body that is sleeping has zero velocity.
        if (Core.enable_asserts) body2.validateMotion();

        // Check for soft bodies
        if (body2.isSoftBody()) {
            // If the 2nd body is a soft body and not active, we activate it now
            if (!body2.isActive())
                self.body_manager.activateBodies((&body_pair.body_b)[0..1]);

            // Soft body processing is done later in the pipeline
            return;
        }

        // Ensure that body1 has the higher motion type (i.e. dynamic trumps kinematic), this ensures that we do the collision detection in the space of a moving body,
        // which avoids accuracy problems when testing a very large static object against a small dynamic object
        // Ensure that body1 id < body2 id when motion types are the same.
        if (@intFromEnum(body1.getMotionType()) < @intFromEnum(body2.getMotionType()) or
            (body1.getMotionType() == body2.getMotionType() and body_pair.body_b.lessThan(body_pair.body_a)))
            std.mem.swap(*Body, &body1, &body2);

        // Check if the contact points from the previous frame are reusable and if so copy them
        var pair_handled = false;
        if (self.physics_settings.use_body_pair_contact_cache and !(body1.isCollisionCacheInvalid() or body2.isCollisionCacheInvalid()))
            pair_handled = self.contact_manager.getContactsFromCache(contact_allocator, body1, body2);

        // If the cache hasn't handled this body pair do actual collision detection
        if (!pair_handled) {
            const start_ticks = if (track_simulation_stats) getProcessorTickCount() else {};

            // Create entry in the cache for this body pair
            // Needs to happen irrespective if we found a collision or not (we want to remember that no collision was found too)
            const body_pair_handle = self.contact_manager.addBodyPair(contact_allocator, body1, body2);
            if (body_pair_handle == null)
                return; // Out of cache space

            // Create the query settings
            var settings: CollideShapeSettings = .{};
            settings.collect_faces_mode = .collect_faces;
            settings.active_edge_mode = if (self.physics_settings.check_active_edges) .collide_only_with_active else .collide_with_all;
            settings.max_separation_distance = if (body1.isSensor() or body2.isSensor()) 0.0 else self.physics_settings.speculative_contact_distance;
            settings.active_edge_movement_direction = body1.getLinearVelocity().sub(body2.getLinearVelocity());
            settings.internal_edge_removal_vertex_tolerance_sq = self.physics_settings.internal_edge_removal_vertex_tolerance_sq;

            // Create shape filter
            var shape_filter: SimShapeFilterWrapper = .init(self.sim_shape_filter, body1);
            shape_filter.setBody2(body2);

            // Get transforms relative to body1
            const offset = body1.getCenterOfMassPosition();
            const transform1 = Mat44.rotationQuat(body1.getRotation());
            const transform2 = body2.getCenterOfMassTransform().postTranslatedRVec3(offset.negate()).toMat44();

            if (self.physics_settings.use_manifold_reduction // Check global flag
            and body1.getUseManifoldReductionWithBody(body2)) // Check body flag
            {
                // Version WITH contact manifold reduction

                const MyManifold = struct {
                    base: ContactManifold,
                    first_world_space_normal: Vec3,
                };

                // A temporary structure that allows us to keep track of the all manifolds between this body pair
                const Manifolds = StaticArray(MyManifold, 32);

                // Create collector
                const ReductionCollideShapeCollector = struct {
                    pub const overrides = .{.addHit};

                    base: CollideShapeCollector = .init(@This()),
                    system: *const Self,
                    body1: *const Body,
                    body2: *const Body,
                    validate_body_pair: bool = true,
                    manifolds: Manifolds = .empty,

                    pub fn addHit(collector: *@This(), result: *const CollideShapeResult) void {
                        // The first body should be the one with the highest motion type
                        std.debug.assert(@intFromEnum(collector.body1.getMotionType()) >= @intFromEnum(collector.body2.getMotionType()));
                        std.debug.assert(!collector.base.shouldEarlyOut());

                        // Test if we want to accept this hit
                        if (collector.validate_body_pair) {
                            switch (collector.system.contact_manager.validateContactPoint(collector.body1, collector.body2, collector.body1.getCenterOfMassPosition(), result)) {
                                .accept_contact => {
                                    // We're just accepting this one, nothing to do
                                },

                                .accept_all_contacts_for_this_body_pair => {
                                    // Accept and stop calling the validate callback
                                    collector.validate_body_pair = false;
                                },

                                .reject_contact => {
                                    // Skip this contact
                                    return;
                                },

                                .reject_all_contacts_for_this_body_pair => {
                                    // Skip this and early out
                                    collector.base.forceEarlyOut();
                                    return;
                                },
                            }
                        }

                        // Calculate normal
                        const world_space_normal = result.penetration_axis.normalized();

                        // Check if we can add it to an existing manifold
                        const contact_normal_cos_max_delta_rot = collector.system.physics_settings.contact_normal_cos_max_delta_rotation;
                        var manifold_index: u32 = 0;
                        while (manifold_index < collector.manifolds.len) : (manifold_index += 1) {
                            const m = collector.manifolds.at(manifold_index);
                            if (world_space_normal.dot(m.first_world_space_normal) >= contact_normal_cos_max_delta_rot) {
                                // Update average normal
                                m.base.world_space_normal = m.base.world_space_normal.add(world_space_normal);
                                m.base.penetration_depth = math.max(m.base.penetration_depth, result.penetration_depth);
                                break;
                            }
                        }
                        if (manifold_index == collector.manifolds.len) {
                            // Check if array is full
                            if (collector.manifolds.len == Manifolds.capacity) {
                                // Full, find manifold with least amount of penetration
                                manifold_index = 0;
                                var m: u32 = 1;
                                while (m < collector.manifolds.len) : (m += 1) {
                                    if (collector.manifolds.get(m).base.penetration_depth < collector.manifolds.get(manifold_index).base.penetration_depth)
                                        manifold_index = m;
                                }

                                // If this contacts penetration is smaller than the smallest manifold, we skip this contact
                                if (result.penetration_depth < collector.manifolds.get(manifold_index).base.penetration_depth)
                                    return;

                                // Replace the manifold
                                collector.manifolds.at(manifold_index).* = .{ .base = .{ .base_offset = collector.body1.getCenterOfMassPosition(), .world_space_normal = world_space_normal, .penetration_depth = result.penetration_depth, .sub_shape_id1 = result.sub_shape_id1, .sub_shape_id2 = result.sub_shape_id2 }, .first_world_space_normal = world_space_normal };
                            } else {
                                // Not full, create new manifold
                                collector.manifolds.append(.{ .base = .{ .base_offset = collector.body1.getCenterOfMassPosition(), .world_space_normal = world_space_normal, .penetration_depth = result.penetration_depth, .sub_shape_id1 = result.sub_shape_id1, .sub_shape_id2 = result.sub_shape_id2 }, .first_world_space_normal = world_space_normal });
                                manifold_index = collector.manifolds.len - 1;
                            }
                        }
                        const manifold = collector.manifolds.at(manifold_index);

                        // Determine contact points
                        const settings_ps = &collector.system.physics_settings;
                        manifoldBetweenTwoFaces(result.contact_point_on1, result.contact_point_on2, result.penetration_axis, settings_ps.speculative_contact_distance + settings_ps.manifold_tolerance, &result.shape1_face, &result.shape2_face, &manifold.base.relative_contact_points_on1, &manifold.base.relative_contact_points_on2, collector.system.manifoldOptions(collector.body1.getCenterOfMassPosition()));

                        // Prune if we have more than 32 points (this means we could run out of space in the next iteration)
                        if (manifold.base.relative_contact_points_on1.len > 32)
                            pruneContactPoints(manifold.first_world_space_normal, &manifold.base.relative_contact_points_on1, &manifold.base.relative_contact_points_on2, collector.system.manifoldOptions(manifold.base.base_offset));
                    }
                };
                var collector: ReductionCollideShapeCollector = .{ .system = self, .body1 = body1, .body2 = body2 };

                // Perform collision detection between the two shapes
                try self.sim_collide_body_vs_body.call(self.allocator, body1, body2, transform1, transform2, &settings, &collector.base, shape_filter.getFilter());

                // Add the contacts
                var link_and_activate_bodies = true;
                for (collector.manifolds.slice()) |*my_manifold| {
                    const manifold = &my_manifold.base;

                    // Normalize the normal (is a sum of all normals from merged manifolds)
                    manifold.world_space_normal = manifold.world_space_normal.normalized();

                    // If we still have too many points, prune them now
                    if (manifold.relative_contact_points_on1.len > 4)
                        pruneContactPoints(manifold.world_space_normal, &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2, self.manifoldOptions(manifold.base_offset));

                    // Actually add the contact points to the manager
                    self.contact_manager.addContactConstraint(contact_allocator, &link_and_activate_bodies, body_pair_handle, body1, body2, manifold);
                }
            } else {
                // Version WITHOUT contact manifold reduction

                // Create collector
                const NonReductionCollideShapeCollector = struct {
                    pub const overrides = .{.addHit};

                    base: CollideShapeCollector = .init(@This()),
                    system: *Self,
                    contact_allocator: *ContactAllocator,
                    body1: *Body,
                    body2: *Body,
                    body_pair_handle: ContactConstraintManager.BodyPairHandle,
                    validate_body_pair: bool = true,
                    link_and_activate_bodies: bool = true,

                    pub fn addHit(collector: *@This(), result: *const CollideShapeResult) void {
                        // The first body should be the one with the highest motion type
                        std.debug.assert(@intFromEnum(collector.body1.getMotionType()) >= @intFromEnum(collector.body2.getMotionType()));
                        std.debug.assert(!collector.base.shouldEarlyOut());

                        // Test if we want to accept this hit
                        if (collector.validate_body_pair) {
                            switch (collector.system.contact_manager.validateContactPoint(collector.body1, collector.body2, collector.body1.getCenterOfMassPosition(), result)) {
                                .accept_contact => {
                                    // We're just accepting this one, nothing to do
                                },

                                .accept_all_contacts_for_this_body_pair => {
                                    // Accept and stop calling the validate callback
                                    collector.validate_body_pair = false;
                                },

                                .reject_contact => {
                                    // Skip this contact
                                    return;
                                },

                                .reject_all_contacts_for_this_body_pair => {
                                    // Skip this and early out
                                    collector.base.forceEarlyOut();
                                    return;
                                },
                            }
                        }

                        // Determine contact points
                        var manifold: ContactManifold = .{};
                        manifold.base_offset = collector.body1.getCenterOfMassPosition();
                        const settings_ps = &collector.system.physics_settings;
                        manifoldBetweenTwoFaces(result.contact_point_on1, result.contact_point_on2, result.penetration_axis, settings_ps.speculative_contact_distance + settings_ps.manifold_tolerance, &result.shape1_face, &result.shape2_face, &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2, collector.system.manifoldOptions(manifold.base_offset));

                        // Calculate normal
                        manifold.world_space_normal = result.penetration_axis.normalized();

                        // Store penetration depth
                        manifold.penetration_depth = result.penetration_depth;

                        // Prune if we have more than 4 points
                        if (manifold.relative_contact_points_on1.len > 4)
                            pruneContactPoints(manifold.world_space_normal, &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2, collector.system.manifoldOptions(manifold.base_offset));

                        // Set other properties
                        manifold.sub_shape_id1 = result.sub_shape_id1;
                        manifold.sub_shape_id2 = result.sub_shape_id2;

                        // Actually add the contact points to the manager
                        collector.system.contact_manager.addContactConstraint(collector.contact_allocator, &collector.link_and_activate_bodies, collector.body_pair_handle, collector.body1, collector.body2, &manifold);
                    }
                };
                var collector: NonReductionCollideShapeCollector = .{ .system = self, .contact_allocator = contact_allocator, .body1 = body1, .body2 = body2, .body_pair_handle = body_pair_handle };

                // Perform collision detection between the two shapes
                try self.sim_collide_body_vs_body.call(self.allocator, body1, body2, transform1, transform2, &settings, &collector.base, shape_filter.getFilter());
            }

            if (track_simulation_stats) {
                // Track time spent processing collision for this body pair
                var num_ticks = getProcessorTickCount() -% start_ticks;
                if (@intFromEnum(body1.getMotionType()) > @intFromEnum(body2.getMotionType())) {
                    // Assign all ticks to the body with the higher motion type
                    _ = body1.getMotionPropertiesMut().getSimulationStatsMut().narrow_phase_ticks.fetchAdd(num_ticks, .monotonic);
                } else {
                    // When two bodies with the same motion type are involved, we give both bodies half the ticks
                    std.debug.assert(body1.getMotionType() == body2.getMotionType());
                    num_ticks /= 2;
                    _ = body1.getMotionPropertiesMut().getSimulationStatsMut().narrow_phase_ticks.fetchAdd(num_ticks, .monotonic);
                    _ = body2.getMotionPropertiesMut().getSimulationStatsMut().narrow_phase_ticks.fetchAdd(num_ticks, .monotonic);
                }
            }
        }
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

    /// (Zolt D5 and SimulationArchitecture section 8: the closure checks the out of memory latch at job entry only. A job
    /// that has entered claims and splits islands until none are left even when the update got poisoned meanwhile,
    /// otherwise `next_split_island` never reaches the number of large islands and the jobs in the batch loop wait
    /// forever. Nothing in here allocates.)
    fn jobSolveVelocityConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        // We update velocities and need to read positions to do so
        var grant: BodyAccess.Grant = .init(.read_write, .read);
        defer grant.deinit();

        const delta_time = context.step_delta_time;
        const active_constraints = context.active_constraints.items;

        // Only the first step to correct for the delta time difference in the previous update
        const warm_start_impulse_ratio: f32 = if (self.physics_settings.constraint_warm_start) (if (step.is_first) context.warm_start_impulse_ratio else 1.0) else 0.0;

        var check_islands = true;
        var check_split_islands = self.physics_settings.use_large_island_splitter;
        while (true) {
            // First try to get work from large islands
            if (check_split_islands) {
                var batch: LargeIslandSplitter.Batch = undefined;
                switch (self.large_island_splitter.fetchNextBatch(&batch)) {
                    .batch_retrieved => {
                        const start_tick = if (track_simulation_stats) getProcessorTickCount() else {};

                        if (batch.first_iteration) {
                            // Iteration 0 is used to warm start the batch (we added 1 to the number of iterations in LargeIslandSplitter::SplitIsland)
                            var dummy: DummyCalculateSolverSteps = .{};
                            ConstraintManager.warmStartVelocityConstraints(active_constraints, batch.constraints, warm_start_impulse_ratio, &dummy);
                            self.contact_manager.warmStartVelocityConstraints(batch.contacts, warm_start_impulse_ratio, &dummy);
                        } else {
                            // Solve velocity constraints
                            _ = ConstraintManager.solveVelocityConstraints(active_constraints, batch.constraints, delta_time);
                            _ = self.contact_manager.solveVelocityConstraints(batch.contacts);
                        }

                        // Mark the batch as processed
                        var last_iteration: bool = undefined;
                        var final_batch: bool = undefined;
                        self.large_island_splitter.markBatchProcessed(batch.split_island_index, batch.constraints, batch.contacts, &last_iteration, &final_batch);

                        // Save back the lambdas in the contact cache for the warm start of the next physics update
                        if (last_iteration)
                            self.contact_manager.storeAppliedImpulses(batch.contacts);

                        if (track_simulation_stats) {
                            const num_ticks = getProcessorTickCount() -% start_tick;
                            _ = self.island_builder.getIslandStats(self.large_island_splitter.getIslandIndex(batch.split_island_index)).velocity_constraint_ticks.fetchAdd(num_ticks, .monotonic);
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

                const start_tick = if (track_simulation_stats) getProcessorTickCount() else {};

                // Convert indices to offsets so that we lose one indirection in the solver. We couldn't do this earlier as we need constraint indices to build islands.
                self.contact_manager.constraintIdxToConstraintOffset(contacts);

                // Sort constraints to give a deterministic simulation
                ConstraintManager.sortConstraints(active_constraints, constraints);

                // Sort contacts to give a deterministic simulation
                self.contact_manager.sortContacts(contacts);

                // Split up large islands
                var is_large_island = true;
                var steps_calculator: CalculateSolverSteps = .init(&self.physics_settings);
                if (!self.physics_settings.use_large_island_splitter or
                    !self.large_island_splitter.splitIsland(island_idx, &self.island_builder, &self.body_manager, &self.contact_manager, active_constraints, &steps_calculator))
                {
                    is_large_island = false;

                    // We didn't create a split, just run the solver now for this entire island. Begin by warm starting.
                    ConstraintManager.warmStartVelocityConstraints(active_constraints, constraints, warm_start_impulse_ratio, &steps_calculator);
                    self.contact_manager.warmStartVelocityConstraints(contacts, warm_start_impulse_ratio, &steps_calculator);
                    steps_calculator.finalize();

                    // Store the number of position steps for later
                    self.island_builder.setNumPositionSteps(island_idx, steps_calculator.getNumPositionSteps());

                    // Solve velocity constraints
                    var velocity_step: u32 = 0;
                    while (velocity_step < steps_calculator.getNumVelocitySteps()) : (velocity_step += 1) {
                        var applied_impulse = ConstraintManager.solveVelocityConstraints(active_constraints, constraints, delta_time);
                        applied_impulse = self.contact_manager.solveVelocityConstraints(contacts) or applied_impulse; // Jolt: applied_impulse |= ..., the contacts are always solved
                        if (!applied_impulse)
                            break;
                    }

                    // Save back the lambdas in the contact cache for the warm start of the next physics update
                    self.contact_manager.storeAppliedImpulses(contacts);
                }

                if (track_simulation_stats) {
                    const num_ticks = getProcessorTickCount() -% start_tick;
                    const stats = self.island_builder.getIslandStats(island_idx);
                    stats.num_velocity_steps = @truncate(steps_calculator.getNumVelocitySteps());
                    stats.num_position_steps = @truncate(steps_calculator.getNumPositionSteps());
                    _ = stats.velocity_constraint_ticks.fetchAdd(num_ticks, .monotonic);
                    stats.is_large_island = is_large_island;
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

                if (DeterminismLog.enabled) DeterminismLog.log(.{ "JobIntegrateVelocity: id: ", body_id, " v: ", body.getLinearVelocity(), " w: ", body.getAngularVelocity() });

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
                            // (Zolt: a mesh / height field / mirrored ScaledShape reaches this with a zero or negative inner radius, Jolt's release build continues: no UB in ReleaseFast)
                            if (Core.enable_asserts) Core.assertRecoverable(inner_radius > 0.0, "The shape has no inner radius, this makes the shape unsuitable for the linear cast motion quality as we cannot move it without risking tunneling.", @src());

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

    /// (Zolt D5: castAABoxNoLock returns Allocator.Error when a node stack of the QuadTree cannot grow on the heap; the job
    /// then latches the error and takes no further bodies, ResolveCCDContacts skips its work)
    fn jobFindCCDContacts(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        // We only read positions, but the validate callback may read body positions and velocities
        var grant: BodyAccess.Grant = .init(.read, .read);
        defer grant.deinit();

        // Allocation context for allocating new contact points
        var contact_allocator = self.contact_manager.getContactAllocator();

        // Settings
        var settings: ShapeCastSettings = .{};
        settings.use_shrunken_shape_and_convex_radius = true;
        settings.back_face_mode_triangles = .ignore_back_faces;
        settings.back_face_mode_convex = .ignore_back_faces;
        settings.return_deepest_point = true;
        settings.collect_faces_mode = .collect_faces;
        settings.active_edge_mode = if (self.physics_settings.check_active_edges) .collide_only_with_active else .collide_with_all;

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

            // Draw start and end shape of cast (JPH_DEBUG_RENDERER)
            if (Core.debug_renderer) {
                if (self.debug_renderer) |renderer| {
                    if (renderer.flags.physics_system.draw_motion_quality_linear_cast) {
                        const com = body.getCenterOfMassTransform();
                        body.getShape().draw(renderer, com, Vec3.one(), Color.green, false, true);
                        renderer.drawArrow(com.getTranslation(), com.getTranslation().addVec3(ccd_body.delta_position), Color.green, 0.1);
                        body.getShape().draw(renderer, com.postTranslated(ccd_body.delta_position), Vec3.one(), Color.red, false, true);
                    }
                }
            }

            // Create a collector that will find the maximum distance allowed to travel while not penetrating more than 'max penetration'
            const CCDNarrowPhaseCollector = struct {
                pub const overrides = .{.addHit};

                base: CastShapeCollector = .init(@This()),
                /// If we still have to call the ValidateContactPoint for this body pair
                validate_body_pair: bool = true,
                /// Reject all further contacts between this body pair
                reject_all: bool = false,
                body_manager: *const BodyManager,
                contact_constraint_manager: *const ContactConstraintManager,
                soft_body_contact_listener: ?SoftBodyContactListener,
                ccd_body: *CCDBody,
                result: *ShapeCastResult,
                delta_time: f32,

                pub fn addHit(collector: *@This(), result: *const ShapeCastResult) void {
                    // Check if this is a possible earlier hit than the one before
                    const fraction = result.fraction;
                    if (fraction < collector.ccd_body.fraction_plus_slop) {
                        // Normalize normal
                        const normal = result.base.penetration_axis.normalized();

                        // Calculate how much we can add to the fraction to penetrate the collision point by mMaxPenetration.
                        // Note that the normal is pointing towards body 2!
                        // Let the extra distance that we can travel along delta_pos be 'dist': mMaxPenetration / dist = cos(angle between normal and delta_pos) = normal . delta_pos / |delta_pos|
                        // <=> dist = mMaxPenetration * |delta_pos| / normal . delta_pos
                        // Converting to a faction: delta_fraction = dist / |delta_pos| = mLinearCastTreshold / normal . delta_pos
                        const denominator = normal.dot(collector.ccd_body.delta_position);
                        if (denominator > collector.ccd_body.max_penetration) // Avoid dividing by zero, if extra hit fraction > 1 there's also no point in continuing
                        {
                            const fraction_plus_slop = fraction + collector.ccd_body.max_penetration / denominator;
                            if (fraction_plus_slop < collector.ccd_body.fraction_plus_slop) {
                                const body2 = collector.body_manager.getBody(result.base.body_id2);

                                // Check if we've already accepted all hits from this body
                                if (collector.validate_body_pair) {
                                    // Validate the contact result
                                    const body1 = collector.body_manager.getBody(collector.ccd_body.body_id1);
                                    if (body2.isRigidBody()) {
                                        const validate_result = collector.contact_constraint_manager.validateContactPoint(body1, body2, body1.getCenterOfMassPosition(), &result.base); // Note that the center of mass of body 1 is the start of the sweep and is used as base offset below
                                        switch (validate_result) {
                                            .accept_contact => {
                                                // Just continue
                                            },

                                            .accept_all_contacts_for_this_body_pair => {
                                                // Accept this and all following contacts from this body
                                                collector.validate_body_pair = false;
                                            },

                                            .reject_contact => return,

                                            .reject_all_contacts_for_this_body_pair => {
                                                // Reject this and all following contacts from this body
                                                collector.reject_all = true;
                                                collector.base.forceEarlyOut();
                                                return;
                                            },
                                        }
                                    } else {
                                        var sb_settings: SoftBodyContactSettings = .{};
                                        sb_settings.is_sensor = false;
                                        if ((collector.soft_body_contact_listener == null or
                                            collector.soft_body_contact_listener.?.onSoftBodyContactValidate(body2, body1, &sb_settings) == .accept_contact) // Note reversal, soft body needs to be first parameter
                                        and !sb_settings.is_sensor) // If the contact listener turned this into a sensor, we want to ignore it
                                        {
                                            // Convert the soft body contact settings (note bodies are swapped)
                                            collector.ccd_body.contact_settings.inv_mass_scale1 = sb_settings.inv_mass_scale2;
                                            collector.ccd_body.contact_settings.inv_mass_scale2 = sb_settings.inv_mass_scale1;
                                            collector.ccd_body.contact_settings.inv_inertia_scale1 = sb_settings.inv_inertia_scale2;

                                            // Accept this and all following contacts from this body
                                            collector.validate_body_pair = false;
                                        } else {
                                            // Reject this and all following contacts from this body
                                            collector.reject_all = true;
                                            collector.base.forceEarlyOut();
                                            return;
                                        }
                                    }
                                }

                                // This is the earliest hit so far, store it
                                collector.ccd_body.contact_normal = normal;
                                collector.ccd_body.body_id2 = result.base.body_id2;
                                collector.ccd_body.sub_shape_id2 = result.base.sub_shape_id2;
                                collector.ccd_body.fraction = fraction;
                                collector.ccd_body.fraction_plus_slop = fraction_plus_slop;
                                collector.result.* = result.*;

                                // Result was assuming body 2 is not moving, but it is, so we need to correct for it
                                const movement2 = calculateBodyMotion(body2, collector.delta_time).mulScalar(fraction);
                                if (!movement2.isNearZero(.{})) {
                                    collector.result.base.contact_point_on1 = collector.result.base.contact_point_on1.add(movement2);
                                    collector.result.base.contact_point_on2 = collector.result.base.contact_point_on2.add(movement2);
                                    for (collector.result.base.shape1_face.slice()) |*v|
                                        v.* = v.add(movement2);
                                    for (collector.result.base.shape2_face.slice()) |*v|
                                        v.* = v.add(movement2);
                                }

                                // Update early out fraction
                                collector.base.updateEarlyOutFraction(fraction_plus_slop);
                            }
                        }
                    }
                }
            };

            // Narrowphase collector
            var cast_shape_result: ShapeCastResult = .{};
            var np_collector: CCDNarrowPhaseCollector = .{
                .body_manager = &self.body_manager,
                .contact_constraint_manager = &self.contact_manager,
                .soft_body_contact_listener = self.soft_body_contact_listener,
                .ccd_body = ccd_body,
                .result = &cast_shape_result,
                .delta_time = context.step_delta_time,
            };

            // This collector wraps the narrowphase collector and collects the closest hit
            const CCDBroadPhaseCollector = struct {
                pub const overrides = .{.addHit};

                base: CastShapeBodyCollector = .init(@This()),
                ccd_body: *const CCDBody,
                body1: *const Body,
                body1_extent: Vec3,
                shape_cast: RShapeCast,
                shape_cast_settings: *ShapeCastSettings,
                shape_filter: *SimShapeFilterWrapper,
                collector: *CCDNarrowPhaseCollector,
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

                    // Reset collector (this is a new body pair)
                    collector.collector.base.resetEarlyOutFraction(.{ .fraction = collector.base.getEarlyOutFraction() });
                    collector.collector.validate_body_pair = true;
                    collector.collector.reject_all = false;

                    // Set body ID on shape filter
                    collector.shape_filter.setBody2(body2);

                    // Provide direction as hint for the active edges algorithm
                    collector.shape_cast_settings.active_edge_movement_direction = direction;

                    // Do narrow phase collision check
                    const relative_cast: RShapeCast = .initWithBounds(collector.shape_cast.shape, collector.shape_cast.scale, collector.shape_cast.center_of_mass_start, direction, collector.shape_cast.shape_world_bounds);
                    var transformed_shape = body2.getTransformedShape();
                    defer transformed_shape.deinit();
                    transformed_shape.castShape(&relative_cast, collector.shape_cast_settings, collector.shape_cast.center_of_mass_start.getTranslation(), &collector.collector.base, .{ .shape_filter = collector.shape_filter.getFilterMut() });

                    // Update early out fraction based on narrow phase collector
                    if (!collector.collector.reject_all)
                        collector.base.updateEarlyOutFraction(collector.collector.base.getEarlyOutFraction());
                }
            };

            // Create shape filter
            var shape_filter: SimShapeFilterWrapper = .init(self.sim_shape_filter, body);

            const start_tick = if (track_simulation_stats) getProcessorTickCount() else {};

            // Check if we collide with any other body. Note that we use the non-locking interface as we know the broadphase cannot be modified at this point.
            const shape_cast: RShapeCast = .init(body.getShape(), Vec3.one(), body.getCenterOfMassTransform(), ccd_body.delta_position);
            var bp_collector: CCDBroadPhaseCollector = .{
                .ccd_body = ccd_body,
                .body1 = body,
                .body1_extent = shape_cast.shape_world_bounds.getExtent(),
                .shape_cast = shape_cast,
                .shape_cast_settings = &settings,
                .shape_filter = &shape_filter,
                .collector = &np_collector,
                .body_manager = &self.body_manager,
                .step = step,
                .delta_time = context.step_delta_time,
            };
            self.getBroadPhase().castAABoxNoLock(&.{ .box = shape_cast.shape_world_bounds, .direction = shape_cast.direction }, &bp_collector.base, &broadphase_layer_filter.base, &object_layer_filter.base) catch {
                // Zolt (D5): the broad phase ran out of heap memory, latched. The hit of this body is incomplete, so
                // this job takes no further bodies (ResolveCCDContacts skips its work)
                context.setOutOfMemory();
                break;
            };

            if (track_simulation_stats) {
                const num_ticks = getProcessorTickCount() -% start_tick;
                // (Jolt: a const_cast of the stats of the const body, Zolt: the mutable body)
                _ = self.body_manager.getBodyMut(ccd_body.body_id1).getMotionPropertiesMut().getSimulationStatsMut().ccd_ticks.fetchAdd(num_ticks, .monotonic);
            }

            // Check if there was a hit
            if (ccd_body.fraction_plus_slop < 1.0) {
                const body2 = self.body_manager.getBody(ccd_body.body_id2);

                // Determine contact manifold
                var manifold: ContactManifold = .{};
                manifold.base_offset = shape_cast.center_of_mass_start.getTranslation();
                manifoldBetweenTwoFaces(cast_shape_result.base.contact_point_on1, cast_shape_result.base.contact_point_on2, cast_shape_result.base.penetration_axis, self.physics_settings.manifold_tolerance, &cast_shape_result.base.shape1_face, &cast_shape_result.base.shape2_face, &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2, self.manifoldOptions(manifold.base_offset));
                manifold.sub_shape_id1 = cast_shape_result.base.sub_shape_id1;
                manifold.sub_shape_id2 = cast_shape_result.base.sub_shape_id2;
                manifold.penetration_depth = cast_shape_result.base.penetration_depth;
                manifold.world_space_normal = ccd_body.contact_normal;

                // Call contact point callbacks
                if (body2.isRigidBody())
                    self.contact_manager.onCCDContactAdded(&contact_allocator, body, body2, &manifold, &ccd_body.contact_settings)
                else {
                    // We already have mass and inertia scale from the OnSoftBodyContactValidate callback, but we need to fill in the rest of the contact settings
                    ccd_body.contact_settings.combined_friction = 0.0; // Soft bodies CCD contacts don't apply friction
                    ccd_body.contact_settings.combined_restitution = self.contact_manager.getCombineRestitution()(body, .empty, body2, .empty); // Soft bodies don't pass sub shape IDs because the restitution is uniform across the entire body
                    ccd_body.contact_settings.is_sensor = false; // We've filtered out sensors already
                }

                if (ccd_body.contact_settings.is_sensor) {
                    // If this is a sensor, we don't want to solve the contact
                    ccd_body.fraction_plus_slop = 1.0;
                    ccd_body.body_id2 = .invalid;
                } else {
                    // Calculate the average position from the manifold (this will result in the same impulse applied as when we apply impulses to all contact points)
                    if (manifold.relative_contact_points_on2.len > 1) {
                        var average_contact_point = Vec3.zero();
                        for (manifold.relative_contact_points_on2.constSlice()) |v|
                            average_contact_point = average_contact_point.add(v);
                        average_contact_point = average_contact_point.divScalar(@floatFromInt(manifold.relative_contact_points_on2.len));
                        ccd_body.contact_point_on2 = manifold.base_offset.addVec3(average_contact_point);
                    } else ccd_body.contact_point_on2 = manifold.base_offset.addVec3(cast_shape_result.base.contact_point_on2);
                }
            }
        }

        // Collect information from the contact allocator and accumulate it in the step.
        finalizeContactAllocator(step, &contact_allocator);
    }

    /// Solve a CCD contact against rigid body 2 (Jolt's template sSolveCCDContact<EMotionType Type2>, the dispatch table in
    /// JobResolveCCDContacts is an `inline else` switch over the motion type of body 2)
    fn solveCCDContact(comptime type2: MotionType, body1: *Body, inv_m1: f32, inv_i1: Mat44, r1_plus_u: Vec3, body2: *Body, r2: Vec3, contact_normal: Vec3, normal_velocity_bias: f32, friction_direction_in: Vec3, contact_settings: *const ContactSettings) void {
        const body_mp = body1.getMotionPropertiesMut();

        // Get inverse mass of body 2
        var inv_m2: f32 = undefined;
        var inv_i2: Mat44 = undefined;
        if (comptime type2 == .dynamic) {
            inv_m2 = contact_settings.inv_mass_scale2 * body2.getMotionPropertiesUnchecked().?.getInverseMassUnchecked();
            inv_i2 = body2.getInverseInertia().mulScalar(contact_settings.inv_inertia_scale2);
        } else {
            inv_m2 = 0.0;
            inv_i2 = Mat44.zero();
        }

        // Get velocities
        var linear_velocity1 = body_mp.getLinearVelocity();
        var angular_velocity1 = body_mp.getAngularVelocity();
        var linear_velocity2: Vec3 = undefined;
        var angular_velocity2: Vec3 = undefined;
        if (comptime type2 != .static) {
            const body2_mp = body2.getMotionPropertiesUnchecked().?;
            linear_velocity2 = body2_mp.getLinearVelocity();
            angular_velocity2 = body2_mp.getAngularVelocity();
        } else {
            linear_velocity2 = Vec3.zero();
            angular_velocity2 = Vec3.zero();
        }

        // Solve contact constraint
        var contact_constraint: ContactConstraintPart(.dynamic, .dynamic) = .{};
        contact_constraint.setTotalLambda(0.0);
        contact_constraint.calculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, contact_normal, .{ .bias = normal_velocity_bias });
        _ = contact_constraint.solveVelocityConstraint(&linear_velocity1, &angular_velocity1, &linear_velocity2, &angular_velocity2, inv_m1, inv_m2, contact_normal, -math.flt_max, math.flt_max);

        // Apply friction
        if (contact_settings.combined_friction > 0.0) {
            // Calculate friction direction by removing normal velocity from the relative velocity
            const friction_direction_len_sq = friction_direction_in.lengthSq();
            if (friction_direction_len_sq > 1.0e-12) {
                // Normalize friction direction
                const friction_direction = friction_direction_in.divScalar(@sqrt(friction_direction_len_sq));

                // Calculate max friction impulse
                const max_lambda_f = contact_settings.combined_friction * contact_constraint.getTotalLambda();

                var friction: ContactConstraintPart(.dynamic, .dynamic) = .{};
                friction.setTotalLambda(0.0);
                friction.calculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, friction_direction, .{ .bias = 0.0 });
                _ = friction.solveVelocityConstraint(&linear_velocity1, &angular_velocity1, &linear_velocity2, &angular_velocity2, inv_m1, inv_m2, friction_direction, -max_lambda_f, max_lambda_f);
            }
        }

        // Write back velocities
        body_mp.setLinearVelocityClamped(linear_velocity1);
        body_mp.setAngularVelocityClamped(angular_velocity1);
        if (comptime type2 != .static) {
            const body2_mp = body2.getMotionPropertiesMut();
            body2_mp.setLinearVelocityClamped(linear_velocity2);
            body2_mp.setAngularVelocityClamped(angular_velocity2);
        }
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
        const bodies_batch = 64;
        var bodies_to_activate: [bodies_batch]BodyID = undefined; // JPH_STACK_ALLOC with a fixed size
        var num_bodies_to_activate: usize = 0;

        for (sorted_ccd_bodies.items) |ccd_body| {
            const body1 = self.body_manager.getBodyMut(ccd_body.body_id1);
            const body_mp = body1.getMotionPropertiesMut();

            // If there was a hit
            if (!ccd_body.body_id2.isInvalid()) {
                const body2 = self.body_manager.getBodyMut(ccd_body.body_id2);

                // Determine if the other body has a CCD body
                const ccd_body2 = getCCDBody(body2, step);
                if (ccd_body2) |other| {
                    std.debug.assert(!other.body_id2.eql(ccd_body.body_id1)); // If we collided with another body, that other body should have ignored collisions with us!

                    // Check if the other body found a hit that is further away
                    if (other.fraction > ccd_body.fraction) {
                        // Reset the colliding body of the other CCD body. The other body will shorten its distance traveled and will not do any collision response (we'll do that).
                        // This means that at this point we have triggered a contact point add/persist for our further hit by accident for the other body.
                        // We accept this as calling the contact point callbacks here would require persisting the manifolds up to this point and doing the callbacks single threaded.
                        other.body_id2 = .invalid;
                        other.fraction_plus_slop = ccd_body.fraction;
                    }
                }

                // If the other body moved less than us before hitting something, we're not colliding with it so we again have triggered contact point add/persist callbacks by accident.
                // We'll just move to the collision position anyway (as that's the last position we know is good), but we won't do any collision response.
                if (ccd_body2 == null or ccd_body2.?.fraction >= ccd_body.fraction) {
                    const contact_settings = &ccd_body.contact_settings;

                    // Calculate contact point velocity for body 1
                    const r1_plus_u = toVec3(ccd_body.contact_point_on2.sub(body1.getCenterOfMassPosition().addVec3(ccd_body.delta_position.mulScalar(ccd_body.fraction))));
                    const v1 = body1.getPointVelocityCOM(r1_plus_u);

                    // Calculate inverse mass for body 1
                    const inv_m1 = contact_settings.inv_mass_scale1 * body_mp.getInverseMass();
                    const inv_i1 = body1.getInverseInertia().mulScalar(contact_settings.inv_inertia_scale1);

                    if (body2.isRigidBody()) {
                        // Calculate contact point velocity for body 2
                        const r2 = toVec3(ccd_body.contact_point_on2.sub(body2.getCenterOfMassPosition()));
                        const v2 = body2.getPointVelocityCOM(r2);

                        // Calculate relative contact velocity
                        const relative_velocity = v2.sub(v1);
                        const normal_velocity = relative_velocity.dot(ccd_body.contact_normal);

                        // Calculate velocity bias due to restitution
                        var normal_velocity_bias: f32 = undefined;
                        if (contact_settings.combined_restitution > 0.0 and normal_velocity < -self.physics_settings.min_velocity_for_restitution)
                            normal_velocity_bias = contact_settings.combined_restitution * normal_velocity
                        else
                            normal_velocity_bias = 0.0;

                        // Calculate direction in which the friction operates
                        const friction_direction = relative_velocity.sub(ccd_body.contact_normal.mulScalar(normal_velocity));

                        // Dispatch to the correct form
                        switch (body2.getMotionType()) {
                            inline else => |type2| solveCCDContact(type2, body1, inv_m1, inv_i1, r1_plus_u, body2, r2, ccd_body.contact_normal, normal_velocity_bias, friction_direction, contact_settings),
                        }
                    } else {
                        // (Zolt: the resolve job is serial and body 2 is mutable, so the vertices are written through
                        // getVertexMut, RemainingFeaturesArchitecture.md R18)
                        const soft_mp = SoftBodyMotionProperties.fromBase(body2.getMotionPropertiesMut());
                        const soft_shape = body2.getShape().cast(SoftBodyShape);

                        // Convert the sub shape ID of the soft body to a face
                        const face_idx = soft_shape.getFaceIndex(ccd_body.sub_shape_id2);
                        const face = soft_mp.getFace(face_idx);

                        // Get vertices of the face
                        const vtx0 = soft_mp.getVertexMut(face.vertex[0]);
                        const vtx1 = soft_mp.getVertexMut(face.vertex[1]);
                        const vtx2 = soft_mp.getVertexMut(face.vertex[2]);

                        // Inverse mass of the face
                        const vtx0_mass: f32 = if (vtx0.inv_mass > 0.0) 1.0 / vtx0.inv_mass else 1.0e10;
                        const vtx1_mass: f32 = if (vtx1.inv_mass > 0.0) 1.0 / vtx1.inv_mass else 1.0e10;
                        const vtx2_mass: f32 = if (vtx2.inv_mass > 0.0) 1.0 / vtx2.inv_mass else 1.0e10;
                        const inv_m2 = 1.0 / (vtx0_mass + vtx1_mass + vtx2_mass);

                        // Calculate barycentric coordinates of the contact point on the soft body's face
                        const inv_body2_transform = body2.getInverseCenterOfMassTransform();
                        const local_contact = toVec3(inv_body2_transform.mulRVec3(ccd_body.contact_point_on2));
                        const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(vtx0.position.sub(local_contact), vtx1.position.sub(local_contact), vtx2.position.sub(local_contact));
                        const u = bary.u;
                        const v = bary.v;
                        const w = bary.w;

                        // Calculate contact point velocity for the face
                        const v2 = inv_body2_transform.multiply3x3Transposed(vtx0.velocity.mulScalar(u).add(vtx1.velocity.mulScalar(v)).add(vtx2.velocity.mulScalar(w)));
                        const normal_velocity = v2.sub(v1).dot(ccd_body.contact_normal);

                        // Calculate velocity bias due to restitution
                        var normal_velocity_bias: f32 = undefined;
                        if (contact_settings.combined_restitution > 0.0 and normal_velocity < -self.physics_settings.min_velocity_for_restitution)
                            normal_velocity_bias = contact_settings.combined_restitution * normal_velocity
                        else
                            normal_velocity_bias = 0.0;

                        // Calculate resulting velocity change (the math here is similar to AxisConstraintPart but without an inertia term for body 2 as we treat it as a point mass)
                        const r1_plus_u_x_n = r1_plus_u.cross(ccd_body.contact_normal);
                        const invi1_r1_plus_u_x_n = inv_i1.multiply3x3(r1_plus_u_x_n);
                        const jv = r1_plus_u_x_n.dot(body_mp.getAngularVelocity()) - normal_velocity - normal_velocity_bias;
                        const inv_effective_mass = inv_m1 + inv_m2 + invi1_r1_plus_u_x_n.dot(r1_plus_u_x_n);
                        const lambda = jv / inv_effective_mass;
                        body_mp.subLinearVelocityStep(ccd_body.contact_normal.mulScalar(lambda * inv_m1));
                        body_mp.clampLinearVelocity();
                        body_mp.subAngularVelocityStep(invi1_r1_plus_u_x_n.mulScalar(lambda));
                        body_mp.clampAngularVelocity();
                        const delta_v2 = inv_body2_transform.multiply3x3(ccd_body.contact_normal.mulScalar(lambda));
                        vtx0.velocity = vtx0.velocity.add(delta_v2.mulScalar(vtx0.inv_mass));
                        vtx1.velocity = vtx1.velocity.add(delta_v2.mulScalar(vtx1.inv_mass));
                        vtx2.velocity = vtx2.velocity.add(delta_v2.mulScalar(vtx2.inv_mass));

                        // If there's a contact listener, we need to flag this vertex as having had collision and force a contact callback.
                        // Note that during the soft body update that follows this, it is likely that a new collision will be found and that this will be overwritten.
                        if (self.soft_body_contact_listener != null) {
                            const body_id = ccd_body.body_id1;
                            const local_normal = inv_body2_transform.multiply3x3(ccd_body.contact_normal);
                            const contact_plane = Plane.fromPointAndNormal(local_contact, local_normal);
                            vtx0.markCCDContact(body_id, contact_plane);
                            vtx1.markCCDContact(body_id, contact_plane);
                            vtx2.markCCDContact(body_id, contact_plane);
                            soft_mp.requestContactCallback();
                        }
                    }

                    // Activate the 2nd body if it is not already active
                    if (body2.isDynamic() and !body2.isActive()) {
                        bodies_to_activate[num_bodies_to_activate] = ccd_body.body_id2;
                        num_bodies_to_activate += 1;
                        if (num_bodies_to_activate == bodies_batch) {
                            // Batch is full, activate now
                            self.body_manager.activateBodies(bodies_to_activate[0..num_bodies_to_activate]);
                            num_bodies_to_activate = 0;
                        }
                    }

                    if (Core.debug_renderer) {
                        if (self.debug_renderer) |renderer| {
                            if (renderer.flags.physics_system.draw_motion_quality_linear_cast) {
                                // Draw the collision location
                                const collision_transform = body1.getCenterOfMassTransform().postTranslated(ccd_body.delta_position.mulScalar(ccd_body.fraction));
                                body1.getShape().draw(renderer, collision_transform, Vec3.one(), Color.yellow, false, true);

                                // Draw the collision location + slop
                                const collision_transform_plus_slop = body1.getCenterOfMassTransform().postTranslated(ccd_body.delta_position.mulScalar(ccd_body.fraction_plus_slop));
                                body1.getShape().draw(renderer, collision_transform_plus_slop, Vec3.one(), Color.orange, false, true);

                                // Draw contact normal
                                renderer.drawArrow(ccd_body.contact_point_on2, ccd_body.contact_point_on2.subVec3(ccd_body.contact_normal), Color.yellow, 0.1);

                                // Draw post contact velocity
                                renderer.drawArrow(collision_transform.getTranslation(), collision_transform.getTranslation().addVec3(body1.getLinearVelocity()), Color.orange, 0.1);
                                renderer.drawArrow(collision_transform.getTranslation(), collision_transform.getTranslation().addVec3(body1.getAngularVelocity()), Color.purple, 0.1);
                            }
                        }
                    }
                }
            }

            // Update body position
            body1.addPositionStep(ccd_body.delta_position.mulScalar(ccd_body.fraction_plus_slop));
        }

        // Activate the requested bodies
        if (num_bodies_to_activate > 0)
            self.body_manager.activateBodies(bodies_to_activate[0..num_bodies_to_activate]);
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

    /// Zolt (D5, RemainingFeaturesArchitecture.md R17): a protocol job. It takes the "no active soft bodies" path (kick
    /// StartNextStep) once poisoned, and when it cannot copy the active bodies or allocate the contexts (the error is
    /// latched). A job that it cannot create is not an error, it does the work of that job itself:
    /// - no Finalize job: it runs the whole update inline (`softBodyUpdateInline`), then kicks StartNextStep;
    /// - a missing Simulate job: it removes one Finalize dependency for it; the handle stays invalid and the Collide jobs
    ///   skip it;
    /// - no Simulate job at all: it creates no Collide job, collides and simulates inline, then removes the Finalize
    ///   dependencies (a running Collide job could fill the colliding shapes while the inline simulation reads them);
    /// - a missing Collide job: after the creations it runs the collide claim loop itself, then removes the dependencies
    ///   of the missing Collide jobs from every Simulate job.
    fn jobSoftBodyPrepare(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        const has_active_soft_bodies = blk: {
            // Reading soft body positions
            var grant: BodyAccess.Grant = .init(.none, .read);
            defer grant.deinit();

            // Get the active soft bodies (Zolt: the copy allocates; once poisoned the list stays empty)
            var active_bodies: BodyIDVector = .empty;
            defer active_bodies.deinit(self.allocator);
            if (!context.isOutOfMemory())
                self.body_manager.getActiveBodies(self.allocator, .soft_body, &active_bodies) catch context.setOutOfMemory();

            // Quit if there are no active soft bodies
            if (active_bodies.items.len == 0)
                break :blk false;

            // Sort to get a deterministic update order
            quickSort(BodyID, active_bodies.items, {}, struct {
                fn lessThan(_: void, id1: BodyID, id2: BodyID) bool {
                    return id1.lessThan(id2);
                }
            }.lessThan);

            // Allocate soft body contexts
            context.soft_body_update_contexts = TempArray(SoftBodyUpdateContext).alloc(context.temp_allocator, active_bodies.items.len) catch {
                // Zolt (R17): latch the error and take the "no active soft bodies" path
                context.setOutOfMemory();
                break :blk false;
            };

            // Initialize soft body contexts
            for (context.soft_body_update_contexts.items, active_bodies.items) |*sb_ctx, body_id| {
                sb_ctx.* = .{};
                const body = self.body_manager.getBodyMut(body_id);
                const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesMut());
                mp.initializeUpdateContext(context.step_delta_time, body, self, sb_ctx);
            }
            break :blk true;
        };

        // (Zolt: the kick happens after the scope of the grant, Jolt kicks inside it, see jobFindCollisions)
        if (!has_active_soft_bodies) {
            // Kick the next step
            if (step.start_next_step.isValid())
                step.start_next_step.removeDependency(.{});
            return;
        }

        // We're ready to collide the first soft body
        context.soft_body_to_collide.store(0, .release);

        // Determine number of jobs to spawn
        const num_soft_body_jobs = context.getMaxConcurrency();

        // Create finalize job
        step.soft_body_finalize = context.job_system.createJob("SoftBodyFinalize", color_soft_body_finalize, .init(struct {
            fn call(c: *PhysicsUpdateContext, s: *Step) void {
                // Zolt (R17): a protocol job, it always frees the contexts
                c.physics_system.jobSoftBodyFinalize(c);

                // Kick the next step
                if (s.start_next_step.isValid())
                    s.start_next_step.removeDependency(.{});
            }
        }.call, .{ context, step }), .{ .num_dependencies = num_soft_body_jobs }) catch {
            // Zolt (R17): not an error, do the work of all soft body jobs here
            self.softBodyUpdateInline(context);

            // Kick the next step
            if (step.start_next_step.isValid())
                step.start_next_step.removeDependency(.{});
            return;
        }; // depends on: soft body simulate
        context.barrier.addJob(&step.soft_body_finalize);

        // Create simulate jobs
        PhysicsUpdateContext.resizeHandles(&step.soft_body_simulate, num_soft_body_jobs);
        var num_simulate_jobs: u32 = 0;
        for (step.soft_body_simulate.slice(), 0..) |*h, i| {
            h.* = context.job_system.createJob("SoftBodySimulate", color_soft_body_simulate, .init(struct {
                fn call(s: *Step, thread_index: u32) void {
                    s.context.physics_system.jobSoftBodySimulate(s.context, thread_index);

                    s.soft_body_finalize.removeDependency(.{});
                }
            }.call, .{ step, @as(u32, @intCast(i)) }), .{ .num_dependencies = num_soft_body_jobs }) catch continue; // depends on: soft body collide (Zolt (R17): the handle of a job that cannot be created stays invalid)
            num_simulate_jobs += 1;
        }
        if (num_simulate_jobs == 0) {
            // Zolt (R17): no Simulate job could be created. Create no Collide job, collide and simulate here, then let
            // Finalize run.
            self.jobSoftBodyCollide(context);
            self.jobSoftBodySimulate(context, 0);
            step.soft_body_finalize.removeDependency(.{ .count = @intCast(num_soft_body_jobs) });
            return;
        }
        if (num_simulate_jobs < num_soft_body_jobs) {
            // Zolt (R17): the Simulate jobs that could not be created do not keep Finalize waiting
            step.soft_body_finalize.removeDependency(.{ .count = @intCast(num_soft_body_jobs - num_simulate_jobs) });
        }
        addValidJobs(context.barrier, step.soft_body_simulate.constSlice());

        // Create collision jobs
        PhysicsUpdateContext.resizeHandles(&step.soft_body_collide, num_soft_body_jobs);
        var num_missing_collide_jobs: u32 = 0;
        for (step.soft_body_collide.slice()) |*h| {
            h.* = context.job_system.createJob("SoftBodyCollide", color_soft_body_collide, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    c.physics_system.jobSoftBodyCollide(c);

                    for (s.soft_body_simulate.constSlice()) |*sh| {
                        if (sh.isValid()) // Zolt (R17): skip the Simulate jobs that could not be created
                            sh.removeDependency(.{});
                    }
                }
            }.call, .{ context, step }), .{}) catch {
                // Zolt (R17): not an error, collide below
                num_missing_collide_jobs += 1;
                continue;
            }; // depends on: nothing
        }
        addValidJobs(context.barrier, step.soft_body_collide.constSlice());

        if (num_missing_collide_jobs > 0) {
            // Zolt (R17): take the soft bodies that the missing Collide jobs would have taken, then release the Simulate
            // jobs from the dependencies of the missing Collide jobs
            self.jobSoftBodyCollide(context);
            for (step.soft_body_simulate.constSlice()) |*h| {
                if (h.isValid())
                    h.removeDependency(.{ .count = @intCast(num_missing_collide_jobs) });
            }
        }
    }

    /// Zolt (R17): add the jobs of `handles` to the barrier, without the handles of the jobs that could not be created
    fn addValidJobs(barrier: *JobSystem.Barrier, handles: []const JobHandle) void {
        for (handles) |*h| {
            if (!h.isValid()) {
                for (handles) |*valid| {
                    if (valid.isValid())
                        barrier.addJob(valid);
                }
                return;
            }
        }
        barrier.addJobs(handles);
    }

    /// Zolt (R17): the SoftBodyFinalize job could not be created. Do the work of the Collide, Simulate and Finalize jobs
    /// here, each under the grants of the job it replaces.
    fn softBodyUpdateInline(self: *Self, context: *PhysicsUpdateContext) void {
        self.jobSoftBodyCollide(context);
        self.jobSoftBodySimulate(context, 0);
        self.jobSoftBodyFinalize(context);
    }

    /// Zolt (R17): checks the out of memory latch before every claim. When determineCollidingShapes cannot allocate, the
    /// error is latched and the claims stop; SoftBodyFinalize clears the partial colliding lists.
    fn jobSoftBodyCollide(self: *const Self, context: *PhysicsUpdateContext) void {
        // Reading rigid body positions and velocities
        var grant: BodyAccess.Grant = .init(.read, .read);
        defer grant.deinit();

        while (!context.isOutOfMemory()) {
            // Fetch the next soft body
            const sb_idx = context.soft_body_to_collide.fetchAdd(1, .acquire);
            if (sb_idx >= context.getNumSoftBodies())
                break;

            // Do a broadphase check
            const sb_ctx = &context.soft_body_update_contexts.items[sb_idx];
            sb_ctx.motion_properties.determineCollidingShapes(sb_ctx, self, self.getBodyLockInterfaceNoLock()) catch context.setOutOfMemory();
        }
    }

    /// Zolt (R17): checks the out of memory latch at entry only, never inside parallelUpdate: the other Simulate jobs wait
    /// for the batches that this one claimed. A poisoned Simulate job does nothing (its closure still removes its Finalize
    /// dependency). Simulate never allocates.
    fn jobSoftBodySimulate(self: *const Self, context: *PhysicsUpdateContext, thread_index: u32) void {
        // Updating velocities of soft bodies, allow the contact listener to read the soft body state
        var grant: BodyAccess.Grant = .init(.read_write, .read);
        defer grant.deinit();

        if (context.isOutOfMemory())
            return;

        // Calculate at which body we start to distribute the workload across the threads
        const num_soft_bodies = context.getNumSoftBodies();
        const start_idx = thread_index * num_soft_bodies / context.getMaxConcurrency();

        // Keep running partial updates until everything has been updated
        const did_work = @intFromEnum(SoftBodyMotionProperties.Status.did_work);
        var status: u32 = undefined;
        while (true) {
            // Reset status
            status = 0;

            // Update all soft bodies
            var i: u32 = 0;
            while (i < num_soft_bodies) : (i += 1) {
                // Fetch the soft body context
                const sb_ctx = &context.soft_body_update_contexts.items[(start_idx + i) % num_soft_bodies];

                // To avoid trashing the cache too much, we prefer to stick to one soft body until we cannot progress it any further
                var sb_status: u32 = undefined;
                while (true) {
                    sb_status = @intFromEnum(sb_ctx.motion_properties.parallelUpdate(sb_ctx, &self.physics_settings));
                    status |= sb_status;
                    if (sb_status != did_work)
                        break;
                }
            }

            // If we didn't perform any work, yield the thread so that something else can run
            if ((status & did_work) == 0)
                std.Thread.yield() catch {};

            if (status == @intFromEnum(SoftBodyMotionProperties.Status.done))
                break;
        }
    }

    /// Zolt (R17): a protocol job. When poisoned it skips the velocity, position, bounds and sleep updates (the contexts
    /// may not be done). It always clears the colliding shapes and sensors of every context (they hold shape references)
    /// and frees the contexts at Jolt's position.
    fn jobSoftBodyFinalize(self: *Self, context: *PhysicsUpdateContext) void {
        // Updating rigid body velocities and soft body positions / velocities
        var grant: BodyAccess.Grant = .init(.read_write, .read_write);
        defer grant.deinit();

        // Can activate and deactivate bodies
        var grant_active: BodyManager.GrantActiveBodiesAccess = .init(true, true);
        defer grant_active.deinit();

        if (context.isOutOfMemory()) {
            // Zolt (R17): release what the Collide jobs collected
            for (context.soft_body_update_contexts.items) |*sb_ctx|
                sb_ctx.motion_properties.clearCollidingShapesAndSensors();
        } else {
            const bodies_batch = 64;
            var bodies_to_update_bounds: [bodies_batch]BodyID = undefined; // JPH_STACK_ALLOC with a fixed size
            var num_bodies_to_update_bounds: usize = 0;
            var bodies_to_put_to_sleep: [bodies_batch]BodyID = undefined; // JPH_STACK_ALLOC with a fixed size
            var num_bodies_to_put_to_sleep: usize = 0;

            for (context.soft_body_update_contexts.items) |*sb_ctx| {
                // Apply the rigid body velocity deltas
                sb_ctx.motion_properties.updateRigidBodyVelocities(sb_ctx, self.getBodyInterfaceNoLockMut());

                // Update the position
                sb_ctx.body.setPositionAndRotationInternal(sb_ctx.body.getPosition().addVec3(sb_ctx.delta_position), sb_ctx.body.getRotation(), .{ .reset_sleep_timer = false });

                const id = sb_ctx.body.getID();
                bodies_to_update_bounds[num_bodies_to_update_bounds] = id;
                num_bodies_to_update_bounds += 1;
                if (num_bodies_to_update_bounds == bodies_batch) {
                    // Buffer full, flush now
                    self.getBroadPhase().notifyBodiesAABBChanged(bodies_to_update_bounds[0..num_bodies_to_update_bounds], .{ .take_lock = false });
                    num_bodies_to_update_bounds = 0;
                }

                if (sb_ctx.can_sleep == .can_sleep) {
                    // This body should go to sleep
                    bodies_to_put_to_sleep[num_bodies_to_put_to_sleep] = id;
                    num_bodies_to_put_to_sleep += 1;
                    if (num_bodies_to_put_to_sleep == bodies_batch) {
                        self.body_manager.deactivateBodies(bodies_to_put_to_sleep[0..num_bodies_to_put_to_sleep]);
                        num_bodies_to_put_to_sleep = 0;
                    }
                }
            }

            // Notify change bounds on requested bodies
            if (num_bodies_to_update_bounds > 0)
                self.getBroadPhase().notifyBodiesAABBChanged(bodies_to_update_bounds[0..num_bodies_to_update_bounds], .{ .take_lock = false });

            // Notify bodies to go to sleep
            if (num_bodies_to_put_to_sleep > 0)
                self.body_manager.deactivateBodies(bodies_to_put_to_sleep[0..num_bodies_to_put_to_sleep]);
        }

        // Free soft body contexts
        context.soft_body_update_contexts.free(context.temp_allocator);
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
            if (!try self.contact_manager.restoreState(stream, opts.filter))
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
