//! Port of: Jolt/Physics/PhysicsSettings.h
//! Status: complete
//!
//! The constants are file level declarations (`cDefaultCollisionTolerance` -> `default_collision_tolerance`), the
//! struct `PhysicsSettings` keeps Jolt's defaults (`int` -> `i32`, `uint` -> `u32`). `Square(x)` in a default is
//! evaluated in f32 like the C++ (comptime arithmetic on typed f32 values rounds to f32).

const math = @import("../Math/Math.zig");

/// If objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter)
pub const default_collision_tolerance: f32 = 1.0e-4;

/// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
pub const default_penetration_tolerance: f32 = 1.0e-4; // Stop when there's less than 1% change

/// How much padding to add around objects
pub const default_convex_radius: f32 = 0.05;

/// Used by (Tapered)CapsuleShape to determine when supporting face is an edge rather than a point (unit: meter)
pub const capsule_projection_slop: f32 = 0.02;

/// Max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm to determine if two edges are shared (unit: meter^2)
pub const default_internal_edge_removal_vertex_tolerance_sq: f32 = 1.0e-8;

/// Maximum amount of jobs to allow
pub const max_physics_jobs: i32 = 2048;

/// Maximum amount of barriers to allow
pub const max_physics_barriers: i32 = 8;

pub const PhysicsSettings = struct {
    /// Size of body pairs array, corresponds to the maximum amount of potential body pairs that can be in flight at any time.
    /// Setting this to a low value will use less memory but slow down simulation as threads may run out of narrow phase work.
    max_in_flight_body_pairs: i32 = 16384,

    /// How many PhysicsStepListeners to notify in 1 batch
    step_listeners_batch_size: i32 = 8,

    /// How many step listener batches are needed before spawning another job (set to INT_MAX if no parallelism is desired)
    step_listener_batches_per_job: i32 = 1,

    /// Baumgarte stabilization factor (how much of the position error to 'fix' in 1 update) (unit: dimensionless, 0 = nothing, 1 = 100%)
    baumgarte: f32 = 0.2,

    /// Radius around objects inside which speculative contact points will be detected. Note that if this is too big
    /// you will get ghost collisions as speculative contacts are based on the closest points during the collision detection
    /// step which may not be the actual closest points by the time the two objects hit (unit: meters)
    speculative_contact_distance: f32 = 0.02,

    /// How much bodies are allowed to sink into each other (unit: meters)
    penetration_slop: f32 = 0.02,

    /// Fraction of its inner radius a body must move per step to enable casting for the LinearCast motion quality
    linear_cast_threshold: f32 = 0.75,

    /// Fraction of its inner radius a body may penetrate another body for the LinearCast motion quality
    linear_cast_max_penetration: f32 = 0.25,

    /// Max distance to use to determine if two points are on the same plane for determining the contact manifold between two shape faces (unit: meter)
    manifold_tolerance: f32 = 1.0e-3,

    /// Maximum distance to correct in a single iteration when solving position constraints (unit: meters)
    max_penetration_distance: f32 = 0.2,

    /// Maximum relative delta position for body pairs to be able to reuse collision results from last frame (units: meter^2)
    body_pair_cache_max_delta_position_sq: f32 = math.square(@as(f32, 0.001)), // 1 mm

    /// Maximum relative delta orientation for body pairs to be able to reuse collision results from last frame, stored as cos(max angle / 2)
    body_pair_cache_cos_max_delta_rotation_div2: f32 = 0.99984769515639123915701155881391, // cos(2 degrees / 2)

    /// Maximum angle between normals that allows manifolds between different sub shapes of the same body pair to be combined
    contact_normal_cos_max_delta_rotation: f32 = 0.99619469809174553229501040247389, // cos(5 degree)

    /// Maximum allowed distance between old and new contact point to preserve contact forces for warm start (units: meter^2)
    contact_point_preserve_lambda_max_dist_sq: f32 = math.square(@as(f32, 0.01)), // 1 cm

    /// Max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm to determine if two edges are shared. (unit: meter^2)
    internal_edge_removal_vertex_tolerance_sq: f32 = default_internal_edge_removal_vertex_tolerance_sq,

    /// Number of solver velocity iterations to run
    /// Note that this needs to be >= 2 in order for friction to work (friction is applied using the non-penetration impulse from the previous iteration)
    num_velocity_steps: u32 = 10,

    /// Number of solver position iterations to run
    num_position_steps: u32 = 2,

    /// Minimal velocity needed before a collision can be elastic. If the relative velocity between colliding objects
    /// in the direction of the contact normal is lower than this, the restitution will be zero regardless of the configured
    /// value. This lets an object settle sooner. Must be a positive number. (unit: m)
    min_velocity_for_restitution: f32 = 1.0,

    /// Time before object is allowed to go to sleep (unit: seconds)
    time_before_sleep: f32 = 0.5,

    /// To detect if an object is sleeping, we use 3 points:
    /// - The center of mass.
    /// - The centers of the faces of the bounding box that are furthest away from the center.
    /// The movement of these points is tracked and if the velocity of all 3 points is lower than this value,
    /// the object is allowed to go to sleep. Must be a positive number. (unit: m/s)
    point_velocity_sleep_threshold: f32 = 0.03,

    // @name These variables are mainly for debugging purposes, they allow turning on/off certain subsystems. You probably want to leave them alone.
    // @{

    /// Whether or not to use warm starting for constraints (initially applying previous frames impulses)
    constraint_warm_start: bool = true,

    /// Whether or not to use the body pair cache, which removes the need for narrow phase collision detection when orientation between two bodies didn't change
    use_body_pair_contact_cache: bool = true,

    /// Whether or not to reduce manifolds with similar contact normals into one contact manifold (see description at Body::SetUseManifoldReduction)
    use_manifold_reduction: bool = true,

    /// If we split up large islands into smaller parallel batches of work (to improve performance)
    use_large_island_splitter: bool = true,

    /// If objects can go to sleep or not
    allow_sleeping: bool = true,

    /// When false, we prevent collision against non-active (shared) edges. Mainly for debugging the algorithm.
    check_active_edges: bool = true,

    // @}
};

test "PhysicsSettings: Square() in the defaults is evaluated in f32" {
    const std = @import("std");
    var x: f32 = 0.001;
    var y: f32 = 0.01;
    _ = .{ &x, &y };
    const settings: PhysicsSettings = .{};
    try std.testing.expectEqual(x * x, settings.body_pair_cache_max_delta_position_sq);
    try std.testing.expectEqual(y * y, settings.contact_point_preserve_lambda_max_dist_sq);
    try std.testing.expectEqual(default_internal_edge_removal_vertex_tolerance_sq, settings.internal_edge_removal_vertex_tolerance_sq);
}
