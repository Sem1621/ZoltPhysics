//! Port of: Jolt/Physics/PhysicsSettings.h (prototype, reduced to the constants used by the collision code)
//! Status: partial
//! Missing: PhysicsSettings struct

/// If objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter)
pub const default_collision_tolerance: f32 = 1.0e-4;

/// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
pub const default_penetration_tolerance: f32 = 1.0e-4; // Stop when there's less than 1% change

/// How much padding to add around objects
pub const default_convex_radius: f32 = 0.05;

/// Used by (Tapered)CapsuleShape to determine when supporting face is an edge rather than a point (unit: meter)
pub const capsule_projection_slop: f32 = 0.02;

/// Max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm (unit: meter^2)
pub const default_internal_edge_removal_vertex_tolerance_sq: f32 = 1.0e-8;
