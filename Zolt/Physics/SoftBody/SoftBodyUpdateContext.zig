//! Port of: Jolt/Physics/SoftBody/SoftBodyUpdateContext.h
//! Status: complete
//!
//! Created by Phase 5 F1 (SimulationArchitecture D1) so that PhysicsSystem's soft body jobs (F2) and the soft body
//! update (Phase 9) use the final type. Jolt's header only declares data, so the port is complete: the fields with
//! Jolt's names and types, uninitialized (`undefined`) where Jolt leaves them uninitialized, and the atomics with
//! Jolt's initial values. PhysicsSystem default constructs one per active soft body (`ctx.* = .{}` for Jolt's
//! `new (sb_ctx) SoftBodyUpdateContext`) and `SoftBodyMotionProperties.InitializeUpdateContext` fills in the input.
//! - `EState` is declared before the fields (Zig allows no declarations between fields).
//! - `NonCopyable` in Jolt: the jobs of a step point to it, so it must not be copied or moved after it is filled in.
//! - The `SoftBodyContactListener *` (null when no listener is set) is stored by value, like every pattern B interface
//!   (SimulationArchitecture D7): `?SoftBodyContactListener`.

const std = @import("std");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const Body = @import("../Body/Body.zig").Body;
const CanSleep = @import("../Body/MotionProperties.zig").CanSleep;
const SimShapeFilter = @import("../Collision/SimShapeFilter.zig").SimShapeFilter;
const SoftBodyMotionProperties = @import("SoftBodyMotionProperties.zig").SoftBodyMotionProperties;
const SoftBodyContactListener = @import("SoftBodyContactListener.zig").SoftBodyContactListener;

/// Temporary data used by the update of a soft body
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
pub const SoftBodyUpdateContext = struct {
    /// Number of vertices to process in a batch in DetermineCollisionPlanes
    pub const vertex_collision_batch: u32 = 64;
    /// Number of vertices to group for processing batches of constraints in ApplyEdgeConstraints
    pub const vertex_constraint_batch: u32 = 256;

    /// Describes progress in the current update
    pub const State = enum(u32) {
        /// Determine collision planes for vertices in parallel
        determine_collision_planes,
        /// Determine collisions with sensors in parallel
        determine_sensor_collisions,
        /// Apply constraints in parallel
        apply_constraints,
        /// Update is finished
        done,
    };

    // Input
    /// Body that is being updated
    body: *Body = undefined,
    /// Motion properties of that body
    motion_properties: *SoftBodyMotionProperties = undefined,
    /// Contact listener to fire callbacks to
    contact_listener: ?SoftBodyContactListener = undefined,
    /// Shape filter to use for collision detection
    sim_shape_filter: ?*const SimShapeFilter = undefined,
    /// Transform of the body relative to the soft body
    center_of_mass_transform: RMat44 = undefined,
    /// Gravity vector in local space of the soft body
    gravity: Vec3 = undefined,
    /// Displacement of the center of mass due to gravity in the current time step
    displacement_due_to_gravity: Vec3 = undefined,
    /// Delta time for the current time step
    delta_time: f32 = undefined,
    /// Delta time for each sub step
    sub_step_delta_time: f32 = undefined,

    // State of the update
    /// Current state of the update
    state: std.atomic.Value(State) = .init(.determine_collision_planes),
    /// Next vertex to process for DetermineCollisionPlanes
    next_collision_vertex: std.atomic.Value(u32) = .init(0),
    /// Number of vertices processed by DetermineCollisionPlanes, used to determine if we can go to the next step
    num_collision_vertices_processed: std.atomic.Value(u32) = .init(0),
    /// Next sensor to process for DetermineCollisionPlanes
    next_sensor_index: std.atomic.Value(u32) = .init(0),
    /// Number of sensors processed by DetermineSensorCollisions, used to determine if we can go to the next step
    num_sensors_processed: std.atomic.Value(u32) = .init(0),
    /// Next simulation iteration to process
    next_iteration: std.atomic.Value(u32) = .init(0),
    /// Next constraint group to process
    next_constraint_group: std.atomic.Value(u32) = .init(0),
    /// Number of groups processed, used to determine if we can go to the next iteration
    num_constraint_groups_processed: std.atomic.Value(u32) = .init(0),

    // Output
    /// Delta position of the body in the current time step, should be applied after the update
    delta_position: Vec3 = undefined,
    /// Can the body sleep? Should be applied after the update
    can_sleep: CanSleep = undefined,
};

test "SoftBodyUpdateContext: default construction and state" {
    const expectEqual = std.testing.expectEqual;
    try expectEqual(@as(u32, 64), SoftBodyUpdateContext.vertex_collision_batch);
    try expectEqual(@as(u32, 256), SoftBodyUpdateContext.vertex_constraint_batch);

    // new (sb_ctx) SoftBodyUpdateContext, then the input is filled in
    var body: Body = .{ .id = .init(3) };
    var motion_properties: SoftBodyMotionProperties = .{};
    var ctx: SoftBodyUpdateContext = .{};
    ctx.body = &body;
    ctx.motion_properties = &motion_properties;
    ctx.contact_listener = null;
    ctx.sim_shape_filter = null;
    try expectEqual(SoftBodyUpdateContext.State.determine_collision_planes, ctx.state.load(.acquire));
    inline for (.{ "next_collision_vertex", "num_collision_vertices_processed", "next_sensor_index", "num_sensors_processed", "next_iteration", "next_constraint_group", "num_constraint_groups_processed" }) |name|
        try expectEqual(@as(u32, 0), @field(ctx, name).load(.monotonic));

    // The progress of an update (ParallelDetermineCollisionPlanes / StartFirstIteration)
    try expectEqual(@as(u32, 0), ctx.next_collision_vertex.fetchAdd(SoftBodyUpdateContext.vertex_collision_batch, .acquire));
    ctx.state.store(.apply_constraints, .release);
    try expectEqual(SoftBodyUpdateContext.State.apply_constraints, ctx.state.load(.acquire));
    try expectEqual(@as(u32, 3), ctx.body.getID().getIndexAndSequenceNumber());
    try expectEqual(&motion_properties.base, &ctx.motion_properties.base);
}
