//! Port of: Jolt/Physics/SoftBody/SoftBodyMotionProperties.h, Jolt/Physics/SoftBody/SoftBodyMotionProperties.cpp
//! Status: partial
//! Missing: DetermineCollidingShapes, DetermineCollisionPlanes, DetermineSensorCollisions, ApplyPressure,
//!   IntegratePositions, the Apply*Constraints solvers, ApplyCollisionConstraintsAndUpdateVelocities,
//!   UpdateSoftBodyState, StartFirstIteration, StartNextIteration, the Parallel* functions, ProcessGroup,
//!   InitializeUpdateContext, UpdateRigidBodyVelocities, SkinVertices, CustomUpdate, SaveState, RestoreState (stubs that
//!   panic, group S4); the JPH_DEBUG_RENDERER draw functions (group D4)
//!
//! RemainingFeaturesArchitecture.md R15, R16 and SimulationArchitecture D1:
//! - `SoftBodyMotionProperties : public MotionProperties` embeds its base as `base` (the first field), then Jolt's
//!   fields in Jolt's order. `fromBase` / `fromBaseConst` are Jolt's `static_cast<SoftBodyMotionProperties *>`.
//! - `allocator` (Zolt) is the allocator of the BodyManager that created the soft body. It allocates the arrays.
//!   `deinit` is Jolt's `~SoftBodyMotionProperties`: it frees the arrays and releases the shared settings and the leaf
//!   shapes of the colliding shapes and sensors.
//! - `RodState` is an extern struct with an extern union: IntegratePositions reuses the storage of the angular velocity
//!   for the previous rotation, which is defined for an extern union.
//! - `mSkinStateTransform` only exists with JPH_DEBUG_RENDERER: `skin_state_transform` is `void` without the option.
//! - `mNeedContactCallback` is a `std.atomic.Value(bool)`, Jolt's relaxed order is `.monotonic`. `mNumSensors` (Jolt's
//!   TSAN workaround) is a `std.atomic.Value(u32)`; Jolt leaves it uninitialized until DetermineCollidingShapes, Zolt
//!   starts it at 0.
//! - Const overloads: `getVertices` / `getVerticesMut` (a slice: the values can change, the number cannot),
//!   `getVertex` / `getVertexMut`.
//! - `initialize` returns `Allocator.Error`: it allocates the vertices, the rod states and the skin states. It reserves
//!   them first, so out of memory leaves the motion properties unchanged. A second call resizes the arrays and never
//!   leaks. Like Jolt it keeps the old rod states and skin states when the new settings have none, and it encapsulates
//!   the new vertices into the old local bounds.
//! - Zolt (R20, UB2): `initialize` panics for shared settings without update groups (`optimize` was not called). Jolt
//!   asserts in ParallelApplyConstraints and then reads out of bounds. It also panics for null shared settings, where
//!   Jolt dereferences the null pointer.
//! - New vertices are zero before `initialize` fills them in: Jolt leaves `mCollisionPlane` uninitialized, the parity
//!   reference zero-fills its allocations.
//! - `is_stub`: soft bodies cannot be simulated yet. The simulation parity harness blocks the soft body scenes while it
//!   is declared. Group S4 removes it.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const TempAllocator = @import("../../Core/TempAllocator.zig").TempAllocator;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const Body = @import("../Body/Body.zig").Body;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const MotionProperties = @import("../Body/MotionProperties.zig").MotionProperties;
const MassProperties = @import("../Body/MassProperties.zig").MassProperties;
const BodyInterface = @import("../Body/BodyInterface.zig").BodyInterface;
const BodyLockInterface = @import("../Body/BodyLockInterface.zig").BodyLockInterface;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsSettings = @import("../PhysicsSettings.zig").PhysicsSettings;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const SoftBodyCreationSettings = @import("SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;
const SoftBodyVertex = @import("SoftBodyVertex.zig").SoftBodyVertex;
const SoftBodyUpdateContext = @import("SoftBodyUpdateContext.zig").SoftBodyUpdateContext;

/// This class contains the runtime information of a soft body.
//
// Based on: XPBD, Extended Position Based Dynamics, Matthias Muller, Ten Minute Physics
// See: https://matthias-research.github.io/pages/tenMinutePhysics/09-xpbd.pdf
pub const SoftBodyMotionProperties = struct {
    /// Soft bodies cannot be simulated yet (the update is not ported, group S4). The simulation parity harness skips
    /// the soft body scenes while this is declared.
    pub const is_stub = true;

    pub const Vertex = SoftBodyVertex;
    pub const Edge = SoftBodySharedSettings.Edge;
    pub const RodStretchShear = SoftBodySharedSettings.RodStretchShear;
    pub const RodBendTwist = SoftBodySharedSettings.RodBendTwist;
    pub const Face = SoftBodySharedSettings.Face;
    pub const DihedralBend = SoftBodySharedSettings.DihedralBend;
    pub const Volume = SoftBodySharedSettings.Volume;
    pub const InvBind = SoftBodySharedSettings.InvBind;
    pub const SkinWeight = SoftBodySharedSettings.SkinWeight;
    pub const Skinned = SoftBodySharedSettings.Skinned;
    pub const LRA = SoftBodySharedSettings.LRA;

    /// Return code for ParallelUpdate
    pub const Status = enum(u32) {
        /// No work was done because other threads were still working on a batch that cannot run concurrently
        no_work = 1 << 0,
        /// Work was done to progress the update
        did_work = 1 << 1,
        /// All work is done
        done = 1 << 2,
    };

    /// Information about a leaf shape that we're colliding with (private in Jolt)
    pub const LeafShape = struct {
        /// Transform of the shape relative to the soft body
        transform: Mat44,
        /// Scale of the shape
        scale: Vec3,
        /// Shape
        shape: RefConst(Shape) = .empty,

        /// Constructor LeafShape(inTransform, inScale, inShape) (adds a reference to the shape)
        pub fn init(transform: Mat44, scale: Vec3, shape: *const Shape) LeafShape {
            return .{ .transform = transform, .scale = scale, .shape = .init(shape) };
        }

        /// Destructor (releases the shape)
        pub fn deinit(self: *LeafShape) void {
            self.shape.deinit();
        }
    };

    /// Collect information about the colliding bodies (private in Jolt)
    pub const CollidingShape = struct {
        /// Transform of the body relative to the soft body
        center_of_mass_transform: Mat44,
        /// Leaf shapes of the body we hit
        shapes: std.ArrayList(LeafShape) = .empty,
        /// Body ID of the body we hit
        body_id: BodyID,
        /// Motion type of the body we hit
        motion_type: MotionType,
        /// Inverse mass of the body we hit
        inv_mass: f32,
        /// Combined friction of the two bodies
        friction: f32,
        /// Combined restitution of the two bodies
        restitution: f32,
        /// Scale factor for the inverse mass of the soft body vertices
        soft_body_inv_mass_scale: f32,
        /// If the linear/angular velocity changed and the body needs to be updated
        update_velocities: bool,
        /// Inverse inertia in local space to the soft body
        inv_inertia: Mat44,
        /// Linear velocity of the body in local space to the soft body
        linear_velocity: Vec3,
        /// Angular velocity of the body in local space to the soft body
        angular_velocity: Vec3,
        /// Linear velocity of the body in local space to the soft body at start
        original_linear_velocity: Vec3,
        /// Angular velocity of the body in local space to the soft body at start
        original_angular_velocity: Vec3,

        /// Get the velocity of a point on this body
        pub fn getPointVelocity(self: *const CollidingShape, point_relative_to_com: Vec3) Vec3 {
            return self.linear_velocity.add(self.angular_velocity.cross(point_relative_to_com));
        }

        /// Destructor (releases the leaf shapes and frees their array)
        pub fn deinit(self: *CollidingShape, allocator: Allocator) void {
            for (self.shapes.items) |*s| s.deinit();
            self.shapes.deinit(allocator);
        }
    };

    /// Collect information about the colliding sensors (private in Jolt)
    pub const CollidingSensor = struct {
        /// Transform of the body relative to the soft body
        center_of_mass_transform: Mat44,
        /// Leaf shapes of the body we hit
        shapes: std.ArrayList(LeafShape) = .empty,
        /// Body ID of the body we hit
        body_id: BodyID,
        /// If the sensor collided with the soft body
        has_contact: bool,

        /// Destructor (releases the leaf shapes and frees their array)
        pub fn deinit(self: *CollidingSensor, allocator: Allocator) void {
            for (self.shapes.items) |*s| s.deinit();
            self.shapes.deinit(allocator);
        }
    };

    /// Information about the current state of a rod (private in Jolt).
    pub const RodState = extern struct {
        /// Rotation of the rod, relative to center of mass transform
        rotation: Quat,
        /// Jolt's anonymous union
        u: extern union {
            /// Angular velocity of the rod, relative to center of mass transform, valid only outside of the simulation.
            angular_velocity: Vec3,
            /// Internal use only. Previous rotation of the rod, relative to center of mass transform, valid only during the simulation.
            previous_rotation_internal: Quat,
        },

        comptime {
            std.debug.assert(@sizeOf(RodState) == 32);
        }
    };

    /// Information about the state of all skinned vertices (private in Jolt)
    pub const SkinState = struct {
        /// Previous position of the skinned vertex, used to interpolate between the previous and current position
        previous_position: Vec3 = Vec3.zero(),
        /// Current position of the skinned vertex
        position: Vec3 = Vec3.nan(),
        /// Normal of the skinned vertex
        normal: Vec3 = Vec3.nan(),
    };

    base: MotionProperties = .{},

    /// The matrix that transforms mSkinState to world space (JPH_DEBUG_RENDERER)
    skin_state_transform: if (Core.debug_renderer) RMat44 else void = if (Core.debug_renderer) RMat44.identity() else {},

    /// Configuration of the particles and constraints
    settings: RefConst(SoftBodySharedSettings) = .empty,
    /// Current state of all vertices in the simulation
    vertices: std.ArrayList(Vertex) = .empty,
    /// Current state of all rods in the simulation
    rod_states: std.ArrayList(RodState) = .empty,
    /// List of colliding shapes retrieved during the last update
    colliding_shapes: std.ArrayList(CollidingShape) = .empty,
    /// List of colliding sensors retrieved during the last update
    colliding_sensors: std.ArrayList(CollidingSensor) = .empty,
    /// List of skinned positions (1-on-1 with mVertices but only those that are used by the skinning constraints are filled in)
    skin_state: std.ArrayList(SkinState) = .empty,
    /// Bounding box of all vertices
    local_bounds: AABox = .empty,
    /// Predicted bounding box for all vertices using extrapolation of velocity by last step delta time
    local_predicted_bounds: AABox = .empty,
    /// Number of solver iterations
    num_iterations: u32 = undefined,
    /// Workaround for TSAN false positive: store mCollidingSensors.size() in a separate variable.
    num_sensors: std.atomic.Value(u32) = .init(0),
    /// n * R * T, amount of substance * ideal gas constant * absolute temperature, see https://en.wikipedia.org/wiki/Pressure
    pressure: f32 = undefined,
    /// Multiplier applied to Skinned::mMaxDistance to allow tightening or loosening of the skin constraints
    skinned_max_distance_multiplier: f32 = 1.0,
    /// How big the particles are, can be used to push the vertices a little bit away from the surface of other bodies to prevent z-fighting
    vertex_radius: f32 = 0.0,
    /// Update the position of the body while simulating (set to false for something that is attached to the static world)
    update_position: bool = undefined,
    /// If the faces in this soft body should be treated as double sided for the purpose of collision detection (ray cast / collide shape / cast shape)
    faces_double_sided: bool = undefined,
    /// True if the soft body has collided with anything in the last update
    need_contact_callback: std.atomic.Value(bool) = .init(false),
    /// If skin constraints are enabled
    enable_skin_constraints: bool = true,
    /// True if the skinning was updated in the last update so that the previous position of the skin state is valid
    skin_state_previous_position_valid: bool = false,

    /// Zolt: allocates the arrays (the allocator of the BodyManager)
    allocator: Allocator,

    /// static_cast<SoftBodyMotionProperties *>(MotionProperties *)
    pub fn fromBase(mp: *MotionProperties) *SoftBodyMotionProperties {
        return @alignCast(@fieldParentPtr("base", mp)); // RMat44 (skin_state_transform) can align this above its base
    }

    /// static_cast<const SoftBodyMotionProperties *>(const MotionProperties *)
    pub fn fromBaseConst(mp: *const MotionProperties) *const SoftBodyMotionProperties {
        return @alignCast(@fieldParentPtr("base", mp));
    }

    /// Destructor (~SoftBodyMotionProperties): frees the arrays, releases the shared settings and the leaf shapes
    pub fn deinit(self: *SoftBodyMotionProperties) void {
        const allocator = self.allocator;
        self.settings.deinit();
        self.vertices.deinit(allocator);
        self.rod_states.deinit(allocator);
        for (self.colliding_shapes.items) |*cs| cs.deinit(allocator);
        self.colliding_shapes.deinit(allocator);
        for (self.colliding_sensors.items) |*cs| cs.deinit(allocator);
        self.colliding_sensors.deinit(allocator);
        self.skin_state.deinit(allocator);
    }

    /// Initialize the soft body motion properties
    pub fn initialize(self: *SoftBodyMotionProperties, settings: *const SoftBodyCreationSettings) Allocator.Error!void {
        const shared = settings.settings.get() orelse @panic("SoftBodyMotionProperties.initialize: the creation settings have no shared settings (Jolt dereferences a null mSettings)");

        // Zolt (UB2): Jolt asserts during the simulation and then reads out of bounds
        if (shared.update_groups.items.len == 0)
            @panic("SoftBodySharedSettings: call optimize() before creating a soft body");

        // Zolt: reserve the arrays first, so that out of memory leaves the motion properties unchanged
        const num_vertices = shared.vertices.items.len;
        const num_rods = shared.rod_stretch_shear_constraints.items.len;
        try self.vertices.ensureTotalCapacity(self.allocator, num_vertices);
        if (num_rods != 0)
            try self.rod_states.ensureTotalCapacity(self.allocator, num_rods);
        if (shared.skinned_constraints.items.len != 0)
            try self.skin_state.ensureTotalCapacity(self.allocator, num_vertices);

        // Store settings
        self.settings.set(shared);
        self.num_iterations = settings.num_iterations;
        self.pressure = settings.pressure;
        self.update_position = settings.update_position;
        self.faces_double_sided = settings.faces_double_sided;
        self.setVertexRadius(settings.vertex_radius);

        // Initialize vertices
        const old_num_vertices = self.vertices.items.len;
        self.vertices.items.len = num_vertices; // resize, the capacity was reserved above
        if (num_vertices > old_num_vertices)
            @memset(std.mem.sliceAsBytes(self.vertices.items[old_num_vertices..]), 0);
        const rotation = if (settings.make_rotation_identity) Mat44.rotationQuat(settings.rotation) else Mat44.identity();
        for (self.vertices.items, shared.vertices.items) |*out_vertex, *in_vertex| {
            out_vertex.position = rotation.mulVec3(Vec3.fromFloat3(in_vertex.position));
            out_vertex.previous_position = out_vertex.position;
            out_vertex.velocity = rotation.multiply3x3(Vec3.fromFloat3(in_vertex.velocity));
            out_vertex.resetCollision();
            out_vertex.inv_mass = in_vertex.inv_mass;
            self.local_bounds.encapsulateVec3(out_vertex.position);
        }

        // Initialize rods
        if (num_rods != 0) {
            self.rod_states.items.len = num_rods; // resize, the capacity was reserved above
            const rotation_q = rotation.getQuaternion();
            for (self.rod_states.items, shared.rod_stretch_shear_constraints.items) |*out_rod, *in_rod| {
                out_rod.rotation = rotation_q.mul(in_rod.bishop);
                out_rod.u = .{ .angular_velocity = Vec3.zero() };
            }
        }

        // Allocate space for skinned vertices
        if (shared.skinned_constraints.items.len != 0) {
            const old_len = self.skin_state.items.len;
            self.skin_state.items.len = num_vertices; // resize, the capacity was reserved above
            if (num_vertices > old_len) {
                for (self.skin_state.items[old_len..]) |*s| s.* = .{};
            }
        }

        // We don't know delta time yet, so we can't predict the bounds and use the local bounds as the predicted bounds
        self.local_predicted_bounds = self.local_bounds;

        self.calculateMassAndInertia();
    }

    /// Get the shared settings of the soft body
    pub fn getSettings(self: *const SoftBodyMotionProperties) ?*const SoftBodySharedSettings {
        return self.settings.get();
    }

    /// Get the vertices of the soft body
    pub fn getVertices(self: *const SoftBodyMotionProperties) []const Vertex {
        return self.vertices.items;
    }

    /// Get the vertices of the soft body (non-const GetVertices: the values can change, the number cannot)
    pub fn getVerticesMut(self: *SoftBodyMotionProperties) []Vertex {
        return self.vertices.items;
    }

    /// Access an individual vertex
    pub fn getVertex(self: *const SoftBodyMotionProperties, index: u32) *const Vertex {
        return &self.vertices.items[index];
    }

    /// Access an individual vertex (non-const GetVertex)
    pub fn getVertexMut(self: *SoftBodyMotionProperties, index: u32) *Vertex {
        return &self.vertices.items[index];
    }

    /// Access to the state of rods
    pub fn getRodRotation(self: *const SoftBodyMotionProperties, index: u32) Quat {
        return self.rod_states.items[index].rotation;
    }

    pub fn getRodAngularVelocity(self: *const SoftBodyMotionProperties, index: u32) Vec3 {
        return self.rod_states.items[index].u.angular_velocity;
    }

    /// Get the materials of the soft body
    pub fn getMaterials(self: *const SoftBodyMotionProperties) []const RefConst(PhysicsMaterial) {
        return self.settings.get().?.materials.items;
    }

    /// Get the faces of the soft body
    pub fn getFaces(self: *const SoftBodyMotionProperties) []const Face {
        return self.settings.get().?.faces.items;
    }

    /// Access to an individual face
    pub fn getFace(self: *const SoftBodyMotionProperties, index: u32) *const Face {
        return &self.settings.get().?.faces.items[index];
    }

    /// Get the number of solver iterations
    pub fn getNumIterations(self: *const SoftBodyMotionProperties) u32 {
        return self.num_iterations;
    }

    pub fn setNumIterations(self: *SoftBodyMotionProperties, num_iterations: u32) void {
        self.num_iterations = num_iterations;
    }

    /// Get the pressure of the soft body
    pub fn getPressure(self: *const SoftBodyMotionProperties) f32 {
        return self.pressure;
    }

    pub fn setPressure(self: *SoftBodyMotionProperties, pressure: f32) void {
        self.pressure = pressure;
    }

    /// Update the position of the body while simulating (set to false for something that is attached to the static world)
    pub fn getUpdatePosition(self: *const SoftBodyMotionProperties) bool {
        return self.update_position;
    }

    pub fn setUpdatePosition(self: *SoftBodyMotionProperties, update_position: bool) void {
        self.update_position = update_position;
    }

    /// If the faces in this soft body should be treated as double sided for the purpose of collision detection (ray cast / collide shape / cast shape)
    pub fn getFacesDoubleSided(self: *const SoftBodyMotionProperties) bool {
        return self.faces_double_sided;
    }

    pub fn setFacesDoubleSided(self: *SoftBodyMotionProperties, double_sided: bool) void {
        self.faces_double_sided = double_sided;
    }

    /// Global setting to turn on/off skin constraints
    pub fn getEnableSkinConstraints(self: *const SoftBodyMotionProperties) bool {
        return self.enable_skin_constraints;
    }

    pub fn setEnableSkinConstraints(self: *SoftBodyMotionProperties, enable_skin_constraints: bool) void {
        self.enable_skin_constraints = enable_skin_constraints;
    }

    /// Multiplier applied to Skinned::mMaxDistance to allow tightening or loosening of the skin constraints. 0 to hard skin all vertices.
    pub fn getSkinnedMaxDistanceMultiplier(self: *const SoftBodyMotionProperties) f32 {
        return self.skinned_max_distance_multiplier;
    }

    pub fn setSkinnedMaxDistanceMultiplier(self: *SoftBodyMotionProperties, skinned_max_distance_multiplier: f32) void {
        self.skinned_max_distance_multiplier = skinned_max_distance_multiplier;
    }

    /// How big the particles are, can be used to push the vertices a little bit away from the surface of other bodies to prevent z-fighting
    pub fn getVertexRadius(self: *const SoftBodyMotionProperties) f32 {
        return self.vertex_radius;
    }

    pub fn setVertexRadius(self: *SoftBodyMotionProperties, vertex_radius: f32) void {
        if (Core.enable_asserts) std.debug.assert(vertex_radius >= 0.0);
        self.vertex_radius = vertex_radius;
    }

    /// Get local bounding box
    pub fn getLocalBounds(self: *const SoftBodyMotionProperties) AABox {
        return self.local_bounds;
    }

    /// Get the volume of the soft body. Note can become negative if the shape is inside out!
    pub fn getVolume(self: *const SoftBodyMotionProperties) f32 {
        return self.getVolumeTimesSix() / 6.0;
    }

    /// Calculate the total mass and inertia of this body based on the current state of the vertices
    pub fn calculateMassAndInertia(self: *SoftBodyMotionProperties) void {
        var mp: MassProperties = .{};

        for (self.vertices.items) |*v| {
            if (v.inv_mass > 0.0) {
                const pos: [3]f32 = .{ v.position.getX(), v.position.getY(), v.position.getZ() };

                // Accumulate mass
                const mass = 1.0 / v.inv_mass;
                mp.mass += mass;

                // Inertia tensor, diagonal
                // See equations https://en.wikipedia.org/wiki/Moment_of_inertia section 'Inertia Tensor'
                for (0..3) |i| {
                    const ii: u32 = @intCast(i);
                    mp.inertia.set(ii, ii, mp.inertia.get(ii, ii) + mass * (math.square(pos[(i + 1) % 3]) + math.square(pos[(i + 2) % 3])));
                }

                // Inertia tensor off diagonal
                for (0..3) |i| {
                    for (0..3) |j| {
                        if (i != j) {
                            const ii: u32 = @intCast(i);
                            const jj: u32 = @intCast(j);
                            mp.inertia.set(ii, jj, mp.inertia.get(ii, jj) - mass * pos[i] * pos[j]);
                        }
                    }
                }
            } else {
                // If one vertex is kinematic, the entire body will have infinite mass and inertia
                self.base.setInverseMass(0.0);
                self.base.setInverseInertia(Vec3.zero(), Quat.identity());
                return;
            }
        }

        self.base.setMassProperties(.all, &mp);
    }

    // TODO(debug_renderer): DrawVertices, DrawVertexVelocities, DrawEdgeConstraints, DrawRods, DrawRodStates,
    // DrawRodBendTwistConstraints, DrawBendConstraints, DrawVolumeConstraints, DrawSkinConstraints, DrawLRAConstraints,
    // DrawPredictedBounds and the DrawConstraints helper (group D4)

    /// Saving state for replay
    pub fn saveState(self: *const SoftBodyMotionProperties, stream: *StateRecorder) void {
        _ = .{ self, stream };
        @panic("SoftBodyMotionProperties.saveState: not ported yet (Phase 9 group S4)");
    }

    /// Restoring state for replay
    pub fn restoreState(self: *SoftBodyMotionProperties, stream: *StateRecorder) void {
        _ = .{ self, stream };
        @panic("SoftBodyMotionProperties.restoreState: not ported yet (Phase 9 group S4)");
    }

    /// Skin vertices to supplied joints, information is used by the skinned constraints.
    /// @param center_of_mass_transform Value of Body::GetCenterOfMassTransform().
    /// @param joint_matrices The joint matrices must be expressed relative to center_of_mass_transform (the slice length is Jolt's inNumJoints, used only for validating out of bounds).
    /// @param hard_skin_all Can be used to position all vertices on the skinned vertices and can be used to hard reset the soft body.
    /// @param temp_allocator Allocator.
    pub fn skinVertices(self: *SoftBodyMotionProperties, center_of_mass_transform: RMat44, joint_matrices: []const Mat44, hard_skin_all: bool, temp_allocator: TempAllocator) TempAllocator.Error!void {
        _ = .{ self, center_of_mass_transform, joint_matrices, hard_skin_all, temp_allocator };
        @panic("SoftBodyMotionProperties.skinVertices: not ported yet (Phase 9 group S4)");
    }

    /// This function allows you to update the soft body immediately without going through the PhysicsSystem.
    /// This is useful if the soft body is teleported and needs to 'settle' or it can be used if a the soft body
    /// is not added to the PhysicsSystem and needs to be updated manually. One reason for not adding it to the
    /// PhysicsSystem is that you might want to update a soft body immediately after updating an animated object
    /// that has the soft body attached to it. If the soft body is added to the PhysicsSystem it will be updated
    /// by it, so calling this function will effectively update it twice. Note that when you use this function,
    /// only the current thread will be used, whereas if you update through the PhysicsSystem, multiple threads may
    /// be used.
    /// Note that this will bypass any sleep checks. Since the dynamic objects that the soft body touches
    /// will not move during this call, there can be simulation artifacts if you call this function multiple times
    /// without running the physics simulation step.
    pub fn customUpdate(self: *SoftBodyMotionProperties, delta_time: f32, soft_body: *Body, system: *PhysicsSystem) Allocator.Error!void {
        _ = .{ self, delta_time, soft_body, system };
        @panic("SoftBodyMotionProperties.customUpdate: not ported yet (Phase 9 group S4)");
    }

    ////////////////////////////////////////////////////////////
    // FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE ONLY
    ////////////////////////////////////////////////////////////

    /// Initialize the update context. Not part of the public API.
    pub fn initializeUpdateContext(self: *SoftBodyMotionProperties, delta_time: f32, soft_body: *Body, system: *const PhysicsSystem, context: *SoftBodyUpdateContext) void {
        _ = .{ self, delta_time, soft_body, system, context };
        @panic("SoftBodyMotionProperties.initializeUpdateContext: not ported yet (Phase 9 group S4)");
    }

    /// Do a broad phase check and collect all bodies that can possibly collide with this soft body. Not part of the public API.
    pub fn determineCollidingShapes(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, system: *const PhysicsSystem, body_lock_interface: *const BodyLockInterface) Allocator.Error!void {
        _ = .{ self, context, system, body_lock_interface };
        @panic("SoftBodyMotionProperties.determineCollidingShapes: not ported yet (Phase 9 group S4)");
    }

    /// Update the soft body, will process a batch of work. Not part of the public API.
    pub fn parallelUpdate(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext, physics_settings: *const PhysicsSettings) Status {
        _ = .{ self, context, physics_settings };
        @panic("SoftBodyMotionProperties.parallelUpdate: not ported yet (Phase 9 group S4)");
    }

    /// Update the velocities of all rigid bodies that we collided with. Not part of the public API.
    pub fn updateRigidBodyVelocities(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, body_interface: *BodyInterface) void {
        _ = .{ self, context, body_interface };
        @panic("SoftBodyMotionProperties.updateRigidBodyVelocities: not ported yet (Phase 9 group S4)");
    }

    /// Set a flag to indicate that the ContactListener::OnSoftBodyContactAdded should be called
    pub fn requestContactCallback(self: *SoftBodyMotionProperties) void {
        self.need_contact_callback.store(true, .monotonic);
    }

    /// Returns 6 times the volume of the soft body (private in Jolt)
    pub fn getVolumeTimesSix(self: *const SoftBodyMotionProperties) f32 {
        var six_volume: f32 = 0.0;
        for (self.settings.get().?.faces.items) |*f| {
            const x1 = self.vertices.items[f.vertex[0]].position;
            const x2 = self.vertices.items[f.vertex[1]].position;
            const x3 = self.vertices.items[f.vertex[2]].position;
            six_volume += x1.cross(x2).dot(x3); // We pick zero as the origin as this is the center of the bounding box so should give good accuracy
        }
        return six_volume;
    }
};

/// Optimized looking settings for the tests: one hand-made update group, so that `initialize` accepts them
fn testSettings(allocator: Allocator, num_rods: usize, skinned: bool) !*SoftBodySharedSettings {
    const s = try SoftBodySharedSettings.create(allocator);
    errdefer {
        s.deinit();
        allocator.destroy(s);
    }
    const a = allocator;
    try s.vertices.append(a, .init(.{ .x = 1, .y = 0, .z = 0 }, .{ .velocity = .{ .x = 0, .y = 1, .z = 0 } }));
    try s.vertices.append(a, .init(.{ .x = 0, .y = 1, .z = 0 }, .{ .inv_mass = 0.5 }));
    try s.vertices.append(a, .init(.{ .x = 0, .y = 0, .z = 1 }, .{ .inv_mass = 0.25 }));
    try s.addFace(.init(0, 1, 2, .{}));
    for (0..num_rods) |_| {
        var rod: SoftBodySharedSettings.RodStretchShear = .init(0, 1, .{});
        rod.bishop = Quat.identity();
        try s.rod_stretch_shear_constraints.append(a, rod);
    }
    if (skinned) try s.skinned_constraints.append(a, .init(1, 0.5, 0.25, 1.0));
    try s.update_groups.append(a, .{ .edge_end_index = 0, .lra_end_index = 0, .rod_stretch_shear_end_index = @intCast(num_rods), .rod_bend_twist_end_index = 0, .dihedral_bend_end_index = 0, .volume_end_index = 0, .skinned_end_index = @intFromBool(skinned) });
    return s;
}

test "SoftBodyMotionProperties: layout and initialize" {
    const allocator = std.testing.allocator;
    const expectEqual = std.testing.expectEqual;

    // RodState: the rotation and the union of 16 bytes each
    try expectEqual(@as(usize, 32), @sizeOf(SoftBodyMotionProperties.RodState));
    try expectEqual(@as(usize, 16), @offsetOf(SoftBodyMotionProperties.RodState, "u"));
    // The debug-only member costs nothing without the debug renderer
    if (!Core.debug_renderer) try expectEqual(@as(usize, 0), @sizeOf(@FieldType(SoftBodyMotionProperties, "skin_state_transform")));

    const shared = try testSettings(allocator, 2, true);
    var creation: SoftBodyCreationSettings = .init(shared, .zero(), Quat.rotation(Vec3.axisZ(), 0.5 * std.math.pi), 0);
    defer creation.deinit();
    creation.num_iterations = 7;
    creation.pressure = 3.0;
    creation.vertex_radius = 0.125;
    creation.update_position = false;
    creation.faces_double_sided = true;

    var mp: SoftBodyMotionProperties = .{ .allocator = allocator };
    defer mp.deinit();
    if (Core.enable_asserts) mp.base.cached_body_type = .soft_body; // Body.applySoftBodyCreationSettings sets these
    if (Core.enable_asserts) mp.base.cached_motion_type = .dynamic;
    try mp.initialize(&creation);
    try std.testing.expect(mp.getSettings() == shared);
    try expectEqual(@as(u32, 7), mp.getNumIterations());
    try expectEqual(@as(f32, 3.0), mp.getPressure());
    try expectEqual(@as(f32, 0.125), mp.getVertexRadius());
    try std.testing.expect(!mp.getUpdatePosition() and mp.getFacesDoubleSided());
    try expectEqual(@as(usize, 3), mp.getVertices().len);
    try expectEqual(@as(usize, 2), mp.rod_states.items.len);
    try expectEqual(@as(usize, 3), mp.skin_state.items.len);
    try std.testing.expect(mp.skin_state.items[0].position.isNaN());

    // The rotation is baked in the vertices: (1, 0, 0) turns to (0, 1, 0) around Z
    const v0 = mp.getVertex(0);
    try std.testing.expect(v0.position.isClose(Vec3.axisY(), .{}));
    try std.testing.expect(v0.previous_position.eql(v0.position));
    try std.testing.expect(v0.velocity.isClose(Vec3.axisX().negate(), .{}));
    try expectEqual(@as(i32, -1), v0.colliding_shape_index);
    try std.testing.expect(!v0.has_contact);
    try expectEqual(@as(f32, 0.5), mp.getVertex(1).inv_mass);
    try std.testing.expect(mp.getLocalBounds().eql(mp.local_predicted_bounds));
    try std.testing.expect(mp.getRodAngularVelocity(1).eql(Vec3.zero()));
    try expectEqual(@as(usize, 1), mp.getFaces().len);
    try std.testing.expect(mp.getMaterials()[0].get() == PhysicsMaterial.default);

    // Mass 1 + 2 + 4
    try expectEqual(@as(f32, 1.0 / 7.0), mp.base.getInverseMass());

    // A second call resizes: fewer vertices, no rods and no skinning keep the old rod and skin states (Jolt)
    const bounds_before = mp.getLocalBounds();
    const smaller = try SoftBodySharedSettings.create(allocator);
    try smaller.vertices.append(allocator, .init(.{ .x = 2, .y = 0, .z = 0 }, .{ .inv_mass = 0.0 }));
    try smaller.update_groups.append(allocator, std.mem.zeroes(SoftBodySharedSettings.UpdateGroup));
    var creation2: SoftBodyCreationSettings = .init(smaller, .zero(), Quat.identity(), 0);
    defer creation2.deinit();
    try mp.initialize(&creation2);
    try expectEqual(@as(usize, 1), mp.getVertices().len);
    try expectEqual(@as(usize, 2), mp.rod_states.items.len);
    try expectEqual(@as(usize, 3), mp.skin_state.items.len);
    try std.testing.expect(mp.getSettings() == smaller);
    // Jolt does not reset the local bounds: the new vertex is added to the bounds of the old vertices
    var expected_bounds = bounds_before;
    expected_bounds.encapsulateVec3(Vec3.init(2, 0, 0));
    try std.testing.expect(mp.getLocalBounds().eql(expected_bounds));
    try std.testing.expect(mp.local_predicted_bounds.eql(expected_bounds));
    try std.testing.expect(mp.getLocalBounds().min.getX() < -0.5); // The old vertex at (-1, 0, 0)
    // A kinematic vertex makes the mass infinite
    try expectEqual(@as(f32, 0.0), mp.base.getInverseMass());

    // requestContactCallback
    try std.testing.expect(!mp.need_contact_callback.load(.monotonic));
    mp.requestContactCallback();
    try std.testing.expect(mp.need_contact_callback.load(.monotonic));
}

test "SoftBodyMotionProperties: initialize out of memory leaves the state unchanged and leaks nothing" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            const shared = try testSettings(allocator, 1, true);
            var creation: SoftBodyCreationSettings = .init(shared, .zero(), Quat.identity(), 0);
            defer creation.deinit();
            var mp: SoftBodyMotionProperties = .{ .allocator = allocator };
            defer mp.deinit();
            mp.initialize(&creation) catch |err| {
                // Nothing changed
                try std.testing.expect(mp.getSettings() == null);
                try std.testing.expectEqual(@as(usize, 0), mp.vertices.items.len);
                return err;
            };
            try mp.initialize(&creation); // A second call with the same settings does not allocate
        }
    }.f, .{});
}
