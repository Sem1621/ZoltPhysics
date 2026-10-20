//! Port of: Jolt/Physics/SoftBody/SoftBodyMotionProperties.h, Jolt/Physics/SoftBody/SoftBodyMotionProperties.cpp
//! Status: complete
//!
//! RemainingFeaturesArchitecture.md R15, R16, R17 and SimulationArchitecture D1:
//! - `SoftBodyMotionProperties : public MotionProperties` embeds its base as `base` (the first field), then Jolt's
//!   fields in Jolt's order. `fromBase` / `fromBaseConst` are Jolt's `static_cast<SoftBodyMotionProperties *>`.
//! - `allocator` (Zolt) is the allocator of the BodyManager that created the soft body. It allocates the arrays.
//!   `deinit` is Jolt's `~SoftBodyMotionProperties`: it frees the arrays and releases the shared settings and the leaf
//!   shapes of the colliding shapes and sensors.
//! - `RodState` is an extern struct with an extern union: IntegratePositions reuses the storage of the angular velocity
//!   for the previous rotation, which is defined for an extern union.
//! - `mSkinStateTransform` only exists with JPH_DEBUG_RENDERER: `skin_state_transform` is `void` without the option. Its
//!   two writes (SkinVertices, UpdateSoftBodyState) are `if (Core.debug_renderer)` statements.
//! - `mNeedContactCallback` is a `std.atomic.Value(bool)`, Jolt's relaxed order is `.monotonic`. `mNumSensors` (Jolt's
//!   TSAN workaround) is a `std.atomic.Value(u32)`; Jolt leaves it uninitialized until DetermineCollidingShapes, Zolt
//!   starts it at 0. The update context's atomics use Jolt's memory orders (relaxed is `.monotonic`).
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
//! - `determineCollidingShapes` grows the colliding shape, sensor and leaf shape arrays with `allocator` (R16: a heap
//!   allocation inside the SoftBodyCollide job). It returns `error.OutOfMemory` when an allocation fails: it stops the
//!   broad phase query, keeps what it collected (the lists are valid but partial, R17) and still stores the number of
//!   sensors. The PhysicsSystem latches the error.
//! - Zolt (R20, UB1): a soft body without vertices. Jolt's state machine never leaves DetermineCollisionPlanes then (a
//!   job spins forever, CustomUpdate asserts and continues with an uninitialized delta position). In Zolt the first
//!   claimant of the empty vertex range advances the state, as the last batch would. The body simulates nothing.
//! - `customUpdate` is literal: a single-threaded loop through the same state machine, with the locking interfaces. It
//!   returns `error.OutOfMemory` when `determineCollidingShapes` cannot allocate. Then it clears the colliding shapes and
//!   sensors and returns before the simulation: the vertices are unchanged. Its `setPosition` is Jolt's. A body that was
//!   created but never added has an ID, so the lock succeeds and the body moves. Only the broad phase update and the
//!   activation need a body in the broad phase. A body without an ID (`createSoftBodyWithoutID`) does not move: the
//!   lock fails.
//! - `skinVertices` takes the joint matrices as a slice (Jolt: a pointer and `inNumJoints`, only used by an assert). It
//!   returns `TempAllocator.Error` before it changes anything (Jolt's TempAllocatorImpl aborts).
//! - `clearCollidingShapesAndSensors` (Zolt) is the end of UpdateRigidBodyVelocities: it releases the leaf shapes. The
//!   SoftBodyFinalize job of a poisoned update calls it too (R17).
//! - JPH_DEBUG_RENDERER (DrawVertices, DrawVertexVelocities, the Draw*Constraints functions, DrawRods, DrawRodStates,
//!   DrawPredictedBounds and the DrawConstraints helper): TODO(debug_renderer), group D4.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const trig = @import("../../Math/Trigonometry.zig");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const TempAllocatorFile = @import("../../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const Body = @import("../Body/Body.zig").Body;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const MotionProperties = @import("../Body/MotionProperties.zig").MotionProperties;
const MassProperties = @import("../Body/MassProperties.zig").MassProperties;
const BodyInterface = @import("../Body/BodyInterface.zig").BodyInterface;
const BodyLockInterface = @import("../Body/BodyLockInterface.zig").BodyLockInterface;
const BodyLockRead = @import("../Body/BodyLock.zig").BodyLockRead;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const SubShapeIDCreator = @import("../Collision/Shape/SubShapeID.zig").SubShapeIDCreator;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const CollideShapeBodyCollector = @import("../Collision/BroadPhase/BroadPhaseQuery.zig").CollideShapeBodyCollector;
const CollideSoftBodyVertexIterator = @import("../Collision/CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const SimShapeFilterWrapper = @import("../Collision/SimShapeFilterWrapper.zig").SimShapeFilterWrapper;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const ContactConstraintManager = @import("../Constraints/ContactConstraintManager.zig").ContactConstraintManager;
const PhysicsSettings = @import("../PhysicsSettings.zig").PhysicsSettings;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const SoftBodyCreationSettings = @import("SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;
const SoftBodyVertex = @import("SoftBodyVertex.zig").SoftBodyVertex;
const SoftBodyUpdateContext = @import("SoftBodyUpdateContext.zig").SoftBodyUpdateContext;
const SoftBodyContactSettings = @import("SoftBodyContactListener.zig").SoftBodyContactSettings;
const SoftBodyContactListener = @import("SoftBodyContactListener.zig").SoftBodyContactListener;
const SoftBodyValidateResult = @import("SoftBodyContactListener.zig").SoftBodyValidateResult;
const SoftBodyManifold = @import("SoftBodyManifold.zig").SoftBodyManifold;

/// This class contains the runtime information of a soft body.
//
// Based on: XPBD, Extended Position Based Dynamics, Matthias Muller, Ten Minute Physics
// See: https://matthias-research.github.io/pages/tenMinutePhysics/09-xpbd.pdf
pub const SoftBodyMotionProperties = struct {
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
        self.base.saveState(stream);

        for (self.vertices.items) |*v| {
            stream.write(v.position);
            stream.write(v.velocity);
        }

        for (self.rod_states.items) |*r| {
            stream.write(r.rotation);
            stream.write(r.u.angular_velocity);
        }

        for (self.skin_state.items) |*s| {
            stream.write(s.previous_position);
            stream.write(s.position);
            stream.write(s.normal);
        }

        stream.write(self.local_bounds.min);
        stream.write(self.local_bounds.max);
        stream.write(self.local_predicted_bounds.min);
        stream.write(self.local_predicted_bounds.max);
    }

    /// Restoring state for replay
    pub fn restoreState(self: *SoftBodyMotionProperties, stream: *StateRecorder) void {
        self.base.restoreState(stream);

        for (self.vertices.items) |*v| {
            stream.read(&v.position);
            stream.read(&v.velocity);
        }

        for (self.rod_states.items) |*r| {
            stream.read(&r.rotation);
            stream.read(&r.u.angular_velocity);
        }

        for (self.skin_state.items) |*s| {
            stream.read(&s.previous_position);
            stream.read(&s.position);
            stream.read(&s.normal);
        }

        stream.read(&self.local_bounds.min);
        stream.read(&self.local_bounds.max);
        stream.read(&self.local_predicted_bounds.min);
        stream.read(&self.local_predicted_bounds.max);
    }

    /// Skin vertices to supplied joints, information is used by the skinned constraints.
    /// @param center_of_mass_transform Value of Body::GetCenterOfMassTransform().
    /// @param joint_matrices The joint matrices must be expressed relative to center_of_mass_transform (the slice length is Jolt's inNumJoints, used only for validating out of bounds).
    /// @param hard_skin_all Can be used to position all vertices on the skinned vertices and can be used to hard reset the soft body.
    /// @param temp_allocator Allocator.
    pub fn skinVertices(self: *SoftBodyMotionProperties, center_of_mass_transform: RMat44, joint_matrices: []const Mat44, hard_skin_all: bool, temp_allocator: TempAllocator) TempAllocator.Error!void {
        const settings = self.settings.get().?;

        // Calculate the skin matrices
        const num_skin_matrices: u32 = @intCast(settings.inv_bind_matrices.items.len);
        var skin_matrices = try TempArray(Mat44).alloc(temp_allocator, num_skin_matrices);
        defer skin_matrices.free(temp_allocator);
        for (skin_matrices.items, settings.inv_bind_matrices.items) |*s, *inv_bind_matrix| {
            if (Core.enable_asserts) std.debug.assert(inv_bind_matrix.joint_index < joint_matrices.len);
            s.* = joint_matrices[inv_bind_matrix.joint_index].mul(inv_bind_matrix.inv_bind);
        }

        // Skin the vertices
        if (Core.debug_renderer) self.skin_state_transform = center_of_mass_transform;
        const num_vertices: u32 = @intCast(settings.vertices.items.len);
        if (Core.enable_asserts) std.debug.assert(self.skin_state.items.len == num_vertices);
        const in_vertices = settings.vertices.items;
        for (settings.skinned_constraints.items) |*s| {
            // Get bind pose
            if (Core.enable_asserts) std.debug.assert(s.vertex < num_vertices);
            const bind_pos = Vec3.loadFloat3Unsafe(&in_vertices[s.vertex].position);

            // Skin vertex
            var pos = Vec3.zero();
            for (s.weights) |w| {
                // We assume that the first zero weight is the end of the list
                if (w.weight == 0.0)
                    break;

                if (Core.enable_asserts) std.debug.assert(w.inv_bind_index < num_skin_matrices);
                pos = pos.add(skin_matrices.items[w.inv_bind_index].mulVec3(bind_pos).mulScalar(w.weight));
            }
            const skin_state = &self.skin_state.items[s.vertex];
            skin_state.previous_position = skin_state.position;
            skin_state.position = pos;
        }

        // Calculate the normals
        for (settings.skinned_constraints.items) |*s| {
            var normal = Vec3.zero();
            const num_faces = s.normal_info >> 24;
            if (num_faces > 0) {
                // Calculate normal
                const first = s.normal_info & 0xffffff;
                for (settings.skinned_constraint_normals.items[first .. first + num_faces]) |f| {
                    const face = &settings.faces.items[f];
                    const v0 = self.skin_state.items[face.vertex[0]].position;
                    const v1 = self.skin_state.items[face.vertex[1]].position;
                    const v2 = self.skin_state.items[face.vertex[2]].position;
                    normal = normal.add(v1.sub(v0).cross(v2.sub(v0)).normalizedOr(Vec3.zero()));
                }
                normal = normal.normalizedOr(Vec3.zero());
            }
            self.skin_state.items[s.vertex].normal = normal;
        }

        if (hard_skin_all) {
            // Hard skin all vertices and reset their velocities
            for (settings.skinned_constraints.items) |*s| {
                const vertex = &self.vertices.items[s.vertex];
                const skin_state = &self.skin_state.items[s.vertex];
                skin_state.previous_position = skin_state.position;
                vertex.position = skin_state.position;
                vertex.velocity = Vec3.zero();
            }
        } else if (!self.enable_skin_constraints) {
            // Hard skin only the kinematic vertices as we will not solve the skin constraints later
            for (settings.skinned_constraints.items) |*s| {
                if (s.max_distance == 0.0) {
                    const vertex = &self.vertices.items[s.vertex];
                    vertex.position = self.skin_state.items[s.vertex].position;
                }
            }
        }

        // Indicate that the previous positions are valid for the coming update
        self.skin_state_previous_position_valid = true;
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
    /// (Zolt: `error.OutOfMemory` when the colliding shapes cannot be stored. The colliding shapes are cleared then and
    /// the soft body is not simulated.)
    pub fn customUpdate(self: *SoftBodyMotionProperties, delta_time: f32, soft_body: *Body, system: *PhysicsSystem) Allocator.Error!void {
        // Create update context
        var context: SoftBodyUpdateContext = .{};
        self.initializeUpdateContext(delta_time, soft_body, system, &context);

        // Determine bodies we're colliding with
        self.determineCollidingShapes(&context, system, system.getBodyLockInterface()) catch |err| {
            // Zolt: do not simulate with a partial list of colliding shapes, release their references
            self.clearCollidingShapesAndSensors();
            return err;
        };

        // Call the internal update until it finishes
        const settings = system.getPhysicsSettings();
        var status = self.parallelUpdate(&context, settings);
        while (status == .did_work)
            status = self.parallelUpdate(&context, settings);
        if (Core.enable_asserts) std.debug.assert(status == .done);

        // Update the state of the bodies we've collided with
        self.updateRigidBodyVelocities(&context, system.getBodyInterfaceMut());

        // Update position of the soft body
        if (self.update_position)
            system.getBodyInterfaceMut().setPosition(soft_body.getID(), soft_body.getPosition().addVec3(context.delta_position), .dont_activate);
    }

    ////////////////////////////////////////////////////////////
    // FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE ONLY
    ////////////////////////////////////////////////////////////

    /// Initialize the update context. Not part of the public API.
    pub fn initializeUpdateContext(self: *SoftBodyMotionProperties, delta_time: f32, soft_body: *Body, system: *const PhysicsSystem, context: *SoftBodyUpdateContext) void {
        // Store body
        context.body = soft_body;
        context.motion_properties = self;
        context.contact_listener = system.getSoftBodyContactListener();
        context.sim_shape_filter = system.getSimShapeFilter();

        // Convert gravity to local space
        context.center_of_mass_transform = soft_body.getCenterOfMassTransform();
        context.gravity = context.center_of_mass_transform.multiply3x3Transposed(system.getGravity().mulScalar(self.base.getGravityFactor()));

        // Calculate delta time for sub step
        context.delta_time = delta_time;
        context.sub_step_delta_time = delta_time / @as(f32, @floatFromInt(self.num_iterations));

        // Calculate total displacement we'll have due to gravity over all sub steps
        // The total displacement as produced by our integrator can be written as: Sum(i * g * dt^2, i = 0..mNumIterations).
        // This is bigger than 0.5 * g * dt^2 because we first increment the velocity and then update the position
        // Using Sum(i, i = 0..n) = n * (n + 1) / 2 we can write this as:
        const num_iterations: f32 = @floatFromInt(self.num_iterations);
        const num_iterations_plus_one: f32 = @floatFromInt(self.num_iterations +% 1);
        context.displacement_due_to_gravity = context.gravity.mulScalar(0.5 * num_iterations * num_iterations_plus_one * math.square(context.sub_step_delta_time));
    }

    /// Do a broad phase check and collect all bodies that can possibly collide with this soft body. Not part of the public API.
    /// (Zolt: `error.OutOfMemory` when the colliding shapes cannot be stored, the lists keep what was collected)
    pub fn determineCollidingShapes(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, system: *const PhysicsSystem, body_lock_interface: *const BodyLockInterface) Allocator.Error!void {
        const Collector = struct {
            pub const overrides = .{.addHit};

            base: CollideShapeBodyCollector = .init(@This()),
            context: *const SoftBodyUpdateContext,
            inverse_transform: RMat44,
            local_bounds: AABox,
            world_bounds: AABox,
            body_lock_interface: *const BodyLockInterface,
            combine_friction: ContactConstraintManager.CombineFunction,
            combine_restitution: ContactConstraintManager.CombineFunction,
            shape_filter: *SimShapeFilterWrapper,
            hits: *std.ArrayList(CollidingShape),
            sensors: *std.ArrayList(CollidingSensor),
            /// Zolt: grows the lists
            allocator: Allocator,
            /// Zolt: an allocation failed, the query stops
            out_of_memory: bool = false,

            pub fn addHit(c: *@This(), result: *const BodyID) void {
                var lock: BodyLockRead = .init(c.body_lock_interface, result.*);
                defer lock.deinit();
                if (lock.succeeded()) {
                    const soft_body: *const Body = c.context.body;
                    const body = lock.getBody();
                    if (body.isRigidBody() // TODO: We should support soft body vs soft body
                    and soft_body.getCollisionGroup().canCollide(body.getCollisionGroup()) and c.world_bounds.overlaps(body.getWorldSpaceBounds().*)) // In the broadphase we widen the bounding box when a body moves, do a final check to see if the bounding boxes actually overlap
                    {
                        var settings: SoftBodyContactSettings = .{};
                        settings.is_sensor = body.isSensor();

                        if (c.context.contact_listener) |listener| {
                            // Call the contact listener to see if we should accept this contact
                            if (listener.onSoftBodyContactValidate(soft_body, body, &settings) != .accept_contact)
                                return;

                            // Check if there will be any interaction
                            if (!settings.is_sensor and settings.inv_mass_scale1 == 0.0 and (body.getMotionType() != .dynamic or settings.inv_mass_scale2 == 0.0))
                                return;
                        } else {
                            // If we have no contact listener, we can ignore sensors
                            if (settings.is_sensor)
                                return;
                        }

                        // Calculate transform of this body relative to the soft body
                        const com = c.inverse_transform.mul(body.getCenterOfMassTransform()).toMat44();

                        // Collect leaf shapes
                        c.shape_filter.setBody2(body);
                        const LeafShapeCollector = struct {
                            pub const overrides = .{.addHit};

                            base: TransformedShapeCollector = .init(@This()),
                            hits: std.ArrayList(LeafShape) = .empty,
                            /// Zolt: grows `hits`
                            allocator: Allocator,
                            /// Zolt: an allocation failed, the collection stops
                            out_of_memory: bool = false,

                            pub fn addHit(lc: *@This(), leaf: *const TransformedShape) void {
                                lc.hits.ensureUnusedCapacity(lc.allocator, 1) catch {
                                    lc.out_of_memory = true;
                                    lc.base.forceEarlyOut();
                                    return;
                                };
                                lc.hits.appendAssumeCapacity(.init(Mat44.rotationTranslation(leaf.shape_rotation, leaf.shape_position_com.toVec3()), leaf.getShapeScale(), leaf.shape.get().?));
                            }
                        };
                        var collector: LeafShapeCollector = .{ .allocator = c.allocator };
                        body.getShape().collectTransformedShapes(c.local_bounds, com.getTranslation(), com.getQuaternion(), Vec3.one(), .{}, &collector.base, c.shape_filter.getFilter());
                        if (collector.out_of_memory) {
                            c.outOfMemory(&collector.hits);
                            return;
                        }
                        if (collector.hits.items.len == 0)
                            return;

                        if (settings.is_sensor) {
                            c.sensors.ensureUnusedCapacity(c.allocator, 1) catch {
                                c.outOfMemory(&collector.hits);
                                return;
                            };
                            c.sensors.appendAssumeCapacity(.{
                                .center_of_mass_transform = com,
                                .shapes = collector.hits,
                                .body_id = result.*,
                                .has_contact = false, // Jolt leaves it uninitialized, DetermineSensorCollisions sets it
                            });
                        } else {
                            c.hits.ensureUnusedCapacity(c.allocator, 1) catch {
                                c.outOfMemory(&collector.hits);
                                return;
                            };
                            // The members that Jolt leaves uninitialized for a body that is not dynamic are undefined
                            var cs: CollidingShape = .{
                                .center_of_mass_transform = com,
                                .shapes = collector.hits,
                                .body_id = result.*,
                                .motion_type = body.getMotionType(),
                                .inv_mass = undefined,
                                .friction = c.combine_friction(soft_body, .empty, body, .empty),
                                .restitution = c.combine_restitution(soft_body, .empty, body, .empty),
                                .soft_body_inv_mass_scale = settings.inv_mass_scale1,
                                .update_velocities = false,
                                .inv_inertia = undefined,
                                .linear_velocity = undefined,
                                .angular_velocity = undefined,
                                .original_linear_velocity = undefined,
                                .original_angular_velocity = undefined,
                            };
                            if (cs.motion_type == .dynamic) {
                                const mp = body.getMotionProperties();
                                cs.inv_mass = settings.inv_mass_scale2 * mp.getInverseMass();
                                cs.inv_inertia = mp.getInverseInertiaForRotation(cs.center_of_mass_transform.getRotation()).mulScalar(settings.inv_inertia_scale2);
                                cs.linear_velocity = c.inverse_transform.multiply3x3(mp.getLinearVelocity());
                                cs.original_linear_velocity = cs.linear_velocity;
                                cs.angular_velocity = c.inverse_transform.multiply3x3(mp.getAngularVelocity());
                                cs.original_angular_velocity = cs.angular_velocity;
                            }
                            c.hits.appendAssumeCapacity(cs);
                        }
                    }
                }
            }

            /// Zolt: release the leaf shapes that could not be stored, latch the error and stop the query
            fn outOfMemory(c: *@This(), leaf_shapes: *std.ArrayList(LeafShape)) void {
                for (leaf_shapes.items) |*s| s.deinit();
                leaf_shapes.deinit(c.allocator);
                c.out_of_memory = true;
                c.base.forceEarlyOut();
            }
        };

        // Calculate local bounding box
        var local_bounds = self.local_bounds;
        local_bounds.encapsulate(self.local_predicted_bounds);
        local_bounds.expandBy(Vec3.replicate(self.vertex_radius));

        // Calculate world space bounding box
        const world_bounds = local_bounds.transformedRMat44(context.center_of_mass_transform);

        // Create shape filter
        var shape_filter: SimShapeFilterWrapper = .init(context.sim_shape_filter, context.body);

        var collector: Collector = .{
            .context = context,
            .inverse_transform = context.center_of_mass_transform.inversedRotationTranslation(),
            .local_bounds = local_bounds,
            .world_bounds = world_bounds,
            .body_lock_interface = body_lock_interface,
            .combine_friction = system.getCombineFriction(),
            .combine_restitution = system.getCombineRestitution(),
            .shape_filter = &shape_filter,
            .hits = &self.colliding_shapes,
            .sensors = &self.colliding_sensors,
            .allocator = self.allocator,
        };
        const layer = context.body.getObjectLayer();
        const broadphase_layer_filter = system.getDefaultBroadPhaseLayerFilter(layer);
        const object_layer_filter = system.getDefaultLayerFilter(layer);
        system.getBroadPhaseQuery().collideAABox(&world_bounds, &collector.base, .{ .broad_phase_layer_filter = &broadphase_layer_filter.base, .object_layer_filter = &object_layer_filter.base });
        self.num_sensors.store(@intCast(self.colliding_sensors.items.len), .monotonic); // Workaround for TSAN false positive: store mCollidingSensors.size() in a separate variable.
        if (collector.out_of_memory)
            return error.OutOfMemory;
    }

    /// Update the soft body, will process a batch of work. Not part of the public API.
    pub fn parallelUpdate(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext, physics_settings: *const PhysicsSettings) Status {
        // Jolt's default case (JPH_ASSERT(false), NoWork) cannot happen: the switch covers every state
        return switch (context.state.load(.acquire)) {
            .determine_collision_planes => self.parallelDetermineCollisionPlanes(context),
            .determine_sensor_collisions => self.parallelDetermineSensorCollisions(context),
            .apply_constraints => self.parallelApplyConstraints(context, physics_settings),
            .done => .done,
        };
    }

    /// Update the velocities of all rigid bodies that we collided with. Not part of the public API.
    pub fn updateRigidBodyVelocities(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, body_interface: *BodyInterface) void {
        // Write back velocity deltas
        for (self.colliding_shapes.items) |*cs| {
            if (cs.update_velocities)
                body_interface.addLinearAndAngularVelocity(cs.body_id, context.center_of_mass_transform.multiply3x3(cs.linear_velocity.sub(cs.original_linear_velocity)), context.center_of_mass_transform.multiply3x3(cs.angular_velocity.sub(cs.original_angular_velocity)));
        }

        // Clear colliding shapes/sensors to avoid hanging on to references to shapes
        self.clearCollidingShapesAndSensors();
    }

    /// Zolt: clear the colliding shapes and sensors and release their leaf shapes (`mCollidingShapes.clear();
    /// mCollidingSensors.clear();`, the end of UpdateRigidBodyVelocities). Also used by the SoftBodyFinalize job of a
    /// poisoned update (RemainingFeaturesArchitecture.md R17) and by customUpdate after an allocation failure.
    pub fn clearCollidingShapesAndSensors(self: *SoftBodyMotionProperties) void {
        for (self.colliding_shapes.items) |*cs| cs.deinit(self.allocator);
        self.colliding_shapes.clearRetainingCapacity();
        for (self.colliding_sensors.items) |*cs| cs.deinit(self.allocator);
        self.colliding_sensors.clearRetainingCapacity();
    }

    /// Set a flag to indicate that the ContactListener::OnSoftBodyContactAdded should be called
    pub fn requestContactCallback(self: *SoftBodyMotionProperties) void {
        self.need_contact_callback.store(true, .monotonic);
    }

    /// Do a narrow phase check and determine the closest feature that we can collide with (private in Jolt)
    fn determineCollisionPlanes(self: *SoftBodyMotionProperties, vertex_start: u32, num_vertices: u32) void {
        // Generate collision planes
        for (self.colliding_shapes.items, 0..) |*cs, cs_index| {
            for (cs.shapes.items) |*shape| {
                const vertices: CollideSoftBodyVertexIterator = .initFromVertices(self.vertices.items.ptr + vertex_start);
                shape.shape.get().?.collideSoftBodyVertices(shape.transform, shape.scale, &vertices, num_vertices, @intCast(cs_index));
            }
        }
    }

    /// Do a narrow phase check between a single sensor and the soft body (private in Jolt)
    fn determineSensorCollisions(self: *SoftBodyMotionProperties, sensor: *CollidingSensor) void {
        var collision_plane: Plane = undefined;
        var largest_penetration: f32 = -math.flt_max;
        var colliding_shape_idx: i32 = -1;

        // Collide sensor against all vertices (Jolt: &mVertices[0], also for an empty array: the pointer is not read then)
        const vertices = self.vertices.items.ptr;
        const vertex_iterator: CollideSoftBodyVertexIterator = .init(
            .init(&vertices[0].position, .{ .stride = @sizeOf(SoftBodyVertex) }), // The position and mass come from the soft body vertex
            .init(&vertices[0].inv_mass, .{ .stride = @sizeOf(SoftBodyVertex) }),
            .init(&collision_plane, .{ .stride = 0 }), // We want all vertices to result in a single collision so we pass stride 0
            .init(&largest_penetration, .{ .stride = 0 }),
            .init(&colliding_shape_idx, .{ .stride = 0 }),
        );
        for (sensor.shapes.items) |*shape|
            shape.shape.get().?.collideSoftBodyVertices(shape.transform, shape.scale, &vertex_iterator, @intCast(self.vertices.items.len), 0);
        sensor.has_contact = largest_penetration > 0.0;

        // We need a contact callback if one of the sensors collided
        if (sensor.has_contact)
            self.requestContactCallback();
    }

    /// Apply pressure force and update the vertex velocities (private in Jolt)
    fn applyPressure(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext) void {
        const dt = context.sub_step_delta_time;
        const pressure_coefficient = self.pressure;
        if (pressure_coefficient > 0.0) {
            // Calculate total volume
            const six_volume = self.getVolumeTimesSix();
            if (six_volume > 0.0) {
                // Apply pressure
                // p = F / A = n R T / V (see https://en.wikipedia.org/wiki/Pressure)
                // Our pressure coefficient is n R T so the impulse is:
                // P = F dt = pressure_coefficient / V * A * dt
                const coefficient = pressure_coefficient * dt / six_volume; // Need to still multiply by 6 for the volume
                for (self.settings.get().?.faces.items) |*f| {
                    const x1 = self.vertices.items[f.vertex[0]].position;
                    const x2 = self.vertices.items[f.vertex[1]].position;
                    const x3 = self.vertices.items[f.vertex[2]].position;

                    const impulse = x2.sub(x1).cross(x3.sub(x1)).mulScalar(coefficient); // Area is half the cross product so need to still divide by 2
                    for (f.vertex) |i| {
                        const v = &self.vertices.items[i];
                        v.velocity = v.velocity.add(impulse.mulScalar(v.inv_mass)); // Want to divide by 3 because we spread over 3 vertices
                    }
                }
            }
        }
    }

    /// Integrate the positions of all vertices by 1 sub step (private in Jolt)
    fn integratePositions(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext) void {
        const dt = context.sub_step_delta_time;
        const linear_damping = math.max(@as(f32, 0.0), 1.0 - self.base.getLinearDamping() * dt); // See: MotionProperties::ApplyForceTorqueAndDragInternal

        // Integrate
        const sub_step_gravity = context.gravity.mulScalar(dt);
        const sub_step_impulse = self.base.getAccumulatedForce().mulScalar(dt).divScalar(math.max(@as(f32, @floatFromInt(self.vertices.items.len)), 1.0));
        for (self.vertices.items) |*v| {
            if (v.inv_mass > 0.0) {
                // Gravity
                v.velocity = v.velocity.add(sub_step_gravity.add(sub_step_impulse.mulScalar(v.inv_mass)));

                // Damping
                v.velocity = v.velocity.mulScalar(linear_damping);
            }

            // Integrate
            const position = v.position;
            v.previous_position = position;
            v.position = position.add(v.velocity.mulScalar(dt));
        }

        // Integrate rod orientations
        const half_dt = 0.5 * dt;
        for (self.rod_states.items) |*r| {
            // Damping
            r.u.angular_velocity = r.u.angular_velocity.mulScalar(linear_damping);

            // Integrate
            const rotation = r.rotation;
            const delta_rotation = Quat.multiplyImaginary(r.u.angular_velocity, rotation).mulScalar(half_dt);
            r.u = .{ .previous_rotation_internal = rotation }; // Overwrites mAngularVelocity
            r.rotation = rotation.add(delta_rotation).normalized();
        }
    }

    /// Enforce all bend constraints (private in Jolt)
    fn applyDihedralBendConstraints(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, start_index: u32, end_index: u32) void {
        const inv_dt_sq = 1.0 / math.square(context.sub_step_delta_time);

        for (self.settings.get().?.dihedral_bend_constraints.items[start_index..end_index]) |*b| {
            const v0 = &self.vertices.items[b.vertex[0]];
            const v1 = &self.vertices.items[b.vertex[1]];
            const v2 = &self.vertices.items[b.vertex[2]];
            const v3 = &self.vertices.items[b.vertex[3]];

            // Get positions
            const x0 = v0.position;
            const x1 = v1.position;
            const x2 = v2.position;
            const x3 = v3.position;

            //    x2
            // e1/  \e3
            //  /    \
            // x0----x1
            //  \ e0 /
            // e2\  /e4
            //    x3

            // Calculate the shared edge of the triangles
            const e = x1.sub(x0);
            const e_len = e.length();
            if (e_len < 1.0e-6)
                continue;

            // Calculate the normals of the triangles
            const x1x2 = x2.sub(x1);
            const x1x3 = x3.sub(x1);
            var n1 = x2.sub(x0).cross(x1x2);
            var n2 = x1x3.cross(x3.sub(x0));
            const n1_len_sq = n1.lengthSq();
            const n2_len_sq = n2.lengthSq();
            const n1_len_sq_n2_len_sq = n1_len_sq * n2_len_sq;
            if (n1_len_sq_n2_len_sq < 1.0e-24)
                continue;

            // Calculate constraint equation
            // As per "Strain Based Dynamics" Appendix A we need to negate the gradients when (n1 x n2) . e > 0, instead we make sure that the sign of the constraint equation is correct
            const sign = math.sign(n2.cross(n1).dot(e));
            const d = math.mulRSqrtApproximate(n1.dot(n2), n1_len_sq_n2_len_sq);
            var c = sign * trig.acosApproximate(d) - b.initial_angle;

            // Ensure the range is -PI to PI
            if (c > math.pi) {
                c -= 2.0 * math.pi;
            } else if (c < -math.pi) {
                c += 2.0 * math.pi;
            }

            // Calculate gradient of constraint equation
            // Taken from "Strain Based Dynamics" - Matthias Muller et al. (Appendix A)
            // with p1 = x2, p2 = x3, p3 = x0 and p4 = x1
            // which in turn is based on "Simulation of Clothing with Folds and Wrinkles" - R. Bridson et al. (Section 4)
            n1 = n1.divScalar(n1_len_sq);
            n2 = n2.divScalar(n2_len_sq);
            const d0c = n1.mulScalar(x1x2.dot(e)).add(n2.mulScalar(x1x3.dot(e))).divScalar(e_len);
            const d2c = n1.mulScalar(e_len);
            const d3c = n2.mulScalar(e_len);

            // The sum of the gradients must be zero (see "Strain Based Dynamics" section 4)
            const d1c = d0c.negate().sub(d2c).sub(d3c);

            // Get masses
            const w0 = v0.inv_mass;
            const w1 = v1.inv_mass;
            const w2 = v2.inv_mass;
            const w3 = v3.inv_mass;

            // Calculate -lambda
            const denom = w0 * d0c.lengthSq() + w1 * d1c.lengthSq() + w2 * d2c.lengthSq() + w3 * d3c.lengthSq() + b.compliance * inv_dt_sq;
            if (denom < 1.0e-12)
                continue;
            const minus_lambda = c / denom;

            // Apply correction
            v0.position = x0.sub(d0c.mulScalar(minus_lambda * w0));
            v1.position = x1.sub(d1c.mulScalar(minus_lambda * w1));
            v2.position = x2.sub(d2c.mulScalar(minus_lambda * w2));
            v3.position = x3.sub(d3c.mulScalar(minus_lambda * w3));
        }
    }

    /// Enforce all volume constraints (private in Jolt)
    fn applyVolumeConstraints(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, start_index: u32, end_index: u32) void {
        const inv_dt_sq = 1.0 / math.square(context.sub_step_delta_time);

        // Satisfy volume constraints
        for (self.settings.get().?.volume_constraints.items[start_index..end_index]) |*v| {
            const v1 = &self.vertices.items[v.vertex[0]];
            const v2 = &self.vertices.items[v.vertex[1]];
            const v3 = &self.vertices.items[v.vertex[2]];
            const v4 = &self.vertices.items[v.vertex[3]];

            const x1 = v1.position;
            const x2 = v2.position;
            const x3 = v3.position;
            const x4 = v4.position;

            // Calculate constraint equation
            const x1x2 = x2.sub(x1);
            const x1x3 = x3.sub(x1);
            const x1x4 = x4.sub(x1);
            const c = @abs(x1x2.cross(x1x3).dot(x1x4)) - v.six_rest_volume;

            // Calculate gradient of constraint equation
            const d1c = x4.sub(x2).cross(x3.sub(x2));
            const d2c = x1x3.cross(x1x4);
            const d3c = x1x4.cross(x1x2);
            const d4c = x1x2.cross(x1x3);

            // Get masses
            const w1 = v1.inv_mass;
            const w2 = v2.inv_mass;
            const w3 = v3.inv_mass;
            const w4 = v4.inv_mass;

            // Calculate -lambda
            const denom = w1 * d1c.lengthSq() + w2 * d2c.lengthSq() + w3 * d3c.lengthSq() + w4 * d4c.lengthSq() + v.compliance * inv_dt_sq;
            if (denom < 1.0e-12)
                continue;
            const minus_lambda = c / denom;

            // Apply correction
            v1.position = x1.sub(d1c.mulScalar(minus_lambda * w1));
            v2.position = x2.sub(d2c.mulScalar(minus_lambda * w2));
            v3.position = x3.sub(d3c.mulScalar(minus_lambda * w3));
            v4.position = x4.sub(d4c.mulScalar(minus_lambda * w4));
        }
    }

    /// Enforce all skin constraints (private in Jolt)
    fn applySkinConstraints(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, start_index: u32, end_index: u32) void {
        const settings = self.settings.get().?;

        // Early out if nothing to do
        if (settings.skinned_constraints.items.len == 0 or !self.enable_skin_constraints)
            return;

        // We're going to iterate multiple times over the skin constraints, update the skinned position accordingly.
        // If we don't do this, the simulation will see a big jump and the first iteration will cause a big velocity change in the system.
        const factor: f32 = if (self.skin_state_previous_position_valid) @as(f32, @floatFromInt(context.next_iteration.load(.monotonic))) / @as(f32, @floatFromInt(self.num_iterations)) else 1.0;
        const prev_factor = 1.0 - factor;

        // Apply the constraints
        const vertices = self.vertices.items;
        const skin_states = self.skin_state.items;
        for (settings.skinned_constraints.items[start_index..end_index]) |*s| {
            const vertex = &vertices[s.vertex];
            const skin_state = &skin_states[s.vertex];
            const max_distance = s.max_distance * self.skinned_max_distance_multiplier;

            // Calculate the skinned position by interpolating from previous to current position
            const skin_pos = skin_state.previous_position.mulScalar(prev_factor).add(skin_state.position.mulScalar(factor));

            if (max_distance > 0.0) {
                // Move vertex if it violated the back stop
                if (s.back_stop_distance < max_distance) {
                    // Center of the back stop sphere
                    const center = skin_pos.sub(skin_state.normal.mulScalar(s.back_stop_distance + s.back_stop_radius));

                    // Check if we're inside the back stop sphere
                    const delta = vertex.position.sub(center);
                    const delta_len_sq = delta.lengthSq();
                    if (delta_len_sq < math.square(s.back_stop_radius)) {
                        // Push the vertex to the surface of the back stop sphere
                        const delta_len = @sqrt(delta_len_sq);
                        vertex.position = if (delta_len > 0.0)
                            center.add(delta.mulScalar(s.back_stop_radius / delta_len))
                        else
                            center.add(skin_state.normal.mulScalar(s.back_stop_radius));
                    }
                }

                // Clamp vertex distance to max distance from skinned position
                if (max_distance < math.flt_max) {
                    const delta = vertex.position.sub(skin_pos);
                    const delta_len_sq = delta.lengthSq();
                    const max_distance_sq = math.square(max_distance);
                    if (delta_len_sq > max_distance_sq)
                        vertex.position = skin_pos.add(delta.mulScalar(@sqrt(max_distance_sq / delta_len_sq)));
                }
            } else {
                // Kinematic: Just update the vertex position
                vertex.position = skin_pos;
            }
        }
    }

    /// Enforce all edge constraints (private in Jolt)
    fn applyEdgeConstraints(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, start_index: u32, end_index: u32) void {
        const inv_dt_sq = 1.0 / math.square(context.sub_step_delta_time);

        // Satisfy edge constraints
        for (self.settings.get().?.edge_constraints.items[start_index..end_index]) |*e| {
            const v0 = &self.vertices.items[e.vertex[0]];
            const v1 = &self.vertices.items[e.vertex[1]];

            // Get positions
            const x0 = v0.position;
            const x1 = v1.position;

            // Calculate current length
            const delta = x1.sub(x0);
            const length = delta.length();

            // Apply correction
            const denom = length * (v0.inv_mass + v1.inv_mass + e.compliance * inv_dt_sq);
            if (denom < 1.0e-12)
                continue;
            const correction = delta.mulScalar(length - e.rest_length).divScalar(denom);
            v0.position = x0.add(correction.mulScalar(v0.inv_mass));
            v1.position = x1.sub(correction.mulScalar(v1.inv_mass));
        }
    }

    /// Enforce all rod stretch and shear constraints (private in Jolt)
    fn applyRodStretchShearConstraints(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, start_index: u32, end_index: u32) void {
        const inv_dt_sq = 1.0 / math.square(context.sub_step_delta_time);

        for (self.settings.get().?.rod_stretch_shear_constraints.items[start_index..end_index], self.rod_states.items[start_index..end_index]) |*r, *rod_state| {
            // Get positions
            const v0 = &self.vertices.items[r.vertex[0]];
            const v1 = &self.vertices.items[r.vertex[1]];

            // Apply stretch and shear constraint
            // Equation 37 from "Position and Orientation Based Cosserat Rods" - Kugelstadt and Schoemer - SIGGRAPH 2016
            const denom = v0.inv_mass + v1.inv_mass + 4.0 * r.inv_mass * math.square(r.length) + r.compliance * inv_dt_sq;
            if (denom < 1.0e-12)
                continue;
            const x0 = v0.position;
            const x1 = v1.position;
            var rotation = rod_state.rotation;
            const d3 = rotation.rotateAxisZ();
            const delta = x1.sub(x0).sub(d3.mulScalar(r.length)).divScalar(denom);
            v0.position = x0.add(delta.mulScalar(v0.inv_mass));
            v1.position = x1.sub(delta.mulScalar(v1.inv_mass));
            // q * e3_bar = q * (0, 0, -1, 0) = [-qy, qx, -qw, qz]
            const q_e3_bar = Quat.fromVec4(rotation.getXYZW().swizzle(.y, .x, .w, .z).flipSign(-1, 1, -1, 1));
            rotation = rotation.add(Quat.multiplyImaginary(delta, q_e3_bar).mulScalar(2.0 * r.inv_mass * r.length));

            // Renormalize
            rod_state.rotation = rotation.normalized();
        }
    }

    /// Enforce all rod bend and twist constraints (private in Jolt)
    fn applyRodBendTwistConstraints(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, start_index: u32, end_index: u32) void {
        const inv_dt_sq = 1.0 / math.square(context.sub_step_delta_time);

        const settings = self.settings.get().?;
        const rods = settings.rod_stretch_shear_constraints.items;

        for (settings.rod_bend_twist_constraints.items[start_index..end_index]) |*r| {
            const rod1_index = r.rod[0];
            const rod2_index = r.rod[1];
            const rod1 = &rods[rod1_index];
            const rod2 = &rods[rod2_index];
            const rod1_state = &self.rod_states.items[rod1_index];
            const rod2_state = &self.rod_states.items[rod2_index];

            // Apply bend and twist constraint
            // Equation 40 from "Position and Orientation Based Cosserat Rods" - Kugelstadt and Schoemer - SIGGRAPH 2016
            const denom = rod1.inv_mass + rod2.inv_mass + r.compliance * inv_dt_sq;
            if (denom < 1.0e-12)
                continue;
            var rotation1 = rod1_state.rotation;
            var rotation2 = rod2_state.rotation;
            const omega = rotation1.conjugated().mul(rotation2);
            const omega0 = r.omega0;
            const omega_min_omega0 = omega.sub(omega0).getXYZW();
            const omega_plus_omega0 = omega.add(omega0).getXYZW();
            // Take the shortest of the two rotations
            var delta_omega = Quat.fromVec4(Vec4.select(omega_min_omega0, omega_plus_omega0, Vec4.less(omega_plus_omega0.dotV(omega_plus_omega0), omega_min_omega0.dotV(omega_min_omega0))));
            delta_omega = delta_omega.divScalar(denom);
            delta_omega.setW(0.0); // Scalar part needs to be zero because the real part of the Darboux vector doesn't vanish, see text between eq. 39 and 40.
            const delta_rod2 = rotation1.mulScalar(rod2.inv_mass).mul(delta_omega);
            rotation1 = rotation1.add(rotation2.mulScalar(rod1.inv_mass).mul(delta_omega));
            rotation2 = rotation2.sub(delta_rod2);

            // Renormalize
            rod1_state.rotation = rotation1.normalized();
            rod2_state.rotation = rotation2.normalized();
        }
    }

    /// Enforce all LRA constraints (private in Jolt)
    fn applyLRAConstraints(self: *SoftBodyMotionProperties, start_index: u32, end_index: u32) void {
        // Satisfy LRA constraints
        const vertices = self.vertices.items;
        for (self.settings.get().?.lra_constraints.items[start_index..end_index]) |*lra| {
            if (Core.enable_asserts) std.debug.assert(lra.vertex[0] < vertices.len);
            if (Core.enable_asserts) std.debug.assert(lra.vertex[1] < vertices.len);
            const vertex0 = &vertices[lra.vertex[0]];
            const vertex1 = &vertices[lra.vertex[1]];

            const x0 = vertex0.position;
            const delta = vertex1.position.sub(x0);
            const delta_len_sq = delta.lengthSq();
            if (delta_len_sq > math.square(lra.max_distance))
                vertex1.position = x0.add(delta.mulScalar(math.mulRSqrtApproximate(lra.max_distance, delta_len_sq)));
        }
    }

    /// Enforce all collision constraints & update all velocities according the XPBD algorithm (private in Jolt)
    fn applyCollisionConstraintsAndUpdateVelocities(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext) void {
        const dt = context.sub_step_delta_time;
        const restitution_threshold = -2.0 * context.gravity.length() * dt;
        const vertex_radius = self.vertex_radius;
        for (self.vertices.items) |*v| {
            if (v.inv_mass > 0.0) {
                // Remember previous velocity for restitution calculations
                const prev_v = v.velocity;

                // XPBD velocity update
                v.velocity = v.position.sub(v.previous_position).divScalar(dt);

                // Satisfy collision constraint
                comptime std.debug.assert(@as(i32, @bitCast(BodyID.broad_phase_bit)) < 0); // CCD contacts should be negative too (see: SoftBodyVertex::MarkCCDContact)
                if (v.colliding_shape_index >= 0) {
                    // Check if there is a collision
                    const projected_distance = -v.collision_plane.signedDistance(v.position) + vertex_radius;
                    if (projected_distance > 0.0) {
                        // Remember that there was a collision
                        v.has_contact = true;

                        // We need a contact callback if one of the vertices collided
                        self.requestContactCallback();

                        // Note that we already calculated the velocity, so this does not affect the velocity (next iteration starts by setting previous position to current position)
                        const cs = &self.colliding_shapes.items[@intCast(v.colliding_shape_index)];
                        const contact_normal = v.collision_plane.getNormal();
                        v.position = v.position.add(contact_normal.mulScalar(projected_distance));

                        // Apply friction as described in Detailed Rigid Body Simulation with Extended Position Based Dynamics - Matthias Muller et al.
                        // See section 3.6:
                        // Inverse mass: w1 = 1 / m1, w2 = 1 / m2 + (r2 x n)^T I^-1 (r2 x n) = 0 for a static object
                        // r2 are the contact point relative to the center of mass of body 2
                        // Lagrange multiplier for contact: lambda = -c / (w1 + w2)
                        // Where c is the constraint equation (the distance to the plane, negative because penetrating)
                        // Contact normal force: fn = lambda / dt^2
                        // Delta velocity due to friction dv = -vt / |vt| * min(dt * friction * fn * (w1 + w2), |vt|) = -vt * min(-friction * c / (|vt| * dt), 1)
                        // Note that I think there is an error in the paper, I added a mass term, see: https://github.com/matthias-research/pages/issues/29
                        // Relative velocity: vr = v1 - v2 - omega2 x r2
                        // Normal velocity: vn = vr . contact_normal
                        // Tangential velocity: vt = vr - contact_normal * vn
                        // Impulse: p = dv / (w1 + w2)
                        // Changes in particle velocities:
                        // v1 = v1 + p / m1
                        // v2 = v2 - p / m2 (no change when colliding with a static body)
                        // w2 = w2 - I^-1 (r2 x p) (no change when colliding with a static body)
                        if (cs.motion_type == .dynamic) {
                            // Calculate normal and tangential velocity (equation 30)
                            const r2 = v.position.sub(cs.center_of_mass_transform.getTranslation());
                            const v2 = cs.getPointVelocity(r2);
                            const relative_velocity = v.velocity.sub(v2);
                            const v_normal = contact_normal.mulScalar(contact_normal.dot(relative_velocity));
                            const v_tangential = relative_velocity.sub(v_normal);
                            const v_tangential_length = v_tangential.length();

                            // Calculate resulting inverse mass of vertex
                            const vertex_inv_mass = cs.soft_body_inv_mass_scale * v.inv_mass;

                            // Calculate inverse effective mass
                            const r2_cross_n = r2.cross(contact_normal);
                            const w2 = cs.inv_mass + r2_cross_n.dot(cs.inv_inertia.mulVec3(r2_cross_n));
                            const w1_plus_w2 = vertex_inv_mass + w2;
                            if (w1_plus_w2 > 0.0) {
                                // Calculate delta relative velocity due to friction (modified equation 31)
                                var dv: Vec3 = undefined;
                                if (v_tangential_length > 0.0)
                                    dv = v_tangential.mulScalar(math.min(cs.friction * projected_distance / (v_tangential_length * dt), 1.0))
                                else
                                    dv = Vec3.zero();

                                // Calculate delta relative velocity due to restitution (equation 35)
                                dv = dv.add(v_normal);
                                const prev_v_normal = prev_v.sub(v2).dot(contact_normal);
                                if (prev_v_normal < restitution_threshold)
                                    dv = dv.add(contact_normal.mulScalar(cs.restitution * prev_v_normal));

                                // Calculate impulse
                                const p = dv.divScalar(w1_plus_w2);

                                // Apply impulse to particle
                                v.velocity = v.velocity.sub(p.mulScalar(vertex_inv_mass));

                                // Apply impulse to rigid body
                                cs.linear_velocity = cs.linear_velocity.add(p.mulScalar(cs.inv_mass));
                                cs.angular_velocity = cs.angular_velocity.add(cs.inv_inertia.mulVec3(r2.cross(p)));

                                // Mark that the velocities of the body we hit need to be updated
                                cs.update_velocities = true;
                            }
                        } else if (cs.soft_body_inv_mass_scale > 0.0) {
                            // Body is not movable, equations are simpler

                            // Calculate normal and tangential velocity (equation 30)
                            const v_normal = contact_normal.mulScalar(contact_normal.dot(v.velocity));
                            const v_tangential = v.velocity.sub(v_normal);
                            const v_tangential_length = v_tangential.length();

                            // Apply friction (modified equation 31)
                            if (v_tangential_length > 0.0)
                                v.velocity = v.velocity.sub(v_tangential.mulScalar(math.min(cs.friction * projected_distance / (v_tangential_length * dt), 1.0)));

                            // Apply restitution (equation 35)
                            v.velocity = v.velocity.sub(v_normal);
                            const prev_v_normal = prev_v.dot(contact_normal);
                            if (prev_v_normal < restitution_threshold)
                                v.velocity = v.velocity.sub(contact_normal.mulScalar(cs.restitution * prev_v_normal));
                        }
                    }
                }
            }
        }

        // Calculate the new angular velocity for all rods
        const two_div_dt = 2.0 / dt;
        for (self.rod_states.items) |*r|
            r.u = .{ .angular_velocity = r.rotation.mul(r.u.previous_rotation_internal.conjugated()).getXYZ().mulScalar(two_div_dt) }; // Overwrites mPreviousRotationInternal
    }

    /// Update the state of the soft body (position, velocity, bounds) (private in Jolt)
    fn updateSoftBodyState(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext, physics_settings: *const PhysicsSettings) void {
        // Contact callback
        if (self.need_contact_callback.load(.monotonic)) {
            if (context.contact_listener) |listener| {
                // Remove non-colliding sensors from the list
                var i = self.colliding_sensors.items.len;
                while (i > 0) {
                    i -= 1;
                    if (!self.colliding_sensors.items[i].has_contact) {
                        // mCollidingSensors[i] = std::move(mCollidingSensors.back()); mCollidingSensors.pop_back();
                        self.colliding_sensors.items[i].deinit(self.allocator);
                        self.colliding_sensors.items[i] = self.colliding_sensors.items[self.colliding_sensors.items.len - 1];
                        self.colliding_sensors.items.len -= 1;
                    }
                }

                listener.onSoftBodyContactAdded(context.body, &SoftBodyManifold.init(self));

                self.need_contact_callback.store(false, .monotonic);
            }
        }

        // Loop through vertices once more to update the global state
        const dt = context.delta_time;
        const max_linear_velocity_sq = math.square(self.base.getMaxLinearVelocity());
        var max_v_sq: f32 = 0.0;
        var linear_velocity = Vec3.zero();
        var angular_velocity = Vec3.zero();
        self.local_bounds = .empty;
        self.local_predicted_bounds = self.local_bounds;
        for (self.vertices.items) |*v| {
            // Calculate max square velocity
            const v_sq = v.velocity.lengthSq();
            max_v_sq = math.max(max_v_sq, v_sq);

            // Clamp if velocity is too high
            if (v_sq > max_linear_velocity_sq)
                v.velocity = v.velocity.mulScalar(@sqrt(max_linear_velocity_sq / v_sq));

            // Calculate local linear/angular velocity
            linear_velocity = linear_velocity.add(v.velocity);
            angular_velocity = angular_velocity.add(v.position.cross(v.velocity));

            // Update local bounding box
            self.local_bounds.encapsulateVec3(v.position);

            // Create predicted position for the next frame in order to detect collisions before they happen
            self.local_predicted_bounds.encapsulateVec3(v.position.add(v.velocity.mulScalar(dt)).add(context.displacement_due_to_gravity));

            // Reset collision data for the next iteration
            v.resetCollision();
        }

        // Calculate linear/angular velocity of the body by averaging all vertices and bringing the value to world space
        const num_vertices_divider: f32 = @floatFromInt(@max(@as(i32, @intCast(self.vertices.items.len)), 1));
        self.base.setLinearVelocityClamped(context.center_of_mass_transform.multiply3x3(linear_velocity.divScalar(num_vertices_divider)));
        self.base.setAngularVelocity(context.center_of_mass_transform.multiply3x3(angular_velocity.divScalar(num_vertices_divider)));

        if (self.update_position) {
            // Shift the body so that the position is the center of the local bounds
            const delta = self.local_bounds.getCenter();
            context.delta_position = context.center_of_mass_transform.multiply3x3(delta);
            for (self.vertices.items) |*v|
                v.position = v.position.sub(delta);

            // Update the skin state too since we will use this position as the previous position in the next update
            for (self.skin_state.items) |*s|
                s.position = s.position.sub(delta);
            if (Core.debug_renderer) self.skin_state_transform.setTranslation(self.skin_state_transform.getTranslation().addVec3(context.delta_position));

            // Offset bounds to match new position
            self.local_bounds.translate(delta.negate());
            self.local_predicted_bounds.translate(delta.negate());
        } else context.delta_position = Vec3.zero();

        // Test if we should go to sleep
        if (self.base.getAllowSleeping()) {
            if (max_v_sq > physics_settings.point_velocity_sleep_threshold) {
                self.base.resetSleepTestTimer();
                context.can_sleep = .cannot_sleep;
            } else context.can_sleep = self.base.accumulateSleepTime(dt, physics_settings.time_before_sleep);
        } else context.can_sleep = .cannot_sleep;

        // If SkinVertices is not called after this then don't use the previous position as the skin is static
        self.skin_state_previous_position_valid = false;

        // Reset force accumulator
        self.base.resetForce();
    }

    /// Start the first solver iteration (private in Jolt)
    fn startFirstIteration(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext) void {
        // Start the first iteration
        const iteration = context.next_iteration.fetchAdd(1, .monotonic);
        if (Core.enable_asserts) std.debug.assert(iteration == 0);
        self.startNextIteration(context);
        context.state.store(.apply_constraints, .release);
    }

    /// Executes tasks that need to run on the start of an iteration (i.e. the stuff that can't run in parallel) (private in Jolt)
    fn startNextIteration(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext) void {
        self.applyPressure(context);

        self.integratePositions(context);
    }

    /// Helper function for ParallelUpdate that works on batches of collision planes (private in Jolt)
    fn parallelDetermineCollisionPlanes(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext) Status {
        const batch = SoftBodyUpdateContext.vertex_collision_batch;

        // Do a relaxed read first to see if there is any work to do (this prevents us from doing expensive atomic operations and also prevents us from continuously incrementing the counter and overflowing it)
        const num_vertices: u32 = @intCast(self.vertices.items.len);
        if (hasVertexBatch(context.next_collision_vertex.load(.monotonic), num_vertices)) {
            // Fetch next batch of vertices to process
            const next_vertex = context.next_collision_vertex.fetchAdd(batch, .acquire);
            if (hasVertexBatch(next_vertex, num_vertices)) {
                // Process collision planes
                const num_vertices_to_process = @min(batch, num_vertices - next_vertex);
                if (num_vertices_to_process > 0) // Zolt (R20 UB1): an empty vertex range has nothing to collide
                    self.determineCollisionPlanes(next_vertex, num_vertices_to_process);
                const vertices_processed = context.num_collision_vertices_processed.fetchAdd(batch, .acq_rel) + num_vertices_to_process;
                if (vertices_processed >= num_vertices) {
                    // Determine next state
                    if (self.colliding_sensors.items.len == 0)
                        self.startFirstIteration(context)
                    else
                        context.state.store(.determine_sensor_collisions, .release);
                }
                return .did_work;
            }
        }

        return .no_work;
    }

    /// Zolt: Jolt's `next_vertex < num_vertices`. Without vertices (R20 UB1) the first claimant of the empty vertex range
    /// (next vertex 0) gets it too, so that it advances the state as the last batch would. Jolt never leaves the state.
    fn hasVertexBatch(next_vertex: u32, num_vertices: u32) bool {
        return next_vertex < num_vertices or (num_vertices == 0 and next_vertex == 0);
    }

    /// Helper function for ParallelUpdate that works on sensor collisions (private in Jolt)
    fn parallelDetermineSensorCollisions(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext) Status {
        // Do a relaxed read to see if there are more sensors to process
        const num_sensors = self.num_sensors.load(.monotonic);
        if (context.next_sensor_index.load(.monotonic) < num_sensors) {
            // Fetch next sensor to process
            const sensor_index = context.next_sensor_index.fetchAdd(1, .acquire);
            if (sensor_index < num_sensors) {
                // Process this sensor
                self.determineSensorCollisions(&self.colliding_sensors.items[sensor_index]);

                // Determine next state
                const sensors_processed = context.num_sensors_processed.fetchAdd(1, .acq_rel) + 1;
                if (sensors_processed >= num_sensors)
                    self.startFirstIteration(context);
                return .did_work;
            }
        }

        return .no_work;
    }

    /// Helper function for ParallelUpdate that works on batches of constraints (private in Jolt)
    fn parallelApplyConstraints(self: *SoftBodyMotionProperties, context: *SoftBodyUpdateContext, physics_settings: *const PhysicsSettings) Status {
        var num_groups: u32 = @intCast(self.settings.get().?.update_groups.items.len);
        std.debug.assert(num_groups > 0); // SoftBodySharedSettings::Optimize should have been called! (initialize checks it, R20 UB2)
        num_groups -= 1; // Last group is the non-parallel group, we don't want to execute it in parallel

        // Do a relaxed read first to see if there is any work to do (this prevents us from doing expensive atomic operations and also prevents us from continuously incrementing the counter and overflowing it)
        var next_group = context.next_constraint_group.load(.monotonic);
        if (next_group < num_groups or (num_groups == 0 and next_group == 0)) {
            // Fetch the next group process
            next_group = context.next_constraint_group.fetchAdd(1, .acquire);
            if (next_group < num_groups or (num_groups == 0 and next_group == 0)) {
                var num_groups_processed: u32 = 0;
                if (num_groups > 0) {
                    // Process this group
                    self.processGroup(context, next_group);

                    // Increment total number of groups processed
                    num_groups_processed = context.num_constraint_groups_processed.fetchAdd(1, .acq_rel) + 1;
                }

                if (num_groups_processed >= num_groups) {
                    // Finish the iteration

                    // Process non-parallel group
                    self.processGroup(context, num_groups);

                    self.applyCollisionConstraintsAndUpdateVelocities(context);

                    const iteration = context.next_iteration.fetchAdd(1, .monotonic);
                    if (iteration < self.num_iterations) {
                        // Start a new iteration
                        self.startNextIteration(context);

                        // Reset group logic
                        context.num_constraint_groups_processed.store(0, .release);
                        context.next_constraint_group.store(0, .release);
                    } else {
                        // On final iteration we update the state
                        self.updateSoftBodyState(context, physics_settings);

                        context.state.store(.done, .release);
                        return .done;
                    }
                }

                return .did_work;
            }
        }
        return .no_work;
    }

    /// Helper function to update a single group of constraints (private in Jolt)
    fn processGroup(self: *SoftBodyMotionProperties, context: *const SoftBodyUpdateContext, group_index: u32) void {
        // Determine start and end
        const start: SoftBodySharedSettings.UpdateGroup = .{ .edge_end_index = 0, .lra_end_index = 0, .rod_stretch_shear_end_index = 0, .rod_bend_twist_end_index = 0, .dihedral_bend_end_index = 0, .volume_end_index = 0, .skinned_end_index = 0 };
        const update_groups = self.settings.get().?.update_groups.items;
        const prev = if (group_index > 0) &update_groups[group_index - 1] else &start;
        const current = &update_groups[group_index];

        // Process volume constraints
        self.applyVolumeConstraints(context, prev.volume_end_index, current.volume_end_index);

        // Process bend constraints
        self.applyDihedralBendConstraints(context, prev.dihedral_bend_end_index, current.dihedral_bend_end_index);

        // Process skinned constraints
        self.applySkinConstraints(context, prev.skinned_end_index, current.skinned_end_index);

        // Process edges
        self.applyEdgeConstraints(context, prev.edge_end_index, current.edge_end_index);

        // Process rods
        self.applyRodStretchShearConstraints(context, prev.rod_stretch_shear_end_index, current.rod_stretch_shear_end_index);
        self.applyRodBendTwistConstraints(context, prev.rod_bend_twist_end_index, current.rod_bend_twist_end_index);

        // Process LRA constraints
        self.applyLRAConstraints(prev.lra_end_index, current.lra_end_index);
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

/// A physics system with one object layer for the update tests (in place: it must not move)
const TestWorld = struct {
    const BroadPhaseLayerInterfaceTable = @import("../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
    const ObjectVsBroadPhaseLayerFilterTable = @import("../Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
    const ObjectLayerPairFilterTable = @import("../Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
    const BodyCreationSettings = @import("../Body/BodyCreationSettings.zig").BodyCreationSettings;
    const BoxShape = @import("../Collision/Shape/BoxShape.zig").BoxShape;
    const RVec3 = @import("../../Math/Real.zig").RVec3;

    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    system: PhysicsSystem,

    fn init(self: *TestWorld, allocator: Allocator) !void {
        self.layer_interface = try .init(allocator, 1, 1);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(0, .init(0));
        self.pair_filter = try .init(allocator, 1);
        errdefer self.pair_filter.deinit();
        self.pair_filter.enableCollision(0, 0);
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, 1, &self.pair_filter.base, 1);
        errdefer self.ovb_filter.deinit();
        self.system = .empty;
        try self.system.init(allocator, std.testing.io, 16, 0, 64, 64, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
    }

    fn deinit(self: *TestWorld) void {
        self.system.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }

    /// A box body added to the system
    fn addBox(self: *TestWorld, half_extent: Vec3, position: RVec3, motion_type: MotionType) !BodyID {
        const box = try BoxShape.create(self.system.allocator, half_extent, .{});
        return self.addBody(box.asShape(), position, .{ .motion_type = motion_type });
    }

    /// A body with `shape` added to the system (the body takes a reference to the shape)
    fn addBody(self: *TestWorld, shape: *const Shape, position: RVec3, opts: struct { motion_type: MotionType = .static, is_sensor: bool = false }) !BodyID {
        var settings: BodyCreationSettings = .initWithShape(shape, position, Quat.identity(), opts.motion_type, 0);
        defer settings.deinit();
        settings.is_sensor = opts.is_sensor;
        const id = try self.system.getBodyInterfaceMut().createAndAddBody(&settings, .dont_activate);
        try std.testing.expect(!id.isInvalid());
        return id;
    }

    /// A soft body that is not added to the system (BodyInterface::CreateSoftBody)
    fn createSoftBody(self: *TestWorld, shared: *const SoftBodySharedSettings, position: RVec3) !*Body {
        var settings: SoftBodyCreationSettings = .init(shared, position, Quat.identity(), 0);
        defer settings.deinit();
        settings.allow_sleeping = false;
        return (try self.system.getBodyInterfaceMut().createSoftBody(&settings)).?;
    }
};

/// A horizontal cloth of n x n vertices with distance constraints, optimized (reference count 0)
fn testCloth(allocator: Allocator, n: u32, spacing: f32) !*SoftBodySharedSettings {
    const s = try SoftBodySharedSettings.create(allocator);
    errdefer {
        s.deinit();
        allocator.destroy(s);
    }
    const offset = 0.5 * spacing * @as(f32, @floatFromInt(n - 1));
    for (0..n) |z| for (0..n) |x| {
        try s.vertices.append(allocator, .init(.{ .x = spacing * @as(f32, @floatFromInt(x)) - offset, .y = 0, .z = spacing * @as(f32, @floatFromInt(z)) - offset }, .{}));
    };
    for (0..n - 1) |z| for (0..n - 1) |x| {
        const v0: u32 = @intCast(z * n + x);
        try s.addFace(.init(v0, v0 + n, v0 + 1, .{}));
        try s.addFace(.init(v0 + 1, v0 + n, v0 + n + 1, .{}));
    };
    try s.createConstraints(&.{.init(1.0e-5, 1.0e-5, 1.0e-4, .{})}, .{});
    try s.optimize();
    return s;
}

test "SoftBodyMotionProperties: customUpdate drops a cloth on a box, SaveState and RestoreState" {
    const allocator = std.testing.allocator;
    const RVec3 = TestWorld.RVec3;
    const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    _ = try world.addBox(Vec3.init(2, 0.5, 2), RVec3.init(0, -1, 0), .static);

    const body = try world.createSoftBody(try testCloth(allocator, 6, 0.2), RVec3.init(0, 0.25, 0));
    defer world.system.getBodyInterfaceMut().destroyBody(body.getID());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);
    try std.testing.expectEqual(@as(usize, 36), mp.getVertices().len);

    // Fall for a second: the cloth comes to rest on the top of the box at y = -0.5
    for (0..60) |_| {
        try mp.customUpdate(1.0 / 60.0, body, &world.system);
        try std.testing.expectEqual(@as(usize, 0), mp.colliding_shapes.items.len);
        try std.testing.expectEqual(@as(usize, 0), mp.colliding_sensors.items.len);
    }
    const com = body.getCenterOfMassTransform();
    for (mp.getVertices()) |*v| {
        const y = com.mulVec3(v.position).getY();
        try std.testing.expect(y > -0.55 and y < -0.3);
        try std.testing.expectEqual(@as(i32, -1), v.colliding_shape_index); // ResetCollision after the update
    }

    // SaveState, an update, RestoreState: the state of before the update
    var before: StateRecorderImpl = .init(allocator);
    defer before.deinit();
    mp.saveState(before.recorder());
    try mp.customUpdate(1.0 / 30.0, body, &world.system);
    before.rewind();
    mp.restoreState(before.recorder());
    var after: StateRecorderImpl = .init(allocator);
    defer after.deinit();
    mp.saveState(after.recorder());
    try std.testing.expectEqualSlices(u8, before.getData(), after.getData());
}

test "SoftBodyMotionProperties: a soft body without vertices advances its state (R20 UB1)" {
    const allocator = std.testing.allocator;
    const RVec3 = TestWorld.RVec3;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    _ = try world.addBox(Vec3.replicate(1), RVec3.init(0, -2, 0), .static);

    // A body with one kinematic vertex, then emptied: Jolt's SetMassProperties asserts when a body without vertices
    // is created (its mass is 0)
    const shared = try SoftBodySharedSettings.create(allocator);
    try shared.vertices.append(allocator, .init(.{ .x = 0, .y = 0, .z = 0 }, .{ .inv_mass = 0.0 }));
    try shared.update_groups.append(allocator, std.mem.zeroes(SoftBodySharedSettings.UpdateGroup));
    const body = try world.createSoftBody(shared, RVec3.init(0, 0, 0));
    defer world.system.getBodyInterfaceMut().destroyBody(body.getID());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);
    mp.vertices.clearRetainingCapacity();
    mp.local_bounds = .empty;
    mp.local_predicted_bounds = .empty;

    // The empty local bounds become the whole world: the box is a colliding shape
    var context: SoftBodyUpdateContext = .{};
    mp.initializeUpdateContext(1.0 / 60.0, body, &world.system, &context);
    try mp.determineCollidingShapes(&context, &world.system, world.system.getBodyLockInterface());
    try std.testing.expectEqual(@as(usize, 1), mp.colliding_shapes.items.len);

    // The first claimant of the empty vertex range advances the state (Jolt never leaves DetermineCollisionPlanes)
    try std.testing.expectEqual(SoftBodyMotionProperties.Status.did_work, mp.parallelUpdate(&context, world.system.getPhysicsSettings()));
    try std.testing.expectEqual(SoftBodyUpdateContext.State.apply_constraints, context.state.load(.acquire));
    var num_calls: u32 = 1;
    while (mp.parallelUpdate(&context, world.system.getPhysicsSettings()) == .did_work) : (num_calls += 1)
        try std.testing.expect(num_calls < 100);
    try std.testing.expectEqual(SoftBodyUpdateContext.State.done, context.state.load(.acquire));
    try std.testing.expect(context.delta_position.eql(Vec3.zero()));
    mp.updateRigidBodyVelocities(&context, world.system.getBodyInterfaceMut());
    try std.testing.expectEqual(@as(usize, 0), mp.colliding_shapes.items.len);

    // customUpdate finishes too and does not move the body
    try mp.customUpdate(1.0 / 60.0, body, &world.system);
    try std.testing.expect(body.getPosition().eql(RVec3.zero()));
}

/// A soft body contact listener for the tests: accepts every contact, counts the calls and remembers the last body that
/// it validated
const TestSoftBodyListener = struct {
    num_validates: u32 = 0,
    num_added: u32 = 0,
    last_validated: BodyID = .invalid,

    pub fn onSoftBodyContactValidate(self: *TestSoftBodyListener, soft_body: *const Body, other_body: *const Body, settings: *SoftBodyContactSettings) SoftBodyValidateResult {
        _ = .{ soft_body, settings };
        self.num_validates += 1;
        self.last_validated = other_body.getID();
        return .accept_contact;
    }

    pub fn onSoftBodyContactAdded(self: *TestSoftBodyListener, soft_body: *const Body, manifold: *const SoftBodyManifold) void {
        _ = .{ soft_body, manifold };
        self.num_added += 1;
    }
};

test "SoftBodyMotionProperties: a soft body without vertices passes through the sensor state (R20 UB1)" {
    const allocator = std.testing.allocator;
    const RVec3 = TestWorld.RVec3;
    const BoxShape = TestWorld.BoxShape;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    _ = try world.addBox(Vec3.replicate(1), RVec3.init(0, -2, 0), .static);
    const sensor_id = try world.addBody((try BoxShape.create(allocator, Vec3.replicate(1), .{})).asShape(), RVec3.init(3, 0, 0), .{ .is_sensor = true });
    var listener: TestSoftBodyListener = .{};
    world.system.setSoftBodyContactListener(SoftBodyContactListener.init(&listener));

    // A body with one kinematic vertex, then emptied (see the test above)
    const shared = try SoftBodySharedSettings.create(allocator);
    try shared.vertices.append(allocator, .init(.{ .x = 0, .y = 0, .z = 0 }, .{ .inv_mass = 0.0 }));
    try shared.update_groups.append(allocator, std.mem.zeroes(SoftBodySharedSettings.UpdateGroup));
    const body = try world.createSoftBody(shared, RVec3.init(0, 0, 0));
    defer world.system.getBodyInterfaceMut().destroyBody(body.getID());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);
    mp.vertices.clearRetainingCapacity();
    mp.local_bounds = .empty;
    mp.local_predicted_bounds = .empty;

    // The empty local bounds become the whole world: the box and the sensor are validated and collected
    var context: SoftBodyUpdateContext = .{};
    mp.initializeUpdateContext(1.0 / 60.0, body, &world.system, &context);
    try mp.determineCollidingShapes(&context, &world.system, world.system.getBodyLockInterface());
    try std.testing.expectEqual(@as(u32, 2), listener.num_validates);
    try std.testing.expectEqual(@as(usize, 1), mp.colliding_shapes.items.len);
    try std.testing.expectEqual(@as(usize, 1), mp.colliding_sensors.items.len);
    try std.testing.expectEqual(@as(u32, 1), mp.num_sensors.load(.monotonic));
    try std.testing.expect(mp.colliding_sensors.items[0].body_id.eql(sensor_id));

    // The first claimant of the empty vertex range goes to the sensor state, the sensor collides 0 vertices
    const settings = world.system.getPhysicsSettings();
    try std.testing.expectEqual(SoftBodyMotionProperties.Status.did_work, mp.parallelUpdate(&context, settings));
    try std.testing.expectEqual(SoftBodyUpdateContext.State.determine_sensor_collisions, context.state.load(.acquire));
    try std.testing.expectEqual(SoftBodyMotionProperties.Status.did_work, mp.parallelUpdate(&context, settings));
    try std.testing.expectEqual(SoftBodyUpdateContext.State.apply_constraints, context.state.load(.acquire));
    try std.testing.expect(!mp.colliding_sensors.items[0].has_contact);
    var num_calls: u32 = 2;
    while (mp.parallelUpdate(&context, settings) == .did_work) : (num_calls += 1)
        try std.testing.expect(num_calls < 100);
    try std.testing.expectEqual(SoftBodyUpdateContext.State.done, context.state.load(.acquire));
    try std.testing.expect(context.delta_position.eql(Vec3.zero()));
    try std.testing.expect(!mp.need_contact_callback.load(.monotonic));
    try std.testing.expectEqual(@as(u32, 0), listener.num_added);
    mp.updateRigidBodyVelocities(&context, world.system.getBodyInterfaceMut());
    try std.testing.expectEqual(@as(usize, 0), mp.colliding_shapes.items.len + mp.colliding_sensors.items.len);

    // customUpdate finishes too and does not move the body
    try mp.customUpdate(1.0 / 60.0, body, &world.system);
    try std.testing.expectEqual(@as(u32, 4), listener.num_validates);
    try std.testing.expectEqual(@as(u32, 0), listener.num_added);
    try std.testing.expect(body.getPosition().eql(RVec3.zero()));
}

test "SoftBodyMotionProperties: customUpdate moves a soft body that was never added, not one without an ID" {
    const allocator = std.testing.allocator;
    const RVec3 = TestWorld.RVec3;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const bi = world.system.getBodyInterfaceMut();

    // Created, not added: it has an ID, so SetPosition moves it (without the broad phase, like Jolt)
    const shared = try testCloth(allocator, 3, 0.5);
    const body = try world.createSoftBody(shared, RVec3.init(0, 1, 0));
    defer bi.destroyBody(body.getID());
    try std.testing.expect(!body.isInBroadPhase());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);
    try std.testing.expect(mp.getUpdatePosition());
    try mp.customUpdate(1.0 / 60.0, body, &world.system);
    try std.testing.expect(body.getPosition().getY() < 1.0);
    try std.testing.expect(!body.isInBroadPhase());

    // Without an ID the lock of SetPosition fails: the body stays where it is
    var settings: SoftBodyCreationSettings = .init(shared, RVec3.init(0, 1, 0), Quat.identity(), 0);
    defer settings.deinit();
    const body2 = try bi.createSoftBodyWithoutID(&settings);
    defer bi.destroyBodyWithoutID(body2);
    try std.testing.expect(body2.getID().isInvalid());
    const mp2 = SoftBodyMotionProperties.fromBase(body2.getMotionPropertiesUncheckedMut().?);
    try mp2.customUpdate(1.0 / 60.0, body2, &world.system);
    try std.testing.expect(body2.getPosition().eql(RVec3.init(0, 1, 0)));
}

test "SoftBodyMotionProperties: customUpdate out of memory releases the colliding shapes and changes no vertex" {
    const RVec3 = TestWorld.RVec3;
    const allocator = std.testing.allocator;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    for (0..3) |i|
        _ = try world.addBox(Vec3.init(0.5, 0.5, 0.5), RVec3.init(@as(f32, @floatFromInt(i)) - 1.0, -0.4, 0), .static);

    const body = try world.createSoftBody(try testCloth(allocator, 4, 0.5), RVec3.init(0, 0, 0));
    defer world.system.getBodyInterfaceMut().destroyBody(body.getID());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);

    // Fail every allocation of the motion properties in turn: the colliding shape array, the leaf shape arrays
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        mp.colliding_shapes.clearAndFree(allocator); // The same allocations in every round
        var failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = fail_index });
        mp.allocator = failing.allocator();
        defer mp.allocator = allocator;
        const vertices_before = try allocator.dupe(SoftBodyVertex, mp.getVertices());
        defer allocator.free(vertices_before);
        mp.customUpdate(1.0 / 60.0, body, &world.system) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            try std.testing.expectEqual(@as(usize, 0), mp.colliding_shapes.items.len);
            try std.testing.expectEqual(@as(usize, 0), mp.colliding_sensors.items.len);
            try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(vertices_before), std.mem.sliceAsBytes(mp.getVertices()));
            continue;
        };
        try std.testing.expect(fail_index > 3); // The array of 3 colliding shapes and 3 leaf shape arrays
        break;
    }
}

test "SoftBodyMotionProperties: out of memory with a sensor and a compound keeps the number of sensors and leaks nothing" {
    const RVec3 = TestWorld.RVec3;
    const BoxShape = TestWorld.BoxShape;
    const StaticCompoundShapeSettings = @import("../Collision/Shape/StaticCompoundShape.zig").StaticCompoundShapeSettings;
    const allocator = std.testing.allocator;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    var listener: TestSoftBodyListener = .{};
    world.system.setSoftBodyContactListener(SoftBodyContactListener.init(&listener));

    // Under the cloth: a box (1 leaf shape), a compound of two boxes (2 leaf shapes) and a sensor (1 leaf shape)
    const box_id = try world.addBox(Vec3.replicate(0.5), RVec3.init(-1, -0.4, 0), .static);
    var compound_settings: StaticCompoundShapeSettings = .init(allocator);
    defer compound_settings.deinit();
    for ([_]f32{ -0.5, 0.5 }) |z|
        try compound_settings.base.addShapePtr(Vec3.init(0, 0, z), Quat.identity(), (try BoxShape.create(allocator, Vec3.init(0.4, 0.5, 0.4), .{})).asShape(), .{});
    var compound = try compound_settings.asShapeSettings().createShape(allocator);
    defer compound.deinit();
    const compound_id = try world.addBody(compound.getPtr().?, RVec3.init(0, -0.4, 0), .{});
    const sensor_id = try world.addBody((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape(), RVec3.init(1, -0.4, 0), .{ .is_sensor = true });

    const body = try world.createSoftBody(try testCloth(allocator, 4, 0.5), RVec3.init(0, 0, 0));
    defer world.system.getBodyInterfaceMut().destroyBody(body.getID());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);

    // Fail every allocation of the motion properties in turn. Every growth allocates (no remap), so the second leaf
    // shape of the compound has an allocation of its own: a failure there releases a list with one leaf shape.
    var failures_after: [3]u32 = @splat(0); // The failures after the validation of the box, the compound and the sensor
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        // determineCollidingShapes stores the number of sensors that it kept, also after an error (R17)
        mp.colliding_shapes.clearAndFree(allocator); // The same allocations in every round
        mp.colliding_sensors.clearAndFree(allocator);
        var failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = fail_index, .resize_fail_index = 0 });
        mp.allocator = failing.allocator();
        var context: SoftBodyUpdateContext = .{};
        mp.initializeUpdateContext(1.0 / 60.0, body, &world.system, &context);
        listener.last_validated = .invalid;
        const result = mp.determineCollidingShapes(&context, &world.system, world.system.getBodyLockInterface());
        try std.testing.expectEqual(@as(u32, @intCast(mp.colliding_sensors.items.len)), mp.num_sensors.load(.monotonic));
        if (result) |_| {
            try std.testing.expectEqual(@as(usize, 2), mp.colliding_shapes.items.len);
            try std.testing.expectEqual(@as(usize, 1), mp.colliding_sensors.items.len);
            try std.testing.expectEqual(@as(usize, 3), mp.colliding_shapes.items[0].shapes.items.len + mp.colliding_shapes.items[1].shapes.items.len);
        } else |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            const last = listener.last_validated;
            try std.testing.expect(last.eql(box_id) or last.eql(compound_id) or last.eql(sensor_id));
            const which: usize = if (last.eql(box_id)) 0 else if (last.eql(compound_id)) 1 else 2;
            failures_after[which] += 1;
        }
        mp.clearCollidingShapesAndSensors();
        mp.allocator = allocator;

        // customUpdate fails at the same allocation: it releases the colliding shapes and sensors, the vertices stay
        mp.colliding_shapes.clearAndFree(allocator);
        mp.colliding_sensors.clearAndFree(allocator);
        var failing2: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = fail_index, .resize_fail_index = 0 });
        mp.allocator = failing2.allocator();
        defer mp.allocator = allocator;
        const vertices_before = try allocator.dupe(SoftBodyVertex, mp.getVertices());
        defer allocator.free(vertices_before);
        mp.customUpdate(1.0 / 60.0, body, &world.system) catch |err| {
            try std.testing.expect(std.meta.isError(result));
            try std.testing.expectEqual(error.OutOfMemory, err);
            try std.testing.expectEqual(@as(usize, 0), mp.colliding_shapes.items.len + mp.colliding_sensors.items.len);
            try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(vertices_before), std.mem.sliceAsBytes(mp.getVertices()));
            continue;
        };
        try std.testing.expect(!std.meta.isError(result));
        break;
    }

    // The sensor fails twice (its leaf shape, the sensor list); the compound once more than the box (its second leaf shape)
    try std.testing.expectEqual(@as(u32, 2), failures_after[2]);
    try std.testing.expectEqual(failures_after[0] + 1, failures_after[1]);
    try std.testing.expect(failures_after[0] > 0);
}

test "SoftBodyMotionProperties: skinVertices, hard skinning and out of memory" {
    const allocator = std.testing.allocator;
    const RVec3 = TestWorld.RVec3;
    const TempAllocatorImpl = TempAllocatorFile.TempAllocatorImpl;
    const FailingTempAllocator = TempAllocatorFile.FailingTempAllocator;

    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();

    // A cloth with every vertex skinned to joint 1 through the second inverse bind matrix
    const shared = try testCloth(allocator, 3, 0.5);
    try shared.inv_bind_matrices.append(allocator, .init(0, Mat44.identity()));
    try shared.inv_bind_matrices.append(allocator, .init(1, Mat44.translation(Vec3.init(0, -1, 0))));
    for (0..shared.vertices.items.len) |v| {
        var skinned: SoftBodySharedSettings.Skinned = .init(@intCast(v), 0.1, 0.0, 1.0);
        skinned.weights[0] = .init(1, 1.0);
        try shared.skinned_constraints.append(allocator, skinned);
    }
    try shared.calculateSkinnedConstraintNormals();
    try shared.optimize();
    const body = try world.createSoftBody(shared, RVec3.init(0, 0, 0));
    defer world.system.getBodyInterfaceMut().destroyBody(body.getID());
    const mp = SoftBodyMotionProperties.fromBase(body.getMotionPropertiesUncheckedMut().?);

    var temp: TempAllocatorImpl = try .init(allocator, 4096);
    defer temp.deinit();
    const joints = [_]Mat44{ Mat44.identity(), Mat44.translation(Vec3.init(0, 3, 0)) };

    // Out of memory: nothing changes
    var failing: FailingTempAllocator = .init(temp.tempAllocator(), .{ .fail_index = 0 });
    try std.testing.expectError(error.OutOfMemory, mp.skinVertices(body.getCenterOfMassTransform(), &joints, true, failing.tempAllocator()));
    try std.testing.expect(!mp.skin_state_previous_position_valid);
    try std.testing.expect(mp.skin_state.items[0].position.isNaN());

    // Hard skin all: every vertex moves up by 3 - 1 = 2, the normals point up (the faces are counter clockwise seen from above)
    try mp.skinVertices(body.getCenterOfMassTransform(), &joints, true, temp.tempAllocator());
    try std.testing.expect(temp.isEmpty());
    try std.testing.expect(mp.skin_state_previous_position_valid);
    for (mp.getVertices(), shared.vertices.items, mp.skin_state.items) |*v, *in_v, *s| {
        try std.testing.expect(v.position.isClose(Vec3.loadFloat3Unsafe(&in_v.position).add(Vec3.init(0, 2, 0)), .{}));
        try std.testing.expect(v.velocity.eql(Vec3.zero()));
        try std.testing.expect(s.previous_position.eql(s.position));
        try std.testing.expect(s.normal.isClose(Vec3.axisY(), .{}) or s.normal.isClose(Vec3.axisY().negate(), .{}));
    }

    // An update clears the flag (UpdateSoftBodyState)
    try mp.customUpdate(1.0 / 60.0, body, &world.system);
    try std.testing.expect(!mp.skin_state_previous_position_valid);
}
