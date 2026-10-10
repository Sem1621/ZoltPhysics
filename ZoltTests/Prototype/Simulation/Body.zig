//! Phase 5 architecture prototype (not a port, removed after the foundation lands): reduced forms of
//! Jolt/Physics/Body/{MotionType, MotionQuality, BodyType, AllowedDOFs, MotionProperties, Body, BodyCreationSettings}.h
//! and Jolt/Physics/SoftBody/SoftBodyMotionProperties.h (stub), showing the decided mechanisms
//! (Docs/Zolt/SimulationArchitecture.md D1, D6):
//! - Body keeps Jolt's fields in Jolt's order; its size (128 / 160 bytes) and alignment are checked at compile time.
//! - Body::EFlags (private, used with atomic or / and) are u8 constants; public flag enums (EAllowedDOFs) are packed
//!   structs with Jolt's bits.
//! - MotionProperties.index_in_active_bodies is a monotonic atomic: Jolt reads it in FindCollidingPairsCanCollide while
//!   another job activates the body (a TSAN-suppressed race in Jolt, a defined access in Zolt at zero cost).
//! - Body.fixedToWorld(): a comptime constant in read-only memory whose dummy shape is a static EmptyShape (never
//!   reference counted, so RefConst(Shape) on it is legal).
//! - Soft bodies: SoftBodyMotionProperties is a stub with its final names; Body's soft body branches are ported and
//!   unreachable because no soft body can be created before Phase 9.
//! - BodyCreationSettings: Jolt's mutually exclusive mShape / mShapePtr are a tagged union; getShape works on a const
//!   settings (Rule M: the cache is in the ShapeSettings behind the Ref).

const std = @import("std");
const zolt = @import("zolt");
const Core = zolt.Core;
const math = zolt.math;
const Vec3 = zolt.Vec3;
const Float3 = zolt.Float3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const RMat44 = zolt.RMat44;
const AABox = zolt.AABox;
const BodyID = zolt.BodyID;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const ShapeResult = zolt.ShapeResult;
const EmptyShape = zolt.EmptyShape;
const MassProperties = zolt.MassProperties;
const CollisionGroup = zolt.CollisionGroup;
const ObjectLayer = zolt.ObjectLayer;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const Allocator = std.mem.Allocator;
const StateRecorder = @import("StateRecorder.zig").StateRecorder;

/// Motion type of a physics body (EMotionType). The order matters: ProcessBodyPair compares motion types.
pub const MotionType = enum(u8) {
    /// Non movable
    static,
    /// Movable using velocities only, does not respond to forces
    kinematic,
    /// Responds to forces as a normal physics object
    dynamic,
};

/// Motion quality, or how well it detects collisions when it has a high velocity (EMotionQuality)
pub const MotionQuality = enum(u8) {
    /// Update the body in discrete steps
    discrete,
    /// Use a linear cast to find the first collision during the step (CCD)
    linear_cast,
};

/// Type of body (EBodyType)
pub const BodyType = enum(u8) {
    rigid_body,
    soft_body,
};

/// Number of body types (cBodyTypeCount)
pub const body_type_count = 2;

/// Enum used by AddBody to determine if the body needs to be initially active (EActivation)
pub const Activation = enum(u8) {
    /// Activate the body, making it part of the simulation
    activate,
    /// Leave activation state as it is (will not deactivate an active body)
    dont_activate,
};

/// Enum used in BodyCreationSettings and MotionProperties to indicate which degrees of freedom a body has (EAllowedDOFs).
/// A public flag enum with operators becomes a packed struct with Jolt's bits; streams write the same byte.
pub const AllowedDOFs = packed struct(u8) {
    translation_x: bool = false,
    translation_y: bool = false,
    translation_z: bool = false,
    rotation_x: bool = false,
    rotation_y: bool = false,
    rotation_z: bool = false,
    _padding: u2 = 0,

    /// No degrees of freedom are allowed. Note that this is not valid and will crash. Use a static body instead.
    pub const none: AllowedDOFs = .{};
    /// All degrees of freedom are allowed
    pub const all: AllowedDOFs = .{ .translation_x = true, .translation_y = true, .translation_z = true, .rotation_x = true, .rotation_y = true, .rotation_z = true };
    /// Body can only move in X and Y axis and rotate around Z axis
    pub const plane_2d: AllowedDOFs = .{ .translation_x = true, .translation_y = true, .rotation_z = true };

    comptime {
        std.debug.assert(@as(u8, @bitCast(all)) == 0b111111);
        std.debug.assert(@as(u8, @bitCast(plane_2d)) == 0b100011);
    }
};

/// The Body class only keeps track of state for static bodies, the MotionProperties class keeps the additional state
/// needed for a moving Body. It has a 1-on-1 relationship with the body. (reduced: no inertia, no sleep test spheres)
pub const MotionProperties = struct {
    /// Constant indicating that body is not active (Body::cInactiveIndex)
    pub const inactive_index: u32 = 0xffffffff;

    linear_velocity: Vec3 = Vec3.zero(),
    angular_velocity: Vec3 = Vec3.zero(),
    force: Float3 = .init(0, 0, 0),
    inv_mass: f32 = 0.0,
    linear_damping: f32 = 0.0,
    angular_damping: f32 = 0.0,
    max_linear_velocity: f32 = 0.0,
    max_angular_velocity: f32 = 0.0,
    gravity_factor: f32 = 1.0,
    /// If the body is active, this is the index in BodyManager::mActiveBodies. Jolt: a plain uint32 that
    /// FindCollidingPairsCanCollide reads while another job activates the body (TSAN suppressed); Zolt: a monotonic
    /// atomic (a plain mov on x86 / ARM), so the race is defined.
    index_in_active_bodies: std.atomic.Value(u32) = .init(inactive_index),
    island_index: u32 = inactive_index,
    motion_quality: MotionQuality = .discrete,
    allowed_dofs: AllowedDOFs = .all,
    allow_sleeping: bool = true,
    /// Debug only state (JPH_ENABLE_ASSERTS): typed `void` when asserts are off, so it cannot affect release results
    cached_motion_type: if (Core.enable_asserts) MotionType else void = if (Core.enable_asserts) .static else {},

    pub fn getLinearVelocity(self: *const MotionProperties) Vec3 {
        return self.linear_velocity;
    }

    pub fn getMotionQuality(self: *const MotionProperties) MotionQuality {
        return self.motion_quality;
    }

    pub fn getAccumulatedForce(self: *const MotionProperties) Vec3 {
        return Vec3.fromFloat3(self.force);
    }

    /// Access to the index in the BodyManager::mActiveBodies list
    pub fn getIndexInActiveBodiesInternal(self: *const MotionProperties) u32 {
        return self.index_in_active_bodies.load(.monotonic);
    }

    /// Clamp linear velocity according to limit
    pub fn clampLinearVelocity(self: *MotionProperties) void {
        const len_sq = self.linear_velocity.lengthSq();
        std.debug.assert(std.math.isFinite(len_sq));
        if (len_sq > math.square(self.max_linear_velocity))
            self.linear_velocity = self.linear_velocity.mulScalar(self.max_linear_velocity / @sqrt(len_sq));
    }

    /// Clamp angular velocity according to limit
    pub fn clampAngularVelocity(self: *MotionProperties) void {
        const len_sq = self.angular_velocity.lengthSq();
        std.debug.assert(std.math.isFinite(len_sq));
        if (len_sq > math.square(self.max_angular_velocity))
            self.angular_velocity = self.angular_velocity.mulScalar(self.max_angular_velocity / @sqrt(len_sq));
    }

    /// Apply the gravity, forces and drag (ApplyForceTorqueAndDragInternal, reduced: no torque)
    pub fn applyForceTorqueAndDragInternal(self: *MotionProperties, body_rotation: Quat, gravity: Vec3, delta_time: f32) void {
        _ = body_rotation; // Jolt uses it for the torque (no inertia in the prototype)

        // Update linear velocity
        self.linear_velocity = self.linear_velocity.add(gravity.mulScalar(self.gravity_factor).add(self.getAccumulatedForce().mulScalar(self.inv_mass)).mulScalar(delta_time));

        // Linear damping: dv/dt = -c * v
        self.linear_velocity = self.linear_velocity.mulScalar(math.max(@as(f32, 0.0), 1.0 - self.linear_damping * delta_time));
        self.angular_velocity = self.angular_velocity.mulScalar(math.max(@as(f32, 0.0), 1.0 - self.angular_damping * delta_time));

        // Clamp velocities
        self.clampLinearVelocity();
        self.clampAngularVelocity();
    }

    /// Reset force (ResetForce)
    pub fn resetForce(self: *MotionProperties) void {
        self.force = .init(0, 0, 0);
    }

    /// Saving state for replay (only what changes at runtime, in Jolt's order)
    pub fn saveState(self: *const MotionProperties, stream: *StateRecorder) void {
        stream.write(self.linear_velocity);
        stream.write(self.angular_velocity);
        stream.write(self.force);
        stream.write(self.allow_sleeping);
    }

    /// Restoring state for replay
    pub fn restoreState(self: *MotionProperties, stream: *StateRecorder) void {
        stream.read(&self.linear_velocity);
        stream.read(&self.angular_velocity);
        stream.read(&self.force);
        stream.read(&self.allow_sleeping);
    }
};

/// Phase 9 stub of Jolt/Physics/SoftBody/SoftBodyMotionProperties.h: the final names and signatures that Body,
/// BodyManager and PhysicsSystem call. Unreachable in Phase 5: no soft body can be created (BodyInterface /
/// BodyManager declare no soft body creation function before Phase 9).
pub const SoftBodyMotionProperties = struct {
    base: MotionProperties,

    /// static_cast<SoftBodyMotionProperties *>(MotionProperties *)
    pub fn fromBase(mp: *MotionProperties) *SoftBodyMotionProperties {
        return @fieldParentPtr("base", mp);
    }

    pub fn fromBaseConst(mp: *const MotionProperties) *const SoftBodyMotionProperties {
        return @fieldParentPtr("base", mp);
    }

    pub fn saveState(self: *const SoftBodyMotionProperties, stream: *StateRecorder) void {
        _ = self;
        _ = stream;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    pub fn restoreState(self: *SoftBodyMotionProperties, stream: *StateRecorder) void {
        _ = self;
        _ = stream;
        @panic("SoftBodyMotionProperties: Phase 9");
    }
};

/// A rigid body that can be simulated using the physics system
///
/// Note that internally all properties (position, velocity etc.) are tracked relative to the center of mass of the object to simplify the simulation of the object.
pub const Body = struct {
    /// Constant indicating that body is not active
    pub const inactive_index = MotionProperties.inactive_index;

    /// Body::EFlags: private bits of `flags`, used with atomic or / and like Jolt
    pub const Flags = struct {
        pub const is_sensor: u8 = 1 << 0;
        pub const collide_kinematic_vs_non_dynamic: u8 = 1 << 1;
        pub const is_in_broad_phase: u8 = 1 << 2;
        pub const invalidate_contact_cache: u8 = 1 << 3;
        pub const use_manifold_reduction: u8 = 1 << 4;
        pub const apply_gyroscopic_force: u8 = 1 << 5;
        pub const enhanced_internal_edge_removal: u8 = 1 << 6;
    };

    // 16 byte aligned
    position: RVec3, // World space position of center of mass
    rotation: Quat, // World space rotation of center of mass
    bounds: AABox = .empty, // World space bounding box of the body

    // 8 byte aligned
    shape: RefConst(Shape) = .empty, // Shape representing the volume of this body
    motion_properties: ?*MotionProperties = null, // If this is a keyframed or dynamic object, this object holds all information about the movement
    user_data: u64 = 0, // User data, can be used for anything by the application
    collision_group: CollisionGroup = .{}, // The collision group this body belongs to (determines if two objects can collide)

    // 4 byte aligned
    friction: f32 = 0.0,
    restitution: f32 = 0.0,
    id: BodyID = .invalid, // ID of the body (index in the bodies array)

    // 2 or 4 bytes aligned
    object_layer: ObjectLayer = zolt.object_layer_invalid,

    // 1 byte aligned
    body_type: BodyType = .rigid_body,
    broad_phase_layer: BroadPhaseLayer = zolt.broad_phase_layer_invalid,
    motion_type: MotionType = .static,
    flags: std.atomic.Value(u8) = .init(0),

    comptime {
        // static_assert(sizeof(Body) == JPH_IF_SINGLE_PRECISION_ELSE(128, 160)) holds with Zig's field ordering too
        if (@sizeOf(usize) == 8 and Core.object_layer_bits == 16)
            std.debug.assert(@sizeOf(Body) == if (Core.double_precision) 160 else 128);
        std.debug.assert(@alignOf(Body) == @max(Core.vector_alignment, zolt.rvector_alignment));
    }

    /// static const EmptyShape sFixedToWorldShape: a static shape (never reference counted) in read-only memory
    const fixed_to_world_shape: EmptyShape = .initStatic(Vec3.zero());

    /// Body::sFixedToWorld storage (Body(bool) constructor): a comptime constant in read-only memory, like
    /// PhysicsMaterial.default
    const fixed_to_world_storage: Body = .{
        .position = RVec3.zero(),
        .rotation = Quat.identity(),
        .shape = .{ .ptr = fixed_to_world_shape.asShape() }, // Dummy shape (static: no reference to add)
        .friction = 0.0,
        .restitution = 0.0,
        .object_layer = zolt.object_layer_invalid,
        .motion_type = .static,
    };

    /// A dummy body that can be used by constraints to attach a constraint to the world instead of another body
    /// (Body::sFixedToWorld). The single audited `@constCast` of Phase 5: constraints store `*Body` like Jolt's `Body &`,
    /// and nothing writes a static body (every solver write path checks IsDynamic / the motion properties first), so a
    /// stray write faults instead of corrupting state shared by all physics systems.
    pub fn fixedToWorld() *Body {
        return @constCast(&fixed_to_world_storage);
    }

    /// Get the id of this body
    pub fn getID(self: *const Body) BodyID {
        return self.id;
    }

    pub fn getBodyType(self: *const Body) BodyType {
        return self.body_type;
    }

    pub fn isRigidBody(self: *const Body) bool {
        return self.body_type == .rigid_body;
    }

    pub fn isSoftBody(self: *const Body) bool {
        return self.body_type == .soft_body;
    }

    /// If this body is currently actively simulating (true) or sleeping (false)
    pub fn isActive(self: *const Body) bool {
        return self.motion_properties != null and self.motion_properties.?.getIndexInActiveBodiesInternal() != inactive_index;
    }

    pub fn isStatic(self: *const Body) bool {
        return self.motion_type == .static;
    }

    pub fn isKinematic(self: *const Body) bool {
        return self.motion_type == .kinematic;
    }

    pub fn isDynamic(self: *const Body) bool {
        return self.motion_type == .dynamic;
    }

    pub fn getMotionType(self: *const Body) MotionType {
        return self.motion_type;
    }

    pub fn isSensor(self: *const Body) bool {
        return (self.flags.load(.monotonic) & Flags.is_sensor) != 0;
    }

    pub fn getCollideKinematicVsNonDynamic(self: *const Body) bool {
        return (self.flags.load(.monotonic) & Flags.collide_kinematic_vs_non_dynamic) != 0;
    }

    /// Check if this body has been added to the physics system
    pub fn isInBroadPhase(self: *const Body) bool {
        return (self.flags.load(.monotonic) & Flags.is_in_broad_phase) != 0;
    }

    /// Flag if body is in the broadphase (should only be called by the BroadPhase)
    pub fn setInBroadPhaseInternal(self: *Body, in_broad_phase: bool) void {
        if (in_broad_phase)
            _ = self.flags.fetchOr(Flags.is_in_broad_phase, .monotonic)
        else
            _ = self.flags.fetchAnd(~Flags.is_in_broad_phase, .monotonic);
    }

    pub fn getShape(self: *const Body) *const Shape {
        return self.shape.get().?;
    }

    pub fn getObjectLayer(self: *const Body) ObjectLayer {
        return self.object_layer;
    }

    /// World space position of the body (GetPosition: position of the shape origin)
    pub fn getPosition(self: *const Body) RVec3 {
        return self.position.subVec3(self.rotation.mulVec3(self.getShape().getCenterOfMass()));
    }

    /// World space position of the center of mass
    pub fn getCenterOfMassPosition(self: *const Body) RVec3 {
        return self.position;
    }

    pub fn getRotation(self: *const Body) Quat {
        return self.rotation;
    }

    /// Calculates the transform for this body's center of mass
    pub fn getCenterOfMassTransform(self: *const Body) RMat44 {
        return RMat44.rotationTranslation(self.rotation, self.position);
    }

    /// Get world space bounding box
    pub fn getWorldSpaceBounds(self: *const Body) *const AABox {
        return &self.bounds;
    }

    pub fn getUserData(self: *const Body) u64 {
        return self.user_data;
    }

    pub fn setUserData(self: *Body, user_data: u64) void {
        self.user_data = user_data;
    }

    pub fn getLinearVelocity(self: *const Body) Vec3 {
        return if (self.motion_properties) |mp| mp.getLinearVelocity() else Vec3.zero();
    }

    pub fn setLinearVelocity(self: *Body, linear_velocity: Vec3) void {
        std.debug.assert(!self.isStatic());
        self.motion_properties.?.linear_velocity = linear_velocity;
        self.motion_properties.?.clampLinearVelocity();
    }

    pub fn getAngularVelocity(self: *const Body) Vec3 {
        return if (self.motion_properties) |mp| mp.angular_velocity else Vec3.zero();
    }

    /// Add force (unit: N) at center of mass for the next time step, will be reset after the next call to PhysicsSystem::Update
    pub fn addForce(self: *Body, force: Vec3) void {
        std.debug.assert(self.isDynamic());
        const mp = self.motion_properties.?;
        Vec3.fromFloat3(mp.force).add(force).storeFloat3(&mp.force);
    }

    /// Access to the motion properties (asserts that the body is not static). The motion properties live behind a
    /// pointer field, so a const body hands out a mutable pointer (Rule M allows this, like Jolt).
    pub fn getMotionProperties(self: *const Body) *MotionProperties {
        std.debug.assert(!self.isStatic());
        return self.motion_properties.?;
    }

    /// Access to the motion properties (version that does not check if the object is kinematic or dynamic)
    pub fn getMotionPropertiesUnchecked(self: *const Body) ?*MotionProperties {
        return self.motion_properties;
    }

    /// Access to the index in the BodyManager::mActiveBodies list
    pub fn getIndexInActiveBodiesInternal(self: *const Body) u32 {
        return if (self.motion_properties) |mp| mp.getIndexInActiveBodiesInternal() else inactive_index;
    }

    /// Reset the sleep timer (reduced: no sleep test spheres)
    pub fn resetSleepTimer(self: *Body) void {
        _ = self;
    }

    /// Updates world space bounding box (should only be called by the PhysicsSystem)
    pub fn calculateWorldSpaceBoundsInternal(self: *Body) void {
        self.bounds = self.getShape().getWorldSpaceBoundsRMat44(self.getCenterOfMassTransform(), Vec3.one());
    }

    /// Update position using an Euler step (used during position integrate & constraint solving)
    pub fn addPositionStep(self: *Body, linear_velocity_times_delta_time: Vec3) void {
        std.debug.assert(self.isRigidBody());
        self.position = self.position.addVec3(linear_velocity_times_delta_time);
        std.debug.assert(!self.position.isNaN());
    }

    /// Update rotation using an Euler step (used during position integrate & constraint solving)
    pub fn addRotationStep(self: *Body, angular_velocity_times_delta_time: Vec3) void {
        std.debug.assert(self.isRigidBody());

        // So now we split w(t) * dt into an axis and angle part and create a quaternion with it.
        // Note that the resulting quaternion is normalized since otherwise numerical drift will eventually make the rotation non-normalized.
        const len = angular_velocity_times_delta_time.length();
        if (len > 1.0e-6) {
            self.rotation = Quat.rotation(angular_velocity_times_delta_time.divScalar(len), len).mul(self.rotation).normalized();
            std.debug.assert(!self.rotation.isNaN());
        }
    }

    /// Helper function for BroadPhase::FindCollidingPairs that returns true when two bodies can collide
    /// It assumes that body 1 is dynamic and active and guarantees that it body 1 collides with body 2 that body 2 will not collide with body 1 in order to avoid finding duplicate collision pairs
    pub fn findCollidingPairsCanCollide(body1: *const Body, body2: *const Body) bool {
        // First body should never be a soft body
        std.debug.assert(!body1.isSoftBody());

        // One of these conditions must be true
        if (!body1.getCollideKinematicVsNonDynamic() and !body2.getCollideKinematicVsNonDynamic() and (!body1.isDynamic() and !body2.isDynamic()) and !(body1.isKinematic() and body2.isSensor()) and !(body2.isKinematic() and body1.isSensor()))
            return false;

        // Check that body 1 is active
        const body1_index_in_active_bodies = body1.getIndexInActiveBodiesInternal();
        std.debug.assert(!body1.isStatic() and body1_index_in_active_bodies != inactive_index); // This function assumes that Body 1 is active

        // If the pair A, B collides we need to ensure that the pair B, A does not collide or else we will handle the collision twice.
        // If A is the same body as B we don't want to collide (1)
        // If A is dynamic / kinematic and B is static we should collide (2)
        // If A is dynamic / kinematic and B is dynamic / kinematic we should only collide if
        //  - A is active and B is not active (3)
        //  - A is active and B will become active during this simulation step (4)
        //  - A is active and B is active, we require a condition that makes A, B collide and B, A not (5)
        //
        // In order to implement this we use the index in the active body list and make use of the fact that
        // a body not in the active list has Body.Index = 0xffffffff which is the highest possible value for an uint32.
        //
        // Because we know that A is active we know that A.Index != 0xffffffff:
        // (1) Because A.Index != 0xffffffff, if A.Index = B.Index then A = B, so to collide A.Index != B.Index
        // (2) A.Index != 0xffffffff, B.Index = 0xffffffff (because it's static and cannot be in the active list), so to collide A.Index != B.Index
        // (3) A.Index != 0xffffffff, B.Index = 0xffffffff (because it's not yet active), so to collide A.Index != B.Index
        // (4) A.Index != 0xffffffff, B.Index = 0xffffffff currently. But it can activate during the Broad/NarrowPhase step at which point it
        //     will be added to the end of the active list which will make B.Index > A.Index (this holds only true when we don't deactivate
        //     bodies during the Broad/NarrowPhase step), so to collide A.Index < B.Index.
        // (5) As tie breaker we can use the same condition A.Index < B.Index to collide, this means that if A, B collides then B, A won't
        if (!body2.isSoftBody() and body1_index_in_active_bodies >= body2.getIndexInActiveBodiesInternal())
            return false;
        std.debug.assert(!body1.getID().eql(body2.getID())); // A and B are the same body which should not be possible!

        // Check collision group filter
        if (!body1.collision_group.canCollide(&body2.collision_group))
            return false;

        return true;
    }

    /// Saving state for replay
    pub fn saveState(self: *const Body, stream: *StateRecorder) void {
        // Only write properties that can change at runtime
        stream.write(self.position);
        stream.write(self.rotation);

        if (self.motion_properties) |mp| {
            if (self.isSoftBody())
                SoftBodyMotionProperties.fromBaseConst(mp).saveState(stream) // Unreachable before Phase 9
            else
                mp.saveState(stream);
        }
    }

    /// Restoring state for replay
    pub fn restoreState(self: *Body, stream: *StateRecorder) void {
        stream.read(&self.position);
        stream.read(&self.rotation);

        if (self.motion_properties) |mp| {
            if (self.isSoftBody())
                SoftBodyMotionProperties.fromBase(mp).restoreState(stream) // Unreachable before Phase 9
            else
                mp.restoreState(stream);
        }

        // Initialize bounding box
        self.calculateWorldSpaceBoundsInternal();
    }
};

/// Settings for constructing a rigid body (reduced)
pub const BodyCreationSettings = struct {
    position: RVec3 = RVec3.zero(),
    rotation: Quat = Quat.identity(),
    linear_velocity: Vec3 = Vec3.zero(),
    angular_velocity: Vec3 = Vec3.zero(),
    user_data: u64 = 0,
    object_layer: ObjectLayer = 0,
    collision_group: CollisionGroup = .{},
    motion_type: MotionType = .dynamic,
    allowed_dofs: AllowedDOFs = .all,
    allow_dynamic_or_kinematic: bool = false,
    is_sensor: bool = false,
    motion_quality: MotionQuality = .discrete,
    allow_sleeping: bool = true,
    friction: f32 = 0.2,
    restitution: f32 = 0.0,
    linear_damping: f32 = 0.05,
    angular_damping: f32 = 0.05,
    max_linear_velocity: f32 = 500.0,
    max_angular_velocity: f32 = 0.25 * math.pi * 60.0,
    gravity_factor: f32 = 1.0,

    /// Collision volume for the body. Jolt's private, mutually exclusive `RefConst<ShapeSettings> mShape` (can be
    /// serialized) and `RefConst<Shape> mShapePtr` (cannot be serialized): a tagged union makes the exclusion structural.
    shape: ShapeSource = .none,

    pub const ShapeSource = union(enum) {
        none,
        /// `Ref`, not Jolt's RefConst: creating the shape writes the settings' cache (Rule M, CollisionArchitecture D3)
        settings: Ref(ShapeSettings),
        shape: RefConst(Shape),
    };

    /// BodyCreationSettings(const ShapeSettings *, position, rotation, motion type, object layer)
    pub fn initWithSettings(shape: *ShapeSettings, position: RVec3, rotation: Quat, motion_type: MotionType, object_layer: ObjectLayer) BodyCreationSettings {
        return .{ .position = position, .rotation = rotation, .object_layer = object_layer, .motion_type = motion_type, .shape = .{ .settings = .init(shape) } };
    }

    /// BodyCreationSettings(const Shape *, position, rotation, motion type, object layer)
    pub fn initWithShape(shape: *const Shape, position: RVec3, rotation: Quat, motion_type: MotionType, object_layer: ObjectLayer) BodyCreationSettings {
        return .{ .position = position, .rotation = rotation, .object_layer = object_layer, .motion_type = motion_type, .shape = .{ .shape = .init(shape) } };
    }

    /// Destructor (releases the references)
    pub fn deinit(self: *BodyCreationSettings) void {
        self.releaseShape();
        self.collision_group.deinit();
    }

    fn releaseShape(self: *BodyCreationSettings) void {
        switch (self.shape) {
            .none => {},
            .settings => |*s| s.deinit(),
            .shape => |*s| s.deinit(),
        }
        self.shape = .none;
    }

    /// Copy constructor
    pub fn clone(self: *const BodyCreationSettings) BodyCreationSettings {
        var copy = self.*;
        copy.shape = switch (self.shape) {
            .none => .none,
            .settings => |s| .{ .settings = s.clone() },
            .shape => |s| .{ .shape = s.clone() },
        };
        copy.collision_group = self.collision_group.clone();
        return copy;
    }

    /// Access to the shape settings object. This contains serializable (non-runtime optimized) information about the Shape.
    pub fn getShapeSettings(self: *const BodyCreationSettings) ?*ShapeSettings {
        return switch (self.shape) {
            .settings => |s| s.get(),
            else => null,
        };
    }

    /// Sets the shape settings, and invalidates the shape (SetShapeSettings)
    pub fn setShapeSettings(self: *BodyCreationSettings, shape: *ShapeSettings) void {
        const new: ShapeSource = .{ .settings = .init(shape) }; // Reference first: the old and new object may be the same
        self.releaseShape();
        self.shape = new;
    }

    /// Sets the shape and invalidates the shape settings (SetShape)
    pub fn setShape(self: *BodyCreationSettings, shape: *const Shape) void {
        const new: ShapeSource = .{ .shape = .init(shape) };
        self.releaseShape();
        self.shape = new;
    }

    /// Convert ShapeSettings object into a Shape object. This will free the ShapeSettings object and make the object ready for runtime. Serialization is no longer possible after this.
    pub fn convertShapeSettings(self: *BodyCreationSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        switch (self.shape) {
            // If we already have a shape, return it
            .shape => |s| {
                var result: ShapeResult = .empty;
                result.set(.init(@constCast(s.get().?))); // Jolt: const_cast, only the reference count is written (Rule M exception)
                return result;
            },

            // Check if we have shape settings
            .none => {
                var result: ShapeResult = .empty;
                result.setError("No shape present!");
                return result;
            },

            // Create the shape
            .settings => |s| {
                const result = try s.get().?.createShape(allocator);
                if (result.isValid())
                    self.setShape(result.getPtr().?)
                else
                    self.releaseShape();
                return result;
            },
        }
    }

    /// Access to the run-time shape object. Will convert from ShapeSettings object if needed. `self` is const like
    /// Jolt's: the shape settings' cache is written through the pointer in the union (Rule M). Jolt's semantics: null
    /// when there is no shape; on a creation error Jolt traces and asserts, then returns null.
    pub fn getShape(self: *const BodyCreationSettings, allocator: Allocator) Allocator.Error!?*const Shape {
        switch (self.shape) {
            // If we already have a shape, return it
            .shape => |s| return s.get().?,

            // Check if we have shape settings
            .none => return null,

            // Create the shape
            .settings => |s| {
                var result = try s.get().?.createShape(allocator);
                defer result.deinit();
                if (result.isValid())
                    return result.getPtr().?; // The settings' cache keeps the shape alive

                std.log.scoped(.zolt).warn("Error: {s}", .{result.getError()});
                if (Core.enable_asserts) @panic("An error occurred during shape creation. Use convertShapeSettings() to convert the shape and get the error!");
                return null;
            },
        }
    }

    /// Check if body mass properties are needed
    pub fn hasMassProperties(self: *const BodyCreationSettings) bool {
        return self.allow_dynamic_or_kinematic or self.motion_type != .static;
    }

    /// Calculate (or return when overridden) the mass and inertia for this body (reduced: CalculateMassAndInertia)
    pub fn getMassProperties(self: *const BodyCreationSettings, shape: *const Shape) MassProperties {
        _ = self;
        return shape.getMassProperties();
    }
};
