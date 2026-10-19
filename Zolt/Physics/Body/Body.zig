//! Port of: Jolt/Physics/Body/Body.h, Jolt/Physics/Body/Body.inl, Jolt/Physics/Body/Body.cpp
//! Status: complete
//!
//! SimulationArchitecture D1:
//! - `Body` is a plain struct with Jolt's fields in Jolt's order and alignment groups; its size (128 / 160 bytes) and
//!   alignment are checked at compile time. Members that Jolt's private default constructor leaves uninitialized default
//!   to `undefined`, so `Body{}` is Jolt's `Body()` (bodies are created by BodyManager.allocateBody; tests may create
//!   `Body{ .id = ... }` to pass a body to a collector or listener).
//! - `Body::EFlags` are the integer constants of `Body.Flags` on the atomic `flags`; Jolt's `memory_order_relaxed` is
//!   `.monotonic`.
//! - `Body::sFixedToWorld` is `Body.fixedToWorld()`: a compile time constant in read-only memory (Jolt's `Body(bool)`
//!   constructor on a zero initialized static) whose dummy shape is a static EmptyShape (never reference counted). It is
//!   the one audited `@constCast` of Phase 5: constraints store `*Body` like Jolt's `Body &`, nothing writes a static
//!   body, and a stray write faults instead of changing state that all physics systems share.
//! - Soft body branches are ported. `applySoftBodyCreationSettings` returns `Allocator.Error`: the motion properties
//!   allocate their vertices, rod states and skin states (RemainingFeaturesArchitecture.md R16).
//! - Const / non-const overloads: `getMotionProperties` / `getMotionPropertiesMut`, `getMotionPropertiesUnchecked` /
//!   `getMotionPropertiesUncheckedMut`, `getCollisionGroup` / `getCollisionGroupMut`.
//! - Overloads: `AddForce(force, position)` is `addForceAtPosition`, `AddImpulse(impulse, position)` is
//!   `addImpulseAtPosition`, `ApplyBuoyancyImpulse(total volume, submerged volume, ...)` is
//!   `applyBuoyancyImpulseWithVolumes`. Out parameters become returned structs (`getSubmergedVolume`), the defaulted
//!   `inResetSleepTimer` is an option (`setPositionAndRotationInternal(p, r, .{ .reset_sleep_timer = false })`).
//! - `GetBodyCreationSettings` returns a settings value that owns references (`deinit` it);
//!   `applyBodyCreationSettings` takes an allocator (it may create the shape from its settings) and returns
//!   `CreateBodyError` before changing anything when there is no shape.
//! - The BodyAccess checks are `BodyAccess.assertPositionAccess(.read)` etc. (only evaluated with asserts).
//! - JPH_DEBUG_RENDERER (the draw call in ApplyBuoyancyImpulse, the debug position in GetSubmergedVolume):
//!   TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const DVec3 = @import("../../Math/DVec3.zig").DVec3;
const Real = @import("../../Math/Real.zig");
const RVec3 = Real.RVec3;
const RMat44 = Real.RMat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const EmptyShape = @import("../Collision/Shape/EmptyShape.zig").EmptyShape;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const object_layer_invalid = @import("../Collision/ObjectLayer.zig").object_layer_invalid;
const BroadPhaseLayerFile = @import("../Collision/BroadPhase/BroadPhaseLayer.zig");
const BroadPhaseLayer = BroadPhaseLayerFile.BroadPhaseLayer;
const BroadPhaseLayerInterface = BroadPhaseLayerFile.BroadPhaseLayerInterface;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const BodyID = @import("BodyID.zig").BodyID;
const BodyType = @import("BodyType.zig").BodyType;
const MotionType = @import("MotionType.zig").MotionType;
const BodyAccess = @import("BodyAccess.zig").BodyAccess;
const MotionPropertiesFile = @import("MotionProperties.zig");
const MotionProperties = MotionPropertiesFile.MotionProperties;
const CanSleep = MotionPropertiesFile.CanSleep;
const MassProperties = @import("MassProperties.zig").MassProperties;
const BodyCreationSettingsFile = @import("BodyCreationSettings.zig");
const BodyCreationSettings = BodyCreationSettingsFile.BodyCreationSettings;
const CreateBodyError = BodyCreationSettingsFile.CreateBodyError;
const SoftBodyCreationSettings = @import("../SoftBody/SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const SoftBodyMotionProperties = @import("../SoftBody/SoftBodyMotionProperties.zig").SoftBodyMotionProperties;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;

/// A rigid body that can be simulated using the physics system
///
/// Note that internally all properties (position, velocity etc.) are tracked relative to the center of mass of the object to simplify the simulation of the object.
///
/// The offset between the position of the body and the center of mass position of the body is GetShape()->GetCenterOfMass().
/// The functions that get/set the position of the body all indicate if they are relative to the center of mass or to the original position in which the shape was created.
///
/// The linear velocity is also velocity of the center of mass, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$.
pub const Body = struct {
    /// Constant indicating that body is not active
    pub const inactive_index: u32 = MotionProperties.inactive_index;

    /// Body::EFlags: the bits of `flags`, changed with atomic or / and like Jolt
    pub const Flags = struct {
        /// If this object is a sensor. A sensor will receive collision callbacks, but will not cause any collision responses and can be used as a trigger volume.
        pub const is_sensor: u8 = 1 << 0;
        /// If kinematic objects can generate contact points against other kinematic or static objects.
        pub const collide_kinematic_vs_non_dynamic: u8 = 1 << 1;
        /// Set this bit to indicate that the body is in the broadphase
        pub const is_in_broad_phase: u8 = 1 << 2;
        /// Set this bit to indicate that all collision caches for this body are invalid, will be reset the next simulation step.
        pub const invalidate_contact_cache: u8 = 1 << 3;
        /// Set this bit to indicate that this body can use manifold reduction (if PhysicsSettings::mUseManifoldReduction is true)
        pub const use_manifold_reduction: u8 = 1 << 4;
        /// Set this bit to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
        pub const apply_gyroscopic_force: u8 = 1 << 5;
        /// Set this bit to indicate that enhanced internal edge removal should be used for this body (see BodyCreationSettings::mEnhancedInternalEdgeRemoval)
        pub const enhanced_internal_edge_removal: u8 = 1 << 6;
    };

    /// Result of GetSubmergedVolume (Jolt's out parameters)
    pub const SubmergedVolume = struct {
        /// The total volume of the shape
        total_volume: f32,
        /// The submerged volume of the shape
        submerged_volume: f32,
        /// The center of mass of the submerged volume relative to the center of mass of the body
        relative_center_of_buoyancy: Vec3,
    };

    // 16 byte aligned
    /// World space position of center of mass
    position: RVec3 = undefined,
    /// World space rotation of center of mass
    rotation: Quat = undefined,
    /// World space bounding box of the body
    bounds: AABox = .empty,

    // 8 byte aligned
    /// Shape representing the volume of this body
    shape: RefConst(Shape) = .empty,
    /// If this is a keyframed or dynamic object, this object holds all information about the movement
    motion_properties: ?*MotionProperties = null,
    /// User data, can be used for anything by the application
    user_data: u64 = 0,
    /// The collision group this body belongs to (determines if two objects can collide)
    collision_group: CollisionGroup = .{},

    // 4 byte aligned
    /// Friction of the body (dimensionless number, usually between 0 and 1, 0 = no friction, 1 = friction force equals force that presses the two bodies together). Note that bodies can have negative friction but the combined friction (see PhysicsSystem::SetCombineFriction) should never go below zero.
    friction: f32 = undefined,
    /// Restitution of body (dimensionless number, usually between 0 and 1, 0 = completely inelastic collision response, 1 = completely elastic collision response). Note that bodies can have negative restitution but the combined restitution (see PhysicsSystem::SetCombineRestitution) should never go below zero.
    restitution: f32 = undefined,
    /// ID of the body (index in the bodies array)
    id: BodyID = .invalid,

    // 2 or 4 bytes aligned
    /// The collision layer this body belongs to (determines if two objects can collide)
    object_layer: ObjectLayer = undefined,

    // 1 byte aligned
    /// Type of body (rigid or soft)
    body_type: BodyType = undefined,
    /// The broad phase layer this body belongs to
    broad_phase_layer: BroadPhaseLayer = undefined,
    /// Type of motion (static, dynamic or kinematic)
    motion_type: MotionType = undefined,
    /// See Flags for possible flags
    flags: std.atomic.Value(u8) = .init(0),

    // 122 bytes up to here (64-bit mode, single precision, 16-bit ObjectLayer)

    comptime {
        // static_assert(sizeof(Body) == JPH_IF_SINGLE_PRECISION_ELSE(128, 160)), "Body size is incorrect"): holds with Zig's field ordering too
        if (@sizeOf(usize) == 8 and Core.object_layer_bits == 16)
            std.debug.assert(@sizeOf(Body) == if (Core.double_precision) 160 else 128);
        // static_assert(alignof(Body) == max(JPH_VECTOR_ALIGNMENT, JPH_RVECTOR_ALIGNMENT), "Body should properly align")
        std.debug.assert(@alignOf(Body) == @max(Core.vector_alignment, Real.rvector_alignment));
    }

    /// static const EmptyShape sFixedToWorldShape: a static shape (never reference counted) in read-only memory
    const fixed_to_world_shape: EmptyShape = .initStatic(Vec3.zero());

    /// Body Body::sFixedToWorld(false): Jolt's `Body(bool)` constructor on a static object, whose other members are zero
    /// initialized (rigid body, broad phase layer 0) or default constructed (invalid ID, empty bounds)
    const fixed_to_world_storage: Body = .{
        .position = RVec3.zero(),
        .rotation = Quat.identity(),
        .shape = .{ .ptr = fixed_to_world_shape.asShape() }, // Dummy shape (static: no reference to add)
        .friction = 0.0,
        .restitution = 0.0,
        .object_layer = object_layer_invalid,
        .body_type = .rigid_body,
        .broad_phase_layer = .init(0),
        .motion_type = .static,
    };

    /// A dummy body that can be used by constraints to attach a constraint to the world instead of another body (Body::sFixedToWorld).
    /// It lives in read-only memory: never write to it (see the file comment).
    pub fn fixedToWorld() *Body {
        return @constCast(&fixed_to_world_storage);
    }

    /// Get the id of this body
    pub fn getID(self: *const Body) BodyID {
        return self.id;
    }

    /// Get the type of body (rigid or soft)
    pub fn getBodyType(self: *const Body) BodyType {
        return self.body_type;
    }

    /// Check if this body is a rigid body
    pub fn isRigidBody(self: *const Body) bool {
        return self.body_type == .rigid_body;
    }

    /// Check if this body is a soft body
    pub fn isSoftBody(self: *const Body) bool {
        return self.body_type == .soft_body;
    }

    // See comment at GetIndexInActiveBodiesInternal for reasoning why TSAN is disabled here (Zolt: a monotonic atomic)
    /// If this body is currently actively simulating (true) or sleeping (false)
    pub fn isActive(self: *const Body) bool {
        return self.motion_properties != null and self.motion_properties.?.index_in_active_bodies.load(.monotonic) != inactive_index;
    }

    /// Check if this body is static (not movable)
    pub fn isStatic(self: *const Body) bool {
        return self.motion_type == .static;
    }

    /// Check if this body is kinematic (keyframed), which means that it will move according to its current velocity, but forces don't affect it
    pub fn isKinematic(self: *const Body) bool {
        return self.motion_type == .kinematic;
    }

    /// Check if this body is dynamic, which means that it moves and forces can act on it
    pub fn isDynamic(self: *const Body) bool {
        return self.motion_type == .dynamic;
    }

    /// Check if a body could be made kinematic or dynamic (if it was created dynamic or with mAllowDynamicOrKinematic set to true)
    pub fn canBeKinematicOrDynamic(self: *const Body) bool {
        return self.motion_properties != null;
    }

    /// Set or clear a flag with an atomic or / and (memory_order_relaxed)
    fn setFlag(self: *Body, flag: u8, set: bool) void {
        if (set)
            _ = self.flags.fetchOr(flag, .monotonic)
        else
            _ = self.flags.fetchAnd(~flag, .monotonic);
    }

    /// Test a flag (memory_order_relaxed)
    fn getFlag(self: *const Body, flag: u8) bool {
        return (self.flags.load(.monotonic) & flag) != 0;
    }

    /// Change the body to a sensor. A sensor will receive collision callbacks, but will not cause any collision responses and can be used as a trigger volume.
    /// The cheapest sensor (in terms of CPU usage) is a sensor with motion type Static (they can be moved around using BodyInterface::SetPosition/SetPositionAndRotation).
    /// These sensors will only detect collisions with active Dynamic or Kinematic bodies. As soon as a body go to sleep, the contact point with the sensor will be lost.
    /// If you make a sensor Dynamic or Kinematic and activate them, the sensor will be able to detect collisions with sleeping bodies too. An active sensor will never go to sleep automatically.
    /// When you make a Dynamic or Kinematic sensor, make sure it is in an ObjectLayer that does not collide with Static bodies or other sensors to avoid extra overhead in the broad phase.
    pub fn setIsSensor(self: *Body, is_sensor: bool) void {
        std.debug.assert(self.isRigidBody());
        self.setFlag(Flags.is_sensor, is_sensor);
    }

    /// Check if this body is a sensor.
    pub fn isSensor(self: *const Body) bool {
        return self.getFlag(Flags.is_sensor);
    }

    /// If kinematic objects can generate contact points against other kinematic or static objects.
    /// Note that turning this on can be CPU intensive as much more collision detection work will be done without any effect on the simulation (kinematic objects are not affected by other kinematic/static objects).
    /// This can be used to make sensors detect static objects. Note that the sensor must be kinematic and active for it to detect static objects.
    pub fn setCollideKinematicVsNonDynamic(self: *Body, collide: bool) void {
        std.debug.assert(self.isRigidBody());
        self.setFlag(Flags.collide_kinematic_vs_non_dynamic, collide);
    }

    /// Check if kinematic objects can generate contact points against other kinematic or static objects.
    pub fn getCollideKinematicVsNonDynamic(self: *const Body) bool {
        return self.getFlag(Flags.collide_kinematic_vs_non_dynamic);
    }

    /// If PhysicsSettings::mUseManifoldReduction is true, this allows turning off manifold reduction for this specific body.
    /// Manifold reduction by default will combine contacts with similar normals that come from different SubShapeIDs (e.g. different triangles in a mesh shape or different compound shapes).
    /// If the application requires tracking exactly which SubShapeIDs are in contact, you can turn off manifold reduction. Note that this comes at a performance cost.
    /// Consider using BodyInterface::SetUseManifoldReduction if the body could already be in contact with other bodies to ensure that the contact cache is invalidated and you get the correct contact callbacks.
    pub fn setUseManifoldReduction(self: *Body, use_reduction: bool) void {
        std.debug.assert(self.isRigidBody());
        self.setFlag(Flags.use_manifold_reduction, use_reduction);
    }

    /// Check if this body can use manifold reduction.
    pub fn getUseManifoldReduction(self: *const Body) bool {
        return self.getFlag(Flags.use_manifold_reduction);
    }

    /// Checks if the combination of this body and body2 should use manifold reduction
    pub fn getUseManifoldReductionWithBody(self: *const Body, body2: *const Body) bool {
        return ((self.flags.load(.monotonic) & body2.flags.load(.monotonic)) & Flags.use_manifold_reduction) != 0;
    }

    /// Set to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
    pub fn setApplyGyroscopicForce(self: *Body, apply: bool) void {
        std.debug.assert(self.isRigidBody());
        self.setFlag(Flags.apply_gyroscopic_force, apply);
    }

    /// Check if the gyroscopic force is being applied for this body
    pub fn getApplyGyroscopicForce(self: *const Body) bool {
        return self.getFlag(Flags.apply_gyroscopic_force);
    }

    /// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
    pub fn setEnhancedInternalEdgeRemoval(self: *Body, apply: bool) void {
        std.debug.assert(self.isRigidBody());
        self.setFlag(Flags.enhanced_internal_edge_removal, apply);
    }

    /// Check if enhanced internal edge removal is turned on
    pub fn getEnhancedInternalEdgeRemoval(self: *const Body) bool {
        return self.getFlag(Flags.enhanced_internal_edge_removal);
    }

    /// Checks if the combination of this body and body2 should use enhanced internal edge removal
    pub fn getEnhancedInternalEdgeRemovalWithBody(self: *const Body, body2: *const Body) bool {
        return ((self.flags.load(.monotonic) | body2.flags.load(.monotonic)) & Flags.enhanced_internal_edge_removal) != 0;
    }

    /// Get the bodies motion type.
    pub fn getMotionType(self: *const Body) MotionType {
        return self.motion_type;
    }

    /// Set the motion type of this body. Consider using BodyInterface::SetMotionType instead of this function if the body may be active or if it needs to be activated.
    pub fn setMotionType(self: *Body, motion_type: MotionType) void {
        if (self.motion_type == motion_type)
            return;

        if (Core.enable_asserts) std.debug.assert(motion_type == .static or self.motion_properties != null); // Body needs to be created with mAllowDynamicOrKinematic set to true
        if (Core.enable_asserts) std.debug.assert(motion_type != .static or !self.isActive()); // Deactivate body first
        if (Core.enable_asserts) std.debug.assert(motion_type == .dynamic or !self.isSoftBody()); // Soft bodies can only be dynamic, you can make individual vertices kinematic by setting their inverse mass to 0

        // Store new motion type
        self.motion_type = motion_type;

        if (self.motion_properties) |mp| {
            // Update cache
            if (Core.enable_asserts) mp.cached_motion_type = motion_type;

            switch (motion_type) {
                .static, .kinematic => {
                    if (motion_type == .static) {
                        // Stop the object
                        mp.linear_velocity = Vec3.zero();
                        mp.angular_velocity = Vec3.zero();
                    }
                    // [[fallthrough]]

                    // Cancel forces
                    mp.resetForce();
                    mp.resetTorque();
                },

                .dynamic => {},
            }
        }
    }

    /// Get broadphase layer, this determines in which broad phase sub-tree the object is placed
    pub fn getBroadPhaseLayer(self: *const Body) BroadPhaseLayer {
        return self.broad_phase_layer;
    }

    /// Get object layer, this determines which other objects it collides with
    pub fn getObjectLayer(self: *const Body) ObjectLayer {
        return self.object_layer;
    }

    /// Collision group and sub-group ID, determines which other objects it collides with
    pub fn getCollisionGroup(self: *const Body) *const CollisionGroup {
        return &self.collision_group;
    }

    /// Collision group and sub-group ID, determines which other objects it collides with (non-const overload)
    pub fn getCollisionGroupMut(self: *Body) *CollisionGroup {
        return &self.collision_group;
    }

    /// Set the collision group (a copy: adds a reference to the group filter)
    pub fn setCollisionGroup(self: *Body, group: *const CollisionGroup) void {
        self.collision_group.assign(group);
    }

    /// If this body can go to sleep. Note that disabling sleeping on a sleeping object will not wake it up.
    pub fn getAllowSleeping(self: *const Body) bool {
        return self.motion_properties.?.allow_sleeping;
    }

    pub fn setAllowSleeping(self: *Body, allow: bool) void {
        self.motion_properties.?.allow_sleeping = allow;
        if (allow)
            self.resetSleepTimer();
    }

    /// Resets the sleep timer. This does not wake up the body if it is sleeping, but allows resetting the system that detects when a body is sleeping.
    pub fn resetSleepTimer(self: *Body) void {
        const points = self.getSleepTestPoints();
        self.motion_properties.?.resetSleepTestSpheres(&points);
    }

    /// Friction (dimensionless number, usually between 0 and 1, 0 = no friction, 1 = friction force equals force that presses the two bodies together). Note that bodies can have negative friction but the combined friction (see PhysicsSystem::SetCombineFriction) should never go below zero.
    pub fn getFriction(self: *const Body) f32 {
        return self.friction;
    }

    pub fn setFriction(self: *Body, friction: f32) void {
        self.friction = friction;
    }

    /// Restitution (dimensionless number, usually between 0 and 1, 0 = completely inelastic collision response, 1 = completely elastic collision response). Note that bodies can have negative restitution but the combined restitution (see PhysicsSystem::SetCombineRestitution) should never go below zero.
    pub fn getRestitution(self: *const Body) f32 {
        return self.restitution;
    }

    pub fn setRestitution(self: *Body, restitution: f32) void {
        self.restitution = restitution;
    }

    /// Get world space linear velocity of the center of mass (unit: m/s)
    pub fn getLinearVelocity(self: *const Body) Vec3 {
        return if (!self.isStatic()) self.motion_properties.?.getLinearVelocity() else Vec3.zero();
    }

    /// Set world space linear velocity of the center of mass (unit: m/s).
    /// Note that it is illegal to set a non-zero linear velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetLinearVelocity instead.
    pub fn setLinearVelocity(self: *Body, linear_velocity: Vec3) void {
        std.debug.assert(!self.isStatic());
        self.motion_properties.?.setLinearVelocity(linear_velocity);
    }

    /// Set world space linear velocity of the center of mass, will make sure the value is clamped against the maximum linear velocity.
    /// Note that it is illegal to set a non-zero linear velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetLinearVelocity instead.
    pub fn setLinearVelocityClamped(self: *Body, linear_velocity: Vec3) void {
        std.debug.assert(!self.isStatic());
        self.motion_properties.?.setLinearVelocityClamped(linear_velocity);
    }

    /// Get world space angular velocity of the center of mass (unit: rad/s)
    pub fn getAngularVelocity(self: *const Body) Vec3 {
        return if (!self.isStatic()) self.motion_properties.?.getAngularVelocity() else Vec3.zero();
    }

    /// Set world space angular velocity of the center of mass (unit: rad/s).
    /// Note that it is illegal to set a non-zero angular velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetAngularVelocity instead.
    pub fn setAngularVelocity(self: *Body, angular_velocity: Vec3) void {
        std.debug.assert(!self.isStatic());
        self.motion_properties.?.setAngularVelocity(angular_velocity);
    }

    /// Set world space angular velocity of the center of mass, will make sure the value is clamped against the maximum angular velocity.
    /// Note that it is illegal to set a non-zero angular velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetAngularVelocity instead.
    pub fn setAngularVelocityClamped(self: *Body, angular_velocity: Vec3) void {
        std.debug.assert(!self.isStatic());
        self.motion_properties.?.setAngularVelocityClamped(angular_velocity);
    }

    /// Velocity of point point_relative_to_com (in center of mass space, e.g. on the surface of the body) of the body (unit: m/s)
    pub fn getPointVelocityCOM(self: *const Body, point_relative_to_com: Vec3) Vec3 {
        return if (!self.isStatic()) self.motion_properties.?.getPointVelocityCOM(point_relative_to_com) else Vec3.zero();
    }

    /// Velocity of point point (in world space, e.g. on the surface of the body) of the body (unit: m/s)
    pub fn getPointVelocity(self: *const Body, point: RVec3) Vec3 {
        BodyAccess.assertPositionAccess(.read);
        return self.getPointVelocityCOM(point.sub(self.position).toVec3());
    }

    /// Add force (unit: N) at center of mass for the next time step, will be reset after the next call to PhysicsSystem::Update.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::AddForce instead.
    pub fn addForce(self: *Body, force: Vec3) void {
        std.debug.assert(self.isDynamic());
        const mp = self.motion_properties.?;
        Vec3.loadFloat3Unsafe(&mp.force).add(force).storeFloat3(&mp.force);
    }

    /// Add force (unit: N) at world space position position for the next time step, will be reset after the next call to PhysicsSystem::Update.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::AddForce instead. (AddForce(inForce, inPosition))
    pub fn addForceAtPosition(self: *Body, force: Vec3, position: RVec3) void {
        self.addForce(force);
        self.addTorque(position.sub(self.position).toVec3().cross(force));
    }

    /// Add torque (unit: N m) for the next time step, will be reset after the next call to PhysicsSystem::Update.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::AddTorque instead.
    pub fn addTorque(self: *Body, torque: Vec3) void {
        std.debug.assert(self.isDynamic());
        const mp = self.motion_properties.?;
        Vec3.loadFloat3Unsafe(&mp.torque).add(torque).storeFloat3(&mp.torque);
    }

    // Get the total amount of force applied to the center of mass this time step (through AddForce calls). Note that it will reset to zero after PhysicsSystem::Update.
    pub fn getAccumulatedForce(self: *const Body) Vec3 {
        std.debug.assert(self.isDynamic());
        return self.motion_properties.?.getAccumulatedForce();
    }

    // Get the total amount of torque applied to the center of mass this time step (through AddForce/AddTorque calls). Note that it will reset to zero after PhysicsSystem::Update.
    pub fn getAccumulatedTorque(self: *const Body) Vec3 {
        std.debug.assert(self.isDynamic());
        return self.motion_properties.?.getAccumulatedTorque();
    }

    // Reset the total accumulated force, not that this will be done automatically after every time step.
    pub fn resetForce(self: *Body) void {
        std.debug.assert(self.isDynamic());
        self.motion_properties.?.resetForce();
    }

    // Reset the total accumulated torque, not that this will be done automatically after every time step.
    pub fn resetTorque(self: *Body) void {
        std.debug.assert(self.isDynamic());
        self.motion_properties.?.resetTorque();
    }

    // Reset the current velocity and accumulated force and torque.
    pub fn resetMotion(self: *Body) void {
        std.debug.assert(!self.isStatic());
        self.motion_properties.?.resetMotion();
    }

    /// Get inverse inertia tensor in world space
    pub fn getInverseInertia(self: *const Body) Mat44 {
        std.debug.assert(self.isDynamic());

        return self.getMotionProperties().getInverseInertiaForRotation(Mat44.rotationQuat(self.rotation));
    }

    /// Add impulse to center of mass (unit: kg m/s).
    /// If you want the body to wake up when it is sleeping, use BodyInterface::AddImpulse instead.
    pub fn addImpulse(self: *Body, impulse: Vec3) void {
        std.debug.assert(self.isDynamic());

        const mp = self.motion_properties.?;
        self.setLinearVelocityClamped(mp.getLinearVelocity().add(impulse.mulScalar(mp.getInverseMass())));
    }

    /// Add impulse to point in world space (unit: kg m/s).
    /// If you want the body to wake up when it is sleeping, use BodyInterface::AddImpulse instead. (AddImpulse(inImpulse, inPosition))
    pub fn addImpulseAtPosition(self: *Body, impulse: Vec3, position: RVec3) void {
        std.debug.assert(self.isDynamic());

        const mp = self.motion_properties.?;
        self.setLinearVelocityClamped(mp.getLinearVelocity().add(impulse.mulScalar(mp.getInverseMass())));

        self.setAngularVelocityClamped(mp.getAngularVelocity().add(mp.multiplyWorldSpaceInverseInertiaByVector(self.rotation, position.sub(self.position).toVec3().cross(impulse))));
    }

    /// Add angular impulse in world space (unit: N m s).
    /// If you want the body to wake up when it is sleeping, use BodyInterface::AddAngularImpulse instead.
    pub fn addAngularImpulse(self: *Body, angular_impulse: Vec3) void {
        std.debug.assert(self.isDynamic());

        const mp = self.motion_properties.?;
        self.setAngularVelocityClamped(mp.getAngularVelocity().add(mp.multiplyWorldSpaceInverseInertiaByVector(self.rotation, angular_impulse)));
    }

    /// Set velocity of body such that it will be positioned at target_position/target_rotation in delta_time seconds.
    /// Note that it is illegal to provide a new position/orientation on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::MoveKinematic instead.
    pub fn moveKinematic(self: *Body, target_position: RVec3, target_rotation: Quat, delta_time: f32) void {
        std.debug.assert(self.isRigidBody()); // Only valid for rigid bodies
        std.debug.assert(!self.isStatic());
        BodyAccess.assertPositionAccess(.read);

        // Calculate center of mass at end situation
        const new_com = target_position.addVec3(target_rotation.mulVec3(self.getShape().getCenterOfMass()));

        // Calculate delta position and rotation
        const delta_pos = new_com.sub(self.position).toVec3();
        const delta_rotation = target_rotation.mul(self.rotation.conjugated());

        self.motion_properties.?.moveKinematic(delta_pos, delta_rotation, delta_time);
    }

    /// Gets the properties needed to do buoyancy calculations
    /// @param surface_position Position of the fluid surface in world space
    /// @param surface_normal Normal of the fluid surface (should point up)
    /// @return The total volume of the shape, the submerged volume of the shape and the center of mass of the submerged volume relative to the center of mass of the body
    pub fn getSubmergedVolume(self: *const Body, surface_position: RVec3, surface_normal: Vec3) SubmergedVolume {
        // For GetSubmergedVolume we transform the surface relative to the body position for increased precision
        const rotation = Mat44.rotationQuat(self.rotation);
        const surface_relative_to_body = Plane.fromPointAndNormalRVec3(surface_position.sub(self.position), surface_normal);

        // Calculate amount of volume that is submerged and what the center of buoyancy is
        // TODO(debug_renderer): JPH_IF_DEBUG_RENDERER(, mPosition) (the base offset for drawing)
        const result = self.getShape().getSubmergedVolume(rotation, Vec3.one(), surface_relative_to_body);
        return .{ .total_volume = result.total_volume, .submerged_volume = result.submerged_volume, .relative_center_of_buoyancy = result.center_of_buoyancy };
    }

    /// Applies an impulse to the body that simulates fluid buoyancy and drag.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::ApplyBuoyancyImpulse instead.
    /// @param surface_position Position of the fluid surface in world space
    /// @param surface_normal Normal of the fluid surface (should point up)
    /// @param buoyancy The buoyancy factor for the body. 1 = neutral body, < 1 sinks, > 1 floats. Note that we don't use the fluid density since it is harder to configure than a simple number between [0, 2]
    /// @param linear_drag Linear drag factor that slows down the body when in the fluid (approx. 0.5)
    /// @param angular_drag Angular drag factor that slows down rotation when the body is in the fluid (approx. 0.01)
    /// @param fluid_velocity The average velocity of the fluid (in m/s) in which the body resides
    /// @param gravity The gravity vector (pointing down)
    /// @param delta_time Delta time of the next simulation step (in s)
    /// @return true if an impulse was applied, false if the body was not in the fluid
    pub fn applyBuoyancyImpulse(self: *Body, surface_position: RVec3, surface_normal: Vec3, buoyancy: f32, linear_drag: f32, angular_drag: f32, fluid_velocity: Vec3, gravity: Vec3, delta_time: f32) bool {
        const volume = self.getSubmergedVolume(surface_position, surface_normal);

        return self.applyBuoyancyImpulseWithVolumes(volume.total_volume, volume.submerged_volume, volume.relative_center_of_buoyancy, buoyancy, linear_drag, angular_drag, fluid_velocity, gravity, delta_time);
    }

    /// Applies an impulse to the body that simulates fluid buoyancy and drag. (ApplyBuoyancyImpulse(inTotalVolume, inSubmergedVolume, ...))
    /// If you want the body to wake up when it is sleeping, use BodyInterface::ApplyBuoyancyImpulse instead.
    /// @param total_volume Total volume of the shape of this body (m^3)
    /// @param submerged_volume Submerged volume of the shape of this body (m^3)
    /// @param relative_center_of_buoyancy The center of mass of the submerged volume relative to the center of mass of the body
    /// @param buoyancy The buoyancy factor for the body. 1 = neutral body, < 1 sinks, > 1 floats. Note that we don't use the fluid density since it is harder to configure than a simple number between [0, 2]
    /// @param linear_drag Linear drag factor that slows down the body when in the fluid (approx. 0.5)
    /// @param angular_drag Angular drag factor that slows down rotation when the body is in the fluid (approx. 0.01)
    /// @param fluid_velocity The average velocity of the fluid (in m/s) in which the body resides
    /// @param gravity The gravity vector (pointing down)
    /// @param delta_time Delta time of the next simulation step (in s)
    /// @return true if an impulse was applied, false if the body was not in the fluid
    pub fn applyBuoyancyImpulseWithVolumes(self: *Body, total_volume: f32, submerged_volume: f32, relative_center_of_buoyancy: Vec3, buoyancy: f32, linear_drag: f32, angular_drag: f32, fluid_velocity: Vec3, gravity: Vec3, delta_time: f32) bool {
        std.debug.assert(self.isRigidBody()); // Only implemented for rigid bodies currently

        // We follow the approach from 'Game Programming Gems 6' 2.5 Exact Buoyancy for Polyhedra
        // All quantities below are in world space

        // If we're not submerged, there's no point in doing the rest of the calculations
        if (submerged_volume > 0.0) {
            // TODO(debug_renderer): draw the submerged volume properties (Shape::sDrawSubmergedVolumes)

            const mp = self.motion_properties.?;

            // When buoyancy is 1 we want neutral buoyancy, this means that the density of the liquid is the same as the density of the body at that point.
            // Buoyancy > 1 should make the object float, < 1 should make it sink.
            const inverse_mass = mp.getInverseMass();
            const fluid_density = buoyancy / (total_volume * inverse_mass);

            // Buoyancy force = Density of Fluid * Submerged volume * Magnitude of gravity * Up direction (eq 2.5.1)
            // Impulse = Force * Delta time
            // We should apply this at the center of buoyancy (= center of mass of submerged volume)
            const buoyancy_impulse = gravity.mulScalar(-fluid_density * submerged_volume * mp.getGravityFactor()).mulScalar(delta_time);

            // Calculate the velocity of the center of buoyancy relative to the fluid
            const linear_velocity = mp.getLinearVelocity();
            const angular_velocity = mp.getAngularVelocity();
            const center_of_buoyancy_velocity = linear_velocity.add(angular_velocity.cross(relative_center_of_buoyancy));
            const relative_center_of_buoyancy_velocity = fluid_velocity.sub(center_of_buoyancy_velocity);

            // Here we deviate from the article, instead of eq 2.5.14 we use a quadratic drag formula: https://en.wikipedia.org/wiki/Drag_%28physics%29
            // Drag force = 0.5 * Fluid Density * (Velocity of fluid - Velocity of center of buoyancy)^2 * Linear Drag * Area Facing the Relative Fluid Velocity
            // Again Impulse = Force * Delta Time
            // We should apply this at the center of buoyancy (= center of mass for submerged volume with no center of mass offset)

            // Get size of local bounding box
            const size = self.getShape().getLocalBounds().getSize();

            // Determine area of the local space bounding box in the direction of the relative velocity between the fluid and the center of buoyancy
            var area: f32 = 0.0;
            const relative_center_of_buoyancy_velocity_len_sq = relative_center_of_buoyancy_velocity.lengthSq();
            if (relative_center_of_buoyancy_velocity_len_sq > 1.0e-12) {
                const local_relative_center_of_buoyancy_velocity = self.getRotation().inverseRotate(relative_center_of_buoyancy_velocity);
                area = local_relative_center_of_buoyancy_velocity.abs().dot(size.swizzle(.y, .z, .x).mul(size.swizzle(.z, .x, .y))) / @sqrt(relative_center_of_buoyancy_velocity_len_sq);
            }

            // Calculate the impulse
            var drag_impulse = relative_center_of_buoyancy_velocity.mulScalar(0.5 * fluid_density * linear_drag * area * delta_time).mulScalar(relative_center_of_buoyancy_velocity.length());

            // Clamp magnitude against current linear velocity to prevent overshoot
            const linear_velocity_len_sq = linear_velocity.lengthSq();
            const drag_delta_linear_velocity_len_sq = drag_impulse.mulScalar(inverse_mass).lengthSq();
            if (drag_delta_linear_velocity_len_sq > linear_velocity_len_sq)
                drag_impulse = drag_impulse.mulScalar(@sqrt(linear_velocity_len_sq / drag_delta_linear_velocity_len_sq));

            // Calculate the resulting delta linear velocity due to buoyancy and drag
            const delta_linear_velocity = drag_impulse.add(buoyancy_impulse).mulScalar(inverse_mass);
            mp.addLinearVelocityStep(delta_linear_velocity);

            // Determine average width of the body (across the three axis)
            const l = (size.getX() + size.getY() + size.getZ()) / 3.0;

            // Drag torque = -Angular Drag * Mass * Submerged volume / Total volume * (Average width of body)^2 * Angular velocity (eq 2.5.15)
            const drag_angular_impulse = angular_velocity.mulScalar(-angular_drag * submerged_volume / total_volume * delta_time * math.square(l) / inverse_mass);
            const inv_inertia = self.getInverseInertia();
            var drag_delta_angular_velocity = inv_inertia.mulVec3(drag_angular_impulse);

            // Clamp magnitude against the current angular velocity to prevent overshoot
            const angular_velocity_len_sq = angular_velocity.lengthSq();
            const drag_delta_angular_velocity_len_sq = drag_delta_angular_velocity.lengthSq();
            if (drag_delta_angular_velocity_len_sq > angular_velocity_len_sq)
                drag_delta_angular_velocity = drag_delta_angular_velocity.mulScalar(@sqrt(angular_velocity_len_sq / drag_delta_angular_velocity_len_sq));

            // Calculate total delta angular velocity due to drag and buoyancy
            const delta_angular_velocity = drag_delta_angular_velocity.add(inv_inertia.mulVec3(relative_center_of_buoyancy.cross(buoyancy_impulse.add(drag_impulse))));
            mp.addAngularVelocityStep(delta_angular_velocity);
            return true;
        }

        return false;
    }

    /// Check if this body has been added to the physics system
    pub fn isInBroadPhase(self: *const Body) bool {
        return self.getFlag(Flags.is_in_broad_phase);
    }

    /// Check if this body has been changed in such a way that the collision cache should be considered invalid for any body interacting with this body
    pub fn isCollisionCacheInvalid(self: *const Body) bool {
        return self.getFlag(Flags.invalidate_contact_cache);
    }

    /// Get the shape of this body
    pub fn getShape(self: *const Body) *const Shape {
        return self.shape.get().?;
    }

    /// World space position of the body
    pub fn getPosition(self: *const Body) RVec3 {
        BodyAccess.assertPositionAccess(.read);
        return self.position.subVec3(self.rotation.mulVec3(self.getShape().getCenterOfMass()));
    }

    /// World space rotation of the body
    pub fn getRotation(self: *const Body) Quat {
        BodyAccess.assertPositionAccess(.read);
        return self.rotation;
    }

    /// Calculates the transform of this body
    pub fn getWorldTransform(self: *const Body) RMat44 {
        BodyAccess.assertPositionAccess(.read);

        return RMat44.rotationTranslation(self.rotation, self.position).preTranslated(self.getShape().getCenterOfMass().negate());
    }

    /// Gets the world space position of this body's center of mass
    pub fn getCenterOfMassPosition(self: *const Body) RVec3 {
        BodyAccess.assertPositionAccess(.read);
        return self.position;
    }

    /// Calculates the transform for this body's center of mass
    pub fn getCenterOfMassTransform(self: *const Body) RMat44 {
        BodyAccess.assertPositionAccess(.read);

        return RMat44.rotationTranslation(self.rotation, self.position);
    }

    /// Calculates the inverse of the transform for this body's center of mass
    pub fn getInverseCenterOfMassTransform(self: *const Body) RMat44 {
        BodyAccess.assertPositionAccess(.read);

        return RMat44.inverseRotationTranslation(self.rotation, self.position);
    }

    /// Get world space bounding box
    pub fn getWorldSpaceBounds(self: *const Body) *const AABox {
        return &self.bounds;
    }

    /// Validate that the cached bounding box of the body matches the actual bounding box of the body (JPH_ENABLE_ASSERTS).
    /// If this check fails then there are a number of possible causes:
    /// 1. Shape is being modified without notifying the system of the change. E.g. if you modify a MutableCompoundShape
    /// without calling BodyInterface::NotifyShapeChanged then there will be a mismatch between the cached bounding box
    /// in the broad phase and the bounding box of the Shape.
    /// 2. You are calling functions postfixed with 'Internal' which are not meant to be called by the application.
    /// 3. If the actual bounds and cached bounds are very close, it could mean that you have a mismatch in floating
    /// point unit state between threads. E.g. one thread has flush to zero (FTZ) or denormals are zero (DAZ) set and
    /// the other thread does not. Or if the rounding mode differs between threads. This can cause small differences
    /// in floating point calculations. If you are using JobSystemThreadPool you can use JobSystemThreadPool::SetThreadInitFunction
    /// to initialize the floating point unit state.
    pub fn validateCachedBounds(self: *const Body) void {
        if (Core.enable_asserts) {
            const actual_body_bounds = self.getShape().getWorldSpaceBoundsRMat44(self.getCenterOfMassTransform(), Vec3.one());
            std.debug.assert(actual_body_bounds.eql(self.bounds)); // Mismatch between cached bounding box and actual bounding box
        }
    }

    /// Validates that a body that is sleeping has zero velocity (JPH_ENABLE_ASSERTS).
    pub fn validateMotion(self: *const Body) void {
        if (Core.enable_asserts) {
            if (!self.isStatic() and !self.isActive()) {
                // If this fails then you have set a velocity on a body without waking it up.
                // Use BodyInterface::SetLinearVelocity or BodyInterface::SetAngularVelocity to set the velocity and wake up the body at the same time.
                std.debug.assert(self.motion_properties.?.getLinearVelocity().isNearZero(.{})); // Sleeping body has non-zero linear velocity!
                std.debug.assert(self.motion_properties.?.getAngularVelocity().isNearZero(.{})); // Sleeping body has non-zero angular velocity!
            }
        }
    }

    /// Access to the motion properties
    pub fn getMotionProperties(self: *const Body) *const MotionProperties {
        std.debug.assert(!self.isStatic());
        return self.motion_properties.?;
    }

    /// Access to the motion properties (non-const overload)
    pub fn getMotionPropertiesMut(self: *Body) *MotionProperties {
        std.debug.assert(!self.isStatic());
        return self.motion_properties.?;
    }

    /// Access to the motion properties (version that does not check if the object is kinematic or dynamic)
    pub fn getMotionPropertiesUnchecked(self: *const Body) ?*const MotionProperties {
        return self.motion_properties;
    }

    /// Access to the motion properties (version that does not check if the object is kinematic or dynamic, non-const overload)
    pub fn getMotionPropertiesUncheckedMut(self: *Body) ?*MotionProperties {
        return self.motion_properties;
    }

    /// Access to the user data, can be used for anything by the application
    pub fn getUserData(self: *const Body) u64 {
        return self.user_data;
    }

    pub fn setUserData(self: *Body, user_data: u64) void {
        self.user_data = user_data;
    }

    /// Get surface normal of a particular sub shape and its world space surface position on this body
    pub fn getWorldSpaceSurfaceNormal(self: *const Body, sub_shape_id: SubShapeID, position: RVec3) Vec3 {
        const inv_com = self.getInverseCenterOfMassTransform();
        return inv_com.multiply3x3Transposed(self.getShape().getSurfaceNormal(sub_shape_id, inv_com.mulRVec3(position).toVec3())).normalized();
    }

    /// Get the transformed shape of this body, which can be used to do collision detection outside of a body lock.
    /// The returned value holds a reference to the shape (`deinit` it).
    pub fn getTransformedShape(self: *const Body) TransformedShape {
        BodyAccess.assertPositionAccess(.read);
        return TransformedShape.init(self.position, self.rotation, self.getShape(), self.id, .{});
    }

    /// Function to convert a body back to a body creation settings object to be able to save/recreate the body later. Can e.g. be used to move a body from one PhysicsSystem to another.
    /// The result holds references to the shape and the group filter (`deinit` it).
    pub fn getBodyCreationSettings(self: *const Body) BodyCreationSettings {
        std.debug.assert(self.isRigidBody());

        var result: BodyCreationSettings = .{};

        const mp = self.motion_properties;
        result.position = self.getPosition();
        result.rotation = self.getRotation();
        result.linear_velocity = if (mp) |m| m.getLinearVelocity() else Vec3.zero();
        result.angular_velocity = if (mp) |m| m.getAngularVelocity() else Vec3.zero();
        result.object_layer = self.getObjectLayer();
        result.user_data = self.user_data;
        result.collision_group = self.getCollisionGroup().clone();
        result.motion_type = self.getMotionType();
        result.allowed_dofs = if (mp) |m| m.getAllowedDOFs() else .all;
        result.allow_dynamic_or_kinematic = mp != null;
        result.is_sensor = self.isSensor();
        result.collide_kinematic_vs_non_dynamic = self.getCollideKinematicVsNonDynamic();
        result.use_manifold_reduction = self.getUseManifoldReduction();
        result.apply_gyroscopic_force = self.getApplyGyroscopicForce();
        result.motion_quality = if (mp) |m| m.getMotionQuality() else .discrete;
        result.enhanced_internal_edge_removal = self.getEnhancedInternalEdgeRemoval();
        result.allow_sleeping = if (mp != null) self.getAllowSleeping() else true;
        result.friction = self.getFriction();
        result.restitution = self.getRestitution();
        result.linear_damping = if (mp) |m| m.getLinearDamping() else 0.0;
        result.angular_damping = if (mp) |m| m.getAngularDamping() else 0.0;
        result.max_linear_velocity = if (mp) |m| m.getMaxLinearVelocity() else 0.0;
        result.max_angular_velocity = if (mp) |m| m.getMaxAngularVelocity() else 0.0;
        result.gravity_factor = if (mp) |m| m.getGravityFactor() else 1.0;
        result.num_velocity_steps_override = if (mp) |m| m.getNumVelocityStepsOverride() else 0;
        result.num_position_steps_override = if (mp) |m| m.getNumPositionStepsOverride() else 0;
        result.override_mass_properties = .mass_and_inertia_provided;

        // Invert inertia and mass
        if (mp) |m| {
            const inv_mass = m.getInverseMassUnchecked();
            const inv_inertia = m.getLocalSpaceInverseInertiaUnchecked();

            // Get mass
            result.mass_properties_override.mass = if (inv_mass != 0.0) 1.0 / inv_mass else math.flt_max;

            // Get inertia
            var inertia: Mat44 = undefined;
            if (inertia.setInversed3x3(inv_inertia)) {
                // Inertia was invertible, we can use it
                result.mass_properties_override.inertia = inertia;
            } else {
                // Prevent division by zero
                const diagonal = Vec3.max(inv_inertia.getDiagonal3(), Vec3.replicate(math.flt_min));
                result.mass_properties_override.inertia = Mat44.scaleVec3(diagonal.reciprocal());
            }
        } else {
            result.mass_properties_override.mass = math.flt_max;
            result.mass_properties_override.inertia = Mat44.scaleVec3(Vec3.replicate(math.flt_max));
        }

        result.setShape(self.getShape());

        return result;
    }

    /// Function to overwrite current body state with body_creation_settings. Can only be done when the body is not in the physics system and cannot add MotionProperties if the body was created without one.
    /// Note that if you change the shape of the body and this body may have been in contact with other bodies before, you should call BodyInterface::InvalidateContactCache to invalidate any previous contacts.
    /// Zolt: `allocator` creates the shape when the settings hold shape settings; when there is no shape (Jolt crashes)
    /// or the mass properties cannot be computed, the body is left unchanged and the error is returned.
    pub fn applyBodyCreationSettings(self: *Body, allocator: Allocator, body_creation_settings: *const BodyCreationSettings, bpl_interface: *const BroadPhaseLayerInterface) CreateBodyError!void {
        std.debug.assert(self.isRigidBody() and !self.isInBroadPhase());

        // Zolt: everything that can fail first (Jolt evaluates GetShape() and GetMassProperties() at their positions below)
        const shape = (try body_creation_settings.getShape(allocator)) orelse return error.InvalidShape;
        const mass_properties: MassProperties = if (body_creation_settings.hasMassProperties()) try body_creation_settings.getMassProperties(allocator) else undefined;

        self.shape.set(shape);
        self.user_data = body_creation_settings.user_data;
        self.setFriction(body_creation_settings.friction);
        self.setRestitution(body_creation_settings.restitution);
        self.motion_type = body_creation_settings.motion_type;
        self.setIsSensor(body_creation_settings.is_sensor);
        self.setCollideKinematicVsNonDynamic(body_creation_settings.collide_kinematic_vs_non_dynamic);
        self.setUseManifoldReduction(body_creation_settings.use_manifold_reduction);
        self.setApplyGyroscopicForce(body_creation_settings.apply_gyroscopic_force);
        self.setEnhancedInternalEdgeRemoval(body_creation_settings.enhanced_internal_edge_removal);
        self.setObjectLayerInternal(body_creation_settings.object_layer, bpl_interface);
        self.collision_group.assign(&body_creation_settings.collision_group);

        if (body_creation_settings.hasMassProperties()) {
            const mp = self.motion_properties orelse @panic("Body was created without MotionProperties, ApplyBodyCreationSettings can't create one!");
            mp.setLinearDamping(body_creation_settings.linear_damping);
            mp.setAngularDamping(body_creation_settings.angular_damping);
            mp.setMaxLinearVelocity(body_creation_settings.max_linear_velocity);
            mp.setMaxAngularVelocity(body_creation_settings.max_angular_velocity);
            mp.setMassProperties(body_creation_settings.allowed_dofs, &mass_properties);
            mp.setLinearVelocity(body_creation_settings.linear_velocity); // Needs to happen after setting the max linear/angular velocity and setting allowed DOFs
            mp.setAngularVelocity(body_creation_settings.angular_velocity);
            mp.setGravityFactor(body_creation_settings.gravity_factor);
            mp.setNumVelocityStepsOverride(body_creation_settings.num_velocity_steps_override);
            mp.setNumPositionStepsOverride(body_creation_settings.num_position_steps_override);
            mp.motion_quality = body_creation_settings.motion_quality;
            mp.allow_sleeping = body_creation_settings.allow_sleeping;
            if (Core.enable_asserts) mp.cached_body_type = self.body_type;
            if (Core.enable_asserts) mp.cached_motion_type = self.motion_type;
        }

        self.setPositionAndRotationInternal(body_creation_settings.position, body_creation_settings.rotation, .{});
    }

    /// Function to convert a soft body back to a body creation settings object to be able to save/recreate the body later. Can e.g. be used to move a body from one PhysicsSystem to another.
    /// The result holds references to the shared settings and the group filter (`deinit` it).
    pub fn getSoftBodyCreationSettings(self: *const Body) SoftBodyCreationSettings {
        std.debug.assert(self.isSoftBody());

        var result: SoftBodyCreationSettings = .{};

        result.position = self.getPosition();
        result.rotation = self.getRotation();
        result.user_data = self.user_data;
        result.object_layer = self.getObjectLayer();
        result.collision_group = self.getCollisionGroup().clone();
        result.friction = self.getFriction();
        result.restitution = self.getRestitution();
        const mp = SoftBodyMotionProperties.fromBaseConst(self.motion_properties.?);
        result.num_iterations = mp.getNumIterations();
        result.linear_damping = mp.base.getLinearDamping();
        result.max_linear_velocity = mp.base.getMaxLinearVelocity();
        result.gravity_factor = mp.base.getGravityFactor();
        result.pressure = mp.getPressure();
        result.update_position = mp.getUpdatePosition();
        result.vertex_radius = mp.getVertexRadius();
        result.allow_sleeping = mp.base.getAllowSleeping();
        result.faces_double_sided = mp.getFacesDoubleSided();
        result.settings = .init(mp.getSettings());

        return result;
    }

    /// Function to overwrite current body state with soft_body_creation_settings. Can only be done when the body is not in the physics system and cannot add MotionProperties if the body was created without one.
    /// Zolt: `error.OutOfMemory` when the motion properties cannot allocate their arrays. They are unchanged then, the
    /// body members that Jolt sets before (user data, friction, ...) have the new values.
    pub fn applySoftBodyCreationSettings(self: *Body, soft_body_creation_settings: *const SoftBodyCreationSettings, bpl_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        std.debug.assert(self.isSoftBody() and !self.isInBroadPhase());

        self.user_data = soft_body_creation_settings.user_data;
        self.setFriction(soft_body_creation_settings.friction);
        self.setRestitution(soft_body_creation_settings.restitution);
        self.motion_type = .dynamic;
        self.setObjectLayerInternal(soft_body_creation_settings.object_layer, bpl_interface);
        self.collision_group.assign(&soft_body_creation_settings.collision_group);

        const mp = SoftBodyMotionProperties.fromBase(self.motion_properties.?);
        mp.base.setLinearDamping(soft_body_creation_settings.linear_damping);
        mp.base.setAngularDamping(0);
        mp.base.setMaxLinearVelocity(soft_body_creation_settings.max_linear_velocity);
        mp.base.setMaxAngularVelocity(math.flt_max);
        mp.base.setLinearVelocity(Vec3.zero());
        mp.base.setAngularVelocity(Vec3.zero());
        mp.base.setGravityFactor(soft_body_creation_settings.gravity_factor);
        mp.base.motion_quality = .discrete;
        mp.base.allow_sleeping = soft_body_creation_settings.allow_sleeping;
        if (Core.enable_asserts) mp.base.cached_body_type = self.body_type;
        if (Core.enable_asserts) mp.base.cached_motion_type = self.motion_type;
        try mp.initialize(soft_body_creation_settings);

        self.setPositionAndRotationInternal(soft_body_creation_settings.position, if (soft_body_creation_settings.make_rotation_identity) Quat.identity() else soft_body_creation_settings.rotation, .{});
    }

    ///@name THESE FUNCTIONS ARE FOR INTERNAL USE ONLY AND SHOULD NOT BE CALLED BY THE APPLICATION
    /// Helper function for BroadPhase::FindCollidingPairs that returns true when two bodies can collide
    /// It assumes that body 1 is dynamic and active and guarantees that it body 1 collides with body 2 that body 2 will not collide with body 1 in order to avoid finding duplicate collision pairs
    pub fn findCollidingPairsCanCollide(body1: *const Body, body2: *const Body) bool {
        // First body should never be a soft body
        std.debug.assert(!body1.isSoftBody());

        // One of these conditions must be true
        // - We always allow detecting collisions between kinematic and non-dynamic bodies
        // - One of the bodies must be dynamic to collide
        // - A kinematic object can collide with a sensor
        if (!body1.getCollideKinematicVsNonDynamic() and
            !body2.getCollideKinematicVsNonDynamic() and
            (!body1.isDynamic() and !body2.isDynamic()) and
            !(body1.isKinematic() and body2.isSensor()) and
            !(body2.isKinematic() and body1.isSensor()))
            return false;

        // Check that body 1 is active
        const body1_index_in_active_bodies = body1.getIndexInActiveBodiesInternal();
        std.debug.assert(!body1.isStatic() and body1_index_in_active_bodies != inactive_index); // This function assumes that Body 1 is active

        // If the pair A, B collides we need to ensure that the pair B, A does not collide or else we will handle the collision twice.
        // If A is the same body as B we don't want to collide (1)
        // If A is dynamic / kinematic and B is static we should collide (2)
        // If A is dynamic / kinematic and B is dynamic / kinematic we should only collide if
        //    - A is active and B is not active (3)
        //    - A is active and B will become active during this simulation step (4)
        //    - A is active and B is active, we require a condition that makes A, B collide and B, A not (5)
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
        comptime std.debug.assert(inactive_index == 0xffffffff); // The algorithm below uses this value
        if (!body2.isSoftBody() and body1_index_in_active_bodies >= body2.getIndexInActiveBodiesInternal())
            return false;
        std.debug.assert(!body1.getID().eql(body2.getID())); // Read the comment above, A and B are the same body which should not be possible!

        // Check collision group filter
        if (!body1.getCollisionGroup().canCollide(body2.getCollisionGroup()))
            return false;

        return true;
    }

    /// Update position using an Euler step (used during position integrate & constraint solving)
    pub fn addPositionStep(self: *Body, linear_velocity_times_delta_time: Vec3) void {
        std.debug.assert(self.isRigidBody());
        BodyAccess.assertPositionAccess(.read_write);
        self.position = self.position.addVec3(self.motion_properties.?.lockTranslation(linear_velocity_times_delta_time));
        if (Core.enable_asserts) std.debug.assert(!self.position.isNaN());
    }

    pub fn subPositionStep(self: *Body, linear_velocity_times_delta_time: Vec3) void {
        std.debug.assert(self.isRigidBody());
        BodyAccess.assertPositionAccess(.read_write);
        self.position = self.position.subVec3(self.motion_properties.?.lockTranslation(linear_velocity_times_delta_time));
        if (Core.enable_asserts) std.debug.assert(!self.position.isNaN());
    }

    /// Update rotation using an Euler step (used during position integrate & constraint solving)
    pub fn addRotationStep(self: *Body, angular_velocity_times_delta_time: Vec3) void {
        std.debug.assert(self.isRigidBody());
        BodyAccess.assertPositionAccess(.read_write);

        // This used to use the equation: d/dt R(t) = 1/2 * w(t) * R(t) so that R(t + dt) = R(t) + 1/2 * w(t) * R(t) * dt
        // See: Appendix B of An Introduction to Physically Based Modeling: Rigid Body Simulation II-Nonpenetration Constraints
        // URL: https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
        // But this is a first order approximation and does not work well for kinematic ragdolls that are driven to a new
        // pose if the poses differ enough. So now we split w(t) * dt into an axis and angle part and create a quaternion with it.
        // Note that the resulting quaternion is normalized since otherwise numerical drift will eventually make the rotation non-normalized.
        const len = angular_velocity_times_delta_time.length();
        if (len > 1.0e-6) {
            self.rotation = Quat.rotation(angular_velocity_times_delta_time.divScalar(len), len).mul(self.rotation).normalized();
            if (Core.enable_asserts) std.debug.assert(!self.rotation.isNaN());
        }
    }

    pub fn subRotationStep(self: *Body, angular_velocity_times_delta_time: Vec3) void {
        std.debug.assert(self.isRigidBody());
        BodyAccess.assertPositionAccess(.read_write);

        // See comment at Body::AddRotationStep
        const len = angular_velocity_times_delta_time.length();
        if (len > 1.0e-6) {
            self.rotation = Quat.rotation(angular_velocity_times_delta_time.divScalar(len), -len).mul(self.rotation).normalized();
            if (Core.enable_asserts) std.debug.assert(!self.rotation.isNaN());
        }
    }

    /// Function to update body's layer (should only be called internally since it also requires updating the broadphase)
    pub fn setObjectLayerInternal(self: *Body, layer: ObjectLayer, bpl_interface: *const BroadPhaseLayerInterface) void {
        self.object_layer = layer;
        self.broad_phase_layer = bpl_interface.getBroadPhaseLayer(layer);
    }

    /// Flag if body is in the broadphase (should only be called by the BroadPhase)
    pub fn setInBroadPhaseInternal(self: *Body, in_broad_phase: bool) void {
        self.setFlag(Flags.is_in_broad_phase, in_broad_phase);
    }

    /// Invalidate the contact cache (should only be called by the BodyManager), will be reset the next simulation step. Returns true if the contact cache was still valid.
    pub fn invalidateContactCacheInternal(self: *Body) bool {
        return (self.flags.fetchOr(Flags.invalidate_contact_cache, .monotonic) & Flags.invalidate_contact_cache) == 0;
    }

    /// Reset the collision cache invalid flag (should only be called by the BodyManager).
    pub fn validateContactCacheInternal(self: *Body) void {
        const old_val = self.flags.fetchAnd(~Flags.invalidate_contact_cache, .monotonic);
        if (Core.enable_asserts) std.debug.assert((old_val & Flags.invalidate_contact_cache) != 0);
    }

    /// Updates world space bounding box (should only be called by the PhysicsSystem)
    pub fn calculateWorldSpaceBoundsInternal(self: *Body) void {
        self.bounds = self.getShape().getWorldSpaceBoundsRMat44(self.getCenterOfMassTransform(), Vec3.one());
    }

    /// Function to update body's position (should only be called by the BodyInterface since it also requires updating the broadphase)
    pub fn setPositionAndRotationInternal(self: *Body, position: RVec3, rotation: Quat, opts: struct { reset_sleep_timer: bool = true }) void {
        BodyAccess.assertPositionAccess(.read_write);

        self.position = position.addVec3(rotation.mulVec3(self.getShape().getCenterOfMass()));
        self.rotation = rotation;

        // Initialize bounding box
        self.calculateWorldSpaceBoundsInternal();

        // Reset sleeping test
        if (opts.reset_sleep_timer and self.motion_properties != null)
            self.resetSleepTimer();
    }

    /// Updates the center of mass and optionally mass properties after shifting the center of mass or changes to the shape (should only be called by the BodyInterface since it also requires updating the broadphase)
    /// @param previous_center_of_mass Center of mass of the shape before the alterations
    /// @param update_mass_properties When true, the mass and inertia tensor is recalculated
    pub fn updateCenterOfMassInternal(self: *Body, previous_center_of_mass: Vec3, update_mass_properties: bool) void {
        // Update center of mass position so the world position for this body stays the same
        self.position = self.position.addVec3(self.rotation.mulVec3(self.getShape().getCenterOfMass().sub(previous_center_of_mass)));

        // Recalculate mass and inertia if requested
        if (update_mass_properties) {
            if (self.motion_properties) |mp| {
                const mass_properties = self.getShape().getMassProperties();
                mp.setMassProperties(mp.getAllowedDOFs(), &mass_properties);
            }
        }
    }

    /// Function to update a body's shape (should only be called by the BodyInterface since it also requires updating the broadphase)
    /// @param shape The new shape for this body
    /// @param update_mass_properties When true, the mass and inertia tensor is recalculated
    pub fn setShapeInternal(self: *Body, shape: *const Shape, update_mass_properties: bool) void {
        std.debug.assert(self.isRigidBody()); // Only valid for rigid bodies
        BodyAccess.assertPositionAccess(.read_write);

        // Get the old center of mass
        const old_com = self.getShape().getCenterOfMass();

        // Update the shape
        self.shape.set(shape);

        // Update center of mass
        self.updateCenterOfMassInternal(old_com, update_mass_properties);

        // Recalculate bounding box
        self.calculateWorldSpaceBoundsInternal();
    }

    // TSAN detects a race between BodyManager::AddBodyToActiveBodies coming from PhysicsSystem::ProcessBodyPair and Body::GetIndexInActiveBodiesInternal coming from PhysicsSystem::ProcessBodyPair.
    // When PhysicsSystem::ProcessBodyPair activates a body, it updates mIndexInActiveBodies and then updates BodyManager::mNumActiveBodies with release semantics. PhysicsSystem::ProcessBodyPair will
    // then finish its loop of active bodies and at the end of the loop it will read BodyManager::mNumActiveBodies with acquire semantics to see if any bodies were activated during the loop.
    // This means that changes to mIndexInActiveBodies must be visible to the thread, so TSANs report must be a false positive. We suppress the warning here.
    // (Zolt: a monotonic atomic load, SimulationArchitecture section 0 rule 3)
    /// Access to the index in the BodyManager::mActiveBodies list
    pub fn getIndexInActiveBodiesInternal(self: *const Body) u32 {
        return if (self.motion_properties) |mp| mp.index_in_active_bodies.load(.monotonic) else inactive_index;
    }

    /// Update eligibility for sleeping
    pub fn updateSleepStateInternal(self: *Body, delta_time: f32, max_movement: f32, time_before_sleep: f32) CanSleep {
        const mp = self.motion_properties.?;

        // Check override & sensors will never go to sleep (they would stop detecting collisions with sleeping bodies)
        if (!mp.allow_sleeping or self.isSensor())
            return .cannot_sleep;

        // Get the points to test
        const points = self.getSleepTestPoints();

        // Get base offset for spheres (JPH_DOUBLE_PRECISION)
        const offset: if (Core.double_precision) DVec3 else void = if (Core.double_precision) mp.getSleepTestOffset() else {};

        for (0..3) |i| {
            const sphere = &mp.sleep_test_spheres[i];

            // Make point relative to base offset
            const p: Vec3 = if (Core.double_precision) points[i].sub(offset).toVec3() else points[i];

            // Encapsulate the point in a sphere
            sphere.encapsulatePoint(p);

            // Test if it exceeded the max movement
            if (sphere.getRadius() > max_movement) {
                // Body is not sleeping, reset test
                mp.resetSleepTestSpheres(&points);
                return .cannot_sleep;
            }
        }

        return mp.accumulateSleepTime(delta_time, time_before_sleep);
    }

    /// Saving state for replay
    pub fn saveState(self: *const Body, stream: *StateRecorder) void {
        // Only write properties that can change at runtime
        stream.write(self.position);
        stream.write(self.rotation);

        if (self.motion_properties) |mp| {
            if (self.isSoftBody())
                SoftBodyMotionProperties.fromBaseConst(mp).saveState(stream)
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
                SoftBodyMotionProperties.fromBase(mp).restoreState(stream)
            else
                mp.restoreState(stream);

            if (Core.enable_asserts) mp.cached_motion_type = self.motion_type;
        }

        // Initialize bounding box
        self.calculateWorldSpaceBoundsInternal();
    }

    /// Determine points to test for checking if body is sleeping: COM, COM + largest bounding box axis, COM + second largest bounding box axis
    fn getSleepTestPoints(self: *const Body) [3]RVec3 {
        BodyAccess.assertPositionAccess(.read);

        var out_points: [3]RVec3 = undefined;

        // Center of mass is the first position
        out_points[0] = self.position;

        // The second and third position are on the largest axis of the bounding box
        const extent = self.getShape().getLocalBounds().getExtent();
        const lowest_component = extent.getLowestComponentIndex();
        const rotation = Mat44.rotationQuat(self.rotation);
        switch (lowest_component) {
            0 => {
                out_points[1] = self.position.addVec3(rotation.getColumn3(1).mulScalar(extent.getY()));
                out_points[2] = self.position.addVec3(rotation.getColumn3(2).mulScalar(extent.getZ()));
            },
            1 => {
                out_points[1] = self.position.addVec3(rotation.getColumn3(0).mulScalar(extent.getX()));
                out_points[2] = self.position.addVec3(rotation.getColumn3(2).mulScalar(extent.getZ()));
            },
            2 => {
                out_points[1] = self.position.addVec3(rotation.getColumn3(0).mulScalar(extent.getX()));
                out_points[2] = self.position.addVec3(rotation.getColumn3(1).mulScalar(extent.getY()));
            },
            else => unreachable, // JPH_ASSERT(false): GetLowestComponentIndex returns 0, 1 or 2
        }

        return out_points;
    }
};

test "Body layout, fixed to world and the static shape" {
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;
    if (@sizeOf(usize) == 8 and Core.object_layer_bits == 16)
        try expectEqual(@as(usize, if (Core.double_precision) 160 else 128), @sizeOf(Body));

    const fixed = Body.fixedToWorld();
    try expect(fixed.isStatic());
    try expect(fixed.isRigidBody());
    try expect(!fixed.isActive());
    try expect(!fixed.isSensor());
    try expect(fixed.getID().isInvalid());
    try expect(fixed.getShape().getSubType() == .empty);
    try expectEqual(Body.inactive_index, fixed.getIndexInActiveBodiesInternal());
    try expect(fixed.getLinearVelocity().eql(Vec3.zero()));
    try expect(fixed.getPosition().eql(RVec3.zero()));
    try expect(fixed.getWorldSpaceBounds().eql(AABox.empty));
    try expectEqual(object_layer_invalid, fixed.getObjectLayer());
    try expectEqual(@as(u8, 0), fixed.getBroadPhaseLayer().getValue());

    // The dummy shape is static (read-only memory, never reference counted): references to it are legal, also in
    // ReleaseFast, where a write to read-only memory would fault
    var shape = RefConst(Shape).init(fixed.getShape());
    var copy = shape.clone();
    copy.deinit();
    shape.deinit();
    try expect(fixed.getShape().is_static);
    var transformed = fixed.getTransformedShape();
    transformed.deinit();
}
