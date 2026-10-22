//! Port of: Jolt/Physics/Body/BodyInterface.h, Jolt/Physics/Body/BodyInterface.cpp
//! Status: complete
//!
//! SimulationArchitecture D6:
//! - Initialized by PhysicsSystem.init (`BodyInterface.init(lock_interface, body_manager, broad_phase)`), it holds
//!   pointers into the PhysicsSystem. Jolt's non-const member functions take `*BodyInterface`, the const ones
//!   `*const BodyInterface` (PhysicsSystem.getBodyInterfaceMut / getBodyInterface).
//! - Overloads get names: `AssignBodyID(Body *, const BodyID &)` is `assignBodyIDWithID`, `AddForce(id, force,
//!   point)` / `AddImpulse(id, impulse, point)` are `addForceAtPosition` / `addImpulseAtPosition` (like Body's).
//!   Jolt's defaulted `EActivation inActivationMode = EActivation::Activate` is an option
//!   (`.{ .activation = .dont_activate }`). Out parameters (`GetPositionAndRotation`,
//!   `GetLinearAndAngularVelocity`) return structs.
//! - Errors (Zolt): body creation returns `CreateBodyError` (allocation, a settings object without shape) and keeps
//!   Jolt's null / invalid ID for "out of bodies". The soft body functions follow the same rules
//!   (RemainingFeaturesArchitecture.md R16), only allocation can fail there. `addBody` and `addBodiesPrepare` return
//!   `Allocator.Error` (the broad phase allocates in AddBodiesPrepare); on error the body was not added, and
//!   `createAndAddBody` / `createAndAddSoftBody` destroy the body they created. The functions that flag the contact
//!   cache invalid (`setShape`, `notifyShapeChanged`, `setUseManifoldReduction`, `invalidateContactCache`) return the
//!   `Allocator.Error` of BodyManager.invalidateContactCacheForBody. They flag the contact cache before they change the body (Jolt: after,
//!   the result is the same), so a failed call changed nothing and can be repeated. `activateBodiesInAABox` collects
//!   the bodies in an AllHitCollisionCollector (Allocator.Error, nothing was activated). `createConstraint` allocates
//!   the constraint with the body manager's allocator (Jolt: `new`), its reference count is 0 like Jolt's.
//!   `setObjectLayer` returns the `Allocator.Error` of BroadPhase.notifyBodiesLayerChanged (the QuadTree adds the body
//!   to the tree of its new layer); it restores the layer of the body then, so a failed call changed nothing.
//! - `getShape` and `getTransformedShape` return values that hold a shape reference: the caller `deinit`s them.

const std = @import("std");
const Allocator = std.mem.Allocator;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const ObjectLayerFile = @import("../Collision/ObjectLayer.zig");
const ObjectLayer = ObjectLayerFile.ObjectLayer;
const ObjectLayerFilter = ObjectLayerFile.ObjectLayerFilter;
const BroadPhaseLayerFilter = @import("../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerFilter;
const BroadPhase = @import("../Collision/BroadPhase/BroadPhase.zig").BroadPhase;
const CollideShapeBodyCollector = @import("../Collision/BroadPhase/BroadPhaseQuery.zig").CollideShapeBodyCollector;
const AllHitCollisionCollector = @import("../Collision/CollisionCollectorImpl.zig").AllHitCollisionCollector;
const TwoBodyConstraintFile = @import("../Constraints/TwoBodyConstraint.zig");
const TwoBodyConstraint = TwoBodyConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = TwoBodyConstraintFile.TwoBodyConstraintSettings;
const Activation = @import("../EActivation.zig").Activation;
const Body = @import("Body.zig").Body;
const BodyID = @import("BodyID.zig").BodyID;
const BodyType = @import("BodyType.zig").BodyType;
const MotionType = @import("MotionType.zig").MotionType;
const MotionQuality = @import("MotionQuality.zig").MotionQuality;
const BodyCreationSettingsFile = @import("BodyCreationSettings.zig");
const BodyCreationSettings = BodyCreationSettingsFile.BodyCreationSettings;
const CreateBodyError = BodyCreationSettingsFile.CreateBodyError;
const SoftBodyCreationSettings = @import("../SoftBody/SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const BodyManager = @import("BodyManager.zig").BodyManager;
const BodyLockInterface = @import("BodyLockInterface.zig").BodyLockInterface;
const BodyLockFile = @import("BodyLock.zig");
const BodyLockRead = BodyLockFile.BodyLockRead;
const BodyLockWrite = BodyLockFile.BodyLockWrite;
const BodyLockMultiWrite = @import("BodyLockMulti.zig").BodyLockMultiWrite;

/// Class that provides operations on bodies using a body ID. Note that if you need to do multiple operations on a single body, it is more efficient to lock the body once and combine the operations.
/// All quantities are in world space unless otherwise specified.
pub const BodyInterface = struct {
    /// Handle used during adding bodies to the broadphase
    pub const AddState = BroadPhase.AddState;

    body_lock_interface: *const BodyLockInterface = undefined,
    body_manager: *BodyManager = undefined,
    broad_phase: *BroadPhase = undefined,

    /// Not initialized (Jolt's default constructor)
    pub const empty: BodyInterface = .{};

    /// Initialize the interface (should only be called by PhysicsSystem)
    pub fn init(body_lock_interface: *const BodyLockInterface, body_manager: *BodyManager, broad_phase: *BroadPhase) BodyInterface {
        return .{ .body_lock_interface = body_lock_interface, .body_manager = body_manager, .broad_phase = broad_phase };
    }

    /// Activate a body or reset its sleep timer (private in Jolt)
    fn activateBodyInternal(self: *const BodyInterface, body: *Body) void {
        // Activate body or reset its sleep timer.
        // Note that BodyManager::ActivateBodies also resets the sleep timer internally, but we avoid a mutex lock if the body is already active by calling ResetSleepTimer directly.
        if (!body.isActive()) {
            const id = body.getID();
            self.body_manager.activateBodies((&id)[0..1]);
        } else body.resetSleepTimer();
    }

    /// Create a rigid body
    /// @return Created body or null when out of bodies
    pub fn createBody(self: *BodyInterface, settings: *const BodyCreationSettings) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return null;
        }
        return body;
    }

    /// Create a soft body
    /// @return Created body or null when out of bodies
    pub fn createSoftBody(self: *BodyInterface, settings: *const SoftBodyCreationSettings) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateSoftBody(settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return null;
        }
        return body;
    }

    /// Create a rigid body with specified ID. This function can be used if a simulation is to run in sync between clients or if a simulation needs to be restored exactly.
    /// The ID created on the server can be replicated to the client and used to create a deterministic simulation.
    /// @return Created body or null when the body ID is invalid or a body of the same ID already exists.
    pub fn createBodyWithID(self: *BodyInterface, body_id: BodyID, settings: *const BodyCreationSettings) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBodyWithCustomID(body, body_id)) {
            self.body_manager.freeBody(body);
            return null;
        }
        return body;
    }

    /// Create a soft body with specified ID. See comments at CreateBodyWithID.
    pub fn createSoftBodyWithID(self: *BodyInterface, body_id: BodyID, settings: *const SoftBodyCreationSettings) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateSoftBody(settings);
        if (!self.body_manager.addBodyWithCustomID(body, body_id)) {
            self.body_manager.freeBody(body);
            return null;
        }
        return body;
    }

    /// Advanced use only. Creates a rigid body without specifying an ID. This body cannot be added to the physics system until it has been assigned a body ID.
    /// This can be used to decouple allocation from registering the body. A call to CreateBodyWithoutID followed by AssignBodyID is equivalent to calling CreateBodyWithID.
    /// @return Created body
    pub fn createBodyWithoutID(self: *const BodyInterface, settings: *const BodyCreationSettings) CreateBodyError!*Body {
        return self.body_manager.allocateBody(settings);
    }

    /// Advanced use only. Creates a body without specifying an ID. See comments at CreateBodyWithoutID.
    pub fn createSoftBodyWithoutID(self: *const BodyInterface, settings: *const SoftBodyCreationSettings) CreateBodyError!*Body {
        return self.body_manager.allocateSoftBody(settings);
    }

    /// Advanced use only. Destroy a body previously created with CreateBodyWithoutID that hasn't gotten an ID yet through the AssignBodyID function,
    /// or a body that has had its body ID unassigned through UnassignBodyIDs. Bodies that have an ID should be destroyed through DestroyBody.
    pub fn destroyBodyWithoutID(self: *const BodyInterface, body: *Body) void {
        self.body_manager.freeBody(body);
    }

    /// Advanced use only. Assigns the next available body ID to a body that was created using CreateBodyWithoutID. After this call, the body can be added to the physics system.
    /// @return false if the body already has an ID or out of body ids.
    pub fn assignBodyID(self: *BodyInterface, body: *Body) bool {
        return self.body_manager.addBody(body);
    }

    /// Advanced use only. Assigns a body ID to a body that was created using CreateBodyWithoutID. After this call, the body can be added to the physics system.
    /// @return false if the body already has an ID or if the ID is not valid.
    pub fn assignBodyIDWithID(self: *BodyInterface, body: *Body, body_id: BodyID) bool {
        return self.body_manager.addBodyWithCustomID(body, body_id);
    }

    /// Advanced use only. See UnassignBodyIDs. Unassigns the ID of a single body.
    pub fn unassignBodyID(self: *BodyInterface, body_id: BodyID) *Body {
        var body: [1]*Body = undefined;
        self.body_manager.removeBodies((&body_id)[0..1], &body);
        return body[0];
    }

    /// Advanced use only. Removes a number of body IDs from their bodies and returns the body pointers. Before calling this, the body should have been removed from the physics system.
    /// The body can be destroyed through DestroyBodyWithoutID. This can be used to decouple deallocation. A call to UnassignBodyIDs followed by calls to DestroyBodyWithoutID is equivalent to calling DestroyBodies.
    /// @param body_ids A list of body IDs
    /// @param out_bodies If not null on input, this will contain a list of body pointers corresponding to inBodyIDs that can be destroyed afterwards (caller assumes ownership over these).
    pub fn unassignBodyIDs(self: *BodyInterface, body_ids: []const BodyID, out_bodies: ?[]*Body) void {
        self.body_manager.removeBodies(body_ids, out_bodies);
    }

    /// Destroy a body.
    /// Make sure that you remove the body from the physics system using BodyInterface::RemoveBody before calling this function.
    pub fn destroyBody(self: *BodyInterface, body_id: BodyID) void {
        self.body_manager.destroyBodies((&body_id)[0..1]);
    }

    /// Destroy multiple bodies
    /// Make sure that you remove the bodies from the physics system using BodyInterface::RemoveBody before calling this function.
    pub fn destroyBodies(self: *BodyInterface, body_ids: []const BodyID) void {
        self.body_manager.destroyBodies(body_ids);
    }

    /// Add body to the physics system.
    /// Note that if you need to add multiple bodies, use the AddBodiesPrepare/AddBodiesFinalize function.
    /// Adding many bodies, one at a time, results in a really inefficient broadphase until PhysicsSystem::OptimizeBroadPhase is called or when PhysicsSystem::Update rebuilds the tree!
    /// After adding, to get a body by ID use the BodyLockRead or BodyLockWrite interface!
    /// (Zolt: Allocator.Error from the broad phase, the body was not added then)
    pub fn addBody(self: *BodyInterface, body_id: BodyID, activation_mode: Activation) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Add to broadphase
            var id = body_id;
            const add_state = try self.broad_phase.addBodiesPrepare((&id)[0..1]);
            self.broad_phase.addBodiesFinalize((&id)[0..1], add_state);

            // Optionally activate body
            if (activation_mode == .activate and !body.isStatic())
                self.body_manager.activateBodies((&body_id)[0..1]);
        }
    }

    /// Remove body from the physics system.
    pub fn removeBody(self: *BodyInterface, body_id: BodyID) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeededAndIsInBroadPhase()) {
            const body = lock.getBody();

            // Deactivate body
            if (body.isActive())
                self.body_manager.deactivateBodies((&body_id)[0..1]);

            // Remove from broadphase
            var id = body_id;
            self.broad_phase.removeBodies((&id)[0..1]);
        }
    }

    /// Check if a body has been added to the physics system.
    pub fn isAdded(self: *const BodyInterface, body_id: BodyID) bool {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        return lock.succeededAndIsInBroadPhase();
    }

    /// Combines CreateBody and AddBody
    /// @return Created body ID or an invalid ID when out of bodies
    /// (Zolt: on an error nothing is left behind, a created body is destroyed again)
    pub fn createAndAddBody(self: *BodyInterface, settings: *const BodyCreationSettings, activation_mode: Activation) CreateBodyError!BodyID {
        const b = (try self.createBody(settings)) orelse return .invalid; // Out of bodies
        const id = b.getID();
        errdefer self.destroyBody(id);
        try self.addBody(id, activation_mode);
        return id;
    }

    /// Combines CreateSoftBody and AddBody
    /// @return Created body ID or an invalid ID when out of bodies
    /// (Zolt: on an error nothing is left behind, a created body is destroyed again)
    pub fn createAndAddSoftBody(self: *BodyInterface, settings: *const SoftBodyCreationSettings, activation_mode: Activation) CreateBodyError!BodyID {
        const b = (try self.createSoftBody(settings)) orelse return .invalid; // Out of bodies
        const id = b.getID();
        errdefer self.destroyBody(id);
        try self.addBody(id, activation_mode);
        return id;
    }

    /// Prepare adding inNumber bodies at ioBodies to the physics system, returns a handle that should be used in AddBodiesFinalize/Abort.
    /// This can be done on a background thread without influencing the physics system.
    /// ioBodies may be shuffled around by this function and should be kept that way until AddBodiesFinalize/Abort is called.
    pub fn addBodiesPrepare(self: *BodyInterface, bodies: []BodyID) Allocator.Error!AddState {
        return self.broad_phase.addBodiesPrepare(bodies);
    }

    /// Finalize adding bodies to the physics system, supply the return value of AddBodiesPrepare in inAddState.
    /// Please use the same ioBodies and inNumber as provided to AddBodiesPrepare.
    pub fn addBodiesFinalize(self: *BodyInterface, bodies: []BodyID, add_state: AddState, activation_mode: Activation) void {
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, bodies);
        defer lock.deinit();

        // Add to broadphase
        self.broad_phase.addBodiesFinalize(bodies, add_state);

        // Optionally activate bodies
        if (activation_mode == .activate)
            self.body_manager.activateBodies(bodies);
    }

    /// Abort adding bodies to the physics system, supply the return value of AddBodiesPrepare in inAddState.
    /// This can be done on a background thread without influencing the physics system.
    /// Please use the same ioBodies and inNumber as provided to AddBodiesPrepare.
    pub fn addBodiesAbort(self: *BodyInterface, bodies: []BodyID, add_state: AddState) void {
        self.broad_phase.addBodiesAbort(bodies, add_state);
    }

    /// Remove inNumber bodies in ioBodies from the physics system.
    /// ioBodies may be shuffled around by this function.
    pub fn removeBodies(self: *BodyInterface, bodies: []BodyID) void {
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, bodies);
        defer lock.deinit();

        // Deactivate bodies
        self.body_manager.deactivateBodies(bodies);

        // Remove from broadphase
        self.broad_phase.removeBodies(bodies);
    }

    ///@name Activate / deactivate a body
    pub fn activateBody(self: *BodyInterface, body_id: BodyID) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            self.activateBodyInternal(body);
        }
    }

    pub fn activateBodies(self: *BodyInterface, body_ids: []const BodyID) void {
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, body_ids);
        defer lock.deinit();

        self.body_manager.activateBodies(body_ids);
    }

    /// (Zolt: Allocator.Error from the collector, no body was activated then)
    pub fn activateBodiesInAABox(self: *BodyInterface, box: *const AABox, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) Allocator.Error!void {
        var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(self.body_manager.allocator);
        defer collector.deinit();
        self.broad_phase.asQuery().collideAABox(box, &collector.base, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
        try collector.checkError();
        self.activateBodies(collector.hits.items);
    }

    pub fn deactivateBody(self: *BodyInterface, body_id: BodyID) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            if (body.isActive())
                self.body_manager.deactivateBodies((&body_id)[0..1]);
        }
    }

    pub fn deactivateBodies(self: *BodyInterface, body_ids: []const BodyID) void {
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, body_ids);
        defer lock.deinit();

        self.body_manager.deactivateBodies(body_ids);
    }

    /// Checks if a body is currently actively simulating
    pub fn isActive(self: *const BodyInterface, body_id: BodyID) bool {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        return lock.succeeded() and lock.getBody().isActive();
    }

    /// @see Body::ResetSleepTimer
    pub fn resetSleepTimer(self: *BodyInterface, body_id: BodyID) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            lock.getBody().resetSleepTimer();
    }

    /// Create a two body constraint
    /// (Zolt: allocated with the body manager's allocator, reference count 0 like Jolt's `new`)
    pub fn createConstraint(self: *BodyInterface, settings: *const TwoBodyConstraintSettings, body_id1: BodyID, body_id2: BodyID) Allocator.Error!*TwoBodyConstraint {
        const constraint_bodies = [_]BodyID{ body_id1, body_id2 };
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, &constraint_bodies);
        defer lock.deinit();

        const body1 = lock.getBody(0);
        const body2 = lock.getBody(1);

        std.debug.assert(body1 != null or body2 != null);
        std.debug.assert(body1 != body2);

        return settings.createConstraint(self.body_manager.allocator, body1 orelse Body.fixedToWorld(), body2 orelse Body.fixedToWorld());
    }

    /// Activate non-static bodies attached to a constraint
    pub fn activateConstraint(self: *BodyInterface, constraint: *const TwoBodyConstraint) void {
        const bodies = [_]BodyID{ constraint.getBody1().getID(), constraint.getBody2().getID() };
        self.activateBodies(&bodies);
    }

    ///@name Access to the shape of a body
    /// Get the current shape (the caller owns the returned reference: `deinit` it)
    pub fn getShape(self: *const BodyInterface, body_id: BodyID) RefConst(Shape) {
        var shape: RefConst(Shape) = .empty;
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            shape.set(lock.getBody().getShape());
        return shape;
    }

    /// Set a new shape on the body
    /// @param body_id Body ID of body that had its shape changed
    /// @param shape The new shape
    /// @param update_mass_properties When true, the mass and inertia tensor is recalculated
    /// @param activation_mode Whether or not to activate the body
    pub fn setShape(self: *const BodyInterface, body_id: BodyID, shape: *const Shape, update_mass_properties: bool, activation_mode: Activation) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Check if shape actually changed
            if (body.getShape() != shape) {
                // Flag collision cache invalid for this body (Zolt: before the shape is updated, Jolt does it after, so
                // that a failed call changes nothing and can be repeated; the flag does not depend on the shape)
                try self.body_manager.invalidateContactCacheForBody(body);

                // Update the shape
                body.setShapeInternal(shape, update_mass_properties);

                // Notify broadphase of change
                if (body.isInBroadPhase()) {
                    var id = body.getID();
                    self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});

                    // Optionally activate body
                    if (activation_mode == .activate and !body.isStatic())
                        self.activateBodyInternal(body);
                }
            }
        }
    }

    /// Notify all systems to indicate that a shape has changed (usable for MutableCompoundShapes)
    /// @param body_id Body ID of body that had its shape changed
    /// @param previous_center_of_mass Center of mass of the shape before the alterations
    /// @param update_mass_properties When true, the mass and inertia tensor is recalculated
    /// @param activation_mode Whether or not to activate the body
    pub fn notifyShapeChanged(self: *const BodyInterface, body_id: BodyID, previous_center_of_mass: Vec3, update_mass_properties: bool, activation_mode: Activation) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Flag collision cache invalid for this body (Zolt: before the center of mass is updated, Jolt does it
            // after, so that a failed call changes nothing and can be repeated; updating the center of mass moves the
            // body relative to previous_center_of_mass, a second update after a failure would move it again)
            try self.body_manager.invalidateContactCacheForBody(body);

            // Update center of mass, mass and inertia
            body.updateCenterOfMassInternal(previous_center_of_mass, update_mass_properties);

            // Recalculate bounding box
            body.calculateWorldSpaceBoundsInternal();

            // Notify broadphase of change
            if (body.isInBroadPhase()) {
                var id = body.getID();
                self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});

                // Optionally activate body
                if (activation_mode == .activate and !body.isStatic())
                    self.activateBodyInternal(body);
            }
        }
    }

    ///@name Object layer of a body
    /// The collision layer this body belongs to (determines if two objects can collide)
    /// (Zolt: Allocator.Error from the broad phase, which moves the body to the tree of its new layer; nothing changed then)
    pub fn setObjectLayer(self: *BodyInterface, body_id: BodyID, layer: ObjectLayer) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Check if layer actually changed, updating the broadphase is rather expensive
            if (body.getObjectLayer() != layer) {
                // Update the layer on the body
                const old_layer = body.getObjectLayer();
                body.setObjectLayerInternal(layer, self.body_manager.getBroadPhaseLayerInterface());

                // Notify broadphase of change
                if (body.isInBroadPhase()) {
                    var id = body.getID();
                    self.broad_phase.notifyBodiesLayerChanged((&id)[0..1]) catch |err| {
                        // Zolt: the broad phase is unchanged, restore the layer of the body
                        body.setObjectLayerInternal(old_layer, self.body_manager.getBroadPhaseLayerInterface());
                        return err;
                    };
                }
            }
        }
    }

    /// @see Body::GetObjectLayer
    pub fn getObjectLayer(self: *const BodyInterface, body_id: BodyID) ObjectLayer {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getObjectLayer()
        else
            return ObjectLayerFile.object_layer_invalid;
    }

    ///@name Position and rotation of a body
    pub fn setPositionAndRotation(self: *BodyInterface, body_id: BodyID, position: RVec3, rotation: Quat, activation_mode: Activation) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Update the position
            body.setPositionAndRotationInternal(position, rotation, .{});

            // Notify broadphase of change
            if (body.isInBroadPhase()) {
                var id = body.getID();
                self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});

                // Optionally activate body
                if (activation_mode == .activate and !body.isStatic())
                    self.activateBodyInternal(body);
            }
        }
    }

    /// Will only update the position/rotation and activate the body when the difference is larger than a very small number. This avoids updating the broadphase/waking up a body when the resulting position/orientation doesn't really change.
    pub fn setPositionAndRotationWhenChanged(self: *BodyInterface, body_id: BodyID, position: RVec3, rotation: Quat, activation_mode: Activation) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Check if there is enough change
            if (!body.getPosition().isClose(position, .{}) or !body.getRotation().isClose(rotation, .{})) {
                // Update the position
                body.setPositionAndRotationInternal(position, rotation, .{});

                // Notify broadphase of change
                if (body.isInBroadPhase()) {
                    var id = body.getID();
                    self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});

                    // Optionally activate body
                    if (activation_mode == .activate and !body.isStatic())
                        self.activateBodyInternal(body);
                }
            }
        }
    }

    /// Position and rotation of a body (Jolt's out parameters)
    pub const PositionAndRotation = struct {
        position: RVec3,
        rotation: Quat,
    };

    pub fn getPositionAndRotation(self: *const BodyInterface, body_id: BodyID) PositionAndRotation {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            return .{ .position = body.getPosition(), .rotation = body.getRotation() };
        } else {
            return .{ .position = RVec3.zero(), .rotation = Quat.identity() };
        }
    }

    pub fn setPosition(self: *BodyInterface, body_id: BodyID, position: RVec3, activation_mode: Activation) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Update the position
            body.setPositionAndRotationInternal(position, body.getRotation(), .{});

            // Notify broadphase of change
            if (body.isInBroadPhase()) {
                var id = body.getID();
                self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});

                // Optionally activate body
                if (activation_mode == .activate and !body.isStatic())
                    self.activateBodyInternal(body);
            }
        }
    }

    pub fn getPosition(self: *const BodyInterface, body_id: BodyID) RVec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getPosition()
        else
            return RVec3.zero();
    }

    /// @see Body::GetCenterOfMassPosition
    pub fn getCenterOfMassPosition(self: *const BodyInterface, body_id: BodyID) RVec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getCenterOfMassPosition()
        else
            return RVec3.zero();
    }

    pub fn setRotation(self: *BodyInterface, body_id: BodyID, rotation: Quat, activation_mode: Activation) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Update the position
            body.setPositionAndRotationInternal(body.getPosition(), rotation, .{});

            // Notify broadphase of change
            if (body.isInBroadPhase()) {
                var id = body.getID();
                self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});

                // Optionally activate body
                if (activation_mode == .activate and !body.isStatic())
                    self.activateBodyInternal(body);
            }
        }
    }

    pub fn getRotation(self: *const BodyInterface, body_id: BodyID) Quat {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getRotation()
        else
            return Quat.identity();
    }

    /// @see Body::GetWorldTransform
    pub fn getWorldTransform(self: *const BodyInterface, body_id: BodyID) RMat44 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getWorldTransform()
        else
            return RMat44.identity();
    }

    /// @see Body::GetCenterOfMassTransform
    pub fn getCenterOfMassTransform(self: *const BodyInterface, body_id: BodyID) RMat44 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getCenterOfMassTransform()
        else
            return RMat44.identity();
    }

    /// Set velocity of body such that it will be positioned at inTargetPosition/Rotation in inDeltaTime seconds (will activate body if needed)
    pub fn moveKinematic(self: *BodyInterface, body_id: BodyID, target_position: RVec3, target_rotation: Quat, delta_time: f32) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            body.moveKinematic(target_position, target_rotation, delta_time);

            if (!body.isActive() and (!body.getLinearVelocity().isNearZero(.{}) or !body.getAngularVelocity().isNearZero(.{})) and body.isInBroadPhase())
                self.body_manager.activateBodies((&body_id)[0..1]);
        }
    }

    /// Linear or angular velocity (functions will activate body if needed).
    /// Note that the linear velocity is the velocity of the center of mass, which may not coincide with the position of your object, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$
    pub fn setLinearAndAngularVelocity(self: *BodyInterface, body_id: BodyID, linear_velocity: Vec3, angular_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setLinearVelocityClamped(linear_velocity);
                body.setAngularVelocityClamped(angular_velocity);

                if (!body.isActive() and (!linear_velocity.isNearZero(.{}) or !angular_velocity.isNearZero(.{})) and body.isInBroadPhase())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// Linear and angular velocity of a body (Jolt's out parameters)
    pub const LinearAndAngularVelocity = struct {
        linear_velocity: Vec3,
        angular_velocity: Vec3,
    };

    pub fn getLinearAndAngularVelocity(self: *const BodyInterface, body_id: BodyID) LinearAndAngularVelocity {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic())
                return .{ .linear_velocity = body.getLinearVelocity(), .angular_velocity = body.getAngularVelocity() };
        }

        return .{ .linear_velocity = Vec3.zero(), .angular_velocity = Vec3.zero() };
    }

    /// @see Body::SetLinearVelocity
    pub fn setLinearVelocity(self: *BodyInterface, body_id: BodyID, linear_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setLinearVelocityClamped(linear_velocity);

                if (!body.isActive() and !linear_velocity.isNearZero(.{}) and body.isInBroadPhase())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// @see Body::GetLinearVelocity
    pub fn getLinearVelocity(self: *const BodyInterface, body_id: BodyID) Vec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic())
                return body.getLinearVelocity();
        }

        return Vec3.zero();
    }

    /// Add velocity to current velocity
    pub fn addLinearVelocity(self: *BodyInterface, body_id: BodyID, linear_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setLinearVelocityClamped(body.getLinearVelocity().add(linear_velocity));

                if (!body.isActive() and !body.getLinearVelocity().isNearZero(.{}) and body.isInBroadPhase())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// Add linear and angular to current velocities
    pub fn addLinearAndAngularVelocity(self: *BodyInterface, body_id: BodyID, linear_velocity: Vec3, angular_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setLinearVelocityClamped(body.getLinearVelocity().add(linear_velocity));
                body.setAngularVelocityClamped(body.getAngularVelocity().add(angular_velocity));

                if (!body.isActive() and (!body.getLinearVelocity().isNearZero(.{}) or !body.getAngularVelocity().isNearZero(.{})) and body.isInBroadPhase())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// @see Body::SetAngularVelocity
    pub fn setAngularVelocity(self: *BodyInterface, body_id: BodyID, angular_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setAngularVelocityClamped(angular_velocity);

                if (!body.isActive() and !angular_velocity.isNearZero(.{}) and body.isInBroadPhase())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// @see Body::GetAngularVelocity
    pub fn getAngularVelocity(self: *const BodyInterface, body_id: BodyID) Vec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic())
                return body.getAngularVelocity();
        }

        return Vec3.zero();
    }

    /// Velocity of point inPoint (in world space, e.g. on the surface of the body) of the body. @see Body::GetPointVelocity
    pub fn getPointVelocity(self: *const BodyInterface, body_id: BodyID, point: RVec3) Vec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic())
                return body.getPointVelocity(point);
        }

        return Vec3.zero();
    }

    /// Set the complete motion state of a body.
    /// Note that the linear velocity is the velocity of the center of mass, which may not coincide with the position of your object, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$
    pub fn setPositionRotationAndVelocity(self: *BodyInterface, body_id: BodyID, position: RVec3, rotation: Quat, linear_velocity: Vec3, angular_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Update the position
            body.setPositionAndRotationInternal(position, rotation, .{});

            // Notify broadphase of change
            if (body.isInBroadPhase()) {
                var id = body.getID();
                self.broad_phase.notifyBodiesAABBChanged((&id)[0..1], .{});
            }

            if (!body.isStatic()) {
                body.setLinearVelocityClamped(linear_velocity);
                body.setAngularVelocityClamped(angular_velocity);

                // Optionally activate body
                if (!body.isActive() and (!linear_velocity.isNearZero(.{}) or !angular_velocity.isNearZero(.{})) and body.isInBroadPhase())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    ///@name Add forces to the body
    /// @see Body::AddForce
    pub fn addForce(self: *BodyInterface, body_id: BodyID, force: Vec3, opts: struct { activation: Activation = .activate }) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic() and (opts.activation == .activate or body.isActive())) {
                body.addForce(force);

                if (opts.activation == .activate)
                    self.activateBodyInternal(body);
            }
        }
    }

    /// Applied at world space position inPoint. @see Body::AddForce (Jolt: AddForce overload)
    pub fn addForceAtPosition(self: *BodyInterface, body_id: BodyID, force: Vec3, point: RVec3, opts: struct { activation: Activation = .activate }) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic() and (opts.activation == .activate or body.isActive())) {
                body.addForceAtPosition(force, point);

                if (opts.activation == .activate)
                    self.activateBodyInternal(body);
            }
        }
    }

    /// @see Body::AddTorque
    pub fn addTorque(self: *BodyInterface, body_id: BodyID, torque: Vec3, opts: struct { activation: Activation = .activate }) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic() and (opts.activation == .activate or body.isActive())) {
                body.addTorque(torque);

                if (opts.activation == .activate)
                    self.activateBodyInternal(body);
            }
        }
    }

    /// A combination of Body::AddForce and Body::AddTorque
    pub fn addForceAndTorque(self: *BodyInterface, body_id: BodyID, force: Vec3, torque: Vec3, opts: struct { activation: Activation = .activate }) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic() and (opts.activation == .activate or body.isActive())) {
                body.addForce(force);
                body.addTorque(torque);

                if (opts.activation == .activate)
                    self.activateBodyInternal(body);
            }
        }
    }

    ///@name Add an impulse to the body
    /// Applied at center of mass. @see Body::AddImpulse
    pub fn addImpulse(self: *BodyInterface, body_id: BodyID, impulse: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic()) {
                body.addImpulse(impulse);

                if (!body.isActive())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// Applied at world space position inPoint. @see Body::AddImpulse (Jolt: AddImpulse overload)
    pub fn addImpulseAtPosition(self: *BodyInterface, body_id: BodyID, impulse: Vec3, point: RVec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic()) {
                body.addImpulseAtPosition(impulse, point);

                if (!body.isActive())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// @see Body::AddAngularImpulse
    pub fn addAngularImpulse(self: *BodyInterface, body_id: BodyID, angular_impulse: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic()) {
                body.addAngularImpulse(angular_impulse);

                if (!body.isActive())
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// @see Body::ApplyBuoyancyImpulse
    pub fn applyBuoyancyImpulse(self: *BodyInterface, body_id: BodyID, surface_position: RVec3, surface_normal: Vec3, buoyancy: f32, linear_drag: f32, angular_drag: f32, fluid_velocity: Vec3, gravity: Vec3, delta_time: f32, opts: Body.BuoyancyOptions) bool {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic() and body.applyBuoyancyImpulse(surface_position, surface_normal, buoyancy, linear_drag, angular_drag, fluid_velocity, gravity, delta_time, opts)) {
                self.activateBodyInternal(body);
                return true;
            }
        }

        return false;
    }

    ///@name Body type
    /// @see Body::GetBodyType
    pub fn getBodyType(self: *const BodyInterface, body_id: BodyID) BodyType {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getBodyType()
        else
            return .rigid_body;
    }

    ///@name Body motion type
    /// @see Body::SetMotionType
    pub fn setMotionType(self: *BodyInterface, body_id: BodyID, motion_type: MotionType, activation_mode: Activation) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Deactivate if we're making the body static
            if (body.isActive() and motion_type == .static)
                self.body_manager.deactivateBodies((&body_id)[0..1]);

            body.setMotionType(motion_type);

            // Activate body if requested
            if (motion_type != .static and activation_mode == .activate and body.isInBroadPhase())
                self.activateBodyInternal(body);
        }
    }

    /// @see Body::GetMotionType
    pub fn getMotionType(self: *const BodyInterface, body_id: BodyID) MotionType {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getMotionType()
        else
            return .static;
    }

    ///@name Body motion quality
    /// How well it detects collisions when it has a high velocity
    pub fn setMotionQuality(self: *BodyInterface, body_id: BodyID, motion_quality: MotionQuality) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            self.body_manager.setMotionQuality(lock.getBody(), motion_quality);
    }

    /// @see MotionProperties::GetMotionQuality
    pub fn getMotionQuality(self: *const BodyInterface, body_id: BodyID) MotionQuality {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded() and !lock.getBody().isStatic())
            return lock.getBody().getMotionProperties().getMotionQuality()
        else
            return .discrete;
    }

    /// Get inverse inertia tensor in world space
    /// @see Body::GetInverseInertia
    pub fn getInverseInertia(self: *const BodyInterface, body_id: BodyID) Mat44 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getInverseInertia()
        else
            return Mat44.identity();
    }

    ///@name Restitution
    /// @see Body::SetRestitution
    pub fn setRestitution(self: *BodyInterface, body_id: BodyID, restitution: f32) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            lock.getBody().setRestitution(restitution);
    }

    /// @see Body::GetRestitution
    pub fn getRestitution(self: *const BodyInterface, body_id: BodyID) f32 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getRestitution()
        else
            return 0.0;
    }

    ///@name Friction
    /// @see Body::SetFriction
    pub fn setFriction(self: *BodyInterface, body_id: BodyID, friction: f32) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            lock.getBody().setFriction(friction);
    }

    /// @see Body::GetFriction
    pub fn getFriction(self: *const BodyInterface, body_id: BodyID) f32 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getFriction()
        else
            return 0.0;
    }

    ///@name Gravity factor
    /// @see MotionProperties::SetGravityFactor
    pub fn setGravityFactor(self: *BodyInterface, body_id: BodyID, gravity_factor: f32) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            if (lock.getBody().getMotionPropertiesUncheckedMut()) |mp|
                mp.setGravityFactor(gravity_factor);
        }
    }

    /// @see MotionProperties::GetGravityFactor
    pub fn getGravityFactor(self: *const BodyInterface, body_id: BodyID) f32 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            if (lock.getBody().getMotionPropertiesUnchecked()) |mp|
                return mp.getGravityFactor();
        }
        return 1.0;
    }

    ///@name Max linear velocity
    /// @see MotionProperties::SetMaxLinearVelocity
    pub fn setMaxLinearVelocity(self: *BodyInterface, body_id: BodyID, linear_velocity: f32) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            if (lock.getBody().getMotionPropertiesUncheckedMut()) |mp|
                mp.setMaxLinearVelocity(linear_velocity);
        }
    }

    /// @see MotionProperties::GetMaxLinearVelocity
    pub fn getMaxLinearVelocity(self: *const BodyInterface, body_id: BodyID) f32 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            if (lock.getBody().getMotionPropertiesUnchecked()) |mp|
                return mp.getMaxLinearVelocity();
        }
        return 500.0;
    }

    ///@name Max angular velocity
    /// @see MotionProperties::SetMaxAngularVelocity
    pub fn setMaxAngularVelocity(self: *BodyInterface, body_id: BodyID, angular_velocity: f32) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            if (lock.getBody().getMotionPropertiesUncheckedMut()) |mp|
                mp.setMaxAngularVelocity(angular_velocity);
        }
    }

    /// @see MotionProperties::GetMaxAngularVelocity
    pub fn getMaxAngularVelocity(self: *const BodyInterface, body_id: BodyID) f32 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            if (lock.getBody().getMotionPropertiesUnchecked()) |mp|
                return mp.getMaxAngularVelocity();
        }
        return 0.25 * math.pi * 60.0;
    }

    ///@name Manifold reduction
    /// @see Body::SetUseManifoldReduction
    /// (Zolt: Allocator.Error from flagging the contact cache, nothing was changed)
    pub fn setUseManifoldReduction(self: *BodyInterface, body_id: BodyID, use_reduction: bool) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.getUseManifoldReduction() != use_reduction) {
                // Flag collision cache invalid for this body (Zolt: before the flag is set, Jolt does it after, so that
                // a failed call changes nothing and can be repeated)
                try self.body_manager.invalidateContactCacheForBody(body);

                body.setUseManifoldReduction(use_reduction);
            }
        }
    }

    /// @see Body::GetUseManifoldReduction
    pub fn getUseManifoldReduction(self: *const BodyInterface, body_id: BodyID) bool {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getUseManifoldReduction()
        else
            return true;
    }

    ///@name Sensor
    /// @see Body::SetIsSensor
    pub fn setIsSensor(self: *BodyInterface, body_id: BodyID, is_sensor: bool) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            lock.getBody().setIsSensor(is_sensor);
    }

    /// @see Body::IsSensor
    pub fn isSensor(self: *const BodyInterface, body_id: BodyID) bool {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().isSensor()
        else
            return false;
    }

    ///@name Collision group
    /// @see Body::SetCollisionGroup
    pub fn setCollisionGroup(self: *BodyInterface, body_id: BodyID, collision_group: *const CollisionGroup) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            lock.getBody().setCollisionGroup(collision_group);
    }

    /// @see Body::GetCollisionGroup (a pointer into the body, like Jolt's reference)
    pub fn getCollisionGroup(self: *const BodyInterface, body_id: BodyID) *const CollisionGroup {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getCollisionGroup()
        else
            return &CollisionGroup.invalid;
    }

    /// Get transform and shape for this body, used to perform collision detection
    /// (the caller owns the shape reference of the result: `deinit` it)
    pub fn getTransformedShape(self: *const BodyInterface, body_id: BodyID) TransformedShape {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getTransformedShape()
        else
            return .{};
    }

    /// Get the user data for a body
    pub fn getUserData(self: *const BodyInterface, body_id: BodyID) u64 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getUserData()
        else
            return 0;
    }

    /// Set the user data for a body
    pub fn setUserData(self: *const BodyInterface, body_id: BodyID, user_data: u64) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            lock.getBody().setUserData(user_data);
    }

    /// Get the material for a particular sub shape
    pub fn getMaterial(self: *const BodyInterface, body_id: BodyID, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            return lock.getBody().getShape().getMaterial(sub_shape_id)
        else
            return PhysicsMaterial.default;
    }

    /// Set the Body::EFlags::InvalidateContactCache flag for the specified body. This means that the collision cache is invalid for any body pair involving that body until the next physics step.
    /// (Zolt: Allocator.Error from BodyManager.invalidateContactCacheForBody)
    pub fn invalidateContactCache(self: *BodyInterface, body_id: BodyID) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            try self.body_manager.invalidateContactCacheForBody(lock.getBody());
    }
};

/// A physics system for the soft body tests (in place: it must not move)
const TestSoftBodyWorld = struct {
    const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
    const BroadPhaseLayerInterfaceTable = @import("../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
    const ObjectVsBroadPhaseLayerFilterTable = @import("../Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
    const ObjectLayerPairFilterTable = @import("../Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;

    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    system: PhysicsSystem,

    fn init(self: *TestSoftBodyWorld, allocator: Allocator, max_bodies: u32) !void {
        self.layer_interface = try .init(allocator, 1, 1);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(0, .init(0));
        self.pair_filter = try .init(allocator, 1);
        errdefer self.pair_filter.deinit();
        self.pair_filter.enableCollision(0, 0);
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, 1, &self.pair_filter.base, 1);
        errdefer self.ovb_filter.deinit();
        self.system = .empty;
        try self.system.init(allocator, std.testing.io, max_bodies, 0, 64, 64, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
    }

    fn deinit(self: *TestSoftBodyWorld) void {
        self.system.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }

    /// Shared settings that SoftBodyMotionProperties.initialize accepts (one hand-made update group), reference count 0
    fn sharedSettings(allocator: Allocator) !*@import("../SoftBody/SoftBodySharedSettings.zig").SoftBodySharedSettings {
        const SoftBodySharedSettings = @import("../SoftBody/SoftBodySharedSettings.zig").SoftBodySharedSettings;
        const s = try SoftBodySharedSettings.create(allocator);
        errdefer {
            s.deinit();
            allocator.destroy(s);
        }
        try s.vertices.append(allocator, .init(.{ .x = -1, .y = 0, .z = 0 }, .{}));
        try s.vertices.append(allocator, .init(.{ .x = 1, .y = 0, .z = 0 }, .{}));
        try s.vertices.append(allocator, .init(.{ .x = 0, .y = 1, .z = 1 }, .{}));
        try s.addFace(.init(0, 1, 2, .{}));
        try s.update_groups.append(allocator, std.mem.zeroes(SoftBodySharedSettings.UpdateGroup));
        return s;
    }
};

test "BodyInterface: createSoftBody, createSoftBodyWithID, createSoftBodyWithoutID and createAndAddSoftBody (R16)" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;

    var world: TestSoftBodyWorld = undefined;
    try world.init(allocator, 4);
    defer world.deinit();
    const bi = world.system.getBodyInterfaceMut();

    var settings: SoftBodyCreationSettings = .init(try TestSoftBodyWorld.sharedSettings(allocator), RVec3.init(0, 5, 0), Quat.identity(), 0);
    defer settings.deinit();

    // CreateSoftBody: an ID, not in the broad phase yet
    const a = (try bi.createSoftBody(&settings)).?;
    try expect(a.isSoftBody() and !a.getID().isInvalid());
    try expect(!bi.isAdded(a.getID()));

    // CreateSoftBodyWithID: the requested ID, null when it is taken
    const id = BodyID.init(3);
    const b = (try bi.createSoftBodyWithID(id, &settings)).?;
    try expect(b.getID().eql(id));
    try expect((try bi.createSoftBodyWithID(id, &settings)) == null);

    // CreateSoftBodyWithoutID: no ID until AssignBodyID
    const c = try bi.createSoftBodyWithoutID(&settings);
    try expect(c.getID().isInvalid());
    try expect(bi.assignBodyID(c));

    // CreateAndAddSoftBody: added and active
    const d = try bi.createAndAddSoftBody(&settings, .activate);
    try expect(bi.isAdded(d) and bi.isActive(d));
    const stats = world.system.body_manager.getBodyStats();
    try expectEqual(@as(u32, 4), stats.num_soft_bodies);
    try expectEqual(@as(u32, 1), stats.num_active_soft_bodies);

    // Out of bodies: null and an invalid ID, nothing is left behind
    try expect((try bi.createSoftBody(&settings)) == null);
    try expect((try bi.createAndAddSoftBody(&settings, .activate)).isInvalid());
    const without_id = try bi.createSoftBodyWithoutID(&settings);
    try expect(!bi.assignBodyID(without_id));
    bi.destroyBodyWithoutID(without_id);

    // Remove and destroy, the last soft bodies go with the system
    bi.removeBody(d);
    bi.destroyBody(d);
    bi.destroyBody(a.getID());
    try expectEqual(@as(u32, 2), world.system.getNumBodies());
}

test "BodyInterface: out of memory in createAndAddSoftBody" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            var world: TestSoftBodyWorld = undefined;
            try world.init(allocator, 2);
            defer world.deinit();
            var settings: SoftBodyCreationSettings = .init(try TestSoftBodyWorld.sharedSettings(allocator), RVec3.zero(), Quat.identity(), 0);
            defer settings.deinit();
            const bi = world.system.getBodyInterfaceMut();
            const id = try bi.createAndAddSoftBody(&settings, .activate);
            try std.testing.expect(bi.isAdded(id));
            // On error the body was destroyed again: only the bodies that were added are left
            try std.testing.expectEqual(@as(u32, 1), world.system.getNumBodies());
        }
    }.f, .{});
}
