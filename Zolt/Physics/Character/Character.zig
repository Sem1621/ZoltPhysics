//! Port of: Jolt/Physics/Character/Character.h, Jolt/Physics/Character/Character.cpp
//! Status: complete
//!
//! CharacterVehicleArchitecture E2:
//! - `CharacterSettings` embeds `base: CharacterBaseSettings` (pattern A, CharacterBase.zig). `init(allocator)` is
//!   Jolt's default constructor for settings on the stack or in a member (`base.setEmbedded()` before taking
//!   references, `base.deinit()` at the end), `create(allocator)` is `new CharacterSettings` (reference count 0, the
//!   last `release` of a `RefConst(CharacterBaseSettings)` destroys it), `initCopy` / `assign` the defaulted copy
//!   constructor / assignment (they take a reference to the shape like Jolt's RefConst copy).
//! - `Character` embeds `base: CharacterBase` (pattern A with `ref_count`). `create(allocator, settings, position,
//!   rotation, user_data, system)` is `new Character(...)` (reference count 0: the last `release` of a
//!   `RefConst(CharacterBase)` runs the destructor, which destroys the body), `init` is the same constructor for a
//!   character on the stack or in a member (`base.setEmbedded()` before taking references, `base.deinit()`).
//!   Character overrides no virtual function: SaveState / RestoreState are CharacterBase's
//!   (`character.base.saveState(recorder)`), and so are the ground getters (`character.base.getGroundState()`).
//! - Jolt's defaulted arguments are options: `EActivation inActivationMode = EActivation::Activate` is `.activation`,
//!   `bool inLockBodies = true` is `.lock_bodies` (`character.setPosition(position, .{ .activation = .dont_activate,
//!   .lock_bodies = false })`). `lock_bodies` selects the locking or the no-lock BodyInterface, BodyLockInterface and
//!   NarrowPhaseQuery of the system exactly where Jolt does (sCharacterGetBodyInterface and its siblings); the
//!   constructor and the destructor always use the locking BodyInterface, as in Jolt.
//! - `GetPositionAndRotation(outPosition, outRotation)` returns BodyInterface's `PositionAndRotation`. The two private
//!   overloads of `CheckCollision` are `checkCollisionWithTransform` (a center of mass transform) and
//!   `checkCollisionAtBody` (the current transform of the body); the public overload keeps the name `checkCollision`.
//!   The local classes of Character.cpp (the body filter of CheckCollision, the collectors of PostSimulation and
//!   SetShape) are pattern A structs (CollisionArchitecture D5, D7).
//! - `getTransformedShape` and `getCharacterSettings` return values that hold a reference to the shape: the caller
//!   `deinit`s the transformed shape and `base.deinit()`s the settings (the settings use the character's allocator;
//!   `base.setEmbedded()` before taking references to them).
//!
//! Errors (Zolt):
//! - `create` / `init` return `CreateBodyError`: allocation failure, or settings without a shape (Jolt dereferences the
//!   null shape in CreateBody). Nothing is left behind (no body, the references of CharacterBase are released). Out of
//!   bodies is not an error, as in Jolt: the character has no body (`getBodyID()` is invalid) and every function that
//!   goes through the BodyInterface does nothing (PostSimulation returns without changing the ground state).
//! - `addToPhysicsSystem` (the broad phase allocates in AddBodiesPrepare), `setLayer` (BroadPhase.
//!   notifyBodiesLayerChanged) and `setShape` (BodyInterface.setShape flags the contact cache invalid) return
//!   `Allocator.Error`. A failed call changes nothing: `setLayer` restores the layer of the character, `setShape` sets
//!   the character's shape only after the body's (Jolt sets mShape first; the reference counts end the same).
//!
//! Jolt behavior that Zolt does not reproduce (undefined in Jolt, documented deviations):
//! - `~Character` calls DestroyBody(mBodyID) without checking the ID: for a character whose body could not be created
//!   (out of bodies) Jolt's BodyManager indexes its body array with the index of the invalid ID (out of bounds: a crash
//!   or memory corruption). Zolt's destructor skips the invalid ID.
//! - `GetCharacterSettings` asserts that the body lock succeeded and then dereferences the body: Zolt panics with a
//!   message when the character has no body (also without asserts, where Jolt dereferences null).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const IgnoreSingleBodyFilter = @import("../Body/BodyFilter.zig").IgnoreSingleBodyFilter;
const AllowedDOFs = @import("../Body/AllowedDOFs.zig").AllowedDOFs;
const BodyCreationSettingsFile = @import("../Body/BodyCreationSettings.zig");
const BodyCreationSettings = BodyCreationSettingsFile.BodyCreationSettings;
const CreateBodyError = BodyCreationSettingsFile.CreateBodyError;
const BodyInterface = @import("../Body/BodyInterface.zig").BodyInterface;
const BodyLockInterface = @import("../Body/BodyLockInterface.zig").BodyLockInterface;
const BodyLockRead = @import("../Body/BodyLock.zig").BodyLockRead;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const CollideShapeFile = @import("../Collision/CollideShape.zig");
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const NarrowPhaseQuery = @import("../Collision/NarrowPhaseQuery.zig").NarrowPhaseQuery;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const Activation = @import("../EActivation.zig").Activation;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const CharacterBaseFile = @import("CharacterBase.zig");
const CharacterBaseSettings = CharacterBaseFile.CharacterBaseSettings;
const CharacterBase = CharacterBaseFile.CharacterBase;

/// Contains the configuration of a character
pub const CharacterSettings = struct {
    /// Jolt's class has no virtual function besides the destructor of CharacterBaseSettings
    pub const overrides = .{};

    base: CharacterBaseSettings,

    /// Layer that this character will be added to
    layer: ObjectLayer = 0,

    /// Mass of the character
    mass: f32 = 80.0,

    /// Friction for the character
    friction: f32 = 0.2,

    /// Value to multiply gravity with for this character
    gravity_factor: f32 = 1.0,

    /// Allowed degrees of freedom for this character
    allowed_dofs: AllowedDOFs = .{ .translation_x = true, .translation_y = true, .translation_z = true },

    /// Constructor (settings on the stack or in a member: `base.setEmbedded()` before taking references,
    /// `base.deinit()` at the end)
    pub fn init(allocator: Allocator) CharacterSettings {
        return .{ .base = .init(CharacterSettings, allocator) };
    }

    /// `new CharacterSettings`: reference count 0, the last release of a RefConst(CharacterBaseSettings) destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*CharacterSettings {
        const self = try allocator.create(CharacterSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The defaulted copy constructor: copies the settings of `other` (taking a reference to its shape), not its
    /// reference count
    pub fn initCopy(allocator: Allocator, other: *const CharacterSettings) CharacterSettings {
        return .{
            .base = .initCopy(CharacterBaseSettings.vtableFor(CharacterSettings), allocator, &other.base),
            .layer = other.layer,
            .mass = other.mass,
            .friction = other.friction,
            .gravity_factor = other.gravity_factor,
            .allowed_dofs = other.allowed_dofs,
        };
    }

    /// The defaulted copy assignment (operator =): copies the settings of `other` (the shape reference too), keeps the
    /// reference count and the allocator
    pub fn assign(self: *CharacterSettings, other: *const CharacterSettings) void {
        self.base.assign(&other.base);
        self.layer = other.layer;
        self.mass = other.mass;
        self.friction = other.friction;
        self.gravity_factor = other.gravity_factor;
        self.allowed_dofs = other.allowed_dofs;
    }
};

/// sCharacterGetBodyLockInterface: the locking or the no-lock body lock interface of the system
fn characterGetBodyLockInterface(system: *const PhysicsSystem, lock_bodies: bool) *const BodyLockInterface {
    return if (lock_bodies) system.getBodyLockInterface() else system.getBodyLockInterfaceNoLock();
}

/// sCharacterGetBodyInterface: the locking or the no-lock body interface of the system
fn characterGetBodyInterface(system: *PhysicsSystem, lock_bodies: bool) *BodyInterface {
    return if (lock_bodies) system.getBodyInterfaceMut() else system.getBodyInterfaceNoLockMut();
}

/// sCharacterGetNarrowPhaseQuery: the locking or the no-lock narrow phase query of the system
fn characterGetNarrowPhaseQuery(system: *const PhysicsSystem, lock_bodies: bool) *const NarrowPhaseQuery {
    return if (lock_bodies) system.getNarrowPhaseQuery() else system.getNarrowPhaseQueryNoLock();
}

/// Runtime character object.
/// This object usually represents the player or a humanoid AI. It uses a single rigid body,
/// usually with a capsule shape to simulate movement and collision for the character.
/// The character is a keyframed object, the application controls it by setting the velocity.
pub const Character = struct {
    /// Character overrides no virtual function of CharacterBase
    pub const overrides = .{};

    base: CharacterBase,

    /// The body of this character
    body_id: BodyID = .invalid,

    /// The layer the body is in
    layer: ObjectLayer,

    /// The options of the functions with Jolt's defaulted `bool inLockBodies = true`
    pub const LockOptions = struct {
        /// If the bodies should be locked (true: the locking interfaces of the system, false: the no-lock interfaces)
        lock_bodies: bool = true,
    };

    /// The options of the functions with Jolt's defaulted `EActivation inActivationMode = EActivation::Activate, bool
    /// inLockBodies = true`
    pub const ActivationOptions = struct {
        /// If the body should be activated
        activation: Activation = .activate,
        /// If the bodies should be locked (true: the locking interfaces of the system, false: the no-lock interfaces)
        lock_bodies: bool = true,
    };

    /// Constructor: `new Character(...)`, reference count 0 (put it in a RefConst(CharacterBase): the last release
    /// destroys the character and its body)
    /// @param settings The settings for the character
    /// @param position Initial position for the character
    /// @param rotation Initial rotation for the character (usually only around Y)
    /// @param user_data Application specific value
    /// @param system Physics system that this character will be added to later
    pub fn create(allocator: Allocator, settings: *const CharacterSettings, position: RVec3, rotation: Quat, user_data: u64, system: *PhysicsSystem) CreateBodyError!*Character {
        const self = try allocator.create(Character);
        errdefer allocator.destroy(self);
        self.* = try init(allocator, settings, position, rotation, user_data, system);
        return self;
    }

    /// Constructor of a character on the stack or in a member: `base.setEmbedded()` before taking references,
    /// `base.deinit()` at the end (destroys its body). See `create`.
    pub fn init(allocator: Allocator, settings: *const CharacterSettings, position: RVec3, rotation: Quat, user_data: u64, system: *PhysicsSystem) CreateBodyError!Character {
        var self: Character = .{
            .base = .init(Character, allocator, &settings.base, system),
            .layer = settings.layer,
        };
        errdefer self.base.deinit(); // Zolt: no body was created, release the references that CharacterBase took

        // Construct rigid body
        var body_settings: BodyCreationSettings = .initWithShape(self.base.shape.get(), position, rotation, .dynamic, self.layer);
        defer body_settings.deinit();
        body_settings.allowed_dofs = settings.allowed_dofs;
        body_settings.enhanced_internal_edge_removal = settings.base.enhanced_internal_edge_removal;
        body_settings.override_mass_properties = .mass_and_inertia_provided;
        body_settings.mass_properties_override.mass = settings.mass;
        body_settings.friction = settings.friction;
        body_settings.gravity_factor = settings.gravity_factor;
        body_settings.user_data = user_data;
        const body = try system.getBodyInterfaceMut().createBody(&body_settings);
        if (body) |b|
            self.body_id = b.getID();
        return self;
    }

    /// Destructor (~Character, called by the generated destructor chain before CharacterBase's)
    pub fn destruct(self: *Character) void {
        // Destroy the body
        // (Zolt: not for the invalid ID of a character without body, which Jolt passes on, see the file header)
        if (!self.body_id.isInvalid())
            self.base.system.getBodyInterfaceMut().destroyBody(self.body_id);
    }

    /// Add bodies and constraints to the system and optionally activate the bodies
    /// (Zolt: Allocator.Error from the broad phase, the body was not added then)
    pub fn addToPhysicsSystem(self: *Character, opts: ActivationOptions) Allocator.Error!void {
        try characterGetBodyInterface(self.base.system, opts.lock_bodies).addBody(self.body_id, opts.activation);
    }

    /// Remove bodies and constraints from the system
    pub fn removeFromPhysicsSystem(self: *Character, opts: LockOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).removeBody(self.body_id);
    }

    /// Wake up the character
    pub fn activate(self: *Character, opts: LockOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).activateBody(self.body_id);
    }

    /// Check collisions between shape and the world using the center of mass transform (private CheckCollision)
    fn checkCollisionWithTransform(self: *const Character, center_of_mass_transform: RMat44, movement_direction: Vec3, max_separation_distance: f32, shape: *const Shape, base_offset: RVec3, collector: *CollideShapeCollector, lock_bodies: bool) void {
        // Create query broadphase layer filter
        const broadphase_layer_filter = self.base.system.getDefaultBroadPhaseLayerFilter(self.layer);

        // Create query object layer filter
        const object_layer_filter = self.base.system.getDefaultLayerFilter(self.layer);

        // Ignore sensors and my own body
        const body_filter: CharacterBodyFilter = .init(self.body_id);

        // Settings for collide shape
        var settings: CollideShapeSettings = .{};
        settings.max_separation_distance = max_separation_distance;
        settings.active_edge_mode = .collide_only_with_active;
        settings.active_edge_movement_direction = movement_direction;
        settings.back_face_mode = .ignore_back_faces;

        characterGetNarrowPhaseQuery(self.base.system, lock_bodies).collideShape(shape, Vec3.one(), center_of_mass_transform, &settings, base_offset, collector, .{
            .broad_phase_layer_filter = &broadphase_layer_filter.base,
            .object_layer_filter = &object_layer_filter.base,
            .body_filter = &body_filter.base.base,
        });
    }

    /// @brief Get all contacts for the character at a particular location
    /// @param position Position to test.
    /// @param rotation Rotation at which to test the shape.
    /// @param movement_direction A hint in which direction the character is moving, will be used to calculate a proper normal.
    /// @param max_separation_distance How much distance around the character you want to report contacts in (can be 0 to match the character exactly).
    /// @param shape Shape to test collision with.
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
    /// @param collector Collision collector that receives the collision results.
    /// @param opts.lock_bodies If the collision query should use the locking body interface (true) or the non locking body interface (false)
    pub fn checkCollision(self: *const Character, position: RVec3, rotation: Quat, movement_direction: Vec3, max_separation_distance: f32, shape: *const Shape, base_offset: RVec3, collector: *CollideShapeCollector, opts: LockOptions) void {
        // Calculate center of mass transform
        const center_of_mass = RMat44.rotationTranslation(rotation, position).preTranslated(shape.getCenterOfMass());

        self.checkCollisionWithTransform(center_of_mass, movement_direction, max_separation_distance, shape, base_offset, collector, opts.lock_bodies);
    }

    /// Check collisions between shape and the world using the current position / rotation of the character (private
    /// CheckCollision)
    fn checkCollisionAtBody(self: *const Character, shape: *const Shape, max_separation_distance: f32, base_offset: RVec3, collector: *CollideShapeCollector, lock_bodies: bool) void {
        // Determine position and velocity of body
        var query_transform: RMat44 = undefined;
        var velocity: Vec3 = undefined;
        {
            var lock: BodyLockRead = .init(characterGetBodyLockInterface(self.base.system, lock_bodies), self.body_id);
            defer lock.deinit();
            if (!lock.succeeded())
                return;

            const body = lock.getBody();

            // Correct the center of mass transform for the difference between the old and new center of mass shape
            query_transform = body.getCenterOfMassTransform().preTranslated(shape.getCenterOfMass().sub(self.base.shape.get().?.getCenterOfMass()));
            velocity = body.getLinearVelocity();
        }

        self.checkCollisionWithTransform(query_transform, velocity, max_separation_distance, shape, base_offset, collector, lock_bodies);
    }

    /// Needs to be called after every PhysicsSystem::Update
    /// @param max_separation_distance Max distance between the floor and the character to still consider the character standing on the floor
    /// @param opts.lock_bodies If the collision query should use the locking body interface (true) or the non locking body interface (false)
    pub fn postSimulation(self: *Character, max_separation_distance: f32, opts: LockOptions) void {
        // Get character position, rotation and velocity
        var char_pos: RVec3 = undefined;
        var char_rot: Quat = undefined;
        var char_vel: Vec3 = undefined;
        {
            var lock: BodyLockRead = .init(characterGetBodyLockInterface(self.base.system, opts.lock_bodies), self.body_id);
            defer lock.deinit();
            if (!lock.succeeded())
                return;
            const body = lock.getBody();
            char_pos = body.getPosition();
            char_rot = body.getRotation();
            char_vel = body.getLinearVelocity();
        }

        // Collide shape
        var collector: GroundCollector = .init(self.base.up, char_pos);
        self.checkCollision(char_pos, char_rot, char_vel, max_separation_distance, self.base.shape.get().?, char_pos, &collector.base, opts);

        // Copy results
        self.base.ground_body_id = collector.ground_body_id;
        self.base.ground_body_sub_shape_id = collector.ground_body_sub_shape_id;
        self.base.ground_position = collector.ground_position;
        self.base.ground_normal = collector.ground_normal;

        // Get additional data from body
        var lock: BodyLockRead = .init(characterGetBodyLockInterface(self.base.system, opts.lock_bodies), self.base.ground_body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Update ground state
            const inv_transform = RMat44.inverseRotationTranslation(char_rot, char_pos);
            if (self.base.supporting_volume.signedDistance(inv_transform.mulRVec3(self.base.ground_position).toVec3()) > 0.0)
                self.base.ground_state = .not_supported
            else if (self.base.isSlopeTooSteep(self.base.ground_normal))
                self.base.ground_state = .on_steep_ground
            else
                self.base.ground_state = .on_ground;

            // Copy other body properties
            self.base.ground_material.set(body.getShape().getMaterial(self.base.ground_body_sub_shape_id));
            self.base.ground_velocity = body.getPointVelocity(self.base.ground_position);
            self.base.ground_user_data = body.getUserData();
        } else {
            self.base.ground_state = .in_air;
            self.base.ground_material.set(PhysicsMaterial.default);
            self.base.ground_velocity = Vec3.zero();
            self.base.ground_user_data = 0;
        }
    }

    /// Control the velocity of the character
    pub fn setLinearAndAngularVelocity(self: *Character, linear_velocity: Vec3, angular_velocity: Vec3, opts: LockOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).setLinearAndAngularVelocity(self.body_id, linear_velocity, angular_velocity);
    }

    /// Get the linear velocity of the character (m / s)
    pub fn getLinearVelocity(self: *const Character, opts: LockOptions) Vec3 {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getLinearVelocity(self.body_id);
    }

    /// Set the linear velocity of the character (m / s)
    pub fn setLinearVelocity(self: *Character, linear_velocity: Vec3, opts: LockOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).setLinearVelocity(self.body_id, linear_velocity);
    }

    /// Add world space linear velocity to current velocity (m / s)
    pub fn addLinearVelocity(self: *Character, linear_velocity: Vec3, opts: LockOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).addLinearVelocity(self.body_id, linear_velocity);
    }

    /// Add impulse to the center of mass of the character
    pub fn addImpulse(self: *Character, impulse: Vec3, opts: LockOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).addImpulse(self.body_id, impulse);
    }

    /// Get the body associated with this character
    pub fn getBodyID(self: *const Character) BodyID {
        return self.body_id;
    }

    /// Get position / rotation of the body
    pub fn getPositionAndRotation(self: *const Character, opts: LockOptions) BodyInterface.PositionAndRotation {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getPositionAndRotation(self.body_id);
    }

    /// Set the position / rotation of the body, optionally activating it.
    pub fn setPositionAndRotation(self: *const Character, position: RVec3, rotation: Quat, opts: ActivationOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).setPositionAndRotation(self.body_id, position, rotation, opts.activation);
    }

    /// Get the position of the character
    pub fn getPosition(self: *const Character, opts: LockOptions) RVec3 {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getPosition(self.body_id);
    }

    /// Set the position of the character, optionally activating it.
    pub fn setPosition(self: *Character, position: RVec3, opts: ActivationOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).setPosition(self.body_id, position, opts.activation);
    }

    /// Get the rotation of the character
    pub fn getRotation(self: *const Character, opts: LockOptions) Quat {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getRotation(self.body_id);
    }

    /// Set the rotation of the character, optionally activating it.
    pub fn setRotation(self: *Character, rotation: Quat, opts: ActivationOptions) void {
        characterGetBodyInterface(self.base.system, opts.lock_bodies).setRotation(self.body_id, rotation, opts.activation);
    }

    /// Position of the center of mass of the underlying rigid body
    pub fn getCenterOfMassPosition(self: *const Character, opts: LockOptions) RVec3 {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getCenterOfMassPosition(self.body_id);
    }

    /// Calculate the world transform of the character
    pub fn getWorldTransform(self: *const Character, opts: LockOptions) RMat44 {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getWorldTransform(self.body_id);
    }

    /// Get the layer of the character
    pub fn getLayer(self: *const Character) ObjectLayer {
        return self.layer;
    }

    /// Update the layer of the character
    /// (Zolt: Allocator.Error from the broad phase, nothing changed then)
    pub fn setLayer(self: *Character, layer: ObjectLayer, opts: LockOptions) Allocator.Error!void {
        const old_layer = self.layer;
        self.layer = layer;
        errdefer self.layer = old_layer; // Zolt: the body kept its layer, so does the character

        try characterGetBodyInterface(self.base.system, opts.lock_bodies).setObjectLayer(self.body_id, layer);
    }

    /// Switch the shape of the character (e.g. for stance). When max_penetration_depth is not FLT_MAX, it checks
    /// if the new shape collides before switching shape. Returns true if the switch succeeded.
    /// (Zolt: Allocator.Error from BodyInterface.setShape, nothing changed then)
    pub fn setShape(self: *Character, shape: *const Shape, max_penetration_depth: f32, opts: LockOptions) Allocator.Error!bool {
        if (max_penetration_depth < math.flt_max) {
            // Test if anything is in the way of switching
            const char_pos = self.getPosition(opts);
            var collector: PenetrationCollector = .init(max_penetration_depth);
            self.checkCollisionAtBody(shape, 0.0, char_pos, &collector.base, opts.lock_bodies);
            if (collector.had_collision)
                return false;
        }

        // Switch the shape
        // (Zolt: the body first, BodyInterface.setShape fails with nothing changed; Jolt assigns mShape first and passes
        // it on, the shape and the reference counts are the same afterwards)
        try characterGetBodyInterface(self.base.system, opts.lock_bodies).setShape(self.body_id, shape, false, .activate);
        self.base.shape.set(shape);
        return true;
    }

    /// Get the transformed shape that represents the volume of the character, can be used for collision checks.
    /// (the result holds a reference to the shape: `deinit` it)
    pub fn getTransformedShape(self: *const Character, opts: LockOptions) TransformedShape {
        return characterGetBodyInterface(self.base.system, opts.lock_bodies).getTransformedShape(self.body_id);
    }

    /// Get the character settings that can recreate this character
    /// (a value with the character's allocator that holds a reference to the shape: `base.setEmbedded()` before taking
    /// references to it, `base.deinit()` at the end)
    pub fn getCharacterSettings(self: *const Character, opts: LockOptions) CharacterSettings {
        var lock: BodyLockRead = .init(characterGetBodyLockInterface(self.base.system, opts.lock_bodies), self.body_id);
        defer lock.deinit();
        if (!lock.succeeded()) // JPH_ASSERT(lock.Succeeded()), after which Jolt dereferences the null body
            @panic("Character.getCharacterSettings: the character has no body");
        const body = lock.getBody();

        var settings: CharacterSettings = .init(self.base.allocator);
        settings.base.up = self.base.up;
        settings.base.supporting_volume = self.base.supporting_volume;
        settings.base.max_slope_angle = trigonometry.acos(self.base.cos_max_slope_angle);
        settings.base.enhanced_internal_edge_removal = body.getEnhancedInternalEdgeRemoval();
        settings.base.shape.set(self.base.shape.get());
        settings.layer = self.layer;
        const mp = body.getMotionProperties();
        settings.mass = 1.0 / mp.getInverseMass();
        settings.friction = body.getFriction();
        settings.gravity_factor = mp.getGravityFactor();
        settings.allowed_dofs = mp.getAllowedDOFs();
        return settings;
    }

    /// Ignore sensors and my own body (the local class CharacterBodyFilter of CheckCollision)
    const CharacterBodyFilter = struct {
        pub const overrides = .{.shouldCollideLocked};

        base: IgnoreSingleBodyFilter,

        /// using IgnoreSingleBodyFilter::IgnoreSingleBodyFilter
        fn init(body_id: BodyID) CharacterBodyFilter {
            return .{ .base = .{ .base = .init(CharacterBodyFilter), .body_id = body_id } };
        }

        pub fn shouldCollideLocked(self: *const CharacterBodyFilter, body: *const Body) bool {
            _ = self;
            return !body.isSensor();
        }
    };

    /// Collector that finds the hit with the normal that is the most 'up' (the local class MyCollector of
    /// PostSimulation)
    const GroundCollector = struct {
        pub const overrides = .{.addHit};

        base: CollideShapeCollector,
        ground_body_id: BodyID = .invalid,
        ground_body_sub_shape_id: SubShapeID = .empty,
        ground_position: RVec3 = RVec3.zero(),
        ground_normal: Vec3 = Vec3.zero(),
        base_offset: RVec3,
        up: Vec3,
        best_dot: f32 = -math.flt_max,

        // Constructor
        fn init(up: Vec3, base_offset: RVec3) GroundCollector {
            return .{ .base = .init(GroundCollector), .base_offset = base_offset, .up = up };
        }

        // See: CollectorType::AddHit
        pub fn addHit(self: *GroundCollector, result: *const CollideShapeResult) void {
            const normal = result.penetration_axis.normalized().negate();
            const dot = normal.dot(self.up);
            if (dot > self.best_dot) { // Find the hit that is most aligned with the up vector
                self.ground_body_id = result.body_id2;
                self.ground_body_sub_shape_id = result.sub_shape_id2;
                self.ground_position = self.base_offset.addVec3(result.contact_point_on2);
                self.ground_normal = normal;
                self.best_dot = dot;
            }
        }
    };

    /// Collector that checks if there is anything in the way while switching to a shape (the local class MyCollector of
    /// SetShape)
    const PenetrationCollector = struct {
        pub const overrides = .{.addHit};

        base: CollideShapeCollector,
        max_penetration_depth: f32,
        had_collision: bool = false,

        // Constructor
        fn init(max_penetration_depth: f32) PenetrationCollector {
            return .{ .base = .init(PenetrationCollector), .max_penetration_depth = max_penetration_depth };
        }

        // See: CollectorType::AddHit
        pub fn addHit(self: *PenetrationCollector, result: *const CollideShapeResult) void {
            if (result.penetration_depth > self.max_penetration_depth) {
                self.had_collision = true;
                self.base.forceEarlyOut();
            }
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt has no Character unit tests): every function with and without locking the bodies, the four ground states
// on boxes, slopes and meshes, moving and rotating ground, SetShape with and without max penetration depth, layer
// changes, a simulated walk, out of bodies, a settings object without shape and allocation failure. The bit exact
// comparison with Jolt is the simulation parity (ZoltParity/Physics/SimScenes.zig, the Character region).

const testing = std.testing;
const Color = @import("../../Core/Color.zig").Color;
const TempAllocatorImpl = @import("../../Core/TempAllocator.zig").TempAllocatorImpl;
const JobSystemSingleThreaded = @import("../../Core/JobSystemSingleThreaded.zig").JobSystemSingleThreaded;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const Triangle = @import("../../Geometry/Triangle.zig").Triangle;
const BroadPhaseLayerInterfaceTable = @import("../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
const ObjectVsBroadPhaseLayerFilterTable = @import("../Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
const ObjectLayerPairFilterTable = @import("../Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
const BoxShape = @import("../Collision/Shape/BoxShape.zig").BoxShape;
const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;
const CapsuleShape = @import("../Collision/Shape/CapsuleShape.zig").CapsuleShape;
const RotatedTranslatedShape = @import("../Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShape;
const MeshShapeSettings = @import("../Collision/Shape/MeshShape.zig").MeshShapeSettings;
const ShapeSettings = ShapeFile.ShapeSettings;
const AllHitCollisionCollector = @import("../Collision/CollisionCollectorImpl.zig").AllHitCollisionCollector;
const PhysicsMaterialSimple = @import("../Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const Ref = @import("../../Core/Reference.zig").Ref;
const Real = RealFile.Real;

/// The object layers of the tests
const TestLayers = struct {
    /// Static bodies (broad phase layer 0)
    const non_moving: ObjectLayer = 0;
    /// Dynamic and kinematic bodies (broad phase layer 1)
    const moving: ObjectLayer = 1;
    /// Collides with nothing (broad phase layer 2: a change between moving and ghost moves the body to another tree of
    /// the broad phase, which allocates)
    const ghost: ObjectLayer = 2;
    const count = 3;
    const num_broad_phase_layers = 3;
};

/// A physics system with the test layers, a temp allocator and a job system (in place: it must not move)
const TestWorld = struct {
    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    system: PhysicsSystem,
    temp: TempAllocatorImpl,
    jobs: JobSystemSingleThreaded,

    fn init(self: *TestWorld, allocator: Allocator, max_bodies: u32) !void {
        self.layer_interface = try .init(allocator, TestLayers.count, TestLayers.num_broad_phase_layers);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(TestLayers.non_moving, .init(0));
        self.layer_interface.mapObjectToBroadPhaseLayer(TestLayers.moving, .init(1));
        self.layer_interface.mapObjectToBroadPhaseLayer(TestLayers.ghost, .init(2));
        self.pair_filter = try .init(allocator, TestLayers.count);
        errdefer self.pair_filter.deinit();
        self.pair_filter.enableCollision(TestLayers.non_moving, TestLayers.moving);
        self.pair_filter.enableCollision(TestLayers.moving, TestLayers.moving);
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, TestLayers.num_broad_phase_layers, &self.pair_filter.base, TestLayers.count);
        errdefer self.ovb_filter.deinit();
        self.system = .empty;
        try self.system.init(allocator, testing.io, max_bodies, 0, 256, 256, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
        errdefer self.system.deinit();
        self.temp = try .init(allocator, 1024 * 1024);
        errdefer self.temp.deinit();
        self.jobs = try .init(allocator, testing.io, 256);
    }

    fn deinit(self: *TestWorld) void {
        self.jobs.deinit();
        self.temp.deinit();
        self.system.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }

    /// One update of 1/60 s
    fn update(self: *TestWorld) !void {
        const errors = try self.system.update(1.0 / 60.0, 1, self.temp.tempAllocator(), self.jobs.jobSystem());
        try testing.expect(errors.isNone());
        try testing.expect(self.temp.isEmpty());
    }

    /// A body added to the system (static bodies in the non moving layer, the others in the moving layer)
    fn addBody(self: *TestWorld, shape: *const Shape, position: RVec3, rotation: Quat, motion_type: MotionType, user_data: u64) !BodyID {
        var settings: BodyCreationSettings = .initWithShape(shape, position, rotation, motion_type, if (motion_type == .static) TestLayers.non_moving else TestLayers.moving);
        defer settings.deinit();
        settings.user_data = user_data;
        const id = try self.system.getBodyInterfaceMut().createAndAddBody(&settings, if (motion_type == .static) .dont_activate else .activate);
        try testing.expect(!id.isInvalid());
        return id;
    }
};

/// The shapes of the tests: the capsules of Jolt's CharacterTest sample (cCharacterHeightStanding = 1.35,
/// cCharacterHeightCrouching = 0.8, cCharacterRadiusStanding = cCharacterRadiusCrouching = 0.3) with their bottom at the
/// origin of the character
const TestShapes = struct {
    standing: RefConst(Shape),
    crouching: RefConst(Shape),

    fn init(allocator: Allocator) !TestShapes {
        var standing = RefConst(Shape).init(try capsule(allocator, 0.5 * 1.35, 0.3));
        errdefer standing.deinit();
        return .{ .standing = standing, .crouching = .init(try capsule(allocator, 0.5 * 0.8, 0.3)) };
    }

    fn deinit(self: *TestShapes) void {
        self.crouching.deinit();
        self.standing.deinit();
    }

    /// RotatedTranslatedShapeSettings(Vec3(0, half height + radius, 0), Quat::sIdentity(), new CapsuleShape(half height,
    /// radius)) (reference count 0)
    fn capsule(allocator: Allocator, half_height: f32, radius: f32) !*const Shape {
        const inner = try CapsuleShape.create(allocator, half_height, radius, .{});
        var inner_ref = RefConst(Shape).init(inner.asShape());
        defer inner_ref.deinit();
        const shape = try RotatedTranslatedShape.create(allocator, Vec3.init(0, half_height + radius, 0), Quat.identity(), inner.asShape());
        return shape.asShape();
    }
};

/// Heap settings of a test character in the moving layer (the caller puts them in a RefConst)
fn testSettings(allocator: Allocator, shape: *const Shape) !*CharacterSettings {
    const settings = try CharacterSettings.create(allocator);
    settings.base.shape.set(shape);
    settings.layer = TestLayers.moving;
    return settings;
}

/// A heap character from heap settings and the reference to it (`deinit` removes the character from the system before
/// the last release)
const TestCharacter = struct {
    character: *Character,
    ref: RefConst(CharacterBase),

    fn create(allocator: Allocator, world: *TestWorld, shape: *const Shape, position: RVec3, opts: struct { configure: ?*const fn (settings: *CharacterSettings) void = null, rotation: Quat = Quat.identity(), user_data: u64 = 0 }) !TestCharacter {
        const settings = try testSettings(allocator, shape);
        var settings_ref = RefConst(CharacterBaseSettings).init(&settings.base);
        defer settings_ref.deinit();
        if (opts.configure) |configure| configure(settings);
        const character = try Character.create(allocator, settings, position, opts.rotation, opts.user_data, &world.system);
        return .{ .character = character, .ref = .init(&character.base) };
    }

    fn deinit(self: *TestCharacter) void {
        self.character.removeFromPhysicsSystem(.{});
        self.ref.deinit();
    }
};

/// The ground of a character as a value (to compare the results of the lock variants)
const Ground = struct {
    state: CharacterBase.GroundState,
    body_id: BodyID,
    sub_shape_id: SubShapeID,
    position: RVec3,
    normal: Vec3,
    velocity: Vec3,
    material: *const PhysicsMaterial,
    user_data: u64,

    fn of(character: *const Character) Ground {
        const b = &character.base;
        return .{
            .state = b.getGroundState(),
            .body_id = b.getGroundBodyID(),
            .sub_shape_id = b.getGroundSubShapeID(),
            .position = b.getGroundPosition(),
            .normal = b.getGroundNormal(),
            .velocity = b.getGroundVelocity(),
            .material = b.getGroundMaterial(),
            .user_data = b.getGroundUserData(),
        };
    }

    fn expectEqual(expected: Ground, actual: Ground) !void {
        try testing.expectEqual(expected.state, actual.state);
        try testing.expect(expected.body_id.eql(actual.body_id));
        try testing.expect(expected.sub_shape_id.eql(actual.sub_shape_id));
        try testing.expect(expected.position.eql(actual.position));
        try testing.expect(expected.normal.eql(actual.normal));
        try testing.expect(expected.velocity.eql(actual.velocity));
        try testing.expect(expected.material == actual.material);
        try testing.expectEqual(expected.user_data, actual.user_data);
    }
};

/// PostSimulation with and without locking the bodies (both must give the same ground), returns the ground
fn postSimulationBothWays(character: *Character, max_separation_distance: f32) !Ground {
    character.postSimulation(max_separation_distance, .{});
    const locked = Ground.of(character);
    character.postSimulation(max_separation_distance, .{ .lock_bodies = false });
    try Ground.expectEqual(locked, Ground.of(character));
    return locked;
}

test "CharacterSettings: Jolt's defaults, copy, assignment, heap settings" {
    const allocator = testing.allocator;
    var settings: CharacterSettings = .init(allocator);
    defer settings.base.deinit();
    try testing.expectEqual(@as(ObjectLayer, 0), settings.layer);
    try testing.expectEqual(@as(f32, 80.0), settings.mass);
    try testing.expectEqual(@as(f32, 0.2), settings.friction);
    try testing.expectEqual(@as(f32, 1.0), settings.gravity_factor);
    try testing.expectEqual(@as(u8, 0b000111), settings.allowed_dofs.toInt()); // TranslationX | TranslationY | TranslationZ
    try testing.expect(settings.base.up.eql(Vec3.axisY()));
    try testing.expectEqual(math.degreesToRadians(50.0), settings.base.max_slope_angle);
    try testing.expect(!settings.base.enhanced_internal_edge_removal);
    try testing.expect(settings.base.shape.get() == null);
    try testing.expect(settings.base.vtable == CharacterBaseSettings.vtableFor(CharacterSettings));

    const sphere = try SphereShape.create(allocator, 0.5, .{});
    var shape_ref = RefConst(Shape).init(sphere.asShape());
    defer shape_ref.deinit();
    const shape = sphere.asShape();
    settings.base.shape.set(shape);
    settings.base.max_slope_angle = 0.3;
    settings.layer = 2;
    settings.mass = 60.0;
    settings.friction = 0.7;
    settings.gravity_factor = 0.25;
    settings.allowed_dofs = .all;
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());
    {
        // The copy takes a reference to the shape, not the reference count
        var copy: CharacterSettings = .initCopy(allocator, &settings);
        defer copy.base.deinit();
        try testing.expect(copy.base.vtable == CharacterBaseSettings.vtableFor(CharacterSettings));
        try testing.expectEqual(@as(u32, 0), copy.base.getRefCount());
        try testing.expectEqual(@as(u32, 3), shape.getRefCount());
        try testing.expect(copy.base.shape.get() == shape);
        try testing.expectEqual(@as(f32, 0.3), copy.base.max_slope_angle);
        try testing.expectEqual(@as(ObjectLayer, 2), copy.layer);
        try testing.expectEqual(@as(f32, 60.0), copy.mass);
        try testing.expectEqual(@as(f32, 0.7), copy.friction);
        try testing.expectEqual(@as(f32, 0.25), copy.gravity_factor);
        try testing.expect(copy.allowed_dofs.eql(.all));

        // Copy assignment
        var other: CharacterSettings = .init(allocator);
        defer other.base.deinit();
        other.assign(&copy);
        try testing.expectEqual(@as(u32, 4), shape.getRefCount());
        try testing.expectEqual(@as(f32, 0.3), other.base.max_slope_angle);
        try testing.expectEqual(@as(ObjectLayer, 2), other.layer);
        try testing.expectEqual(@as(f32, 60.0), other.mass);
        try testing.expectEqual(@as(f32, 0.7), other.friction);
        try testing.expectEqual(@as(f32, 0.25), other.gravity_factor);
        try testing.expect(other.allowed_dofs.eql(.all));
    }
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());

    // Heap settings are destroyed through the vtable by the last release (with their shape reference)
    const heap = try CharacterSettings.create(allocator);
    heap.base.shape.set(shape);
    var heap_ref = RefConst(CharacterBaseSettings).init(&heap.base);
    try testing.expectEqual(@as(u32, 1), heap.base.getRefCount());
    try testing.expectEqual(@as(u32, 3), shape.getRefCount());
    heap_ref.deinit();
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());
}

test "Character: the constructor creates the body, GetCharacterSettings, the destructor destroys the body" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const standing = shapes.standing.get().?;

    const settings = try testSettings(allocator, standing);
    var settings_ref = RefConst(CharacterBaseSettings).init(&settings.base);
    defer settings_ref.deinit();
    settings.mass = 70.0;
    settings.friction = 0.4;
    settings.gravity_factor = 0.5;
    settings.allowed_dofs = .{ .translation_x = true, .translation_y = true, .translation_z = true, .rotation_y = true };
    settings.base.max_slope_angle = math.degreesToRadians(45.0);
    settings.base.enhanced_internal_edge_removal = true;
    settings.base.up = Vec3.init(0, 0.6, 0.8);
    settings.base.supporting_volume = Plane.init(Vec3.axisY(), -0.3);

    const rotation = Quat.rotation(Vec3.axisY(), 0.3);
    const character = try Character.create(allocator, settings, RVec3.init(1, 2, 3), rotation, 42, &world.system);
    var character_ref = RefConst(CharacterBase).init(&character.base);
    var released = false;
    defer if (!released) character_ref.deinit();

    // CharacterBase got the settings
    try testing.expect(character.base.system == &world.system);
    try testing.expect(character.base.getShape().? == standing);
    try testing.expect(character.base.getUp().eql(Vec3.init(0, 0.6, 0.8)));
    try testing.expectEqual(@as(f32, -0.3), character.base.getSupportingVolume().getConstant());
    try testing.expectEqual(trigonometry.cos(math.degreesToRadians(45.0)), character.base.getCosMaxSlopeAngle());
    try testing.expectEqual(CharacterBase.GroundState.in_air, character.base.getGroundState());

    // The body: dynamic, not in the system, with the settings of the character
    const id = character.getBodyID();
    try testing.expect(!id.isInvalid());
    try testing.expectEqual(TestLayers.moving, character.getLayer());
    try testing.expectEqual(@as(u32, 1), world.system.getNumBodies());
    const bi = world.system.getBodyInterface();
    try testing.expect(!bi.isAdded(id));
    try testing.expect(!bi.isActive(id));
    {
        var lock: BodyLockRead = .init(world.system.getBodyLockInterface(), id);
        defer lock.deinit();
        try testing.expect(lock.succeeded());
        const body = lock.getBody();
        try testing.expect(body.isDynamic());
        try testing.expectEqual(@as(u64, 42), body.getUserData());
        try testing.expectEqual(TestLayers.moving, body.getObjectLayer());
        try testing.expect(body.getShape() == standing);
        try testing.expectEqual(@as(f32, 0.4), body.getFriction());
        try testing.expect(body.getEnhancedInternalEdgeRemoval());
        try testing.expect(!body.isSensor());
        const mp = body.getMotionProperties();
        try testing.expectEqual(@as(f32, 1.0) / @as(f32, 70.0), mp.getInverseMass());
        try testing.expectEqual(@as(f32, 0.5), mp.getGravityFactor());
        try testing.expect(mp.getAllowedDOFs().eql(settings.allowed_dofs));
        try testing.expect(body.getPosition().isClose(RVec3.init(1, 2, 3), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(body.getRotation().eql(rotation));
    }
    // References to the shape: TestShapes, the settings, the character, the body
    try testing.expectEqual(@as(u32, 4), standing.getRefCount());

    // GetCharacterSettings with and without locking the body
    for ([_]bool{ true, false }) |lock_bodies| {
        var s = character.getCharacterSettings(.{ .lock_bodies = lock_bodies });
        defer s.base.deinit();
        try testing.expect(s.base.vtable == CharacterBaseSettings.vtableFor(CharacterSettings));
        try testing.expect(s.base.up.eql(Vec3.init(0, 0.6, 0.8)));
        try testing.expect(s.base.supporting_volume.normal_and_constant.eql(settings.base.supporting_volume.normal_and_constant));
        try testing.expectEqual(trigonometry.acos(character.base.getCosMaxSlopeAngle()), s.base.max_slope_angle);
        try testing.expect(s.base.enhanced_internal_edge_removal);
        try testing.expect(s.base.shape.get() == standing);
        try testing.expectEqual(@as(u32, 5), standing.getRefCount());
        try testing.expectEqual(TestLayers.moving, s.layer);
        try testing.expectEqual(@as(f32, 1.0) / (@as(f32, 1.0) / @as(f32, 70.0)), s.mass);
        try testing.expectEqual(@as(f32, 0.4), s.friction);
        try testing.expectEqual(@as(f32, 0.5), s.gravity_factor);
        try testing.expect(s.allowed_dofs.eql(settings.allowed_dofs));
    }

    // The last release destroys the body and releases the references
    character_ref.deinit();
    released = true;
    try testing.expectEqual(@as(u32, 0), world.system.getNumBodies());
    try testing.expectEqual(@as(u32, 2), standing.getRefCount());
}

test "Character: an embedded character, SaveState / RestoreState are CharacterBase's" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    const floor = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0x55);

    var settings: CharacterSettings = .init(allocator);
    settings.base.setEmbedded();
    defer settings.base.deinit();
    settings.base.shape.set(shapes.standing.get());
    settings.layer = TestLayers.moving;
    var character: Character = try .init(allocator, &settings, RVec3.init(0.5, 0.01, -0.5), Quat.identity(), 0, &world.system);
    character.base.setEmbedded();
    defer character.base.deinit();
    try testing.expectEqual(@as(u32, 2), world.system.getNumBodies());
    try character.addToPhysicsSystem(.{});
    defer character.removeFromPhysicsSystem(.{});

    character.postSimulation(0.05, .{});
    try testing.expectEqual(CharacterBase.GroundState.on_ground, character.base.getGroundState());
    try testing.expect(character.base.getGroundBodyID().eql(floor));
    try testing.expectEqual(@as(u64, 0x55), character.base.getGroundUserData());
    const ground = Ground.of(&character);

    // SaveState is CharacterBase's: ground state, body ID, sub shape ID, position, normal, velocity
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    character.base.saveState(recorder.recorder());
    try testing.expect(!recorder.isFailed());
    try testing.expectEqual(@as(usize, 4 + 4 + 4 + 3 * @sizeOf(Real) + 12 + 12), recorder.getData().len);
    try testing.expectEqual(@as(i32, 0), std.mem.bytesToValue(i32, recorder.getData()[0..4]));
    try testing.expectEqual(floor.getIndexAndSequenceNumber(), std.mem.bytesToValue(u32, recorder.getData()[4..8]));

    // In the air, then restore: the ground is back, except the user data and the material that cannot be restored
    character.setPosition(RVec3.init(0.5, 3, -0.5), .{});
    character.postSimulation(0.05, .{});
    try testing.expectEqual(CharacterBase.GroundState.in_air, character.base.getGroundState());
    recorder.rewind();
    try character.base.restoreState(recorder.recorder());
    try testing.expect(!recorder.isFailed());
    var restored = ground;
    restored.user_data = 0;
    restored.material = PhysicsMaterial.default;
    try Ground.expectEqual(restored, Ground.of(&character));
}

test "Character: AddToPhysicsSystem, RemoveFromPhysicsSystem and Activate with and without locking the bodies" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 1, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    const id = character.getBodyID();
    const bi = world.system.getBodyInterface();

    for ([_]bool{ true, false }) |lock_bodies| {
        try character.addToPhysicsSystem(.{ .activation = .dont_activate, .lock_bodies = lock_bodies });
        try testing.expect(bi.isAdded(id));
        try testing.expect(!bi.isActive(id));
        character.activate(.{ .lock_bodies = lock_bodies });
        try testing.expect(bi.isActive(id));
        character.removeFromPhysicsSystem(.{ .lock_bodies = lock_bodies });
        try testing.expect(!bi.isAdded(id));
        try testing.expect(!bi.isActive(id));
        try character.addToPhysicsSystem(.{ .lock_bodies = lock_bodies });
        try testing.expect(bi.isAdded(id));
        try testing.expect(bi.isActive(id));
        character.removeFromPhysicsSystem(.{ .lock_bodies = lock_bodies });
        try testing.expect(!bi.isAdded(id));
    }
}

test "Character: velocities, impulses, positions, rotations and transforms with and without locking the bodies" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const standing = shapes.standing.get().?;
    const configure = struct {
        fn f(s: *CharacterSettings) void {
            s.mass = 50.0;
        }
    }.f;
    var tc: TestCharacter = try .create(allocator, &world, standing, RVec3.init(1, 2, 3), .{ .configure = configure });
    defer tc.deinit();
    const character = tc.character;
    const id = character.getBodyID();
    const bi = world.system.getBodyInterfaceMut();
    const com = standing.getCenterOfMass();
    try testing.expect(com.isClose(Vec3.init(0, 0.975, 0), .{}));

    for ([_]bool{ true, false }) |lock_bodies| {
        const lock: Character.LockOptions = .{ .lock_bodies = lock_bodies };
        character.removeFromPhysicsSystem(lock);
        try character.addToPhysicsSystem(.{ .activation = .dont_activate, .lock_bodies = lock_bodies });

        // Velocities (the setters wake the body up)
        character.setLinearAndAngularVelocity(Vec3.init(1, 0, -1), Vec3.init(0, 2, 0), lock);
        try testing.expect(bi.isActive(id));
        try testing.expect(character.getLinearVelocity(lock).eql(Vec3.init(1, 0, -1)));
        try testing.expect(bi.getAngularVelocity(id).eql(Vec3.zero())); // The rotation degrees of freedom are not allowed
        character.setLinearVelocity(Vec3.init(2, 0, 1), lock);
        try testing.expect(character.getLinearVelocity(lock).eql(Vec3.init(2, 0, 1)));
        character.addLinearVelocity(Vec3.init(0, 1, 0), lock);
        try testing.expect(character.getLinearVelocity(lock).eql(Vec3.init(2, 1, 1)));
        character.addImpulse(Vec3.init(0, 50, 0), lock); // 1 m/s for a mass of 50 kg
        try testing.expect(character.getLinearVelocity(lock).isClose(Vec3.init(2, 2, 1), .{}));

        // Position and rotation
        const rotation = Quat.rotation(Vec3.axisY(), 0.5);
        bi.deactivateBody(id); // (also resets the velocity)
        character.setPositionAndRotation(RVec3.init(-1, 4, 2), rotation, .{ .activation = .dont_activate, .lock_bodies = lock_bodies });
        try testing.expect(!bi.isActive(id));
        const pr = character.getPositionAndRotation(lock);
        try testing.expect(pr.position.isClose(RVec3.init(-1, 4, 2), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(pr.rotation.eql(rotation));
        try testing.expect(character.getPosition(lock).eql(pr.position));
        try testing.expect(character.getRotation(lock).eql(rotation));
        character.setPosition(RVec3.init(3, 1, 0), .{ .lock_bodies = lock_bodies });
        try testing.expect(bi.isActive(id));
        try testing.expect(character.getPosition(lock).isClose(RVec3.init(3, 1, 0), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(character.getRotation(lock).eql(rotation));
        const rotation2 = Quat.rotation(Vec3.axisY(), -1.0);
        character.setRotation(rotation2, .{ .activation = .dont_activate, .lock_bodies = lock_bodies });
        try testing.expect(character.getRotation(lock).eql(rotation2));

        // The center of mass is the center of the capsule
        const com_position = character.getCenterOfMassPosition(lock);
        try testing.expect(com_position.isClose(character.getPosition(lock).addVec3(rotation2.mulVec3(com)), .{ .max_dist_sq = 1.0e-10 }));
        const transform = character.getWorldTransform(lock);
        try testing.expect(transform.getTranslation().isClose(character.getPosition(lock), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(transform.getRotation().isClose(Mat44.rotationQuat(rotation2), .{}));

        // The transformed shape of the body
        var ts = character.getTransformedShape(lock);
        defer ts.deinit();
        try testing.expect(ts.shape.get() == standing);
        try testing.expect(ts.body_id.eql(id));
        try testing.expect(ts.shape_position_com.eql(com_position));
        try testing.expect(ts.shape_rotation.eql(rotation2));
        try testing.expect(ts.getShapeScale().eql(Vec3.one()));
    }
}

test "Character: the four ground states on boxes, the most up aligned hit, sensors and the own body are ignored" {
    const allocator = testing.allocator;
    var floor_material = try PhysicsMaterialSimple.init(allocator, "Floor", Color.green); // Outlives the bodies
    floor_material.base.setEmbedded();
    defer floor_material.base.deinit();
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();

    // A floor (top at y = 0) with a material, a wall at x = 3 and a sensor box around the start position
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{ .material = floor_material.material() });
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    const floor = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0x1234);
    const wall_shape = try BoxShape.create(allocator, Vec3.init(0.5, 3, 10), .{});
    var wall_ref = RefConst(Shape).init(wall_shape.asShape());
    defer wall_ref.deinit();
    const wall = try world.addBody(wall_shape.asShape(), RVec3.init(3.5, 3, 0), Quat.identity(), .static, 0x5678);
    const sensor_shape = try BoxShape.create(allocator, Vec3.init(1, 1, 1), .{});
    var sensor_ref = RefConst(Shape).init(sensor_shape.asShape());
    defer sensor_ref.deinit();
    {
        var sensor: BodyCreationSettings = .initWithShape(sensor_shape.asShape(), RVec3.init(0, 0.5, 0), Quat.identity(), .static, TestLayers.non_moving);
        defer sensor.deinit();
        sensor.is_sensor = true;
        _ = try world.system.getBodyInterfaceMut().createAndAddBody(&sensor, .dont_activate);
    }

    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 0, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{});

    // Standing on the floor: OnGround, the floor's properties (the sensor and the own body are not the ground)
    var ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    try testing.expect(character.base.isSupported());
    try testing.expect(ground.body_id.eql(floor));
    try testing.expect(ground.normal.isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(ground.position.isClose(RVec3.zero(), .{ .max_dist_sq = 1.0e-6 }));
    try testing.expect(ground.velocity.eql(Vec3.zero()));
    try testing.expect(ground.material == floor_material.material());
    try testing.expectEqual(@as(u64, 0x1234), ground.user_data);

    // A too steep ground: through the up vector, then through the max slope angle
    character.base.setUp(Vec3.init(0.8, 0.6, 0));
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_steep_ground, ground.state);
    try testing.expect(character.base.isSupported());
    character.base.setUp(Vec3.axisY());
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    character.base.setMaxSlopeAngle(math.degreesToRadians(30.0));
    character.base.setUp(Vec3.init(0.6, 0.8, 0)); // 36.9 degrees from the floor normal
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_steep_ground, ground.state);
    character.base.setMaxSlopeAngle(0.0); // Turns the slope check off
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    character.base.setUp(Vec3.axisY());
    character.base.setMaxSlopeAngle(math.degreesToRadians(50.0));

    // The contact point in front of the supporting volume: NotSupported (the floor stays the ground body)
    character.base.setSupportingVolume(Plane.init(Vec3.axisY(), 0.5));
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.not_supported, ground.state);
    try testing.expect(!character.base.isSupported());
    try testing.expect(ground.body_id.eql(floor));
    try testing.expect(ground.material == floor_material.material());

    // CharacterTest's supporting volume: standing on the floor is supported, touching only the wall in the air is not
    character.base.setSupportingVolume(Plane.init(Vec3.axisY(), -0.3));
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    character.setPosition(RVec3.init(2.69, 1.5, 0), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.not_supported, ground.state);
    try testing.expect(ground.body_id.eql(wall));
    try testing.expect(ground.normal.isClose(Vec3.init(-1, 0, 0), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(@as(u64, 0x5678), ground.user_data);
    try testing.expect(ground.material == PhysicsMaterial.default);

    // On the floor next to the wall: both are hit, the floor is the most up aligned
    character.setPosition(RVec3.init(2.69, 0.0, 0), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    try testing.expect(ground.body_id.eql(floor));

    // Too far from everything: InAir, the ground is reset
    character.setPosition(RVec3.init(0, 3, 0), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.in_air, ground.state);
    try testing.expect(!character.base.isSupported());
    try testing.expect(ground.body_id.isInvalid());
    try testing.expect(ground.sub_shape_id.eql(SubShapeID.empty));
    try testing.expect(ground.position.eql(RVec3.zero()));
    try testing.expect(ground.normal.eql(Vec3.zero()));
    try testing.expect(ground.velocity.eql(Vec3.zero()));
    try testing.expect(ground.material == PhysicsMaterial.default);
    try testing.expectEqual(@as(u64, 0), ground.user_data);

    // Within the max separation distance of the floor: on the ground again
    character.setPosition(RVec3.init(0, 0.04, 0), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    ground = try postSimulationBothWays(character, 0.03);
    try testing.expectEqual(CharacterBase.GroundState.in_air, ground.state);
}

test "Character: ground states on slopes" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const slope_shape = try BoxShape.create(allocator, Vec3.init(5, 0.5, 5), .{});
    var slope_ref = RefConst(Shape).init(slope_shape.asShape());
    defer slope_ref.deinit();

    // A box tilted around Z, the character just above the center of its top face (the bottom sphere of the capsule
    // touches the face, nothing else does); the max slope angle is Jolt's default of 50 degrees
    for ([_]f32{ 10.0, 30.0, 45.0, 60.0, 75.0 }, [_]CharacterBase.GroundState{ .on_ground, .on_ground, .on_ground, .on_steep_ground, .on_steep_ground }, 0..) |degrees, expected, i| {
        const offset = RVec3.init(@floatFromInt(20 * i), 0, 0);
        const rotation = Quat.rotation(Vec3.init(0, 0, 1), math.degreesToRadians(degrees));
        const slope = try world.addBody(slope_shape.asShape(), offset, rotation, .static, i);
        const normal = rotation.mulVec3(Vec3.axisY());
        const position = offset.addVec3(normal.mulScalar(0.81).sub(Vec3.init(0, 0.3, 0)));
        var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, position, .{});
        defer tc.deinit();
        const character = tc.character;
        try character.addToPhysicsSystem(.{});
        const ground = try postSimulationBothWays(character, 0.05);
        try testing.expectEqual(expected, ground.state);
        try testing.expect(ground.body_id.eql(slope));
        try testing.expect(ground.normal.isClose(normal, .{ .max_dist_sq = 1.0e-8 }));
        try testing.expectEqual(@as(u64, i), ground.user_data);
    }
}

test "Character: ground on a mesh, the material and the sub shape of the triangle" {
    const allocator = testing.allocator;
    var grass = try PhysicsMaterialSimple.init(allocator, "Grass", Color.green); // The materials outlive the bodies
    grass.base.setEmbedded();
    defer grass.base.deinit();
    var rock = try PhysicsMaterialSimple.init(allocator, "Rock", Color.grey);
    rock.base.setEmbedded();
    defer rock.base.deinit();
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();

    // A flat quad of two triangles with their own materials, and a ramp of 60 degrees from x = 5
    const h: f32 = 5.0 * 1.7320508; // tan(60 degrees) * 5
    const triangles = [_]Triangle{
        .init(Vec3.init(-5, 0, -5), Vec3.init(-5, 0, 5), Vec3.init(5, 0, 5), .{ .material_index = 0 }),
        .init(Vec3.init(-5, 0, -5), Vec3.init(5, 0, 5), Vec3.init(5, 0, -5), .{ .material_index = 1 }),
        .init(Vec3.init(5, 0, -5), Vec3.init(5, 0, 5), Vec3.init(10, h, 5), .{ .material_index = 1 }),
        .init(Vec3.init(5, 0, -5), Vec3.init(10, h, 5), Vec3.init(10, h, -5), .{ .material_index = 1 }),
    };
    const mesh_settings = try MeshShapeSettings.create(allocator, &triangles, .{ .materials = &.{ grass.material(), rock.material() } });
    var mesh_settings_ref = Ref(ShapeSettings).init(mesh_settings.asShapeSettings());
    defer mesh_settings_ref.deinit();
    var mesh_result = try mesh_settings.asShapeSettings().createShape(allocator);
    defer mesh_result.deinit();
    try testing.expect(mesh_result.isValid());
    const mesh_shape = mesh_result.getPtr().?;
    const mesh = try world.addBody(mesh_shape, RVec3.zero(), Quat.identity(), .static, 0xabc);

    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(-3, 0, 2), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{});

    // On the first triangle (grass)
    var ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    try testing.expect(ground.body_id.eql(mesh));
    try testing.expect(ground.material == grass.material());
    try testing.expect(mesh_shape.getMaterial(ground.sub_shape_id) == grass.material());
    try testing.expect(ground.normal.isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(@as(u64, 0xabc), ground.user_data);
    const first_triangle = ground.sub_shape_id;

    // On the second triangle (rock)
    character.setPosition(RVec3.init(3, 0, -2), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    try testing.expect(ground.material == rock.material());
    try testing.expect(!ground.sub_shape_id.eql(first_triangle));

    // On the ramp: too steep
    const ramp_normal = Vec3.init(-h, 5, 0).normalized();
    const on_ramp = Vec3.init(7.5, 0.5 * h, 0).add(ramp_normal.mulScalar(0.31)).sub(Vec3.init(0, 0.3, 0));
    character.setPosition(RVec3.init(on_ramp.getX(), on_ramp.getY(), on_ramp.getZ()), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_steep_ground, ground.state);
    try testing.expect(ground.normal.isClose(ramp_normal, .{ .max_dist_sq = 1.0e-8 }));
    try testing.expect(ground.material == rock.material());

    // Below the mesh: back faces are ignored, in the air
    character.setPosition(RVec3.init(-3, -1.96, 2), .{});
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.in_air, ground.state);
}

test "Character: standing on moving and rotating bodies" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();

    // A kinematic platform (top at y = 0) that moves and rotates
    const platform_shape = try BoxShape.create(allocator, Vec3.init(3, 0.25, 3), .{});
    var platform_ref = RefConst(Shape).init(platform_shape.asShape());
    defer platform_ref.deinit();
    const platform = try world.addBody(platform_shape.asShape(), RVec3.init(0, -0.25, 0), Quat.identity(), .kinematic, 77);
    const bi = world.system.getBodyInterfaceMut();
    bi.setLinearAndAngularVelocity(platform, Vec3.init(1, 0, 0.5), Vec3.zero());

    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(2, 0, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{});

    // Moving: the ground velocity is the velocity of the platform
    var ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    try testing.expect(ground.body_id.eql(platform));
    try testing.expect(ground.velocity.eql(Vec3.init(1, 0, 0.5)));
    try testing.expectEqual(@as(u64, 77), ground.user_data);

    // Rotating: the velocity of the platform at the contact point
    bi.setLinearAndAngularVelocity(platform, Vec3.zero(), Vec3.init(0, 1, 0));
    ground = try postSimulationBothWays(character, 0.05);
    try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
    try testing.expect(ground.velocity.eql(bi.getPointVelocity(platform, ground.position)));
    try testing.expect(ground.velocity.isClose(Vec3.init(0, 0, -2), .{ .max_dist_sq = 1.0e-6 }));

    // Simulated: the character is carried along (friction) and keeps standing on the platform
    for (0..30) |_| {
        try world.update();
        ground = try postSimulationBothWays(character, 0.05);
        try testing.expectEqual(CharacterBase.GroundState.on_ground, ground.state);
        try testing.expect(ground.body_id.eql(platform));
    }
    try testing.expect(character.getPosition(.{}).getZ() < -0.1);
}

test "Character: a simulated walk: falls, lands, walks, jumps, lands again" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const floor_shape = try BoxShape.create(allocator, Vec3.init(50, 0.5, 50), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    const floor = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0);

    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 0.5, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{});

    // Falls and lands
    var landed_after: ?usize = null;
    for (0..60) |i| {
        try world.update();
        character.postSimulation(0.05, .{});
        if (landed_after == null and character.base.getGroundState() == .on_ground) landed_after = i;
    }
    try testing.expect(landed_after != null and landed_after.? > 5);
    try testing.expect(character.base.getGroundBodyID().eql(floor));
    try testing.expect(@abs(character.getPosition(.{}).getY()) < 0.02);

    // Walks (CharacterTest: the horizontal velocity is set every frame, the vertical one is kept)
    for (0..60) |_| {
        const velocity = character.getLinearVelocity(.{});
        character.setLinearVelocity(Vec3.init(2, velocity.getY(), 0), .{});
        try world.update();
        character.postSimulation(0.05, .{});
        try testing.expectEqual(CharacterBase.GroundState.on_ground, character.base.getGroundState());
    }
    try testing.expect(character.getPosition(.{}).getX() > 1.5);

    // Jumps: in the air, then on the ground again
    character.addLinearVelocity(Vec3.init(0, 4, 0), .{ .lock_bodies = false });
    var was_in_air = false;
    var landed_again = false;
    for (0..120) |_| {
        try world.update();
        character.postSimulation(0.05, .{ .lock_bodies = false });
        if (character.base.getGroundState() == .in_air) was_in_air = true;
        if (was_in_air and character.base.getGroundState() == .on_ground) landed_again = true;
    }
    try testing.expect(was_in_air and landed_again);
}

test "Character: SetShape with and without max penetration depth (crouch / stand under a ceiling)" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const standing = shapes.standing.get().?;
    const crouching = shapes.crouching.get().?;

    // A floor and a ceiling whose bottom is at y = 1.5 over x < 0: the crouching capsule (1.4 m) fits below it, the
    // standing one (1.95 m) penetrates it by 0.45 m
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    _ = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0);
    const ceiling_shape = try BoxShape.create(allocator, Vec3.init(5, 0.5, 5), .{});
    var ceiling_ref = RefConst(Shape).init(ceiling_shape.asShape());
    defer ceiling_ref.deinit();
    _ = try world.addBody(ceiling_shape.asShape(), RVec3.init(-5, 2, 0), Quat.identity(), .static, 0);

    var tc: TestCharacter = try .create(allocator, &world, crouching, RVec3.init(-2, 0, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{ .activation = .dont_activate });
    const id = character.getBodyID();
    const bi = world.system.getBodyInterfaceMut();
    const position = character.getPosition(.{});

    for ([_]bool{ true, false }) |lock_bodies| {
        const lock: Character.LockOptions = .{ .lock_bodies = lock_bodies };

        // Standing up collides with the ceiling: rejected, nothing changes
        try testing.expect(!try character.setShape(standing, 0.1, lock));
        try testing.expect(character.base.getShape().? == crouching);
        var body_shape = bi.getShape(id);
        try testing.expect(body_shape.get() == crouching);
        body_shape.deinit();
        try testing.expect(!bi.isActive(id));

        // A penetration of 0.45 is allowed with a max penetration depth of 0.5
        try testing.expect(try character.setShape(standing, 0.5, lock));
        try testing.expect(character.base.getShape().? == standing);
        body_shape = bi.getShape(id);
        try testing.expect(body_shape.get() == standing);
        body_shape.deinit();
        try testing.expect(bi.isActive(id)); // SetShape activates the body
        try testing.expect(character.getPosition(.{}).isClose(position, .{ .max_dist_sq = 1.0e-10 })); // The body origin stays in place

        // Without max penetration depth the shape is switched without a check
        try testing.expect(try character.setShape(crouching, math.flt_max, lock));
        try testing.expect(character.base.getShape().? == crouching);
        try testing.expect(try character.setShape(standing, math.flt_max, lock));
        try testing.expect(character.base.getShape().? == standing);
        try testing.expect(try character.setShape(crouching, 0.0, lock)); // Crouching always fits
        bi.deactivateBody(id);
    }

    // Away from the ceiling standing up succeeds with any max penetration depth (the floor contact has depth 0)
    character.setPosition(RVec3.init(6, 0, 0), .{});
    try testing.expect(try character.setShape(standing, 0.0, .{}));
    try testing.expect(character.base.getShape().? == standing);

    // The references: TestShapes, the character and the body hold the standing capsule (the settings are released)
    try testing.expectEqual(@as(u32, 3), standing.getRefCount());
    try testing.expectEqual(@as(u32, 1), crouching.getRefCount());
}

test "Character: SetLayer with and without locking the bodies" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    _ = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0);

    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 0, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{});
    const id = character.getBodyID();
    const bi = world.system.getBodyInterface();

    for ([_]bool{ true, false }) |lock_bodies| {
        const lock: Character.LockOptions = .{ .lock_bodies = lock_bodies };
        character.postSimulation(0.05, lock);
        try testing.expectEqual(CharacterBase.GroundState.on_ground, character.base.getGroundState());

        // The ghost layer collides with nothing: the query of PostSimulation uses the layer of the character
        try character.setLayer(TestLayers.ghost, lock);
        try testing.expectEqual(TestLayers.ghost, character.getLayer());
        try testing.expectEqual(TestLayers.ghost, bi.getObjectLayer(id));
        character.postSimulation(0.05, lock);
        try testing.expectEqual(CharacterBase.GroundState.in_air, character.base.getGroundState());

        try character.setLayer(TestLayers.moving, lock);
        try testing.expectEqual(TestLayers.moving, character.getLayer());
        try testing.expectEqual(TestLayers.moving, bi.getObjectLayer(id));
        character.postSimulation(0.05, lock);
        try testing.expectEqual(CharacterBase.GroundState.on_ground, character.base.getGroundState());
    }

    // In the ghost layer the character falls through the floor
    try character.setLayer(TestLayers.ghost, .{});
    for (0..30) |_| try world.update();
    try testing.expect(character.getPosition(.{}).getY() < -0.5);
}

test "Character: CheckCollision, the shape at a position with a base offset" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    const floor = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0);
    const crate_shape = try BoxShape.create(allocator, Vec3.init(0.5, 0.5, 0.5), .{});
    var crate_ref = RefConst(Shape).init(crate_shape.asShape());
    defer crate_ref.deinit();
    const crate = try world.addBody(crate_shape.asShape(), RVec3.init(5, 0.5, 0), Quat.identity(), .dynamic, 0);
    {
        var sensor: BodyCreationSettings = .initWithShape(crate_shape.asShape(), RVec3.init(5, 1.5, 0), Quat.identity(), .static, TestLayers.non_moving);
        defer sensor.deinit();
        sensor.is_sensor = true;
        _ = try world.system.getBodyInterfaceMut().createAndAddBody(&sensor, .dont_activate);
    }

    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 0, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try character.addToPhysicsSystem(.{});

    for ([_]bool{ true, false }) |lock_bodies| {
        // At the character's position: the floor only (not its own body)
        var collector: AllHitCollisionCollector(CollideShapeCollector) = .init(allocator);
        defer collector.deinit();
        character.checkCollision(RVec3.init(0, 0, 0), Quat.identity(), Vec3.zero(), 0.05, shapes.standing.get().?, RVec3.zero(), &collector.base, .{ .lock_bodies = lock_bodies });
        try collector.checkError();
        try testing.expect(collector.hits.items.len > 0);
        for (collector.hits.items) |hit| try testing.expect(hit.body_id2.eql(floor));

        // Next to the crate (and in the sensor above it): the floor and the crate, relative to the base offset
        collector.reset();
        const base_offset = RVec3.init(4, 0, 0);
        character.checkCollision(RVec3.init(4.4, 0, 0), Quat.identity(), Vec3.init(1, 0, 0), 0.05, shapes.crouching.get().?, base_offset, &collector.base, .{ .lock_bodies = lock_bodies });
        try collector.checkError();
        var found_crate = false;
        for (collector.hits.items) |hit| {
            try testing.expect(hit.body_id2.eql(floor) or hit.body_id2.eql(crate));
            if (hit.body_id2.eql(crate)) {
                found_crate = true;
                // The contact point on the crate's side x = 4.5 (relative to the base offset)
                try testing.expect(@abs(hit.contact_point_on2.getX() - 0.5) < 1.0e-3);
            }
        }
        try testing.expect(found_crate);
    }
}

test "Character: out of bodies, the character has no body" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 1);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    _ = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0);

    // The system is full: the character exists without a body (Jolt's behavior)
    var tc: TestCharacter = try .create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 0, 0), .{});
    defer tc.deinit();
    const character = tc.character;
    try testing.expect(character.getBodyID().isInvalid());
    try testing.expectEqual(@as(u32, 1), world.system.getNumBodies());

    // Every body operation does nothing, the getters return the defaults of the BodyInterface
    try character.addToPhysicsSystem(.{});
    character.activate(.{});
    character.setLinearAndAngularVelocity(Vec3.init(1, 0, 0), Vec3.zero(), .{});
    character.setLinearVelocity(Vec3.init(1, 0, 0), .{});
    character.addLinearVelocity(Vec3.init(1, 0, 0), .{});
    character.addImpulse(Vec3.init(1, 0, 0), .{});
    character.setPositionAndRotation(RVec3.init(1, 0, 0), Quat.identity(), .{});
    character.setPosition(RVec3.init(1, 0, 0), .{});
    character.setRotation(Quat.identity(), .{});
    try testing.expect(character.getLinearVelocity(.{}).eql(Vec3.zero()));
    try testing.expect(character.getPosition(.{}).eql(RVec3.zero()));
    try testing.expect(character.getRotation(.{}).eql(Quat.identity()));
    try testing.expect(character.getCenterOfMassPosition(.{}).eql(RVec3.zero()));
    var ts = character.getTransformedShape(.{});
    defer ts.deinit();
    try testing.expect(ts.shape.get() == null);

    // PostSimulation returns early: the ground state stays InAir
    character.postSimulation(0.05, .{});
    try testing.expectEqual(CharacterBase.GroundState.in_air, character.base.getGroundState());

    // The layer and the shape of the character change (there is no body to check or to change)
    try character.setLayer(TestLayers.ghost, .{});
    try testing.expectEqual(TestLayers.ghost, character.getLayer());
    try testing.expect(try character.setShape(shapes.crouching.get().?, 0.0, .{}));
    try testing.expect(character.base.getShape().? == shapes.crouching.get().?);
    character.removeFromPhysicsSystem(.{});

    // CheckCollision at a position works without a body (the ghost layer collides with nothing)
    var collector: AllHitCollisionCollector(CollideShapeCollector) = .init(allocator);
    defer collector.deinit();
    character.checkCollision(RVec3.zero(), Quat.identity(), Vec3.zero(), 0.05, shapes.standing.get().?, RVec3.zero(), &collector.base, .{});
    try collector.checkError();
    try testing.expectEqual(@as(usize, 0), collector.hits.items.len);
    try character.setLayer(TestLayers.moving, .{});
    character.checkCollision(RVec3.zero(), Quat.identity(), Vec3.zero(), 0.05, shapes.standing.get().?, RVec3.zero(), &collector.base, .{});
    try collector.checkError();
    try testing.expect(collector.hits.items.len > 0);
}

test "Character: settings without a shape (Jolt crashes in CreateBody) return error.InvalidShape and leave nothing" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 16);
    defer world.deinit();
    const settings = try CharacterSettings.create(allocator);
    var settings_ref = RefConst(CharacterBaseSettings).init(&settings.base);
    defer settings_ref.deinit();
    try testing.expectError(error.InvalidShape, Character.create(allocator, settings, RVec3.zero(), Quat.identity(), 0, &world.system));
    try testing.expectEqual(@as(u32, 0), world.system.getNumBodies());
}

/// Everything that allocates: the creation of the character (with the world), adding it, the layer changes (moving and
/// ghost are in different broad phase layers: BroadPhaseQuadTree.notifyBodiesLayerChanged allocates to move the body to
/// the other tree), shape switches, updates, GetCharacterSettings, the removal and the destruction
fn characterAllocationRun(allocator: Allocator) !void {
    var world: TestWorld = undefined;
    try world.init(allocator, 8);
    defer world.deinit();
    var shapes: TestShapes = try .init(allocator);
    defer shapes.deinit();
    const floor_shape = try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{});
    var floor_ref = RefConst(Shape).init(floor_shape.asShape());
    defer floor_ref.deinit();
    _ = try world.addBody(floor_shape.asShape(), RVec3.init(0, -0.5, 0), Quat.identity(), .static, 0);
    const crate_shape = try BoxShape.create(allocator, Vec3.init(0.25, 0.25, 0.25), .{});
    var crate_ref = RefConst(Shape).init(crate_shape.asShape());
    defer crate_ref.deinit();
    _ = try world.addBody(crate_shape.asShape(), RVec3.init(0.8, 0.25, 0), Quat.identity(), .dynamic, 0);

    const num_bodies = world.system.getNumBodies();
    {
        var tc: TestCharacter = TestCharacter.create(allocator, &world, shapes.standing.get().?, RVec3.init(0, 0.01, 0), .{}) catch |err| {
            // Nothing is left of a failed creation
            try testing.expectEqual(num_bodies, world.system.getNumBodies());
            return err;
        };
        defer tc.deinit();
        const character = tc.character;
        const id = character.getBodyID();
        const bi = world.system.getBodyInterface();

        character.addToPhysicsSystem(.{}) catch |err| {
            try testing.expect(!bi.isAdded(id));
            return err;
        };
        character.setLayer(TestLayers.ghost, .{}) catch |err| {
            // Nothing changed (the character's layer is restored, Zolt's errdefer)
            try testing.expectEqual(TestLayers.moving, character.getLayer());
            try testing.expectEqual(TestLayers.moving, bi.getObjectLayer(id));
            return err;
        };
        try testing.expectEqual(TestLayers.ghost, bi.getObjectLayer(id));
        character.setLayer(TestLayers.moving, .{ .lock_bodies = false }) catch |err| {
            try testing.expectEqual(TestLayers.ghost, character.getLayer());
            try testing.expectEqual(TestLayers.ghost, bi.getObjectLayer(id));
            return err;
        };
        _ = character.setShape(shapes.crouching.get().?, 0.1, .{}) catch |err| {
            // Nothing changed
            try testing.expect(character.base.getShape().? == shapes.standing.get().?);
            var body_shape = bi.getShape(id);
            defer body_shape.deinit();
            try testing.expect(body_shape.get() == shapes.standing.get().?);
            return err;
        };
        character.setLinearVelocity(Vec3.init(1, 0, 0), .{});
        for (0..3) |_| {
            try world.update();
            character.postSimulation(0.05, .{});
        }
        _ = try character.setShape(shapes.standing.get().?, math.flt_max, .{ .lock_bodies = false });
        var settings = character.getCharacterSettings(.{});
        defer settings.base.deinit();
        try testing.expect(settings.base.shape.get() == shapes.standing.get().?);
    }
    try testing.expectEqual(num_bodies, world.system.getNumBodies());
}

test "Character: allocation failure leaves nothing behind" {
    try characterAllocationRun(testing.allocator);
    try testing.checkAllAllocationFailures(testing.allocator, characterAllocationRun, .{});
}
