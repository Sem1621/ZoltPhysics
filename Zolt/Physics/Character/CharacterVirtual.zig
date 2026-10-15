//! Port of: Jolt/Physics/Character/CharacterVirtual.h, Jolt/Physics/Character/CharacterVirtual.cpp
//! Status: complete
//! Not ported: JPH_DEBUG_RENDERER (sDrawConstraints, sDrawWalkStairs, sDrawStickToFloor, sDrawSupportingVolume and the
//! drawing in Update, MoveShape, SolveConstraints, WalkStairs and StickToFloor: TODO(debug renderer))
//!
//! Docs/Zolt/CharacterVehicleArchitecture.md E2 / E5:
//! - `CharacterVirtualSettings` (a CharacterBaseSettings) and `CharacterVirtual` (a CharacterBase) are pattern A levels
//!   under the roots of CharacterBase.zig: `base` is the parent, `overrides` lists the virtual functions (SaveState /
//!   RestoreState). `new CharacterVirtualSettings()` is `CharacterVirtualSettings.create(allocator)`, a stack or member
//!   object `init(allocator)`; both consume a CharacterID like Jolt's default member initializer `mID =
//!   CharacterID::sNextCharacterID()`, the copy (`initCopy`, `assign`) does not. `GetCharacterVirtualSettings` returns a
//!   value that consumes one too (the caller `deinit`s it: it holds references to the shapes).
//! - `new CharacterVirtual(settings, position, rotation, user data, system)` is `CharacterVirtual.create(...)` (reference
//!   count 0 like `new`, the last `release` of a `Ref(CharacterBase)` destroys it), an embedded character `init(...)`.
//!   Jolt's constructor without user data is the same call with `user_data = 0`. A failed creation (out of memory while
//!   creating or adding the inner body) leaves nothing behind: no inner body, no references.
//! - `CharacterContactListener` and `CharacterVsCharacterCollision` are pattern B interfaces (`init(&impl)`): every
//!   listener callback is optional (Jolt's defaults do nothing / return true), the two collision functions are required.
//!   `CharacterVsCharacterCollisionSimple` keeps Jolt's `Array<CharacterVirtual *>` order and passes the other character
//!   to the collector through `setUserData(@intFromPtr(character))`.
//! - `CharacterContact : CharacterContactKey` embeds the key as `base`. Both are extern structs with Jolt's layout (the
//!   contact lists live in temp memory, so the temp allocator sees Jolt's sizes); every member has a default (Jolt leaves
//!   some uninitialized, the code always writes them before they are read).
//! - The contacts, constraints and ignored contacts of Jolt's `Array<T, STLTempAllocator<T>>` are `TempList(T)`: one block
//!   of the caller's TempAllocator, allocated where Jolt reserves (`reserve(0)` allocates nothing, like Jolt) and freed at
//!   the end of the C++ scope (LIFO, also on the error paths).
//! - The listener bookkeeping `ListenerContacts` is Jolt's `UnorderedMap<CharacterContactKey, ListenerContactValue>`, the
//!   ported HashTable: FinishTrackingContactChanges calls the removal callbacks in its iteration order, which depends on
//!   the hash of the keys and the bucket count, exactly like Jolt.
//! - Queries: the four filters of Jolt's functions stay positional; the shape filter is `?*ShapeFilter` (null is Jolt's
//!   `{ }` default filter) because the NarrowPhaseQuery writes its `mBodyID2` (Rule M, CollisionArchitecture D7).
//!   `TempAllocator &inAllocator` is the `temp_allocator` value.
//! - Allocation failure (Zolt): every function that allocates returns `Allocator.Error`: the creation, Update,
//!   ExtendedUpdate, StickToFloor, WalkStairs, RefreshContacts, SetShape, CheckCollision (the heap fallback of the
//!   InternalEdgeRemovingCollector with enhanced internal edge removal uses the character's allocator, Jolt's global
//!   heap), StartTrackingContactChanges, SetInnerBodyShape, RestoreState and CharacterVsCharacterCollisionSimple.add. The
//!   character stays consistent after an error: the temp memory is freed in LIFO order, StartTrackingContactChanges /
//!   FinishTrackingContactChanges stay paired (FinishTrackingContactChanges runs on the error path like Jolt's
//!   JPH_SCOPE_EXIT), the active contacts are replaced atomically, and a contact is entered in the listener bookkeeping
//!   before its OnContactAdded callback (the listener cannot observe the order), so that a failure happens before the
//!   callback. The position and contacts are then partially updated (as far as the operation got); restore a saved state
//!   or repeat the operation. CharacterBase's virtual RestoreState returns Allocator.Error for this class (it resizes the
//!   active contacts).
//! - Rule M: Jolt's `mutable mMaxHitsExceeded` is written by GetContactsAtPosition, whose callers are all non-const: it
//!   takes a mutable receiver.
//! - Jolt behavior reproduced on purpose: RestoreState does not restore mIsBackFacingContact, so a non-validating restore
//!   keeps the flag of the contact that was at the same index before (new contacts get false); GetCharacterVirtualSettings
//!   consumes a CharacterID; a new listener contact that is added a second time in one tracking session gets an
//!   OnContactPersisted callback; the SolveConstraints sort tie breaker orders by descending motion type (its comment
//!   says static first); MoveToContact matches contacts by body and sub shape only (two characters with the same sub
//!   shape ID match); UpdateSupportingContact's support test solves with a delta time of 1, so it can apply impulses.
//! - A restored EMotionType byte that is not a valid motion type becomes Static (Jolt keeps the invalid value; a stream
//!   from SaveState never holds one). Jolt asserts that are reachable with user input (an invalid CharacterID, a listener
//!   changed between StartTrackingContactChanges and FinishTrackingContactChanges, more Finish than Start calls, a
//!   character shape that sCorrectFractionForCharacterPadding does not support) only assert with asserts enabled; the
//!   release behavior is Jolt's (an untracked active contact in FinishTrackingContactChanges is skipped where Jolt writes
//!   through an end() iterator).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const TempAllocatorFile = @import("../../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const HashCombine = @import("../../Core/HashCombine.zig");
const UnorderedMap = @import("../../Core/UnorderedMap.zig").UnorderedMap;
const quickSort = @import("../../Core/QuickSort.zig").quickSort;
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const rayAABoxHitsDirection = @import("../../Geometry/RayAABox.zig").rayAABoxHitsDirection;
const GJKClosestPoint = @import("../../Geometry/GJKClosestPoint.zig").GJKClosestPoint;
const ConvexSupport = @import("../../Geometry/ConvexSupport.zig");
const PolygonConvexSupport = ConvexSupport.PolygonConvexSupport;
const AddConvexRadius = ConvexSupport.AddConvexRadius;
const physics_settings = @import("../PhysicsSettings.zig");
const DeterminismLog = @import("../DeterminismLog.zig");
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const BodyCreationSettings = @import("../Body/BodyCreationSettings.zig").BodyCreationSettings;
const BodyFilterFile = @import("../Body/BodyFilter.zig");
const BodyFilter = BodyFilterFile.BodyFilter;
const IgnoreSingleBodyFilterChained = BodyFilterFile.IgnoreSingleBodyFilterChained;
const BodyLockFile = @import("../Body/BodyLock.zig");
const BodyLockRead = BodyLockFile.BodyLockRead;
const BodyLockWrite = BodyLockFile.BodyLockWrite;
const ObjectLayerFile = @import("../Collision/ObjectLayer.zig");
const ObjectLayer = ObjectLayerFile.ObjectLayer;
const ObjectLayerFilter = ObjectLayerFile.ObjectLayerFilter;
const BroadPhaseLayerFilter = @import("../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerFilter;
const BackFaceMode = @import("../Collision/BackFaceMode.zig").BackFaceMode;
const ShapeFilter = @import("../Collision/ShapeFilter.zig").ShapeFilter;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const CollideShapeFile = @import("../Collision/CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const ShapeCastFile = @import("../Collision/ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const RShapeCast = ShapeCastFile.RShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const CollisionDispatch = @import("../Collision/CollisionDispatch.zig");
const NarrowPhaseQuery = @import("../Collision/NarrowPhaseQuery.zig").NarrowPhaseQuery;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const ConvexShape = @import("../Collision/Shape/ConvexShape.zig").ConvexShape;
const RotatedTranslatedShape = @import("../Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShape;
const ScaledShape = @import("../Collision/Shape/ScaledShape.zig").ScaledShape;
const CompoundShape = @import("../Collision/Shape/CompoundShape.zig").CompoundShape;
const CharacterBaseFile = @import("CharacterBase.zig");
const CharacterBase = CharacterBaseFile.CharacterBase;
const CharacterBaseSettings = CharacterBaseFile.CharacterBaseSettings;
const CharacterID = @import("CharacterID.zig").CharacterID;

/// Contains the configuration of a character
pub const CharacterVirtualSettings = struct {
    pub const overrides = .{};

    base: CharacterBaseSettings,

    /// ID to give to this character. This is used for deterministically sorting and as an identifier to represent the character in the contact removal callback.
    /// (Jolt's default member initializer `CharacterID::sNextCharacterID()` runs in `init` / `create`)
    id: CharacterID,

    /// Character mass (kg). Used to push down objects with gravity when the character is standing on top.
    mass: f32 = 70.0,

    /// Maximum force with which the character can push other bodies (N).
    max_strength: f32 = 100.0,

    /// An extra offset applied to the shape in local space. This allows applying an extra offset to the shape in local space.
    shape_offset: Vec3 = Vec3.zero(),

    ///@name Movement settings
    /// When colliding with back faces, the character will not be able to move through back facing triangles. Use this if you have triangles that need to collide on both sides.
    back_face_mode: BackFaceMode = .collide_with_back_faces,
    /// How far to scan outside of the shape for predictive contacts. A value of 0 will most likely cause the character to get stuck as it cannot properly calculate a sliding direction anymore. A value that's too high will cause ghost collisions.
    predictive_contact_distance: f32 = 0.1,
    /// Max amount of collision loops
    max_collision_iterations: u32 = 5,
    /// How often to try stepping in the constraint solving
    max_constraint_iterations: u32 = 15,
    /// Early out condition: If this much time is left to simulate we are done
    min_time_remaining: f32 = 1.0e-4,
    /// How far we're willing to penetrate geometry
    collision_tolerance: f32 = 1.0e-3,
    /// How far we try to stay away from the geometry, this ensures that the sweep will hit as little as possible lowering the collision cost and reducing the risk of getting stuck
    character_padding: f32 = 0.02,
    /// Max num hits to collect in order to avoid excess of contact points collection
    max_num_hits: u32 = 256,
    /// Cos(angle) where angle is the maximum angle between two hits contact normals that are allowed to be merged during hit reduction. Default is around 2.5 degrees. Set to -1 to turn off.
    hit_reduction_cos_max_angle: f32 = 0.999,
    /// This value governs how fast a penetration will be resolved, 0 = nothing is resolved, 1 = everything in one update
    penetration_recovery_speed: f32 = 1.0,

    /// This character can optionally have an inner rigid body. This rigid body can be used to give the character presence in the world. When set it means that:
    /// - Regular collision checks (e.g. NarrowPhaseQuery::CastRay) will collide with the rigid body (they cannot collide with CharacterVirtual since it is not added to the broad phase)
    /// - Regular contact callbacks will be called through the ContactListener (next to the ones that will be passed to the CharacterContactListener)
    /// - Fast moving objects of motion quality LinearCast will not be able to pass through the CharacterVirtual in 1 time step
    inner_body_shape: RefConst(Shape) = .empty,

    /// For a deterministic simulation, it is important to have a deterministic body ID. When set and when mInnerBodyShape is specified,
    /// the inner body will be created with this specified ID instead of a generated ID.
    inner_body_id_override: BodyID = .invalid,

    /// Layer that the inner rigid body will be added to
    inner_body_layer: ObjectLayer = 0,

    /// Constructor (the defaulted default constructor): consumes the next CharacterID
    pub fn init(allocator: Allocator) CharacterVirtualSettings {
        return .{ .base = .init(CharacterVirtualSettings, allocator), .id = CharacterID.nextCharacterID() };
    }

    /// new CharacterVirtualSettings() (reference count 0): consumes the next CharacterID
    pub fn create(allocator: Allocator) Allocator.Error!*CharacterVirtualSettings {
        const self = try allocator.create(CharacterVirtualSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The defaulted copy constructor: copies the settings (taking references to the shapes), not the reference count,
    /// and does not consume a CharacterID
    pub fn initCopy(allocator: Allocator, other: *const CharacterVirtualSettings) CharacterVirtualSettings {
        var copy = other.*;
        copy.base = .initCopy(CharacterBaseSettings.vtableFor(CharacterVirtualSettings), allocator, &other.base);
        copy.inner_body_shape = other.inner_body_shape.clone();
        return copy;
    }

    /// The defaulted copy assignment (operator =): copies the settings (the shape references too), keeps the vtable, the
    /// reference count and the allocator
    pub fn assign(self: *CharacterVirtualSettings, other: *const CharacterVirtualSettings) void {
        self.base.assign(&other.base);
        self.id = other.id;
        self.mass = other.mass;
        self.max_strength = other.max_strength;
        self.shape_offset = other.shape_offset;
        self.back_face_mode = other.back_face_mode;
        self.predictive_contact_distance = other.predictive_contact_distance;
        self.max_collision_iterations = other.max_collision_iterations;
        self.max_constraint_iterations = other.max_constraint_iterations;
        self.min_time_remaining = other.min_time_remaining;
        self.collision_tolerance = other.collision_tolerance;
        self.character_padding = other.character_padding;
        self.max_num_hits = other.max_num_hits;
        self.hit_reduction_cos_max_angle = other.hit_reduction_cos_max_angle;
        self.penetration_recovery_speed = other.penetration_recovery_speed;
        self.inner_body_shape.set(other.inner_body_shape.get());
        self.inner_body_id_override = other.inner_body_id_override;
        self.inner_body_layer = other.inner_body_layer;
    }

    /// The CharacterBaseSettings part (for Ref / RefConst)
    pub fn asCharacterBaseSettings(self: *CharacterVirtualSettings) *CharacterBaseSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap (embedded / stack)
    pub fn deinit(self: *CharacterVirtualSettings) void {
        self.base.deinit();
    }

    /// The destructor of this level (generated chain): releases the inner body shape
    pub fn destruct(self: *CharacterVirtualSettings) void {
        self.inner_body_shape.deinit();
    }
};

/// This class contains settings that allow you to override the behavior of a character's collision response
pub const CharacterContactSettings = struct {
    /// True when the object can push the virtual character.
    can_push_character: bool = true,

    /// True when the virtual character can apply impulses (push) the body.
    /// Note that this only works against rigid bodies. Other CharacterVirtual objects can only be moved in their own update,
    /// so you must ensure that in their OnCharacterContactAdded mCanPushCharacter is true.
    can_receive_impulses: bool = true,
};

/// Uniquely identifies a contact between a character and another body or character
pub const CharacterContactKey = extern struct {
    /// ID of body we're colliding with (if not invalid)
    body_b: BodyID = .invalid,
    /// Character we're colliding with (if not invalid)
    character_id_b: CharacterID = .invalid,
    /// Sub shape ID of body or character we're colliding with
    sub_shape_id_b: SubShapeID = .empty,

    comptime {
        std.debug.assert(@sizeOf(BodyID) + @sizeOf(CharacterID) + @sizeOf(SubShapeID) == @sizeOf(CharacterContactKey)); // No padding expected
    }

    /// Constructor (CharacterContactKey(const BodyID &, const SubShapeID &))
    pub fn fromBody(body_b: BodyID, sub_shape_id: SubShapeID) CharacterContactKey {
        return .{ .body_b = body_b, .sub_shape_id_b = sub_shape_id };
    }

    /// Constructor (CharacterContactKey(const CharacterID &, const SubShapeID &))
    pub fn fromCharacter(character_id_b: CharacterID, sub_shape_id: SubShapeID) CharacterContactKey {
        return .{ .character_id_b = character_id_b, .sub_shape_id_b = sub_shape_id };
    }

    /// Checks if two contacts refer to the same body (or virtual character)
    pub fn isSameBody(self: *const CharacterContactKey, other: *const CharacterContactKey) bool {
        return self.body_b.eql(other.body_b) and self.character_id_b.eql(other.character_id_b);
    }

    /// Equality operator
    pub fn eql(self: CharacterContactKey, rhs: CharacterContactKey) bool {
        return self.body_b.eql(rhs.body_b) and self.character_id_b.eql(rhs.character_id_b) and self.sub_shape_id_b.eql(rhs.sub_shape_id_b);
    }

    /// Hash of this structure
    pub fn getHash(self: CharacterContactKey) u64 {
        return HashCombine.hashBytes(std.mem.asBytes(&self));
    }

    // Saving / restoring state for replay
    pub fn saveState(self: *const CharacterContactKey, stream: *StateRecorder) void {
        stream.write(self.body_b);
        stream.write(self.character_id_b);
        stream.write(self.sub_shape_id_b);
    }

    pub fn restoreState(self: *CharacterContactKey, stream: *StateRecorder) void {
        stream.read(&self.body_b);
        stream.read(&self.character_id_b);
        stream.read(&self.sub_shape_id_b);
    }
};

/// Encapsulates a collision contact between a character and another rigid body / character
pub const CharacterContact = extern struct {
    /// The CharacterContactKey part (C++ base class)
    base: CharacterContactKey = .{},
    /// Position where the character makes contact
    position: RVec3 = RVec3.zero(),
    /// Velocity of the contact point
    linear_velocity: Vec3 = Vec3.zero(),
    /// Contact normal, pointing towards the character
    contact_normal: Vec3 = Vec3.zero(),
    /// Surface normal of the contact. Has been flipped if the contact is back facing (mIsBackFacingContact) and is equal to the contact normal if the contact normal is pointing up more.
    surface_normal: Vec3 = Vec3.zero(),
    /// Distance to the contact <= 0 means that it is an actual contact, > 0 means predictive
    distance: f32 = 0.0,
    /// Fraction along the path where this contact takes place
    fraction: f32 = 0.0,
    /// Motion type of B, used to determine the priority of the contact
    motion_type_b: MotionType = .static,
    /// If B is a sensor
    is_sensor_b: bool = false,
    /// Character we're colliding with (if not nullptr). Note that this may be a dangling pointer when accessed through GetActiveContacts(), use mCharacterIDB instead.
    character_b: ?*const CharacterVirtual = null,
    /// User data of B
    user_data: u64 = 0,
    /// Material of B
    material: *const PhysicsMaterial = PhysicsMaterial.default,
    /// If the character actually collided with the contact (can be false if a predictive contact never becomes a real one)
    had_collision: bool = false,
    /// If the contact validate callback chose to discard this contact or when the body is a sensor
    was_discarded: bool = false,
    /// When true, the velocity of the contact point can push the character
    can_push_character: bool = true,
    /// If this contact came from a back facing triangle / collision shape
    is_back_facing_contact: bool = false,

    comptime {
        if (@sizeOf(usize) == 8) std.debug.assert(@sizeOf(CharacterContact) == if (Core.double_precision) 160 else 128); // Jolt's sizeof (the temp memory per contact)
    }

    /// Checks if two contacts refer to the same body (or virtual character) (CharacterContactKey::IsSameBody)
    pub fn isSameBody(self: *const CharacterContact, other: *const CharacterContact) bool {
        return self.base.isSameBody(&other.base);
    }

    // Saving / restoring state for replay
    pub fn saveState(self: *const CharacterContact, stream: *StateRecorder) void {
        self.base.saveState(stream);

        stream.write(self.position);
        stream.write(self.linear_velocity);
        stream.write(self.contact_normal);
        stream.write(self.surface_normal);
        stream.write(self.distance);
        stream.write(self.fraction);
        stream.write(self.motion_type_b);
        stream.write(self.is_sensor_b);
        stream.write(self.had_collision);
        stream.write(self.was_discarded);
        stream.write(self.can_push_character);
        // Cannot store pointers to character B, user data and material
    }

    pub fn restoreState(self: *CharacterContact, stream: *StateRecorder) void {
        self.base.restoreState(stream);

        stream.read(&self.position);
        stream.read(&self.linear_velocity);
        stream.read(&self.contact_normal);
        stream.read(&self.surface_normal);
        stream.read(&self.distance);
        stream.read(&self.fraction);
        readMotionType(stream, &self.motion_type_b);
        readBool(stream, &self.is_sensor_b);
        readBool(stream, &self.had_collision);
        readBool(stream, &self.was_discarded);
        readBool(stream, &self.can_push_character);
        self.character_b = null; // Cannot restore character B
        self.user_data = 0; // Cannot restore user data
        self.material = PhysicsMaterial.default; // Cannot restore material
    }
};

/// Read a bool as a byte (Jolt reads the raw byte into its bool)
fn readBool(stream: *StateRecorder, value: *bool) void {
    var raw: u8 = @intFromBool(value.*);
    stream.read(&raw);
    value.* = raw != 0;
}

/// Read an EMotionType byte (a value that is not a motion type becomes Static, see the file header)
fn readMotionType(stream: *StateRecorder, value: *MotionType) void {
    var raw: u8 = @intFromEnum(value.*);
    stream.read(&raw);
    value.* = std.enums.fromInt(MotionType, raw) orelse .static;
}

/// This class receives callbacks when a virtual character hits something.
/// Once created, register it on a CharacterVirtual by using the character's SetListener method.
/// (Zolt: pattern B, `CharacterContactListener.init(&impl)` wires the callbacks that the implementation declares, a
/// missing callback behaves like Jolt's default. The listener does not own the implementation.)
pub const CharacterContactListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    /// One nullable entry per virtual function, null is Jolt's default implementation
    pub const VTable = struct {
        /// Callback to adjust the velocity of a body as seen by the character. Can be adjusted to e.g. implement a conveyor belt or an inertial dampener system of a sci-fi space ship.
        /// Note that body2 is locked during the callback so you can read its properties freely.
        onAdjustBodyVelocity: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, body2: *const Body, linear_velocity: *Vec3, angular_velocity: *Vec3) void = null,

        /// Checks if a character can collide with specified body. Return true if the contact is valid.
        onContactValidate: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact) bool = null,

        /// Called whenever the character collides with a body for the first time.
        /// @param character Character that is being solved
        /// @param contact Contact that we're colliding against
        /// @param settings Settings returned by the contact callback to indicate how the character should behave
        onContactAdded: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void = null,

        /// Called whenever the character persists colliding with a body.
        /// @param character Character that is being solved
        /// @param contact Contact that we're colliding against
        /// @param settings Settings returned by the contact callback to indicate how the character should behave
        onContactPersisted: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void = null,

        /// Called whenever the character loses contact with a body.
        /// Note that there is no guarantee that the body or its sub shape still exists at this point. The body may have been deleted since the last update.
        /// @param character Character that is being solved
        /// @param body_id2 Body ID of body that is being hit
        /// @param sub_shape_id2 Sub shape ID of shape that is being hit
        onContactRemoved: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void = null,

        /// Same as OnContactValidate but when colliding with a CharacterVirtual
        onCharacterContactValidate: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact) bool = null,

        /// Same as OnContactAdded but when colliding with a CharacterVirtual
        onCharacterContactAdded: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void = null,

        /// Same as OnContactPersisted but when colliding with a CharacterVirtual
        onCharacterContactPersisted: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void = null,

        /// Same as OnContactRemoved but when colliding with a CharacterVirtual
        /// Note that other_character_id can be the ID of a character that has been deleted. This happens if the character was in contact with this character during the last update, but has been deleted since.
        onCharacterContactRemoved: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) void = null,

        /// Called whenever a contact is being used by the solver. Allows the listener to override the resulting character velocity (e.g. by preventing sliding along certain surfaces).
        /// @param character Character that is being solved
        /// @param body_id2 Body ID of body that is being hit
        /// @param sub_shape_id2 Sub shape ID of shape that is being hit
        /// @param contact_position World space contact position
        /// @param contact_normal World space contact normal
        /// @param contact_velocity World space velocity of contact point (e.g. for a moving platform)
        /// @param contact_material Material of contact point
        /// @param character_velocity World space velocity of the character prior to hitting this contact
        /// @param new_character_velocity Contains the calculated world space velocity of the character after hitting this contact, this velocity slides along the surface of the contact. Can be modified by the listener to provide an alternative velocity.
        onContactSolve: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void = null,

        /// Same as OnContactSolve but when colliding with a CharacterVirtual
        onCharacterContactSolve: ?*const fn (ptr: *anyopaque, character: *const CharacterVirtual, other_character: *const CharacterVirtual, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void = null,
    };

    /// Wrap any `*T` that declares some of the callbacks
    pub fn init(impl: anytype) CharacterContactListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing the functions of the outer struct
            fn cast(ptr: *anyopaque) *T {
                return @ptrCast(@alignCast(ptr));
            }
            fn onAdjustBodyVelocityThunk(ptr: *anyopaque, character: *const CharacterVirtual, body2: *const Body, linear_velocity: *Vec3, angular_velocity: *Vec3) void {
                cast(ptr).onAdjustBodyVelocity(character, body2, linear_velocity, angular_velocity);
            }
            fn onContactValidateThunk(ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
                return cast(ptr).onContactValidate(character, contact);
            }
            fn onContactAddedThunk(ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
                cast(ptr).onContactAdded(character, contact, settings);
            }
            fn onContactPersistedThunk(ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
                cast(ptr).onContactPersisted(character, contact, settings);
            }
            fn onContactRemovedThunk(ptr: *anyopaque, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void {
                cast(ptr).onContactRemoved(character, body_id2, sub_shape_id2);
            }
            fn onCharacterContactValidateThunk(ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
                return cast(ptr).onCharacterContactValidate(character, contact);
            }
            fn onCharacterContactAddedThunk(ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
                cast(ptr).onCharacterContactAdded(character, contact, settings);
            }
            fn onCharacterContactPersistedThunk(ptr: *anyopaque, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
                cast(ptr).onCharacterContactPersisted(character, contact, settings);
            }
            fn onCharacterContactRemovedThunk(ptr: *anyopaque, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) void {
                cast(ptr).onCharacterContactRemoved(character, other_character_id, sub_shape_id2);
            }
            fn onContactSolveThunk(ptr: *anyopaque, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
                cast(ptr).onContactSolve(character, body_id2, sub_shape_id2, contact_position, contact_normal, contact_velocity, contact_material, character_velocity, new_character_velocity);
            }
            fn onCharacterContactSolveThunk(ptr: *anyopaque, character: *const CharacterVirtual, other_character: *const CharacterVirtual, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
                cast(ptr).onCharacterContactSolve(character, other_character, sub_shape_id2, contact_position, contact_normal, contact_velocity, contact_material, character_velocity, new_character_velocity);
            }
            const vtable: VTable = .{
                .onAdjustBodyVelocity = if (@hasDecl(T, "onAdjustBodyVelocity")) onAdjustBodyVelocityThunk else null,
                .onContactValidate = if (@hasDecl(T, "onContactValidate")) onContactValidateThunk else null,
                .onContactAdded = if (@hasDecl(T, "onContactAdded")) onContactAddedThunk else null,
                .onContactPersisted = if (@hasDecl(T, "onContactPersisted")) onContactPersistedThunk else null,
                .onContactRemoved = if (@hasDecl(T, "onContactRemoved")) onContactRemovedThunk else null,
                .onCharacterContactValidate = if (@hasDecl(T, "onCharacterContactValidate")) onCharacterContactValidateThunk else null,
                .onCharacterContactAdded = if (@hasDecl(T, "onCharacterContactAdded")) onCharacterContactAddedThunk else null,
                .onCharacterContactPersisted = if (@hasDecl(T, "onCharacterContactPersisted")) onCharacterContactPersistedThunk else null,
                .onCharacterContactRemoved = if (@hasDecl(T, "onCharacterContactRemoved")) onCharacterContactRemovedThunk else null,
                .onContactSolve = if (@hasDecl(T, "onContactSolve")) onContactSolveThunk else null,
                .onCharacterContactSolve = if (@hasDecl(T, "onCharacterContactSolve")) onCharacterContactSolveThunk else null,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// See VTable.onAdjustBodyVelocity (default: do nothing, the linear and angular velocity are already filled in)
    pub fn onAdjustBodyVelocity(self: CharacterContactListener, character: *const CharacterVirtual, body2: *const Body, linear_velocity: *Vec3, angular_velocity: *Vec3) void {
        if (self.vtable.onAdjustBodyVelocity) |f| f(self.ptr, character, body2, linear_velocity, angular_velocity);
    }

    /// See VTable.onContactValidate (default: true)
    pub fn onContactValidate(self: CharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
        if (self.vtable.onContactValidate) |f| return f(self.ptr, character, contact);
        return true;
    }

    /// See VTable.onContactAdded (default: do nothing)
    pub fn onContactAdded(self: CharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        if (self.vtable.onContactAdded) |f| f(self.ptr, character, contact, settings);
    }

    /// See VTable.onContactPersisted (default: do nothing)
    pub fn onContactPersisted(self: CharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        if (self.vtable.onContactPersisted) |f| f(self.ptr, character, contact, settings);
    }

    /// See VTable.onContactRemoved (default: do nothing)
    pub fn onContactRemoved(self: CharacterContactListener, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void {
        if (self.vtable.onContactRemoved) |f| f(self.ptr, character, body_id2, sub_shape_id2);
    }

    /// See VTable.onCharacterContactValidate (default: true)
    pub fn onCharacterContactValidate(self: CharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
        if (self.vtable.onCharacterContactValidate) |f| return f(self.ptr, character, contact);
        return true;
    }

    /// See VTable.onCharacterContactAdded (default: do nothing)
    pub fn onCharacterContactAdded(self: CharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        if (self.vtable.onCharacterContactAdded) |f| f(self.ptr, character, contact, settings);
    }

    /// See VTable.onCharacterContactPersisted (default: do nothing)
    pub fn onCharacterContactPersisted(self: CharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        if (self.vtable.onCharacterContactPersisted) |f| f(self.ptr, character, contact, settings);
    }

    /// See VTable.onCharacterContactRemoved (default: do nothing)
    pub fn onCharacterContactRemoved(self: CharacterContactListener, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) void {
        if (self.vtable.onCharacterContactRemoved) |f| f(self.ptr, character, other_character_id, sub_shape_id2);
    }

    /// See VTable.onContactSolve (default: do nothing)
    pub fn onContactSolve(self: CharacterContactListener, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
        if (self.vtable.onContactSolve) |f| f(self.ptr, character, body_id2, sub_shape_id2, contact_position, contact_normal, contact_velocity, contact_material, character_velocity, new_character_velocity);
    }

    /// See VTable.onCharacterContactSolve (default: do nothing)
    pub fn onCharacterContactSolve(self: CharacterContactListener, character: *const CharacterVirtual, other_character: *const CharacterVirtual, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
        if (self.vtable.onCharacterContactSolve) |f| f(self.ptr, character, other_character, sub_shape_id2, contact_position, contact_normal, contact_velocity, contact_material, character_velocity, new_character_velocity);
    }
};

/// Interface class that allows a CharacterVirtual to check collision with other CharacterVirtual instances.
/// Since CharacterVirtual instances are not registered anywhere, it is up to the application to test collision against relevant characters.
/// The characters could be stored in a tree structure to make this more efficient.
/// (Zolt: pattern B, `CharacterVsCharacterCollision.init(&impl)`; both functions are pure virtual in Jolt, the
/// implementation must declare them. The interface does not own the implementation.)
pub const CharacterVsCharacterCollision = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Collide a character against other CharacterVirtuals.
        /// @param character The character to collide.
        /// @param center_of_mass_transform Center of mass transform for this character.
        /// @param collide_shape_settings Settings for the collision check.
        /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
        /// @param collector Collision collector that receives the collision results.
        collideCharacter: *const fn (ptr: *anyopaque, character: *const CharacterVirtual, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector) void,

        /// Cast a character against other CharacterVirtuals.
        /// @param character The character to cast.
        /// @param center_of_mass_transform Center of mass transform for this character.
        /// @param direction Direction and length to cast in.
        /// @param shape_cast_settings Settings for the shape cast.
        /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
        /// @param collector Collision collector that receives the collision results.
        castCharacter: *const fn (ptr: *anyopaque, character: *const CharacterVirtual, center_of_mass_transform: RMat44, direction: Vec3, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector) void,
    };

    /// Wrap any `*T` that declares `collideCharacter` and `castCharacter` (Jolt's const member functions)
    pub fn init(impl: anytype) CharacterVsCharacterCollision {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn cast(ptr: *anyopaque) *const T {
                return @ptrCast(@alignCast(ptr));
            }
            fn collideCharacterThunk(ptr: *anyopaque, character: *const CharacterVirtual, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector) void {
                cast(ptr).collideCharacter(character, center_of_mass_transform, collide_shape_settings, base_offset, collector);
            }
            fn castCharacterThunk(ptr: *anyopaque, character: *const CharacterVirtual, center_of_mass_transform: RMat44, direction: Vec3, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector) void {
                cast(ptr).castCharacter(character, center_of_mass_transform, direction, shape_cast_settings, base_offset, collector);
            }
            const vtable: VTable = .{ .collideCharacter = collideCharacterThunk, .castCharacter = castCharacterThunk };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// See VTable.collideCharacter
    pub fn collideCharacter(self: CharacterVsCharacterCollision, character: *const CharacterVirtual, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector) void {
        self.vtable.collideCharacter(self.ptr, character, center_of_mass_transform, collide_shape_settings, base_offset, collector);
    }

    /// See VTable.castCharacter
    pub fn castCharacter(self: CharacterVsCharacterCollision, character: *const CharacterVirtual, center_of_mass_transform: RMat44, direction: Vec3, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector) void {
        self.vtable.castCharacter(self.ptr, character, center_of_mass_transform, direction, shape_cast_settings, base_offset, collector);
    }
};

/// Simple collision checker that loops over all registered characters.
/// This is a brute force checking algorithm. If you have a lot of characters you may want to store your characters
/// in a hierarchical structure to make this more efficient.
/// Note that this is not thread safe, so make sure that only one CharacterVirtual is checking collision at a time.
/// (Zolt: `init(allocator)` / `deinit`, the interface for SetCharacterVsCharacterCollision is `characterVsCharacterCollision()`)
pub const CharacterVsCharacterCollisionSimple = struct {
    /// Owns the memory of `characters`
    allocator: Allocator,

    /// The list of characters to check collision against
    characters: std.ArrayList(*CharacterVirtual) = .empty,

    pub fn init(allocator: Allocator) CharacterVsCharacterCollisionSimple {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *CharacterVsCharacterCollisionSimple) void {
        self.characters.deinit(self.allocator);
    }

    /// The CharacterVsCharacterCollision interface of this object (it must not move while a character uses it)
    pub fn characterVsCharacterCollision(self: *CharacterVsCharacterCollisionSimple) CharacterVsCharacterCollision {
        return .init(self);
    }

    /// Add a character to the list of characters to check collision against.
    pub fn add(self: *CharacterVsCharacterCollisionSimple, character: *CharacterVirtual) Allocator.Error!void {
        try self.characters.append(self.allocator, character);
    }

    /// Remove a character from the list of characters to check collision against.
    pub fn remove(self: *CharacterVsCharacterCollisionSimple, character: *const CharacterVirtual) void {
        for (self.characters.items, 0..) |c, i| {
            if (c == character) {
                _ = self.characters.orderedRemove(i);
                return;
            }
        }
    }

    // See: CharacterVsCharacterCollision
    pub fn collideCharacter(self: *const CharacterVsCharacterCollisionSimple, character: *const CharacterVirtual, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector) void {
        // Make shape 1 relative to inBaseOffset
        const transform1 = center_of_mass_transform.postTranslatedRVec3(base_offset.negate()).toMat44();

        const shape1 = character.getShape().?;
        var settings = collide_shape_settings.*;

        // Get bounds for character
        const bounds1 = shape1.getWorldSpaceBounds(transform1, Vec3.one());

        // Iterate over all characters
        for (self.characters.items) |c| {
            if (c != character and !collector.shouldEarlyOut()) {
                // Make shape 2 relative to inBaseOffset
                const transform2 = c.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();

                // We need to add the padding of character 2 so that we will detect collision with its outer shell
                settings.max_separation_distance = collide_shape_settings.max_separation_distance + c.getCharacterPadding();

                // Check if the bounding boxes of the characters overlap
                const shape2 = c.getShape().?;
                var bounds2 = shape2.getWorldSpaceBounds(transform2, Vec3.one());
                bounds2.expandBy(Vec3.replicate(settings.max_separation_distance));
                if (!bounds1.overlaps(bounds2))
                    continue;

                // Collector needs to know which character we're colliding with
                collector.setUserData(@intFromPtr(c));

                // Note that this collides against the character's shape without padding, this will be corrected for in CharacterVirtual::GetContactsAtPosition
                CollisionDispatch.collideShapeVsShape(shape1, shape2, Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, collector, &.{});
            }
        }

        // Reset the user data
        collector.setUserData(0);
    }

    // See: CharacterVsCharacterCollision
    pub fn castCharacter(self: *const CharacterVsCharacterCollisionSimple, character: *const CharacterVirtual, center_of_mass_transform: RMat44, direction: Vec3, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector) void {
        // Convert shape cast relative to inBaseOffset
        const transform1 = center_of_mass_transform.postTranslatedRVec3(base_offset.negate()).toMat44();
        const shape_cast = ShapeCast.init(character.getShape().?, Vec3.one(), transform1, direction);

        // Get world space bounds of the character in the form of center and extent
        const origin = shape_cast.shape_world_bounds.getCenter();
        const extents = shape_cast.shape_world_bounds.getExtent().add(Vec3.replicate(shape_cast_settings.extra_convex_radius));

        // Create a copy of the settings so we can modify it for each character we test against
        var cast_settings = shape_cast_settings.*;

        // Iterate over all characters
        for (self.characters.items) |c| {
            if (c != character and !collector.shouldEarlyOut()) {
                // Make shape 2 relative to inBaseOffset
                const transform2 = c.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();

                // We need to add the padding of character 2 so that we will detect collision with its outer shell
                cast_settings.extra_convex_radius = shape_cast_settings.extra_convex_radius + c.getCharacterPadding();

                // Sweep bounding box of the character against the bounding box of the other character to see if they can collide
                const shape2 = c.getShape().?;
                var bounds2 = shape2.getWorldSpaceBounds(transform2, Vec3.one());
                bounds2.expandBy(extents.add(Vec3.replicate(c.getCharacterPadding())));
                if (!rayAABoxHitsDirection(origin, direction, bounds2.min, bounds2.max))
                    continue;

                // Collector needs to know which character we're colliding with
                collector.setUserData(@intFromPtr(c));

                // Note that this collides against the character's shape without padding, this will be corrected for in CharacterVirtual::ValidateMovement
                CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &cast_settings, shape2, Vec3.one(), &.{}, transform2, .{}, .{}, collector);
            }
        }

        // Reset the user data
        collector.setUserData(0);
    }
};

/// Jolt's `Array<T, STLTempAllocator<T>>` as CharacterVirtual uses it: one block of the temp allocator that is reserved
/// once (Jolt reserves the final capacity, the STL adapter cannot reallocate) and freed at the end of the C++ scope.
fn TempList(comptime T: type) type {
    return struct {
        const Self = @This();

        /// The reserved block (empty: nothing reserved)
        buffer: TempArray(T) = .empty,
        /// Number of items in use
        len: usize = 0,

        /// reserve(n) of an empty list: one block of n items, nothing for n == 0 (like Jolt's reserve)
        fn reserve(self: *Self, temp_allocator: TempAllocator, n: usize) TempAllocator.Error!void {
            std.debug.assert(self.buffer.isEmpty() and self.len == 0);
            if (n > 0)
                self.buffer = try .alloc(temp_allocator, n);
        }

        /// resize(n) of an empty list (the items are undefined, Jolt's pointers are not initialized either)
        fn resize(self: *Self, temp_allocator: TempAllocator, n: usize) TempAllocator.Error!void {
            try self.reserve(temp_allocator, n);
            self.len = n;
        }

        /// The (begin, end, allocator) constructor: a copy of `items`
        fn assignSlice(self: *Self, temp_allocator: TempAllocator, items_to_copy: []const T) TempAllocator.Error!void {
            try self.resize(temp_allocator, items_to_copy.len);
            @memcpy(self.items(), items_to_copy);
        }

        /// The destructor: frees the block
        fn free(self: *Self, temp_allocator: TempAllocator) void {
            self.buffer.free(temp_allocator);
            self.len = 0;
        }

        fn items(self: *const Self) []T {
            return self.buffer.items[0..self.len];
        }

        /// push_back / emplace_back (within the reserved capacity)
        fn append(self: *Self, item: T) void {
            std.debug.assert(self.len < self.buffer.items.len); // Jolt would reallocate, which a TempAllocatorImpl rejects
            self.buffer.items[self.len] = item;
            self.len += 1;
        }

        /// back()
        fn last(self: *const Self) *T {
            return &self.buffer.items[self.len - 1];
        }

        /// pop_back()
        fn pop(self: *Self) void {
            self.len -= 1;
        }

        /// erase(begin() + index)
        fn orderedRemove(self: *Self, index: usize) void {
            const buf = self.buffer.items;
            std.mem.copyForwards(T, buf[index .. self.len - 1], buf[index + 1 .. self.len]);
            self.len -= 1;
        }

        /// clear()
        fn clear(self: *Self) void {
            self.len = 0;
        }
    };
}

/// The filters of the CharacterVirtual queries (Jolt passes the four references to every private function)
const Filters = struct {
    broad_phase_layer_filter: *const BroadPhaseLayerFilter,
    object_layer_filter: *const ObjectLayerFilter,
    body_filter: *const BodyFilter,
    /// null: Jolt's default ShapeFilter
    shape_filter: ?*ShapeFilter,
};

/// Runtime character object.
/// This object usually represents the player. Contrary to the Character class it doesn't use a rigid body but moves doing collision checks only (hence the name virtual).
/// The advantage of this is that you can determine when the character moves in the frame (usually this has to happen at a very particular point in the frame)
/// but the downside is that other objects don't see this virtual character. To make a CharacterVirtual visible to the simulation, you can optionally create an inner
/// rigid body through CharacterVirtualSettings::mInnerBodyShape. A CharacterVirtual is not tracked by the PhysicsSystem so you need to update it yourself. This also means
/// that a call to PhysicsSystem::SaveState will not save its state, you need to call CharacterVirtual::SaveState yourself.
pub const CharacterVirtual = struct {
    pub const overrides = .{ .saveState, .restoreState };

    base: CharacterBase,

    // ID
    id: CharacterID,

    // Our main listener for contacts
    listener: ?CharacterContactListener = null,

    // Interface to detect collision between characters
    character_vs_character_collision: ?CharacterVsCharacterCollision = null,

    // Movement settings
    back_face_mode: BackFaceMode, // When colliding with back faces, the character will not be able to move through back facing triangles. Use this if you have triangles that need to collide on both sides.
    predictive_contact_distance: f32, // How far to scan outside of the shape for predictive contacts. A value of 0 will most likely cause the character to get stuck as it cannot properly calculate a sliding direction anymore. A value that's too high will cause ghost collisions.
    max_collision_iterations: u32, // Max amount of collision loops
    max_constraint_iterations: u32, // How often to try stepping in the constraint solving
    min_time_remaining: f32, // Early out condition: If this much time is left to simulate we are done
    collision_tolerance: f32, // How far we're willing to penetrate geometry
    character_padding: f32, // How far we try to stay away from the geometry, this ensures that the sweep will hit as little as possible lowering the collision cost and reducing the risk of getting stuck
    max_num_hits: u32, // Max num hits to collect in order to avoid excess of contact points collection
    hit_reduction_cos_max_angle: f32, // Cos(angle) where angle is the maximum angle between two hits contact normals that are allowed to be merged during hit reduction. Default is around 2.5 degrees. Set to -1 to turn off.
    penetration_recovery_speed: f32, // This value governs how fast a penetration will be resolved, 0 = nothing is resolved, 1 = everything in one update
    enhanced_internal_edge_removal: bool, // Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.

    // Character mass (kg)
    mass: f32 = undefined,

    // Maximum force with which the character can push other bodies (N)
    max_strength: f32 = undefined,

    // An extra offset applied to the shape in local space. This allows applying an extra offset to the shape in local space.
    shape_offset: Vec3 = Vec3.zero(),

    // Current position (of the base, not the center of mass)
    position: RVec3 = RVec3.zero(),

    // Current rotation (of the base, not of the center of mass)
    rotation: Quat = Quat.identity(),

    // Current linear velocity
    linear_velocity: Vec3 = Vec3.zero(),

    // List of contacts that were active in the last frame
    active_contacts: ContactList = .empty,

    // Remembers how often we called StartTrackingContactChanges
    tracking_contact_changes: i32 = 0,

    // View from a contact listener perspective on which contacts have been added/removed
    listener_contacts: ListenerContacts = .empty,

    // Remembers the delta time of the last update
    last_delta_time: f32 = 1.0 / 60.0,

    // Remember if we exceeded the maximum number of hits and had to remove similar contacts
    // (`mutable` in Jolt: written by getContactsAtPosition, which takes a mutable receiver, Rule M)
    max_hits_exceeded: bool = false,

    // User data, can be used for anything by the application
    user_data: u64 = 0,

    // The inner rigid body that proxies the character in the world
    inner_body_id: BodyID = .invalid,

    pub const ContactList = std.ArrayList(CharacterContact);

    // View from a contact listener perspective on which contacts have been added/removed
    const ListenerContactValue = struct {
        settings: CharacterContactSettings = .{},
        count: i32 = 0,
    };

    const ListenerContacts = UnorderedMap(CharacterContactKey, ListenerContactValue, .{});

    /// Constructor (an embedded character; `create` is `new CharacterVirtual`)
    /// @param settings The settings for the character
    /// @param position Initial position for the character
    /// @param rotation Initial rotation for the character (usually only around the up-axis)
    /// @param user_data Application specific value (Jolt's constructor without user data passes 0)
    /// @param system Physics system that this character will be added to
    pub fn init(allocator: Allocator, settings: *const CharacterVirtualSettings, position: RVec3, rotation: Quat, user_data: u64, system: *PhysicsSystem) Allocator.Error!CharacterVirtual {
        var self: CharacterVirtual = .{
            .base = .init(CharacterVirtual, allocator, &settings.base, system),
            .id = settings.id,
            .back_face_mode = settings.back_face_mode,
            .predictive_contact_distance = settings.predictive_contact_distance,
            .max_collision_iterations = settings.max_collision_iterations,
            .max_constraint_iterations = settings.max_constraint_iterations,
            .min_time_remaining = settings.min_time_remaining,
            .collision_tolerance = settings.collision_tolerance,
            .character_padding = settings.character_padding,
            .max_num_hits = settings.max_num_hits,
            .hit_reduction_cos_max_angle = settings.hit_reduction_cos_max_angle,
            .penetration_recovery_speed = settings.penetration_recovery_speed,
            .enhanced_internal_edge_removal = settings.base.enhanced_internal_edge_removal,
            .shape_offset = settings.shape_offset,
            .position = position,
            .rotation = rotation,
            .user_data = user_data,
        };
        errdefer CharacterBase.destruct(&self.base); // Zolt: nothing is left behind (the inner body was not created or is destroyed below)

        if (Core.enable_asserts) std.debug.assert(!self.id.isInvalid()); // User input, Jolt's release build continues

        // Copy settings
        self.setMaxStrength(settings.max_strength);
        self.setMass(settings.mass);

        // Create an inner rigid body if requested
        if (settings.inner_body_shape.get()) |inner_body_shape| {
            var body_settings: BodyCreationSettings = .initWithShape(inner_body_shape, self.getInnerBodyPosition(), self.rotation, .kinematic, settings.inner_body_layer);
            defer body_settings.deinit();
            body_settings.allow_sleeping = false; // Disable sleeping so that we will receive sensor callbacks
            body_settings.user_data = user_data;

            const bi = system.getBodyInterfaceMut();
            const create_result = if (settings.inner_body_id_override.isInvalid())
                bi.createBody(&body_settings)
            else
                bi.createBodyWithID(settings.inner_body_id_override, &body_settings);
            const inner_body = create_result catch |err| switch (err) {
                error.OutOfMemory => return error.OutOfMemory,
                error.InvalidShape => unreachable, // The settings hold a shape
            };
            if (inner_body) |body| {
                self.inner_body_id = body.getID();
                bi.addBody(self.inner_body_id, .activate) catch |err| {
                    // Zolt: the broad phase could not add the body, destroy it again
                    bi.destroyBody(self.inner_body_id);
                    return err;
                };
            }
        }

        return self;
    }

    /// new CharacterVirtual(settings, position, rotation, user data, system): reference count 0, the last release of a
    /// Ref(CharacterBase) destroys it
    pub fn create(allocator: Allocator, settings: *const CharacterVirtualSettings, position: RVec3, rotation: Quat, user_data: u64, system: *PhysicsSystem) Allocator.Error!*CharacterVirtual {
        const self = try allocator.create(CharacterVirtual);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, settings, position, rotation, user_data, system);
        return self;
    }

    /// Destructor (~CharacterVirtual, the destructor level of the generated chain): removes and destroys the inner body
    pub fn destruct(self: *CharacterVirtual) void {
        if (!self.inner_body_id.isInvalid()) {
            const bi = self.base.system.getBodyInterfaceMut();
            bi.removeBody(self.inner_body_id);
            bi.destroyBody(self.inner_body_id);
        }
        self.listener_contacts.deinit(self.base.allocator);
        self.active_contacts.deinit(self.base.allocator);
    }

    /// The CharacterBase part (for Ref / RefConst and the virtual functions)
    pub fn asCharacterBase(self: *CharacterVirtual) *CharacterBase {
        return &self.base;
    }

    /// The ID of this character
    pub fn getID(self: *const CharacterVirtual) CharacterID {
        return self.id;
    }

    /// Set the contact listener
    pub fn setListener(self: *CharacterVirtual, listener: ?CharacterContactListener) void {
        self.listener = listener;
    }

    /// Get the current contact listener
    pub fn getListener(self: *const CharacterVirtual) ?CharacterContactListener {
        return self.listener;
    }

    /// Set the character vs character collision interface
    pub fn setCharacterVsCharacterCollision(self: *CharacterVirtual, character_vs_character_collision: ?CharacterVsCharacterCollision) void {
        self.character_vs_character_collision = character_vs_character_collision;
    }

    /// Get the linear velocity of the character (m / s)
    pub fn getLinearVelocity(self: *const CharacterVirtual) Vec3 {
        return self.linear_velocity;
    }

    /// Set the linear velocity of the character (m / s)
    pub fn setLinearVelocity(self: *CharacterVirtual, linear_velocity: Vec3) void {
        self.linear_velocity = linear_velocity;
    }

    /// Get the position of the character
    pub fn getPosition(self: *const CharacterVirtual) RVec3 {
        return self.position;
    }

    /// Set the position of the character
    pub fn setPosition(self: *CharacterVirtual, position: RVec3) void {
        self.position = position;
        self.updateInnerBodyTransform();
    }

    /// Get the rotation of the character
    pub fn getRotation(self: *const CharacterVirtual) Quat {
        return self.rotation;
    }

    /// Set the rotation of the character
    pub fn setRotation(self: *CharacterVirtual, rotation: Quat) void {
        self.rotation = rotation;
        self.updateInnerBodyTransform();
    }

    // Get the center of mass position of the shape
    pub fn getCenterOfMassPosition(self: *const CharacterVirtual) RVec3 {
        return self.position.addVec3(self.rotation.mulVec3(self.shape_offset.add(self.currentShape().getCenterOfMass())).add(self.base.up.mulScalar(self.character_padding)));
    }

    /// Calculate the world transform of the character
    pub fn getWorldTransform(self: *const CharacterVirtual) RMat44 {
        return RMat44.rotationTranslation(self.rotation, self.position);
    }

    /// Calculates the transform for this character's center of mass
    pub fn getCenterOfMassTransform(self: *const CharacterVirtual) RMat44 {
        return self.getCenterOfMassTransformAt(self.position, self.rotation, self.currentShape());
    }

    /// Character mass (kg)
    pub fn getMass(self: *const CharacterVirtual) f32 {
        return self.mass;
    }

    pub fn setMass(self: *CharacterVirtual, mass: f32) void {
        self.mass = mass;
    }

    /// Maximum force with which the character can push other bodies (N)
    pub fn getMaxStrength(self: *const CharacterVirtual) f32 {
        return self.max_strength;
    }

    pub fn setMaxStrength(self: *CharacterVirtual, max_strength: f32) void {
        self.max_strength = max_strength;
    }

    /// This value governs how fast a penetration will be resolved, 0 = nothing is resolved, 1 = everything in one update
    pub fn getPenetrationRecoverySpeed(self: *const CharacterVirtual) f32 {
        return self.penetration_recovery_speed;
    }

    pub fn setPenetrationRecoverySpeed(self: *CharacterVirtual, speed: f32) void {
        self.penetration_recovery_speed = speed;
    }

    /// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
    pub fn getEnhancedInternalEdgeRemoval(self: *const CharacterVirtual) bool {
        return self.enhanced_internal_edge_removal;
    }

    pub fn setEnhancedInternalEdgeRemoval(self: *CharacterVirtual, apply: bool) void {
        self.enhanced_internal_edge_removal = apply;
    }

    /// Character padding
    pub fn getCharacterPadding(self: *const CharacterVirtual) f32 {
        return self.character_padding;
    }

    /// Max num hits to collect in order to avoid excess of contact points collection
    pub fn getMaxNumHits(self: *const CharacterVirtual) u32 {
        return self.max_num_hits;
    }

    pub fn setMaxNumHits(self: *CharacterVirtual, max_hits: u32) void {
        self.max_num_hits = max_hits;
    }

    /// Cos(angle) where angle is the maximum angle between two hits contact normals that are allowed to be merged during hit reduction. Default is around 2.5 degrees. Set to -1 to turn off.
    pub fn getHitReductionCosMaxAngle(self: *const CharacterVirtual) f32 {
        return self.hit_reduction_cos_max_angle;
    }

    pub fn setHitReductionCosMaxAngle(self: *CharacterVirtual, cos_max_angle: f32) void {
        self.hit_reduction_cos_max_angle = cos_max_angle;
    }

    /// Returns if we exceeded the maximum number of hits during the last collision check and had to discard hits based on distance.
    /// This can be used to find areas that have too complex geometry for the character to navigate properly.
    /// To solve you can either increase the max number of hits or simplify the geometry. Note that the character simulation will
    /// try to do its best to select the most relevant contacts to avoid the character from getting stuck.
    pub fn getMaxHitsExceeded(self: *const CharacterVirtual) bool {
        return self.max_hits_exceeded;
    }

    /// An extra offset applied to the shape in local space. This allows applying an extra offset to the shape in local space. Note that setting it on the fly can cause the shape to teleport into collision.
    pub fn getShapeOffset(self: *const CharacterVirtual) Vec3 {
        return self.shape_offset;
    }

    pub fn setShapeOffset(self: *CharacterVirtual, shape_offset: Vec3) void {
        self.shape_offset = shape_offset;
        self.updateInnerBodyTransform();
    }

    /// Access to the user data, can be used for anything by the application
    pub fn getUserData(self: *const CharacterVirtual) u64 {
        return self.user_data;
    }

    pub fn setUserData(self: *CharacterVirtual, user_data: u64) void {
        self.user_data = user_data;

        if (!self.inner_body_id.isInvalid())
            self.base.system.getBodyInterface().setUserData(self.inner_body_id, user_data);
    }

    /// Optional inner rigid body that proxies the character in the world. Can be used to update body properties.
    pub fn getInnerBodyID(self: *const CharacterVirtual) BodyID {
        return self.inner_body_id;
    }

    /// Get the current shape that the character is using (CharacterBase::GetShape)
    pub fn getShape(self: *const CharacterVirtual) ?*const Shape {
        return self.base.getShape();
    }

    /// Jolt's `mShape` where Jolt dereferences it
    fn currentShape(self: *const CharacterVirtual) *const Shape {
        return self.base.shape.get().?;
    }

    /// This function can be called prior to calling Update() to convert a desired velocity into a velocity that won't make the character move further onto steep slopes.
    /// This velocity can then be set on the character using SetLinearVelocity()
    /// @param desired_velocity Velocity to clamp against steep walls
    /// @return A new velocity vector that won't make the character move up steep slopes
    pub fn cancelVelocityTowardsSteepSlopes(self: *const CharacterVirtual, desired_velocity_in: Vec3) Vec3 {
        // If we're not pushing against a steep slope, return the desired velocity
        // Note: This is important as WalkStairs overrides the ground state to OnGround when its first check fails but the second succeeds
        if (self.base.ground_state == .on_ground or self.base.ground_state == .in_air)
            return desired_velocity_in;

        var desired_velocity = desired_velocity_in;
        for (self.active_contacts.items) |*c| {
            if (c.had_collision and !c.was_discarded and self.base.isSlopeTooSteep(c.surface_normal)) {
                // Note that we use the contact normal to allow for better sliding as the surface normal may be in the opposite direction of movement.
                var normal = c.contact_normal;

                // Remove normal vertical component
                normal = normal.sub(self.base.up.mulScalar(normal.dot(self.base.up)));

                // Cancel horizontal movement in opposite direction
                const dot = normal.dot(desired_velocity);
                if (dot < 0.0)
                    desired_velocity = desired_velocity.sub(normal.mulScalar(dot).divScalar(normal.lengthSq()));
            }
        }
        return desired_velocity;
    }

    /// This function is internally called by Update, WalkStairs, StickToFloor and ExtendedUpdate and is responsible for tracking if contacts are added, persisted or removed.
    /// If you want to do multiple operations on a character (e.g. first Update then WalkStairs), you can surround the code with a StartTrackingContactChanges and FinishTrackingContactChanges pair
    /// to only receive a single callback per contact on the CharacterContactListener. If you don't do this then you could for example receive a contact added callback during the Update and a
    /// contact persisted callback during WalkStairs.
    /// (Zolt: on error.OutOfMemory nothing changed, no FinishTrackingContactChanges is needed)
    pub fn startTrackingContactChanges(self: *CharacterVirtual) Allocator.Error!void {
        // Check if we're starting for the first time
        self.tracking_contact_changes += 1;
        if (self.tracking_contact_changes > 1)
            return;

        // No need to track anything if we don't have a listener
        if (Core.enable_asserts) std.debug.assert(self.listener_contacts.isEmpty()); // The listener changed between Start and Finish, Jolt's release build continues
        if (self.listener == null)
            return;

        // Zolt: undo on an allocation failure
        errdefer {
            self.listener_contacts.clearRetainingCapacity();
            self.tracking_contact_changes -= 1;
        }

        // Mark all current contacts as not seen
        try self.listener_contacts.ensureTotalCapacity(self.base.allocator, @intCast(self.active_contacts.items.len));
        for (self.active_contacts.items) |*c| {
            if (c.had_collision)
                _ = try self.listener_contacts.insert(self.base.allocator, .{ .key = c.base, .value = .{} });
        }
    }

    /// This call triggers contact removal callbacks and is used in conjunction with StartTrackingContactChanges.
    pub fn finishTrackingContactChanges(self: *CharacterVirtual) void {
        // Check if we have to do anything
        self.tracking_contact_changes -= 1;
        const count = self.tracking_contact_changes;
        if (Core.enable_asserts) std.debug.assert(count >= 0); // Called FinishTrackingContactChanges more times than StartTrackingContactChanges
        if (count > 0)
            return;

        // No need to track anything if we don't have a listener
        const listener = self.listener orelse return;

        // Since we can do multiple operations (e.g. Update followed by WalkStairs)
        // we can end up with contacts that were marked as active to the listener but that are
        // no longer in the active contact list. We go over all contacts and mark them again
        // to ensure that these lists are in sync.
        var reset_it = self.listener_contacts.iterator();
        while (reset_it.next()) |c|
            c.value.count = 0;
        for (self.active_contacts.items) |*c| {
            if (c.had_collision) {
                if (self.listener_contacts.findPtr(c.base)) |it| {
                    it.value.count = 1;
                } else if (Core.enable_asserts) {
                    @panic("CharacterVirtual: an active contact is not tracked"); // Jolt: JPH_ASSERT(it != mListenerContacts.end())
                }
            }
        }

        // Call contact removal callbacks
        var it = self.listener_contacts.iterator();
        while (it.next()) |kv| {
            if (kv.value.count == 0) {
                const c = &kv.key;
                if (!c.character_id_b.isInvalid())
                    listener.onCharacterContactRemoved(self, c.character_id_b, c.sub_shape_id_b)
                else
                    listener.onContactRemoved(self, c.body_b, c.sub_shape_id_b);
            }
        }
        self.listener_contacts.clearRetainingCapacity();
    }

    /// This is the main update function. It moves the character according to its current velocity (the character is similar to a kinematic body in the sense
    /// that you set the velocity and the character will follow unless collision is blocking the way). Note it's your own responsibility to apply gravity to the character velocity!
    /// Different surface materials (like ice) can be emulated by getting the ground material and adjusting the velocity and/or the max slope angle accordingly every frame.
    /// @param delta_time Time step to simulate.
    /// @param gravity Gravity vector (m/s^2). This gravity vector is only used when the character is standing on top of another object to apply downward force.
    /// @param broad_phase_layer_filter Filter that is used to check if the character collides with something in the broadphase.
    /// @param object_layer_filter Filter that is used to check if a character collides with a layer.
    /// @param body_filter Filter that is used to check if a character collides with a body.
    /// @param shape_filter Filter that is used to check if a character collides with a subshape (null: Jolt's default filter).
    /// @param temp_allocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
    pub fn update(self: *CharacterVirtual, delta_time: f32, gravity: Vec3, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter, temp_allocator: TempAllocator) Allocator.Error!void {
        return self.updateInternal(delta_time, gravity, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter, .shape_filter = shape_filter }, temp_allocator);
    }

    fn updateInternal(self: *CharacterVirtual, delta_time: f32, gravity: Vec3, filters: Filters, temp_allocator: TempAllocator) Allocator.Error!void {
        // If there's no delta time, we don't need to do anything
        if (delta_time <= 0.0)
            return;

        // TODO(debug renderer): draw the supporting volume pre movement (sDrawSupportingVolume)

        try self.startTrackingContactChanges();
        defer self.finishTrackingContactChanges();

        // Remember delta time for checking if we're supported by the ground
        self.last_delta_time = delta_time;

        // Slide the shape through the world
        try self.moveShape(&self.position, self.linear_velocity, delta_time, &self.active_contacts, filters, temp_allocator);

        // Determine the object that we're standing on
        try self.updateSupportingContact(false, temp_allocator);

        // Ensure that the rigid body ends up at the new position
        self.updateInnerBodyTransform();

        // If we're on the ground
        if (!self.base.ground_body_id.isInvalid() and self.mass > 0.0) {
            // Add the impulse to the ground due to gravity: P = F dt = M g dt
            const normal_dot_gravity = self.base.ground_normal.dot(gravity);
            if (normal_dot_gravity < 0.0) {
                const world_impulse = gravity.mulScalar(-(self.mass * normal_dot_gravity / gravity.length() * delta_time));
                self.base.system.getBodyInterfaceMut().addImpulseAtPosition(self.base.ground_body_id, world_impulse, self.base.ground_position);
            }
        }
    }

    /// This function will return true if the character has moved into a slope that is too steep (e.g. a vertical wall).
    /// You would call WalkStairs to attempt to step up stairs.
    /// @param linear_velocity The linear velocity that the player desired. This is used to determine if we're pushing into a step.
    pub fn canWalkStairs(self: *const CharacterVirtual, linear_velocity: Vec3) bool {
        // We can only walk stairs if we're supported
        if (!self.base.isSupported())
            return false;

        // Check if there's enough horizontal velocity to trigger a stair walk
        const horizontal_velocity = linear_velocity.sub(self.base.up.mulScalar(linear_velocity.dot(self.base.up)));
        if (horizontal_velocity.isNearZero(.{ .max_dist_sq = 1.0e-6 }))
            return false;

        // Check contacts for steep slopes
        for (self.active_contacts.items) |*c| {
            if (c.had_collision and
                !c.was_discarded and
                c.surface_normal.dot(horizontal_velocity.sub(c.linear_velocity)) < 0.0 and // Pushing into the contact
                self.base.isSlopeTooSteep(c.surface_normal)) // Slope too steep
                return true;
        }

        return false;
    }

    /// When stair walking is needed, you can call the WalkStairs function to cast up, forward and down again to try to find a valid position
    /// @param delta_time Time step to simulate.
    /// @param step_up The direction and distance to step up (this corresponds to the max step height)
    /// @param step_forward The direction and distance to step forward after the step up
    /// @param step_forward_test When running at a high frequency, inStepForward can be very small and it's likely that you hit the side of the stairs on the way down. This could produce a normal that violates the max slope angle. If this happens, we test again using this distance from the up position to see if we find a valid slope.
    /// @param step_down_extra An additional translation that is added when stepping down at the end. Allows you to step further down than up. Set to zero if you don't want this. Should be in the opposite direction of up.
    /// @param broad_phase_layer_filter Filter that is used to check if the character collides with something in the broadphase.
    /// @param object_layer_filter Filter that is used to check if a character collides with a layer.
    /// @param body_filter Filter that is used to check if a character collides with a body.
    /// @param shape_filter Filter that is used to check if a character collides with a subshape (null: Jolt's default filter).
    /// @param temp_allocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
    /// @return true if the stair walk was successful
    pub fn walkStairs(self: *CharacterVirtual, delta_time: f32, step_up: Vec3, step_forward: Vec3, step_forward_test: Vec3, step_down_extra: Vec3, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter, temp_allocator: TempAllocator) Allocator.Error!bool {
        return self.walkStairsInternal(delta_time, step_up, step_forward, step_forward_test, step_down_extra, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter, .shape_filter = shape_filter }, temp_allocator);
    }

    fn walkStairsInternal(self: *CharacterVirtual, delta_time: f32, step_up: Vec3, step_forward: Vec3, step_forward_test: Vec3, step_down_extra: Vec3, filters: Filters, temp_allocator: TempAllocator) Allocator.Error!bool {
        try self.startTrackingContactChanges();
        defer self.finishTrackingContactChanges();

        // Move up. Note that we used ValidateMovement here so that we ignore initial collisions and character padding on the sides while moving up. This makes moving up a bit easier.
        var up = step_up;
        var contact: CharacterContact = .{};
        const dummy_ignored_contacts: []const CharacterContactKey = &.{}; // IgnoredContactList dummy_ignored_contacts(inAllocator): never allocates
        if (self.validateMovement(self.position, up, &contact, dummy_ignored_contacts, filters)) {
            if (contact.fraction < 1.0e-6)
                return false; // No movement, cancel

            // Limit up movement to the first contact point
            up = up.mulScalar(contact.fraction);
        }
        const up_position = self.position.addVec3(up);

        // TODO(debug renderer): draw sweep up (sDrawWalkStairs)

        // Collect normals of steep slopes that we would like to walk stairs on.
        // We need to do this before calling MoveShape because it will update mActiveContacts.
        const character_velocity = step_forward.divScalar(delta_time);
        var steep_slope_normals: TempList(Vec3) = .{};
        try steep_slope_normals.reserve(temp_allocator, self.active_contacts.items.len);
        defer steep_slope_normals.free(temp_allocator);
        for (self.active_contacts.items) |*c| {
            if (c.had_collision and
                !c.was_discarded and
                c.surface_normal.dot(character_velocity.sub(c.linear_velocity)) < 0.0 and // Pushing into the contact
                self.base.isSlopeTooSteep(c.surface_normal)) // Slope too steep
                steep_slope_normals.append(c.surface_normal);
        }
        if (steep_slope_normals.len == 0)
            return false; // No steep slopes, cancel

        // Horizontal movement
        var new_position = up_position;
        try self.moveShape(&new_position, character_velocity, delta_time, null, filters, temp_allocator);
        const horizontal_movement = new_position.sub(up_position).toVec3();
        const horizontal_movement_sq = horizontal_movement.lengthSq();
        if (horizontal_movement_sq < 1.0e-8)
            return false; // No movement, cancel

        // Check if we made any progress towards any of the steep slopes, if not we just slid along the slope
        // so we need to cancel the stair walk or else we will move faster than we should as we've done
        // normal movement first and then stair walk.
        var made_progress = false;
        const step_forward_length = step_forward.length();
        const step_forward_normalized = step_forward.divScalar(step_forward_length);
        for (steep_slope_normals.items()) |normal| {
            var direction = self.base.up.mulScalar(normal.dot(self.base.up)).sub(normal);
            const direction_len = direction.length();
            if (direction_len > 1.0e-6) {
                direction = direction.divScalar(direction_len);
                if (direction.dot(step_forward_normalized) > 0.707107 and // If angle between forward and movement is more than 45 degrees we were mostly sliding
                    direction.dot(horizontal_movement) > 0.05 * step_forward_length) // If we did not achieve 5% of movement cancel too
                {
                    made_progress = true;
                    break;
                }
            }
        }
        if (!made_progress)
            return false;

        // TODO(debug renderer): draw horizontal sweep (sDrawWalkStairs)

        // Move down towards the floor.
        // Note that we travel the same amount down as we traveled up with the specified extra
        const down = up.negate().add(step_down_extra);
        if (!self.getFirstContactForSweep(new_position, down, &contact, dummy_ignored_contacts, filters))
            return false; // No floor found, we're in mid air, cancel stair walk
        const final_position = new_position.addVec3(down.mulScalar(contact.fraction));

        var surface_normal = contact.surface_normal;
        var too_steep = self.base.isSlopeTooSteep(surface_normal);

        // TODO(debug renderer): draw sweep down (sDrawWalkStairs)

        // Test for floor that will support the character
        if (too_steep) {
            // If no test position was provided, we cancel the stair walk
            if (step_forward_test.isNearZero(.{}))
                return false;

            // Delta time may be very small, so it may be that we hit the edge of a step and the normal is too horizontal.
            // In order to judge if the floor is flat further along the sweep, we test again for a floor at inStepForwardTest
            // and check if the normal is valid there.
            var test_position = up_position;
            try self.moveShape(&test_position, step_forward_test.divScalar(delta_time), delta_time, null, filters, temp_allocator);
            const test_horizontal_movement_sq = test_position.sub(up_position).toVec3().lengthSq();
            if (test_horizontal_movement_sq <= horizontal_movement_sq + 1.0e-8)
                return false; // We didn't move any further than in the previous test

            // TODO(debug renderer): draw 2nd sweep horizontal (sDrawWalkStairs)

            // Then sweep down
            var test_contact: CharacterContact = .{};
            if (!self.getFirstContactForSweep(test_position, down, &test_contact, dummy_ignored_contacts, filters))
                return false;

            surface_normal = test_contact.surface_normal;
            too_steep = self.base.isSlopeTooSteep(surface_normal);

            // TODO(debug renderer): draw 2nd sweep down (sDrawWalkStairs)

            if (too_steep)
                return false;
        }

        // If we don't gain any height compared to our contact then the stair walk is pointless
        if (final_position.sub(self.position).toVec3().dot(surface_normal) < 1.0e-4)
            return false;

        // Move the character to the new location
        try self.moveToContact(final_position, &contact, filters, temp_allocator);

        // Override ground state to 'on ground', it is possible that the contact normal is too steep, but in this case the inStepForwardTest has found a contact normal that is not too steep
        self.base.ground_state = .on_ground;

        return true;
    }

    /// This function can be used to artificially keep the character to the floor. Normally when a character is on a small step and starts moving horizontally, the character will
    /// lose contact with the floor because the initial vertical velocity is zero while the horizontal velocity is quite high. To prevent the character from losing contact with the floor,
    /// we do an additional collision check downwards and if we find the floor within a certain distance, we project the character onto the floor.
    /// @param step_down Max amount to project the character downwards (if no floor is found within this distance, the function will return false)
    /// @param broad_phase_layer_filter Filter that is used to check if the character collides with something in the broadphase.
    /// @param object_layer_filter Filter that is used to check if a character collides with a layer.
    /// @param body_filter Filter that is used to check if a character collides with a body.
    /// @param shape_filter Filter that is used to check if a character collides with a subshape (null: Jolt's default filter).
    /// @param temp_allocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
    /// @return True if the character was successfully projected onto the floor.
    pub fn stickToFloor(self: *CharacterVirtual, step_down: Vec3, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter, temp_allocator: TempAllocator) Allocator.Error!bool {
        return self.stickToFloorInternal(step_down, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter, .shape_filter = shape_filter }, temp_allocator);
    }

    fn stickToFloorInternal(self: *CharacterVirtual, step_down: Vec3, filters: Filters, temp_allocator: TempAllocator) Allocator.Error!bool {
        try self.startTrackingContactChanges();
        defer self.finishTrackingContactChanges();

        // Try to find the floor
        var contact: CharacterContact = .{};
        const dummy_ignored_contacts: []const CharacterContactKey = &.{}; // IgnoredContactList dummy_ignored_contacts(inAllocator): never allocates
        if (!self.getFirstContactForSweep(self.position, step_down, &contact, dummy_ignored_contacts, filters))
            return false; // If no floor found, don't update our position

        // Calculate new position
        const new_position = self.position.addVec3(step_down.mulScalar(contact.fraction));

        // TODO(debug renderer): draw sweep down (sDrawStickToFloor)

        // Move the character to the new location
        try self.moveToContact(new_position, &contact, filters, temp_allocator);
        return true;
    }

    /// Settings struct with settings for ExtendedUpdate
    pub const ExtendedUpdateSettings = struct {
        /// See StickToFloor inStepDown parameter. Can be zero to turn off.
        stick_to_floor_step_down: Vec3 = Vec3.init(0, -0.5, 0),
        /// See WalkStairs inStepUp parameter. Can be zero to turn off.
        walk_stairs_step_up: Vec3 = Vec3.init(0, 0.4, 0),
        /// See WalkStairs inStepForward parameter. Note that the parameter only indicates a magnitude, direction is taken from current velocity.
        walk_stairs_min_step_forward: f32 = 0.02,
        /// See WalkStairs inStepForwardTest parameter. Note that the parameter only indicates a magnitude, direction is taken from current velocity.
        walk_stairs_step_forward_test: f32 = 0.15,
        /// Cos(angle) where angle is the maximum angle between the ground normal in the horizontal plane and the character forward vector where we're willing to adjust the step forward test towards the contact normal.
        walk_stairs_cos_angle_forward_contact: f32 = default_walk_stairs_cos_angle_forward_contact,
        /// See WalkStairs inStepDownExtra
        walk_stairs_step_down_extra: Vec3 = Vec3.zero(),

        /// Cos(DegreesToRadians(75.0f)) (Jolt's Cos, evaluated at compile time)
        pub const default_walk_stairs_cos_angle_forward_contact: f32 = trigonometry.cos(math.degreesToRadians(75.0));
    };

    /// This function combines Update, StickToFloor and WalkStairs. This function serves as an example of how these functions could be combined.
    /// Before calling, call SetLinearVelocity to update the horizontal/vertical speed of the character, typically this is:
    /// - When on OnGround and not moving away from ground: velocity = GetGroundVelocity() + horizontal speed as input by player + optional vertical jump velocity + delta time * gravity
    /// - Else: velocity = current vertical velocity + horizontal speed as input by player + delta time * gravity
    /// @param delta_time Time step to simulate.
    /// @param gravity Gravity vector (m/s^2). This gravity vector is only used when the character is standing on top of another object to apply downward force.
    /// @param settings A structure containing settings for the algorithm.
    /// @param broad_phase_layer_filter Filter that is used to check if the character collides with something in the broadphase.
    /// @param object_layer_filter Filter that is used to check if a character collides with a layer.
    /// @param body_filter Filter that is used to check if a character collides with a body.
    /// @param shape_filter Filter that is used to check if a character collides with a subshape (null: Jolt's default filter).
    /// @param temp_allocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
    pub fn extendedUpdate(self: *CharacterVirtual, delta_time: f32, gravity: Vec3, settings: *const ExtendedUpdateSettings, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter, temp_allocator: TempAllocator) Allocator.Error!void {
        const filters: Filters = .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter, .shape_filter = shape_filter };

        try self.startTrackingContactChanges();
        defer self.finishTrackingContactChanges();

        // Update the velocity
        const desired_velocity = self.linear_velocity;
        self.linear_velocity = self.cancelVelocityTowardsSteepSlopes(desired_velocity);

        // Remember old position
        const old_position = self.position;

        // Track if on ground before the update
        var ground_to_air = self.base.isSupported();

        // Update the character position (instant, do not have to wait for physics update)
        try self.updateInternal(delta_time, gravity, filters, temp_allocator);

        // ... and that we got into air after
        if (self.base.isSupported())
            ground_to_air = false;

        // If stick to floor enabled and we're going from supported to not supported
        if (ground_to_air and !settings.stick_to_floor_step_down.isNearZero(.{})) {
            // If we're not moving up, stick to the floor
            const velocity = self.position.sub(old_position).toVec3().dot(self.base.up) / delta_time;
            if (velocity <= 1.0e-6)
                _ = try self.stickToFloorInternal(settings.stick_to_floor_step_down, filters, temp_allocator);
        }

        // If walk stairs enabled
        if (!settings.walk_stairs_step_up.isNearZero(.{})) {
            // Calculate how much we wanted to move horizontally
            var desired_horizontal_step = desired_velocity.mulScalar(delta_time);
            desired_horizontal_step = desired_horizontal_step.sub(self.base.up.mulScalar(desired_horizontal_step.dot(self.base.up)));
            const desired_horizontal_step_len = desired_horizontal_step.length();
            if (desired_horizontal_step_len > 0.0) {
                // Calculate how much we moved horizontally
                var achieved_horizontal_step = self.position.sub(old_position).toVec3();
                achieved_horizontal_step = achieved_horizontal_step.sub(self.base.up.mulScalar(achieved_horizontal_step.dot(self.base.up)));

                // Only count movement in the direction of the desired movement
                // (otherwise we find it ok if we're sliding downhill while we're trying to climb uphill)
                const step_forward_normalized = desired_horizontal_step.divScalar(desired_horizontal_step_len);
                achieved_horizontal_step = step_forward_normalized.mulScalar(math.max(@as(f32, 0.0), achieved_horizontal_step.dot(step_forward_normalized)));
                const achieved_horizontal_step_len = achieved_horizontal_step.length();

                // If we didn't move as far as we wanted and we're against a slope that's too steep
                if (achieved_horizontal_step_len + 1.0e-4 < desired_horizontal_step_len and
                    self.canWalkStairs(desired_velocity))
                {
                    // Find the contact that is most opposing our movement. Our extra walk stairs test is going to go in that direction instead
                    // of the movement direction. This allows us to step up stairs if we're moving at a big angle along the stairs.
                    var walk_stairs_direction = step_forward_normalized;
                    var max_dot = settings.walk_stairs_cos_angle_forward_contact; // Discard contacts that have an angle that is too big
                    for (self.active_contacts.items) |*c| {
                        if (c.had_collision and
                            !c.was_discarded and
                            c.surface_normal.dot(desired_velocity.sub(c.linear_velocity)) < 0.0 and // Pushing into the contact
                            self.base.isSlopeTooSteep(c.surface_normal)) // Slope too steep
                        {
                            // Remove vertical component of the surface normal and negate it so that it points in the direction of movement
                            var test_direction = self.base.up.mulScalar(c.surface_normal.dot(self.base.up)).sub(c.surface_normal);
                            const test_len = test_direction.length();
                            if (test_len > 1.0e-6) {
                                test_direction = test_direction.divScalar(test_len);
                                const dot = test_direction.dot(step_forward_normalized);
                                if (dot > max_dot) {
                                    walk_stairs_direction = test_direction;
                                    max_dot = dot;
                                }
                            }
                        }
                    }

                    // Calculate how much we should step forward
                    // Note that we clamp the step forward to a minimum distance. This is done because at very high frame rates the delta time
                    // may be very small, causing a very small step forward. If the step becomes small enough, we may not move far enough
                    // horizontally to actually end up at the top of the step.
                    const step_forward = walk_stairs_direction.mulScalar(math.max(settings.walk_stairs_min_step_forward, desired_horizontal_step_len - achieved_horizontal_step_len));
                    const step_forward_test = walk_stairs_direction.mulScalar(settings.walk_stairs_step_forward_test);

                    _ = try self.walkStairsInternal(delta_time, settings.walk_stairs_step_up, step_forward, step_forward_test, settings.walk_stairs_step_down_extra, filters, temp_allocator);
                }
            }
        }
    }

    /// This function can be used after a character has teleported to determine the new contacts with the world.
    pub fn refreshContacts(self: *CharacterVirtual, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter, temp_allocator: TempAllocator) Allocator.Error!void {
        const filters: Filters = .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter, .shape_filter = shape_filter };

        // Determine the contacts
        var contacts: TempList(CharacterContact) = .{};
        try contacts.reserve(temp_allocator, self.max_num_hits);
        defer contacts.free(temp_allocator);
        try self.getContactsAtPosition(self.position, self.linear_velocity.normalizedOr(Vec3.zero()), self.currentShape(), &contacts, filters);

        try self.storeActiveContacts(contacts.items(), temp_allocator);
    }

    /// Use the ground body ID to get an updated estimate of the ground velocity. This function can be used if the ground body has moved / changed velocity and you want a new estimate of the ground velocity.
    /// It will not perform collision detection, so is less accurate than RefreshContacts but a lot faster.
    pub fn updateGroundVelocity(self: *CharacterVirtual) void {
        var lock: BodyLockRead = .init(self.base.system.getBodyLockInterface(), self.base.ground_body_id);
        defer lock.deinit();
        if (lock.succeededAndIsInBroadPhase()) {
            const body = lock.getBody();

            // Get adjusted body velocity
            const velocity = self.getAdjustedBodyVelocity(body);

            // Calculate the ground velocity
            self.base.ground_velocity = self.calculateCharacterGroundVelocity(body.getCenterOfMassPosition(), velocity.linear, velocity.angular, self.last_delta_time);
        }
    }

    /// Switch the shape of the character (e.g. for stance).
    /// @param new_shape The shape to switch to.
    /// @param max_penetration_depth When max_penetration_depth is not FLT_MAX, it checks if the new shape collides before switching shape. This is the max penetration we're willing to accept after the switch.
    /// @param broad_phase_layer_filter Filter that is used to check if the character collides with something in the broadphase.
    /// @param object_layer_filter Filter that is used to check if a character collides with a layer.
    /// @param body_filter Filter that is used to check if a character collides with a body.
    /// @param shape_filter Filter that is used to check if a character collides with a subshape (null: Jolt's default filter).
    /// @param temp_allocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
    /// @return Returns true if the switch succeeded.
    pub fn setShape(self: *CharacterVirtual, new_shape: ?*const Shape, max_penetration_depth: f32, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter, temp_allocator: TempAllocator) Allocator.Error!bool {
        const filters: Filters = .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter, .shape_filter = shape_filter };

        if (self.base.shape.get() == null) { // Jolt also checks mSystem == nullptr, which Zolt's non-optional pointer cannot be
            // It hasn't been initialized yet
            self.base.shape.set(new_shape);
            return true;
        }

        if (new_shape != self.base.shape.get() and new_shape != null) {
            if (max_penetration_depth < math.flt_max) {
                // Check collision around the new shape
                var contacts: TempList(CharacterContact) = .{};
                try contacts.reserve(temp_allocator, self.max_num_hits);
                defer contacts.free(temp_allocator);
                try self.getContactsAtPosition(self.position, self.linear_velocity.normalizedOr(Vec3.zero()), new_shape.?, &contacts, filters);

                // Test if this results in penetration, if so cancel the transition
                for (contacts.items()) |*c| {
                    if (c.distance < -max_penetration_depth and
                        !c.is_sensor_b)
                        return false;
                }

                try self.storeActiveContacts(contacts.items(), temp_allocator);
            }

            // Set new shape
            self.base.shape.set(new_shape);
        }

        return self.base.shape.get() == new_shape;
    }

    /// Updates the shape of the inner rigid body. Should be called after a successful call to SetShape.
    pub fn setInnerBodyShape(self: *CharacterVirtual, new_shape: *const Shape) Allocator.Error!void {
        try self.base.system.getBodyInterface().setShape(self.inner_body_id, new_shape, false, .dont_activate);
    }

    /// Get the transformed shape that represents the volume of the character, can be used for collision checks.
    /// (the caller owns the shape reference of the result: `deinit` it)
    pub fn getTransformedShape(self: *const CharacterVirtual) TransformedShape {
        return .init(self.getCenterOfMassPosition(), self.rotation, self.base.shape.get(), self.inner_body_id, .{});
    }

    /// @brief Get all contacts for the character at a particular location.
    /// When colliding with another character virtual, this pointer will be provided through CollideShapeCollector::SetUserContext before adding a hit.
    /// @param position Position to test, note that this position will be corrected for the character padding.
    /// @param rotation Rotation at which to test the shape.
    /// @param movement_direction A hint in which direction the character is moving, will be used to calculate a proper normal.
    /// @param max_separation_distance How much distance around the character you want to report contacts in (can be 0 to match the character exactly).
    /// @param query_shape Shape to test collision with.
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
    /// @param collector Collision collector that receives the collision results.
    /// @param broad_phase_layer_filter Filter that is used to check if the character collides with something in the broadphase.
    /// @param object_layer_filter Filter that is used to check if a character collides with a layer.
    /// @param body_filter Filter that is used to check if a character collides with a body.
    /// @param shape_filter Filter that is used to check if a character collides with a subshape (null: Jolt's default filter).
    /// (Zolt: error.OutOfMemory when the heap fallback of the internal edge removal fails, the collector then has a partial result)
    pub fn checkCollision(self: *const CharacterVirtual, position: RVec3, rotation: Quat, movement_direction: Vec3, max_separation_distance: f32, query_shape: *const Shape, base_offset: RVec3, collector: *CollideShapeCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter, body_filter: *const BodyFilter, shape_filter: ?*ShapeFilter) Allocator.Error!void {
        // Query shape transform
        const transform = self.getCenterOfMassTransformAt(position, rotation, query_shape);

        // Settings for collide shape
        var settings: CollideShapeSettings = .{};
        settings.back_face_mode = self.back_face_mode;
        settings.active_edge_movement_direction = movement_direction;
        settings.max_separation_distance = self.character_padding + max_separation_distance;
        settings.active_edge_mode = .collide_only_with_active;

        // Body filter
        var chained_body_filter: IgnoreSingleBodyFilterChained = .init(self.inner_body_id, body_filter);

        // Select the right function and collide shape
        const query = self.base.system.getNarrowPhaseQuery();
        const query_filters: NarrowPhaseQuery.QueryFilters = .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = &chained_body_filter.base, .shape_filter = shape_filter };
        if (self.enhanced_internal_edge_removal)
            try query.collideShapeWithInternalEdgeRemoval(self.base.allocator, query_shape, Vec3.one(), transform, &settings, base_offset, collector, query_filters)
        else
            query.collideShape(query_shape, Vec3.one(), transform, &settings, base_offset, collector, query_filters);

        // Also collide with other characters
        if (self.character_vs_character_collision) |character_vs_character_collision| {
            collector.setContext(null); // We're no longer colliding with a transformed shape, reset
            character_vs_character_collision.collideCharacter(self, transform, &settings, base_offset, collector);
        }
    }

    /// Get the character settings that can recreate this character (consumes a CharacterID like Jolt's default member
    /// initializer of the returned settings; the caller deinits the settings, they hold references to the shapes)
    pub fn getCharacterVirtualSettings(self: *const CharacterVirtual) CharacterVirtualSettings {
        var settings: CharacterVirtualSettings = .init(self.base.allocator);
        settings.base.up = self.base.up;
        settings.base.supporting_volume = self.base.supporting_volume;
        settings.base.max_slope_angle = trigonometry.acos(self.base.cos_max_slope_angle);
        settings.base.enhanced_internal_edge_removal = self.enhanced_internal_edge_removal;
        settings.base.shape.set(self.base.shape.get());
        settings.id = self.id;
        settings.mass = self.mass;
        settings.max_strength = self.max_strength;
        settings.shape_offset = self.shape_offset;
        settings.back_face_mode = self.back_face_mode;
        settings.predictive_contact_distance = self.predictive_contact_distance;
        settings.max_collision_iterations = self.max_collision_iterations;
        settings.max_constraint_iterations = self.max_constraint_iterations;
        settings.min_time_remaining = self.min_time_remaining;
        settings.collision_tolerance = self.collision_tolerance;
        settings.character_padding = self.character_padding;
        settings.max_num_hits = self.max_num_hits;
        settings.hit_reduction_cos_max_angle = self.hit_reduction_cos_max_angle;
        settings.penetration_recovery_speed = self.penetration_recovery_speed;
        var lock: BodyLockRead = .init(self.base.system.getBodyLockInterface(), self.inner_body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            settings.inner_body_shape.set(body.getShape());
            settings.inner_body_id_override = body.getID();
            settings.inner_body_layer = body.getObjectLayer();
        }
        return settings;
    }

    // Saving / restoring state for replay
    pub fn saveState(self: *const CharacterVirtual, stream: *StateRecorder) void {
        CharacterBase.impl.saveState(&self.base, stream);

        stream.write(self.position);
        stream.write(self.rotation);
        stream.write(self.linear_velocity);
        stream.write(self.last_delta_time);
        stream.write(self.max_hits_exceeded);

        // Store contacts that had collision, we're using it at the beginning of the step in CancelVelocityTowardsSteepSlopes
        var num_contacts: u32 = 0;
        for (self.active_contacts.items) |*c| {
            if (c.had_collision)
                num_contacts += 1;
        }
        stream.write(num_contacts);
        for (self.active_contacts.items) |*c| {
            if (c.had_collision)
                c.saveState(stream);
        }
    }

    /// (Zolt: error.OutOfMemory when the active contacts cannot grow to the restored count; the base state, position,
    /// rotation and velocity are restored then, the active contacts are unchanged)
    pub fn restoreState(self: *CharacterVirtual, stream: *StateRecorder) Allocator.Error!void {
        try CharacterBase.impl.restoreState(&self.base, stream);

        stream.read(&self.position);
        stream.read(&self.rotation);
        stream.read(&self.linear_velocity);
        stream.read(&self.last_delta_time);
        readBool(stream, &self.max_hits_exceeded);

        // When validating remove contacts that don't have collision since we didn't save them
        if (stream.isValidating()) {
            var i = self.active_contacts.items.len;
            while (i > 0) {
                i -= 1;
                if (!self.active_contacts.items[i].had_collision)
                    _ = self.active_contacts.orderedRemove(i);
            }
        }

        var num_contacts: u32 = @intCast(self.active_contacts.items.len);
        stream.read(&num_contacts);

        // mActiveContacts.resize(num_contacts): new contacts are default constructed, existing ones keep the members that
        // RestoreState does not read (mIsBackFacingContact)
        const old_len = self.active_contacts.items.len;
        if (num_contacts > old_len) {
            try self.active_contacts.ensureTotalCapacity(self.base.allocator, num_contacts);
            self.active_contacts.appendNTimesAssumeCapacity(.{}, num_contacts - old_len);
        } else {
            self.active_contacts.shrinkRetainingCapacity(num_contacts);
        }
        for (self.active_contacts.items) |*c|
            c.restoreState(stream);
    }

    /// Access to the internal list of contacts that the character has found.
    /// Note that only contacts that have their mHadCollision flag set are actual contacts.
    pub fn getActiveContacts(self: *const CharacterVirtual) []const CharacterContact {
        return self.active_contacts.items;
    }

    /// Check if the character is currently in contact with or has collided with another body in the last operation (e.g. Update or WalkStairs)
    pub fn hasCollidedWith(self: *const CharacterVirtual, body: BodyID) bool {
        for (self.active_contacts.items) |*c| {
            if (c.had_collision and c.base.body_b.eql(body))
                return true;
        }
        return false;
    }

    /// Check if the character is currently in contact with or has collided with another character in the last time step (e.g. Update or WalkStairs)
    /// (Jolt's HasCollidedWith(const CharacterID &))
    pub fn hasCollidedWithCharacterID(self: *const CharacterVirtual, character_id: CharacterID) bool {
        for (self.active_contacts.items) |*c| {
            if (c.had_collision and c.base.character_id_b.eql(character_id))
                return true;
        }
        return false;
    }

    /// Check if the character is currently in contact with or has collided with another character in the last time step (e.g. Update or WalkStairs)
    /// (Jolt's HasCollidedWith(const CharacterVirtual *))
    pub fn hasCollidedWithCharacter(self: *const CharacterVirtual, character: *const CharacterVirtual) bool {
        return self.hasCollidedWithCharacterID(character.getID());
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Private

    // Sorting predicate for making contact order deterministic (ContactOrderingPredicate)
    fn contactOrderingLessThan(_: void, lhs: CharacterContact, rhs: CharacterContact) bool {
        if (!lhs.base.body_b.eql(rhs.base.body_b))
            return lhs.base.body_b.lessThan(rhs.base.body_b);

        if (!lhs.base.character_id_b.eql(rhs.base.character_id_b))
            return lhs.base.character_id_b.lessThan(rhs.base.character_id_b);

        return lhs.base.sub_shape_id_b.getValue() < rhs.base.sub_shape_id_b.getValue();
    }

    // A constraint that limits the movement of the character (an extern struct with Jolt's layout: temp memory)
    const Constraint = extern struct {
        /// Contact that this constraint was generated from
        contact: *CharacterContact,
        /// Calculated time of impact (can be negative if penetrating)
        toi: f32 = 0.0,
        /// Velocity of the contact projected on the contact normal (negative if separating)
        projected_velocity: f32 = 0.0,
        /// Velocity of the contact (can contain a corrective velocity to resolve penetration)
        linear_velocity: Vec3 = Vec3.zero(),
        /// Plane around the origin that describes how far we can displace (from the origin)
        plane: Plane = Plane.init(Vec3.zero(), 0.0),
        /// If this constraint belongs to a steep slope
        is_steep_slope: bool = false,

        comptime {
            if (@sizeOf(usize) == 8) std.debug.assert(@sizeOf(Constraint) == 64); // Jolt's sizeof
        }
    };

    // The sort of SolveConstraints: the constraints in order of time of impact
    fn constraintLessThan(_: void, lhs: *Constraint, rhs: *Constraint) bool {
        // If both constraints hit at t = 0 then order the one that will push the character furthest first
        // Note that because we add velocity to penetrating contacts, this will also resolve contacts that penetrate the most
        if (lhs.toi <= 0.0 and rhs.toi <= 0.0)
            return lhs.projected_velocity > rhs.projected_velocity;

        // Then sort on time of impact
        if (lhs.toi != rhs.toi)
            return lhs.toi < rhs.toi;

        // As a tie breaker sort static first so it has the most influence
        return @intFromEnum(lhs.contact.motion_type_b) > @intFromEnum(rhs.contact.motion_type_b);
    }

    // Collision collector that collects hits for CollideShape
    const ContactCollector = struct {
        pub const overrides = .{ .setUserData, .addHit };

        base: CollideShapeCollector,
        base_offset: RVec3,
        up: Vec3,
        system: *PhysicsSystem,
        character: *const CharacterVirtual,
        other_character: ?*const CharacterVirtual = null,
        contacts: *TempList(CharacterContact),
        max_hits: u32,
        hit_reduction_cos_max_angle: f32,
        max_hits_exceeded: bool = false,

        fn init(system: *PhysicsSystem, character: *const CharacterVirtual, max_hits: u32, hit_reduction_cos_max_angle: f32, up: Vec3, base_offset: RVec3, out_contacts: *TempList(CharacterContact)) ContactCollector {
            return .{ .base = .init(ContactCollector), .base_offset = base_offset, .up = up, .system = system, .character = character, .contacts = out_contacts, .max_hits = max_hits, .hit_reduction_cos_max_angle = hit_reduction_cos_max_angle };
        }

        pub fn setUserData(self: *ContactCollector, user_data: u64) void {
            self.other_character = @ptrFromInt(user_data);
        }

        pub fn addHit(self: *ContactCollector, result: *const CollideShapeResult) void {
            const contacts = self.contacts;

            // If we exceed our contact limit, try to clean up near-duplicate contacts
            if (contacts.len == self.max_hits) {
                // Flag that we hit this code path
                self.max_hits_exceeded = true;

                // Check if we can do reduction
                if (self.hit_reduction_cos_max_angle > -1.0) {
                    // The reserved block does not move, so references into it stay valid while the list shrinks
                    const buf = contacts.buffer.items;

                    // Loop all contacts and find similar contacts
                    var i: isize = @as(isize, @intCast(contacts.len)) - 1;
                    while (i >= 0) : (i -= 1) {
                        const contact_i = &buf[@intCast(i)];
                        var j: isize = i - 1;
                        while (j >= 0) : (j -= 1) {
                            const contact_j = &buf[@intCast(j)];
                            if (contact_i.isSameBody(contact_j) and
                                contact_i.contact_normal.dot(contact_j.contact_normal) > self.hit_reduction_cos_max_angle) // Very similar contact normals
                            {
                                // Remove the contact with the biggest distance
                                const i_is_last = i == @as(isize, @intCast(contacts.len)) - 1;
                                if (contact_i.distance > contact_j.distance) {
                                    // Remove i
                                    if (!i_is_last)
                                        contact_i.* = contacts.last().*;
                                    contacts.pop();

                                    // Break out of the loop, i is now an element that we already processed
                                    break;
                                } else {
                                    // Remove j
                                    contact_j.* = contacts.last().*;
                                    contacts.pop();

                                    // If i was the last element, we just moved it into position j. Break out of the loop, we'll see it again later.
                                    if (i_is_last)
                                        break;
                                }
                            }
                        }
                    }
                }

                if (contacts.len == self.max_hits) {
                    // There are still too many hits, give up!
                    self.base.forceEarlyOut();
                    return;
                }
            }

            if (result.body_id2.isInvalid()) {
                // Assuming this is a hit against another character
                std.debug.assert(self.other_character != null);

                // Create contact with other character
                contacts.append(.{});
                const contact = contacts.last();
                fillCharacterContactProperties(contact, self.other_character.?, self.base_offset, result);
                contact.fraction = 0.0;
            } else {
                // Create contact with other body
                var lock: BodyLockRead = .init(self.system.getBodyLockInterface(), result.body_id2);
                defer lock.deinit();
                if (lock.succeededAndIsInBroadPhase()) {
                    contacts.append(.{});
                    const contact = contacts.last();
                    fillContactProperties(self.character, contact, lock.getBody(), self.up, self.base_offset, self.base.getContext().?, result);
                    contact.fraction = 0.0;
                }
            }
        }
    };

    // A collision collector that collects hits for CastShape
    fn ContactCastCollector(comptime ignore_initial_overlap: bool) type {
        return struct {
            const Self = @This();
            pub const overrides = .{ .setUserData, .addHit };

            base: CastShapeCollector,
            base_offset: RVec3,
            displacement: Vec3,
            up: Vec3,
            system: *PhysicsSystem,
            character: *const CharacterVirtual,
            other_character: ?*const CharacterVirtual = null,
            ignored_contacts: []const CharacterContactKey,
            contact: *CharacterContact,

            fn init(system: *PhysicsSystem, character: *const CharacterVirtual, displacement: Vec3, up: Vec3, ignored_contacts: []const CharacterContactKey, base_offset: RVec3, out_contact: *CharacterContact) Self {
                return .{ .base = .init(Self), .base_offset = base_offset, .displacement = displacement, .up = up, .system = system, .character = character, .ignored_contacts = ignored_contacts, .contact = out_contact };
            }

            pub fn setUserData(self: *Self, user_data: u64) void {
                self.other_character = @ptrFromInt(user_data);
            }

            pub fn addHit(self: *Self, result: *const ShapeCastResult) void {
                if (result.fraction < self.contact.fraction and // Since we're doing checks against the world and against characters, we may get a hit with a higher fraction than the previous hit
                    (!ignore_initial_overlap or result.fraction > 0.0) and // Ignore collisions at fraction = 0
                    result.base.penetration_axis.dot(self.displacement) > 0.0) // Ignore penetrations that we're moving away from
                {
                    // Test if this contact should be ignored
                    for (self.ignored_contacts) |c| {
                        if (c.body_b.eql(result.base.body_id2) and c.sub_shape_id_b.eql(result.base.sub_shape_id2))
                            return;
                    }

                    var contact: CharacterContact = .{};

                    if (result.base.body_id2.isInvalid()) {
                        // Assuming this is a hit against another character
                        std.debug.assert(self.other_character != null);

                        // Create contact with other character
                        fillCharacterContactProperties(&contact, self.other_character.?, self.base_offset, &result.base);
                    } else {
                        // Lock body only while we fetch contact properties
                        var lock: BodyLockRead = .init(self.system.getBodyLockInterface(), result.base.body_id2);
                        defer lock.deinit();
                        if (!lock.succeededAndIsInBroadPhase())
                            return;

                        // Sweeps don't result in OnContactAdded callbacks so we can ignore sensors here
                        const body = lock.getBody();
                        if (body.isSensor())
                            return;

                        // Convert the hit result into a contact
                        fillContactProperties(self.character, &contact, body, self.up, self.base_offset, self.base.getContext().?, &result.base);
                    }

                    contact.fraction = result.fraction;

                    // Check if the contact that will make us penetrate more than the allowed tolerance
                    if (contact.distance + contact.contact_normal.dot(self.displacement) < -self.character.collision_tolerance and
                        self.character.validateContact(&contact))
                    {
                        self.contact.* = contact;
                        self.base.updateEarlyOutFraction(contact.fraction);
                    }
                }
            }
        };
    }

    // Move the shape from position and try to displace it by velocity * delta_time, this will try to slide the shape along the world geometry
    fn moveShape(self: *CharacterVirtual, position: *RVec3, velocity: Vec3, delta_time: f32, out_active_contacts: ?*ContactList, filters: Filters, temp_allocator: TempAllocator) Allocator.Error!void {
        if (DeterminismLog.enabled) DeterminismLog.log(.{ "CharacterVirtual::MoveShape: pos: ", position.*, " vel: ", velocity, " dt: ", delta_time });

        const movement_direction = velocity.normalizedOr(Vec3.zero());

        var time_remaining = delta_time;
        var iteration: u32 = 0;
        while (iteration < self.max_collision_iterations and time_remaining >= self.min_time_remaining) : (iteration += 1) {
            if (DeterminismLog.enabled) DeterminismLog.log(.{ "iter: ", iteration, " time: ", time_remaining });

            // Determine contacts in the neighborhood
            var contacts: TempList(CharacterContact) = .{};
            try contacts.reserve(temp_allocator, self.max_num_hits);
            defer contacts.free(temp_allocator);
            try self.getContactsAtPosition(position.*, movement_direction, self.currentShape(), &contacts, filters);

            if (DeterminismLog.enabled) {
                for (contacts.items()) |*c|
                    DeterminismLog.log(.{ "contact: ", c.position, " vel: ", c.linear_velocity, " cnormal: ", c.contact_normal, " snormal: ", c.surface_normal, " dist: ", c.distance, " fraction: ", c.fraction, " body: ", c.base.body_b, " subshape: ", c.base.sub_shape_id_b });
            }

            // Remove contacts with the same body that have conflicting normals
            var ignored_contacts: TempList(CharacterContactKey) = .{};
            try ignored_contacts.reserve(temp_allocator, contacts.len);
            defer ignored_contacts.free(temp_allocator);
            self.removeConflictingContacts(&contacts, &ignored_contacts);

            // Convert contacts into constraints
            var constraints: TempList(Constraint) = .{};
            try constraints.reserve(temp_allocator, contacts.len * 2);
            defer constraints.free(temp_allocator);
            self.determineConstraints(contacts.items(), delta_time, &constraints);

            // TODO(debug renderer): draw the constraints of iteration 0 (sDrawConstraints)

            // Solve the displacement using these constraints
            const solved = try self.solveConstraints(velocity, delta_time, time_remaining, &constraints, &ignored_contacts, temp_allocator);
            var displacement = solved.displacement;
            var time_simulated = solved.time_simulated;

            // Store the contacts now that the colliding ones have been marked
            if (out_active_contacts) |out|
                try self.assignContacts(out, contacts.items());

            // Do a sweep to test if the path is really unobstructed
            var cast_contact: CharacterContact = .{};
            if (self.validateMovement(position.*, displacement, &cast_contact, ignored_contacts.items(), filters)) {
                displacement = displacement.mulScalar(cast_contact.fraction);
                time_simulated *= cast_contact.fraction;
            }

            // Update the position
            position.* = position.addVec3(displacement);
            time_remaining -= time_simulated;

            // If the displacement during this iteration was too small we assume we cannot further progress this update
            if (displacement.lengthSq() < 1.0e-8)
                break;
        }
    }

    /// `assign(begin, end)` of a ContactList: atomic (on error.OutOfMemory the list is unchanged)
    fn assignContacts(self: *const CharacterVirtual, list: *ContactList, contacts: []const CharacterContact) Allocator.Error!void {
        try list.ensureTotalCapacity(self.base.allocator, contacts.len);
        list.clearRetainingCapacity();
        list.appendSliceAssumeCapacity(contacts);
    }

    // Ask the callback if contact is a valid contact point
    fn validateContact(self: *const CharacterVirtual, contact: *const CharacterContact) bool {
        const listener = self.listener orelse return true;

        if (contact.character_b != null)
            return listener.onCharacterContactValidate(self, contact)
        else
            return listener.onContactValidate(self, contact);
    }

    // Trigger the contact callback for contact and get the contact settings
    fn contactAdded(self: *CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) Allocator.Error!void {
        if (self.listener) |listener| {
            // Check if we already know this contact
            if (self.listener_contacts.findPtr(contact.base)) |it| {
                // Max 1 contact persisted callback
                it.value.count += 1;
                if (it.value.count == 1) {
                    if (contact.character_b != null)
                        listener.onCharacterContactPersisted(self, contact, settings)
                    else
                        listener.onContactPersisted(self, contact, settings);
                    it.value.settings = settings.*;
                } else {
                    // Reuse the settings from the last call
                    settings.* = it.value.settings;
                }
            } else {
                // New contact (Zolt: entered before the callback, so that an allocation failure happens before it; the
                // listener cannot observe the map, and nothing inserts into it during the callback)
                const inserted = try self.listener_contacts.insert(self.base.allocator, .{ .key = contact.base, .value = .{} });
                if (contact.character_b != null)
                    listener.onCharacterContactAdded(self, contact, settings)
                else
                    listener.onContactAdded(self, contact, settings);
                inserted.ptr.value = .{ .settings = settings.*, .count = 0 };
            }
        }
    }

    // Tests the shape for collision around position
    fn getContactsAtPosition(self: *CharacterVirtual, position: RVec3, movement_direction: Vec3, query_shape: *const Shape, out_contacts: *TempList(CharacterContact), filters: Filters) Allocator.Error!void {
        // Remove previous results
        out_contacts.clear();

        // Body filter
        var chained_body_filter: IgnoreSingleBodyFilterChained = .init(self.inner_body_id, filters.body_filter);

        // Collide shape
        var collector: ContactCollector = .init(self.base.system, self, self.max_num_hits, self.hit_reduction_cos_max_angle, self.base.up, self.position, out_contacts);
        try self.checkCollision(position, self.rotation, movement_direction, self.predictive_contact_distance, query_shape, self.position, &collector.base, filters.broad_phase_layer_filter, filters.object_layer_filter, &chained_body_filter.base, filters.shape_filter);

        // The broadphase bounding boxes will not be deterministic, which means that the order in which the contacts are received by the collector is not deterministic.
        // Therefore we need to sort the contacts to preserve determinism. Note that currently this will fail if we exceed mMaxNumHits hits.
        quickSort(CharacterContact, out_contacts.items(), {}, contactOrderingLessThan);

        // Flag if we exceeded the max number of hits
        self.max_hits_exceeded = collector.max_hits_exceeded;

        // Reduce distance to contact by padding to ensure we stay away from the object by a little margin
        // (this will make collision detection cheaper - especially for sweep tests as they won't hit the surface if we're properly sliding)
        for (out_contacts.items()) |*c| {
            c.distance -= self.character_padding;

            if (c.character_b) |character_b|
                c.distance -= character_b.character_padding;
        }
    }

    // Remove penetrating contacts with the same body that have conflicting normals, leaving these will make the character mover get stuck
    fn removeConflictingContacts(self: *const CharacterVirtual, contacts: *TempList(CharacterContact), out_ignored_contacts: *TempList(CharacterContactKey)) void {
        // Only use this algorithm if we're penetrating further than this (due to numerical precision issues we can always penetrate a little bit and we don't want to discard contacts if they just have a tiny penetration)
        // We do need to account for padding (see GetContactsAtPosition) that is removed from the contact distances, to compensate we add it to the cMinRequiredPenetration
        const min_required_penetration = 1.25 * self.character_padding;

        // Discard conflicting penetrating contacts
        var c1: usize = 0;
        while (c1 < contacts.len) : (c1 +%= 1) {
            const contact1 = &contacts.buffer.items[c1];
            if (contact1.distance <= -min_required_penetration) { // Only for penetrations
                var c2: usize = c1 + 1;
                while (c2 < contacts.len) : (c2 +%= 1) {
                    const contact2 = &contacts.buffer.items[c2];
                    if (contact1.isSameBody(contact2) and
                        contact2.distance <= -min_required_penetration and // Only for penetrations
                        contact1.contact_normal.dot(contact2.contact_normal) < 0.0) // Only opposing normals
                    {
                        // Discard contacts with the least amount of penetration
                        if (contact1.distance < contact2.distance) {
                            // Discard the 2nd contact
                            out_ignored_contacts.append(contact2.base);
                            contacts.orderedRemove(c2);
                            c2 -%= 1;
                        } else {
                            // Discard the first contact
                            out_ignored_contacts.append(contact1.base);
                            contacts.orderedRemove(c1);
                            c1 -%= 1;
                            break;
                        }
                    }
                }
            }
        }
    }

    // Convert contacts into constraints. The character is assumed to start at the origin and the constraints are planes around the origin that confine the movement of the character.
    fn determineConstraints(self: *const CharacterVirtual, contacts: []CharacterContact, delta_time: f32, out_constraints: *TempList(Constraint)) void {
        for (contacts) |*c| {
            var contact_velocity = c.linear_velocity;

            // Penetrating contact: Add a contact velocity that pushes the character out at the desired speed
            if (c.distance < 0.0)
                contact_velocity = contact_velocity.sub(c.contact_normal.mulScalar(c.distance).mulScalar(self.penetration_recovery_speed).divScalar(delta_time));

            // Convert to a constraint
            out_constraints.append(.{ .contact = c });
            const constraint = out_constraints.last();
            constraint.linear_velocity = contact_velocity;
            constraint.plane = Plane.init(c.contact_normal, c.distance);

            // Next check if the angle is too steep and if it is add an additional constraint that holds the character back
            if (self.base.isSlopeTooSteep(c.surface_normal)) {
                // Only take planes that point up.
                // Note that we use the contact normal to allow for better sliding as the surface normal may be in the opposite direction of movement.
                const dot = c.contact_normal.dot(self.base.up);
                if (dot > 1.0e-3) { // Add a little slack, if the normal is perfectly horizontal we already have our vertical plane.
                    // Mark the slope constraint as steep
                    constraint.is_steep_slope = true;

                    // Make horizontal normal
                    const normal = c.contact_normal.sub(self.base.up.mulScalar(dot)).normalized();

                    // Create a secondary constraint that blocks horizontal movement
                    out_constraints.append(.{ .contact = c });
                    const vertical_constraint = out_constraints.last();
                    vertical_constraint.linear_velocity = normal.mulScalar(c.linear_velocity.dot(normal)); // Project the contact velocity on the new normal so that both planes push at an equal rate. We ignore velocity added to push characters out of collision as that can get characters stuck if they are surrounded on all sides by steep slopes.
                    vertical_constraint.plane = Plane.init(normal, c.distance / normal.dot(c.contact_normal)); // Calculate the distance we have to travel horizontally to hit the contact plane
                }
            }
        }
    }

    /// The out parameters of SolveConstraints
    const SolveResult = struct {
        displacement: Vec3,
        time_simulated: f32,
    };

    // Use the constraints to solve the displacement of the character. This will slide the character on the planes around the origin for as far as possible.
    fn solveConstraints(self: *CharacterVirtual, velocity_in: Vec3, delta_time: f32, time_remaining_in: f32, constraints: *TempList(Constraint), ignored_contacts: *TempList(CharacterContactKey), temp_allocator: TempAllocator) Allocator.Error!SolveResult {
        var time_remaining = time_remaining_in;

        // If there are no constraints we can immediately move to our target
        if (constraints.len == 0)
            return .{ .displacement = velocity_in.mulScalar(time_remaining), .time_simulated = time_remaining };

        // Create array that holds the constraints in order of time of impact (sort will happen later)
        var sorted_constraints: TempList(*Constraint) = .{};
        try sorted_constraints.resize(temp_allocator, constraints.len);
        defer sorted_constraints.free(temp_allocator);
        for (sorted_constraints.items(), constraints.items()) |*sorted, *c|
            sorted.* = c;

        // This is the velocity we use for the displacement, if we hit something it will be shortened
        var velocity = velocity_in;

        // Keep track of the last velocity that was applied to the character so that we can detect when the velocity reverses
        var last_velocity = velocity_in;

        // Start with no displacement
        var out_displacement = Vec3.zero();
        var out_time_simulated: f32 = 0.0;

        // These are the contacts that we hit previously without moving a significant distance
        var previous_contacts: TempList(*Constraint) = .{};
        try previous_contacts.resize(temp_allocator, self.max_constraint_iterations);
        defer previous_contacts.free(temp_allocator);
        var num_previous_contacts: usize = 0;

        // Loop for a max amount of iterations
        var iteration: u32 = 0;
        while (iteration < self.max_constraint_iterations) : (iteration += 1) {
            // Calculate time of impact for all constraints
            for (constraints.items()) |*c| {
                // Project velocity on plane direction
                c.projected_velocity = c.plane.getNormal().dot(c.linear_velocity.sub(velocity));
                if (c.projected_velocity < 1.0e-6) {
                    c.toi = math.flt_max;
                } else {
                    // Distance to plane
                    const dist = c.plane.signedDistance(out_displacement);

                    if (dist - c.projected_velocity * time_remaining > -1.0e-4) {
                        // Too little penetration, accept the movement
                        c.toi = math.flt_max;
                    } else {
                        // Calculate time of impact
                        c.toi = math.max(@as(f32, 0.0), dist / c.projected_velocity);
                    }
                }
            }

            // Sort constraints on proximity
            quickSort(*Constraint, sorted_constraints.items(), {}, constraintLessThan);

            // Find the first valid constraint
            var found_constraint: ?*Constraint = null;
            for (sorted_constraints.items()) |c| {
                // Take the first contact and see if we can reach it
                if (c.toi >= time_remaining) {
                    // We can reach our goal!
                    out_displacement = out_displacement.add(velocity.mulScalar(time_remaining));
                    out_time_simulated += time_remaining;
                    return .{ .displacement = out_displacement, .time_simulated = out_time_simulated };
                }

                // Test if this contact was discarded by the contact callback before
                if (c.contact.was_discarded)
                    continue;

                // Handle the contact
                if (!c.contact.had_collision and
                    !(try self.handleContact(velocity, c, delta_time)))
                {
                    // Constraint should be ignored, remove it from the list
                    c.contact.was_discarded = true;

                    // Mark it as ignored for ValidateMovement
                    ignored_contacts.append(c.contact.base);
                    continue;
                }

                // Cancel velocity of constraint if it cannot push the character
                if (!c.contact.can_push_character)
                    c.linear_velocity = Vec3.zero();

                // We found the first constraint that we want to collide with
                found_constraint = c;
                break;
            }

            const constraint = found_constraint orelse {
                // All constraints were discarded, we can reach our goal!
                out_displacement = out_displacement.add(velocity.mulScalar(time_remaining));
                out_time_simulated += time_remaining;
                return .{ .displacement = out_displacement, .time_simulated = out_time_simulated };
            };

            // Move to the contact
            out_displacement = out_displacement.add(velocity.mulScalar(constraint.toi));
            time_remaining -= constraint.toi;
            out_time_simulated += constraint.toi;

            // If there's not enough time left to be simulated, bail
            if (time_remaining < self.min_time_remaining)
                return .{ .displacement = out_displacement, .time_simulated = out_time_simulated };

            // If we've moved significantly, clear all previous contacts
            if (constraint.toi > 1.0e-4)
                num_previous_contacts = 0;

            // Get the normal of the plane we're hitting
            const plane_normal = constraint.plane.getNormal();

            // If we're hitting a steep slope we cancel the velocity towards the slope first so that we don't end up sliding up the slope
            // (we may hit the slope before the vertical wall constraint we added which will result in a small movement up causing jitter in the character movement)
            if (constraint.is_steep_slope) {
                // We're hitting a steep slope, create a vertical plane that blocks any further movement up the slope (note: not normalized)
                const vertical_plane_normal = plane_normal.sub(self.base.up.mulScalar(plane_normal.dot(self.base.up)));

                // Get the relative velocity between the character and the constraint
                const relative_velocity = velocity.sub(constraint.linear_velocity);

                // Remove velocity towards the slope
                velocity = velocity.sub(vertical_plane_normal.mulScalar(math.min(@as(f32, 0.0), relative_velocity.dot(vertical_plane_normal))).divScalar(vertical_plane_normal.lengthSq()));
            }

            // Get the relative velocity between the character and the constraint
            const relative_velocity = velocity.sub(constraint.linear_velocity);

            // Calculate new velocity if we cancel the relative velocity in the normal direction
            var new_velocity = velocity.sub(plane_normal.mulScalar(relative_velocity.dot(plane_normal)));

            // Find the normal of the previous contact that we will violate the most if we move in this new direction
            var highest_penetration: f32 = 0.0;
            var other_constraint: ?*const Constraint = null;
            for (previous_contacts.items()[0..num_previous_contacts]) |c| {
                if (c != constraint) {
                    // Calculate how much we will penetrate if we move in this direction
                    const other_normal = c.plane.getNormal();
                    const penetration = c.linear_velocity.sub(new_velocity).dot(other_normal);
                    if (penetration > highest_penetration) {
                        // We don't want parallel or anti-parallel normals as that will cause our cross product below to become zero. Slack is approx 10 degrees.
                        const dot = other_normal.dot(plane_normal);
                        if (dot < 0.984 and dot > -0.984) {
                            highest_penetration = penetration;
                            other_constraint = c;
                        }
                    }

                    // Cancel the constraint velocity in the other constraint plane's direction so that we won't try to apply it again and keep ping ponging between planes
                    constraint.linear_velocity = constraint.linear_velocity.sub(other_normal.mulScalar(math.min(@as(f32, 0.0), constraint.linear_velocity.dot(other_normal))));

                    // Cancel the other constraints velocity in this constraint plane's direction so that we won't try to apply it again and keep ping ponging between planes
                    c.linear_velocity = c.linear_velocity.sub(plane_normal.mulScalar(math.min(@as(f32, 0.0), c.linear_velocity.dot(plane_normal))));
                }
            }

            // Check if we found a 2nd constraint
            if (other_constraint) |oc| {
                // Calculate the sliding direction and project the new velocity onto that sliding direction
                const other_normal = oc.plane.getNormal();
                const slide_dir = plane_normal.cross(other_normal).normalized();
                const velocity_in_slide_dir = slide_dir.mulScalar(new_velocity.dot(slide_dir));

                // Calculate the velocity of this constraint perpendicular to the slide direction
                const perpendicular_velocity = constraint.linear_velocity.sub(slide_dir.mulScalar(constraint.linear_velocity.dot(slide_dir)));

                // Calculate the velocity of the other constraint perpendicular to the slide direction
                const other_perpendicular_velocity = oc.linear_velocity.sub(slide_dir.mulScalar(oc.linear_velocity.dot(slide_dir)));

                // Add all components together
                new_velocity = velocity_in_slide_dir.add(perpendicular_velocity).add(other_perpendicular_velocity);
            }

            // Allow application to modify calculated velocity
            if (self.listener) |listener| {
                const c = constraint.contact;
                if (c.character_b) |character_b|
                    listener.onCharacterContactSolve(self, character_b, c.base.sub_shape_id_b, c.position, c.contact_normal, c.linear_velocity, c.material, velocity, &new_velocity)
                else
                    listener.onContactSolve(self, c.base.body_b, c.base.sub_shape_id_b, c.position, c.contact_normal, c.linear_velocity, c.material, velocity, &new_velocity);
            }

            // TODO(debug renderer): draw the constraint planes and the velocities (inDrawConstraints)

            // Update the velocity
            velocity = new_velocity;

            // Add the contact to the list so that next iteration we can avoid violating it again
            previous_contacts.items()[num_previous_contacts] = constraint;
            num_previous_contacts += 1;

            // Check early out
            if (constraint.projected_velocity < 1.0e-8 and // Constraint should not be pushing, otherwise there may be other constraints that are pushing us
                velocity.lengthSq() < 1.0e-8) // There's not enough velocity left
                return .{ .displacement = out_displacement, .time_simulated = out_time_simulated };

            // If the constraint has velocity we accept the new velocity, otherwise check that we didn't reverse velocity
            if (!constraint.linear_velocity.isNearZero(.{ .max_dist_sq = 1.0e-8 }))
                last_velocity = constraint.linear_velocity
            else if (velocity.dot(last_velocity) < 0.0)
                return .{ .displacement = out_displacement, .time_simulated = out_time_simulated };
        }

        return .{ .displacement = out_displacement, .time_simulated = out_time_simulated };
    }

    /// The out parameters of GetAdjustedBodyVelocity
    const BodyVelocity = struct {
        linear: Vec3,
        angular: Vec3,
    };

    // Get the velocity of a body adjusted by the contact listener
    fn getAdjustedBodyVelocity(self: *const CharacterVirtual, body: *const Body) BodyVelocity {
        // Get real velocity of body
        var linear_velocity: Vec3 = undefined;
        var angular_velocity: Vec3 = undefined;
        if (!body.isStatic()) {
            const mp = body.getMotionPropertiesUnchecked().?;
            linear_velocity = mp.getLinearVelocity();
            angular_velocity = mp.getAngularVelocity();
        } else {
            linear_velocity = Vec3.zero();
            angular_velocity = Vec3.zero();
        }

        // Allow application to override
        if (self.listener) |listener|
            listener.onAdjustBodyVelocity(self, body, &linear_velocity, &angular_velocity);

        return .{ .linear = linear_velocity, .angular = angular_velocity };
    }

    // Calculate the ground velocity of the character assuming it's standing on an object with specified linear and angular velocity and with specified center of mass.
    // Note that we don't just take the point velocity because a point on an object with angular velocity traces an arc,
    // so if you just take point velocity * delta time you get an error that accumulates over time
    fn calculateCharacterGroundVelocity(self: *const CharacterVirtual, center_of_mass: RVec3, linear_velocity: Vec3, angular_velocity: Vec3, delta_time: f32) Vec3 {
        // Get angular velocity
        const angular_velocity_len_sq = angular_velocity.lengthSq();
        if (angular_velocity_len_sq < 1.0e-12)
            return linear_velocity;
        const angular_velocity_len = @sqrt(angular_velocity_len_sq);

        // Calculate the rotation that the object will make in the time step
        const rotation = Quat.rotation(angular_velocity.divScalar(angular_velocity_len), angular_velocity_len * delta_time);

        // Calculate where the new character position will be
        const new_position = center_of_mass.addVec3(rotation.mulVec3(self.position.sub(center_of_mass).toVec3()));

        // Calculate the velocity
        return linear_velocity.add(new_position.sub(self.position).toVec3().divScalar(delta_time));
    }

    // Handle contact with physics object that we're colliding against
    fn handleContact(self: *CharacterVirtual, velocity: Vec3, constraint: *Constraint, delta_time: f32) Allocator.Error!bool {
        const contact = constraint.contact;

        // Validate the contact point
        if (!self.validateContact(contact))
            return false;

        // We collided
        contact.had_collision = true;

        // Send contact added event
        var settings: CharacterContactSettings = .{};
        try self.contactAdded(contact, &settings);
        contact.can_push_character = settings.can_push_character;

        // We don't have any further interaction with sensors beyond an OnContactAdded notification
        if (contact.is_sensor_b)
            return false;

        // If body B cannot receive an impulse, we're done
        if (!settings.can_receive_impulses or contact.motion_type_b != .dynamic)
            return true;

        // Lock the body we're colliding with
        var lock: BodyLockWrite = .init(self.base.system.getBodyLockInterface(), contact.base.body_b);
        defer lock.deinit();
        if (!lock.succeededAndIsInBroadPhase())
            return false; // Body has been removed, we should not collide with it anymore
        const body: *const Body = lock.getBody();

        // Calculate the velocity that we want to apply at B so that it will start moving at the character's speed at the contact point
        const damping: f32 = 0.9;
        const penetration_resolution: f32 = 0.4;
        const relative_velocity = velocity.sub(contact.linear_velocity);
        const projected_velocity = relative_velocity.dot(contact.contact_normal);
        const delta_velocity = -projected_velocity * damping - math.min(contact.distance, 0.0) * penetration_resolution / delta_time;

        // Don't apply impulses if we're separating
        if (delta_velocity < 0.0)
            return true;

        // Determine mass properties of the body we're colliding with
        const motion_properties = body.getMotionProperties();
        const center_of_mass = body.getCenterOfMassPosition();
        const inverse_inertia = body.getInverseInertia();
        const inverse_mass = motion_properties.getInverseMass();

        // Calculate the inverse of the mass of body B as seen at the contact point in the direction of the contact normal
        const jacobian = contact.position.sub(center_of_mass).toVec3().cross(contact.contact_normal);
        const inv_effective_mass = inverse_inertia.multiply3x3(jacobian).dot(jacobian) + inverse_mass;

        // Impulse P = M dv
        var impulse = delta_velocity / inv_effective_mass;

        // Clamp the impulse according to the character strength, character strength is a force in newtons, P = F dt
        const max_impulse = self.max_strength * delta_time;
        impulse = math.min(impulse, max_impulse);

        // Calculate the world space impulse to apply
        var world_impulse = contact.contact_normal.mulScalar(-impulse);

        // Cancel impulse in down direction (we apply gravity later)
        const impulse_dot_up = world_impulse.dot(self.base.up);
        if (impulse_dot_up < 0.0)
            world_impulse = world_impulse.sub(self.base.up.mulScalar(impulse_dot_up));

        // Now apply the impulse (body is already locked so we use the no-lock interface)
        self.base.system.getBodyInterfaceNoLockMut().addImpulseAtPosition(contact.base.body_b, world_impulse, contact.position);
        return true;
    }

    // Does a swept test of the shape from position with displacement, returns true if there was a collision.
    // This sweeps to see if there is enough space for the character to move. Character padding is only applied in the movement direction of the character.
    // Ignores collisions when they already exist at the start of the movement.
    fn validateMovement(self: *const CharacterVirtual, position: RVec3, displacement: Vec3, out_contact: *CharacterContact, ignored_contacts: []const CharacterContactKey, filters: Filters) bool {
        // Too small distance -> skip checking
        const displacement_len_sq = displacement.lengthSq();
        if (displacement_len_sq < 1.0e-8)
            return false;

        // Calculate start transform
        const start = self.getCenterOfMassTransformAt(position, self.rotation, self.currentShape());

        // Settings for the cast
        var settings: ShapeCastSettings = .{};
        settings.back_face_mode_triangles = self.back_face_mode;
        settings.back_face_mode_convex = .ignore_back_faces;
        settings.active_edge_mode = .collide_only_with_active;
        settings.use_shrunken_shape_and_convex_radius = true;
        settings.return_deepest_point = false;

        // Calculate how much extra fraction we need to add to the cast to account for the character padding
        const character_padding_fraction = self.character_padding / @sqrt(displacement_len_sq);

        // Body filter
        var chained_body_filter: IgnoreSingleBodyFilterChained = .init(self.inner_body_id, filters.body_filter);

        // Cast shape
        var contact: CharacterContact = .{};
        contact.fraction = 1.0 + character_padding_fraction;
        const base_offset = start.getTranslation();
        var collector: ContactCastCollector(true) = .init(self.base.system, self, displacement, self.base.up, ignored_contacts, base_offset, &contact);
        collector.base.resetEarlyOutFraction(.{ .fraction = contact.fraction });
        const shape_cast: RShapeCast = .init(self.currentShape(), Vec3.one(), start, displacement);
        self.base.system.getNarrowPhaseQuery().castShape(&shape_cast, &settings, base_offset, &collector.base, .{ .broad_phase_layer_filter = filters.broad_phase_layer_filter, .object_layer_filter = filters.object_layer_filter, .body_filter = &chained_body_filter.base, .shape_filter = filters.shape_filter });

        // Also collide with other characters
        if (self.character_vs_character_collision) |character_vs_character_collision| {
            collector.base.setContext(null); // We're no longer colliding with a transformed shape, reset
            character_vs_character_collision.castCharacter(self, start, displacement, &settings, base_offset, &collector.base);
        }

        if (contact.base.body_b.isInvalid() and contact.base.character_id_b.isInvalid())
            return false;

        // Store contact
        out_contact.* = contact;

        var ts: TransformedShape = .{};
        defer ts.deinit();
        var character_padding = self.character_padding;
        if (out_contact.character_b) |character_b| {
            // Create a transformed shape for the character
            const com = character_b.getCenterOfMassTransform();
            ts = .init(com.getTranslation(), com.getQuaternion(), character_b.getShape(), .invalid, .{});

            // We need to take the other character's padding into account as well
            character_padding += character_b.character_padding;
        } else {
            // Create a transformed shape for the body
            ts = self.base.system.getBodyInterface().getTransformedShape(out_contact.base.body_b);
        }

        var corrected = false;
        if (ts.shape.get() != null) { // If body B has been removed between CastShape and GetTransformedShape, TransformedShape will be empty
            // Fetch the face we're colliding with
            var face: Shape.SupportingFace = .{};
            ts.getSupportingFace(out_contact.base.sub_shape_id_b, out_contact.contact_normal.negate(), base_offset, &face);

            if (face.len >= 2) {
                // Inflate the colliding face by the character padding
                const polygon: PolygonConvexSupport = .init(face.constSlice());
                const add_cvx: AddConvexRadius(PolygonConvexSupport) = .init(&polygon, character_padding);

                // Correct fraction to hit this inflated face instead of the inner shape
                corrected = correctFractionForCharacterPadding(self.currentShape(), start.getRotation(), displacement, Vec3.one(), &add_cvx, &out_contact.fraction);
            }
        }
        if (!corrected) {
            // When there's only a single contact point or when we were unable to correct the fraction,
            // we can just move the fraction back so that the character and its padding don't hit the contact point anymore
            out_contact.fraction = math.max(@as(f32, 0.0), out_contact.fraction - character_padding_fraction);
        }

        // Ensure that we never return a fraction that's bigger than 1 (which could happen due to float precision issues).
        out_contact.fraction = math.min(out_contact.fraction, 1.0);

        return true;
    }

    // Does a swept test of the shape from position with displacement, returns true if there was a collision.
    // This does a full sweep of character including character padding.
    // Includes collisions when they already exist at the start of the movement.
    fn getFirstContactForSweep(self: *const CharacterVirtual, position: RVec3, displacement: Vec3, out_contact: *CharacterContact, ignored_contacts: []const CharacterContactKey, filters: Filters) bool {
        // Calculate start transform
        const start = self.getCenterOfMassTransformAt(position, self.rotation, self.currentShape());

        // Settings for the cast
        var settings: ShapeCastSettings = .{};
        settings.extra_convex_radius = self.character_padding;
        settings.back_face_mode_triangles = self.back_face_mode;
        settings.back_face_mode_convex = .ignore_back_faces;
        settings.active_edge_mode = .collide_only_with_active;
        settings.use_shrunken_shape_and_convex_radius = true;
        settings.return_deepest_point = false;

        // Body filter
        var chained_body_filter: IgnoreSingleBodyFilterChained = .init(self.inner_body_id, filters.body_filter);

        // Cast shape
        var contact: CharacterContact = .{};
        contact.fraction = 1.0 + math.flt_min;
        const base_offset = start.getTranslation();
        var collector: ContactCastCollector(false) = .init(self.base.system, self, displacement, self.base.up, ignored_contacts, base_offset, &contact);
        const shape_cast: RShapeCast = .init(self.currentShape(), Vec3.one(), start, displacement);
        self.base.system.getNarrowPhaseQuery().castShape(&shape_cast, &settings, base_offset, &collector.base, .{ .broad_phase_layer_filter = filters.broad_phase_layer_filter, .object_layer_filter = filters.object_layer_filter, .body_filter = &chained_body_filter.base, .shape_filter = filters.shape_filter });

        // Also collide with other characters
        if (self.character_vs_character_collision) |character_vs_character_collision| {
            collector.base.setContext(null); // We're no longer colliding with a transformed shape, reset
            character_vs_character_collision.castCharacter(self, start, displacement, &settings, base_offset, &collector.base);
        }

        if (contact.base.body_b.isInvalid() and contact.base.character_id_b.isInvalid())
            return false;

        // Store contact
        out_contact.* = contact;
        return true;
    }

    // Store contacts so that we have proper ground information
    fn storeActiveContacts(self: *CharacterVirtual, contacts: []const CharacterContact, temp_allocator: TempAllocator) Allocator.Error!void {
        try self.startTrackingContactChanges();
        defer self.finishTrackingContactChanges(); // Zolt: also on the error path, so that the calls stay paired

        try self.assignContacts(&self.active_contacts, contacts);

        try self.updateSupportingContact(true, temp_allocator);
    }

    // This function will determine which contacts are touching the character and will calculate the one that is supporting us
    fn updateSupportingContact(self: *CharacterVirtual, skip_contact_velocity_check: bool, temp_allocator: TempAllocator) Allocator.Error!void {
        // Flag contacts as having a collision if they're close enough but ignore contacts we're moving away from.
        // Note that if we did MoveShape before we want to preserve any contacts that it marked as colliding
        for (self.active_contacts.items) |*c| {
            if (!c.was_discarded and
                !c.had_collision and
                c.distance < self.collision_tolerance and
                (skip_contact_velocity_check or c.surface_normal.dot(self.linear_velocity.sub(c.linear_velocity)) <= 1.0e-4))
            {
                if (self.validateContact(c)) {
                    var dummy: CharacterContactSettings = .{};
                    try self.contactAdded(c, &dummy);
                    c.had_collision = true;
                } else c.was_discarded = true;
            }
        }

        // Calculate transform that takes us to character local space
        const inv_transform = RMat44.inverseRotationTranslation(self.rotation, self.position);

        // Determine if we're supported or not
        var num_supported: i32 = 0;
        var num_sliding: i32 = 0;
        var num_avg_normal: i32 = 0;
        var avg_normal = Vec3.zero();
        var avg_velocity = Vec3.zero();
        var supporting_contact: ?*const CharacterContact = null;
        var max_cos_angle: f32 = -math.flt_max;
        var deepest_contact: ?*const CharacterContact = null;
        var smallest_distance: f32 = math.flt_max;
        for (self.active_contacts.items) |*c| {
            if (c.had_collision and !c.was_discarded) {
                // Calculate the angle between the plane normal and the up direction
                const cos_angle = c.surface_normal.dot(self.base.up);

                // Find the deepest contact
                if (c.distance < smallest_distance) {
                    deepest_contact = c;
                    smallest_distance = c.distance;
                }

                // If this contact is in front of our plane, we cannot be supported by it
                if (self.base.supporting_volume.signedDistance(inv_transform.mulRVec3(c.position).toVec3()) > 0.0)
                    continue;

                // Find the contact with the normal that is pointing most upwards and store it
                if (max_cos_angle < cos_angle) {
                    supporting_contact = c;
                    max_cos_angle = cos_angle;
                }

                // Check if this is a sliding or supported contact
                const is_supported = self.base.cos_max_slope_angle > CharacterBase.no_max_slope_angle or cos_angle >= self.base.cos_max_slope_angle;
                if (is_supported)
                    num_supported += 1
                else
                    num_sliding += 1;

                // If the angle between the two is less than 85 degrees we also use it to calculate the average normal
                if (cos_angle >= 0.08) {
                    avg_normal = avg_normal.add(c.surface_normal);
                    num_avg_normal += 1;

                    // For static or dynamic objects or for contacts that don't support us just take the contact velocity
                    if (c.motion_type_b != .kinematic or !is_supported) {
                        avg_velocity = avg_velocity.add(c.linear_velocity);
                    } else {
                        // For keyframed objects that support us calculate the velocity at our position rather than at the contact position so that we properly follow the object
                        var lock: BodyLockRead = .init(self.base.system.getBodyLockInterface(), c.base.body_b);
                        defer lock.deinit();
                        if (lock.succeededAndIsInBroadPhase()) {
                            const body = lock.getBody();

                            // Get adjusted body velocity
                            const velocity = self.getAdjustedBodyVelocity(body);

                            // Calculate the ground velocity
                            avg_velocity = avg_velocity.add(self.calculateCharacterGroundVelocity(body.getCenterOfMassPosition(), velocity.linear, velocity.angular, self.last_delta_time));
                        } else {
                            // Fall back to contact velocity
                            avg_velocity = avg_velocity.add(c.linear_velocity);
                        }
                    }
                }
            }
        }

        // Take either the most supporting contact or the deepest contact
        const best_contact: ?*const CharacterContact = supporting_contact orelse deepest_contact;

        // Calculate average normal and velocity
        if (num_avg_normal >= 1) {
            self.base.ground_normal = avg_normal.normalized();
            self.base.ground_velocity = avg_velocity.divScalar(@floatFromInt(num_avg_normal));
        } else if (best_contact) |c| {
            self.base.ground_normal = c.surface_normal;
            self.base.ground_velocity = c.linear_velocity;
        } else {
            self.base.ground_normal = Vec3.zero();
            self.base.ground_velocity = Vec3.zero();
        }

        // Copy contact properties
        if (best_contact) |c| {
            self.base.ground_body_id = c.base.body_b;
            self.base.ground_body_sub_shape_id = c.base.sub_shape_id_b;
            self.base.ground_position = c.position;
            self.base.ground_material.set(c.material);
            self.base.ground_user_data = c.user_data;
        } else {
            self.base.ground_body_id = .invalid;
            self.base.ground_body_sub_shape_id = .empty;
            self.base.ground_position = RVec3.zero();
            self.base.ground_material.set(PhysicsMaterial.default);
            self.base.ground_user_data = 0;
        }

        // Determine ground state
        if (num_supported > 0) {
            // We made contact with something that supports us
            self.base.ground_state = .on_ground;
        } else if (num_sliding > 0) {
            if (self.linear_velocity.sub(deepest_contact.?.linear_velocity).dot(self.base.up) > 1.0e-4) {
                // We cannot be on ground if we're moving upwards relative to the ground
                self.base.ground_state = .on_steep_ground;
            } else {
                // If we're sliding down, we may actually be standing on multiple sliding contacts in such a way that we can't slide off, in this case we're also supported

                // Convert the contacts into constraints
                var contacts: TempList(CharacterContact) = .{};
                try contacts.assignSlice(temp_allocator, self.active_contacts.items);
                defer contacts.free(temp_allocator);
                var constraints: TempList(Constraint) = .{};
                try constraints.reserve(temp_allocator, contacts.len * 2);
                defer constraints.free(temp_allocator);
                self.determineConstraints(contacts.items(), self.last_delta_time, &constraints);

                // Solve the displacement using these constraints, this is used to check if we didn't move at all because we are supported
                var ignored_contacts: TempList(CharacterContactKey) = .{};
                try ignored_contacts.reserve(temp_allocator, contacts.len);
                defer ignored_contacts.free(temp_allocator);
                const solved = try self.solveConstraints(self.base.up.negate(), 1.0, 1.0, &constraints, &ignored_contacts, temp_allocator);

                // If we're blocked then we're supported, otherwise we're sliding
                const min_required_displacement_sq = math.square(0.6 * self.last_delta_time);
                if (solved.time_simulated < 0.001 or solved.displacement.lengthSq() < min_required_displacement_sq)
                    self.base.ground_state = .on_ground
                else
                    self.base.ground_state = .on_steep_ground;
            }
        } else {
            // Not supported by anything
            self.base.ground_state = if (best_contact != null) .not_supported else .in_air;
        }
    }

    /// This function can be called after moving the character to a new colliding position
    fn moveToContact(self: *CharacterVirtual, position: RVec3, contact: *const CharacterContact, filters: Filters, temp_allocator: TempAllocator) Allocator.Error!void {
        // Set the new position
        self.setPosition(position);

        // Trigger contact added callback
        var dummy: CharacterContactSettings = .{};
        try self.contactAdded(contact, &dummy);

        // Determine the contacts
        var contacts: TempList(CharacterContact) = .{};
        try contacts.reserve(temp_allocator, @as(usize, self.max_num_hits) + 1); // +1 because we can add one extra below
        defer contacts.free(temp_allocator);
        try self.getContactsAtPosition(self.position, self.linear_velocity.normalizedOr(Vec3.zero()), self.currentShape(), &contacts, filters);

        // Ensure that we mark inContact as colliding
        var found_contact = false;
        for (contacts.items()) |*c| {
            if (c.base.body_b.eql(contact.base.body_b) and
                c.base.sub_shape_id_b.eql(contact.base.sub_shape_id_b))
            {
                c.had_collision = true;
                found_contact = true;
            }
        }
        if (!found_contact) {
            contacts.append(contact.*);

            const copy = contacts.last();
            copy.had_collision = true;
        }

        try self.storeActiveContacts(contacts.items(), temp_allocator);
        if (Core.enable_asserts) std.debug.assert(self.base.ground_state != .in_air);

        // Ensure that the rigid body ends up at the new position
        self.updateInnerBodyTransform();
    }

    // This function returns the actual center of mass of the shape, not corrected for the character padding
    // (Jolt's private GetCenterOfMassTransform(RVec3Arg, QuatArg, const Shape *) overload)
    fn getCenterOfMassTransformAt(self: *const CharacterVirtual, position: RVec3, rotation: Quat, query_shape: *const Shape) RMat44 {
        return RMat44.rotationTranslation(rotation, position).preTranslated(self.shape_offset.add(query_shape.getCenterOfMass())).postTranslated(self.base.up.mulScalar(self.character_padding));
    }

    // This function returns the position of the inner rigid body
    fn getInnerBodyPosition(self: *const CharacterVirtual) RVec3 {
        return self.position.addVec3(self.rotation.mulVec3(self.shape_offset).add(self.base.up.mulScalar(self.character_padding)));
    }

    // Move the inner rigid body to the current position
    fn updateInnerBodyTransform(self: *CharacterVirtual) void {
        if (!self.inner_body_id.isInvalid())
            self.base.system.getBodyInterfaceMut().setPositionAndRotation(self.inner_body_id, self.getInnerBodyPosition(), self.rotation, .dont_activate);
    }
};

// Helper function to convert a Jolt collision result into a contact (sFillContactProperties; `context` is the
// collector's GetContext())
fn fillContactProperties(character: *const CharacterVirtual, out_contact: *CharacterContact, body: *const Body, up: Vec3, base_offset: RVec3, context: *const TransformedShape, result: *const CollideShapeResult) void {
    // Get adjusted body velocity
    const velocity = character.getAdjustedBodyVelocity(body);

    out_contact.position = base_offset.addVec3(result.contact_point_on2);
    out_contact.linear_velocity = velocity.linear.add(velocity.angular.cross(out_contact.position.sub(body.getCenterOfMassPosition()).toVec3())); // Calculate point velocity
    out_contact.contact_normal = result.penetration_axis.normalizedOr(Vec3.zero()).negate();
    out_contact.surface_normal = context.getWorldSpaceSurfaceNormal(result.sub_shape_id2, out_contact.position);
    out_contact.is_back_facing_contact = out_contact.contact_normal.dot(out_contact.surface_normal) < -0.0174524; // Allow a 1 degree slop, perpendicular is not backfacing
    if (out_contact.is_back_facing_contact)
        out_contact.surface_normal = out_contact.surface_normal.negate(); // Flip surface normal if we're hitting a back face
    if (out_contact.contact_normal.dot(up) > out_contact.surface_normal.dot(up))
        out_contact.surface_normal = out_contact.contact_normal; // Replace surface normal with contact normal if the contact normal is pointing more upwards
    out_contact.distance = -result.penetration_depth;
    out_contact.base.body_b = result.body_id2;
    out_contact.base.sub_shape_id_b = result.sub_shape_id2;
    out_contact.motion_type_b = body.getMotionType();
    out_contact.is_sensor_b = body.isSensor();
    out_contact.user_data = body.getUserData();
    out_contact.material = context.getMaterial(result.sub_shape_id2);
}

// sFillCharacterContactProperties
fn fillCharacterContactProperties(out_contact: *CharacterContact, other_character: *const CharacterVirtual, base_offset: RVec3, result: *const CollideShapeResult) void {
    out_contact.position = base_offset.addVec3(result.contact_point_on2);
    out_contact.linear_velocity = other_character.getLinearVelocity();
    out_contact.contact_normal = result.penetration_axis.normalizedOr(Vec3.zero()).negate();
    out_contact.surface_normal = out_contact.contact_normal;
    out_contact.distance = -result.penetration_depth;
    out_contact.base.character_id_b = other_character.getID();
    out_contact.character_b = other_character;
    out_contact.base.sub_shape_id_b = result.sub_shape_id2;
    out_contact.motion_type_b = .kinematic; // Other character is kinematic, we can't directly move it
    out_contact.is_sensor_b = false;
    out_contact.user_data = other_character.getUserData();
    out_contact.material = PhysicsMaterial.default;
}

// sCorrectFractionForCharacterPadding (`polygon`: a pointer to the convex object, Jolt's `const T &inPolygon`)
fn correctFractionForCharacterPadding(shape: *const Shape, start: Mat44, displacement: Vec3, scale: Vec3, polygon: anytype, fraction: *f32) bool {
    if (shape.getType() == .convex) {
        // Get the support function for the shape we're casting
        const convex_shape = shape.cast(ConvexShape);
        var buffer: ConvexShape.SupportBuffer = .{};
        const support = convex_shape.getSupportFunction(.include_convex_radius, &buffer, scale);

        // Cast the shape against the polygon
        var gjk: GJKClosestPoint = .{};
        return gjk.castShape(start, displacement, physics_settings.default_collision_tolerance, support, polygon, fraction);
    } else if (shape.getSubType() == .rotated_translated) {
        const rt_shape = shape.cast(RotatedTranslatedShape);
        return correctFractionForCharacterPadding(rt_shape.base.getInnerShape().?, start.mul(Mat44.rotationQuat(rt_shape.getRotation())), displacement, rt_shape.transformScale(scale), polygon, fraction);
    } else if (shape.getSubType() == .scaled) {
        const scaled_shape = shape.cast(ScaledShape);
        return correctFractionForCharacterPadding(scaled_shape.base.getInnerShape().?, start, displacement, scale.mul(scaled_shape.getScale()), polygon, fraction);
    } else if (shape.getType() == .compound) {
        const compound = shape.cast(CompoundShape);
        var return_value = false;
        for (compound.getSubShapes()) |*sub_shape| {
            const corrected = correctFractionForCharacterPadding(sub_shape.shape.get().?, start.mul(sub_shape.getLocalTransformNoScale(scale)), displacement, sub_shape.transformScale(scale), polygon, fraction); // return_value |= ...: always evaluated
            return_value = return_value or corrected;
        }
        return return_value;
    } else {
        if (Core.enable_asserts) @panic("Not supported yet!");
        return false;
    }
}
