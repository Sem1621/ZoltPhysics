//! Port of: Jolt/Physics/Collision/ContactListener.h
//! Status: complete
//!
//! - `ContactManifold` and `ContactSettings` are value types. Members that Jolt leaves uninitialized (the vectors and
//!   the depth of a manifold, the combined friction / restitution and the sensor flag of the settings) default to
//!   `undefined`; the sub shape IDs default to empty and the contact point arrays to empty, like their C++ default
//!   constructors.
//! - `ContactListener` is an interface (porting guide pattern B, Docs/Zolt/CollisionArchitecture.md D7): a type
//!   erased `ptr: *anyopaque` (the callbacks are non-const in Jolt) and a vtable with a nullable entry per callback.
//!   Every callback has a default (empty) implementation in Jolt, so `ContactListener.init(&my_listener)` only wires
//!   the callbacks that the implementation type declares, a missing one behaves like Jolt's default:
//!   ```zig
//!   const MyListener = struct {
//!       num_added: u32 = 0,
//!       pub fn onContactAdded(self: *MyListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void { ... }
//!   };
//!   var my_listener: MyListener = .{};
//!   const listener = ContactListener.init(&my_listener);
//!   ```
//!   The listener does not own the implementation (the C++ virtual destructor has no counterpart).
//! - `Body` is the Phase 4 stub (only used as a pointer type here).

const std = @import("std");
const StaticArray = @import("../../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Body = @import("../Body/Body.zig").Body;
const CollideShapeResult = @import("CollideShape.zig").CollideShapeResult;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const SubShapeIDPair = @import("Shape/SubShapeIDPair.zig").SubShapeIDPair;

/// Array of contact points
pub const ContactPoints = StaticArray(Vec3, 64);

/// Manifold class, describes the contact surface between two bodies
pub const ContactManifold = struct {
    /// Offset to which all the contact points are relative
    base_offset: RVec3 = undefined,
    /// Normal for this manifold, direction along which to move body 2 out of collision along the shortest path
    world_space_normal: Vec3 = undefined,
    /// Penetration depth (move shape 2 by this distance to resolve the collision). If this value is negative, this is a speculative contact point and may not actually result in a velocity change as during solving the bodies may not actually collide.
    penetration_depth: f32 = undefined,
    /// Sub shapes that formed this manifold (note that when multiple manifolds are combined because they're coplanar, we lose some information here because we only keep track of one sub shape pair that we encounter, see description at Body::SetUseManifoldReduction)
    sub_shape_id1: SubShapeID = .empty,
    sub_shape_id2: SubShapeID = .empty,
    /// Contact points on the surface of shape 1 relative to mBaseOffset.
    relative_contact_points_on1: ContactPoints = .empty,
    /// Contact points on the surface of shape 2 relative to mBaseOffset. If there's no penetration, this will be the same as mRelativeContactPointsOn1. If there is penetration they will be different.
    relative_contact_points_on2: ContactPoints = .empty,

    /// Swaps shape 1 and 2
    pub fn swapShapes(self: *const ContactManifold) ContactManifold {
        return .{
            .base_offset = self.base_offset,
            .world_space_normal = self.world_space_normal.negate(),
            .penetration_depth = self.penetration_depth,
            .sub_shape_id1 = self.sub_shape_id2,
            .sub_shape_id2 = self.sub_shape_id1,
            .relative_contact_points_on1 = self.relative_contact_points_on2,
            .relative_contact_points_on2 = self.relative_contact_points_on1,
        };
    }

    /// Access to the world space contact positions
    pub fn getWorldSpaceContactPointOn1(self: *const ContactManifold, index: u32) RVec3 {
        return self.base_offset.addVec3(self.relative_contact_points_on1.get(index));
    }

    pub fn getWorldSpaceContactPointOn2(self: *const ContactManifold, index: u32) RVec3 {
        return self.base_offset.addVec3(self.relative_contact_points_on2.get(index));
    }
};

/// When a contact point is added or persisted, the callback gets a chance to override certain properties of the contact constraint.
/// The values are filled in with their defaults by the system so the callback doesn't need to modify anything, but it can if it wants to.
pub const ContactSettings = struct {
    /// Combined friction for the body pair (see: PhysicsSystem::SetCombineFriction)
    combined_friction: f32 = undefined,
    /// Combined restitution for the body pair (see: PhysicsSystem::SetCombineRestitution)
    combined_restitution: f32 = undefined,
    /// Scale factor for the inverse mass of body 1 (0 = infinite mass, 1 = use original mass, 2 = body has half the mass). For the same contact pair, you should strive to keep the value the same over time.
    inv_mass_scale1: f32 = 1.0,
    /// Scale factor for the inverse inertia of body 1 (usually same as mInvMassScale1)
    inv_inertia_scale1: f32 = 1.0,
    /// Scale factor for the inverse mass of body 2 (0 = infinite mass, 1 = use original mass, 2 = body has half the mass). For the same contact pair, you should strive to keep the value the same over time.
    inv_mass_scale2: f32 = 1.0,
    /// Scale factor for the inverse inertia of body 2 (usually same as mInvMassScale2)
    inv_inertia_scale2: f32 = 1.0,
    /// If the contact should be treated as a sensor vs body contact (no collision response)
    is_sensor: bool = undefined,
    /// Relative linear surface velocity between the bodies (world space surface velocity of body 2 - world space surface velocity of body 1), can be used to create a conveyor belt effect
    relative_linear_surface_velocity: Vec3 = Vec3.zero(),
    /// Relative angular surface velocity between the bodies (world space angular surface velocity of body 2 - world space angular surface velocity of body 1). Note that this angular velocity is relative to the center of mass of body 1, so if you want it relative to body 2's center of mass you need to add body 2 angular velocity x (body 1 world space center of mass - body 2 world space center of mass) to mRelativeLinearSurfaceVelocity.
    relative_angular_surface_velocity: Vec3 = Vec3.zero(),
};

/// Return value for the OnContactValidate callback. Determines if the contact is being processed or not.
/// Results are ordered so that the strongest accept has the lowest number and the strongest reject the highest number (which allows for easy combining of results)
pub const ValidateResult = enum(i32) {
    /// Accept this and any further contact points for this body pair
    accept_all_contacts_for_this_body_pair,
    /// Accept this contact only (and continue calling this callback for every contact manifold for the same body pair)
    accept_contact,
    /// Reject this contact only (but process any other contact manifolds for the same body pair)
    reject_contact,
    /// Rejects this and any further contact points for this body pair
    reject_all_contacts_for_this_body_pair,
};

/// A listener class that receives collision contact events. It can be registered through PhysicsSystem::SetContactListener.
/// Only a single contact listener can be registered. A common pattern is to create a contact listener that casts Body::GetUserData
/// to a game object and then forwards the call to a handler specific for that game object.
/// Typically this is done on both objects involved in a collision event.
///
/// Note that contact listener callbacks are called from multiple threads at the same time when all bodies are locked, this means you cannot
/// use PhysicsSystem::GetBodyInterface / PhysicsSystem::GetBodyLockInterface but must use PhysicsSystem::GetBodyInterfaceNoLock / PhysicsSystem::GetBodyLockInterfaceNoLock instead.
/// If you use a locking interface, the simulation will deadlock. You're only allowed to read from the bodies and you can't change physics state.
///
/// While a callback can come from multiple threads, all callbacks relating to a single body pair are serialized.
/// For EMotionQuality::Discrete bodies, during every 'collision step' in a PhysicsSystem::Update, you will receive at most one OnContactAdded/Persisted/Removed call per body/sub shape pair.
/// For EMotionQuality::LinearCast bodies, you may get an OnContactAdded followed by an OnContactPersisted for the same body/sub shape pair.
/// This happens when a body collides both in the discrete and the continuous collision detection stage.
pub const ContactListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    /// One nullable entry per virtual function, null is Jolt's default implementation
    pub const VTable = struct {
        /// Called after detecting a collision between a body pair, but before calling OnContactAdded and before adding the contact constraint.
        /// If the function rejects the contact, the contact will not be processed by the simulation.
        /// This is a rather expensive time to reject a contact point since a lot of the collision detection has happened already, make sure you
        /// filter out the majority of undesired body pairs through the ObjectLayerPairFilter that is registered on the PhysicsSystem.
        ///
        /// This function may not be called again the next update if a contact persists and no new contact pairs between sub shapes are found.
        ///
        /// Note that this callback is called when all bodies are locked, so don't use any locking functions! See detailed class description of ContactListener.
        ///
        /// Body 1 will have a motion type that is larger or equal than body 2's motion type (order from large to small: dynamic -> kinematic -> static). When motion types are equal, they are ordered by BodyID.
        ///
        /// The collision result (inCollisionResult) is reported relative to inBaseOffset.
        onContactValidate: ?*const fn (ptr: *anyopaque, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult = null,

        /// Called whenever a new contact point is detected.
        ///
        /// Note that this callback is called when all bodies are locked, so don't use any locking functions! See detailed class description of ContactListener.
        ///
        /// Body 1 and 2 will be sorted such that body 1 ID < body 2 ID, so body 1 may not be dynamic.
        ///
        /// Note that only active bodies will report contacts, as soon as a body goes to sleep the contacts between that body and all other
        /// bodies will receive an OnContactRemoved callback, if this is the case then Body::IsActive() will return false during the callback.
        ///
        /// When contacts are added, the constraint solver has not run yet, so the collision impulse is unknown at that point.
        /// The velocities of inBody1 and inBody2 are the velocities before the contact has been resolved, so you can use this to
        /// estimate the collision impulse to e.g. determine the volume of the impact sound to play (see: EstimateCollisionResponse).
        onContactAdded: ?*const fn (ptr: *anyopaque, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void = null,

        /// Called whenever a contact is detected that was also detected last update.
        ///
        /// Note that this callback is called when all bodies are locked, so don't use any locking functions! See detailed class description of ContactListener.
        ///
        /// Body 1 and 2 will be sorted such that body 1 ID < body 2 ID, so body 1 may not be dynamic.
        ///
        /// If the structure of the shape of a body changes between simulation steps (e.g. by adding/removing a child shape of a compound shape),
        /// it is possible that the same sub shape ID used to identify the removed child shape is now reused for a different child shape. The physics
        /// system cannot detect this, so may send a 'contact persisted' callback even though the contact is now on a different child shape. You can
        /// detect this by keeping the old shape (before adding/removing a part) around until the next PhysicsSystem::Update (when the OnContactPersisted
        /// callbacks are triggered) and resolving the sub shape ID against both the old and new shape to see if they still refer to the same child shape.
        onContactPersisted: ?*const fn (ptr: *anyopaque, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void = null,

        /// Called whenever a contact was detected last update but is not detected anymore.
        ///
        /// Note that this callback is called when all bodies are locked, so don't use any locking functions! See detailed class description of ContactListener.
        ///
        /// Beware when trying to access the bodies at the time of callback:
        /// - Some properties of the bodies are being modified from another thread at the same time.
        /// Things that the simulation doesn't modify are safe to read (e.g. user data, if sensor, if added to system etc.).
        /// Things that the simulation does modify (e.g. position, rotation, velocity, bounding box) are not safe to read.
        /// Checking Body::IsActive may return that the body is active but another thread can be in the process of making it go to sleep.
        /// Writing to the bodies is not allowed.
        /// - The body may have been removed and destroyed (you'll receive an OnContactRemoved callback in the PhysicsSystem::Update after the body has been removed). Check that the body still exists.
        ///
        /// If you need the properties that are being written, cache what you need in the OnContactAdded and OnContactPersisted callbacks and store it in a separate structure to use during this callback.
        /// Alternatively, you could just record that the contact was removed and process it after PhysicsSystem::Update.
        ///
        /// Body 1 and 2 will be sorted such that body 1 ID < body 2 ID, so body 1 may not be dynamic.
        ///
        /// The sub shape IDs were created in the previous simulation step, so if the structure of a shape changes (e.g. by adding/removing a child shape of a compound shape),
        /// the sub shape ID may not be valid / may not point to the same sub shape anymore.
        /// If you want to know if this is the last contact between the two bodies, use PhysicsSystem::WereBodiesInContact.
        onContactRemoved: ?*const fn (ptr: *anyopaque, sub_shape_pair: *const SubShapeIDPair) void = null,
    };

    /// Wrap any `*T` that declares some of the callbacks (`onContactValidate`, `onContactAdded`, `onContactPersisted`,
    /// `onContactRemoved`, with the parameters of the vtable entries after `ptr`)
    pub fn init(impl: anytype) ContactListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing the dispatchers of the outer struct
            fn onContactValidateThunk(ptr: *anyopaque, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.onContactValidate(body1, body2, base_offset, collision_result);
            }
            fn onContactAddedThunk(ptr: *anyopaque, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onContactAdded(body1, body2, manifold, settings);
            }
            fn onContactPersistedThunk(ptr: *anyopaque, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onContactPersisted(body1, body2, manifold, settings);
            }
            fn onContactRemovedThunk(ptr: *anyopaque, sub_shape_pair: *const SubShapeIDPair) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onContactRemoved(sub_shape_pair);
            }
            const vtable: VTable = .{
                .onContactValidate = if (@hasDecl(T, "onContactValidate")) onContactValidateThunk else null,
                .onContactAdded = if (@hasDecl(T, "onContactAdded")) onContactAddedThunk else null,
                .onContactPersisted = if (@hasDecl(T, "onContactPersisted")) onContactPersistedThunk else null,
                .onContactRemoved = if (@hasDecl(T, "onContactRemoved")) onContactRemovedThunk else null,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// See VTable.onContactValidate (default: accept all contacts for this body pair)
    pub fn onContactValidate(self: ContactListener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        if (self.vtable.onContactValidate) |f| return f(self.ptr, body1, body2, base_offset, collision_result);
        return .accept_all_contacts_for_this_body_pair;
    }

    /// See VTable.onContactAdded (default: do nothing)
    pub fn onContactAdded(self: ContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        if (self.vtable.onContactAdded) |f| f(self.ptr, body1, body2, manifold, settings);
    }

    /// See VTable.onContactPersisted (default: do nothing)
    pub fn onContactPersisted(self: ContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        if (self.vtable.onContactPersisted) |f| f(self.ptr, body1, body2, manifold, settings);
    }

    /// See VTable.onContactRemoved (default: do nothing)
    pub fn onContactRemoved(self: ContactListener, sub_shape_pair: *const SubShapeIDPair) void {
        if (self.vtable.onContactRemoved) |f| f(self.ptr, sub_shape_pair);
    }
};

test "ContactManifold: SwapShapes, world space contact points" {
    const expect = std.testing.expect;

    var manifold: ContactManifold = .{
        .base_offset = RVec3.init(1, 2, 3),
        .world_space_normal = Vec3.init(0, 1, -0.0),
        .penetration_depth = 0.25,
    };
    manifold.sub_shape_id1.setValue(5);
    manifold.sub_shape_id2.setValue(7);
    manifold.relative_contact_points_on1.append(Vec3.init(1, 0, 0));
    manifold.relative_contact_points_on1.append(Vec3.init(0, 1, 0));
    manifold.relative_contact_points_on2.append(Vec3.init(0, 0, 1));

    try expect(manifold.getWorldSpaceContactPointOn1(1).eql(RVec3.init(1, 3, 3)));
    try expect(manifold.getWorldSpaceContactPointOn2(0).eql(RVec3.init(1, 2, 4)));

    const swapped = manifold.swapShapes();
    try expect(swapped.base_offset.eql(manifold.base_offset));
    try expect(swapped.world_space_normal.eql(Vec3.init(0, -1, 0)));
    try std.testing.expectEqual(@as(u32, 0), @as(u32, @bitCast(swapped.world_space_normal.getZ()))); // 0 - (-0) = +0
    try std.testing.expectEqual(@as(f32, 0.25), swapped.penetration_depth);
    try std.testing.expectEqual(@as(u32, 7), swapped.sub_shape_id1.getValue());
    try std.testing.expectEqual(@as(u32, 5), swapped.sub_shape_id2.getValue());
    try std.testing.expectEqual(@as(u32, 1), swapped.relative_contact_points_on1.len);
    try std.testing.expectEqual(@as(u32, 2), swapped.relative_contact_points_on2.len);
    try expect(swapped.getWorldSpaceContactPointOn1(0).eql(RVec3.init(1, 2, 4)));
    try expect(swapped.getWorldSpaceContactPointOn2(1).eql(RVec3.init(1, 3, 3)));

    // Defaults
    const empty: ContactManifold = .{};
    try expect(empty.sub_shape_id1.eql(SubShapeID.empty) and empty.sub_shape_id2.eql(SubShapeID.empty));
    try expect(empty.relative_contact_points_on1.isEmpty() and empty.relative_contact_points_on2.isEmpty());
}

test "ContactSettings, ValidateResult" {
    const settings: ContactSettings = .{};
    try std.testing.expectEqual(@as(f32, 1.0), settings.inv_mass_scale1);
    try std.testing.expectEqual(@as(f32, 1.0), settings.inv_inertia_scale1);
    try std.testing.expectEqual(@as(f32, 1.0), settings.inv_mass_scale2);
    try std.testing.expectEqual(@as(f32, 1.0), settings.inv_inertia_scale2);
    try std.testing.expect(settings.relative_linear_surface_velocity.eql(Vec3.zero()));
    try std.testing.expect(settings.relative_angular_surface_velocity.eql(Vec3.zero()));

    // Ordered from the strongest accept to the strongest reject
    try std.testing.expectEqual(@as(i32, 0), @intFromEnum(ValidateResult.accept_all_contacts_for_this_body_pair));
    try std.testing.expectEqual(@as(i32, 1), @intFromEnum(ValidateResult.accept_contact));
    try std.testing.expectEqual(@as(i32, 2), @intFromEnum(ValidateResult.reject_contact));
    try std.testing.expectEqual(@as(i32, 3), @intFromEnum(ValidateResult.reject_all_contacts_for_this_body_pair));
}

test "ContactListener: default callbacks and an implementation" {
    const BodyID = @import("../Body/BodyID.zig").BodyID;
    const expect = std.testing.expect;

    const body1: Body = .{ .id = .init(1) };
    const body2: Body = .{ .id = .init(2) };
    var manifold: ContactManifold = .{ .base_offset = RVec3.zero(), .world_space_normal = Vec3.init(0, 1, 0), .penetration_depth = 0.1 };
    const result = CollideShapeResult.init(Vec3.zero(), Vec3.zero(), Vec3.init(0, -1, 0), 0.1, .empty, .empty, .init(2));
    const pair = SubShapeIDPair.init(.init(1), .empty, .init(2), .empty);

    // An implementation without callbacks behaves like Jolt's ContactListener base class
    const Nothing = struct {};
    var nothing: Nothing = .{};
    const default_listener = ContactListener.init(&nothing);
    try expect(default_listener.vtable.onContactAdded == null);
    var settings: ContactSettings = .{};
    try std.testing.expectEqual(ValidateResult.accept_all_contacts_for_this_body_pair, default_listener.onContactValidate(&body1, &body2, RVec3.zero(), &result));
    default_listener.onContactAdded(&body1, &body2, &manifold, &settings);
    default_listener.onContactPersisted(&body1, &body2, &manifold, &settings);
    default_listener.onContactRemoved(&pair);

    // An implementation that logs every call and changes the settings (LoggingContactListener style)
    const Logging = struct {
        validated: u32 = 0,
        added: u32 = 0,
        persisted: u32 = 0,
        removed: u32 = 0,
        last_body2: BodyID = .invalid,

        pub fn onContactValidate(self: *@This(), b1: *const Body, b2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
            _ = b1;
            _ = base_offset;
            self.validated += 1;
            self.last_body2 = b2.getID();
            return if (collision_result.penetration_depth > 0.0) .reject_contact else .accept_contact;
        }

        pub fn onContactAdded(self: *@This(), b1: *const Body, b2: *const Body, m: *const ContactManifold, s: *ContactSettings) void {
            _ = b1;
            _ = b2;
            _ = m;
            self.added += 1;
            s.inv_mass_scale1 = 0.5;
        }

        pub fn onContactPersisted(self: *@This(), b1: *const Body, b2: *const Body, m: *const ContactManifold, s: *ContactSettings) void {
            _ = b1;
            _ = b2;
            _ = m;
            self.persisted += 1;
            s.is_sensor = true;
        }

        pub fn onContactRemoved(self: *@This(), sub_shape_pair: *const SubShapeIDPair) void {
            self.removed += 1;
            self.last_body2 = sub_shape_pair.getBody2ID();
        }
    };
    var logging: Logging = .{};
    const listener = ContactListener.init(&logging);
    try std.testing.expectEqual(ValidateResult.reject_contact, listener.onContactValidate(&body1, &body2, RVec3.zero(), &result));
    try expect(logging.last_body2.eql(.init(2)));
    listener.onContactAdded(&body1, &body2, &manifold, &settings);
    try std.testing.expectEqual(@as(f32, 0.5), settings.inv_mass_scale1);
    listener.onContactPersisted(&body1, &body2, &manifold, &settings);
    try expect(settings.is_sensor);
    logging.last_body2 = .invalid;
    listener.onContactRemoved(&pair);
    try expect(logging.last_body2.eql(.init(2)));
    try std.testing.expectEqual(@as(u32, 1), logging.validated);
    try std.testing.expectEqual(@as(u32, 1), logging.added);
    try std.testing.expectEqual(@as(u32, 1), logging.persisted);
    try std.testing.expectEqual(@as(u32, 1), logging.removed);

    // An implementation with only some callbacks
    const OnlyRemoved = struct {
        removed: u32 = 0,
        pub fn onContactRemoved(self: *@This(), sub_shape_pair: *const SubShapeIDPair) void {
            _ = sub_shape_pair;
            self.removed += 1;
        }
    };
    var only_removed: OnlyRemoved = .{};
    const partial = ContactListener.init(&only_removed);
    try expect(partial.vtable.onContactValidate == null and partial.vtable.onContactRemoved != null);
    try std.testing.expectEqual(ValidateResult.accept_all_contacts_for_this_body_pair, partial.onContactValidate(&body1, &body2, RVec3.zero(), &result));
    partial.onContactRemoved(&pair);
    try std.testing.expectEqual(@as(u32, 1), only_removed.removed);
}
