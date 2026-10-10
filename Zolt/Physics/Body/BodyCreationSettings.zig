//! Port of: Jolt/Physics/Body/BodyCreationSettings.h, Jolt/Physics/Body/BodyCreationSettings.cpp
//! Status: partial
//! Missing: SaveBinaryState, RestoreBinaryState, SaveWithChildren, sRestoreWithChildren (the binary state, Wave A6)
//!
//! SimulationArchitecture D6:
//! - Jolt's private, mutually exclusive `RefConst<ShapeSettings> mShape` / `RefConst<Shape> mShapePtr` are the tagged
//!   union `shape: ShapeSource`. It holds a `Ref(ShapeSettings)`, not Jolt's RefConst, because creating the shape
//!   writes the settings' cache (Rule M, CollisionArchitecture D3). `getShape` therefore works on a const settings.
//! - `getShape(allocator) Allocator.Error!?*const Shape` keeps Jolt's semantics: null without a shape; on a creation
//!   error it traces (std.log info), panics when asserts are on and returns null.
//! - `getMassProperties(allocator)` needs the shape for the calculated modes: Jolt dereferences GetShape() (a crash
//!   without a shape), Zolt returns `error.InvalidShape` (`CreateBodyError`, also returned by BodyManager.allocateBody
//!   and BodyInterface.createBody).
//! - Constructors: `BodyCreationSettings{}` (default), `initWithSettings` (`const ShapeSettings *` overload),
//!   `initWithShape` (`const Shape *` overload). `clone` / `deinit` are the copy constructor / destructor (they add and
//!   release the references of the shape and the collision group), `eql` is `operator ==`.
//! - TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Reference = @import("../../Core/Reference.zig");
const Ref = Reference.Ref;
const RefConst = Reference.RefConst;
const Result = @import("../../Core/Result.zig").Result;
const ObjectToIDMap = @import("../../Core/ObjectToIDMap.zig").ObjectToIDMap;
const IDToObjectMap = @import("../../Core/ObjectToIDMap.zig").IDToObjectMap;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeResult = ShapeFile.ShapeResult;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const GroupFilter = @import("../Collision/GroupFilter.zig").GroupFilter;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const MotionType = @import("MotionType.zig").MotionType;
const MotionQuality = @import("MotionQuality.zig").MotionQuality;
const AllowedDOFs = @import("AllowedDOFs.zig").AllowedDOFs;
const MassProperties = @import("MassProperties.zig").MassProperties;

const log = std.log.scoped(.zolt);

/// Errors of creating a body (BodyManager.allocateBody, BodyInterface.createBody, Body.applyBodyCreationSettings,
/// BodyCreationSettings.getMassProperties): the settings have no (valid) shape, which Jolt dereferences, or out of memory
pub const CreateBodyError = error{
    /// The settings have no shape or the shape could not be created (Jolt crashes)
    InvalidShape,
} || Allocator.Error;

/// Enum used in BodyCreationSettings to indicate how mass and inertia should be calculated (EOverrideMassProperties)
pub const OverrideMassProperties = enum(u8) {
    /// Tells the system to calculate the mass and inertia based on density
    calculate_mass_and_inertia,
    /// Tells the system to take the mass from mMassPropertiesOverride and to calculate the inertia based on density of the shapes and to scale it to the provided mass
    calculate_inertia,
    /// Tells the system to take the mass and inertia from mMassPropertiesOverride
    mass_and_inertia_provided,
};

/// Settings for constructing a rigid body
pub const BodyCreationSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL

    pub const GroupFilterToIDMap = ObjectToIDMap(GroupFilter);
    pub const IDToGroupFilterMap = IDToObjectMap(GroupFilter);
    pub const ShapeToIDMap = Shape.ShapeToIDMap;
    pub const IDToShapeMap = Shape.IDToShapeMap;
    pub const MaterialToIDMap = ObjectToIDMap(PhysicsMaterial);
    pub const IDToMaterialMap = IDToObjectMap(PhysicsMaterial);

    pub const BCSResult = Result(BodyCreationSettings);

    /// Collision volume for the body: Jolt's mutually exclusive `mShape` / `mShapePtr`
    pub const ShapeSource = union(enum) {
        /// No shape (both null)
        none,
        /// Shape settings, can be serialized (mShape). `Ref`, not Jolt's RefConst: creating the shape writes the settings' cache (Rule M, CollisionArchitecture D3)
        settings: Ref(ShapeSettings),
        /// Actual shape, cannot be serialized (mShapePtr)
        shape: RefConst(Shape),
    };

    /// Position of the body (not of the center of mass)
    position: RVec3 = RVec3.zero(),
    /// Rotation of the body
    rotation: Quat = Quat.identity(),
    /// World space linear velocity of the center of mass (m/s)
    linear_velocity: Vec3 = Vec3.zero(),
    /// World space angular velocity (rad/s)
    angular_velocity: Vec3 = Vec3.zero(),

    /// User data value (can be used by application)
    user_data: u64 = 0,

    ///@name Collision settings
    /// The collision layer this body belongs to (determines if two objects can collide)
    object_layer: ObjectLayer = 0,
    /// The collision group this body belongs to (determines if two objects can collide)
    collision_group: CollisionGroup = .{},

    ///@name Simulation properties
    /// Motion type, determines if the object is static, dynamic or kinematic
    motion_type: MotionType = .dynamic,
    /// Which degrees of freedom this body has (can be used to limit simulation to 2D)
    allowed_dofs: AllowedDOFs = .all,
    /// When this body is created as static, this setting tells the system to create a MotionProperties object so that the object can be switched to kinematic or dynamic
    allow_dynamic_or_kinematic: bool = false,
    /// If this body is a sensor. A sensor will receive collision callbacks, but will not cause any collision responses and can be used as a trigger volume. See description at Body::SetIsSensor.
    is_sensor: bool = false,
    /// If kinematic objects can generate contact points against other kinematic or static objects. See description at Body::SetCollideKinematicVsNonDynamic.
    collide_kinematic_vs_non_dynamic: bool = false,
    /// If this body should use manifold reduction (see description at Body::SetUseManifoldReduction)
    use_manifold_reduction: bool = true,
    /// Set to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
    apply_gyroscopic_force: bool = false,
    /// Motion quality, or how well it detects collisions when it has a high velocity
    motion_quality: MotionQuality = .discrete,
    /// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
    enhanced_internal_edge_removal: bool = false,
    /// If this body can go to sleep or not
    allow_sleeping: bool = true,
    /// Friction of the body (dimensionless number, usually between 0 and 1, 0 = no friction, 1 = friction force equals force that presses the two bodies together). Note that bodies can have negative friction but the combined friction (see PhysicsSystem::SetCombineFriction) should never go below zero.
    friction: f32 = 0.2,
    /// Restitution of body (dimensionless number, usually between 0 and 1, 0 = completely inelastic collision response, 1 = completely elastic collision response). Note that bodies can have negative restitution but the combined restitution (see PhysicsSystem::SetCombineRestitution) should never go below zero.
    restitution: f32 = 0.0,
    /// Linear damping: dv/dt = -c * v. c. Value should be zero or positive and is usually close to 0.
    linear_damping: f32 = 0.05,
    /// Angular damping: dw/dt = -c * w. c. Value should be zero or positive and is usually close to 0.
    angular_damping: f32 = 0.05,
    /// Maximum linear velocity that this body can reach (m/s)
    max_linear_velocity: f32 = 500.0,
    /// Maximum angular velocity that this body can reach (rad/s)
    max_angular_velocity: f32 = 0.25 * math.pi * 60.0,
    /// Value to multiply gravity with for this body
    gravity_factor: f32 = 1.0,
    /// Used only when this body is dynamic and colliding. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_velocity_steps_override: u32 = 0,
    /// Used only when this body is dynamic and colliding. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_position_steps_override: u32 = 0,

    ///@name Mass properties of the body (by default calculated by the shape)
    /// Determines how mMassPropertiesOverride will be used
    override_mass_properties: OverrideMassProperties = .calculate_mass_and_inertia,
    /// When calculating the inertia (not when it is provided) the calculated inertia will be multiplied by this value
    inertia_multiplier: f32 = 1.0,
    /// Contains replacement mass settings which override the automatically calculated values
    mass_properties_override: MassProperties = .{},

    /// Collision volume for the body (private in Jolt, use the accessors)
    shape: ShapeSource = .none,

    /// Constructor BodyCreationSettings(const ShapeSettings *, position, rotation, motion type, object layer)
    pub fn initWithSettings(shape: ?*ShapeSettings, position: RVec3, rotation: Quat, motion_type: MotionType, object_layer: ObjectLayer) BodyCreationSettings {
        return .{ .position = position, .rotation = rotation, .object_layer = object_layer, .motion_type = motion_type, .shape = if (shape) |s| .{ .settings = .init(s) } else .none };
    }

    /// Constructor BodyCreationSettings(const Shape *, position, rotation, motion type, object layer)
    pub fn initWithShape(shape: ?*const Shape, position: RVec3, rotation: Quat, motion_type: MotionType, object_layer: ObjectLayer) BodyCreationSettings {
        return .{ .position = position, .rotation = rotation, .object_layer = object_layer, .motion_type = motion_type, .shape = if (shape) |s| .{ .shape = .init(s) } else .none };
    }

    /// Copy constructor: adds references to the shape (settings) and the group filter
    pub fn clone(self: *const BodyCreationSettings) BodyCreationSettings {
        var result = self.*;
        result.shape = switch (self.shape) {
            .none => .none,
            .settings => |s| .{ .settings = s.clone() },
            .shape => |s| .{ .shape = s.clone() },
        };
        result.collision_group = self.collision_group.clone();
        return result;
    }

    /// Destructor: releases the references
    pub fn deinit(self: *BodyCreationSettings) void {
        self.releaseShape();
        self.collision_group.deinit();
    }

    /// Release the shape (settings): mShape = nullptr; mShapePtr = nullptr
    fn releaseShape(self: *BodyCreationSettings) void {
        switch (self.shape) {
            .none => {},
            .settings => |*s| s.deinit(),
            .shape => |*s| s.deinit(),
        }
        self.shape = .none;
    }

    /// Test if two BodyCreationSettings are equal
    pub fn eql(self: *const BodyCreationSettings, rhs: *const BodyCreationSettings) bool {
        return self.position.eql(rhs.position) and
            self.rotation.eql(rhs.rotation) and
            self.linear_velocity.eql(rhs.linear_velocity) and
            self.angular_velocity.eql(rhs.angular_velocity) and
            self.user_data == rhs.user_data and
            self.object_layer == rhs.object_layer and
            self.collision_group.eql(&rhs.collision_group) and
            self.motion_type == rhs.motion_type and
            self.allowed_dofs.eql(rhs.allowed_dofs) and
            self.allow_dynamic_or_kinematic == rhs.allow_dynamic_or_kinematic and
            self.is_sensor == rhs.is_sensor and
            self.collide_kinematic_vs_non_dynamic == rhs.collide_kinematic_vs_non_dynamic and
            self.use_manifold_reduction == rhs.use_manifold_reduction and
            self.apply_gyroscopic_force == rhs.apply_gyroscopic_force and
            self.motion_quality == rhs.motion_quality and
            self.enhanced_internal_edge_removal == rhs.enhanced_internal_edge_removal and
            self.allow_sleeping == rhs.allow_sleeping and
            self.friction == rhs.friction and
            self.restitution == rhs.restitution and
            self.linear_damping == rhs.linear_damping and
            self.angular_damping == rhs.angular_damping and
            self.max_linear_velocity == rhs.max_linear_velocity and
            self.max_angular_velocity == rhs.max_angular_velocity and
            self.gravity_factor == rhs.gravity_factor and
            self.num_velocity_steps_override == rhs.num_velocity_steps_override and
            self.num_position_steps_override == rhs.num_position_steps_override and
            self.override_mass_properties == rhs.override_mass_properties and
            self.inertia_multiplier == rhs.inertia_multiplier and
            self.mass_properties_override.eql(&rhs.mass_properties_override) and
            self.getShapeSettings() == rhs.getShapeSettings() and
            self.getShapePtr() == rhs.getShapePtr();
    }

    /// The `mShapePtr` member (null when the settings hold shape settings or nothing)
    fn getShapePtr(self: *const BodyCreationSettings) ?*const Shape {
        return switch (self.shape) {
            .shape => |s| s.get(),
            else => null,
        };
    }

    /// Access to the shape settings object. This contains serializable (non-runtime optimized) information about the Shape.
    pub fn getShapeSettings(self: *const BodyCreationSettings) ?*ShapeSettings {
        return switch (self.shape) {
            .settings => |s| s.get(),
            else => null,
        };
    }

    /// Sets the shape settings and invalidates the shape (SetShapeSettings: mShape = inShape; mShapePtr = nullptr)
    pub fn setShapeSettings(self: *BodyCreationSettings, shape: ?*ShapeSettings) void {
        const new: ShapeSource = if (shape) |s| .{ .settings = .init(s) } else .none; // Reference first: the old and the new object may be the same
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
                    self.setShape(result.getPtr().?) // mShapePtr = result.Get(); mShape = nullptr
                else
                    self.releaseShape(); // mShape = nullptr
                return result;
            },
        }
    }

    /// Access to the run-time shape object. Will convert from ShapeSettings object if needed.
    pub fn getShape(self: *const BodyCreationSettings, allocator: Allocator) Allocator.Error!?*const Shape {
        switch (self.shape) {
            // If we already have a shape, return it
            .shape => |s| return s.get(),

            // Check if we have shape settings
            .none => return null,

            // Create the shape
            .settings => |s| {
                var result = try s.get().?.createShape(allocator);
                defer result.deinit();
                if (result.isValid())
                    return result.getPtr().?; // The shape settings' cache keeps the shape alive

                log.info("Error: {s}", .{result.getError()});
                if (Core.enable_asserts) @panic("An error occurred during shape creation. Use ConvertShapeSettings() to convert the shape and get the error!");
                return null;
            },
        }
    }

    /// Sets the shape and invalidates the shape settings (SetShape: mShapePtr = inShape; mShape = nullptr)
    pub fn setShape(self: *BodyCreationSettings, shape: ?*const Shape) void {
        const new: ShapeSource = if (shape) |s| .{ .shape = .init(s) } else .none; // Reference first: the old and the new object may be the same
        self.releaseShape();
        self.shape = new;
    }

    /// Check if the mass properties of this body will be calculated (only relevant for kinematic or dynamic objects that need a MotionProperties object)
    pub fn hasMassProperties(self: *const BodyCreationSettings) bool {
        return self.allow_dynamic_or_kinematic or self.motion_type != .static;
    }

    /// Calculate (or return when overridden) the mass and inertia for this body.
    /// Zolt: `error.InvalidShape` where Jolt dereferences a null GetShape() (the calculated modes without a shape).
    pub fn getMassProperties(self: *const BodyCreationSettings, allocator: Allocator) CreateBodyError!MassProperties {
        // Calculate mass properties
        var mass_properties: MassProperties = .{};
        switch (self.override_mass_properties) {
            .calculate_mass_and_inertia => {
                const shape = (try self.getShape(allocator)) orelse return error.InvalidShape;
                mass_properties = shape.getMassProperties();
                mass_properties.inertia = mass_properties.inertia.mulScalar(self.inertia_multiplier);
                mass_properties.inertia.set(3, 3, 1.0);
            },
            .calculate_inertia => {
                const shape = (try self.getShape(allocator)) orelse return error.InvalidShape;
                mass_properties = shape.getMassProperties();
                mass_properties.scaleToMass(self.mass_properties_override.mass);
                mass_properties.inertia = mass_properties.inertia.mulScalar(self.inertia_multiplier);
                mass_properties.inertia.set(3, 3, 1.0);
            },
            .mass_and_inertia_provided => {
                mass_properties = self.mass_properties_override;
            },
        }
        return mass_properties;
    }
};

test "BodyCreationSettings defaults" {
    var settings: BodyCreationSettings = .{};
    defer settings.deinit();
    try std.testing.expect(settings.motion_type == .dynamic);
    try std.testing.expect(settings.allowed_dofs.eql(.all));
    try std.testing.expect(settings.use_manifold_reduction and settings.allow_sleeping and !settings.is_sensor);
    try std.testing.expectEqual(@as(f32, 0.2), settings.friction);
    try std.testing.expectEqual(@as(f32, 0.05), settings.linear_damping);
    try std.testing.expectEqual(@as(f32, 0.25 * math.pi * 60.0), settings.max_angular_velocity);
    try std.testing.expect(settings.override_mass_properties == .calculate_mass_and_inertia);
    try std.testing.expect(settings.hasMassProperties());
    settings.motion_type = .static;
    try std.testing.expect(!settings.hasMassProperties());
    settings.allow_dynamic_or_kinematic = true;
    try std.testing.expect(settings.hasMassProperties());
    var copy = settings.clone();
    defer copy.deinit();
    try std.testing.expect(copy.eql(&settings));
    copy.friction = 0.5;
    try std.testing.expect(!copy.eql(&settings));
}
