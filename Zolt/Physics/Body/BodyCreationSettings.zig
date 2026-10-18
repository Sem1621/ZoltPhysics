//! Port of: Jolt/Physics/Body/BodyCreationSettings.h, Jolt/Physics/Body/BodyCreationSettings.cpp
//! Status: complete
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
//! - The binary state (`saveBinaryState`, `restoreBinaryState`, `saveWithChildren`, `restoreWithChildren`) writes
//!   Jolt's bytes. `saveWithChildren` / `restoreWithChildren` use the Phase 4 ID maps (Shape.saveWithChildren /
//!   restoreWithChildren, StreamUtils' SaveObjectReference / RestoreObjectReference for the group filter) and return
//!   Jolt's `BCSResult` value with Jolt's error texts; allocation failure is `error.OutOfMemory`. `saveWithChildren`
//!   resolves the shape before it writes anything: without a shape (Jolt dereferences a null GetShape()) it returns
//!   `error.InvalidShape` and the stream is unchanged.
//! - Zolt: `restoreBinaryState` reads the enums (`EMotionType`, `EMotionQuality`, `EOverrideMassProperties`) as their
//!   tag integers and the bools as bytes (`!= 0`). Jolt stores any byte; a byte that is not a value of the Zig enum
//!   leaves the field unchanged, and `restoreWithChildren` then reports Jolt's "Error reading body creation settings"
//!   (only a corrupt stream does this, Jolt would continue with an invalid enum value).
//! - TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL.

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
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
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const StreamUtils = @import("../../Core/StreamUtils.zig");
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
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(BodyCreationSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "BodyCreationSettings",
        .kind = .non_virtual,
    });
    /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
    pub const serialization_pending = true;

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

    /// Saves the state of this object in binary form to inStream. Doesn't store the shape nor the group filter.
    pub fn saveBinaryState(self: *const BodyCreationSettings, stream: StreamOut) void {
        stream.write(self.position);
        stream.write(self.rotation);
        stream.write(self.linear_velocity);
        stream.write(self.angular_velocity);
        self.collision_group.saveBinaryState(stream);
        stream.write(self.object_layer);
        stream.write(self.motion_type);
        stream.write(self.allowed_dofs);
        stream.write(self.allow_dynamic_or_kinematic);
        stream.write(self.is_sensor);
        stream.write(self.collide_kinematic_vs_non_dynamic);
        stream.write(self.use_manifold_reduction);
        stream.write(self.apply_gyroscopic_force);
        stream.write(self.motion_quality);
        stream.write(self.enhanced_internal_edge_removal);
        stream.write(self.allow_sleeping);
        stream.write(self.friction);
        stream.write(self.restitution);
        stream.write(self.linear_damping);
        stream.write(self.angular_damping);
        stream.write(self.max_linear_velocity);
        stream.write(self.max_angular_velocity);
        stream.write(self.gravity_factor);
        stream.write(self.num_velocity_steps_override);
        stream.write(self.num_position_steps_override);
        stream.write(self.override_mass_properties);
        stream.write(self.inertia_multiplier);
        self.mass_properties_override.saveBinaryState(stream);
    }

    /// Restore the state of this object from inStream. Doesn't restore the shape nor the group filter.
    pub fn restoreBinaryState(self: *BodyCreationSettings, stream: StreamIn) void {
        _ = self.restoreBinaryStateChecked(stream);
    }

    /// RestoreBinaryState, returns false when an enum integer was not a value of its enum (Zolt, see the file header)
    fn restoreBinaryStateChecked(self: *BodyCreationSettings, stream: StreamIn) bool {
        var valid = true;
        stream.read(&self.position);
        stream.read(&self.rotation);
        stream.read(&self.linear_velocity);
        stream.read(&self.angular_velocity);
        self.collision_group.restoreBinaryState(stream);
        stream.read(&self.object_layer);
        valid = readEnum(MotionType, stream, &self.motion_type) and valid;
        stream.read(&self.allowed_dofs);
        readBool(stream, &self.allow_dynamic_or_kinematic);
        readBool(stream, &self.is_sensor);
        readBool(stream, &self.collide_kinematic_vs_non_dynamic);
        readBool(stream, &self.use_manifold_reduction);
        readBool(stream, &self.apply_gyroscopic_force);
        valid = readEnum(MotionQuality, stream, &self.motion_quality) and valid;
        readBool(stream, &self.enhanced_internal_edge_removal);
        readBool(stream, &self.allow_sleeping);
        stream.read(&self.friction);
        stream.read(&self.restitution);
        stream.read(&self.linear_damping);
        stream.read(&self.angular_damping);
        stream.read(&self.max_linear_velocity);
        stream.read(&self.max_angular_velocity);
        stream.read(&self.gravity_factor);
        stream.read(&self.num_velocity_steps_override);
        stream.read(&self.num_position_steps_override);
        valid = readEnum(OverrideMassProperties, stream, &self.override_mass_properties) and valid;
        stream.read(&self.inertia_multiplier);
        self.mass_properties_override.restoreBinaryState(stream);
        return valid;
    }

    /// Read an enum as its tag integer (Jolt reads the raw byte). Keeps the current value and returns false when the
    /// integer is not a value of E.
    fn readEnum(comptime E: type, stream: StreamIn, value: *E) bool {
        var raw: @typeInfo(E).@"enum".tag_type = @intFromEnum(value.*);
        stream.read(&raw);
        value.* = std.enums.fromInt(E, raw) orelse return false;
        return true;
    }

    /// Read a bool as a byte (Jolt reads the raw byte into its bool)
    fn readBool(stream: StreamIn, value: *bool) void {
        var raw: u8 = @intFromBool(value.*);
        stream.read(&raw);
        value.* = raw != 0;
    }

    /// Save body creation settings, its shape, materials and group filter. Pass in an empty map in shape_map / material_map / group_filter_map or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
    /// Pass null to shape_map and material_map to skip saving shapes
    /// Pass null to group_filter_map to skip saving group filters
    /// (Zolt: `allocator` creates the shape from shape settings and grows the maps. When the shapes are saved and there
    /// is no shape, Jolt dereferences a null GetShape(): Zolt returns `error.InvalidShape` and writes nothing.)
    pub fn saveWithChildren(self: *const BodyCreationSettings, allocator: Allocator, stream: StreamOut, shape_map: ?*ShapeToIDMap, material_map: ?*MaterialToIDMap, group_filter_map: ?*GroupFilterToIDMap) CreateBodyError!void {
        // Zolt: resolve the shape before anything is written (Jolt calls GetShape() after SaveBinaryState)
        const save_shapes = shape_map != null and material_map != null;
        const shape: ?*const Shape = if (save_shapes) (try self.getShape(allocator)) orelse return error.InvalidShape else null;

        // Save creation settings
        self.saveBinaryState(stream);

        // Save shape
        if (save_shapes)
            try shape.?.saveWithChildren(allocator, stream, shape_map.?, material_map.?)
        else
            stream.write(~@as(u32, 0));

        // Save group filter
        try StreamUtils.saveObjectReference(GroupFilter, allocator, stream, self.collision_group.getGroupFilter(), group_filter_map);
    }

    /// Restore body creation settings, its shape, materials and group filter. Pass in an empty map in shape_map / material_map / group_filter_map or reuse the same map while reading multiple shapes from the same stream in order to restore duplicates.
    /// (Jolt: sRestoreWithChildren. The result owns the settings, `deinit` it.)
    pub fn restoreWithChildren(allocator: Allocator, stream: StreamIn, shape_map: *IDToShapeMap, material_map: *IDToMaterialMap, group_filter_map: *IDToGroupFilterMap) Allocator.Error!BCSResult {
        var result: BCSResult = .empty;

        // Read creation settings
        var settings: BodyCreationSettings = .{};
        defer settings.deinit();
        const valid_enums = settings.restoreBinaryStateChecked(stream);
        if (stream.isEOF() or stream.isFailed() or !valid_enums) {
            result.setError("Error reading body creation settings");
            return result;
        }

        // Read shape
        var shape_result = try Shape.restoreWithChildren(allocator, stream, shape_map, material_map);
        defer shape_result.deinit();
        if (shape_result.hasError()) {
            result.setError(shape_result.getError());
            return result;
        }
        settings.setShape(shape_result.getPtr());

        // Read group filter
        var gfresult = try StreamUtils.restoreObjectReference(GroupFilter, allocator, stream, group_filter_map);
        defer gfresult.deinit();
        if (gfresult.hasError()) {
            result.setError(gfresult.getError());
            return result;
        }
        settings.collision_group.setGroupFilter(gfresult.getPtr());

        result.set(settings.clone());
        return result;
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

// ---------------------------------------------------------------------------------------------------------------------
// Tests of the binary state

const testing = std.testing;
const virtual = @import("../../Core/Virtual.zig");
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const Color = @import("../../Core/Color.zig").Color;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Real = @import("../../Math/Real.zig").Real;
const BoxShape = @import("../Collision/Shape/BoxShape.zig").BoxShape;
const SphereShapeSettings = @import("../Collision/Shape/SphereShape.zig").SphereShapeSettings;
const GroupFilterTable = @import("../Collision/GroupFilterTable.zig").GroupFilterTable;
const PhysicsMaterialSimple = @import("../Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;

/// Settings with every serialized field different from its default
fn testSettings() BodyCreationSettings {
    return .{
        .position = RVec3.init(1, 2, 3),
        .rotation = Quat.init(0.5, -0.5, 0.5, 0.5),
        .linear_velocity = Vec3.init(4, 5, 6),
        .angular_velocity = Vec3.init(-1, -2, -3),
        .user_data = 0, // Not serialized
        .object_layer = 3,
        .collision_group = .init(null, 7, 9),
        .motion_type = .kinematic,
        .allowed_dofs = .plane_2d,
        .allow_dynamic_or_kinematic = true,
        .is_sensor = true,
        .collide_kinematic_vs_non_dynamic = true,
        .use_manifold_reduction = false,
        .apply_gyroscopic_force = true,
        .motion_quality = .linear_cast,
        .enhanced_internal_edge_removal = true,
        .allow_sleeping = false,
        .friction = 0.7,
        .restitution = 0.3,
        .linear_damping = 0.1,
        .angular_damping = 0.2,
        .max_linear_velocity = 100,
        .max_angular_velocity = 10,
        .gravity_factor = 2,
        .num_velocity_steps_override = 11,
        .num_position_steps_override = 13,
        .override_mass_properties = .mass_and_inertia_provided,
        .inertia_multiplier = 1.5,
        .mass_properties_override = .{ .mass = 12, .inertia = Mat44.scaleVec3(Vec3.init(1, 2, 3)) },
    };
}

/// Byte offset of mMotionType in the binary state
const motion_type_offset = 3 * @sizeOf(Real) + 16 + 12 + 12 + 8 + @sizeOf(ObjectLayer);

/// Size of the binary state: position, rotation, velocities, group ids, object layer, 10 bytes of enums and bools, 7
/// floats, 2 step counts, the mass properties override mode, the inertia multiplier and the mass properties
const binary_state_size = motion_type_offset + 10 + 28 + 8 + 1 + 4 + 68;

test "BodyCreationSettings: saveBinaryState / restoreBinaryState" {
    var settings = testSettings();
    defer settings.deinit();

    var buffer: [512]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    settings.saveBinaryState(out.streamOut());
    const bytes = writer.buffered();
    try testing.expectEqual(@as(usize, binary_state_size), bytes.len);
    try testing.expectEqual(@as(u8, @intFromEnum(MotionType.kinematic)), bytes[motion_type_offset]);

    // Restore into default settings: everything but the shape, the group filter and the user data
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var restored: BodyCreationSettings = .{};
    defer restored.deinit();
    restored.restoreBinaryState(in.streamIn());
    try testing.expect(!in.streamIn().isEOF() and !in.streamIn().isFailed());
    try testing.expect(restored.eql(&settings));

    // An enum byte that is not a value of the enum leaves the field unchanged (Zolt)
    var corrupt = buffer;
    corrupt[motion_type_offset] = 3;
    var corrupt_reader: std.Io.Reader = .fixed(corrupt[0..bytes.len]);
    var corrupt_in = StreamWrapper.StreamInWrapper.init(&corrupt_reader);
    var corrupt_restored: BodyCreationSettings = .{};
    defer corrupt_restored.deinit();
    corrupt_restored.restoreBinaryState(corrupt_in.streamIn());
    try testing.expect(corrupt_restored.motion_type == .dynamic);
    try testing.expectEqual(@as(f32, 0.7), corrupt_restored.friction);
}

/// The maps of a restore, released by deinit
const RestoreMaps = struct {
    shapes: BodyCreationSettings.IDToShapeMap = .empty,
    materials: BodyCreationSettings.IDToMaterialMap = .empty,
    group_filters: BodyCreationSettings.IDToGroupFilterMap = .empty,

    fn deinit(self: *RestoreMaps, allocator: Allocator) void {
        for (self.shapes.items) |*s| s.deinit();
        self.shapes.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
        for (self.group_filters.items) |*g| g.deinit();
        self.group_filters.deinit(allocator);
    }
};

/// The maps of a save
const SaveMaps = struct {
    shapes: BodyCreationSettings.ShapeToIDMap = .empty,
    materials: BodyCreationSettings.MaterialToIDMap = .empty,
    group_filters: BodyCreationSettings.GroupFilterToIDMap = .empty,

    fn deinit(self: *SaveMaps, allocator: Allocator) void {
        self.shapes.deinit(allocator);
        self.materials.deinit(allocator);
        self.group_filters.deinit(allocator);
    }
};

/// Three settings that share a shape, a material and a group filter
const ChildrenScene = struct {
    material: RefConst(PhysicsMaterial) = .empty,
    filter: RefConst(GroupFilter) = .empty,
    shape: RefConst(Shape) = .empty,
    sphere_settings: Ref(ShapeSettings) = .empty,
    settings: [3]BodyCreationSettings = @splat(.{}),

    fn init(self: *ChildrenScene, allocator: Allocator) !void {
        errdefer self.deinit();
        const material = try PhysicsMaterialSimple.create(allocator, "Wood", Color.orange);
        self.material = .init(material.material());
        const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 5 });
        self.filter = .init(filter.asGroupFilter());
        filter.disableCollision(1, 3);
        const box = try BoxShape.create(allocator, Vec3.init(1, 2, 3), .{ .material = material.material() });
        self.shape = .init(box.asShape());
        const sphere = try SphereShapeSettings.create(allocator, 0.5, .{ .material = material.material() });
        self.sphere_settings = .init(sphere.asShapeSettings());

        self.settings[0] = testSettings();
        self.settings[0].setShape(box.asShape());
        self.settings[0].collision_group.setGroupFilter(filter.asGroupFilter());
        self.settings[1] = .initWithShape(box.asShape(), RVec3.init(5, 6, 7), Quat.identity(), .static, 1);
        self.settings[1].collision_group = .init(filter.asGroupFilter(), 1, 2);
        self.settings[2] = .initWithSettings(sphere.asShapeSettings(), RVec3.init(-1, 0, 0), Quat.identity(), .dynamic, 2);
    }

    fn deinit(self: *ChildrenScene) void {
        for (&self.settings) |*s| s.deinit();
        self.sphere_settings.deinit();
        self.shape.deinit();
        self.filter.deinit();
        self.material.deinit();
    }
};

/// saveWithChildren of the settings into `buffer` (`with_shapes` / `with_filters` select the maps), returns the bytes
fn saveChildren(allocator: Allocator, settings: []const BodyCreationSettings, buffer: []u8, maps: *SaveMaps, with_shapes: bool, with_filters: bool) ![]const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    for (settings) |*s|
        try s.saveWithChildren(allocator, out.streamOut(), if (with_shapes) &maps.shapes else null, if (with_shapes) &maps.materials else null, if (with_filters) &maps.group_filters else null);
    return writer.buffered();
}

/// restoreWithChildren of `out.len` settings, every result must be valid
fn restoreChildren(allocator: Allocator, bytes: []const u8, maps: *RestoreMaps, out: []BodyCreationSettings) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    for (out) |*o| {
        var result = try BodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.shapes, &maps.materials, &maps.group_filters);
        defer result.deinit();
        if (!result.isValid()) return error.TestUnexpectedResult;
        o.deinit();
        o.* = result.get().clone();
    }
}

test "BodyCreationSettings: saveWithChildren / restoreWithChildren with shared shapes, materials and group filters" {
    const allocator = testing.allocator;
    var scene: ChildrenScene = .{};
    try scene.init(allocator);
    defer scene.deinit();

    // Save with all maps: the shared box, the material and the group filter are written once
    var buffer: [4096]u8 = undefined;
    var save_maps: SaveMaps = .{};
    defer save_maps.deinit(allocator);
    const bytes = try saveChildren(allocator, &scene.settings, &buffer, &save_maps, true, true);
    try testing.expectEqual(@as(u32, 2), save_maps.shapes.count());
    try testing.expectEqual(@as(u32, 1), save_maps.materials.count());
    try testing.expectEqual(@as(u32, 1), save_maps.group_filters.count());

    var maps: RestoreMaps = .{};
    defer maps.deinit(allocator);
    var restored: [3]BodyCreationSettings = @splat(.{});
    defer for (&restored) |*r| r.deinit();
    try restoreChildren(allocator, bytes, &maps, &restored);
    try testing.expectEqual(@as(usize, 2), maps.shapes.items.len);
    try testing.expectEqual(@as(usize, 1), maps.materials.items.len);
    try testing.expectEqual(@as(usize, 1), maps.group_filters.items.len);

    // The values, the sharing and the restored objects
    var expected = scene.settings[0].clone();
    defer expected.deinit();
    expected.setShape(restored[0].getShapePtr());
    expected.collision_group.setGroupFilter(restored[0].collision_group.getGroupFilter());
    try testing.expect(restored[0].eql(&expected));
    try testing.expect(restored[0].getShapePtr().? == restored[1].getShapePtr().?);
    try testing.expect(restored[0].getShapePtr().? != scene.shape.get().?);
    try testing.expect(restored[0].getShapePtr().?.cast(BoxShape).half_extent.eql(Vec3.init(1, 2, 3)));
    try testing.expect(restored[0].collision_group.getGroupFilter().? == restored[1].collision_group.getGroupFilter().?);
    const filter = virtual.downcast(GroupFilterTable, restored[0].collision_group.getGroupFilter().?);
    try testing.expect(!filter.isCollisionEnabled(1, 3) and filter.isCollisionEnabled(1, 2));
    try testing.expect(restored[2].collision_group.getGroupFilter() == null);
    try testing.expect(restored[2].getShapeSettings() == null); // The shape settings were converted to a shape
    try testing.expectEqual(@as(f32, 0.5), restored[2].getShapePtr().?.getInnerRadius());
    try testing.expectEqualStrings("Wood", restored[2].getShapePtr().?.getMaterial(.empty).getDebugName());
    try testing.expect(restored[1].position.eql(RVec3.init(5, 6, 7)) and restored[1].motion_type == .static);

    // Without maps: no shapes (null) and no group filters
    var save_maps2: SaveMaps = .{};
    defer save_maps2.deinit(allocator);
    var buffer2: [4096]u8 = undefined;
    const bytes2 = try saveChildren(allocator, &scene.settings, &buffer2, &save_maps2, false, false);
    try testing.expectEqual(@as(u32, 0), save_maps2.shapes.count());
    try testing.expectEqual(3 * (binary_state_size + 8), bytes2.len);
    var maps2: RestoreMaps = .{};
    defer maps2.deinit(allocator);
    var restored2: [3]BodyCreationSettings = @splat(.{});
    defer for (&restored2) |*r| r.deinit();
    try restoreChildren(allocator, bytes2, &maps2, &restored2);
    for (&restored2) |*r| {
        try testing.expect(r.shape == .none);
        try testing.expect(r.collision_group.getGroupFilter() == null);
    }
    try testing.expectEqual(@as(u32, 9), restored2[0].collision_group.getSubGroupID());
}

test "BodyCreationSettings: restoreWithChildren errors" {
    const allocator = testing.allocator;
    var scene: ChildrenScene = .{};
    try scene.init(allocator);
    defer scene.deinit();

    var buffer: [4096]u8 = undefined;
    var save_maps: SaveMaps = .{};
    defer save_maps.deinit(allocator);
    const bytes = try saveChildren(allocator, scene.settings[0..1], &buffer, &save_maps, true, true);

    // Truncated streams fail with one of Jolt's texts at every length, nothing leaks
    for (0..bytes.len) |len| {
        var reader: std.Io.Reader = .fixed(bytes[0..len]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var maps: RestoreMaps = .{};
        defer maps.deinit(allocator);
        var result = try BodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.shapes, &maps.materials, &maps.group_filters);
        defer result.deinit();
        if (len < binary_state_size) {
            try testing.expectEqualStrings("Error reading body creation settings", result.getError());
        } else if (len < binary_state_size + 4) {
            try testing.expectEqualStrings("Failed to read shape id", result.getError());
        } else {
            // A shape that cannot be read, or a group filter that cannot be read (Jolt does not check the EOF of the
            // filter ID, a missing ID is the null filter)
            try testing.expect(result.hasError() or result.get().collision_group.getGroupFilter() == null);
        }
    }

    // An enum value that does not exist
    var corrupt = buffer;
    corrupt[motion_type_offset] = 9;
    {
        var reader: std.Io.Reader = .fixed(corrupt[0..bytes.len]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var maps: RestoreMaps = .{};
        defer maps.deinit(allocator);
        var result = try BodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.shapes, &maps.materials, &maps.group_filters);
        defer result.deinit();
        try testing.expectEqualStrings("Error reading body creation settings", result.getError());
    }

    // A group filter of an unknown type: the RTTI hash follows the filter ID 0 at the end of the stream
    const filter_hash_offset = blk: {
        const hash = GroupFilterTable.rtti.hash;
        var i: usize = bytes.len - 4;
        while (i > binary_state_size) : (i -= 1)
            if (std.mem.readInt(u32, bytes[i..][0..4], .little) == hash and std.mem.readInt(u32, bytes[i - 4 ..][0..4], .little) == 0) break :blk i;
        return error.TestUnexpectedResult;
    };
    corrupt = buffer;
    std.mem.writeInt(u32, corrupt[filter_hash_offset..][0..4], 0x12345678, .little);
    {
        var reader: std.Io.Reader = .fixed(corrupt[0..bytes.len]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var maps: RestoreMaps = .{};
        defer maps.deinit(allocator);
        var result = try BodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.shapes, &maps.materials, &maps.group_filters);
        defer result.deinit();
        try testing.expectEqualStrings("Failed to create instance of type", result.getError());
    }

    // Saving the shapes without a shape (Jolt dereferences null): error.InvalidShape, nothing is written
    var no_shape: BodyCreationSettings = .{};
    defer no_shape.deinit();
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    try testing.expectError(error.InvalidShape, no_shape.saveWithChildren(allocator, out.streamOut(), &save_maps.shapes, &save_maps.materials, null));
    try testing.expectEqual(@as(usize, 0), writer.buffered().len);
    // Without the shape maps it saves the null shape
    try no_shape.saveWithChildren(allocator, out.streamOut(), null, null, null);
    try testing.expectEqual(@as(usize, binary_state_size + 8), writer.buffered().len);
}

fn saveRestoreChildren(allocator: Allocator) !void {
    var scene: ChildrenScene = .{};
    try scene.init(allocator);
    defer scene.deinit();

    var buffer: [4096]u8 = undefined;
    var save_maps: SaveMaps = .{};
    defer save_maps.deinit(allocator);
    const bytes = try saveChildren(allocator, &scene.settings, &buffer, &save_maps, true, true);

    var maps: RestoreMaps = .{};
    defer maps.deinit(allocator);
    var restored: [3]BodyCreationSettings = @splat(.{});
    defer for (&restored) |*r| r.deinit();
    try restoreChildren(allocator, bytes, &maps, &restored);
}

test "BodyCreationSettings: out of memory in saveWithChildren / restoreWithChildren" {
    try testing.checkAllAllocationFailures(testing.allocator, saveRestoreChildren, .{});
}
