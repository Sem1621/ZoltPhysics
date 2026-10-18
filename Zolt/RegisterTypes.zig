//! Port of: Jolt/RegisterTypes.h, Jolt/RegisterTypes.cpp
//! Status: complete
//!
//! D4 (Docs/Zolt/CollisionArchitecture.md): Jolt's RegisterTypes() fills mutable static tables at startup
//! (CollisionDispatch::sInit, every shape's sRegister, the Factory and PhysicsMaterial::sDefault). Zolt replays the
//! same registration code at compile time and keeps the final state as immutable constants, so there is nothing to
//! call before using the library:
//! - `registration_order` lists the classes in the order of RegisterTypes.cpp (Jolt registrations only). Every class has
//!   a `register(comptime r: *Registry)` that is a line by line port of its sRegister (an empty one while the shape is a
//!   stub). Shape porters never edit this list.
//! - `registry` = `Registry.build(registration_order ++ user registrations)`: later registrations override earlier
//!   ones exactly like in Jolt (`Registry.build` runs CollisionDispatch::sInit first).
//! - User shapes (User1..8, UserConvex1..8), a replacement for the default material (Jolt: "If you implement your own
//!   default material (PhysicsMaterial::sDefault) make sure to initialize it before this function") and user material
//!   types come from the module `zolt_user_types`. build.zig wires an empty default module; an application replaces it
//!   with `zolt_module.addImport("zolt_user_types", my_module)` (a module import cycle, which Zig allows). The module
//!   may declare:
//!   - `registrations`: a tuple of types with `pub fn register(comptime r: *zolt.CollisionDispatch.Registry) void`,
//!     run after Jolt's registrations (so they override them);
//!   - `default_material: *const zolt.PhysicsMaterial` (a static material, see PhysicsMaterial.initStatic);
//!   - `material_types`: a tuple of material classes that PhysicsMaterial.restoreFromBinaryState can create;
//!   - `group_filter_types`: a tuple of GroupFilter classes that GroupFilter.restoreFromBinaryState can create;
//!   - `constraint_settings_types`: a tuple of ConstraintSettings classes that ConstraintSettings.restoreFromBinaryState
//!     can create;
//!   - `path_constraint_path_types`: a tuple of PathConstraintPath classes that PathConstraintPath.restoreFromBinaryState
//!     can create;
//!   - `vehicle_controller_settings_types`: a tuple of VehicleControllerSettings classes that
//!     VehicleConstraintSettings.restoreBinaryState can create.
//!
//! Not needed in Zolt: RegisterTypesInternal / UnregisterTypes (nothing to register or unregister at runtime) and
//! VerifyJoltVersionID (Zig compiles the library together with the application, so the ABI cannot mismatch).
//! TODO(serialization): the Factory registration of the RTTI types (`Factory::sInstance->Register(types, ...)`),
//! with ObjectStream (Phase 8).

const PhysicsMaterial = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const GroupFilterTable = @import("Physics/Collision/GroupFilterTable.zig").GroupFilterTable;
const Registry = @import("Physics/Collision/CollisionDispatch.zig").Registry;
const BoxShape = @import("Physics/Collision/Shape/BoxShape.zig").BoxShape;
const CapsuleShape = @import("Physics/Collision/Shape/CapsuleShape.zig").CapsuleShape;
const CompoundShape = @import("Physics/Collision/Shape/CompoundShape.zig").CompoundShape;
const ConvexHullShape = @import("Physics/Collision/Shape/ConvexHullShape.zig").ConvexHullShape;
const ConvexShape = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShape;
const CylinderShape = @import("Physics/Collision/Shape/CylinderShape.zig").CylinderShape;
const EmptyShape = @import("Physics/Collision/Shape/EmptyShape.zig").EmptyShape;
const HeightFieldShape = @import("Physics/Collision/Shape/HeightFieldShape.zig").HeightFieldShape;
const MeshShape = @import("Physics/Collision/Shape/MeshShape.zig").MeshShape;
const MutableCompoundShape = @import("Physics/Collision/Shape/MutableCompoundShape.zig").MutableCompoundShape;
const OffsetCenterOfMassShape = @import("Physics/Collision/Shape/OffsetCenterOfMassShape.zig").OffsetCenterOfMassShape;
const PlaneShape = @import("Physics/Collision/Shape/PlaneShape.zig").PlaneShape;
const RotatedTranslatedShape = @import("Physics/Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShape;
const ScaledShape = @import("Physics/Collision/Shape/ScaledShape.zig").ScaledShape;
const SphereShape = @import("Physics/Collision/Shape/SphereShape.zig").SphereShape;
const StaticCompoundShape = @import("Physics/Collision/Shape/StaticCompoundShape.zig").StaticCompoundShape;
const TaperedCapsuleShape = @import("Physics/Collision/Shape/TaperedCapsuleShape.zig").TaperedCapsuleShape;
const TaperedCylinderShape = @import("Physics/Collision/Shape/TaperedCylinderShape.zig").TaperedCylinderShape;
const TriangleShape = @import("Physics/Collision/Shape/TriangleShape.zig").TriangleShape;
const SoftBodyShape = @import("Physics/SoftBody/SoftBodyShape.zig").SoftBodyShape;
const ConeConstraintSettings = @import("Physics/Constraints/ConeConstraint.zig").ConeConstraintSettings;
const DistanceConstraintSettings = @import("Physics/Constraints/DistanceConstraint.zig").DistanceConstraintSettings;
const FixedConstraintSettings = @import("Physics/Constraints/FixedConstraint.zig").FixedConstraintSettings;
const GearConstraintSettings = @import("Physics/Constraints/GearConstraint.zig").GearConstraintSettings;
const HingeConstraintSettings = @import("Physics/Constraints/HingeConstraint.zig").HingeConstraintSettings;
const PathConstraintSettings = @import("Physics/Constraints/PathConstraint.zig").PathConstraintSettings;
const PathConstraintPathHermite = @import("Physics/Constraints/PathConstraintPathHermite.zig").PathConstraintPathHermite;
const PointConstraintSettings = @import("Physics/Constraints/PointConstraint.zig").PointConstraintSettings;
const PulleyConstraintSettings = @import("Physics/Constraints/PulleyConstraint.zig").PulleyConstraintSettings;
const RackAndPinionConstraintSettings = @import("Physics/Constraints/RackAndPinionConstraint.zig").RackAndPinionConstraintSettings;
const SixDOFConstraintSettings = @import("Physics/Constraints/SixDOFConstraint.zig").SixDOFConstraintSettings;
const SliderConstraintSettings = @import("Physics/Constraints/SliderConstraint.zig").SliderConstraintSettings;
const SwingTwistConstraintSettings = @import("Physics/Constraints/SwingTwistConstraint.zig").SwingTwistConstraintSettings;
const VehicleConstraintSettings = @import("Physics/Vehicle/VehicleConstraint.zig").VehicleConstraintSettings;
const WheeledVehicleControllerSettings = @import("Physics/Vehicle/WheeledVehicleController.zig").WheeledVehicleControllerSettings;
const TrackedVehicleControllerSettings = @import("Physics/Vehicle/TrackedVehicleController.zig").TrackedVehicleControllerSettings;
const MotorcycleControllerSettings = @import("Physics/Vehicle/MotorcycleController.zig").MotorcycleControllerSettings;

/// The user types module (see the file comment), an empty module unless the application provides one
pub const user_types = @import("zolt_user_types");

/// The sRegister calls of RegisterTypesInternal, in Jolt's order (after CollisionDispatch::sInit, which
/// `Registry.build` runs first)
pub const registration_order = .{
    // Register base classes first so that we can specialize them later
    CompoundShape,
    ConvexShape,

    // Register compounds before others so that we can specialize them later (register them in reverse order of collision complexity)
    MutableCompoundShape,
    StaticCompoundShape,

    // Leaf classes
    TriangleShape,
    PlaneShape,
    SphereShape,
    BoxShape,
    CapsuleShape,
    TaperedCapsuleShape,
    CylinderShape,
    TaperedCylinderShape,
    MeshShape,
    ConvexHullShape,
    HeightFieldShape,
    SoftBodyShape,

    // Register these last because their collision functions are simple so we want to execute them first (register them in reverse order of collision complexity)
    RotatedTranslatedShape,
    OffsetCenterOfMassShape,
    ScaledShape,
    EmptyShape,
};

/// The user registrations (run after Jolt's, so they override them)
pub const user_registrations = if (@hasDecl(user_types, "registrations")) user_types.registrations else .{};

/// The collision dispatch tables and ShapeFunctions after RegisterTypes() (immutable)
pub const registry: Registry = .build(registration_order ++ user_registrations);

/// PhysicsMaterial::sDefault: the user's default material, or the one RegisterTypes creates
/// (`new PhysicsMaterialSimple("Default", Color::sGrey)`, a compile time constant here)
pub const default_material: *const PhysicsMaterial = if (@hasDecl(user_types, "default_material")) user_types.default_material else &PhysicsMaterialSimple.default_material.base;

/// The material classes that PhysicsMaterial.restoreFromBinaryState can create (the material part of the Factory
/// registration, until the Factory of Phase 8): Jolt's PhysicsMaterial and PhysicsMaterialSimple plus the user's
pub const material_types = .{ PhysicsMaterial, PhysicsMaterialSimple } ++ (if (@hasDecl(user_types, "material_types")) user_types.material_types else .{});

/// The GroupFilter classes that GroupFilter.restoreFromBinaryState can create (the group filter part of the Factory
/// registration, until the Factory of Phase 8): Jolt's GroupFilterTable plus the user's
pub const group_filter_types = .{GroupFilterTable} ++ (if (@hasDecl(user_types, "group_filter_types")) user_types.group_filter_types else .{});

/// The ConstraintSettings classes that ConstraintSettings.restoreFromBinaryState can create (the constraint part of the
/// Factory registration, until the Factory of Phase 8): Jolt's settings classes in the order of RegisterTypes.cpp plus
/// the user's. The classes that are still stubs declare `pub const is_stub = true`: restoreFromBinaryState
/// skips them (their hash gives Jolt's "Failed to create instance of type", as for a type the Factory does not know), so
/// a porter only removes the marker and never edits this list.
pub const constraint_settings_types = .{
    PointConstraintSettings,
    SixDOFConstraintSettings,
    SliderConstraintSettings,
    SwingTwistConstraintSettings,
    DistanceConstraintSettings,
    HingeConstraintSettings,
    FixedConstraintSettings,
    ConeConstraintSettings,
    PathConstraintSettings,
    VehicleConstraintSettings, // Phase 7 (Vehicle core group)
    RackAndPinionConstraintSettings,
    GearConstraintSettings,
    PulleyConstraintSettings,
} ++ (if (@hasDecl(user_types, "constraint_settings_types")) user_types.constraint_settings_types else .{});

/// The PathConstraintPath classes that PathConstraintPath.restoreFromBinaryState can create (the path part of the Factory
/// registration, until the Factory of Phase 8): Jolt's PathConstraintPathHermite (PathConstraintPath itself is abstract)
/// plus the user's (`path_constraint_path_types` of `zolt_user_types`). A stub declares `is_stub` (see above).
pub const path_constraint_path_types = .{PathConstraintPathHermite} ++ (if (@hasDecl(user_types, "path_constraint_path_types")) user_types.path_constraint_path_types else .{});

/// The VehicleControllerSettings classes that VehicleConstraintSettings.restoreBinaryState can create from the RTTI hash
/// of its controller (`Factory::sInstance->Find(hash)`, the vehicle part of the Factory registration until the Factory of
/// Phase 8, CharacterVehicleArchitecture E3): Jolt's controller settings classes in the order of RegisterTypes.cpp plus
/// the user's (`vehicle_controller_settings_types` of `zolt_user_types`). Each one declares `rtti_name` and
/// `createDefault(allocator) Allocator.Error!*T`. A class that is still a stub declares `is_stub`: the restore skips it
/// (its hash is unknown, which panics like Jolt's null dereference, see VehicleConstraint.zig).
pub const vehicle_controller_settings_types = .{
    WheeledVehicleControllerSettings,
    TrackedVehicleControllerSettings, // Phase 7 (Vehicle variants group)
    MotorcycleControllerSettings, // Phase 7 (Vehicle variants group)
} ++ (if (@hasDecl(user_types, "vehicle_controller_settings_types")) user_types.vehicle_controller_settings_types else .{});

// ---- Phase 8 foundation (group F1): a minimal Factory registration. Group F2 replaces this whole block. ----------
// F2 lists Jolt's 49 types of RegisterTypes.cpp (Docs/Zolt/SerializationArchitecture.md S8). Until then the list holds
// the classes of that list that have an RTTI declaration: only MotorSettings.

const Factory = @import("Core/Factory.zig").Factory;
const MotorSettings = @import("Physics/Constraints/MotorSettings.zig").MotorSettings;

/// The classes that RegisterTypes() registers with the Factory (`types[]` of RegisterTypes.cpp), in Jolt's order.
/// Tests and applications that need more classes build their own factory:
/// `Factory.init(RegisterTypes.factory_types ++ &[_]type{MyClass})`.
pub const factory_types: []const type = &.{MotorSettings};

/// The serializable classes of the user (`serializable_types` of `zolt_user_types`, a slice of types)
const user_serializable_types: []const type = if (@hasDecl(user_types, "serializable_types")) user_types.serializable_types else &.{};

/// Factory::sInstance after RegisterTypes(): Jolt's classes, then the user's (a compile time constant). It is the
/// default factory of ObjectStreamIn.
pub const factory: Factory = .init(factory_types ++ user_serializable_types);

// ---- End of the F1 block ------------------------------------------------------------------------------------------

test "RegisterTypes: Jolt's registration order, the user hook, the default material and material types" {
    const std = @import("std");
    const testing = std.testing;
    const CollisionDispatch = @import("Physics/Collision/CollisionDispatch.zig");
    const ShapeFunctions = @import("Physics/Collision/Shape/Shape.zig").ShapeFunctions;
    const TestShapes = @import("Physics/Collision/Shape/TestShapes.zig");

    // RegisterTypesInternal's order
    const expected = [_][]const u8{ "CompoundShape", "ConvexShape", "MutableCompoundShape", "StaticCompoundShape", "TriangleShape", "PlaneShape", "SphereShape", "BoxShape", "CapsuleShape", "TaperedCapsuleShape", "CylinderShape", "TaperedCylinderShape", "MeshShape", "ConvexHullShape", "HeightFieldShape", "SoftBodyShape", "RotatedTranslatedShape", "OffsetCenterOfMassShape", "ScaledShape", "EmptyShape" };
    try testing.expectEqual(expected.len, registration_order.len);
    inline for (registration_order, expected) |T, name| {
        const full = @typeName(T);
        try testing.expect(std.mem.endsWith(u8, full, "." ++ name) or std.mem.eql(u8, full, name));
    }

    // The inline tests register their test shapes through zolt_user_types (see build.zig), after Jolt's registrations
    try testing.expectEqual(@as(usize, 1), user_registrations.len);
    try testing.expect(user_registrations[0] == TestShapes.TestShapeRegistration);
    const rebuilt = comptime Registry.build(registration_order ++ user_registrations);
    try testing.expect(std.meta.eql(rebuilt, registry));
    try testing.expect(registry.getCollideShape(.user1, .user1) == &TestShapes.collideBoxVsBox);
    try testing.expect(ShapeFunctions.get(.user1).construct != null);

    // A later registration overrides the earlier ones, like a user registration overrides Jolt's
    const Override = struct {
        pub fn register(comptime r: *Registry) void {
            r.registerCollideShape(.user1, .user1, CollisionDispatch.reversedCollideShape);
            r.shapeFunctions(.user1).construct = null;
        }
    };
    const overridden = comptime Registry.build(registration_order ++ user_registrations ++ .{Override});
    try testing.expect(overridden.getCollideShape(.user1, .user1) == &CollisionDispatch.reversedCollideShape);
    try testing.expect(overridden.shape_functions[@intFromEnum(@import("Physics/Collision/Shape/Shape.zig").ShapeSubType.user1)].construct == null);

    // The default material is Jolt's (the tests do not replace it), user material types are added to Jolt's
    try testing.expect(default_material == &PhysicsMaterialSimple.default_material.base);
    try testing.expect(PhysicsMaterial.default == default_material);
    try testing.expectEqualStrings("Default", default_material.getDebugName());
    try testing.expectEqual(@as(usize, 3), material_types.len);
    try testing.expect(material_types[0] == PhysicsMaterial and material_types[1] == PhysicsMaterialSimple and material_types[2] == TestShapes.TestMaterial);
}

test "RegisterTypes: the constraint settings and path classes in Jolt's order, stubs are marked" {
    const std = @import("std");
    const testing = std.testing;
    const TestConstraints = @import("Physics/Constraints/TestConstraints.zig");
    const ConstraintSettings = @import("Physics/Constraints/Constraint.zig").ConstraintSettings;

    // RegisterTypesInternal's order of the constraint settings, then the user's (the inline tests register
    // TestConstraintSettings through zolt_user_types)
    const expected = [_][]const u8{ "PointConstraintSettings", "SixDOFConstraintSettings", "SliderConstraintSettings", "SwingTwistConstraintSettings", "DistanceConstraintSettings", "HingeConstraintSettings", "FixedConstraintSettings", "ConeConstraintSettings", "PathConstraintSettings", "VehicleConstraintSettings", "RackAndPinionConstraintSettings", "GearConstraintSettings", "PulleyConstraintSettings", "TestConstraintSettings" };
    try testing.expectEqual(expected.len, constraint_settings_types.len);
    inline for (constraint_settings_types, expected) |T, name| {
        try testing.expectEqualStrings(name, T.rtti_name);
        try testing.expect(std.mem.endsWith(u8, @typeName(T), "." ++ name));
    }
    try testing.expect(constraint_settings_types[constraint_settings_types.len - 1] == TestConstraints.TestConstraintSettings);

    // The ported classes can be created by restoreFromBinaryState, the stubs are marked (no RTTI hash collisions)
    inline for (constraint_settings_types, 0..) |T, i| {
        if (!@hasDecl(T, "is_stub")) try testing.expect(@hasDecl(T, "createDefault"));
        inline for (constraint_settings_types, 0..) |U, j| {
            if (i != j) try testing.expect(ConstraintSettings.rttiHash(T.rtti_name) != ConstraintSettings.rttiHash(U.rtti_name));
        }
    }
    try testing.expect(!@hasDecl(constraint_settings_types[0], "is_stub")); // Point
    try testing.expect(!@hasDecl(constraint_settings_types[6], "is_stub")); // Fixed

    try testing.expectEqual(@as(usize, 1), path_constraint_path_types.len);
    try testing.expectEqualStrings("PathConstraintPathHermite", path_constraint_path_types[0].rtti_name);
}

test "RegisterTypes: the vehicle controller settings classes in Jolt's order" {
    const std = @import("std");
    const testing = std.testing;
    const VehicleControllerSettings = @import("Physics/Vehicle/VehicleController.zig").VehicleControllerSettings;

    const expected = [_][]const u8{ "WheeledVehicleControllerSettings", "TrackedVehicleControllerSettings", "MotorcycleControllerSettings" };
    try testing.expectEqual(expected.len, vehicle_controller_settings_types.len);
    inline for (vehicle_controller_settings_types, expected, 0..) |T, name, i| {
        try testing.expectEqualStrings(name, T.rtti_name);
        if (!@hasDecl(T, "is_stub")) try testing.expect(@hasDecl(T, "createDefault"));
        inline for (vehicle_controller_settings_types, 0..) |U, j| {
            if (i != j) try testing.expect(VehicleControllerSettings.rttiHash(T.rtti_name) != VehicleControllerSettings.rttiHash(U.rtti_name));
        }
    }
}
