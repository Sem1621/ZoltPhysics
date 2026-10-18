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
//! - `factory_types` lists the types of RegisterTypes.cpp in Jolt's order (`const RTTI *types[]`), without
//!   SoftBodyCreationSettings (Phase 9). `factory` = `Factory.init(factory_types ++ user serializable types)` is
//!   `Factory::sInstance` after RegisterTypes() (Docs/Zolt/SerializationArchitecture.md S8).
//! - User shapes (User1..8, UserConvex1..8), a replacement for the default material (Jolt: "If you implement your own
//!   default material (PhysicsMaterial::sDefault) make sure to initialize it before this function") and user
//!   serializable classes come from the module `zolt_user_types`. build.zig wires an empty default module; an
//!   application replaces it with `zolt_module.addImport("zolt_user_types", my_module)` (a module import cycle, which Zig
//!   allows). The module may declare:
//!   - `registrations`: a tuple of types with `pub fn register(comptime r: *zolt.CollisionDispatch.Registry) void`,
//!     run after Jolt's registrations (so they override them);
//!   - `default_material: *const zolt.PhysicsMaterial` (a static material, see PhysicsMaterial.initStatic);
//!   - `serializable_types`: a tuple of classes with an RTTI (`pub const rtti`) that `factory` registers after Jolt's
//!     (Jolt: `Factory::sInstance->Register(JPH_RTTI(MyClass))`). The binary state restores (StreamUtils.restoreObject)
//!     and ObjectStreamIn create them.
//!
//! Not needed in Zolt: RegisterTypesInternal / UnregisterTypes (nothing to register or unregister at runtime) and
//! VerifyJoltVersionID (Zig compiles the library together with the application, so the ABI cannot mismatch).

const Factory = @import("Core/Factory.zig").Factory;
const PhysicsMaterial = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const GroupFilter = @import("Physics/Collision/GroupFilter.zig").GroupFilter;
const GroupFilterTable = @import("Physics/Collision/GroupFilterTable.zig").GroupFilterTable;
const Registry = @import("Physics/Collision/CollisionDispatch.zig").Registry;
const BoxShapeFile = @import("Physics/Collision/Shape/BoxShape.zig");
const CapsuleShapeFile = @import("Physics/Collision/Shape/CapsuleShape.zig");
const CompoundShapeFile = @import("Physics/Collision/Shape/CompoundShape.zig");
const ConvexHullShapeFile = @import("Physics/Collision/Shape/ConvexHullShape.zig");
const CylinderShapeFile = @import("Physics/Collision/Shape/CylinderShape.zig");
const EmptyShapeFile = @import("Physics/Collision/Shape/EmptyShape.zig");
const HeightFieldShapeFile = @import("Physics/Collision/Shape/HeightFieldShape.zig");
const MeshShapeFile = @import("Physics/Collision/Shape/MeshShape.zig");
const MutableCompoundShapeFile = @import("Physics/Collision/Shape/MutableCompoundShape.zig");
const OffsetCenterOfMassShapeFile = @import("Physics/Collision/Shape/OffsetCenterOfMassShape.zig");
const PlaneShapeFile = @import("Physics/Collision/Shape/PlaneShape.zig");
const RotatedTranslatedShapeFile = @import("Physics/Collision/Shape/RotatedTranslatedShape.zig");
const ScaledShapeFile = @import("Physics/Collision/Shape/ScaledShape.zig");
const SphereShapeFile = @import("Physics/Collision/Shape/SphereShape.zig");
const StaticCompoundShapeFile = @import("Physics/Collision/Shape/StaticCompoundShape.zig");
const TaperedCapsuleShapeFile = @import("Physics/Collision/Shape/TaperedCapsuleShape.zig");
const TaperedCylinderShapeFile = @import("Physics/Collision/Shape/TaperedCylinderShape.zig");
const TriangleShapeFile = @import("Physics/Collision/Shape/TriangleShape.zig");
const BoxShape = BoxShapeFile.BoxShape;
const CapsuleShape = CapsuleShapeFile.CapsuleShape;
const CompoundShape = CompoundShapeFile.CompoundShape;
const ConvexHullShape = ConvexHullShapeFile.ConvexHullShape;
const ConvexShape = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShape;
const CylinderShape = CylinderShapeFile.CylinderShape;
const EmptyShape = EmptyShapeFile.EmptyShape;
const HeightFieldShape = HeightFieldShapeFile.HeightFieldShape;
const MeshShape = MeshShapeFile.MeshShape;
const MutableCompoundShape = MutableCompoundShapeFile.MutableCompoundShape;
const OffsetCenterOfMassShape = OffsetCenterOfMassShapeFile.OffsetCenterOfMassShape;
const PlaneShape = PlaneShapeFile.PlaneShape;
const RotatedTranslatedShape = RotatedTranslatedShapeFile.RotatedTranslatedShape;
const ScaledShape = ScaledShapeFile.ScaledShape;
const SphereShape = SphereShapeFile.SphereShape;
const StaticCompoundShape = StaticCompoundShapeFile.StaticCompoundShape;
const TaperedCapsuleShape = TaperedCapsuleShapeFile.TaperedCapsuleShape;
const TaperedCylinderShape = TaperedCylinderShapeFile.TaperedCylinderShape;
const TriangleShape = TriangleShapeFile.TriangleShape;
const SoftBodyShape = @import("Physics/SoftBody/SoftBodyShape.zig").SoftBodyShape;
const ConeConstraintSettings = @import("Physics/Constraints/ConeConstraint.zig").ConeConstraintSettings;
const DistanceConstraintSettings = @import("Physics/Constraints/DistanceConstraint.zig").DistanceConstraintSettings;
const FixedConstraintSettings = @import("Physics/Constraints/FixedConstraint.zig").FixedConstraintSettings;
const GearConstraintSettings = @import("Physics/Constraints/GearConstraint.zig").GearConstraintSettings;
const HingeConstraintSettings = @import("Physics/Constraints/HingeConstraint.zig").HingeConstraintSettings;
const MotorSettings = @import("Physics/Constraints/MotorSettings.zig").MotorSettings;
const PathConstraintSettings = @import("Physics/Constraints/PathConstraint.zig").PathConstraintSettings;
const PathConstraintPath = @import("Physics/Constraints/PathConstraintPath.zig").PathConstraintPath;
const PathConstraintPathHermite = @import("Physics/Constraints/PathConstraintPathHermite.zig").PathConstraintPathHermite;
const PointConstraintSettings = @import("Physics/Constraints/PointConstraint.zig").PointConstraintSettings;
const PulleyConstraintSettings = @import("Physics/Constraints/PulleyConstraint.zig").PulleyConstraintSettings;
const RackAndPinionConstraintSettings = @import("Physics/Constraints/RackAndPinionConstraint.zig").RackAndPinionConstraintSettings;
const SixDOFConstraintSettings = @import("Physics/Constraints/SixDOFConstraint.zig").SixDOFConstraintSettings;
const SliderConstraintSettings = @import("Physics/Constraints/SliderConstraint.zig").SliderConstraintSettings;
const SwingTwistConstraintSettings = @import("Physics/Constraints/SwingTwistConstraint.zig").SwingTwistConstraintSettings;
const VehicleConstraintSettings = @import("Physics/Vehicle/VehicleConstraint.zig").VehicleConstraintSettings;
const WheeledVehicleControllerFile = @import("Physics/Vehicle/WheeledVehicleController.zig");
const TrackedVehicleControllerFile = @import("Physics/Vehicle/TrackedVehicleController.zig");
const MotorcycleControllerSettings = @import("Physics/Vehicle/MotorcycleController.zig").MotorcycleControllerSettings;
const BodyCreationSettings = @import("Physics/Body/BodyCreationSettings.zig").BodyCreationSettings;
const PhysicsScene = @import("Physics/PhysicsScene.zig").PhysicsScene;
const RagdollSettings = @import("Physics/Ragdoll/Ragdoll.zig").RagdollSettings;
const Skeleton = @import("Skeleton/Skeleton.zig").Skeleton;
const SkeletalAnimation = @import("Skeleton/SkeletalAnimation.zig").SkeletalAnimation;

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

/// Create list of all types (RegisterTypesInternal's `const RTTI *types[]`, without SoftBodyCreationSettings: Phase 9)
pub const factory_types: []const type = &.{
    SkeletalAnimation,
    Skeleton,
    CompoundShapeFile.CompoundShapeSettings,
    StaticCompoundShapeFile.StaticCompoundShapeSettings,
    MutableCompoundShapeFile.MutableCompoundShapeSettings,
    TriangleShapeFile.TriangleShapeSettings,
    PlaneShapeFile.PlaneShapeSettings,
    SphereShapeFile.SphereShapeSettings,
    BoxShapeFile.BoxShapeSettings,
    CapsuleShapeFile.CapsuleShapeSettings,
    TaperedCapsuleShapeFile.TaperedCapsuleShapeSettings,
    CylinderShapeFile.CylinderShapeSettings,
    TaperedCylinderShapeFile.TaperedCylinderShapeSettings,
    ScaledShapeFile.ScaledShapeSettings,
    MeshShapeFile.MeshShapeSettings,
    ConvexHullShapeFile.ConvexHullShapeSettings,
    HeightFieldShapeFile.HeightFieldShapeSettings,
    RotatedTranslatedShapeFile.RotatedTranslatedShapeSettings,
    OffsetCenterOfMassShapeFile.OffsetCenterOfMassShapeSettings,
    EmptyShapeFile.EmptyShapeSettings,
    RagdollSettings,
    PointConstraintSettings,
    SixDOFConstraintSettings,
    SliderConstraintSettings,
    SwingTwistConstraintSettings,
    DistanceConstraintSettings,
    HingeConstraintSettings,
    FixedConstraintSettings,
    ConeConstraintSettings,
    PathConstraintSettings,
    VehicleConstraintSettings,
    WheeledVehicleControllerFile.WheeledVehicleControllerSettings,
    WheeledVehicleControllerFile.WheelSettingsWV,
    TrackedVehicleControllerFile.TrackedVehicleControllerSettings,
    TrackedVehicleControllerFile.WheelSettingsTV,
    MotorcycleControllerSettings,
    PathConstraintPath,
    PathConstraintPathHermite,
    RackAndPinionConstraintSettings,
    GearConstraintSettings,
    PulleyConstraintSettings,
    MotorSettings,
    PhysicsScene,
    PhysicsMaterial,
    PhysicsMaterialSimple,
    GroupFilter,
    GroupFilterTable,
    BodyCreationSettings,
};

/// The serializable classes of the user (`serializable_types` of the user types module)
pub const user_serializable_types: []const type = if (@hasDecl(user_types, "serializable_types")) typeList(user_types.serializable_types) else &.{};

/// Factory::sInstance after RegisterTypes(): Jolt's types, then the user's
pub const factory: Factory = .init(factory_types ++ user_serializable_types);

comptime {
    // The lists of the binary state restores before the Factory existed (removed without a transition period,
    // SerializationArchitecture.md section 11)
    for ([_][]const u8{ "material_types", "group_filter_types", "constraint_settings_types", "path_constraint_path_types", "vehicle_controller_settings_types" }) |name| {
        if (@hasDecl(user_types, name))
            @compileError("zolt_user_types." ++ name ++ " is no longer used: list the classes in `pub const serializable_types` (RegisterTypes.factory, Docs/Zolt/SerializationArchitecture.md S9)");
    }
}

/// A tuple, an array or a slice of types as a slice
fn typeList(comptime types: anytype) []const type {
    var list: []const type = &.{};
    for (types) |T| list = list ++ &[_]type{T};
    return list;
}

test "RegisterTypes: Jolt's registration order, the user hook and the default material" {
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

    // The default material is Jolt's (the tests do not replace it)
    try testing.expect(default_material == &PhysicsMaterialSimple.default_material.base);
    try testing.expect(PhysicsMaterial.default == default_material);
    try testing.expectEqualStrings("Default", default_material.getDebugName());
}

test "RegisterTypes: the factory has Jolt's types in Jolt's order, then the user's" {
    const std = @import("std");
    const testing = std.testing;
    const RTTI = @import("Core/RTTI.zig").RTTI;
    const TestShapes = @import("Physics/Collision/Shape/TestShapes.zig");
    const TestConstraints = @import("Physics/Constraints/TestConstraints.zig");

    // RegisterTypesInternal's `types[]` without SoftBodyCreationSettings (Phase 9)
    const expected = [_][]const u8{ "SkeletalAnimation", "Skeleton", "CompoundShapeSettings", "StaticCompoundShapeSettings", "MutableCompoundShapeSettings", "TriangleShapeSettings", "PlaneShapeSettings", "SphereShapeSettings", "BoxShapeSettings", "CapsuleShapeSettings", "TaperedCapsuleShapeSettings", "CylinderShapeSettings", "TaperedCylinderShapeSettings", "ScaledShapeSettings", "MeshShapeSettings", "ConvexHullShapeSettings", "HeightFieldShapeSettings", "RotatedTranslatedShapeSettings", "OffsetCenterOfMassShapeSettings", "EmptyShapeSettings", "RagdollSettings", "PointConstraintSettings", "SixDOFConstraintSettings", "SliderConstraintSettings", "SwingTwistConstraintSettings", "DistanceConstraintSettings", "HingeConstraintSettings", "FixedConstraintSettings", "ConeConstraintSettings", "PathConstraintSettings", "VehicleConstraintSettings", "WheeledVehicleControllerSettings", "WheelSettingsWV", "TrackedVehicleControllerSettings", "WheelSettingsTV", "MotorcycleControllerSettings", "PathConstraintPath", "PathConstraintPathHermite", "RackAndPinionConstraintSettings", "GearConstraintSettings", "PulleyConstraintSettings", "MotorSettings", "PhysicsScene", "PhysicsMaterial", "PhysicsMaterialSimple", "GroupFilter", "GroupFilterTable", "BodyCreationSettings" };
    try testing.expectEqual(expected.len, factory_types.len);
    inline for (factory_types, expected) |T, name| {
        try testing.expectEqualStrings(name, RTTI.of(T).name);
        try testing.expect(factory.find(name) == RTTI.of(T));
        try testing.expect(factory.findHash(RTTI.hashName(name)) == RTTI.of(T));
    }

    // The first registered class is Jolt's first type, the bases follow the class that names them (Factory::Register)
    try testing.expect(factory.getAllClasses()[0] == RTTI.of(factory_types[0]));
    try testing.expect(factory.find("ShapeSettings") != null and factory.find("SerializableObject") != null);
    try testing.expect(factory.find("TwoBodyConstraintSettings").?.isAbstract());
    try testing.expect(factory.find("ConstraintSettings").? == @import("Physics/Constraints/Constraint.zig").ConstraintSettings.rtti);
    try testing.expect(!factory.find("ConstraintSettings").?.isAbstract()); // Jolt: JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL
    try testing.expect(factory.find("SoftBodyCreationSettings") == null);

    // The inline tests register their test classes through zolt_user_types.serializable_types (see build.zig)
    try testing.expectEqual(@as(usize, 2), user_serializable_types.len);
    try testing.expect(user_serializable_types[0] == TestShapes.TestMaterial);
    try testing.expect(user_serializable_types[1] == TestConstraints.TestConstraintSettings);
    try testing.expect(factory.find("TestMaterial") == TestShapes.TestMaterial.rtti);
    try testing.expect(factory.find("TestConstraintSettings") == TestConstraints.TestConstraintSettings.rtti);

    // Every class of the factory is found by its name and its hash
    for (factory.getAllClasses()) |rtti| {
        try testing.expect(factory.find(rtti.name) == rtti);
        try testing.expect(factory.findHash(rtti.hash) == rtti);
    }

    // A factory of a test or an application: Jolt's types and some more (Jolt: Factory::sInstance->Register)
    const Extra = struct {
        pub const rtti = RTTI.declare(@This(), .{ .name = "Extra", .kind = .non_virtual });
        value: u32 = 0,
    };
    const extended: Factory = comptime .init(factory_types ++ &[_]type{Extra});
    try testing.expect(extended.find("Extra") == Extra.rtti and factory.find("Extra") == null);
    try testing.expect(extended.find("BodyCreationSettings") == BodyCreationSettings.rtti);
}

test "RegisterTypes: every creatable class of the factory can be created and destroyed" {
    const std = @import("std");
    const RTTI = @import("Core/RTTI.zig").RTTI;
    for (factory.getAllClasses()) |rtti| {
        if (rtti.isAbstract()) continue;
        try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
            fn f(allocator: std.mem.Allocator, r: *const RTTI) !void {
                const object = (try r.createObject(allocator)).?;
                if (r.ref_count) |refCount| try std.testing.expectEqual(@as(u32, 0), refCount(object).get()); // `new T`
                r.destructObject(object, allocator);
            }
        }.f, .{rtti});
    }
}
