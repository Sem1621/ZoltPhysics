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
//!   - `material_types`: a tuple of material classes that PhysicsMaterial.restoreFromBinaryState can create.
//!
//! Not needed in Zolt: RegisterTypesInternal / UnregisterTypes (nothing to register or unregister at runtime) and
//! VerifyJoltVersionID (Zig compiles the library together with the application, so the ABI cannot mismatch).
//! TODO(serialization): the Factory registration of the RTTI types (`Factory::sInstance->Register(types, ...)`),
//! with ObjectStream (Phase 8).

const PhysicsMaterial = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
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
