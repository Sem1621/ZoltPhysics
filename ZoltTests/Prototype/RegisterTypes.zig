//! Port of: Jolt/RegisterTypes.h, Jolt/RegisterTypes.cpp (prototype: the collision part)
//! Status: partial
//!
//! D4: Jolt's RegisterTypes() fills mutable static tables at startup. Zolt replays the same registration code at
//! compile time and keeps the final state as an immutable constant:
//! - `registration_order` lists the classes in the order of RegisterTypes.cpp (lines 102-130); every class has a
//!   `register(comptime r: *Registry)` that is a line by line port of its sRegister. The foundation step creates every
//!   shape file (as a stub with an empty `register` when not ported yet), so this list is complete from the start and
//!   the shape porters never edit this file.
//! - `registry` = `Registry.build(registration_order ++ user registrations)`: later registrations override earlier
//!   ones exactly like in Jolt. No mutable global, no initialization call, nothing to synchronize.
//! - User shapes (User1..8, UserConvex1..8), a replacement for the default material (Jolt's "create a suitable default
//!   material in sDefault") and user material types come from the module `zolt_user_types`: build.zig wires an empty
//!   default module, an application replaces it with `zolt_module.addImport("zolt_user_types", my_module)` (a module
//!   import cycle, which Zig 0.16 allows). The prototype lives in the test binary, so it uses a local empty namespace.
//! - PhysicsMaterial::sDefault and the RTTI factory for materials (Phase 8) are covered by `default_material` and
//!   `material_types`.

const CollisionDispatch = @import("Physics/Collision/CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const PhysicsMaterial = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const CompoundShape = @import("Physics/Collision/Shape/CompoundShape.zig").CompoundShape;
const ConvexShape = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShape;
const StaticCompoundShape = @import("Physics/Collision/Shape/StaticCompoundShape.zig").StaticCompoundShape;
const TriangleShape = @import("Physics/Collision/Shape/TriangleShape.zig").TriangleShape;
const SphereShape = @import("Physics/Collision/Shape/SphereShape.zig").SphereShape;
const BoxShape = @import("Physics/Collision/Shape/BoxShape.zig").BoxShape;
const ScaledShape = @import("Physics/Collision/Shape/ScaledShape.zig").ScaledShape;

/// The user types module (`@import("zolt_user_types")` in the library)
const user_types = struct {
    pub const registrations = .{};
};

/// Jolt/RegisterTypes.cpp order. The prototype does not have the shapes in comments.
pub const registration_order = .{
    // Register base classes first so that we can specialize them later
    CompoundShape,
    ConvexShape,

    // Register compounds before others so that we can specialize them later (register them in reverse order of collision complexity)
    // MutableCompoundShape,
    StaticCompoundShape,

    // Leaf classes
    TriangleShape,
    // PlaneShape,
    SphereShape,
    BoxShape,
    // CapsuleShape, TaperedCapsuleShape, CylinderShape, TaperedCylinderShape, MeshShape, ConvexHullShape, HeightFieldShape, SoftBodyShape,

    // Register these last because their collision functions are simple so we want to execute them first (register them in reverse order of collision complexity)
    // RotatedTranslatedShape, OffsetCenterOfMassShape,
    ScaledShape,
    // EmptyShape,
};

/// The collision dispatch tables and ShapeFunctions after RegisterTypes() (immutable)
pub const registry: Registry = .build(registration_order ++ user_types.registrations);

/// PhysicsMaterial::sDefault
pub const default_material: *const PhysicsMaterial = if (@hasDecl(user_types, "default_material")) user_types.default_material else &PhysicsMaterialSimple.default_material.base;

/// Material classes that PhysicsMaterial.restoreFromBinaryState can create (Factory::sInstance until Phase 8)
pub const material_types = .{ PhysicsMaterial, PhysicsMaterialSimple } ++ (if (@hasDecl(user_types, "material_types")) user_types.material_types else .{});
