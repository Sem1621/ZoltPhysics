//! Port of: Jolt/Physics/Vehicle/VehicleCollisionTester.h, Jolt/Physics/Vehicle/VehicleCollisionTester.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! classes before they are ported (ZoltParity/Physics/SimScenes.zig: the vehicle blockers, SimulationParity.zig:
//! `VehicleSlot`). The porter of the Vehicle core group replaces the stub following
//! Docs/Zolt/CharacterVehicleArchitecture.md (E2: `VehicleCollisionTester` is an abstract pattern A root with
//! `ref_count`; E6: CastCylinder embeds its cylinder shape, `is_static`), removes the `is_stub` markers and fills the
//! harness slot (the recipe is in the SimScenes header).

/// Class that does collision detection between wheels and ground
pub const VehicleCollisionTester = struct {};

/// Collision tester that tests collision using a raycast
pub const VehicleCollisionTesterRay = struct {
    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Collision tester that tests collision using a sphere cast
pub const VehicleCollisionTesterCastSphere = struct {
    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Collision tester that tests collision using a cylinder shape
pub const VehicleCollisionTesterCastCylinder = struct {
    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};
