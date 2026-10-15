//! Port of: Jolt/Physics/Vehicle/MotorcycleController.h, Jolt/Physics/Vehicle/MotorcycleController.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! classes before they are ported (ZoltParity/Physics/SimScenes.zig: `Blocker.motorcycles`, SimulationParity.zig:
//! `MotorcycleSlot`). The porter of the Vehicle variants group replaces the stub following
//! Docs/Zolt/CharacterVehicleArchitecture.md (E2: MotorcycleControllerSettings / MotorcycleController embed the wheeled
//! classes as `base`), removes `is_stub` and fills the harness slot (the recipe is in the SimScenes header).

/// Settings of a two wheeled motorcycle (adds a spring to balance the motorcycle)
/// Note: The motor cycle controller is still in development and may need a lot of tweaks/hacks to work properly!
pub const MotorcycleControllerSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "MotorcycleControllerSettings";

    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Runtime controller class
pub const MotorcycleController = struct {};
