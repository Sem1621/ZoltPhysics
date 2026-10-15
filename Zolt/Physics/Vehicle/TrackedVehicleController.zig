//! Port of: Jolt/Physics/Vehicle/TrackedVehicleController.h, Jolt/Physics/Vehicle/TrackedVehicleController.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! classes before they are ported (ZoltParity/Physics/SimScenes.zig: `Blocker.tracked_vehicles`, SimulationParity.zig:
//! `TrackedVehicleSlot`). The porter of the Vehicle variants group replaces the stub following
//! Docs/Zolt/CharacterVehicleArchitecture.md (E2, E3), removes the `is_stub` markers and fills the harness slot (the
//! recipe is in the SimScenes header).

/// WheelSettings object specifically for TrackedVehicleController
pub const WheelSettingsTV = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "WheelSettingsTV";

    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Wheel object specifically for TrackedVehicleController
pub const WheelTV = struct {};

/// Settings of a vehicle with tank tracks
///
/// Default settings are based around what I could find about the M1 Abrams tank.
/// Note to avoid issues with very heavy objects vs very light objects the mass of the tank should be a lot lower (say 10x) than that of a real tank. That means that the engine/brake torque is also 10x less.
pub const TrackedVehicleControllerSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "TrackedVehicleControllerSettings";

    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Runtime controller class for vehicle with tank tracks
pub const TrackedVehicleController = struct {};
