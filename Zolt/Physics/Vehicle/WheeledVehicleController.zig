//! Port of: Jolt/Physics/Vehicle/WheeledVehicleController.h, Jolt/Physics/Vehicle/WheeledVehicleController.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! classes before they are ported (ZoltParity/Physics/SimScenes.zig: `Blocker.wheeled_vehicles` and
//! `Blocker.motorcycles`, SimulationParity.zig: `VehicleSlot`). The porter of the Vehicle core group replaces the stub
//! following Docs/Zolt/CharacterVehicleArchitecture.md (E2; E3: the settings go into
//! `RegisterTypes.vehicle_controller_settings_types`), removes the `is_stub` markers and fills the harness slot (the
//! recipe is in the SimScenes header).

/// WheelSettings object specifically for WheeledVehicleController
pub const WheelSettingsWV = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "WheelSettingsWV";

    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Wheel object specifically for WheeledVehicleController
pub const WheelWV = struct {};

/// Settings of a vehicle with regular wheels
///
/// The properties in this controller are largely based on "Car Physics for Games" by Marco Monster.
/// See: https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
pub const WheeledVehicleControllerSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "WheeledVehicleControllerSettings";

    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Runtime controller class
pub const WheeledVehicleController = struct {};
