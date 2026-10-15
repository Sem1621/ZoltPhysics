//! Port of: Jolt/Physics/Vehicle/VehicleConstraint.h, Jolt/Physics/Vehicle/VehicleConstraint.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that RegisterTypes.zig (`constraint_settings_types`,
//! at its place after PathConstraintSettings) and the simulation parity harness can name the classes before they are
//! ported (ZoltParity/Physics/SimScenes.zig: the vehicle blockers, SimulationParity.zig: `VehicleSlot`). The porter of
//! the Vehicle core group replaces the stub following Docs/Zolt/CharacterVehicleArchitecture.md (E2, E6:
//! `VehicleConstraintSettings` embeds `base: ConstraintSettings`, `VehicleConstraint` is a pattern A level directly under
//! `Constraint` and hands out its `PhysicsStepListener` with `stepListener()`) and the porter template in
//! FixedConstraint.zig, removes `is_stub` (ConstraintSettings.restoreFromBinaryState skips stubs) and fills the harness
//! slot (the recipe is in the SimScenes header).

/// Configuration for constraint that simulates a wheeled vehicle.
///
/// The properties in this constraint are largely based on "Car Physics for Games" by Marco Monster.
/// See: https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
pub const VehicleConstraintSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "VehicleConstraintSettings";

    /// Zolt marker of a stub class: ConstraintSettings.restoreFromBinaryState does not create it (Jolt's "Failed to
    /// create instance of type") and the simulation parity skips the scenes that need it, until it is ported (remove
    /// with the stub)
    pub const is_stub = true;
};

/// Constraint that simulates a vehicle
/// Note: Don't forget to register the constraint as a StepListener with the PhysicsSystem!
pub const VehicleConstraint = struct {};
