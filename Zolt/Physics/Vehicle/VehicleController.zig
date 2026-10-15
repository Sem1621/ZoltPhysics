//! Port of: Jolt/Physics/Vehicle/VehicleController.h, Jolt/Physics/Vehicle/VehicleController.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names (the abstract bases of the controllers, see
//! WheeledVehicleController.zig). The porter of the Vehicle core group replaces the stub following
//! Docs/Zolt/CharacterVehicleArchitecture.md (E2: `VehicleControllerSettings` is an abstract pattern A root with
//! `ref_count`; `VehicleController` is a pattern A root without `ref_count`, owned by VehicleConstraint).

/// Basic settings object for interface that controls acceleration / deceleration of the vehicle
pub const VehicleControllerSettings = struct {};

/// Runtime data for interface that controls acceleration / deceleration of the vehicle
pub const VehicleController = struct {};
