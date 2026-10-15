//! Port of: Jolt/Physics/Vehicle/Wheel.h, Jolt/Physics/Vehicle/Wheel.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names. The porter of the Vehicle core group replaces the stub
//! following Docs/Zolt/CharacterVehicleArchitecture.md (E2: `WheelSettings` is a concrete pattern A root with
//! `ref_count`, `Wheel` a pattern A root without `ref_count`, owned by VehicleConstraint).

/// Base class for wheel settings, each VehicleController can implement a derived class of this
pub const WheelSettings = struct {};

/// Base class for runtime data for a wheel, each VehicleController can implement a derived class of this
pub const Wheel = struct {};
