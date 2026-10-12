//! Port of: Jolt/Physics/Constraints/SliderConstraint.h, Jolt/Physics/Constraints/SliderConstraint.cpp
//! Status: stub
//!
//! Created by the Phase 6 foundation with the final type names, so that RegisterTypes.zig (`constraint_settings_types`)
//! and the parity harnesses (ZoltParity/Physics/ConstraintsParity.zig, SimulationParity.zig) can name the classes
//! before they are ported. The porter of this constraint replaces the stub following the porter template in
//! FixedConstraint.zig (and removes `is_stub`, then fills the harness slots).

/// Slider constraint settings, used to create a slider constraint
pub const SliderConstraintSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "SliderConstraintSettings";

    /// Zolt marker of a stub class: ConstraintSettings.restoreFromBinaryState does not create it (Jolt's "Failed to
    /// create instance of type") and the parity harnesses skip it, until it is ported (remove with the stub)
    pub const is_stub = true;
};

/// A slider constraint allows movement in only 1 axis (and no rotation). Also known as a prismatic constraint.
pub const SliderConstraint = struct {};
