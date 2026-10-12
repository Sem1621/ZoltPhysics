//! Port of: Jolt/Physics/Constraints/SixDOFConstraint.h, Jolt/Physics/Constraints/SixDOFConstraint.cpp
//! Status: stub
//!
//! Created by the Phase 6 foundation with the final type names, so that RegisterTypes.zig (`constraint_settings_types`)
//! and the parity harnesses (ZoltParity/Physics/ConstraintsParity.zig, SimulationParity.zig) can name the classes
//! before they are ported. The porter of this constraint replaces the stub following the porter template in
//! FixedConstraint.zig (and removes `is_stub`, then fills the harness slots).

/// 6 Degree Of Freedom Constraint setup structure. Allows control over each of the 6 degrees of freedom.
pub const SixDOFConstraintSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "SixDOFConstraintSettings";

    /// Zolt marker of a stub class: ConstraintSettings.restoreFromBinaryState does not create it (Jolt's "Failed to
    /// create instance of type") and the parity harnesses skip it, until it is ported (remove with the stub)
    pub const is_stub = true;
};

/// 6 Degree Of Freedom Constraint. Allows control over each of the 6 degrees of freedom.
pub const SixDOFConstraint = struct {};
