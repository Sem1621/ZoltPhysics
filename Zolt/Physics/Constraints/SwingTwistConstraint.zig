//! Port of: Jolt/Physics/Constraints/SwingTwistConstraint.h, Jolt/Physics/Constraints/SwingTwistConstraint.cpp
//! Status: stub
//!
//! Created by the Phase 6 foundation with the final type names, so that RegisterTypes.zig (`constraint_settings_types`)
//! and the parity harnesses (ZoltParity/Physics/ConstraintsParity.zig, SimulationParity.zig) can name the classes
//! before they are ported. The porter of this constraint replaces the stub following the porter template in
//! FixedConstraint.zig (and removes `is_stub`, then fills the harness slots).

/// Swing twist constraint settings, used to create a swing twist constraint
pub const SwingTwistConstraintSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "SwingTwistConstraintSettings";

    /// Zolt marker of a stub class: ConstraintSettings.restoreFromBinaryState does not create it (Jolt's "Failed to
    /// create instance of type") and the parity harnesses skip it, until it is ported (remove with the stub)
    pub const is_stub = true;
};

/// A swing twist constraint is a specialized constraint for humanoid ragdolls that allows limited rotation only
pub const SwingTwistConstraint = struct {};
