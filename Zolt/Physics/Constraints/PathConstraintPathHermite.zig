//! Port of: Jolt/Physics/Constraints/PathConstraintPathHermite.h, Jolt/Physics/Constraints/PathConstraintPathHermite.cpp
//! Status: stub
//!
//! Created by the Phase 6 foundation with the final type name, so that RegisterTypes.zig (`path_constraint_path_types`)
//! and the parity harnesses can name the class before it is ported. The porter replaces the stub following the porter
//! template in FixedConstraint.zig (a path is a PathConstraintPath, not a constraint) and removes `is_stub`.

/// A path that follows a Hermite spline
pub const PathConstraintPathHermite = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "PathConstraintPathHermite";

    /// Zolt marker of a stub class: PathConstraintPath.restoreFromBinaryState does not create it (Jolt's "Failed to
    /// create instance of type") and the parity harnesses skip it, until it is ported (remove with the stub)
    pub const is_stub = true;
};
