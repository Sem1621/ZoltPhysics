//! Port of: Jolt/Physics/Constraints/PathConstraintPath.h, Jolt/Physics/Constraints/PathConstraintPath.cpp
//! Status: stub
//!
//! Created by the Phase 6 foundation with the final type name, so that RegisterTypes.zig (`path_constraint_path_types`,
//! the classes that PathConstraintPath::sRestoreFromBinaryState can create) and the parity harnesses can name the
//! classes before they are ported. The porter replaces the stub (an abstract RefTarget root like GroupFilter, pattern A:
//! `rtti_name` data entry, `restoreFromBinaryState` over `RegisterTypes.path_constraint_path_types` like
//! ConstraintSettings.restoreFromBinaryState, skipping classes that declare `is_stub`).

/// The path for a path constraint. It allows attaching two bodies to each other while giving the second body the freedom to move along a path relative to the first.
pub const PathConstraintPath = struct {};
