//! Port of: Jolt/Physics/Constraints/Constraint.h, Jolt/Physics/Constraints/Constraint.cpp
//! Status: stub
//!
//! Stub for Phase 5 F1 (the Body package), replaced by the real port in F2 (SimulationArchitecture D8, section 2). F1
//! only uses `Constraint` as a pointer type: `StateRecorderFilter.shouldSaveConstraint(*const Constraint)`. It is an
//! opaque type so that pointers to it are ordinary pointers until F2 declares the struct.

/// Base class for all physics constraints (stub, see the file comment)
pub const Constraint = opaque {};
