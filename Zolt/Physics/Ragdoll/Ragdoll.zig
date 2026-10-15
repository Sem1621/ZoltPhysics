//! Port of: Jolt/Physics/Ragdoll/Ragdoll.h, Jolt/Physics/Ragdoll/Ragdoll.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! classes before they are ported (ZoltParity/Physics/SimScenes.zig: `Blocker.ragdolls`, SimulationParity.zig:
//! `RagdollSlot`). The stub does not import the Skeleton files (Zolt/Skeleton is the Skeleton group's). The porter of
//! the Ragdoll group replaces the stub following Docs/Zolt/CharacterVehicleArchitecture.md (E2: `RagdollSettings` and
//! `Ragdoll` are reference counted plain structs that keep their allocator, `RagdollSettings.Part` embeds
//! `base: BodyCreationSettings`; E7), removes `is_stub` (which turns the scenes of `Blocker.ragdolls` on) and fills the
//! harness slot (the recipe is in the SimScenes header).

/// Contains the structure of a ragdoll
pub const RagdollSettings = struct {
    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Runtime ragdoll information
pub const Ragdoll = struct {};
