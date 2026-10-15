//! Port of: Jolt/Physics/Character/Character.h, Jolt/Physics/Character/Character.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! class before it is ported (ZoltParity/Physics/SimScenes.zig: `Blocker.characters`, SimulationParity.zig:
//! `CharacterSlot`). The porter of the Character group replaces the stub following
//! Docs/Zolt/CharacterVehicleArchitecture.md (E2: `CharacterSettings` embeds `base: CharacterBaseSettings`, `Character`
//! embeds `base: CharacterBase`, both pattern A, see CharacterBase.zig), removes `is_stub` (which turns the scenes of
//! `Blocker.characters` on) and fills the harness slot (the recipe is in the SimScenes header).

/// Contains the configuration of a character
pub const CharacterSettings = struct {
    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// Runtime character object.
/// This object usually represents the player or a humanoid AI. It uses a single rigid body,
/// usually with a capsule shape to simulate movement and collision for the character.
/// The character is a keyframed object, the application controls it by setting the velocity.
pub const Character = struct {};
