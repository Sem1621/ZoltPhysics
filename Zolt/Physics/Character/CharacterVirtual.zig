//! Port of: Jolt/Physics/Character/CharacterVirtual.h, Jolt/Physics/Character/CharacterVirtual.cpp
//! Status: stub
//!
//! Created by the Phase 7 foundation with the final type names, so that the simulation parity harness can name the
//! classes before they are ported (ZoltParity/Physics/SimScenes.zig: `Blocker.character_virtuals`,
//! SimulationParity.zig: `CharacterVirtualSlot`). The porter of the CharacterVirtual group replaces the stub following
//! Docs/Zolt/CharacterVehicleArchitecture.md (E2, E5: `CharacterVirtualSettings` / `CharacterVirtual` derive from
//! CharacterBaseSettings / CharacterBase in CharacterBase.zig, `CharacterContactListener` and
//! `CharacterVsCharacterCollision` are pattern B interfaces, `CharacterContact` embeds `base: CharacterContactKey`),
//! removes `is_stub` (which turns the scenes of `Blocker.character_virtuals` on) and fills the harness slot (the recipe
//! is in the SimScenes header).

/// Contains the configuration of a character
pub const CharacterVirtualSettings = struct {
    /// Zolt marker of a stub class: the simulation parity skips the scenes that need it until it is ported (remove with
    /// the stub)
    pub const is_stub = true;
};

/// This class contains settings that allow you to override the behavior of a character's collision response
pub const CharacterContactSettings = struct {};

/// Uniquely identifies a contact between a character and another body or character
pub const CharacterContactKey = struct {};

/// Encapsulates a collision contact between a character and another rigid body / character
pub const CharacterContact = struct {};

/// This class receives callbacks when a virtual character hits something.
/// Once created, register it on a CharacterVirtual by using the character's SetListener method.
pub const CharacterContactListener = struct {};

/// Interface class that allows a CharacterVirtual to check collision with other CharacterVirtual instances.
/// Since CharacterVirtual instances are not registered anywhere, it is up to the application to test collision against relevant characters.
/// The characters could be stored in a tree structure to make this more efficient.
pub const CharacterVsCharacterCollision = struct {};

/// Simple collision checker that loops over all registered characters.
/// This is a brute force checking algorithm. If you have a lot of characters you may want to store your characters
/// in a hierarchical structure to make this more efficient.
/// Note that this is not thread safe, so make sure that only one CharacterVirtual is checking collision at a time.
pub const CharacterVsCharacterCollisionSimple = struct {};

/// Runtime character object.
/// This object usually represents the player. Contrary to the Character class it doesn't use a rigid body but moves doing collision checks only (hence the name virtual).
/// The advantage of this is that you can determine when the character moves in the frame (usually this has to happen at a very particular point in the frame)
/// but the downside is that other objects don't see this virtual character. To make a CharacterVirtual visible to the simulation, you can optionally create an inner
/// rigid body through CharacterVirtualSettings::mInnerBodyShape. A CharacterVirtual is not tracked by the PhysicsSystem so you need to update it yourself. This also means
/// that a call to PhysicsSystem::SaveState will not save its state, you need to call CharacterVirtual::SaveState yourself.
pub const CharacterVirtual = struct {};
