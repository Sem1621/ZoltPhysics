//! Port of: Jolt/Physics/EActivation.h
//! Status: complete
//!
//! `EActivation` is `Activation` (the guide drops the `E`); Jolt's defaulted `EActivation inActivationMode = Activate`
//! parameters become options (`.{ .activation = .activate }`, SimulationArchitecture D6).

/// Enum used by AddBody to determine if the body needs to be initially active (EActivation)
pub const Activation = enum {
    /// Activate the body, making it part of the simulation
    activate,
    /// Leave activation state as it is (will not deactivate an active body)
    dont_activate,
};
