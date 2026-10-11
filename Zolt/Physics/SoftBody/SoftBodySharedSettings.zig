//! Port of: Jolt/Physics/SoftBody/SoftBodySharedSettings.h, Jolt/Physics/SoftBody/SoftBodySharedSettings.cpp
//! Status: stub
//!
//! Phase 9 (SoftBody) stub, created by Phase 5 F1 (SimulationArchitecture D1) because `SoftBodyCreationSettings.settings`
//! and `SoftBodyMotionProperties.getSettings` refer to it. It only has the reference count of Jolt's
//! `RefTarget<SoftBodySharedSettings>`. No shared settings can be created on the heap before Phase 9, so the last
//! `release` (which would destroy a heap object) panics.

const RefCount = @import("../../Core/Reference.zig").RefCount;

/// This class defines the setup of all particles and their constraints. It is used during the simulation and can be shared between multiple soft bodies. (stub)
pub const SoftBodySharedSettings = struct {
    ref_count: RefCount = .{},

    pub fn addRef(self: *const SoftBodySharedSettings) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const SoftBodySharedSettings) void {
        if (self.ref_count.release())
            @panic("SoftBodySharedSettings: Phase 9"); // Unreachable: no heap object exists before Phase 9
    }

    /// Mark this object as embedded (stack or member object)
    pub fn setEmbedded(self: *const SoftBodySharedSettings) void {
        self.ref_count.setEmbedded();
    }
};
