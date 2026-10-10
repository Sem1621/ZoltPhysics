//! Port of: Jolt/Physics/SoftBody/SoftBodyMotionProperties.h, Jolt/Physics/SoftBody/SoftBodyMotionProperties.cpp
//! Status: stub
//!
//! Phase 9 (SoftBody) stub, created by Phase 5 F1 (SimulationArchitecture D1): the final names and signatures that
//! Body and BodyManager call. `SoftBodyMotionProperties : public MotionProperties` embeds its base as `base`;
//! `fromBase` / `fromBaseConst` are Jolt's `static_cast<SoftBodyMotionProperties *>(mMotionProperties)`. The functions
//! panic: no soft body can be created before Phase 9 (`BodyManager.allocateSoftBody` and the BodyInterface soft body
//! creation functions are not declared yet), so they are unreachable from the public API.

const MotionProperties = @import("../Body/MotionProperties.zig").MotionProperties;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const SoftBodyCreationSettings = @import("SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;

/// This class contains the runtime information of a soft body. (stub)
pub const SoftBodyMotionProperties = struct {
    base: MotionProperties = .{},

    /// static_cast<SoftBodyMotionProperties *>(MotionProperties *)
    pub fn fromBase(mp: *MotionProperties) *SoftBodyMotionProperties {
        return @fieldParentPtr("base", mp);
    }

    /// static_cast<const SoftBodyMotionProperties *>(const MotionProperties *)
    pub fn fromBaseConst(mp: *const MotionProperties) *const SoftBodyMotionProperties {
        return @fieldParentPtr("base", mp);
    }

    /// Initialize the soft body motion properties
    pub fn initialize(self: *SoftBodyMotionProperties, settings: *const SoftBodyCreationSettings) void {
        _ = .{ self, settings };
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Get the shared settings of the soft body
    pub fn getSettings(self: *const SoftBodyMotionProperties) ?*const SoftBodySharedSettings {
        _ = self;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Get the number of solver iterations
    pub fn getNumIterations(self: *const SoftBodyMotionProperties) u32 {
        _ = self;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Get the pressure of the soft body
    pub fn getPressure(self: *const SoftBodyMotionProperties) f32 {
        _ = self;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Update the position of the body while simulating (set to false for something that is attached to the static world)
    pub fn getUpdatePosition(self: *const SoftBodyMotionProperties) bool {
        _ = self;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Global setting to turn on/off double sided face collision detection
    pub fn getFacesDoubleSided(self: *const SoftBodyMotionProperties) bool {
        _ = self;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Get the vertex radius of the soft body
    pub fn getVertexRadius(self: *const SoftBodyMotionProperties) f32 {
        _ = self;
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Saving state for replay
    pub fn saveState(self: *const SoftBodyMotionProperties, stream: *StateRecorder) void {
        _ = .{ self, stream };
        @panic("SoftBodyMotionProperties: Phase 9");
    }

    /// Restoring state for replay
    pub fn restoreState(self: *SoftBodyMotionProperties, stream: *StateRecorder) void {
        _ = .{ self, stream };
        @panic("SoftBodyMotionProperties: Phase 9");
    }
};
