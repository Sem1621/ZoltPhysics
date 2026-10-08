//! Port of: Jolt/Physics/Collision/CastResult.h
//! Status: complete
//!
//! Value types with non virtual inheritance are flattened (D12): `RayCastResult : BroadPhaseCastResult` repeats the
//! base fields first, in the same order, so field access reads like the C++ (`hit.mFraction` -> `hit.fraction`).
//! `virtual.checkPrefix` keeps the copies in sync.

const zolt = @import("zolt");
const math = zolt.math;
const virtual = @import("../../Core/Virtual.zig");
const BodyID = @import("../Body/BodyID.zig").BodyID;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;

/// Structure that holds a ray cast or other object cast hit
pub const BroadPhaseCastResult = struct {
    /// Body that was hit
    body_id: BodyID = .invalid,
    /// Hit fraction of the ray/object [0, 1], HitPoint = Start + mFraction * (End - Start)
    fraction: f32 = 1.0 + math.flt_epsilon,

    /// Function required by the CollisionCollector. A smaller fraction is considered to be a 'better hit'. For rays/cast shapes we can just use the collision fraction.
    pub fn getEarlyOutFraction(self: *const BroadPhaseCastResult) f32 {
        return self.fraction;
    }

    /// Reset this result so it can be reused for a new cast.
    pub fn reset(self: *BroadPhaseCastResult) void {
        self.body_id = .invalid;
        self.fraction = 1.0 + math.flt_epsilon;
    }
};

/// Specialization of cast result against a shape
pub const RayCastResult = struct {
    // BroadPhaseCastResult
    body_id: BodyID = .invalid,
    fraction: f32 = 1.0 + math.flt_epsilon,

    /// Sub shape ID of shape that we collided against
    sub_shape_id2: SubShapeID = .empty,

    /// See BroadPhaseCastResult::GetEarlyOutFraction
    pub fn getEarlyOutFraction(self: *const RayCastResult) f32 {
        return self.fraction;
    }

    /// See BroadPhaseCastResult::Reset
    pub fn reset(self: *RayCastResult) void {
        self.body_id = .invalid;
        self.fraction = 1.0 + math.flt_epsilon;
    }

    comptime {
        virtual.checkPrefix(BroadPhaseCastResult, RayCastResult);
    }
};
