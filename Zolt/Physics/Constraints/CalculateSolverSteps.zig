//! Port of: Jolt/Physics/Constraints/CalculateSolverSteps.h
//! Status: complete
//!
//! `template <class Type> void operator () (const Type *inObject)` is `call(comptime Type, object: *const Type)`: the
//! template parameter stays a comptime parameter, so a caller that is generic over the calculator (Jolt's
//! `MotionPropertiesCallback` template parameter of the contact constraint manager) calls
//! `calculator.call(MotionProperties, mp)` on either `CalculateSolverSteps` or `DummyCalculateSolverSteps`. `Type`
//! needs `getNumVelocityStepsOverride()` and `getNumPositionStepsOverride()` returning `u32` (MotionProperties,
//! Constraint).

const std = @import("std");
const PhysicsSettings = @import("../PhysicsSettings.zig").PhysicsSettings;

/// @cond INTERNAL
/// Internal class used to calculate the total number of velocity and position steps
pub const CalculateSolverSteps = struct {
    settings: *const PhysicsSettings,

    num_velocity_steps: u32 = 0,
    num_position_steps: u32 = 0,

    apply_default_velocity: bool = false,
    apply_default_position: bool = false,

    /// Constructor
    pub fn init(settings: *const PhysicsSettings) CalculateSolverSteps {
        return .{ .settings = settings };
    }

    /// Combine the number of velocity and position steps for this body/constraint with the current values (operator ())
    pub fn call(self: *CalculateSolverSteps, comptime Type: type, object: *const Type) void {
        const num_velocity_steps: u32 = object.getNumVelocityStepsOverride();
        self.num_velocity_steps = @max(self.num_velocity_steps, num_velocity_steps);
        self.apply_default_velocity = self.apply_default_velocity or num_velocity_steps == 0;

        const num_position_steps: u32 = object.getNumPositionStepsOverride();
        self.num_position_steps = @max(self.num_position_steps, num_position_steps);
        self.apply_default_position = self.apply_default_position or num_position_steps == 0;
    }

    /// Must be called after all bodies/constraints have been processed
    pub fn finalize(self: *CalculateSolverSteps) void {
        // If we have a default velocity/position step count, take the max of the default and the overrides
        if (self.apply_default_velocity)
            self.num_velocity_steps = @max(self.num_velocity_steps, self.settings.num_velocity_steps);
        if (self.apply_default_position)
            self.num_position_steps = @max(self.num_position_steps, self.settings.num_position_steps);
    }

    /// Get the results of the calculation
    pub fn getNumPositionSteps(self: *const CalculateSolverSteps) u32 {
        return self.num_position_steps;
    }

    pub fn getNumVelocitySteps(self: *const CalculateSolverSteps) u32 {
        return self.num_velocity_steps;
    }
};
/// @endcond
/// @cond INTERNAL
/// Dummy class to replace the steps calculator when we don't need the result
pub const DummyCalculateSolverSteps = struct {
    /// operator ()
    pub fn call(self: *const DummyCalculateSolverSteps, comptime Type: type, object: *const Type) void {
        // Nothing to do
        _ = .{ self, object };
    }
};
/// @endcond
/// Test object with the two step overrides (a body's MotionProperties or a Constraint in the solver)
const TestStepsObject = struct {
    velocity_steps: u32,
    position_steps: u32,

    fn getNumVelocityStepsOverride(self: *const TestStepsObject) u32 {
        return self.velocity_steps;
    }

    fn getNumPositionStepsOverride(self: *const TestStepsObject) u32 {
        return self.position_steps;
    }
};

test "CalculateSolverSteps combines overrides with the defaults" {
    const testing = std.testing;
    const settings: PhysicsSettings = .{ .num_velocity_steps = 10, .num_position_steps = 2 };

    // Nothing processed: 0 steps, the defaults only apply when an object without override was seen
    var empty = CalculateSolverSteps.init(&settings);
    empty.finalize();
    try testing.expectEqual(@as(u32, 0), empty.getNumVelocitySteps());
    try testing.expectEqual(@as(u32, 0), empty.getNumPositionSteps());

    // Only overrides: the max of the overrides, also when it is below the default
    var overrides = CalculateSolverSteps.init(&settings);
    overrides.call(TestStepsObject, &.{ .velocity_steps = 3, .position_steps = 7 });
    overrides.call(TestStepsObject, &.{ .velocity_steps = 5, .position_steps = 1 });
    overrides.finalize();
    try testing.expectEqual(@as(u32, 5), overrides.getNumVelocitySteps());
    try testing.expectEqual(@as(u32, 7), overrides.getNumPositionSteps());

    // An object without override pulls in the default, per kind of step
    var mixed = CalculateSolverSteps.init(&settings);
    mixed.call(TestStepsObject, &.{ .velocity_steps = 0, .position_steps = 1 });
    mixed.call(TestStepsObject, &.{ .velocity_steps = 4, .position_steps = 3 });
    mixed.finalize();
    try testing.expectEqual(@as(u32, 10), mixed.getNumVelocitySteps());
    try testing.expectEqual(@as(u32, 3), mixed.getNumPositionSteps());

    var large = CalculateSolverSteps.init(&settings);
    large.call(TestStepsObject, &.{ .velocity_steps = 0, .position_steps = 0 });
    large.call(TestStepsObject, &.{ .velocity_steps = 255, .position_steps = 200 });
    large.finalize();
    try testing.expectEqual(@as(u32, 255), large.getNumVelocitySteps());
    try testing.expectEqual(@as(u32, 200), large.getNumPositionSteps());

    // The dummy accepts the same calls
    const dummy: DummyCalculateSolverSteps = .{};
    dummy.call(TestStepsObject, &.{ .velocity_steps = 1, .position_steps = 1 });
}

test "CalculateSolverSteps with MotionProperties" {
    const testing = std.testing;
    const MotionProperties = @import("../Body/MotionProperties.zig").MotionProperties;

    const settings: PhysicsSettings = .{};
    var mp1: MotionProperties = .{};
    mp1.setNumVelocityStepsOverride(12);
    mp1.setNumPositionStepsOverride(0);
    var mp2: MotionProperties = .{};
    mp2.setNumVelocityStepsOverride(6);
    mp2.setNumPositionStepsOverride(4);

    var calculator = CalculateSolverSteps.init(&settings);
    calculator.call(MotionProperties, &mp1);
    calculator.call(MotionProperties, &mp2);
    calculator.finalize();
    try testing.expectEqual(@as(u32, 12), calculator.getNumVelocitySteps());
    try testing.expectEqual(@as(u32, 4), calculator.getNumPositionSteps()); // max(4, default 2): mp1 has no override
}
