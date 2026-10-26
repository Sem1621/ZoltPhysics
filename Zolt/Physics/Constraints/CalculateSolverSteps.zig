//! Port of: Jolt/Physics/Constraints/CalculateSolverSteps.h
//! Status: complete
//!
//! `template <class Type> void operator () (const Type *inObject)` is `call(comptime Type, object: *const Type)`: the
//! template parameter stays a comptime parameter, so a caller that is generic over the calculator (Jolt's
//! `MotionPropertiesCallback` template parameter of the contact constraint manager) calls
//! `calculator.call(MotionProperties, mp)` on either `CalculateSolverSteps` or `DummyCalculateSolverSteps`. `Type`
//! needs `getNumVelocityStepsOverride()` and `getNumPositionStepsOverride()` returning `u32` (MotionProperties,
//! Constraint).
//!
//! Differences with the C++ version: the Zolt addition `Overrides` + `apply` (Phase 12, class A) collects the overrides
//! of many objects in four integers and gives the calculator the state that `call` on each of them gives (the max of
//! the counts, the default flag when an override is 0). LargeIslandSplitter.splitIsland uses it in its loops.

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

    /// Zolt addition (Phase 12, class A): the effect of many `call`s, collected in four integers (the largest and the
    /// smallest override of each kind) for the loops of LargeIslandSplitter.splitIsland, where the four fields of the
    /// calculator did not stay in registers. `apply` then gives the calculator the state that these `call`s give, in
    /// any order: `call` takes the max of the step counts (associative and commutative) and sets the flag when an
    /// override is 0, which is when the smallest override is 0. Without an object the smallest stays maxInt(u32) and
    /// nothing changes.
    pub const Overrides = struct {
        max_velocity_steps: u32 = 0,
        min_velocity_steps: u32 = std.math.maxInt(u32),
        max_position_steps: u32 = 0,
        min_position_steps: u32 = std.math.maxInt(u32),

        /// The overrides of a body or constraint (what `call` reads)
        pub fn add(self: *Overrides, comptime Type: type, object: *const Type) void {
            const num_velocity_steps: u32 = object.getNumVelocityStepsOverride();
            self.max_velocity_steps = @max(self.max_velocity_steps, num_velocity_steps);
            self.min_velocity_steps = @min(self.min_velocity_steps, num_velocity_steps);

            const num_position_steps: u32 = object.getNumPositionStepsOverride();
            self.max_position_steps = @max(self.max_position_steps, num_position_steps);
            self.min_position_steps = @min(self.min_position_steps, num_position_steps);
        }
    };

    /// Zolt addition (Phase 12, class A): `call` for every object of `overrides` (see Overrides)
    pub fn apply(self: *CalculateSolverSteps, overrides: Overrides) void {
        self.num_velocity_steps = @max(self.num_velocity_steps, overrides.max_velocity_steps);
        self.apply_default_velocity = self.apply_default_velocity or overrides.min_velocity_steps == 0;

        self.num_position_steps = @max(self.num_position_steps, overrides.max_position_steps);
        self.apply_default_position = self.apply_default_position or overrides.min_position_steps == 0;
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

test "CalculateSolverSteps.Overrides + apply gives the state of call on every object" {
    // Zolt addition (Phase 12, class A)
    const testing = std.testing;
    const settings: PhysicsSettings = .{ .num_velocity_steps = 10, .num_position_steps = 2 };
    var prng = std.Random.DefaultPrng.init(12);
    const random = prng.random();
    for (0..500) |round| {
        // Start from a calculator that already saw objects (the contacts of the island before the loops of splitIsland)
        var with_call = CalculateSolverSteps.init(&settings);
        for (0..round % 3) |_|
            with_call.call(TestStepsObject, &.{ .velocity_steps = random.uintLessThan(u32, 4), .position_steps = random.uintLessThan(u32, 4) });
        var with_overrides = with_call;

        var overrides: CalculateSolverSteps.Overrides = .{};
        for (0..random.uintLessThan(usize, 20)) |_| {
            // Mostly no override (0), sometimes the largest u32
            const object: TestStepsObject = .{
                .velocity_steps = if (random.uintLessThan(u32, 4) == 0) random.uintLessThan(u32, 300) else if (round == 7) std.math.maxInt(u32) else 0,
                .position_steps = if (random.uintLessThan(u32, 3) == 0) random.uintLessThan(u32, 300) else 0,
            };
            with_call.call(TestStepsObject, &object);
            overrides.add(TestStepsObject, &object);
        }
        with_overrides.apply(overrides);
        try testing.expectEqual(with_call.num_velocity_steps, with_overrides.num_velocity_steps);
        try testing.expectEqual(with_call.num_position_steps, with_overrides.num_position_steps);
        try testing.expectEqual(with_call.apply_default_velocity, with_overrides.apply_default_velocity);
        try testing.expectEqual(with_call.apply_default_position, with_overrides.apply_default_position);
        with_call.finalize();
        with_overrides.finalize();
        try testing.expectEqual(with_call.getNumVelocitySteps(), with_overrides.getNumVelocitySteps());
        try testing.expectEqual(with_call.getNumPositionSteps(), with_overrides.getNumPositionSteps());
    }
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
