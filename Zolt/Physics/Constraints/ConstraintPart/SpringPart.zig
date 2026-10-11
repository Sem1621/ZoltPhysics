//! Port of: Jolt/Physics/Constraints/ConstraintPart/SpringPart.h
//! Status: complete
//!
//! The out parameter `float &outEffectiveMass` of the CalculateSpringProperties* functions is the return value (Jolt
//! writes it on every path). In CalculateSpringPropertiesWithFrequencyAndDamping the local `c` (the damping constant)
//! is `c_damping`, because the parameter `inC` is `c`.

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const SpringSettings = @import("../SpringSettings.zig").SpringSettings;

/// Class used in other constraint parts to calculate the required bias factor in the lagrange multiplier for creating springs
pub const SpringPart = struct {
    bias: f32 = 0.0,
    softness: f32 = 0.0,

    /// Turn off the spring and set a bias only
    ///
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    pub fn calculateSpringPropertiesWithBias(self: *SpringPart, bias: f32) void {
        self.softness = 0.0;
        self.bias = bias;
    }

    /// Calculate spring properties with spring Stiffness (k) and damping (c), this is based on the spring equation: F = -k * x - c * v
    ///
    /// @param inDeltaTime Time step
    /// @param inInvEffectiveMass Inverse effective mass K
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C).
    /// @param inStiffness Spring stiffness k.
    /// @param inDamping Spring damping coefficient c.
    /// @return (outEffectiveMass) the new effective mass K^-1
    pub fn calculateSpringPropertiesWithStiffnessAndDamping(self: *SpringPart, delta_time: f32, inv_effective_mass: f32, bias: f32, c: f32, stiffness: f32, damping: f32) f32 {
        if (Core.enable_asserts) std.debug.assert(stiffness > 0.0 or damping > 0.0);

        // Soft constraints as per: Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011

        // Note that the calculation of beta and gamma below are based on the solution of an implicit Euler integration scheme
        // This scheme is unconditionally stable but has built in damping, so even when you set the damping ratio to 0 there will still
        // be damping. See page 16 and 32.

        // Calculate softness (gamma in the slides)
        // See page 34 and note that the gamma needs to be divided by delta time since we're working with impulses rather than forces:
        // softness = 1 / (dt * (c + dt * k))
        // Note that the spring stiffness is k and the spring damping is c
        self.softness = 1.0 / (delta_time * (damping + delta_time * stiffness));

        // Calculate bias factor (baumgarte stabilization):
        // beta = dt * k / (c + dt * k) = dt * k^2 * softness
        // b = beta / dt * C = dt * k * softness * C
        self.bias = bias + delta_time * stiffness * self.softness * c;

        // Update the effective mass, see post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
        //
        // Newton's Law:
        // M * (v2 - v1) = J^T * lambda
        //
        // Velocity constraint with softness and Baumgarte:
        // J * v2 + softness * lambda + b = 0
        //
        // where b = beta * C / dt
        //
        // We know everything except v2 and lambda.
        //
        // First solve Newton's law for v2 in terms of lambda:
        //
        // v2 = v1 + M^-1 * J^T * lambda
        //
        // Substitute this expression into the velocity constraint:
        //
        // J * (v1 + M^-1 * J^T * lambda) + softness * lambda + b = 0
        //
        // Now collect coefficients of lambda:
        //
        // (J * M^-1 * J^T + softness) * lambda = - J * v1 - b
        //
        // Now we define:
        //
        // K = J * M^-1 * J^T + softness
        //
        // So our new effective mass is K^-1
        return 1.0 / (inv_effective_mass + self.softness);
    }

    /// Calculate spring properties based on frequency and damping ratio
    ///
    /// @param inDeltaTime Time step
    /// @param inInvEffectiveMass Inverse effective mass K
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C).
    /// @param inFrequency Oscillation frequency (Hz).
    /// @param inDamping Damping factor (0 = no damping, 1 = critical damping).
    /// @return (outEffectiveMass) the new effective mass K^-1
    pub fn calculateSpringPropertiesWithFrequencyAndDamping(self: *SpringPart, delta_time: f32, inv_effective_mass: f32, bias: f32, c: f32, frequency: f32, damping: f32) f32 {
        if (Core.enable_asserts) std.debug.assert(frequency > 0.0);

        const effective_mass = 1.0 / inv_effective_mass;

        // Calculate angular frequency
        const omega = 2.0 * math.pi * frequency;

        // Calculate spring stiffness k and damping constant c (page 45)
        const k = effective_mass * math.square(omega);
        const c_damping = 2.0 * effective_mass * damping * omega;

        return self.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, c, k, c_damping);
    }

    /// Calculate spring properties with spring stiffness (k) and damping (c) in acceleration mode, this is based on the spring equation: F = m_eff * (-k * x - c * v) where m_eff is the effective mass of the constraint
    ///
    /// @param inDeltaTime Time step
    /// @param inInvEffectiveMass Inverse effective mass K
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C).
    /// @param inStiffness Spring stiffness k.
    /// @param inDamping Spring damping coefficient c.
    /// @return (outEffectiveMass) the new effective mass K^-1
    pub fn calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(self: *SpringPart, delta_time: f32, inv_effective_mass: f32, bias: f32, c: f32, stiffness: f32, damping: f32) f32 {
        return self.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, c, stiffness / inv_effective_mass, damping / inv_effective_mass);
    }

    /// Calculate spring properties based on SpringSettings object.
    /// Assumes the spring has either stiffness or damping.
    /// @return (outEffectiveMass) the new effective mass K^-1
    pub fn calculateSpringPropertiesWithSettings(self: *SpringPart, delta_time: f32, inv_effective_mass: f32, bias: f32, c: f32, spring_settings: *const SpringSettings) f32 {
        return switch (spring_settings.mode) {
            .frequency_and_damping => self.calculateSpringPropertiesWithFrequencyAndDamping(delta_time, inv_effective_mass, bias, c, spring_settings.frequency, spring_settings.damping),
            .stiffness_and_damping => self.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, c, spring_settings.getStiffness(), spring_settings.damping),
            .mass_normalized_stiffness_and_damping => self.calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(delta_time, inv_effective_mass, bias, c, spring_settings.getStiffness(), spring_settings.damping),
        };
    }

    /// Returns if this spring is active
    pub fn isActive(self: *const SpringPart) bool {
        return self.softness != 0.0;
    }

    /// Get total bias b, including supplied bias and bias for spring: lambda = J v + b
    pub fn getBias(self: *const SpringPart, total_lambda: f32) f32 {
        // Remainder of post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
        //
        // Each iteration we are not computing the whole impulse, we are computing an increment to the impulse and we are updating the velocity.
        // Also, as we solve each constraint we get a perfect v2, but then some other constraint will come along and mess it up.
        // So we want to patch up the constraint while acknowledging the accumulated impulse and the damaged velocity.
        // To help with that we use P for the accumulated impulse and lambda as the update. Mathematically we have:
        //
        // M * (v2new - v2damaged) = J^T * lambda
        // J * v2new + softness * (total_lambda + lambda) + b = 0
        //
        // If we solve this we get:
        //
        // v2new = v2damaged + M^-1 * J^T * lambda
        // J * (v2damaged + M^-1 * J^T * lambda) + softness * total_lambda + softness * lambda + b = 0
        //
        // (J * M^-1 * J^T + softness) * lambda = -(J * v2damaged + softness * total_lambda + b)
        //
        // So our lagrange multiplier becomes:
        //
        // lambda = -K^-1 (J v + softness * total_lambda + b)
        //
        // So we return the bias: softness * total_lambda + b
        return self.softness * total_lambda + self.bias;
    }
};

test "SpringPart without spring is inactive and returns the bias" {
    const testing = std.testing;
    var part: SpringPart = .{};
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getBias(5.0));

    part.calculateSpringPropertiesWithBias(0.5);
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.5), part.getBias(123.0));
}

test "SpringPart stiffness and damping" {
    const testing = std.testing;
    var dt: f32 = 1.0 / 60.0;
    var inv_effective_mass: f32 = 0.5;
    var bias: f32 = 0.1;
    var c: f32 = 0.2;
    var k: f32 = 100.0;
    var damping: f32 = 3.0;
    _ = .{ &dt, &inv_effective_mass, &bias, &c, &k, &damping };

    var part: SpringPart = .{};
    const effective_mass = part.calculateSpringPropertiesWithStiffnessAndDamping(dt, inv_effective_mass, bias, c, k, damping);
    const softness = 1.0 / (dt * (damping + dt * k));
    try testing.expect(part.isActive());
    try testing.expectEqual(softness, part.softness);
    try testing.expectEqual(bias + dt * k * softness * c, part.bias);
    try testing.expectEqual(1.0 / (inv_effective_mass + softness), effective_mass);
    try testing.expectEqual(softness * 2.0 + part.bias, part.getBias(2.0));

    // Damping only
    const effective_mass_damping = part.calculateSpringPropertiesWithStiffnessAndDamping(dt, inv_effective_mass, bias, c, 0.0, damping);
    try testing.expectEqual(1.0 / (dt * damping), part.softness);
    try testing.expectEqual(bias, part.bias);
    try testing.expectEqual(1.0 / (inv_effective_mass + part.softness), effective_mass_damping);

    // Mass normalized: stiffness and damping are multiplied by the effective mass
    var normalized: SpringPart = .{};
    const effective_mass_normalized = normalized.calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(dt, inv_effective_mass, bias, c, k, damping);
    var reference: SpringPart = .{};
    const effective_mass_reference = reference.calculateSpringPropertiesWithStiffnessAndDamping(dt, inv_effective_mass, bias, c, k / inv_effective_mass, damping / inv_effective_mass);
    try testing.expectEqual(effective_mass_reference, effective_mass_normalized);
    try testing.expectEqual(reference.softness, normalized.softness);
    try testing.expectEqual(reference.bias, normalized.bias);
}

test "SpringPart frequency and damping and settings dispatch" {
    const testing = std.testing;
    var dt: f32 = 1.0 / 60.0;
    var inv_effective_mass: f32 = 0.25;
    var frequency: f32 = 2.0;
    var damping: f32 = 0.5;
    _ = .{ &dt, &inv_effective_mass, &frequency, &damping };

    var part: SpringPart = .{};
    const effective_mass = part.calculateSpringPropertiesWithFrequencyAndDamping(dt, inv_effective_mass, 0.0, 1.0, frequency, damping);
    const m = 1.0 / inv_effective_mass;
    const omega = 2.0 * math.pi * frequency;
    var reference: SpringPart = .{};
    const effective_mass_reference = reference.calculateSpringPropertiesWithStiffnessAndDamping(dt, inv_effective_mass, 0.0, 1.0, m * (omega * omega), 2.0 * m * damping * omega);
    try testing.expectEqual(effective_mass_reference, effective_mass);
    try testing.expectEqual(reference.softness, part.softness);
    try testing.expectEqual(reference.bias, part.bias);

    // The settings select the same functions
    const modes = [_]@import("../SpringSettings.zig").SpringMode{ .frequency_and_damping, .stiffness_and_damping, .mass_normalized_stiffness_and_damping };
    for (modes) |mode| {
        const settings = SpringSettings.init(mode, 3.0, 0.75);
        var with_settings: SpringPart = .{};
        const em1 = with_settings.calculateSpringPropertiesWithSettings(dt, inv_effective_mass, 0.1, 0.3, &settings);
        var direct: SpringPart = .{};
        const em2 = switch (mode) {
            .frequency_and_damping => direct.calculateSpringPropertiesWithFrequencyAndDamping(dt, inv_effective_mass, 0.1, 0.3, 3.0, 0.75),
            .stiffness_and_damping => direct.calculateSpringPropertiesWithStiffnessAndDamping(dt, inv_effective_mass, 0.1, 0.3, 3.0, 0.75),
            .mass_normalized_stiffness_and_damping => direct.calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(dt, inv_effective_mass, 0.1, 0.3, 3.0, 0.75),
        };
        try testing.expectEqual(em2, em1);
        try testing.expectEqual(direct.softness, with_settings.softness);
        try testing.expectEqual(direct.bias, with_settings.bias);
    }
}
