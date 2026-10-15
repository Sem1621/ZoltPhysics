//! Port of: UnitTests/Physics/OffsetCenterOfMassShapeTests.cpp
//! Status: complete
//!
//! The four TEST_CASEs repeat the same body creation and mass / inertia checks: `createBody` holds that part (same
//! inputs, same expected values and tolerances), the tests keep the impulse / torque parts.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Body = zolt.Body;
const BoxShapeSettings = zolt.BoxShapeSettings;
const Mat44 = zolt.Mat44;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const allocator = std.testing.allocator;

const half_extent = Vec3.init(0.5, 1.0, 1.5);

/// The box settings, the center of mass settings and the body of a test case (the settings are embedded, like Jolt's
/// stack objects)
const TestBody = struct {
    box: BoxShapeSettings,
    com: OffsetCenterOfMassShapeSettings,
    body: *Body,
    inertia_y: f32,

    fn deinit(self: *TestBody) void {
        // Reverse order of construction
        self.com.deinit();
        self.box.deinit();
    }
};

/// Create box, create body with COM offset `com_offset` and check mass and inertia calculated correctly (in place, the
/// center of mass settings point to the box settings)
fn createBody(self: *TestBody, c: *PhysicsTestContext, com_offset: Vec3) !void {
    // Create box
    self.box = BoxShapeSettings.init(allocator, half_extent, .{});
    self.box.asShapeSettings().setEmbedded();
    errdefer self.box.deinit();

    // Create body with COM offset
    self.com = OffsetCenterOfMassShapeSettings.init(allocator, com_offset, self.box.asShapeSettings());
    self.com.asShapeSettings().setEmbedded();
    errdefer self.com.deinit();
    self.body = try c.createBodyWithShapeSettings(self.com.asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, .dont_activate);

    // Check mass and inertia calculated correctly
    const mass = (8.0 * half_extent.getX() * half_extent.getY() * half_extent.getZ()) * self.box.base.density;
    try checkApproxEqual(self.body.getMotionProperties().getInverseMass(), 1.0 / mass, .{});
    self.inertia_y = if (com_offset.eql(Vec3.zero()))
        mass / 12.0 * (math.square(2.0 * half_extent.getX()) + math.square(2.0 * half_extent.getZ())) // See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    else
        mass / 12.0 * (math.square(2.0 * half_extent.getX()) + math.square(2.0 * half_extent.getZ())) + mass * math.square(com_offset.getX()); // See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia & https://en.wikipedia.org/wiki/Parallel_axis_theorem
    try checkApproxEqual(self.body.getMotionProperties().getInverseInertiaForRotation(Mat44.identity()).get(1, 1), 1.0 / self.inertia_y, .{});
}

fn testAddAngularImpulse(com_offset: Vec3) !void {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    var t: TestBody = undefined;
    try createBody(&t, &c, com_offset);
    defer t.deinit();
    const body = t.body;

    // Add impulse
    const impulse = Vec3.init(0, 10000, 0);
    try expect(!body.isActive());
    c.getBodyInterface().addAngularImpulse(body.getID(), impulse);
    try expect(body.isActive());

    // Check resulting velocity change
    // dv = I^-1 * L
    const delta_v = (1.0 / t.inertia_y) * impulse.getY();
    try checkApproxEqual(body.getLinearVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(body.getAngularVelocity(), Vec3.init(0, delta_v, 0), .{});
}

fn testAddTorque(com_offset: Vec3) !void {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    var t: TestBody = undefined;
    try createBody(&t, &c, com_offset);
    defer t.deinit();
    const body = t.body;

    // Add torque
    const torque = Vec3.init(0, 100000, 0);
    try expect(!body.isActive());
    c.getBodyInterface().addTorque(body.getID(), torque, .{});
    try expect(body.isActive());
    try expect(body.getAngularVelocity().eql(Vec3.zero())); // Angular velocity change should come after the next time step
    _ = try c.simulateSingleStep();

    // Check resulting velocity change
    // dv = I^-1 * T * dt
    const delta_v = (1.0 / t.inertia_y) * torque.getY() * c.getDeltaTime();
    try checkApproxEqual(body.getLinearVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(body.getAngularVelocity(), Vec3.init(0, delta_v, 0), .{});
}

test "TestAddAngularImpulseCOMZero" {
    // Create body with COM offset 0
    try testAddAngularImpulse(Vec3.zero());
}

test "TestAddAngularImpulseCOMOffset" {
    // Create body with COM offset
    try testAddAngularImpulse(Vec3.init(5.0, 0, 0));
}

test "TestAddTorqueCOMZero" {
    // Create body with COM offset 0
    try testAddTorque(Vec3.zero());
}

test "TestAddTorqueCOMOffset" {
    // Create body with COM offset
    try testAddTorque(Vec3.init(5.0, 0, 0));
}
