//! Zolt addition, no Jolt file: bodies for the inline tests of the constraint parts
//! Status: complete
//!
//! Only used by the inline tests of `Zolt/Physics/Constraints/ConstraintPart/*.zig` (Jolt has no unit test for the
//! parts, it tests them through the constraints and the physics system). The bodies are created through a BodyManager
//! like ZoltTests/Physics/BodyTestContext.zig does it (AllocateBody + AddBody, marked as being in the broad phase,
//! activated when not static), with a box shape, a given motion type, allowed DOFs, position, rotation and velocities.
//! `motion_type_pairs` lists every combination of static / kinematic / dynamic bodies, `checkUnmoved` checks that the
//! solver did not change a static or kinematic body.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Quat = @import("../../../Math/Quat.zig").Quat;
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const AllowedDOFs = @import("../../Body/AllowedDOFs.zig").AllowedDOFs;
const Body = @import("../../Body/Body.zig").Body;
const BodyCreationSettings = @import("../../Body/BodyCreationSettings.zig").BodyCreationSettings;
const CreateBodyError = @import("../../Body/BodyCreationSettings.zig").CreateBodyError;
const BodyManager = @import("../../Body/BodyManager.zig").BodyManager;
const MotionType = @import("../../Body/MotionType.zig").MotionType;
const BroadPhaseLayer = @import("../../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayer;
const BroadPhaseLayerInterfaceTable = @import("../../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
const BoxShape = @import("../../Collision/Shape/BoxShape.zig").BoxShape;

/// A pair of motion types for body 1 and body 2
pub const MotionTypePair = struct { MotionType, MotionType };

/// Every combination of static / kinematic / dynamic bodies
pub const motion_type_pairs = blk: {
    const types = [_]MotionType{ .static, .kinematic, .dynamic };
    var pairs: [9]MotionTypePair = undefined;
    for (types, 0..) |t1, i| {
        for (types, 0..) |t2, j|
            pairs[i * 3 + j] = .{ t1, t2 };
    }
    break :blk pairs;
};

/// The pairs where at least one body is dynamic (the only ones where a constraint can do something)
pub const dynamic_motion_type_pairs = [_]MotionTypePair{
    .{ .static, .dynamic },    .{ .kinematic, .dynamic }, .{ .dynamic, .static },
    .{ .dynamic, .kinematic }, .{ .dynamic, .dynamic },
};

/// Options for a test body
pub const BodyOptions = struct {
    position: RVec3 = RVec3.zero(),
    rotation: Quat = Quat.identity(),
    linear_velocity: Vec3 = Vec3.zero(),
    angular_velocity: Vec3 = Vec3.zero(),
    half_extent: Vec3 = Vec3.init(0.5, 1.0, 1.5),
    allowed_dofs: AllowedDOFs = .all,
};

/// A body manager with one object layer for the tests
pub const ConstraintPartTestContext = struct {
    allocator: Allocator,
    layer_interface: BroadPhaseLayerInterfaceTable,
    body_manager: BodyManager = .empty,

    /// Initialize in place (the body manager must not move)
    pub fn init(self: *ConstraintPartTestContext, allocator: Allocator, io: std.Io) Allocator.Error!void {
        self.* = .{ .allocator = allocator, .layer_interface = try .init(allocator, 1, 1) };
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(0, .init(0));
        try self.body_manager.init(allocator, io, 16, 0, &self.layer_interface.base);
    }

    pub fn deinit(self: *ConstraintPartTestContext) void {
        self.body_manager.deinit();
        self.layer_interface.deinit();
    }

    /// Create a body with a box shape, add it to the body manager, mark it as being in the broad phase and activate it
    /// when it is not static
    pub fn createBody(self: *ConstraintPartTestContext, motion_type: MotionType, opts: BodyOptions) CreateBodyError!*Body {
        const box = try BoxShape.create(self.allocator, opts.half_extent, .{});
        var settings = BodyCreationSettings.initWithShape(box.asShape(), opts.position, opts.rotation, motion_type, 0);
        defer settings.deinit();
        settings.allowed_dofs = opts.allowed_dofs;
        if (motion_type != .static) {
            settings.linear_velocity = opts.linear_velocity;
            settings.angular_velocity = opts.angular_velocity;
        }

        const body = try self.body_manager.allocateBody(&settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            @panic("ConstraintPartTestContext: out of bodies"); // The tests create few bodies
        }
        body.setInBroadPhaseInternal(true);
        if (motion_type != .static)
            self.body_manager.activateBodies(&.{body.getID()});
        return body;
    }

    /// Create the two bodies of a test with standard positions, rotations and velocities
    pub fn createBodyPair(self: *ConstraintPartTestContext, pair: MotionTypePair) CreateBodyError![2]*Body {
        const body1 = try self.createBody(pair[0], .{
            .position = RVec3.init(1.0, 2.0, 3.0),
            .rotation = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.3),
            .linear_velocity = Vec3.init(0.5, -1.0, 2.0),
            .angular_velocity = Vec3.init(-0.3, 0.2, 0.7),
        });
        const body2 = try self.createBody(pair[1], .{
            .position = RVec3.init(2.0, 1.5, 4.0),
            .rotation = Quat.rotation(Vec3.init(-2, 1, 0.5).normalized(), 1.1),
            .linear_velocity = Vec3.init(-1.5, 0.25, 0.5),
            .angular_velocity = Vec3.init(0.4, -0.6, 0.1),
            .half_extent = Vec3.init(1.0, 0.5, 0.75),
        });
        return .{ body1, body2 };
    }

    /// Remove and destroy bodies (deactivate, take them out of the "broad phase", destroy)
    pub fn destroyBodies(self: *ConstraintPartTestContext, bodies: []const *Body) void {
        for (bodies) |body| {
            const id = body.getID();
            self.body_manager.deactivateBodies(&.{id});
            body.setInBroadPhaseInternal(false);
            self.body_manager.destroyBodies(&.{id});
        }
    }
};

/// Snapshot of the state that the solver may change
pub const BodySnapshot = struct {
    position: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,

    pub fn take(body: *const Body) BodySnapshot {
        return .{ .position = body.getCenterOfMassPosition(), .rotation = body.getRotation(), .linear_velocity = body.getLinearVelocity(), .angular_velocity = body.getAngularVelocity() };
    }

    pub fn eql(self: BodySnapshot, other: BodySnapshot) bool {
        return self.position.eql(other.position) and self.rotation.eql(other.rotation) and self.linear_velocity.eql(other.linear_velocity) and self.angular_velocity.eql(other.angular_velocity);
    }
};

/// Check that a static or kinematic body did not change (the solver only writes dynamic bodies)
pub fn checkUnmoved(body: *const Body, before: BodySnapshot) !void {
    if (!body.isDynamic())
        try std.testing.expect(BodySnapshot.take(body).eql(before));
}

test "ConstraintPartTestContext creates bodies of every motion type" {
    const testing = std.testing;
    var context: ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    for (motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        try testing.expectEqual(pair[0], bodies[0].getMotionType());
        try testing.expectEqual(pair[1], bodies[1].getMotionType());
        try testing.expectEqual(pair[0] != .static, bodies[0].isActive());
        try testing.expect(bodies[0].isInBroadPhase());
        context.destroyBodies(&bodies);
    }
    try testing.expectEqual(@as(u32, 0), context.body_manager.getNumBodies());

    // Restricted DOFs: a 2D body has no inverse inertia around X and Y
    const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
    const body = try context.createBody(.dynamic, .{ .allowed_dofs = .plane_2d });
    const inv_inertia = body.getMotionProperties().getInverseInertiaForRotation(Mat44.identity());
    try testing.expectEqual(@as(f32, 0.0), inv_inertia.get(0, 0));
    try testing.expectEqual(@as(f32, 0.0), inv_inertia.get(1, 1));
    try testing.expect(inv_inertia.get(2, 2) > 0.0);
    try testing.expect(body.getMotionProperties().getInverseMass() > 0.0);
}

test "ConstraintPartTestContext out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: ConstraintPartTestContext = undefined;
            try context.init(allocator, std.testing.io);
            defer context.deinit();
            _ = try context.createBodyPair(.{ .dynamic, .kinematic });
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, S.run, .{});
}
