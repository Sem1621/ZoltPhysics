//! Zolt addition, no Jolt file: tests of ConstraintManager and the constraint base classes (SimulationArchitecture D8)
//! Status: complete
//!
//! Jolt has no unit test for the ConstraintManager. The constraints are `zolt.test_constraints.TestConstraint`s (a
//! TwoBodyConstraint made with virtual.make that counts the solver calls), on bodies of a BodyManager: adding and
//! removing (the swap remove keeps `constraint_index`), the reference counts, the active constraints, the sort by
//! priority and index, the solver loops (every constraint is solved, the results are OR-ed), building islands (which
//! activates sleeping bodies), save / restore (validating, non-validating, filtered), allocation failures and adding /
//! removing from several threads. One test uses Jolt's FixedConstraint and PointConstraint (Phase 6) in the manager:
//! the active constraints, the solver loops stopping the bodies, and save / restore of the lambdas of their parts.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const BodyTestContext = @import("BodyTestContext.zig").BodyTestContext;

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const Constraint = zolt.Constraint;
const ConstraintManager = zolt.ConstraintManager;
const IslandBuilder = zolt.IslandBuilder;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StateRecorderFilter = zolt.StateRecorderFilter;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const TestConstraint = zolt.test_constraints.TestConstraint;
const TestConstraintSettings = zolt.test_constraints.TestConstraintSettings;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

const io = std.testing.io;

/// A body manager with bodies and a constraint manager (in place)
const Context = struct {
    bodies: BodyTestContext = .{},
    manager: ConstraintManager = .empty,
    settings: TestConstraintSettings = undefined,
    allocator: Allocator = undefined,

    fn init(self: *Context, allocator: Allocator) !void {
        self.* = .{ .allocator = allocator };
        try self.bodies.init(allocator, 64, 0);
        errdefer self.bodies.deinit();
        try self.manager.init(allocator, io, &self.bodies.body_manager);
        self.settings = .init(allocator);
    }

    fn deinit(self: *Context) void {
        self.manager.deinit();
        self.settings.asSettings().deinit();
        self.bodies.deinit();
    }

    fn createBody(self: *Context, motion_type: zolt.MotionType, activation: zolt.Activation) !*Body {
        var settings = BodyCreationSettings.initWithShape((try BoxShape.create(self.allocator, Vec3.replicate(0.5), .{})).asShape(), RVec3.zero(), Quat.identity(), motion_type, if (motion_type == .static) Layers.non_moving else Layers.moving);
        defer settings.deinit();
        return (try self.bodies.createAndAddBody(&settings, activation)) orelse error.OutOfBodies;
    }

    /// A constraint between two bodies, the caller owns the reference
    fn createConstraint(self: *Context, body1: *Body, body2: *Body) !Ref(Constraint) {
        const two_body = try self.settings.base.createConstraint(self.allocator, body1, body2);
        return .init(&two_body.base);
    }
};

fn testConstraint(c: *Constraint) *TestConstraint {
    return zolt.virtual.downcast(TestConstraint, zolt.virtual.downcast(zolt.TwoBodyConstraint, c));
}

test "ConstraintManager add and remove keep the constraint indices" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const a = try c.createBody(.dynamic, .activate);
    const b = try c.createBody(.dynamic, .activate);

    var refs: [5]Ref(Constraint) = undefined;
    for (&refs) |*r| r.* = try c.createConstraint(a, b);
    defer for (&refs) |*r| r.deinit();
    var ptrs: [5]*Constraint = undefined;
    for (&ptrs, refs) |*p, r| p.* = r.get().?;

    for (ptrs) |p| try expectEqual(Constraint.invalid_constraint_index, p.constraint_index);
    try c.manager.add(ptrs[0..3]);
    try c.manager.add(ptrs[3..]);
    try expectEqual(@as(u32, 5), c.manager.getNumConstraints());
    for (ptrs, 0..) |p, i| {
        try expectEqual(@as(u32, @intCast(i)), p.constraint_index);
        try expectEqual(@as(u32, 2), p.getRefCount()); // Ours and the manager's
    }

    // Removing from the middle moves the last constraint into the hole
    c.manager.remove(&.{ptrs[1]});
    try expectEqual(Constraint.invalid_constraint_index, ptrs[1].constraint_index);
    try expectEqual(@as(u32, 1), ptrs[1].getRefCount());
    try expectEqual(@as(u32, 1), ptrs[4].constraint_index);
    try expect(c.manager.constraints.items[1].get().? == ptrs[4]);
    try expectEqual(@as(u32, 4), c.manager.getNumConstraints());

    // Removing the last one moves nothing; several at once
    c.manager.remove(&.{ ptrs[3], ptrs[0] });
    try expectEqual(@as(u32, 2), c.manager.getNumConstraints());
    try expectEqual(@as(u32, 0), ptrs[2].constraint_index); // Moved into the hole of ptrs[0]
    try expectEqual(@as(u32, 1), ptrs[4].constraint_index);
    for (c.manager.constraints.items, 0..) |r, i|
        try expectEqual(@as(u32, @intCast(i)), r.get().?.constraint_index);

    // getConstraints returns references
    {
        var copy = try c.manager.getConstraints(allocator);
        defer {
            for (copy.items) |*r| r.deinit();
            copy.deinit(allocator);
        }
        try expectEqual(@as(usize, 2), copy.items.len);
        try expect(copy.items[0].get().? == ptrs[2]);
        try expectEqual(@as(u32, 3), ptrs[2].getRefCount());
    }
    try expectEqual(@as(u32, 2), ptrs[2].getRefCount());

    // The manager releases its references when it is destroyed (the constraint without other references is freed)
    var owned_by_manager = try c.createConstraint(a, b);
    try c.manager.add(&.{owned_by_manager.get().?});
    owned_by_manager.deinit(); // Leak checked: freed by the manager's deinit
}

test "ConstraintManager active constraints and sorting" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const active = try c.createBody(.dynamic, .activate);
    const sleeping = try c.createBody(.dynamic, .dont_activate);
    const static = try c.createBody(.static, .dont_activate);
    const kinematic = try c.createBody(.kinematic, .activate);

    // [0] active, [1] disabled, [2] both bodies inactive, [3] static vs active, [4] kinematic vs static (no dynamic
    // body), [5] active vs sleeping
    const pairs = [_][2]*Body{ .{ active, sleeping }, .{ active, active }, .{ sleeping, static }, .{ static, active }, .{ kinematic, static }, .{ active, sleeping } };
    var refs: [pairs.len]Ref(Constraint) = undefined;
    for (&refs, pairs) |*r, p| r.* = try c.createConstraint(p[0], p[1]);
    defer for (&refs) |*r| r.deinit();
    refs[1].get().?.setEnabled(false);
    var ptrs: [pairs.len]*Constraint = undefined;
    for (&ptrs, refs) |*p, r| p.* = r.get().?;
    try c.manager.add(&ptrs);

    var out: [pairs.len]*Constraint = undefined;
    const n = c.manager.getActiveConstraints(0, pairs.len, &out);
    try expectEqual(@as(u32, 3), n);
    try expect(out[0] == ptrs[0] and out[1] == ptrs[3] and out[2] == ptrs[5]);

    // A sub range (DetermineActiveConstraints works in batches)
    try expectEqual(@as(u32, 1), c.manager.getActiveConstraints(1, 4, &out));
    try expect(out[0] == ptrs[3]);

    // Sort: priority first, then the index in the manager
    ptrs[0].setConstraintPriority(5);
    ptrs[3].setConstraintPriority(1);
    ptrs[5].setConstraintPriority(1);
    ptrs[1].setConstraintPriority(0);
    var idx = [_]u32{ 0, 1, 2, 3, 4, 5 };
    ConstraintManager.sortConstraints(&ptrs, &idx);
    // priorities [5, 0, 0, 1, 0, 1] -> 1, 2, 4 (0), 3, 5 (1), 0 (5)
    try std.testing.expectEqualSlices(u32, &.{ 1, 2, 4, 3, 5, 0 }, &idx);
}

test "ConstraintManager solver loops solve every constraint and OR the results" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const a = try c.createBody(.dynamic, .activate);
    const b = try c.createBody(.dynamic, .activate);

    var refs: [4]Ref(Constraint) = undefined;
    for (&refs) |*r| r.* = try c.createConstraint(a, b);
    defer for (&refs) |*r| r.deinit();
    var ptrs: [4]*Constraint = undefined;
    for (&ptrs, refs) |*p, r| p.* = r.get().?;
    const idx = [_]u32{ 3, 0, 2 }; // Not every constraint, in this order

    ConstraintManager.setupVelocityConstraints(&ptrs, 1.0 / 60.0);
    for (ptrs) |p| try expectEqual(@as(u32, 1), testConstraint(p).num_setup.load(.monotonic));

    // Warm start calls the callback (CalculateSolverSteps) with each constraint before warm starting it
    const Callback = struct {
        visited: [4]?*const Constraint = @splat(null),
        count: usize = 0,
        pub fn call(self: *@This(), comptime T: type, object: *const T) void {
            self.visited[self.count] = object;
            self.count += 1;
        }
    };
    var callback: Callback = .{};
    ConstraintManager.warmStartVelocityConstraints(&ptrs, &idx, 0.5, &callback);
    try expectEqual(@as(usize, 3), callback.count);
    for (idx, 0..) |i, n| try expect(callback.visited[n].? == ptrs[i]);
    try expectEqual(@as(f32, 0.5), testConstraint(ptrs[3]).last_warm_start_impulse_ratio);
    try expectEqual(@as(u32, 0), testConstraint(ptrs[1]).num_warm_start.load(.monotonic));

    // No impulse anywhere: false
    try expect(!ConstraintManager.solveVelocityConstraints(&ptrs, &idx, 1.0 / 60.0));
    try expect(!ConstraintManager.solvePositionConstraints(&ptrs, &idx, 1.0 / 60.0, 0.2));

    // An impulse in the first constraint: true, and the others are still solved
    testConstraint(ptrs[3]).velocity_result = true;
    testConstraint(ptrs[3]).position_result = true;
    try expect(ConstraintManager.solveVelocityConstraints(&ptrs, &idx, 1.0 / 60.0));
    try expect(ConstraintManager.solvePositionConstraints(&ptrs, &idx, 1.0 / 60.0, 0.2));
    for (idx) |i| {
        try expectEqual(@as(u32, 2), testConstraint(ptrs[i]).num_solve_velocity.load(.monotonic));
        try expectEqual(@as(u32, 2), testConstraint(ptrs[i]).num_solve_position.load(.monotonic));
    }
    try expectEqual(@as(u32, 0), testConstraint(ptrs[1]).num_solve_velocity.load(.monotonic));
}

test "ConstraintManager buildIslands activates bodies and links them" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const a = try c.createBody(.dynamic, .activate);
    const b = try c.createBody(.dynamic, .dont_activate);
    const d = try c.createBody(.dynamic, .activate);
    const e = try c.createBody(.dynamic, .activate);
    const static = try c.createBody(.static, .dont_activate);

    // a - b - d form one island (b is woken up), e is attached to the static body only
    const pairs = [_][2]*Body{ .{ a, b }, .{ b, d }, .{ static, e } };
    var refs: [pairs.len]Ref(Constraint) = undefined;
    for (&refs, pairs) |*r, p| r.* = try c.createConstraint(p[0], p[1]);
    defer for (&refs) |*r| r.deinit();
    var ptrs: [pairs.len]*Constraint = undefined;
    for (&ptrs, refs) |*p, r| p.* = r.get().?;

    var temp: TempAllocatorImpl = try .init(allocator, 64 * 1024);
    defer temp.deinit();
    var builder: IslandBuilder = .empty;
    try builder.init(allocator, 64);
    defer builder.deinit();
    try builder.prepareContactConstraints(0, temp.tempAllocator());
    try builder.prepareNonContactConstraints(pairs.len, temp.tempAllocator());

    try expect(!b.isActive());
    ConstraintManager.buildIslands(&ptrs, &builder, &c.bodies.body_manager);
    try expect(b.isActive());

    // Active bodies: a, d, e, b (b was activated last)
    const bm = &c.bodies.body_manager;
    const num_active = bm.getNumActiveBodies(.rigid_body);
    try expectEqual(@as(u32, 4), num_active);
    try builder.finalize(bm.getActiveBodiesUnsafe(.rigid_body)[0..num_active], 0, temp.tempAllocator());
    try expectEqual(@as(u32, 2), builder.getNumIslands());

    // The island with the most constraints comes first
    const island0 = builder.getBodiesInIsland(0);
    try expectEqual(@as(usize, 3), island0.len);
    try std.testing.expectEqualSlices(BodyID, &.{ a.getID(), d.getID(), b.getID() }, island0); // In active body order
    try std.testing.expectEqualSlices(u32, &.{ 0, 1 }, builder.getConstraintsInIsland(0));
    try std.testing.expectEqualSlices(BodyID, &.{e.getID()}, builder.getBodiesInIsland(1));
    try std.testing.expectEqualSlices(u32, &.{2}, builder.getConstraintsInIsland(1));

    builder.resetIslands(temp.tempAllocator());
    try expect(temp.isEmpty());
}

test "ConstraintManager saveState and restoreState" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const a = try c.createBody(.dynamic, .activate);
    const b = try c.createBody(.dynamic, .activate);

    var refs: [3]Ref(Constraint) = undefined;
    for (&refs, 0..) |*r, i| {
        r.* = try c.createConstraint(a, b);
        testConstraint(r.get().?).value = @floatFromInt(i + 1);
    }
    defer for (&refs) |*r| r.deinit();
    var ptrs: [3]*Constraint = undefined;
    for (&ptrs, refs) |*p, r| p.* = r.get().?;
    try c.manager.add(&ptrs);
    ptrs[1].setEnabled(false);

    // Count, then per constraint its index, Constraint::SaveState (enabled) and the TestConstraint's value
    var state = StateRecorderImpl.init(allocator);
    defer state.deinit();
    try c.manager.saveState(state.recorder(), null);
    try expectEqual(@as(usize, 4 + 3 * (4 + 1 + 4)), state.getData().len);
    try expectEqual(@as(u32, 3), std.mem.readInt(u32, state.getData()[0..4], .little));
    try expectEqual(@as(u8, 0), state.getData()[4 + 9 + 4]); // Enabled of constraint 1

    // Change everything and restore
    for (ptrs) |p| {
        testConstraint(p).value = -1.0;
        p.setEnabled(true);
    }
    try expect(c.manager.restoreState(state.recorder()));
    for (ptrs, 0..) |p, i| try expectEqual(@as(f32, @floatFromInt(i + 1)), testConstraint(p).value);
    try expect(!ptrs[1].getEnabled() and ptrs[0].getEnabled());

    // Validating restore of the current state
    state.rewind();
    state.recorder().setValidating(true);
    try expect(c.manager.restoreState(state.recorder()));
    try expectEqual(@as(u32, 0), state.num_mismatches);

    // A filter: only the disabled constraint
    const OnlyDisabled = struct {
        pub const overrides = .{.shouldSaveConstraint};
        base: StateRecorderFilter = .init(@This()),
        pub fn shouldSaveConstraint(self: *const @This(), constraint: *const Constraint) bool {
            _ = self;
            return !constraint.getEnabled();
        }
    };
    const filter: OnlyDisabled = .{};
    var partial = StateRecorderImpl.init(allocator);
    defer partial.deinit();
    try c.manager.saveState(partial.recorder(), &filter.base);
    try expectEqual(@as(usize, 4 + 4 + 1 + 4), partial.getData().len);
    for (ptrs) |p| testConstraint(p).value = -1.0;
    try expect(c.manager.restoreState(partial.recorder())); // Non-validating: restores only constraint 1
    try expectEqual(@as(f32, -1.0), testConstraint(ptrs[0]).value);
    try expectEqual(@as(f32, 2.0), testConstraint(ptrs[1]).value);
    try expectEqual(@as(f32, -1.0), testConstraint(ptrs[2]).value);
}

test "ConstraintManager with FixedConstraint and PointConstraint: active constraints, solver, save and restore" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const a = try c.createBody(.dynamic, .activate);
    const b = try c.createBody(.dynamic, .activate);
    const static = try c.createBody(.static, .dont_activate);

    // [0] a fixed to the static body, [1] a point between a and b, [2] a disabled point between the static body and b
    var fixed_settings = zolt.FixedConstraintSettings.init(allocator);
    defer fixed_settings.asSettings().deinit();
    fixed_settings.auto_detect_point = true;
    var point_settings = zolt.PointConstraintSettings.init(allocator);
    defer point_settings.asSettings().deinit();
    point_settings.point1 = RVec3.init(0.25, 0.5, 0.0);
    point_settings.point2 = RVec3.init(0.25, 0.5, 0.0);
    var refs = [_]Ref(Constraint){
        .init(&(try fixed_settings.createConstraint(allocator, static, a)).base),
        .init(&(try point_settings.createConstraint(allocator, a, b)).base),
        .init(&(try point_settings.createConstraint(allocator, static, b)).base),
    };
    defer for (&refs) |*r| r.deinit();
    var ptrs: [3]*Constraint = undefined;
    for (&ptrs, refs) |*p, r| p.* = r.get().?;
    ptrs[2].setEnabled(false);
    try c.manager.add(&ptrs);
    try expectEqual(zolt.ConstraintSubType.fixed, ptrs[0].getSubType());
    try expectEqual(zolt.ConstraintSubType.point, ptrs[1].getSubType());

    var active: [3]*Constraint = undefined;
    try expectEqual(@as(u32, 2), c.manager.getActiveConstraints(0, 3, &active));
    try expect(active[0] == ptrs[0] and active[1] == ptrs[1]);
    var idx = [_]u32{ 0, 1 };
    ConstraintManager.sortConstraints(active[0..2], &idx);

    // Both bodies fall: the solver stops a (fixed to the static body) and b (hanging from a)
    a.setLinearVelocity(Vec3.init(0.0, -1.0, 0.0));
    b.setLinearVelocity(Vec3.init(0.5, -1.0, 0.0));
    const NoCallback = struct {
        pub fn call(self: *@This(), comptime T: type, object: *const T) void {
            _ = self;
            _ = object;
        }
    };
    var callback: NoCallback = .{};
    ConstraintManager.setupVelocityConstraints(active[0..2], 1.0 / 60.0);
    ConstraintManager.warmStartVelocityConstraints(active[0..2], &idx, 1.0, &callback);
    try expect(ConstraintManager.solveVelocityConstraints(active[0..2], &idx, 1.0 / 60.0));
    for (0..30) |_| _ = ConstraintManager.solveVelocityConstraints(active[0..2], &idx, 1.0 / 60.0);
    try expect(a.getLinearVelocity().isNearZero(.{ .max_dist_sq = 1.0e-6 }));
    try expect(a.getAngularVelocity().isNearZero(.{ .max_dist_sq = 1.0e-6 }));
    const fixed = zolt.virtual.downcast(zolt.FixedConstraint, zolt.virtual.downcast(zolt.TwoBodyConstraint, ptrs[0]));
    const point = zolt.virtual.downcast(zolt.PointConstraint, zolt.virtual.downcast(zolt.TwoBodyConstraint, ptrs[1]));
    const fixed_lambda = fixed.getTotalLambdaPosition();
    const point_lambda = point.getTotalLambdaPosition();
    try expect(!fixed_lambda.isNearZero(.{}) and !point_lambda.isNearZero(.{}));

    // Count, then per constraint its index, enabled and the lambdas of its parts (fixed: rotation and point part)
    var state = StateRecorderImpl.init(allocator);
    defer state.deinit();
    try c.manager.saveState(state.recorder(), null);
    try expectEqual(@as(usize, 4 + (4 + 1 + 12 + 12) + (4 + 1 + 12) + (4 + 1 + 12)), state.getData().len);

    // Reset and restore
    for (ptrs) |p| p.resetWarmStart();
    try expect(fixed.getTotalLambdaPosition().isNearZero(.{}) and point.getTotalLambdaPosition().isNearZero(.{}));
    ptrs[2].setEnabled(true);
    try expect(c.manager.restoreState(state.recorder()));
    try expect(fixed.getTotalLambdaPosition().eql(fixed_lambda) and point.getTotalLambdaPosition().eql(point_lambda));
    try expect(!ptrs[2].getEnabled());

    // Validating restore of the current state
    state.rewind();
    state.recorder().setValidating(true);
    try expect(c.manager.restoreState(state.recorder()));
    try expectEqual(@as(u32, 0), state.num_mismatches);
}

test "ConstraintManager out of memory" {
    const Ctx = struct {
        fn run(allocator: Allocator) !void {
            var c: Context = undefined;
            try c.init(allocator);
            defer c.deinit();
            const a = try c.createBody(.dynamic, .activate);
            const b = try c.createBody(.dynamic, .activate);

            var r1 = try c.createConstraint(a, b);
            defer r1.deinit();
            var r2 = try c.createConstraint(a, b);
            defer r2.deinit();
            try c.manager.add(&.{r1.get().?});

            // A failing add adds nothing
            c.manager.add(&.{r2.get().?}) catch |err| {
                try expectEqual(Constraint.invalid_constraint_index, r2.get().?.constraint_index);
                try expectEqual(@as(u32, 1), c.manager.getNumConstraints());
                return err;
            };
            try expectEqual(@as(u32, 1), r2.get().?.constraint_index);

            var copy = try c.manager.getConstraints(allocator);
            defer {
                for (copy.items) |*r| r.deinit();
                copy.deinit(allocator);
            }

            // The filtered save allocates a temporary list (the recorder's own memory is not part of the sweep)
            const filter: StateRecorderFilter = .{};
            var state = StateRecorderImpl.init(std.testing.allocator);
            defer state.deinit();
            try c.manager.saveState(state.recorder(), &filter);
            try expect(!state.isFailed());
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.run, .{});
}

test "ConstraintManager add and remove from several threads" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator);
    defer c.deinit();
    const a = try c.createBody(.dynamic, .activate);
    const b = try c.createBody(.dynamic, .activate);

    const num_threads = 4;
    const per_thread = 50;
    var refs: [num_threads * per_thread]Ref(Constraint) = undefined;
    for (&refs) |*r| r.* = try c.createConstraint(a, b);
    defer for (&refs) |*r| r.deinit();

    const Worker = struct {
        fn run(manager: *ConstraintManager, mine: []Ref(Constraint), failures: *std.atomic.Value(u32)) void {
            // Add all, remove every other one, read the list in between
            for (mine) |r| manager.add(&.{r.get().?}) catch {
                _ = failures.fetchAdd(1, .monotonic);
            };
            var i: usize = 0;
            while (i < mine.len) : (i += 2) {
                manager.remove(&.{mine[i].get().?});
                var copy = manager.getConstraints(std.testing.allocator) catch {
                    _ = failures.fetchAdd(1, .monotonic);
                    continue;
                };
                for (copy.items) |*r| r.deinit();
                copy.deinit(std.testing.allocator);
            }
        }
    };
    var failures: std.atomic.Value(u32) = .init(0);
    var threads: [num_threads]std.Thread = undefined;
    for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Worker.run, .{ &c.manager, refs[i * per_thread ..][0..per_thread], &failures });
    for (threads) |t| t.join();
    try expectEqual(@as(u32, 0), failures.load(.monotonic));

    // Half of them remain and every index matches the position in the list
    try expectEqual(@as(u32, num_threads * per_thread / 2), c.manager.getNumConstraints());
    for (c.manager.constraints.items, 0..) |r, i|
        try expectEqual(@as(u32, @intCast(i)), r.get().?.constraint_index);
    for (refs, 0..) |r, i|
        try expectEqual(i % 2 == 1, r.get().?.constraint_index != Constraint.invalid_constraint_index);
}
