//! Zolt addition, no Jolt file: tests of IslandBuilder (SimulationArchitecture D4, D5)
//! Status: complete
//!
//! Jolt has no unit test for the IslandBuilder. These tests check the islands of a known set of links (bodies grouped
//! by island in active body order, constraints and contacts per island, islands sorted by the number of constraints),
//! the reset of the links for the next update, every temp allocation failing in turn (the temp allocator is empty
//! after resetIslands and the links are restored), and the lock-free linkBodies from several threads against a
//! single threaded reference.

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const BodyID = zolt.BodyID;
const IslandBuilder = zolt.IslandBuilder;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const FailingTempAllocator = zolt.FailingTempAllocator;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualSlices = std.testing.expectEqualSlices;

/// The BodyID of the body at index `i` of the active bodies (different from the index, to tell them apart)
fn bodyID(i: u32) BodyID {
    return .init(100 + i);
}

fn activeBodies(comptime n: u32) [n]BodyID {
    var ids: [n]BodyID = undefined;
    for (&ids, 0..) |*id, i| id.* = bodyID(@intCast(i));
    return ids;
}

/// One update with the links of the test: 7 active bodies, A = {0, 1, 3, 4} linked, 2, 5 and 6 alone, a link with a
/// body that is not active (ignored); constraints 0 and 2 and contact 0 on body 6, constraint 1 on body 5, contact 1
/// on body 2
fn buildTestIslands(builder: *IslandBuilder, temp_allocator: TempAllocator) TempAllocator.Error!void {
    try builder.prepareContactConstraints(4, temp_allocator);
    try builder.prepareNonContactConstraints(3, temp_allocator);
    builder.linkBodies(0, 1);
    builder.linkBodies(3, 4);
    builder.linkBodies(4, 1);
    builder.linkBodies(2, zolt.Body.inactive_index); // A static or inactive body
    builder.linkConstraint(0, 6);
    builder.linkConstraint(1, 5);
    builder.linkConstraint(2, 6);
    builder.linkContact(0, 6);
    builder.linkContact(1, 2);
    const active = activeBodies(7);
    try builder.finalize(&active, 2, temp_allocator);
}

test "IslandBuilder islands, constraints, contacts and the sort" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 64 * 1024);
    defer temp.deinit();
    var builder: IslandBuilder = .empty;
    try builder.init(allocator, 16);
    defer builder.deinit();

    try buildTestIslands(&builder, temp.tempAllocator());
    try expectEqual(@as(u32, 4), builder.getNumIslands());

    // Islands by the number of constraints and contacts: {6} 3, {2} 1, {5} 1, A 0 (ties keep their order)
    try expectEqualSlices(BodyID, &.{bodyID(6)}, builder.getBodiesInIsland(0));
    try expectEqualSlices(u32, &.{ 0, 2 }, builder.getConstraintsInIsland(0));
    try expectEqualSlices(u32, &.{0}, builder.getContactsInIsland(0));

    try expectEqualSlices(BodyID, &.{bodyID(2)}, builder.getBodiesInIsland(1));
    try expectEqual(@as(usize, 0), builder.getConstraintsInIsland(1).len);
    try expectEqualSlices(u32, &.{1}, builder.getContactsInIsland(1));

    try expectEqualSlices(BodyID, &.{bodyID(5)}, builder.getBodiesInIsland(2));
    try expectEqualSlices(u32, &.{1}, builder.getConstraintsInIsland(2));
    try expectEqual(@as(usize, 0), builder.getContactsInIsland(2).len);

    try expectEqualSlices(BodyID, &.{ bodyID(0), bodyID(1), bodyID(3), bodyID(4) }, builder.getBodiesInIsland(3));
    try expectEqual(@as(usize, 0), builder.getConstraintsInIsland(3).len);
    try expectEqual(@as(usize, 0), builder.getContactsInIsland(3).len);

    // Position steps per island
    for (0..4) |i| builder.setNumPositionSteps(@intCast(i), @intCast(i + 1));
    for (0..4) |i| try expectEqual(@as(u32, @intCast(i + 1)), builder.getNumPositionSteps(@intCast(i)));

    builder.resetIslands(temp.tempAllocator());
    try expect(temp.isEmpty());

    // The links were reset: without links every body is its own island, in active body order (nothing to sort)
    try builder.prepareContactConstraints(4, temp.tempAllocator());
    try builder.prepareNonContactConstraints(0, temp.tempAllocator());
    const active = activeBodies(7);
    try builder.finalize(&active, 0, temp.tempAllocator());
    try expectEqual(@as(u32, 7), builder.getNumIslands());
    for (0..7) |i| {
        try expectEqualSlices(BodyID, &.{bodyID(@intCast(i))}, builder.getBodiesInIsland(@intCast(i)));
        try expectEqual(@as(usize, 0), builder.getConstraintsInIsland(@intCast(i)).len);
    }
    builder.resetIslands(temp.tempAllocator());
    try expect(temp.isEmpty());

    // No active bodies at all
    try builder.prepareContactConstraints(0, temp.tempAllocator());
    try builder.prepareNonContactConstraints(0, temp.tempAllocator());
    try builder.finalize(&.{}, 0, temp.tempAllocator());
    try expectEqual(@as(u32, 0), builder.getNumIslands());
    builder.resetIslands(temp.tempAllocator());
    try expect(temp.isEmpty());
}

test "IslandBuilder out of temp memory at every allocation" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 64 * 1024);
    defer temp.deinit();
    var builder: IslandBuilder = .empty;
    try builder.init(allocator, 16);
    defer builder.deinit();

    // Count the allocations of one update
    var counting: FailingTempAllocator = .init(temp.tempAllocator(), .{});
    try buildTestIslands(&builder, counting.tempAllocator());
    builder.resetIslands(counting.tempAllocator());
    const num_allocations = counting.num_allocations;
    try expect(num_allocations >= 10);

    for (0..num_allocations) |fail_index| {
        var failing: FailingTempAllocator = .init(temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
        try std.testing.expectError(error.OutOfMemory, buildTestIslands(&builder, failing.tempAllocator()));
        builder.resetIslands(failing.tempAllocator());
        try expect(temp.isEmpty());

        // The links were restored: an update without links gives one island per body
        try builder.prepareContactConstraints(0, temp.tempAllocator());
        try builder.prepareNonContactConstraints(0, temp.tempAllocator());
        const active = activeBodies(7);
        try builder.finalize(&active, 0, temp.tempAllocator());
        try expectEqual(@as(u32, 7), builder.getNumIslands());
        builder.resetIslands(temp.tempAllocator());

        // And the full update works again
        try buildTestIslands(&builder, temp.tempAllocator());
        try expectEqual(@as(u32, 4), builder.getNumIslands());
        builder.resetIslands(temp.tempAllocator());
        try expect(temp.isEmpty());
    }
}

test "IslandBuilder out of memory in init" {
    const Ctx = struct {
        fn run(allocator: Allocator) !void {
            var builder: IslandBuilder = .empty;
            try builder.init(allocator, 1024);
            builder.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.run, .{});
}

test "IslandBuilder linkBodies from several threads" {
    const allocator = std.testing.allocator;
    const num_bodies = 2000;
    const num_links = 1500;

    // Random links (fewer links than bodies: many islands of different sizes)
    var prng: std.Random.DefaultPrng = .init(0x5eed);
    const random = prng.random();
    var links: [num_links][2]u32 = undefined;
    for (&links) |*l| l.* = .{ random.uintLessThan(u32, num_bodies), random.uintLessThan(u32, num_bodies) };

    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    const active = try allocator.alloc(BodyID, num_bodies);
    defer allocator.free(active);
    for (active, 0..) |*id, i| id.* = bodyID(@intCast(i));

    // Reference: single threaded
    var reference: IslandBuilder = .empty;
    try reference.init(allocator, num_bodies);
    defer reference.deinit();
    try reference.prepareContactConstraints(0, temp.tempAllocator());
    try reference.prepareNonContactConstraints(0, temp.tempAllocator());
    for (links) |l| reference.linkBodies(l[0], l[1]);
    try reference.finalize(active, 0, temp.tempAllocator());

    // The same links from 4 threads, several times
    for (0..4) |_| {
        var temp2: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
        defer temp2.deinit();
        var builder: IslandBuilder = .empty;
        try builder.init(allocator, num_bodies);
        defer builder.deinit();
        try builder.prepareContactConstraints(0, temp2.tempAllocator());
        try builder.prepareNonContactConstraints(0, temp2.tempAllocator());

        const Worker = struct {
            fn run(b: *IslandBuilder, my_links: []const [2]u32) void {
                // Each pair reversed: the islands do not depend on the direction of a link
                for (my_links) |l| b.linkBodies(l[1], l[0]);
            }
        };
        const num_threads = 4;
        var threads: [num_threads]std.Thread = undefined;
        const per_thread = num_links / num_threads;
        for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Worker.run, .{ &builder, links[i * per_thread ..][0..per_thread] });
        for (threads) |t| t.join();
        try builder.finalize(active, 0, temp2.tempAllocator());

        // Identical islands (the components do not depend on the order of the links)
        try expectEqual(reference.getNumIslands(), builder.getNumIslands());
        var i: u32 = 0;
        while (i < builder.getNumIslands()) : (i += 1)
            try expectEqualSlices(BodyID, reference.getBodiesInIsland(i), builder.getBodiesInIsland(i));

        builder.resetIslands(temp2.tempAllocator());
        try expect(temp2.isEmpty());
    }

    reference.resetIslands(temp.tempAllocator());
    try expect(temp.isEmpty());
}
