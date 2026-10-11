//! Port of: UnitTests/Physics/CollisionGroupTests.cpp

const std = @import("std");
const zolt = @import("zolt");

const CollisionGroup = zolt.CollisionGroup;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const Ref = zolt.Ref;

const expect = std.testing.expect;

test "TestCollisionGroup1" {
    // Test group filter with no sub groups
    const group_filter = try GroupFilterTable.create(std.testing.allocator, .{});
    var group_filter_ref = Ref(GroupFilter).init(group_filter.asGroupFilterMut());
    defer group_filter_ref.deinit();

    // Check that doesn't collide with self
    var g1 = CollisionGroup.init(group_filter.asGroupFilter(), 0, 0);
    defer g1.deinit();
    try expect(!g1.canCollide(&g1));

    // Check that collides with other group
    var g2 = CollisionGroup.init(group_filter.asGroupFilter(), 1, 0);
    defer g2.deinit();
    try expect(g1.canCollide(&g2));
    try expect(g2.canCollide(&g1));
}

test "TestCollisionGroup2" {
    // Test group filter with no sub groups
    const group_filter1 = try GroupFilterTable.create(std.testing.allocator, .{ .num_sub_groups = 10 });
    var group_filter1_ref = Ref(GroupFilter).init(group_filter1.asGroupFilterMut());
    defer group_filter1_ref.deinit();
    const group_filter2 = try GroupFilterTable.create(std.testing.allocator, .{ .num_sub_groups = 10 });
    var group_filter2_ref = Ref(GroupFilter).init(group_filter2.asGroupFilterMut());
    defer group_filter2_ref.deinit();

    // Disable some pairs
    const SubGroupPair = struct { first: CollisionGroup.SubGroupID, second: CollisionGroup.SubGroupID };
    const pairs = [_]SubGroupPair{
        .{ .first = 1, .second = 2 },
        .{ .first = 9, .second = 5 },
        .{ .first = 3, .second = 7 },
        .{ .first = 6, .second = 1 },
        .{ .first = 8, .second = 1 },
    };
    for (pairs) |p| {
        group_filter1.disableCollision(p.first, p.second);
        group_filter2.disableCollision(p.first, p.second);
    }

    const find = struct {
        fn find(list: []const SubGroupPair, first: CollisionGroup.SubGroupID, second: CollisionGroup.SubGroupID) bool {
            for (list) |p|
                if (p.first == first and p.second == second) return true;
            return false;
        }
    }.find;

    var i: CollisionGroup.SubGroupID = 0;
    while (i < 10) : (i += 1) {
        var j: CollisionGroup.SubGroupID = 0;
        while (j < 10) : (j += 1) {
            // Check that doesn't collide with self
            var g1 = CollisionGroup.init(group_filter1.asGroupFilter(), 0, i);
            defer g1.deinit();
            try expect(!g1.canCollide(&g1));

            // Same filter, same group, check if pairs collide
            var g2 = CollisionGroup.init(group_filter1.asGroupFilter(), 0, j);
            defer g2.deinit();
            if (i == j or find(&pairs, i, j) or find(&pairs, j, i)) {
                try expect(!g1.canCollide(&g2));
                try expect(!g2.canCollide(&g1));
            } else {
                try expect(g1.canCollide(&g2));
                try expect(g2.canCollide(&g1));
            }

            // Using different group always collides
            var g3 = CollisionGroup.init(group_filter1.asGroupFilter(), 1, j);
            defer g3.deinit();
            try expect(g1.canCollide(&g3));
            try expect(g3.canCollide(&g1));

            // Using different filter with equal group should not collide
            var g4 = CollisionGroup.init(group_filter2.asGroupFilter(), 0, j);
            defer g4.deinit();
            try expect(!g1.canCollide(&g4));
            try expect(!g4.canCollide(&g1));

            // Using different filter with non-equal group should collide
            var g5 = CollisionGroup.init(group_filter2.asGroupFilter(), 1, j);
            defer g5.deinit();
            try expect(g1.canCollide(&g5));
            try expect(g5.canCollide(&g1));
        }
    }
}
