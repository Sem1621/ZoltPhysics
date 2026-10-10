//! Port of: Jolt/Physics/Collision/NarrowPhaseStats.h, Jolt/Physics/Collision/NarrowPhaseStats.cpp
//! Status: complete
//!
//! Narrow phase profiling counters. Jolt only compiles them with JPH_TRACK_NARROWPHASE_STATS (a developer option,
//! off by default); Zolt always declares the types (so they are type checked and tested) and only uses them when the
//! comptime switch `track_narrowphase_stats` is true (`JPH_IF_TRACK_NARROWPHASE_STATS(...)` becomes
//! `if (track_narrowphase_stats) ...`, see CollisionDispatch.zig). The switch is off, like Jolt's default.
//!
//! - The static tables `NarrowPhaseStat::sCollideShape` / `sCastShape` are `NarrowPhaseStat.collide_shape` /
//!   `cast_shape` (`pub var`, static members), the `thread_local` root of the chain is
//!   `TrackNarrowPhaseStat.root` (`threadlocal var`).
//! - The RAII trackers become `var track: TrackNarrowPhaseStat = undefined; track.init(&stat); defer track.deinit();`
//!   (in place: the root points to the tracker, so it must not be moved).
//! - `Trace` is `std.log` (info level) and the percentages are formatted by Zig (`{d}`), not by `std::stringstream`.

const std = @import("std");
const getProcessorTickCount = @import("../../Core/TickCounter.zig").getProcessorTickCount;
const ShapeFile = @import("Shape/Shape.zig");
const ShapeSubType = ShapeFile.ShapeSubType;
const all_sub_shape_types = ShapeFile.all_sub_shape_types;
const sub_shape_type_names = ShapeFile.sub_shape_type_names;
const num_sub_shape_types = ShapeFile.num_sub_shape_types;

const log = std.log.scoped(.zolt);

/// JPH_TRACK_NARROWPHASE_STATS: collect narrow phase timing information (developer option, off like Jolt's default)
pub const track_narrowphase_stats = false;

/// Structure that tracks narrow phase timing information for a particular combination of shapes
pub const NarrowPhaseStat = struct {
    num_queries: std.atomic.Value(u64) = .init(0),
    hits_reported: std.atomic.Value(u64) = .init(0),
    total_ticks: std.atomic.Value(u64) = .init(0),
    child_ticks: std.atomic.Value(u64) = .init(0),

    /// Stats of CollisionDispatch::sCollideShapeVsShape per pair of sub shape types (sCollideShape)
    pub var collide_shape: [num_sub_shape_types][num_sub_shape_types]NarrowPhaseStat = @splat(@splat(.{}));
    /// Stats of CollisionDispatch::sCastShapeVsShapeLocalSpace per pair of sub shape types (sCastShape)
    pub var cast_shape: [num_sub_shape_types][num_sub_shape_types]NarrowPhaseStat = @splat(@splat(.{}));

    /// Trace an individual stat in CSV form.
    pub fn reportStats(self: *const NarrowPhaseStat, name: []const u8, type1: ShapeSubType, type2: ShapeSubType, ticks_100_pct: u64) void {
        const total_ticks = self.total_ticks.load(.seq_cst);
        const child_ticks = self.child_ticks.load(.seq_cst);
        const num_queries = self.num_queries.load(.seq_cst);
        const total_pct = 100.0 * @as(f64, @floatFromInt(total_ticks)) / @as(f64, @floatFromInt(ticks_100_pct));
        const total_pct_excl_children = 100.0 * @as(f64, @floatFromInt(total_ticks -% child_ticks)) / @as(f64, @floatFromInt(ticks_100_pct));

        log.info("{s}, {s}, {s}, {d}, {d}, {d}, {d}, {d}", .{ name, sub_shape_type_names[@intFromEnum(type1)], sub_shape_type_names[@intFromEnum(type2)], num_queries, total_pct, total_pct_excl_children, total_pct_excl_children / @as(f64, @floatFromInt(num_queries)), self.hits_reported.load(.seq_cst) });
    }

    /// Trace the collected broadphase stats in CSV form.
    /// This report can be used to judge and tweak the efficiency of the broadphase.
    pub fn reportAllStats() void {
        log.info("Query Type, Shape Type 1, Shape Type 2, Num Queries, Total Time (%), Total Time Excl Children (%), Total Time Excl. Children / Query (%), Hits Reported", .{});

        var total_ticks: u64 = 0;
        for (all_sub_shape_types) |t1|
            for (all_sub_shape_types) |t2| {
                const collide_stat = &collide_shape[@intFromEnum(t1)][@intFromEnum(t2)];
                total_ticks +%= collide_stat.total_ticks.load(.seq_cst) -% collide_stat.child_ticks.load(.seq_cst);

                const cast_stat = &cast_shape[@intFromEnum(t1)][@intFromEnum(t2)];
                total_ticks +%= cast_stat.total_ticks.load(.seq_cst) -% cast_stat.child_ticks.load(.seq_cst);
            };

        for (all_sub_shape_types) |t1|
            for (all_sub_shape_types) |t2| {
                const stat = &collide_shape[@intFromEnum(t1)][@intFromEnum(t2)];
                if (stat.num_queries.load(.seq_cst) > 0)
                    stat.reportStats("CollideShape", t1, t2, total_ticks);
            };

        for (all_sub_shape_types) |t1|
            for (all_sub_shape_types) |t2| {
                const stat = &cast_shape[@intFromEnum(t1)][@intFromEnum(t2)];
                if (stat.num_queries.load(.seq_cst) > 0)
                    stat.reportStats("CastShape", t1, t2, total_ticks);
            };
    }
};

/// Object that tracks the start and end of a narrow phase operation
pub const TrackNarrowPhaseStat = struct {
    stat: *NarrowPhaseStat,
    parent: ?*TrackNarrowPhaseStat,
    start: u64,

    /// Root of the chain of the current thread (thread_local sRoot)
    pub threadlocal var root: ?*TrackNarrowPhaseStat = null;

    /// Constructor (in place: the tracker becomes the root of the chain and must not move until `deinit`)
    pub fn init(self: *TrackNarrowPhaseStat, stat: *NarrowPhaseStat) void {
        self.* = .{ .stat = stat, .parent = root, .start = getProcessorTickCount() };

        // Make this the new root of the chain
        root = self;
    }

    /// Destructor
    pub fn deinit(self: *TrackNarrowPhaseStat) void {
        const delta_ticks = getProcessorTickCount() -% self.start;

        // Notify parent of time spent in child
        if (self.parent) |parent|
            _ = parent.stat.child_ticks.fetchAdd(delta_ticks, .seq_cst);

        // Increment stats at this level
        _ = self.stat.num_queries.fetchAdd(1, .seq_cst);
        _ = self.stat.total_ticks.fetchAdd(delta_ticks, .seq_cst);

        // Restore root pointer
        std.debug.assert(root == self);
        root = self.parent;
    }
};

/// Object that tracks the start and end of a hit being processed by a collision collector
pub const TrackNarrowPhaseCollector = struct {
    start: u64,

    /// Constructor
    pub fn init() TrackNarrowPhaseCollector {
        return .{ .start = getProcessorTickCount() };
    }

    /// Destructor
    pub fn deinit(self: *TrackNarrowPhaseCollector) void {
        // Mark time spent in collector as 'child' time for the parent
        const delta_ticks = getProcessorTickCount() -% self.start;
        if (TrackNarrowPhaseStat.root) |r|
            _ = r.stat.child_ticks.fetchAdd(delta_ticks, .seq_cst);

        // Notify all parents of a hit
        var track = TrackNarrowPhaseStat.root;
        while (track) |t| : (track = t.parent)
            _ = t.stat.hits_reported.fetchAdd(1, .seq_cst);
    }
};

test "NarrowPhaseStats: trackers, nesting and hits" {
    const expectEqual = std.testing.expectEqual;

    // Jolt's default: off, the dispatch functions do not track anything
    try std.testing.expect(!track_narrowphase_stats);

    var outer_stat: NarrowPhaseStat = .{};
    var inner_stat: NarrowPhaseStat = .{};
    {
        var outer: TrackNarrowPhaseStat = undefined;
        outer.init(&outer_stat);
        defer outer.deinit();
        try std.testing.expect(TrackNarrowPhaseStat.root == &outer);
        {
            var inner: TrackNarrowPhaseStat = undefined;
            inner.init(&inner_stat);
            defer inner.deinit();
            try std.testing.expect(inner.parent == &outer);

            var hit = TrackNarrowPhaseCollector.init();
            hit.deinit();
        }
        try std.testing.expect(TrackNarrowPhaseStat.root == &outer);
    }
    try std.testing.expect(TrackNarrowPhaseStat.root == null);

    try expectEqual(@as(u64, 1), outer_stat.num_queries.load(.seq_cst));
    try expectEqual(@as(u64, 1), inner_stat.num_queries.load(.seq_cst));
    try expectEqual(@as(u64, 1), outer_stat.hits_reported.load(.seq_cst)); // The hit is reported to every parent
    try expectEqual(@as(u64, 1), inner_stat.hits_reported.load(.seq_cst));

    // Reporting (info level, not printed by the test runner)
    inner_stat.reportStats("CollideShape", .sphere, .box, 1000);
    NarrowPhaseStat.reportAllStats();
}
