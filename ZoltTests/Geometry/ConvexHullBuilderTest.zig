//! Port of: UnitTests/Geometry/ConvexHullBuilderTest.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const math = zolt.math;
const trigonometry = zolt.trigonometry;
const ConvexHullBuilder = zolt.ConvexHullBuilder;
const Vec3 = zolt.Vec3;

const tolerance: f32 = 1.0e-3;
const Positions = ConvexHullBuilder.Positions;
const no_limit = std.math.maxInt(i32); // INT_MAX

test "TestDegenerate" {
    const allocator = std.testing.allocator;

    {
        // Too few points / coinciding points should be degenerate
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        try positions.append(allocator, Vec3.init(1, 2, 3));

        // Jolt's builder keeps a reference to the array, Zolt's a slice: point it at the array again after it grew
        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.too_few_points, (try builder.initialize(no_limit, tolerance)).result);
        try positions.append(allocator, Vec3.init(1 + 0.5 * tolerance, 2, 3));
        builder.positions = positions.items;
        try expectEqual(ConvexHullBuilder.Result.too_few_points, (try builder.initialize(no_limit, tolerance)).result);
        try positions.append(allocator, Vec3.init(1, 2 + 0.5 * tolerance, 3));
        builder.positions = positions.items;
        try expectEqual(ConvexHullBuilder.Result.degenerate, (try builder.initialize(no_limit, tolerance)).result);
        try positions.append(allocator, Vec3.init(1, 2, 3 + 0.5 * tolerance));
        builder.positions = positions.items;
        try expectEqual(ConvexHullBuilder.Result.degenerate, (try builder.initialize(no_limit, tolerance)).result);
    }

    {
        // A line should be degenerate
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        var v: f32 = 0.0;
        while (v < 1.01) : (v += 0.1)
            try positions.append(allocator, Vec3.init(v, 0, 0));

        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.degenerate, (try builder.initialize(no_limit, tolerance)).result);
    }
}

test "Test2DHull" {
    const allocator = std.testing.allocator;

    {
        // A triangle
        const positions = [_]Vec3{ Vec3.init(-1, 0, -1), Vec3.init(1, 0, -1), Vec3.init(-1, 0, 1) };

        var builder = ConvexHullBuilder.init(allocator, &positions);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);
        try expectEqual(3, try builder.getNumVerticesUsed());
        try expectEqual(2, builder.getFaces().len);
        try expect(builder.containsFace(&.{ 0, 1, 2 }));
        try expect(builder.containsFace(&.{ 2, 1, 0 }));
    }

    {
        // A quad with many interior points
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        for (0..10) |x| {
            for (0..10) |z| {
                const fx: f32 = @floatFromInt(x);
                const fz: f32 = @floatFromInt(z);
                const one: f32 = 1.0;
                try positions.append(allocator, Vec3.init(0.1 * fx, 0, one * 0.2 * fz));
            }
        }

        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);
        try expectEqual(4, try builder.getNumVerticesUsed());
        try expectEqual(2, builder.getFaces().len);
        try expect(builder.containsFace(&.{ 0, 9, 99, 90 }));
        try expect(builder.containsFace(&.{ 90, 99, 9, 0 }));
    }

    {
        // Add disc with many interior points
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        for (0..10) |r| {
            for (0..10) |phi| {
                const f_r = 2.0 * @as(f32, @floatFromInt(r));
                const f_phi = 2.0 * math.pi * @as(f32, @floatFromInt(phi)) / 10;
                try positions.append(allocator, Vec3.init(f_r * trigonometry.cos(f_phi), f_r * trigonometry.sin(f_phi), 0));
            }
        }

        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);
        try expectEqual(10, try builder.getNumVerticesUsed());
        try expectEqual(2, builder.getFaces().len);
        try expect(builder.containsFace(&.{ 90, 91, 92, 93, 94, 95, 96, 97, 98, 99 }));
        try expect(builder.containsFace(&.{ 99, 98, 97, 96, 95, 94, 93, 92, 91, 90 }));
    }
}

test "Test3DHull" {
    const allocator = std.testing.allocator;

    {
        // A cube with lots of interior points
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        for (0..10) |x| {
            for (0..10) |y| {
                for (0..10) |z| {
                    const fx: f32 = @floatFromInt(x);
                    const fy: f32 = @floatFromInt(y);
                    const fz: f32 = @floatFromInt(z);
                    const two: f32 = 2.0;
                    try positions.append(allocator, Vec3.init(0.1 * fx, 1.0 + 0.2 * fy, two * 0.3 * fz));
                }
            }
        }

        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);
        try expectEqual(8, try builder.getNumVerticesUsed());
        try expectEqual(6, builder.getFaces().len);
        try expect(builder.containsFace(&.{ 0, 9, 99, 90 }));
        try expect(builder.containsFace(&.{ 0, 90, 990, 900 }));
        try expect(builder.containsFace(&.{ 900, 990, 999, 909 }));
        try expect(builder.containsFace(&.{ 9, 909, 999, 99 }));
        try expect(builder.containsFace(&.{ 90, 99, 999, 990 }));
        try expect(builder.containsFace(&.{ 0, 900, 909, 9 }));
    }

    {
        // Add sphere with many interior points
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        for (0..10) |r| {
            for (0..10) |phi| {
                for (0..10) |theta| {
                    const f_r = 2.0 * @as(f32, @floatFromInt(r));
                    const f_phi = 2.0 * math.pi * @as(f32, @floatFromInt(phi)) / 10; // [0, 2 PI)
                    const f_theta = math.pi * @as(f32, @floatFromInt(theta)) / 9; // [0, PI] (inclusive!)
                    try positions.append(allocator, Vec3.unitSpherical(f_theta, f_phi).mulScalar(f_r));
                }
            }
        }

        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);
        try expectEqual(82, try builder.getNumVerticesUsed()); // The two ends of the sphere have 10 points that have the same position

        // Too many faces, calculate the error instead
        const max_error = builder.determineMaxError();
        try expect(max_error.max_error < math.max(max_error.coplanar_distance, tolerance));
    }
}

test "TestRandomHull" {
    const allocator = std.testing.allocator;

    var random = fw.UnitTestRandom.init(0x1ee7c0de);

    const zero_one = fw.UniformFloatDistribution.init(0.0, 1.0);
    const zero_two = fw.UniformFloatDistribution.init(0.0, 2.0);
    const scale_start = fw.UniformFloatDistribution.init(0.1, 0.5);
    const scale_range = fw.UniformFloatDistribution.init(0.1, 2.0);

    // (C++ leaves the evaluation order of constructor / function arguments and of operator operands unspecified,
    // Zolt draws them left to right)
    for (0..100) |_| {
        // Define vertex scale
        const start = scale_start.next(&random);
        const vertex_scale = fw.UniformFloatDistribution.init(start, start + scale_range.next(&random));

        // Define shape scale to make shape less sphere like
        const shape_scale = fw.UniformFloatDistribution.init(0.1, 1.0);
        const scale_x = shape_scale.next(&random);
        const scale_y = shape_scale.next(&random);
        const scale_z = shape_scale.next(&random);
        const scale = Vec3.init(scale_x, scale_y, scale_z);

        // Add some random points
        var positions: Positions = .empty;
        defer positions.deinit(allocator);
        for (0..100) |_| {
            // Add random point
            const p1_scale = vertex_scale.next(&random);
            const p1 = Vec3.random(&random).mulScalar(p1_scale).mul(scale);
            try positions.append(allocator, p1);

            // Point close to p1
            const p2_scale = tolerance * zero_two.next(&random);
            const p2 = p1.add(Vec3.random(&random).mulScalar(p2_scale));
            try positions.append(allocator, p2);

            // Point on a line to another point
            const fraction = zero_one.next(&random);
            const other = positions.items[random.next() % positions.items.len];
            const p3 = p1.mulScalar(fraction).add(other.mulScalar(1.0 - fraction));
            try positions.append(allocator, p3);

            // Point close to p3
            const p4_scale = tolerance * zero_two.next(&random);
            const p4 = p3.add(Vec3.random(&random).mulScalar(p4_scale));
            try positions.append(allocator, p4);
        }

        // Build hull
        var builder = ConvexHullBuilder.init(allocator, positions.items);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);

        // Calculate error
        const max_error = builder.determineMaxError();
        try expect(max_error.max_error < math.max(max_error.coplanar_distance, 1.2 * tolerance));
    }
}

test "TestHullEdgeCases" {
    const allocator = std.testing.allocator;

    const positions = [_][]const Vec3{
        &.{
            // A hull with 2 faces that are nearly coplanar
            Vec3.init(-0.020472288, -0.195635557, 0.308015466),
            Vec3.init(0.136248738, 0.633286834, 0.135366619),
            Vec3.init(0.286418647, -0.228475571, 0.308084548),
            Vec3.init(-0.267285109, 1.024676085, 0.308042824),
            Vec3.init(0.396568149, -0.971658647, 0.308055162),
            Vec3.init(0.321081549, -1.024676085, 0.308036327),
            Vec3.init(0.034643859, -0.404506862, 0.308015764),
            Vec3.init(0.189224690, -0.252762139, 0.308060408),
        },
        &.{
            // Nearly coplanar points
            Vec3.init(0.917345762, 0.157111734, 1.650970459),
            Vec3.init(-0.098074198, 0.157116055, 0.664742708),
            Vec3.init(1.777100325, 0.157112047, 1.238879442),
            Vec3.init(2.114324570, 0.157112464, 0.780688763),
            Vec3.init(1.926570415, 0.157114446, 0.240761161),
            Vec3.init(-1.045998096, 0.157108605, 1.548911095),
            Vec3.init(-1.820045233, 0.157106474, 1.050360918),
            Vec3.init(-1.918573976, 0.157108605, 0.039246202),
            Vec3.init(0.042619467, 0.157113969, -1.405336142),
            Vec3.init(0.575986624, 0.157114401, -1.370834589),
            Vec3.init(1.402592659, 0.157115221, -0.834864557),
            Vec3.init(1.110557318, 0.157113969, -1.336267948),
            Vec3.init(1.689781666, 0.157115355, -0.308773756),
            Vec3.init(2.205337524, 0.157113209, -0.281754494),
            Vec3.init(-1.346967936, 0.157110974, -0.978962541),
            Vec3.init(-1.346967936, 0.157110974, -0.978962541),
            Vec3.init(-2.085033417, 0.157106936, -0.506602883),
            Vec3.init(-0.981224537, 0.157110706, -1.445893764),
            Vec3.init(-0.481085658, 0.157112658, -1.426232934),
            Vec3.init(-0.981224537, 0.157110706, -1.445893764),
        },
    };

    for (positions) |p| {
        // Build hull
        var builder = ConvexHullBuilder.init(allocator, p);
        defer builder.deinit();
        try expectEqual(ConvexHullBuilder.Result.success, (try builder.initialize(no_limit, tolerance)).result);

        // Calculate error
        const max_error = builder.determineMaxError();
        try expect(max_error.max_error < math.max(max_error.coplanar_distance, 1.2 * tolerance));
    }
}
