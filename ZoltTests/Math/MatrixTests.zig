//! Port of: UnitTests/Math/MatrixTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const expect = fw.expect;
const expectEqual = fw.expectEqual;

const Matrix = zolt.Matrix;

test "TestMatrixEquals" {
    const m1 = Matrix(3, 5).zero();
    const m2 = Matrix(3, 5).zero();
    const m3 = Matrix(3, 5).identity();

    try expect(m1.eql(m2));
    try expect(!!m1.eql(m2)); // !(m1 != m2)
    try expect(!m1.eql(m3)); // m1 != m3
    try expect(!m1.eql(m3));
}

test "TestMatrixStream" {
    const m1 = Matrix(3, 5).identity();

    var buf: [128]u8 = undefined;
    const str = try std.fmt.bufPrint(&buf, "{f}", .{m1});
    try std.testing.expectEqualStrings("[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0], [0, 0, 0]", str);
}

test "TestMatrixZero" {
    const m = Matrix(3, 5).zero();

    for (0..3) |r|
        for (0..5) |c|
            try expectEqual(0.0, m.get(@intCast(r), @intCast(c)));
}

test "TestMatrixIdentity" {
    const m = Matrix(3, 5).identity();

    for (0..3) |r|
        for (0..5) |c|
            try expectEqual(@as(f32, if (r == c) 1.0 else 0.0), m.get(@intCast(r), @intCast(c)));
}

test "TestMatrixMultiply" {
    var m1 = Matrix(3, 5).zero();
    var m2 = Matrix(5, 4).zero();

    for (0..3) |r|
        for (0..5) |c|
            m1.set(@intCast(r), @intCast(c), @floatFromInt(r * 5 + c + 1));

    for (0..5) |r|
        for (0..4) |c|
            m2.set(@intCast(r), @intCast(c), @floatFromInt(r * 4 + c + 1));

    const m3: Matrix(3, 4) = m1.mul(m2);

    try expectEqual(175.0, m3.get(0, 0));
    try expectEqual(400.0, m3.get(1, 0));
    try expectEqual(625.0, m3.get(2, 0));
    try expectEqual(190.0, m3.get(0, 1));
    try expectEqual(440.0, m3.get(1, 1));
    try expectEqual(690.0, m3.get(2, 1));
    try expectEqual(205.0, m3.get(0, 2));
    try expectEqual(480.0, m3.get(1, 2));
    try expectEqual(755.0, m3.get(2, 2));
    try expectEqual(220.0, m3.get(0, 3));
    try expectEqual(520.0, m3.get(1, 3));
    try expectEqual(820.0, m3.get(2, 3));
}

test "TestMatrixInversed" {
    var mat = Matrix(4, 4).zero();
    mat.set(1, 0, 4);
    mat.set(3, 0, 8);
    mat.set(0, 1, 2);
    mat.set(2, 1, 16);
    mat.set(1, 2, 16);
    mat.set(3, 2, 4);
    mat.set(0, 3, 8);
    mat.set(2, 3, 2);
    var inverse: Matrix(4, 4) = undefined;
    try expect(inverse.setInversed(mat));
    const identity = mat.mul(inverse);
    try expect(identity.eql(Matrix(4, 4).identity()));

    // Make non-invertible
    mat.set(1, 0, 0);
    mat.set(3, 0, 0);
    try expect(!inverse.setInversed(mat));
}

test "TestMatrix22Inversed" {
    // SetInverse is specialized for 2x2 matrices
    var mat: Matrix(2, 2) = undefined;
    mat.set(0, 0, 1);
    mat.set(0, 1, 2);
    mat.set(1, 0, 3);
    mat.set(1, 1, 4);
    var inverse: Matrix(2, 2) = undefined;
    try expect(inverse.setInversed(mat));
    const identity = mat.mul(inverse);
    try expect(identity.eql(Matrix(2, 2).identity()));

    // Make non-invertible
    mat.set(0, 0, 0);
    mat.set(1, 0, 0);
    try expect(!inverse.setInversed(mat));
}
