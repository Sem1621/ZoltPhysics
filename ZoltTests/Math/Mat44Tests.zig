//! Port of: UnitTests/Math/Mat44Tests.cpp
//! Status: complete
//!
//! Mat44Tests.cpp has two test cases named TestMat44Scale and two named TestMat44Decompose. Zig does not
//! allow duplicate test names, so the second ones are called TestMat44Scale2 and TestMat44Decompose2
//! (`-Dtest-filter=TestMat44Scale` still runs both).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const checkApproxEqual = fw.checkApproxEqual;
const math = zolt.math;
const Float4 = zolt.Float4;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

test "TestMat44Zero" {
    const zero = Mat44.zero();

    for (0..4) |row| {
        for (0..4) |col|
            try expectEqual(@as(f32, 0.0), zero.get(@intCast(row), @intCast(col)));
    }
}

test "TestMat44Column" {
    var mat = Mat44.zero();
    mat.setColumn4(0, Vec4.init(1, 2, 3, 4));
    try expect(mat.getColumn4(0).eql(Vec4.init(1, 2, 3, 4)));
    mat.setColumn3(0, Vec3.init(5, 6, 7));
    try expect(mat.getColumn3(0).eql(Vec3.init(5, 6, 7)));
    try expect(mat.getColumn4(0).eql(Vec4.init(5, 6, 7, 0)));

    mat.setAxisX(Vec3.init(8, 9, 10));
    mat.setAxisY(Vec3.init(11, 12, 13));
    mat.setAxisZ(Vec3.init(14, 15, 16));
    mat.setTranslation(Vec3.init(17, 18, 19));
    try expect(mat.getAxisX().eql(Vec3.init(8, 9, 10)));
    try expect(mat.getAxisY().eql(Vec3.init(11, 12, 13)));
    try expect(mat.getAxisZ().eql(Vec3.init(14, 15, 16)));
    try expect(mat.getTranslation().eql(Vec3.init(17, 18, 19)));

    mat.setDiagonal3(Vec3.init(20, 21, 22));
    try expect(mat.getDiagonal3().eql(Vec3.init(20, 21, 22)));
    try expect(mat.getAxisX().eql(Vec3.init(20, 9, 10)));
    try expect(mat.getAxisY().eql(Vec3.init(11, 21, 13)));
    try expect(mat.getAxisZ().eql(Vec3.init(14, 15, 22)));

    mat.setDiagonal4(Vec4.init(23, 24, 25, 26));
    try expect(mat.getDiagonal4().eql(Vec4.init(23, 24, 25, 26)));
    try expect(mat.getAxisX().eql(Vec3.init(23, 9, 10)));
    try expect(mat.getAxisY().eql(Vec3.init(11, 24, 13)));
    try expect(mat.getAxisZ().eql(Vec3.init(14, 15, 25)));
    try expect(mat.getColumn4(3).eql(Vec4.init(17, 18, 19, 26)));
}

test "TestMat44NaN" {
    const nan = Mat44.nan();

    for (0..4) |row| {
        for (0..4) |col|
            try expect(std.math.isNan(nan.get(@intCast(row), @intCast(col))));
    }
}

test "TestMat44Identity" {
    const identity = Mat44.identity();

    for (0..4) |row| {
        for (0..4) |col| {
            if (row != col)
                try expectEqual(@as(f32, 0.0), identity.get(@intCast(row), @intCast(col)))
            else
                try expectEqual(@as(f32, 1.0), identity.get(@intCast(row), @intCast(col)));
        }
    }
}

test "TestMat44Construct" {
    var mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));

    try expectEqual(@as(f32, 1.0), mat.get(0, 0));
    try expectEqual(@as(f32, 2.0), mat.get(1, 0));
    try expectEqual(@as(f32, 3.0), mat.get(2, 0));
    try expectEqual(@as(f32, 4.0), mat.get(3, 0));

    try expectEqual(@as(f32, 5.0), mat.get(0, 1));
    try expectEqual(@as(f32, 6.0), mat.get(1, 1));
    try expectEqual(@as(f32, 7.0), mat.get(2, 1));
    try expectEqual(@as(f32, 8.0), mat.get(3, 1));

    try expectEqual(@as(f32, 9.0), mat.get(0, 2));
    try expectEqual(@as(f32, 10.0), mat.get(1, 2));
    try expectEqual(@as(f32, 11.0), mat.get(2, 2));
    try expectEqual(@as(f32, 12.0), mat.get(3, 2));

    try expectEqual(@as(f32, 13.0), mat.get(0, 3));
    try expectEqual(@as(f32, 14.0), mat.get(1, 3));
    try expectEqual(@as(f32, 15.0), mat.get(2, 3));
    try expectEqual(@as(f32, 16.0), mat.get(3, 3));

    const mat2 = mat;

    try expectEqual(@as(f32, 1.0), mat2.get(0, 0));
    try expectEqual(@as(f32, 2.0), mat2.get(1, 0));
    try expectEqual(@as(f32, 3.0), mat2.get(2, 0));
    try expectEqual(@as(f32, 4.0), mat2.get(3, 0));

    try expectEqual(@as(f32, 5.0), mat2.get(0, 1));
    try expectEqual(@as(f32, 6.0), mat2.get(1, 1));
    try expectEqual(@as(f32, 7.0), mat2.get(2, 1));
    try expectEqual(@as(f32, 8.0), mat2.get(3, 1));

    try expectEqual(@as(f32, 9.0), mat2.get(0, 2));
    try expectEqual(@as(f32, 10.0), mat2.get(1, 2));
    try expectEqual(@as(f32, 11.0), mat2.get(2, 2));
    try expectEqual(@as(f32, 12.0), mat2.get(3, 2));

    try expectEqual(@as(f32, 13.0), mat2.get(0, 3));
    try expectEqual(@as(f32, 14.0), mat2.get(1, 3));
    try expectEqual(@as(f32, 15.0), mat2.get(2, 3));
    try expectEqual(@as(f32, 16.0), mat2.get(3, 3));

    // Check equal
    try expect(mat.eql(mat2));
    try expect(!(!mat.eql(mat2)));

    // Make unequal
    mat.set(3, 3, 1);

    // Check non-equal
    try expect(!mat.eql(mat2));
    try expect(!mat.eql(mat2));
}

test "TestMat44IsClose" {
    const mat = Mat44.identity();
    var mat2 = mat;

    try expect(mat.isClose(mat2, .{ .max_dist_sq = math.square(@as(f32, 0.1)) }));

    mat2.set(0, 1, 0.09);
    try expect(mat.isClose(mat2, .{ .max_dist_sq = math.square(@as(f32, 0.1)) }));

    mat2.set(0, 1, 0.11);
    try expect(!mat.isClose(mat2, .{ .max_dist_sq = math.square(@as(f32, 0.1)) }));
}

test "TestMat44Translation" {
    try expect(Mat44.translation(Vec3.init(2, 3, 4)).eql(Mat44.init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), Vec4.init(2, 3, 4, 1))));
}

test "TestMat44Scale" {
    try expect(Mat44.scale(2).eql(Mat44.init(Vec4.init(2, 0, 0, 0), Vec4.init(0, 2, 0, 0), Vec4.init(0, 0, 2, 0), Vec4.init(0, 0, 0, 1))));
    try expect(Mat44.scaleVec3(Vec3.init(2, 3, 4)).eql(Mat44.init(Vec4.init(2, 0, 0, 0), Vec4.init(0, 3, 0, 0), Vec4.init(0, 0, 4, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44Rotation" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(5, 6, 7, 0), Vec4.init(9, 10, 11, 0), Vec4.init(13, 14, 15, 16));
    try expect(mat.getRotation().eql(Mat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(5, 6, 7, 0), Vec4.init(9, 10, 11, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44SetRotation" {
    var mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const mat2 = Mat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), Vec4.init(29, 30, 31, 32));

    mat.setRotation(mat2);
    try expect(mat.eql(Mat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), Vec4.init(13, 14, 15, 16))));
}

test "TestMat44RotationSafe" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    try expect(mat.getRotationSafe().eql(Mat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(5, 6, 7, 0), Vec4.init(9, 10, 11, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44LoadStore" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));

    var storage: [4]Float4 = undefined;
    mat.storeFloat4x4(&storage);

    try expectEqual(@as(f32, 1.0), storage[0].x);
    try expectEqual(@as(f32, 2.0), storage[0].y);
    try expectEqual(@as(f32, 3.0), storage[0].z);
    try expectEqual(@as(f32, 4.0), storage[0].w);

    try expectEqual(@as(f32, 5.0), storage[1].x);
    try expectEqual(@as(f32, 6.0), storage[1].y);
    try expectEqual(@as(f32, 7.0), storage[1].z);
    try expectEqual(@as(f32, 8.0), storage[1].w);

    try expectEqual(@as(f32, 9.0), storage[2].x);
    try expectEqual(@as(f32, 10.0), storage[2].y);
    try expectEqual(@as(f32, 11.0), storage[2].z);
    try expectEqual(@as(f32, 12.0), storage[2].w);

    try expectEqual(@as(f32, 13.0), storage[3].x);
    try expectEqual(@as(f32, 14.0), storage[3].y);
    try expectEqual(@as(f32, 15.0), storage[3].z);
    try expectEqual(@as(f32, 16.0), storage[3].w);

    const mat2 = Mat44.loadFloat4x4(&storage);
    try expect(mat2.eql(mat));
}

test "TestMat44LoadAligned" {
    const values: [16]f32 align(16) = .{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    const mat = Mat44.loadFloat4x4Aligned(@ptrCast(&values));
    try expect(mat.eql(Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16))));
}

test "TestMat44MultiplyMat44" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const mat2 = Mat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), Vec4.init(29, 30, 31, 32));

    const result = mat.mul(mat2);
    try expect(result.eql(Mat44.init(Vec4.init(538, 612, 686, 760), Vec4.init(650, 740, 830, 920), Vec4.init(762, 868, 974, 1080), Vec4.init(874, 996, 1118, 1240))));
}

test "TestMat44Add" {
    var mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const mat2 = Mat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), Vec4.init(29, 30, 31, 32));

    const result = mat.add(mat2);
    try expect(result.eql(Mat44.init(Vec4.init(18, 20, 22, 24), Vec4.init(26, 28, 30, 32), Vec4.init(34, 36, 38, 40), Vec4.init(42, 44, 46, 48))));

    mat = mat.add(mat2);
    try expect(mat.eql(Mat44.init(Vec4.init(18, 20, 22, 24), Vec4.init(26, 28, 30, 32), Vec4.init(34, 36, 38, 40), Vec4.init(42, 44, 46, 48))));
}

test "TestMat44Sub" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const mat2 = Mat44.init(Vec4.init(32, 31, 30, 29), Vec4.init(28, 27, 26, 25), Vec4.init(24, 23, 22, 21), Vec4.init(20, 19, 18, 17));

    const result = mat.sub(mat2);
    try expect(result.eql(Mat44.init(Vec4.init(-31, -29, -27, -25), Vec4.init(-23, -21, -19, -17), Vec4.init(-15, -13, -11, -9), Vec4.init(-7, -5, -3, -1))));
}

test "TestMat44Negate" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));

    const result = mat.negate();
    try expect(result.eql(Mat44.init(Vec4.init(-1, -2, -3, -4), Vec4.init(-5, -6, -7, -8), Vec4.init(-9, -10, -11, -12), Vec4.init(-13, -14, -15, -16))));
}

test "TestMat44MultiplyVec3" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const vec = Vec3.init(17, 18, 19);

    var result = mat.mulVec3(vec);
    try expect(result.eql(Vec3.init(291, 346, 401)));

    result = mat.multiply3x3(vec);
    try expect(result.eql(Vec3.init(278, 332, 386)));

    result = mat.multiply3x3Transposed(vec);
    try expect(result.eql(Vec3.init(110, 326, 542)));
}

test "TestMat44MultiplyVec4" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const vec = Vec4.init(17, 18, 19, 20);

    const result = mat.mulVec4(vec);
    try expect(result.eql(Vec4.init(538, 612, 686, 760)));
}

// Second TEST_CASE("TestMat44Scale") in Mat44Tests.cpp, renamed because Zig doesn't allow duplicate test names
test "TestMat44Scale2" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    var result = mat.mulScalar(2.0);
    try expect(result.eql(Mat44.init(Vec4.init(2, 4, 6, 8), Vec4.init(10, 12, 14, 16), Vec4.init(18, 20, 22, 24), Vec4.init(26, 28, 30, 32))));
    try expect(!result.eql(mat));
    result = result.mulScalar(0.5);
    try expect(result.eql(mat));
}

test "TestMat44Transposed" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const result = mat.transposed();
    try expect(result.eql(Mat44.init(Vec4.init(1, 5, 9, 13), Vec4.init(2, 6, 10, 14), Vec4.init(3, 7, 11, 15), Vec4.init(4, 8, 12, 16))));
}

test "TestMat44Transposed3x3" {
    const mat = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const result = mat.transposed3x3();
    try expect(result.eql(Mat44.init(Vec4.init(1, 5, 9, 0), Vec4.init(2, 6, 10, 0), Vec4.init(3, 7, 11, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44Multiply3x3" {
    const mat1 = Mat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(4, 5, 6, 0), Vec4.init(7, 8, 9, 0), Vec4.init(10, 11, 12, 1));
    const mat2 = Mat44.init(Vec4.init(13, 14, 15, 0), Vec4.init(16, 17, 18, 0), Vec4.init(19, 20, 21, 0), Vec4.init(22, 23, 24, 1));
    const result = mat1.multiply3x3Mat44(mat2);
    try expect(result.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));
    const result2 = mat1.getRotationSafe().mul(mat2.getRotationSafe());
    try expect(result.eql(result2));
}

test "TestMat44Multiply3x3LeftTransposed" {
    const mat1 = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    const mat2 = Mat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), Vec4.init(29, 30, 31, 32));
    const result = mat1.multiply3x3LeftTransposed(mat2);
    try expect(result.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));
    const result2 = mat1.getRotationSafe().transposed().mul(mat2.getRotationSafe());
    try expect(result.eql(result2));
}

test "TestMat44Multiply3x3RightTransposed" {
    const mat1 = Mat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(4, 5, 6, 0), Vec4.init(7, 8, 9, 0), Vec4.init(10, 11, 12, 1));
    const mat2 = Mat44.init(Vec4.init(13, 14, 15, 0), Vec4.init(16, 17, 18, 0), Vec4.init(19, 20, 21, 0), Vec4.init(22, 23, 24, 1));
    const result = mat1.multiply3x3RightTransposed(mat2);
    try expect(result.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));
    const result2 = mat1.getRotationSafe().mul(mat2.getRotationSafe().transposed());
    try expect(result.eql(result2));
}

test "TestMat44Inversed" {
    const mat = Mat44.init(Vec4.init(2, 3, 5, 7), Vec4.init(11, 13, 17, 19), Vec4.init(23, 29, 31, 37), Vec4.init(41, 43, 47, 53));
    const inverse = mat.inversed();
    const identity = mat.mul(inverse);
    try expect(identity.isClose(Mat44.identity(), .{ .max_dist_sq = 2.0e-12 }));
}

test "TestMat44Inversed3x3" {
    const mat = Mat44.init(Vec4.init(2, 3, 5, 0), Vec4.init(11, 13, 17, 0), Vec4.init(23, 29, 31, 0), Vec4.init(41, 43, 47, 53));
    const inverse = mat.inversed3x3();
    try expect(inverse.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));
    const identity = mat.multiply3x3Mat44(inverse);
    try expect(identity.isClose(Mat44.identity(), .{ .max_dist_sq = 4.0e-12 }));
}

test "TestMat44SetInversed3x3" {
    var mat = Mat44.init(Vec4.init(2, 3, 5, 0), Vec4.init(11, 13, 17, 0), Vec4.init(23, 29, 31, 0), Vec4.init(41, 43, 47, 53));

    // First test succeeding inverse
    var inverse: Mat44 = undefined;
    try expect(inverse.setInversed3x3(mat));
    try expect(inverse.getColumn4(3).eql(Vec4.init(0, 0, 0, 1)));
    const identity = mat.multiply3x3Mat44(inverse);
    try expect(identity.isClose(Mat44.identity(), .{ .max_dist_sq = 4.0e-12 }));

    // Now make singular
    mat.setColumn4(0, Vec4.zero());
    try expect(!inverse.setInversed3x3(mat));
}

test "TestMat44GetDeterminant3x3" {
    const mat = Mat44.init(Vec4.init(2, 3, 5, 7), Vec4.init(11, 13, 17, 19), Vec4.init(23, 29, 31, 37), Vec4.init(41, 43, 47, 53));
    try expectEqual(@as(f32, 70), mat.getDeterminant3x3());
}

test "TestMat44Adjointed3x3" {
    const mat = Mat44.init(Vec4.init(2, 3, 5, 7), Vec4.init(11, 13, 17, 19), Vec4.init(23, 29, 31, 37), Vec4.init(41, 43, 47, 53));
    const result = mat.adjointed3x3();
    try expect(result.eql(Mat44.init(Vec4.init(-90, 52, -14, 0), Vec4.init(50, -53, 21, 0), Vec4.init(20, 11, -7, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44RotationXYZ" {
    var rot = Mat44.rotationX(0.5 * math.pi);
    var v = rot.mulVec3(Vec3.init(1, 0, 0));
    try expect(v.eql(Vec3.init(1, 0, 0)));
    v = rot.mulVec3(Vec3.init(0, 1, 0));
    try checkApproxEqual(v, Vec3.init(0, 0, 1), .{});
    v = rot.mulVec3(Vec3.init(0, 0, 1));
    try checkApproxEqual(v, Vec3.init(0, -1, 0), .{});

    rot = Mat44.rotationY(0.5 * math.pi);
    v = rot.mulVec3(Vec3.init(1, 0, 0));
    try checkApproxEqual(v, Vec3.init(0, 0, -1), .{});
    v = rot.mulVec3(Vec3.init(0, 1, 0));
    try expect(v.eql(Vec3.init(0, 1, 0)));
    v = rot.mulVec3(Vec3.init(0, 0, 1));
    try checkApproxEqual(v, Vec3.init(1, 0, 0), .{});

    rot = Mat44.rotationZ(0.5 * math.pi);
    v = rot.mulVec3(Vec3.init(1, 0, 0));
    try checkApproxEqual(v, Vec3.init(0, 1, 0), .{});
    v = rot.mulVec3(Vec3.init(0, 1, 0));
    try checkApproxEqual(v, Vec3.init(-1, 0, 0), .{});
    v = rot.mulVec3(Vec3.init(0, 0, 1));
    try expect(v.eql(Vec3.init(0, 0, 1)));
}

test "TestMat44RotationAxisAngle" {
    var r1 = Mat44.rotationX(0.1 * math.pi);
    var r2 = Mat44.rotation(Vec3.init(1, 0, 0), 0.1 * math.pi);
    try checkApproxEqual(r1, r2, .{});

    r1 = Mat44.rotationY(0.2 * math.pi);
    r2 = Mat44.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);
    try checkApproxEqual(r1, r2, .{});

    r1 = Mat44.rotationZ(0.3 * math.pi);
    r2 = Mat44.rotation(Vec3.init(0, 0, 1), 0.3 * math.pi);
    try checkApproxEqual(r1, r2, .{});
}

test "TestMat44CrossProduct" {
    const v1 = Vec3.init(1, 2, 3);
    const v2 = Vec3.init(4, 5, 6);
    const v3 = v1.cross(v2);
    const v4 = Mat44.crossProduct(v1).mulVec3(v2);
    try expect(v3.eql(v4));
}

test "TestMat44OuterProduct" {
    const v1 = Vec3.init(1, 2, 3);
    const v2 = Vec3.init(4, 5, 6);
    try expect(Mat44.outerProduct(v1, v2).eql(Mat44.init(Vec4.init(1 * 4, 2 * 4, 3 * 4, 0), Vec4.init(1 * 5, 2 * 5, 3 * 5, 0), Vec4.init(1 * 6, 2 * 6, 3 * 6, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44QuatLeftMultiply" {
    const p = Quat.init(2, 3, 4, 1);
    const q = Quat.init(6, 7, 8, 5);

    const r1 = p.mul(q);
    const r2 = Quat.fromVec4(Mat44.quatLeftMultiply(p).mulVec4(q.getXYZW()));
    try expect(r1.eql(r2));
}

test "TestMat44QuatRightMultiply" {
    const p = Quat.init(2, 3, 4, 1);
    const q = Quat.init(6, 7, 8, 5);

    const r1 = q.mul(p);
    const r2 = Quat.fromVec4(Mat44.quatRightMultiply(p).mulVec4(q.getXYZW()));
    try expect(r1.eql(r2));
}

test "TestMat44InverseRotateTranslate" {
    const rot = Quat.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);
    const pos = Vec3.init(2, 3, 4);

    const m1 = Mat44.rotationTranslation(rot, pos).inversed();
    const m2 = Mat44.inverseRotationTranslation(rot, pos);

    try checkApproxEqual(m1, m2, .{});
}

test "TestMat44InversedRotationTranslation" {
    const rot = Quat.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);
    const pos = Vec3.init(2, 3, 4);

    const m1 = Mat44.rotationTranslation(rot, pos).inversedRotationTranslation();
    const m2 = Mat44.inverseRotationTranslation(rot, pos);

    try checkApproxEqual(m1, m2, .{});
}

test "TestMat44Decompose" {
    const rotation = Mat44.rotationX(0.1 * math.pi).mul(Mat44.rotationZ(0.2 * math.pi));
    const scale = Vec3.init(-1, 2, 3);
    const mat = rotation.mul(Mat44.scaleVec3(scale));
    try expect(mat.getDeterminant3x3() < 0); // Left handed

    const decomposed = mat.decompose();
    const new_scale = decomposed.scale;
    const new_rotation = decomposed.rotation_translation;
    try expect(new_rotation.getDeterminant3x3() > 0); // Right handed

    const mat2 = new_rotation.mul(Mat44.scaleVec3(new_scale));
    try expect(mat.isClose(mat2, .{}));
}

test "TestMat44PrePostScaled" {
    const m = Mat44.init(Vec4.init(2, 3, 4, 0), Vec4.init(5, 6, 7, 0), Vec4.init(8, 9, 10, 0), Vec4.init(11, 12, 13, 1));
    const v = Vec3.init(14, 15, 16);

    try expect(m.preScaled(v).eql(m.mul(Mat44.scaleVec3(v))));
    try expect(m.postScaled(v).eql(Mat44.scaleVec3(v).mul(m)));
}

test "TestMat44PrePostTranslated" {
    const m = Mat44.init(Vec4.init(2, 3, 4, 0), Vec4.init(5, 6, 7, 0), Vec4.init(8, 9, 10, 0), Vec4.init(11, 12, 13, 1));
    const v = Vec3.init(14, 15, 16);

    try expect(m.preTranslated(v).eql(m.mul(Mat44.translation(v))));
    try expect(m.postTranslated(v).eql(Mat44.translation(v).mul(m)));
}

// Second TEST_CASE("TestMat44Decompose") in Mat44Tests.cpp, renamed because Zig doesn't allow duplicate test names
test "TestMat44Decompose2" {
    // Create a rotation/translation matrix
    const rot = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    const pos = Vec3.init(2, 3, 4);
    const rotation_translation = Mat44.rotationTranslation(rot, pos);

    // Scale the matrix
    const scale = Vec3.init(2, 1, 3);
    const m1 = rotation_translation.mul(Mat44.scaleVec3(scale));

    // Decompose scale
    const decomposed = m1.decompose();
    const scale_out = decomposed.scale;
    const m2 = decomposed.rotation_translation;

    // Check individual components
    try checkApproxEqual(rotation_translation, m2, .{});
    try checkApproxEqual(scale, scale_out, .{});
}

test "TestMat44DecomposeSkewed" {
    // Create a rotation/translation matrix
    const rot = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    const pos = Vec3.init(2, 3, 4);
    const rotation_translation = Mat44.rotationTranslation(rot, pos);

    // Skew the matrix by applying a non-uniform scale
    const skewed_rotation_translation = Mat44.scaleVec3(Vec3.init(1.0, 0.99, 0.98)).mul(rotation_translation);
    const val = skewed_rotation_translation.getAxisX().cross(skewed_rotation_translation.getAxisY()).dot(skewed_rotation_translation.getAxisZ());
    try expect(@abs(val - 1.0) > 0.01); // Check not matrix is no longer perpendicular

    // Scale the matrix
    const scale = Vec3.init(2, 1, 3);
    const m1 = skewed_rotation_translation.mul(Mat44.scaleVec3(scale));

    // Decompose scale
    const decomposed = m1.decompose();
    const scale_out = decomposed.scale;
    const m2 = decomposed.rotation_translation;

    // Check individual components
    try checkApproxEqual(m2.getAxisX(), skewed_rotation_translation.getAxisX().normalized(), .{}); // Check X axis didn't change
    try checkApproxEqual(m2.getAxisY(), skewed_rotation_translation.getAxisY().normalized(), .{ .tolerance = 0.003 }); // Y axis may move a bit
    try checkApproxEqual(m2.getAxisZ(), skewed_rotation_translation.getAxisZ().normalized(), .{ .tolerance = 0.02 }); // Z axis may move a bit
    try checkApproxEqual(m2.getAxisX().cross(m2.getAxisY()).dot(m2.getAxisZ()), 1.0, .{}); // Check perpendicular
    try checkApproxEqual(scale, scale_out, .{ .tolerance = 0.05 }); // Scale may change a bit
}

test "TestDMat44GetQuaternion" {
    const rot = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    const mat = Mat44.rotationQuat(rot);
    try checkApproxEqual(mat.getQuaternion(), rot, .{});
}

test "TestDMat44ConvertToString" {
    const v = Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 16));
    var buf: [128]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16", try std.fmt.bufPrint(&buf, "{f}", .{v}));
}
