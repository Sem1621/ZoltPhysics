//! Port of: UnitTests/Math/DMat44Tests.cpp
//! Status: complete
//!
//! DMat44Tests.cpp has two test cases named TestDMat44Rotation and two named TestDMat44PrePostTranslated.
//! Zig does not allow duplicate test names, so the second ones are called TestDMat44Rotation2 and
//! TestDMat44PrePostTranslated2.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const checkApproxEqual = fw.checkApproxEqual;
const math = zolt.math;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

test "TestDMat44Zero" {
    const zero = DMat44.zero();

    try expect(zero.eql(DMat44.init(Vec4.init(0, 0, 0, 0), Vec4.init(0, 0, 0, 0), Vec4.init(0, 0, 0, 0), DVec3.init(0, 0, 0))));
    try expect(zero.getAxisX().eql(Vec3.zero()));
    try expect(zero.getAxisY().eql(Vec3.zero()));
    try expect(zero.getAxisZ().eql(Vec3.zero()));
    try expect(zero.getTranslation().eql(DVec3.zero()));
}

test "TestDMat44Identity" {
    const identity = DMat44.identity();

    try expect(identity.eql(DMat44.init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), DVec3.init(0, 0, 0))));

    // Check non-equality
    try expect(!identity.eql(DMat44.init(Vec4.init(0, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), DVec3.init(0, 0, 0))));
    try expect(!identity.eql(DMat44.init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 0, 0, 0), Vec4.init(0, 0, 1, 0), DVec3.init(0, 0, 0))));
    try expect(!identity.eql(DMat44.init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 0, 0), DVec3.init(0, 0, 0))));
    try expect(!identity.eql(DMat44.init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), DVec3.init(1, 0, 0))));
}

test "TestDMat44Construct" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));

    try expect(mat.getColumn4(0).eql(Vec4.init(1, 2, 3, 4)));
    try expect(mat.getColumn4(1).eql(Vec4.init(5, 6, 7, 8)));
    try expect(mat.getColumn4(2).eql(Vec4.init(9, 10, 11, 12)));
    try expect(mat.getTranslation().eql(DVec3.init(13, 14, 15)));

    const mat2 = mat;

    try expect(mat2.getColumn4(0).eql(Vec4.init(1, 2, 3, 4)));
    try expect(mat2.getColumn4(1).eql(Vec4.init(5, 6, 7, 8)));
    try expect(mat2.getColumn4(2).eql(Vec4.init(9, 10, 11, 12)));
    try expect(mat2.getTranslation().eql(DVec3.init(13, 14, 15)));
}

test "TestDMat44Scale" {
    try expect(DMat44.scaleVec3(Vec3.init(2, 3, 4)).eql(DMat44.init(Vec4.init(2, 0, 0, 0), Vec4.init(0, 3, 0, 0), Vec4.init(0, 0, 4, 0), DVec3.init(0, 0, 0))));
}

test "TestDMat44Rotation" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));
    try expect(mat.getRotation().eql(Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(0, 0, 0, 1))));
}

test "TestMat44SetRotation" {
    var mat = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));
    const mat2 = Mat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), Vec4.init(29, 30, 31, 32));

    mat.setRotation(mat2);
    try expect(mat.eql(DMat44.init(Vec4.init(17, 18, 19, 20), Vec4.init(21, 22, 23, 24), Vec4.init(25, 26, 27, 28), DVec3.init(13, 14, 15))));
}

test "TestDMat44Rotation2" {
    const q = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    try expect(DMat44.rotationQuat(q).toMat44().eql(Mat44.rotationQuat(q)));
}

test "TestDMat44Translation" {
    try expect(DMat44.translation(DVec3.init(1, 2, 3)).eql(DMat44.init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), DVec3.init(1, 2, 3))));
}

test "TestDMat44RotationTranslation" {
    const q = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    try expect(DMat44.rotationTranslation(q, DVec3.init(1, 2, 3)).toMat44().eql(Mat44.rotationTranslation(q, Vec3.init(1, 2, 3))));
}

test "TestDMat44MultiplyMat44" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(5, 6, 7, 0), Vec4.init(9, 10, 11, 0), DVec3.init(13, 14, 15));
    const mat2 = Mat44.init(Vec4.init(17, 18, 19, 0), Vec4.init(21, 22, 23, 0), Vec4.init(25, 26, 27, 0), Vec4.init(29, 30, 31, 1));

    const result = mat.mulMat44(mat2);
    try expect(result.eql(DMat44.init(Vec4.init(278, 332, 386, 0), Vec4.init(338, 404, 470, 0), Vec4.init(398, 476, 554, 0), DVec3.init(471, 562, 653))));
}

test "TestDMat44MultiplyDMat44" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 0), Vec4.init(5, 6, 7, 0), Vec4.init(9, 10, 11, 0), DVec3.init(13, 14, 15));
    const mat2 = DMat44.init(Vec4.init(17, 18, 19, 0), Vec4.init(21, 22, 23, 0), Vec4.init(25, 26, 27, 0), DVec3.init(29, 30, 31));

    const result = mat.mul(mat2);
    try expect(result.eql(DMat44.init(Vec4.init(278, 332, 386, 0), Vec4.init(338, 404, 470, 0), Vec4.init(398, 476, 554, 0), DVec3.init(471, 562, 653))));
}

test "TestDMat44MultiplyVec3" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));
    const vec = Vec3.init(17, 18, 19);

    const result = mat.mulDVec3(DVec3.fromVec3(vec));
    try expect(result.eql(DVec3.init(291, 346, 401)));

    const result2 = mat.mulVec3(vec);
    try expect(result2.eql(DVec3.init(291, 346, 401)));

    const result3 = mat.multiply3x3(vec);
    try expect(result3.eql(Vec3.init(278, 332, 386)));

    const result4 = mat.multiply3x3Transposed(vec);
    try expect(result4.eql(Vec3.init(110, 326, 542)));
}

test "TestDMat44Inversed" {
    const mat = DMat44.init(Vec4.init(1, 16, 2, 0), Vec4.init(2, 8, 4, 0), Vec4.init(8, 4, 1, 0), DVec3.init(4, 2, 8));
    const inverse = mat.inversed();
    const identity = mat.mul(inverse);
    try checkApproxEqual(identity, DMat44.identity(), .{});
}

test "TestDMat44InverseRotateTranslate" {
    const rot = Quat.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);
    const pos = DVec3.init(2, 3, 4);

    const m1 = DMat44.rotationTranslation(rot, pos).inversed();
    const m2 = DMat44.inverseRotationTranslation(rot, pos);

    try checkApproxEqual(m1, m2, .{});
}

test "TestDMat44InversedRotationTranslation" {
    const rot = Quat.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);
    const pos = DVec3.init(2, 3, 4);

    const m1 = DMat44.rotationTranslation(rot, pos).inversedRotationTranslation();
    const m2 = DMat44.inverseRotationTranslation(rot, pos);

    try checkApproxEqual(m1, m2, .{});
}

test "TestDMat44PrePostScaled" {
    const m = DMat44.init(Vec4.init(2, 3, 4, 0), Vec4.init(5, 6, 7, 0), Vec4.init(8, 9, 10, 0), DVec3.init(11, 12, 13));
    const v = Vec3.init(14, 15, 16);

    try expect(m.preScaled(v).eql(m.mul(DMat44.scaleVec3(v))));
    try expect(m.postScaled(v).eql(DMat44.scaleVec3(v).mul(m)));
}

test "TestDMat44PrePostTranslated" {
    const m = DMat44.init(Vec4.init(2, 3, 4, 0), Vec4.init(5, 6, 7, 0), Vec4.init(8, 9, 10, 0), DVec3.init(11, 12, 13));
    const v = Vec3.init(14, 15, 16);

    try checkApproxEqual(m.preTranslated(v), m.mul(DMat44.translation(DVec3.fromVec3(v))), .{});
    try checkApproxEqual(m.postTranslated(v), DMat44.translation(DVec3.fromVec3(v)).mul(m), .{});
}

test "TestDMat44Decompose" {
    // Create a rotation/translation matrix
    const rot = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    const pos = DVec3.init(2, 3, 4);
    const rotation_translation = DMat44.rotationTranslation(rot, pos);

    // Scale the matrix
    const scale = Vec3.init(2, 1, 3);
    const m1 = rotation_translation.mul(DMat44.scaleVec3(scale));

    // Decompose scale
    const decomposed = m1.decompose();
    const scale_out = decomposed.scale;
    const m2 = decomposed.rotation_translation;

    // Check individual components
    try checkApproxEqual(rotation_translation, m2, .{});
    try checkApproxEqual(scale, scale_out, .{});
}

test "TestDMat44ToMat44" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));
    try expect(mat.toMat44().eql(Mat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), Vec4.init(13, 14, 15, 1))));
}

test "TestDMat44Column" {
    var mat = DMat44.zero();
    mat.setColumn4(0, Vec4.init(1, 2, 3, 4));
    try expect(mat.getColumn4(0).eql(Vec4.init(1, 2, 3, 4)));
    mat.setColumn3(0, Vec3.init(5, 6, 7));
    try expect(mat.getColumn3(0).eql(Vec3.init(5, 6, 7)));
    try expect(mat.getColumn4(0).eql(Vec4.init(5, 6, 7, 0)));

    mat.setAxisX(Vec3.init(8, 9, 10));
    mat.setAxisY(Vec3.init(11, 12, 13));
    mat.setAxisZ(Vec3.init(14, 15, 16));
    mat.setTranslation(DVec3.init(17, 18, 19));
    try expect(mat.getAxisX().eql(Vec3.init(8, 9, 10)));
    try expect(mat.getAxisY().eql(Vec3.init(11, 12, 13)));
    try expect(mat.getAxisZ().eql(Vec3.init(14, 15, 16)));
    try expect(mat.getTranslation().eql(DVec3.init(17, 18, 19)));
}

test "TestDMat44Transposed" {
    const mat = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));
    const result = mat.transposed3x3();
    try expect(result.eql(Mat44.init(Vec4.init(1, 5, 9, 0), Vec4.init(2, 6, 10, 0), Vec4.init(3, 7, 11, 0), Vec4.init(0, 0, 0, 1))));
}

test "TestDMat44GetQuaternion" {
    const rot = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.2 * math.pi);
    const mat = DMat44.rotationQuat(rot);
    try checkApproxEqual(mat.getQuaternion(), rot, .{});
}

test "TestDMat44PrePostTranslated2" {
    const m = DMat44.init(Vec4.init(2, 3, 4, 0), Vec4.init(5, 6, 7, 0), Vec4.init(8, 9, 10, 0), DVec3.init(11, 12, 13));
    const v = DVec3.init(14, 15, 16);

    try expect(m.preTranslatedDVec3(v).eql(m.mul(DMat44.translation(v))));
    try expect(m.postTranslatedDVec3(v).eql(DMat44.translation(v).mul(m)));
}

test "TestDMat44ConvertToString" {
    const v = DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15));
    var buf: [128]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15", try std.fmt.bufPrint(&buf, "{f}", .{v}));
}
