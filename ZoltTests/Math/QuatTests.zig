//! Port of: UnitTests/Math/QuatTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const checkApproxEqual = fw.checkApproxEqual;
const math = zolt.math;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// std::uniform_real_distribution<float> as implemented by libstdc++: generate_canonical<float, 24> uses a single
/// 32 bit sample divided by 2^32 (clamped to stay below 1) and maps it to [a, b) with canonical * (b - a) + a.
/// The C++ standard does not fully specify this distribution, so the checks that use it are all approximate.
const UniformFloatDistribution = struct {
    a: f32,
    b: f32,

    fn init(a: f32, b: f32) UniformFloatDistribution {
        return .{ .a = a, .b = b };
    }

    fn next(self: UniformFloatDistribution, random: *fw.UnitTestRandom) f32 {
        var canonical = @as(f32, @floatFromInt(random.next() - fw.UnitTestRandom.min_value)) / 4294967296.0;
        if (canonical >= 1.0)
            canonical = std.math.nextAfter(f32, 1.0, 0.0);
        return canonical * (self.b - self.a) + self.a;
    }
};

fn newRandom() fw.UnitTestRandom {
    return fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
}

test "TestQuatSetXYZW" {
    var q = Quat.init(0, 0, 0, 0);
    try expect(q.eql(Quat.init(0, 0, 0, 0)));
    q.setX(1);
    q.setY(2);
    q.setZ(3);
    q.setW(4);
    try expect(q.eql(Quat.init(1, 2, 3, 4)));

    q.set(4, 3, 2, 1);
    try expect(q.eql(Quat.init(4, 3, 2, 1)));
}

test "TestQuatEqual" {
    try expect(Quat.init(1, 2, 3, 4).eql(Quat.init(1, 2, 3, 4)));
    try expect(!Quat.init(1, 2, 3, 4).eql(Quat.init(0, 2, 3, 4)));
    try expect(!Quat.init(1, 2, 3, 4).eql(Quat.init(1, 0, 3, 4)));
    try expect(!Quat.init(1, 2, 3, 4).eql(Quat.init(1, 2, 0, 4)));
    try expect(!Quat.init(1, 2, 3, 4).eql(Quat.init(1, 2, 3, 0)));
}

test "TestQuatZero" {
    const zero = Quat.zero();
    try expect(zero.eql(Quat.init(0, 0, 0, 0)));
}

test "TestQuatIdentity" {
    const identity = Quat.identity();

    try checkApproxEqual(identity.getX(), 0.0, .{});
    try checkApproxEqual(identity.getY(), 0.0, .{});
    try checkApproxEqual(identity.getZ(), 0.0, .{});
    try checkApproxEqual(identity.getW(), 1.0, .{});
}

test "TestQuatIsNaN" {
    const nan = std.math.nan(f32);
    try expect(Quat.init(nan, 0, 0, 0).isNaN());
    try expect(Quat.init(0, nan, 0, 0).isNaN());
    try expect(Quat.init(0, 0, nan, 0).isNaN());
    try expect(Quat.init(0, 0, 0, nan).isNaN());
}

test "TestQuatOperators" {
    try expect(Quat.init(1, 2, 3, 4).negate().eql(Quat.init(-1, -2, -3, -4)));
    try expect(Quat.init(1, 2, 3, 4).add(Quat.init(5, 6, 7, 8)).eql(Quat.init(6, 8, 10, 12)));
    try expect(Quat.init(5, 6, 7, 8).sub(Quat.init(4, 3, 2, 1)).eql(Quat.init(1, 3, 5, 7)));
    try expect(Quat.init(1, 2, 3, 4).mulScalar(5.0).eql(Quat.init(5, 10, 15, 20)));
    try expect(Quat.init(1, 2, 3, 4).mulScalar(5.0).eql(Quat.init(5, 10, 15, 20))); // 5.0f * Quat
    try expect(Quat.init(2, 4, 6, 8).divScalar(2.0).eql(Quat.init(1, 2, 3, 4)));

    var v = Quat.init(1, 2, 3, 4);
    v = v.add(Quat.init(5, 6, 7, 8));
    try expect(v.eql(Quat.init(6, 8, 10, 12)));
    v = v.sub(Quat.init(4, 3, 2, 1));
    try expect(v.eql(Quat.init(2, 5, 8, 11)));
    v = v.mulScalar(2.0);
    try expect(v.eql(Quat.init(4, 10, 16, 22)));
    v = v.divScalar(2.0);
    try expect(v.eql(Quat.init(2, 5, 8, 11)));
}

test "TestQuatPerpendicular" {
    const q1 = Quat.init(1, 2, 3, 4);
    try expectEqual(@as(f32, 0.0), q1.getPerpendicular().dot(q1));

    const q2 = Quat.init(-5, 4, -3, 2);
    try expectEqual(@as(f32, 0.0), q2.getPerpendicular().dot(q2));
}

test "TestQuatNormalized" {
    try expect(Quat.init(1, 0, 0, 0).isNormalized(.{}));
    try expect(Quat.init(-0.7071067, 0.7071067, 0, 0).isNormalized(.{}));
    try expect(Quat.init(0.5773502, -0.5773502, 0.5773502, 0).isNormalized(.{}));
    try expect(Quat.init(0.5, -0.5, 0.5, -0.5).isNormalized(.{}));
    try expect(!Quat.init(2, 0, 0, 0).isNormalized(.{}));
    try expect(!Quat.init(0, 2, 0, 0).isNormalized(.{}));
    try expect(!Quat.init(0, 0, 2, 0).isNormalized(.{}));
    try expect(!Quat.init(0, 0, 0, 2).isNormalized(.{}));
}

test "TestQuatConvertMatrix" {
    var random = newRandom();
    const zero_to_two_pi = UniformFloatDistribution.init(0.0, 2.0 * math.pi);
    for (0..1000) |_| {
        const axis = Vec3.random(&random);
        const angle = zero_to_two_pi.next(&random);

        const m1 = Mat44.rotation(axis, angle);
        const q1 = m1.getQuaternion();
        const q2 = Quat.rotation(axis, angle);
        try checkApproxEqual(q1, q2, .{});
        const m2 = Mat44.rotationQuat(q2);
        try checkApproxEqual(m1, m2, .{});
    }
}

test "TestQuatMultiplyVec3" {
    var random = newRandom();
    const zero_to_two_pi = UniformFloatDistribution.init(0.0, 2.0 * math.pi);
    for (0..1000) |_| {
        const axis = Vec3.random(&random);
        const angle = zero_to_two_pi.next(&random);
        const m1 = Mat44.rotation(axis, angle);
        const q1 = Quat.rotation(axis, angle);

        const rv = Vec3.random(&random).mulScalar(10.0);
        const r1 = m1.mulVec3(rv);
        const r2 = q1.mulVec3(rv);
        try checkApproxEqual(r1, r2, .{ .tolerance = 1.0e-5 });

        const r3 = q1.inverseRotate(r2);
        try checkApproxEqual(r3, rv, .{ .tolerance = 1.0e-5 });
    }
}

test "TestQuatRotateAxisXYZ" {
    var random = newRandom();
    const zero_to_two_pi = UniformFloatDistribution.init(0.0, 2.0 * math.pi);
    for (0..1000) |_| {
        const axis = Vec3.random(&random);
        const angle = zero_to_two_pi.next(&random);
        const q1 = Quat.rotation(axis, angle);

        var r1 = q1.mulVec3(Vec3.axisX());
        var r2 = q1.rotateAxisX();
        try checkApproxEqual(r1, r2, .{ .tolerance = 1.0e-5 });

        r1 = q1.mulVec3(Vec3.axisY());
        r2 = q1.rotateAxisY();
        try checkApproxEqual(r1, r2, .{ .tolerance = 1.0e-5 });

        r1 = q1.mulVec3(Vec3.axisZ());
        r2 = q1.rotateAxisZ();
        try checkApproxEqual(r1, r2, .{ .tolerance = 1.0e-5 });
    }
}

test "TestQuatMultiplyQuat" {
    {
        // We use a right handed system, so test that: i * j = k
        const r1 = Quat.init(1, 0, 0, 0).mul(Quat.init(0, 1, 0, 0));
        const r2 = Quat.init(0, 0, 1, 0);
        try expect(r1.isClose(r2, .{}));
    }

    {
        // Test: j * i = -k
        const r1 = Quat.init(0, 1, 0, 0).mul(Quat.init(1, 0, 0, 0));
        const r2 = Quat.init(0, 0, -1, 0);
        try expect(r1.isClose(r2, .{}));
    }

    {
        // Test predefined multiplication
        const r1 = Quat.init(2, 3, 4, 1).mul(Quat.init(6, 7, 8, 5));
        const r2 = Quat.init(12, 30, 24, -60);
        try expect(r1.isClose(r2, .{}));
    }

    // Compare random matrix multiplications with quaternion multiplications
    var random = newRandom();
    const zero_to_two_pi = UniformFloatDistribution.init(0.0, 2.0 * math.pi);
    for (0..1000) |_| {
        const axis1 = Vec3.random(&random);
        const angle1 = zero_to_two_pi.next(&random);
        const q1 = Quat.rotation(axis1, angle1);
        const m1 = Mat44.rotation(axis1, angle1);

        const axis2 = Vec3.random(&random);
        const angle2 = zero_to_two_pi.next(&random);
        const q2 = Quat.rotation(axis2, angle2);
        const m2 = Mat44.rotation(axis2, angle2);

        const r1 = q1.mul(q2);
        const r2 = m1.mul(m2).getQuaternion();

        try checkApproxEqual(r1, r2, .{});
    }
}

test "TestQuatRotationAxisAngle" {
    var r1 = Mat44.rotation(Vec3.init(1, 0, 0), 0.1 * math.pi);
    var r2 = Mat44.rotationQuat(Quat.rotation(Vec3.init(1, 0, 0), 0.1 * math.pi));
    try checkApproxEqual(r1, r2, .{});

    r1 = Mat44.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);
    r2 = Mat44.rotationQuat(Quat.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi));
    try checkApproxEqual(r1, r2, .{});

    r1 = Mat44.rotation(Vec3.init(0, 0, 1), 0.3 * math.pi);
    r2 = Mat44.rotationQuat(Quat.rotation(Vec3.init(0, 0, 1), 0.3 * math.pi));
    try checkApproxEqual(r1, r2, .{});
}

test "TestQuatGetAxisAngle" {
    // Test identity rotation
    {
        const r = Quat.identity().getAxisAngle();
        try checkApproxEqual(Vec3.zero(), r.axis, .{});
        try checkApproxEqual(@as(f32, 0.0), r.angle, .{});
    }

    {
        const r = Quat.identity().negate().getAxisAngle();
        try checkApproxEqual(Vec3.zero(), r.axis, .{});
        try checkApproxEqual(@as(f32, 0.0), r.angle, .{});
    }

    // Test positive rotation
    const q1 = Quat.rotation(Vec3.init(0, 1, 0), 0.2 * math.pi);

    {
        const r = q1.getAxisAngle();
        try checkApproxEqual(Vec3.init(0, 1, 0), r.axis, .{});
        try checkApproxEqual(0.2 * math.pi, r.angle, .{ .tolerance = 1.0e-5 });
    }

    {
        const r = q1.negate().getAxisAngle();
        try checkApproxEqual(Vec3.init(0, 1, 0), r.axis, .{});
        try checkApproxEqual(0.2 * math.pi, r.angle, .{ .tolerance = 1.0e-5 });
    }

    // Test negative rotation
    const q2 = Quat.rotation(Vec3.init(0, 1, 0), -0.2 * math.pi);

    {
        const r = q2.getAxisAngle();
        try checkApproxEqual(Vec3.init(0, -1, 0), r.axis, .{});
        try checkApproxEqual(0.2 * math.pi, r.angle, .{ .tolerance = 1.0e-5 });
    }

    {
        const r = q2.negate().getAxisAngle();
        try checkApproxEqual(Vec3.init(0, -1, 0), r.axis, .{});
        try checkApproxEqual(0.2 * math.pi, r.angle, .{ .tolerance = 1.0e-5 });
    }

    // Test keeping range between [0, PI]
    const q3 = Quat.rotation(Vec3.init(0, 1, 0), 1.1 * math.pi);

    {
        const r = q3.getAxisAngle();
        try checkApproxEqual(Vec3.init(0, -1, 0), r.axis, .{});
        try checkApproxEqual(0.9 * math.pi, r.angle, .{ .tolerance = 1.0e-5 });
    }

    {
        const r = q3.negate().getAxisAngle();
        try checkApproxEqual(Vec3.init(0, -1, 0), r.axis, .{});
        try checkApproxEqual(0.9 * math.pi, r.angle, .{ .tolerance = 1.0e-5 });
    }
}

test "TestQuatGetAngularVelocity" {
    const c_delta_time: f32 = 1.0 / 60.0;

    var random = newRandom();
    const angle_range = UniformFloatDistribution.init(-math.pi, math.pi);
    for (0..1000) |_| {
        const axis = Vec3.random(&random);
        const angle = angle_range.next(&random);
        const expected_angular_velocity = axis.mulScalar(angle).divScalar(c_delta_time);

        const q = Quat.rotation(axis, angle);

        const max_error = 2.0e-4 * expected_angular_velocity.length();

        var angular_velocity = q.getAngularVelocity(c_delta_time);
        try checkApproxEqual(angular_velocity, expected_angular_velocity, .{ .tolerance = max_error });

        angular_velocity = q.negate().getAngularVelocity(c_delta_time);
        try checkApproxEqual(angular_velocity, expected_angular_velocity, .{ .tolerance = max_error });
    }

    const c_small_angle: f32 = 1.0e-6;

    // Test very small angles
    {
        const q = Quat.rotation(Vec3.axisX(), c_small_angle);
        const angular_velocity = q.getAngularVelocity(c_delta_time);
        try checkApproxEqual(angular_velocity, Vec3.init(c_small_angle / c_delta_time, 0, 0), .{ .tolerance = 1.0e-5 * c_small_angle });
    }

    {
        const q = Quat.rotation(Vec3.axisY().negate(), c_small_angle);
        const angular_velocity = q.getAngularVelocity(c_delta_time);
        try checkApproxEqual(angular_velocity, Vec3.init(0, -c_small_angle / c_delta_time, 0), .{ .tolerance = 1.0e-5 * c_small_angle });
    }

    {
        const q = Quat.rotation(Vec3.axisZ(), -c_small_angle).negate();
        const angular_velocity = q.getAngularVelocity(c_delta_time);
        try checkApproxEqual(angular_velocity, Vec3.init(0, 0, -c_small_angle / c_delta_time), .{ .tolerance = 1.0e-5 * c_small_angle });
    }
}

test "TestQuatInverse" {
    var random = newRandom();
    const zero_to_two_pi = UniformFloatDistribution.init(0.0, 2.0 * math.pi);
    for (0..1000) |_| {
        const axis = Vec3.random(&random);
        const angle = zero_to_two_pi.next(&random);

        const q1 = Quat.rotation(axis, angle);
        const q2 = q1.inversed();

        try checkApproxEqual(Quat.identity(), q1.mul(q2), .{});
    }
}

test "TestQuatConjugate" {
    try expect(Quat.init(1, 2, 3, 4).conjugated().eql(Quat.init(-1, -2, -3, 4)));
    try expect(Quat.init(-1, -2, -3, -4).conjugated().eql(Quat.init(1, 2, 3, -4)));
}

test "TestQuatEnsureWPositive" {
    try expect(Quat.init(1, -2, 3, -4).ensureWPositive().eql(Quat.init(-1, 2, -3, 4)));
    try expect(Quat.init(-4, 5, -6, 7).ensureWPositive().eql(Quat.init(-4, 5, -6, 7)));
    try expect(Quat.init(1, 2, 3, 0).ensureWPositive().eql(Quat.init(1, 2, 3, 0)));
}

test "TestQuatStoreFloat3" {
    var q1: Float3 = undefined;
    Quat.init(0.7071067, 0, 0, -0.7071067).storeFloat3(&q1);
    try expect(q1.eql(Float3.init(-0.7071067, 0, 0)));

    var q2: Float3 = undefined;
    Quat.init(0, 0.7071067, 0, 0.7071067).storeFloat3(&q2);
    try expect(q2.eql(Float3.init(0, 0.7071067, 0)));

    var q3: Float3 = undefined;
    Quat.init(0, 0, 1, 0).storeFloat3(&q3);
    try expect(q3.eql(Float3.init(0, 0, 1)));
}

test "TestQuatGetTwistAxis" {
    const q1 = Quat.rotation(Vec3.axisX(), math.degreesToRadians(-10.0));
    const q2 = Quat.rotation(Vec3.axisY(), math.degreesToRadians(20.0));
    const q = q1.mul(q2);

    const twist1 = q.getTwist(Vec3.axisX());
    try checkApproxEqual(twist1, q1, .{});
    const swing1 = twist1.inversed().mul(q);
    try checkApproxEqual(swing1, q2, .{});
    const twist2 = swing1.getTwist(Vec3.axisY());
    try checkApproxEqual(twist2, q2, .{});
    const swing2 = twist2.inversed().mul(swing1);
    try checkApproxEqual(swing2, Quat.identity(), .{});

    try expect(Quat.zero().getTwist(Vec3.axisX()).eql(Quat.identity()));
}

test "TestQuatGetRotationAngle" {
    const q1 = Quat.rotation(Vec3.axisX(), math.degreesToRadians(-10.0));
    const q2 = Quat.rotation(Vec3.axisY(), math.degreesToRadians(20.0));
    const q3 = Quat.rotation(Vec3.axisZ(), math.degreesToRadians(-95.0));

    var a = q1.getRotationAngle(Vec3.axisX());
    try checkApproxEqual(a, math.degreesToRadians(-10.0), .{ .tolerance = 1.0e-5 });

    a = q2.getRotationAngle(Vec3.axisY());
    try checkApproxEqual(a, math.degreesToRadians(20.0), .{ .tolerance = 1.0e-5 });

    a = q3.getRotationAngle(Vec3.axisZ());
    try checkApproxEqual(a, math.degreesToRadians(-95.0), .{ .tolerance = 1.0e-5 });

    a = q1.mul(q2).getRotationAngle(Vec3.axisX());
    try checkApproxEqual(a, math.degreesToRadians(-10.0), .{ .tolerance = 1.0e-5 });

    a = q3.mul(q1).getRotationAngle(Vec3.axisX());
    try checkApproxEqual(a, math.degreesToRadians(-10.0), .{ .tolerance = 1.0e-5 });
}

test "TestQuatEulerAngles" {
    var random = newRandom();

    // Pitch clamped to +/- 85 degrees to avoid gimbal lock singularity near +/- 90 degrees
    const full_range = UniformFloatDistribution.init(math.degreesToRadians(-180.0), math.degreesToRadians(180.0));
    const pitch_range = UniformFloatDistribution.init(math.degreesToRadians(-85.0), math.degreesToRadians(85.0));

    // i == -1 runs a fixed regression case before random cases
    const fixed_input = Vec3.init(math.degreesToRadians(-10.0), math.degreesToRadians(20.0), math.degreesToRadians(-95.0));
    var i: i32 = -1;
    while (i < 1000) : (i += 1) {
        const input = if (i == -1)
            fixed_input
        else
            Vec3.init(full_range.next(&random), pitch_range.next(&random), full_range.next(&random));

        // Create ground truth by multiplying 3 separate axis rotations (ZYX order)
        const qx = Quat.rotation(Vec3.axisX(), input.getX());
        const qy = Quat.rotation(Vec3.axisY(), input.getY());
        const qz = Quat.rotation(Vec3.axisZ(), input.getZ());
        const q_expected = qz.mul(qy).mul(qx);

        // Test eulerAngles (Euler -> Quat)
        const q_actual = Quat.eulerAngles(input);
        try checkApproxEqual(q_expected, q_actual, .{ .tolerance = 1.0e-4 });

        // Test getEulerAngles (Quat -> Euler), tested against both q_actual and q_expected
        // to catch cases where eulerAngles and getEulerAngles share a bug
        try checkApproxEqual(q_actual.getEulerAngles(), input, .{ .tolerance = 1.0e-4 });
        try checkApproxEqual(q_expected.getEulerAngles(), input, .{ .tolerance = 1.0e-4 });
    }
}

test "TestQuatRotationFromTo" {
    {
        // Parallel vectors
        const v1 = Vec3.init(10, 0, 0);
        const v2 = Vec3.init(20, 0, 0);
        const q = Quat.fromTo(v1, v2);
        try checkApproxEqual(q, Quat.identity(), .{});
    }

    {
        // Perpendicular vectors
        const v1 = Vec3.init(10, 0, 0);
        const v2 = Vec3.init(0, 20, 0);
        const q = Quat.fromTo(v1, v2);
        try checkApproxEqual(v2.normalized(), q.mulVec3(v1).normalized(), .{});
    }

    {
        // Vectors with 180 degree angle
        const v1 = Vec3.init(10, 0, 0);
        const v2 = Vec3.init(-20, 0, 0);
        const q = Quat.fromTo(v1, v2);
        try checkApproxEqual(v2.normalized(), q.mulVec3(v1).normalized(), .{});
    }

    {
        // Test v1 zero
        const v1 = Vec3.zero();
        const v2 = Vec3.init(10, 0, 0);
        const q = Quat.fromTo(v1, v2);
        try expect(q.eql(Quat.identity()));
    }

    {
        // Test v2 zero
        const v1 = Vec3.init(10, 0, 0);
        const v2 = Vec3.zero();
        const q = Quat.fromTo(v1, v2);
        try expect(q.eql(Quat.identity()));
    }

    {
        // Length of a vector is squared inside the function: try with sqrt(FLT_MIN) to see if that still returns a valid rotation
        const v1 = Vec3.init(0, @sqrt(math.flt_min), 0);
        const v2 = Vec3.init(1, 0, 0);
        const q = Quat.fromTo(v1, v2);
        try checkApproxEqual(v2.normalized(), q.mulVec3(v1).normalized(), .{});
    }
}

test "TestQuatRotationFromToRandom" {
    var random = newRandom();
    const one_to_ten = UniformFloatDistribution.init(1.0, 10.0);
    for (0..1000) |_| {
        const s1 = one_to_ten.next(&random);
        const v1 = Vec3.random(&random).mulScalar(s1);
        const s2 = one_to_ten.next(&random);
        const v2 = Vec3.random(&random).mulScalar(s2);

        const q = Quat.fromTo(v1, v2);

        const v1t = q.mulVec3(v1).normalized();
        const v2t = v2.normalized();
        try checkApproxEqual(v2t, v1t, .{ .tolerance = 0.5e-4 });
    }
}

test "TestQuatConvertToString" {
    const v = Quat.init(1, 2, 3, 4);
    var buf: [64]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3, 4", try std.fmt.bufPrint(&buf, "{f}", .{v}));
}

test "TestQuatLERP" {
    const v1 = Quat.init(1, 2, 3, 4);
    const v2 = Quat.init(5, 6, 7, 8);
    try expect(v1.lerp(v2, 0.25).eql(Quat.init(2, 3, 4, 5)));
}

test "TestQuatSLERP" {
    const v1 = Quat.identity();
    const v2 = Quat.rotation(Vec3.axisX(), 0.99 * math.pi);
    try checkApproxEqual(v1.slerp(v2, 0.25), Quat.rotation(Vec3.axisX(), @as(f32, 0.25) * 0.99 * math.pi), .{});

    // Check that we ignore the sign
    const v3 = Quat.init(1, 2, 3, 4).normalized();
    try checkApproxEqual(v3.slerp(v3.negate(), 0.5), v3, .{});
}

test "TestQuatMultiplyImaginary" {
    var random = newRandom();
    for (0..1000) |_| {
        const imaginary = Vec3.random(&random);
        const quat = Quat.random(&random);

        const r1 = Quat.multiplyImaginary(imaginary, quat);
        const r2 = Quat.fromVec4(Vec4.fromVec3W(imaginary, 0)).mul(quat);
        try checkApproxEqual(r1, r2, .{});
    }
}

test "TestQuatCompressUnitQuat" {
    var random = newRandom();
    for (0..1000) |_| {
        const quat = Quat.random(&random);
        const compressed = quat.compressUnitQuat();
        const decompressed = Quat.decompressUnitQuat(compressed);
        const axis_angle = quat.mul(decompressed.conjugated()).getAxisAngle();
        try expect(@abs(axis_angle.angle) < 0.009);
    }
}
