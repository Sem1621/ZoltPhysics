//! Port of: UnitTests/Geometry/RayAABoxTests.cpp
//! Status: complete

const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const flt_max = zolt.math.flt_max;
const rayAABox = zolt.rayAABox;
const RayInvDirection = zolt.RayInvDirection;
const Vec3 = zolt.Vec3;

test "TestRayAABox" {
    // Jolt: AABox box(Vec3::sReplicate(-1.0f), Vec3::sOne()). The test only uses the bounds of the box
    // (box.mMin, box.mMax), so it holds them in a plain struct here.
    // TODO(Geometry merge): use `const box = AABox.init(Vec3.replicate(-1.0), Vec3.one());`
    const box = .{ .min = Vec3.replicate(-1.0), .max = Vec3.one() };

    for (0..3) |axis_index| {
        const axis: u32 = @intCast(axis_index);

        {
            // Ray starting in the center of the box, pointing high
            const origin = Vec3.zero();
            var direction = Vec3.zero();
            direction.setComponent(axis, 1.0);
            const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
            try fw.checkApproxEqual(@as(f32, -1.0), fraction, .{ .tolerance = 1.0e-6 });
        }

        {
            // Ray starting in the center of the box, pointing low
            const origin = Vec3.zero();
            var direction = Vec3.zero();
            direction.setComponent(axis, -1.0);
            const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
            try fw.checkApproxEqual(@as(f32, -1.0), fraction, .{ .tolerance = 1.0e-6 });
        }

        {
            // Ray starting high, pointing to low
            var origin = Vec3.zero();
            origin.setComponent(axis, 1.1);
            var direction = Vec3.zero();
            direction.setComponent(axis, -1.0);
            const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
            try fw.checkApproxEqual(@as(f32, 0.1), fraction, .{ .tolerance = 1.0e-6 });
        }

        {
            // Ray starting high, pointing to high
            var origin = Vec3.zero();
            origin.setComponent(axis, 1.1);
            var direction = Vec3.zero();
            direction.setComponent(axis, 1.0);
            const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
            try fw.expect(fraction == flt_max);
        }

        {
            // Ray starting low, pointing to high
            var origin = Vec3.zero();
            origin.setComponent(axis, -1.1);
            var direction = Vec3.zero();
            direction.setComponent(axis, 1.0);
            const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
            try fw.checkApproxEqual(@as(f32, 0.1), fraction, .{ .tolerance = 1.0e-6 });
        }

        {
            // Ray starting low, pointing to low
            var origin = Vec3.zero();
            origin.setComponent(axis, -1.1);
            var direction = Vec3.zero();
            direction.setComponent(axis, -1.0);
            const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
            try fw.expect(fraction == flt_max);
        }
    }

    {
        // Test ray that hits top plane under an angle
        const expected_hit = Vec3.init(0, 1, 0);
        const expected_fraction: f32 = 0.123;
        const direction = Vec3.init(4, -4, 0);
        const origin = expected_hit.sub(direction.mulScalar(expected_fraction));
        const fraction = rayAABox(origin, RayInvDirection.init(direction), box.min, box.max);
        try fw.checkApproxEqual(expected_fraction, fraction, .{ .tolerance = 1.0e-6 });
    }
}
