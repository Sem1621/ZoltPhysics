//! Port of: UnitTests/Math/EigenValueSymmetricTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const math = zolt.math;
const eigenValueSymmetric = zolt.eigenValueSymmetric;
const Mat44 = zolt.Mat44;
const Matrix = zolt.Matrix;
const Vec3 = zolt.Vec3;
const Vector = zolt.Vector;

test "TestEigenValueSymmetric" {
    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    const angle_distribution = fw.UniformFloatDistribution.init(0, 2.0 * math.pi);
    const scale_distribution = fw.UniformFloatDistribution.init(0.1, 10.0);

    for (0..1000) |_| {
        // Random scale vector
        // (C++ leaves the evaluation order of constructor / function arguments unspecified, Zolt draws them left to right)
        const scale_x = scale_distribution.next(&random);
        const scale_y = scale_distribution.next(&random);
        const scale_z = scale_distribution.next(&random);
        const scale = Vec3.init(scale_x, scale_y, scale_z);

        // Random rotation matrix
        const axis = Vec3.random(&random);
        const rotation = Mat44.rotation(axis, angle_distribution.next(&random));

        // Construct a symmetric tensor from this rotation and scale
        const tensor4 = rotation.multiply3x3Mat44(Mat44.scaleVec3(scale)).multiply3x3RightTransposed(rotation);

        // Get the eigenvalues and eigenvectors
        var tensor: Matrix(3, 3) = undefined;
        var eigen_vec = Matrix(3, 3).identity();
        var eigen_val: Vector(3) = undefined;
        tensor.copyPart(tensor4, 0, 0, 3, 3, 0, 0);
        try expect(eigenValueSymmetric(tensor, &eigen_vec, &eigen_val));

        for (0..3) |c_usize| {
            const c: u32 = @intCast(c_usize);

            // Check that we found a valid eigenvalue
            var found = false;
            for (0..3) |c2| {
                if (@abs(scale.getComponent(@intCast(c2)) - eigen_val.getComponent(c)) < 1.0e-5) {
                    found = true;
                    break;
                }
            }
            try expect(found);

            // Check if the eigenvector is normalized
            try expect(eigen_vec.getColumn(c).isNormalized(.{}));

            // Check if matrix * eigen_vector = eigen_value * eigen_vector
            const mat_eigvec = tensor.mulVector(eigen_vec.getColumn(c));
            const eigval_eigvec = eigen_vec.getColumn(c).mulScalar(eigen_val.getComponent(c));
            try expect(mat_eigvec.isClose(eigval_eigvec, .{ .max_dist_sq = math.square(@as(f32, 1.0e-5)) }));
        }
    }
}
