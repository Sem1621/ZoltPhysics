//! Port of: Jolt/Math/EigenValueSymmetric.h
//! Status: complete
//!
//! Jolt disables the invalid floating point exception during this function (FPExceptionDisableInvalid).
//! Zolt never enables floating point exceptions, so there is nothing to port for that.

const std = @import("std");
const math = @import("Math.zig");
const Core = @import("../Core/Core.zig");

/// Function to determine the eigen vectors and values of a N x N real symmetric matrix
/// by Jacobi transformations. This method is most suitable for N < 10.
///
/// Taken and adapted from Numerical Recipes paragraph 11.1
///
/// An eigen vector is a vector v for which \f$A \: v = \lambda \: v\f$
///
/// Where:
/// A: A square matrix.
/// \f$\lambda\f$: a non-zero constant value.
///
/// @see https://en.wikipedia.org/wiki/Eigenvalues_and_eigenvectors
///
/// Matrix is a matrix type, which has dimensions N x N (e.g. `Matrix(3, 3)`), the vector type
/// has N rows (e.g. `Vector(3)`).
/// @param matrix is the matrix of which to return the eigenvalues and vectors
/// @param eig_vec will contain a matrix whose columns contain the normalized eigenvectors (must be identity before call)
/// @param eig_val will contain the eigenvalues (pointer to a vector)
pub fn eigenValueSymmetric(matrix: anytype, eig_vec: *@TypeOf(matrix), eig_val: anytype) bool {
    const VectorT = @typeInfo(@TypeOf(eig_val)).pointer.child;

    // This algorithm can generate infinite values, see comment below
    // (FPExceptionDisableInvalid: not needed, Zolt doesn't enable floating point exceptions)

    // Maximum number of sweeps to make
    const max_sweeps = 50;

    // Get problem dimension
    const n: u32 = matrix.getRows();

    // Make sure the dimensions are right
    std.debug.assert(matrix.getRows() == n);
    std.debug.assert(matrix.getCols() == n);
    std.debug.assert(eig_vec.getRows() == n);
    std.debug.assert(eig_vec.getCols() == n);
    std.debug.assert(eig_val.getRows() == n);
    std.debug.assert(eig_vec.isIdentity());

    // Get the matrix in a so we can mess with it
    var a = matrix;

    var b: VectorT = undefined;
    var z: VectorT = undefined;

    var ip: u32 = 0;
    while (ip < n) : (ip += 1) {
        // Initialize b to diagonal of a
        b.setComponent(ip, a.get(ip, ip));

        // Initialize output to diagonal of a
        eig_val.setComponent(ip, a.get(ip, ip));

        // Reset z
        z.setComponent(ip, 0.0);
    }

    var sweep: i32 = 0;
    while (sweep < max_sweeps) : (sweep += 1) {
        // Get the sum of the off-diagonal elements of a
        var sm: f32 = 0.0;
        ip = 0;
        while (ip < n - 1) : (ip += 1) {
            var iq: u32 = ip + 1;
            while (iq < n) : (iq += 1)
                sm += @abs(a.get(ip, iq));
        }
        const avg_sm = sm / @as(f32, @floatFromInt(math.square(n)));

        // Normal return, convergence to machine underflow
        if (avg_sm < math.flt_min) { // Original code: sm == 0.0f, when the average is denormal, we also consider it machine underflow
            // Sanity checks
            if (Core.enable_asserts) {
                var c: u32 = 0;
                while (c < n) : (c += 1) {
                    // Check if the eigenvector is normalized
                    std.debug.assert(eig_vec.getColumn(c).isNormalized(.{}));

                    // Check if inMatrix * eigen_vector = eigen_value * eigen_vector
                    const mat_eigvec: VectorT = matrix.mulVector(eig_vec.getColumn(c));
                    const eigval_eigvec: VectorT = eig_vec.getColumn(c).mulScalar(eig_val.getComponent(c));
                    std.debug.assert(mat_eigvec.isClose(eigval_eigvec, .{ .max_dist_sq = math.max(mat_eigvec.lengthSq(), eigval_eigvec.lengthSq()) * 1.0e-6 }));
                }
            }

            // Success
            return true;
        }

        // On the first three sweeps use a fraction of the sum of the off diagonal elements as threshold
        // Note that we pick a minimum threshold of FLT_MIN because dividing by a denormalized number is likely to result in infinity.
        const thresh: f32 = if (sweep < 4) 0.2 * avg_sm else math.flt_min; // Original code: 0.0f instead of FLT_MIN

        ip = 0;
        while (ip < n - 1) : (ip += 1) {
            var iq: u32 = ip + 1;
            while (iq < n) : (iq += 1) {
                // a_pq, eigval_p and eigval_q are references into a and eig_val in Jolt
                const a_pq = a.get(ip, iq);

                const abs_a_pq = @abs(a_pq);
                const g = 100.0 * abs_a_pq;

                // After four sweeps, skip the rotation if the off-diagonal element is small
                if (sweep > 4 and
                    @abs(eig_val.getComponent(ip)) + g == @abs(eig_val.getComponent(ip)) and
                    @abs(eig_val.getComponent(iq)) + g == @abs(eig_val.getComponent(iq)))
                {
                    a.set(ip, iq, 0.0);
                } else if (abs_a_pq > thresh) {
                    var h = eig_val.getComponent(iq) - eig_val.getComponent(ip);
                    const abs_h = @abs(h);

                    var t: f32 = undefined;
                    if (abs_h + g == abs_h) {
                        t = a_pq / h;
                    } else {
                        const theta = 0.5 * h / a_pq; // Warning: Can become infinite if a(ip, iq) is very small which may trigger an invalid float exception
                        t = 1.0 / (@abs(theta) + math.sqrt(1.0 + theta * theta)); // If theta becomes inf, t will be 0 so the infinite is not a problem for the algorithm
                        if (theta < 0.0) t = -t;
                    }

                    const c = 1.0 / math.sqrt(1.0 + t * t);
                    const s = t * c;
                    const tau = s / (1.0 + c);
                    h = t * a_pq;

                    a.set(ip, iq, 0.0);

                    z.setComponent(ip, z.getComponent(ip) - h);
                    z.setComponent(iq, z.getComponent(iq) + h);

                    eig_val.setComponent(ip, eig_val.getComponent(ip) - h);
                    eig_val.setComponent(iq, eig_val.getComponent(iq) + h);

                    var j: u32 = 0;
                    while (j < ip) : (j += 1) evsRotate(&a, j, ip, j, iq, s, tau);
                    j = ip + 1;
                    while (j < iq) : (j += 1) evsRotate(&a, ip, j, j, iq, s, tau);
                    j = iq + 1;
                    while (j < n) : (j += 1) evsRotate(&a, ip, j, iq, j, s, tau);
                    j = 0;
                    while (j < n) : (j += 1) evsRotate(eig_vec, j, ip, j, iq, s, tau);
                }
            }
        }

        // Update eigenvalues with the sum of ta_pq and reinitialize z
        ip = 0;
        while (ip < n) : (ip += 1) {
            b.setComponent(ip, b.getComponent(ip) + z.getComponent(ip));
            eig_val.setComponent(ip, b.getComponent(ip));
            z.setComponent(ip, 0.0);
        }
    }

    // Failure
    // JPH_ASSERT(false, "Too many iterations"). Not std.debug.assert(false): that is undefined behavior in
    // ReleaseFast, while this path is reachable (e.g. NaN input) and Jolt returns false when asserts are disabled.
    if (Core.enable_asserts) @panic("Too many iterations");
    return false;
}

/// JPH_EVS_ROTATE(a, i, j, k, l). In Jolt the macro writes g and h of the enclosing function, which are not read
/// again after the rotations, so they are locals here.
fn evsRotate(a: anytype, i: u32, j: u32, k: u32, l: u32, s: f32, tau: f32) void {
    const g = a.get(i, j);
    const h = a.get(k, l);
    a.set(i, j, g - s * (h + g * tau));
    a.set(k, l, h + s * (g - h * tau));
}

fn testEigenValues(comptime n: u32, matrix: @import("Matrix.zig").Matrix(n, n), expected_eigen_values: [n]f32) !void {
    const Matrix = @import("Matrix.zig").Matrix;
    const Vector = @import("Vector.zig").Vector;

    var eigen_vec = Matrix(n, n).identity();
    var eigen_val: Vector(n) = undefined;
    try std.testing.expect(eigenValueSymmetric(matrix, &eigen_vec, &eigen_val));

    var found = [_]bool{false} ** n;
    for (0..n) |c_usize| {
        const c: u32 = @intCast(c_usize);

        // Check that we found a valid eigenvalue (each expected value only once)
        for (0..n) |e| {
            if (!found[e] and @abs(expected_eigen_values[e] - eigen_val.getComponent(c)) < 1.0e-5) {
                found[e] = true;
                break;
            }
        } else return error.TestUnexpectedEigenValue;

        // Check if the eigenvector is normalized
        try std.testing.expect(eigen_vec.getColumn(c).isNormalized(.{}));

        // Check if matrix * eigen_vector = eigen_value * eigen_vector
        const mat_eigvec = matrix.mulVector(eigen_vec.getColumn(c));
        const eigval_eigvec = eigen_vec.getColumn(c).mulScalar(eigen_val.getComponent(c));
        try std.testing.expect(mat_eigvec.isClose(eigval_eigvec, .{ .max_dist_sq = math.square(1.0e-5) }));
    }
}

test "eigenValueSymmetric" {
    const Matrix = @import("Matrix.zig").Matrix;

    // Already diagonal: converged immediately, eigenvectors stay identity
    try testEigenValues(3, Matrix(3, 3).diagonal(.{ .f32s = .{ 3, 1, 2 } }), .{ 3, 1, 2 });

    // [[2, 1], [1, 2]] has eigenvalues 1 and 3
    try testEigenValues(2, .{ .col = .{ .{ .f32s = .{ 2, 1 } }, .{ .f32s = .{ 1, 2 } } } }, .{ 1, 3 });

    // Tridiagonal [[2, -1, 0], [-1, 2, -1], [0, -1, 2]] has eigenvalues 2 - sqrt(2), 2 and 2 + sqrt(2)
    try testEigenValues(3, .{ .col = .{
        .{ .f32s = .{ 2, -1, 0 } },
        .{ .f32s = .{ -1, 2, -1 } },
        .{ .f32s = .{ 0, -1, 2 } },
    } }, .{ 2.0 - std.math.sqrt2, 2.0, 2.0 + std.math.sqrt2 });

    // Symmetric 4x4 [[4, 1, 0, 0], [1, 4, 0, 0], [0, 0, 1, 2], [0, 0, 2, 1]] has eigenvalues 3, 5, -1 and 3
    try testEigenValues(4, .{ .col = .{
        .{ .f32s = .{ 4, 1, 0, 0 } },
        .{ .f32s = .{ 1, 4, 0, 0 } },
        .{ .f32s = .{ 0, 0, 1, 2 } },
        .{ .f32s = .{ 0, 0, 2, 1 } },
    } }, .{ 3, 5, -1, 3 });
}
