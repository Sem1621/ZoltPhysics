//! Port of: Jolt/Math/GaussianElimination.h
//! Status: complete

const std = @import("std");

/// Maximum dimension N of a runtime sized matrix (e.g. DynMatrix) passed to `gaussianElimination`.
/// Jolt allocates the pivot bookkeeping array on the stack with JPH_STACK_ALLOC (alloca), which Zig
/// doesn't have. Matrices with a comptime known size (Matrix) use an array of exactly N elements,
/// runtime sized matrices use a fixed size stack buffer of this many elements.
pub const max_dynamic_size: u32 = 256;

/// This function performs Gauss-Jordan elimination to solve a matrix equation.
/// A must be an NxN matrix and B must be an NxM matrix forming the equation A * x = B
/// on output B will contain x and A will be destroyed.
///
/// This code can be used for example to compute the inverse of a matrix.
/// Set A to the matrix to invert, set B to identity and let GaussianElimination solve
/// the equation, on return B will be the inverse of A. And A is destroyed.
///
/// Taken and adapted from Numerical Recipes in C paragraph 2.1
///
/// `a` and `b` are pointers to matrices (Matrix, DynMatrix) that provide `getRows()`, `getCols()`,
/// `get(row, col)` and `set(row, col, value)`.
pub fn gaussianElimination(a: anytype, b: anytype, opts: struct { tolerance: f32 = 1.0e-16 }) bool {
    const MatrixA = @typeInfo(@TypeOf(a)).pointer.child;

    // Get problem dimensions
    const n: u32 = a.getCols();
    const m: u32 = b.getCols();

    // Check matrix requirement
    std.debug.assert(a.getRows() == n);
    std.debug.assert(b.getRows() == n);

    // Create array for bookkeeping on pivoting
    const max_n: u32 = if (@hasDecl(MatrixA, "col_count")) MatrixA.col_count else max_dynamic_size;
    std.debug.assert(n <= max_n);
    var ipiv_buffer: [max_n]i32 = undefined;
    const ipiv = ipiv_buffer[0..n];
    @memset(ipiv, 0);

    var i: u32 = 0;
    while (i < n) : (i += 1) {
        // Initialize pivot element as the diagonal
        var pivot_row: u32 = i;
        var pivot_col: u32 = i;

        // Determine pivot element
        var largest_element: f32 = 0.0;
        var j: u32 = 0;
        while (j < n) : (j += 1) {
            if (ipiv[j] != 1) {
                var k: u32 = 0;
                while (k < n) : (k += 1) {
                    if (ipiv[k] == 0) {
                        const element = @abs(a.get(j, k));
                        if (element >= largest_element) {
                            largest_element = element;
                            pivot_row = j;
                            pivot_col = k;
                        }
                    } else if (ipiv[k] > 1) {
                        return false;
                    }
                }
            }
        }

        // Mark this column as used
        ipiv[pivot_col] += 1;

        // Exchange rows when needed so that the pivot element is at ioA(pivot_col, pivot_col) instead of at ioA(pivot_row, pivot_col)
        if (pivot_row != pivot_col) {
            j = 0;
            while (j < n) : (j += 1) {
                const tmp = a.get(pivot_row, j);
                a.set(pivot_row, j, a.get(pivot_col, j));
                a.set(pivot_col, j, tmp);
            }
            j = 0;
            while (j < m) : (j += 1) {
                const tmp = b.get(pivot_row, j);
                b.set(pivot_row, j, b.get(pivot_col, j));
                b.set(pivot_col, j, tmp);
            }
        }

        // Get diagonal element that we are about to set to 1
        const diagonal_element = a.get(pivot_col, pivot_col);
        if (@abs(diagonal_element) < opts.tolerance)
            return false;

        // Divide the whole row by the pivot element, making ioA(pivot_col, pivot_col) = 1
        j = 0;
        while (j < n) : (j += 1)
            a.set(pivot_col, j, a.get(pivot_col, j) / diagonal_element);
        j = 0;
        while (j < m) : (j += 1)
            b.set(pivot_col, j, b.get(pivot_col, j) / diagonal_element);
        a.set(pivot_col, pivot_col, 1.0);

        // Next reduce the rows, except for the pivot one,
        // after this step the pivot_col column is zero except for the pivot element which is 1
        j = 0;
        while (j < n) : (j += 1) {
            if (j != pivot_col) {
                const element = a.get(j, pivot_col);
                var k: u32 = 0;
                while (k < n) : (k += 1)
                    a.set(j, k, a.get(j, k) - a.get(pivot_col, k) * element);
                k = 0;
                while (k < m) : (k += 1)
                    b.set(j, k, b.get(j, k) - b.get(pivot_col, k) * element);
                a.set(j, pivot_col, 0.0);
            }
        }
    }

    // Success
    return true;
}

test "gaussianElimination" {
    const expect = std.testing.expect;
    const expectApproxEqAbs = std.testing.expectApproxEqAbs;
    const DynMatrix = @import("DynMatrix.zig").DynMatrix;
    const Matrix = @import("Matrix.zig").Matrix;

    // Solve: 2x + y - z = 8, -3x - y + 2z = -11, -2x + y + 2z = -3 => x = 2, y = 3, z = -1
    const a_values = [3][3]f32{ .{ 2, 1, -1 }, .{ -3, -1, 2 }, .{ -2, 1, 2 } };
    const b_values = [3]f32{ 8, -11, -3 };
    const x_values = [3]f32{ 2, 3, -1 };

    // With DynMatrix (runtime size)
    {
        var a = try DynMatrix.init(std.testing.allocator, 3, 3);
        defer a.deinit();
        var b = try DynMatrix.init(std.testing.allocator, 3, 1);
        defer b.deinit();
        for (0..3) |r| {
            for (0..3) |c|
                a.set(@intCast(r), @intCast(c), a_values[r][c]);
            b.set(@intCast(r), 0, b_values[r]);
        }
        try expect(gaussianElimination(&a, &b, .{}));
        for (0..3) |r|
            try expectApproxEqAbs(x_values[r], b.get(@intCast(r), 0), 1.0e-5);
    }

    // With Matrix (comptime size), mixed with a DynMatrix for B
    {
        var a = Matrix(3, 3).zero();
        var b = try DynMatrix.init(std.testing.allocator, 3, 1);
        defer b.deinit();
        for (0..3) |r| {
            for (0..3) |c|
                a.set(@intCast(r), @intCast(c), a_values[r][c]);
            b.set(@intCast(r), 0, b_values[r]);
        }
        try expect(gaussianElimination(&a, &b, .{}));
        for (0..3) |r|
            try expectApproxEqAbs(x_values[r], b.get(@intCast(r), 0), 1.0e-5);
    }

    // Singular matrix
    {
        var a = Matrix(2, 2).zero();
        a.set(0, 0, 1);
        a.set(0, 1, 2);
        a.set(1, 0, 2);
        a.set(1, 1, 4);
        var b = Matrix(2, 2).identity();
        try expect(!gaussianElimination(&a, &b, .{}));
    }

    // Tolerance: a diagonal element of 1.0e-3 is accepted by default and rejected with a tolerance of 1.0e-2
    {
        var a = Matrix(2, 2).identity();
        a.set(1, 1, 1.0e-3);
        var b = Matrix(2, 2).identity();
        var a_copy = a;
        var b_copy = b;
        try expect(gaussianElimination(&a_copy, &b_copy, .{}));
        try expect(!gaussianElimination(&a, &b, .{ .tolerance = 1.0e-2 }));
    }
}
