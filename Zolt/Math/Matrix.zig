//! Port of: Jolt/Math/Matrix.h
//! Status: complete
//!
//! The `Matrix<2, 2>::SetInversed` specialization is a comptime branch in `setInversed`.

const std = @import("std");
const math = @import("Math.zig");
const Vector = @import("Vector.zig").Vector;
const gaussianElimination = @import("GaussianElimination.zig").gaussianElimination;

/// Templatized matrix class
pub fn Matrix(comptime rows: u32, comptime cols: u32) type {
    return extern struct {
        const Self = @This();

        /// Number of rows (the `Rows` template parameter), usable at comptime
        pub const row_count: u32 = rows;

        /// Number of columns (the `Cols` template parameter), usable at comptime
        pub const col_count: u32 = cols;

        /// Column
        col: [cols]Vector(rows),

        /// Dimensions
        pub fn getRows(self: Self) u32 {
            _ = self;
            return rows;
        }

        pub fn getCols(self: Self) u32 {
            _ = self;
            return cols;
        }

        /// Zero matrix
        pub fn setZero(self: *Self) void {
            for (0..cols) |c|
                self.col[c].setZero();
        }

        pub fn zero() Self {
            var m: Self = undefined;
            m.setZero();
            return m;
        }

        /// Check if this matrix consists of all zeros
        pub fn isZero(self: Self) bool {
            for (0..cols) |c|
                if (!self.col[c].isZero())
                    return false;

            return true;
        }

        /// Identity matrix
        pub fn setIdentity(self: *Self) void {
            // Clear matrix
            self.setZero();

            // Set diagonal to 1
            for (0..@min(rows, cols)) |rc|
                self.col[rc].f32s[rc] = 1.0;
        }

        pub fn identity() Self {
            var m: Self = undefined;
            m.setIdentity();
            return m;
        }

        /// Check if this matrix is identity
        pub fn isIdentity(self: Self) bool {
            return self.eql(identity());
        }

        /// Diagonal matrix
        pub fn setDiagonal(self: *Self, v: Vector(@min(rows, cols))) void {
            // Clear matrix
            self.setZero();

            // Set diagonal
            for (0..@min(rows, cols)) |rc|
                self.col[rc].f32s[rc] = v.f32s[rc];
        }

        pub fn diagonal(v: Vector(@min(rows, cols))) Self {
            var m: Self = undefined;
            m.setDiagonal(v);
            return m;
        }

        /// Copy a (part) of another matrix into this matrix.
        /// `m` can be any matrix type with a `get(row, col)` method.
        pub fn copyPart(self: *Self, m: anytype, source_row: u32, source_col: u32, num_rows: u32, num_cols: u32, dest_row: u32, dest_col: u32) void {
            var c: u32 = 0;
            while (c < num_cols) : (c += 1) {
                var r: u32 = 0;
                while (r < num_rows) : (r += 1)
                    self.col[dest_col + c].f32s[dest_row + r] = m.get(source_row + r, source_col + c);
            }
        }

        /// Get float component by element index (operator () const)
        pub fn get(self: Self, row: u32, column: u32) f32 {
            std.debug.assert(row < rows);
            std.debug.assert(column < cols);
            return self.col[column].f32s[row];
        }

        /// Set float component by element index (operator ())
        pub fn set(self: *Self, row: u32, column: u32, v: f32) void {
            std.debug.assert(row < rows);
            std.debug.assert(column < cols);
            self.col[column].f32s[row] = v;
        }

        /// Comparison (operator ==, use !eql for operator !=)
        pub fn eql(self: Self, m2: Self) bool {
            for (0..cols) |c|
                if (!self.col[c].eql(m2.col[c]))
                    return false;
            return true;
        }

        /// Multiply matrix by matrix (operator * (const Matrix<Cols, OtherCols> &)).
        /// `m` must be a `Matrix(cols, other_cols)`, the result is a `Matrix(rows, other_cols)`.
        pub fn mul(self: Self, m: anytype) Matrix(rows, @TypeOf(m).col_count) {
            const other_cols = @TypeOf(m).col_count;
            if (@TypeOf(m) != Matrix(cols, other_cols))
                @compileError("Matrix.mul: expected Matrix(" ++ std.fmt.comptimePrint("{d}", .{cols}) ++ ", N), got " ++ @typeName(@TypeOf(m)));

            var result: Matrix(rows, other_cols) = undefined;
            for (0..other_cols) |c| {
                for (0..rows) |r| {
                    var dot: f32 = 0.0;
                    for (0..cols) |i|
                        dot += self.col[i].f32s[r] * m.col[c].f32s[i];
                    result.col[c].f32s[r] = dot;
                }
            }
            return result;
        }

        /// Multiply vector by matrix (operator * (const Vector<Cols> &))
        pub fn mulVector(self: Self, v: Vector(cols)) Vector(rows) {
            var result: Vector(rows) = undefined;
            for (0..rows) |r| {
                var dot: f32 = 0.0;
                for (0..cols) |c|
                    dot += self.col[c].f32s[r] * v.f32s[c];
                result.f32s[r] = dot;
            }
            return result;
        }

        /// Multiply matrix with float (operator * (float) and operator * (float, Matrix))
        pub fn mulScalar(self: Self, v: f32) Self {
            var m: Self = undefined;
            for (0..cols) |c|
                m.col[c] = self.col[c].mulScalar(v);
            return m;
        }

        /// Per element addition of matrix (operator +)
        pub fn add(self: Self, m2: Self) Self {
            var m: Self = undefined;
            for (0..cols) |c|
                m.col[c] = self.col[c].add(m2.col[c]);
            return m;
        }

        /// Per element subtraction of matrix (operator -)
        pub fn sub(self: Self, m2: Self) Self {
            var m: Self = undefined;
            for (0..cols) |c|
                m.col[c] = self.col[c].sub(m2.col[c]);
            return m;
        }

        /// Transpose matrix
        pub fn transposed(self: Self) Matrix(cols, rows) {
            var m: Matrix(cols, rows) = undefined;
            for (0..rows) |r| {
                for (0..cols) |c|
                    m.col[r].f32s[c] = self.col[c].f32s[r];
            }
            return m;
        }

        /// Inverse matrix
        pub fn setInversed(self: *Self, m: Self) bool {
            if (rows != cols)
                @compileError("Matrix.setInversed requires a square matrix");

            if (rows == 2) {
                // Specialization of SetInversed for 2x2 matrix

                // Fetch elements
                const a = m.col[0].f32s[0];
                const b = m.col[1].f32s[0];
                const c = m.col[0].f32s[1];
                const d = m.col[1].f32s[1];

                // Calculate determinant
                const det = math.differenceOfProducts(a, d, b, c);
                if (det == 0.0)
                    return false;

                // Construct inverse
                self.col[0].f32s[0] = d / det;
                self.col[1].f32s[0] = -b / det;
                self.col[0].f32s[1] = -c / det;
                self.col[1].f32s[1] = a / det;
                return true;
            } else {
                var copy = m;
                self.setIdentity();
                return gaussianElimination(&copy, self, .{});
            }
        }

        pub fn inversed(self: Self) Self {
            var m: Self = undefined;
            _ = m.setInversed(self);
            return m;
        }

        /// To String
        pub fn format(self: Self, writer: *std.Io.Writer) std.Io.Writer.Error!void {
            for (0..cols - 1) |i|
                try writer.print("{f}, ", .{self.col[i]});
            try writer.print("{f}", .{self.col[cols - 1]});
        }

        /// Column access
        pub fn getColumn(self: Self, idx: u32) Vector(rows) {
            return self.col[idx];
        }

        /// Column access, returns a pointer to the column that can be modified (non const GetColumn)
        pub fn getColumnPtr(self: *Self, idx: u32) *Vector(rows) {
            return &self.col[idx];
        }
    };
}

test "Matrix size and layout" {
    // Matrix is stored as an array of column vectors, without padding
    try std.testing.expectEqual(6 * @sizeOf(f32), @sizeOf(Matrix(2, 3)));
}

test "Matrix functions" {
    // Generic functions are only type checked when instantiated, so call every function here
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;
    const DynMatrix = @import("DynMatrix.zig").DynMatrix;

    const M = Matrix(2, 3);
    var m = M.zero();
    try expectEqual(2, M.row_count);
    try expectEqual(3, M.col_count);
    try expectEqual(2, m.getRows());
    try expectEqual(3, m.getCols());
    try expect(m.isZero());
    try expect(!m.isIdentity());
    m.setIdentity();
    try expect(m.isIdentity());
    try expect(!m.isZero());

    // Diagonal
    const d = M.diagonal(.{ .f32s = .{ 2, 3 } });
    try expect(d.eql(.{ .col = .{ .{ .f32s = .{ 2, 0 } }, .{ .f32s = .{ 0, 3 } }, .{ .f32s = .{ 0, 0 } } } }));

    // m = [[1, 2, 3], [4, 5, 6]]
    for (0..2) |r|
        for (0..3) |c|
            m.set(@intCast(r), @intCast(c), @floatFromInt(r * 3 + c + 1));
    try expectEqual(6.0, m.get(1, 2));
    try expect(m.getColumn(1).eql(.{ .f32s = .{ 2, 5 } }));

    // Transpose
    const t: Matrix(3, 2) = m.transposed();
    for (0..2) |r|
        for (0..3) |c|
            try expectEqual(m.get(@intCast(r), @intCast(c)), t.get(@intCast(c), @intCast(r)));

    // Matrix * vector
    const mv = m.mulVector(.{ .f32s = .{ 1, 1, 1 } });
    try expect(mv.eql(.{ .f32s = .{ 6, 15 } }));

    // Element wise operations
    try expect(m.add(m).eql(m.mulScalar(2.0)));
    try expect(m.sub(m).isZero());

    // Column pointer
    m.getColumnPtr(2).setComponent(0, 10);
    try expectEqual(10.0, m.get(0, 2));

    // Copy part of another Matrix and of a DynMatrix
    var big = Matrix(4, 4).zero();
    big.copyPart(m, 0, 1, 2, 2, 1, 2); // big(1..2, 2..3) = m(0..1, 1..2)
    try expectEqual(2.0, big.get(1, 2));
    try expectEqual(10.0, big.get(1, 3));
    try expectEqual(5.0, big.get(2, 2));
    try expectEqual(6.0, big.get(2, 3));
    try expectEqual(0.0, big.get(0, 0));

    var dyn = try DynMatrix.init(std.testing.allocator, 1, 2);
    defer dyn.deinit();
    dyn.set(0, 0, 7);
    dyn.set(0, 1, 8);
    big.copyPart(&dyn, 0, 0, 1, 2, 3, 0);
    try expectEqual(7.0, big.get(3, 0));
    try expectEqual(8.0, big.get(3, 1));
}

test "Matrix inverse" {
    const expect = std.testing.expect;

    // 2x2 uses the specialization, 3x3 Gaussian elimination
    const m2: Matrix(2, 2) = .{ .col = .{ .{ .f32s = .{ 4, 2 } }, .{ .f32s = .{ 7, 6 } } } };
    const inv2 = m2.inversed();
    try expect(inv2.eql(.{ .col = .{ .{ .f32s = .{ 0.6, -0.2 } }, .{ .f32s = .{ -0.7, 0.4 } } } }));

    // Upper triangular matrix with an exact inverse
    var m3 = Matrix(3, 3).identity();
    m3.set(0, 1, 2);
    m3.set(1, 2, 4);
    m3.set(2, 2, 2);
    const inv3 = m3.inversed();
    try expect(m3.mul(inv3).isIdentity());
    try expect(inv3.mul(m3).isIdentity());

    // Singular
    var inv: Matrix(3, 3) = undefined;
    try expect(!inv.setInversed(Matrix(3, 3).zero()));
}
