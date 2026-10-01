//! Port of: Jolt/Math/DynMatrix.h
//! Status: complete
//!
//! Jolt's `Array<float>` is only used as a fixed size buffer here, so the elements are a slice
//! allocated in `init`. Like Jolt's `Array::resize`, the elements are not initialized.

const std = @import("std");

/// Dynamic resizable matrix class
pub const DynMatrix = struct {
    rows: u32,
    cols: u32,
    elements: []f32,
    allocator: std.mem.Allocator,

    /// Constructor (DynMatrix(uint inRows, uint inCols)). The elements are undefined.
    pub fn init(allocator: std.mem.Allocator, rows: u32, cols: u32) !DynMatrix {
        return .{
            .rows = rows,
            .cols = cols,
            .elements = try allocator.alloc(f32, @as(usize, rows) * cols),
            .allocator = allocator,
        };
    }

    /// Copy constructor (DynMatrix(const DynMatrix &)), the copy uses the same allocator
    pub fn clone(self: *const DynMatrix) !DynMatrix {
        return .{
            .rows = self.rows,
            .cols = self.cols,
            .elements = try self.allocator.dupe(f32, self.elements),
            .allocator = self.allocator,
        };
    }

    pub fn deinit(self: *DynMatrix) void {
        self.allocator.free(self.elements);
        self.* = undefined;
    }

    /// Access an element (operator () const)
    pub fn get(self: *const DynMatrix, row: u32, col: u32) f32 {
        std.debug.assert(row < self.rows and col < self.cols);
        return self.elements[row * self.cols + col];
    }

    /// Access an element (operator ())
    pub fn set(self: *DynMatrix, row: u32, col: u32, v: f32) void {
        std.debug.assert(row < self.rows and col < self.cols);
        self.elements[row * self.cols + col] = v;
    }

    /// Get dimensions
    pub fn getCols(self: *const DynMatrix) u32 {
        return self.cols;
    }

    pub fn getRows(self: *const DynMatrix) u32 {
        return self.rows;
    }
};

test "DynMatrix" {
    var m = try DynMatrix.init(std.testing.allocator, 2, 3);
    defer m.deinit();
    try std.testing.expectEqual(@as(u32, 2), m.getRows());
    try std.testing.expectEqual(@as(u32, 3), m.getCols());
    m.set(1, 2, 5.0);
    m.set(0, 1, 3.0);
    try std.testing.expectEqual(@as(f32, 5.0), m.get(1, 2));
    try std.testing.expectEqual(@as(f32, 3.0), m.get(0, 1));

    // Row major storage, like Jolt
    try std.testing.expectEqual(@as(f32, 5.0), m.elements[1 * 3 + 2]);

    var copy = try m.clone();
    defer copy.deinit();
    m.set(1, 2, 6.0);
    try std.testing.expectEqual(@as(f32, 5.0), copy.get(1, 2));
}
