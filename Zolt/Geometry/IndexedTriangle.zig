//! Port of: Jolt/Geometry/IndexedTriangle.h
//! Status: complete
//!
//! `IndexedTriangle` derives from `IndexedTriangleNoMaterial` in Jolt. In Zolt it is a flat struct with the same
//! layout (`idx`, then `material_index` and `user_data`) that declares the inherited methods too. Where Jolt passes
//! an `IndexedTriangle` as a `const IndexedTriangleNoMaterial &` (slicing), use `toNoMaterial()`.
//!
//! The `VertexList` arguments (Jolt: `const VertexList &`) are slices of Float3 (`list.items`).
//! JPH_MAKE_STD_HASH: `HashCombine.hash(triangle)` calls `getHash()`, like `Hash<IndexedTriangle>` in Jolt.

const std = @import("std");
const HashCombine = @import("../Core/HashCombine.zig");
const Float3 = @import("../Math/Float3.zig").Float3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// Array<Float3>, defined in Math/Float3.zig like in Jolt, re-exported for convenience
pub const VertexList = @import("../Math/Float3.zig").VertexList;

/// Triangle with 32-bit indices
pub const IndexedTriangleNoMaterial = extern struct {
    idx: [3]u32,

    comptime {
        // Class should have no padding
        std.debug.assert(@sizeOf(IndexedTriangleNoMaterial) == 3 * @sizeOf(u32));
    }

    /// Constructor
    pub fn init(idx1: u32, idx2: u32, idx3: u32) IndexedTriangleNoMaterial {
        return .{ .idx = .{ idx1, idx2, idx3 } };
    }

    /// Check if two triangles are identical (operator ==)
    pub fn eql(self: IndexedTriangleNoMaterial, rhs: IndexedTriangleNoMaterial) bool {
        return self.idx[0] == rhs.idx[0] and self.idx[1] == rhs.idx[1] and self.idx[2] == rhs.idx[2];
    }

    /// Check if two triangles are equivalent (using the same vertices)
    pub fn isEquivalent(self: IndexedTriangleNoMaterial, rhs: IndexedTriangleNoMaterial) bool {
        return (self.idx[0] == rhs.idx[0] and self.idx[1] == rhs.idx[1] and self.idx[2] == rhs.idx[2]) or
            (self.idx[0] == rhs.idx[1] and self.idx[1] == rhs.idx[2] and self.idx[2] == rhs.idx[0]) or
            (self.idx[0] == rhs.idx[2] and self.idx[1] == rhs.idx[0] and self.idx[2] == rhs.idx[1]);
    }

    /// Check if two triangles are opposite (using the same vertices but in opposing order)
    pub fn isOpposite(self: IndexedTriangleNoMaterial, rhs: IndexedTriangleNoMaterial) bool {
        return (self.idx[0] == rhs.idx[0] and self.idx[1] == rhs.idx[2] and self.idx[2] == rhs.idx[1]) or
            (self.idx[0] == rhs.idx[1] and self.idx[1] == rhs.idx[0] and self.idx[2] == rhs.idx[2]) or
            (self.idx[0] == rhs.idx[2] and self.idx[1] == rhs.idx[1] and self.idx[2] == rhs.idx[0]);
    }

    /// Check if triangle is degenerate
    pub fn isDegenerate(self: IndexedTriangleNoMaterial, vertices: []const Float3) bool {
        const v0 = Vec3.fromFloat3(vertices[self.idx[0]]);
        const v1 = Vec3.fromFloat3(vertices[self.idx[1]]);
        const v2 = Vec3.fromFloat3(vertices[self.idx[2]]);

        return v1.sub(v0).cross(v2.sub(v0)).isNearZero(.{});
    }

    /// Rotate the vertices so that the second vertex becomes first etc. This does not change the represented triangle.
    pub fn rotate(self: *IndexedTriangleNoMaterial) void {
        const tmp = self.idx[0];
        self.idx[0] = self.idx[1];
        self.idx[1] = self.idx[2];
        self.idx[2] = tmp;
    }

    /// Get center of triangle
    pub fn getCentroid(self: IndexedTriangleNoMaterial, vertices: []const Float3) Vec3 {
        return Vec3.fromFloat3(vertices[self.idx[0]]).add(Vec3.fromFloat3(vertices[self.idx[1]])).add(Vec3.fromFloat3(vertices[self.idx[2]])).divScalar(3.0);
    }

    /// Get the hash value of this structure
    pub fn getHash(self: IndexedTriangleNoMaterial) u64 {
        return HashCombine.hashBytes(std.mem.asBytes(&self));
    }
};

/// Triangle with 32-bit indices and material index
pub const IndexedTriangle = extern struct {
    idx: [3]u32,
    material_index: u32 = 0,
    /// User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
    user_data: u32 = 0,

    comptime {
        // Class should have no padding
        std.debug.assert(@sizeOf(IndexedTriangle) == 5 * @sizeOf(u32));
    }

    /// Optional arguments of `init`
    pub const Options = struct {
        material_index: u32 = 0,
        user_data: u32 = 0,
    };

    /// Constructor. Covers both the constructor inherited from IndexedTriangleNoMaterial (`IndexedTriangle(i1, i2, i3)`,
    /// material index 0) and `IndexedTriangle(i1, i2, i3, inMaterialIndex, inUserData = 0)`.
    pub fn init(idx1: u32, idx2: u32, idx3: u32, opts: Options) IndexedTriangle {
        return .{ .idx = .{ idx1, idx2, idx3 }, .material_index = opts.material_index, .user_data = opts.user_data };
    }

    /// The IndexedTriangleNoMaterial part of this triangle (C++ derived to base conversion)
    pub fn toNoMaterial(self: IndexedTriangle) IndexedTriangleNoMaterial {
        return .{ .idx = self.idx };
    }

    /// Check if two triangles are identical (operator ==)
    pub fn eql(self: IndexedTriangle, rhs: IndexedTriangle) bool {
        return self.material_index == rhs.material_index and self.user_data == rhs.user_data and self.toNoMaterial().eql(rhs.toNoMaterial());
    }

    /// Check if two triangles are equivalent (using the same vertices), see IndexedTriangleNoMaterial.isEquivalent
    pub fn isEquivalent(self: IndexedTriangle, rhs: IndexedTriangleNoMaterial) bool {
        return self.toNoMaterial().isEquivalent(rhs);
    }

    /// Check if two triangles are opposite (using the same vertices but in opposing order), see IndexedTriangleNoMaterial.isOpposite
    pub fn isOpposite(self: IndexedTriangle, rhs: IndexedTriangleNoMaterial) bool {
        return self.toNoMaterial().isOpposite(rhs);
    }

    /// Check if triangle is degenerate, see IndexedTriangleNoMaterial.isDegenerate
    pub fn isDegenerate(self: IndexedTriangle, vertices: []const Float3) bool {
        return self.toNoMaterial().isDegenerate(vertices);
    }

    /// Rotate the vertices so that the second vertex becomes first etc. This does not change the represented triangle.
    pub fn rotate(self: *IndexedTriangle) void {
        var base = self.toNoMaterial();
        base.rotate();
        self.idx = base.idx;
    }

    /// Get center of triangle, see IndexedTriangleNoMaterial.getCentroid
    pub fn getCentroid(self: IndexedTriangle, vertices: []const Float3) Vec3 {
        return self.toNoMaterial().getCentroid(vertices);
    }

    /// Rotate the vertices so that the lowest vertex becomes the first. This does not change the represented triangle.
    pub fn getLowestIndexFirst(self: IndexedTriangle) IndexedTriangle {
        const opts: Options = .{ .material_index = self.material_index, .user_data = self.user_data };
        if (self.idx[0] < self.idx[1]) {
            if (self.idx[0] < self.idx[2])
                return init(self.idx[0], self.idx[1], self.idx[2], opts) // 0 is smallest
            else
                return init(self.idx[2], self.idx[0], self.idx[1], opts); // 2 is smallest
        } else {
            if (self.idx[1] < self.idx[2])
                return init(self.idx[1], self.idx[2], self.idx[0], opts) // 1 is smallest
            else
                return init(self.idx[2], self.idx[0], self.idx[1], opts); // 2 is smallest
        }
    }

    /// Get the hash value of this structure
    pub fn getHash(self: IndexedTriangle) u64 {
        return HashCombine.hashBytes(std.mem.asBytes(&self));
    }
};

/// Array<IndexedTriangleNoMaterial>, use with an explicit allocator
pub const IndexedTriangleNoMaterialList = std.ArrayList(IndexedTriangleNoMaterial);

/// Array<IndexedTriangle>, use with an explicit allocator
pub const IndexedTriangleList = std.ArrayList(IndexedTriangle);

test "IndexedTriangleNoMaterial" {
    const t = IndexedTriangleNoMaterial.init(1, 2, 3);
    try std.testing.expect(t.eql(.init(1, 2, 3)));
    try std.testing.expect(!t.eql(.init(2, 3, 1)));
    try std.testing.expect(t.isEquivalent(.init(1, 2, 3)));
    try std.testing.expect(t.isEquivalent(.init(2, 3, 1)));
    try std.testing.expect(t.isEquivalent(.init(3, 1, 2)));
    try std.testing.expect(!t.isEquivalent(.init(1, 3, 2)));
    try std.testing.expect(t.isOpposite(.init(1, 3, 2)));
    try std.testing.expect(t.isOpposite(.init(2, 1, 3)));
    try std.testing.expect(t.isOpposite(.init(3, 2, 1)));
    try std.testing.expect(!t.isOpposite(.init(1, 2, 3)));

    var r = t;
    r.rotate();
    try std.testing.expect(r.eql(.init(2, 3, 1)));

    const vertices = [_]Float3{ .init(0, 0, 0), .init(3, 0, 0), .init(0, 3, 0), .init(0, 0, 3), .init(6, 0, 0) };
    try std.testing.expect(!t.isDegenerate(&vertices));
    try std.testing.expect(IndexedTriangleNoMaterial.init(0, 1, 4).isDegenerate(&vertices));
    try std.testing.expect(IndexedTriangleNoMaterial.init(1, 1, 2).isDegenerate(&vertices));
    try std.testing.expect(t.getCentroid(&vertices).isClose(Vec3.init(1, 1, 1), .{}));

    // The hash is FNV-1a over the 12 bytes of the structure
    try std.testing.expectEqual(HashCombine.hashBytes(std.mem.asBytes(&[3]u32{ 1, 2, 3 })), t.getHash());
    try std.testing.expectEqual(t.getHash(), HashCombine.hash(t));
    try std.testing.expect(t.getHash() != r.getHash());
}

test "IndexedTriangle" {
    const t = IndexedTriangle.init(5, 2, 3, .{ .material_index = 7, .user_data = 8 });
    try std.testing.expect(t.eql(.init(5, 2, 3, .{ .material_index = 7, .user_data = 8 })));
    try std.testing.expect(!t.eql(.init(5, 2, 3, .{ .material_index = 7 })));
    try std.testing.expect(!t.eql(.init(5, 2, 3, .{ .user_data = 8 })));
    try std.testing.expect(!t.eql(.init(5, 2, 4, .{ .material_index = 7, .user_data = 8 })));
    try std.testing.expectEqual(@as(u32, 0), IndexedTriangle.init(1, 2, 3, .{}).material_index);

    try std.testing.expect(t.isEquivalent(.init(2, 3, 5)));
    try std.testing.expect(t.isOpposite(.init(5, 3, 2)));
    try std.testing.expect(t.isEquivalent(IndexedTriangle.init(3, 5, 2, .{}).toNoMaterial()));

    // Lowest index first keeps the winding order and the material
    try std.testing.expect(t.getLowestIndexFirst().eql(.init(2, 3, 5, .{ .material_index = 7, .user_data = 8 })));
    try std.testing.expect(IndexedTriangle.init(1, 2, 3, .{}).getLowestIndexFirst().eql(.init(1, 2, 3, .{})));
    try std.testing.expect(IndexedTriangle.init(2, 3, 1, .{}).getLowestIndexFirst().eql(.init(1, 2, 3, .{})));
    try std.testing.expect(IndexedTriangle.init(3, 1, 2, .{}).getLowestIndexFirst().eql(.init(1, 2, 3, .{})));
    try std.testing.expect(IndexedTriangle.init(2, 1, 3, .{}).getLowestIndexFirst().eql(.init(1, 3, 2, .{})));

    var r = t;
    r.rotate();
    try std.testing.expect(r.eql(.init(2, 3, 5, .{ .material_index = 7, .user_data = 8 })));

    const vertices = [_]Float3{ .init(0, 0, 0), .init(3, 0, 0), .init(0, 3, 0), .init(0, 0, 3), .init(6, 0, 0), .init(0, 0, 0) };
    try std.testing.expect(!IndexedTriangle.init(1, 2, 3, .{}).isDegenerate(&vertices));
    try std.testing.expect(IndexedTriangle.init(0, 1, 4, .{}).isDegenerate(&vertices));
    try std.testing.expect(IndexedTriangle.init(1, 2, 3, .{}).getCentroid(&vertices).isClose(Vec3.init(1, 1, 1), .{}));

    // The hash is FNV-1a over the 20 bytes of the structure
    try std.testing.expectEqual(HashCombine.hashBytes(std.mem.asBytes(&[5]u32{ 5, 2, 3, 7, 8 })), t.getHash());
    try std.testing.expectEqual(t.getHash(), HashCombine.hash(t));
}
