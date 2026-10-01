//! Zolt: a Zig port of Jolt Physics (https://github.com/jrouwe/JoltPhysics).
//!
//! This file is the root of the `zolt` module. It re-exports the public API in a flat namespace,
//! mirroring the C++ `JPH::` namespace (so `JPH::Vec3` is `zolt.Vec3`). Source files inside the
//! module import each other through relative paths, never through this file.
//!
//! See Docs/Zolt/PortingGuide.md for the C++ -> Zig conventions and Docs/Zolt/Roadmap.md for the plan.

const std = @import("std");

// Core
pub const atomicMax = @import("Core/Atomics.zig").atomicMax;
pub const atomicMin = @import("Core/Atomics.zig").atomicMin;
pub const binaryHeapPop = @import("Core/BinaryHeap.zig").binaryHeapPop;
pub const binaryHeapPush = @import("Core/BinaryHeap.zig").binaryHeapPush;
pub const Color = @import("Core/Color.zig").Color;
pub const Core = @import("Core/Core.zig");
pub const HashCombine = @import("Core/HashCombine.zig");
pub const insertionSort = @import("Core/InsertionSort.zig").insertionSort;
pub const Mt19937 = @import("Core/Mt19937.zig");
pub const prefetchL1 = @import("Core/Prefetch.zig").prefetchL1;
pub const quickSort = @import("Core/QuickSort.zig").quickSort;
pub const Ref = @import("Core/Reference.zig").Ref;
pub const RefConst = @import("Core/Reference.zig").RefConst;
pub const RefCount = @import("Core/Reference.zig").RefCount;
pub const RefTargetVirtual = @import("Core/Reference.zig").RefTargetVirtual;
pub const StaticArray = @import("Core/StaticArray.zig").StaticArray;

// Math
pub const BVec16 = @import("Math/BVec16.zig").BVec16;
pub const Double3 = @import("Math/Double3.zig").Double3;
pub const DVec3 = @import("Math/DVec3.zig").DVec3;
pub const DynMatrix = @import("Math/DynMatrix.zig").DynMatrix;
pub const eigenValueSymmetric = @import("Math/EigenValueSymmetric.zig").eigenValueSymmetric;
pub const findRoot = @import("Math/FindRoot.zig").findRoot;
pub const FindRootResult = @import("Math/FindRoot.zig").FindRootResult;
pub const Float2 = @import("Math/Float2.zig").Float2;
pub const Float3 = @import("Math/Float3.zig").Float3;
pub const VertexList = @import("Math/Float3.zig").VertexList;
pub const Float4 = @import("Math/Float4.zig").Float4;
pub const gaussianElimination = @import("Math/GaussianElimination.zig").gaussianElimination;
pub const half_float = @import("Math/HalfFloat.zig");
pub const HalfFloat = @import("Math/HalfFloat.zig").HalfFloat;
pub const math = @import("Math/Math.zig");
pub const Matrix = @import("Math/Matrix.zig").Matrix;
pub const Swizzle = @import("Math/Swizzle.zig").Swizzle;
pub const trigonometry = @import("Math/Trigonometry.zig");
pub const Mat44 = @import("Math/Mat44.zig").Mat44;
pub const Quat = @import("Math/Quat.zig").Quat;
pub const UVec4 = @import("Math/UVec4.zig").UVec4;
pub const Vec3 = @import("Math/Vec3.zig").Vec3;
pub const Vec4 = @import("Math/Vec4.zig").Vec4;
pub const Vector = @import("Math/Vector.zig").Vector;

/// Every source file of the module. Used by the test below to make sure that all of them are
/// compiled and that their inline tests run. Add new files here when porting them.
const source_files = .{
    @import("Core/Atomics.zig"),
    @import("Core/BinaryHeap.zig"),
    @import("Core/Color.zig"),
    @import("Core/Core.zig"),
    @import("Core/HashCombine.zig"),
    @import("Core/InsertionSort.zig"),
    @import("Core/Mt19937.zig"),
    @import("Core/Prefetch.zig"),
    @import("Core/QuickSort.zig"),
    @import("Core/Reference.zig"),
    @import("Core/StaticArray.zig"),
    @import("Math/BVec16.zig"),
    @import("Math/Double3.zig"),
    @import("Math/DVec3.zig"),
    @import("Math/DynMatrix.zig"),
    @import("Math/EigenValueSymmetric.zig"),
    @import("Math/FindRoot.zig"),
    @import("Math/Float2.zig"),
    @import("Math/Float3.zig"),
    @import("Math/Float4.zig"),
    @import("Math/GaussianElimination.zig"),
    @import("Math/HalfFloat.zig"),
    @import("Math/Math.zig"),
    @import("Math/Matrix.zig"),
    @import("Math/Mat44.zig"),
    @import("Math/Math.zig"),
    @import("Math/Quat.zig"),
    @import("Math/Swizzle.zig"),
    @import("Math/Trigonometry.zig"),
    @import("Math/UVec4.zig"),
    @import("Math/Vec3.zig"),
    @import("Math/Vec4.zig"),
    @import("Math/Vector.zig"),
};

test {
    // Zig only analyzes code that is referenced. Reference every public declaration (recursively
    // into nested types) so that ported functions that are not called by any test yet still get
    // type checked. Generic functions (comptime / anytype parameters) can only be checked by calling them.
    inline for (source_files) |file| {
        refAllDeclsRecursive(file);
    }
}

fn refAllDeclsRecursive(comptime T: type) void {
    @setEvalBranchQuota(100_000);
    inline for (comptime std.meta.declarations(T)) |decl| {
        const value = @field(T, decl.name);
        if (@TypeOf(value) == type) {
            switch (@typeInfo(value)) {
                .@"struct", .@"enum", .@"union", .@"opaque" => {
                    // Only recurse into types declared inside T, aliases to other types are checked by their own file
                    if (comptime std.mem.startsWith(u8, @typeName(value), @typeName(T) ++ "."))
                        refAllDeclsRecursive(value);
                },
                else => {},
            }
        }
        _ = &@field(T, decl.name);
    }
}
