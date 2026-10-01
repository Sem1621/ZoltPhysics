//! Port of: UnitTests/ (the Jolt unit test suite), root of the `zolt-unit-tests` test binary.
//!
//! Each file mirrors a file in Jolt's UnitTests/ directory, e.g. UnitTests/Math/Vec3Tests.cpp ->
//! ZoltTests/Math/Vec3Tests.zig. Tests only use the public API through @import("zolt").
//! Add new test files to the list below (keep it sorted by path).

test {
    _ = @import("UnitTestFramework.zig");

    // Core
    _ = @import("Core/BinaryHeapTest.zig");
    _ = @import("Core/HashCombineTest.zig");
    _ = @import("Core/InsertionSortTest.zig");
    _ = @import("Core/QuickSortTest.zig");

    // Math
    _ = @import("Math/BVec16Tests.zig");
    _ = @import("Math/DVec3Tests.zig");
    _ = @import("Math/HalfFloatTests.zig");
    _ = @import("Math/MathTests.zig");
    _ = @import("Math/MatrixTests.zig");
    _ = @import("Math/TrigonometryTests.zig");
    _ = @import("Math/UVec4Tests.zig");
    _ = @import("Math/Vec3Tests.zig");
    _ = @import("Math/Vec4Tests.zig");
    _ = @import("Math/VectorTests.zig");
}
