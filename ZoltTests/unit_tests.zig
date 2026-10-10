//! Port of: UnitTests/ (the Jolt unit test suite), root of the `zolt-unit-tests` test binary.
//!
//! Each file mirrors a file in Jolt's UnitTests/ directory, e.g. UnitTests/Math/Vec3Tests.cpp ->
//! ZoltTests/Math/Vec3Tests.zig. Tests only use the public API through @import("zolt").
//! Add new test files to the list below (keep it sorted by path).

test {
    _ = @import("Layers.zig");
    _ = @import("UnitTestFramework.zig");

    // Core
    _ = @import("Core/BinaryHeapTest.zig");
    _ = @import("Core/FPFlushDenormalsTest.zig");
    _ = @import("Core/HashCombineTest.zig");
    _ = @import("Core/InsertionSortTest.zig");
    _ = @import("Core/JobSystemTest.zig");
    _ = @import("Core/LinearCurveTest.zig");
    _ = @import("Core/QuickSortTest.zig");
    _ = @import("Core/STLLocalAllocatorTest.zig");
    _ = @import("Core/StringToolsTest.zig");
    _ = @import("Core/UnorderedMapTest.zig");
    _ = @import("Core/UnorderedSetTest.zig");

    // Geometry
    _ = @import("Geometry/ClosestPointTests.zig");
    _ = @import("Geometry/ConvexHullBuilderTest.zig");
    _ = @import("Geometry/EllipseTest.zig");
    _ = @import("Geometry/EPATests.zig");
    _ = @import("Geometry/GJKTests.zig");
    _ = @import("Geometry/PlaneTests.zig");
    _ = @import("Geometry/RayAABoxTests.zig");

    // Math
    _ = @import("Math/BVec16Tests.zig");
    _ = @import("Math/DMat44Tests.zig");
    _ = @import("Math/DVec3Tests.zig");
    _ = @import("Math/EigenValueSymmetricTests.zig");
    _ = @import("Math/HalfFloatTests.zig");
    _ = @import("Math/Mat44Tests.zig");
    _ = @import("Math/MathTests.zig");
    _ = @import("Math/MatrixTests.zig");
    _ = @import("Math/QuatTests.zig");
    _ = @import("Math/TrigonometryTests.zig");
    _ = @import("Math/UVec4Tests.zig");
    _ = @import("Math/Vec3Tests.zig");
    _ = @import("Math/Vec4Tests.zig");
    _ = @import("Math/VectorTests.zig");

    // Physics
    _ = @import("Physics/ActiveEdgesTests.zig");
    _ = @import("Physics/CastShapeTests.zig");
    _ = @import("Physics/CollidePointTests.zig");
    _ = @import("Physics/CollideShapeTests.zig");
    _ = @import("Physics/CollisionGroupTests.zig");
    _ = @import("Physics/ConvexVsTrianglesTest.zig");
    _ = @import("Physics/HeightFieldShapeTests.zig");
    _ = @import("Physics/MutableCompoundShapeTests.zig");
    _ = @import("Physics/ObjectLayerPairFilterMaskTests.zig");
    _ = @import("Physics/ObjectLayerPairFilterTableTests.zig");
    _ = @import("Physics/OffsetCenterOfMassShapeTests.zig");
    _ = @import("Physics/RayShapeTests.zig");
    _ = @import("Physics/ShapeFilterTests.zig");
    _ = @import("Physics/ShapeTests.zig");
    _ = @import("Physics/SubShapeIDTest.zig");
    _ = @import("Physics/TaperedCylinderShapeTests.zig");
    _ = @import("Physics/TransformedShapeTests.zig");
}
