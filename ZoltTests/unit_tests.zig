//! Port of: UnitTests/ (the Jolt unit test suite), root of the `zolt-unit-tests` test binary.
//!
//! Each file mirrors a file in Jolt's UnitTests/ directory, e.g. UnitTests/Math/Vec3Tests.cpp ->
//! ZoltTests/Math/Vec3Tests.zig. Tests only use the public API through @import("zolt").
//! Add new test files to the list below (keep it sorted by path).

test {
    _ = @import("Layers.zig");
    _ = @import("LoggingBodyActivationListener.zig");
    _ = @import("LoggingContactListener.zig");
    _ = @import("PhysicsTestContext.zig");
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
    _ = @import("Physics/BodyInterfaceTests.zig");
    _ = @import("Physics/BodyManagerTests.zig");
    _ = @import("Physics/BodyTestContext.zig");
    _ = @import("Physics/BodyTests.zig");
    _ = @import("Physics/BroadPhaseBruteForceTests.zig");
    _ = @import("Physics/BroadPhaseQuadTreeTests.zig");
    _ = @import("Physics/BroadPhaseTests.zig");
    _ = @import("Physics/CastShapeTests.zig");
    _ = @import("Physics/CollidePointTests.zig");
    _ = @import("Physics/CollideShapeTests.zig");
    _ = @import("Physics/CollisionGroupTests.zig");
    _ = @import("Physics/ConeConstraintTests.zig");
    _ = @import("Physics/ConstraintManagerTests.zig");
    _ = @import("Physics/ContactConstraintManagerTests.zig");
    _ = @import("Physics/ContactListenerTests.zig");
    _ = @import("Physics/ConvexVsTrianglesTest.zig");
    _ = @import("Physics/DeterminismLogTests.zig");
    _ = @import("Physics/EstimateCollisionResponseTest.zig");
    _ = @import("Physics/FixedConstraintTests.zig");
    _ = @import("Physics/GearConstraintTests.zig");
    _ = @import("Physics/HeightFieldShapeTests.zig");
    _ = @import("Physics/HingeConstraintTests.zig");
    _ = @import("Physics/IslandBuilderTests.zig");
    _ = @import("Physics/LargeIslandSplitterTests.zig");
    _ = @import("Physics/MotionQualityLinearCastTests.zig");
    _ = @import("Physics/MutableCompoundShapeTests.zig");
    _ = @import("Physics/NarrowPhaseQueryTests.zig");
    _ = @import("Physics/ObjectLayerPairFilterMaskTests.zig");
    _ = @import("Physics/ObjectLayerPairFilterTableTests.zig");
    _ = @import("Physics/OffsetCenterOfMassShapeTests.zig");
    _ = @import("Physics/PhysicsDeterminismTests.zig");
    _ = @import("Physics/PhysicsStepListenerTests.zig");
    _ = @import("Physics/PhysicsSystemTests.zig");
    _ = @import("Physics/PhysicsTests.zig");
    _ = @import("Physics/PointConstraintTests.zig");
    _ = @import("Physics/RackAndPinionConstraintTests.zig");
    _ = @import("Physics/RayShapeTests.zig");
    _ = @import("Physics/SensorTests.zig");
    _ = @import("Physics/ShapeFilterTests.zig");
    _ = @import("Physics/ShapeTests.zig");
    _ = @import("Physics/SliderConstraintTests.zig");
    _ = @import("Physics/SixDOFConstraintTests.zig");
    _ = @import("Physics/SubShapeIDTest.zig");
    _ = @import("Physics/SwingTwistConstraintTests.zig");
    _ = @import("Physics/TaperedCylinderShapeTests.zig");
    _ = @import("Physics/TransformedShapeTests.zig");
}
