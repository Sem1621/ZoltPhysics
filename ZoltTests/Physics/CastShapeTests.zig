//! Port of: UnitTests/Physics/CastShapeTests.cpp
//! Status: stub
//! Missing: TestCastSphereTriangle, TestCastSphereVsDegenerateTriangle (need TriangleShape, MeshShape and
//!   CastSphereVsTriangles, Phase 4 Wave A / B), TestCast2DBoxVsBox (needs ConvexHullShape, Phase 4 Wave A),
//!   TestCastShapeSphereVsBox, TestCastShapeCylinderVsCapsule, TestCastShapePenetrationDepthOrdering,
//!   TestDeepestPenetrationAtFraction0, TestClosestHitPerBodyCollisionCollector, TestCastShapeExtraConvexRadius,
//!   TestCastShapeInitiallyIntersecting (need PhysicsTestContext, Phase 5)
//!
//! Every TEST_CASE of this file needs a shape that is not ported yet or PhysicsTestContext. Sphere and box casts through
//! CollisionDispatch (ConvexShape's castConvexVsConvex) are tested inline in Zolt/Physics/Collision/Shape/ConvexShape.zig
//! and compared bit for bit with Jolt in ZoltParity/Physics/ConvexParity.zig.
