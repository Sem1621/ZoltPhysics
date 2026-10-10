//! Port of: UnitTests/Physics/ActiveEdgesTests.cpp
//! Status: stub
//! Missing: CollideShapeMesh, CastShapeMesh (need MeshShape and CapsuleShape, Phase 4 Wave A / B), TestNonManifoldMesh
//!   (needs MeshShape, Phase 4 Wave B), CollideShapeHeightField, CastShapeHeightField (need HeightFieldShape and
//!   CapsuleShape, Phase 4 Wave A / B), DiscreteCubeSlideMesh, DiscreteCubeSlideHeightField, LinearCastCubeSlideMesh,
//!   LinearCastCubeSlideHeightField (need PhysicsTestContext, Phase 5)
//!
//! Every TEST_CASE of this file collides with a MeshShape or a HeightFieldShape (most of them with a CapsuleShape probe),
//! so none can be ported before those shapes are merged. The active edge handling itself (ActiveEdges::FixNormal through
//! CollideConvexVsTriangles / CollideSphereVsTriangles / CastConvexVsTriangles / CastSphereVsTriangles with active edge
//! flags) is covered by ZoltTests/Physics/ConvexVsTrianglesTest.zig, the inline tests of Zolt/Physics/Collision/ActiveEdges.zig
//! and compared bit for bit with Jolt in ZoltParity/Physics/TrianglesParity.zig; a TriangleShape has all edges active
//! (ZoltParity/Physics/TriangleShapeParity.zig compares both active edge modes).
