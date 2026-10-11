// Reference implementation for the Jolt/Geometry primitives parity tests (`zig build parity`): thin C ABI wrappers
// around the C++ Jolt library. ZoltParity/Geometry/PrimitivesParity.zig calls these and checks that Zolt produces
// the same bits.
//
// Conventions: Vec3 = 3 floats, Vec4 / Plane = 4 floats, UVec4 = 4 uint32, Mat44 = 16 floats (column major),
// AABox = 6 floats (min xyz, max xyz), Sphere = 4 floats (center xyz, radius), DVec3 = 3 doubles, DMat44 = 12 floats
// (3 rotation columns) + 3 doubles (translation). 4 boxes in SOA layout ("bounds") = 24 floats: min x, min y, min z,
// max x, max y, max z (4 floats each). Polygons are arrays of Vec3 (3 floats per vertex) plus a vertex count.
// Triangle and IndexedTriangle are passed as raw memory, Zolt's structs have the same layout.

#include <Jolt/Jolt.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Geometry/AABox4.h>
#include <Jolt/Geometry/ClipPoly.h>
#include <Jolt/Geometry/Ellipse.h>
#include <Jolt/Geometry/IndexedTriangle.h>
#include <Jolt/Geometry/Indexify.h>
#include <Jolt/Geometry/MortonCode.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Geometry/Sphere.h>
#include <Jolt/Geometry/Triangle.h>

using namespace JPH;

static_assert(sizeof(Triangle) == 44, "Zolt passes Triangle as raw memory");
static_assert(sizeof(IndexedTriangle) == 20, "Zolt passes IndexedTriangle as raw memory");
static_assert(sizeof(IndexedTriangleNoMaterial) == 12, "Zolt passes IndexedTriangleNoMaterial as raw memory");

// Jolt's Array allocates through the Allocate / Free hooks, which must be set before the first allocation
static void EnsureAllocator()
{
	RegisterDefaultAllocator();
}

static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static void Store4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void StoreU4(UVec4Arg inV, uint32 *outV)		{ inV.StoreInt4(outV); }
static Mat44 LoadMat44(const float *inM)			{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
static void StoreMat44(Mat44Arg inM, float *outM)	{ for (uint c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
static DVec3 LoadD3(const double *inV)				{ return DVec3(inV[0], inV[1], inV[2]); }
static DMat44 LoadDMat44(const float *inCols, const double *inT) { return DMat44(Load4(inCols), Load4(inCols + 4), Load4(inCols + 8), LoadD3(inT)); }
static AABox LoadBox(const float *inB)				{ return AABox(Load3(inB), Load3(inB + 3)); }
static void StoreBox(const AABox &inB, float *outB)	{ Store3(inB.mMin, outB); Store3(inB.mMax, outB + 3); }
static Plane LoadPlane(const float *inP)			{ return Plane(Load4(inP)); }
static void StorePlane(const Plane &inP, float *outP) { inP.StoreFloat4(reinterpret_cast<Float4 *>(outP)); }
static Sphere LoadSphere(const float *inS)			{ return Sphere(Load3(inS), inS[3]); }
static void StoreSphere(const Sphere &inS, float *outS) { Store3(inS.GetCenter(), outS); outS[3] = inS.GetRadius(); }
static VertexList LoadVertices(const float *inV, uint32 inNumVertices) { EnsureAllocator(); return VertexList(reinterpret_cast<const Float3 *>(inV), reinterpret_cast<const Float3 *>(inV) + inNumVertices); }

static Array<Vec3> LoadPolygon(const float *inV, uint32 inNumVertices)
{
	EnsureAllocator();
	Array<Vec3> polygon;
	for (uint32 i = 0; i < inNumVertices; ++i)
		polygon.push_back(Load3(inV + 3 * i));
	return polygon;
}

// Returns the number of vertices, or 0xffffffff if outV is too small
static uint32 StorePolygon(const Array<Vec3> &inPolygon, float *outV, uint32 inMaxVertices)
{
	if (inPolygon.size() > inMaxVertices)
		return 0xffffffff;
	for (size_t i = 0; i < inPolygon.size(); ++i)
		Store3(inPolygon[i], outV + 3 * i);
	return (uint32)inPolygon.size();
}

// 4 boxes in SOA layout
struct Bounds
{
	explicit		Bounds(const float *inB) : mMinX(Load4(inB)), mMinY(Load4(inB + 4)), mMinZ(Load4(inB + 8)), mMaxX(Load4(inB + 12)), mMaxY(Load4(inB + 16)), mMaxZ(Load4(inB + 20)) { }

	void			Store(float *outB) const { Store4(mMinX, outB); Store4(mMinY, outB + 4); Store4(mMinZ, outB + 8); Store4(mMaxX, outB + 12); Store4(mMaxY, outB + 16); Store4(mMaxZ, outB + 20); }

	Vec4			mMinX, mMinY, mMinZ, mMaxX, mMaxY, mMaxZ;
};

extern "C"
{

// Geometry/Triangle.h
void jolt_triangle_get_centroid(const void *inTriangle, float *outV) { Store3(static_cast<const Triangle *>(inTriangle)->GetCentroid(), outV); }

// Geometry/IndexedTriangle.h
bool jolt_indexed_triangle_no_material_eq(const void *inA, const void *inB) { return *static_cast<const IndexedTriangleNoMaterial *>(inA) == *static_cast<const IndexedTriangleNoMaterial *>(inB); }
bool jolt_indexed_triangle_no_material_is_equivalent(const void *inA, const void *inB) { return static_cast<const IndexedTriangleNoMaterial *>(inA)->IsEquivalent(*static_cast<const IndexedTriangleNoMaterial *>(inB)); }
bool jolt_indexed_triangle_no_material_is_opposite(const void *inA, const void *inB) { return static_cast<const IndexedTriangleNoMaterial *>(inA)->IsOpposite(*static_cast<const IndexedTriangleNoMaterial *>(inB)); }
bool jolt_indexed_triangle_no_material_is_degenerate(const void *inT, const float *inVertices, uint32 inNumVertices) { return static_cast<const IndexedTriangleNoMaterial *>(inT)->IsDegenerate(LoadVertices(inVertices, inNumVertices)); }
void jolt_indexed_triangle_no_material_rotate(const void *inT, void *outT) { IndexedTriangleNoMaterial t = *static_cast<const IndexedTriangleNoMaterial *>(inT); t.Rotate(); *static_cast<IndexedTriangleNoMaterial *>(outT) = t; }
void jolt_indexed_triangle_no_material_get_centroid(const void *inT, const float *inVertices, uint32 inNumVertices, float *outV) { Store3(static_cast<const IndexedTriangleNoMaterial *>(inT)->GetCentroid(LoadVertices(inVertices, inNumVertices)), outV); }
uint64 jolt_indexed_triangle_no_material_get_hash(const void *inT) { return static_cast<const IndexedTriangleNoMaterial *>(inT)->GetHash(); }
uint64 jolt_indexed_triangle_no_material_hash(const void *inT) { return Hash<IndexedTriangleNoMaterial> { } (*static_cast<const IndexedTriangleNoMaterial *>(inT)); }
uint64 jolt_indexed_triangle_no_material_std_hash(const void *inT) { return std::hash<IndexedTriangleNoMaterial> { } (*static_cast<const IndexedTriangleNoMaterial *>(inT)); }
bool jolt_indexed_triangle_eq(const void *inA, const void *inB) { return *static_cast<const IndexedTriangle *>(inA) == *static_cast<const IndexedTriangle *>(inB); }
bool jolt_indexed_triangle_is_equivalent(const void *inA, const void *inB) { return static_cast<const IndexedTriangle *>(inA)->IsEquivalent(*static_cast<const IndexedTriangle *>(inB)); }
bool jolt_indexed_triangle_is_opposite(const void *inA, const void *inB) { return static_cast<const IndexedTriangle *>(inA)->IsOpposite(*static_cast<const IndexedTriangle *>(inB)); }
bool jolt_indexed_triangle_is_degenerate(const void *inT, const float *inVertices, uint32 inNumVertices) { return static_cast<const IndexedTriangle *>(inT)->IsDegenerate(LoadVertices(inVertices, inNumVertices)); }
void jolt_indexed_triangle_rotate(const void *inT, void *outT) { IndexedTriangle t = *static_cast<const IndexedTriangle *>(inT); t.Rotate(); *static_cast<IndexedTriangle *>(outT) = t; }
void jolt_indexed_triangle_get_centroid(const void *inT, const float *inVertices, uint32 inNumVertices, float *outV) { Store3(static_cast<const IndexedTriangle *>(inT)->GetCentroid(LoadVertices(inVertices, inNumVertices)), outV); }
void jolt_indexed_triangle_get_lowest_index_first(const void *inT, void *outT) { *static_cast<IndexedTriangle *>(outT) = static_cast<const IndexedTriangle *>(inT)->GetLowestIndexFirst(); }
uint64 jolt_indexed_triangle_get_hash(const void *inT) { return static_cast<const IndexedTriangle *>(inT)->GetHash(); }
uint64 jolt_indexed_triangle_hash(const void *inT) { return Hash<IndexedTriangle> { } (*static_cast<const IndexedTriangle *>(inT)); }
uint64 jolt_indexed_triangle_std_hash(const void *inT) { return std::hash<IndexedTriangle> { } (*static_cast<const IndexedTriangle *>(inT)); }

// Geometry/Plane.h
void jolt_plane_init(const float *inNormal, float inConstant, float *outP) { StorePlane(Plane(Load3(inNormal), inConstant), outP); }
void jolt_plane_from_point_and_normal(const float *inPoint, const float *inNormal, float *outP) { StorePlane(Plane::sFromPointAndNormal(Load3(inPoint), Load3(inNormal)), outP); }
void jolt_plane_from_point_and_normal_dvec3(const double *inPoint, const float *inNormal, float *outP) { StorePlane(Plane::sFromPointAndNormal(LoadD3(inPoint), Load3(inNormal)), outP); }
void jolt_plane_from_points_ccw(const float *inV1, const float *inV2, const float *inV3, float *outP) { StorePlane(Plane::sFromPointsCCW(Load3(inV1), Load3(inV2), Load3(inV3)), outP); }
void jolt_plane_set_normal_constant(const float *inP, const float *inNormal, float inConstant, float *outNormal, float *outConstant, float *outP)
{
	Plane p = LoadPlane(inP);
	Store3(p.GetNormal(), outNormal);
	*outConstant = p.GetConstant();
	p.SetNormal(Load3(inNormal));
	p.SetConstant(inConstant);
	StorePlane(p, outP);
}
void jolt_plane_offset(const float *inP, float inDistance, float *outP) { StorePlane(LoadPlane(inP).Offset(inDistance), outP); }
void jolt_plane_get_transformed(const float *inP, const float *inM, float *outP) { StorePlane(LoadPlane(inP).GetTransformed(LoadMat44(inM)), outP); }
void jolt_plane_scaled(const float *inP, const float *inScale, float *outP) { StorePlane(LoadPlane(inP).Scaled(Load3(inScale)), outP); }
float jolt_plane_signed_distance(const float *inP, const float *inPoint) { return LoadPlane(inP).SignedDistance(Load3(inPoint)); }
void jolt_plane_project_point_on_plane(const float *inP, const float *inPoint, float *outV) { Store3(LoadPlane(inP).ProjectPointOnPlane(Load3(inPoint)), outV); }
bool jolt_plane_intersect_planes(const float *inP1, const float *inP2, const float *inP3, float *outV)
{
	Vec3 point = Vec3::sZero();
	bool result = Plane::sIntersectPlanes(LoadPlane(inP1), LoadPlane(inP2), LoadPlane(inP3), point);
	Store3(point, outV);
	return result;
}

// Geometry/AABox.h
void jolt_aabox_constants(float *outEmpty, float *outBiggest) { StoreBox(AABox(), outEmpty); StoreBox(AABox::sBiggest(), outBiggest); }
void jolt_aabox_from_dvec3(const double *inMin, const double *inMax, float *outB) { StoreBox(AABox(LoadD3(inMin), LoadD3(inMax)), outB); }
void jolt_aabox_from_center_and_radius(const float *inCenter, float inRadius, float *outB) { StoreBox(AABox(Load3(inCenter), inRadius), outB); }
void jolt_aabox_from_two_points(const float *inP1, const float *inP2, float *outB) { StoreBox(AABox::sFromTwoPoints(Load3(inP1), Load3(inP2)), outB); }
void jolt_aabox_from_triangle(const float *inVertices, uint32 inNumVertices, const void *inTriangle, float *outB) { StoreBox(AABox::sFromTriangle(LoadVertices(inVertices, inNumVertices), *static_cast<const IndexedTriangle *>(inTriangle)), outB); }
void jolt_aabox_compare(const float *inA, const float *inB, bool *outEqual, bool *outNotEqual, bool *outIsValid)
{
	AABox a = LoadBox(inA), b = LoadBox(inB);
	*outEqual = a == b;
	*outNotEqual = a != b;
	*outIsValid = a.IsValid();
}
void jolt_aabox_encapsulate(const float *inA, const float *inB, const float *inPoint, const void *inTriangle, const float *inVertices, uint32 inNumVertices, const void *inIndexedTriangle, float *outVec3, float *outAABox, float *outTriangle, float *outIndexedTriangle)
{
	AABox a = LoadBox(inA);
	a.Encapsulate(Load3(inPoint));
	StoreBox(a, outVec3);
	a = LoadBox(inA);
	a.Encapsulate(LoadBox(inB));
	StoreBox(a, outAABox);
	a = LoadBox(inA);
	a.Encapsulate(*static_cast<const Triangle *>(inTriangle));
	StoreBox(a, outTriangle);
	a = LoadBox(inA);
	a.Encapsulate(LoadVertices(inVertices, inNumVertices), *static_cast<const IndexedTriangle *>(inIndexedTriangle));
	StoreBox(a, outIndexedTriangle);
}
void jolt_aabox_intersect(const float *inA, const float *inB, float *outB) { StoreBox(LoadBox(inA).Intersect(LoadBox(inB)), outB); }
void jolt_aabox_ensure_minimal_edge_length(const float *inA, float inLength, float *outB) { AABox a = LoadBox(inA); a.EnsureMinimalEdgeLength(inLength); StoreBox(a, outB); }
void jolt_aabox_expand_by(const float *inA, const float *inV, float *outB) { AABox a = LoadBox(inA); a.ExpandBy(Load3(inV)); StoreBox(a, outB); }
void jolt_aabox_properties(const float *inA, float *outCenter, float *outExtent, float *outSize, float *outSurfaceArea, float *outVolume)
{
	AABox a = LoadBox(inA);
	Store3(a.GetCenter(), outCenter);
	Store3(a.GetExtent(), outExtent);
	Store3(a.GetSize(), outSize);
	*outSurfaceArea = a.GetSurfaceArea();
	*outVolume = a.GetVolume();
}
void jolt_aabox_contains_overlaps(const float *inA, const float *inB, const float *inPoint, const double *inDPoint, const float *inPlane, bool *outContains, bool *outContainsVec3, bool *outContainsDVec3, bool *outOverlaps, bool *outOverlapsPlane)
{
	AABox a = LoadBox(inA);
	*outContains = a.Contains(LoadBox(inB));
	*outContainsVec3 = a.Contains(Load3(inPoint));
	*outContainsDVec3 = a.Contains(LoadD3(inDPoint));
	*outOverlaps = a.Overlaps(LoadBox(inB));
	*outOverlapsPlane = a.Overlaps(LoadPlane(inPlane));
}
void jolt_aabox_translate(const float *inA, const float *inTranslation, float *outB) { AABox a = LoadBox(inA); a.Translate(Load3(inTranslation)); StoreBox(a, outB); }
void jolt_aabox_translate_dvec3(const float *inA, const double *inTranslation, float *outB) { AABox a = LoadBox(inA); a.Translate(LoadD3(inTranslation)); StoreBox(a, outB); }
void jolt_aabox_transformed(const float *inA, const float *inM, float *outB) { StoreBox(LoadBox(inA).Transformed(LoadMat44(inM)), outB); }
void jolt_aabox_transformed_dmat44(const float *inA, const float *inCols, const double *inT, float *outB) { StoreBox(LoadBox(inA).Transformed(LoadDMat44(inCols, inT)), outB); }
void jolt_aabox_scaled(const float *inA, const float *inScale, float *outB) { StoreBox(LoadBox(inA).Scaled(Load3(inScale)), outB); }
void jolt_aabox_get_support(const float *inA, const float *inDirection, float *outV) { Store3(LoadBox(inA).GetSupport(Load3(inDirection)), outV); }
void jolt_aabox_get_supporting_face(const float *inA, const float *inDirection, float *outV)
{
	StaticArray<Vec3, 8> face;
	LoadBox(inA).GetSupportingFace(Load3(inDirection), face);
	JPH_ASSERT(face.size() == 4);
	for (uint i = 0; i < 4; ++i)
		Store3(face[i], outV + 3 * i);
}
void jolt_aabox_get_closest_point(const float *inA, const float *inPoint, float *outV, float *outSqDistance)
{
	AABox a = LoadBox(inA);
	Store3(a.GetClosestPoint(Load3(inPoint)), outV);
	*outSqDistance = a.GetSqDistanceTo(Load3(inPoint));
}

// Geometry/Sphere.h
void jolt_sphere_get_support(const float *inS, const float *inDirection, float *outV) { Store3(LoadSphere(inS).GetSupport(Load3(inDirection)), outV); }
bool jolt_sphere_overlaps(const float *inA, const float *inB) { return LoadSphere(inA).Overlaps(LoadSphere(inB)); }
bool jolt_sphere_overlaps_aabox(const float *inS, const float *inB) { return LoadSphere(inS).Overlaps(LoadBox(inB)); }
void jolt_sphere_encapsulate_point(const float *inS, const float *inPoint, float *outS) { Sphere s = LoadSphere(inS); s.EncapsulatePoint(Load3(inPoint)); StoreSphere(s, outS); }

// Geometry/OrientedBox.h
void jolt_oriented_box_from_aabox(const float *inM, const float *inB, float *outM, float *outHalfExtents)
{
	OrientedBox box(LoadMat44(inM), LoadBox(inB));
	StoreMat44(box.mOrientation, outM);
	Store3(box.mHalfExtents, outHalfExtents);
}
bool jolt_oriented_box_overlaps_aabox(const float *inM, const float *inHalfExtents, const float *inB, float inEpsilon) { return OrientedBox(LoadMat44(inM), Load3(inHalfExtents)).Overlaps(LoadBox(inB), inEpsilon); }
bool jolt_oriented_box_overlaps(const float *inM1, const float *inHalfExtents1, const float *inM2, const float *inHalfExtents2, float inEpsilon) { return OrientedBox(LoadMat44(inM1), Load3(inHalfExtents1)).Overlaps(OrientedBox(LoadMat44(inM2), Load3(inHalfExtents2)), inEpsilon); }

// Geometry/AABox4.h
void jolt_aabox4_vs_box(const float *inBox, const float *inBounds, uint32 *outV) { Bounds b(inBounds); StoreU4(AABox4VsBox(LoadBox(inBox), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ), outV); }
void jolt_aabox4_scale(const float *inScale, const float *inBounds, float *outBounds)
{
	Bounds b(inBounds), out(inBounds);
	AABox4Scale(Load3(inScale), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ, out.mMinX, out.mMinY, out.mMinZ, out.mMaxX, out.mMaxY, out.mMaxZ);
	out.Store(outBounds);
}
void jolt_aabox4_enlarge_with_extent(const float *inExtent, const float *inBounds, float *outBounds)
{
	Bounds b(inBounds);
	AABox4EnlargeWithExtent(Load3(inExtent), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ);
	b.Store(outBounds);
}
void jolt_aabox4_vs_point(const float *inPoint, const float *inBounds, uint32 *outV) { Bounds b(inBounds); StoreU4(AABox4VsPoint(Load3(inPoint), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ), outV); }
void jolt_aabox4_vs_oriented_box_mat44(const float *inM, const float *inHalfExtents, const float *inBounds, float inEpsilon, uint32 *outV) { Bounds b(inBounds); StoreU4(AABox4VsBox(LoadMat44(inM), Load3(inHalfExtents), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ, inEpsilon), outV); }
void jolt_aabox4_vs_oriented_box(const float *inM, const float *inHalfExtents, const float *inBounds, float inEpsilon, uint32 *outV) { Bounds b(inBounds); StoreU4(AABox4VsBox(OrientedBox(LoadMat44(inM), Load3(inHalfExtents)), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ, inEpsilon), outV); }
void jolt_aabox4_distance_sq_to_point(const float *inPoint, const float *inBounds, float *outV) { Bounds b(inBounds); Store4(AABox4DistanceSqToPoint(Load4(inPoint), Load4(inPoint + 4), Load4(inPoint + 8), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ), outV); }
void jolt_aabox4_distance_sq_to_point_vec3(const float *inPoint, const float *inBounds, float *outV) { Bounds b(inBounds); Store4(AABox4DistanceSqToPoint(Load3(inPoint), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ), outV); }
void jolt_aabox4_vs_sphere(const float *inCenter, const float *inRadiusSq, const float *inBounds, uint32 *outV) { Bounds b(inBounds); StoreU4(AABox4VsSphere(Load4(inCenter), Load4(inCenter + 4), Load4(inCenter + 8), Load4(inRadiusSq), b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ), outV); }
void jolt_aabox4_vs_sphere_vec3(const float *inCenter, float inRadiusSq, const float *inBounds, uint32 *outV) { Bounds b(inBounds); StoreU4(AABox4VsSphere(Load3(inCenter), inRadiusSq, b.mMinX, b.mMinY, b.mMinZ, b.mMaxX, b.mMaxY, b.mMaxZ), outV); }

// Geometry/ClipPoly.h (all functions return the number of output vertices, or 0xffffffff if outV is too small)
uint32 jolt_clip_poly_vs_plane(const float *inPolygon, uint32 inNumVertices, const float *inPlaneOrigin, const float *inPlaneNormal, float *outV, uint32 inMaxVertices)
{
	Array<Vec3> clipped;
	ClipPolyVsPlane(LoadPolygon(inPolygon, inNumVertices), Load3(inPlaneOrigin), Load3(inPlaneNormal), clipped);
	return StorePolygon(clipped, outV, inMaxVertices);
}
uint32 jolt_clip_poly_vs_poly(const float *inPolygon, uint32 inNumVertices, const float *inClippingPolygon, uint32 inNumClippingVertices, const float *inClippingPolygonNormal, float *outV, uint32 inMaxVertices)
{
	Array<Vec3> clipped;
	ClipPolyVsPoly(LoadPolygon(inPolygon, inNumVertices), LoadPolygon(inClippingPolygon, inNumClippingVertices), Load3(inClippingPolygonNormal), clipped);
	return StorePolygon(clipped, outV, inMaxVertices);
}
uint32 jolt_clip_poly_vs_edge(const float *inPolygon, uint32 inNumVertices, const float *inEdgeVertex1, const float *inEdgeVertex2, const float *inClippingEdgeNormal, float *outV, uint32 inMaxVertices)
{
	Array<Vec3> clipped;
	ClipPolyVsEdge(LoadPolygon(inPolygon, inNumVertices), Load3(inEdgeVertex1), Load3(inEdgeVertex2), Load3(inClippingEdgeNormal), clipped);
	return StorePolygon(clipped, outV, inMaxVertices);
}
uint32 jolt_clip_poly_vs_aabox(const float *inPolygon, uint32 inNumVertices, const float *inBox, float *outV, uint32 inMaxVertices)
{
	Array<Vec3> clipped;
	ClipPolyVsAABox(LoadPolygon(inPolygon, inNumVertices), LoadBox(inBox), clipped);
	return StorePolygon(clipped, outV, inMaxVertices);
}

// Geometry/MortonCode.h
uint32 jolt_morton_code_expand_bits(float inV) { return MortonCode::sExpandBits(inV); }
uint32 jolt_morton_code_get_morton_code(const float *inVector, const float *inBounds) { return MortonCode::sGetMortonCode(Load3(inVector), LoadBox(inBounds)); }

// Geometry/Indexify.h
// outVertices must have room for 3 * inNumTriangles vertices, outTriangles for inNumTriangles triangles
void jolt_indexify(const void *inTriangles, uint32 inNumTriangles, float inVertexWeldDistance, float *outVertices, uint32 *outNumVertices, void *outTriangles, uint32 *outNumTriangles)
{
	EnsureAllocator();
	const Triangle *triangles = static_cast<const Triangle *>(inTriangles);
	TriangleList in_triangles(triangles, triangles + inNumTriangles);
	VertexList vertices;
	IndexedTriangleList indexed;
	Indexify(in_triangles, vertices, indexed, inVertexWeldDistance);
	JPH_ASSERT(vertices.size() <= 3 * inNumTriangles && indexed.size() <= inNumTriangles);
	memcpy(outVertices, vertices.data(), vertices.size() * sizeof(Float3));
	*outNumVertices = (uint32)vertices.size();
	memcpy(outTriangles, indexed.data(), indexed.size() * sizeof(IndexedTriangle));
	*outNumTriangles = (uint32)indexed.size();
}
// outTriangles must have room for inNumTriangles triangles
void jolt_deindexify(const float *inVertices, uint32 inNumVertices, const void *inTriangles, uint32 inNumTriangles, void *outTriangles)
{
	EnsureAllocator();
	const IndexedTriangle *triangles = static_cast<const IndexedTriangle *>(inTriangles);
	IndexedTriangleList in_triangles(triangles, triangles + inNumTriangles);
	TriangleList out;
	Deindexify(LoadVertices(inVertices, inNumVertices), in_triangles, out);
	memcpy(outTriangles, out.data(), out.size() * sizeof(Triangle));
}

// Geometry/Ellipse.h
bool jolt_ellipse_is_inside(float inA, float inB, const float *inPoint) { return Ellipse(inA, inB).IsInside(Float2(inPoint[0], inPoint[1])); }
void jolt_ellipse_get_closest_point(float inA, float inB, const float *inPoint, float *outPoint) { Float2 p = Ellipse(inA, inB).GetClosestPoint(Float2(inPoint[0], inPoint[1])); outPoint[0] = p.x; outPoint[1] = p.y; }
void jolt_ellipse_get_normal(float inA, float inB, const float *inPoint, float *outNormal) { Float2 n = Ellipse(inA, inB).GetNormal(Float2(inPoint[0], inPoint[1])); outNormal[0] = n.x; outNormal[1] = n.y; }

} // extern "C"
