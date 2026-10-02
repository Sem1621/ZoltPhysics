// Reference implementation for the Geometry queries parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library (ClosestPoint, RayAABox, RayTriangle, RaySphere, RayCylinder, RayCapsule, ConvexSupport and
// ConvexHullBuilder2D). ZoltParity/Geometry/QueriesParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), matrices as 16 floats in column major order,
// UVec4 masks as 4 uint32. Out parameters of Jolt that are always written are returned through pointers.

#include <Jolt/Jolt.h>
#include <Jolt/Core/StaticArray.h>
#include <Jolt/Geometry/ClosestPoint.h>
#include <Jolt/Geometry/ConvexHullBuilder2D.h>
#include <Jolt/Geometry/ConvexSupport.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Geometry/RayCapsule.h>
#include <Jolt/Geometry/RayCylinder.h>
#include <Jolt/Geometry/RaySphere.h>
#include <Jolt/Geometry/RayTriangle.h>

using namespace JPH;

static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void Store4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void StoreU4(UVec4Arg inV, uint32 *outV)		{ inV.StoreInt4(outV); }
static Mat44 LoadMat44(const float *inM)			{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }

// Vertex array used for the supporting faces (Shape::SupportingFace)
using Face = StaticArray<Vec3, 32>;

// Number of support points and faces that jolt_convex_support computes, must match ConvexSupportResult in QueriesParity.zig
static constexpr int cNumSupports = 13;
static constexpr int cNumFaces = 6;

template <class VERTEX_ARRAY>
static void StoreFace(const VERTEX_ARRAY &inFace, float *outVertices, int &outCount)
{
	outCount = int(inFace.size());
	for (int i = 0; i < outCount; ++i)
		Store3(inFace[i], outVertices + 3 * i);
}

extern "C"
{

// Geometry/ClosestPoint.h
bool jolt_closest_point_bary_line(const float *inA, const float *inB, float *outUV)
{
	return ClosestPoint::GetBaryCentricCoordinates(Load3(inA), Load3(inB), outUV[0], outUV[1]);
}

bool jolt_closest_point_bary_triangle(const float *inA, const float *inB, const float *inC, float *outUVW)
{
	return ClosestPoint::GetBaryCentricCoordinates(Load3(inA), Load3(inB), Load3(inC), outUVW[0], outUVW[1], outUVW[2]);
}

uint32 jolt_closest_point_on_line(const float *inA, const float *inB, float *outPoint)
{
	uint32 set = 0xdeadbeef;
	Store3(ClosestPoint::GetClosestPointOnLine(Load3(inA), Load3(inB), set), outPoint);
	return set;
}

uint32 jolt_closest_point_on_triangle(const float *inA, const float *inB, const float *inC, bool inMustIncludeC, float *outPoint)
{
	uint32 set = 0xdeadbeef;
	Vec3 p = inMustIncludeC?
		ClosestPoint::GetClosestPointOnTriangle<true>(Load3(inA), Load3(inB), Load3(inC), set) :
		ClosestPoint::GetClosestPointOnTriangle<false>(Load3(inA), Load3(inB), Load3(inC), set);
	Store3(p, outPoint);
	return set;
}

bool jolt_origin_outside_of_plane(const float *inA, const float *inB, const float *inC, const float *inD)
{
	return ClosestPoint::OriginOutsideOfPlane(Load3(inA), Load3(inB), Load3(inC), Load3(inD));
}

void jolt_origin_outside_of_tetrahedron_planes(const float *inA, const float *inB, const float *inC, const float *inD, uint32 *outMask)
{
	StoreU4(ClosestPoint::OriginOutsideOfTetrahedronPlanes(Load3(inA), Load3(inB), Load3(inC), Load3(inD)), outMask);
}

uint32 jolt_closest_point_on_tetrahedron(const float *inA, const float *inB, const float *inC, const float *inD, bool inMustIncludeD, float *outPoint)
{
	uint32 set = 0xdeadbeef;
	Vec3 p = inMustIncludeD?
		ClosestPoint::GetClosestPointOnTetrahedron<true>(Load3(inA), Load3(inB), Load3(inC), Load3(inD), set) :
		ClosestPoint::GetClosestPointOnTetrahedron<false>(Load3(inA), Load3(inB), Load3(inC), Load3(inD), set);
	Store3(p, outPoint);
	return set;
}

// Geometry/RayAABox.h
void jolt_ray_inv_direction(const float *inDirection, float *outInvDirection, uint32 *outIsParallel)
{
	RayInvDirection inv(Load3(inDirection));
	Store4(Vec4(inv.mInvDirection), outInvDirection); // All 4 lanes, W is the reciprocal of Z like in Zolt
	StoreU4(inv.mIsParallel, outIsParallel);
}

float jolt_ray_aabox(const float *inOrigin, const float *inDirection, const float *inMin, const float *inMax)
{
	return RayAABox(Load3(inOrigin), RayInvDirection(Load3(inDirection)), Load3(inMin), Load3(inMax));
}

// inBounds: min x, min y, min z, max x, max y, max z (4 floats each)
void jolt_ray_aabox4(const float *inOrigin, const float *inDirection, const float *inBounds, float *outFractions)
{
	Store4(RayAABox4(Load3(inOrigin), RayInvDirection(Load3(inDirection)), Load4(inBounds), Load4(inBounds + 4), Load4(inBounds + 8), Load4(inBounds + 12), Load4(inBounds + 16), Load4(inBounds + 20)), outFractions);
}

void jolt_ray_aabox_min_max(const float *inOrigin, const float *inDirection, const float *inMin, const float *inMax, float *outMinMax)
{
	RayAABox(Load3(inOrigin), RayInvDirection(Load3(inDirection)), Load3(inMin), Load3(inMax), outMinMax[0], outMinMax[1]);
}

bool jolt_ray_aabox_hits(const float *inOrigin, const float *inDirection, const float *inMin, const float *inMax, float inClosest)
{
	return RayAABoxHits(Load3(inOrigin), RayInvDirection(Load3(inDirection)), Load3(inMin), Load3(inMax), inClosest);
}

bool jolt_ray_aabox_hits_direction(const float *inOrigin, const float *inDirection, const float *inMin, const float *inMax)
{
	return RayAABoxHits(Load3(inOrigin), Load3(inDirection), Load3(inMin), Load3(inMax));
}

// Geometry/RayTriangle.h
float jolt_ray_triangle(const float *inOrigin, const float *inDirection, const float *inV0, const float *inV1, const float *inV2)
{
	return RayTriangle(Load3(inOrigin), Load3(inDirection), Load3(inV0), Load3(inV1), Load3(inV2));
}

// inV: v0 x, v0 y, v0 z, v1 x, v1 y, v1 z, v2 x, v2 y, v2 z (4 floats each)
void jolt_ray_triangle4(const float *inOrigin, const float *inDirection, const float *inV, float *outFractions)
{
	Store4(RayTriangle4(Load3(inOrigin), Load3(inDirection), Load4(inV), Load4(inV + 4), Load4(inV + 8), Load4(inV + 12), Load4(inV + 16), Load4(inV + 20), Load4(inV + 24), Load4(inV + 28), Load4(inV + 32)), outFractions);
}

// Geometry/RaySphere.h
float jolt_ray_sphere(const float *inOrigin, const float *inDirection, const float *inCenter, float inRadius)
{
	return RaySphere(Load3(inOrigin), Load3(inDirection), Load3(inCenter), inRadius);
}

int jolt_ray_sphere_min_max(const float *inOrigin, const float *inDirection, const float *inCenter, float inRadius, float *outMinMax)
{
	return RaySphere(Load3(inOrigin), Load3(inDirection), Load3(inCenter), inRadius, outMinMax[0], outMinMax[1]);
}

// Geometry/RayCylinder.h
float jolt_ray_infinite_cylinder(const float *inOrigin, const float *inDirection, float inRadius)
{
	return RayCylinder(Load3(inOrigin), Load3(inDirection), inRadius);
}

float jolt_ray_cylinder(const float *inOrigin, const float *inDirection, float inHalfHeight, float inRadius)
{
	return RayCylinder(Load3(inOrigin), Load3(inDirection), inHalfHeight, inRadius);
}

// Geometry/RayCapsule.h
float jolt_ray_capsule(const float *inOrigin, const float *inDirection, float inHalfHeight, float inRadius)
{
	return RayCapsule(Load3(inOrigin), Load3(inDirection), inHalfHeight, inRadius);
}

// Geometry/ConvexSupport.h: support points and supporting faces of all wrappers, nested like GJK / EPA use them.
// outSupports: cNumSupports * 3 floats, outFaces: cNumFaces * 32 * 3 floats, outFaceCounts: cNumFaces ints.
void jolt_convex_support(const float *inTransform, const float *inPoint, const float *inTriangle, const float *inPolygon, int inPolygonCount, float inRadius, const float *inDirection, float *outSupports, float *outFaces, int *outFaceCounts)
{
	RegisterDefaultAllocator();

	Mat44 transform = LoadMat44(inTransform);
	Vec3 direction = Load3(inDirection);

	PointConvexSupport point { Load3(inPoint) };
	TriangleConvexSupport triangle(Load3(inTriangle), Load3(inTriangle + 3), Load3(inTriangle + 6));
	Array<Vec3> polygon_vertices;
	for (int i = 0; i < inPolygonCount; ++i)
		polygon_vertices.push_back(Load3(inPolygon + 3 * i));
	PolygonConvexSupport polygon(polygon_vertices);
	Face polygon_face_vertices;
	for (int i = 0; i < inPolygonCount; ++i)
		polygon_face_vertices.push_back(Load3(inPolygon + 3 * i));
	PolygonConvexSupport polygon_face(polygon_face_vertices);

	AddConvexRadius add_radius_triangle(triangle, inRadius);
	AddConvexRadius add_radius_polygon(polygon, inRadius);
	MinkowskiDifference triangle_minus_polygon(triangle, polygon);
	MinkowskiDifference rounded_polygon_minus_point(add_radius_polygon, point);
	TransformedConvexObject transformed_triangle(transform, triangle);
	TransformedConvexObject transformed_rounded_polygon(transform, add_radius_polygon);
	TransformedConvexObject transformed_rounded_triangle(transform, add_radius_triangle);
	MinkowskiDifference epa_like(transformed_rounded_triangle, add_radius_polygon);
	TransformedConvexObject transformed_difference(transform, triangle_minus_polygon);
	TransformedConvexObject transformed_point(transform, point);
	MinkowskiDifference gjk_cast_like(polygon, transformed_triangle);
	TransformedConvexObject transformed_polygon(transform, polygon);
	TransformedConvexObject transformed_polygon_face(transform, polygon_face);

	Vec3 supports[cNumSupports] = {
		point.GetSupport(direction),
		triangle.GetSupport(direction),
		polygon.GetSupport(direction),
		add_radius_triangle.GetSupport(direction),
		add_radius_polygon.GetSupport(direction),
		triangle_minus_polygon.GetSupport(direction),
		rounded_polygon_minus_point.GetSupport(direction),
		transformed_triangle.GetSupport(direction),
		transformed_rounded_polygon.GetSupport(direction),
		epa_like.GetSupport(direction),
		transformed_difference.GetSupport(direction),
		transformed_point.GetSupport(direction),
		gjk_cast_like.GetSupport(direction),
	};
	for (int i = 0; i < cNumSupports; ++i)
		Store3(supports[i], outSupports + 3 * i);

	Face faces[cNumFaces - 1];
	triangle.GetSupportingFace(direction, faces[0]);
	polygon.GetSupportingFace(direction, faces[1]);
	transformed_triangle.GetSupportingFace(direction, faces[2]);
	faces[3].push_back(Load3(inPoint)); // Vertices that are already in the array get transformed too
	transformed_polygon.GetSupportingFace(direction, faces[3]);
	transformed_polygon_face.GetSupportingFace(direction, faces[4]);
	for (int i = 0; i < cNumFaces - 1; ++i)
		StoreFace(faces[i], outFaces + 32 * 3 * i, outFaceCounts[i]);

	// The last face goes into an Array<Vec3> (VertexArrayList in Zolt) and accumulates: every call transforms all vertices again
	Array<Vec3> face_list;
	face_list.push_back(Load3(inPoint));
	transformed_triangle.GetSupportingFace(direction, face_list);
	transformed_triangle.GetSupportingFace(direction, face_list);
	StoreFace(face_list, outFaces + 32 * 3 * (cNumFaces - 1), outFaceCounts[cNumFaces - 1]);
}

// Geometry/ConvexHullBuilder2D.h: returns the result code, outEdges must have room for inCount entries
int jolt_convex_hull_builder_2d(const float *inPositions, int inCount, int inIdx1, int inIdx2, int inIdx3, int inMaxVertices, float inTolerance, int *outEdges, int *outNumEdges)
{
	RegisterDefaultAllocator();

	ConvexHullBuilder2D::Positions positions;
	for (int i = 0; i < inCount; ++i)
		positions.push_back(Load3(inPositions + 3 * i));

	ConvexHullBuilder2D builder(positions);
	ConvexHullBuilder2D::Edges edges;
	ConvexHullBuilder2D::EResult result = builder.Initialize(inIdx1, inIdx2, inIdx3, inMaxVertices, inTolerance, edges);

	*outNumEdges = int(edges.size());
	for (size_t i = 0; i < edges.size(); ++i)
		outEdges[i] = edges[i];
	return int(result);
}

} // extern "C"
