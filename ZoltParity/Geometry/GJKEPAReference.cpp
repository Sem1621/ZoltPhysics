// Reference implementation for the GJK / EPA parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library (GJKClosestPoint, EPAConvexHullBuilder and EPAPenetrationDepth). ZoltParity/Geometry/GJKEPAParity.zig calls
// these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as 3 floats, matrices as 16 floats in column major order. Jolt's in/out and out
// parameters are passed as in/out pointers: the caller fills them with a sentinel, so outputs that Jolt does not write
// keep the sentinel. The convex objects are described by a ShapeDesc (must match ShapeDesc in GJKEPAParity.zig) and
// built from Jolt's ConvexSupport wrappers around Sphere, AABox, triangles, polygons and points.

#include <Jolt/Jolt.h>
#include <Jolt/Core/StaticArray.h>
#include <Jolt/Geometry/AABox.h>
#include <Jolt/Geometry/ConvexSupport.h>
#include <Jolt/Geometry/EPAConvexHullBuilder.h>
#include <Jolt/Geometry/EPAPenetrationDepth.h>
#include <Jolt/Geometry/GJKClosestPoint.h>
#include <Jolt/Geometry/Sphere.h>

using namespace JPH;

static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Mat44 LoadMat44(const float *inM)			{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }

// Description of a convex object, must match ShapeDesc in GJKEPAParity.zig
struct ShapeDesc
{
	int					mKind;					// 0 = point, 1 = sphere, 2 = box, 3 = triangle, 4 = polygon
	int					mMode;					// 0 = plain, 1 = AddConvexRadius, 2 = TransformedConvexObject, 3 = TransformedConvexObject(AddConvexRadius), 4 = AddConvexRadius(TransformedConvexObject)
	float				mRadius;				// Radius of the AddConvexRadius wrapper
	int					mCount;					// Number of polygon vertices
	float				mTransform[16];			// Transform of the TransformedConvexObject wrapper
	float				mData[3 * 32];			// point: position; sphere: center, radius; box: min, max; triangle: 3 vertices; polygon: vertices
};

using PolygonVertices = StaticArray<Vec3, 32>;

// A point, sphere, box, triangle or polygon
class BaseShape : public NonCopyable
{
public:
	explicit			BaseShape(const ShapeDesc &inDesc) :
		mKind(inDesc.mKind),
		mPolygon(mVertices)
	{
		const float *d = inDesc.mData;
		switch (mKind)
		{
		case 0:		mPoint.mPoint = Load3(d); break;
		case 1:		mSphere = Sphere(Load3(d), d[3]); break;
		case 2:		mBox = AABox(Load3(d), Load3(d + 3)); break;
		case 3:		mTriangle = TriangleConvexSupport(Load3(d), Load3(d + 3), Load3(d + 6)); break;
		default:	for (int i = 0; i < inDesc.mCount; ++i) mVertices.push_back(Load3(d + 3 * i)); break;
		}
	}

	Vec3				GetSupport(Vec3Arg inDirection) const
	{
		switch (mKind)
		{
		case 0:		return mPoint.GetSupport(inDirection);
		case 1:		return mSphere.GetSupport(inDirection);
		case 2:		return mBox.GetSupport(inDirection);
		case 3:		return mTriangle.GetSupport(inDirection);
		default:	return mPolygon.GetSupport(inDirection);
		}
	}

private:
	int					mKind;
	PointConvexSupport	mPoint { Vec3::sZero() };
	Sphere				mSphere;
	AABox				mBox;
	TriangleConvexSupport mTriangle { Vec3::sZero(), Vec3::sZero(), Vec3::sZero() };
	PolygonVertices		mVertices;
	PolygonConvexSupport<PolygonVertices> mPolygon;
};

// A BaseShape, optionally wrapped in AddConvexRadius and / or TransformedConvexObject
class Shape : public NonCopyable
{
public:
	explicit			Shape(const ShapeDesc &inDesc) :
		mBase(inDesc),
		mMode(inDesc.mMode),
		mRadius(inDesc.mRadius),
		mTransform(LoadMat44(inDesc.mTransform))
	{
	}

	Vec3				GetSupport(Vec3Arg inDirection) const
	{
		switch (mMode)
		{
		case 0:
			return mBase.GetSupport(inDirection);

		case 1:
			return AddConvexRadius<BaseShape>(mBase, mRadius).GetSupport(inDirection);

		case 2:
			return TransformedConvexObject<BaseShape>(mTransform, mBase).GetSupport(inDirection);

		case 3:
			{
				AddConvexRadius<BaseShape> rounded(mBase, mRadius);
				return TransformedConvexObject<AddConvexRadius<BaseShape>>(mTransform, rounded).GetSupport(inDirection);
			}

		default:
			{
				TransformedConvexObject<BaseShape> transformed(mTransform, mBase);
				return AddConvexRadius<TransformedConvexObject<BaseShape>>(transformed, mRadius).GetSupport(inDirection);
			}
		}
	}

private:
	BaseShape			mBase;
	int					mMode;
	float				mRadius;
	Mat44				mTransform;
};

// Simplex of the GJK algorithm, must match Simplex in GJKEPAParity.zig
struct Simplex
{
	float				mY[4][3];
	float				mP[4][3];
	float				mQ[4][3];
	uint32				mNumPoints;
};

static void StoreSimplex(const GJKClosestPoint &inGJK, Simplex &outSimplex)
{
	Vec3 y[4] = { Vec3::sZero(), Vec3::sZero(), Vec3::sZero(), Vec3::sZero() };
	Vec3 p[4] = { Vec3::sZero(), Vec3::sZero(), Vec3::sZero(), Vec3::sZero() };
	Vec3 q[4] = { Vec3::sZero(), Vec3::sZero(), Vec3::sZero(), Vec3::sZero() };
	uint num_points = 0;
	inGJK.GetClosestPointsSimplex(y, p, q, num_points);
	for (uint i = 0; i < num_points; ++i)
	{
		Store3(y[i], outSimplex.mY[i]);
		Store3(p[i], outSimplex.mP[i]);
		Store3(q[i], outSimplex.mQ[i]);
	}
	outSimplex.mNumPoints = num_points;
}

// Record of the hull builder test, must match HullRecord in GJKEPAParity.zig
struct HullRecord
{
	int					mType;					// 0 = FindFacingTriangle, 1 = AddPoint, 2 = new triangle, 3 = peeked triangle, 4 = popped triangle
	int					mValueInt;				// FindFacingTriangle: 1 if a triangle was found; AddPoint: the result
	float				mValue;					// FindFacingTriangle: the best distance^2
	int					mStartIdx[3];
	int					mNeighbourEdge[3];		// -1 if there is no neighbour
	int					mNeighbourStartIdx[3][3];
	float				mNormal[3];
	float				mCentroid[3];
	float				mClosestLenSq;
	float				mLambda[2];				// Only when the closest point is interior (otherwise Jolt may not have calculated it)
	int					mFlags;					// 1 = lambda relative to 0 (only when interior), 2 = closest point interior, 4 = removed, 8 = in queue
};

using HullTriangle = EPAConvexHullBuilder::Triangle;

static void StoreTriangle(int inType, const HullTriangle *inT, HullRecord &outRecord)
{
	memset(&outRecord, 0, sizeof(outRecord));
	outRecord.mType = inType;
	for (int e = 0; e < 3; ++e)
	{
		const EPAConvexHullBuilder::Edge &edge = inT->mEdge[e];
		outRecord.mStartIdx[e] = edge.mStartIdx;
		if (edge.mNeighbourTriangle != nullptr)
		{
			outRecord.mNeighbourEdge[e] = edge.mNeighbourEdge;
			for (int k = 0; k < 3; ++k)
				outRecord.mNeighbourStartIdx[e][k] = edge.mNeighbourTriangle->mEdge[k].mStartIdx;
		}
		else
		{
			outRecord.mNeighbourEdge[e] = -1;
			for (int k = 0; k < 3; ++k)
				outRecord.mNeighbourStartIdx[e][k] = -1;
		}
	}
	Store3(inT->mNormal, outRecord.mNormal);
	Store3(inT->mCentroid, outRecord.mCentroid);
	outRecord.mClosestLenSq = inT->mClosestLenSq;
	if (inT->mClosestPointInterior)
	{
		outRecord.mLambda[0] = inT->mLambda[0];
		outRecord.mLambda[1] = inT->mLambda[1];
		outRecord.mFlags |= inT->mLambdaRelativeTo0? 1 : 0;
		outRecord.mFlags |= 2;
	}
	outRecord.mFlags |= inT->mRemoved? 4 : 0;
	outRecord.mFlags |= inT->mInQueue? 8 : 0;
}

extern "C"
{

// Geometry/GJKClosestPoint.h
bool jolt_gjk_intersects(const ShapeDesc *inA, const ShapeDesc *inB, float inTolerance, float *ioV, Simplex *outSimplex)
{
	Shape a(*inA), b(*inB);
	GJKClosestPoint gjk;
	Vec3 v = Load3(ioV);
	bool result = gjk.Intersects(a, b, inTolerance, v);
	Store3(v, ioV);
	StoreSimplex(gjk, *outSimplex);
	return result;
}

float jolt_gjk_get_closest_points(const ShapeDesc *inA, const ShapeDesc *inB, float inTolerance, float inMaxDistSq, float *ioV, float *ioPointA, float *ioPointB, Simplex *outSimplex)
{
	Shape a(*inA), b(*inB);
	GJKClosestPoint gjk;
	Vec3 v = Load3(ioV), pa = Load3(ioPointA), pb = Load3(ioPointB);
	float result = gjk.GetClosestPoints(a, b, inTolerance, inMaxDistSq, v, pa, pb);
	Store3(v, ioV);
	Store3(pa, ioPointA);
	Store3(pb, ioPointB);
	StoreSimplex(gjk, *outSimplex);
	return result;
}

bool jolt_gjk_cast_ray(const float *inOrigin, const float *inDirection, float inTolerance, const ShapeDesc *inA, float *ioLambda, Simplex *outSimplex)
{
	Shape a(*inA);
	GJKClosestPoint gjk;
	bool result = gjk.CastRay(Load3(inOrigin), Load3(inDirection), inTolerance, a, *ioLambda);
	StoreSimplex(gjk, *outSimplex);
	return result;
}

bool jolt_gjk_cast_shape(const float *inStart, const float *inDirection, float inTolerance, const ShapeDesc *inA, const ShapeDesc *inB, float *ioLambda, Simplex *outSimplex)
{
	Shape a(*inA), b(*inB);
	GJKClosestPoint gjk;
	bool result = gjk.CastShape(LoadMat44(inStart), Load3(inDirection), inTolerance, a, b, *ioLambda);
	StoreSimplex(gjk, *outSimplex);
	return result;
}

bool jolt_gjk_cast_shape_radius(const float *inStart, const float *inDirection, float inTolerance, const ShapeDesc *inA, const ShapeDesc *inB, float inConvexRadiusA, float inConvexRadiusB, float *ioLambda, float *ioPointA, float *ioPointB, float *ioSeparatingAxis, Simplex *outSimplex)
{
	Shape a(*inA), b(*inB);
	GJKClosestPoint gjk;
	Vec3 pa = Load3(ioPointA), pb = Load3(ioPointB), axis = Load3(ioSeparatingAxis);
	bool result = gjk.CastShape(LoadMat44(inStart), Load3(inDirection), inTolerance, a, b, inConvexRadiusA, inConvexRadiusB, *ioLambda, pa, pb, axis);
	Store3(pa, ioPointA);
	Store3(pb, ioPointB);
	Store3(axis, ioSeparatingAxis);
	StoreSimplex(gjk, *outSimplex);
	return result;
}

// Geometry/EPAPenetrationDepth.h
// The GJK step followed by the EPA step (on the same variables, like GetPenetrationDepth) when the GJK step is indeterminate.
// The objects including convex radius are AddConvexRadius(A, inConvexRadiusA) (inIncludeMode = 0) or the objects themselves (inIncludeMode = 1).
// outEPAResult: -1 if the EPA step did not run, otherwise its result.
int jolt_epa_penetration_depth_steps(const ShapeDesc *inA, float inConvexRadiusA, const ShapeDesc *inB, float inConvexRadiusB, int inIncludeMode, float inCollisionTolerance, float inPenetrationTolerance, float *ioV, float *ioPointA, float *ioPointB, float *outGJKV, float *outGJKPointA, float *outGJKPointB, int *outEPAResult)
{
	Shape a(*inA), b(*inB);
	AddConvexRadius<Shape> a_incl(a, inConvexRadiusA), b_incl(b, inConvexRadiusB);
	EPAPenetrationDepth epa;
	Vec3 v = Load3(ioV), pa = Load3(ioPointA), pb = Load3(ioPointB);
	EPAPenetrationDepth::EStatus status = epa.GetPenetrationDepthStepGJK(a, inConvexRadiusA, b, inConvexRadiusB, inCollisionTolerance, v, pa, pb);
	Store3(v, outGJKV);
	Store3(pa, outGJKPointA);
	Store3(pb, outGJKPointB);
	*outEPAResult = -1;
	if (status == EPAPenetrationDepth::EStatus::Indeterminate)
		*outEPAResult = (inIncludeMode == 0?
			epa.GetPenetrationDepthStepEPA(a_incl, b_incl, inPenetrationTolerance, v, pa, pb) :
			epa.GetPenetrationDepthStepEPA(a, b, inPenetrationTolerance, v, pa, pb))? 1 : 0;
	Store3(v, ioV);
	Store3(pa, ioPointA);
	Store3(pb, ioPointB);
	return int(status);
}

bool jolt_epa_get_penetration_depth(const ShapeDesc *inA, float inConvexRadiusA, const ShapeDesc *inB, float inConvexRadiusB, int inIncludeMode, float inCollisionToleranceSq, float inPenetrationTolerance, float *ioV, float *ioPointA, float *ioPointB)
{
	Shape a(*inA), b(*inB);
	AddConvexRadius<Shape> a_incl(a, inConvexRadiusA), b_incl(b, inConvexRadiusB);
	EPAPenetrationDepth epa;
	Vec3 v = Load3(ioV), pa = Load3(ioPointA), pb = Load3(ioPointB);
	bool result = inIncludeMode == 0?
		epa.GetPenetrationDepth(a, a_incl, inConvexRadiusA, b, b_incl, inConvexRadiusB, inCollisionToleranceSq, inPenetrationTolerance, v, pa, pb) :
		epa.GetPenetrationDepth(a, a, inConvexRadiusA, b, b, inConvexRadiusB, inCollisionToleranceSq, inPenetrationTolerance, v, pa, pb);
	Store3(v, ioV);
	Store3(pa, ioPointA);
	Store3(pb, ioPointB);
	return result;
}

bool jolt_epa_cast_shape(const float *inStart, const float *inDirection, float inCollisionTolerance, float inPenetrationTolerance, const ShapeDesc *inA, const ShapeDesc *inB, float inConvexRadiusA, float inConvexRadiusB, int inReturnDeepestPoint, float *ioLambda, float *ioPointA, float *ioPointB, float *ioContactNormal)
{
	Shape a(*inA), b(*inB);
	EPAPenetrationDepth epa;
	Vec3 pa = Load3(ioPointA), pb = Load3(ioPointB), normal = Load3(ioContactNormal);
	bool result = epa.CastShape(LoadMat44(inStart), Load3(inDirection), inCollisionTolerance, inPenetrationTolerance, a, b, inConvexRadiusA, inConvexRadiusB, inReturnDeepestPoint != 0, *ioLambda, pa, pb, normal);
	Store3(pa, ioPointA);
	Store3(pb, ioPointB);
	Store3(normal, ioContactNormal);
	return result;
}

// Geometry/EPAConvexHullBuilder.h
// Builds a hull like the first phase of EPA: Initialize(0, 1, 2), then for every further point FindFacingTriangle + AddPoint
// (with inClosestDistSq[i] as the closest distance). Every inPopEvery points the closest triangle is peeked and popped
// (and freed if removed), at the end all triangles are popped. Returns the number of records written.
int jolt_epa_hull(const float *inPoints, int inNumPoints, const float *inClosestDistSq, int inPopEvery, HullRecord *outRecords)
{
	EPAConvexHullBuilder::Points points;
	for (int i = 0; i < inNumPoints; ++i)
		points.push_back(Load3(inPoints + 3 * i));

	int num_records = 0;
	EPAConvexHullBuilder hull(points);
	hull.Initialize(0, 1, 2);
	for (int i = 3; i < inNumPoints; ++i)
	{
		float dist_sq;
		HullTriangle *t = hull.FindFacingTriangle(points[i], dist_sq);
		HullRecord &facing = outRecords[num_records++];
		if (t != nullptr)
			StoreTriangle(0, t, facing);
		else
			memset(&facing, 0, sizeof(facing));
		facing.mValueInt = t != nullptr? 1 : 0;
		facing.mValue = dist_sq;

		if (t != nullptr)
		{
			EPAConvexHullBuilder::NewTriangles new_triangles;
			bool ok = hull.AddPoint(t, i, inClosestDistSq[i], new_triangles);
			HullRecord &add = outRecords[num_records++];
			memset(&add, 0, sizeof(add));
			add.mType = 1;
			add.mValueInt = ok? 1 : 0;
			add.mValue = float(new_triangles.size());
			for (const HullTriangle *nt : new_triangles)
				StoreTriangle(2, nt, outRecords[num_records++]);
			if (!ok)
				return num_records; // The hull is broken after a failed AddPoint
		}

		if (inPopEvery > 0 && i % inPopEvery == 0 && hull.HasNextTriangle())
		{
			StoreTriangle(3, hull.PeekClosestTriangleInQueue(), outRecords[num_records++]);
			HullTriangle *pt = hull.PopClosestTriangleFromQueue();
			StoreTriangle(4, pt, outRecords[num_records++]);
			if (pt->mRemoved)
				hull.FreeTriangle(pt);
		}
	}

	while (hull.HasNextTriangle())
	{
		HullTriangle *pt = hull.PopClosestTriangleFromQueue();
		StoreTriangle(4, pt, outRecords[num_records++]);
		if (pt->mRemoved)
			hull.FreeTriangle(pt);
	}

	return num_records;
}

} // extern "C"
