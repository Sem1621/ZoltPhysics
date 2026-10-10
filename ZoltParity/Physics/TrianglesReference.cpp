// Reference implementation for the triangle collision algorithm parity tests (`zig build parity`): thin C ABI wrappers
// around the C++ Jolt library for CollideConvexVsTriangles, CollideSphereVsTriangles, CastConvexVsTriangles,
// CastSphereVsTriangles, ManifoldBetweenTwoFaces / PruneContactPoints, InternalEdgeRemovingCollector,
// CollideShapeVsShapePerLeaf and CollideSoftBodyVerticesVsTriangles. The triangle classes are driven directly with
// triangle sequences (no triangle shape exists in Zolt yet), the convex shapes (SphereShape, BoxShape) are built from
// their settings. ZoltParity/Physics/TrianglesParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), Mat44 as 16 floats in column major order,
// planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see the porting guide). The structs
// must match the extern structs in TrianglesParity.zig. All functions are prefixed with jolt_triangles_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/CastConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CastSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollideConvexVsTriangles.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideShapeVsShapePerLeaf.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h> // CollideSoftBodyVerticesVsTriangles.h uses it without including it
#include <Jolt/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h>
#include <Jolt/Physics/Collision/CollideSphereVsTriangles.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/InternalEdgeRemovingCollector.h>
#include <Jolt/Physics/Collision/ManifoldBetweenTwoFaces.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

using namespace JPH;

namespace {

constexpr uint cMaxTriangles = 64;
constexpr uint cMaxHits = 64;
constexpr uint cMaxSoftBodyVertices = 16;
constexpr uint cMaxSoftBodyTriangles = 32;

void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

Vec3 Load3(const float *inV)						{ return Vec3(inV[0], inV[1], inV[2]); }
Vec4 Load4(const float *inV)						{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
Mat44 LoadMat44(const float *inM)					{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
void Store3(Vec3Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
void StorePlane(const Plane &inPlane, float *outPlane) { Store3(inPlane.GetNormal(), outPlane); outPlane[3] = inPlane.GetConstant(); }
SubShapeIDCreator MakeCreator(const uint32 *inCreator) { return SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }
SubShapeID MakeID(uint32 inValue)					{ SubShapeID id; id.SetValue(inValue); return id; }

// Shape description, must match TriShapeDesc in TrianglesParity.zig
struct TriShapeDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape
	float					mConvexRadius;			// BoxShape
};

// Build the shape from its settings (the descriptions are always valid)
Ref<Shape> CreateShape(const TriShapeDesc &inDesc)
{
	Shape::ShapeResult result;
	if (inDesc.mKind == 0)
	{
		SphereShapeSettings settings(inDesc.mRadius);
		settings.SetEmbedded();
		result = settings.Create();
	}
	else
	{
		BoxShapeSettings settings(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
		settings.SetEmbedded();
		result = settings.Create();
	}
	return result.Get();
}

// A triangle, must match TriangleInput in TrianglesParity.zig
struct TriangleInput
{
	float					mV[9];
	uint32					mActiveEdges;
	uint32					mSubShapeID2;
};

// A hit, must match HitOutput in TrianglesParity.zig (also the input of the recorded hits)
struct HitOutput
{
	float					mFraction;				// Cast only
	int						mBackFace;				// Cast only
	float					mPoint1[3];
	float					mPoint2[3];
	float					mAxis[3];
	float					mDepth;
	uint32					mID1;
	uint32					mID2;
	uint32					mBodyID;
	uint32					mFace1Count;
	uint32					mFace2Count;
	float					mFace1[32 * 3];
	float					mFace2[32 * 3];
};

// All hits of a query in order, must match HitsOutput in TrianglesParity.zig
struct HitsOutput
{
	uint32					mNumHits;
	float					mEarlyOut;				// Early out fraction of the collector after the query
	float					mWrapperEarlyOut;		// Early out fraction of the InternalEdgeRemovingCollector (when used)
	uint32					mPadding;
	HitOutput				mHits[cMaxHits];
};

// Collide a convex shape with triangles, must match CollideTrianglesInput in TrianglesParity.zig
struct CollideTrianglesInput
{
	TriShapeDesc			mShape1;
	float					mScale1[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	uint32					mSubShapeID1;
	int						mActiveEdgeMode;		// 1: CollideWithAll
	int						mCollectFaces;
	int						mBackFaceMode;			// 1: CollideWithBackFaces
	float					mMaxSeparationDistance;
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	float					mActiveEdgeMovementDirection[3];
	float					mVertexToleranceSq;
	int						mCollector;				// 0: AllHit, 1: ClosestHit, 2: AnyHit
	int						mUseSphereCollider;		// 0: CollideConvexVsTriangles, 1: CollideSphereVsTriangles
	int						mInternalEdgeRemoval;	// Wrap the collector in an InternalEdgeRemovingCollector (and Flush)
	float					mEarlyOut;				// Early out fraction of the collector (when < the initial one)
	uint32					mBodyID;				// Body ID of the collector context
	uint32					mNumTriangles;
	TriangleInput			mTriangles[cMaxTriangles];
};

// Cast a convex shape against triangles, must match CastTrianglesInput in TrianglesParity.zig
struct CastTrianglesInput
{
	TriShapeDesc			mShape1;
	float					mScale1[3];
	float					mStart[16];
	float					mDirection[3];
	float					mScale2[3];
	float					mTransform2[16];
	uint32					mCreator1[2];
	int						mActiveEdgeMode;		// 1: CollideWithAll
	int						mCollectFaces;
	int						mBackFaceModeTriangles;	// 1: CollideWithBackFaces
	int						mBackFaceModeConvex;	// 1: CollideWithBackFaces
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	float					mExtraConvexRadius;
	int						mUseShrunkenShape;
	int						mReturnDeepestPoint;
	float					mActiveEdgeMovementDirection[3];
	int						mCollector;				// 0: AllHit, 1: ClosestHit
	int						mUseSphereCaster;		// 0: CastConvexVsTriangles, 1: CastSphereVsTriangles
	float					mEarlyOut;				// Early out fraction of the collector (when < the initial one)
	uint32					mBodyID;
	uint32					mNumTriangles;
	TriangleInput			mTriangles[cMaxTriangles];
};

// ManifoldBetweenTwoFaces (+ PruneContactPoints), must match ManifoldInput / ManifoldOutput in TrianglesParity.zig
struct ManifoldInput
{
	float					mContactPoint1[3];
	float					mContactPoint2[3];
	float					mPenetrationAxis[3];
	float					mMaxContactDistance;
	uint32					mFace1Count;
	uint32					mFace2Count;
	float					mFace1[32 * 3];
	float					mFace2[32 * 3];
	uint32					mNumExisting;			// Points already in the output arrays
	float					mExisting1[32 * 3];
	float					mExisting2[32 * 3];
	int						mPrune;					// Call PruneContactPoints with the normalized axis when there are > 4 points
};

struct ManifoldOutput
{
	uint32					mCount1;
	uint32					mCount2;
	float					mPoints1[64 * 3];
	float					mPoints2[64 * 3];
};

// Recorded hits for an InternalEdgeRemovingCollector, must match RecordedHitsInput in TrianglesParity.zig
struct RecordedHitsInput
{
	uint32					mNumHits;
	float					mVertexToleranceSq;
	int						mCollector;				// 0: AllHit, 1: ClosestHit, 2: ClosestHitPerBody
	int						mUseBodies;				// Call OnBody / OnBodyEnd around the hits of each body (otherwise Flush at the end)
	float					mEarlyOut;				// Early out fraction of the chained collector (when < the initial one)
	uint32					mBodyID;				// Body ID of the context of the chained collector
	uint32					mHitBody[cMaxHits];		// Body index of each hit (non decreasing)
	HitOutput				mHits[cMaxHits];
};

// A pair of shapes, must match ShapePairInput in TrianglesParity.zig
struct ShapePairInput
{
	TriShapeDesc			mShape1;
	TriShapeDesc			mShape2;
	float					mScale1[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	int						mMode;					// 0: InternalEdgeRemovingCollector::sCollideShapeVsShape, 1: CollideShapeVsShapePerLeaf<AnyHit>, 2: CollideShapeVsShapePerLeaf<ClosestHit>
	int						mActiveEdgeMode;		// 1: CollideWithAll
	int						mCollectFaces;
	float					mMaxSeparationDistance;
	float					mVertexToleranceSq;
	int						mCollector;				// 0: AllHit, 1: ClosestHit
	float					mEarlyOut;
	uint32					mBodyID;
};

// CollideSoftBodyVerticesVsTriangles, must match SoftBodyInput / SoftBodyOutput in TrianglesParity.zig
struct SoftBodyInput
{
	float					mTransform[16];
	float					mScale[3];
	float					mTriangleThickness;
	uint32					mNumVertices;
	int						mCollidingShapeIndex;
	float					mPositions[cMaxSoftBodyVertices * 3];
	float					mPenetrations[cMaxSoftBodyVertices];
	float					mPlanes[cMaxSoftBodyVertices * 4];
	int						mIndices[cMaxSoftBodyVertices];
	uint32					mNumTriangles[cMaxSoftBodyVertices]; // Number of triangles processed for each vertex (a prefix of mTriangles)
	float					mTriangles[cMaxSoftBodyTriangles * 9];
};

struct SoftBodyOutput
{
	float					mPenetrations[cMaxSoftBodyVertices];
	float					mPlanes[cMaxSoftBodyVertices * 4];
	int						mIndices[cMaxSoftBodyVertices];
};

void StoreFace(const Shape::SupportingFace &inFace, uint32 &outCount, float *outFace)
{
	outCount = uint32(inFace.size());
	for (uint32 i = 0; i < outCount; ++i)
		Store3(inFace[i], outFace + 3 * i);
}

void StoreCollideHit(const CollideShapeResult &inResult, HitOutput &outHit)
{
	Store3(inResult.mContactPointOn1, outHit.mPoint1);
	Store3(inResult.mContactPointOn2, outHit.mPoint2);
	Store3(inResult.mPenetrationAxis, outHit.mAxis);
	outHit.mDepth = inResult.mPenetrationDepth;
	outHit.mID1 = inResult.mSubShapeID1.GetValue();
	outHit.mID2 = inResult.mSubShapeID2.GetValue();
	outHit.mBodyID = inResult.mBodyID2.GetIndexAndSequenceNumber();
	StoreFace(inResult.mShape1Face, outHit.mFace1Count, outHit.mFace1);
	StoreFace(inResult.mShape2Face, outHit.mFace2Count, outHit.mFace2);
}

void AddCollideHit(const CollideShapeResult &inResult, HitsOutput &ioOutput)
{
	if (ioOutput.mNumHits < cMaxHits)
	{
		HitOutput &h = ioOutput.mHits[ioOutput.mNumHits];
		h.mFraction = 0.0f;
		h.mBackFace = 0;
		StoreCollideHit(inResult, h);
	}
	++ioOutput.mNumHits;
}

void AddCastHit(const ShapeCastResult &inResult, HitsOutput &ioOutput)
{
	if (ioOutput.mNumHits < cMaxHits)
	{
		HitOutput &h = ioOutput.mHits[ioOutput.mNumHits];
		h.mFraction = inResult.mFraction;
		h.mBackFace = inResult.mIsBackFaceHit? 1 : 0;
		StoreCollideHit(inResult, h);
	}
	++ioOutput.mNumHits;
}

CollideShapeResult LoadHit(const HitOutput &inHit)
{
	CollideShapeResult r(Load3(inHit.mPoint1), Load3(inHit.mPoint2), Load3(inHit.mAxis), inHit.mDepth, MakeID(inHit.mID1), MakeID(inHit.mID2), BodyID(inHit.mBodyID));
	for (uint32 i = 0; i < inHit.mFace1Count; ++i)
		r.mShape1Face.push_back(Load3(inHit.mFace1 + 3 * i));
	for (uint32 i = 0; i < inHit.mFace2Count; ++i)
		r.mShape2Face.push_back(Load3(inHit.mFace2 + 3 * i));
	return r;
}

// Store the hits of a collector into ioOutput (for AllHit / ClosestHitPerBody: all hits in order, for ClosestHit / AnyHit: the hit if any)
template <class Collector>
void StoreCollectorHits(const Collector &inCollector, HitsOutput &ioOutput)
{
	for (const CollideShapeResult &r : inCollector.mHits)
		AddCollideHit(r, ioOutput);
}

// Run inQuery(collector) with the requested collector type, the context and the early out fraction
template <class CollectorBase, class Query, class Store>
void RunWithCollector(int inCollector, uint32 inBodyID, float inEarlyOut, HitsOutput &ioOutput, const Query &inQuery, const Store &inStore)
{
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inBodyID));
	switch (inCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CollectorBase> collector;
			collector.SetContext(&context);
			if (inEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inEarlyOut);
			inQuery(collector);
			for (const typename CollectorBase::ResultType &r : collector.mHits)
				inStore(r, ioOutput);
			ioOutput.mEarlyOut = collector.GetEarlyOutFraction();
			break;
		}

	case 1:
		{
			ClosestHitCollisionCollector<CollectorBase> collector;
			collector.SetContext(&context);
			if (inEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inEarlyOut);
			inQuery(collector);
			if (collector.HadHit())
				inStore(collector.mHit, ioOutput);
			ioOutput.mEarlyOut = collector.GetEarlyOutFraction();
			break;
		}

	default:
		{
			AnyHitCollisionCollector<CollectorBase> collector;
			collector.SetContext(&context);
			if (inEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inEarlyOut);
			inQuery(collector);
			if (collector.HadHit())
				inStore(collector.mHit, ioOutput);
			ioOutput.mEarlyOut = collector.GetEarlyOutFraction();
			break;
		}
	}
}

} // namespace

extern "C" {

// CollideConvexVsTriangles / CollideSphereVsTriangles with a sequence of triangles (optionally through an
// InternalEdgeRemovingCollector that is flushed at the end), all hits in order
void jolt_triangles_collide(const CollideTrianglesInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	CollideShapeSettings settings;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mBackFaceMode = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	settings.mInternalEdgeRemovalVertexToleranceSq = inInput->mVertexToleranceSq;
	Vec3 scale1 = Load3(inInput->mScale1);
	Vec3 scale2 = Load3(inInput->mScale2);
	Mat44 transform1 = LoadMat44(inInput->mTransform1);
	Mat44 transform2 = LoadMat44(inInput->mTransform2);
	SubShapeID id1 = MakeID(inInput->mSubShapeID1);

	auto collide = [&](CollideShapeCollector &ioCollector) {
		if (inInput->mUseSphereCollider != 0)
		{
			CollideSphereVsTriangles collider(static_cast<const SphereShape *>(shape1.GetPtr()), scale1, scale2, transform1, transform2, id1, settings, ioCollector);
			for (uint i = 0; i < inInput->mNumTriangles; ++i)
			{
				const TriangleInput &t = inInput->mTriangles[i];
				collider.Collide(Load3(t.mV), Load3(t.mV + 3), Load3(t.mV + 6), uint8(t.mActiveEdges), MakeID(t.mSubShapeID2));
			}
		}
		else
		{
			CollideConvexVsTriangles collider(static_cast<const ConvexShape *>(shape1.GetPtr()), scale1, scale2, transform1, transform2, id1, settings, ioCollector);
			for (uint i = 0; i < inInput->mNumTriangles; ++i)
			{
				const TriangleInput &t = inInput->mTriangles[i];
				collider.Collide(Load3(t.mV), Load3(t.mV + 3), Load3(t.mV + 6), uint8(t.mActiveEdges), MakeID(t.mSubShapeID2));
			}
		}
	};

	outOutput->mNumHits = 0;
	outOutput->mWrapperEarlyOut = 0.0f;
	RunWithCollector<CollideShapeCollector>(inInput->mCollector, inInput->mBodyID, inInput->mEarlyOut, *outOutput, [&](CollideShapeCollector &ioCollector) {
		if (inInput->mInternalEdgeRemoval != 0)
		{
			InternalEdgeRemovingCollector wrapper(ioCollector, settings.mInternalEdgeRemovalVertexToleranceSq);
			collide(wrapper);
			wrapper.Flush();
			outOutput->mWrapperEarlyOut = wrapper.GetEarlyOutFraction();
		}
		else
			collide(ioCollector);
	}, AddCollideHit);
}

// CastConvexVsTriangles / CastSphereVsTriangles with a sequence of triangles, all hits in order
void jolt_triangles_cast(const CastTrianglesInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	ShapeCastSettings settings;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mBackFaceModeTriangles = inInput->mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mBackFaceModeConvex = inInput->mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mUseShrunkenShapeAndConvexRadius = inInput->mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = inInput->mReturnDeepestPoint != 0;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	ShapeCast shape_cast(shape1, Load3(inInput->mScale1), LoadMat44(inInput->mStart), Load3(inInput->mDirection));
	Vec3 scale2 = Load3(inInput->mScale2);
	Mat44 transform2 = LoadMat44(inInput->mTransform2);
	SubShapeIDCreator creator1 = MakeCreator(inInput->mCreator1);

	outOutput->mNumHits = 0;
	outOutput->mWrapperEarlyOut = 0.0f;
	RunWithCollector<CastShapeCollector>(inInput->mCollector, inInput->mBodyID, inInput->mEarlyOut, *outOutput, [&](CastShapeCollector &ioCollector) {
		if (inInput->mUseSphereCaster != 0)
		{
			CastSphereVsTriangles caster(shape_cast, settings, scale2, transform2, creator1, ioCollector);
			for (uint i = 0; i < inInput->mNumTriangles; ++i)
			{
				const TriangleInput &t = inInput->mTriangles[i];
				caster.Cast(Load3(t.mV), Load3(t.mV + 3), Load3(t.mV + 6), uint8(t.mActiveEdges), MakeID(t.mSubShapeID2));
			}
		}
		else
		{
			CastConvexVsTriangles caster(shape_cast, settings, scale2, transform2, creator1, ioCollector);
			for (uint i = 0; i < inInput->mNumTriangles; ++i)
			{
				const TriangleInput &t = inInput->mTriangles[i];
				caster.Cast(Load3(t.mV), Load3(t.mV + 3), Load3(t.mV + 6), uint8(t.mActiveEdges), MakeID(t.mSubShapeID2));
			}
		}
	}, AddCastHit);
}

// ManifoldBetweenTwoFaces, appending to the existing points, then PruneContactPoints when requested
void jolt_triangles_manifold(const ManifoldInput *inInput, ManifoldOutput *outOutput)
{
	EnsureFactory();
	Shape::SupportingFace face1, face2;
	for (uint32 i = 0; i < inInput->mFace1Count; ++i)
		face1.push_back(Load3(inInput->mFace1 + 3 * i));
	for (uint32 i = 0; i < inInput->mFace2Count; ++i)
		face2.push_back(Load3(inInput->mFace2 + 3 * i));
	ContactPoints points1, points2;
	for (uint32 i = 0; i < inInput->mNumExisting; ++i)
	{
		points1.push_back(Load3(inInput->mExisting1 + 3 * i));
		points2.push_back(Load3(inInput->mExisting2 + 3 * i));
	}
	Vec3 axis = Load3(inInput->mPenetrationAxis);
	ManifoldBetweenTwoFaces(Load3(inInput->mContactPoint1), Load3(inInput->mContactPoint2), axis, inInput->mMaxContactDistance, face1, face2, points1, points2);
	if (inInput->mPrune != 0 && points1.size() > 4)
		PruneContactPoints(axis.Normalized(), points1, points2);
	outOutput->mCount1 = uint32(points1.size());
	outOutput->mCount2 = uint32(points2.size());
	for (uint32 i = 0; i < outOutput->mCount1; ++i)
		Store3(points1[i], outOutput->mPoints1 + 3 * i);
	for (uint32 i = 0; i < outOutput->mCount2; ++i)
		Store3(points2[i], outOutput->mPoints2 + 3 * i);
}

// PruneContactPoints on inCount (> 4) points
void jolt_triangles_prune(const float *inAxis, uint32 inCount, const float *inPoints1, const float *inPoints2, ManifoldOutput *outOutput)
{
	EnsureFactory();
	ContactPoints points1, points2;
	for (uint32 i = 0; i < inCount; ++i)
	{
		points1.push_back(Load3(inPoints1 + 3 * i));
		points2.push_back(Load3(inPoints2 + 3 * i));
	}
	PruneContactPoints(Load3(inAxis), points1, points2);
	outOutput->mCount1 = uint32(points1.size());
	outOutput->mCount2 = uint32(points2.size());
	for (uint32 i = 0; i < outOutput->mCount1; ++i)
		Store3(points1[i], outOutput->mPoints1 + 3 * i);
	for (uint32 i = 0; i < outOutput->mCount2; ++i)
		Store3(points2[i], outOutput->mPoints2 + 3 * i);
}

// An InternalEdgeRemovingCollector fed with recorded hits (OnBody / OnBodyEnd per body or a Flush at the end), the hits
// that reach the chained collector in order
void jolt_triangles_internal_edges(const RecordedHitsInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));

	auto feed = [inInput, outOutput](CollideShapeCollector &ioChained) {
		InternalEdgeRemovingCollector wrapper(ioChained, inInput->mVertexToleranceSq);
		if (inInput->mUseBodies != 0)
		{
			uint32 current_body = ~uint32(0);
			for (uint32 i = 0; i < inInput->mNumHits; ++i)
			{
				if (inInput->mHitBody[i] != current_body)
				{
					if (current_body != ~uint32(0))
						wrapper.OnBodyEnd();
					wrapper.OnBody(Body::sFixedToWorld);
					current_body = inInput->mHitBody[i];
				}
				wrapper.AddHit(LoadHit(inInput->mHits[i]));
			}
			if (current_body != ~uint32(0))
				wrapper.OnBodyEnd();
		}
		else
		{
			for (uint32 i = 0; i < inInput->mNumHits; ++i)
				wrapper.AddHit(LoadHit(inInput->mHits[i]));
			wrapper.Flush();
		}
		outOutput->mWrapperEarlyOut = wrapper.GetEarlyOutFraction();
	};

	outOutput->mNumHits = 0;
	switch (inInput->mCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CollideShapeCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			feed(collector);
			StoreCollectorHits(collector, *outOutput);
			outOutput->mEarlyOut = collector.GetEarlyOutFraction();
			break;
		}

	case 1:
		{
			ClosestHitCollisionCollector<CollideShapeCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			feed(collector);
			if (collector.HadHit())
				AddCollideHit(collector.mHit, *outOutput);
			outOutput->mEarlyOut = collector.GetEarlyOutFraction();
			break;
		}

	default:
		{
			ClosestHitPerBodyCollisionCollector<CollideShapeCollector> collector;
			collector.SetContext(&context);
			feed(collector);
			StoreCollectorHits(collector, *outOutput);
			outOutput->mEarlyOut = collector.GetEarlyOutFraction();
			break;
		}
	}
}

// InternalEdgeRemovingCollector::sCollideShapeVsShape and CollideShapeVsShapePerLeaf on a pair of convex shapes
void jolt_triangles_shape_pair(const ShapePairInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	CollideShapeSettings settings;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mInternalEdgeRemovalVertexToleranceSq = inInput->mVertexToleranceSq;
	Vec3 scale1 = Load3(inInput->mScale1);
	Vec3 scale2 = Load3(inInput->mScale2);
	Mat44 transform1 = LoadMat44(inInput->mTransform1);
	Mat44 transform2 = LoadMat44(inInput->mTransform2);
	SubShapeIDCreator creator1 = MakeCreator(inInput->mCreator1);
	SubShapeIDCreator creator2 = MakeCreator(inInput->mCreator2);

	outOutput->mNumHits = 0;
	outOutput->mWrapperEarlyOut = 0.0f;
	RunWithCollector<CollideShapeCollector>(inInput->mCollector, inInput->mBodyID, inInput->mEarlyOut, *outOutput, [&](CollideShapeCollector &ioCollector) {
		switch (inInput->mMode)
		{
		case 0:
			InternalEdgeRemovingCollector::sCollideShapeVsShape(shape1, shape2, scale1, scale2, transform1, transform2, creator1, creator2, settings, ioCollector);
			break;

		case 1:
			CollideShapeVsShapePerLeaf<AnyHitCollisionCollector<CollideShapeCollector>>(shape1, shape2, scale1, scale2, transform1, transform2, creator1, creator2, settings, ioCollector);
			break;

		default:
			CollideShapeVsShapePerLeaf<ClosestHitCollisionCollector<CollideShapeCollector>>(shape1, shape2, scale1, scale2, transform1, transform2, creator1, creator2, settings, ioCollector);
			break;
		}
	}, AddCollideHit);
}

// CollideSoftBodyVerticesVsTriangles: StartVertex, ProcessTriangle for a prefix of the triangles, FinishVertex for every vertex
void jolt_triangles_soft_body(const SoftBodyInput *inInput, SoftBodyOutput *outOutput)
{
	EnsureFactory();
	float old_thickness = CollideSoftBodyVerticesVsTriangles::sTriangleThickness;
	CollideSoftBodyVerticesVsTriangles::sTriangleThickness = inInput->mTriangleThickness;

	Vec3 positions[cMaxSoftBodyVertices];
	Plane planes[cMaxSoftBodyVertices];
	float inv_masses[cMaxSoftBodyVertices];
	for (uint32 i = 0; i < inInput->mNumVertices; ++i)
	{
		positions[i] = Load3(inInput->mPositions + 3 * i);
		planes[i] = Plane(Load4(inInput->mPlanes + 4 * i));
		inv_masses[i] = 1.0f;
		outOutput->mPenetrations[i] = inInput->mPenetrations[i];
		outOutput->mIndices[i] = inInput->mIndices[i];
	}

	CollideSoftBodyVerticesVsTriangles collider(LoadMat44(inInput->mTransform), Load3(inInput->mScale));
	for (uint32 i = 0; i < inInput->mNumVertices; ++i)
	{
		CollideSoftBodyVertexIterator vertex(StridedPtr<const Vec3>(positions + i), StridedPtr<const float>(inv_masses + i), StridedPtr<Plane>(planes + i), StridedPtr<float>(outOutput->mPenetrations + i), StridedPtr<int>(outOutput->mIndices + i));
		collider.StartVertex(vertex);
		for (uint32 t = 0; t < inInput->mNumTriangles[i]; ++t)
		{
			const float *v = inInput->mTriangles + 9 * t;
			collider.ProcessTriangle(Load3(v), Load3(v + 3), Load3(v + 6));
		}
		collider.FinishVertex(vertex, inInput->mCollidingShapeIndex);
	}

	for (uint32 i = 0; i < inInput->mNumVertices; ++i)
		StorePlane(planes[i], outOutput->mPlanes + 4 * i);

	CollideSoftBodyVerticesVsTriangles::sTriangleThickness = old_thickness;
}

} // extern "C"
