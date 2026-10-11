// Reference implementation of the NarrowPhaseQuery parity tests (`zig build parity`): C ABI wrappers around the C++ Jolt
// library that put the shapes of the catalogue of PairwiseReference.cpp (every shape type: convex shapes, decorators,
// compounds, meshes, height fields, planes, empty shapes) into the bodies of a JPH::PhysicsSystem with the layers of
// UnitTests/Layers.h and the QuadTree broad phase, and run every NarrowPhaseQuery (CastRay with a single hit and with a
// collector, CollidePoint, CollideShape, CollideShapeWithInternalEdgeRemoval, CastShape, CollectTransformedShapes) with
// filters and collectors that log every call. EstimateCollisionResponse runs on bodies created from a description.
// ZoltParity/Physics/NarrowPhaseQueryParity.zig does the same with Zolt and checks that both produce the same bits.
//
// Conventions: like PairwiseReference.cpp every query writes its results into a stream of uint32 (floats as their bits,
// see OutStream) and returns the number of values (only the first inCapacity values are written); the layout must match
// NarrowPhaseQueryParity.zig. The structs must match the extern structs there. Booleans are passed as uint32 (never bool,
// see the porting guide). Everything except the C ABI is in an anonymous namespace. All functions have the prefix npq_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cstring>

using namespace JPH;

// The layers of the unit tests (Layers, BroadPhaseLayers, ObjectLayerPairFilterImpl, BPLayerInterfaceImpl,
// ObjectVsBroadPhaseLayerFilterImpl), mirrored by ZoltTests/Layers.zig
#include <UnitTests/Layers.h>

// The shape of a node of the catalogue of PairwiseReference.cpp
extern "C" const Shape *pw_catalogue_shape(void *inCatalogue, uint32 inIndex);

namespace {

// Section markers in the streams, must match NarrowPhaseQueryParity.zig
enum : uint32
{
	cNpqMarkerQuery = 0xD0000001,
	cNpqMarkerOnBody = 0xD0000002,
	cNpqMarkerHit = 0xD0000003,
	cNpqMarkerOnBodyEnd = 0xD0000004,
	cNpqMarkerResult = 0xD0000005,
	cNpqMarkerFilters = 0xD0000006,
	cNpqMarkerEstimate = 0xD0000007,
};

// Query kinds, must match QueryKind in NarrowPhaseQueryParity.zig
enum : uint32
{
	cNpqRay,
	cNpqRayCollector,
	cNpqPoint,
	cNpqCollide,
	cNpqCollideInternalEdgeRemoval,
	cNpqCast,
	cNpqCollect,
};

void NpqEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

// Must match NpqBody in NarrowPhaseQueryParity.zig
struct NpqBody
{
	double					mPosition[3];
	float					mRotation[4];
	uint32					mNode;					// Catalogue node of the shape
	uint32					mObjectLayer;
	uint32					mMotionType;			// EMotionType
	uint32					mUserData;
};

// Must match NpqQuery in NarrowPhaseQueryParity.zig
struct NpqQuery
{
	double					mOrigin[3];				// Ray origin, the point of CollidePoint, the position (center of mass) of the shape of CollideShape / CastShape
	double					mBaseOffset[3];			// CollideShape / CastShape
	uint32					mKind;					// See the query kinds
	uint32					mLocking;				// GetNarrowPhaseQuery or GetNarrowPhaseQueryNoLock
	uint32					mCollector;				// 0..3: NpqLoggingCollector mode, 4 AllHit, 5 ClosestHit, 6 AnyHit, 7 ClosestHitPerBody
	float					mEarlyOut;				// Early out fraction set before the query (when it is smaller than the initial one)
	uint32					mBroadPhaseLayerMask;	// 0xffffffff: the default filter, otherwise NpqBroadPhaseLayerFilter
	uint32					mObjectLayerMask;		// 0xffffffff: the default filter, otherwise NpqObjectLayerFilter
	uint32					mBodyFilter;			// 0: the default filter, otherwise NpqBodyFilter
	uint32					mBodyRejectModulus;
	uint32					mBodyLockedRejectModulus;
	uint32					mShapeFilter;			// 0: the default filter, otherwise NpqShapeFilter
	uint32					mShapeRejectModulus;
	float					mDirection[3];			// Ray / cast direction
	float					mRayFraction;			// Initial fraction of the single hit CastRay
	uint32					mBackFaceModeTriangles;	// RayCastSettings / ShapeCastSettings
	uint32					mBackFaceModeConvex;
	uint32					mTreatConvexAsSolid;
	uint32					mShape;					// Catalogue node of the CollideShape / CastShape shape
	float					mScale[3];
	float					mRotation[4];
	float					mMaxSeparationDistance;	// CollideShapeSettings
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	uint32					mBackFaceMode;
	uint32					mActiveEdgeMode;
	uint32					mCollectFaces;
	float					mActiveEdgeMovementDirection[3];
	float					mVertexToleranceSq;
	float					mExtraConvexRadius;		// ShapeCastSettings
	uint32					mUseShrunkenShape;
	uint32					mReturnDeepestPoint;
	float					mBox[6];				// CollectTransformedShapes
};

// Must match NpqEstimateBody in NarrowPhaseQueryParity.zig
struct NpqEstimateBody
{
	double					mPosition[3];
	float					mRotation[4];
	float					mHalfExtent[3];
	float					mLinearVelocity[3];
	float					mAngularVelocity[3];
	float					mMass;
	float					mInertia[3];			// Diagonal of the inertia (mOverrideMass == 2)
	float					mInertiaRotation[4];
	float					mInertiaMultiplier;
	uint32					mMotionType;
	uint32					mAllowedDOFs;
	uint32					mOverrideMass;			// EOverrideMassProperties
};

// Must match NpqEstimateInput in NarrowPhaseQueryParity.zig
struct NpqEstimateInput
{
	NpqEstimateBody			mBody1;
	NpqEstimateBody			mBody2;
	double					mBaseOffset[3];
	float					mNormal[3];
	float					mPenetrationDepth;
	float					mPoints1[ContactPoints::Capacity][3];
	float					mPoints2[ContactPoints::Capacity][3];
	uint32					mNumPoints;
	float					mCombinedFriction;
	float					mCombinedRestitution;
	float					mMinVelocityForRestitution;
	uint32					mNumIterations;
};

Vec3 Load3(const float *inV)						{ return Vec3(inV[0], inV[1], inV[2]); }
Quat LoadQuat(const float *inV)						{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
RVec3 LoadR3(const double *inV)						{ return RVec3(Real(inV[0]), Real(inV[1]), Real(inV[2])); }

// Output stream, must match Stream in PairwiseParity.zig
class OutStream
{
public:
						OutStream(uint32 *inData, uint32 inCapacity) : mData(inData), mCapacity(inCapacity) { }

	void				U(uint32 inValue)			{ if (mSize < mCapacity) mData[mSize] = inValue; ++mSize; }
	void				B(bool inValue)				{ U(inValue? 1 : 0); }
	void				F(float inValue)			{ uint32 u; memcpy(&u, &inValue, sizeof(u)); U(u); }
	void				D(double inValue)			{ uint64 u; memcpy(&u, &inValue, sizeof(u)); U(uint32(u)); U(uint32(u >> 32)); }
	void				V3(Vec3Arg inValue)			{ F(inValue.GetX()); F(inValue.GetY()); F(inValue.GetZ()); }
	void				R3(RVec3Arg inValue)		{ D(double(inValue.GetX())); D(double(inValue.GetY())); D(double(inValue.GetZ())); }
	void				Q(QuatArg inValue)			{ F(inValue.GetX()); F(inValue.GetY()); F(inValue.GetZ()); F(inValue.GetW()); }
	void				Box(const AABox &inBox)		{ V3(inBox.mMin); V3(inBox.mMax); }
	void				Face(const Shape::SupportingFace &inFace) { U(uint32(inFace.size())); for (Vec3 v : inFace) V3(v); }

	uint32 *			mData;
	uint32				mCapacity;
	uint32				mSize = 0;
};

// A hash of the calls of a filter, must match FilterLog in NarrowPhaseQueryParity.zig
struct FilterLog
{
	void				Add(uint32 inValue)			{ mHash = (mHash ^ inValue) * 0x01000193; ++mCalls; }
	void				Write(OutStream &ioStream) const { ioStream.U(mCalls); ioStream.U(mHash); }

	uint32				mCalls = 0;
	uint32				mHash = 0x811c9dc5;
};

// Rejects a pseudo random subset (value % modulus == 0 after mixing), must match reject in NarrowPhaseQueryParity.zig
bool Reject(uint32 inModulus, uint32 inValue)
{
	return inModulus != 0 && ((inValue * 0x9e3779b1u) >> 16) % inModulus == 0;
}

// Accepts the broad phase layers in a bit mask, logs every call
class NpqBroadPhaseLayerFilter : public BroadPhaseLayerFilter
{
public:
						NpqBroadPhaseLayerFilter(uint32 inMask, FilterLog *inLog) : mMask(inMask), mLog(inLog) { }

	virtual bool		ShouldCollide(BroadPhaseLayer inLayer) const override
	{
		mLog->Add(uint32(inLayer.GetValue()));
		return ((mMask >> uint32(inLayer.GetValue())) & 1) != 0;
	}

	uint32				mMask;
	FilterLog *			mLog;
};

// Accepts the object layers in a bit mask, logs every call
class NpqObjectLayerFilter : public ObjectLayerFilter
{
public:
						NpqObjectLayerFilter(uint32 inMask, FilterLog *inLog) : mMask(inMask), mLog(inLog) { }

	virtual bool		ShouldCollide(ObjectLayer inLayer) const override
	{
		mLog->Add(uint32(inLayer));
		return ((mMask >> uint32(inLayer)) & 1) != 0;
	}

	uint32				mMask;
	FilterLog *			mLog;
};

// Rejects a pseudo random subset of the bodies before and after locking them, logs every call
class NpqBodyFilter : public BodyFilter
{
public:
						NpqBodyFilter(uint32 inRejectModulus, uint32 inLockedRejectModulus, FilterLog *inLog) : mRejectModulus(inRejectModulus), mLockedRejectModulus(inLockedRejectModulus), mLog(inLog) { }

	virtual bool		ShouldCollide(const BodyID &inBodyID) const override
	{
		mLog->Add(1);
		mLog->Add(inBodyID.GetIndexAndSequenceNumber());
		return !Reject(mRejectModulus, inBodyID.GetIndexAndSequenceNumber());
	}

	virtual bool		ShouldCollideLocked(const Body &inBody) const override
	{
		mLog->Add(2);
		mLog->Add(inBody.GetID().GetIndexAndSequenceNumber());
		mLog->Add(uint32(inBody.GetUserData()));
		return !Reject(mLockedRejectModulus, uint32(inBody.GetUserData()));
	}

	uint32				mRejectModulus;
	uint32				mLockedRejectModulus;
	FilterLog *			mLog;
};

// Rejects a pseudo random subset of the shapes (by sub shape ID and body ID), logs every call with the body ID that the
// query sets
class NpqShapeFilter : public ShapeFilter
{
public:
						NpqShapeFilter(uint32 inRejectModulus, FilterLog *inLog) : mRejectModulus(inRejectModulus), mLog(inLog) { }

	virtual bool		ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		mLog->Add(1);
		mLog->Add(uint32(inShape2->GetSubType()));
		mLog->Add(inSubShapeIDOfShape2.GetValue());
		mLog->Add(mBodyID2.GetIndexAndSequenceNumber());
		return !Reject(mRejectModulus, inSubShapeIDOfShape2.GetValue() ^ mBodyID2.GetIndexAndSequenceNumber());
	}

	virtual bool		ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		mLog->Add(2);
		mLog->Add(uint32(inShape1->GetSubType()));
		mLog->Add(inSubShapeIDOfShape1.GetValue());
		mLog->Add(uint32(inShape2->GetSubType()));
		mLog->Add(inSubShapeIDOfShape2.GetValue());
		mLog->Add(mBodyID2.GetIndexAndSequenceNumber());
		return !Reject(mRejectModulus, inSubShapeIDOfShape1.GetValue() * 0x85ebca6bu + (inSubShapeIDOfShape2.GetValue() ^ mBodyID2.GetIndexAndSequenceNumber()));
	}

	uint32				mRejectModulus;
	FilterLog *			mLog;
};

// The result writers (WriteResult in NarrowPhaseQueryParity.zig)
class ResultWriter
{
public:
						ResultWriter(OutStream &inOut, void *inCatalogue, uint32 inNumNodes) : mOut(inOut), mCatalogue(inCatalogue), mNumNodes(inNumNodes) { }

	void				Write(const RayCastResult &inResult) const
	{
		mOut.U(inResult.mBodyID.GetIndexAndSequenceNumber());
		mOut.F(inResult.mFraction);
		mOut.U(inResult.mSubShapeID2.GetValue());
	}

	void				Write(const CollidePointResult &inResult) const
	{
		mOut.U(inResult.mBodyID.GetIndexAndSequenceNumber());
		mOut.U(inResult.mSubShapeID2.GetValue());
	}

	void				Write(const CollideShapeResult &inResult) const
	{
		mOut.V3(inResult.mContactPointOn1);
		mOut.V3(inResult.mContactPointOn2);
		mOut.V3(inResult.mPenetrationAxis);
		mOut.F(inResult.mPenetrationDepth);
		mOut.U(inResult.mSubShapeID1.GetValue());
		mOut.U(inResult.mSubShapeID2.GetValue());
		mOut.U(inResult.mBodyID2.GetIndexAndSequenceNumber());
		mOut.Face(inResult.mShape1Face);
		mOut.Face(inResult.mShape2Face);
	}

	void				Write(const ShapeCastResult &inResult) const
	{
		mOut.F(inResult.mFraction);
		mOut.B(inResult.mIsBackFaceHit);
		Write(static_cast<const CollideShapeResult &>(inResult));
	}

	void				Write(const TransformedShape &inResult) const
	{
		// The shape as the index of the catalogue node (0xffffffff when it is not a node shape)
		uint32 index = 0xffffffff;
		for (uint32 i = 0; i < mNumNodes; ++i)
			if (inResult.mShape == pw_catalogue_shape(mCatalogue, i))
			{
				index = i;
				break;
			}
		mOut.U(index);
		mOut.U(uint32(inResult.mShape->GetSubType()));
		mOut.R3(inResult.mShapePositionCOM);
		mOut.Q(inResult.mShapeRotation);
		mOut.V3(inResult.GetShapeScale());
		mOut.U(inResult.mBodyID.GetIndexAndSequenceNumber());
		mOut.U(inResult.mSubShapeIDCreator.GetID().GetValue());
		mOut.U(inResult.mSubShapeIDCreator.GetNumBitsWritten());
		mOut.Box(inResult.GetWorldSpaceBounds());
	}

	OutStream &			mOut;
	void *				mCatalogue;
	uint32				mNumNodes;
};

// A collector that writes the protocol of the query (OnBody, AddHit with the body of the context, OnBodyEnd) and follows
// one of the collector strategies: 0 keeps every hit, 1 the closest hit, 2 forces an early out at the first hit, 3 the
// closest hit per body. Must match LoggingCollector in NarrowPhaseQueryParity.zig.
template <class CollectorType>
class NpqLoggingCollector : public CollectorType
{
public:
	using ResultType = typename CollectorType::ResultType;

						NpqLoggingCollector(const ResultWriter &inWriter, uint32 inMode) : mWriter(inWriter), mMode(inMode) { }

	virtual void		OnBody(const Body &inBody) override
	{
		mWriter.mOut.U(cNpqMarkerOnBody);
		mWriter.mOut.U(inBody.GetID().GetIndexAndSequenceNumber());
		mWriter.mOut.F(CollectorType::GetEarlyOutFraction());
		mPreviousEarlyOutFraction = CollectorType::GetEarlyOutFraction();
	}

	virtual void		AddHit(const ResultType &inResult) override
	{
		mWriter.mOut.U(cNpqMarkerHit);
		mWriter.mOut.U(CollectorType::GetContext()->mBodyID.GetIndexAndSequenceNumber());
		mWriter.mOut.F(CollectorType::GetEarlyOutFraction());
		mWriter.Write(inResult);
		float fraction = inResult.GetEarlyOutFraction();
		switch (mMode)
		{
		case 1:
		case 3:
			if (fraction < CollectorType::GetEarlyOutFraction())
				CollectorType::UpdateEarlyOutFraction(fraction);
			break;

		case 2:
			CollectorType::ForceEarlyOut();
			break;

		default:
			break;
		}
	}

	virtual void		OnBodyEnd() override
	{
		mWriter.mOut.U(cNpqMarkerOnBodyEnd);
		mWriter.mOut.F(CollectorType::GetEarlyOutFraction());
		if (mMode == 3)
			CollectorType::ResetEarlyOutFraction(mPreviousEarlyOutFraction);
	}

	const ResultWriter &mWriter;
	uint32				mMode;
	float				mPreviousEarlyOutFraction = 0.0f;
};

// Run a query (inQuery(collector)) with the collector of the query, write the hits and the final early out fraction
template <class CollectorType, class Query>
void RunWithCollector(const ResultWriter &inWriter, const NpqQuery &inQuery, const Query &inRun)
{
	OutStream &out = inWriter.mOut;
	auto preset = [&](CollectorType &ioCollector) {
		if (inQuery.mEarlyOut < ioCollector.GetEarlyOutFraction())
			ioCollector.UpdateEarlyOutFraction(inQuery.mEarlyOut);
	};
	switch (inQuery.mCollector)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			NpqLoggingCollector<CollectorType> collector(inWriter, inQuery.mCollector);
			preset(collector);
			inRun(collector);
			out.U(cNpqMarkerResult);
			out.F(collector.GetEarlyOutFraction());
			break;
		}

	case 4:
		{
			AllHitCollisionCollector<CollectorType> collector;
			preset(collector);
			inRun(collector);
			out.U(cNpqMarkerResult);
			out.U(uint32(collector.mHits.size()));
			for (const typename CollectorType::ResultType &hit : collector.mHits)
				inWriter.Write(hit);
			out.F(collector.GetEarlyOutFraction());
			break;
		}

	case 5:
		{
			ClosestHitCollisionCollector<CollectorType> collector;
			preset(collector);
			inRun(collector);
			out.U(cNpqMarkerResult);
			out.B(collector.HadHit());
			if (collector.HadHit())
				inWriter.Write(collector.mHit);
			out.F(collector.GetEarlyOutFraction());
			break;
		}

	case 6:
		{
			AnyHitCollisionCollector<CollectorType> collector;
			preset(collector);
			inRun(collector);
			out.U(cNpqMarkerResult);
			out.B(collector.HadHit());
			if (collector.HadHit())
				inWriter.Write(collector.mHit);
			out.F(collector.GetEarlyOutFraction());
			break;
		}

	default:
		{
			ClosestHitPerBodyCollisionCollector<CollectorType> collector;
			preset(collector);
			inRun(collector);
			out.U(cNpqMarkerResult);
			out.U(uint32(collector.mHits.size()));
			for (const typename CollectorType::ResultType &hit : collector.mHits)
				inWriter.Write(hit);
			out.F(collector.GetEarlyOutFraction());
			break;
		}
	}
}

// A PhysicsSystem with the bodies of a description (NpqWorld in NarrowPhaseQueryParity.zig)
class NpqWorld
{
public:
						NpqWorld(void *inCatalogue, const NpqBody *inBodies, uint32 inNumBodies, uint32 inMaxBodies)
	{
		mSystem.Init(inMaxBodies, 0, 64, 64, mBroadPhaseLayerInterface, mObjectVsBroadPhaseLayerFilter, mObjectLayerPairFilter);
		BodyInterface &bi = mSystem.GetBodyInterfaceNoLock();

		// Create every body, add the first half one by one and the second half as a batch
		for (uint32 i = 0; i < inNumBodies; ++i)
		{
			const NpqBody &b = inBodies[i];
			BodyCreationSettings settings(pw_catalogue_shape(inCatalogue, b.mNode), LoadR3(b.mPosition), LoadQuat(b.mRotation), EMotionType(b.mMotionType), ObjectLayer(b.mObjectLayer));
			settings.mUserData = b.mUserData;
			Body *body = bi.CreateBody(settings);
			mIDs.push_back(body->GetID());
		}
		uint32 half = inNumBodies / 2;
		for (uint32 i = 0; i < half; ++i)
			bi.AddBody(mIDs[i], EActivation::DontActivate);
		if (inNumBodies > half)
		{
			Array<BodyID> batch;
			for (uint32 i = half; i < inNumBodies; ++i)
				batch.push_back(mIDs[i]);
			BodyInterface::AddState state = bi.AddBodiesPrepare(batch.data(), int(batch.size()));
			bi.AddBodiesFinalize(batch.data(), int(batch.size()), state, EActivation::DontActivate);
		}
	}

	PhysicsSystem		mSystem;
	BPLayerInterfaceImpl mBroadPhaseLayerInterface;
	ObjectVsBroadPhaseLayerFilterImpl mObjectVsBroadPhaseLayerFilter;
	ObjectLayerPairFilterImpl mObjectLayerPairFilter;
	TempAllocatorImpl	mTempAllocator { 4 * 1024 * 1024 };
	JobSystemSingleThreaded mJobSystem { cMaxPhysicsJobs };
	Array<BodyID>		mIDs;
};

} // namespace

extern "C"
{

// Create the world: a body per description (a shape of the catalogue of PairwiseReference.cpp)
void *npq_world_create(void *inCatalogue, const NpqBody *inBodies, uint32 inNumBodies, uint32 inMaxBodies)
{
	NpqEnsureFactory();
	return new NpqWorld(inCatalogue, inBodies, inNumBodies, inMaxBodies);
}

void npq_world_destroy(void *inWorld)
{
	delete static_cast<NpqWorld *>(inWorld);
}

// BodyInterface::RemoveBody
void npq_world_remove(void *inWorld, uint32 inBody)
{
	NpqWorld &w = *static_cast<NpqWorld *>(inWorld);
	w.mSystem.GetBodyInterface().RemoveBody(w.mIDs[inBody]);
}

// BodyInterface::AddBody
void npq_world_add(void *inWorld, uint32 inBody)
{
	NpqWorld &w = *static_cast<NpqWorld *>(inWorld);
	w.mSystem.GetBodyInterface().AddBody(w.mIDs[inBody], EActivation::DontActivate);
}

// BodyInterface::SetPositionAndRotation
void npq_world_move(void *inWorld, uint32 inBody, const double *inPosition, const float *inRotation)
{
	NpqWorld &w = *static_cast<NpqWorld *>(inWorld);
	w.mSystem.GetBodyInterface().SetPositionAndRotation(w.mIDs[inBody], LoadR3(inPosition), LoadQuat(inRotation), EActivation::DontActivate);
}

// PhysicsSystem::Update (no body is active: it updates the broad phase, which rebuilds its trees)
uint32 npq_world_update(void *inWorld)
{
	NpqWorld &w = *static_cast<NpqWorld *>(inWorld);
	return uint32(w.mSystem.Update(1.0f / 60.0f, 1, &w.mTempAllocator, &w.mJobSystem));
}

// The body ID of a body of the world
uint32 npq_world_body_id(void *inWorld, uint32 inBody)
{
	return static_cast<NpqWorld *>(inWorld)->mIDs[inBody].GetIndexAndSequenceNumber();
}

// Run a query and write its protocol, hits and filter calls
uint32 npq_query(void *inWorld, void *inCatalogue, uint32 inNumNodes, const NpqQuery *inQuery, uint32 *outData, uint32 inCapacity)
{
	NpqWorld &w = *static_cast<NpqWorld *>(inWorld);
	const NpqQuery &q = *inQuery;
	OutStream out(outData, inCapacity);
	ResultWriter writer(out, inCatalogue, inNumNodes);
	const NarrowPhaseQuery &query = q.mLocking != 0? w.mSystem.GetNarrowPhaseQuery() : w.mSystem.GetNarrowPhaseQueryNoLock();

	// The filters
	FilterLog broad_phase_log, object_log, body_log, shape_log;
	NpqBroadPhaseLayerFilter npq_broad_phase_filter(q.mBroadPhaseLayerMask, &broad_phase_log);
	BroadPhaseLayerFilter default_broad_phase_filter;
	const BroadPhaseLayerFilter &broad_phase_filter = q.mBroadPhaseLayerMask != 0xffffffff? static_cast<const BroadPhaseLayerFilter &>(npq_broad_phase_filter) : default_broad_phase_filter;
	NpqObjectLayerFilter npq_object_filter(q.mObjectLayerMask, &object_log);
	ObjectLayerFilter default_object_filter;
	const ObjectLayerFilter &object_filter = q.mObjectLayerMask != 0xffffffff? static_cast<const ObjectLayerFilter &>(npq_object_filter) : default_object_filter;
	NpqBodyFilter npq_body_filter(q.mBodyRejectModulus, q.mBodyLockedRejectModulus, &body_log);
	BodyFilter default_body_filter;
	const BodyFilter &body_filter = q.mBodyFilter != 0? static_cast<const BodyFilter &>(npq_body_filter) : default_body_filter;
	NpqShapeFilter npq_shape_filter(q.mShapeRejectModulus, &shape_log);
	ShapeFilter default_shape_filter;
	const ShapeFilter &shape_filter = q.mShapeFilter != 0? static_cast<const ShapeFilter &>(npq_shape_filter) : default_shape_filter;

	out.U(cNpqMarkerQuery);
	out.U(q.mKind);
	switch (q.mKind)
	{
	case cNpqRay:
		{
			RayCastResult hit;
			hit.mFraction = q.mRayFraction;
			bool had_hit = query.CastRay(RRayCast(LoadR3(q.mOrigin), Load3(q.mDirection)), hit, broad_phase_filter, object_filter, body_filter);
			out.U(cNpqMarkerResult);
			out.B(had_hit);
			writer.Write(hit);
			break;
		}

	case cNpqRayCollector:
		{
			RayCastSettings settings;
			settings.mBackFaceModeTriangles = q.mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
			settings.mBackFaceModeConvex = q.mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
			settings.mTreatConvexAsSolid = q.mTreatConvexAsSolid != 0;
			RRayCast ray(LoadR3(q.mOrigin), Load3(q.mDirection));
			RunWithCollector<CastRayCollector>(writer, q, [&](CastRayCollector &ioCollector) { query.CastRay(ray, settings, ioCollector, broad_phase_filter, object_filter, body_filter, shape_filter); });
			break;
		}

	case cNpqPoint:
		RunWithCollector<CollidePointCollector>(writer, q, [&](CollidePointCollector &ioCollector) { query.CollidePoint(LoadR3(q.mOrigin), ioCollector, broad_phase_filter, object_filter, body_filter, shape_filter); });
		break;

	case cNpqCollide:
	case cNpqCollideInternalEdgeRemoval:
		{
			CollideShapeSettings settings;
			settings.mMaxSeparationDistance = q.mMaxSeparationDistance;
			settings.mCollisionTolerance = q.mCollisionTolerance;
			settings.mPenetrationTolerance = q.mPenetrationTolerance;
			settings.mBackFaceMode = q.mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
			settings.mActiveEdgeMode = q.mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
			settings.mCollectFacesMode = q.mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
			settings.mActiveEdgeMovementDirection = Load3(q.mActiveEdgeMovementDirection);
			settings.mInternalEdgeRemovalVertexToleranceSq = q.mVertexToleranceSq;
			const Shape *shape = pw_catalogue_shape(inCatalogue, q.mShape);
			RMat44 transform = RMat44::sRotationTranslation(LoadQuat(q.mRotation), LoadR3(q.mOrigin));
			RunWithCollector<CollideShapeCollector>(writer, q, [&](CollideShapeCollector &ioCollector) {
				if (q.mKind == cNpqCollide)
					query.CollideShape(shape, Load3(q.mScale), transform, settings, LoadR3(q.mBaseOffset), ioCollector, broad_phase_filter, object_filter, body_filter, shape_filter);
				else
					query.CollideShapeWithInternalEdgeRemoval(shape, Load3(q.mScale), transform, settings, LoadR3(q.mBaseOffset), ioCollector, broad_phase_filter, object_filter, body_filter, shape_filter);
			});
			break;
		}

	case cNpqCast:
		{
			ShapeCastSettings settings;
			settings.mCollisionTolerance = q.mCollisionTolerance;
			settings.mPenetrationTolerance = q.mPenetrationTolerance;
			settings.mExtraConvexRadius = q.mExtraConvexRadius;
			settings.mBackFaceModeTriangles = q.mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
			settings.mBackFaceModeConvex = q.mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
			settings.mUseShrunkenShapeAndConvexRadius = q.mUseShrunkenShape != 0;
			settings.mReturnDeepestPoint = q.mReturnDeepestPoint != 0;
			settings.mCollectFacesMode = q.mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
			settings.mActiveEdgeMode = q.mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
			settings.mActiveEdgeMovementDirection = Load3(q.mActiveEdgeMovementDirection);
			RShapeCast shape_cast(pw_catalogue_shape(inCatalogue, q.mShape), Load3(q.mScale), RMat44::sRotationTranslation(LoadQuat(q.mRotation), LoadR3(q.mOrigin)), Load3(q.mDirection));
			out.Box(shape_cast.mShapeWorldBounds);
			RunWithCollector<CastShapeCollector>(writer, q, [&](CastShapeCollector &ioCollector) { query.CastShape(shape_cast, settings, LoadR3(q.mBaseOffset), ioCollector, broad_phase_filter, object_filter, body_filter, shape_filter); });
			break;
		}

	case cNpqCollect:
	default:
		{
			AABox box(Load3(q.mBox), Load3(q.mBox + 3));
			RunWithCollector<TransformedShapeCollector>(writer, q, [&](TransformedShapeCollector &ioCollector) { query.CollectTransformedShapes(box, ioCollector, broad_phase_filter, object_filter, body_filter, shape_filter); });
			break;
		}
	}

	// The filter calls
	out.U(cNpqMarkerFilters);
	broad_phase_log.Write(out);
	object_log.Write(out);
	body_log.Write(out);
	shape_log.Write(out);
	return out.mSize;
}

// EstimateCollisionResponse for two bodies created from descriptions in a world without bodies
uint32 npq_estimate(void *inWorld, const NpqEstimateInput *inInput, uint32 *outData, uint32 inCapacity)
{
	NpqWorld &w = *static_cast<NpqWorld *>(inWorld);
	const NpqEstimateInput &in = *inInput;
	OutStream out(outData, inCapacity);
	BodyInterface &bi = w.mSystem.GetBodyInterface();

	// Create the bodies
	Body *bodies[2];
	for (int i = 0; i < 2; ++i)
	{
		const NpqEstimateBody &b = i == 0? in.mBody1 : in.mBody2;
		BodyCreationSettings settings(new BoxShape(Load3(b.mHalfExtent)), LoadR3(b.mPosition), LoadQuat(b.mRotation), EMotionType(b.mMotionType), Layers::MOVING);
		settings.mAllowedDOFs = EAllowedDOFs(b.mAllowedDOFs);
		settings.mOverrideMassProperties = EOverrideMassProperties(b.mOverrideMass);
		settings.mMassPropertiesOverride.mMass = b.mMass;
		settings.mMassPropertiesOverride.mInertia = Mat44::sRotation(LoadQuat(b.mInertiaRotation)) * Mat44::sScale(Load3(b.mInertia)) * Mat44::sRotation(LoadQuat(b.mInertiaRotation)).Transposed();
		settings.mInertiaMultiplier = b.mInertiaMultiplier;
		if (b.mMotionType != uint32(EMotionType::Static))
		{
			settings.mLinearVelocity = Load3(b.mLinearVelocity);
			settings.mAngularVelocity = Load3(b.mAngularVelocity);
		}
		bodies[i] = bi.CreateBody(settings);
	}

	// The manifold
	ContactManifold manifold;
	manifold.mBaseOffset = LoadR3(in.mBaseOffset);
	manifold.mWorldSpaceNormal = Load3(in.mNormal);
	manifold.mPenetrationDepth = in.mPenetrationDepth;
	for (uint32 i = 0; i < in.mNumPoints; ++i)
	{
		manifold.mRelativeContactPointsOn1.push_back(Load3(in.mPoints1[i]));
		manifold.mRelativeContactPointsOn2.push_back(Load3(in.mPoints2[i]));
	}

	CollisionEstimationResult result;
	EstimateCollisionResponse(*bodies[0], *bodies[1], manifold, result, in.mCombinedFriction, in.mCombinedRestitution, in.mMinVelocityForRestitution, in.mNumIterations);

	out.U(cNpqMarkerEstimate);
	out.V3(result.mLinearVelocity1);
	out.V3(result.mAngularVelocity1);
	out.V3(result.mLinearVelocity2);
	out.V3(result.mAngularVelocity2);
	out.V3(result.mFrictionPoint);
	out.V3(result.mFrictionImpulse);
	out.F(result.mAngularFrictionImpulse);
	out.U(uint32(result.mContactImpulse.size()));
	for (float impulse : result.mContactImpulse)
		out.F(impulse);

	// The inputs of the estimate that the bodies compute
	for (const Body *body : bodies)
	{
		out.R3(body->GetCenterOfMassPosition());
		if (body->GetMotionType() == EMotionType::Dynamic)
		{
			out.F(body->GetMotionProperties()->GetInverseMass());
			Mat44 inv_inertia = body->GetInverseInertia();
			for (int c = 0; c < 3; ++c)
				out.V3(inv_inertia.GetColumn3(c));
		}
	}

	for (Body *body : bodies)
		bi.DestroyBody(body->GetID());
	return out.mSize;
}

} // extern "C"
