// Reference implementation for the BroadPhaseQuadTree parity tests (`zig build parity`): C ABI wrappers around Jolt's
// BroadPhaseQuadTree on a BodyManager. ZoltParity/Physics/BroadPhaseQuadTreeParity.zig drives Zolt's broad phase and this
// one through the same operations and compares the trees (read from the private members), every query's hits and
// FindCollidingPairs bit for bit.
//
// A "world" is a BodyManager with a broad phase layer interface (object layer % number of broad phase layers), mask
// based layer filters and a BroadPhaseQuadTree. The structs below must match the extern structs in
// BroadPhaseQuadTreeParity.zig. Booleans are passed as uint32 (never bool, see the porting guide).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyPair.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Geometry/OrientedBox.h>

#include <cstring>
#include <vector>

using namespace JPH;

// Private access (explicit instantiation ignores access checks, `auto` avoids naming the private nested types)
template <typename Tag, auto M>
struct BQRob
{
	friend auto BQGet(Tag) { return M; }
};

#define BQ_ROB(Name, Member) \
	struct Name { friend auto BQGet(Name); }; \
	template struct BQRob<Name, &Member>;

BQ_ROB(BQLayers, BroadPhaseQuadTree::mLayers)
BQ_ROB(BQNumLayers, BroadPhaseQuadTree::mNumLayers)
BQ_ROB(BQTracking, BroadPhaseQuadTree::mTracking)
BQ_ROB(BQQueryLockIdx, BroadPhaseQuadTree::mQueryLockIdx)
BQ_ROB(BQNextLayerToUpdate, BroadPhaseQuadTree::mNextLayerToUpdate)
BQ_ROB(QTAllocator, QuadTree::mAllocator)
BQ_ROB(QTFreeNodeBatch, QuadTree::mFreeNodeBatch)
BQ_ROB(QTNumBodies, QuadTree::mNumBodies)
BQ_ROB(QTRootNode, QuadTree::mRootNode)
BQ_ROB(QTRootNodeIndex, QuadTree::mRootNodeIndex)
BQ_ROB(QTIsDirty, QuadTree::mIsDirty)

static constexpr uint32 cBQInvalidIndex = 0xffffffff;
static constexpr uint32 cBQNumObjectLayers = 8;

static void BQEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

// Must match BQWorldDesc in BroadPhaseQuadTreeParity.zig
struct BQWorldDesc
{
	uint32				mMaxBodies;
	uint32				mNumBroadPhaseLayers;
	uint32				mObjectVsBroadPhaseMask[cBQNumObjectLayers];	///< Bit b of entry l: object layer l collides with broad phase layer b
	uint32				mObjectPairMask[cBQNumObjectLayers];			///< Bit m of entry l: object layer l collides with object layer m
};

// Must match BQBodyDesc in BroadPhaseQuadTreeParity.zig
struct BQBodyDesc
{
	float				mPosition[3];
	float				mRotation[4];
	float				mSize[3];		///< Half extent of the box, the radius of the sphere in x
	uint32				mShape;			///< 0 box (convex radius 0), 1 sphere
	uint32				mMotionType;
	uint32				mObjectLayer;
	uint32				mPadding;
};

// Must match BQQuery in BroadPhaseQuadTreeParity.zig
struct BQQuery
{
	float				mF[16];			///< See BQRunQuery
	uint32				mKind;			///< 0 CastRay, 1 CollideAABox, 2 CollideSphere, 3 CollidePoint, 4 CollideOrientedBox, 5 CastAABox, 6 CastAABoxNoLock
	uint32				mCollectorMode;	///< 0 all hits, 1 closest hit (ray / cast: UpdateEarlyOutFraction), 2 ForceEarlyOut after mMaxHits hits
	uint32				mMaxHits;
	uint32				mBroadPhaseLayerMask;
	uint32				mObjectLayerMask;
	uint32				mPadding[3];
};

// Must match BQHit in BroadPhaseQuadTreeParity.zig
struct BQHit
{
	uint32				mBodyID;
	float				mFraction;
};

// Broad phase layer = object layer % number of layers
class BQLayerInterface final : public BroadPhaseLayerInterface
{
public:
	uint						mNumLayers = 1;
	virtual uint				GetNumBroadPhaseLayers() const override						{ return mNumLayers; }
	virtual BroadPhaseLayer		GetBroadPhaseLayer(ObjectLayer inLayer) const override		{ return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer % mNumLayers)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *		GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

class BQObjectVsBroadPhaseFilter final : public ObjectVsBroadPhaseLayerFilter
{
public:
	uint32						mMask[cBQNumObjectLayers];
	virtual bool				ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override { return ((mMask[inLayer1 % cBQNumObjectLayers] >> inLayer2.GetValue()) & 1) != 0; }
};

class BQObjectPairFilter final : public ObjectLayerPairFilter
{
public:
	uint32						mMask[cBQNumObjectLayers];
	virtual bool				ShouldCollide(ObjectLayer inLayer1, ObjectLayer inLayer2) const override { return ((mMask[inLayer1 % cBQNumObjectLayers] >> (inLayer2 % 32)) & 1) != 0; }
};

class BQBroadPhaseLayerFilter final : public BroadPhaseLayerFilter
{
public:
	explicit					BQBroadPhaseLayerFilter(uint32 inMask) : mMask(inMask) { }
	virtual bool				ShouldCollide(BroadPhaseLayer inLayer) const override { return ((mMask >> inLayer.GetValue()) & 1) != 0; }
	uint32						mMask;
};

class BQObjectLayerFilter final : public ObjectLayerFilter
{
public:
	explicit					BQObjectLayerFilter(uint32 inMask) : mMask(inMask) { }
	virtual bool				ShouldCollide(ObjectLayer inLayer) const override { return ((mMask >> (inLayer % 32)) & 1) != 0; }
	uint32						mMask;
};

struct BQWorld
{
	BQLayerInterface			mLayers;
	BQObjectVsBroadPhaseFilter	mObjectVsBroadPhaseFilter;
	BQObjectPairFilter			mObjectPairFilter;
	BodyManager					mBodyManager;
	BroadPhaseQuadTree			mBroadPhase;
	BroadPhase::UpdateState		mPendingState;
	uint32						mPending = 0;
};

// A collector that records every hit and implements the 3 collector modes of BQQuery
template <class Base>
class BQRecordingCollector final : public Base
{
public:
								BQRecordingCollector(uint32 inMode, uint32 inMaxHits) : mMode(inMode), mMaxHits(inMaxHits) { }

	virtual void				AddHit(const typename Base::ResultType &inResult) override
	{
		BQHit hit;
		if constexpr (std::is_same_v<typename Base::ResultType, BodyID>)
		{
			hit.mBodyID = inResult.GetIndexAndSequenceNumber();
			hit.mFraction = 0.0f;
		}
		else
		{
			hit.mBodyID = inResult.mBodyID.GetIndexAndSequenceNumber();
			hit.mFraction = inResult.mFraction;
			if (mMode == 1 && inResult.mFraction < Base::GetEarlyOutFraction())
				Base::UpdateEarlyOutFraction(inResult.mFraction);
		}
		mHits.push_back(hit);
		if (mMode == 2 && mHits.size() >= mMaxHits)
			Base::ForceEarlyOut();
	}

	uint32						mMode;
	uint32						mMaxHits;
	std::vector<BQHit>			mHits;
};

class BQPairCollector final : public BodyPairCollector
{
public:
	virtual void				AddHit(const BodyPair &inPair) override
	{
		mPairs.push_back(inPair.mBodyA.GetIndexAndSequenceNumber());
		mPairs.push_back(inPair.mBodyB.GetIndexAndSequenceNumber());
	}

	std::vector<uint32>			mPairs;
};

static Vec3 BQLoad3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }

// The nodes of the tree with root node inRootIndex in walk order: index, parent, changed, the 4 child IDs and the bounds
static void BQDumpTree(const QuadTree &inTree, uint32 inRootIndex, std::vector<uint32> &ioOut)
{
	const QuadTree::Allocator &allocator = *(inTree.*BQGet(QTAllocator{}));
	std::vector<uint32> stack;
	stack.push_back(inRootIndex);
	while (!stack.empty())
	{
		uint32 node_idx = stack.back();
		stack.pop_back();
		const auto &node = allocator.Get(node_idx);
		ioOut.push_back(node_idx);
		ioOut.push_back(node.mParentNodeIndex.load());
		ioOut.push_back(node.mIsChanged.load());
		const uint32 *child_ids = reinterpret_cast<const uint32 *>(&node.mChildNodeID[0]);
		for (int i = 0; i < 4; ++i)
		{
			uint32 child = child_ids[i];
			ioOut.push_back(child);
			const atomic<float> *bounds[] = { node.mBoundsMinX, node.mBoundsMinY, node.mBoundsMinZ, node.mBoundsMaxX, node.mBoundsMaxY, node.mBoundsMaxZ };
			for (const atomic<float> *b : bounds)
			{
				float f = b[i].load();
				uint32 bits;
				memcpy(&bits, &f, sizeof(bits));
				ioOut.push_back(bits);
			}
			if (child != cBQInvalidIndex && (child & BodyID::cBroadPhaseBit) != 0)
				stack.push_back(child & ~BodyID::cBroadPhaseBit);
		}
	}
}

static uint32 BQStoreVector(const std::vector<uint32> &inData, uint32 *outData, uint32 inCapacity)
{
	uint32 size = uint32(inData.size());
	if (size <= inCapacity)
		memcpy(outData, inData.data(), size * sizeof(uint32));
	return size;
}

extern "C"
{

void *jolt_bpqt_create(const BQWorldDesc *inDesc)
{
	BQEnsureFactory();
	BQWorld *world = new BQWorld;
	world->mLayers.mNumLayers = inDesc->mNumBroadPhaseLayers;
	memcpy(world->mObjectVsBroadPhaseFilter.mMask, inDesc->mObjectVsBroadPhaseMask, sizeof(world->mObjectVsBroadPhaseFilter.mMask));
	memcpy(world->mObjectPairFilter.mMask, inDesc->mObjectPairMask, sizeof(world->mObjectPairFilter.mMask));
	world->mBodyManager.Init(inDesc->mMaxBodies, 0, world->mLayers);
	world->mBroadPhase.Init(&world->mBodyManager, world->mLayers);
	return world;
}

void jolt_bpqt_destroy(void *inWorld)
{
	delete static_cast<BQWorld *>(inWorld);
}

// Returns the ID of the new body (not in the broad phase yet)
uint32 jolt_bpqt_create_body(void *inWorld, const BQBodyDesc *inDesc)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	RefConst<Shape> shape;
	if (inDesc->mShape == 0)
		shape = new BoxShape(BQLoad3(inDesc->mSize), 0.0f);
	else
		shape = new SphereShape(inDesc->mSize[0]);
	BodyCreationSettings settings(shape, RVec3(BQLoad3(inDesc->mPosition)), Quat(inDesc->mRotation[0], inDesc->mRotation[1], inDesc->mRotation[2], inDesc->mRotation[3]), EMotionType(inDesc->mMotionType), ObjectLayer(inDesc->mObjectLayer));
	Body *body = world->mBodyManager.AllocateBody(settings);
	if (!world->mBodyManager.AddBody(body))
	{
		world->mBodyManager.FreeBody(body);
		return BodyID::cInvalidBodyID;
	}
	return body->GetID().GetIndexAndSequenceNumber();
}

// AddBodiesPrepare (shuffles the IDs, the new order is written to outOrder), then AddBodiesFinalize (inAbort == 0) or
// AddBodiesAbort, then the dynamic bodies are activated (inActivate != 0)
void jolt_bpqt_add(void *inWorld, const uint32 *inIDs, uint32 inCount, uint32 inAbort, uint32 inActivate, uint32 *outOrder)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
		ids.push_back(BodyID(inIDs[i]));
	BroadPhase::AddState state = world->mBroadPhase.AddBodiesPrepare(ids.data(), int(inCount));
	for (uint32 i = 0; i < inCount; ++i)
		outOrder[i] = ids[i].GetIndexAndSequenceNumber();
	if (inAbort != 0)
	{
		world->mBroadPhase.AddBodiesAbort(ids.data(), int(inCount), state);
		return;
	}
	world->mBroadPhase.AddBodiesFinalize(ids.data(), int(inCount), state);
	if (inActivate != 0)
		for (uint32 i = 0; i < inCount; ++i)
			if (world->mBodyManager.GetBody(ids[i]).IsDynamic())
				world->mBodyManager.ActivateBodies(&ids[i], 1);
}

// Deactivates the bodies, then RemoveBodies (the new order is written to outOrder)
void jolt_bpqt_remove(void *inWorld, const uint32 *inIDs, uint32 inCount, uint32 *outOrder)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
		ids.push_back(BodyID(inIDs[i]));
	world->mBodyManager.DeactivateBodies(ids.data(), int(inCount));
	world->mBroadPhase.RemoveBodies(ids.data(), int(inCount));
	for (uint32 i = 0; i < inCount; ++i)
		outOrder[i] = ids[i].GetIndexAndSequenceNumber();
}

// Moves the bodies (3 positions and 4 rotation components per body), then NotifyBodiesAABBChanged (inTakeLock == 0: between
// LockModifications / UnlockModifications)
void jolt_bpqt_move(void *inWorld, const uint32 *inIDs, uint32 inCount, const float *inTransforms, uint32 inTakeLock, uint32 *outOrder)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
	{
		ids.push_back(BodyID(inIDs[i]));
		const float *t = inTransforms + 7 * i;
		world->mBodyManager.GetBody(ids[i]).SetPositionAndRotationInternal(RVec3(BQLoad3(t)), Quat(t[3], t[4], t[5], t[6]));
	}
	if (inTakeLock != 0)
		world->mBroadPhase.NotifyBodiesAABBChanged(ids.data(), int(inCount), true);
	else
	{
		world->mBroadPhase.LockModifications();
		world->mBroadPhase.NotifyBodiesAABBChanged(ids.data(), int(inCount), false);
		world->mBroadPhase.UnlockModifications();
	}
	for (uint32 i = 0; i < inCount; ++i)
		outOrder[i] = ids[i].GetIndexAndSequenceNumber();
}

// Sets the object layers of the bodies (BodyInterface::SetObjectLayer for bodies in the broad phase), then
// NotifyBodiesLayerChanged
void jolt_bpqt_set_layers(void *inWorld, const uint32 *inIDs, uint32 inCount, const uint32 *inLayers, uint32 *outOrder)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
	{
		ids.push_back(BodyID(inIDs[i]));
		world->mBodyManager.GetBody(ids[i]).SetObjectLayerInternal(ObjectLayer(inLayers[i]), world->mLayers);
	}
	world->mBroadPhase.NotifyBodiesLayerChanged(ids.data(), int(inCount));
	for (uint32 i = 0; i < inCount; ++i)
		outOrder[i] = ids[i].GetIndexAndSequenceNumber();
}

void jolt_bpqt_frame_sync(void *inWorld)
{
	static_cast<BQWorld *>(inWorld)->mBroadPhase.FrameSync();
}

void jolt_bpqt_optimize(void *inWorld)
{
	static_cast<BQWorld *>(inWorld)->mBroadPhase.Optimize();
}

// LockModifications and UpdatePrepare: writes the layer of the tree that is updated (or 0xffffffff) and the root ID of the
// new tree to outState, the state is kept until jolt_bpqt_update_finalize
void jolt_bpqt_update_prepare(void *inWorld, uint32 *outState)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	world->mBroadPhase.LockModifications();
	world->mPendingState = world->mBroadPhase.UpdatePrepare();
	world->mPending = 1;

	// UpdateStateImpl { QuadTree *mTree; QuadTree::UpdateState mUpdateState; }
	const QuadTree *tree;
	memcpy(&tree, &world->mPendingState, sizeof(tree));
	const QuadTree *layers = world->mBroadPhase.*BQGet(BQLayers{});
	outState[0] = tree == nullptr? cBQInvalidIndex : uint32(tree - layers);
	memcpy(&outState[1], reinterpret_cast<const uint8 *>(&world->mPendingState) + sizeof(QuadTree *), sizeof(uint32));
	if (tree == nullptr)
		outState[1] = cBQInvalidIndex;
}

// UpdateFinalize and UnlockModifications
void jolt_bpqt_update_finalize(void *inWorld)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	world->mBroadPhase.UpdateFinalize(world->mPendingState);
	world->mBroadPhase.UnlockModifications();
	world->mPending = 0;
}

// Runs a query, returns the number of hits (written up to inCapacity)
uint32 jolt_bpqt_query(void *inWorld, const BQQuery *inQuery, BQHit *outHits, uint32 inCapacity)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	const BroadPhaseQuery &query = world->mBroadPhase;
	const float *f = inQuery->mF;
	BQBroadPhaseLayerFilter bp_filter(inQuery->mBroadPhaseLayerMask);
	BQObjectLayerFilter ol_filter(inQuery->mObjectLayerMask);
	std::vector<BQHit> hits;
	switch (inQuery->mKind)
	{
	case 0:
		{
			BQRecordingCollector<RayCastBodyCollector> collector(inQuery->mCollectorMode, inQuery->mMaxHits);
			query.CastRay(RayCast(BQLoad3(f), BQLoad3(f + 3)), collector, bp_filter, ol_filter);
			hits = collector.mHits;
			break;
		}
	case 1:
	case 2:
	case 3:
	case 4:
		{
			BQRecordingCollector<CollideShapeBodyCollector> collector(inQuery->mCollectorMode, inQuery->mMaxHits);
			if (inQuery->mKind == 1)
				query.CollideAABox(AABox(BQLoad3(f), BQLoad3(f + 3)), collector, bp_filter, ol_filter);
			else if (inQuery->mKind == 2)
				query.CollideSphere(BQLoad3(f), f[3], collector, bp_filter, ol_filter);
			else if (inQuery->mKind == 3)
				query.CollidePoint(BQLoad3(f), collector, bp_filter, ol_filter);
			else
				query.CollideOrientedBox(OrientedBox(Mat44::sRotationTranslation(Quat(f[0], f[1], f[2], f[3]), BQLoad3(f + 4)), BQLoad3(f + 7)), collector, bp_filter, ol_filter);
			hits = collector.mHits;
			break;
		}
	default:
		{
			BQRecordingCollector<CastShapeBodyCollector> collector(inQuery->mCollectorMode, inQuery->mMaxHits);
			AABoxCast cast { AABox(BQLoad3(f), BQLoad3(f + 3)), BQLoad3(f + 6) };
			if (inQuery->mKind == 5)
				query.CastAABox(cast, collector, bp_filter, ol_filter);
			else
				world->mBroadPhase.CastAABoxNoLock(cast, collector, bp_filter, ol_filter);
			hits = collector.mHits;
			break;
		}
	}
	uint32 count = uint32(hits.size());
	if (count <= inCapacity)
		memcpy(outHits, hits.data(), count * sizeof(BQHit));
	return count;
}

// GetBounds: min and max
void jolt_bpqt_get_bounds(void *inWorld, float *outBounds)
{
	AABox bounds = static_cast<BQWorld *>(inWorld)->mBroadPhase.GetBounds();
	for (int i = 0; i < 3; ++i)
	{
		outBounds[i] = bounds.mMin[i];
		outBounds[3 + i] = bounds.mMax[i];
	}
}

// FindCollidingPairs for the active bodies inIDs (reordered, written to outOrder), returns the number of pairs (2 IDs per
// pair written up to inCapacity pairs)
uint32 jolt_bpqt_find_colliding_pairs(void *inWorld, const uint32 *inIDs, uint32 inCount, float inSpeculativeContactDistance, uint32 *outOrder, uint32 *outPairs, uint32 inCapacity)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
		ids.push_back(BodyID(inIDs[i]));
	BQPairCollector collector;
	world->mBroadPhase.FindCollidingPairs(ids.data(), int(inCount), inSpeculativeContactDistance, world->mObjectVsBroadPhaseFilter, world->mObjectPairFilter, collector);
	for (uint32 i = 0; i < inCount; ++i)
		outOrder[i] = ids[i].GetIndexAndSequenceNumber();
	uint32 count = uint32(collector.mPairs.size() / 2);
	if (count <= inCapacity)
		memcpy(outPairs, collector.mPairs.data(), collector.mPairs.size() * sizeof(uint32));
	return count;
}

// The state of the broad phase (see DumpTrees in BroadPhaseQuadTreeParity.zig), returns the number of uint32s (written up
// to inCapacity)
uint32 jolt_bpqt_dump(void *inWorld, uint32 *outData, uint32 inCapacity)
{
	BQWorld *world = static_cast<BQWorld *>(inWorld);
	BroadPhaseQuadTree &bp = world->mBroadPhase;
	const QuadTree *layers = bp.*BQGet(BQLayers{});
	uint num_layers = bp.*BQGet(BQNumLayers{});
	std::vector<uint32> out;
	out.push_back(bp.*BQGet(BQNextLayerToUpdate{}));
	out.push_back((bp.*BQGet(BQQueryLockIdx{})).load());
	out.push_back(num_layers);
	for (uint l = 0; l < num_layers; ++l)
	{
		const QuadTree &tree = layers[l];
		const auto &root_node = tree.*BQGet(QTRootNode{});
		const QuadTree::Allocator::Batch &batch = tree.*BQGet(QTFreeNodeBatch{});
		out.push_back((tree.*BQGet(QTNumBodies{})).load());
		out.push_back((tree.*BQGet(QTIsDirty{})).load()? 1 : 0);
		out.push_back((tree.*BQGet(QTRootNodeIndex{})).load());
		out.push_back(root_node[0].mIndex.load());
		out.push_back(root_node[1].mIndex.load());
		out.push_back(batch.mNumObjects);
		out.push_back(batch.mFirstObjectIndex);
		out.push_back(batch.mLastObjectIndex);
		for (int r = 0; r < 2; ++r)
		{
			uint32 root_idx = root_node[r].mIndex.load();
			if (root_idx != cBQInvalidIndex)
				BQDumpTree(tree, root_idx, out);
		}
	}

	// The new tree of a pending update
	if (world->mPending != 0)
	{
		const QuadTree *tree;
		memcpy(&tree, &world->mPendingState, sizeof(tree));
		if (tree != nullptr)
		{
			uint32 root_id;
			memcpy(&root_id, reinterpret_cast<const uint8 *>(&world->mPendingState) + sizeof(QuadTree *), sizeof(uint32));
			BQDumpTree(*tree, root_id & ~BodyID::cBroadPhaseBit, out);
		}
	}

	// The tracking
	const QuadTree::TrackingVector &tracking = bp.*BQGet(BQTracking{});
	for (const QuadTree::Tracking &t : tracking)
	{
		out.push_back(t.mBroadPhaseLayer.load());
		out.push_back(t.mObjectLayer.load());
		out.push_back(t.mBodyLocation.load());
	}
	return BQStoreVector(out, outData, inCapacity);
}

} // extern "C"
