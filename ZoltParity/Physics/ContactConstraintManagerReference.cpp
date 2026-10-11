// Reference implementation for the ContactConstraintManager parity tests (`zig build parity`): C ABI wrappers that drive
// Jolt's ContactConstraintManager like PhysicsSystem does in one collision step. ZoltParity/Physics/
// ContactConstraintManagerParity.zig drives Zolt's manager through the same calls and checks that both produce the same
// bits.
//
// A "world" owns a BodyManager (box bodies created like BodyInterface::CreateAndAddBody without a broad phase), an
// IslandBuilder, a PhysicsSystem that is only used for its gravity (it is never initialized, like the update context's
// mPhysicsSystem only needs GetGravity), the ContactConstraintManager with its own PhysicsSettings, a TempAllocatorImpl,
// the PhysicsUpdateContext of the current step, 4 ContactAllocators (like the FindCollisions jobs, used in a
// deterministic order) and a recording ContactListener that can override the contact settings with a deterministic
// function of the bodies and sub shapes. Every function that returns data appends uint32 words (floats and doubles as
// their bits) to an output record; the Zig side produces the same record. The private members of the manager are
// read through the explicit instantiation trick (access checks do not apply to explicit instantiation arguments, and
// `auto` keeps the private types unnamed): the constraint buffer (zeroed after PrepareConstraintBuffer on both sides, so
// members that Jolt leaves uninitialized have the same bits), the constraint index to offset table, the constraint
// counter and the write cache (the cached manifold of each constraint).
//
// The structs below must match the extern structs in ContactConstraintManagerParity.zig. Booleans are passed as uint32
// (never bool, see the porting guide).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsUpdateContext.h>
#include <Jolt/Physics/IslandBuilder.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>
#include <Jolt/Physics/Constraints/CalculateSolverSteps.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include <cstring>
#include <new>
#include <optional>
#include <string>

using namespace JPH;

namespace
{

// Must match CCMConfig in ContactConstraintManagerParity.zig
struct CCMConfig
{
	float				mGravity[3];
	float				mBaumgarte;
	float				mPenetrationSlop;
	float				mMaxPenetrationDistance;
	float				mMinVelocityForRestitution;
	float				mBodyPairCacheMaxDeltaPositionSq;
	float				mBodyPairCacheCosMaxDeltaRotationDiv2;
	float				mContactPointPreserveLambdaMaxDistSq;
	uint32				mNumVelocitySteps;
	uint32				mNumPositionSteps;
	uint32				mMaxBodies;
	uint32				mMaxBodyPairs;
	uint32				mMaxContactConstraints;
	uint32				mListener;			///< 1: a recording listener
	uint32				mModifierMode;		///< 1: the listener overrides the contact settings
	uint32				mCustomCombine;		///< 1: custom friction / restitution combine functions
};

// Must match CCMBodyDesc in ContactConstraintManagerParity.zig
struct CCMBodyDesc
{
	Real				mPosition[3];
	float				mRotation[4];
	float				mLinearVelocity[3];
	float				mAngularVelocity[3];
	float				mHalfExtent[3];
	float				mConvexRadius;
	float				mFriction;
	float				mRestitution;
	float				mGravityFactor;
	uint32				mMotionType;
	uint32				mIsSensor;
	uint32				mAllowedDOFs;
	uint32				mActivate;
	uint32				mNumVelocityStepsOverride;
	uint32				mNumPositionStepsOverride;
};

// Must match CCMManifold in ContactConstraintManagerParity.zig
struct CCMManifold
{
	Real				mBaseOffset[3];
	float				mNormal[3];
	float				mPenetrationDepth;
	uint32				mSubShapeID1;
	uint32				mSubShapeID2;
	uint32				mNumPoints;
	float				mOn1[4][3];
	float				mOn2[4][3];
};

// Appends uint32 words to an output record (the counter keeps counting past the capacity, the Zig side checks it)
class CCMRec
{
public:
						CCMRec(uint32 *outData, uint32 inCapacity, uint32 *outNum) : mData(outData), mCapacity(inCapacity), mNum(outNum) { *mNum = 0; }

	void				U(uint32 inV)				{ if (*mNum < mCapacity) mData[*mNum] = inV; ++*mNum; }
	void				F(float inV)				{ uint32 v; memcpy(&v, &inV, sizeof(v)); U(v); }
	void				R(Real inV)
	{
		uint32 v[sizeof(Real) / sizeof(uint32)];
		memcpy(v, &inV, sizeof(Real));
		for (uint32 w : v)
			U(w);
	}
	void				V3(Vec3Arg inV)				{ F(inV.GetX()); F(inV.GetY()); F(inV.GetZ()); }
	void				F3(const Float3 &inV)		{ F(inV.x); F(inV.y); F(inV.z); }
	void				R3(RVec3Arg inV)			{ R(inV.GetX()); R(inV.GetY()); R(inV.GetZ()); }
	void				Q(QuatArg inQ)				{ F(inQ.GetX()); F(inQ.GetY()); F(inQ.GetZ()); F(inQ.GetW()); }
	void				Bytes(const void *inData, uint32 inSize)
	{
		const uint8 *b = static_cast<const uint8 *>(inData);
		for (uint32 i = 0; i < inSize; i += 4)
		{
			uint32 v;
			memcpy(&v, b + i, 4);
			U(v);
		}
	}

private:
	uint32 *			mData;
	uint32				mCapacity;
	uint32 *			mNum;
};

// Private access to the members of the manager (explicit instantiation ignores access checks, `auto` keeps the types unnamed)
template <typename Tag, auto Member>
struct CCMRob
{
	friend auto CCMGet(Tag) { return Member; }
};

#define CCM_ROB(Name, ...) \
	struct Name { friend auto CCMGet(Name); }; \
	template struct CCMRob<Name, __VA_ARGS__>;

CCM_ROB(CCMConstraintsTag, &ContactConstraintManager::mConstraints)
CCM_ROB(CCMIdxToOffsetTag, &ContactConstraintManager::mConstraintIdxToOffset)
CCM_ROB(CCMCounterTag, &ContactConstraintManager::mNumConstraintsAndNextConstraintOffset)
CCM_ROB(CCMWriteCacheTag, &ContactConstraintManager::mWriteCache)

#undef CCM_ROB

// Broad phase layer = object layer % 2
class CCMLayers final : public BroadPhaseLayerInterface
{
public:
	virtual uint				GetNumBroadPhaseLayers() const override						{ return 2; }
	virtual BroadPhaseLayer		GetBroadPhaseLayer(ObjectLayer inLayer) const override		{ return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer % 2)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *		GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

static void CCMSettingsRecord(CCMRec &ioRec, const ContactSettings &inSettings)
{
	ioRec.F(inSettings.mCombinedFriction);
	ioRec.F(inSettings.mCombinedRestitution);
	ioRec.F(inSettings.mInvMassScale1);
	ioRec.F(inSettings.mInvInertiaScale1);
	ioRec.F(inSettings.mInvMassScale2);
	ioRec.F(inSettings.mInvInertiaScale2);
	ioRec.U(inSettings.mIsSensor? 1 : 0);
	ioRec.V3(inSettings.mRelativeLinearSurfaceVelocity);
	ioRec.V3(inSettings.mRelativeAngularSurfaceVelocity);
}

// The deterministic settings override of the listener (the same function is in ContactConstraintManagerParity.zig)
static void CCMModify(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings)
{
	uint32 k = (inBody1.GetID().GetIndex() * 7 + inBody2.GetID().GetIndex() * 13 + inManifold.mSubShapeID1.GetValue() * 3 + inManifold.mSubShapeID2.GetValue()) % 10;
	switch (k)
	{
	case 1:		ioSettings.mInvMassScale1 = 0.5f; ioSettings.mInvInertiaScale1 = 0.25f; break;
	case 2:		ioSettings.mInvMassScale2 = 2.0f; ioSettings.mInvInertiaScale2 = 0.0f; break;
	case 3:		ioSettings.mInvMassScale1 = 0.0f; ioSettings.mInvMassScale2 = 0.0f; break;
	case 4:		ioSettings.mCombinedFriction = 0.0f; break;
	case 5:		ioSettings.mCombinedRestitution = 0.9f; break;
	case 6:		ioSettings.mRelativeLinearSurfaceVelocity = Vec3(0.5f, 0.0f, -0.25f); ioSettings.mRelativeAngularSurfaceVelocity = Vec3(0.0f, 1.0f, 0.2f); break;
	case 7:		ioSettings.mCombinedFriction *= 2.0f; break;
	case 8:		ioSettings.mIsSensor = true; break;
	case 9:		ioSettings.mInvMassScale1 = 0.0f; break;
	default:	break;
	}
}

// Records the callbacks as uint32 words
class CCMListener : public ContactListener
{
public:
	virtual ValidateResult		OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
	{
		return ValidateResult((inBody1.GetID().GetIndex() + inBody2.GetID().GetIndex() + (inBaseOffset.GetX() > 0.0f? 1 : 0)) % 4);
	}

	virtual void				OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		Record(0, inBody1, inBody2, inManifold, ioSettings);
	}

	virtual void				OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		Record(1, inBody1, inBody2, inManifold, ioSettings);
	}

	virtual void				OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{
		mWords.push_back(2);
		mWords.push_back(inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber());
		mWords.push_back(inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber());
		mWords.push_back(inSubShapePair.GetSubShapeID1().GetValue());
		mWords.push_back(inSubShapePair.GetSubShapeID2().GetValue());
	}

	Array<uint32>				mWords;
	uint32						mModifierMode = 0;

private:
	void						Record(uint32 inKind, const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings)
	{
		uint32 words[1024];
		uint32 num;
		CCMRec rec(words, 1024, &num);
		rec.U(inKind);
		rec.U(inBody1.GetID().GetIndexAndSequenceNumber());
		rec.U(inBody2.GetID().GetIndexAndSequenceNumber());
		rec.U(inManifold.mSubShapeID1.GetValue());
		rec.U(inManifold.mSubShapeID2.GetValue());
		rec.R3(inManifold.mBaseOffset);
		rec.V3(inManifold.mWorldSpaceNormal);
		rec.F(inManifold.mPenetrationDepth);
		rec.U(uint32(inManifold.mRelativeContactPointsOn1.size()));
		for (Vec3 v : inManifold.mRelativeContactPointsOn1)
			rec.V3(v);
		for (Vec3 v : inManifold.mRelativeContactPointsOn2)
			rec.V3(v);
		CCMSettingsRecord(rec, ioSettings);
		for (uint32 i = 0; i < num; ++i)
			mWords.push_back(words[i]);

		if (mModifierMode != 0)
			CCMModify(inBody1, inBody2, inManifold, ioSettings);
	}
};

static float CCMCombineFriction(const Body &inBody1, const SubShapeID &, const Body &inBody2, const SubShapeID &inSubShapeID2)
{
	return inBody1.GetFriction() + inBody2.GetFriction() * (inSubShapeID2.GetValue() == 1? 0.5f : 1.0f);
}

static float CCMCombineRestitution(const Body &inBody1, const SubShapeID &, const Body &inBody2, const SubShapeID &)
{
	return inBody1.GetRestitution() * inBody2.GetRestitution();
}

// Filter mode 1: only the contacts of mBody, 2: all but the contacts of mBody
class CCMFilter final : public StateRecorderFilter
{
public:
	virtual bool				ShouldSaveContact(const BodyID &inBody1, const BodyID &inBody2) const override				{ return Accept(inBody1, inBody2); }
	virtual bool				ShouldRestoreContact(const BodyID &inBody1, const BodyID &inBody2) const override			{ return Accept(inBody1, inBody2); }

	bool						Accept(const BodyID &inBody1, const BodyID &inBody2) const
	{
		bool has_body = inBody1 == mBody || inBody2 == mBody;
		return mMode == 1? has_body : !has_body;
	}

	uint32						mMode = 0;
	BodyID						mBody;
};

using ContactAllocator = ContactConstraintManager::ContactAllocator;

struct CCMWorld
{
								CCMWorld(const CCMConfig &inConfig) :
		mManager(mSettings),
		mTempAllocator(64 * 1024 * 1024)
	{
		mSettings.mBaumgarte = inConfig.mBaumgarte;
		mSettings.mPenetrationSlop = inConfig.mPenetrationSlop;
		mSettings.mMaxPenetrationDistance = inConfig.mMaxPenetrationDistance;
		mSettings.mMinVelocityForRestitution = inConfig.mMinVelocityForRestitution;
		mSettings.mBodyPairCacheMaxDeltaPositionSq = inConfig.mBodyPairCacheMaxDeltaPositionSq;
		mSettings.mBodyPairCacheCosMaxDeltaRotationDiv2 = inConfig.mBodyPairCacheCosMaxDeltaRotationDiv2;
		mSettings.mContactPointPreserveLambdaMaxDistSq = inConfig.mContactPointPreserveLambdaMaxDistSq;
		mSettings.mNumVelocitySteps = inConfig.mNumVelocitySteps;
		mSettings.mNumPositionSteps = inConfig.mNumPositionSteps;

		mBodyManager.Init(inConfig.mMaxBodies, 0, mLayers);
		mIslandBuilder.Init(inConfig.mMaxBodies);
		mSystem.SetGravity(Vec3(inConfig.mGravity[0], inConfig.mGravity[1], inConfig.mGravity[2]));
		mManager.Init(inConfig.mMaxBodyPairs, inConfig.mMaxContactConstraints);
		if (inConfig.mListener != 0)
			mManager.SetContactListener(&mListener);
		mListener.mModifierMode = inConfig.mModifierMode;
		if (inConfig.mCustomCombine != 0)
		{
			mManager.SetCombineFriction(CCMCombineFriction);
			mManager.SetCombineRestitution(CCMCombineRestitution);
		}
	}

								~CCMWorld()
	{
		BodyIDVector ids;
		mBodyManager.GetBodyIDs(ids);
		for (const BodyID &id : ids)
		{
			mBodyManager.DeactivateBodies(&id, 1);
			mBodyManager.GetBody(id).SetInBroadPhaseInternal(false);
			mBodyManager.DestroyBodies(&id, 1);
		}
	}

	Body &						GetBody(uint32 inID)		{ return mBodyManager.GetBody(BodyID(inID)); }

	CCMLayers					mLayers;
	PhysicsSettings				mSettings;
	BodyManager					mBodyManager;
	IslandBuilder				mIslandBuilder;
	PhysicsSystem				mSystem;
	ContactConstraintManager	mManager;
	TempAllocatorImpl			mTempAllocator;
	std::optional<PhysicsUpdateContext> mContext;
	alignas(ContactAllocator) uint8 mAllocatorStorage[4][sizeof(ContactAllocator)];	///< ContactAllocator is not copyable, it is constructed in place from GetContactAllocator()
	ContactAllocator *			mAllocators[4] = { };
	CCMListener					mListener;
};

static void CCMLoadManifold(const CCMManifold &inDesc, ContactManifold &outManifold)
{
	outManifold.mBaseOffset = RVec3(inDesc.mBaseOffset[0], inDesc.mBaseOffset[1], inDesc.mBaseOffset[2]);
	outManifold.mWorldSpaceNormal = Vec3(inDesc.mNormal[0], inDesc.mNormal[1], inDesc.mNormal[2]);
	outManifold.mPenetrationDepth = inDesc.mPenetrationDepth;
	outManifold.mSubShapeID1.SetValue(inDesc.mSubShapeID1);
	outManifold.mSubShapeID2.SetValue(inDesc.mSubShapeID2);
	outManifold.mRelativeContactPointsOn1.resize(inDesc.mNumPoints);
	outManifold.mRelativeContactPointsOn2.resize(inDesc.mNumPoints);
	for (uint32 i = 0; i < inDesc.mNumPoints; ++i)
	{
		outManifold.mRelativeContactPointsOn1[i] = Vec3(inDesc.mOn1[i][0], inDesc.mOn1[i][1], inDesc.mOn1[i][2]);
		outManifold.mRelativeContactPointsOn2[i] = Vec3(inDesc.mOn2[i][0], inDesc.mOn2[i][1], inDesc.mOn2[i][2]);
	}
}

} // namespace

extern "C" {

void *jolt_ccm_world_create(const CCMConfig *inConfig)
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}

	return new CCMWorld(*inConfig);
}

void jolt_ccm_world_destroy(void *inWorld)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);

	// A failed comparison can leave a step open
	if (w->mContext.has_value())
	{
		w->mIslandBuilder.ResetIslands(&w->mTempAllocator);
		w->mManager.FinishConstraintBuffer();
		w->mContext.reset();
	}
	delete w;
}

uint32 jolt_ccm_add_body(void *inWorld, const CCMBodyDesc *inDesc)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	EMotionType motion_type = EMotionType(inDesc->mMotionType);
	RefConst<Shape> shape = new BoxShape(Vec3(inDesc->mHalfExtent[0], inDesc->mHalfExtent[1], inDesc->mHalfExtent[2]), inDesc->mConvexRadius);
	BodyCreationSettings settings(shape, RVec3(inDesc->mPosition[0], inDesc->mPosition[1], inDesc->mPosition[2]), Quat(inDesc->mRotation[0], inDesc->mRotation[1], inDesc->mRotation[2], inDesc->mRotation[3]), motion_type, ObjectLayer(motion_type == EMotionType::Static? 0 : 1));
	settings.mLinearVelocity = Vec3(inDesc->mLinearVelocity[0], inDesc->mLinearVelocity[1], inDesc->mLinearVelocity[2]);
	settings.mAngularVelocity = Vec3(inDesc->mAngularVelocity[0], inDesc->mAngularVelocity[1], inDesc->mAngularVelocity[2]);
	settings.mFriction = inDesc->mFriction;
	settings.mRestitution = inDesc->mRestitution;
	settings.mGravityFactor = inDesc->mGravityFactor;
	settings.mIsSensor = inDesc->mIsSensor != 0;
	settings.mAllowedDOFs = EAllowedDOFs(inDesc->mAllowedDOFs);
	settings.mNumVelocityStepsOverride = inDesc->mNumVelocityStepsOverride;
	settings.mNumPositionStepsOverride = inDesc->mNumPositionStepsOverride;
	Body *body = w->mBodyManager.AllocateBody(settings);
	bool added = w->mBodyManager.AddBody(body);
	JPH_ASSERT(added);
	(void)added;
	body->SetInBroadPhaseInternal(true);
	BodyID id = body->GetID();
	if (inDesc->mActivate != 0 && !body->IsStatic())
		w->mBodyManager.ActivateBodies(&id, 1);
	return id.GetIndexAndSequenceNumber();
}

void jolt_ccm_move_body(void *inWorld, uint32 inID, const Real *inPosition, const float *inRotation)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	w->GetBody(inID).SetPositionAndRotationInternal(RVec3(inPosition[0], inPosition[1], inPosition[2]), Quat(inRotation[0], inRotation[1], inRotation[2], inRotation[3]));
}

void jolt_ccm_set_velocity(void *inWorld, uint32 inID, const float *inLinear, const float *inAngular, const float *inForce)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	Body &body = w->GetBody(inID);
	body.SetLinearVelocity(Vec3(inLinear[0], inLinear[1], inLinear[2]));
	body.SetAngularVelocity(Vec3(inAngular[0], inAngular[1], inAngular[2]));
	if (body.IsDynamic())
		body.AddForce(Vec3(inForce[0], inForce[1], inForce[2]));
}

void jolt_ccm_begin_step(void *inWorld, float inDeltaTime, float inWarmStartImpulseRatio)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	w->mContext.emplace(w->mTempAllocator);
	PhysicsUpdateContext &context = *w->mContext;
	context.mPhysicsSystem = &w->mSystem;
	context.mJobSystem = nullptr;
	context.mBarrier = nullptr;
	context.mStepDeltaTime = inDeltaTime;
	context.mWarmStartImpulseRatio = inWarmStartImpulseRatio;
	context.mBodyManager = &w->mBodyManager;
	context.mIslandBuilder = &w->mIslandBuilder;

	w->mManager.PrepareConstraintBuffer(&context);
	memset(w->mManager.*CCMGet(CCMConstraintsTag{}), 0, size_t(w->mManager.GetMaxConstraints()) * ContactConstraintManager::cMaxConstraintSize);
	w->mIslandBuilder.PrepareContactConstraints(w->mManager.GetMaxConstraints(), &w->mTempAllocator);
	for (int i = 0; i < 4; ++i)
		w->mAllocators[i] = new (w->mAllocatorStorage[i]) ContactAllocator(w->mManager.GetContactAllocator());
}

// Returns: 1 when the cache handled the pair, 2 when AddBodyPair failed, 4 when the bodies still need to be linked
uint32 jolt_ccm_process_pair(void *inWorld, uint32 inID1, uint32 inID2, const CCMManifold *inManifolds, uint32 inNumManifolds, uint32 inUseCache, uint32 inAllocator)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	ContactAllocator &allocator = *w->mAllocators[inAllocator];
	Body &body1 = w->GetBody(inID1);
	Body &body2 = w->GetBody(inID2);
	if (inUseCache != 0)
	{
		bool handled;
		w->mManager.GetContactsFromCache(allocator, body1, body2, handled);
		if (handled)
			return 1;
	}
	ContactConstraintManager::BodyPairHandle handle = w->mManager.AddBodyPair(allocator, body1, body2);
	if (handle == nullptr)
		return 2;
	bool link_and_activate = true;
	for (uint32 i = 0; i < inNumManifolds; ++i)
	{
		ContactManifold manifold;
		CCMLoadManifold(inManifolds[i], manifold);
		w->mManager.AddContactConstraint(allocator, link_and_activate, handle, body1, body2, manifold);
	}
	return link_and_activate? 4 : 0;
}

void jolt_ccm_ccd_contact(void *inWorld, uint32 inID1, uint32 inID2, const CCMManifold *inManifold, uint32 inAllocator, uint32 *outData, uint32 inCapacity, uint32 *outNum)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	CCMRec rec(outData, inCapacity, outNum);
	ContactManifold manifold;
	CCMLoadManifold(*inManifold, manifold);
	ContactSettings settings;
	w->mManager.OnCCDContactAdded(*w->mAllocators[inAllocator], w->GetBody(inID1), w->GetBody(inID2), manifold, settings);
	CCMSettingsRecord(rec, settings);
}

uint32 jolt_ccm_validate(void *inWorld, uint32 inID1, uint32 inID2, const Real *inBaseOffset)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	CollideShapeResult result;
	return uint32(w->mManager.ValidateContactPoint(w->GetBody(inID1), w->GetBody(inID2), RVec3(inBaseOffset[0], inBaseOffset[1], inBaseOffset[2]), result));
}

// The constraints of the step (the raw bytes with the body pointers replaced by the body IDs) and the cached manifold of
// each constraint in the write cache (the impulses only after StoreAppliedImpulses)
void jolt_ccm_dump_constraints(void *inWorld, uint32 inWithImpulses, uint32 *outData, uint32 inCapacity, uint32 *outNum)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	CCMRec rec(outData, inCapacity, outNum);
	ContactConstraintManager &m = w->mManager;
	const uint8 *constraints = m.*CCMGet(CCMConstraintsTag{});
	const uint32 *idx_to_offset = m.*CCMGet(CCMIdxToOffsetTag{});
	uint64 counter = (m.*CCMGet(CCMCounterTag{})).load(memory_order_relaxed);
	auto write_cache = m.*CCMGet(CCMWriteCacheTag{});
	rec.U(uint32(counter));
	rec.U(uint32(counter >> 32));
	uint32 num_constraints = m.GetNumConstraints();
	rec.U(num_constraints);
	uint32 buffer_size = m.GetMaxConstraints() * ContactConstraintManager::cMaxConstraintSize;
	for (uint32 i = 0; i < num_constraints; ++i)
	{
		uint32 offset = idx_to_offset[i];
		uint32 end = i + 1 < num_constraints? idx_to_offset[i + 1] : min(offset + ContactConstraintManager::cMaxConstraintSize, buffer_size);
		rec.U(offset);
		rec.U(end - offset);
		const uint8 *c = constraints + offset;
		const Body *body1, *body2;
		memcpy(&body1, c, sizeof(body1));
		memcpy(&body2, c + 8, sizeof(body2));
		rec.U(body1->GetID().GetIndexAndSequenceNumber());
		rec.U(0);
		rec.U(body2->GetID().GetIndexAndSequenceNumber());
		rec.U(0);
		rec.Bytes(c + 16, end - offset - 16);

		// The cached manifold
		uint32 handle, num_points;
		memcpy(&handle, c + 80, sizeof(handle));
		memcpy(&num_points, c + 84, sizeof(num_points));
		auto kv = write_cache->FromHandle(handle);
		rec.Bytes(&kv->GetKey(), sizeof(SubShapeIDPair));
		const auto &cm = kv->GetValue();
		rec.U(cm.mNextWithSameBodyPair);
		rec.F3(cm.mContactNormal);
		rec.U(cm.mFlags.load());
		rec.U(cm.mNumContactPoints);
		for (uint32 k = 0; k < num_points; ++k)
		{
			rec.F3(cm.mContactPoints[k].mPosition1);
			rec.F3(cm.mContactPoints[k].mPosition2);
			if (inWithImpulses != 0)
				rec.F(cm.mContactPoints[k].mNonPenetrationLambda);
		}
		if (inWithImpulses != 0)
		{
			rec.F(cm.mFrictionLambda[0]);
			rec.F(cm.mFrictionLambda[1]);
			rec.F(cm.mAngularFrictionLambda);
		}
	}
}

// FinalizeIslands and the contact part of the solver jobs for every island
void jolt_ccm_solve(void *inWorld, uint32 inVelocitySteps, uint32 inPositionSteps, uint32 inUseDummy, uint32 *outData, uint32 inCapacity, uint32 *outNum)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	CCMRec rec(outData, inCapacity, outNum);
	ContactConstraintManager &m = w->mManager;
	IslandBuilder &ib = w->mIslandBuilder;
	BodyManager &bm = w->mBodyManager;
	ib.Finalize(bm.GetActiveBodiesUnsafe(EBodyType::RigidBody), bm.GetNumActiveBodies(EBodyType::RigidBody), m.GetNumConstraints(), &w->mTempAllocator);
	rec.U(ib.GetNumIslands());
	for (uint32 island = 0; island < ib.GetNumIslands(); ++island)
	{
		BodyID *bodies_begin, *bodies_end;
		ib.GetBodiesInIsland(island, bodies_begin, bodies_end);
		rec.U(uint32(bodies_end - bodies_begin));
		for (BodyID *b = bodies_begin; b < bodies_end; ++b)
			rec.U(b->GetIndexAndSequenceNumber());

		uint32 *contacts_begin, *contacts_end;
		if (!ib.GetContactsInIsland(island, contacts_begin, contacts_end))
		{
			rec.U(0);
			continue;
		}
		rec.U(uint32(contacts_end - contacts_begin));
		m.ConstraintIdxToConstraintOffset(contacts_begin, contacts_end);
		for (uint32 *c = contacts_begin; c < contacts_end; ++c)
			rec.U(*c);
		m.SortContacts(contacts_begin, contacts_end);
		for (uint32 *c = contacts_begin; c < contacts_end; ++c)
			rec.U(*c);
		for (uint32 *c = contacts_begin; c < contacts_end; ++c)
		{
			const Body *body1, *body2;
			m.GetAffectedBodies(*c, body1, body2);
			rec.U(body1->GetID().GetIndexAndSequenceNumber());
			rec.U(body2->GetID().GetIndexAndSequenceNumber());
		}
		if (inUseDummy != 0)
		{
			DummyCalculateSolverSteps dummy;
			m.WarmStartVelocityConstraints(contacts_begin, contacts_end, w->mContext->mWarmStartImpulseRatio, dummy);
		}
		else
		{
			CalculateSolverSteps steps_calculator(w->mSettings);
			m.WarmStartVelocityConstraints(contacts_begin, contacts_end, w->mContext->mWarmStartImpulseRatio, steps_calculator);
			steps_calculator.Finalize();
			rec.U(steps_calculator.GetNumVelocitySteps());
			rec.U(steps_calculator.GetNumPositionSteps());
		}
		for (uint32 i = 0; i < inVelocitySteps; ++i)
			rec.U(m.SolveVelocityConstraints(contacts_begin, contacts_end)? 1 : 0);
		m.StoreAppliedImpulses(contacts_begin, contacts_end);
		for (uint32 i = 0; i < inPositionSteps; ++i)
			rec.U(m.SolvePositionConstraints(contacts_begin, contacts_end)? 1 : 0);
	}
}

// The ContactRemovedCallbacks job and the frees of finishUpdate
void jolt_ccm_end_step(void *inWorld)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	uint32 num_body_pairs = 0, num_manifolds = 0;
	for (ContactAllocator *a : w->mAllocators)
	{
		num_body_pairs += a->mNumBodyPairs;
		num_manifolds += a->mNumManifolds;
	}
	w->mManager.FinalizeContactCacheAndCallContactPointRemovedCallbacks(num_body_pairs, num_manifolds);
	w->mIslandBuilder.ResetIslands(&w->mTempAllocator);
	w->mManager.FinishConstraintBuffer();
	w->mContext.reset();
}

void jolt_ccm_allocator_info(void *inWorld, uint32 inAllocator, uint32 *outInfo)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	const ContactAllocator &a = *w->mAllocators[inAllocator];
	outInfo[0] = a.mNumBodyPairs;
	outInfo[1] = a.mNumManifolds;
	outInfo[2] = uint32(a.mErrors);
	outInfo[3] = w->mManager.GetNumConstraints();
}

void jolt_ccm_dump_bodies(void *inWorld, uint32 *outData, uint32 inCapacity, uint32 *outNum)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	CCMRec rec(outData, inCapacity, outNum);
	BodyManager &bm = w->mBodyManager;
	for (Body *body : bm.GetBodies())
		if (BodyManager::sIsValidBodyPointer(body))
		{
			rec.U(body->GetID().GetIndexAndSequenceNumber());
			rec.R3(body->GetCenterOfMassPosition());
			rec.Q(body->GetRotation());
			rec.V3(body->GetLinearVelocity());
			rec.V3(body->GetAngularVelocity());
			rec.U(body->IsActive()? 1 : 0);
		}
	uint32 num_active = bm.GetNumActiveBodies(EBodyType::RigidBody);
	rec.U(num_active);
	for (uint32 i = 0; i < num_active; ++i)
		rec.U(bm.GetActiveBodiesUnsafe(EBodyType::RigidBody)[i].GetIndexAndSequenceNumber());
}

void jolt_ccm_records(void *inWorld, uint32 *outData, uint32 inCapacity, uint32 *outNum)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	CCMRec rec(outData, inCapacity, outNum);
	for (uint32 v : w->mListener.mWords)
		rec.U(v);
	w->mListener.mWords.clear();
}

uint32 jolt_ccm_were_in_contact(void *inWorld, uint32 inID1, uint32 inID2)
{
	return static_cast<CCMWorld *>(inWorld)->mManager.WereBodiesInContact(BodyID(inID1), BodyID(inID2))? 1 : 0;
}

uint32 jolt_ccm_save(void *inWorld, uint32 inFilterMode, uint32 inFilterBody, uint8 *outData, uint32 inCapacity)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	StateRecorderImpl recorder;
	CCMFilter filter;
	filter.mMode = inFilterMode;
	filter.mBody = BodyID(inFilterBody);
	w->mManager.SaveState(recorder, inFilterMode != 0? &filter : nullptr);
	std::string data = recorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outData, data.data(), size);
	return size;
}

// Returns RestoreState's result in bit 0 and the stream's IsFailed in bit 1
uint32 jolt_ccm_restore(void *inWorld, const uint8 *inData, uint32 inSize, uint32 inValidating, uint32 inFilterMode, uint32 inFilterBody, uint32 inLastPart)
{
	CCMWorld *w = static_cast<CCMWorld *>(inWorld);
	StateRecorderImpl recorder;
	recorder.WriteBytes(inData, inSize);
	recorder.SetValidating(inValidating != 0);
	recorder.SetIsLastPart(inLastPart != 0);
	CCMFilter filter;
	filter.mMode = inFilterMode;
	filter.mBody = BodyID(inFilterBody);
	bool success = w->mManager.RestoreState(recorder, inFilterMode != 0? &filter : nullptr);
	return (success? 1 : 0) | (recorder.IsFailed()? 2 : 0);
}

void jolt_ccm_sizes(uint32 *outSizes)
{
	outSizes[0] = ContactConstraintManager::cMaxConstraintSize;
	outSizes[1] = ContactConstraintManager::cMaxContactConstraintsLimit;
	outSizes[2] = ContactConstraintManager::cMaxBodyPairsLimit;
	outSizes[3] = ContactConstraintManager::MaxContactPoints;
}

} // extern "C"
