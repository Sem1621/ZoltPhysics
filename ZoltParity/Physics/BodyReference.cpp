// Reference implementation for the Body package parity tests (`zig build parity`): C ABI wrappers around Jolt's
// BodyManager, Body, MotionProperties and BodyCreationSettings. ZoltParity/Physics/BodyParity.zig calls these and checks
// that Zolt produces the same bits.
//
// A "world" is a BodyManager with a broad phase layer interface (object layer % 2). Bodies are created like
// BodyInterface::CreateAndAddBody does it without a broad phase (Phase 5 F1 has none): AllocateBody, AddBody (or
// AddBodyWithCustomID), SetInBroadPhaseInternal(true) and optionally ActivateBodies. The structs below must match the
// extern structs in BodyParity.zig. Booleans are passed as uint32 (never bool, see the porting guide).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MotionProperties.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include <cstring>
#include <string>

using namespace JPH;

static void BodyEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static Vec3 BLoad3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 BLoad4(const float *inV)				{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Quat BLoadQuat(const float *inV)				{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 BLoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static void BStore3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void BStore4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void BStoreQuat(QuatArg inQ, float *outV)	{ outV[0] = inQ.GetX(); outV[1] = inQ.GetY(); outV[2] = inQ.GetZ(); outV[3] = inQ.GetW(); }
static void BStoreR3(RVec3Arg inV, Real *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Mat44 BLoadMat44(const float *inM)			{ return Mat44(BLoad4(inM), BLoad4(inM + 4), BLoad4(inM + 8), BLoad4(inM + 12)); }
static void BStoreMat44(Mat44Arg inM, float *outM)	{ for (int c = 0; c < 4; ++c) BStore4(inM.GetColumn4(c), outM + 4 * c); }
static void BStoreRMat44(RMat44Arg inM, float *outCols, Real *outT) { for (int c = 0; c < 3; ++c) BStore4(inM.GetColumn4(c), outCols + 4 * c); BStoreR3(inM.GetTranslation(), outT); }

// Must match BodyShapeDesc in BodyParity.zig
struct BodyShapeDesc
{
	uint32				mKind;				///< 0 sphere, 1 box, 2 capsule, 3 cylinder, 4 offset COM box, 5 rotated translated box, 6 scaled box, 7 static compound of 2 boxes
	uint32				mUseSettings;		///< 1: BodyCreationSettings(const ShapeSettings *), 0: BodyCreationSettings(const Shape *)
	float				mParams[8];			///< 0..2 half extent / radius / half height, 3 convex radius, 4..6 offset / translation / scale
	float				mRotation[4];
};

// Must match BodyDesc in BodyParity.zig
struct BodyDesc
{
	uint64				mUserData;
	Real				mPosition[3];
	float				mRotation[4];
	float				mLinearVelocity[3];
	float				mAngularVelocity[3];
	float				mFriction;
	float				mRestitution;
	float				mLinearDamping;
	float				mAngularDamping;
	float				mMaxLinearVelocity;
	float				mMaxAngularVelocity;
	float				mGravityFactor;
	float				mInertiaMultiplier;
	float				mMass;
	float				mInertia[16];
	uint32				mObjectLayer;
	uint32				mMotionType;
	uint32				mAllowedDOFs;
	uint32				mAllowDynamicOrKinematic;
	uint32				mIsSensor;
	uint32				mCollideKinematicVsNonDynamic;
	uint32				mUseManifoldReduction;
	uint32				mApplyGyroscopicForce;
	uint32				mMotionQuality;
	uint32				mEnhancedInternalEdgeRemoval;
	uint32				mAllowSleeping;
	uint32				mNumVelocityStepsOverride;
	uint32				mNumPositionStepsOverride;
	uint32				mOverrideMassProperties;
	BodyShapeDesc		mShape;
};

// Must match BodyOp in BodyParity.zig
struct BodyOp
{
	Real				mR[6];
	float				mF[24];
	uint32				mKind;
	uint32				mI[7];
	BodyShapeDesc		mShape;
};

// Must match BodyOpResult in BodyParity.zig
struct BodyOpResult
{
	Real				mR[6];
	float				mF[48];
	uint32				mI[16];
};

// Must match BodyOut in BodyParity.zig
struct BodyOut
{
	uint64				mUserData;
	Real				mPosition[3];
	Real				mCOMPosition[3];
	Real				mWorldT[3];
	Real				mCOMT[3];
	Real				mInvCOMT[3];
	float				mRotation[4];
	float				mWorldCols[12];
	float				mCOMCols[12];
	float				mInvCOMCols[12];
	float				mBounds[6];
	float				mLinearVelocity[3];
	float				mAngularVelocity[3];
	float				mForce[3];
	float				mTorque[3];
	float				mInvMass;
	float				mInvInertiaDiagonal[3];
	float				mInertiaRotation[4];
	float				mLocalInvInertia[16];
	float				mInvInertia[16];
	float				mMaxLinearVelocity;
	float				mMaxAngularVelocity;
	float				mLinearDamping;
	float				mAngularDamping;
	float				mGravityFactor;
	float				mFriction;
	float				mRestitution;
	uint32				mID;
	uint32				mBodyType;
	uint32				mMotionType;
	uint32				mFlags;
	uint32				mIndexInActiveBodies;
	uint32				mIslandIndex;
	uint32				mMotionQuality;
	uint32				mAllowedDOFs;
	uint32				mAllowSleeping;
	uint32				mNumVelocityStepsOverride;
	uint32				mNumPositionStepsOverride;
	uint32				mObjectLayer;
	uint32				mBroadPhaseLayer;
	uint32				mHasMotionProperties;
	uint32				mIsActive;
	uint32				mLinearDOFsMask[4];
	uint32				mAngularDOFsMask[4];
	uint32				mStateSize;
	uint8				mState[512];
};

static constexpr float cSentinel = -12345.0f;

// Broad phase layer = object layer % 2
class ParityBPLayerInterface final : public BroadPhaseLayerInterface
{
public:
	virtual uint				GetNumBroadPhaseLayers() const override						{ return 2; }
	virtual BroadPhaseLayer		GetBroadPhaseLayer(ObjectLayer inLayer) const override		{ return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer % 2)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *		GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

struct BodyWorld
{
	ParityBPLayerInterface		mLayers;
	BodyManager					mBodyManager;
};

static RefConst<ShapeSettings> MakeBoxSettings(const BodyShapeDesc &inDesc)
{
	return new BoxShapeSettings(BLoad3(inDesc.mParams), inDesc.mParams[3]);
}

static RefConst<ShapeSettings> MakeShapeSettings(const BodyShapeDesc &inDesc)
{
	const float *p = inDesc.mParams;
	switch (inDesc.mKind)
	{
	case 0:		return new SphereShapeSettings(p[0]);
	case 1:		return MakeBoxSettings(inDesc);
	case 2:		return new CapsuleShapeSettings(p[0], p[1]);
	case 3:		return new CylinderShapeSettings(p[0], p[1], p[3]);
	case 4:		return new OffsetCenterOfMassShapeSettings(BLoad3(p + 4), MakeBoxSettings(inDesc));
	case 5:		return new RotatedTranslatedShapeSettings(BLoad3(p + 4), BLoadQuat(inDesc.mRotation), MakeBoxSettings(inDesc));
	case 6:		return new ScaledShapeSettings(MakeBoxSettings(inDesc), BLoad3(p + 4));
	default:
		{
			StaticCompoundShapeSettings *compound = new StaticCompoundShapeSettings;
			compound->AddShape(BLoad3(p + 4), BLoadQuat(inDesc.mRotation), MakeBoxSettings(inDesc));
			compound->AddShape(-BLoad3(p + 4), Quat::sIdentity(), new SphereShapeSettings(p[0]));
			return compound;
		}
	}
}

static RefConst<Shape> MakeShape(const BodyShapeDesc &inDesc)
{
	Shape::ShapeResult result = MakeShapeSettings(inDesc)->Create();
	return result.Get();
}

static void MakeBodyCreationSettings(const BodyDesc &inDesc, BodyCreationSettings &outSettings)
{
	RVec3 position = BLoadR3(inDesc.mPosition);
	Quat rotation = BLoadQuat(inDesc.mRotation);
	EMotionType motion_type = EMotionType(inDesc.mMotionType);
	ObjectLayer layer = ObjectLayer(inDesc.mObjectLayer);
	if (inDesc.mShape.mUseSettings != 0)
		outSettings = BodyCreationSettings(MakeShapeSettings(inDesc.mShape), position, rotation, motion_type, layer);
	else
		outSettings = BodyCreationSettings(MakeShape(inDesc.mShape), position, rotation, motion_type, layer);
	outSettings.mLinearVelocity = BLoad3(inDesc.mLinearVelocity);
	outSettings.mAngularVelocity = BLoad3(inDesc.mAngularVelocity);
	outSettings.mUserData = inDesc.mUserData;
	outSettings.mAllowedDOFs = EAllowedDOFs(inDesc.mAllowedDOFs);
	outSettings.mAllowDynamicOrKinematic = inDesc.mAllowDynamicOrKinematic != 0;
	outSettings.mIsSensor = inDesc.mIsSensor != 0;
	outSettings.mCollideKinematicVsNonDynamic = inDesc.mCollideKinematicVsNonDynamic != 0;
	outSettings.mUseManifoldReduction = inDesc.mUseManifoldReduction != 0;
	outSettings.mApplyGyroscopicForce = inDesc.mApplyGyroscopicForce != 0;
	outSettings.mMotionQuality = EMotionQuality(inDesc.mMotionQuality);
	outSettings.mEnhancedInternalEdgeRemoval = inDesc.mEnhancedInternalEdgeRemoval != 0;
	outSettings.mAllowSleeping = inDesc.mAllowSleeping != 0;
	outSettings.mFriction = inDesc.mFriction;
	outSettings.mRestitution = inDesc.mRestitution;
	outSettings.mLinearDamping = inDesc.mLinearDamping;
	outSettings.mAngularDamping = inDesc.mAngularDamping;
	outSettings.mMaxLinearVelocity = inDesc.mMaxLinearVelocity;
	outSettings.mMaxAngularVelocity = inDesc.mMaxAngularVelocity;
	outSettings.mGravityFactor = inDesc.mGravityFactor;
	outSettings.mNumVelocityStepsOverride = inDesc.mNumVelocityStepsOverride;
	outSettings.mNumPositionStepsOverride = inDesc.mNumPositionStepsOverride;
	outSettings.mOverrideMassProperties = EOverrideMassProperties(inDesc.mOverrideMassProperties);
	outSettings.mInertiaMultiplier = inDesc.mInertiaMultiplier;
	outSettings.mMassPropertiesOverride.mMass = inDesc.mMass;
	outSettings.mMassPropertiesOverride.mInertia = BLoadMat44(inDesc.mInertia);
}

static uint32 SaveBytes(StateRecorderImpl &inRecorder, uint8 *outBytes, uint32 inCapacity)
{
	std::string data = inRecorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

static void FillSentinel(float *outV, int inCount)
{
	for (int i = 0; i < inCount; ++i)
		outV[i] = cSentinel;
}

static void GetBodyOut(const Body &inBody, BodyOut *outBody)
{
	memset(outBody, 0, sizeof(BodyOut));
	outBody->mUserData = inBody.GetUserData();
	BStoreR3(inBody.GetPosition(), outBody->mPosition);
	BStoreR3(inBody.GetCenterOfMassPosition(), outBody->mCOMPosition);
	BStoreRMat44(inBody.GetWorldTransform(), outBody->mWorldCols, outBody->mWorldT);
	BStoreRMat44(inBody.GetCenterOfMassTransform(), outBody->mCOMCols, outBody->mCOMT);
	BStoreRMat44(inBody.GetInverseCenterOfMassTransform(), outBody->mInvCOMCols, outBody->mInvCOMT);
	BStoreQuat(inBody.GetRotation(), outBody->mRotation);
	BStore3(inBody.GetWorldSpaceBounds().mMin, outBody->mBounds);
	BStore3(inBody.GetWorldSpaceBounds().mMax, outBody->mBounds + 3);
	BStore3(inBody.GetLinearVelocity(), outBody->mLinearVelocity);
	BStore3(inBody.GetAngularVelocity(), outBody->mAngularVelocity);
	const MotionProperties *mp = inBody.GetMotionPropertiesUnchecked();
	if (mp != nullptr)
	{
		BStore3(mp->GetAccumulatedForce(), outBody->mForce);
		BStore3(mp->GetAccumulatedTorque(), outBody->mTorque);
		outBody->mInvMass = mp->GetInverseMassUnchecked();
		BStore3(mp->GetInverseInertiaDiagonal(), outBody->mInvInertiaDiagonal);
		BStoreQuat(mp->GetInertiaRotation(), outBody->mInertiaRotation);
		BStoreMat44(mp->GetLocalSpaceInverseInertiaUnchecked(), outBody->mLocalInvInertia);
		if (inBody.IsDynamic())
			BStoreMat44(inBody.GetInverseInertia(), outBody->mInvInertia);
		else
			FillSentinel(outBody->mInvInertia, 16);
		outBody->mMaxLinearVelocity = mp->GetMaxLinearVelocity();
		outBody->mMaxAngularVelocity = mp->GetMaxAngularVelocity();
		outBody->mLinearDamping = mp->GetLinearDamping();
		outBody->mAngularDamping = mp->GetAngularDamping();
		outBody->mGravityFactor = mp->GetGravityFactor();
		outBody->mIslandIndex = mp->GetIslandIndexInternal();
		outBody->mMotionQuality = uint32(mp->GetMotionQuality());
		outBody->mAllowedDOFs = uint32(mp->GetAllowedDOFs());
		outBody->mAllowSleeping = mp->GetAllowSleeping()? 1 : 0;
		outBody->mNumVelocityStepsOverride = mp->GetNumVelocityStepsOverride();
		outBody->mNumPositionStepsOverride = mp->GetNumPositionStepsOverride();
		outBody->mHasMotionProperties = 1;
		UVec4 linear = mp->GetLinearDOFsMask();
		UVec4 angular = mp->GetAngularDOFsMask();
		linear.StoreInt4(outBody->mLinearDOFsMask);
		angular.StoreInt4(outBody->mAngularDOFsMask);
	}
	else
	{
		FillSentinel(outBody->mForce, 3);
		FillSentinel(outBody->mTorque, 3);
		outBody->mInvMass = cSentinel;
		FillSentinel(outBody->mInvInertiaDiagonal, 3);
		FillSentinel(outBody->mInertiaRotation, 4);
		FillSentinel(outBody->mLocalInvInertia, 16);
		FillSentinel(outBody->mInvInertia, 16);
	}
	outBody->mFriction = inBody.GetFriction();
	outBody->mRestitution = inBody.GetRestitution();
	outBody->mID = inBody.GetID().GetIndexAndSequenceNumber();
	outBody->mBodyType = uint32(inBody.GetBodyType());
	outBody->mMotionType = uint32(inBody.GetMotionType());
	outBody->mFlags = (inBody.IsSensor()? 1 : 0)
		| (inBody.GetCollideKinematicVsNonDynamic()? 2 : 0)
		| (inBody.IsInBroadPhase()? 4 : 0)
		| (inBody.IsCollisionCacheInvalid()? 8 : 0)
		| (inBody.GetUseManifoldReduction()? 16 : 0)
		| (inBody.GetApplyGyroscopicForce()? 32 : 0)
		| (inBody.GetEnhancedInternalEdgeRemoval()? 64 : 0)
		| (inBody.CanBeKinematicOrDynamic()? 128 : 0);
	outBody->mIndexInActiveBodies = inBody.GetIndexInActiveBodiesInternal();
	outBody->mObjectLayer = inBody.GetObjectLayer();
	outBody->mBroadPhaseLayer = inBody.GetBroadPhaseLayer().GetValue();
	outBody->mIsActive = inBody.IsActive()? 1 : 0;

	StateRecorderImpl recorder;
	inBody.SaveState(recorder);
	outBody->mStateSize = SaveBytes(recorder, outBody->mState, sizeof(outBody->mState));
}

extern "C"
{

void *jolt_body_world_create(uint32 inMaxBodies, uint32 inNumBodyMutexes)
{
	BodyEnsureFactory();
	BodyWorld *world = new BodyWorld;
	world->mBodyManager.Init(inMaxBodies, inNumBodyMutexes, world->mLayers);
	return world;
}

void jolt_body_world_destroy(void *inWorld)
{
	delete static_cast<BodyWorld *>(inWorld);
}

// Returns the ID of the new body or cInvalidBodyID when out of bodies (or the custom ID is not available)
uint32 jolt_body_world_create_body(void *inWorld, const BodyDesc *inDesc, uint32 inCustomID, uint32 inActivate)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	BodyCreationSettings settings;
	MakeBodyCreationSettings(*inDesc, settings);
	Body *body = bm.AllocateBody(settings);
	bool added = inCustomID == BodyID::cInvalidBodyID? bm.AddBody(body) : bm.AddBodyWithCustomID(body, BodyID(inCustomID));
	if (!added)
	{
		bm.FreeBody(body);
		return BodyID::cInvalidBodyID;
	}
	body->SetInBroadPhaseInternal(true);
	BodyID id = body->GetID();
	if (inActivate != 0 && !body->IsStatic())
		bm.ActivateBodies(&id, 1);
	return id.GetIndexAndSequenceNumber();
}

void jolt_body_world_destroy_body(void *inWorld, uint32 inID)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	BodyID id(inID);
	bm.DeactivateBodies(&id, 1);
	bm.GetBody(id).SetInBroadPhaseInternal(false);
	bm.DestroyBodies(&id, 1);
}

void jolt_body_world_activate(void *inWorld, const uint32 *inIDs, uint32 inCount, uint32 inActivate)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
		ids.push_back(BodyID(inIDs[i]));
	if (inActivate != 0)
		bm.ActivateBodies(ids.data(), int(inCount));
	else
		bm.DeactivateBodies(ids.data(), int(inCount));
}

void jolt_body_world_set_motion_quality(void *inWorld, uint32 inID, uint32 inQuality)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	bm.SetMotionQuality(bm.GetBody(BodyID(inID)), EMotionQuality(inQuality));
}

uint64 jolt_body_world_mutex_mask(void *inWorld, const uint32 *inIDs, uint32 inCount)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	Array<BodyID> ids;
	for (uint32 i = 0; i < inCount; ++i)
		ids.push_back(BodyID(inIDs[i]));
	return bm.GetMutexMask(ids.data(), int(inCount));
}

// outCounts: number of bodies, max bodies, number of active bodies, number of active CCD bodies, number of body IDs,
// state size, then the BodyStats fields (9)
void jolt_body_world_info(void *inWorld, uint32 *outCounts, uint32 *outActive, uint32 *outBodyIDs, uint8 *outState, uint32 inStateCapacity)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	outCounts[0] = bm.GetNumBodies();
	outCounts[1] = bm.GetMaxBodies();
	uint32 num_active = bm.GetNumActiveBodies(EBodyType::RigidBody);
	outCounts[2] = num_active;
	for (uint32 i = 0; i < num_active; ++i)
		outActive[i] = bm.GetActiveBodiesUnsafe(EBodyType::RigidBody)[i].GetIndexAndSequenceNumber();
	outCounts[3] = bm.GetNumActiveCCDBodies();
	BodyIDVector ids;
	bm.GetBodyIDs(ids);
	outCounts[4] = uint32(ids.size());
	for (size_t i = 0; i < ids.size(); ++i)
		outBodyIDs[i] = ids[i].GetIndexAndSequenceNumber();
	StateRecorderImpl recorder;
	bm.SaveState(recorder, nullptr);
	outCounts[5] = SaveBytes(recorder, outState, inStateCapacity);
	BodyManager::BodyStats stats = bm.GetBodyStats();
	outCounts[6] = stats.mNumBodies;
	outCounts[7] = stats.mMaxBodies;
	outCounts[8] = stats.mNumBodiesStatic;
	outCounts[9] = stats.mNumBodiesDynamic;
	outCounts[10] = stats.mNumActiveBodiesDynamic;
	outCounts[11] = stats.mNumBodiesKinematic;
	outCounts[12] = stats.mNumActiveBodiesKinematic;
	outCounts[13] = stats.mNumSoftBodies;
	outCounts[14] = stats.mNumActiveSoftBodies;
}

// Restore a state saved by jolt_body_world_info, returns RestoreState's result
uint32 jolt_body_world_restore(void *inWorld, const uint8 *inState, uint32 inSize, uint32 inValidating)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	StateRecorderImpl recorder;
	recorder.WriteBytes(inState, inSize);
	recorder.Rewind();
	recorder.SetValidating(inValidating != 0);
	return bm.RestoreState(recorder)? 1 : 0;
}

void jolt_body_get(void *inWorld, uint32 inID, BodyOut *outBody)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	GetBodyOut(bm.GetBody(BodyID(inID)), outBody);
}

// Apply one operation to a body, see BodyOpKind in BodyParity.zig
void jolt_body_op(void *inWorld, uint32 inID, const BodyOp *inOp, BodyOpResult *outResult)
{
	BodyManager &bm = static_cast<BodyWorld *>(inWorld)->mBodyManager;
	Body &body = bm.GetBody(BodyID(inID));
	MotionProperties *mp = body.GetMotionPropertiesUnchecked();
	const float *f = inOp->mF;
	const Real *r = inOp->mR;
	const uint32 *i = inOp->mI;
	float *of = outResult->mF;

	switch (inOp->mKind)
	{
	case 0: // SetMassProperties
		{
			MassProperties mass_properties;
			mass_properties.mMass = f[0];
			mass_properties.mInertia = BLoadMat44(f + 4);
			mp->SetMassProperties(EAllowedDOFs(i[0]), mass_properties);
		}
		break;

	case 1: // ApplyForceTorqueAndDragInternal
		mp->ApplyForceTorqueAndDragInternal(body.GetRotation(), BLoad3(f), f[3]);
		break;

	case 2: // ApplyGyroscopicForceInternal
		mp->ApplyGyroscopicForceInternal(body.GetRotation(), f[0]);
		break;

	case 3: // Set the velocities without clamping, then clamp
		mp->SetLinearVelocity(BLoad3(f));
		mp->SetAngularVelocity(BLoad3(f + 3));
		mp->ClampLinearVelocity();
		mp->ClampAngularVelocity();
		break;

	case 4: // The clamping setters of the body
		body.SetLinearVelocityClamped(BLoad3(f));
		body.SetAngularVelocityClamped(BLoad3(f + 3));
		break;

	case 5: // Forces and torques
		body.AddForce(BLoad3(f));
		body.AddForce(BLoad3(f + 3), BLoadR3(r));
		body.AddTorque(BLoad3(f + 6));
		break;

	case 6: // Impulses
		body.AddImpulse(BLoad3(f));
		body.AddImpulse(BLoad3(f + 3), BLoadR3(r));
		body.AddAngularImpulse(BLoad3(f + 6));
		break;

	case 7: // MoveKinematic
		body.MoveKinematic(BLoadR3(r), BLoadQuat(f), f[4]);
		break;

	case 8: // SetPositionAndRotationInternal
		body.SetPositionAndRotationInternal(BLoadR3(r), BLoadQuat(f), i[0] != 0);
		break;

	case 9: // UpdateSleepStateInternal
		outResult->mI[0] = uint32(body.UpdateSleepStateInternal(f[0], f[1], f[2]));
		break;

	case 10: // ApplyBuoyancyImpulse
		outResult->mI[0] = body.ApplyBuoyancyImpulse(BLoadR3(r), BLoad3(f), f[3], f[4], f[5], BLoad3(f + 6), BLoad3(f + 9), f[12])? 1 : 0;
		break;

	case 11: // GetWorldSpaceSurfaceNormal
		{
			SubShapeID id;
			id.SetValue(i[0]);
			BStore3(body.GetWorldSpaceSurfaceNormal(id, BLoadR3(r)), of);
		}
		break;

	case 12: // Point velocities
		BStore3(body.GetPointVelocity(BLoadR3(r)), of);
		BStore3(body.GetPointVelocityCOM(BLoad3(f)), of + 3);
		break;

	case 13: // Inverse inertia
		BStoreMat44(mp->GetInverseInertiaForRotation(Mat44::sRotation(BLoadQuat(f))), of);
		BStore3(mp->MultiplyWorldSpaceInverseInertiaByVector(BLoadQuat(f + 4), BLoad3(f + 8)), of + 16);
		break;

	case 14: // SetShapeInternal
		{
			RefConst<Shape> shape = MakeShape(inOp->mShape);
			body.SetShapeInternal(shape, i[0] != 0);
		}
		break;

	case 15: // Position and rotation steps
		body.AddPositionStep(BLoad3(f));
		body.AddRotationStep(BLoad3(f + 3));
		body.SubPositionStep(BLoad3(f + 6));
		body.SubRotationStep(BLoad3(f + 9));
		break;

	case 16: // ScaleToMass
		mp->ScaleToMass(f[0]);
		break;

	case 17: // SetMotionType
		body.SetMotionType(EMotionType(i[0]));
		break;

	case 18: // Velocity steps
		mp->AddLinearVelocityStep(BLoad3(f));
		mp->SubLinearVelocityStep(BLoad3(f + 3));
		mp->AddAngularVelocityStep(BLoad3(f + 6));
		mp->SubAngularVelocityStep(BLoad3(f + 9));
		break;

	case 19: // GetSubmergedVolume
		{
			float total, submerged;
			Vec3 center;
			body.GetSubmergedVolume(BLoadR3(r), BLoad3(f), total, submerged, center);
			of[0] = total;
			of[1] = submerged;
			BStore3(center, of + 2);
		}
		break;

	case 20: // Sleep timer
		body.ResetSleepTimer();
		outResult->mI[0] = uint32(mp->AccumulateSleepTime(f[0], f[1]));
		break;

	case 21: // GetBodyCreationSettings
		{
			BodyCreationSettings s = body.GetBodyCreationSettings();
			BStoreR3(s.mPosition, outResult->mR);
			BStoreQuat(s.mRotation, of);
			BStore3(s.mLinearVelocity, of + 4);
			BStore3(s.mAngularVelocity, of + 7);
			of[10] = s.mFriction;
			of[11] = s.mRestitution;
			of[12] = s.mLinearDamping;
			of[13] = s.mAngularDamping;
			of[14] = s.mMaxLinearVelocity;
			of[15] = s.mMaxAngularVelocity;
			of[16] = s.mGravityFactor;
			of[17] = s.mInertiaMultiplier;
			of[18] = s.mMassPropertiesOverride.mMass;
			BStoreMat44(s.mMassPropertiesOverride.mInertia, of + 19);
			uint32 *oi = outResult->mI;
			oi[0] = uint32(s.mUserData);
			oi[1] = s.mObjectLayer;
			oi[2] = uint32(s.mMotionType);
			oi[3] = uint32(s.mAllowedDOFs);
			oi[4] = (s.mAllowDynamicOrKinematic? 1 : 0) | (s.mIsSensor? 2 : 0) | (s.mCollideKinematicVsNonDynamic? 4 : 0) | (s.mUseManifoldReduction? 8 : 0)
				| (s.mApplyGyroscopicForce? 16 : 0) | (s.mEnhancedInternalEdgeRemoval? 32 : 0) | (s.mAllowSleeping? 64 : 0) | (s.GetShape() == body.GetShape()? 128 : 0);
			oi[5] = uint32(s.mMotionQuality);
			oi[6] = s.mNumVelocityStepsOverride;
			oi[7] = s.mNumPositionStepsOverride;
			oi[8] = uint32(s.mOverrideMassProperties);
			oi[9] = s.mCollisionGroup.GetGroupID();
			oi[10] = s.mCollisionGroup.GetSubGroupID();
		}
		break;

	case 22: // Flags and material properties
		body.SetIsSensor(i[0] != 0);
		body.SetCollideKinematicVsNonDynamic(i[1] != 0);
		body.SetUseManifoldReduction(i[2] != 0);
		body.SetApplyGyroscopicForce(i[3] != 0);
		body.SetEnhancedInternalEdgeRemoval(i[4] != 0);
		body.SetFriction(f[0]);
		body.SetRestitution(f[1]);
		if (mp != nullptr)
			body.SetAllowSleeping(i[5] != 0);
		break;

	case 23: // MotionProperties setters
		mp->SetMaxLinearVelocity(f[0]);
		mp->SetMaxAngularVelocity(f[1]);
		mp->SetLinearDamping(f[2]);
		mp->SetAngularDamping(f[3]);
		mp->SetGravityFactor(f[4]);
		mp->SetNumVelocityStepsOverride(i[0]);
		mp->SetNumPositionStepsOverride(i[1]);
		break;

	case 24: // Contact cache
		bm.InvalidateContactCacheForBody(body);
		if (i[0] != 0)
			bm.ValidateContactCacheForAllBodies();
		break;

	case 25: // MoveKinematic of MotionProperties
		mp->MoveKinematic(BLoad3(f), BLoadQuat(f + 4), f[8]);
		break;
	}
}

// BodyCreationSettings::GetMassProperties: outMP = mass + 16 floats of inertia
void jolt_body_bcs_mass_properties(const BodyDesc *inDesc, float *outMP)
{
	BodyEnsureFactory();
	BodyCreationSettings settings;
	MakeBodyCreationSettings(*inDesc, settings);
	MassProperties mp = settings.GetMassProperties();
	outMP[0] = mp.mMass;
	BStoreMat44(mp.mInertia, outMP + 1);
}

} // extern "C"
