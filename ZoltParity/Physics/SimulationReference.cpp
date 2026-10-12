// Reference implementation of the simulation parity tests (`zig build parity`, SimulationArchitecture D12): a
// JPH::PhysicsSystem with the layers of UnitTests/Layers.h, built from a scene description of
// ZoltParity/Physics/SimScenes.zig, stepped with a JobSystemThreadPool. ZoltParity/Physics/SimulationParity.zig builds
// the same scene with Zolt, applies the same operations in the same order and compares the SaveState bytes after every
// update and the listener callbacks (body activation, step listener, contact listener) as sorted records.
//
// The structs below must match the extern structs of SimScenes.zig (checked by jolt_sim_struct_sizes). Booleans are
// passed as uint32 (never bool, see the porting guide). All C functions have the prefix jolt_sim_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/Mutex.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsStepListener.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace JPH;

// The layers of the unit tests (Layers, BroadPhaseLayers, ObjectLayerPairFilterImpl, BPLayerInterfaceImpl,
// ObjectVsBroadPhaseLayerFilterImpl), mirrored by ZoltTests/Layers.zig
#include <UnitTests/Layers.h>

// Jolt's Array / String allocate through the Allocate / Free hooks and the shapes need the Factory (same guard as
// BasicsReference.cpp, RegisterTypes() runs once for the whole parity binary)
static void SimEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

// Must match SimShape in SimScenes.zig
struct SimShape
{
	uint32				mKind;
	float				mParams[12];
	float				mRotation[4];
	uint32				mChild;
	uint32				mCount;
	uint32				mData;
	float				mDensity;
};

// Must match SimCompoundChild in SimScenes.zig
struct SimCompoundChild
{
	uint32				mShape;
	uint32				mUserData;
	float				mPosition[3];
	float				mRotation[4];
};

// Must match SimBody in SimScenes.zig
struct SimBody
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
	uint32				mShape;
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
	uint32				mCollisionGroupID;
	uint32				mCollisionSubGroupID;
};

// Must match SimPhysicsSettings in SimScenes.zig
struct SimPhysicsSettings
{
	int32				mMaxInFlightBodyPairs;
	int32				mStepListenersBatchSize;
	int32				mStepListenerBatchesPerJob;
	float				mBaumgarte;
	float				mSpeculativeContactDistance;
	float				mPenetrationSlop;
	float				mLinearCastThreshold;
	float				mLinearCastMaxPenetration;
	float				mManifoldTolerance;
	float				mMaxPenetrationDistance;
	float				mBodyPairCacheMaxDeltaPositionSq;
	float				mBodyPairCacheCosMaxDeltaRotationDiv2;
	float				mContactNormalCosMaxDeltaRotation;
	float				mContactPointPreserveLambdaMaxDistSq;
	float				mInternalEdgeRemovalVertexToleranceSq;
	uint32				mNumVelocitySteps;
	uint32				mNumPositionSteps;
	float				mMinVelocityForRestitution;
	float				mTimeBeforeSleep;
	float				mPointVelocitySleepThreshold;
	uint32				mConstraintWarmStart;
	uint32				mUseBodyPairContactCache;
	uint32				mUseManifoldReduction;
	uint32				mUseLargeIslandSplitter;
	uint32				mAllowSleeping;
	uint32				mCheckActiveEdges;
};

// Must match SimStepListener in SimScenes.zig
struct SimStepListener
{
	uint32				mBody;
	uint32				mMode;
	float				mF[4];
};

// Must match SimConstraint in SimScenes.zig
struct SimConstraint
{
	uint32				mKind;
	uint32				mBody1;
	uint32				mBody2;
	float				mF[16];
};

// Must match SimOp in SimScenes.zig
struct SimOp
{
	uint32				mUpdate;
	uint32				mKind;
	uint32				mBody;
	uint32				mCount;
	uint32				mActivation;
	uint32				mIndex;
	float				mF[16];
	Real				mR[3];
};

// Must match SimSceneDesc in SimScenes.zig
struct SimSceneDesc
{
	uint32						mMaxBodies;
	uint32						mNumBodyMutexes;
	uint32						mMaxBodyPairs;
	uint32						mMaxContactConstraints;
	float						mGravity[3];
	SimPhysicsSettings			mSettings;
	const SimShape *			mShapes;
	uint32						mNumShapes;
	const SimCompoundChild *	mChildren;
	uint32						mNumChildren;
	const float *				mFloats;
	uint32						mNumFloats;
	const SimBody *				mBodies;
	uint32						mNumBodies;
	const SimStepListener *		mStepListeners;
	uint32						mNumStepListeners;
	const SimConstraint *		mConstraints;
	uint32						mNumConstraints;
	const SimPhysicsSettings *	mPhysicsSettings;
	uint32						mNumPhysicsSettings;
	uint32						mContactListenerMode;
	uint32						mCombineMode;
	uint32						mSimShapeFilterMode;
};

// Must match ContactFlags in SimScenes.zig (bits in the upper 32 bits of the user data of a body)
namespace SimContactFlags
{
	static constexpr uint32 cRejectAll = 1 << 0;
	static constexpr uint32 cRejectSpeculative = 1 << 1;
	static constexpr uint32 cAcceptEach = 1 << 2;
	static constexpr uint32 cZeroFriction = 1 << 3;
	static constexpr uint32 cBouncy = 1 << 4;
	static constexpr uint32 cMassScale = 1 << 5;
	static constexpr uint32 cSurfaceVelocity = 1 << 6;
	static constexpr uint32 cSensor = 1 << 7;
	static constexpr uint32 cFilterSpheres = 1 << 8;
	static constexpr uint32 cRejectMarkedChild = 1 << 9;
	static constexpr uint32 cMarkedChildUserData = 77;
}

// The ContactFlags of a body pair
static uint32 SimContactFlagsOf(const Body &inBody1, const Body &inBody2)
{
	return uint32((inBody1.GetUserData() | inBody2.GetUserData()) >> 32);
}

// ContactFlags.reject_marked_child of SimScenes.zig: the sub shape is a child of a compound with the marked user data
static bool SimIsMarkedChild(const Body &inBody, const SubShapeID &inSubShapeID)
{
	const Shape *shape = inBody.GetShape();
	if (shape->GetType() != EShapeType::Compound)
		return false;
	const CompoundShape *compound = static_cast<const CompoundShape *>(shape);
	SubShapeID remainder;
	return compound->GetCompoundUserData(compound->GetSubShapeIndexFromID(inSubShapeID, remainder)) == SimContactFlags::cMarkedChildUserData;
}

// FNV-1a (32 bit), hashBytes of SimScenes.zig
static uint32 SimHashBytes(uint32 inHash, const void *inData, size_t inSize)
{
	uint32 hash = inHash;
	const uint8 *data = static_cast<const uint8 *>(inData);
	for (size_t i = 0; i < inSize; ++i)
	{
		hash ^= data[i];
		hash *= 16777619u;
	}
	return hash;
}

// The hash of the base offset and the contact points of a manifold, hashManifold of SimulationParity.zig
static uint32 SimHashManifold(const ContactManifold &inManifold)
{
	uint32 hash = 2166136261u;
	Real offset[3] = { inManifold.mBaseOffset.GetX(), inManifold.mBaseOffset.GetY(), inManifold.mBaseOffset.GetZ() };
	hash = SimHashBytes(hash, offset, sizeof(offset));
	for (const ContactPoints *points : { &inManifold.mRelativeContactPointsOn1, &inManifold.mRelativeContactPointsOn2 })
		for (Vec3 p : *points)
		{
			float v[3] = { p.GetX(), p.GetY(), p.GetZ() };
			hash = SimHashBytes(hash, v, sizeof(v));
		}
	return hash;
}

// CombineMode.custom of SimScenes.zig
static float SimCombineFriction(const Body &inBody1, const SubShapeID &inSubShapeID1, const Body &inBody2, const SubShapeID &inSubShapeID2)
{
	return 0.5f * (inBody1.GetFriction() + inBody2.GetFriction());
}

static float SimCombineRestitution(const Body &inBody1, const SubShapeID &inSubShapeID1, const Body &inBody2, const SubShapeID &inSubShapeID2)
{
	return inSubShapeID1.GetValue() == inSubShapeID2.GetValue()? max(inBody1.GetRestitution(), inBody2.GetRestitution()) : min(inBody1.GetRestitution(), inBody2.GetRestitution());
}

// SimShapeFilterMode.filter_spheres of SimScenes.zig
class SimSphereFilter final : public SimShapeFilter
{
public:
	virtual bool				ShouldCollide(const Body &inBody1, const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Body &inBody2, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		if ((SimContactFlagsOf(inBody1, inBody2) & SimContactFlags::cFilterSpheres) == 0)
			return true;
		return inShape1->GetSubType() != EShapeSubType::Sphere && inShape2->GetSubType() != EShapeSubType::Sphere;
	}
};

// Must match SimRecord in SimScenes.zig
struct SimRecord
{
	uint32				mKind;
	uint32				mIDs[4];
	uint32				mData[8];
};

// Must match RecordKind in SimScenes.zig
enum class ESimRecordKind : uint32
{
	BodyActivated = 0,
	BodyDeactivated = 1,
	Step = 2,
	ContactValidate = 3,
	ContactAdded = 4,
	ContactPersisted = 5,
	ContactRemoved = 6,
};

// Must match OpKind in SimScenes.zig
enum class ESimOp : uint32
{
	CreateBody = 0,
	CreateAndAddBody = 1,
	CreateBodyNoAdd = 2,
	AddBody = 3,
	RemoveBody = 4,
	DestroyBody = 5,
	AddBodies = 6,
	RemoveBodies = 7,
	ActivateBody = 8,
	DeactivateBody = 9,
	ActivateBodies = 10,
	DeactivateBodies = 11,
	AddForce = 12,
	AddForceAtPosition = 13,
	AddTorque = 14,
	AddForceAndTorque = 15,
	AddImpulse = 16,
	AddImpulseAtPosition = 17,
	AddAngularImpulse = 18,
	SetLinearVelocity = 19,
	SetAngularVelocity = 20,
	SetLinearAndAngularVelocity = 21,
	AddLinearVelocity = 22,
	AddLinearAndAngularVelocity = 23,
	SetPosition = 24,
	SetRotation = 25,
	SetPositionAndRotation = 26,
	SetPositionAndRotationWhenChanged = 27,
	SetPositionRotationAndVelocity = 28,
	MoveKinematic = 29,
	SetMotionType = 30,
	SetMotionQuality = 31,
	SetGravityFactor = 32,
	SetMaxLinearVelocity = 33,
	SetMaxAngularVelocity = 34,
	ResetSleepTimer = 35,
	SetGravity = 36,
	SetPhysicsSettings = 37,
	AddStepListener = 38,
	RemoveStepListener = 39,
	OptimizeBroadPhase = 40,
	SetObjectLayer = 41,
	SetUserData = 42,
	SetFrictionAndRestitution = 43,
	CreateConstraint = 44,
	SetShape = 45,
	ActivateBodiesInAABox = 46,
	ApplyBuoyancyImpulse = 47,
	ExpectBodyState = 48,
	SetUseManifoldReduction = 49,
	ApplyBodyCreationSettings = 50,
};

static constexpr uint32 cInvalidHandle = 0xffffffff;

static Vec3 SimLoad3(const float *inV)		{ return Vec3(inV[0], inV[1], inV[2]); }
static Quat SimLoadQuat(const float *inV)	{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 SimLoadR3(const Real *inV)		{ return RVec3(inV[0], inV[1], inV[2]); }

static uint32 SimFloatBits(float inV)
{
	uint32 bits;
	memcpy(&bits, &inV, sizeof(bits));
	return bits;
}

static PhysicsSettings SimToPhysicsSettings(const SimPhysicsSettings &inS)
{
	PhysicsSettings s;
	s.mMaxInFlightBodyPairs = inS.mMaxInFlightBodyPairs;
	s.mStepListenersBatchSize = inS.mStepListenersBatchSize;
	s.mStepListenerBatchesPerJob = inS.mStepListenerBatchesPerJob;
	s.mBaumgarte = inS.mBaumgarte;
	s.mSpeculativeContactDistance = inS.mSpeculativeContactDistance;
	s.mPenetrationSlop = inS.mPenetrationSlop;
	s.mLinearCastThreshold = inS.mLinearCastThreshold;
	s.mLinearCastMaxPenetration = inS.mLinearCastMaxPenetration;
	s.mManifoldTolerance = inS.mManifoldTolerance;
	s.mMaxPenetrationDistance = inS.mMaxPenetrationDistance;
	s.mBodyPairCacheMaxDeltaPositionSq = inS.mBodyPairCacheMaxDeltaPositionSq;
	s.mBodyPairCacheCosMaxDeltaRotationDiv2 = inS.mBodyPairCacheCosMaxDeltaRotationDiv2;
	s.mContactNormalCosMaxDeltaRotation = inS.mContactNormalCosMaxDeltaRotation;
	s.mContactPointPreserveLambdaMaxDistSq = inS.mContactPointPreserveLambdaMaxDistSq;
	s.mInternalEdgeRemovalVertexToleranceSq = inS.mInternalEdgeRemovalVertexToleranceSq;
	s.mNumVelocitySteps = inS.mNumVelocitySteps;
	s.mNumPositionSteps = inS.mNumPositionSteps;
	s.mMinVelocityForRestitution = inS.mMinVelocityForRestitution;
	s.mTimeBeforeSleep = inS.mTimeBeforeSleep;
	s.mPointVelocitySleepThreshold = inS.mPointVelocitySleepThreshold;
	s.mConstraintWarmStart = inS.mConstraintWarmStart != 0;
	s.mUseBodyPairContactCache = inS.mUseBodyPairContactCache != 0;
	s.mUseManifoldReduction = inS.mUseManifoldReduction != 0;
	s.mUseLargeIslandSplitter = inS.mUseLargeIslandSplitter != 0;
	s.mAllowSleeping = inS.mAllowSleeping != 0;
	s.mCheckActiveEdges = inS.mCheckActiveEdges != 0;
	return s;
}

static void SimFromPhysicsSettings(const PhysicsSettings &inS, SimPhysicsSettings &outS)
{
	outS.mMaxInFlightBodyPairs = inS.mMaxInFlightBodyPairs;
	outS.mStepListenersBatchSize = inS.mStepListenersBatchSize;
	outS.mStepListenerBatchesPerJob = inS.mStepListenerBatchesPerJob;
	outS.mBaumgarte = inS.mBaumgarte;
	outS.mSpeculativeContactDistance = inS.mSpeculativeContactDistance;
	outS.mPenetrationSlop = inS.mPenetrationSlop;
	outS.mLinearCastThreshold = inS.mLinearCastThreshold;
	outS.mLinearCastMaxPenetration = inS.mLinearCastMaxPenetration;
	outS.mManifoldTolerance = inS.mManifoldTolerance;
	outS.mMaxPenetrationDistance = inS.mMaxPenetrationDistance;
	outS.mBodyPairCacheMaxDeltaPositionSq = inS.mBodyPairCacheMaxDeltaPositionSq;
	outS.mBodyPairCacheCosMaxDeltaRotationDiv2 = inS.mBodyPairCacheCosMaxDeltaRotationDiv2;
	outS.mContactNormalCosMaxDeltaRotation = inS.mContactNormalCosMaxDeltaRotation;
	outS.mContactPointPreserveLambdaMaxDistSq = inS.mContactPointPreserveLambdaMaxDistSq;
	outS.mInternalEdgeRemovalVertexToleranceSq = inS.mInternalEdgeRemovalVertexToleranceSq;
	outS.mNumVelocitySteps = inS.mNumVelocitySteps;
	outS.mNumPositionSteps = inS.mNumPositionSteps;
	outS.mMinVelocityForRestitution = inS.mMinVelocityForRestitution;
	outS.mTimeBeforeSleep = inS.mTimeBeforeSleep;
	outS.mPointVelocitySleepThreshold = inS.mPointVelocitySleepThreshold;
	outS.mConstraintWarmStart = inS.mConstraintWarmStart? 1 : 0;
	outS.mUseBodyPairContactCache = inS.mUseBodyPairContactCache? 1 : 0;
	outS.mUseManifoldReduction = inS.mUseManifoldReduction? 1 : 0;
	outS.mUseLargeIslandSplitter = inS.mUseLargeIslandSplitter? 1 : 0;
	outS.mAllowSleeping = inS.mAllowSleeping? 1 : 0;
	outS.mCheckActiveEdges = inS.mCheckActiveEdges? 1 : 0;
}

class SimWorld;

// Records the callbacks of the listeners (callbacks come from the worker threads)
class SimRecorder
{
public:
	void						Add(const SimRecord &inRecord)
	{
		UniqueLock lock(mMutex);
		mRecords.push_back(inRecord);
	}

	Mutex						mMutex;
	Array<SimRecord>			mRecords;
};

class SimActivationListener final : public BodyActivationListener
{
public:
	virtual void				OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		Record(ESimRecordKind::BodyActivated, inBodyID, inBodyUserData);
	}

	virtual void				OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		Record(ESimRecordKind::BodyDeactivated, inBodyID, inBodyUserData);
	}

	SimRecorder *				mRecorder = nullptr;

private:
	void						Record(ESimRecordKind inKind, const BodyID &inBodyID, uint64 inBodyUserData)
	{
		SimRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inBodyID.GetIndexAndSequenceNumber();
		r.mData[0] = uint32(inBodyUserData);
		r.mData[1] = uint32(inBodyUserData >> 32);
		mRecorder->Add(r);
	}
};

class SimContactListener final : public ContactListener
{
public:
	virtual ValidateResult		OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
	{
		Record(inBody1, inBody2, inCollisionResult);

		// ContactListenerMode.modify of SimScenes.zig
		if (mMode == 1)
		{
			uint32 flags = SimContactFlagsOf(inBody1, inBody2);
			if (flags & SimContactFlags::cRejectAll)
				return ValidateResult::RejectAllContactsForThisBodyPair;
			if ((flags & SimContactFlags::cRejectMarkedChild) && SimIsMarkedChild(inBody2, inCollisionResult.mSubShapeID2))
				return ValidateResult::RejectContact;
			if (flags & SimContactFlags::cRejectSpeculative)
				return inCollisionResult.mPenetrationDepth < 0.0f? ValidateResult::RejectContact : ValidateResult::AcceptContact;
			if (flags & SimContactFlags::cAcceptEach)
				return ValidateResult::AcceptContact;
		}
		return ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	void						Record(const Body &inBody1, const Body &inBody2, const CollideShapeResult &inCollisionResult)
	{
		SimRecord r = { };
		r.mKind = uint32(ESimRecordKind::ContactValidate);
		r.mIDs[0] = inBody1.GetID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inBody2.GetID().GetIndexAndSequenceNumber();
		r.mIDs[2] = inCollisionResult.mSubShapeID1.GetValue();
		r.mIDs[3] = inCollisionResult.mSubShapeID2.GetValue();
		r.mData[0] = SimFloatBits(inCollisionResult.mPenetrationDepth);
		r.mData[1] = SimFloatBits(inCollisionResult.mPenetrationAxis.GetX());
		r.mData[2] = SimFloatBits(inCollisionResult.mPenetrationAxis.GetY());
		r.mData[3] = SimFloatBits(inCollisionResult.mPenetrationAxis.GetZ());
		mRecorder->Add(r);
	}

	virtual void				OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		Record(ESimRecordKind::ContactAdded, inBody1, inBody2, inManifold, ioSettings);
		Modify(inBody1, inBody2, ioSettings);
	}

	virtual void				OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		Record(ESimRecordKind::ContactPersisted, inBody1, inBody2, inManifold, ioSettings);
		Modify(inBody1, inBody2, ioSettings);
	}

	virtual void				OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{
		SimRecord r = { };
		r.mKind = uint32(ESimRecordKind::ContactRemoved);
		r.mIDs[0] = inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inSubShapePair.GetSubShapeID1().GetValue();
		r.mIDs[2] = inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber();
		r.mIDs[3] = inSubShapePair.GetSubShapeID2().GetValue();
		mRecorder->Add(r);
	}

	SimRecorder *				mRecorder = nullptr;
	uint32						mMode = 0;

private:
	// ContactListenerMode.modify of SimScenes.zig
	void						Modify(const Body &inBody1, const Body &inBody2, ContactSettings &ioSettings) const
	{
		if (mMode != 1)
			return;
		uint32 flags = SimContactFlagsOf(inBody1, inBody2);
		if (flags & SimContactFlags::cZeroFriction)
			ioSettings.mCombinedFriction = 0.0f;
		if (flags & SimContactFlags::cBouncy)
			ioSettings.mCombinedRestitution = 0.9f;
		if (flags & SimContactFlags::cMassScale)
		{
			ioSettings.mInvMassScale1 = 0.5f;
			ioSettings.mInvInertiaScale1 = 0.25f;
			ioSettings.mInvMassScale2 = 2.0f;
			ioSettings.mInvInertiaScale2 = 1.5f;
		}
		if (flags & SimContactFlags::cSurfaceVelocity)
		{
			ioSettings.mRelativeLinearSurfaceVelocity = Vec3(1.5f, 0.0f, 0.5f);
			ioSettings.mRelativeAngularSurfaceVelocity = Vec3(0.0f, 0.7f, 0.0f);
		}
		if (flags & SimContactFlags::cSensor)
			ioSettings.mIsSensor = true;
	}

	void						Record(ESimRecordKind inKind, const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, const ContactSettings &inSettings)
	{
		SimRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inBody1.GetID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inBody2.GetID().GetIndexAndSequenceNumber();
		r.mIDs[2] = inManifold.mSubShapeID1.GetValue();
		r.mIDs[3] = inManifold.mSubShapeID2.GetValue();
		r.mData[0] = SimFloatBits(inManifold.mWorldSpaceNormal.GetX());
		r.mData[1] = SimFloatBits(inManifold.mWorldSpaceNormal.GetY());
		r.mData[2] = SimFloatBits(inManifold.mWorldSpaceNormal.GetZ());
		r.mData[3] = SimFloatBits(inManifold.mPenetrationDepth);
		r.mData[4] = uint32(inManifold.mRelativeContactPointsOn1.size()) | ((inSettings.mIsSensor? 1u : 0u) << 8);
		r.mData[5] = SimFloatBits(inSettings.mCombinedFriction);
		r.mData[6] = SimFloatBits(inSettings.mCombinedRestitution);
		r.mData[7] = SimHashManifold(inManifold);
		mRecorder->Add(r);
	}
};

class SimStepListenerImpl final : public PhysicsStepListener
{
public:
	virtual void				OnStep(const PhysicsStepListenerContext &inContext) override;

	SimWorld *					mWorld = nullptr;
	uint32						mIndex = 0;
	uint32						mNumCalls = 0;
};

class SimWorld
{
public:
								SimWorld(const SimSceneDesc &inDesc, uint32 inNumThreads) :
		mDesc(inDesc),
		mJobSystem(cMaxPhysicsJobs, cMaxPhysicsBarriers, int(inNumThreads))
	{
		mSystem.Init(inDesc.mMaxBodies, inDesc.mNumBodyMutexes, inDesc.mMaxBodyPairs, inDesc.mMaxContactConstraints, mBroadPhaseLayerInterface, mObjectVsBroadPhaseLayerFilter, mObjectLayerPairFilter);
		mSystem.SetPhysicsSettings(SimToPhysicsSettings(inDesc.mSettings));
		mSystem.SetGravity(SimLoad3(inDesc.mGravity));
		mActivationListener.mRecorder = &mRecorder;
		mContactListener.mRecorder = &mRecorder;
		mContactListener.mMode = inDesc.mContactListenerMode;
		mSystem.SetBodyActivationListener(&mActivationListener);
		mSystem.SetContactListener(&mContactListener);
		if (inDesc.mCombineMode == 1)
		{
			mSystem.SetCombineFriction(SimCombineFriction);
			mSystem.SetCombineRestitution(SimCombineRestitution);
		}
		if (inDesc.mSimShapeFilterMode == 1)
			mSystem.SetSimShapeFilter(&mSphereFilter);

		for (uint32 i = 0; i < inDesc.mNumShapes; ++i)
			mShapes.push_back(MakeShape(inDesc.mShapes[i]));

		for (uint32 i = 0; i < inDesc.mNumStepListeners; ++i)
		{
			SimStepListenerImpl *listener = new SimStepListenerImpl;
			listener->mWorld = this;
			listener->mIndex = i;
			mStepListeners.push_back(listener);
		}
	}

								~SimWorld()
	{
		for (uint32 i = 0; i < uint32(mStepListeners.size()); ++i)
			if (mStepListenerAdded.size() > i && mStepListenerAdded[i])
				mSystem.RemoveStepListener(mStepListeners[i]);
		for (SimStepListenerImpl *listener : mStepListeners)
			delete listener;
	}

	BodyID						GetHandle(uint32 inHandle) const
	{
		return inHandle < mHandles.size()? mHandles[inHandle] : BodyID();
	}

	// MakeShape of SimulationParity.zig
	RefConst<Shape>				MakeShape(const SimShape &inShape) const
	{
		const float *p = inShape.mParams;
		Ref<ShapeSettings> settings;
		ConvexShapeSettings *convex = nullptr;
		switch (inShape.mKind)
		{
		case 0:		convex = new SphereShapeSettings(p[0]); break;
		case 1:		convex = new BoxShapeSettings(SimLoad3(p), p[3]); break;
		case 2:		convex = new CapsuleShapeSettings(p[0], p[1]); break;
		case 3:		convex = new TaperedCapsuleShapeSettings(p[0], p[1], p[2]); break;
		case 4:		convex = new CylinderShapeSettings(p[0], p[1], p[3]); break;
		case 5:		convex = new TaperedCylinderShapeSettings(p[0], p[1], p[2], p[3]); break;
		case 6:
			{
				Array<Vec3> points;
				for (uint32 i = 0; i < inShape.mCount; ++i)
					points.push_back(SimLoad3(mDesc.mFloats + inShape.mData + 3 * i));
				convex = new ConvexHullShapeSettings(points, p[3]);
			}
			break;
		case 7:		convex = new TriangleShapeSettings(SimLoad3(p), SimLoad3(p + 3), SimLoad3(p + 6), p[9]); break;
		case 8:		settings = new PlaneShapeSettings(Plane(SimLoad3(p), p[3]), nullptr, p[4]); break;
		case 9:
			{
				TriangleList triangles;
				for (uint32 i = 0; i < inShape.mCount; ++i)
				{
					const float *t = mDesc.mFloats + inShape.mData + 9 * i;
					triangles.push_back(Triangle(SimLoad3(t), SimLoad3(t + 3), SimLoad3(t + 6)));
				}
				settings = new MeshShapeSettings(triangles);
			}
			break;
		case 10:	settings = new EmptyShapeSettings(SimLoad3(p)); break;
		case 16:	settings = new HeightFieldShapeSettings(mDesc.mFloats + inShape.mData, SimLoad3(p), SimLoad3(p + 3), inShape.mCount); break;
		case 11:	settings = new RotatedTranslatedShapeSettings(SimLoad3(p), SimLoadQuat(inShape.mRotation), mShapes[inShape.mChild]); break;
		case 12:	settings = new ScaledShapeSettings(mShapes[inShape.mChild], SimLoad3(p)); break;
		case 13:	settings = new OffsetCenterOfMassShapeSettings(SimLoad3(p), mShapes[inShape.mChild]); break;
		case 14:
		case 15:
			{
				CompoundShapeSettings *compound = inShape.mKind == 14? static_cast<CompoundShapeSettings *>(new StaticCompoundShapeSettings) : static_cast<CompoundShapeSettings *>(new MutableCompoundShapeSettings);
				for (uint32 i = 0; i < inShape.mCount; ++i)
				{
					const SimCompoundChild &c = mDesc.mChildren[inShape.mChild + i];
					compound->AddShape(SimLoad3(c.mPosition), SimLoadQuat(c.mRotation), mShapes[c.mShape], c.mUserData);
				}
				settings = compound;
			}
			break;
		default:
			JPH_ASSERT(false);
			abort();
		}
		if (convex != nullptr)
		{
			if (inShape.mDensity != 0.0f)
				convex->SetDensity(inShape.mDensity);
			settings = convex;
		}
		ShapeSettings::ShapeResult result = settings->Create();
		if (result.HasError())
		{
			Trace("SimulationReference: shape error: %s", result.GetError().c_str());
			abort();
		}
		return result.Get();
	}

	// MakeBodyCreationSettings of SimulationParity.zig
	BodyCreationSettings		MakeBodyCreationSettings(const SimBody &inBody) const
	{
		BodyCreationSettings s(mShapes[inBody.mShape], SimLoadR3(inBody.mPosition), SimLoadQuat(inBody.mRotation), EMotionType(inBody.mMotionType), ObjectLayer(inBody.mObjectLayer));
		s.mLinearVelocity = SimLoad3(inBody.mLinearVelocity);
		s.mAngularVelocity = SimLoad3(inBody.mAngularVelocity);
		s.mUserData = inBody.mUserData;
		s.mCollisionGroup = CollisionGroup(nullptr, inBody.mCollisionGroupID, inBody.mCollisionSubGroupID);
		s.mAllowedDOFs = EAllowedDOFs(inBody.mAllowedDOFs);
		s.mAllowDynamicOrKinematic = inBody.mAllowDynamicOrKinematic != 0;
		s.mIsSensor = inBody.mIsSensor != 0;
		s.mCollideKinematicVsNonDynamic = inBody.mCollideKinematicVsNonDynamic != 0;
		s.mUseManifoldReduction = inBody.mUseManifoldReduction != 0;
		s.mApplyGyroscopicForce = inBody.mApplyGyroscopicForce != 0;
		s.mMotionQuality = EMotionQuality(inBody.mMotionQuality);
		s.mEnhancedInternalEdgeRemoval = inBody.mEnhancedInternalEdgeRemoval != 0;
		s.mAllowSleeping = inBody.mAllowSleeping != 0;
		s.mFriction = inBody.mFriction;
		s.mRestitution = inBody.mRestitution;
		s.mLinearDamping = inBody.mLinearDamping;
		s.mAngularDamping = inBody.mAngularDamping;
		s.mMaxLinearVelocity = inBody.mMaxLinearVelocity;
		s.mMaxAngularVelocity = inBody.mMaxAngularVelocity;
		s.mGravityFactor = inBody.mGravityFactor;
		s.mNumVelocityStepsOverride = inBody.mNumVelocityStepsOverride;
		s.mNumPositionStepsOverride = inBody.mNumPositionStepsOverride;
		s.mOverrideMassProperties = EOverrideMassProperties(inBody.mOverrideMassProperties);
		s.mInertiaMultiplier = inBody.mInertiaMultiplier;
		s.mMassPropertiesOverride.mMass = inBody.mMass;
		const float *m = inBody.mInertia;
		s.mMassPropertiesOverride.mInertia = Mat44(Vec4(m[0], m[1], m[2], m[3]), Vec4(m[4], m[5], m[6], m[7]), Vec4(m[8], m[9], m[10], m[11]), Vec4(m[12], m[13], m[14], m[15]));
		return s;
	}

	// The constraint settings of a SimConstraint (ConstraintKind in SimScenes.zig)
	Ref<TwoBodyConstraintSettings> MakeConstraintSettings(const SimConstraint &inConstraint) const
	{
		const float *f = inConstraint.mF;
		switch (inConstraint.mKind)
		{
		case 0:
			{
				FixedConstraintSettings *s = new FixedConstraintSettings;
				s->mAutoDetectPoint = true;
				return s;
			}
		case 1:
			{
				PointConstraintSettings *s = new PointConstraintSettings;
				s->mPoint1 = RVec3(SimLoad3(f));
				s->mPoint2 = RVec3(SimLoad3(f + 3));
				return s;
			}
		case 2:
			{
				DistanceConstraintSettings *s = new DistanceConstraintSettings;
				s->mPoint1 = RVec3(SimLoad3(f));
				s->mPoint2 = RVec3(SimLoad3(f + 3));
				s->mMinDistance = f[6];
				s->mMaxDistance = f[7];
				return s;
			}
		case 3:
			{
				HingeConstraintSettings *s = new HingeConstraintSettings;
				s->mPoint1 = s->mPoint2 = RVec3(SimLoad3(f));
				s->mHingeAxis1 = s->mHingeAxis2 = SimLoad3(f + 3);
				s->mNormalAxis1 = s->mNormalAxis2 = SimLoad3(f + 6);
				return s;
			}
		case 4:
			{
				SliderConstraintSettings *s = new SliderConstraintSettings;
				s->mAutoDetectPoint = true;
				s->SetSliderAxis(SimLoad3(f));
				return s;
			}
		default:
			JPH_ASSERT(false);
			abort();
		}
	}

	// Apply an operation, see OpKind in SimScenes.zig
	uint32						ApplyOp(const SimOp &inOp)
	{
		BodyInterface &bi = mSystem.GetBodyInterface();
		BodyID id = GetHandle(inOp.mBody);
		EActivation activation = inOp.mActivation == 0? EActivation::Activate : EActivation::DontActivate;
		const float *f = inOp.mF;
		RVec3 r = SimLoadR3(inOp.mR);

		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::CreateBody:
		case ESimOp::CreateAndAddBody:
		case ESimOp::CreateBodyNoAdd:
			{
				BodyCreationSettings settings = MakeBodyCreationSettings(mDesc.mBodies[inOp.mIndex]);
				BodyID new_id;
				if (ESimOp(inOp.mKind) == ESimOp::CreateAndAddBody)
					new_id = bi.CreateAndAddBody(settings, activation);
				else
				{
					Body *body = bi.CreateBody(settings);
					if (body != nullptr)
					{
						new_id = body->GetID();
						if (ESimOp(inOp.mKind) == ESimOp::CreateBody)
							bi.AddBody(new_id, activation);
					}
				}
				mHandles.push_back(new_id);
				return new_id.GetIndexAndSequenceNumber();
			}

		case ESimOp::AddBody:					bi.AddBody(id, activation); break;
		case ESimOp::RemoveBody:				bi.RemoveBody(id); break;
		case ESimOp::DestroyBody:				bi.DestroyBody(id); return 0;

		case ESimOp::AddBodies:
			{
				Array<BodyID> ids = GetHandles(inOp);
				BodyInterface::AddState state = bi.AddBodiesPrepare(ids.data(), int(ids.size()));
				bi.AddBodiesFinalize(ids.data(), int(ids.size()), state, activation);
				return 0;
			}

		case ESimOp::RemoveBodies:
			{
				Array<BodyID> ids = GetHandles(inOp);
				bi.RemoveBodies(ids.data(), int(ids.size()));
				return 0;
			}

		case ESimOp::ActivateBody:				bi.ActivateBody(id); break;
		case ESimOp::DeactivateBody:			bi.DeactivateBody(id); break;

		case ESimOp::ActivateBodies:
		case ESimOp::DeactivateBodies:
			{
				Array<BodyID> ids = GetHandles(inOp);
				if (ESimOp(inOp.mKind) == ESimOp::ActivateBodies)
					bi.ActivateBodies(ids.data(), int(ids.size()));
				else
					bi.DeactivateBodies(ids.data(), int(ids.size()));
				return 0;
			}

		case ESimOp::AddForce:					bi.AddForce(id, SimLoad3(f), activation); break;
		case ESimOp::AddForceAtPosition:		bi.AddForce(id, SimLoad3(f), r, activation); break;
		case ESimOp::AddTorque:					bi.AddTorque(id, SimLoad3(f), activation); break;
		case ESimOp::AddForceAndTorque:			bi.AddForceAndTorque(id, SimLoad3(f), SimLoad3(f + 3), activation); break;
		case ESimOp::AddImpulse:				bi.AddImpulse(id, SimLoad3(f)); break;
		case ESimOp::AddImpulseAtPosition:		bi.AddImpulse(id, SimLoad3(f), r); break;
		case ESimOp::AddAngularImpulse:			bi.AddAngularImpulse(id, SimLoad3(f)); break;
		case ESimOp::SetLinearVelocity:			bi.SetLinearVelocity(id, SimLoad3(f)); break;
		case ESimOp::SetAngularVelocity:		bi.SetAngularVelocity(id, SimLoad3(f)); break;
		case ESimOp::SetLinearAndAngularVelocity: bi.SetLinearAndAngularVelocity(id, SimLoad3(f), SimLoad3(f + 3)); break;
		case ESimOp::AddLinearVelocity:			bi.AddLinearVelocity(id, SimLoad3(f)); break;
		case ESimOp::AddLinearAndAngularVelocity: bi.AddLinearAndAngularVelocity(id, SimLoad3(f), SimLoad3(f + 3)); break;
		case ESimOp::SetPosition:				bi.SetPosition(id, r, activation); break;
		case ESimOp::SetRotation:				bi.SetRotation(id, SimLoadQuat(f), activation); break;
		case ESimOp::SetPositionAndRotation:	bi.SetPositionAndRotation(id, r, SimLoadQuat(f), activation); break;
		case ESimOp::SetPositionAndRotationWhenChanged: bi.SetPositionAndRotationWhenChanged(id, r, SimLoadQuat(f), activation); break;
		case ESimOp::SetPositionRotationAndVelocity: bi.SetPositionRotationAndVelocity(id, r, SimLoadQuat(f), SimLoad3(f + 4), SimLoad3(f + 7)); break;
		case ESimOp::MoveKinematic:				bi.MoveKinematic(id, r, SimLoadQuat(f), f[4]); break;
		case ESimOp::SetMotionType:				bi.SetMotionType(id, EMotionType(inOp.mIndex), activation); break;
		case ESimOp::SetMotionQuality:			bi.SetMotionQuality(id, EMotionQuality(inOp.mIndex)); break;
		case ESimOp::SetGravityFactor:			bi.SetGravityFactor(id, f[0]); break;
		case ESimOp::SetMaxLinearVelocity:		bi.SetMaxLinearVelocity(id, f[0]); break;
		case ESimOp::SetMaxAngularVelocity:		bi.SetMaxAngularVelocity(id, f[0]); break;
		case ESimOp::ResetSleepTimer:			bi.ResetSleepTimer(id); break;
		case ESimOp::SetObjectLayer:			bi.SetObjectLayer(id, ObjectLayer(inOp.mIndex)); break;
		case ESimOp::SetUserData:				bi.SetUserData(id, uint64(inOp.mIndex) | (uint64(inOp.mCount) << 32)); break;

		case ESimOp::SetFrictionAndRestitution:
			bi.SetFriction(id, f[0]);
			bi.SetRestitution(id, f[1]);
			break;

		case ESimOp::SetGravity:				mSystem.SetGravity(SimLoad3(f)); return 0;
		case ESimOp::SetPhysicsSettings:		mSystem.SetPhysicsSettings(SimToPhysicsSettings(mDesc.mPhysicsSettings[inOp.mIndex])); return 0;

		case ESimOp::AddStepListener:
			if (mStepListenerAdded.size() <= inOp.mIndex)
				mStepListenerAdded.resize(inOp.mIndex + 1, false);
			mSystem.AddStepListener(mStepListeners[inOp.mIndex]);
			mStepListenerAdded[inOp.mIndex] = true;
			return 0;

		case ESimOp::RemoveStepListener:
			mSystem.RemoveStepListener(mStepListeners[inOp.mIndex]);
			mStepListenerAdded[inOp.mIndex] = false;
			return 0;

		case ESimOp::OptimizeBroadPhase:		mSystem.OptimizeBroadPhase(); return 0;
		case ESimOp::SetShape:					bi.SetShape(id, mShapes[inOp.mIndex], inOp.mCount != 0, activation); break;

		case ESimOp::ActivateBodiesInAABox:
			bi.ActivateBodiesInAABox(AABox(SimLoad3(f), SimLoad3(f + 3)), { }, { });
			return 0;

		case ESimOp::ApplyBuoyancyImpulse:
			{
				bool applied = bi.ApplyBuoyancyImpulse(id, r, SimLoad3(f), f[3], f[4], f[5], SimLoad3(f + 6), SimLoad3(f + 9), f[12]);
				return (applied? 4 : 0) | (bi.IsActive(id)? 1 : 0) | (bi.IsAdded(id)? 2 : 0);
			}

		case ESimOp::ExpectBodyState:			break; // Changes nothing, the driver checks the result
		case ESimOp::SetUseManifoldReduction:	bi.SetUseManifoldReduction(id, inOp.mCount != 0); break;

		case ESimOp::ApplyBodyCreationSettings:
			{
				BodyCreationSettings settings = MakeBodyCreationSettings(mDesc.mBodies[inOp.mIndex]);
				BodyLockWrite lock(mSystem.GetBodyLockInterface(), id);
				JPH_ASSERT(lock.Succeeded());
				lock.GetBody().ApplyBodyCreationSettings(settings, mSystem.GetBroadPhaseLayerInterface());
				break;
			}

		case ESimOp::CreateConstraint:
			{
				const SimConstraint &c = mDesc.mConstraints[inOp.mIndex];
				Ref<TwoBodyConstraintSettings> settings = MakeConstraintSettings(c);
				TwoBodyConstraint *constraint = bi.CreateConstraint(settings, GetHandle(c.mBody1), GetHandle(c.mBody2));
				mSystem.AddConstraint(constraint);
				return 0;
			}

		default:
			JPH_ASSERT(false);
			abort();
		}

		// Body operations return IsActive and IsAdded of the body afterwards
		return (bi.IsActive(id)? 1 : 0) | (bi.IsAdded(id)? 2 : 0);
	}

	Array<BodyID>				GetHandles(const SimOp &inOp) const
	{
		Array<BodyID> ids;
		for (uint32 i = 0; i < inOp.mCount; ++i)
			ids.push_back(GetHandle(inOp.mBody + i));
		return ids;
	}

	const SimSceneDesc &		mDesc;
	BPLayerInterfaceImpl		mBroadPhaseLayerInterface;
	ObjectVsBroadPhaseLayerFilterImpl mObjectVsBroadPhaseLayerFilter;
	ObjectLayerPairFilterImpl	mObjectLayerPairFilter;
	TempAllocatorImpl			mTempAllocator { 10 * 1024 * 1024 };
	JobSystemThreadPool			mJobSystem;
	SimRecorder					mRecorder;
	SimActivationListener		mActivationListener;
	SimContactListener			mContactListener;
	SimSphereFilter				mSphereFilter;
	PhysicsSystem				mSystem;
	Array<RefConst<Shape>>		mShapes;
	Array<BodyID>				mHandles;
	Array<SimStepListenerImpl *> mStepListeners;
	Array<bool>					mStepListenerAdded;
};

void SimStepListenerImpl::OnStep(const PhysicsStepListenerContext &inContext)
{
	SimRecord r = { };
	r.mKind = uint32(ESimRecordKind::Step);
	r.mIDs[0] = mIndex;
	r.mIDs[1] = mNumCalls++;
	r.mData[0] = SimFloatBits(inContext.mDeltaTime);
	r.mData[1] = inContext.mIsFirstStep? 1 : 0;
	r.mData[2] = inContext.mIsLastStep? 1 : 0;
	mWorld->mRecorder.Add(r);

	// Change the velocity of the body (StepListenerMode in SimScenes.zig), all bodies are locked: the no-lock interface
	const SimStepListener &desc = mWorld->mDesc.mStepListeners[mIndex];
	if (desc.mBody == cInvalidHandle)
		return;
	BodyID id = mWorld->GetHandle(desc.mBody);
	BodyInterface &bi = inContext.mPhysicsSystem->GetBodyInterfaceNoLock();
	float dt = inContext.mDeltaTime;
	Vec3 delta = Vec3(desc.mF[1], desc.mF[2], desc.mF[3]) * dt;
	switch (desc.mMode)
	{
	case 0:		bi.SetLinearVelocity(id, bi.GetLinearVelocity(id) * desc.mF[0] + delta); break;
	case 1:		bi.SetAngularVelocity(id, bi.GetAngularVelocity(id) * desc.mF[0] + delta); break;
	case 2:		bi.AddLinearVelocity(id, delta); break;
	default:	break;
	}
}

extern "C"
{

// Sizes of the structs shared with SimScenes.zig
void jolt_sim_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(SimShape);
	outSizes[1] = sizeof(SimCompoundChild);
	outSizes[2] = sizeof(SimBody);
	outSizes[3] = sizeof(SimPhysicsSettings);
	outSizes[4] = sizeof(SimStepListener);
	outSizes[5] = sizeof(SimConstraint);
	outSizes[6] = sizeof(SimOp);
	outSizes[7] = sizeof(SimSceneDesc);
	outSizes[8] = sizeof(SimRecord);
	outSizes[9] = offsetof(SimBody, mShape);
	outSizes[10] = offsetof(SimSceneDesc, mShapes);
	outSizes[11] = offsetof(SimOp, mR);
}

// Jolt's default PhysicsSettings
void jolt_sim_default_settings(SimPhysicsSettings *outSettings)
{
	SimFromPhysicsSettings(PhysicsSettings(), *outSettings);
}

// Create the physics system of a scene (the bodies are created by the operations), `inDesc` must outlive the world
void *jolt_sim_create(const SimSceneDesc *inDesc, uint32 inNumThreads)
{
	SimEnsureFactory();
	return new SimWorld(*inDesc, inNumThreads);
}

void jolt_sim_destroy(void *inWorld)
{
	delete static_cast<SimWorld *>(inWorld);
}

uint32 jolt_sim_apply_op(void *inWorld, const SimOp *inOp)
{
	return static_cast<SimWorld *>(inWorld)->ApplyOp(*inOp);
}

// PhysicsSystem::Update, returns the EPhysicsUpdateError bits
uint32 jolt_sim_update(void *inWorld, float inDeltaTime, uint32 inCollisionSteps)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	EPhysicsUpdateError errors = world->mSystem.Update(inDeltaTime, int(inCollisionSteps), &world->mTempAllocator, &world->mJobSystem);
	return uint32(errors);
}

// PhysicsSystem::SaveState(EStateRecorderState(inState)), returns the size (the bytes are copied when they fit)
uint32 jolt_sim_save_state(void *inWorld, uint32 inState, uint8 *outBytes, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	StateRecorderImpl recorder;
	world->mSystem.SaveState(recorder, EStateRecorderState(inState));
	std::string data = recorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// PhysicsSystem::RestoreState (not validating), returns its result
uint32 jolt_sim_restore_state(void *inWorld, const uint8 *inBytes, uint32 inSize)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	StateRecorderImpl recorder;
	recorder.WriteBytes(inBytes, inSize);
	recorder.Rewind();
	return world->mSystem.RestoreState(recorder)? 1 : 0;
}

// Returns the number of records; when they fit in inCapacity they are copied and removed
uint32 jolt_sim_take_records(void *inWorld, SimRecord *outRecords, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	UniqueLock lock(world->mRecorder.mMutex);
	uint32 n = uint32(world->mRecorder.mRecords.size());
	if (n <= inCapacity)
	{
		if (n > 0)
			memcpy(outRecords, world->mRecorder.mRecords.data(), n * sizeof(SimRecord));
		world->mRecorder.mRecords.clear();
	}
	return n;
}

// For the state decoder: 1 when the body has motion properties (its SaveState writes them)
uint32 jolt_sim_body_has_motion_properties(void *inWorld, uint32 inBodyID)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	BodyLockRead lock(world->mSystem.GetBodyLockInterfaceNoLock(), BodyID(inBodyID));
	return lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked() != nullptr? 1 : 0;
}

// For the state decoder: the number of constraints and the size of the SaveState of constraint inIndex
uint32 jolt_sim_constraint_state_size(void *inWorld, uint32 inIndex, uint32 *outNumConstraints)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	Constraints constraints = world->mSystem.GetConstraints();
	*outNumConstraints = uint32(constraints.size());
	if (inIndex >= constraints.size())
		return 0;
	StateRecorderImpl recorder;
	constraints[inIndex]->SaveState(recorder);
	return uint32(recorder.GetData().size());
}

} // extern "C"
