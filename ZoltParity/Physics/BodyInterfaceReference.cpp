// Reference implementation of the BodyInterface / BodyCreationSettings binary state / PhysicsScene parity tests
// (`zig build parity`, Phase 5 Waves A6 and A8). ZoltParity/Physics/BodyInterfaceParity.zig calls these C functions and
// checks that Zolt produces the same bits.
//
// - A "world" is a BodyManager, a BroadPhaseBruteForce, a BodyLockInterfaceLocking and a BodyInterface on top of them
//   (BodyInterface::Init is public), with the layers of UnitTests/Layers.h. The Zolt side is a PhysicsSystem with the
//   brute force broad phase, whose locking BodyInterface works on the same kind of objects, so even the order in which
//   ActivateBodiesInAABox collects the bodies is the same on both sides. Operations are applied through the
//   BodyInterface one by one (jolt_bi_op), and after every operation jolt_bi_snapshot writes the state of every body
//   (its GetBodyCreationSettings binary state, user data, shape, flags, bounds and center of mass), the active body
//   list in its raw order and the BodyManager's SaveState bytes.
// - The shapes, materials and group filters of a test come from a table that both sides build from the same
//   descriptions (BITables), so that the bodies share them and a shape is identified by its index in the table.
// - jolt_bi_bcs_save / jolt_bi_bcs_resave: SaveWithChildren of a list of BodyCreationSettings with shared maps, and
//   sRestoreWithChildren of a stream followed by a new save (the cross check: each side restores the other side's
//   bytes). jolt_bi_scene_save / jolt_bi_scene_resave do the same for a PhysicsScene, jolt_bi_scene_create_bodies
//   restores a scene, fixes its invalid scales (FixInvalidScales) and instantiates it in a PhysicsSystem (CreateBodies).
//   The scenes have constraints (Phase 6: the kinds whose Zolt class is ported); each is two body indices and a
//   description of the constraint parity (CSSettings of ConstraintsReference.cpp, created by its CSMakeConstraintSettings
//   for every kind). CreateBodies creates them, and PhysicsScene::FromPhysicsSystem converts the system back.
//
// The structs below must match the extern structs of BodyInterfaceParity.zig (checked by jolt_bi_struct_sizes).
// Booleans are passed as uint32 (never bool, see the porting guide). All C functions have the prefix jolt_bi_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/QuickSort.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Body/BodyLockInterface.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

// The layers of the unit tests, mirrored by ZoltTests/Layers.zig
#include <UnitTests/Layers.h>

static void BIEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static constexpr uint32 cBINone = 0xffffffff;
static constexpr uint32 cBIDefaultMaterial = 0xfffffffe;

static Vec3 BILoad3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 BILoad4(const float *inV)				{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Quat BILoadQuat(const float *inV)			{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 BILoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static void BIStore3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void BIStore4(Vec4Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); outV[3] = inV.GetW(); }
static void BIStoreQuat(QuatArg inQ, float *outV)	{ outV[0] = inQ.GetX(); outV[1] = inQ.GetY(); outV[2] = inQ.GetZ(); outV[3] = inQ.GetW(); }
static void BIStoreR3(RVec3Arg inV, Real *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Mat44 BILoadMat44(const float *inM)			{ return Mat44(BILoad4(inM), BILoad4(inM + 4), BILoad4(inM + 8), BILoad4(inM + 12)); }

// Must match BIShapeDesc in BodyInterfaceParity.zig
struct BIShapeDesc
{
	uint32				mKind;				///< 0 sphere, 1 box, 2 capsule, 3 offset COM box, 4 rotated translated box, 5 scaled box, 6 static compound of a box and a sphere, 7 scaled sphere
	uint32				mMaterial;			///< Index in the material table or cBINone
	float				mParams[8];			///< 0..2 half extent / radius / half height, 3 convex radius, 4..6 offset / translation / scale
	float				mRotation[4];
};

// Must match BIMaterialDesc in BodyInterfaceParity.zig
struct BIMaterialDesc
{
	char				mName[16];			///< Zero terminated
	uint32				mColor;
};

// Must match BIGroupFilterDesc in BodyInterfaceParity.zig
struct BIGroupFilterDesc
{
	uint32				mNumSubGroups;
	uint32				mNumDisabled;
	uint32				mDisabled[16];		///< Pairs of sub group IDs
};

// Must match BITables in BodyInterfaceParity.zig
struct BITables
{
	const BIShapeDesc *			mShapes;
	uint32						mNumShapes;
	const BIMaterialDesc *		mMaterials;
	uint32						mNumMaterials;
	const BIGroupFilterDesc *	mFilters;
	uint32						mNumFilters;
};

// Must match BIBodyDesc in BodyInterfaceParity.zig
struct BIBodyDesc
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
	uint32				mShape;				///< Index in the shape table or cBINone (no shape)
	uint32				mUseShapeSettings;	///< 1: new ShapeSettings from the description of the shape (GetShape creates the shape)
	uint32				mGroupFilter;		///< Index in the group filter table or cBINone
	uint32				mGroupID;
	uint32				mSubGroupID;
};

// The settings of the description inSettings[inIndex] of the constraint parity (an array of CSSettings, see
// ConstraintsReference.cpp, which defines this function): every kind of its CSMakeSettings, any other kind aborts
Ref<TwoBodyConstraintSettings> CSMakeConstraintSettings(const void *inSettings, uint32 inIndex);

// Must match BIOp in BodyInterfaceParity.zig
struct BIOp
{
	Real				mR[6];
	float				mF[24];
	uint32				mKind;
	uint32				mBody;
	uint32				mI[16];
};

// Must match BIOut in BodyInterfaceParity.zig
struct BIOut
{
	Real				mR[6];
	float				mF[48];
	uint32				mI[24];
};

// The shapes, materials and group filters of a test
class BITableObjects
{
public:
	void						Init(const BITables &inTables)
	{
		for (uint32 i = 0; i < inTables.mNumMaterials; ++i)
		{
			const BIMaterialDesc &m = inTables.mMaterials[i];
			mMaterials.push_back(new PhysicsMaterialSimple(m.mName, Color(m.mColor)));
		}
		for (uint32 i = 0; i < inTables.mNumFilters; ++i)
		{
			const BIGroupFilterDesc &f = inTables.mFilters[i];
			GroupFilterTable *filter = new GroupFilterTable(f.mNumSubGroups);
			for (uint32 j = 0; j < f.mNumDisabled; ++j)
				filter->DisableCollision(f.mDisabled[2 * j], f.mDisabled[2 * j + 1]);
			mFilters.push_back(filter);
		}
		mShapeDescs = inTables.mShapes;
		for (uint32 i = 0; i < inTables.mNumShapes; ++i)
			mShapes.push_back(MakeShapeSettings(inTables.mShapes[i])->Create().Get());
	}

	const PhysicsMaterial *		GetMaterial(uint32 inIndex) const
	{
		return inIndex < mMaterials.size()? mMaterials[inIndex].GetPtr() : nullptr;
	}

	Ref<ShapeSettings>			MakeBoxSettings(const BIShapeDesc &inDesc) const
	{
		return new BoxShapeSettings(BILoad3(inDesc.mParams), inDesc.mParams[3], GetMaterial(inDesc.mMaterial));
	}

	// MakeShapeSettings of BodyInterfaceParity.zig
	Ref<ShapeSettings>			MakeShapeSettings(const BIShapeDesc &inDesc) const
	{
		const float *p = inDesc.mParams;
		const PhysicsMaterial *material = GetMaterial(inDesc.mMaterial);
		switch (inDesc.mKind)
		{
		case 0:		return new SphereShapeSettings(p[0], material);
		case 1:		return MakeBoxSettings(inDesc);
		case 2:		return new CapsuleShapeSettings(p[0], p[1], material);
		case 3:		return new OffsetCenterOfMassShapeSettings(BILoad3(p + 4), MakeBoxSettings(inDesc));
		case 4:		return new RotatedTranslatedShapeSettings(BILoad3(p + 4), BILoadQuat(inDesc.mRotation), MakeBoxSettings(inDesc));
		case 5:		return new ScaledShapeSettings(MakeBoxSettings(inDesc), BILoad3(p + 4));
		case 6:
			{
				StaticCompoundShapeSettings *compound = new StaticCompoundShapeSettings;
				compound->AddShape(BILoad3(p + 4), BILoadQuat(inDesc.mRotation), MakeBoxSettings(inDesc));
				compound->AddShape(-BILoad3(p + 4), Quat::sIdentity(), new SphereShapeSettings(p[0], material));
				return compound;
			}
		default:	return new ScaledShapeSettings(new SphereShapeSettings(p[0], material), BILoad3(p + 4));
		}
	}

	// The index of a shape in the table, cBINone when it is not in the table
	uint32						ShapeIndex(const Shape *inShape) const
	{
		for (uint32 i = 0; i < uint32(mShapes.size()); ++i)
			if (mShapes[i] == inShape)
				return i;
		return cBINone;
	}

	uint32						MaterialIndex(const PhysicsMaterial *inMaterial) const
	{
		if (inMaterial == PhysicsMaterial::sDefault)
			return cBIDefaultMaterial;
		for (uint32 i = 0; i < uint32(mMaterials.size()); ++i)
			if (mMaterials[i] == inMaterial)
				return i;
		return cBINone;
	}

	uint32						FilterIndex(const GroupFilter *inFilter) const
	{
		for (uint32 i = 0; i < uint32(mFilters.size()); ++i)
			if (mFilters[i] == inFilter)
				return i;
		return cBINone;
	}

	// MakeBodyCreationSettings of BodyInterfaceParity.zig
	BodyCreationSettings		MakeBodyCreationSettings(const BIBodyDesc &inDesc) const
	{
		BodyCreationSettings s;
		s.mPosition = BILoadR3(inDesc.mPosition);
		s.mRotation = BILoadQuat(inDesc.mRotation);
		s.mLinearVelocity = BILoad3(inDesc.mLinearVelocity);
		s.mAngularVelocity = BILoad3(inDesc.mAngularVelocity);
		s.mUserData = inDesc.mUserData;
		s.mObjectLayer = ObjectLayer(inDesc.mObjectLayer);
		s.mCollisionGroup = CollisionGroup(inDesc.mGroupFilter < mFilters.size()? mFilters[inDesc.mGroupFilter].GetPtr() : nullptr, inDesc.mGroupID, inDesc.mSubGroupID);
		s.mMotionType = EMotionType(inDesc.mMotionType);
		s.mAllowedDOFs = EAllowedDOFs(inDesc.mAllowedDOFs);
		s.mAllowDynamicOrKinematic = inDesc.mAllowDynamicOrKinematic != 0;
		s.mIsSensor = inDesc.mIsSensor != 0;
		s.mCollideKinematicVsNonDynamic = inDesc.mCollideKinematicVsNonDynamic != 0;
		s.mUseManifoldReduction = inDesc.mUseManifoldReduction != 0;
		s.mApplyGyroscopicForce = inDesc.mApplyGyroscopicForce != 0;
		s.mMotionQuality = EMotionQuality(inDesc.mMotionQuality);
		s.mEnhancedInternalEdgeRemoval = inDesc.mEnhancedInternalEdgeRemoval != 0;
		s.mAllowSleeping = inDesc.mAllowSleeping != 0;
		s.mFriction = inDesc.mFriction;
		s.mRestitution = inDesc.mRestitution;
		s.mLinearDamping = inDesc.mLinearDamping;
		s.mAngularDamping = inDesc.mAngularDamping;
		s.mMaxLinearVelocity = inDesc.mMaxLinearVelocity;
		s.mMaxAngularVelocity = inDesc.mMaxAngularVelocity;
		s.mGravityFactor = inDesc.mGravityFactor;
		s.mNumVelocityStepsOverride = inDesc.mNumVelocityStepsOverride;
		s.mNumPositionStepsOverride = inDesc.mNumPositionStepsOverride;
		s.mOverrideMassProperties = EOverrideMassProperties(inDesc.mOverrideMassProperties);
		s.mInertiaMultiplier = inDesc.mInertiaMultiplier;
		s.mMassPropertiesOverride.mMass = inDesc.mMass;
		s.mMassPropertiesOverride.mInertia = BILoadMat44(inDesc.mInertia);
		if (inDesc.mShape < mShapes.size())
		{
			if (inDesc.mUseShapeSettings != 0)
				s.SetShapeSettings(MakeShapeSettings(mShapeDescs[inDesc.mShape]));
			else
				s.SetShape(mShapes[inDesc.mShape]);
		}
		return s;
	}

	Array<RefConst<PhysicsMaterial>>	mMaterials;
	Array<Ref<GroupFilterTable>>		mFilters;
	Array<RefConst<Shape>>				mShapes;
	const BIShapeDesc *					mShapeDescs = nullptr;
};

// The state of the bodies (see the file header): written with a StateRecorderImpl, compared byte by byte. The SaveState
// bytes follow (written by the caller).
static void BIWriteBodies(const Array<const Body *> &inBodies, const Array<uint32> &inActive, const BITableObjects &inTables, StateRecorderImpl &ioRecorder)
{
	ioRecorder.Write(uint32(inBodies.size()));
	for (const Body *b : inBodies)
	{
		const Body &body = *b;
		ioRecorder.Write(body.GetID().GetIndexAndSequenceNumber());
		body.GetBodyCreationSettings().SaveBinaryState(ioRecorder);
		ioRecorder.Write(body.GetUserData());
		ioRecorder.Write(inTables.ShapeIndex(body.GetShape()));
		ioRecorder.Write(uint8(body.IsActive()? 1 : 0));
		ioRecorder.Write(uint8(body.IsInBroadPhase()? 1 : 0));
		ioRecorder.Write(body.GetWorldSpaceBounds().mMin);
		ioRecorder.Write(body.GetWorldSpaceBounds().mMax);
		ioRecorder.Write(body.GetCenterOfMassPosition());
		ioRecorder.Write(body.GetBroadPhaseLayer().GetValue());
	}

	// The active bodies
	ioRecorder.Write(uint32(inActive.size()));
	for (uint32 a : inActive)
		ioRecorder.Write(a);
}

// The snapshot of a BodyManager: the bodies in ID order, the active bodies in the order of the active body list and
// BodyManager::SaveState
static void BIWriteSnapshot(const BodyManager &inBodyManager, const BITableObjects &inTables, StateRecorderImpl &ioRecorder)
{
	BodyIDVector ids;
	inBodyManager.GetBodyIDs(ids);
	Array<const Body *> bodies;
	for (const BodyID &id : ids)
		bodies.push_back(&inBodyManager.GetBody(id));
	Array<uint32> active;
	uint32 num_active = inBodyManager.GetNumActiveBodies(EBodyType::RigidBody);
	for (uint32 i = 0; i < num_active; ++i)
		active.push_back(inBodyManager.GetActiveBodiesUnsafe(EBodyType::RigidBody)[i].GetIndexAndSequenceNumber());
	BIWriteBodies(bodies, active, inTables, ioRecorder);
	inBodyManager.SaveState(ioRecorder, nullptr);
}

static uint32 BICopyBytes(const std::string &inData, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inData.size());
	if (size <= inCapacity)
		memcpy(outBytes, inData.data(), size);
	return size;
}

static void BICopyError(const String &inError, char *outError, uint32 inCapacity)
{
	size_t n = std::min<size_t>(inError.size(), inCapacity - 1);
	memcpy(outError, inError.data(), n);
	outError[n] = 0;
}

// A BodyManager, a BroadPhaseBruteForce and a BodyInterface on top of them
class BIWorld
{
public:
								BIWorld(uint32 inMaxBodies, uint32 inNumBodyMutexes, const BITables &inTables) :
		mLockInterface(mBodyManager)
	{
		mTables.Init(inTables);
		mBodyManager.Init(inMaxBodies, inNumBodyMutexes, mLayers);
		mBroadPhase = new BroadPhaseBruteForce;
		mBroadPhase->Init(&mBodyManager, mLayers);
		mBodyInterface.Init(mLockInterface, mBodyManager, *mBroadPhase);
	}

								~BIWorld()
	{
		// Remove and destroy every body (like PhysicsSystem's destructor through the BodyManager)
		BodyIDVector ids;
		mBodyManager.GetBodyIDs(ids);
		for (const BodyID &id : ids)
			mBodyInterface.RemoveBody(id);
		mBodyInterface.DestroyBodies(ids.data(), int(ids.size()));
		for (Body *body : mUnassigned)
			mBodyInterface.DestroyBodyWithoutID(body);
		delete mBroadPhase;
	}

	BPLayerInterfaceImpl		mLayers;
	BodyManager					mBodyManager;
	BroadPhaseBruteForce *		mBroadPhase = nullptr;
	BodyLockInterfaceLocking	mLockInterface;
	BodyInterface				mBodyInterface;
	BITableObjects				mTables;
	Array<Body *>				mUnassigned;
};

// Must match OpKind in BodyInterfaceParity.zig
enum class EBIOp : uint32
{
	SetPositionAndRotation = 0,
	SetPositionAndRotationWhenChanged = 1,
	GetPositionAndRotation = 2,
	SetPosition = 3,
	GetPosition = 4,
	GetCenterOfMassPosition = 5,
	SetRotation = 6,
	GetRotation = 7,
	GetWorldTransform = 8,
	GetCenterOfMassTransform = 9,
	MoveKinematic = 10,
	SetLinearAndAngularVelocity = 11,
	GetLinearAndAngularVelocity = 12,
	SetLinearVelocity = 13,
	GetLinearVelocity = 14,
	AddLinearVelocity = 15,
	AddLinearAndAngularVelocity = 16,
	SetAngularVelocity = 17,
	GetAngularVelocity = 18,
	GetPointVelocity = 19,
	SetPositionRotationAndVelocity = 20,
	AddForce = 21,
	AddForceAtPosition = 22,
	AddTorque = 23,
	AddForceAndTorque = 24,
	AddImpulse = 25,
	AddImpulseAtPosition = 26,
	AddAngularImpulse = 27,
	ApplyBuoyancyImpulse = 28,
	GetBodyType = 29,
	SetMotionType = 30,
	GetMotionType = 31,
	SetMotionQuality = 32,
	GetMotionQuality = 33,
	GetInverseInertia = 34,
	SetRestitution = 35,
	GetRestitution = 36,
	SetFriction = 37,
	GetFriction = 38,
	SetGravityFactor = 39,
	GetGravityFactor = 40,
	SetMaxLinearVelocity = 41,
	GetMaxLinearVelocity = 42,
	SetMaxAngularVelocity = 43,
	GetMaxAngularVelocity = 44,
	SetUseManifoldReduction = 45,
	GetUseManifoldReduction = 46,
	SetIsSensor = 47,
	IsSensor = 48,
	SetCollisionGroup = 49,
	GetCollisionGroup = 50,
	GetTransformedShape = 51,
	SetUserData = 52,
	GetUserData = 53,
	GetMaterial = 54,
	SetShape = 55,
	NotifyShapeChanged = 56,
	SetObjectLayer = 57,
	GetObjectLayer = 58,
	ActivateBody = 59,
	ActivateBodies = 60,
	ActivateBodiesInAABox = 61,
	DeactivateBody = 62,
	DeactivateBodies = 63,
	IsActive = 64,
	ResetSleepTimer = 65,
	AddBody = 66,
	RemoveBody = 67,
	IsAdded = 68,
	DestroyBody = 69,
	AddBodies = 70,
	AddBodiesAbort = 71,
	RemoveBodies = 72,
	InvalidateContactCache = 73,
	GetShape = 74,
	UnassignAndReassign = 75,
	UnassignAndDestroy = 76,
	DestroyBodies = 77,
};

extern "C"
{

// Sizes of the structs, checked against the Zig side
void jolt_bi_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(BIShapeDesc);
	outSizes[1] = sizeof(BIMaterialDesc);
	outSizes[2] = sizeof(BIGroupFilterDesc);
	outSizes[3] = sizeof(BITables);
	outSizes[4] = sizeof(BIBodyDesc);
	outSizes[5] = sizeof(BIOp);
	outSizes[6] = sizeof(BIOut);
}

void *jolt_bi_world_create(uint32 inMaxBodies, uint32 inNumBodyMutexes, const BITables *inTables)
{
	BIEnsureFactory();
	return new BIWorld(inMaxBodies, inNumBodyMutexes, *inTables);
}

void jolt_bi_world_destroy(void *inWorld)
{
	delete static_cast<BIWorld *>(inWorld);
}

// Create a body through the BodyInterface. inMode: 0 CreateBody + AddBody, 1 CreateAndAddBody, 2 CreateBody (not
// added), 3 CreateBodyWithID(inCustomID) + AddBody, 4 CreateBodyWithoutID + AssignBodyID + AddBody, 5
// CreateBodyWithoutID + AssignBodyID(inCustomID) + AddBody (a body that gets no ID is destroyed with
// DestroyBodyWithoutID). inActivation: EActivation. Returns the ID (cInvalidBodyID when the body could not be created).
uint32 jolt_bi_create_body(void *inWorld, const BIBodyDesc *inDesc, uint32 inMode, uint32 inCustomID, uint32 inActivation)
{
	BIWorld &w = *static_cast<BIWorld *>(inWorld);
	BodyInterface &bi = w.mBodyInterface;
	BodyCreationSettings settings = w.mTables.MakeBodyCreationSettings(*inDesc);
	EActivation activation = EActivation(inActivation);
	switch (inMode)
	{
	case 0:
	case 2:
		{
			Body *body = bi.CreateBody(settings);
			if (body == nullptr)
				return BodyID::cInvalidBodyID;
			if (inMode == 0)
				bi.AddBody(body->GetID(), activation);
			return body->GetID().GetIndexAndSequenceNumber();
		}

	case 1:
		return bi.CreateAndAddBody(settings, activation).GetIndexAndSequenceNumber();

	case 3:
		{
			Body *body = bi.CreateBodyWithID(BodyID(inCustomID), settings);
			if (body == nullptr)
				return BodyID::cInvalidBodyID;
			bi.AddBody(body->GetID(), activation);
			return body->GetID().GetIndexAndSequenceNumber();
		}

	default:
		{
			Body *body = bi.CreateBodyWithoutID(settings);
			bool assigned = inMode == 4? bi.AssignBodyID(body) : bi.AssignBodyID(body, BodyID(inCustomID));
			if (!assigned)
			{
				bi.DestroyBodyWithoutID(body);
				return BodyID::cInvalidBodyID;
			}
			bi.AddBody(body->GetID(), activation);
			return body->GetID().GetIndexAndSequenceNumber();
		}
	}
}

// Apply one BodyInterface operation (see OpKind in BodyInterfaceParity.zig for the meaning of the fields)
void jolt_bi_op(void *inWorld, const BIOp *inOp, BIOut *outResult)
{
	BIWorld &w = *static_cast<BIWorld *>(inWorld);
	BodyInterface &bi = w.mBodyInterface;
	const BodyID id(inOp->mBody);
	const Real *r = inOp->mR;
	const float *f = inOp->mF;
	const uint32 *i = inOp->mI;
	Real *out_r = outResult->mR;
	float *out_f = outResult->mF;
	uint32 *out_i = outResult->mI;
	EActivation activation = EActivation(i[0]);

	// The body IDs of the list operations: i[1] is the count, i[2..] the IDs
	Array<BodyID> ids;
	for (uint32 k = 0; k < i[1] && k < 14; ++k)
		ids.push_back(BodyID(i[2 + k]));

	switch (EBIOp(inOp->mKind))
	{
	case EBIOp::SetPositionAndRotation:				bi.SetPositionAndRotation(id, BILoadR3(r), BILoadQuat(f), activation); break;
	case EBIOp::SetPositionAndRotationWhenChanged:	bi.SetPositionAndRotationWhenChanged(id, BILoadR3(r), BILoadQuat(f), activation); break;
	case EBIOp::GetPositionAndRotation:
		{
			RVec3 p;
			Quat q;
			bi.GetPositionAndRotation(id, p, q);
			BIStoreR3(p, out_r);
			BIStoreQuat(q, out_f);
		}
		break;
	case EBIOp::SetPosition:						bi.SetPosition(id, BILoadR3(r), activation); break;
	case EBIOp::GetPosition:						BIStoreR3(bi.GetPosition(id), out_r); break;
	case EBIOp::GetCenterOfMassPosition:			BIStoreR3(bi.GetCenterOfMassPosition(id), out_r); break;
	case EBIOp::SetRotation:						bi.SetRotation(id, BILoadQuat(f), activation); break;
	case EBIOp::GetRotation:						BIStoreQuat(bi.GetRotation(id), out_f); break;
	case EBIOp::GetWorldTransform:
	case EBIOp::GetCenterOfMassTransform:
		{
			RMat44 m = EBIOp(inOp->mKind) == EBIOp::GetWorldTransform? bi.GetWorldTransform(id) : bi.GetCenterOfMassTransform(id);
			for (int c = 0; c < 3; ++c)
				BIStore4(m.GetColumn4(c), out_f + 4 * c);
			BIStoreR3(m.GetTranslation(), out_r);
		}
		break;
	case EBIOp::MoveKinematic:						bi.MoveKinematic(id, BILoadR3(r), BILoadQuat(f), f[4]); break;
	case EBIOp::SetLinearAndAngularVelocity:		bi.SetLinearAndAngularVelocity(id, BILoad3(f), BILoad3(f + 3)); break;
	case EBIOp::GetLinearAndAngularVelocity:
		{
			Vec3 v, a;
			bi.GetLinearAndAngularVelocity(id, v, a);
			BIStore3(v, out_f);
			BIStore3(a, out_f + 3);
		}
		break;
	case EBIOp::SetLinearVelocity:					bi.SetLinearVelocity(id, BILoad3(f)); break;
	case EBIOp::GetLinearVelocity:					BIStore3(bi.GetLinearVelocity(id), out_f); break;
	case EBIOp::AddLinearVelocity:					bi.AddLinearVelocity(id, BILoad3(f)); break;
	case EBIOp::AddLinearAndAngularVelocity:		bi.AddLinearAndAngularVelocity(id, BILoad3(f), BILoad3(f + 3)); break;
	case EBIOp::SetAngularVelocity:					bi.SetAngularVelocity(id, BILoad3(f)); break;
	case EBIOp::GetAngularVelocity:					BIStore3(bi.GetAngularVelocity(id), out_f); break;
	case EBIOp::GetPointVelocity:					BIStore3(bi.GetPointVelocity(id, BILoadR3(r)), out_f); break;
	case EBIOp::SetPositionRotationAndVelocity:		bi.SetPositionRotationAndVelocity(id, BILoadR3(r), BILoadQuat(f), BILoad3(f + 4), BILoad3(f + 7)); break;
	case EBIOp::AddForce:							bi.AddForce(id, BILoad3(f), activation); break;
	case EBIOp::AddForceAtPosition:					bi.AddForce(id, BILoad3(f), BILoadR3(r), activation); break;
	case EBIOp::AddTorque:							bi.AddTorque(id, BILoad3(f), activation); break;
	case EBIOp::AddForceAndTorque:					bi.AddForceAndTorque(id, BILoad3(f), BILoad3(f + 3), activation); break;
	case EBIOp::AddImpulse:							bi.AddImpulse(id, BILoad3(f)); break;
	case EBIOp::AddImpulseAtPosition:				bi.AddImpulse(id, BILoad3(f), BILoadR3(r)); break;
	case EBIOp::AddAngularImpulse:					bi.AddAngularImpulse(id, BILoad3(f)); break;
	case EBIOp::ApplyBuoyancyImpulse:				out_i[0] = bi.ApplyBuoyancyImpulse(id, BILoadR3(r), BILoad3(f), f[3], f[4], f[5], BILoad3(f + 6), BILoad3(f + 9), f[12])? 1 : 0; break;
	case EBIOp::GetBodyType:						out_i[0] = uint32(bi.GetBodyType(id)); break;
	case EBIOp::SetMotionType:						bi.SetMotionType(id, EMotionType(i[2]), activation); break;
	case EBIOp::GetMotionType:						out_i[0] = uint32(bi.GetMotionType(id)); break;
	case EBIOp::SetMotionQuality:					bi.SetMotionQuality(id, EMotionQuality(i[2])); break;
	case EBIOp::GetMotionQuality:					out_i[0] = uint32(bi.GetMotionQuality(id)); break;
	case EBIOp::GetInverseInertia:
		{
			Mat44 m = bi.GetInverseInertia(id);
			for (int c = 0; c < 4; ++c)
				BIStore4(m.GetColumn4(c), out_f + 4 * c);
		}
		break;
	case EBIOp::SetRestitution:						bi.SetRestitution(id, f[0]); break;
	case EBIOp::GetRestitution:						out_f[0] = bi.GetRestitution(id); break;
	case EBIOp::SetFriction:						bi.SetFriction(id, f[0]); break;
	case EBIOp::GetFriction:						out_f[0] = bi.GetFriction(id); break;
	case EBIOp::SetGravityFactor:					bi.SetGravityFactor(id, f[0]); break;
	case EBIOp::GetGravityFactor:					out_f[0] = bi.GetGravityFactor(id); break;
	case EBIOp::SetMaxLinearVelocity:				bi.SetMaxLinearVelocity(id, f[0]); break;
	case EBIOp::GetMaxLinearVelocity:				out_f[0] = bi.GetMaxLinearVelocity(id); break;
	case EBIOp::SetMaxAngularVelocity:				bi.SetMaxAngularVelocity(id, f[0]); break;
	case EBIOp::GetMaxAngularVelocity:				out_f[0] = bi.GetMaxAngularVelocity(id); break;
	case EBIOp::SetUseManifoldReduction:			bi.SetUseManifoldReduction(id, i[2] != 0); break;
	case EBIOp::GetUseManifoldReduction:			out_i[0] = bi.GetUseManifoldReduction(id)? 1 : 0; break;
	case EBIOp::SetIsSensor:						bi.SetIsSensor(id, i[2] != 0); break;
	case EBIOp::IsSensor:							out_i[0] = bi.IsSensor(id)? 1 : 0; break;
	case EBIOp::SetCollisionGroup:
		bi.SetCollisionGroup(id, CollisionGroup(i[4] < w.mTables.mFilters.size()? w.mTables.mFilters[i[4]].GetPtr() : nullptr, i[2], i[3]));
		break;
	case EBIOp::GetCollisionGroup:
		{
			const CollisionGroup &group = bi.GetCollisionGroup(id);
			out_i[0] = group.GetGroupID();
			out_i[1] = group.GetSubGroupID();
			out_i[2] = w.mTables.FilterIndex(group.GetGroupFilter());
		}
		break;
	case EBIOp::GetTransformedShape:
		{
			TransformedShape ts = bi.GetTransformedShape(id);
			out_i[0] = ts.mShape != nullptr? 1 : 0;
			out_i[1] = w.mTables.ShapeIndex(ts.mShape);
			out_i[2] = ts.mBodyID.GetIndexAndSequenceNumber();
			out_i[3] = ts.mSubShapeIDCreator.GetID().GetValue();
			out_i[4] = ts.mSubShapeIDCreator.GetNumBitsWritten();
			BIStore3(ts.GetShapeScale(), out_f);
			if (ts.mShape != nullptr)
			{
				BIStoreR3(ts.mShapePositionCOM, out_r);
				BIStoreQuat(ts.mShapeRotation, out_f + 3);
				AABox bounds = ts.GetWorldSpaceBounds();
				BIStore3(bounds.mMin, out_f + 7);
				BIStore3(bounds.mMax, out_f + 10);
				RMat44 com = ts.GetCenterOfMassTransform();
				for (int c = 0; c < 3; ++c)
					BIStore4(com.GetColumn4(c), out_f + 13 + 4 * c);
				BIStoreR3(com.GetTranslation(), out_r + 3);
			}
		}
		break;
	case EBIOp::SetUserData:						bi.SetUserData(id, uint64(i[2]) | (uint64(i[3]) << 32)); break;
	case EBIOp::GetUserData:
		{
			uint64 user_data = bi.GetUserData(id);
			out_i[0] = uint32(user_data);
			out_i[1] = uint32(user_data >> 32);
		}
		break;
	case EBIOp::GetMaterial:
		{
			SubShapeID sub_shape_id;
			sub_shape_id.SetValue(i[2]);
			out_i[0] = w.mTables.MaterialIndex(bi.GetMaterial(id, sub_shape_id));
		}
		break;
	case EBIOp::SetShape:							bi.SetShape(id, w.mTables.mShapes[i[2]], i[3] != 0, activation); break;
	case EBIOp::NotifyShapeChanged:					bi.NotifyShapeChanged(id, BILoad3(f), i[3] != 0, activation); break;
	case EBIOp::SetObjectLayer:						bi.SetObjectLayer(id, ObjectLayer(i[2])); break;
	case EBIOp::GetObjectLayer:						out_i[0] = uint32(bi.GetObjectLayer(id)); break;
	case EBIOp::ActivateBody:						bi.ActivateBody(id); break;
	case EBIOp::ActivateBodies:						bi.ActivateBodies(ids.data(), int(ids.size())); break;
	case EBIOp::ActivateBodiesInAABox:				bi.ActivateBodiesInAABox(AABox(BILoad3(f), BILoad3(f + 3)), { }, { }); break;
	case EBIOp::DeactivateBody:						bi.DeactivateBody(id); break;
	case EBIOp::DeactivateBodies:					bi.DeactivateBodies(ids.data(), int(ids.size())); break;
	case EBIOp::IsActive:							out_i[0] = bi.IsActive(id)? 1 : 0; break;
	case EBIOp::ResetSleepTimer:					bi.ResetSleepTimer(id); break;
	case EBIOp::AddBody:							bi.AddBody(id, activation); break;
	case EBIOp::RemoveBody:							bi.RemoveBody(id); break;
	case EBIOp::IsAdded:							out_i[0] = bi.IsAdded(id)? 1 : 0; break;
	case EBIOp::DestroyBody:						bi.DestroyBody(id); break;
	case EBIOp::AddBodies:
	case EBIOp::AddBodiesAbort:
		{
			BodyInterface::AddState state = bi.AddBodiesPrepare(ids.data(), int(ids.size()));
			if (EBIOp(inOp->mKind) == EBIOp::AddBodies)
				bi.AddBodiesFinalize(ids.data(), int(ids.size()), state, activation);
			else
				bi.AddBodiesAbort(ids.data(), int(ids.size()), state);
			for (size_t k = 0; k < ids.size(); ++k)
				out_i[k] = ids[k].GetIndexAndSequenceNumber(); // The order after AddBodiesPrepare
		}
		break;
	case EBIOp::RemoveBodies:
		bi.RemoveBodies(ids.data(), int(ids.size()));
		for (size_t k = 0; k < ids.size(); ++k)
			out_i[k] = ids[k].GetIndexAndSequenceNumber();
		break;
	case EBIOp::InvalidateContactCache:				bi.InvalidateContactCache(id); break;
	case EBIOp::GetShape:							out_i[0] = w.mTables.ShapeIndex(bi.GetShape(id)); break;
	case EBIOp::UnassignAndReassign:
		{
			Body *body = bi.UnassignBodyID(id);
			bool assigned = i[2] == BodyID::cInvalidBodyID? bi.AssignBodyID(body) : bi.AssignBodyID(body, BodyID(i[2]));
			if (assigned)
				out_i[0] = body->GetID().GetIndexAndSequenceNumber();
			else
			{
				out_i[0] = BodyID::cInvalidBodyID;
				w.mUnassigned.push_back(body);
			}
		}
		break;
	case EBIOp::UnassignAndDestroy:
		{
			Body *bodies[14];
			bi.UnassignBodyIDs(ids.data(), int(ids.size()), bodies);
			for (size_t k = 0; k < ids.size(); ++k)
				bi.DestroyBodyWithoutID(bodies[k]);
		}
		break;
	case EBIOp::DestroyBodies:						bi.DestroyBodies(ids.data(), int(ids.size())); break;
	}
}

// Write the snapshot of the world (see BIWriteSnapshot), returns its size (the bytes are copied when they fit)
uint32 jolt_bi_snapshot(void *inWorld, uint8 *outBytes, uint32 inCapacity)
{
	BIWorld &w = *static_cast<BIWorld *>(inWorld);
	StateRecorderImpl recorder;
	BIWriteSnapshot(w.mBodyManager, w.mTables, recorder);
	return BICopyBytes(recorder.GetData(), outBytes, inCapacity);
}

// SaveWithChildren of the bodies with shared maps (inSaveShapes / inSaveGroupFilters select the maps), returns the size
uint32 jolt_bi_bcs_save(const BITables *inTables, const BIBodyDesc *inBodies, uint32 inNumBodies, uint32 inSaveShapes, uint32 inSaveGroupFilters, uint8 *outBytes, uint32 inCapacity)
{
	BIEnsureFactory();
	BITableObjects tables;
	tables.Init(*inTables);
	BodyCreationSettings::ShapeToIDMap shape_map;
	BodyCreationSettings::MaterialToIDMap material_map;
	BodyCreationSettings::GroupFilterToIDMap filter_map;
	// Keep all settings alive while saving: the maps are keyed by address, a shape that GetShape creates from shape
	// settings must not be freed (and its address reused) before the save is done
	Array<BodyCreationSettings> bodies;
	for (uint32 b = 0; b < inNumBodies; ++b)
		bodies.push_back(tables.MakeBodyCreationSettings(inBodies[b]));
	std::stringstream data;
	StreamOutWrapper out(data);
	for (const BodyCreationSettings &b : bodies)
		b.SaveWithChildren(out, inSaveShapes != 0? &shape_map : nullptr, inSaveShapes != 0? &material_map : nullptr, inSaveGroupFilters != 0? &filter_map : nullptr);
	return BICopyBytes(data.str(), outBytes, inCapacity);
}

// sRestoreWithChildren of inNumBodies settings from the bytes (shared maps), then SaveWithChildren of the restored
// settings with new maps. Returns the size of the new bytes, or cBINone and Jolt's error text when a restore failed.
uint32 jolt_bi_bcs_resave(const uint8 *inBytes, uint32 inSize, uint32 inNumBodies, uint32 inSaveShapes, uint32 inSaveGroupFilters, uint8 *outBytes, uint32 inCapacity, char *outError, uint32 inErrorCapacity)
{
	BIEnsureFactory();
	std::stringstream in_data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(in_data);
	BodyCreationSettings::IDToShapeMap id_to_shape;
	BodyCreationSettings::IDToMaterialMap id_to_material;
	BodyCreationSettings::IDToGroupFilterMap id_to_filter;
	Array<BodyCreationSettings> bodies;
	for (uint32 b = 0; b < inNumBodies; ++b)
	{
		BodyCreationSettings::BCSResult result = BodyCreationSettings::sRestoreWithChildren(in, id_to_shape, id_to_material, id_to_filter);
		if (result.HasError())
		{
			BICopyError(result.GetError(), outError, inErrorCapacity);
			return cBINone;
		}
		bodies.push_back(result.Get());
	}

	BodyCreationSettings::ShapeToIDMap shape_map;
	BodyCreationSettings::MaterialToIDMap material_map;
	BodyCreationSettings::GroupFilterToIDMap filter_map;
	std::stringstream data;
	StreamOutWrapper out(data);
	for (const BodyCreationSettings &b : bodies)
		b.SaveWithChildren(out, inSaveShapes != 0? &shape_map : nullptr, inSaveShapes != 0? &material_map : nullptr, inSaveGroupFilters != 0? &filter_map : nullptr);
	return BICopyBytes(data.str(), outBytes, inCapacity);
}

// PhysicsScene::SaveBinaryState of a scene with the bodies and the constraints (constraint c: the body indices
// inConstraintBodies[2 * c], inConstraintBodies[2 * c + 1] and the CSSettings description inConstraintSettings[c]),
// returns the size
uint32 jolt_bi_scene_save(const BITables *inTables, const BIBodyDesc *inBodies, uint32 inNumBodies, const uint32 *inConstraintBodies, const void *inConstraintSettings, uint32 inNumConstraints, uint32 inSaveShapes, uint32 inSaveGroupFilters, uint8 *outBytes, uint32 inCapacity)
{
	BIEnsureFactory();
	BITableObjects tables;
	tables.Init(*inTables);
	Ref<PhysicsScene> scene = new PhysicsScene;
	for (uint32 b = 0; b < inNumBodies; ++b)
		scene->AddBody(tables.MakeBodyCreationSettings(inBodies[b]));
	for (uint32 c = 0; c < inNumConstraints; ++c)
		scene->AddConstraint(CSMakeConstraintSettings(inConstraintSettings, c), inConstraintBodies[2 * c], inConstraintBodies[2 * c + 1]);
	std::stringstream data;
	StreamOutWrapper out(data);
	scene->SaveBinaryState(out, inSaveShapes != 0, inSaveGroupFilters != 0);
	return BICopyBytes(data.str(), outBytes, inCapacity);
}

// PhysicsScene::sRestoreFromBinaryState of the bytes, then SaveBinaryState of the restored scene. Returns the size of
// the new bytes, or cBINone and Jolt's error text when the restore failed.
uint32 jolt_bi_scene_resave(const uint8 *inBytes, uint32 inSize, uint32 inSaveShapes, uint32 inSaveGroupFilters, uint8 *outBytes, uint32 inCapacity, char *outError, uint32 inErrorCapacity)
{
	BIEnsureFactory();
	std::stringstream in_data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(in_data);
	PhysicsScene::PhysicsSceneResult result = PhysicsScene::sRestoreFromBinaryState(in);
	if (result.HasError())
	{
		BICopyError(result.GetError(), outError, inErrorCapacity);
		return cBINone;
	}
	std::stringstream data;
	StreamOutWrapper out(data);
	result.Get()->SaveBinaryState(out, inSaveShapes != 0, inSaveGroupFilters != 0);
	return BICopyBytes(data.str(), outBytes, inCapacity);
}

// Restore a scene from the bytes (saved with its shapes), fix its invalid scales (FixInvalidScales) and instantiate it
// with CreateBodies in a PhysicsSystem with the layers of the unit tests. Its QuadTree broad phase sorts the bodies by
// layer in AddBodiesPrepare (Zolt's brute force broad phase does not), so the active bodies are written sorted.
// outResult[0]: CreateBodies' result | FixInvalidScales' result << 1. Returns the size of the snapshot (the fixed scene's
// SaveBinaryState with shapes, the bodies in ID order, the sorted active bodies, PhysicsSystem::SaveState(Bodies |
// Constraints) and the SaveBinaryState with shapes of PhysicsScene::FromPhysicsSystem; the shape indices are cBINone:
// the shapes are the restored ones).
uint32 jolt_bi_scene_create_bodies(const uint8 *inBytes, uint32 inSize, uint32 inMaxBodies, uint32 *outResult, uint8 *outBytes, uint32 inCapacity)
{
	BIEnsureFactory();
	std::stringstream in_data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(in_data);
	PhysicsScene::PhysicsSceneResult result = PhysicsScene::sRestoreFromBinaryState(in);
	if (result.HasError())
	{
		outResult[0] = cBINone;
		return 0;
	}

	StateRecorderImpl recorder;
	PhysicsScene *scene = result.Get();
	bool fixed = scene->FixInvalidScales();
	{
		std::stringstream data;
		StreamOutWrapper out(data);
		scene->SaveBinaryState(out, true, false);
		std::string str = data.str();
		recorder.WriteBytes(str.data(), str.size());
	}

	BPLayerInterfaceImpl layers;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broad_phase;
	ObjectLayerPairFilterImpl pairs;
	PhysicsSystem system;
	system.Init(inMaxBodies, 0, 64, 64, layers, object_vs_broad_phase, pairs);
	outResult[0] = (scene->CreateBodies(&system)? 1 : 0) | (fixed? 2 : 0);

	BodyIDVector ids;
	system.GetBodies(ids);
	Array<const Body *> bodies;
	for (const BodyID &id : ids)
		bodies.push_back(system.GetBodyLockInterfaceNoLock().TryGetBody(id));
	BodyIDVector active_ids;
	system.GetActiveBodies(EBodyType::RigidBody, active_ids);
	Array<uint32> active;
	for (const BodyID &id : active_ids)
		active.push_back(id.GetIndexAndSequenceNumber());
	QuickSort(active.begin(), active.end());

	BITableObjects no_tables;
	BIWriteBodies(bodies, active, no_tables, recorder);
	system.SaveState(recorder, EStateRecorderState::Bodies | EStateRecorderState::Constraints);

	// The system back to a scene (the constraints through GetConstraintSettings)
	Ref<PhysicsScene> from_system = new PhysicsScene;
	from_system->FromPhysicsSystem(&system);
	{
		std::stringstream data;
		StreamOutWrapper out(data);
		from_system->SaveBinaryState(out, true, false);
		std::string str = data.str();
		recorder.WriteBytes(str.data(), str.size());
	}
	return BICopyBytes(recorder.GetData(), outBytes, inCapacity);
}

} // extern "C"
