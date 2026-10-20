// Reference implementation for the SoftBodyUpdate parity layer of group S4 (`zig build parity`,
// Docs/Zolt/RemainingFeaturesArchitecture.md section 4): a PhysicsSystem with a few rigid bodies and one soft body, no
// PhysicsSystem::Update. The operations of a case (SoftBodyMotionProperties::CustomUpdate, SkinVertices, the setters,
// SaveState / RestoreState, ...) run on the soft body, and after each one the state is written to a transcript.
// ZoltParity/Physics/SoftBodyUpdateParity.zig runs the same case on Zolt and compares the transcripts byte for byte.
//
// - The shared settings come as SaveBinaryState bytes (both sides restore them).
// - The transcript holds per operation: the soft body (position, rotation, bounds, the motion properties, every
//   vertex, rod and skin state, the local and predicted bounds, the volume, the private flags), its SaveState bytes and
//   the rigid bodies (position, rotation, velocities, active). The contact listener writes its calls when they happen,
//   with the collision data of every vertex. The staged update (SBUStagedUpdate) also writes the colliding shapes and
//   sensors and the collision data of the vertices in the middle of CustomUpdate, where they are not reset yet.
// - Values are written as their bytes: u32, f32, and RVec3 as 3 Real (f64 with JPH_DOUBLE_PRECISION).
//
// The structs below must match the extern structs in SoftBodyUpdateParity.zig. Booleans are passed as uint32. All C
// functions have the prefix jolt_sbu_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/SimShapeFilter.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>
#include <Jolt/Physics/SoftBody/SoftBodyUpdateContext.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

#include <UnitTests/Layers.h>

// The process hooks (R1): zero-filled allocations, a printing Trace and, with JPH_ENABLE_ASSERTS, an AssertFailed that
// prints and counts. Every exported function that allocates through Jolt calls ParityInstallHooks() first.
#include <ZoltParity/ParityHooks.h>

// Private access to the members of the motion properties that Jolt has no getter for (explicit instantiation ignores
// access checks, `auto` avoids naming the private nested types)
template <typename Tag, auto M>
struct SBURob
{
	friend auto SBUGet(Tag) { return M; }
};

#define SBU_ROB(Name, Member) \
	struct Name { friend auto SBUGet(Name); }; \
	template struct SBURob<Name, &Member>;

SBU_ROB(SBUSkinState, SoftBodyMotionProperties::mSkinState)
SBU_ROB(SBULocalPredictedBounds, SoftBodyMotionProperties::mLocalPredictedBounds)
SBU_ROB(SBUSkinStatePreviousPositionValid, SoftBodyMotionProperties::mSkinStatePreviousPositionValid)
SBU_ROB(SBUNeedContactCallback, SoftBodyMotionProperties::mNeedContactCallback)
SBU_ROB(SBUCollidingShapes, SoftBodyMotionProperties::mCollidingShapes)
SBU_ROB(SBUCollidingSensors, SoftBodyMotionProperties::mCollidingSensors)
SBU_ROB(SBUSkinnedMaxDistanceMultiplier, SoftBodyMotionProperties::mSkinnedMaxDistanceMultiplier)
SBU_ROB(SBUNumSensors, SoftBodyMotionProperties::mNumSensors)

/// The world of a case (SBUWorld in SoftBodyUpdateParity.zig)
struct SBUWorldDesc
{
	float			mGravity[3];
	uint32			mNumBodies;
	uint32			mOptimizeBroadPhase;	///< 1: PhysicsSystem::OptimizeBroadPhase after adding the bodies
	uint32			mSimShapeFilter;		///< 1: install SBUSimShapeFilter
	uint32			mGroupFilterDisabled;	///< The pairs of sub groups (cSBUGroupFilterPairs) whose collision the group filter table disables, one bit per pair
};

/// A rigid body (SBURigidBody in SoftBodyUpdateParity.zig)
struct SBURigidBodyDesc
{
	double			mPosition[3];
	float			mRotation[4];
	float			mParams[8];				///< See SBUMakeShapeSettings
	float			mLinearVelocity[3];
	float			mAngularVelocity[3];
	uint32			mShape;					///< 0 box, 1 sphere, 2 capsule, 3 mesh, 4 static compound of a box and a sphere, 5 scaled box
	uint32			mMotionType;
	uint32			mIsSensor;
	uint32			mObjectLayer;
	float			mFriction;
	float			mRestitution;
	uint32			mUserData;
	uint32			mFirstTriangle;			///< Mesh: index of its first triangle in the triangle pool (9 floats per triangle)
	uint32			mNumTriangles;
	uint32			mGroup;					///< The collision group (see SBUMakeCollisionGroup)
};

/// The soft body (SBUSoftBody in SoftBodyUpdateParity.zig)
struct SBUSoftBodyDesc
{
	double			mPosition[3];
	float			mRotation[4];
	uint32			mMakeRotationIdentity;
	uint32			mObjectLayer;
	uint32			mNumIterations;
	float			mLinearDamping;
	float			mMaxLinearVelocity;
	float			mRestitution;
	float			mFriction;
	float			mPressure;
	float			mGravityFactor;
	float			mVertexRadius;
	uint32			mUpdatePosition;
	uint32			mFacesDoubleSided;
	uint32			mAllowSleeping;
	uint32			mAddToSystem;			///< 1: BodyInterface::AddBody after BodyInterface::CreateSoftBody
	uint32			mGroup;					///< The collision group (see SBUMakeCollisionGroup)
	uint32			mPadding;
};

/// An operation (SBUOp in SoftBodyUpdateParity.zig)
struct SBUOp
{
	uint32			mKind;					///< SBUOpKind
	uint32			mU[4];
	float			mF[8];
};

/// The operations (must match OpKind in SoftBodyUpdateParity.zig)
enum class SBUOpKind : uint32
{
	CustomUpdate,			///< CustomUpdate(mF[0])
	SkinVertices,			///< SkinVertices with mU[1] joint matrices from the pool at mU[0] (16 floats each), mU[2] = hard skin all
	SetOptions,				///< mU[0] iterations, mU[1] update position, mU[2] faces double sided, mU[3] enable skin constraints, mF[0] pressure, mF[1] vertex radius, mF[2] skinned max distance multiplier, mF[3] gravity factor, mF[4] linear damping
	AddForce,				///< Body::AddForce(mF[0..2])
	SaveUpdateRestore,		///< SaveState, CustomUpdate(mF[0]), RestoreState
	CalculateMassAndInertia,
	SetVertex,				///< vertex mU[0]: velocity mF[0..2], inverse mass mF[3]
	SetListener,			///< mode mU[0] (0 none, 1 accept, 2 reject odd body indices, 3 odd body indices are sensors, 4 the mass scales mF[0..2])
	SetBodyVelocity,		///< rigid body mU[0]: BodyInterface::SetLinearAndAngularVelocity(mF[0..2], mF[3..5])
	SetBodyPosition,		///< rigid body mU[0]: BodyInterface::SetPosition(mF[0..2], DontActivate)
	StagedUpdate,			///< CustomUpdate(mF[0]) step by step, with the state written in between (SBUStagedUpdate)
};

// Tags of the transcript (must match SoftBodyUpdateParity.zig)
static constexpr uint32 cSBUTagOp = 0x5B000000;
static constexpr uint32 cSBUTagValidate = 0x5B100000;
static constexpr uint32 cSBUTagAdded = 0x5B200000;
static constexpr uint32 cSBUTagStaged = 0x5B300000;

/// The pairs of sub groups that SBUWorldDesc::mGroupFilterDisabled can disable (the group filter table has 4 sub groups)
static constexpr uint32 cSBUGroupFilterPairs[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };

/// The transcript: the bytes of the written values, in order
class SBUTranscript
{
public:
						SBUTranscript(uint8 *inBuffer, uint32 inCapacity) : mBuffer(inBuffer), mCapacity(inCapacity) { }

	void				Bytes(const void *inData, size_t inSize)
	{
		if (mOverflow || mSize + inSize > mCapacity)
		{
			mOverflow = true;
			return;
		}
		memcpy(mBuffer + mSize, inData, inSize);
		mSize += uint32(inSize);
	}

	void				U32(uint32 inValue)				{ Bytes(&inValue, sizeof(inValue)); }
	void				F32(float inValue)				{ Bytes(&inValue, sizeof(inValue)); }
	void				R(Real inValue)					{ Bytes(&inValue, sizeof(inValue)); }
	void				V3(Vec3Arg inValue)				{ F32(inValue.GetX()); F32(inValue.GetY()); F32(inValue.GetZ()); }
	void				Q(QuatArg inValue)				{ F32(inValue.GetX()); F32(inValue.GetY()); F32(inValue.GetZ()); F32(inValue.GetW()); }
	void				R3(RVec3Arg inValue)			{ R(inValue.GetX()); R(inValue.GetY()); R(inValue.GetZ()); }
	void				Box(const AABox &inValue)		{ V3(inValue.mMin); V3(inValue.mMax); }
	void				Mat(Mat44Arg inValue, uint inNumColumns) { for (uint i = 0; i < inNumColumns; ++i) V3(inValue.GetColumn3(i)); } ///< The 3x3 part (and the translation with 4 columns)

	uint8 *				mBuffer;
	uint32				mCapacity;
	uint32				mSize = 0;
	bool				mOverflow = false;
};

/// The collision data of a vertex (DumpVertexCollision of SoftBodyUpdateParity.zig)
static void SBUDumpVertexCollision(SBUTranscript &t, const SoftBodyVertex &inVertex)
{
	t.V3(inVertex.mCollisionPlane.GetNormal());
	t.F32(inVertex.mCollisionPlane.GetConstant());
	t.U32(uint32(inVertex.mCollidingShapeIndex));
	t.U32(inVertex.mHasContact? 1 : 0);
	t.F32(inVertex.mLargestPenetration);
}

/// The contact listener: writes its calls to the transcript
class SBUListener : public SoftBodyContactListener
{
public:
	virtual SoftBodyValidateResult OnSoftBodyContactValidate(const Body &inSoftBody, const Body &inOtherBody, SoftBodyContactSettings &ioSettings) override
	{
		SBUTranscript &t = *mTranscript;
		t.U32(cSBUTagValidate);
		t.U32(inSoftBody.GetID().GetIndexAndSequenceNumber());
		t.U32(inOtherBody.GetID().GetIndexAndSequenceNumber());
		t.U32(ioSettings.mIsSensor? 1 : 0);

		SoftBodyValidateResult result = SoftBodyValidateResult::AcceptContact;
		bool odd = (inOtherBody.GetID().GetIndex() & 1) != 0;
		switch (mMode)
		{
		case 2:
			if (odd)
				result = SoftBodyValidateResult::RejectContact;
			break;

		case 3:
			if (odd)
				ioSettings.mIsSensor = true;
			break;

		case 4:
			ioSettings.mInvMassScale1 = mScales[0];
			ioSettings.mInvMassScale2 = mScales[1];
			ioSettings.mInvInertiaScale2 = mScales[2];
			break;

		default:
			break;
		}

		t.F32(ioSettings.mInvMassScale1);
		t.F32(ioSettings.mInvMassScale2);
		t.F32(ioSettings.mInvInertiaScale2);
		t.U32(ioSettings.mIsSensor? 1 : 0);
		t.U32(uint32(result));
		return result;
	}

	virtual void		OnSoftBodyContactAdded(const Body &inSoftBody, const SoftBodyManifold &inManifold) override
	{
		SBUTranscript &t = *mTranscript;
		t.U32(cSBUTagAdded);
		t.U32(inSoftBody.GetID().GetIndexAndSequenceNumber());
		const Array<SoftBodyVertex> &vertices = inManifold.GetVertices();
		t.U32(uint32(vertices.size()));
		for (const SoftBodyVertex &v : vertices)
		{
			bool has_contact = inManifold.HasContact(v);
			t.U32(has_contact? 1 : 0);
			if (has_contact)
			{
				t.U32(inManifold.GetContactBodyID(v).GetIndexAndSequenceNumber());
				t.V3(inManifold.GetLocalContactPoint(v));
				t.V3(inManifold.GetContactNormal(v));
			}

			// The collision data of the vertex before UpdateSoftBodyState resets it
			SBUDumpVertexCollision(t, v);
		}
		t.U32(inManifold.GetNumSensorContacts());
		for (uint i = 0; i < inManifold.GetNumSensorContacts(); ++i)
			t.U32(inManifold.GetSensorContactBodyID(i).GetIndexAndSequenceNumber());
	}

	SBUTranscript *		mTranscript = nullptr;
	uint32				mMode = 0;
	float				mScales[3] = { 1.0f, 1.0f, 1.0f };
};

/// The simulation shape filter: rejects the sub shapes with an odd sub shape ID value of the bodies with user data 1
class SBUSimShapeFilter : public SimShapeFilter
{
public:
	virtual bool		ShouldCollide([[maybe_unused]] const Body &inBody1, [[maybe_unused]] const Shape *inShape1, [[maybe_unused]] const SubShapeID &inSubShapeIDOfShape1, const Body &inBody2, [[maybe_unused]] const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		return inBody2.GetUserData() != 1 || (inSubShapeIDOfShape2.GetValue() & 1) == 0;
	}
};

static Vec3 SBULoad3(const float *inV)
{
	return Vec3(inV[0], inV[1], inV[2]);
}

static RVec3 SBULoadR3(const double *inV)
{
	return RVec3(Real(inV[0]), Real(inV[1]), Real(inV[2]));
}

static Quat SBULoadQuat(const float *inV)
{
	return Quat(inV[0], inV[1], inV[2], inV[3]);
}

// MakeShapeSettings of SoftBodyUpdateParity.zig
static Ref<ShapeSettings> SBUMakeShapeSettings(const SBURigidBodyDesc &inDesc, const float *inTriangles)
{
	const float *p = inDesc.mParams;
	switch (inDesc.mShape)
	{
	case 0:		return new BoxShapeSettings(SBULoad3(p), p[3]);
	case 1:		return new SphereShapeSettings(p[0]);
	case 2:		return new CapsuleShapeSettings(p[0], p[1]);
	case 3:
		{
			TriangleList triangles;
			for (uint32 i = 0; i < inDesc.mNumTriangles; ++i)
			{
				const float *t = inTriangles + 9 * (inDesc.mFirstTriangle + i);
				triangles.push_back(Triangle(Vec3(t[0], t[1], t[2]), Vec3(t[3], t[4], t[5]), Vec3(t[6], t[7], t[8])));
			}
			return new MeshShapeSettings(triangles);
		}
	case 4:
		{
			StaticCompoundShapeSettings *compound = new StaticCompoundShapeSettings;
			compound->AddShape(SBULoad3(p + 4), Quat::sIdentity(), new BoxShapeSettings(SBULoad3(p), p[3]));
			compound->AddShape(-SBULoad3(p + 4), Quat::sIdentity(), new SphereShapeSettings(p[7]));
			return compound;
		}
	default:	return new ScaledShapeSettings(new BoxShapeSettings(SBULoad3(p), p[3]), SBULoad3(p + 4));
	}
}

/// The state of the soft body and the rigid bodies after an operation (DumpState of SoftBodyUpdateParity.zig)
static void SBUDumpState(SBUTranscript &t, uint32 inOpIndex, const Body &inSoftBody, const Array<Body *> &inBodies)
{
	const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(inSoftBody.GetMotionProperties());

	t.U32(cSBUTagOp | inOpIndex);

	// The body
	t.R3(inSoftBody.GetPosition());
	t.Q(inSoftBody.GetRotation());
	t.R3(inSoftBody.GetCenterOfMassPosition());
	t.Box(inSoftBody.GetWorldSpaceBounds());

	// The motion properties
	t.V3(mp->GetLinearVelocity());
	t.V3(mp->GetAngularVelocity());
	t.F32(mp->GetInverseMassUnchecked());
	t.V3(mp->GetInverseInertiaDiagonal());
	t.Q(mp->GetInertiaRotation());
	t.V3(mp->GetAccumulatedForce());
	t.U32(mp->GetNumIterations());
	t.F32(mp->GetPressure());
	t.F32(mp->GetVertexRadius());
	t.U32(mp->GetUpdatePosition()? 1 : 0);
	t.U32(mp->GetFacesDoubleSided()? 1 : 0);
	t.U32(mp->GetEnableSkinConstraints()? 1 : 0);
	t.F32(mp->*SBUGet(SBUSkinnedMaxDistanceMultiplier()));

	// The vertices
	const Array<SoftBodyVertex> &vertices = mp->GetVertices();
	t.U32(uint32(vertices.size()));
	for (const SoftBodyVertex &v : vertices)
	{
		t.V3(v.mPreviousPosition);
		t.V3(v.mPosition);
		t.V3(v.mVelocity);
		t.V3(v.mCollisionPlane.GetNormal());
		t.F32(v.mCollisionPlane.GetConstant());
		t.U32(uint32(v.mCollidingShapeIndex));
		t.U32(v.mHasContact? 1 : 0);
		t.F32(v.mLargestPenetration);
		t.F32(v.mInvMass);
	}

	// The rods
	uint32 num_rods = uint32(mp->GetSettings()->mRodStretchShearConstraints.size());
	t.U32(num_rods);
	for (uint32 i = 0; i < num_rods; ++i)
	{
		t.Q(mp->GetRodRotation(i));
		t.V3(mp->GetRodAngularVelocity(i));
	}

	// The skin states
	const auto &skin_state = mp->*SBUGet(SBUSkinState());
	t.U32(uint32(skin_state.size()));
	for (const auto &s : skin_state)
	{
		t.V3(s.mPreviousPosition);
		t.V3(s.mPosition);
		t.V3(s.mNormal);
	}

	// Bounds, volume and the private state
	t.Box(mp->GetLocalBounds());
	t.Box(mp->*SBUGet(SBULocalPredictedBounds()));
	t.F32(mp->GetVolume());
	t.U32((mp->*SBUGet(SBUSkinStatePreviousPositionValid()))? 1 : 0);
	t.U32((mp->*SBUGet(SBUNeedContactCallback())).load(std::memory_order_relaxed)? 1 : 0);
	t.U32(uint32((mp->*SBUGet(SBUCollidingShapes())).size()));
	t.U32(uint32((mp->*SBUGet(SBUCollidingSensors())).size()));

	// The SaveState bytes of the body
	StateRecorderImpl recorder;
	inSoftBody.SaveState(recorder);
	std::string data = recorder.GetData();
	t.U32(uint32(data.size()));
	t.Bytes(data.data(), data.size());

	// The rigid bodies
	for (const Body *b : inBodies)
	{
		t.R3(b->GetPosition());
		t.Q(b->GetRotation());
		t.V3(b->GetLinearVelocity());
		t.V3(b->GetAngularVelocity());
		t.U32(b->IsActive()? 1 : 0);
	}
}

/// The leaf shapes of a colliding shape or sensor (DumpLeafShapes of SoftBodyUpdateParity.zig)
template <class LeafShapes>
static void SBUDumpLeafShapes(SBUTranscript &t, const LeafShapes &inShapes)
{
	t.U32(uint32(inShapes.size()));
	for (const auto &l : inShapes)
	{
		t.Mat(l.mTransform, 4);
		t.V3(l.mScale);
		t.U32(uint32(l.mShape->GetSubType()));
	}
}

/// The colliding shapes and sensors of the motion properties (DumpColliding of SoftBodyUpdateParity.zig). The members
/// that Jolt leaves uninitialized are skipped: the dynamic members of a body that is not dynamic, and the contact flag of
/// a sensor before DetermineSensorCollisions (inSensorContacts false).
static void SBUDumpColliding(SBUTranscript &t, const SoftBodyMotionProperties &inMP, bool inSensorContacts)
{
	const auto &shapes = inMP.*SBUGet(SBUCollidingShapes());
	t.U32(uint32(shapes.size()));
	for (const auto &cs : shapes)
	{
		t.U32(cs.mBodyID.GetIndexAndSequenceNumber());
		t.U32(uint32(cs.mMotionType));
		t.Mat(cs.mCenterOfMassTransform, 4);
		t.F32(cs.mFriction);
		t.F32(cs.mRestitution);
		t.F32(cs.mSoftBodyInvMassScale);
		t.U32(cs.mUpdateVelocities? 1 : 0);
		SBUDumpLeafShapes(t, cs.mShapes);
		if (cs.mMotionType == EMotionType::Dynamic)
		{
			t.F32(cs.mInvMass);
			t.Mat(cs.mInvInertia, 3);
			t.V3(cs.mLinearVelocity);
			t.V3(cs.mAngularVelocity);
			t.V3(cs.mOriginalLinearVelocity);
			t.V3(cs.mOriginalAngularVelocity);
		}
	}
	const auto &sensors = inMP.*SBUGet(SBUCollidingSensors());
	t.U32(uint32(sensors.size()));
	for (const auto &cs : sensors)
	{
		t.U32(cs.mBodyID.GetIndexAndSequenceNumber());
		t.Mat(cs.mCenterOfMassTransform, 4);
		SBUDumpLeafShapes(t, cs.mShapes);
		if (inSensorContacts)
			t.U32(cs.mHasContact? 1 : 0);
	}
}

/// SoftBodyMotionProperties::CustomUpdate with the state written after DetermineCollidingShapes (stage 0), after the
/// collision phase (stage 1, the vertex collision data and the sensor contacts) and when the update is done (stage 2,
/// before UpdateRigidBodyVelocities clears the colliding shapes). StagedUpdate of SoftBodyUpdateParity.zig.
static bool SBUStagedUpdate(SBUTranscript &t, SoftBodyMotionProperties &ioMP, float inDeltaTime, Body &ioSoftBody, PhysicsSystem &inSystem)
{
	// Create update context
	SoftBodyUpdateContext context;
	ioMP.InitializeUpdateContext(inDeltaTime, ioSoftBody, inSystem, context);

	// Determine bodies we're colliding with
	ioMP.DetermineCollidingShapes(context, inSystem, inSystem.GetBodyLockInterface());
	t.U32(cSBUTagStaged | 0);
	t.V3(context.mGravity);
	t.V3(context.mDisplacementDueToGravity);
	t.F32(context.mSubStepDeltaTime);
	t.U32(ioMP.*SBUGet(SBUNumSensors()));
	SBUDumpColliding(t, ioMP, false);

	// The collision phase: the collision planes, then the sensors
	const PhysicsSettings &settings = inSystem.GetPhysicsSettings();
	for (;;)
	{
		SoftBodyUpdateContext::EState state = context.mState.load(memory_order_acquire);
		if (state != SoftBodyUpdateContext::EState::DetermineCollisionPlanes && state != SoftBodyUpdateContext::EState::DetermineSensorCollisions)
			break;
		if (ioMP.ParallelUpdate(context, settings) != SoftBodyMotionProperties::EStatus::DidWork)
			return false;
	}
	t.U32(cSBUTagStaged | 1);
	t.U32(uint32(context.mState.load(memory_order_acquire)));
	for (const SoftBodyVertex &v : ioMP.GetVertices())
		SBUDumpVertexCollision(t, v);
	for (const auto &cs : ioMP.*SBUGet(SBUCollidingSensors()))
		t.U32(cs.mHasContact? 1 : 0);
	t.U32((ioMP.*SBUGet(SBUNeedContactCallback())).load(memory_order_relaxed)? 1 : 0);

	// Call the internal update until it finishes
	SoftBodyMotionProperties::EStatus status;
	while ((status = ioMP.ParallelUpdate(context, settings)) == SoftBodyMotionProperties::EStatus::DidWork)
		continue;
	if (status != SoftBodyMotionProperties::EStatus::Done)
		return false;
	t.U32(cSBUTagStaged | 2);
	t.V3(context.mDeltaPosition);
	t.U32(uint32(context.mCanSleep));
	SBUDumpColliding(t, ioMP, true);

	// Update the state of the bodies we've collided with
	ioMP.UpdateRigidBodyVelocities(context, inSystem.GetBodyInterface());

	// Update position of the soft body
	if (ioMP.GetUpdatePosition())
		inSystem.GetBodyInterface().SetPosition(ioSoftBody.GetID(), ioSoftBody.GetPosition() + context.mDeltaPosition, EActivation::DontActivate);
	return true;
}

/// The collision group of SBURigidBodyDesc::mGroup and SBUSoftBodyDesc::mGroup (MakeCollisionGroup of
/// SoftBodyUpdateParity.zig): 0 is the default group. Otherwise bits 0 to 7 are the sub group, bits 8 to 15 the group
/// (0xFF: cInvalidGroup) and bit 16 selects the group filter table.
static CollisionGroup SBUMakeCollisionGroup(uint32 inGroup, const GroupFilter *inFilter)
{
	if (inGroup == 0)
		return CollisionGroup();
	uint32 id = (inGroup >> 8) & 0xff;
	return CollisionGroup((inGroup & 0x10000) != 0? inFilter : nullptr, id == 0xff? CollisionGroup::cInvalidGroup : id, inGroup & 0xff);
}

extern "C"
{

/// sizeof of the structs of this file (the Zig side checks its extern structs)
void jolt_sbu_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(SBUWorldDesc);
	outSizes[1] = sizeof(SBURigidBodyDesc);
	outSizes[2] = sizeof(SBUSoftBodyDesc);
	outSizes[3] = sizeof(SBUOp);
}

/// Runs a case (see the file header): creates the world, the rigid bodies and the soft body, runs the operations and
/// writes the transcript to outTranscript. Returns its size, 0xFFFFFFFF when it does not fit or the settings bytes do
/// not restore.
uint32 jolt_sbu_run(const SBUWorldDesc *inWorld, const SBURigidBodyDesc *inBodies, const float *inTriangles, const uint8 *inSettings, uint32 inSettingsSize, const SBUSoftBodyDesc *inSoftBody, const SBUOp *inOps, uint32 inNumOps, const float *inPool, uint8 *outTranscript, uint32 inCapacity)
{
	ParityInstallHooks();

	SBUTranscript t(outTranscript, inCapacity);

	// The shared settings
	Ref<SoftBodySharedSettings> shared = new SoftBodySharedSettings;
	{
		std::stringstream data(std::string(reinterpret_cast<const char *>(inSettings), inSettingsSize));
		StreamInWrapper in(data);
		shared->RestoreBinaryState(in);
		if (in.IsFailed())
			return 0xFFFFFFFF;
	}

	// The world (the layers of UnitTests/Layers.h)
	BPLayerInterfaceImpl layers;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broad_phase;
	ObjectLayerPairFilterImpl pairs;
	PhysicsSystem system;
	system.Init(64, 0, 64, 64, layers, object_vs_broad_phase, pairs);
	system.SetGravity(SBULoad3(inWorld->mGravity));
	SBUSimShapeFilter sim_shape_filter;
	if (inWorld->mSimShapeFilter != 0)
		system.SetSimShapeFilter(&sim_shape_filter);
	SBUListener listener;
	listener.mTranscript = &t;
	BodyInterface &bi = system.GetBodyInterface();

	// The group filter of the collision groups
	Ref<GroupFilterTable> group_filter = new GroupFilterTable(4);
	for (uint32 k = 0; k < 6; ++k)
		if ((inWorld->mGroupFilterDisabled >> k) & 1)
			group_filter->DisableCollision(cSBUGroupFilterPairs[k][0], cSBUGroupFilterPairs[k][1]);

	// The rigid bodies
	Array<Body *> bodies;
	for (uint32 i = 0; i < inWorld->mNumBodies; ++i)
	{
		const SBURigidBodyDesc &d = inBodies[i];
		BodyCreationSettings s(SBUMakeShapeSettings(d, inTriangles), SBULoadR3(d.mPosition), SBULoadQuat(d.mRotation), EMotionType(d.mMotionType), ObjectLayer(d.mObjectLayer));
		s.mLinearVelocity = SBULoad3(d.mLinearVelocity);
		s.mAngularVelocity = SBULoad3(d.mAngularVelocity);
		s.mIsSensor = d.mIsSensor != 0;
		s.mFriction = d.mFriction;
		s.mRestitution = d.mRestitution;
		s.mUserData = d.mUserData;
		s.mCollisionGroup = SBUMakeCollisionGroup(d.mGroup, group_filter);
		Body *body = bi.CreateBody(s);
		if (body == nullptr)
			return 0xFFFFFFFF;
		bi.AddBody(body->GetID(), d.mMotionType == uint32(EMotionType::Static)? EActivation::DontActivate : EActivation::Activate);
		bodies.push_back(body);
	}

	// The soft body
	const SBUSoftBodyDesc &sd = *inSoftBody;
	SoftBodyCreationSettings ss(shared, SBULoadR3(sd.mPosition), SBULoadQuat(sd.mRotation), ObjectLayer(sd.mObjectLayer));
	ss.mMakeRotationIdentity = sd.mMakeRotationIdentity != 0;
	ss.mNumIterations = sd.mNumIterations;
	ss.mLinearDamping = sd.mLinearDamping;
	ss.mMaxLinearVelocity = sd.mMaxLinearVelocity;
	ss.mRestitution = sd.mRestitution;
	ss.mFriction = sd.mFriction;
	ss.mPressure = sd.mPressure;
	ss.mGravityFactor = sd.mGravityFactor;
	ss.mVertexRadius = sd.mVertexRadius;
	ss.mUpdatePosition = sd.mUpdatePosition != 0;
	ss.mFacesDoubleSided = sd.mFacesDoubleSided != 0;
	ss.mAllowSleeping = sd.mAllowSleeping != 0;
	ss.mCollisionGroup = SBUMakeCollisionGroup(sd.mGroup, group_filter);
	Body *soft_body = bi.CreateSoftBody(ss);
	if (soft_body == nullptr)
		return 0xFFFFFFFF;
	if (sd.mAddToSystem != 0)
		bi.AddBody(soft_body->GetID(), EActivation::Activate);
	if (inWorld->mOptimizeBroadPhase != 0)
		system.OptimizeBroadPhase();
	SoftBodyMotionProperties *mp = static_cast<SoftBodyMotionProperties *>(soft_body->GetMotionProperties());

	TempAllocatorImpl temp_allocator(1024 * 1024);

	// The operations
	SBUDumpState(t, 0xFFFFF, *soft_body, bodies);
	for (uint32 op_index = 0; op_index < inNumOps; ++op_index)
	{
		const SBUOp &op = inOps[op_index];
		switch (SBUOpKind(op.mKind))
		{
		case SBUOpKind::CustomUpdate:
			mp->CustomUpdate(op.mF[0], *soft_body, system);
			break;

		case SBUOpKind::SkinVertices:
			{
				Array<Mat44> joints;
				for (uint32 j = 0; j < op.mU[1]; ++j)
				{
					const float *m = inPool + op.mU[0] + 16 * j;
					joints.push_back(Mat44(Vec4(m[0], m[1], m[2], m[3]), Vec4(m[4], m[5], m[6], m[7]), Vec4(m[8], m[9], m[10], m[11]), Vec4(m[12], m[13], m[14], m[15])));
				}
				mp->SkinVertices(soft_body->GetCenterOfMassTransform(), joints.data(), uint(joints.size()), op.mU[2] != 0, temp_allocator);
			}
			break;

		case SBUOpKind::SetOptions:
			mp->SetNumIterations(op.mU[0]);
			mp->SetUpdatePosition(op.mU[1] != 0);
			mp->SetFacesDoubleSided(op.mU[2] != 0);
			mp->SetEnableSkinConstraints(op.mU[3] != 0);
			mp->SetPressure(op.mF[0]);
			mp->SetVertexRadius(op.mF[1]);
			mp->SetSkinnedMaxDistanceMultiplier(op.mF[2]);
			mp->SetGravityFactor(op.mF[3]);
			mp->SetLinearDamping(op.mF[4]);
			break;

		case SBUOpKind::AddForce:
			soft_body->AddForce(SBULoad3(op.mF));
			break;

		case SBUOpKind::SaveUpdateRestore:
			{
				StateRecorderImpl recorder;
				soft_body->SaveState(recorder);
				mp->CustomUpdate(op.mF[0], *soft_body, system);
				recorder.Rewind();
				soft_body->RestoreState(recorder);
			}
			break;

		case SBUOpKind::CalculateMassAndInertia:
			mp->CalculateMassAndInertia();
			break;

		case SBUOpKind::SetVertex:
			{
				SoftBodyVertex &v = mp->GetVertex(op.mU[0]);
				v.mVelocity = SBULoad3(op.mF);
				v.mInvMass = op.mF[3];
			}
			break;

		case SBUOpKind::SetListener:
			listener.mMode = op.mU[0];
			listener.mScales[0] = op.mF[0];
			listener.mScales[1] = op.mF[1];
			listener.mScales[2] = op.mF[2];
			system.SetSoftBodyContactListener(op.mU[0] == 0? nullptr : &listener);
			break;

		case SBUOpKind::SetBodyVelocity:
			bi.SetLinearAndAngularVelocity(bodies[op.mU[0]]->GetID(), SBULoad3(op.mF), SBULoad3(op.mF + 3));
			break;

		case SBUOpKind::SetBodyPosition:
			bi.SetPosition(bodies[op.mU[0]]->GetID(), RVec3(SBULoad3(op.mF)), EActivation::DontActivate);
			break;

		case SBUOpKind::StagedUpdate:
			if (!SBUStagedUpdate(t, *mp, op.mF[0], *soft_body, system))
				return 0xFFFFFFFF;
			break;
		}

		SBUDumpState(t, op_index, *soft_body, bodies);
	}

	// Clean up
	BodyID soft_body_id = soft_body->GetID();
	if (sd.mAddToSystem != 0)
		bi.RemoveBody(soft_body_id);
	bi.DestroyBody(soft_body_id);
	for (Body *b : bodies)
	{
		bi.RemoveBody(b->GetID());
		bi.DestroyBody(b->GetID());
	}

	return t.mOverflow? 0xFFFFFFFF : t.mSize;
}

} // extern "C"
