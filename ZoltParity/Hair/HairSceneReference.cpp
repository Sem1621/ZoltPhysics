// Reference implementation for ZoltParity/Hair/HairSceneParity.zig (`zig build parity`): Jolt's Hair in a
// PhysicsSystem (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, row "Hair scenes").
//
// A scene is a PhysicsSystem with rigid bodies (convex hulls, scaled and inside out hulls, compounds of hulls, rotated
// and translated hulls, and bodies that Hair ignores), a hair on settings that the Zig side built (their
// SaveBinaryState bytes after Init, restored here) and a list of operations: Hair::Update with a delta time and
// optionally joint matrices, Hair::SetPosition / SetRotation / OnTeleported and the movement of a body. After every
// operation the state of the hair goes to a transcript: the members of the hair and the bytes of every compute buffer
// of the hair. After an update also the data of ReadBackGPUState (the positions, rotations, velocities, the grid, the
// render positions and the skinned scalp vertices). ZoltParity/Hair/HairSceneParity.zig runs the same scene on Zolt and
// compares the transcripts byte for byte.
//
// - The hooks of ParityHooks.h are installed: every allocation is zero filled, like Zolt's compute buffers (UB4 of R20).
// - The libm pin (LibMReference.c) gives std::exp and std::pow the generic algorithm of LibM.zig (R13).
// - The transcript is a list of records: a tag (uint32), the number of bytes (uint32, 0xFFFFFFFF for a null buffer) and
//   the bytes.
//
// The structs below must match the extern structs in HairSceneParity.zig. Booleans are passed as uint32. All C
// functions have the prefix jolt_hs_.

#include <Jolt/Jolt.h>

#include <ZoltParity/ParityHooks.h>

#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Compute/ComputeSystem.h>
#include <Jolt/Compute/CPU/ComputeSystemCPU.h>
#include <Jolt/Shaders/HairWrapper.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Hair/Hair.h>
#include <Jolt/Physics/Hair/HairSettings.h>
#include <Jolt/Physics/Hair/HairShaders.h>

#include <cstring>
#include <iterator>
#include <sstream>
#include <string>

using namespace JPH;

#include <UnitTests/Layers.h>

extern "C" int jolt_libm_reference_linked();

/// The world of a scene (World in HairSceneParity.zig)
struct HSWorldDesc
{
	float			mGravity[3];
	uint32			mNumBodies;
	uint32			mOptimizeBroadPhase;	///< 1: PhysicsSystem::OptimizeBroadPhase after adding the bodies
	uint32			mPadding;
};

/// A rigid body (Body in HairSceneParity.zig)
struct HSBodyDesc
{
	double			mPosition[3];
	float			mRotation[4];
	float			mLinearVelocity[3];
	float			mAngularVelocity[3];
	float			mParams[12];			///< See HSMakeShapeSettings
	uint32			mShape;					///< HSShape
	uint32			mMotionType;
	uint32			mIsSensor;
	uint32			mObjectLayer;
	uint32			mFirstPoint;			///< The hull points: the index of the first one in the point pool (3 floats per point)
	uint32			mNumPoints;
};

/// The hair (HairDesc in HairSceneParity.zig)
struct HSHairDesc
{
	double			mPosition[3];
	float			mRotation[4];
	uint32			mObjectLayer;
	uint32			mExternalScalp;			///< 1: before Init, SetScalpVerticesCB and SetScalpTrianglesCB with buffers of the scalp of the settings
};

/// An operation (Op in HairSceneParity.zig)
struct HSOp
{
	uint32			mKind;					///< HSOpKind
	uint32			mU[3];
	double			mD[3];
	float			mF[12];
};

/// The shapes of the bodies (ShapeKind in HairSceneParity.zig)
enum class HSShape : uint32
{
	Hull,					///< ConvexHullShape of the points, max convex radius mParams[0]
	ScaledHull,				///< ScaledShape of the hull, scale mParams[0..2] (inside out when the scale has an odd number of negative components)
	StaticCompound,			///< StaticCompoundShape: the hull at mParams[0..2] with rotation mParams[3..6], the hull scaled by mParams[7..9] at -mParams[0..2], a sphere of radius mParams[10] at the origin
	MutableCompound,		///< MutableCompoundShape: the hull at mParams[0..2], the hull scaled by mParams[3..5] at -mParams[0..2]
	RotatedTranslatedHull,	///< RotatedTranslatedShape of the hull: offset mParams[0..2], rotation mParams[3..6]
	Box,					///< BoxShape: half extent mParams[0..2]
	Sphere,					///< SphereShape: radius mParams[0]
	Capsule,				///< CapsuleShape: half height mParams[0], radius mParams[1]
};

/// The operations (OpKind in HairSceneParity.zig)
enum class HSOpKind : uint32
{
	Update,					///< Hair::Update(mF[0]): joint to hair the 16 floats at pool index mU[1], joint matrices the mU[2] matrices at pool index mU[0] (0xFFFFFFFF: none)
	SetPosition,			///< Hair::SetPosition(mD)
	SetRotation,			///< Hair::SetRotation(mF[0..3])
	OnTeleported,			///< Hair::OnTeleported()
	MoveBody,				///< body mU[0]: BodyInterface::SetPositionRotationAndVelocity(mD, mF[0..3], mF[4..6], mF[7..9])
	SetScalpToHead,			///< Hair::SetScalpToHead with the 16 floats at pool index mU[0]
};

// Tags of the transcript (must match HairSceneParity.zig)
static constexpr uint32 cHSTagOp = 0x45000000;		///< | the operation index (0xFFFFF: after Init)
static constexpr uint32 cHSTagMember = 0x45100000;	///< | the member (see HSDump)
static constexpr uint32 cHSTagBuffer = 0x45200000;	///< | the buffer (see HSDump)
static constexpr uint32 cHSTagRead = 0x45300000;	///< | the read back array (see HSDump)

/// The transcript: the records, in order
class HSTranscript
{
public:
						HSTranscript(uint8 *inBuffer, uint32 inCapacity) : mBuffer(inBuffer), mCapacity(inCapacity) { }

	void				Bytes(const void *inData, size_t inSize)
	{
		if (mOverflow || mSize + inSize > mCapacity)
		{
			mOverflow = true;
			return;
		}
		if (inSize > 0)
			memcpy(mBuffer + mSize, inData, inSize);
		mSize += uint32(inSize);
	}

	void				U32(uint32 inValue)				{ Bytes(&inValue, sizeof(inValue)); }

	/// A record: the tag, the size and the bytes
	void				Record(uint32 inTag, const void *inData, size_t inSize)
	{
		U32(inTag);
		U32(uint32(inSize));
		Bytes(inData, inSize);
	}

	/// A record of a compute buffer: its bytes, or the size 0xFFFFFFFF when it is null
	void				Buffer(uint32 inTag, ComputeBuffer *inBuffer)
	{
		if (inBuffer == nullptr)
		{
			U32(inTag);
			U32(0xFFFFFFFF);
			return;
		}
		const void *data = inBuffer->Map(ComputeBuffer::EMode::Read);
		Record(inTag, data, size_t(inBuffer->GetSize()) * inBuffer->GetStride());
		inBuffer->Unmap();
	}

	uint8 *				mBuffer;
	uint32				mCapacity;
	uint32				mSize = 0;
	bool				mOverflow = false;
};

/// Access to the protected members of Hair (DumpHair of HairSceneParity.zig)
class HSHair : public Hair
{
public:
	using				Hair::Hair;

	/// The members and the compute buffers of the hair (the order of DumpHair in HairSceneParity.zig)
	void				Dump(HSTranscript &t) const
	{
		uint32 teleported = mTeleported? 1 : 0;
		t.Record(cHSTagMember | 0, &teleported, sizeof(teleported));
		Real prev_position[3] = { mPrevPosition.GetX(), mPrevPosition.GetY(), mPrevPosition.GetZ() };
		t.Record(cHSTagMember | 1, prev_position, sizeof(prev_position));
		float prev_rotation[4] = { mPrevRotation.GetX(), mPrevRotation.GetY(), mPrevRotation.GetZ(), mPrevRotation.GetW() };
		t.Record(cHSTagMember | 2, prev_rotation, sizeof(prev_rotation));

		ComputeBuffer *buffers[] = {
			mScalpJointMatricesCB,
			mScalpVerticesCB,
			mScalpTrianglesCB,
			mTargetPositionsCB,
			mTargetGlobalPoseTransformsCB,
			mGlobalPoseTransformsCB,
			mShapePlanesCB,
			mShapeVerticesCB,
			mShapeIndicesCB,
			mCollisionPlanesCB,
			mCollisionShapesCB,
			mMaterialsCB,
			mPreviousPositionsCB,
			mPositionsCB,
			mVelocitiesCB,
			mVelocityAndDensityCB,
			mConstantsCB,
			mRenderPositionsCB,
		};
		for (uint32 i = 0; i < uint32(std::size(buffers)); ++i)
			t.Buffer(cHSTagBuffer | i, buffers[i]);
		uint32 num_iteration_buffers = uint32(mIterationConstantsCB.size());
		t.Record(cHSTagBuffer | 100, &num_iteration_buffers, sizeof(num_iteration_buffers));
		for (uint32 i = 0; i < num_iteration_buffers; ++i)
			t.Buffer(cHSTagBuffer | (101 + i), mIterationConstantsCB[i]);
	}

	/// ReadBackGPUState and the data of the lock (the order of DumpReadBack in HairSceneParity.zig)
	void				DumpReadBack(HSTranscript &t, ComputeQueue *inQueue)
	{
		ReadBackGPUState(inQueue);
		LockReadBackBuffers();

		uint32 num_vertices = uint32(mSettings->mSimVertices.size());
		t.Record(cHSTagRead | 0, GetPositions(), num_vertices * sizeof(Float3));
		Array<float> rotations;
		for (uint32 v = 0; v < num_vertices; ++v)
		{
			Quat q = GetRotations()[v];
			rotations.push_back(q.GetX());
			rotations.push_back(q.GetY());
			rotations.push_back(q.GetZ());
			rotations.push_back(q.GetW());
		}
		t.Record(cHSTagRead | 1, rotations.data(), rotations.size() * sizeof(float));
		Array<Float3> velocities, angular_velocities;
		StridedPtr<const Float3> v_ptr = GetVelocities(), w_ptr = GetAngularVelocities();
		for (uint32 v = 0; v < num_vertices; ++v)
		{
			velocities.push_back(v_ptr[v]);
			angular_velocities.push_back(w_ptr[v]);
		}
		t.Record(cHSTagRead | 2, velocities.data(), velocities.size() * sizeof(Float3));
		t.Record(cHSTagRead | 3, angular_velocities.data(), angular_velocities.size() * sizeof(Float3));
		t.Record(cHSTagRead | 4, GetGridVelocityAndDensity(), mSettings->mNeutralDensity.size() * sizeof(Float4));
		t.Record(cHSTagRead | 5, GetRenderPositions(), mSettings->mRenderVertices.size() * sizeof(Float3));
		if (GetScalpVertices() != nullptr)
			t.Record(cHSTagRead | 6, GetScalpVertices(), mSettings->mScalpVertices.size() * sizeof(Float3));
		else
		{
			t.U32(cHSTagRead | 6);
			t.U32(0xFFFFFFFF);
		}

		UnlockReadBackBuffers();
	}
};

static Vec3 HSLoad3(const float *inV)
{
	return Vec3(inV[0], inV[1], inV[2]);
}

static RVec3 HSLoadR3(const double *inV)
{
	return RVec3(Real(inV[0]), Real(inV[1]), Real(inV[2]));
}

static Quat HSLoadQuat(const float *inV)
{
	return Quat(inV[0], inV[1], inV[2], inV[3]);
}

static Mat44 HSLoadMat44(const float *inM)
{
	return Mat44(Vec4(inM[0], inM[1], inM[2], inM[3]), Vec4(inM[4], inM[5], inM[6], inM[7]), Vec4(inM[8], inM[9], inM[10], inM[11]), Vec4(inM[12], inM[13], inM[14], inM[15]));
}

// MakeShapeSettings of HairSceneParity.zig
static Ref<ShapeSettings> HSMakeShapeSettings(const HSBodyDesc &inDesc, const float *inPoints)
{
	const float *p = inDesc.mParams;
	auto hull = [&inDesc, inPoints](float inMaxConvexRadius) {
		Array<Vec3> points;
		for (uint32 i = 0; i < inDesc.mNumPoints; ++i)
			points.push_back(HSLoad3(inPoints + 3 * (inDesc.mFirstPoint + i)));
		return new ConvexHullShapeSettings(points, inMaxConvexRadius);
	};
	switch (HSShape(inDesc.mShape))
	{
	case HSShape::Hull:
		return hull(p[0]);

	case HSShape::ScaledHull:
		return new ScaledShapeSettings(hull(0.0f), HSLoad3(p));

	case HSShape::StaticCompound:
		{
			StaticCompoundShapeSettings *compound = new StaticCompoundShapeSettings;
			compound->AddShape(HSLoad3(p), HSLoadQuat(p + 3), hull(0.0f));
			compound->AddShape(-HSLoad3(p), Quat::sIdentity(), new ScaledShapeSettings(hull(0.0f), HSLoad3(p + 7)));
			compound->AddShape(Vec3::sZero(), Quat::sIdentity(), new SphereShapeSettings(p[10]));
			return compound;
		}

	case HSShape::MutableCompound:
		{
			MutableCompoundShapeSettings *compound = new MutableCompoundShapeSettings;
			compound->AddShape(HSLoad3(p), Quat::sIdentity(), hull(0.0f));
			compound->AddShape(-HSLoad3(p), Quat::sIdentity(), new ScaledShapeSettings(hull(0.0f), HSLoad3(p + 3)));
			return compound;
		}

	case HSShape::RotatedTranslatedHull:
		return new RotatedTranslatedShapeSettings(HSLoad3(p), HSLoadQuat(p + 3), hull(0.0f));

	case HSShape::Box:
		return new BoxShapeSettings(HSLoad3(p));

	case HSShape::Sphere:
		return new SphereShapeSettings(p[0]);

	case HSShape::Capsule:
	default:
		return new CapsuleShapeSettings(p[0], p[1]);
	}
}

extern "C"
{

/// sizeof of the structs of this file (the Zig side checks its extern structs)
void jolt_hs_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(HSWorldDesc);
	outSizes[1] = sizeof(HSBodyDesc);
	outSizes[2] = sizeof(HSHairDesc);
	outSizes[3] = sizeof(HSOp);
}

/// Runs a scene (see the file header) and writes the transcript to outTranscript. Returns its size, 0xFFFFFFFF when it
/// does not fit, when the settings bytes do not restore or when a body cannot be created.
uint32 jolt_hs_run(const HSWorldDesc *inWorld, const HSBodyDesc *inBodies, const float *inPoints, const uint8 *inSettings, uint32 inSettingsSize, const HSHairDesc *inHair, const HSOp *inOps, uint32 inNumOps, const float *inPool, uint8 *outTranscript, uint32 inCapacity)
{
	ParityInstallHooks();
	jolt_libm_reference_linked();

	HSTranscript t(outTranscript, inCapacity);

	// The settings
	Ref<HairSettings> settings = new HairSettings;
	{
		std::stringstream data(std::string(reinterpret_cast<const char *>(inSettings), inSettingsSize));
		StreamInWrapper in(data);
		settings->RestoreBinaryState(in);
		if (in.IsFailed())
			return 0xFFFFFFFF;
	}

	// The compute system, the shaders and the queue
	Ref<ComputeSystem> compute_system = CreateComputeSystemCPU().Get();
	HairRegisterShaders(StaticCast<ComputeSystemCPU>(compute_system.GetPtr()));
	Ref<HairShaders> shaders = new HairShaders;
	shaders->Init(compute_system);
	Ref<ComputeQueue> queue = compute_system->CreateComputeQueue().Get();
	settings->InitCompute(compute_system);

	// The world (the layers of UnitTests/Layers.h)
	BPLayerInterfaceImpl layers;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broad_phase;
	ObjectLayerPairFilterImpl pairs;
	PhysicsSystem system;
	system.Init(256, 0, 256, 256, layers, object_vs_broad_phase, pairs);
	system.SetGravity(HSLoad3(inWorld->mGravity));
	BodyInterface &bi = system.GetBodyInterface();

	// The bodies
	Array<BodyID> bodies;
	for (uint32 i = 0; i < inWorld->mNumBodies; ++i)
	{
		const HSBodyDesc &d = inBodies[i];
		BodyCreationSettings s(HSMakeShapeSettings(d, inPoints), HSLoadR3(d.mPosition), HSLoadQuat(d.mRotation), EMotionType(d.mMotionType), ObjectLayer(d.mObjectLayer));
		s.mLinearVelocity = HSLoad3(d.mLinearVelocity);
		s.mAngularVelocity = HSLoad3(d.mAngularVelocity);
		s.mIsSensor = d.mIsSensor != 0;
		Body *body = bi.CreateBody(s);
		if (body == nullptr)
			return 0xFFFFFFFF;
		bi.AddBody(body->GetID(), d.mMotionType == uint32(EMotionType::Static)? EActivation::DontActivate : EActivation::Activate);
		bodies.push_back(body->GetID());
	}
	if (inWorld->mOptimizeBroadPhase != 0)
		system.OptimizeBroadPhase();

	// The hair
	HSHair hair(settings, HSLoadR3(inHair->mPosition), HSLoadQuat(inHair->mRotation), ObjectLayer(inHair->mObjectLayer));
	if (inHair->mExternalScalp != 0)
	{
		// The scalp vertices and triangles come from outside (Hair skips skinning the scalp)
		hair.SetScalpVerticesCB(compute_system->CreateComputeBuffer(ComputeBuffer::EType::Buffer, settings->mScalpVertices.size(), sizeof(Float3), settings->mScalpVertices.data()).Get());
		hair.SetScalpTrianglesCB(compute_system->CreateComputeBuffer(ComputeBuffer::EType::Buffer, settings->mScalpTriangles.size() * 3, sizeof(uint32), settings->mScalpTriangles.data()).Get());
	}
	hair.Init(compute_system);
	t.Record(cHSTagOp | 0xFFFFF, nullptr, 0);
	hair.Dump(t);

	// The operations
	for (uint32 op_index = 0; op_index < inNumOps; ++op_index)
	{
		const HSOp &op = inOps[op_index];
		switch (HSOpKind(op.mKind))
		{
		case HSOpKind::Update:
			{
				Array<Mat44> joints;
				if (op.mU[0] != 0xFFFFFFFF)
					for (uint32 j = 0; j < op.mU[2]; ++j)
						joints.push_back(HSLoadMat44(inPool + op.mU[0] + 16 * j));
				hair.Update(op.mF[0], HSLoadMat44(inPool + op.mU[1]), op.mU[0] != 0xFFFFFFFF? joints.data() : nullptr, system, *shaders, compute_system, queue);
			}
			break;

		case HSOpKind::SetPosition:
			hair.SetPosition(HSLoadR3(op.mD));
			break;

		case HSOpKind::SetRotation:
			hair.SetRotation(HSLoadQuat(op.mF));
			break;

		case HSOpKind::OnTeleported:
			hair.OnTeleported();
			break;

		case HSOpKind::MoveBody:
			bi.SetPositionRotationAndVelocity(bodies[op.mU[0]], HSLoadR3(op.mD), HSLoadQuat(op.mF), HSLoad3(op.mF + 4), HSLoad3(op.mF + 7));
			break;

		case HSOpKind::SetScalpToHead:
			hair.SetScalpToHead(HSLoadMat44(inPool + op.mU[0]));
			break;
		}

		t.Record(cHSTagOp | op_index, nullptr, 0);
		hair.Dump(t);
		if (HSOpKind(op.mKind) == HSOpKind::Update)
			hair.DumpReadBack(t, queue);
	}

	// Clean up
	for (BodyID id : bodies)
	{
		bi.RemoveBody(id);
		bi.DestroyBody(id);
	}

	return t.mOverflow? 0xFFFFFFFF : t.mSize;
}

} // extern "C"
