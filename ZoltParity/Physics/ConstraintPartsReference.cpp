// Reference implementation for the constraint part parity tests (`zig build parity`): C ABI wrappers around Jolt's
// SpringSettings, MotorSettings, CalculateSolverSteps and the 14 classes of Jolt/Physics/Constraints/ConstraintPart.
// ZoltParity/Physics/ConstraintPartsParity.zig calls these and checks that Zolt produces the same bits.
//
// Every body based scenario creates two bodies through a BodyManager (AllocateBody + AddBody, marked as being in the
// broad phase, activated when not static, like BodyReference.cpp) with a box shape, then runs a sequence of calls on a
// part and records after every call: the return value, the position, rotation and velocities of both bodies, the
// members of the part (read through the explicit instantiation trick below, like PairwiseReference.cpp) and the
// SaveState bytes. The record (`CPOutput`) is compared value by value with the one of the Zig side, which runs the same
// sequence. The parts are constructed in zeroed memory on both sides, so members that Jolt leaves uninitialized have the
// same (zero) bits. The structs below must match the extern structs in ConstraintPartsParity.zig. Booleans are passed as
// uint32 (never bool, see the porting guide).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/CalculateSolverSteps.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SpringPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AxisConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/ContactConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/DualAxisConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/PointConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/HingeRotationConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/RotationQuatConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/IndependentAxisConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/GearConstraintPart.h>
#include <Jolt/Physics/Constraints/ConstraintPart/RackAndPinionConstraintPart.h>

#include <algorithm>
#include <cstring>
#include <new>
#include <string>

using namespace JPH;

namespace
{

// Must match CPBodyDesc in ConstraintPartsParity.zig
struct CPBodyDesc
{
	Real				mPosition[3];
	float				mRotation[4];
	float				mLinearVelocity[3];
	float				mAngularVelocity[3];
	float				mHalfExtent[3];
	float				mMass;
	float				mInertiaMultiplier;
	float				mInertia[16];
	uint32				mMotionType;
	uint32				mAllowedDOFs;
	uint32				mOverrideMassProperties;
};

// Must match CPInput in ConstraintPartsParity.zig
struct CPInput
{
	CPBodyDesc			mBody[2];
	float				mF[64];
	uint32				mI[16];
};

static constexpr uint32 cMaxR = 256;
static constexpr uint32 cMaxF = 4096;
static constexpr uint32 cMaxI = 1024;

// Must match CPOutput in ConstraintPartsParity.zig
struct CPOutput
{
	Real				mR[cMaxR];
	float				mF[cMaxF];
	uint32				mI[cMaxI];
	uint32				mNumR;
	uint32				mNumF;
	uint32				mNumI;
};

// The allocator hooks and the Factory, created once for the whole parity binary (the same guard as the other wrapper
// files, so that RegisterTypes() runs only once)
static void CPEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static Vec3 CPLoad3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 CPLoad4(const float *inV)				{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Quat CPLoadQuat(const float *inV)			{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 CPLoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static Mat44 CPLoadMat44(const float *inM)			{ return Mat44(CPLoad4(inM), CPLoad4(inM + 4), CPLoad4(inM + 8), CPLoad4(inM + 12)); }

// Appends values to the output record (the counters keep counting past the capacity, the Zig side checks them)
class Rec
{
public:
	explicit			Rec(CPOutput *inOut) : mOut(inOut) { inOut->mNumR = inOut->mNumF = inOut->mNumI = 0; }

	void				R(Real inV)					{ if (mOut->mNumR < cMaxR) mOut->mR[mOut->mNumR] = inV; ++mOut->mNumR; }
	void				F(float inV)				{ if (mOut->mNumF < cMaxF) mOut->mF[mOut->mNumF] = inV; ++mOut->mNumF; }
	void				I(uint32 inV)				{ if (mOut->mNumI < cMaxI) mOut->mI[mOut->mNumI] = inV; ++mOut->mNumI; }
	void				B(bool inV)					{ I(inV? 1 : 0); }
	void				V3(Vec3Arg inV)				{ F(inV.GetX()); F(inV.GetY()); F(inV.GetZ()); }
	void				F3(const Float3 &inV)		{ F(inV.x); F(inV.y); F(inV.z); }
	void				Q(QuatArg inQ)				{ F(inQ.GetX()); F(inQ.GetY()); F(inQ.GetZ()); F(inQ.GetW()); }
	void				M44(Mat44Arg inM)			{ for (uint c = 0; c < 4; ++c) for (uint r = 0; r < 4; ++r) F(inM(r, c)); }
	void				V2(const Vector<2> &inV)	{ F(inV[0]); F(inV[1]); }
	void				M22(const Matrix<2, 2> &inM) { for (uint c = 0; c < 2; ++c) for (uint r = 0; r < 2; ++r) F(inM(r, c)); }

	void				Bytes(const void *inData, size_t inSize)
	{
		const float *f = static_cast<const float *>(inData);
		for (size_t i = 0; i < inSize / sizeof(float); ++i)
			F(f[i]);
	}

	void				BodyState(const Body &inBody)
	{
		RVec3 p = inBody.GetCenterOfMassPosition();
		R(p.GetX()); R(p.GetY()); R(p.GetZ());
		Q(inBody.GetRotation());
		V3(inBody.GetLinearVelocity());
		V3(inBody.GetAngularVelocity());
	}

	template <class T>
	void				State(const T &inPart)
	{
		StateRecorderImpl recorder;
		inPart.SaveState(recorder);
		std::string data = recorder.GetData();
		I(uint32(data.size()));
		for (char c : data)
			I(uint8(c));
	}

	// Start of a step: the step id, the return value and both bodies
	void				Step(uint32 inId, uint32 inResult, const Body &inBody1, const Body &inBody2)
	{
		I(0xC0DE0000 | inId);
		I(inResult);
		BodyState(inBody1);
		BodyState(inBody2);
	}

private:
	CPOutput *			mOut;
};

// Private access to the members of the parts (explicit instantiation ignores access checks)
template <typename Tag, typename Tag::type M>
struct CPRob
{
	friend typename Tag::type CPGet(Tag) { return M; }
};

#define CP_ROB(Name, Class, Type, Member) \
	struct Name { using type = Type Class::*; friend type CPGet(Name); }; \
	template struct CPRob<Name, &Class::Member>;

CP_ROB(SpringBias, SpringPart, float, mBias)
CP_ROB(SpringSoftness, SpringPart, float, mSoftness)

CP_ROB(AxisR1PlusUxAxis, AxisConstraintPart, Float3, mR1PlusUxAxis)
CP_ROB(AxisR2xAxis, AxisConstraintPart, Float3, mR2xAxis)
CP_ROB(AxisInvI1, AxisConstraintPart, Float3, mInvI1_R1PlusUxAxis)
CP_ROB(AxisInvI2, AxisConstraintPart, Float3, mInvI2_R2xAxis)
CP_ROB(AxisEffectiveMass, AxisConstraintPart, float, mEffectiveMass)
CP_ROB(AxisSpring, AxisConstraintPart, SpringPart, mSpringPart)

CP_ROB(DualR1PlusUxN1, DualAxisConstraintPart, Vec3, mR1PlusUxN1)
CP_ROB(DualR1PlusUxN2, DualAxisConstraintPart, Vec3, mR1PlusUxN2)
CP_ROB(DualR2xN1, DualAxisConstraintPart, Vec3, mR2xN1)
CP_ROB(DualR2xN2, DualAxisConstraintPart, Vec3, mR2xN2)
CP_ROB(DualInvI1N1, DualAxisConstraintPart, Vec3, mInvI1_R1PlusUxN1)
CP_ROB(DualInvI1N2, DualAxisConstraintPart, Vec3, mInvI1_R1PlusUxN2)
CP_ROB(DualInvI2N1, DualAxisConstraintPart, Vec3, mInvI2_R2xN1)
CP_ROB(DualInvI2N2, DualAxisConstraintPart, Vec3, mInvI2_R2xN2)
CP_ROB(DualEffectiveMass, DualAxisConstraintPart, DualAxisConstraintPart::Mat22, mEffectiveMass)

CP_ROB(AngleInvI1, AngleConstraintPart, Vec3, mInvI1_Axis)
CP_ROB(AngleInvI2, AngleConstraintPart, Vec3, mInvI2_Axis)
CP_ROB(AngleEffectiveMass, AngleConstraintPart, float, mEffectiveMass)
CP_ROB(AngleSpring, AngleConstraintPart, SpringPart, mSpringPart)

CP_ROB(PointR1, PointConstraintPart, Vec3, mR1)
CP_ROB(PointR2, PointConstraintPart, Vec3, mR2)
CP_ROB(PointInvI1, PointConstraintPart, Mat44, mInvI1_R1X)
CP_ROB(PointInvI2, PointConstraintPart, Mat44, mInvI2_R2X)
CP_ROB(PointEffectiveMass, PointConstraintPart, Mat44, mEffectiveMass)

CP_ROB(HingeA1, HingeRotationConstraintPart, Vec3, mA1)
CP_ROB(HingeB2, HingeRotationConstraintPart, Vec3, mB2)
CP_ROB(HingeC2, HingeRotationConstraintPart, Vec3, mC2)
CP_ROB(HingeInvI1, HingeRotationConstraintPart, Mat44, mInvI1)
CP_ROB(HingeInvI2, HingeRotationConstraintPart, Mat44, mInvI2)
CP_ROB(HingeB2xA1, HingeRotationConstraintPart, Vec3, mB2xA1)
CP_ROB(HingeC2xA1, HingeRotationConstraintPart, Vec3, mC2xA1)
CP_ROB(HingeEffectiveMass, HingeRotationConstraintPart, HingeRotationConstraintPart::Mat22, mEffectiveMass)

CP_ROB(EulerInvI1, RotationEulerConstraintPart, Mat44, mInvI1)
CP_ROB(EulerInvI2, RotationEulerConstraintPart, Mat44, mInvI2)
CP_ROB(EulerEffectiveMass, RotationEulerConstraintPart, Mat44, mEffectiveMass)

CP_ROB(QuatInvI1JPT, RotationQuatConstraintPart, Mat44, mInvI1_JPT)
CP_ROB(QuatInvI2JPT, RotationQuatConstraintPart, Mat44, mInvI2_JPT)
CP_ROB(QuatEffectiveMass, RotationQuatConstraintPart, Mat44, mEffectiveMass)
CP_ROB(QuatEffectiveMassJP, RotationQuatConstraintPart, Mat44, mEffectiveMass_JP)

CP_ROB(STRotationFlags, SwingTwistConstraintPart, uint8, mRotationFlags)
CP_ROB(STSinTwistMin, SwingTwistConstraintPart, float, mSinTwistHalfMinAngle)
CP_ROB(STSinTwistMax, SwingTwistConstraintPart, float, mSinTwistHalfMaxAngle)
CP_ROB(STCosTwistMin, SwingTwistConstraintPart, float, mCosTwistHalfMinAngle)
CP_ROB(STCosTwistMax, SwingTwistConstraintPart, float, mCosTwistHalfMaxAngle)
CP_ROB(STSwingYMin, SwingTwistConstraintPart, float, mSwingYHalfMinAngle)
CP_ROB(STSwingYMax, SwingTwistConstraintPart, float, mSwingYHalfMaxAngle)
CP_ROB(STSwingZMin, SwingTwistConstraintPart, float, mSwingZHalfMinAngle)
CP_ROB(STSwingZMax, SwingTwistConstraintPart, float, mSwingZHalfMaxAngle)
CP_ROB(STSinSwingYMin, SwingTwistConstraintPart, float, mSinSwingYHalfMinAngle)
CP_ROB(STSinSwingYMax, SwingTwistConstraintPart, float, mSinSwingYHalfMaxAngle)
CP_ROB(STSinSwingZMin, SwingTwistConstraintPart, float, mSinSwingZHalfMinAngle)
CP_ROB(STSinSwingZMax, SwingTwistConstraintPart, float, mSinSwingZHalfMaxAngle)
CP_ROB(STCosSwingYMin, SwingTwistConstraintPart, float, mCosSwingYHalfMinAngle)
CP_ROB(STCosSwingYMax, SwingTwistConstraintPart, float, mCosSwingYHalfMaxAngle)
CP_ROB(STCosSwingZMin, SwingTwistConstraintPart, float, mCosSwingZHalfMinAngle)
CP_ROB(STCosSwingZMax, SwingTwistConstraintPart, float, mCosSwingZHalfMaxAngle)
CP_ROB(STSwingYAxis, SwingTwistConstraintPart, Vec3, mWorldSpaceSwingLimitYRotationAxis)
CP_ROB(STSwingZAxis, SwingTwistConstraintPart, Vec3, mWorldSpaceSwingLimitZRotationAxis)
CP_ROB(STTwistAxis, SwingTwistConstraintPart, Vec3, mWorldSpaceTwistLimitRotationAxis)
CP_ROB(STSwingYPart, SwingTwistConstraintPart, AngleConstraintPart, mSwingLimitYConstraintPart)
CP_ROB(STSwingZPart, SwingTwistConstraintPart, AngleConstraintPart, mSwingLimitZConstraintPart)
CP_ROB(STTwistPart, SwingTwistConstraintPart, AngleConstraintPart, mTwistLimitConstraintPart)

CP_ROB(IndR1xN1, IndependentAxisConstraintPart, Vec3, mR1xN1)
CP_ROB(IndInvI1, IndependentAxisConstraintPart, Vec3, mInvI1_R1xN1)
CP_ROB(IndRatioR2xN2, IndependentAxisConstraintPart, Vec3, mRatioR2xN2)
CP_ROB(IndInvI2, IndependentAxisConstraintPart, Vec3, mInvI2_RatioR2xN2)
CP_ROB(IndEffectiveMass, IndependentAxisConstraintPart, float, mEffectiveMass)

CP_ROB(GearInvI1, GearConstraintPart, Vec3, mInvI1_A)
CP_ROB(GearInvI2, GearConstraintPart, Vec3, mInvI2_B)
CP_ROB(GearEffectiveMass, GearConstraintPart, float, mEffectiveMass)

CP_ROB(RackInvI1, RackAndPinionConstraintPart, Vec3, mInvI1_A)
CP_ROB(RackRatioInvM2, RackAndPinionConstraintPart, Vec3, mRatio_InvM2_B)
CP_ROB(RackEffectiveMass, RackAndPinionConstraintPart, float, mEffectiveMass)

#undef CP_ROB

// Broad phase layer = object layer
class CPLayers final : public BroadPhaseLayerInterface
{
public:
	virtual uint				GetNumBroadPhaseLayers() const override						{ return 1; }
	virtual BroadPhaseLayer		GetBroadPhaseLayer(ObjectLayer inLayer) const override		{ return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *		GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

struct CPWorld
{
	CPLayers					mLayers;
	BodyManager					mBodyManager;
};

static Body *CreateBody(BodyManager &ioBodyManager, const CPBodyDesc &inDesc)
{
	RefConst<Shape> shape = new BoxShape(CPLoad3(inDesc.mHalfExtent));
	BodyCreationSettings settings(shape, CPLoadR3(inDesc.mPosition), CPLoadQuat(inDesc.mRotation), EMotionType(inDesc.mMotionType), 0);
	settings.mLinearVelocity = CPLoad3(inDesc.mLinearVelocity);
	settings.mAngularVelocity = CPLoad3(inDesc.mAngularVelocity);
	settings.mAllowedDOFs = EAllowedDOFs(inDesc.mAllowedDOFs);
	settings.mOverrideMassProperties = EOverrideMassProperties(inDesc.mOverrideMassProperties);
	settings.mInertiaMultiplier = inDesc.mInertiaMultiplier;
	settings.mMassPropertiesOverride.mMass = inDesc.mMass;
	settings.mMassPropertiesOverride.mInertia = CPLoadMat44(inDesc.mInertia);
	Body *body = ioBodyManager.AllocateBody(settings);
	bool added = ioBodyManager.AddBody(body);
	JPH_ASSERT(added);
	(void)added;
	body->SetInBroadPhaseInternal(true);
	if (!body->IsStatic())
	{
		BodyID id = body->GetID();
		ioBodyManager.ActivateBodies(&id, 1);
	}
	return body;
}

static void DestroyBody(BodyManager &ioBodyManager, Body *inBody)
{
	BodyID id = inBody->GetID();
	ioBodyManager.DeactivateBodies(&id, 1);
	inBody->SetInBroadPhaseInternal(false);
	ioBodyManager.DestroyBodies(&id, 1);
}

// The two bodies of a scenario, destroyed at the end
class BodyPairScope
{
public:
						BodyPairScope(void *inWorld, const CPInput *inInput) : mBodyManager(static_cast<CPWorld *>(inWorld)->mBodyManager)
	{
		mBody1 = CreateBody(mBodyManager, inInput->mBody[0]);
		mBody2 = CreateBody(mBodyManager, inInput->mBody[1]);
	}

						~BodyPairScope()
	{
		DestroyBody(mBodyManager, mBody2);
		DestroyBody(mBodyManager, mBody1);
	}

	BodyManager &		mBodyManager;
	Body *				mBody1;
	Body *				mBody2;
};

// A part constructed in zeroed memory
template <class T>
class ZeroedPart
{
public:
						ZeroedPart()				{ memset(mStorage, 0, sizeof(mStorage)); mPart = new (mStorage) T; }
						~ZeroedPart()				{ mPart->~T(); }

	T &					operator * ()				{ return *mPart; }
	T *					operator -> ()				{ return mPart; }

private:
	alignas(T) uint8	mStorage[sizeof(T)];
	T *					mPart;
};

template <class T>
static void RestoreFloats(T &ioPart, const float *inValues, uint inCount)
{
	StateRecorderImpl recorder;
	for (uint i = 0; i < inCount; ++i)
		recorder.Write(inValues[i]);
	ioPart.RestoreState(recorder);
}

static void DumpSpring(Rec &ioRec, const SpringPart &inSpring)
{
	ioRec.F(inSpring.*CPGet(SpringBias()));
	ioRec.F(inSpring.*CPGet(SpringSoftness()));
	ioRec.B(inSpring.IsActive());
}

static void DumpAxis(Rec &ioRec, AxisConstraintPart &inPart, const Body &inBody1, const Body &inBody2)
{
	if (!inBody1.IsStatic()) ioRec.F3(inPart.*CPGet(AxisR1PlusUxAxis()));
	if (!inBody2.IsStatic()) ioRec.F3(inPart.*CPGet(AxisR2xAxis()));
	if (inBody1.IsDynamic()) ioRec.F3(inPart.*CPGet(AxisInvI1()));
	if (inBody2.IsDynamic()) ioRec.F3(inPart.*CPGet(AxisInvI2()));
	ioRec.F(inPart.*CPGet(AxisEffectiveMass()));
	DumpSpring(ioRec, inPart.*CPGet(AxisSpring()));
	ioRec.F(inPart.GetTotalLambda());
	ioRec.B(inPart.IsActive());
	ioRec.State(inPart);
}

static void DumpAngle(Rec &ioRec, AngleConstraintPart &inPart)
{
	ioRec.V3(inPart.*CPGet(AngleInvI1()));
	ioRec.V3(inPart.*CPGet(AngleInvI2()));
	ioRec.F(inPart.*CPGet(AngleEffectiveMass()));
	DumpSpring(ioRec, inPart.*CPGet(AngleSpring()));
	ioRec.F(inPart.GetTotalLambda());
	ioRec.B(inPart.IsActive());
	ioRec.State(inPart);
}

// Scenario for the axis and angle parts: restore lambda, calculate (overload), warm start, velocity and position steps, deactivate
template <class Part, class CalcFn, class WarmStartFn, class SolveVelFn, class SolvePosFn, class DumpFn>
static void RunSpringScenario(Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput, uint inFirst, CalcFn &&inCalc, WarmStartFn &&inWarmStart, SolveVelFn &&inSolveVel, SolvePosFn &&inSolvePos, DumpFn &&inDump)
{
	const float *f = inInput->mF + inFirst;
	const uint32 *ui = inInput->mI;

	ZeroedPart<Part> part;
	RestoreFloats(*part, f + 9, 1);
	ioRec.Step(0, 0, ioBody1, ioBody2);
	inDump(*part);

	inCalc(*part, ui[0], f[4] /* dt */, f[0] /* bias */, f[1] /* C */, f[2] /* frequency / stiffness */, f[3] /* damping */, SpringSettings(ESpringMode(ui[1]), f[2], f[3]));
	ioRec.Step(1, 0, ioBody1, ioBody2);
	inDump(*part);

	if (ui[4] != 0)
		return; // Static vs static: only the properties

	inWarmStart(*part, f[5]);
	ioRec.Step(2, 0, ioBody1, ioBody2);
	inDump(*part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = inSolveVel(*part, f[6], f[7]);
		ioRec.Step(3, result? 1 : 0, ioBody1, ioBody2);
		inDump(*part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = inSolvePos(*part, i == 0? f[8] : f[10], f[11]);
		ioRec.Step(4, result? 1 : 0, ioBody1, ioBody2);
		inDump(*part);
	}

	part->Deactivate();
	ioRec.Step(5, 0, ioBody1, ioBody2);
	inDump(*part);
}

// Contact constraint part scenario for motion types Type1, Type2
template <EMotionType Type1, EMotionType Type2>
static void RunContact(Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput)
{
	using Part = ContactConstraintPart<Type1, Type2>;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;

	float inv_m1 = f[0], inv_m2 = f[17];
	Mat44 inv_i1 = CPLoadMat44(f + 1), inv_i2 = CPLoadMat44(f + 18);
	Vec3 r1_plus_u = CPLoad3(f + 34), r2 = CPLoad3(f + 37), axis = CPLoad3(f + 40);
	Vec3 v1 = ioBody1.GetLinearVelocity(), w1 = ioBody1.GetAngularVelocity(), v2 = ioBody2.GetLinearVelocity(), w2 = ioBody2.GetAngularVelocity();

	auto dump = [&](uint32 inId, uint32 inResult, Part &inPart) {
		ioRec.Step(inId, inResult, ioBody1, ioBody2);
		ioRec.V3(v1); ioRec.V3(w1); ioRec.V3(v2); ioRec.V3(w2);
		ioRec.Bytes(&inPart, sizeof(Part));
		ioRec.F(inPart.GetTotalLambda());
		ioRec.B(inPart.IsActive());
	};

	ZeroedPart<Part> part;
	part->SetTotalLambda(f[44]);
	dump(0, 0, *part);

	if (ui[0] != 0)
		part->CalculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, axis, f[43]);
	else
		part->CalculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, axis);
	dump(1, 0, *part);

	bool result = part->WarmStart(v1, w1, v2, w2, inv_m1, inv_m2, axis, f[45]);
	dump(2, result? 1 : 0, *part);

	if constexpr (Type1 != EMotionType::Static || Type2 != EMotionType::Static)
	{
		for (uint32 i = 0; i < ui[2]; ++i)
		{
			if (ui[5] != 0)
			{
				float total_lambda = part->SolveVelocityConstraintGetTotalLambda(v1, w1, v2, w2, axis);
				ioRec.F(total_lambda);
				total_lambda = Clamp(total_lambda, f[46], f[47]);
				result = part->SolveVelocityConstraintApplyLambda(v1, w1, v2, w2, inv_m1, inv_m2, axis, total_lambda);
			}
			else
				result = part->SolveVelocityConstraint(v1, w1, v2, w2, inv_m1, inv_m2, axis, f[46], f[47]);
			dump(3, result? 1 : 0, *part);
		}
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		result = part->SolvePositionConstraint(ioBody1, inv_m1, ioBody2, inv_m2, axis, i == 0? f[48] : f[50], f[49]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// Angular friction constraint part scenario for motion types Type1, Type2
template <EMotionType Type1, EMotionType Type2>
static void RunAngularFriction(Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput)
{
	using Part = AngularFrictionConstraintPart<Type1, Type2>;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;

	Mat44 inv_i1 = CPLoadMat44(f), inv_i2 = CPLoadMat44(f + 16);
	Vec3 axis = CPLoad3(f + 32);
	Vec3 w1 = ioBody1.GetAngularVelocity(), w2 = ioBody2.GetAngularVelocity();

	auto dump = [&](uint32 inId, uint32 inResult, Part &inPart) {
		ioRec.Step(inId, inResult, ioBody1, ioBody2);
		ioRec.V3(w1); ioRec.V3(w2);
		ioRec.Bytes(&inPart, sizeof(Part));
		ioRec.F(inPart.GetTotalLambda());
		ioRec.B(inPart.IsActive());
	};

	ZeroedPart<Part> part;
	part->SetTotalLambda(f[36]);
	dump(0, 0, *part);

	if (ui[0] != 0)
		part->CalculateConstraintProperties(inv_i1, inv_i2, axis, f[35]);
	else
		part->CalculateConstraintProperties(inv_i1, inv_i2, axis);
	dump(1, 0, *part);

	bool result = part->WarmStart(w1, w2, f[37]);
	dump(2, result? 1 : 0, *part);

	if constexpr (Type1 != EMotionType::Static || Type2 != EMotionType::Static)
	{
		for (uint32 i = 0; i < ui[2]; ++i)
		{
			result = part->SolveVelocityConstraint(w1, w2, axis, f[38], f[39]);
			dump(3, result? 1 : 0, *part);
		}
	}

	part->Deactivate();
	dump(5, 0, *part);
}

template <template <EMotionType, EMotionType> class Runner>
struct MotionTypeDispatch
{
	template <EMotionType Type1>
	static void Second(EMotionType inType2, Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput)
	{
		switch (inType2)
		{
		case EMotionType::Static:		Runner<Type1, EMotionType::Static>::Run(ioRec, ioBody1, ioBody2, inInput); break;
		case EMotionType::Kinematic:	Runner<Type1, EMotionType::Kinematic>::Run(ioRec, ioBody1, ioBody2, inInput); break;
		case EMotionType::Dynamic:		Runner<Type1, EMotionType::Dynamic>::Run(ioRec, ioBody1, ioBody2, inInput); break;
		}
	}

	static void Run(Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput)
	{
		EMotionType type2 = ioBody2.GetMotionType();
		switch (ioBody1.GetMotionType())
		{
		case EMotionType::Static:		Second<EMotionType::Static>(type2, ioRec, ioBody1, ioBody2, inInput); break;
		case EMotionType::Kinematic:	Second<EMotionType::Kinematic>(type2, ioRec, ioBody1, ioBody2, inInput); break;
		case EMotionType::Dynamic:		Second<EMotionType::Dynamic>(type2, ioRec, ioBody1, ioBody2, inInput); break;
		}
	}
};

template <EMotionType Type1, EMotionType Type2>
struct ContactRunner
{
	static void Run(Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput) { RunContact<Type1, Type2>(ioRec, ioBody1, ioBody2, inInput); }
};

template <EMotionType Type1, EMotionType Type2>
struct AngularFrictionRunner
{
	static void Run(Rec &ioRec, Body &ioBody1, Body &ioBody2, const CPInput *inInput) { RunAngularFriction<Type1, Type2>(ioRec, ioBody1, ioBody2, inInput); }
};

static void DumpSwingTwist(Rec &ioRec, SwingTwistConstraintPart &inPart)
{
	ioRec.I(inPart.*CPGet(STRotationFlags()));
	ioRec.I(uint32(inPart.GetSwingType()));
	ioRec.F(inPart.*CPGet(STSinTwistMin()));
	ioRec.F(inPart.*CPGet(STSinTwistMax()));
	ioRec.F(inPart.*CPGet(STCosTwistMin()));
	ioRec.F(inPart.*CPGet(STCosTwistMax()));
	ioRec.F(inPart.*CPGet(STSwingYMin()));
	ioRec.F(inPart.*CPGet(STSwingYMax()));
	ioRec.F(inPart.*CPGet(STSwingZMin()));
	ioRec.F(inPart.*CPGet(STSwingZMax()));
	ioRec.F(inPart.*CPGet(STSinSwingYMin()));
	ioRec.F(inPart.*CPGet(STSinSwingYMax()));
	ioRec.F(inPart.*CPGet(STSinSwingZMin()));
	ioRec.F(inPart.*CPGet(STSinSwingZMax()));
	ioRec.F(inPart.*CPGet(STCosSwingYMin()));
	ioRec.F(inPart.*CPGet(STCosSwingYMax()));
	ioRec.F(inPart.*CPGet(STCosSwingZMin()));
	ioRec.F(inPart.*CPGet(STCosSwingZMax()));
	ioRec.V3(inPart.*CPGet(STSwingYAxis()));
	ioRec.V3(inPart.*CPGet(STSwingZAxis()));
	ioRec.V3(inPart.*CPGet(STTwistAxis()));
	DumpAngle(ioRec, inPart.*CPGet(STSwingYPart()));
	DumpAngle(ioRec, inPart.*CPGet(STSwingZPart()));
	DumpAngle(ioRec, inPart.*CPGet(STTwistPart()));
	ioRec.F(inPart.GetTotalSwingYLambda());
	ioRec.F(inPart.GetTotalSwingZLambda());
	ioRec.F(inPart.GetTotalTwistLambda());
	ioRec.B(inPart.IsActive());
	ioRec.State(inPart);
}

// Object with step overrides for CalculateSolverSteps
struct StepsObject
{
	uint				GetNumVelocityStepsOverride() const		{ return mVelocity; }
	uint				GetNumPositionStepsOverride() const		{ return mPosition; }

	uint				mVelocity;
	uint				mPosition;
};

} // namespace

extern "C"
{

void *jolt_cp_world_create()
{
	CPEnsureFactory();
	CPWorld *world = new CPWorld;
	world->mBodyManager.Init(8, 0, world->mLayers);
	return world;
}

void jolt_cp_world_destroy(void *inWorld)
{
	delete static_cast<CPWorld *>(inWorld);
}

// SpringPart: inF = dt, inv effective mass, bias, C, frequency / stiffness, damping, total lambda; inOp selects the
// function (0 bias, 1 stiffness and damping, 2 frequency and damping, 3 mass normalized, 4 settings with inMode)
void jolt_cp_spring_part(const float *inF, uint32 inOp, uint32 inMode, float *outF)
{
	CPEnsureFactory();
	SpringPart part;
	float effective_mass = -12345.0f;
	switch (inOp)
	{
	case 0:		part.CalculateSpringPropertiesWithBias(inF[2]); break;
	case 1:		part.CalculateSpringPropertiesWithStiffnessAndDamping(inF[0], inF[1], inF[2], inF[3], inF[4], inF[5], effective_mass); break;
	case 2:		part.CalculateSpringPropertiesWithFrequencyAndDamping(inF[0], inF[1], inF[2], inF[3], inF[4], inF[5], effective_mass); break;
	case 3:		part.CalculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(inF[0], inF[1], inF[2], inF[3], inF[4], inF[5], effective_mass); break;
	default:	part.CalculateSpringPropertiesWithSettings(inF[0], inF[1], inF[2], inF[3], SpringSettings(ESpringMode(inMode), inF[4], inF[5]), effective_mass); break;
	}
	outF[0] = effective_mass;
	outF[1] = part.*CPGet(SpringBias());
	outF[2] = part.*CPGet(SpringSoftness());
	outF[3] = part.GetBias(inF[6]);
	outF[4] = part.IsActive()? 1.0f : 0.0f;
}

// SpringSettings: save the settings, the checks, and restore from inRestore (9 bytes). outBytes gets 9 bytes,
// outInfo: HasStiffness, HasStiffnessOrDamping, restored mode; outRestored: frequency, damping
void jolt_cp_spring_settings(uint32 inMode, float inFrequency, float inDamping, const uint8 *inRestore, uint8 *outBytes, uint32 *outInfo, float *outRestored)
{
	CPEnsureFactory();
	SpringSettings settings(ESpringMode(inMode), inFrequency, inDamping);
	StateRecorderImpl recorder;
	settings.SaveBinaryState(recorder);
	std::string data = recorder.GetData();
	memcpy(outBytes, data.data(), std::min<size_t>(data.size(), 9));
	outInfo[0] = settings.HasStiffness()? 1 : 0;
	outInfo[1] = settings.HasStiffnessOrDamping()? 1 : 0;
	outInfo[3] = uint32(data.size());

	StateRecorderImpl restore;
	restore.WriteBytes(inRestore, 9);
	SpringSettings restored;
	restored.RestoreBinaryState(restore);
	outInfo[2] = uint32(restored.mMode);
	outRestored[0] = restored.mFrequency;
	outRestored[1] = restored.mDamping;
	outRestored[2] = restored.mStiffness;
}

// MotorSettings: inConstructor selects the constructor (0 default, 1 (f, d), 2 (mode, f, d), 3 (f, d, force, torque),
// 4 (mode, f, d, force, torque)), inSetter the setter (0 none, 1 SetForceLimits, 2 SetTorqueLimits, 3 SetForceLimit,
// 4 SetTorqueLimit) with inF[5], inF[6]. outBytes: 25 bytes of SaveBinaryState; outF: the fields (7) of the settings
// and of the settings restored from inRestore (25 bytes); outInfo: IsValid, mode, restored mode, state size
void jolt_cp_motor_settings(uint32 inConstructor, uint32 inMode, const float *inF, uint32 inSetter, const uint8 *inRestore, uint8 *outBytes, float *outF, uint32 *outInfo)
{
	CPEnsureFactory();
	MotorSettings settings;
	switch (inConstructor)
	{
	case 0:		break;
	case 1:		settings = MotorSettings(inF[0], inF[1]); break;
	case 2:		settings = MotorSettings(ESpringMode(inMode), inF[0], inF[1]); break;
	case 3:		settings = MotorSettings(inF[0], inF[1], inF[2], inF[3]); break;
	default:	settings = MotorSettings(ESpringMode(inMode), inF[0], inF[1], inF[2], inF[3]); break;
	}
	switch (inSetter)
	{
	case 0:		break;
	case 1:		settings.SetForceLimits(inF[5], inF[6]); break;
	case 2:		settings.SetTorqueLimits(inF[5], inF[6]); break;
	case 3:		settings.SetForceLimit(inF[5]); break;
	default:	settings.SetTorqueLimit(inF[5]); break;
	}

	StateRecorderImpl recorder;
	settings.SaveBinaryState(recorder);
	std::string data = recorder.GetData();
	memcpy(outBytes, data.data(), std::min<size_t>(data.size(), 25));

	auto fields = [](const MotorSettings &inSettings, float *outFields) {
		outFields[0] = inSettings.mSpringSettings.mFrequency;
		outFields[1] = inSettings.mSpringSettings.mDamping;
		outFields[2] = inSettings.mMinForceLimit;
		outFields[3] = inSettings.mMaxForceLimit;
		outFields[4] = inSettings.mMinTorqueLimit;
		outFields[5] = inSettings.mMaxTorqueLimit;
		outFields[6] = inSettings.mSpringSettings.mStiffness;
	};
	fields(settings, outF);

	StateRecorderImpl restore;
	restore.WriteBytes(inRestore, 25);
	MotorSettings restored;
	restored.RestoreBinaryState(restore);
	fields(restored, outF + 7);

	outInfo[0] = settings.IsValid()? 1 : 0;
	outInfo[1] = uint32(settings.mSpringSettings.mMode);
	outInfo[2] = uint32(restored.mSpringSettings.mMode);
	outInfo[3] = uint32(data.size());
	outInfo[4] = restored.IsValid()? 1 : 0;
}

// IsVelocityMotor / IsPositionMotor for a motor state
uint32 jolt_cp_motor_state(uint32 inState)
{
	EMotorState state = EMotorState(inState);
	return (IsVelocityMotor(state)? 1 : 0) | (IsPositionMotor(state)? 2 : 0);
}

// CalculateSolverSteps over inCount objects (velocity, position overrides), outSteps: velocity, position steps before
// and after Finalize
void jolt_cp_solver_steps(uint32 inDefaultVelocity, uint32 inDefaultPosition, const uint32 *inOverrides, uint32 inCount, uint32 *outSteps)
{
	PhysicsSettings settings;
	settings.mNumVelocitySteps = inDefaultVelocity;
	settings.mNumPositionSteps = inDefaultPosition;
	CalculateSolverSteps calculator(settings);
	DummyCalculateSolverSteps dummy;
	for (uint32 i = 0; i < inCount; ++i)
	{
		StepsObject object { inOverrides[2 * i], inOverrides[2 * i + 1] };
		calculator(&object);
		dummy(&object);
	}
	outSteps[0] = calculator.GetNumVelocitySteps();
	outSteps[1] = calculator.GetNumPositionSteps();
	calculator.Finalize();
	outSteps[2] = calculator.GetNumVelocitySteps();
	outSteps[3] = calculator.GetNumPositionSteps();
}

// AxisConstraintPart. mF: r1 + u (3), r2 (3), axis (3), then the spring scenario values at 9 (bias, C, frequency /
// stiffness, damping, dt, warm start ratio, min lambda, max lambda, C for the first position step, initial lambda, C
// for the next position steps, baumgarte). mI: calculate overload, spring mode, velocity steps, position steps, static
// vs static
void jolt_cp_axis(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	Vec3 r1_plus_u = CPLoad3(f), r2 = CPLoad3(f + 3), axis = CPLoad3(f + 6);

	RunSpringScenario<AxisConstraintPart>(rec, body1, body2, inInput, 9,
		[&](AxisConstraintPart &ioPart, uint32 inOverload, float inDeltaTime, float inBias, float inC, float inFrequency, float inDamping, const SpringSettings &inSettings) {
			switch (inOverload)
			{
			case 0:		ioPart.CalculateConstraintProperties(body1, r1_plus_u, body2, r2, axis, inBias); break;
			case 1:		ioPart.CalculateConstraintPropertiesWithFrequencyAndDamping(inDeltaTime, body1, r1_plus_u, body2, r2, axis, inBias, inC, inFrequency, inDamping); break;
			case 2:		ioPart.CalculateConstraintPropertiesWithStiffnessAndDamping(inDeltaTime, body1, r1_plus_u, body2, r2, axis, inBias, inC, inFrequency, inDamping); break;
			case 3:		ioPart.CalculateConstraintPropertiesWithSettingsForLimit(inDeltaTime, body1, r1_plus_u, body2, r2, axis, inBias, inC, inSettings); break;
			case 4:		ioPart.CalculateConstraintPropertiesWithSettingsForMotor(inDeltaTime, body1, r1_plus_u, body2, r2, axis, inBias, inC, inSettings); break;
			default:	ioPart.CalculateConstraintProperties(body1, r1_plus_u, body2, r2, axis); break;
			}
		},
		[&](AxisConstraintPart &ioPart, float inRatio) { ioPart.WarmStart(body1, body2, axis, inRatio); },
		[&](AxisConstraintPart &ioPart, float inMin, float inMax) { return ioPart.SolveVelocityConstraint(body1, body2, axis, inMin, inMax); },
		[&](AxisConstraintPart &ioPart, float inC, float inBaumgarte) { return ioPart.SolvePositionConstraint(body1, body2, axis, inC, inBaumgarte); },
		[&](AxisConstraintPart &inPart) { DumpAxis(rec, inPart, body1, body2); });
}

// AngleConstraintPart. mF: axis (3), then the spring scenario values at 3 (see jolt_cp_axis)
void jolt_cp_angle(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	Vec3 axis = CPLoad3(inInput->mF);

	RunSpringScenario<AngleConstraintPart>(rec, body1, body2, inInput, 3,
		[&](AngleConstraintPart &ioPart, uint32 inOverload, float inDeltaTime, float inBias, float inC, float inFrequency, float inDamping, const SpringSettings &inSettings) {
			switch (inOverload)
			{
			case 0:		ioPart.CalculateConstraintProperties(body1, body2, axis, inBias); break;
			case 1:		ioPart.CalculateConstraintPropertiesWithFrequencyAndDamping(inDeltaTime, body1, body2, axis, inBias, inC, inFrequency, inDamping); break;
			case 2:		ioPart.CalculateConstraintPropertiesWithStiffnessAndDamping(inDeltaTime, body1, body2, axis, inBias, inC, inFrequency, inDamping); break;
			case 3:		ioPart.CalculateConstraintPropertiesWithSettingsForLimit(inDeltaTime, body1, body2, axis, inBias, inC, inSettings); break;
			case 4:		ioPart.CalculateConstraintPropertiesWithSettingsForMotor(inDeltaTime, body1, body2, axis, inBias, inC, inSettings); break;
			default:	ioPart.CalculateConstraintProperties(body1, body2, axis); break;
			}
		},
		[&](AngleConstraintPart &ioPart, float inRatio) { ioPart.WarmStart(body1, body2, inRatio); },
		[&](AngleConstraintPart &ioPart, float inMin, float inMax) { return ioPart.SolveVelocityConstraint(body1, body2, axis, inMin, inMax); },
		[&](AngleConstraintPart &ioPart, float inC, float inBaumgarte) { return ioPart.SolvePositionConstraint(body1, body2, inC, inBaumgarte); },
		[&](AngleConstraintPart &inPart) { DumpAngle(rec, inPart); });
}

// ContactConstraintPart<Type1, Type2> with the motion types of the bodies. mF: inv mass 1, inv inertia 1 (16), inv mass
// 2, inv inertia 2 (16), r1 + u, r2, axis, bias, initial lambda, warm start ratio, min lambda, max lambda, C for the
// first position step, baumgarte, C for the next position steps. mI: bias given, -, velocity steps, position steps, -,
// split velocity step
void jolt_cp_contact(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	MotionTypeDispatch<ContactRunner>::Run(rec, *bodies.mBody1, *bodies.mBody2, inInput);
}

// AngularFrictionConstraintPart<Type1, Type2> with the motion types of the bodies. mF: inv inertia 1 (16), inv inertia 2
// (16), axis, bias, initial lambda, warm start ratio, min lambda, max lambda. mI: bias given, -, velocity steps
void jolt_cp_angular_friction(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	MotionTypeDispatch<AngularFrictionRunner>::Run(rec, *bodies.mBody1, *bodies.mBody2, inInput);
}

// DualAxisConstraintPart. mF: r1 + u, r2, n1, n2, initial lambda (2), warm start ratio, u, baumgarte, u for the next
// position steps. mI: -, -, velocity steps, position steps
void jolt_cp_dual_axis(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;
	Vec3 r1_plus_u = CPLoad3(f), r2 = CPLoad3(f + 3), n1 = CPLoad3(f + 6), n2 = CPLoad3(f + 9);

	auto dump = [&](uint32 inId, uint32 inResult, DualAxisConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		rec.V3(inPart.*CPGet(DualR1PlusUxN1()));
		rec.V3(inPart.*CPGet(DualR1PlusUxN2()));
		rec.V3(inPart.*CPGet(DualR2xN1()));
		rec.V3(inPart.*CPGet(DualR2xN2()));
		if (body1.IsDynamic()) { rec.V3(inPart.*CPGet(DualInvI1N1())); rec.V3(inPart.*CPGet(DualInvI1N2())); }
		if (body2.IsDynamic()) { rec.V3(inPart.*CPGet(DualInvI2N1())); rec.V3(inPart.*CPGet(DualInvI2N2())); }
		rec.M22(inPart.*CPGet(DualEffectiveMass()));
		rec.V2(inPart.GetTotalLambda());
		rec.B(inPart.IsActive());
		rec.State(inPart);
	};

	ZeroedPart<DualAxisConstraintPart> part;
	DualAxisConstraintPart::Vec2 lambda;
	lambda[0] = f[12];
	lambda[1] = f[13];
	part->SetTotalLambda(lambda);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, Mat44::sRotation(body1.GetRotation()), r1_plus_u, body2, Mat44::sRotation(body2.GetRotation()), r2, n1, n2);
	dump(1, 0, *part);

	part->WarmStart(body1, body2, n1, n2, f[14]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2, n1, n2);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, CPLoad3(i == 0? f + 15 : f + 19), n1, n2, f[18]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// PointConstraintPart. mF: r1, r2, initial lambda (3), warm start ratio, baumgarte. mI: -, -, velocity steps, position steps
void jolt_cp_point(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;

	auto dump = [&](uint32 inId, uint32 inResult, PointConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		rec.V3(inPart.*CPGet(PointR1()));
		rec.V3(inPart.*CPGet(PointR2()));
		if (body1.IsDynamic()) rec.M44(inPart.*CPGet(PointInvI1()));
		if (body2.IsDynamic()) rec.M44(inPart.*CPGet(PointInvI2()));
		rec.M44(inPart.*CPGet(PointEffectiveMass()));
		rec.V3(inPart.GetTotalLambda());
		rec.B(inPart.IsActive());
		rec.State(inPart);
	};

	ZeroedPart<PointConstraintPart> part;
	RestoreFloats(*part, f + 6, 3);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, Mat44::sRotation(body1.GetRotation()), CPLoad3(f), body2, Mat44::sRotation(body2.GetRotation()), CPLoad3(f + 3));
	dump(1, 0, *part);

	part->WarmStart(body1, body2, f[9]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, f[10]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// HingeRotationConstraintPart. mF: hinge axis 1, hinge axis 2, initial lambda (2), warm start ratio, baumgarte. mI: -, -,
// velocity steps, position steps
void jolt_cp_hinge_rotation(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;

	auto dump = [&](uint32 inId, uint32 inResult, HingeRotationConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		rec.V3(inPart.*CPGet(HingeA1()));
		rec.V3(inPart.*CPGet(HingeB2()));
		rec.V3(inPart.*CPGet(HingeC2()));
		rec.M44(inPart.*CPGet(HingeInvI1()));
		rec.M44(inPart.*CPGet(HingeInvI2()));
		rec.V3(inPart.*CPGet(HingeB2xA1()));
		rec.V3(inPart.*CPGet(HingeC2xA1()));
		rec.M22(inPart.*CPGet(HingeEffectiveMass()));
		rec.V2(inPart.GetTotalLambda());
		rec.State(inPart);
	};

	ZeroedPart<HingeRotationConstraintPart> part;
	RestoreFloats(*part, f + 6, 2);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, Mat44::sRotation(body1.GetRotation()), CPLoad3(f), body2, Mat44::sRotation(body2.GetRotation()), CPLoad3(f + 3));
	dump(1, 0, *part);

	part->WarmStart(body1, body2, f[8]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, f[9]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// RotationEulerConstraintPart. mF: axis X1, Y1 (or Z1), X2, Y2 (or Z2), initial lambda (3), warm start ratio,
// baumgarte, quaternion. mI: inverse initial orientation for the position steps (0 bodies, 1 XY, 2 XZ, 3 the
// quaternion), -, velocity steps, position steps
void jolt_cp_rotation_euler(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;

	auto dump = [&](uint32 inId, uint32 inResult, RotationEulerConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		rec.M44(inPart.*CPGet(EulerInvI1()));
		rec.M44(inPart.*CPGet(EulerInvI2()));
		rec.M44(inPart.*CPGet(EulerEffectiveMass()));
		rec.V3(inPart.GetTotalLambda());
		rec.B(inPart.IsActive());
		rec.State(inPart);
	};

	Quat from_bodies = RotationEulerConstraintPart::sGetInvInitialOrientation(body1, body2);
	Quat xy = RotationEulerConstraintPart::sGetInvInitialOrientationXY(CPLoad3(f), CPLoad3(f + 3), CPLoad3(f + 6), CPLoad3(f + 9));
	Quat xz = RotationEulerConstraintPart::sGetInvInitialOrientationXZ(CPLoad3(f), CPLoad3(f + 3), CPLoad3(f + 6), CPLoad3(f + 9));
	rec.Q(from_bodies);
	rec.Q(xy);
	rec.Q(xz);
	Quat inv_initial_orientation = ui[0] == 0? from_bodies : (ui[0] == 1? xy : (ui[0] == 2? xz : CPLoadQuat(f + 17)));

	ZeroedPart<RotationEulerConstraintPart> part;
	RestoreFloats(*part, f + 12, 3);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, Mat44::sRotation(body1.GetRotation()), body2, Mat44::sRotation(body2.GetRotation()));
	dump(1, 0, *part);

	part->WarmStart(body1, body2, f[15]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, inv_initial_orientation, f[16]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// RotationQuatConstraintPart. mF: quaternion, initial lambda (3), warm start ratio, baumgarte. mI: inverse initial
// orientation (0 bodies, 1 the quaternion), -, velocity steps, position steps
void jolt_cp_rotation_quat(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;

	auto dump = [&](uint32 inId, uint32 inResult, RotationQuatConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		rec.M44(inPart.*CPGet(QuatInvI1JPT()));
		rec.M44(inPart.*CPGet(QuatInvI2JPT()));
		rec.M44(inPart.*CPGet(QuatEffectiveMass()));
		rec.M44(inPart.*CPGet(QuatEffectiveMassJP()));
		rec.V3(inPart.GetTotalLambda());
		rec.B(inPart.IsActive());
		rec.State(inPart);
	};

	Quat from_bodies = RotationQuatConstraintPart::sGetInvInitialOrientation(body1, body2);
	rec.Q(from_bodies);
	Quat inv_initial_orientation = ui[0] == 0? from_bodies : CPLoadQuat(f);

	ZeroedPart<RotationQuatConstraintPart> part;
	RestoreFloats(*part, f + 4, 3);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, Mat44::sRotation(body1.GetRotation()), body2, Mat44::sRotation(body2.GetRotation()), inv_initial_orientation);
	dump(1, 0, *part);

	part->WarmStart(body1, body2, f[7]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, inv_initial_orientation, f[8]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// SwingTwistConstraintPart::ClampSwingTwist and sDistanceToMinShorter. inF: limits (6), swing, twist, delta min / max.
// outF: swing, twist; outI: clamped axis, distance to min shorter
void jolt_cp_swing_twist_clamp(const float *inF, uint32 inSwingType, float *outF, uint32 *outI)
{
	CPEnsureFactory();
	SwingTwistConstraintPart part;
	part.SetSwingType(ESwingType(inSwingType));
	part.SetLimits(inF[0], inF[1], inF[2], inF[3], inF[4], inF[5]);
	Quat swing = CPLoadQuat(inF + 6), twist = CPLoadQuat(inF + 10);
	uint clamped_axis;
	part.ClampSwingTwist(swing, twist, clamped_axis);
	outF[0] = swing.GetX(); outF[1] = swing.GetY(); outF[2] = swing.GetZ(); outF[3] = swing.GetW();
	outF[4] = twist.GetX(); outF[5] = twist.GetY(); outF[6] = twist.GetZ(); outF[7] = twist.GetW();
	outI[0] = clamped_axis;
	outI[1] = SwingTwistConstraintPart::sDistanceToMinShorter(inF[14], inF[15])? 1 : 0;
}

// SwingTwistConstraintPart. mF: limits (6), constraint rotation, constraint to body 1, constraint to body 2, initial
// lambdas (3), warm start ratio, baumgarte. mI: swing type, -, velocity steps, position steps
void jolt_cp_swing_twist(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;
	Quat constraint_rotation = CPLoadQuat(f + 6), constraint_to_body1 = CPLoadQuat(f + 10), constraint_to_body2 = CPLoadQuat(f + 14);

	auto dump = [&](uint32 inId, uint32 inResult, SwingTwistConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		DumpSwingTwist(rec, inPart);
	};

	ZeroedPart<SwingTwistConstraintPart> part;
	part->SetSwingType(ESwingType(ui[0]));
	part->SetLimits(f[0], f[1], f[2], f[3], f[4], f[5]);
	RestoreFloats(*part, f + 18, 3);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, body2, constraint_rotation, body1.GetRotation() * constraint_to_body1);
	dump(1, 0, *part);

	part->WarmStart(body1, body2, f[21]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, constraint_rotation, constraint_to_body1, constraint_to_body2, f[22]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// IndependentAxisConstraintPart. mF: r1, n1, r2, n2, ratio, initial lambda, warm start ratio, min lambda, max lambda,
// C, baumgarte. mI: -, -, velocity steps, position steps
void jolt_cp_independent_axis(void *inWorld, const CPInput *inInput, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;
	Vec3 n1 = CPLoad3(f + 3), n2 = CPLoad3(f + 9);
	float ratio = f[12];

	auto dump = [&](uint32 inId, uint32 inResult, IndependentAxisConstraintPart &inPart) {
		rec.Step(inId, inResult, body1, body2);
		rec.V3(inPart.*CPGet(IndR1xN1()));
		rec.V3(inPart.*CPGet(IndInvI1()));
		rec.V3(inPart.*CPGet(IndRatioR2xN2()));
		rec.V3(inPart.*CPGet(IndInvI2()));
		rec.F(inPart.*CPGet(IndEffectiveMass()));
		rec.F(inPart.GetTotalLambda());
		rec.B(inPart.IsActive());
		rec.State(inPart);
	};

	ZeroedPart<IndependentAxisConstraintPart> part;
	RestoreFloats(*part, f + 13, 1);
	dump(0, 0, *part);

	part->CalculateConstraintProperties(body1, body2, CPLoad3(f), n1, CPLoad3(f + 6), n2, ratio);
	dump(1, 0, *part);

	part->WarmStart(body1, body2, n1, n2, ratio, f[14]);
	dump(2, 0, *part);

	for (uint32 i = 0; i < ui[2]; ++i)
	{
		bool result = part->SolveVelocityConstraint(body1, body2, n1, n2, ratio, f[15], f[16]);
		dump(3, result? 1 : 0, *part);
	}

	for (uint32 i = 0; i < ui[3]; ++i)
	{
		bool result = part->SolvePositionConstraint(body1, body2, n1, n2, ratio, f[17], f[18]);
		dump(4, result? 1 : 0, *part);
	}

	part->Deactivate();
	dump(5, 0, *part);
}

// GearConstraintPart (inRack == 0) or RackAndPinionConstraintPart (inRack != 0). mF: axis 1, axis 2, ratio, initial
// lambda, warm start ratio, C, baumgarte. mI: -, -, velocity steps, position steps
void jolt_cp_gear(void *inWorld, const CPInput *inInput, uint32 inRack, CPOutput *outOutput)
{
	Rec rec(outOutput);
	BodyPairScope bodies(inWorld, inInput);
	Body &body1 = *bodies.mBody1, &body2 = *bodies.mBody2;
	const float *f = inInput->mF;
	const uint32 *ui = inInput->mI;
	Vec3 a = CPLoad3(f), b = CPLoad3(f + 3);
	float ratio = f[6];

	auto run = [&](auto &ioPart, auto &&inDump) {
		RestoreFloats(ioPart, f + 7, 1);
		rec.Step(0, 0, body1, body2);
		inDump(ioPart);

		ioPart.CalculateConstraintProperties(body1, a, body2, b, ratio);
		rec.Step(1, 0, body1, body2);
		inDump(ioPart);

		ioPart.WarmStart(body1, body2, f[8]);
		rec.Step(2, 0, body1, body2);
		inDump(ioPart);

		for (uint32 i = 0; i < ui[2]; ++i)
		{
			bool result = ioPart.SolveVelocityConstraint(body1, a, body2, b, ratio);
			rec.Step(3, result? 1 : 0, body1, body2);
			inDump(ioPart);
		}

		for (uint32 i = 0; i < ui[3]; ++i)
		{
			bool result = ioPart.SolvePositionConstraint(body1, body2, f[9], f[10]);
			rec.Step(4, result? 1 : 0, body1, body2);
			inDump(ioPart);
		}

		ioPart.Deactivate();
		rec.Step(5, 0, body1, body2);
		inDump(ioPart);
	};

	if (inRack == 0)
	{
		ZeroedPart<GearConstraintPart> part;
		run(*part, [&](GearConstraintPart &inPart) {
			rec.V3(inPart.*CPGet(GearInvI1()));
			rec.V3(inPart.*CPGet(GearInvI2()));
			rec.F(inPart.*CPGet(GearEffectiveMass()));
			rec.F(inPart.GetTotalLambda());
			rec.B(inPart.IsActive());
			rec.State(inPart);
		});
	}
	else
	{
		ZeroedPart<RackAndPinionConstraintPart> part;
		run(*part, [&](RackAndPinionConstraintPart &inPart) {
			rec.V3(inPart.*CPGet(RackInvI1()));
			rec.V3(inPart.*CPGet(RackRatioInvM2()));
			rec.F(inPart.*CPGet(RackEffectiveMass()));
			rec.F(inPart.GetTotalLambda());
			rec.B(inPart.IsActive());
			rec.State(inPart);
		});
	}
}

// sizeof of the templated contact parts, for the 9 motion type combinations (index 3 * type1 + type2)
void jolt_cp_contact_sizes(uint32 *outContact, uint32 *outAngularFriction)
{
	constexpr EMotionType S = EMotionType::Static, K = EMotionType::Kinematic, D = EMotionType::Dynamic;
	uint32 contact[] = { sizeof(ContactConstraintPart<S, S>), sizeof(ContactConstraintPart<S, K>), sizeof(ContactConstraintPart<S, D>),
		sizeof(ContactConstraintPart<K, S>), sizeof(ContactConstraintPart<K, K>), sizeof(ContactConstraintPart<K, D>),
		sizeof(ContactConstraintPart<D, S>), sizeof(ContactConstraintPart<D, K>), sizeof(ContactConstraintPart<D, D>) };
	uint32 angular[] = { sizeof(AngularFrictionConstraintPart<S, S>), sizeof(AngularFrictionConstraintPart<S, K>), sizeof(AngularFrictionConstraintPart<S, D>),
		sizeof(AngularFrictionConstraintPart<K, S>), sizeof(AngularFrictionConstraintPart<K, K>), sizeof(AngularFrictionConstraintPart<K, D>),
		sizeof(AngularFrictionConstraintPart<D, S>), sizeof(AngularFrictionConstraintPart<D, K>), sizeof(AngularFrictionConstraintPart<D, D>) };
	memcpy(outContact, contact, sizeof(contact));
	memcpy(outAngularFriction, angular, sizeof(angular));
}

} // extern "C"
