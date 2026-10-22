// Reference implementation of the constraint parity tests (`zig build parity`, Phase 6): C ABI wrappers that create
// and drive every two body constraint class of Jolt/Physics/Constraints. ZoltParity/Physics/ConstraintsParity.zig calls
// these C functions with the same descriptions as the Zolt side and compares the records bit for bit.
//
// A scenario (CScenario) is two bodies (body 1 may be Body::sFixedToWorld) created through a BodyManager like
// ConstraintPartsReference.cpp (a box shape, optionally behind an OffsetCenterOfMassShape so that the center of mass is
// not at the body origin), a constraint created from settings (CSettings: the base class fields plus per kind arrays
// whose layout is documented at MakeSettings) with TwoBodyConstraintSettings::Create, a list of operations (COp: the
// generic ones below and the setters of the kind, see ApplyKindOp) and a solver sequence per frame: the operations of
// the frame, SetupVelocityConstraint, WarmStartVelocityConstraint, SolveVelocityConstraint (several), and
// SolvePositionConstraint (several). After the creation and after every call jolt_cs_run records (Rec, COutput): the
// step id and the return value, position / rotation / velocities of both bodies, IsActive, the SaveState bytes, the
// matrices GetConstraintToBody1/2Matrix and every getter of the kind (RecordGetters). At the start and at the end it
// records the SaveBinaryState bytes of GetConstraintSettings, after the creation the getters of the settings class
// (CSRecordSettingsGetters). The descriptions also call the helpers of the settings classes that compute values
// (SetSliderAxis, SetRatio, MakeFreeAxis / MakeFixedAxis / SetLimitedAxis, see CSMakeSettings).
//
// The kind of a constraint is its EConstraintSubType value (ConstraintKind in ConstraintsParity.zig). This file covers
// every kind; the Zolt side fills its slots as the classes are ported.
//
// jolt_cs_draw_world_* is the C++ side of DrawWorld in ConstraintsParity.zig (group D3, the debug renderer parity):
// random scenarios in one BodyManager and one ConstraintManager, stepped frame by frame without records.
//
// Every allocation through Jolt is zero filled (the hooks of ParityHooks.h, installed by CSEnsureFactory): a pulley with
// a static body reads members that Jolt never writes (see CSEnsureFactory).
//
// jolt_cs_settings_save writes the binary state of the settings of a description, jolt_cs_settings_resave restores
// settings from bytes (ConstraintSettings::sRestoreFromBinaryState) and saves them again (the cross language round
// trip). The structs below must match the extern structs of ConstraintsParity.zig (checked by jolt_cs_struct_sizes).
// Booleans are passed as uint32 (never bool, see the porting guide). All C functions have the prefix jolt_cs_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Constraints/ConstraintManager.h>
#include <ZoltParity/ParityHooks.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

namespace
{

// Must match CSBodyDesc in ConstraintsParity.zig
struct CSBodyDesc
{
	Real				mPosition[3];
	float				mRotation[4];
	float				mLinearVelocity[3];
	float				mAngularVelocity[3];
	float				mHalfExtent[3];
	float				mCOMOffset[3];				// Not zero: the box is wrapped in an OffsetCenterOfMassShape
	float				mMass;
	float				mInertiaMultiplier;
	float				mInertia[16];
	uint32				mMotionType;
	uint32				mAllowedDOFs;
	uint32				mOverrideMassProperties;
	uint32				mFixedToWorld;				// Body 1 only: use Body::sFixedToWorld instead of creating a body
	uint32				mAllowDynamicOrKinematic;	// A static body with motion properties (CanBeKinematicOrDynamic)
};

// Must match CSSettings in ConstraintsParity.zig: the settings of a constraint, mI / mR / mF per kind (see MakeSettings)
struct CSSettings
{
	uint32				mKind;						// EConstraintSubType
	uint32				mEnabled;
	uint32				mConstraintPriority;
	uint32				mNumVelocityStepsOverride;
	uint32				mNumPositionStepsOverride;
	float				mDrawConstraintSize;
	uint64				mUserData;
	uint32				mI[32];
	Real				mR[12];
	float				mF[192];
};

// Must match CSOp in ConstraintsParity.zig: an operation applied before frame mFrame (mFrame == number of frames: after
// the last frame), see ApplyOp / ApplyKindOp
struct CSOp
{
	uint32				mOp;
	uint32				mFrame;
	uint32				mI[4];
	float				mF[12];
	Real				mR[3];
};

static constexpr uint32 cMaxOps = 24;

// Must match CSScenario in ConstraintsParity.zig
struct CSScenario
{
	CSBodyDesc			mBody[2];
	CSSettings			mSettings;
	float				mDeltaTime;
	float				mBaumgarte;
	float				mWarmStartRatio[4];			// Per frame
	uint32				mNumFrames;					// 1 .. 4
	uint32				mNumVelocitySteps;
	uint32				mNumPositionSteps;
	uint32				mNumOps;
	CSOp				mOps[cMaxOps];
};

static constexpr uint32 cMaxR = 1024;
static constexpr uint32 cMaxF = 16384;
static constexpr uint32 cMaxI = 8192;
static constexpr uint32 cMaxB = 32768;

// Must match CSOutput in ConstraintsParity.zig
struct CSOutput
{
	Real				mR[cMaxR];
	float				mF[cMaxF];
	uint32				mI[cMaxI];
	uint8				mB[cMaxB];
	uint32				mNumR;
	uint32				mNumF;
	uint32				mNumI;
	uint32				mNumB;
};

// The generic operations (CSOp::mOp), the kind specific ones start at cFirstKindOp (see ApplyKindOp)
enum ECSOp : uint32
{
	SetEnabled = 0,							// mI[0]
	NotifyShapeChanged = 1,					// mI[0]: 0 body 1, 1 body 2, 2 another body ID; mF[0..3] delta COM
	ResetWarmStart = 2,
	SaveRestoreState = 3,					// SaveState, ResetWarmStart, RestoreState of the saved bytes
	SetSolverSettings = 4,					// SetConstraintPriority(mI[0]), SetNumVelocityStepsOverride(mI[1]), SetNumPositionStepsOverride(mI[2]), SetUserData(mI[3])
};

static constexpr uint32 cFirstKindOp = 16;

// The allocation hooks of ParityHooks.h (zero-filled blocks) and the Factory, created once for the whole parity binary
// (the same guard as the other wrapper files, so that RegisterTypes() runs only once). Every function of this file that
// allocates through Jolt calls it first. The blocks must be zero filled: the generator makes pulleys with a static body,
// and IndependentAxisConstraintPart::SolveVelocityConstraint reads members that Jolt never writes for a static body
// (mR1xN1, mRatioR2xN2) and multiplies them with its zero angular velocity. A NaN or an infinity in a reused block would
// make the velocity of the pulley's dynamic body NaN. Zolt starts these members at zero.
static void CSEnsureFactory()
{
	ParityInstallHooks();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static Vec3 CSLoad3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 CSLoad4(const float *inV)				{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Quat CSLoadQuat(const float *inV)			{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 CSLoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static Mat44 CSLoadMat44(const float *inM)			{ return Mat44(CSLoad4(inM), CSLoad4(inM + 4), CSLoad4(inM + 8), CSLoad4(inM + 12)); }

// A spring: mode, frequency / stiffness, damping
static SpringSettings CSLoadSpring(uint32 inMode, const float *inF)
{
	SpringSettings s;
	s.mMode = ESpringMode(inMode);
	s.mFrequency = inF[0];
	s.mDamping = inF[1];
	return s;
}

// A motor: the spring (mode, frequency / stiffness, damping), min / max force limit, min / max torque limit
static MotorSettings CSLoadMotor(uint32 inMode, const float *inF)
{
	MotorSettings m;
	m.mSpringSettings = CSLoadSpring(inMode, inF);
	m.mMinForceLimit = inF[2];
	m.mMaxForceLimit = inF[3];
	m.mMinTorqueLimit = inF[4];
	m.mMaxTorqueLimit = inF[5];
	return m;
}

// Appends values to the output record (the counters keep counting past the capacity, the Zig side checks them)
class Rec
{
public:
	explicit			Rec(CSOutput *inOut) : mOut(inOut) { inOut->mNumR = inOut->mNumF = inOut->mNumI = inOut->mNumB = 0; }

	void				R(Real inV)					{ if (mOut->mNumR < cMaxR) mOut->mR[mOut->mNumR] = inV; ++mOut->mNumR; }
	void				F(float inV)				{ if (mOut->mNumF < cMaxF) mOut->mF[mOut->mNumF] = inV; ++mOut->mNumF; }
	void				I(uint32 inV)				{ if (mOut->mNumI < cMaxI) mOut->mI[mOut->mNumI] = inV; ++mOut->mNumI; }
	void				B(bool inV)					{ I(inV? 1 : 0); }
	void				V3(Vec3Arg inV)				{ F(inV.GetX()); F(inV.GetY()); F(inV.GetZ()); }
	void				R3(RVec3Arg inV)			{ R(inV.GetX()); R(inV.GetY()); R(inV.GetZ()); }
	void				Q(QuatArg inQ)				{ F(inQ.GetX()); F(inQ.GetY()); F(inQ.GetZ()); F(inQ.GetW()); }
	void				M44(Mat44Arg inM)			{ for (uint c = 0; c < 4; ++c) for (uint r = 0; r < 4; ++r) F(inM(r, c)); }
	void				V2(const Vector<2> &inV)	{ F(inV[0]); F(inV[1]); }

	void				Spring(const SpringSettings &inS)	{ I(uint32(inS.mMode)); F(inS.mFrequency); F(inS.mDamping); }
	void				Motor(const MotorSettings &inM)		{ Spring(inM.mSpringSettings); F(inM.mMinForceLimit); F(inM.mMaxForceLimit); F(inM.mMinTorqueLimit); F(inM.mMaxTorqueLimit); }

	// A block of bytes: the size in I, the bytes in B
	void				Bytes(const std::string &inData)
	{
		I(uint32(inData.size()));
		for (char c : inData)
		{
			if (mOut->mNumB < cMaxB) mOut->mB[mOut->mNumB] = uint8(c);
			++mOut->mNumB;
		}
	}

	void				BodyState(const Body &inBody)
	{
		R3(inBody.GetCenterOfMassPosition());
		Q(inBody.GetRotation());
		V3(inBody.GetLinearVelocity());
		V3(inBody.GetAngularVelocity());
	}

private:
	CSOutput *			mOut;
};

// Broad phase layer = object layer
class CSLayers final : public BroadPhaseLayerInterface
{
public:
	virtual uint				GetNumBroadPhaseLayers() const override						{ return 1; }
	virtual BroadPhaseLayer		GetBroadPhaseLayer(ObjectLayer inLayer) const override		{ return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *		GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

struct CSWorld
{
	CSLayers					mLayers;
	BodyManager					mBodyManager;
};

static Body *CSCreateBody(BodyManager &ioBodyManager, const CSBodyDesc &inDesc)
{
	RefConst<Shape> shape = new BoxShape(CSLoad3(inDesc.mHalfExtent));
	Vec3 com_offset = CSLoad3(inDesc.mCOMOffset);
	if (com_offset != Vec3::sZero())
		shape = new OffsetCenterOfMassShape(shape, com_offset);
	BodyCreationSettings settings(shape, CSLoadR3(inDesc.mPosition), CSLoadQuat(inDesc.mRotation), EMotionType(inDesc.mMotionType), 0);
	settings.mLinearVelocity = CSLoad3(inDesc.mLinearVelocity);
	settings.mAngularVelocity = CSLoad3(inDesc.mAngularVelocity);
	settings.mAllowedDOFs = EAllowedDOFs(inDesc.mAllowedDOFs);
	settings.mAllowDynamicOrKinematic = inDesc.mAllowDynamicOrKinematic != 0;
	settings.mOverrideMassProperties = EOverrideMassProperties(inDesc.mOverrideMassProperties);
	settings.mInertiaMultiplier = inDesc.mInertiaMultiplier;
	settings.mMassPropertiesOverride.mMass = inDesc.mMass;
	settings.mMassPropertiesOverride.mInertia = CSLoadMat44(inDesc.mInertia);
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

static void CSDestroyBody(BodyManager &ioBodyManager, Body *inBody)
{
	if (inBody == &Body::sFixedToWorld)
		return;
	BodyID id = inBody->GetID();
	ioBodyManager.DeactivateBodies(&id, 1);
	inBody->SetInBroadPhaseInternal(false);
	ioBodyManager.DestroyBodies(&id, 1);
}

// The Hermite path of a path constraint description (see MakeSettings)
static Ref<PathConstraintPathHermite> CSMakePath(const CSSettings &inS)
{
	const uint32 *i = inS.mI;
	const float *f = inS.mF;
	Ref<PathConstraintPathHermite> path = new PathConstraintPathHermite;
	path->SetIsLooping(i[1] != 0);
	for (uint32 p = 0; p < i[2]; ++p)
	{
		const float *pf = f + 16 + 9 * p;
		path->AddPoint(CSLoad3(pf), CSLoad3(pf + 3), CSLoad3(pf + 6));
	}
	return path;
}

// The settings of a description. Layout of mI / mR / mF per kind (a spring is: mode in mI, frequency / stiffness and
// damping in mF; a motor is: mode in mI, frequency / stiffness, damping, min / max force limit, min / max torque limit
// in mF, see CSLoadSpring / CSLoadMotor):
// - Fixed: I[0] space, I[1] auto detect point; R[0..3] point 1, R[3..6] point 2; F[0..3] axis X1, F[3..6] axis Y1,
//   F[6..9] axis X2, F[9..12] axis Y2
// - Point: I[0] space; R[0..3] point 1, R[3..6] point 2
// - Hinge: I[0] space; R[0..3] point 1, R[3..6] point 2; F[0..3] hinge axis 1, F[3..6] normal axis 1, F[6..9] hinge
//   axis 2, F[9..12] normal axis 2, F[12] limits min, F[13] limits max, limits spring I[1] F[14..16], F[16] max friction
//   torque, motor I[2] F[17..23]
// - Slider: I[0] space, I[1] auto detect point, I[2] != 0: SetSliderAxis(F[0..3]) instead of the four axes; R[0..3]
//   point 1, R[3..6] point 2; F[0..3] slider axis 1, F[3..6] normal axis 1, F[6..9] slider axis 2, F[9..12] normal axis
//   2, F[12] limits min, F[13] limits max, limits spring I[3] F[14..16], F[16] max friction force, motor I[4] F[17..23]
// - Distance: I[0] space; R[0..3] point 1, R[3..6] point 2; F[0] min distance, F[1] max distance, limits spring I[1]
//   F[2..4]
// - Cone: I[0] space; R[0..3] point 1, R[3..6] point 2; F[0..3] twist axis 1, F[3..6] twist axis 2, F[6] half cone angle
// - SwingTwist: I[0] space, I[1] swing type; R[0..3] position 1, R[3..6] position 2; F[0..3] twist axis 1, F[3..6] plane
//   axis 1, F[6..9] twist axis 2, F[9..12] plane axis 2, F[12] normal half cone angle, F[13] plane half cone angle,
//   F[14] twist min angle, F[15] twist max angle, F[16] max friction torque, swing motor I[2] F[17..23], twist motor
//   I[3] F[23..29]
// - SixDOF: I[0] space, I[1] swing type; R[0..3] position 1, R[3..6] position 2; F[0..3] axis X1, F[3..6] axis Y1,
//   F[6..9] axis X2, F[9..12] axis Y2, F[12..18] max friction, F[18..24] limit min, F[24..30] limit max, limits spring
//   k (k < 3) I[2 + k] F[30 + 2k..32 + 2k], motor k (k < 6) I[5 + k] F[36 + 6k..42 + 6k]; I[11 + k] how the limits of
//   axis k are set: 1 SetLimitedAxis(k, F[18 + k], F[24 + k]), 2 MakeFreeAxis(k), 3 MakeFixedAxis(k), any other value
//   the members mLimitMin[k] / mLimitMax[k] = F[18 + k] / F[24 + k] (0 and 1 give the same settings)
// - Path: I[0] unused (the settings always have a path: SaveBinaryState dereferences it), I[1] path is looping, I[2]
//   number of path points (<= 16), I[3] rotation constraint type, motor I[4]; F[0..3] path position, F[3..7] path
//   rotation, F[7] path fraction, F[8] max friction force, motor F[9..15], point p: F[16 + 9p..] position, tangent,
//   normal
// - RackAndPinion: I[0] space, I[1] != 0: SetRatio(int(I[2]) teeth of the rack, F[7] rack length, int(I[3]) teeth of the
//   pinion) instead of F[6]; F[0..3] hinge axis, F[3..6] slider axis, F[6] ratio
// - Gear: I[0] space, I[1] != 0: SetRatio(int(I[2]) teeth of gear 1, int(I[3]) teeth of gear 2) instead of F[6]; F[0..3]
//   hinge axis 1, F[3..6] hinge axis 2, F[6] ratio
// - Pulley: I[0] space; R[0..3] body point 1, R[3..6] fixed point 1, R[6..9] body point 2, R[9..12] fixed point 2; F[0]
//   ratio, F[1] min length, F[2] max length
static Ref<TwoBodyConstraintSettings> CSMakeSettings(const CSSettings &inS)
{
	const uint32 *i = inS.mI;
	const Real *r = inS.mR;
	const float *f = inS.mF;
	Ref<TwoBodyConstraintSettings> settings;
	switch (EConstraintSubType(inS.mKind))
	{
	case EConstraintSubType::Fixed:
		{
			FixedConstraintSettings *s = new FixedConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mAutoDetectPoint = i[1] != 0;
			s->mPoint1 = CSLoadR3(r);
			s->mAxisX1 = CSLoad3(f);
			s->mAxisY1 = CSLoad3(f + 3);
			s->mPoint2 = CSLoadR3(r + 3);
			s->mAxisX2 = CSLoad3(f + 6);
			s->mAxisY2 = CSLoad3(f + 9);
			settings = s;
			break;
		}

	case EConstraintSubType::Point:
		{
			PointConstraintSettings *s = new PointConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = CSLoadR3(r);
			s->mPoint2 = CSLoadR3(r + 3);
			settings = s;
			break;
		}

	case EConstraintSubType::Hinge:
		{
			HingeConstraintSettings *s = new HingeConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = CSLoadR3(r);
			s->mHingeAxis1 = CSLoad3(f);
			s->mNormalAxis1 = CSLoad3(f + 3);
			s->mPoint2 = CSLoadR3(r + 3);
			s->mHingeAxis2 = CSLoad3(f + 6);
			s->mNormalAxis2 = CSLoad3(f + 9);
			s->mLimitsMin = f[12];
			s->mLimitsMax = f[13];
			s->mLimitsSpringSettings = CSLoadSpring(i[1], f + 14);
			s->mMaxFrictionTorque = f[16];
			s->mMotorSettings = CSLoadMotor(i[2], f + 17);
			settings = s;
			break;
		}

	case EConstraintSubType::Slider:
		{
			SliderConstraintSettings *s = new SliderConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mAutoDetectPoint = i[1] != 0;
			s->mPoint1 = CSLoadR3(r);
			s->mPoint2 = CSLoadR3(r + 3);
			if (i[2] != 0)
				s->SetSliderAxis(CSLoad3(f));
			else
			{
				s->mSliderAxis1 = CSLoad3(f);
				s->mNormalAxis1 = CSLoad3(f + 3);
				s->mSliderAxis2 = CSLoad3(f + 6);
				s->mNormalAxis2 = CSLoad3(f + 9);
			}
			s->mLimitsMin = f[12];
			s->mLimitsMax = f[13];
			s->mLimitsSpringSettings = CSLoadSpring(i[3], f + 14);
			s->mMaxFrictionForce = f[16];
			s->mMotorSettings = CSLoadMotor(i[4], f + 17);
			settings = s;
			break;
		}

	case EConstraintSubType::Distance:
		{
			DistanceConstraintSettings *s = new DistanceConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = CSLoadR3(r);
			s->mPoint2 = CSLoadR3(r + 3);
			s->mMinDistance = f[0];
			s->mMaxDistance = f[1];
			s->mLimitsSpringSettings = CSLoadSpring(i[1], f + 2);
			settings = s;
			break;
		}

	case EConstraintSubType::Cone:
		{
			ConeConstraintSettings *s = new ConeConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = CSLoadR3(r);
			s->mTwistAxis1 = CSLoad3(f);
			s->mPoint2 = CSLoadR3(r + 3);
			s->mTwistAxis2 = CSLoad3(f + 3);
			s->mHalfConeAngle = f[6];
			settings = s;
			break;
		}

	case EConstraintSubType::SwingTwist:
		{
			SwingTwistConstraintSettings *s = new SwingTwistConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mSwingType = ESwingType(i[1]);
			s->mPosition1 = CSLoadR3(r);
			s->mTwistAxis1 = CSLoad3(f);
			s->mPlaneAxis1 = CSLoad3(f + 3);
			s->mPosition2 = CSLoadR3(r + 3);
			s->mTwistAxis2 = CSLoad3(f + 6);
			s->mPlaneAxis2 = CSLoad3(f + 9);
			s->mNormalHalfConeAngle = f[12];
			s->mPlaneHalfConeAngle = f[13];
			s->mTwistMinAngle = f[14];
			s->mTwistMaxAngle = f[15];
			s->mMaxFrictionTorque = f[16];
			s->mSwingMotorSettings = CSLoadMotor(i[2], f + 17);
			s->mTwistMotorSettings = CSLoadMotor(i[3], f + 23);
			settings = s;
			break;
		}

	case EConstraintSubType::SixDOF:
		{
			SixDOFConstraintSettings *s = new SixDOFConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mSwingType = ESwingType(i[1]);
			s->mPosition1 = CSLoadR3(r);
			s->mAxisX1 = CSLoad3(f);
			s->mAxisY1 = CSLoad3(f + 3);
			s->mPosition2 = CSLoadR3(r + 3);
			s->mAxisX2 = CSLoad3(f + 6);
			s->mAxisY2 = CSLoad3(f + 9);
			for (int a = 0; a < SixDOFConstraintSettings::EAxis::Num; ++a)
			{
				SixDOFConstraintSettings::EAxis axis = SixDOFConstraintSettings::EAxis(a);
				s->mMaxFriction[a] = f[12 + a];
				switch (i[11 + a])
				{
				case 1:		s->SetLimitedAxis(axis, f[18 + a], f[24 + a]); break;
				case 2:		s->MakeFreeAxis(axis); break;
				case 3:		s->MakeFixedAxis(axis); break;
				default:	s->mLimitMin[a] = f[18 + a]; s->mLimitMax[a] = f[24 + a]; break;
				}
				s->mMotorSettings[a] = CSLoadMotor(i[5 + a], f + 36 + 6 * a);
			}
			for (int a = 0; a < SixDOFConstraintSettings::EAxis::NumTranslation; ++a)
				s->mLimitsSpringSettings[a] = CSLoadSpring(i[2 + a], f + 30 + 2 * a);
			settings = s;
			break;
		}

	case EConstraintSubType::Path:
		{
			PathConstraintSettings *s = new PathConstraintSettings;
			s->mPath = CSMakePath(inS);
			s->mPathPosition = CSLoad3(f);
			s->mPathRotation = CSLoadQuat(f + 3);
			s->mPathFraction = f[7];
			s->mMaxFrictionForce = f[8];
			s->mPositionMotorSettings = CSLoadMotor(i[4], f + 9);
			s->mRotationConstraintType = EPathRotationConstraintType(i[3]);
			settings = s;
			break;
		}

	case EConstraintSubType::RackAndPinion:
		{
			RackAndPinionConstraintSettings *s = new RackAndPinionConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mHingeAxis = CSLoad3(f);
			s->mSliderAxis = CSLoad3(f + 3);
			if (i[1] != 0)
				s->SetRatio(int(i[2]), f[7], int(i[3]));
			else
				s->mRatio = f[6];
			settings = s;
			break;
		}

	case EConstraintSubType::Gear:
		{
			GearConstraintSettings *s = new GearConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mHingeAxis1 = CSLoad3(f);
			s->mHingeAxis2 = CSLoad3(f + 3);
			if (i[1] != 0)
				s->SetRatio(int(i[2]), int(i[3]));
			else
				s->mRatio = f[6];
			settings = s;
			break;
		}

	case EConstraintSubType::Pulley:
		{
			PulleyConstraintSettings *s = new PulleyConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mBodyPoint1 = CSLoadR3(r);
			s->mFixedPoint1 = CSLoadR3(r + 3);
			s->mBodyPoint2 = CSLoadR3(r + 6);
			s->mFixedPoint2 = CSLoadR3(r + 9);
			s->mRatio = f[0];
			s->mMinLength = f[1];
			s->mMaxLength = f[2];
			settings = s;
			break;
		}

	case EConstraintSubType::Vehicle:
	case EConstraintSubType::User1:
	case EConstraintSubType::User2:
	case EConstraintSubType::User3:
	case EConstraintSubType::User4:
	default:
		JPH_ASSERT(false);
		abort();
	}

	// The ConstraintSettings members
	settings->mEnabled = inS.mEnabled != 0;
	settings->mConstraintPriority = inS.mConstraintPriority;
	settings->mNumVelocityStepsOverride = inS.mNumVelocityStepsOverride;
	settings->mNumPositionStepsOverride = inS.mNumPositionStepsOverride;
	settings->mDrawConstraintSize = inS.mDrawConstraintSize;
	settings->mUserData = inS.mUserData;
	return settings;
}

// A hinge from the world to inBody in world space at the center of mass of the body (the gear / rack and pinion
// references). The normal of body 2 is the normal of body 1 rotated by inAngle around the axis, so the hinge angle of
// the body at creation is inAngle (up to the sign): the scenarios do not integrate the bodies, without it the gear /
// rack and pinion would never see a position error to correct
static Ref<Constraint> CSMakeReferenceHinge(Body &inBody, const float *inAxis, const float *inNormal, float inAngle)
{
	HingeConstraintSettings s;
	s.mPoint1 = s.mPoint2 = inBody.GetCenterOfMassPosition();
	s.mHingeAxis1 = s.mHingeAxis2 = CSLoad3(inAxis);
	s.mNormalAxis1 = CSLoad3(inNormal);
	s.mNormalAxis2 = Quat::sRotation(s.mHingeAxis1, inAngle) * s.mNormalAxis1;
	return s.Create(Body::sFixedToWorld, inBody);
}

// A slider from the world to inBody in world space at the center of mass of the body (the rack reference). The point of
// body 2 is moved by inOffset along the axis, so the slider position of the body at creation is inOffset (see
// CSMakeReferenceHinge)
static Ref<Constraint> CSMakeReferenceSlider(Body &inBody, const float *inAxis, const float *inNormal, float inOffset)
{
	SliderConstraintSettings s;
	s.mPoint1 = inBody.GetCenterOfMassPosition();
	s.mSliderAxis1 = s.mSliderAxis2 = CSLoad3(inAxis);
	s.mNormalAxis1 = s.mNormalAxis2 = CSLoad3(inNormal);
	s.mPoint2 = s.mPoint1 + inOffset * s.mSliderAxis1;
	return s.Create(Body::sFixedToWorld, inBody);
}

// The setters of a kind. Op numbers (CSOp::mOp, from cFirstKindOp = 16):
// - Point: 16 SetPoint1(EConstraintSpace(I[0]), R), 17 SetPoint2(EConstraintSpace(I[0]), R)
// - Hinge: 16 SetMaxFrictionTorque(F[0]), 17 SetMotorState(I[0]), 18 SetTargetAngularVelocity(F[0]), 19
//   SetTargetAngle(F[0]), 20 SetTargetOrientationBS(F[0..4]), 21 SetLimits(F[0], F[1]), 22 SetLimitsSpringSettings
//   (I[0] F[0..2]), 23 GetMotorSettings() = motor I[0] F[0..6]
// - Slider: 16 SetMaxFrictionForce(F[0]), 17 SetMotorState(I[0]), 18 SetTargetVelocity(F[0]), 19 SetTargetPosition(F[0]),
//   20 SetLimits(F[0], F[1]), 21 SetLimitsSpringSettings(I[0] F[0..2]), 22 GetMotorSettings() = motor I[0] F[0..6]
// - Distance: 16 SetDistance(F[0], F[1]), 17 SetLimitsSpringSettings(I[0] F[0..2])
// - Cone: 16 SetHalfConeAngle(F[0])
// - SwingTwist: 16 SetNormalHalfConeAngle(F[0]), 17 SetPlaneHalfConeAngle(F[0]), 18 SetTwistMinAngle(F[0]), 19
//   SetTwistMaxAngle(F[0]), 20 SetMaxFrictionTorque(F[0]), 21 SetSwingMotorState(I[0]), 22 SetTwistMotorState(I[0]), 23
//   SetTargetAngularVelocityCS(F[0..3]), 24 SetTargetAngularVelocityBS(F[0..3]), 25 SetTargetOrientationCS(F[0..4]), 26
//   SetTargetOrientationBS(F[0..4]), 27 GetSwingMotorSettings() = motor I[0] F[0..6], 28 GetTwistMotorSettings() = ...
// - SixDOF: 16 SetTranslationLimits(F[0..3], F[3..6]), 17 SetRotationLimits(F[0..3], F[3..6]), 18
//   SetLimitsSpringSettings(EAxis(I[0]), spring I[1] F[0..2]), 19 SetMaxFriction(EAxis(I[0]), F[0]), 20
//   SetMotorState(EAxis(I[0]), I[1]), 21 SetTargetVelocityCS(F[0..3]), 22 SetTargetAngularVelocityCS(F[0..3]), 23
//   SetTargetPositionCS(F[0..3]), 24 SetTargetOrientationCS(F[0..4]), 25 SetTargetOrientationBS(F[0..4]), 26
//   GetMotorSettings(EAxis(I[0])) = motor I[1] F[0..6]
// - Path: 16 SetPath(I[0] != 0? the path of the settings description (a new object) : nullptr, F[0]) (nullptr only
//   after the last frame: the solver dereferences the path), 17 SetMaxFrictionForce(F[0]), 18
//   SetPositionMotorState(I[0]), 19 SetTargetVelocity(F[0]), 20 SetTargetPathFraction(F[0]), 21
//   GetPositionMotorSettings() = motor I[0] F[0..6]
// - RackAndPinion: 16 SetConstraints(I[0] & 1? a hinge from the world to body 1 (axis F[0..3], normal F[3..6], world
//   space, angle float(R[0]) at creation) : nullptr, I[0] & 2? a slider from the world to body 2 (axis F[6..9], normal
//   F[9..12], position float(R[1]) at creation) : nullptr)
// - Gear: 16 SetConstraints(I[0] & 1? a hinge from the world to body 1 (axis F[0..3], normal F[3..6], angle float(R[0]))
//   : nullptr, I[0] & 2? a hinge from the world to body 2 (axis F[6..9], normal F[9..12], angle float(R[1])) : nullptr)
// - Pulley: 16 SetLength(F[0], F[1])
static void CSApplyKindOp(TwoBodyConstraint &ioC, const CSOp &inOp, const CSSettings &inS)
{
	const uint32 *i = inOp.mI;
	const float *f = inOp.mF;
	switch (ioC.GetSubType())
	{
	case EConstraintSubType::Point:
		{
			PointConstraint &c = static_cast<PointConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetPoint1(EConstraintSpace(i[0]), CSLoadR3(inOp.mR)); return;
			case 17:	c.SetPoint2(EConstraintSpace(i[0]), CSLoadR3(inOp.mR)); return;
			}
			break;
		}

	case EConstraintSubType::Hinge:
		{
			HingeConstraint &c = static_cast<HingeConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetMaxFrictionTorque(f[0]); return;
			case 17:	c.SetMotorState(EMotorState(i[0])); return;
			case 18:	c.SetTargetAngularVelocity(f[0]); return;
			case 19:	c.SetTargetAngle(f[0]); return;
			case 20:	c.SetTargetOrientationBS(CSLoadQuat(f)); return;
			case 21:	c.SetLimits(f[0], f[1]); return;
			case 22:	c.SetLimitsSpringSettings(CSLoadSpring(i[0], f)); return;
			case 23:	c.GetMotorSettings() = CSLoadMotor(i[0], f); return;
			}
			break;
		}

	case EConstraintSubType::Slider:
		{
			SliderConstraint &c = static_cast<SliderConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetMaxFrictionForce(f[0]); return;
			case 17:	c.SetMotorState(EMotorState(i[0])); return;
			case 18:	c.SetTargetVelocity(f[0]); return;
			case 19:	c.SetTargetPosition(f[0]); return;
			case 20:	c.SetLimits(f[0], f[1]); return;
			case 21:	c.SetLimitsSpringSettings(CSLoadSpring(i[0], f)); return;
			case 22:	c.GetMotorSettings() = CSLoadMotor(i[0], f); return;
			}
			break;
		}

	case EConstraintSubType::Distance:
		{
			DistanceConstraint &c = static_cast<DistanceConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetDistance(f[0], f[1]); return;
			case 17:	c.SetLimitsSpringSettings(CSLoadSpring(i[0], f)); return;
			}
			break;
		}

	case EConstraintSubType::Cone:
		{
			ConeConstraint &c = static_cast<ConeConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetHalfConeAngle(f[0]); return;
			}
			break;
		}

	case EConstraintSubType::SwingTwist:
		{
			SwingTwistConstraint &c = static_cast<SwingTwistConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetNormalHalfConeAngle(f[0]); return;
			case 17:	c.SetPlaneHalfConeAngle(f[0]); return;
			case 18:	c.SetTwistMinAngle(f[0]); return;
			case 19:	c.SetTwistMaxAngle(f[0]); return;
			case 20:	c.SetMaxFrictionTorque(f[0]); return;
			case 21:	c.SetSwingMotorState(EMotorState(i[0])); return;
			case 22:	c.SetTwistMotorState(EMotorState(i[0])); return;
			case 23:	c.SetTargetAngularVelocityCS(CSLoad3(f)); return;
			case 24:	c.SetTargetAngularVelocityBS(CSLoad3(f)); return;
			case 25:	c.SetTargetOrientationCS(CSLoadQuat(f)); return;
			case 26:	c.SetTargetOrientationBS(CSLoadQuat(f)); return;
			case 27:	c.GetSwingMotorSettings() = CSLoadMotor(i[0], f); return;
			case 28:	c.GetTwistMotorSettings() = CSLoadMotor(i[0], f); return;
			}
			break;
		}

	case EConstraintSubType::SixDOF:
		{
			SixDOFConstraint &c = static_cast<SixDOFConstraint &>(ioC);
			using EAxis = SixDOFConstraint::EAxis;
			switch (inOp.mOp)
			{
			case 16:	c.SetTranslationLimits(CSLoad3(f), CSLoad3(f + 3)); return;
			case 17:	c.SetRotationLimits(CSLoad3(f), CSLoad3(f + 3)); return;
			case 18:	c.SetLimitsSpringSettings(EAxis(i[0]), CSLoadSpring(i[1], f)); return;
			case 19:	c.SetMaxFriction(EAxis(i[0]), f[0]); return;
			case 20:	c.SetMotorState(EAxis(i[0]), EMotorState(i[1])); return;
			case 21:	c.SetTargetVelocityCS(CSLoad3(f)); return;
			case 22:	c.SetTargetAngularVelocityCS(CSLoad3(f)); return;
			case 23:	c.SetTargetPositionCS(CSLoad3(f)); return;
			case 24:	c.SetTargetOrientationCS(CSLoadQuat(f)); return;
			case 25:	c.SetTargetOrientationBS(CSLoadQuat(f)); return;
			case 26:	c.GetMotorSettings(EAxis(i[0])) = CSLoadMotor(i[1], f); return;
			}
			break;
		}

	case EConstraintSubType::Path:
		{
			PathConstraint &c = static_cast<PathConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:
				{
					Ref<PathConstraintPathHermite> path = i[0] != 0? CSMakePath(inS) : nullptr;
					c.SetPath(path, f[0]);
					return;
				}
			case 17:	c.SetMaxFrictionForce(f[0]); return;
			case 18:	c.SetPositionMotorState(EMotorState(i[0])); return;
			case 19:	c.SetTargetVelocity(f[0]); return;
			case 20:	c.SetTargetPathFraction(f[0]); return;
			case 21:	c.GetPositionMotorSettings() = CSLoadMotor(i[0], f); return;
			}
			break;
		}

	case EConstraintSubType::RackAndPinion:
		{
			RackAndPinionConstraint &c = static_cast<RackAndPinionConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:
				{
					Ref<Constraint> pinion = (i[0] & 1) != 0? CSMakeReferenceHinge(*c.GetBody1(), f, f + 3, float(inOp.mR[0])) : nullptr;
					Ref<Constraint> rack = (i[0] & 2) != 0? CSMakeReferenceSlider(*c.GetBody2(), f + 6, f + 9, float(inOp.mR[1])) : nullptr;
					c.SetConstraints(pinion, rack);
					return;
				}
			}
			break;
		}

	case EConstraintSubType::Gear:
		{
			GearConstraint &c = static_cast<GearConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:
				{
					Ref<Constraint> gear1 = (i[0] & 1) != 0? CSMakeReferenceHinge(*c.GetBody1(), f, f + 3, float(inOp.mR[0])) : nullptr;
					Ref<Constraint> gear2 = (i[0] & 2) != 0? CSMakeReferenceHinge(*c.GetBody2(), f + 6, f + 9, float(inOp.mR[1])) : nullptr;
					c.SetConstraints(gear1, gear2);
					return;
				}
			}
			break;
		}

	case EConstraintSubType::Pulley:
		{
			PulleyConstraint &c = static_cast<PulleyConstraint &>(ioC);
			switch (inOp.mOp)
			{
			case 16:	c.SetLength(f[0], f[1]); return;
			}
			break;
		}

	case EConstraintSubType::Fixed:
	case EConstraintSubType::Vehicle:
	case EConstraintSubType::User1:
	case EConstraintSubType::User2:
	case EConstraintSubType::User3:
	case EConstraintSubType::User4:
		break;
	}

	JPH_ASSERT(false, "Unknown operation");
	abort();
}

// Every getter of a kind
static void CSRecordGetters(Rec &ioRec, const TwoBodyConstraint &inC)
{
	switch (inC.GetSubType())
	{
	case EConstraintSubType::Fixed:
		{
			const FixedConstraint &c = static_cast<const FixedConstraint &>(inC);
			ioRec.V3(c.GetTotalLambdaPosition());
			ioRec.V3(c.GetTotalLambdaRotation());
			break;
		}

	case EConstraintSubType::Point:
		{
			const PointConstraint &c = static_cast<const PointConstraint &>(inC);
			ioRec.V3(c.GetLocalSpacePoint1());
			ioRec.V3(c.GetLocalSpacePoint2());
			ioRec.V3(c.GetTotalLambdaPosition());
			break;
		}

	case EConstraintSubType::Hinge:
		{
			const HingeConstraint &c = static_cast<const HingeConstraint &>(inC);
			ioRec.V3(c.GetLocalSpacePoint1());
			ioRec.V3(c.GetLocalSpacePoint2());
			ioRec.V3(c.GetLocalSpaceHingeAxis1());
			ioRec.V3(c.GetLocalSpaceHingeAxis2());
			ioRec.V3(c.GetLocalSpaceNormalAxis1());
			ioRec.V3(c.GetLocalSpaceNormalAxis2());
			ioRec.F(c.GetCurrentAngle());
			ioRec.F(c.GetMaxFrictionTorque());
			ioRec.Motor(c.GetMotorSettings());
			ioRec.I(uint32(c.GetMotorState()));
			ioRec.F(c.GetTargetAngularVelocity());
			ioRec.F(c.GetTargetAngle());
			ioRec.F(c.GetLimitsMin());
			ioRec.F(c.GetLimitsMax());
			ioRec.B(c.HasLimits());
			ioRec.Spring(c.GetLimitsSpringSettings());
			ioRec.V3(c.GetTotalLambdaPosition());
			ioRec.V2(c.GetTotalLambdaRotation());
			ioRec.F(c.GetTotalLambdaRotationLimits());
			ioRec.F(c.GetTotalLambdaMotor());
			break;
		}

	case EConstraintSubType::Slider:
		{
			const SliderConstraint &c = static_cast<const SliderConstraint &>(inC);
			ioRec.F(c.GetCurrentPosition());
			ioRec.F(c.GetMaxFrictionForce());
			ioRec.Motor(c.GetMotorSettings());
			ioRec.I(uint32(c.GetMotorState()));
			ioRec.F(c.GetTargetVelocity());
			ioRec.F(c.GetTargetPosition());
			ioRec.F(c.GetLimitsMin());
			ioRec.F(c.GetLimitsMax());
			ioRec.B(c.HasLimits());
			ioRec.Spring(c.GetLimitsSpringSettings());
			ioRec.V2(c.GetTotalLambdaPosition());
			ioRec.F(c.GetTotalLambdaPositionLimits());
			ioRec.V3(c.GetTotalLambdaRotation());
			ioRec.F(c.GetTotalLambdaMotor());
			break;
		}

	case EConstraintSubType::Distance:
		{
			const DistanceConstraint &c = static_cast<const DistanceConstraint &>(inC);
			ioRec.F(c.GetMinDistance());
			ioRec.F(c.GetMaxDistance());
			ioRec.Spring(c.GetLimitsSpringSettings());
			ioRec.F(c.GetTotalLambdaPosition());
			break;
		}

	case EConstraintSubType::Cone:
		{
			const ConeConstraint &c = static_cast<const ConeConstraint &>(inC);
			ioRec.F(c.GetCosHalfConeAngle());
			ioRec.V3(c.GetTotalLambdaPosition());
			ioRec.F(c.GetTotalLambdaRotation());
			break;
		}

	case EConstraintSubType::SwingTwist:
		{
			const SwingTwistConstraint &c = static_cast<const SwingTwistConstraint &>(inC);
			ioRec.V3(c.GetLocalSpacePosition1());
			ioRec.V3(c.GetLocalSpacePosition2());
			ioRec.Q(c.GetConstraintToBody1());
			ioRec.Q(c.GetConstraintToBody2());
			ioRec.F(c.GetNormalHalfConeAngle());
			ioRec.F(c.GetPlaneHalfConeAngle());
			ioRec.F(c.GetTwistMinAngle());
			ioRec.F(c.GetTwistMaxAngle());
			ioRec.Motor(c.GetSwingMotorSettings());
			ioRec.Motor(c.GetTwistMotorSettings());
			ioRec.F(c.GetMaxFrictionTorque());
			ioRec.I(uint32(c.GetSwingMotorState()));
			ioRec.I(uint32(c.GetTwistMotorState()));
			ioRec.V3(c.GetTargetAngularVelocityCS());
			ioRec.Q(c.GetTargetOrientationCS());
			ioRec.Q(c.GetRotationInConstraintSpace());
			ioRec.V3(c.GetTotalLambdaPosition());
			ioRec.F(c.GetTotalLambdaTwist());
			ioRec.F(c.GetTotalLambdaSwingY());
			ioRec.F(c.GetTotalLambdaSwingZ());
			ioRec.V3(c.GetTotalLambdaMotor());
			break;
		}

	case EConstraintSubType::SixDOF:
		{
			const SixDOFConstraint &c = static_cast<const SixDOFConstraint &>(inC);
			using EAxis = SixDOFConstraint::EAxis;
			for (int a = 0; a < EAxis::Num; ++a)
			{
				EAxis axis = EAxis(a);
				ioRec.F(c.GetLimitsMin(axis));
				ioRec.F(c.GetLimitsMax(axis));
				ioRec.B(c.IsFixedAxis(axis));
				ioRec.B(c.IsFreeAxis(axis));
				ioRec.F(c.GetMaxFriction(axis));
				ioRec.I(uint32(c.GetMotorState(axis)));
				ioRec.Motor(c.GetMotorSettings(axis));
				if (a < EAxis::NumTranslation)
					ioRec.Spring(c.GetLimitsSpringSettings(axis));
			}
			ioRec.V3(c.GetTranslationLimitsMin());
			ioRec.V3(c.GetTranslationLimitsMax());
			ioRec.V3(c.GetRotationLimitsMin());
			ioRec.V3(c.GetRotationLimitsMax());
			ioRec.Q(c.GetRotationInConstraintSpace());
			ioRec.V3(c.GetTargetVelocityCS());
			ioRec.V3(c.GetTargetAngularVelocityCS());
			ioRec.V3(c.GetTargetPositionCS());
			ioRec.Q(c.GetTargetOrientationCS());
			ioRec.V3(c.GetTotalLambdaPosition());
			ioRec.V3(c.GetTotalLambdaRotation());
			ioRec.V3(c.GetTotalLambdaMotorTranslation());
			ioRec.V3(c.GetTotalLambdaMotorRotation());
			break;
		}

	case EConstraintSubType::Path:
		{
			const PathConstraint &c = static_cast<const PathConstraint &>(inC);
			const PathConstraintPath *path = c.GetPath();
			ioRec.B(path != nullptr);
			if (path != nullptr)
			{
				ioRec.B(path->IsLooping());
				ioRec.F(path->GetPathMaxFraction());
				Vec3 position, tangent, normal, binormal;
				path->GetPointOnPath(c.GetPathFraction(), position, tangent, normal, binormal);
				ioRec.V3(position);
				ioRec.V3(tangent);
				ioRec.V3(normal);
				ioRec.V3(binormal);
			}
			ioRec.F(c.GetPathFraction());
			ioRec.F(c.GetMaxFrictionForce());
			ioRec.Motor(c.GetPositionMotorSettings());
			ioRec.I(uint32(c.GetPositionMotorState()));
			ioRec.F(c.GetTargetVelocity());
			ioRec.F(c.GetTargetPathFraction());
			ioRec.V2(c.GetTotalLambdaPosition());
			ioRec.F(c.GetTotalLambdaPositionLimits());
			ioRec.F(c.GetTotalLambdaMotor());
			ioRec.V2(c.GetTotalLambdaRotationHinge());
			ioRec.V3(c.GetTotalLambdaRotation());
			break;
		}

	case EConstraintSubType::RackAndPinion:
		ioRec.F(static_cast<const RackAndPinionConstraint &>(inC).GetTotalLambda());
		break;

	case EConstraintSubType::Gear:
		ioRec.F(static_cast<const GearConstraint &>(inC).GetTotalLambda());
		break;

	case EConstraintSubType::Pulley:
		{
			const PulleyConstraint &c = static_cast<const PulleyConstraint &>(inC);
			ioRec.F(c.GetMinLength());
			ioRec.F(c.GetMaxLength());
			ioRec.F(c.GetCurrentLength());
			ioRec.F(c.GetTotalLambdaPosition());
			break;
		}

	case EConstraintSubType::Vehicle:
	case EConstraintSubType::User1:
	case EConstraintSubType::User2:
	case EConstraintSubType::User3:
	case EConstraintSubType::User4:
		JPH_ASSERT(false);
		break;
	}
}

// The getters of the settings class of a kind (the helpers that compute a value from the settings), recorded after the
// creation. The Zolt slot of the kind records the same values in its `recordSettingsGetters`:
// - SixDOF: IsFreeAxis(k), IsFixedAxis(k) of every axis k
static void CSRecordSettingsGetters(Rec &ioRec, const TwoBodyConstraintSettings &inSettings, EConstraintSubType inKind)
{
	switch (inKind)
	{
	case EConstraintSubType::SixDOF:
		{
			const SixDOFConstraintSettings &s = static_cast<const SixDOFConstraintSettings &>(inSettings);
			for (int a = 0; a < SixDOFConstraintSettings::EAxis::Num; ++a)
			{
				SixDOFConstraintSettings::EAxis axis = SixDOFConstraintSettings::EAxis(a);
				ioRec.B(s.IsFreeAxis(axis));
				ioRec.B(s.IsFixedAxis(axis));
			}
			break;
		}

	case EConstraintSubType::Fixed:
	case EConstraintSubType::Point:
	case EConstraintSubType::Hinge:
	case EConstraintSubType::Slider:
	case EConstraintSubType::Distance:
	case EConstraintSubType::Cone:
	case EConstraintSubType::SwingTwist:
	case EConstraintSubType::Path:
	case EConstraintSubType::Vehicle:
	case EConstraintSubType::RackAndPinion:
	case EConstraintSubType::Gear:
	case EConstraintSubType::Pulley:
	case EConstraintSubType::User1:
	case EConstraintSubType::User2:
	case EConstraintSubType::User3:
	case EConstraintSubType::User4:
		break;
	}
}

// The SaveState bytes of a constraint
static std::string CSSaveState(const Constraint &inC)
{
	StateRecorderImpl recorder;
	inC.SaveState(recorder);
	return recorder.GetData();
}

// The SaveBinaryState bytes of settings
static std::string CSSaveSettings(const ConstraintSettings &inSettings)
{
	std::stringstream data;
	StreamOutWrapper out(data);
	inSettings.SaveBinaryState(out);
	return data.str();
}

// Start of a step: the step id, the return value, both bodies and the generic state of the constraint
static void CSStep(Rec &ioRec, uint32 inId, uint32 inResult, const TwoBodyConstraint &inC)
{
	ioRec.I(0xC0DE0000 | inId);
	ioRec.I(inResult);
	ioRec.BodyState(*inC.GetBody1());
	ioRec.BodyState(*inC.GetBody2());
	ioRec.B(inC.IsActive());
	ioRec.B(inC.GetEnabled());
	ioRec.Bytes(CSSaveState(inC));
	ioRec.M44(inC.GetConstraintToBody1Matrix());
	ioRec.M44(inC.GetConstraintToBody2Matrix());
	CSRecordGetters(ioRec, inC);
}

// The settings of the constraint (GetConstraintSettings) and the base class values
static void CSRecordSettings(Rec &ioRec, const TwoBodyConstraint &inC)
{
	// PathConstraint::GetConstraintSettings is not implemented in Jolt (it asserts and returns nullptr)
	if (inC.GetSubType() == EConstraintSubType::Path)
	{
		ioRec.I(0xFFFFFFFF);
		return;
	}
	Ref<ConstraintSettings> settings = inC.GetConstraintSettings();
	ioRec.Bytes(CSSaveSettings(*settings));
	ioRec.I(settings->GetRefCount());
	ioRec.I(inC.GetConstraintPriority());
	ioRec.I(inC.GetNumVelocityStepsOverride());
	ioRec.I(inC.GetNumPositionStepsOverride());
	ioRec.I(uint32(inC.GetUserData()));
	ioRec.I(uint32(inC.GetUserData() >> 32));
}

// Apply an operation (generic or of the kind), see ECSOp and CSApplyKindOp
static void CSApplyOp(TwoBodyConstraint &ioC, const CSOp &inOp, const CSSettings &inS)
{
	const uint32 *i = inOp.mI;
	switch (inOp.mOp)
	{
	case ECSOp::SetEnabled:
		ioC.SetEnabled(i[0] != 0);
		break;

	case ECSOp::NotifyShapeChanged:
		{
			BodyID id = i[0] == 0? ioC.GetBody1()->GetID() : (i[0] == 1? ioC.GetBody2()->GetID() : BodyID(12345));
			ioC.NotifyShapeChanged(id, CSLoad3(inOp.mF));
			break;
		}

	case ECSOp::ResetWarmStart:
		ioC.ResetWarmStart();
		break;

	case ECSOp::SaveRestoreState:
		{
			StateRecorderImpl recorder;
			ioC.SaveState(recorder);
			ioC.ResetWarmStart();
			ioC.RestoreState(recorder);
			break;
		}

	case ECSOp::SetSolverSettings:
		ioC.SetConstraintPriority(i[0]);
		ioC.SetNumVelocityStepsOverride(i[1]);
		ioC.SetNumPositionStepsOverride(i[2]);
		ioC.SetUserData(i[3]);
		break;

	default:
		CSApplyKindOp(ioC, inOp, inS);
		break;
	}
}

// The draw world of group D3 (DrawWorld in ConstraintsParity.zig)
struct CSDrawWorld
{
	CSLayers					mLayers;
	BodyManager					mBodyManager;
	ConstraintManager			mConstraintManager { JPH_IF_ENABLE_ASSERTS(&mBodyManager) };
	Array<CSScenario>			mScenarios;
	Array<Body *>				mBodies;					///< Body 1 and body 2 of every scenario
	Array<Ref<Constraint>>		mConstraints;
	Array<uint32>				mFrames;					///< The next frame of every scenario
};

static void CSApplyOps(Rec &ioRec, TwoBodyConstraint &ioC, const CSScenario &inScenario, uint32 inFrame)
{
	for (uint32 o = 0; o < inScenario.mNumOps; ++o)
	{
		const CSOp &op = inScenario.mOps[o];
		if (op.mFrame == inFrame)
		{
			CSApplyOp(ioC, op, inScenario.mSettings);
			CSStep(ioRec, 0x100 | op.mOp, 0, ioC);
		}
	}
}

} // namespace

// The settings of the description inSettings[inIndex] (a CSSettings array) for the other wrapper files: the constraints
// of the PhysicsScene parity in BodyInterfaceReference.cpp, which declares this function (its Zolt side is
// makeDescriptionSettings of ConstraintsParity.zig). Every kind of CSMakeSettings, any other kind aborts.
Ref<TwoBodyConstraintSettings> CSMakeConstraintSettings(const void *inSettings, uint32 inIndex)
{
	CSEnsureFactory();
	return CSMakeSettings(static_cast<const CSSettings *>(inSettings)[inIndex]);
}

extern "C"
{

void jolt_cs_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(CSBodyDesc);
	outSizes[1] = sizeof(CSSettings);
	outSizes[2] = sizeof(CSOp);
	outSizes[3] = sizeof(CSScenario);
	outSizes[4] = sizeof(CSOutput);
	outSizes[5] = offsetof(CSSettings, mUserData);
	outSizes[6] = offsetof(CSSettings, mR);
	outSizes[7] = offsetof(CSSettings, mF);
	outSizes[8] = offsetof(CSOp, mR);
	outSizes[9] = offsetof(CSScenario, mSettings);
	outSizes[10] = offsetof(CSScenario, mOps);
	outSizes[11] = offsetof(CSOutput, mNumR);
}

// 1 when the allocation hooks of ParityHooks.h are installed after CSEnsureFactory, which every function of this file
// that allocates through Jolt calls first
uint32 jolt_cs_hooks_installed()
{
	CSEnsureFactory();
	return ParityHooksInstalled()? 1 : 0;
}

void *jolt_cs_world_create()
{
	CSEnsureFactory();
	CSWorld *world = new CSWorld;
	world->mBodyManager.Init(8, 0, world->mLayers);
	return world;
}

void jolt_cs_world_destroy(void *inWorld)
{
	delete static_cast<CSWorld *>(inWorld);
}

// Run a scenario (see the file comment)
void jolt_cs_run(void *inWorld, const CSScenario *inScenario, CSOutput *outOutput)
{
	CSEnsureFactory();
	BodyManager &bm = static_cast<CSWorld *>(inWorld)->mBodyManager;
	Rec rec(outOutput);

	Body *body1 = inScenario->mBody[0].mFixedToWorld != 0? &Body::sFixedToWorld : CSCreateBody(bm, inScenario->mBody[0]);
	Body *body2 = CSCreateBody(bm, inScenario->mBody[1]);
	{
		Ref<TwoBodyConstraintSettings> settings = CSMakeSettings(inScenario->mSettings);
		Ref<TwoBodyConstraint> constraint = settings->Create(*body1, *body2);
		TwoBodyConstraint &c = *constraint;

		// Created
		rec.I(uint32(c.GetType()));
		rec.I(uint32(c.GetSubType()));
		CSRecordSettingsGetters(rec, *settings, c.GetSubType());
		CSStep(rec, 0, 0, c);
		CSRecordSettings(rec, c);

		// The frames
		for (uint32 frame = 0; frame < inScenario->mNumFrames; ++frame)
		{
			CSApplyOps(rec, c, *inScenario, frame);

			c.SetupVelocityConstraint(inScenario->mDeltaTime);
			CSStep(rec, 1, 0, c);

			c.WarmStartVelocityConstraint(inScenario->mWarmStartRatio[frame]);
			CSStep(rec, 2, 0, c);

			for (uint32 v = 0; v < inScenario->mNumVelocitySteps; ++v)
			{
				bool result = c.SolveVelocityConstraint(inScenario->mDeltaTime);
				CSStep(rec, 3, result? 1 : 0, c);
			}

			for (uint32 p = 0; p < inScenario->mNumPositionSteps; ++p)
			{
				bool result = c.SolvePositionConstraint(inScenario->mDeltaTime, inScenario->mBaumgarte);
				CSStep(rec, 4, result? 1 : 0, c);
			}
		}
		CSApplyOps(rec, c, *inScenario, inScenario->mNumFrames);

		// The final settings
		CSRecordSettings(rec, c);
	}
	CSDestroyBody(bm, body2);
	CSDestroyBody(bm, body1);
}

// SaveBinaryState of the settings of a description, returns the size (the bytes are written when they fit)
uint32 jolt_cs_settings_save(const CSSettings *inSettings, uint8 *outBytes, uint32 inCapacity)
{
	CSEnsureFactory();
	Ref<TwoBodyConstraintSettings> settings = CSMakeSettings(*inSettings);
	std::string data = CSSaveSettings(*settings);
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// ConstraintSettings::sRestoreFromBinaryState of inBytes, then SaveBinaryState of the result. Returns the size of the
// saved bytes, or 0xFFFFFFFF with the error text in outError.
uint32 jolt_cs_settings_resave(const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity, char *outError, uint32 inErrorCapacity)
{
	CSEnsureFactory();
	std::stringstream in_data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(in_data);
	ConstraintSettings::ConstraintResult result = ConstraintSettings::sRestoreFromBinaryState(in);
	if (result.HasError())
	{
		const String &error = result.GetError();
		size_t n = std::min<size_t>(error.size(), inErrorCapacity - 1);
		memcpy(outError, error.data(), n);
		outError[n] = 0;
		return 0xFFFFFFFF;
	}
	std::string data = CSSaveSettings(*result.Get());
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// The draw world of group D3 (DrawWorld.create in ConstraintsParity.zig): the bodies of the scenarios in scenario order,
// then the constraints, added to the ConstraintManager in one call. Its blocks are zero filled (CSEnsureFactory).
void *jolt_cs_draw_world_create(const CSScenario *inScenarios, uint32 inCount)
{
	CSEnsureFactory();
	CSDrawWorld *world = new CSDrawWorld;
	world->mScenarios.assign(inScenarios, inScenarios + inCount);
	world->mFrames.resize(inCount, 0);
	world->mBodyManager.Init(2 * inCount, 0, world->mLayers);
	for (const CSScenario &sc : world->mScenarios)
	{
		world->mBodies.push_back(sc.mBody[0].mFixedToWorld != 0? &Body::sFixedToWorld : CSCreateBody(world->mBodyManager, sc.mBody[0]));
		world->mBodies.push_back(CSCreateBody(world->mBodyManager, sc.mBody[1]));
	}
	for (uint32 i = 0; i < inCount; ++i)
	{
		Ref<TwoBodyConstraintSettings> settings = CSMakeSettings(world->mScenarios[i].mSettings);
		world->mConstraints.push_back(settings->Create(*world->mBodies[2 * i], *world->mBodies[2 * i + 1]));
	}
	Array<Constraint *> constraints;
	for (Constraint *c : world->mConstraints)
		constraints.push_back(c);
	world->mConstraintManager.Add(constraints.data(), int(constraints.size()));
	return world;
}

void jolt_cs_draw_world_destroy(void *inWorld)
{
	CSDrawWorld *world = static_cast<CSDrawWorld *>(inWorld);
	for (size_t i = world->mBodies.size(); i > 0; i -= 2)
	{
		CSDestroyBody(world->mBodyManager, world->mBodies[i - 1]);
		CSDestroyBody(world->mBodyManager, world->mBodies[i - 2]);
	}
	delete world;
}

Constraint *jolt_cs_draw_world_constraint(void *inWorld, uint32 inIndex)
{
	return static_cast<CSDrawWorld *>(inWorld)->mConstraints[inIndex];
}

ConstraintManager *jolt_cs_draw_world_constraint_manager(void *inWorld)
{
	return &static_cast<CSDrawWorld *>(inWorld)->mConstraintManager;
}

BodyManager *jolt_cs_draw_world_body_manager(void *inWorld)
{
	return &static_cast<CSDrawWorld *>(inWorld)->mBodyManager;
}

// The next frame of the scenario of constraint inIndex (DrawWorld.step in ConstraintsParity.zig)
void jolt_cs_draw_world_step(void *inWorld, uint32 inIndex)
{
	CSDrawWorld *world = static_cast<CSDrawWorld *>(inWorld);
	const CSScenario &sc = world->mScenarios[inIndex];
	uint32 frame = world->mFrames[inIndex];
	if (frame > sc.mNumFrames)
		return;
	world->mFrames[inIndex] = frame + 1;
	TwoBodyConstraint &c = static_cast<TwoBodyConstraint &>(*world->mConstraints[inIndex]);
	for (uint32 o = 0; o < sc.mNumOps; ++o)
		if (sc.mOps[o].mFrame == frame)
			CSApplyOp(c, sc.mOps[o], sc.mSettings);
	if (frame == sc.mNumFrames)
		return;
	c.SetupVelocityConstraint(sc.mDeltaTime);
	c.WarmStartVelocityConstraint(sc.mWarmStartRatio[frame]);
	for (uint32 v = 0; v < sc.mNumVelocitySteps; ++v)
		c.SolveVelocityConstraint(sc.mDeltaTime);
	for (uint32 p = 0; p < sc.mNumPositionSteps; ++p)
		c.SolvePositionConstraint(sc.mDeltaTime, sc.mBaumgarte);
}

} // extern "C"
