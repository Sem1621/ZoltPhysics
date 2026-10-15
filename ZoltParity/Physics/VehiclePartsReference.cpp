// Reference implementation for the vehicle parts parity tests (`zig build parity`): thin C ABI wrappers around the
// standalone parts of the C++ Jolt library's vehicles: VehicleEngine, VehicleTransmission,
// VehicleDifferentialSettings::CalculateTorqueRatio, the default curves, and the binary state of every vehicle settings
// class (VehicleEngineSettings, VehicleTransmissionSettings, VehicleDifferentialSettings, VehicleAntiRollBar,
// VehicleTrackSettings, WheelSettings, WheelSettingsWV, WheeledVehicleControllerSettings and VehicleConstraintSettings
// without wheels). ZoltParity/Physics/VehiclePartsParity.zig calls these and checks that Zolt produces the same bits.
//
// The settings are built from descriptions (VP* structs, must match the extern structs of VehiclePartsParity.zig).
// Booleans are passed as uint32 (never bool, see the porting guide). All C functions have the prefix jolt_vp_. Functions
// that produce bytes write into a caller provided buffer and return the size.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Vehicle/VehicleAntiRollBar.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleDifferential.h>
#include <Jolt/Physics/Vehicle/VehicleEngine.h>
#include <Jolt/Physics/Vehicle/VehicleTrack.h>
#include <Jolt/Physics/Vehicle/VehicleTransmission.h>
#include <Jolt/Physics/Vehicle/Wheel.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

// Jolt allocates through the Allocate / Free hooks and VehicleConstraintSettings restores its controller through the
// Factory (same guard as BasicsReference.cpp, RegisterTypes() runs once for the whole parity binary)
static void VPEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static constexpr uint32 cVPMaxPoints = 8;
static constexpr uint32 cVPMaxRatios = 8;
static constexpr uint32 cVPMaxTrackWheels = 8;
static constexpr uint32 cVPMaxDifferentials = 4;
static constexpr uint32 cVPMaxAntiRollBars = 4;

// Must match VPCurve in VehiclePartsParity.zig
struct VPCurve
{
	uint32				mNumPoints;
	float				mPoints[2 * cVPMaxPoints];			///< x, y of each point (in this order, not sorted)
};

// Must match VPEngine in VehiclePartsParity.zig: VehicleEngineSettings
struct VPEngine
{
	float				mMaxTorque;
	float				mMinRPM;
	float				mMaxRPM;
	VPCurve				mNormalizedTorque;
	float				mInertia;
	float				mAngularDamping;
};

// Must match VPTransmission in VehiclePartsParity.zig: VehicleTransmissionSettings
struct VPTransmission
{
	uint32				mMode;										///< Any byte
	uint32				mNumGearRatios;
	float				mGearRatios[cVPMaxRatios];
	uint32				mNumReverseGearRatios;
	float				mReverseGearRatios[cVPMaxRatios];
	float				mSwitchTime;
	float				mClutchReleaseTime;
	float				mSwitchLatency;
	float				mShiftUpRPM;
	float				mShiftDownRPM;
	float				mClutchStrength;
};

// Must match VPDifferential in VehiclePartsParity.zig: VehicleDifferentialSettings
struct VPDifferential
{
	int32				mLeftWheel;
	int32				mRightWheel;
	float				mDifferentialRatio;
	float				mLeftRightSplit;
	float				mLimitedSlipRatio;
	float				mEngineTorqueRatio;
};

// Must match VPAntiRollBar in VehiclePartsParity.zig: VehicleAntiRollBar
struct VPAntiRollBar
{
	int32				mLeftWheel;
	int32				mRightWheel;
	float				mStiffness;
};

// Must match VPTrack in VehiclePartsParity.zig: VehicleTrackSettings
struct VPTrack
{
	uint32				mDrivenWheel;
	uint32				mNumWheels;
	uint32				mWheels[cVPMaxTrackWheels];
	float				mInertia;
	float				mAngularDamping;
	float				mMaxBrakeTorque;
	float				mDifferentialRatio;
};

// Must match VPWheel in VehiclePartsParity.zig: WheelSettings and the members of WheelSettingsWV
struct VPWheel
{
	float				mPosition[3];
	float				mSuspensionForcePoint[3];
	float				mSuspensionDirection[3];
	float				mSteeringAxis[3];
	float				mWheelUp[3];
	float				mWheelForward[3];
	float				mSuspensionMinLength;
	float				mSuspensionMaxLength;
	float				mSuspensionPreloadLength;
	uint32				mSpringMode;								///< Any byte
	float				mSpringFrequency;
	float				mSpringDamping;
	float				mRadius;
	float				mWidth;
	uint32				mEnableSuspensionForcePoint;
	// WheelSettingsWV
	float				mInertia;
	float				mAngularDamping;
	float				mMaxSteerAngle;
	VPCurve				mLongitudinalFriction;
	VPCurve				mLateralFriction;
	float				mMaxBrakeTorque;
	float				mMaxHandBrakeTorque;
};

// Must match VPController in VehiclePartsParity.zig: WheeledVehicleControllerSettings
struct VPController
{
	VPEngine			mEngine;
	VPTransmission		mTransmission;
	uint32				mNumDifferentials;
	VPDifferential		mDifferentials[cVPMaxDifferentials];
	float				mDifferentialLimitedSlipRatio;
};

// Must match VPConstraint in VehiclePartsParity.zig: VehicleConstraintSettings without wheels
struct VPConstraint
{
	uint32				mEnabled;
	float				mDrawConstraintSize;
	uint32				mConstraintPriority;
	uint32				mNumVelocityStepsOverride;
	uint32				mNumPositionStepsOverride;
	float				mUp[3];
	float				mForward[3];
	float				mMaxPitchRollAngle;
	uint32				mNumAntiRollBars;
	VPAntiRollBar		mAntiRollBars[cVPMaxAntiRollBars];
	VPController		mController;
};

// The settings classes of jolt_vp_save / jolt_vp_resave (must match VPKind in VehiclePartsParity.zig)
enum class VPKind : uint32
{
	Engine,
	Transmission,
	Differential,
	AntiRollBar,
	Track,
	WheelSettings,
	WheelSettingsWV,
	Controller,
	Constraint,
};

// Must match VPEngineOp in VehiclePartsParity.zig
struct VPEngineOp
{
	uint32				mKind;										///< 0 SetCurrentRPM(a), 1 ApplyTorque(a, b), 2 ApplyDamping(a), 3 GetTorque(a), 4 GetAngularVelocity, 5 AllowSleep, 6 ClampRPM, 7 a * cAngularVelocityToRPM
	float				mA;
	float				mB;
};

// Must match VPTransmissionOp in VehiclePartsParity.zig
struct VPTransmissionOp
{
	uint32				mKind;										///< 0 Update(delta time, rpm, forward, can shift up), 1 Set(gear, clutch friction)
	float				mDeltaTime;
	float				mRPM;
	float				mForward;
	uint32				mCanShiftUp;
	int32				mGear;
	float				mClutchFriction;
};

// Must match VPTransmissionResult in VehiclePartsParity.zig
struct VPTransmissionResult
{
	int32				mGear;
	float				mClutchFriction;
	float				mRatio;
	uint32				mIsSwitchingGear;
	uint32				mAllowSleep;
	uint8				mState[20];									///< SaveState: gear, clutch friction and the three timers
};

static void VPMakeCurve(const VPCurve &inDesc, LinearCurve &outCurve)
{
	outCurve.Clear();
	for (uint32 i = 0; i < inDesc.mNumPoints; ++i)
		outCurve.AddPoint(inDesc.mPoints[2 * i], inDesc.mPoints[2 * i + 1]);
}

static void VPMakeEngine(const VPEngine &inDesc, VehicleEngineSettings &outSettings)
{
	outSettings.mMaxTorque = inDesc.mMaxTorque;
	outSettings.mMinRPM = inDesc.mMinRPM;
	outSettings.mMaxRPM = inDesc.mMaxRPM;
	VPMakeCurve(inDesc.mNormalizedTorque, outSettings.mNormalizedTorque);
	outSettings.mInertia = inDesc.mInertia;
	outSettings.mAngularDamping = inDesc.mAngularDamping;
}

static void VPMakeTransmission(const VPTransmission &inDesc, VehicleTransmissionSettings &outSettings)
{
	outSettings.mMode = ETransmissionMode(uint8(inDesc.mMode));
	outSettings.mGearRatios.assign(inDesc.mGearRatios, inDesc.mGearRatios + inDesc.mNumGearRatios);
	outSettings.mReverseGearRatios.assign(inDesc.mReverseGearRatios, inDesc.mReverseGearRatios + inDesc.mNumReverseGearRatios);
	outSettings.mSwitchTime = inDesc.mSwitchTime;
	outSettings.mClutchReleaseTime = inDesc.mClutchReleaseTime;
	outSettings.mSwitchLatency = inDesc.mSwitchLatency;
	outSettings.mShiftUpRPM = inDesc.mShiftUpRPM;
	outSettings.mShiftDownRPM = inDesc.mShiftDownRPM;
	outSettings.mClutchStrength = inDesc.mClutchStrength;
}

static void VPMakeDifferential(const VPDifferential &inDesc, VehicleDifferentialSettings &outSettings)
{
	outSettings.mLeftWheel = inDesc.mLeftWheel;
	outSettings.mRightWheel = inDesc.mRightWheel;
	outSettings.mDifferentialRatio = inDesc.mDifferentialRatio;
	outSettings.mLeftRightSplit = inDesc.mLeftRightSplit;
	outSettings.mLimitedSlipRatio = inDesc.mLimitedSlipRatio;
	outSettings.mEngineTorqueRatio = inDesc.mEngineTorqueRatio;
}

static void VPMakeAntiRollBar(const VPAntiRollBar &inDesc, VehicleAntiRollBar &outBar)
{
	outBar.mLeftWheel = inDesc.mLeftWheel;
	outBar.mRightWheel = inDesc.mRightWheel;
	outBar.mStiffness = inDesc.mStiffness;
}

static void VPMakeTrack(const VPTrack &inDesc, VehicleTrackSettings &outSettings)
{
	outSettings.mDrivenWheel = inDesc.mDrivenWheel;
	outSettings.mWheels.assign(inDesc.mWheels, inDesc.mWheels + inDesc.mNumWheels);
	outSettings.mInertia = inDesc.mInertia;
	outSettings.mAngularDamping = inDesc.mAngularDamping;
	outSettings.mMaxBrakeTorque = inDesc.mMaxBrakeTorque;
	outSettings.mDifferentialRatio = inDesc.mDifferentialRatio;
}

static Vec3 VPVec3(const float *inV)
{
	return Vec3(inV[0], inV[1], inV[2]);
}

static void VPMakeWheel(const VPWheel &inDesc, WheelSettings &outSettings)
{
	outSettings.mPosition = VPVec3(inDesc.mPosition);
	outSettings.mSuspensionForcePoint = VPVec3(inDesc.mSuspensionForcePoint);
	outSettings.mSuspensionDirection = VPVec3(inDesc.mSuspensionDirection);
	outSettings.mSteeringAxis = VPVec3(inDesc.mSteeringAxis);
	outSettings.mWheelUp = VPVec3(inDesc.mWheelUp);
	outSettings.mWheelForward = VPVec3(inDesc.mWheelForward);
	outSettings.mSuspensionMinLength = inDesc.mSuspensionMinLength;
	outSettings.mSuspensionMaxLength = inDesc.mSuspensionMaxLength;
	outSettings.mSuspensionPreloadLength = inDesc.mSuspensionPreloadLength;
	outSettings.mSuspensionSpring.mMode = ESpringMode(uint8(inDesc.mSpringMode));
	outSettings.mSuspensionSpring.mFrequency = inDesc.mSpringFrequency;
	outSettings.mSuspensionSpring.mDamping = inDesc.mSpringDamping;
	outSettings.mRadius = inDesc.mRadius;
	outSettings.mWidth = inDesc.mWidth;
	outSettings.mEnableSuspensionForcePoint = inDesc.mEnableSuspensionForcePoint != 0;
}

static void VPMakeWheelWV(const VPWheel &inDesc, WheelSettingsWV &outSettings)
{
	VPMakeWheel(inDesc, outSettings);
	outSettings.mInertia = inDesc.mInertia;
	outSettings.mAngularDamping = inDesc.mAngularDamping;
	outSettings.mMaxSteerAngle = inDesc.mMaxSteerAngle;
	VPMakeCurve(inDesc.mLongitudinalFriction, outSettings.mLongitudinalFriction);
	VPMakeCurve(inDesc.mLateralFriction, outSettings.mLateralFriction);
	outSettings.mMaxBrakeTorque = inDesc.mMaxBrakeTorque;
	outSettings.mMaxHandBrakeTorque = inDesc.mMaxHandBrakeTorque;
}

static void VPMakeController(const VPController &inDesc, WheeledVehicleControllerSettings &outSettings)
{
	VPMakeEngine(inDesc.mEngine, outSettings.mEngine);
	VPMakeTransmission(inDesc.mTransmission, outSettings.mTransmission);
	outSettings.mDifferentials.resize(inDesc.mNumDifferentials);
	for (uint32 i = 0; i < inDesc.mNumDifferentials; ++i)
		VPMakeDifferential(inDesc.mDifferentials[i], outSettings.mDifferentials[i]);
	outSettings.mDifferentialLimitedSlipRatio = inDesc.mDifferentialLimitedSlipRatio;
}

static void VPMakeConstraint(const VPConstraint &inDesc, VehicleConstraintSettings &outSettings)
{
	outSettings.mEnabled = inDesc.mEnabled != 0;
	outSettings.mDrawConstraintSize = inDesc.mDrawConstraintSize;
	outSettings.mConstraintPriority = inDesc.mConstraintPriority;
	outSettings.mNumVelocityStepsOverride = inDesc.mNumVelocityStepsOverride;
	outSettings.mNumPositionStepsOverride = inDesc.mNumPositionStepsOverride;
	outSettings.mUp = VPVec3(inDesc.mUp);
	outSettings.mForward = VPVec3(inDesc.mForward);
	outSettings.mMaxPitchRollAngle = inDesc.mMaxPitchRollAngle;
	outSettings.mAntiRollBars.resize(inDesc.mNumAntiRollBars);
	for (uint32 i = 0; i < inDesc.mNumAntiRollBars; ++i)
		VPMakeAntiRollBar(inDesc.mAntiRollBars[i], outSettings.mAntiRollBars[i]);
	WheeledVehicleControllerSettings *controller = new WheeledVehicleControllerSettings;
	VPMakeController(inDesc.mController, *controller);
	outSettings.mController = controller;
}

static uint32 VPCopyBytes(const std::string &inData, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inData.size());
	memcpy(outBytes, inData.data(), std::min(size, inCapacity));
	return size;
}

template <class T>
static std::string VPSave(const T &inObject)
{
	std::stringstream data;
	StreamOutWrapper out(data);
	inObject.SaveBinaryState(out);
	return data.str();
}

// Build the object of class inKind from the description inDesc (a VPEngine, VPTransmission, ... of the kind) and call
// inFunction(object) with it
template <class F>
static void VPWithObject(VPKind inKind, const void *inDesc, const F &inFunction)
{
	switch (inKind)
	{
	case VPKind::Engine:
		{
			VehicleEngineSettings s;
			if (inDesc != nullptr)
				VPMakeEngine(*static_cast<const VPEngine *>(inDesc), s);
			inFunction(s);
			break;
		}

	case VPKind::Transmission:
		{
			VehicleTransmissionSettings s;
			if (inDesc != nullptr)
				VPMakeTransmission(*static_cast<const VPTransmission *>(inDesc), s);
			inFunction(s);
			break;
		}

	case VPKind::Differential:
		{
			VehicleDifferentialSettings s;
			if (inDesc != nullptr)
				VPMakeDifferential(*static_cast<const VPDifferential *>(inDesc), s);
			inFunction(s);
			break;
		}

	case VPKind::AntiRollBar:
		{
			VehicleAntiRollBar s;
			if (inDesc != nullptr)
				VPMakeAntiRollBar(*static_cast<const VPAntiRollBar *>(inDesc), s);
			inFunction(s);
			break;
		}

	case VPKind::Track:
		{
			VehicleTrackSettings s;
			s.mDrivenWheel = 0; // Uninitialized in Jolt, Zolt's default (CharacterVehicleArchitecture)
			if (inDesc != nullptr)
				VPMakeTrack(*static_cast<const VPTrack *>(inDesc), s);
			inFunction(s);
			break;
		}

	case VPKind::WheelSettings:
		{
			Ref<WheelSettings> s = new WheelSettings;
			if (inDesc != nullptr)
				VPMakeWheel(*static_cast<const VPWheel *>(inDesc), *s);
			inFunction(*s);
			break;
		}

	case VPKind::WheelSettingsWV:
		{
			Ref<WheelSettingsWV> s = new WheelSettingsWV;
			if (inDesc != nullptr)
				VPMakeWheelWV(*static_cast<const VPWheel *>(inDesc), *s);
			inFunction(*s);
			break;
		}

	case VPKind::Controller:
		{
			Ref<WheeledVehicleControllerSettings> s = new WheeledVehicleControllerSettings;
			if (inDesc != nullptr)
				VPMakeController(*static_cast<const VPController *>(inDesc), *s);
			inFunction(*s);
			break;
		}

	case VPKind::Constraint:
		JPH_ASSERT(false); // Restored through sRestoreFromBinaryState, see jolt_vp_resave
		break;
	}
}

extern "C"
{

// The sizes of the structs that are shared with VehiclePartsParity.zig
void jolt_vp_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = uint32(sizeof(VPCurve));
	outSizes[1] = uint32(sizeof(VPEngine));
	outSizes[2] = uint32(sizeof(VPTransmission));
	outSizes[3] = uint32(sizeof(VPDifferential));
	outSizes[4] = uint32(sizeof(VPAntiRollBar));
	outSizes[5] = uint32(sizeof(VPTrack));
	outSizes[6] = uint32(sizeof(VPWheel));
	outSizes[7] = uint32(sizeof(VPController));
	outSizes[8] = uint32(sizeof(VPConstraint));
	outSizes[9] = uint32(sizeof(VPEngineOp));
	outSizes[10] = uint32(sizeof(VPTransmissionOp));
	outSizes[11] = uint32(sizeof(VPTransmissionResult));
}

// SaveBinaryState of the object of class inKind built from inDesc (null: the default constructed object), returns the
// size of the bytes
uint32 jolt_vp_save(uint32 inKind, const void *inDesc, uint8 *outBytes, uint32 inCapacity)
{
	VPEnsureFactory();
	if (VPKind(inKind) == VPKind::Constraint)
	{
		VehicleConstraintSettings s;
		if (inDesc != nullptr)
			VPMakeConstraint(*static_cast<const VPConstraint *>(inDesc), s);
		else
			s.mController = new WheeledVehicleControllerSettings; // SaveBinaryState needs a controller
		return VPCopyBytes(VPSave(s), outBytes, inCapacity);
	}

	std::string bytes;
	VPWithObject(VPKind(inKind), inDesc, [&bytes](const auto &inObject) { bytes = VPSave(inObject); });
	return VPCopyBytes(bytes, outBytes, inCapacity);
}

// RestoreBinaryState of inBytes into the object of class inKind built from inTemplate (null: the default constructed
// object), then SaveBinaryState of that object. Writes the EOF / failed flags of the stream after the restore and
// returns the size of the saved bytes. For VPKind::Constraint the bytes are restored with
// ConstraintSettings::sRestoreFromBinaryState (inTemplate is not used): outError is 0 on success, 1 "Failed to read type
// hash", 2 "Failed to create instance of type", 3 "Failed to restore object" (the size is then 0).
uint32 jolt_vp_resave(uint32 inKind, const void *inTemplate, const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity, uint32 *outEOF, uint32 *outFailed, uint32 *outError)
{
	VPEnsureFactory();
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(data);
	*outError = 0;

	if (VPKind(inKind) == VPKind::Constraint)
	{
		ConstraintSettings::ConstraintResult result = ConstraintSettings::sRestoreFromBinaryState(in);
		*outEOF = in.IsEOF()? 1 : 0;
		*outFailed = in.IsFailed()? 1 : 0;
		if (result.HasError())
		{
			const String &error = result.GetError();
			*outError = error == "Failed to read type hash"? 1 : (error == "Failed to create instance of type"? 2 : (error == "Failed to restore object"? 3 : 4));
			return 0;
		}
		return VPCopyBytes(VPSave(*result.Get()), outBytes, inCapacity);
	}

	std::string bytes;
	VPWithObject(VPKind(inKind), inTemplate, [&](auto &ioObject) {
		ioObject.RestoreBinaryState(in);
		*outEOF = in.IsEOF()? 1 : 0;
		*outFailed = in.IsFailed()? 1 : 0;
		bytes = VPSave(ioObject);
	});
	return VPCopyBytes(bytes, outBytes, inCapacity);
}

// The defaults that the binary state does not hold: VehicleEngineSettings' mInertia and mAngularDamping, the default
// VehicleEngine's current RPM
void jolt_vp_engine_defaults(float *outInertia, float *outAngularDamping, float *outCurrentRPM)
{
	VPEnsureFactory();
	VehicleEngine engine;
	*outInertia = engine.mInertia;
	*outAngularDamping = engine.mAngularDamping;
	*outCurrentRPM = engine.GetCurrentRPM();
}

// A VehicleEngine with the settings inDesc (assigned to the default constructed engine like WheeledVehicleController
// does, without setting the RPM) runs the operations; after each one outRPM gets the current RPM and outResult the
// result of the operation (0 when it has none, AllowSleep as 0 / 1)
void jolt_vp_engine(const VPEngine *inDesc, const VPEngineOp *inOps, uint32 inNumOps, float *outRPM, float *outResult)
{
	VPEnsureFactory();
	VehicleEngine engine;
	VPMakeEngine(*inDesc, engine);
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const VPEngineOp &op = inOps[i];
		float result = 0.0f;
		switch (op.mKind)
		{
		case 0:		engine.SetCurrentRPM(op.mA);								break;
		case 1:		engine.ApplyTorque(op.mA, op.mB);							break;
		case 2:		engine.ApplyDamping(op.mA);									break;
		case 3:		result = engine.GetTorque(op.mA);							break;
		case 4:		result = engine.GetAngularVelocity();						break;
		case 5:		result = engine.AllowSleep()? 1.0f : 0.0f;					break;
		case 6:		engine.ClampRPM();											break;
		case 7:		result = op.mA * VehicleEngine::cAngularVelocityToRPM;		break;
		}
		outRPM[i] = engine.GetCurrentRPM();
		outResult[i] = result;
	}
}

// A VehicleTransmission with the settings inDesc runs the operations, the results after each one
void jolt_vp_transmission(const VPTransmission *inDesc, const VPTransmissionOp *inOps, uint32 inNumOps, VPTransmissionResult *outResults)
{
	VPEnsureFactory();
	VehicleTransmission transmission;
	VPMakeTransmission(*inDesc, transmission);
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const VPTransmissionOp &op = inOps[i];
		if (op.mKind == 0)
			transmission.Update(op.mDeltaTime, op.mRPM, op.mForward, op.mCanShiftUp != 0);
		else
			transmission.Set(op.mGear, op.mClutchFriction);

		VPTransmissionResult &r = outResults[i];
		r.mGear = transmission.GetCurrentGear();
		r.mClutchFriction = transmission.GetClutchFriction();
		r.mRatio = transmission.GetCurrentRatio();
		r.mIsSwitchingGear = transmission.IsSwitchingGear()? 1 : 0;
		r.mAllowSleep = transmission.AllowSleep()? 1 : 0;
		StateRecorderImpl recorder;
		transmission.SaveState(recorder);
		std::string state = recorder.GetData();
		JPH_ASSERT(state.size() == sizeof(r.mState));
		memcpy(r.mState, state.data(), sizeof(r.mState));
	}
}

// VehicleDifferentialSettings::CalculateTorqueRatio of inDesc for the angular velocity pairs inVelocities (left,
// right), the left and right fractions in outFractions
void jolt_vp_differential(const VPDifferential *inDesc, const float *inVelocities, uint32 inCount, float *outFractions)
{
	VPEnsureFactory();
	VehicleDifferentialSettings d;
	VPMakeDifferential(*inDesc, d);
	for (uint32 i = 0; i < inCount; ++i)
		d.CalculateTorqueRatio(inVelocities[2 * i], inVelocities[2 * i + 1], outFractions[2 * i], outFractions[2 * i + 1]);
}

// The default curves at inX: WheelSettingsWV's longitudinal and lateral friction, VehicleEngineSettings' normalized
// torque (3 values per x)
void jolt_vp_default_curves(const float *inX, uint32 inCount, float *outValues)
{
	VPEnsureFactory();
	Ref<WheelSettingsWV> wheel = new WheelSettingsWV;
	VehicleEngineSettings engine;
	for (uint32 i = 0; i < inCount; ++i)
	{
		outValues[3 * i] = wheel->mLongitudinalFriction.GetValue(inX[i]);
		outValues[3 * i + 1] = wheel->mLateralFriction.GetValue(inX[i]);
		outValues[3 * i + 2] = engine.mNormalizedTorque.GetValue(inX[i]);
	}
}

} // extern "C"
