// Reference implementation of the simulation parity tests (`zig build parity`, SimulationArchitecture D12): a
// JPH::PhysicsSystem with the layers of UnitTests/Layers.h, built from a scene description of
// ZoltParity/Physics/SimScenes.zig, stepped with a JobSystemThreadPool. ZoltParity/Physics/SimulationParity.zig builds
// the same scene with Zolt, applies the same operations in the same order and compares the SaveState bytes after every
// update and the listener callbacks (body activation, step listener, contact listener) as sorted records.
//
// The structs below must match the extern structs of SimScenes.zig (checked by jolt_sim_struct_sizes). Booleans are
// passed as uint32 (never bool, see the porting guide). All C functions have the prefix jolt_sim_.
//
// Constraints (Phase 6): this side creates every two body constraint class from its description (SimConstraint,
// SimMakeConstraintSettings: the layout of CSMakeSettings in ConstraintsReference.cpp plus the SetRatio helpers of Gear /
// RackAndPinion) and applies every setter of every class (ApplyConstraintOp, one block per ConstraintOpRange of
// SimScenes.zig). A later worker group only adds a new operation here, in its own range.
//
// Characters, ragdolls and vehicles (Phase 7): this side creates Character, CharacterVirtual (with the
// CharacterContactListener of SimCharacterContactListener and CharacterVsCharacterCollisionSimple), Ragdoll and
// VehicleConstraint (wheeled, tracked and motorcycle controllers, the three collision testers) from their descriptions and
// applies every setter and action (ApplyObjectOp, one block per Phase7OpRange). After every update the driver compares
// the object state (WriteObjectState: the SaveState bytes of the characters and the getters that the state does not
// hold) and the character listener records in call order; snapshot / restore_cross include the character states.
// The functions of the vehicle variants (tracked, motorcycle: SimMakeTrackedControllerSettings, ApplyTrackedOp,
// WriteTrackedControllerState, ...) are separate, like their slots in SimulationParity.zig. The binary state round
// trips of the vehicle and ragdoll settings (jolt_sim_vehicle_settings_part, jolt_sim_ragdoll_settings_bytes) and the
// CharacterID test (jolt_sim_character_id_sequence) are compared before a scene runs.
//
// Soft bodies (Phase 9, group SH): this side generates the shared settings of every SimSoftBodySettings
// (SimMakeSoftBodySettings: the same generators as makeSoftBodySettings of SimulationParity.zig, whose SaveBinaryState
// bytes jolt_sim_soft_body_settings_bytes returns for the comparison before a scene runs), creates the soft bodies from
// their SimSoftBody descriptions and applies every soft body operation (ApplySoftBodyOp). The soft body contact listener
// (SimSoftBodyContactListener, SoftBodyListenerMode) records into the recorder of the other listeners. The state decoder
// of SimulationParity.zig learns the array sizes of each soft body from jolt_sim_soft_body_info.
//
// The process hooks (ZoltParity/ParityHooks.h): zero-filled allocations (Jolt leaves members uninitialized that its
// states hold), a Trace that prints, and with JPH_ENABLE_ASSERTS (`zig build parity -Dreference_asserts=true`) an
// AssertFailed that prints and counts: the simulation parity fails a scene whose C++ side hit an assert.

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
#include <Jolt/Physics/Collision/CollideShapeVsShapePerLeaf.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
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
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>
#include <Jolt/Physics/Constraints/GearConstraint.h>
#include <Jolt/Physics/Constraints/RackAndPinionConstraint.h>
#include <Jolt/Physics/Constraints/PulleyConstraint.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/Physics/Vehicle/TrackedVehicleController.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodyContactListener.h>
#include <Jolt/Physics/SoftBody/SoftBodyManifold.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

using namespace JPH;

// The layers of the unit tests (Layers, BroadPhaseLayers, ObjectLayerPairFilterImpl, BPLayerInterfaceImpl,
// ObjectVsBroadPhaseLayerFilterImpl), mirrored by ZoltTests/Layers.zig
#include <UnitTests/Layers.h>

// The hooks of the parity process (ZoltParity/ParityHooks.h): Jolt's allocation functions with zero-filled blocks, Trace
// and AssertFailed. The header installs them before main, so before any Jolt allocation of the process, and
// SimEnsureFactory installs them again. The other reference files call RegisterDefaultAllocator(), which installs Jolt's
// allocator again: it is compatible (both free with free), and zero filling changes nothing that is defined.
#include <ZoltParity/ParityHooks.h>

// The hooks (another reference file may have installed Jolt's allocator since) and the Factory that the shapes need
// (same guard as BasicsReference.cpp, RegisterTypes() runs once for the whole parity binary)
static void SimEnsureFactory()
{
	ParityInstallHooks();
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

// Must match SimConstraint in SimScenes.zig: the ConstraintSettings members and the members of the settings class in
// mI / mR / mF (the layout per ConstraintKind is listed in SimScenes.zig and at SimMakeConstraintSettings)
struct SimConstraint
{
	uint32				mKind;						// ConstraintKind of SimScenes.zig (not EConstraintSubType)
	uint32				mBody1;						// Body handles, cInvalidHandle: Body::sFixedToWorld
	uint32				mBody2;
	uint32				mConstraint1;				// Gear / RackAndPinion: the constraint handles of SetConstraints
	uint32				mConstraint2;
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

// Must match SimOp in SimScenes.zig
struct SimOp
{
	uint32				mUpdate;
	uint32				mKind;
	uint32				mBody;
	uint32				mCount;
	uint32				mActivation;
	uint32				mIndex;
	uint32				mI[4];
	float				mF[16];
	Real				mR[3];
};

// Phase 7 (SimScenes.zig "Characters, ragdolls and vehicles"): the descriptions of the characters, ragdolls and vehicles

// Must match SimCharacterBase in SimScenes.zig: the CharacterBaseSettings members
struct SimCharacterBase
{
	float				mUp[3];
	float				mSupportingVolume[4];						// Normal xyz, constant
	float				mMaxSlopeAngle;
	uint32				mEnhancedInternalEdgeRemoval;
	uint32				mShape;										// Shape index, cInvalidHandle: none
};

// Must match SimCharacter in SimScenes.zig
struct SimCharacter
{
	SimCharacterBase	mBase;
	uint32				mLayer;
	float				mMass;
	float				mFriction;
	float				mGravityFactor;
	uint32				mAllowedDOFs;
	Real				mPosition[3];
	float				mRotation[4];
	uint64				mUserData;
};

// Must match SimCharacterVirtual in SimScenes.zig
struct SimCharacterVirtual
{
	SimCharacterBase	mBase;
	uint32				mID;
	float				mMass;
	float				mMaxStrength;
	float				mShapeOffset[3];
	uint32				mBackFaceMode;
	float				mPredictiveContactDistance;
	uint32				mMaxCollisionIterations;
	uint32				mMaxConstraintIterations;
	float				mMinTimeRemaining;
	float				mCollisionTolerance;
	float				mCharacterPadding;
	uint32				mMaxNumHits;
	float				mHitReductionCosMaxAngle;
	float				mPenetrationRecoverySpeed;
	uint32				mInnerBodyShape;							// Shape index, cInvalidHandle: no inner body
	uint32				mInnerBodyIDOverride;
	uint32				mInnerBodyLayer;
	Real				mPosition[3];
	float				mRotation[4];
	uint64				mUserData;
	uint32				mListener;									// CharacterListenerMode of SimScenes.zig
	uint32				mCollisionGroup;							// CharacterVsCharacterCollisionSimple index, cInvalidHandle: none
};

// Must match SimRagdoll in SimScenes.zig
struct SimRagdoll
{
	uint32				mFirstJoint;
	uint32				mNumJoints;
	uint32				mFirstAdditional;
	uint32				mNumAdditional;
	uint32				mStabilize;
	uint32				mCalculateConstraintPriorities;
	uint32				mBasePriority;
	uint32				mDisableParentChildCollisions;
	uint32				mPose;
	float				mMinSeparationDistance;
	uint32				mCollisionGroup;
	uint64				mUserData;
};

// Must match SimRagdollJoint in SimScenes.zig
struct SimRagdollJoint
{
	int32				mParent;
	uint32				mBody;
	uint32				mToParent;
};

// Must match SimRagdollAdditionalConstraint in SimScenes.zig
struct SimRagdollAdditionalConstraint
{
	int32				mBody1;
	int32				mBody2;
	uint32				mConstraint;
};

// Must match SimRagdollPose in SimScenes.zig
struct SimRagdollPose
{
	Real				mRootOffset[3];
	uint32				mData;
	uint32				mCount;
};

// Must match SimVehicleEngine in SimScenes.zig
struct SimVehicleEngine
{
	float				mMaxTorque;
	float				mMinRPM;
	float				mMaxRPM;
	uint32				mTorquePoints;								// Offset in mFloats, cInvalidHandle: the constructor's curve
	uint32				mNumTorquePoints;
	float				mInertia;
	float				mAngularDamping;
};

// Must match SimVehicleTransmission in SimScenes.zig
struct SimVehicleTransmission
{
	uint32				mMode;
	uint32				mGearRatios;								// Offset in mFloats, cInvalidHandle: the constructor's ratios
	uint32				mNumGearRatios;
	uint32				mReverseGearRatios;
	uint32				mNumReverseGearRatios;
	float				mSwitchTime;
	float				mClutchReleaseTime;
	float				mSwitchLatency;
	float				mShiftUpRPM;
	float				mShiftDownRPM;
	float				mClutchStrength;
};

// Must match SimVehicleTrack in SimScenes.zig
struct SimVehicleTrack
{
	uint32				mDrivenWheel;
	uint32				mWheels;									// Offset in mUints
	uint32				mNumWheels;
	float				mInertia;
	float				mAngularDamping;
	float				mMaxBrakeTorque;
	float				mDifferentialRatio;
};

// Must match SimVehicleDifferential in SimScenes.zig
struct SimVehicleDifferential
{
	int32				mLeftWheel;
	int32				mRightWheel;
	float				mDifferentialRatio;
	float				mLeftRightSplit;
	float				mLimitedSlipRatio;
	float				mEngineTorqueRatio;
};

// Must match SimVehicleAntiRollBar in SimScenes.zig
struct SimVehicleAntiRollBar
{
	int32				mLeftWheel;
	int32				mRightWheel;
	float				mStiffness;
};

// Must match SimVehicleWheel in SimScenes.zig
struct SimVehicleWheel
{
	uint32				mKind;										// VehicleWheelKind: 0 WV, 1 TV
	float				mPosition[3];
	float				mSuspensionForcePoint[3];
	float				mSuspensionDirection[3];
	float				mSteeringAxis[3];
	float				mWheelUp[3];
	float				mWheelForward[3];
	float				mSuspensionMinLength;
	float				mSuspensionMaxLength;
	float				mSuspensionPreloadLength;
	uint32				mSuspensionSpringMode;
	float				mSuspensionFrequency;
	float				mSuspensionDamping;
	float				mRadius;
	float				mWidth;
	uint32				mEnableSuspensionForcePoint;
	float				mInertia;
	float				mAngularDamping;
	float				mMaxSteerAngle;
	uint32				mLongitudinalFriction;						// Offset in mFloats, cInvalidHandle: the constructor's curve
	uint32				mNumLongitudinalFriction;
	uint32				mLateralFriction;
	uint32				mNumLateralFriction;
	float				mMaxBrakeTorque;
	float				mMaxHandBrakeTorque;
	float				mLongitudinalFrictionTV;
	float				mLateralFrictionTV;
};

// Must match SimVehicleTester in SimScenes.zig
struct SimVehicleTester
{
	uint32				mKind;										// VehicleTesterKind: 0 ray, 1 cast sphere, 2 cast cylinder
	uint32				mObjectLayer;
	float				mUp[3];
	float				mMaxSlopeAngle;
	float				mRadius;
	float				mConvexRadiusFraction;
};

// Must match SimVehicle in SimScenes.zig
struct SimVehicle
{
	uint32				mBody;
	uint32				mEnabled;
	uint32				mConstraintPriority;
	uint32				mNumVelocityStepsOverride;
	uint32				mNumPositionStepsOverride;
	float				mDrawConstraintSize;
	uint64				mUserData;
	float				mUp[3];
	float				mForward[3];
	float				mMaxPitchRollAngle;
	uint32				mFirstWheel;
	uint32				mNumWheels;
	uint32				mFirstAntiRollBar;
	uint32				mNumAntiRollBars;
	uint32				mController;								// VehicleControllerKind: 0 wheeled, 1 tracked, 2 motorcycle
	SimVehicleEngine	mEngine;
	SimVehicleTransmission mTransmission;
	uint32				mFirstDifferential;
	uint32				mNumDifferentials;
	float				mDifferentialLimitedSlipRatio;
	SimVehicleTrack		mTracks[2];
	float				mMaxLeanAngle;
	float				mLeanSpringConstant;
	float				mLeanSpringDamping;
	float				mLeanSpringIntegrationCoefficient;
	float				mLeanSpringIntegrationCoefficientDecay;
	float				mLeanSmoothingFactor;
	SimVehicleTester	mTester;
	uint32				mCallbacks;									// VehicleCallbacks bits
	uint32				mCombineFriction;							// VehicleCombineFriction
	uint32				mTireMaxImpulse;							// VehicleTireMaxImpulse
};

// Phase 9: must match SimSoftBodySettings in SimScenes.zig (a generator of shared settings, see SimMakeSoftBodySettings)
struct SimSoftBodySettings
{
	uint32				mKind;										// SoftBodySettingsKind
	uint32				mCountX;
	uint32				mCountZ;
	uint32				mCountExtra;
	float				mSpacing;
	float				mHeight;
	float				mCompliance;
	float				mShearCompliance;
	float				mBendCompliance;
	uint32				mLRAType;
	float				mLRAMaxDistanceMultiplier;
	uint32				mBendType;
	uint32				mOptimize;
	uint32				mKinematic;
	float				mSkinMaxDistance;
	float				mBackStopDistance;
	float				mBackStopRadius;
	uint32				mFloats;
	uint32				mUints;
};

// Phase 9: must match SimSoftBody in SimScenes.zig (SoftBodyCreationSettings)
struct SimSoftBody
{
	uint64				mUserData;
	Real				mPosition[3];
	float				mRotation[4];
	uint32				mSettings;									// Index of the SimSoftBodySettings
	uint32				mObjectLayer;
	uint32				mCollisionGroupID;
	uint32				mCollisionSubGroupID;
	uint32				mNumIterations;
	float				mLinearDamping;
	float				mMaxLinearVelocity;
	float				mRestitution;
	float				mFriction;
	float				mPressure;
	float				mGravityFactor;
	float				mVertexRadius;
	uint32				mUpdatePosition;
	uint32				mMakeRotationIdentity;
	uint32				mAllowSleeping;
	uint32				mFacesDoubleSided;
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
	uint32						mTempAllocatorSize;
	uint32						mSimCollideMode;
	// Phase 7
	const SimCharacter *		mCharacters;
	uint32						mNumCharacters;
	const SimCharacterVirtual *	mCharacterVirtuals;
	uint32						mNumCharacterVirtuals;
	const SimRagdoll *			mRagdolls;
	uint32						mNumRagdolls;
	const SimRagdollJoint *		mRagdollJoints;
	uint32						mNumRagdollJoints;
	const SimRagdollAdditionalConstraint *mRagdollAdditionalConstraints;
	uint32						mNumRagdollAdditionalConstraints;
	const SimRagdollPose *		mRagdollPoses;
	uint32						mNumRagdollPoses;
	const SimVehicle *			mVehicles;
	uint32						mNumVehicles;
	const SimVehicleWheel *		mVehicleWheels;
	uint32						mNumVehicleWheels;
	const SimVehicleDifferential *mVehicleDifferentials;
	uint32						mNumVehicleDifferentials;
	const SimVehicleAntiRollBar *mVehicleAntiRollBars;
	uint32						mNumVehicleAntiRollBars;
	const uint32 *				mUints;
	uint32						mNumUints;
	// Phase 9
	const SimSoftBodySettings *	mSoftBodySettings;
	uint32						mNumSoftBodySettings;
	const SimSoftBody *			mSoftBodies;
	uint32						mNumSoftBodies;
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
	static constexpr uint32 cMassOverride = 1 << 10;
}

// ContactFlags.massOverrideScale of SimScenes.zig (sGetInvMassScale of UnitTests/Physics/ContactListenerTests.cpp)
static float SimMassOverrideScale(const Body &inBody)
{
	uint64 ud = inBody.GetUserData();
	int index = ((ud & 1) != 0? (ud >> 1) : (ud >> 3)) & 0b11;
	float mass_overrides[] = { 1.0f, 0.0f, 0.5f, 2.0f };
	return mass_overrides[index];
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

// The state filter of the end of a scene (StateFilter in SimulationParity.zig): bodies whose index is not 1 modulo 3,
// constraints whose user data is even, contacts of body pairs whose index sum is even; restored are the contacts of body
// pairs whose index sum is not a multiple of 3
class SimStateFilter final : public StateRecorderFilter
{
public:
	virtual bool				ShouldSaveBody(const Body &inBody) const override
	{
		return inBody.GetID().GetIndex() % 3 != 1;
	}

	virtual bool				ShouldSaveConstraint(const Constraint &inConstraint) const override
	{
		return (inConstraint.GetUserData() & 1) == 0;
	}

	virtual bool				ShouldSaveContact(const BodyID &inBody1, const BodyID &inBody2) const override
	{
		return ((inBody1.GetIndex() + inBody2.GetIndex()) & 1) == 0;
	}

	virtual bool				ShouldRestoreContact(const BodyID &inBody1, const BodyID &inBody2) const override
	{
		return (inBody1.GetIndex() + inBody2.GetIndex()) % 3 != 0;
	}
};

// SimCollideMode.all of SimScenes.zig (sCollideBodyVsBodyAll of UnitTests/Physics/SensorTests.cpp)
static void SimCollideBodyVsBodyAll(const Body &inBody1, const Body &inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings &ioCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	// Override the back face mode so we get hits with all triangles
	ioCollideShapeSettings.mBackFaceMode = EBackFaceMode::CollideWithBackFaces;
	PhysicsSystem::sDefaultSimCollideBodyVsBody(inBody1, inBody2, inCenterOfMassTransform1, inCenterOfMassTransform2, ioCollideShapeSettings, ioCollector, inShapeFilter);
}

// SimCollideMode.per_body of SimScenes.zig (sCollideBodyVsBodyPerBody of UnitTests/Physics/SensorTests.cpp)
static void SimCollideBodyVsBodyPerBody(const Body &inBody1, const Body &inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings &ioCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	// Max 1 hit per body pair
	AnyHitCollisionCollector<CollideShapeCollector> collector;
	SubShapeIDCreator part1, part2;
	CollisionDispatch::sCollideShapeVsShape(inBody1.GetShape(), inBody2.GetShape(), Vec3::sOne(), Vec3::sOne(), inCenterOfMassTransform1, inCenterOfMassTransform2, part1, part2, ioCollideShapeSettings, collector);
	if (collector.HadHit())
		ioCollector.AddHit(collector.mHit);
}

// SimCollideMode.per_leaf of SimScenes.zig (sCollideBodyVsBodyPerLeaf of UnitTests/Physics/SensorTests.cpp)
static void SimCollideBodyVsBodyPerLeaf(const Body &inBody1, const Body &inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings &ioCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	// Max 1 hit per leaf shape pair
	SubShapeIDCreator part1, part2;
	CollideShapeVsShapePerLeaf<AnyHitCollisionCollector<CollideShapeCollector>>(inBody1.GetShape(), inBody2.GetShape(), Vec3::sOne(), Vec3::sOne(), inCenterOfMassTransform1, inCenterOfMassTransform2, part1, part2, ioCollideShapeSettings, ioCollector, inShapeFilter);
}

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
	// Phase 7: the CharacterContactListener (compared in call order)
	CharacterContactValidate = 7,
	CharacterContactAdded = 8,
	CharacterContactPersisted = 9,
	CharacterContactRemoved = 10,
	CharacterVsCharacterContactValidate = 11,
	CharacterVsCharacterContactAdded = 12,
	CharacterVsCharacterContactPersisted = 13,
	CharacterVsCharacterContactRemoved = 14,
	CharacterContactSolve = 15,
	CharacterVsCharacterContactSolve = 16,
	CharacterAdjustBodyVelocity = 17,
	// Phase 7: the step callbacks of the vehicles (compared sorted)
	VehiclePreStep = 18,
	VehiclePostCollide = 19,
	VehiclePostStep = 20,
	// Phase 9: the soft body contact listener (compared sorted)
	SoftBodyContactValidate = 70,
	SoftBodyContactAdded = 71,
	SoftBodyContactBody = 72,
	SoftBodySensorContact = 73,
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
	WereBodiesInContact = 51,
	SetCollideKinematicVsNonDynamic = 52,

	// Phase 9: the soft body operations (53..69, ApplySoftBodyOp)
	CreateSoftBody = 53,
	CreateAndAddSoftBody = 54,
	CreateSoftBodyNoAdd = 55,
	SoftBodySetVertexPosition = 56,
	SoftBodySetVertexVelocity = 57,
	SoftBodySetVertexInvMass = 58,
	SoftBodySetPressure = 59,
	SoftBodySetNumIterations = 60,
	SoftBodySetUpdatePosition = 61,
	SoftBodySetFacesDoubleSided = 62,
	SoftBodySetVertexRadius = 63,
	SoftBodySetEnableSkinConstraints = 64,
	SoftBodySetSkinnedMaxDistanceMultiplier = 65,
	SoftBodySkinVertices = 66,
	SoftBodyCustomUpdate = 67,
	SoftBodyCreationSettingsHash = 68,
	SetSoftBodyContactListener = 69,

	// Constraint operations (Phase 6), in the reserved ranges of ConstraintOpRange (SimScenes.zig)
	// ConstraintOpRange.generic
	SetConstraintEnabled = 200,
	SetConstraintPriority = 201,
	SetConstraintStepsOverride = 202,
	SetConstraintUserData = 203,
	NotifyConstraintShapeChanged = 204,
	ResetConstraintWarmStart = 205,
	RemoveConstraint = 206,
	AddConstraint = 207,
	RemoveConstraints = 208,
	AddConstraints = 209,
	ActivateConstraint = 210,
	CreateConstraintNoAdd = 211,
	ExpectConstraintState = 212,
	ConstraintSettingsHash = 213,
	// ConstraintOpRange.fixed_point
	PointSetPoint1 = 220,
	PointSetPoint2 = 221,
	// ConstraintOpRange.distance_pulley
	DistanceSetDistance = 240,
	DistanceSetLimitsSpringSettings = 241,
	PulleySetLength = 250,
	// ConstraintOpRange.hinge
	HingeSetMaxFrictionTorque = 260,
	HingeSetMotorState = 261,
	HingeSetTargetAngularVelocity = 262,
	HingeSetTargetAngle = 263,
	HingeSetTargetOrientationBS = 264,
	HingeSetLimits = 265,
	HingeSetLimitsSpringSettings = 266,
	HingeSetMotorSettings = 267,
	// ConstraintOpRange.slider
	SliderSetMaxFrictionForce = 280,
	SliderSetMotorState = 281,
	SliderSetTargetVelocity = 282,
	SliderSetTargetPosition = 283,
	SliderSetLimits = 284,
	SliderSetLimitsSpringSettings = 285,
	SliderSetMotorSettings = 286,
	// ConstraintOpRange.cone_swing_twist
	ConeSetHalfConeAngle = 300,
	SwingTwistSetNormalHalfConeAngle = 305,
	SwingTwistSetPlaneHalfConeAngle = 306,
	SwingTwistSetTwistMinAngle = 307,
	SwingTwistSetTwistMaxAngle = 308,
	SwingTwistSetMaxFrictionTorque = 309,
	SwingTwistSetSwingMotorState = 310,
	SwingTwistSetTwistMotorState = 311,
	SwingTwistSetTargetAngularVelocityCS = 312,
	SwingTwistSetTargetAngularVelocityBS = 313,
	SwingTwistSetTargetOrientationCS = 314,
	SwingTwistSetTargetOrientationBS = 315,
	SwingTwistSetSwingMotorSettings = 316,
	SwingTwistSetTwistMotorSettings = 317,
	// ConstraintOpRange.six_dof
	SixDOFSetTranslationLimits = 320,
	SixDOFSetRotationLimits = 321,
	SixDOFSetLimitsSpringSettings = 322,
	SixDOFSetMaxFriction = 323,
	SixDOFSetMotorState = 324,
	SixDOFSetTargetVelocityCS = 325,
	SixDOFSetTargetAngularVelocityCS = 326,
	SixDOFSetTargetPositionCS = 327,
	SixDOFSetTargetOrientationCS = 328,
	SixDOFSetTargetOrientationBS = 329,
	SixDOFSetMotorSettings = 330,
	// ConstraintOpRange.path
	PathSetPath = 340,
	PathSetMaxFrictionForce = 341,
	PathSetPositionMotorState = 342,
	PathSetTargetVelocity = 343,
	PathSetTargetPathFraction = 344,
	PathSetPositionMotorSettings = 345,
	// ConstraintOpRange.gear_rack_and_pinion
	GearSetConstraints = 360,
	RackAndPinionSetConstraints = 370,

	// Phase 7 operations, in the reserved ranges of Phase7OpRange (SimScenes.zig)
	// Phase7OpRange.character
	CreateCharacter = 400,
	CharacterAddToPhysicsSystem = 401,
	CharacterRemoveFromPhysicsSystem = 402,
	CharacterActivate = 403,
	CharacterPostSimulation = 404,
	CharacterSetLinearAndAngularVelocity = 405,
	CharacterSetLinearVelocity = 406,
	CharacterAddLinearVelocity = 407,
	CharacterAddImpulse = 408,
	CharacterSetPositionAndRotation = 409,
	CharacterSetPosition = 410,
	CharacterSetRotation = 411,
	CharacterSetLayer = 412,
	CharacterSetShape = 413,
	CharacterSetMaxSlopeAngle = 414,
	CharacterSetUp = 415,
	CharacterSetSupportingVolume = 416,
	CharacterDestroy = 417,
	CharacterSettingsHash = 418,
	CharacterCheckCollision = 419,
	CharacterExpectGroundState = 420,
	CharacterBodyHash = 421,
	CharacterExpectBodyState = 422,
	CharacterExpectValue = 423,
	// Phase7OpRange.character_virtual
	CreateCharacterVirtual = 430,
	CVDestroy = 431,
	CVSetLinearVelocity = 432,
	CVSetPosition = 433,
	CVSetRotation = 434,
	CVSetUp = 435,
	CVSetMaxSlopeAngle = 436,
	CVSetSupportingVolume = 437,
	CVSetMass = 438,
	CVSetMaxStrength = 439,
	CVSetPenetrationRecoverySpeed = 440,
	CVSetEnhancedInternalEdgeRemoval = 441,
	CVSetMaxNumHits = 442,
	CVSetHitReductionCosMaxAngle = 443,
	CVSetShapeOffset = 444,
	CVSetUserData = 445,
	CVSetListener = 446,
	CVSetCharacterVsCharacterCollision = 447,
	CVAddToCollisionGroup = 448,
	CVRemoveFromCollisionGroup = 449,
	CVUpdate = 450,
	CVExtendedUpdate = 451,
	CVStickToFloor = 452,
	CVWalkStairs = 453,
	CVCanWalkStairs = 454,
	CVRefreshContacts = 455,
	CVUpdateGroundVelocity = 456,
	CVSetShape = 457,
	CVSetInnerBodyShape = 458,
	CVCancelVelocityTowardsSteepSlopes = 459,
	CVStartTrackingContactChanges = 460,
	CVFinishTrackingContactChanges = 461,
	CVCheckCollision = 462,
	CVSettingsHash = 463,
	CVHasCollidedWithBody = 464,
	CVHasCollidedWithCharacter = 465,
	CVExpectGroundState = 466,
	CVSaveRestoreState = 467,
	CVExpectValue = 468,
	CVSetStepVelocity = 469,
	CVPerfUpdate = 470,
	// Phase7OpRange.ragdoll
	CreateRagdoll = 490,
	RagdollAddToPhysicsSystem = 491,
	RagdollRemoveFromPhysicsSystem = 492,
	RagdollActivate = 493,
	RagdollIsActive = 494,
	RagdollSetGroupID = 495,
	RagdollSetPose = 496,
	RagdollSetPoseMatrices = 497,
	RagdollResetWarmStart = 498,
	RagdollDriveToPoseUsingKinematics = 499,
	RagdollDriveToPoseUsingKinematicsMatrices = 500,
	RagdollDriveToPoseUsingMotors = 501,
	RagdollDriveToPoseUsingMotorsVelocity = 502,
	RagdollSetLinearAndAngularVelocity = 503,
	RagdollSetLinearVelocity = 504,
	RagdollAddLinearVelocity = 505,
	RagdollAddImpulse = 506,
	RagdollDestroy = 507,
	RagdollSettingsHash = 508,
	RagdollGetPoseHash = 509,
	RagdollExpectActive = 510,
	// Phase7OpRange.vehicle
	CreateVehicle = 520,
	VehicleRemove = 521,
	VehicleAdd = 522,
	VehicleSetMaxPitchRollAngle = 523,
	VehicleOverrideGravity = 524,
	VehicleResetGravityOverride = 525,
	VehicleSetNumStepsBetweenCollisionTestActive = 526,
	VehicleSetNumStepsBetweenCollisionTestInactive = 527,
	VehicleSetCollisionTester = 528,
	VehicleTesterSetObjectLayer = 529,
	VehicleTesterSetFilters = 530,
	VehicleSetCombineFriction = 531,
	VehicleSetCallbacks = 532,
	VehicleWheelSetAngularVelocity = 533,
	VehicleWheelSetRotationAngle = 534,
	VehicleWheelSetSteerAngle = 535,
	VehicleSetAntiRollBar = 536,
	VehicleSettingsHash = 537,
	VehicleControllerSettingsHash = 538,
	VehicleExpectWheelContacts = 539,
	VehicleEngineSetCurrentRPM = 540,
	VehicleEngineSetSettings = 541,
	VehicleEngineSetTorqueCurve = 542,
	VehicleTransmissionSet = 543,
	VehicleTransmissionSetSettings = 544,
	VehicleTransmissionSetGearRatios = 545,
	WheeledSetDriverInput = 546,
	WheeledSetForwardInput = 547,
	WheeledSetRightInput = 548,
	WheeledSetBrakeInput = 549,
	WheeledSetHandBrakeInput = 550,
	WheeledSetDifferential = 551,
	WheeledSetDifferentialLimitedSlipRatio = 552,
	WheeledSetTireMaxImpulseCallback = 553,
	VehicleSetEnabled = 554,
	VehicleResetWarmStart = 555,
	VehicleSetConstraintPriority = 556,
	VehicleSetStepsOverride = 557,
	VehicleSetUserData = 558,
	VehicleExpectConstraintState = 559,
	VehicleWheelWVApplyTorque = 560,
	VehicleEngineApplyTorque = 561,
	VehicleEngineApplyDamping = 562,
	WheeledSetDifferentials = 563,
	// Phase7OpRange.vehicle_tracked
	TrackedSetDriverInput = 570,
	TrackedSetForwardInput = 571,
	TrackedSetLeftRatio = 572,
	TrackedSetRightRatio = 573,
	TrackedSetBrakeInput = 574,
	TrackedSetTrack = 575,
	// Phase7OpRange.vehicle_motorcycle
	MotorcycleEnableLeanController = 580,
	MotorcycleEnableLeanSteeringLimit = 581,
	MotorcycleSetLeanSpringConstant = 582,
	MotorcycleSetLeanSpringDamping = 583,
	MotorcycleSetLeanSpringIntegrationCoefficient = 584,
	MotorcycleSetLeanSpringIntegrationCoefficientDecay = 585,
	MotorcycleSetLeanSmoothingFactor = 586,
};

// Phase7OpRange of SimScenes.zig: the Phase 7 operations are 400 .. 599 (400 character, 430 character virtual, 490
// ragdoll, 520 vehicle core, 570 tracked, 580 motorcycle, 590 spare)
static constexpr uint32 cFirstPhase7Op = 400;
static constexpr uint32 cFirstTrackedOp = 570;
static constexpr uint32 cFirstMotorcycleOp = 580;
static constexpr uint32 cFirstSparePhase7Op = 590;
static constexpr uint32 cEndPhase7Op = 600;

// The soft body operations of SimScenes.zig (isSoftBodyOp): 53 .. 69
static constexpr uint32 cFirstSoftBodyOp = 53;
static constexpr uint32 cEndSoftBodyOp = 70;

// Must match ObjectField in SimScenes.zig: the fields of the object state
enum class ESimObjectField : uint16
{
	CharacterSaveState = 1,
	CharacterProperties = 2,
	CharacterBase = 3,
	CharacterBody = 4,
	CharacterTransformedShape = 5,
	CVSaveState = 10,
	CVProperties = 11,
	CVBase = 12,
	CVTransforms = 13,
	CVTransformedShape = 14,
	CVNumActiveContacts = 15,
	CVActiveContact = 16,
	RagdollProperties = 20,
	RagdollBody = 21,
	RagdollRootTransform = 22,
	RagdollPoseRootOffset = 23,
	RagdollPoseJoint = 24,
	RagdollBounds = 25,
	RagdollConstraint = 26,
	VehicleProperties = 30,
	VehicleWheel = 31,
	VehicleWheelContact = 32,
	VehicleWheelTransform = 33,
	VehicleWheelWV = 34,
	VehicleWheelTV = 35,
	VehicleAntiRollBar = 36,
	VehicleEngine = 37,
	VehicleTransmission = 38,
	VehicleWheeled = 39,
	VehicleDifferential = 40,
	VehicleTracked = 41,
	VehicleTrack = 42,
	VehicleMotorcycle = 43,
};

// Must match the bits of CharacterFlags in SimScenes.zig (in the upper 32 bits of a user data value)
namespace SimCharacterFlags
{
	static constexpr uint32 cReject = 1 << 16;
	static constexpr uint32 cCannotPush = 1 << 17;
	static constexpr uint32 cNoImpulses = 1 << 18;
	static constexpr uint32 cConveyor = 1 << 19;
	static constexpr uint32 cSlowSolve = 1 << 20;
}

// Must match VehicleCallbacks of SimScenes.zig
namespace SimVehicleCallbacks
{
	static constexpr uint32 cPreStep = 1 << 0;
	static constexpr uint32 cPostCollide = 1 << 1;
	static constexpr uint32 cPostStep = 1 << 2;
	static constexpr uint32 cSteer = 1 << 3;
}

// ConstraintOpRange of SimScenes.zig: the constraint operations are 200 .. 399 (20 numbers per range)
static constexpr uint32 cFirstConstraintOp = 200;
static constexpr uint32 cEndConstraintOp = 400;

// ConstraintState of SimScenes.zig: the result bits of a constraint operation
static uint32 SimConstraintState(const Constraint &inConstraint)
{
	return (inConstraint.GetEnabled()? 1u : 0u) | (inConstraint.IsActive()? 2u : 0u);
}

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

// A spring of a constraint description: mode, frequency / stiffness, damping
static SpringSettings SimLoadSpring(uint32 inMode, const float *inF)
{
	SpringSettings s;
	s.mMode = ESpringMode(inMode);
	s.mFrequency = inF[0];
	s.mDamping = inF[1];
	return s;
}

// A motor of a constraint description: the spring (mode, frequency / stiffness, damping), min / max force limit, min /
// max torque limit
static MotorSettings SimLoadMotor(uint32 inMode, const float *inF)
{
	MotorSettings m;
	m.mSpringSettings = SimLoadSpring(inMode, inF);
	m.mMinForceLimit = inF[2];
	m.mMaxForceLimit = inF[3];
	m.mMinTorqueLimit = inF[4];
	m.mMaxTorqueLimit = inF[5];
	return m;
}

// The Hermite path of a path constraint description (ConstraintKind.path)
static Ref<PathConstraintPathHermite> SimMakePath(const SimConstraint &inC)
{
	const uint32 *i = inC.mI;
	const float *f = inC.mF;
	Ref<PathConstraintPathHermite> path = new PathConstraintPathHermite;
	path->SetIsLooping(i[1] != 0);
	for (uint32 p = 0; p < i[2]; ++p)
	{
		const float *pf = f + 16 + 9 * p;
		path->AddPoint(SimLoad3(pf), SimLoad3(pf + 3), SimLoad3(pf + 6));
	}
	return path;
}

// The settings of a constraint description (SimConstraint, ConstraintKind in SimScenes.zig): the layout of CSMakeSettings
// in ConstraintsReference.cpp plus the SetRatio helpers of Gear / RackAndPinion. The Zolt side is makeConstraintSettings
// in SimulationParity.zig (its slots), the test "SimScenes constraint descriptions" compares the binary state of both
// and of CSMakeSettings.
static Ref<TwoBodyConstraintSettings> SimMakeConstraintSettings(const SimConstraint &inC)
{
	const uint32 *i = inC.mI;
	const Real *r = inC.mR;
	const float *f = inC.mF;
	Ref<TwoBodyConstraintSettings> settings;
	switch (inC.mKind)
	{
	case 0: // fixed
		{
			FixedConstraintSettings *s = new FixedConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mAutoDetectPoint = i[1] != 0;
			s->mPoint1 = SimLoadR3(r);
			s->mAxisX1 = SimLoad3(f);
			s->mAxisY1 = SimLoad3(f + 3);
			s->mPoint2 = SimLoadR3(r + 3);
			s->mAxisX2 = SimLoad3(f + 6);
			s->mAxisY2 = SimLoad3(f + 9);
			settings = s;
			break;
		}

	case 1: // point
		{
			PointConstraintSettings *s = new PointConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = SimLoadR3(r);
			s->mPoint2 = SimLoadR3(r + 3);
			settings = s;
			break;
		}

	case 2: // distance
		{
			DistanceConstraintSettings *s = new DistanceConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = SimLoadR3(r);
			s->mPoint2 = SimLoadR3(r + 3);
			s->mMinDistance = f[0];
			s->mMaxDistance = f[1];
			s->mLimitsSpringSettings = SimLoadSpring(i[1], f + 2);
			settings = s;
			break;
		}

	case 3: // hinge
		{
			HingeConstraintSettings *s = new HingeConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = SimLoadR3(r);
			s->mHingeAxis1 = SimLoad3(f);
			s->mNormalAxis1 = SimLoad3(f + 3);
			s->mPoint2 = SimLoadR3(r + 3);
			s->mHingeAxis2 = SimLoad3(f + 6);
			s->mNormalAxis2 = SimLoad3(f + 9);
			s->mLimitsMin = f[12];
			s->mLimitsMax = f[13];
			s->mLimitsSpringSettings = SimLoadSpring(i[1], f + 14);
			s->mMaxFrictionTorque = f[16];
			s->mMotorSettings = SimLoadMotor(i[2], f + 17);
			settings = s;
			break;
		}

	case 4: // slider
		{
			SliderConstraintSettings *s = new SliderConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mAutoDetectPoint = i[1] != 0;
			s->mPoint1 = SimLoadR3(r);
			s->mPoint2 = SimLoadR3(r + 3);
			if (i[2] != 0)
				s->SetSliderAxis(SimLoad3(f));
			else
			{
				s->mSliderAxis1 = SimLoad3(f);
				s->mNormalAxis1 = SimLoad3(f + 3);
				s->mSliderAxis2 = SimLoad3(f + 6);
				s->mNormalAxis2 = SimLoad3(f + 9);
			}
			s->mLimitsMin = f[12];
			s->mLimitsMax = f[13];
			s->mLimitsSpringSettings = SimLoadSpring(i[3], f + 14);
			s->mMaxFrictionForce = f[16];
			s->mMotorSettings = SimLoadMotor(i[4], f + 17);
			settings = s;
			break;
		}

	case 5: // cone
		{
			ConeConstraintSettings *s = new ConeConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mPoint1 = SimLoadR3(r);
			s->mTwistAxis1 = SimLoad3(f);
			s->mPoint2 = SimLoadR3(r + 3);
			s->mTwistAxis2 = SimLoad3(f + 3);
			s->mHalfConeAngle = f[6];
			settings = s;
			break;
		}

	case 6: // swing_twist
		{
			SwingTwistConstraintSettings *s = new SwingTwistConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mSwingType = ESwingType(i[1]);
			s->mPosition1 = SimLoadR3(r);
			s->mTwistAxis1 = SimLoad3(f);
			s->mPlaneAxis1 = SimLoad3(f + 3);
			s->mPosition2 = SimLoadR3(r + 3);
			s->mTwistAxis2 = SimLoad3(f + 6);
			s->mPlaneAxis2 = SimLoad3(f + 9);
			s->mNormalHalfConeAngle = f[12];
			s->mPlaneHalfConeAngle = f[13];
			s->mTwistMinAngle = f[14];
			s->mTwistMaxAngle = f[15];
			s->mMaxFrictionTorque = f[16];
			s->mSwingMotorSettings = SimLoadMotor(i[2], f + 17);
			s->mTwistMotorSettings = SimLoadMotor(i[3], f + 23);
			settings = s;
			break;
		}

	case 7: // six_dof
		{
			SixDOFConstraintSettings *s = new SixDOFConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mSwingType = ESwingType(i[1]);
			s->mPosition1 = SimLoadR3(r);
			s->mAxisX1 = SimLoad3(f);
			s->mAxisY1 = SimLoad3(f + 3);
			s->mPosition2 = SimLoadR3(r + 3);
			s->mAxisX2 = SimLoad3(f + 6);
			s->mAxisY2 = SimLoad3(f + 9);
			for (int a = 0; a < SixDOFConstraintSettings::EAxis::Num; ++a)
			{
				s->mMaxFriction[a] = f[12 + a];
				s->mLimitMin[a] = f[18 + a];
				s->mLimitMax[a] = f[24 + a];
				s->mMotorSettings[a] = SimLoadMotor(i[5 + a], f + 36 + 6 * a);
			}
			for (int a = 0; a < SixDOFConstraintSettings::EAxis::NumTranslation; ++a)
				s->mLimitsSpringSettings[a] = SimLoadSpring(i[2 + a], f + 30 + 2 * a);
			settings = s;
			break;
		}

	case 8: // path
		{
			PathConstraintSettings *s = new PathConstraintSettings;
			s->mPath = SimMakePath(inC);
			s->mPathPosition = SimLoad3(f);
			s->mPathRotation = SimLoadQuat(f + 3);
			s->mPathFraction = f[7];
			s->mMaxFrictionForce = f[8];
			s->mPositionMotorSettings = SimLoadMotor(i[4], f + 9);
			s->mRotationConstraintType = EPathRotationConstraintType(i[3]);
			settings = s;
			break;
		}

	case 9: // gear
		{
			GearConstraintSettings *s = new GearConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mHingeAxis1 = SimLoad3(f);
			s->mHingeAxis2 = SimLoad3(f + 3);
			if (i[1] != 0)
				s->SetRatio(int(i[2]), int(i[3]));
			else
				s->mRatio = f[6];
			settings = s;
			break;
		}

	case 10: // rack_and_pinion
		{
			RackAndPinionConstraintSettings *s = new RackAndPinionConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mHingeAxis = SimLoad3(f);
			s->mSliderAxis = SimLoad3(f + 3);
			if (i[1] != 0)
				s->SetRatio(int(i[2]), f[7], int(i[3]));
			else
				s->mRatio = f[6];
			settings = s;
			break;
		}

	case 11: // pulley
		{
			PulleyConstraintSettings *s = new PulleyConstraintSettings;
			s->mSpace = EConstraintSpace(i[0]);
			s->mBodyPoint1 = SimLoadR3(r);
			s->mFixedPoint1 = SimLoadR3(r + 3);
			s->mBodyPoint2 = SimLoadR3(r + 6);
			s->mFixedPoint2 = SimLoadR3(r + 9);
			s->mRatio = f[0];
			s->mMinLength = f[1];
			s->mMaxLength = f[2];
			settings = s;
			break;
		}

	default:
		Trace("SimulationReference: unknown constraint kind %u", inC.mKind);
		abort();
	}

	// The ConstraintSettings members
	settings->mEnabled = inC.mEnabled != 0;
	settings->mConstraintPriority = inC.mConstraintPriority;
	settings->mNumVelocityStepsOverride = inC.mNumVelocityStepsOverride;
	settings->mNumPositionStepsOverride = inC.mNumPositionStepsOverride;
	settings->mDrawConstraintSize = inC.mDrawConstraintSize;
	settings->mUserData = inC.mUserData;
	return settings;
}

// The SaveBinaryState bytes of constraint settings
static std::string SimSaveSettings(const ConstraintSettings &inSettings)
{
	std::stringstream data;
	StreamOutWrapper stream(data);
	inSettings.SaveBinaryState(stream);
	return data.str();
}

// A constraint of the class T (a scene that applies a class operation to another class is an error of the scene)
template <class T>
static T &SimCast(TwoBodyConstraint &ioConstraint, EConstraintSubType inSubType, const SimOp &inOp)
{
	if (ioConstraint.GetSubType() != inSubType)
	{
		Trace("SimulationReference: operation %u on constraint handle %u of subtype %u", inOp.mKind, inOp.mIndex, uint32(ioConstraint.GetSubType()));
		abort();
	}
	return static_cast<T &>(ioConstraint);
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

// Records the callbacks of the listeners (callbacks come from the worker threads). The locks are std::lock_guard:
// Jolt's UniqueLock (PhysicsLock.h) takes a lock context in a build with JPH_ENABLE_ASSERTS.
class SimRecorder
{
public:
	void						Add(const SimRecord &inRecord)
	{
		std::lock_guard<Mutex> lock(mMutex);
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
		if (flags & SimContactFlags::cMassOverride)
		{
			float scale1 = SimMassOverrideScale(inBody1);
			ioSettings.mInvMassScale1 = scale1;
			ioSettings.mInvInertiaScale1 = scale1;
			float scale2 = SimMassOverrideScale(inBody2);
			ioSettings.mInvMassScale2 = scale2;
			ioSettings.mInvInertiaScale2 = scale2;
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

// -------------------------------------------------------------------------------------------------------------------
// Phase 7 helpers: characters, ragdolls and vehicles

// FNV-1a over values (hashBytes of SimScenes.zig, the hash results of the Phase 7 operations)
class SimHasher
{
public:
	void						Bytes(const void *inData, size_t inSize)	{ mHash = SimHashBytes(mHash, inData, inSize); }
	template <class T> void		Value(const T &inValue)						{ Bytes(&inValue, sizeof(T)); }
	void						U32(uint32 inValue)							{ Value(inValue); }
	void						F32(float inValue)							{ Value(inValue); }
	void						Bool(bool inValue)							{ U32(inValue? 1u : 0u); }
	void						Vec3(Vec3Arg inV)							{ float v[3] = { inV.GetX(), inV.GetY(), inV.GetZ() }; Value(v); }
	void						RVec3(RVec3Arg inV)							{ Real v[3] = { inV.GetX(), inV.GetY(), inV.GetZ() }; Value(v); }
	void						Quat(QuatArg inQ)							{ float v[4] = { inQ.GetX(), inQ.GetY(), inQ.GetZ(), inQ.GetW() }; Value(v); }
	void						Plane(const JPH::Plane &inPlane)			{ Vec3(inPlane.GetNormal()); F32(inPlane.GetConstant()); }
	void						Vec4(Vec4Arg inV)							{ float v[4] = { inV.GetX(), inV.GetY(), inV.GetZ(), inV.GetW() }; Value(v); }
	void						RMat44(RMat44Arg inM)						{ for (int c = 0; c < 3; ++c) Vec4(inM.GetColumn4(c)); RVec3(inM.GetTranslation()); }

	uint32						mHash = 2166136261u;
};

// The identity of a material in the records and the object state: the hash of its debug name (materialHash of SimScenes.zig)
static uint32 SimMaterialHash(const PhysicsMaterial *inMaterial)
{
	const char *name = inMaterial != nullptr? inMaterial->GetDebugName() : "null";
	return SimHashBytes(2166136261u, name, strlen(name));
}

// Writes the object state (ObjectStateWriter of SimScenes.zig): fields [field u16][handle u16][item u16][size u16][bytes]
class SimObjectStateWriter
{
public:
	void						Begin(ESimObjectField inField, uint32 inHandle, uint32 inItem)
	{
		mStart = mBytes.size();
		uint16 header[4] = { uint16(inField), uint16(inHandle), uint16(inItem), 0 };
		mBytes.append(reinterpret_cast<const char *>(header), sizeof(header));
	}

	void						End()
	{
		uint16 size = uint16(mBytes.size() - mStart - 4 * sizeof(uint16));
		memcpy(&mBytes[mStart + 3 * sizeof(uint16)], &size, sizeof(size));
	}

	void						Raw(const void *inData, size_t inSize)		{ mBytes.append(static_cast<const char *>(inData), inSize); }
	template <class T> void		Value(const T &inValue)						{ Raw(&inValue, sizeof(T)); }
	void						U32(uint32 inValue)							{ Value(inValue); }
	void						I32(int32 inValue)							{ Value(inValue); }
	void						U64(uint64 inValue)							{ Value(inValue); }
	void						F32(float inValue)							{ Value(inValue); }
	void						Bool(bool inValue)							{ U32(inValue? 1u : 0u); }
	void						Vec3(Vec3Arg inV)							{ float v[3] = { inV.GetX(), inV.GetY(), inV.GetZ() }; Value(v); }
	void						RVec3(RVec3Arg inV)							{ Real v[3] = { inV.GetX(), inV.GetY(), inV.GetZ() }; Value(v); }
	void						Quat(QuatArg inQ)							{ float v[4] = { inQ.GetX(), inQ.GetY(), inQ.GetZ(), inQ.GetW() }; Value(v); }
	void						Plane(const JPH::Plane &inPlane)			{ Vec3(inPlane.GetNormal()); F32(inPlane.GetConstant()); }
	void						Vec4(Vec4Arg inV)							{ float v[4] = { inV.GetX(), inV.GetY(), inV.GetZ(), inV.GetW() }; Value(v); }
	void						Mat44(Mat44Arg inM)							{ for (int c = 0; c < 4; ++c) Vec4(inM.GetColumn4(c)); }
	void						RMat44(RMat44Arg inM)						{ for (int c = 0; c < 3; ++c) Vec4(inM.GetColumn4(c)); RVec3(inM.GetTranslation()); }

	std::string					mBytes;

private:
	size_t						mStart = 0;
};

// hashCharacterContact of SimScenes.zig: the members of a CharacterContact that a contact record does not hold directly
static uint32 SimHashCharacterContact(const CharacterContact &inContact)
{
	SimHasher h;
	h.RVec3(inContact.mPosition);
	h.Vec3(inContact.mLinearVelocity);
	h.Vec3(inContact.mSurfaceNormal);
	h.Value(inContact.mUserData);
	h.U32(SimMaterialHash(inContact.mMaterial));
	return h.mHash;
}

// The flags of a contact record (RecordKind of SimScenes.zig)
static uint32 SimCharacterContactFlags(const CharacterContact &inContact)
{
	return uint32(inContact.mMotionTypeB)
		| ((inContact.mIsSensorB? 1u : 0u) << 8)
		| ((inContact.mHadCollision? 1u : 0u) << 9)
		| ((inContact.mWasDiscarded? 1u : 0u) << 10)
		| ((inContact.mCanPushCharacter? 1u : 0u) << 11)
		| ((inContact.mIsBackFacingContact? 1u : 0u) << 12)
		| ((inContact.mCharacterB != nullptr? 1u : 0u) << 13);
}

static uint32 SimCharacterContactSettingsBits(const CharacterContactSettings &inSettings)
{
	return (inSettings.mCanPushCharacter? 1u : 0u) | ((inSettings.mCanReceiveImpulses? 1u : 0u) << 1);
}

// The CharacterContactListener of the character virtuals (CharacterListenerMode record / modify of SimScenes.zig): records
// every callback in call order, in modify mode acts on the CharacterFlags
class SimCharacterContactListener final : public CharacterContactListener
{
public:
	virtual void				OnAdjustBodyVelocity(const CharacterVirtual *inCharacter, const Body &inBody2, Vec3 &ioLinearVelocity, Vec3 &ioAngularVelocity) override
	{
		SimRecord r = { };
		r.mKind = uint32(ESimRecordKind::CharacterAdjustBodyVelocity);
		r.mIDs[0] = inCharacter->GetID().GetValue();
		r.mIDs[1] = inBody2.GetID().GetIndexAndSequenceNumber();
		SetVec3(r.mData, ioLinearVelocity);
		SetVec3(r.mData + 3, ioAngularVelocity);
		if (mModify && (Flags(inBody2.GetUserData()) & SimCharacterFlags::cConveyor) != 0)
		{
			ioLinearVelocity += Vec3(1.5f, 0.0f, 0.0f);
			ioAngularVelocity += Vec3(0.0f, 0.5f, 0.0f);
		}
		SimHasher h;
		h.Vec3(ioLinearVelocity);
		h.Vec3(ioAngularVelocity);
		r.mData[6] = h.mHash;
		mRecorder->Add(r);
	}

	virtual bool				OnContactValidate(const CharacterVirtual *inCharacter, const CharacterContact &inContact) override
	{
		return Validate(ESimRecordKind::CharacterContactValidate, inCharacter, inContact);
	}

	virtual void				OnContactAdded(const CharacterVirtual *inCharacter, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Settings(ESimRecordKind::CharacterContactAdded, inCharacter, inContact, ioSettings);
	}

	virtual void				OnContactPersisted(const CharacterVirtual *inCharacter, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Settings(ESimRecordKind::CharacterContactPersisted, inCharacter, inContact, ioSettings);
	}

	virtual void				OnContactRemoved(const CharacterVirtual *inCharacter, const BodyID &inBodyID2, const SubShapeID &inSubShapeID2) override
	{
		Removed(ESimRecordKind::CharacterContactRemoved, inCharacter, inBodyID2, CharacterID(), inSubShapeID2);
	}

	virtual bool				OnCharacterContactValidate(const CharacterVirtual *inCharacter, const CharacterContact &inContact) override
	{
		return Validate(ESimRecordKind::CharacterVsCharacterContactValidate, inCharacter, inContact);
	}

	virtual void				OnCharacterContactAdded(const CharacterVirtual *inCharacter, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Settings(ESimRecordKind::CharacterVsCharacterContactAdded, inCharacter, inContact, ioSettings);
	}

	virtual void				OnCharacterContactPersisted(const CharacterVirtual *inCharacter, const CharacterContact &inContact, CharacterContactSettings &ioSettings) override
	{
		Settings(ESimRecordKind::CharacterVsCharacterContactPersisted, inCharacter, inContact, ioSettings);
	}

	virtual void				OnCharacterContactRemoved(const CharacterVirtual *inCharacter, const CharacterID &inOtherCharacterID, const SubShapeID &inSubShapeID2) override
	{
		Removed(ESimRecordKind::CharacterVsCharacterContactRemoved, inCharacter, BodyID(), inOtherCharacterID, inSubShapeID2);
	}

	virtual void				OnContactSolve(const CharacterVirtual *inCharacter, const BodyID &inBodyID2, const SubShapeID &inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, Vec3Arg inContactVelocity, const PhysicsMaterial *inContactMaterial, Vec3Arg inCharacterVelocity, Vec3 &ioNewCharacterVelocity) override
	{
		Solve(ESimRecordKind::CharacterContactSolve, inCharacter, inBodyID2, CharacterID(), inSubShapeID2, inContactPosition, inContactNormal, inContactVelocity, inContactMaterial, inCharacterVelocity, ioNewCharacterVelocity);
	}

	virtual void				OnCharacterContactSolve(const CharacterVirtual *inCharacter, const CharacterVirtual *inOtherCharacter, const SubShapeID &inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, Vec3Arg inContactVelocity, const PhysicsMaterial *inContactMaterial, Vec3Arg inCharacterVelocity, Vec3 &ioNewCharacterVelocity) override
	{
		Solve(ESimRecordKind::CharacterVsCharacterContactSolve, inCharacter, BodyID(), inOtherCharacter->GetID(), inSubShapeID2, inContactPosition, inContactNormal, inContactVelocity, inContactMaterial, inCharacterVelocity, ioNewCharacterVelocity);
	}

	SimRecorder *				mRecorder = nullptr;
	bool						mModify = false;

private:
	static uint32				Flags(uint64 inUserData)					{ return uint32(inUserData >> 32); }

	static void					SetVec3(uint32 *outData, Vec3Arg inV)
	{
		outData[0] = SimFloatBits(inV.GetX());
		outData[1] = SimFloatBits(inV.GetY());
		outData[2] = SimFloatBits(inV.GetZ());
	}

	// A contact record (RecordKind of SimScenes.zig)
	static SimRecord			ContactRecord(ESimRecordKind inKind, const CharacterVirtual *inCharacter, const CharacterContact &inContact, uint32 inSettingsIn, uint32 inResult)
	{
		SimRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inCharacter->GetID().GetValue();
		r.mIDs[1] = inContact.mBodyB.GetIndexAndSequenceNumber();
		r.mIDs[2] = inContact.mCharacterIDB.GetValue();
		r.mIDs[3] = inContact.mSubShapeIDB.GetValue();
		SetVec3(r.mData, inContact.mContactNormal);
		r.mData[3] = SimFloatBits(inContact.mDistance);
		r.mData[4] = SimFloatBits(inContact.mFraction);
		r.mData[5] = SimCharacterContactFlags(inContact) | (inSettingsIn << 16);
		r.mData[6] = SimHashCharacterContact(inContact);
		r.mData[7] = inResult;
		return r;
	}

	bool						Validate(ESimRecordKind inKind, const CharacterVirtual *inCharacter, const CharacterContact &inContact)
	{
		bool result = !(mModify && (Flags(inContact.mUserData) & SimCharacterFlags::cReject) != 0);
		mRecorder->Add(ContactRecord(inKind, inCharacter, inContact, 0, result? 1 : 0));
		return result;
	}

	void						Settings(ESimRecordKind inKind, const CharacterVirtual *inCharacter, const CharacterContact &inContact, CharacterContactSettings &ioSettings)
	{
		uint32 settings_in = SimCharacterContactSettingsBits(ioSettings);
		if (mModify)
		{
			uint32 flags = Flags(inContact.mUserData);
			if (flags & SimCharacterFlags::cCannotPush)
				ioSettings.mCanPushCharacter = false;
			if (flags & SimCharacterFlags::cNoImpulses)
				ioSettings.mCanReceiveImpulses = false;
		}
		mRecorder->Add(ContactRecord(inKind, inCharacter, inContact, settings_in, SimCharacterContactSettingsBits(ioSettings)));
	}

	void						Removed(ESimRecordKind inKind, const CharacterVirtual *inCharacter, const BodyID &inBodyID2, const CharacterID &inCharacterID2, const SubShapeID &inSubShapeID2)
	{
		SimRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inCharacter->GetID().GetValue();
		r.mIDs[1] = inBodyID2.GetIndexAndSequenceNumber();
		r.mIDs[2] = inCharacterID2.GetValue();
		r.mIDs[3] = inSubShapeID2.GetValue();
		mRecorder->Add(r);
	}

	void						Solve(ESimRecordKind inKind, const CharacterVirtual *inCharacter, const BodyID &inBodyID2, const CharacterID &inCharacterID2, const SubShapeID &inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, Vec3Arg inContactVelocity, const PhysicsMaterial *inContactMaterial, Vec3Arg inCharacterVelocity, Vec3 &ioNewCharacterVelocity)
	{
		SimRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inCharacter->GetID().GetValue();
		r.mIDs[1] = inBodyID2.GetIndexAndSequenceNumber();
		r.mIDs[2] = inCharacterID2.GetValue();
		r.mIDs[3] = inSubShapeID2.GetValue();
		SetVec3(r.mData, ioNewCharacterVelocity);
		SetVec3(r.mData + 3, inCharacterVelocity);
		SimHasher h;
		h.RVec3(inContactPosition);
		h.Vec3(inContactNormal);
		h.Vec3(inContactVelocity);
		h.U32(SimMaterialHash(inContactMaterial));
		r.mData[6] = h.mHash;
		if (mModify && (Flags(inCharacter->GetUserData()) & SimCharacterFlags::cSlowSolve) != 0)
			ioNewCharacterVelocity *= 0.5f;
		SimHasher h2;
		h2.Vec3(ioNewCharacterVelocity);
		r.mData[7] = h2.mHash;
		mRecorder->Add(r);
	}
};

// CharacterShapeFilter.no_spheres of SimScenes.zig: shape 2 must not be a sphere
class SimNoSpheresShapeFilter final : public ShapeFilter
{
public:
	virtual bool				ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		return inShape2->GetSubType() != EShapeSubType::Sphere;
	}

	virtual bool				ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		return inShape2->GetSubType() != EShapeSubType::Sphere;
	}
};

// The collector of the CheckCollision operations: the hash of the hits in their order, with the ID of the character that
// CharacterVsCharacterCollisionSimple passes through SetUserData
class SimCheckCollisionCollector final : public CollideShapeCollector
{
public:
	virtual void				SetUserData(uint64 inUserData) override		{ mOtherCharacter = reinterpret_cast<const CharacterVirtual *>(inUserData); }

	virtual void				AddHit(const CollideShapeResult &inResult) override
	{
		mHash.Vec3(inResult.mContactPointOn1);
		mHash.Vec3(inResult.mContactPointOn2);
		mHash.Vec3(inResult.mPenetrationAxis);
		mHash.F32(inResult.mPenetrationDepth);
		mHash.U32(inResult.mSubShapeID1.GetValue());
		mHash.U32(inResult.mSubShapeID2.GetValue());
		mHash.U32(inResult.mBodyID2.GetIndexAndSequenceNumber());
		mHash.U32(mOtherCharacter != nullptr? mOtherCharacter->GetID().GetValue() : CharacterID::cInvalidCharacterID);
		++mNumHits;
	}

	uint32						Result()
	{
		mHash.U32(mNumHits);
		return mHash.mHash;
	}

	SimHasher					mHash;
	uint32						mNumHits = 0;
	const CharacterVirtual *	mOtherCharacter = nullptr;
};

// The CharacterBaseSettings members of a description
static void SimFillCharacterBaseSettings(const SimCharacterBase &inBase, const Array<RefConst<Shape>> &inShapes, CharacterBaseSettings &outSettings)
{
	outSettings.mUp = SimLoad3(inBase.mUp);
	outSettings.mSupportingVolume = Plane(SimLoad3(inBase.mSupportingVolume), inBase.mSupportingVolume[3]);
	outSettings.mMaxSlopeAngle = inBase.mMaxSlopeAngle;
	outSettings.mEnhancedInternalEdgeRemoval = inBase.mEnhancedInternalEdgeRemoval != 0;
	outSettings.mShape = inBase.mShape != cInvalidHandle? inShapes[inBase.mShape] : nullptr;
}

// A LinearCurve from (x, y) points in the float pool (the curve is replaced)
static void SimLoadCurve(const SimSceneDesc &inDesc, uint32 inOffset, uint32 inNumPoints, LinearCurve &outCurve)
{
	outCurve.Clear();
	for (uint32 p = 0; p < inNumPoints; ++p)
		outCurve.AddPoint(inDesc.mFloats[inOffset + 2 * p], inDesc.mFloats[inOffset + 2 * p + 1]);
}

// An array of floats from the float pool
static Array<float> SimLoadFloats(const SimSceneDesc &inDesc, uint32 inOffset, uint32 inCount)
{
	return Array<float>(inDesc.mFloats + inOffset, inDesc.mFloats + inOffset + inCount);
}

// VehicleEngineSettings of a description (the curve is replaced when the description has one)
static void SimFillEngine(const SimSceneDesc &inDesc, const SimVehicleEngine &inEngine, VehicleEngineSettings &outEngine)
{
	outEngine.mMaxTorque = inEngine.mMaxTorque;
	outEngine.mMinRPM = inEngine.mMinRPM;
	outEngine.mMaxRPM = inEngine.mMaxRPM;
	if (inEngine.mTorquePoints != cInvalidHandle)
		SimLoadCurve(inDesc, inEngine.mTorquePoints, inEngine.mNumTorquePoints, outEngine.mNormalizedTorque);
	outEngine.mInertia = inEngine.mInertia;
	outEngine.mAngularDamping = inEngine.mAngularDamping;
}

// VehicleTransmissionSettings of a description (the ratios are replaced when the description has them)
static void SimFillTransmission(const SimSceneDesc &inDesc, const SimVehicleTransmission &inTransmission, VehicleTransmissionSettings &outTransmission)
{
	outTransmission.mMode = ETransmissionMode(inTransmission.mMode);
	if (inTransmission.mGearRatios != cInvalidHandle)
		outTransmission.mGearRatios = SimLoadFloats(inDesc, inTransmission.mGearRatios, inTransmission.mNumGearRatios);
	if (inTransmission.mReverseGearRatios != cInvalidHandle)
		outTransmission.mReverseGearRatios = SimLoadFloats(inDesc, inTransmission.mReverseGearRatios, inTransmission.mNumReverseGearRatios);
	outTransmission.mSwitchTime = inTransmission.mSwitchTime;
	outTransmission.mClutchReleaseTime = inTransmission.mClutchReleaseTime;
	outTransmission.mSwitchLatency = inTransmission.mSwitchLatency;
	outTransmission.mShiftUpRPM = inTransmission.mShiftUpRPM;
	outTransmission.mShiftDownRPM = inTransmission.mShiftDownRPM;
	outTransmission.mClutchStrength = inTransmission.mClutchStrength;
}

// VehicleTrackSettings of a track description
static void SimFillTrack(const SimSceneDesc &inDesc, const SimVehicleTrack &inTrack, VehicleTrackSettings &outTrack)
{
	outTrack.mDrivenWheel = inTrack.mDrivenWheel;
	outTrack.mWheels.assign(inDesc.mUints + inTrack.mWheels, inDesc.mUints + inTrack.mWheels + inTrack.mNumWheels);
	outTrack.mInertia = inTrack.mInertia;
	outTrack.mAngularDamping = inTrack.mAngularDamping;
	outTrack.mMaxBrakeTorque = inTrack.mMaxBrakeTorque;
	outTrack.mDifferentialRatio = inTrack.mDifferentialRatio;
}

// VehicleDifferentialSettings of a differential description
static void SimFillDifferential(const SimVehicleDifferential &inDifferential, VehicleDifferentialSettings &outDifferential)
{
	outDifferential.mLeftWheel = inDifferential.mLeftWheel;
	outDifferential.mRightWheel = inDifferential.mRightWheel;
	outDifferential.mDifferentialRatio = inDifferential.mDifferentialRatio;
	outDifferential.mLeftRightSplit = inDifferential.mLeftRightSplit;
	outDifferential.mLimitedSlipRatio = inDifferential.mLimitedSlipRatio;
	outDifferential.mEngineTorqueRatio = inDifferential.mEngineTorqueRatio;
}

// VehicleAntiRollBar of an anti-roll bar description
static void SimFillAntiRollBar(const SimVehicleAntiRollBar &inBar, VehicleAntiRollBar &outBar)
{
	outBar.mLeftWheel = inBar.mLeftWheel;
	outBar.mRightWheel = inBar.mRightWheel;
	outBar.mStiffness = inBar.mStiffness;
}

// The wheeled controller members of a description (WheeledVehicleControllerSettings, also the base of the motorcycle)
static void SimFillWheeled(const SimSceneDesc &inDesc, const SimVehicle &inVehicle, WheeledVehicleControllerSettings &outSettings)
{
	SimFillEngine(inDesc, inVehicle.mEngine, outSettings.mEngine);
	SimFillTransmission(inDesc, inVehicle.mTransmission, outSettings.mTransmission);
	for (uint32 d = 0; d < inVehicle.mNumDifferentials; ++d)
	{
		VehicleDifferentialSettings diff;
		SimFillDifferential(inDesc.mVehicleDifferentials[inVehicle.mFirstDifferential + d], diff);
		outSettings.mDifferentials.push_back(diff);
	}
	outSettings.mDifferentialLimitedSlipRatio = inVehicle.mDifferentialLimitedSlipRatio;
}

// The tracked controller settings of a vehicle description (the part of the vehicle variants group:
// TrackedVehicleSlot.makeControllerSettings of SimulationParity.zig)
static Ref<VehicleControllerSettings> SimMakeTrackedControllerSettings(const SimSceneDesc &inDesc, const SimVehicle &inVehicle)
{
	TrackedVehicleControllerSettings *s = new TrackedVehicleControllerSettings;
	SimFillEngine(inDesc, inVehicle.mEngine, s->mEngine);
	SimFillTransmission(inDesc, inVehicle.mTransmission, s->mTransmission);
	for (int t = 0; t < 2; ++t)
		SimFillTrack(inDesc, inVehicle.mTracks[t], s->mTracks[t]);
	return s;
}

// The motorcycle controller settings of a vehicle description (MotorcycleSlot.makeControllerSettings: the wheeled
// members with SimFillWheeled, VehicleSlot's, then the lean members)
static Ref<VehicleControllerSettings> SimMakeMotorcycleControllerSettings(const SimSceneDesc &inDesc, const SimVehicle &inVehicle)
{
	MotorcycleControllerSettings *s = new MotorcycleControllerSettings;
	SimFillWheeled(inDesc, inVehicle, *s);
	s->mMaxLeanAngle = inVehicle.mMaxLeanAngle;
	s->mLeanSpringConstant = inVehicle.mLeanSpringConstant;
	s->mLeanSpringDamping = inVehicle.mLeanSpringDamping;
	s->mLeanSpringIntegrationCoefficient = inVehicle.mLeanSpringIntegrationCoefficient;
	s->mLeanSpringIntegrationCoefficientDecay = inVehicle.mLeanSpringIntegrationCoefficientDecay;
	s->mLeanSmoothingFactor = inVehicle.mLeanSmoothingFactor;
	return s;
}

// The controller settings of a vehicle description (VehicleControllerKind of SimScenes.zig): the wheeled settings
// (VehicleSlot), the tracked and motorcycle ones through the functions of the vehicle variants
static Ref<VehicleControllerSettings> SimMakeControllerSettings(const SimSceneDesc &inDesc, const SimVehicle &inVehicle)
{
	switch (inVehicle.mController)
	{
	case 0: // wheeled
		{
			WheeledVehicleControllerSettings *s = new WheeledVehicleControllerSettings;
			SimFillWheeled(inDesc, inVehicle, *s);
			return s;
		}

	case 1:		return SimMakeTrackedControllerSettings(inDesc, inVehicle);
	case 2:		return SimMakeMotorcycleControllerSettings(inDesc, inVehicle);

	default:
		Trace("SimulationReference: unknown vehicle controller kind %u", inVehicle.mController);
		abort();
	}
}

// The WheelSettings members of a wheel description
static void SimFillWheelSettings(const SimVehicleWheel &inWheel, WheelSettings &outSettings)
{
	outSettings.mPosition = SimLoad3(inWheel.mPosition);
	outSettings.mSuspensionForcePoint = SimLoad3(inWheel.mSuspensionForcePoint);
	outSettings.mSuspensionDirection = SimLoad3(inWheel.mSuspensionDirection);
	outSettings.mSteeringAxis = SimLoad3(inWheel.mSteeringAxis);
	outSettings.mWheelUp = SimLoad3(inWheel.mWheelUp);
	outSettings.mWheelForward = SimLoad3(inWheel.mWheelForward);
	outSettings.mSuspensionMinLength = inWheel.mSuspensionMinLength;
	outSettings.mSuspensionMaxLength = inWheel.mSuspensionMaxLength;
	outSettings.mSuspensionPreloadLength = inWheel.mSuspensionPreloadLength;
	outSettings.mSuspensionSpring.mMode = ESpringMode(inWheel.mSuspensionSpringMode);
	outSettings.mSuspensionSpring.mFrequency = inWheel.mSuspensionFrequency;
	outSettings.mSuspensionSpring.mDamping = inWheel.mSuspensionDamping;
	outSettings.mRadius = inWheel.mRadius;
	outSettings.mWidth = inWheel.mWidth;
	outSettings.mEnableSuspensionForcePoint = inWheel.mEnableSuspensionForcePoint != 0;
}

// WheelSettingsTV of a wheel description (TrackedVehicleSlot.makeWheelSettings)
static Ref<WheelSettings> SimMakeWheelSettingsTV(const SimVehicleWheel &inWheel)
{
	WheelSettingsTV *tv = new WheelSettingsTV;
	SimFillWheelSettings(inWheel, *tv);
	tv->mLongitudinalFriction = inWheel.mLongitudinalFrictionTV;
	tv->mLateralFriction = inWheel.mLateralFrictionTV;
	return tv;
}

// The wheel settings of a description (VehicleWheelKind of SimScenes.zig): WheelSettingsWV (VehicleSlot), WheelSettingsTV
// through the function of the vehicle variants
static Ref<WheelSettings> SimMakeWheelSettings(const SimSceneDesc &inDesc, const SimVehicleWheel &inWheel)
{
	if (inWheel.mKind != 0)
		return SimMakeWheelSettingsTV(inWheel);
	WheelSettingsWV *wv = new WheelSettingsWV;
	SimFillWheelSettings(inWheel, *wv);
	wv->mInertia = inWheel.mInertia;
	wv->mAngularDamping = inWheel.mAngularDamping;
	wv->mMaxSteerAngle = inWheel.mMaxSteerAngle;
	if (inWheel.mLongitudinalFriction != cInvalidHandle)
		SimLoadCurve(inDesc, inWheel.mLongitudinalFriction, inWheel.mNumLongitudinalFriction, wv->mLongitudinalFriction);
	if (inWheel.mLateralFriction != cInvalidHandle)
		SimLoadCurve(inDesc, inWheel.mLateralFriction, inWheel.mNumLateralFriction, wv->mLateralFriction);
	wv->mMaxBrakeTorque = inWheel.mMaxBrakeTorque;
	wv->mMaxHandBrakeTorque = inWheel.mMaxHandBrakeTorque;
	return wv;
}

// The VehicleConstraintSettings of a vehicle description
static Ref<VehicleConstraintSettings> SimMakeVehicleSettings(const SimSceneDesc &inDesc, const SimVehicle &inVehicle)
{
	Ref<VehicleConstraintSettings> s = new VehicleConstraintSettings;
	s->mEnabled = inVehicle.mEnabled != 0;
	s->mConstraintPriority = inVehicle.mConstraintPriority;
	s->mNumVelocityStepsOverride = inVehicle.mNumVelocityStepsOverride;
	s->mNumPositionStepsOverride = inVehicle.mNumPositionStepsOverride;
	s->mDrawConstraintSize = inVehicle.mDrawConstraintSize;
	s->mUserData = inVehicle.mUserData;
	s->mUp = SimLoad3(inVehicle.mUp);
	s->mForward = SimLoad3(inVehicle.mForward);
	s->mMaxPitchRollAngle = inVehicle.mMaxPitchRollAngle;
	for (uint32 w = 0; w < inVehicle.mNumWheels; ++w)
		s->mWheels.push_back(SimMakeWheelSettings(inDesc, inDesc.mVehicleWheels[inVehicle.mFirstWheel + w]));
	for (uint32 a = 0; a < inVehicle.mNumAntiRollBars; ++a)
	{
		VehicleAntiRollBar bar;
		SimFillAntiRollBar(inDesc.mVehicleAntiRollBars[inVehicle.mFirstAntiRollBar + a], bar);
		s->mAntiRollBars.push_back(bar);
	}
	s->mController = SimMakeControllerSettings(inDesc, inVehicle);
	return s;
}

// A collision tester (VehicleTesterKind of SimScenes.zig)
static Ref<VehicleCollisionTester> SimMakeTester(uint32 inKind, uint32 inObjectLayer, Vec3Arg inUp, float inMaxSlopeAngle, float inRadius, float inConvexRadiusFraction)
{
	switch (inKind)
	{
	case 0:		return new VehicleCollisionTesterRay(ObjectLayer(inObjectLayer), inUp, inMaxSlopeAngle);
	case 1:		return new VehicleCollisionTesterCastSphere(ObjectLayer(inObjectLayer), inRadius, inUp, inMaxSlopeAngle);
	case 2:		return new VehicleCollisionTesterCastCylinder(ObjectLayer(inObjectLayer), inConvexRadiusFraction);
	default:
		Trace("SimulationReference: unknown vehicle collision tester kind %u", inKind);
		abort();
	}
}

// The ragdoll settings of a description (SimRagdoll of SimScenes.zig); outStabilize: the result of Stabilize (1 / 0,
// 2: not called)
static Ref<RagdollSettings> SimMakeRagdollSettings(const SimSceneDesc &inDesc, const SimRagdoll &inRagdoll, const Array<BodyCreationSettings> &inParts, uint32 *outStabilize = nullptr);

// The SkeletonPose of a pose description (the joint states, then CalculateJointMatrices)
static void SimMakePose(const SimSceneDesc &inDesc, const Skeleton *inSkeleton, uint32 inPose, SkeletonPose &outPose)
{
	const SimRagdollPose &p = inDesc.mRagdollPoses[inPose];
	outPose.SetSkeleton(inSkeleton);
	outPose.SetRootOffset(SimLoadR3(p.mRootOffset));
	for (uint32 j = 0; j < p.mCount && j < outPose.GetJointCount(); ++j)
	{
		const float *f = inDesc.mFloats + p.mData + 7 * j;
		SkeletonPose::JointState &state = outPose.GetJoint(int(j));
		state.mTranslation = SimLoad3(f);
		state.mRotation = SimLoadQuat(f + 3);
	}
	outPose.CalculateJointMatrices();
}

// The joint matrices of a pose description taken as model space transforms (the `_matrices` ragdoll operations)
static Array<Mat44> SimMakePoseMatrices(const SimSceneDesc &inDesc, uint32 inPose, uint32 inNumJoints)
{
	const SimRagdollPose &p = inDesc.mRagdollPoses[inPose];
	Array<Mat44> matrices(inNumJoints, Mat44::sIdentity());
	for (uint32 j = 0; j < p.mCount && j < inNumJoints; ++j)
	{
		const float *f = inDesc.mFloats + p.mData + 7 * j;
		matrices[j] = Mat44::sRotationTranslation(SimLoadQuat(f + 3), SimLoad3(f));
	}
	return matrices;
}

static Ref<RagdollSettings> SimMakeRagdollSettings(const SimSceneDesc &inDesc, const SimRagdoll &inRagdoll, const Array<BodyCreationSettings> &inParts, uint32 *outStabilize)
{
	Ref<Skeleton> skeleton = new Skeleton;
	for (uint32 j = 0; j < inRagdoll.mNumJoints; ++j)
	{
		std::string name = "Joint" + std::to_string(j);
		int parent = inDesc.mRagdollJoints[inRagdoll.mFirstJoint + j].mParent;
		if (parent < int(j))
			skeleton->AddJoint(name, parent);
		else
		{
			// A joint that is its own parent (degenerate input that CreateRagdoll accepts): AddJoint(name, parent)
			// would read the name of a joint that does not exist yet, the parent index is set directly
			skeleton->AddJoint(name);
			skeleton->GetJoints().back().mParentJointIndex = parent;
		}
	}

	Ref<RagdollSettings> settings = new RagdollSettings;
	settings->mSkeleton = skeleton;
	settings->mParts.resize(inRagdoll.mNumJoints);
	for (uint32 j = 0; j < inRagdoll.mNumJoints; ++j)
	{
		const SimRagdollJoint &joint = inDesc.mRagdollJoints[inRagdoll.mFirstJoint + j];
		RagdollSettings::Part &part = settings->mParts[j];
		static_cast<BodyCreationSettings &>(part) = inParts[j];
		if (joint.mToParent != cInvalidHandle)
			part.mToParent = SimMakeConstraintSettings(inDesc.mConstraints[joint.mToParent]);
	}
	for (uint32 a = 0; a < inRagdoll.mNumAdditional; ++a)
	{
		const SimRagdollAdditionalConstraint &c = inDesc.mRagdollAdditionalConstraints[inRagdoll.mFirstAdditional + a];
		settings->mAdditionalConstraints.push_back(RagdollSettings::AdditionalConstraint(c.mBody1, c.mBody2, SimMakeConstraintSettings(inDesc.mConstraints[c.mConstraint])));
	}

	// The preparation, in the order of the members of SimRagdoll
	uint32 stabilize = 2;
	if (inRagdoll.mStabilize != 0)
		stabilize = settings->Stabilize()? 1 : 0;
	if (outStabilize != nullptr)
		*outStabilize = stabilize;
	if (inRagdoll.mCalculateConstraintPriorities != 0)
		settings->CalculateConstraintPriorities(inRagdoll.mBasePriority);
	if (inRagdoll.mDisableParentChildCollisions == 1)
		settings->DisableParentChildCollisions();
	else if (inRagdoll.mDisableParentChildCollisions == 2)
	{
		SkeletonPose pose;
		SimMakePose(inDesc, skeleton, inRagdoll.mPose, pose);
		settings->DisableParentChildCollisions(pose.GetJointMatrices().data(), inRagdoll.mMinSeparationDistance);
	}
	settings->CalculateBodyIndexToConstraintIndex();
	settings->CalculateConstraintIndexToBodyIdxPair();
	return settings;
}

// The number of CharacterVsCharacterCollisionSimple objects of a world (num_character_collision_groups of SimScenes.zig)
static constexpr uint32 cNumCharacterCollisionGroups = 4;

// The filters of a CharacterVirtual query (CharacterQueryFilter of SimScenes.zig): op.i[0] the object layer of the
// default filters (cInvalidHandle: accept all), op.i[1] the CharacterBodyFilter, op.i[2] the CharacterShapeFilter, op.body
// the body of CharacterBodyFilter.ignore_body
class SimCharacterFilters
{
public:
								SimCharacterFilters(const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter &inObjectLayerPairFilter, const SimOp &inOp, const BodyID &inBody, const BodyID &inInnerBody) :
		mLayer(inOp.mI[0]),
		mBodyFilterKind(inOp.mI[1]),
		mShapeFilterKind(inOp.mI[2]),
		mDefaultBroadPhaseLayerFilter(inObjectVsBroadPhaseLayerFilter, ObjectLayer(inOp.mI[0] == cInvalidHandle? 0 : inOp.mI[0])),
		mDefaultObjectLayerFilter(inObjectLayerPairFilter, ObjectLayer(inOp.mI[0] == cInvalidHandle? 0 : inOp.mI[0])),
		mIgnoreBody(inBody),
		mIgnoreInnerBody(inInnerBody)
	{
	}

	const BroadPhaseLayerFilter &BroadPhase() const				{ return mLayer == cInvalidHandle? mAllBroadPhaseLayers : static_cast<const BroadPhaseLayerFilter &>(mDefaultBroadPhaseLayerFilter); }
	const ObjectLayerFilter &	ObjectLayers() const			{ return mLayer == cInvalidHandle? mAllObjectLayers : static_cast<const ObjectLayerFilter &>(mDefaultObjectLayerFilter); }
	const BodyFilter &			Bodies() const					{ return mBodyFilterKind == 1? static_cast<const BodyFilter &>(mIgnoreBody) : mBodyFilterKind == 2? static_cast<const BodyFilter &>(mIgnoreInnerBody) : mAllBodies; }
	const ShapeFilter &			Shapes() const					{ return mShapeFilterKind == 1? static_cast<const ShapeFilter &>(mNoSpheres) : mAllShapes; }

private:
	uint32						mLayer;
	uint32						mBodyFilterKind;
	uint32						mShapeFilterKind;
	BroadPhaseLayerFilter		mAllBroadPhaseLayers;
	ObjectLayerFilter			mAllObjectLayers;
	DefaultBroadPhaseLayerFilter mDefaultBroadPhaseLayerFilter;
	DefaultObjectLayerFilter	mDefaultObjectLayerFilter;
	BodyFilter					mAllBodies;
	IgnoreSingleBodyFilter		mIgnoreBody;
	IgnoreSingleBodyFilter		mIgnoreInnerBody;
	ShapeFilter					mAllShapes;
	SimNoSpheresShapeFilter		mNoSpheres;
};

// The CharacterVirtual::ExtendedUpdateSettings of f[4..16] of cv_extended_update
static CharacterVirtual::ExtendedUpdateSettings SimLoadExtendedUpdateSettings(const float *inF)
{
	CharacterVirtual::ExtendedUpdateSettings s;
	s.mStickToFloorStepDown = SimLoad3(inF);
	s.mWalkStairsStepUp = SimLoad3(inF + 3);
	s.mWalkStairsMinStepForward = inF[6];
	s.mWalkStairsStepForwardTest = inF[7];
	s.mWalkStairsCosAngleForwardContact = inF[8];
	s.mWalkStairsStepDownExtra = SimLoad3(inF + 9);
	return s;
}

// VehicleCombineFriction.custom of SimScenes.zig
static void SimVehicleCombineFriction(uint inWheelIndex, float &ioLongitudinalFriction, float &ioLateralFriction, const Body &inBody2, const SubShapeID &inSubShapeID2)
{
	float body_friction = inBody2.GetFriction();
	ioLongitudinalFriction = 0.5f * (ioLongitudinalFriction + body_friction);
	ioLateralFriction = min(ioLateralFriction, body_friction);
	if ((inWheelIndex & 1) != 0)
	{
		ioLongitudinalFriction *= 0.9f;
		ioLateralFriction *= 0.9f;
	}
}

// VehicleTireMaxImpulse.custom of SimScenes.zig
static void SimVehicleTireMaxImpulse(uint inWheelIndex, float &outLongitudinalImpulse, float &outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float inLongitudinalSlip, float inLateralSlip, float inDeltaTime)
{
	outLongitudinalImpulse = 0.9f * inLongitudinalFriction * inSuspensionImpulse;
	outLateralImpulse = inLateralFriction * inSuspensionImpulse;
	if (inWheelIndex == 0)
	{
		outLongitudinalImpulse *= 0.5f;
		outLateralImpulse *= 0.5f;
	}
}

// A vehicle record (RecordKind.vehicle_* of SimScenes.zig): the vehicle handle and body, the step, the wheels with a
// contact and the hash of the suspension length, angular velocity and steer angle of every wheel
static SimRecord SimVehicleRecord(ESimRecordKind inKind, uint32 inHandle, const VehicleConstraint &inVehicle, const PhysicsStepListenerContext &inContext)
{
	SimRecord r = { };
	r.mKind = uint32(inKind);
	r.mIDs[0] = inHandle;
	r.mIDs[1] = inVehicle.GetVehicleBody()->GetID().GetIndexAndSequenceNumber();
	r.mData[0] = SimFloatBits(inContext.mDeltaTime);
	r.mData[1] = inContext.mIsFirstStep? 1 : 0;
	r.mData[2] = inContext.mIsLastStep? 1 : 0;
	SimHasher h;
	for (uint w = 0; w < uint(inVehicle.GetWheels().size()); ++w)
	{
		const Wheel *wheel = inVehicle.GetWheel(w);
		if (wheel->HasContact())
			r.mData[3] |= 1u << w;
		h.F32(wheel->GetSuspensionLength());
		h.F32(wheel->GetAngularVelocity());
		h.F32(wheel->GetSteerAngle());
	}
	r.mData[4] = h.mHash;
	return r;
}

// A vehicle of the world: the constraint, its tester, the defaults of its callbacks and the filters of
// vehicle_tester_set_filters (the tester keeps pointers to them)
// -------------------------------------------------------------------------------------------------------------------
// Phase 9: soft bodies (makeSoftBodySettings, SoftBodyListenerImpl and hashSoftBody of SimulationParity.zig run the same
// code on the Zolt side)

// The vertices and faces of a cloth (SoftBodySettingsKind.cloth: SoftBodyCreator::CreateCloth without its functions)
static void SimAddCloth(SoftBodySharedSettings &ioSettings, const SimSoftBodySettings &inS)
{
	const float offset_x = -0.5f * inS.mSpacing * float(inS.mCountX - 1);
	const float offset_z = -0.5f * inS.mSpacing * float(inS.mCountZ - 1);
	for (uint32 z = 0; z < inS.mCountZ; ++z)
		for (uint32 x = 0; x < inS.mCountX; ++x)
		{
			SoftBodySharedSettings::Vertex v;
			v.mPosition = Float3(offset_x + float(x) * inS.mSpacing, 0.0f, offset_z + float(z) * inS.mSpacing);
			bool first_or_last_row = z == 0 || z == inS.mCountZ - 1;
			bool kinematic = inS.mKinematic == 1? first_or_last_row && (x == 0 || x == inS.mCountX - 1) : inS.mKinematic == 2 && first_or_last_row;
			v.mInvMass = kinematic? 0.0f : 1.0f;
			ioSettings.mVertices.push_back(v);
		}
	for (uint32 z = 0; z + 1 < inS.mCountZ; ++z)
		for (uint32 x = 0; x + 1 < inS.mCountX; ++x)
		{
			uint32 i = x + z * inS.mCountX;
			ioSettings.AddFace(SoftBodySharedSettings::Face(i, i + inS.mCountX, i + inS.mCountX + 1));
			ioSettings.AddFace(SoftBodySharedSettings::Face(i, i + inS.mCountX + 1, i + 1));
		}
}

// The joints, the inverse bind matrices and the skinned constraints of a skinned cloth (SoftBodySettingsKind.skinned_cloth,
// SoftBodySkinnedConstraintTest::Initialize)
static void SimAddSkin(SoftBodySharedSettings &ioSettings, const SimSoftBodySettings &inS)
{
	const uint32 num_joints = inS.mCountExtra;
	const float offset_z = -0.5f * inS.mSpacing * float(inS.mCountZ - 1);
	const float step = float(inS.mCountZ - 1) * inS.mSpacing / float(num_joints - 1);
	Array<float> joint_z;
	for (uint32 j = 0; j < num_joints; ++j)
	{
		float z = offset_z + float(j) * step;
		joint_z.push_back(z);
		ioSettings.mInvBindMatrices.push_back(SoftBodySharedSettings::InvBind(j, Mat44::sTranslation(Vec3(0, 0, z)).Inversed()));
	}
	for (uint32 v = 0; v < uint32(ioSettings.mVertices.size()); ++v)
	{
		const SoftBodySharedSettings::Vertex &vertex = ioSettings.mVertices[v];
		SoftBodySharedSettings::Skinned skinned(v, vertex.mInvMass > 0.0f? inS.mSkinMaxDistance : 0.0f, inS.mBackStopDistance, inS.mBackStopRadius);

		// Find the two closest joints
		uint32 closest = 0, prev_closest = 0;
		float closest_dist = FLT_MAX, prev_closest_dist = FLT_MAX;
		for (uint32 j = 0; j < num_joints; ++j)
		{
			float dist = std::abs(vertex.mPosition.z - joint_z[j]);
			if (dist < closest_dist)
			{
				prev_closest = closest;
				prev_closest_dist = closest_dist;
				closest = j;
				closest_dist = dist;
			}
			else if (dist < prev_closest_dist)
			{
				prev_closest = j;
				prev_closest_dist = dist;
			}
		}
		if (closest_dist == 0.0f)
		{
			// Hard skin to the closest joint
			skinned.mWeights[0] = SoftBodySharedSettings::SkinWeight(closest, 1.0f);
		}
		else
		{
			// Skin to the two closest joints
			skinned.mWeights[0] = SoftBodySharedSettings::SkinWeight(closest, 1.0f / closest_dist);
			skinned.mWeights[1] = SoftBodySharedSettings::SkinWeight(prev_closest, 1.0f / prev_closest_dist);
			skinned.NormalizeWeights();
		}
		ioSettings.mSkinnedConstraints.push_back(skinned);
	}
}

// The shared settings of a SimSoftBodySettings (SoftBodySettingsKind of SimScenes.zig; makeSoftBodySettings of
// SimulationParity.zig runs the same code)
static Ref<SoftBodySharedSettings> SimMakeSoftBodySettings(const SimSceneDesc &inDesc, const SimSoftBodySettings &inS)
{
	SoftBodySharedSettings::VertexAttributes attributes(inS.mCompliance, inS.mShearCompliance, inS.mBendCompliance, SoftBodySharedSettings::ELRAType(inS.mLRAType), inS.mLRAMaxDistanceMultiplier);
	SoftBodySharedSettings::EBendType bend_type = SoftBodySharedSettings::EBendType(inS.mBendType);
	Ref<SoftBodySharedSettings> settings = new SoftBodySharedSettings;
	switch (inS.mKind)
	{
	case 0: // Cube
		return SoftBodySharedSettings::sCreateCube(inS.mCountX, inS.mSpacing);

	case 1: // Cloth
	case 4: // Skinned cloth
		SimAddCloth(*settings, inS);
		settings->CreateConstraints(&attributes, 1, bend_type);
		if (inS.mOptimize != 0)
			settings->Optimize();
		if (inS.mKind == 4)
		{
			SimAddSkin(*settings, inS);
			settings->CalculateSkinnedConstraintNormals();
			if (inS.mOptimize != 0)
				settings->Optimize();
		}
		return settings;

	case 2: // Sphere (SoftBodyCreator::CreateSphere)
		{
			const float radius = inS.mSpacing;
			const uint32 num_theta = inS.mCountX, num_phi = inS.mCountZ;
			SoftBodySharedSettings::Vertex v;
			(radius * Vec3::sUnitSpherical(0, 0)).StoreFloat3(&v.mPosition);
			settings->mVertices.push_back(v);
			(radius * Vec3::sUnitSpherical(JPH_PI, 0)).StoreFloat3(&v.mPosition);
			settings->mVertices.push_back(v);
			for (uint32 theta = 1; theta < num_theta - 1; ++theta)
				for (uint32 phi = 0; phi < num_phi; ++phi)
				{
					(radius * Vec3::sUnitSpherical(JPH_PI * float(theta) / float(num_theta - 1), 2.0f * JPH_PI * float(phi) / float(num_phi))).StoreFloat3(&v.mPosition);
					settings->mVertices.push_back(v);
				}
			auto vertex_index = [num_theta, num_phi](uint32 inTheta, uint32 inPhi) -> uint32
			{
				if (inTheta == 0)
					return 0;
				else if (inTheta == num_theta - 1)
					return 1;
				else
					return 2 + (inTheta - 1) * num_phi + inPhi % num_phi;
			};
			SoftBodySharedSettings::Face f;
			for (uint32 phi = 0; phi < num_phi; ++phi)
			{
				for (uint32 theta = 0; theta < num_theta - 2; ++theta)
				{
					f.mVertex[0] = vertex_index(theta, phi);
					f.mVertex[1] = vertex_index(theta + 1, phi);
					f.mVertex[2] = vertex_index(theta + 1, phi + 1);
					settings->AddFace(f);
					if (theta > 0)
					{
						f.mVertex[1] = vertex_index(theta + 1, phi + 1);
						f.mVertex[2] = vertex_index(theta, phi + 1);
						settings->AddFace(f);
					}
				}
				f.mVertex[0] = vertex_index(num_theta - 2, phi + 1);
				f.mVertex[1] = vertex_index(num_theta - 2, phi);
				f.mVertex[2] = vertex_index(num_theta - 1, 0);
				settings->AddFace(f);
			}
			settings->CreateConstraints(&attributes, 1, bend_type);
			if (inS.mOptimize != 0)
				settings->Optimize();
			return settings;
		}

	case 3: // Rod chain (the helix of SoftBodyCosseratRodConstraintTest)
		for (uint32 i = 0; i < inS.mCountX; ++i)
		{
			float fraction = float(i) / float(inS.mCountX - 1);
			float alpha = 4.0f * JPH_PI * fraction;
			SoftBodySharedSettings::Vertex v;
			v.mPosition = Float3(inS.mSpacing * Sin(alpha), inS.mHeight * (0.5f - fraction), inS.mSpacing * Cos(alpha));
			v.mInvMass = i == 0 && inS.mKinematic != 0? 0.0f : 1.0f;
			settings->mVertices.push_back(v);
			if (i > 0)
				settings->mRodStretchShearConstraints.push_back(SoftBodySharedSettings::RodStretchShear(i - 1, i, inS.mCompliance));
			if (i > 1)
				settings->mRodBendTwistConstraints.push_back(SoftBodySharedSettings::RodBendTwist(i - 2, i - 1, inS.mShearCompliance));
		}
		settings->CalculateRodProperties();
		if (inS.mOptimize != 0)
			settings->Optimize();
		return settings;

	case 5: // Explicit
		{
			const float *f = inDesc.mFloats + inS.mFloats;
			for (uint32 i = 0; i < inS.mCountX; ++i, f += 7)
				settings->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(f[0], f[1], f[2]), Float3(f[3], f[4], f[5]), f[6]));
			const uint32 *u = inDesc.mUints + inS.mUints;
			for (uint32 i = 0; i < inS.mCountZ; ++i, u += 3)
				settings->AddFace(SoftBodySharedSettings::Face(u[0], u[1], u[2]));
			for (uint32 i = 0; i < inS.mCountExtra; ++i, u += 4)
				settings->mVolumeConstraints.push_back(SoftBodySharedSettings::Volume(u[0], u[1], u[2], u[3], inS.mCompliance));
			if (!settings->mFaces.empty())
				settings->CreateConstraints(&attributes, 1, bend_type);
			if (!settings->mVolumeConstraints.empty())
				settings->CalculateVolumeConstraintVolumes();
			if (inS.mOptimize != 0)
				settings->Optimize();
			return settings;
		}

	default:
		Trace("SimulationReference: unknown soft body settings kind %u", inS.mKind);
		abort();
	}
}

// The SaveBinaryState bytes of shared settings
static std::string SimSaveSoftBodySettings(const SoftBodySharedSettings &inSettings)
{
	std::stringstream data;
	StreamOutWrapper stream(data);
	inSettings.SaveBinaryState(stream);
	return data.str();
}

// The result of the soft body operations 56..67 (OpKind of SimScenes.zig, hashSoftBody of SimulationParity.zig): the
// hash of Body::SaveState, then of the values that the state does not hold
static uint32 SimHashSoftBody(const Body &inBody)
{
	StateRecorderImpl recorder;
	inBody.SaveState(recorder);
	std::string data = recorder.GetData();
	SimHasher h;
	h.Bytes(data.data(), data.size());
	const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(inBody.GetMotionProperties());
	h.U32(mp->GetNumIterations());
	h.F32(mp->GetPressure());
	h.Bool(mp->GetUpdatePosition());
	h.Bool(mp->GetFacesDoubleSided());
	h.Bool(mp->GetEnableSkinConstraints());
	h.F32(mp->GetSkinnedMaxDistanceMultiplier());
	h.F32(mp->GetVertexRadius());
	for (const SoftBodyVertex &v : mp->GetVertices())
		h.F32(v.mInvMass);
	return h.mHash;
}

// The soft body contact listener (SoftBodyListenerMode of SimScenes.zig, SoftBodyListenerImpl of SimulationParity.zig):
// records the callbacks (RecordKind 70..73, called from the soft body jobs), then OnSoftBodyContactValidate acts
class SimSoftBodyContactListener final : public SoftBodyContactListener
{
public:
	virtual SoftBodyValidateResult	OnSoftBodyContactValidate(const Body &inSoftBody, const Body &inOtherBody, SoftBodyContactSettings &ioSettings) override
	{
		SimRecord r = { };
		r.mKind = uint32(ESimRecordKind::SoftBodyContactValidate);
		r.mIDs[0] = inSoftBody.GetID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inOtherBody.GetID().GetIndexAndSequenceNumber();
		SettingsBits(ioSettings, r.mData);
		SoftBodyValidateResult result = SoftBodyValidateResult::AcceptContact;
		switch (mMode)
		{
		case 2:		result = SoftBodyValidateResult::RejectContact; break;
		case 3:		ioSettings.mIsSensor = true; break;
		case 4:		ioSettings.mInvMassScale1 = 0.5f; ioSettings.mInvMassScale2 = 2.0f; ioSettings.mInvInertiaScale2 = 1.5f; break;
		default:	break;
		}
		r.mIDs[2] = result == SoftBodyValidateResult::AcceptContact? 0 : 1;
		SettingsBits(ioSettings, r.mData + 4);
		mRecorder->Add(r);
		return result;
	}

	virtual void					OnSoftBodyContactAdded(const Body &inSoftBody, const SoftBodyManifold &inManifold) override
	{
		const uint32 soft_body = inSoftBody.GetID().GetIndexAndSequenceNumber();
		const Array<SoftBodyVertex> &vertices = inManifold.GetVertices();

		// The call
		SimRecord r = { };
		r.mKind = uint32(ESimRecordKind::SoftBodyContactAdded);
		r.mIDs[0] = soft_body;
		r.mIDs[1] = uint32(vertices.size());
		uint32 hash = 2166136261u, num_contacts = 0;
		for (const SoftBodyVertex &v : vertices)
		{
			uint32 has_contact = inManifold.HasContact(v)? 1 : 0;
			hash = SimHashBytes(hash, &has_contact, sizeof(has_contact));
			num_contacts += has_contact;
		}
		r.mIDs[2] = num_contacts;
		r.mIDs[3] = inManifold.GetNumSensorContacts();
		r.mData[0] = hash;
		mRecorder->Add(r);

		// One record per body that vertices have a contact with
		Array<BodyID> bodies;
		for (const SoftBodyVertex &v : vertices)
			if (inManifold.HasContact(v))
			{
				BodyID id = inManifold.GetContactBodyID(v);
				if (std::find(bodies.begin(), bodies.end(), id) == bodies.end())
					bodies.push_back(id);
			}
		for (const BodyID &id : bodies)
		{
			SimRecord b = { };
			b.mKind = uint32(ESimRecordKind::SoftBodyContactBody);
			b.mIDs[0] = soft_body;
			b.mIDs[1] = id.GetIndexAndSequenceNumber();
			uint32 body_hash = 2166136261u, count = 0;
			for (const SoftBodyVertex &v : vertices)
				if (inManifold.HasContact(v) && inManifold.GetContactBodyID(v) == id)
				{
					Vec3 p = inManifold.GetLocalContactPoint(v);
					Vec3 n = inManifold.GetContactNormal(v);
					float values[6] = { p.GetX(), p.GetY(), p.GetZ(), n.GetX(), n.GetY(), n.GetZ() };
					body_hash = SimHashBytes(body_hash, values, sizeof(values));
					++count;
				}
			b.mIDs[2] = count;
			b.mData[0] = body_hash;
			mRecorder->Add(b);
		}

		// The sensor contacts (the order of the sensors depends on the broad phase: not recorded)
		for (uint i = 0; i < inManifold.GetNumSensorContacts(); ++i)
		{
			SimRecord s = { };
			s.mKind = uint32(ESimRecordKind::SoftBodySensorContact);
			s.mIDs[0] = soft_body;
			s.mIDs[1] = inManifold.GetSensorContactBodyID(i).GetIndexAndSequenceNumber();
			mRecorder->Add(s);
		}
	}

	SimRecorder *					mRecorder = nullptr;
	uint32							mMode = 0;								// SoftBodyListenerMode

private:
	static void						SettingsBits(const SoftBodyContactSettings &inSettings, uint32 *outData)
	{
		outData[0] = SimFloatBits(inSettings.mInvMassScale1);
		outData[1] = SimFloatBits(inSettings.mInvMassScale2);
		outData[2] = SimFloatBits(inSettings.mInvInertiaScale2);
		outData[3] = inSettings.mIsSensor? 1 : 0;
	}
};

struct SimVehicleData
{
	Ref<VehicleConstraint>		mConstraint;
	Ref<VehicleCollisionTester>	mTester;
	uint32						mHandle = 0;
	uint32						mController = 0;						// VehicleControllerKind of SimScenes.zig
	bool						mAdded = false;
	VehicleConstraint::CombineFunction mDefaultCombineFriction;
	WheeledVehicleController::TireMaxImpulseCallback mDefaultTireMaxImpulse;
	std::unique_ptr<DefaultBroadPhaseLayerFilter> mBroadPhaseLayerFilter;
	std::unique_ptr<DefaultObjectLayerFilter> mObjectLayerFilter;
	std::unique_ptr<IgnoreMultipleBodiesFilter> mBodyFilter;
};

class SimWorld
{
public:
								SimWorld(const SimSceneDesc &inDesc, uint32 inNumThreads) :
		mDesc(inDesc),
		mTempAllocator(inDesc.mTempAllocatorSize),
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
		switch (inDesc.mSimCollideMode)
		{
		case 1:		mSystem.SetSimCollideBodyVsBody(SimCollideBodyVsBodyAll); break;
		case 2:		mSystem.SetSimCollideBodyVsBody(SimCollideBodyVsBodyPerLeaf); break;
		case 3:		mSystem.SetSimCollideBodyVsBody(SimCollideBodyVsBodyPerBody); break;
		default:	break;
		}

		for (uint32 i = 0; i < inDesc.mNumShapes; ++i)
			mShapes.push_back(MakeShape(inDesc.mShapes[i]));

		for (uint32 i = 0; i < inDesc.mNumStepListeners; ++i)
		{
			SimStepListenerImpl *listener = new SimStepListenerImpl;
			listener->mWorld = this;
			listener->mIndex = i;
			mStepListeners.push_back(listener);
		}

		// Phase 7: the character listeners record into their own recorder (compared in call order)
		mCharacterRecordListener.mRecorder = &mCharacterRecorder;
		mCharacterModifyListener.mRecorder = &mCharacterRecorder;
		mCharacterModifyListener.mModify = true;

		// Phase 9: the shared settings of every SimSoftBodySettings (the soft bodies of a description share them), the
		// soft body contact listener records with the other listeners (installed by set_soft_body_contact_listener)
		for (uint32 i = 0; i < inDesc.mNumSoftBodySettings; ++i)
			mSoftBodySettings.push_back(SimMakeSoftBodySettings(inDesc, inDesc.mSoftBodySettings[i]));
		mSoftBodyListener.mRecorder = &mRecorder;
	}

								~SimWorld()
	{
		// Phase 7 objects: remove what is in the system (Jolt asserts that a destroyed body is not in the broad phase),
		// then release them while the system exists (their destructors destroy their bodies)
		for (SimVehicleData *v : mVehicles)
		{
			if (v->mAdded)
			{
				mSystem.RemoveStepListener(v->mConstraint);
				mSystem.RemoveConstraint(v->mConstraint);
			}
			delete v;
		}
		mVehicles.clear();
		BodyInterface &bi = mSystem.GetBodyInterface();
		for (Ref<Character> &c : mCharacters)
			if (c != nullptr && !c->GetBodyID().IsInvalid() && bi.IsAdded(c->GetBodyID()))
				c->RemoveFromPhysicsSystem();
		mCharacters.clear();
		for (CharacterVsCharacterCollisionSimple &g : mCharacterGroups)
			g.mCharacters.clear();
		mCharacterVirtuals.clear();
		for (Ref<Ragdoll> &r : mRagdolls)
			if (r != nullptr && bi.IsAdded(r->GetBodyID(0)))
				r->RemoveFromPhysicsSystem();
		mRagdolls.clear();
		mRagdollSettings.clear();

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
		case 17:
			{
				VertexList vertices;
				for (uint32 i = 0; i < inShape.mCount; ++i)
				{
					const float *v = mDesc.mFloats + inShape.mData + 3 * i;
					vertices.push_back(Float3(v[0], v[1], v[2]));
				}
				IndexedTriangleList triangles;
				const float *indices = mDesc.mFloats + inShape.mData + 3 * inShape.mCount;
				for (uint32 i = 0; i < uint32(p[0]); ++i)
					triangles.push_back(IndexedTriangle(uint32(indices[3 * i]), uint32(indices[3 * i + 1]), uint32(indices[3 * i + 2])));
				MeshShapeSettings *mesh = new MeshShapeSettings(vertices, triangles);
				mesh->mMaxTrianglesPerLeaf = uint(p[1]);
				settings = mesh;
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

	// The constraint of a constraint handle (nullptr: not created)
	TwoBodyConstraint *			GetConstraint(uint32 inHandle) const
	{
		return inHandle < mConstraintHandles.size()? mConstraintHandles[inHandle].GetPtr() : nullptr;
	}

	// CreateConstraint of description inIndex (+ SetConstraints of a Gear / RackAndPinion), the next constraint handle
	TwoBodyConstraint *			CreateConstraint(uint32 inIndex)
	{
		const SimConstraint &c = mDesc.mConstraints[inIndex];
		Ref<TwoBodyConstraintSettings> settings = SimMakeConstraintSettings(c);
		TwoBodyConstraint *constraint = mSystem.GetBodyInterface().CreateConstraint(settings, GetHandle(c.mBody1), GetHandle(c.mBody2));
		if (c.mConstraint1 != cInvalidHandle || c.mConstraint2 != cInvalidHandle)
			SetConstraints(*constraint, c.mConstraint1, c.mConstraint2);
		mConstraintHandles.push_back(constraint);
		return constraint;
	}

	// SetConstraints of a Gear / RackAndPinion with two constraint handles (cInvalidHandle: nullptr)
	void						SetConstraints(TwoBodyConstraint &ioConstraint, uint32 inHandle1, uint32 inHandle2) const
	{
		const Constraint *c1 = GetConstraint(inHandle1);
		const Constraint *c2 = GetConstraint(inHandle2);
		switch (ioConstraint.GetSubType())
		{
		case EConstraintSubType::Gear:			static_cast<GearConstraint &>(ioConstraint).SetConstraints(c1, c2); break;
		case EConstraintSubType::RackAndPinion:	static_cast<RackAndPinionConstraint &>(ioConstraint).SetConstraints(c1, c2); break;
		default:
			Trace("SimulationReference: SetConstraints on a constraint of subtype %u", uint32(ioConstraint.GetSubType()));
			abort();
		}
	}

	// The number of constraints in the system (the result of the operations that add or remove constraints)
	uint32						GetNumConstraints() const
	{
		return uint32(mSystem.GetConstraints().size());
	}

	// The constraints of the handles [inOp.mIndex, inOp.mIndex + inOp.mI[0])
	Array<Constraint *>			GetConstraintHandles(const SimOp &inOp) const
	{
		Array<Constraint *> constraints;
		for (uint32 i = 0; i < inOp.mI[0]; ++i)
			constraints.push_back(GetConstraint(inOp.mIndex + i));
		return constraints;
	}

	// A constraint operation (OpKind 200..399 in SimScenes.zig), see the ranges of ConstraintOpRange
	uint32						ApplyConstraintOp(const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;

		// The operations that create constraints or change the constraints of the system
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::CreateConstraintNoAdd:		return SimConstraintState(*CreateConstraint(inOp.mIndex));
		case ESimOp::RemoveConstraint:			mSystem.RemoveConstraint(GetConstraint(inOp.mIndex)); return GetNumConstraints();
		case ESimOp::AddConstraint:				mSystem.AddConstraint(GetConstraint(inOp.mIndex)); return GetNumConstraints();

		case ESimOp::RemoveConstraints:
		case ESimOp::AddConstraints:
			{
				Array<Constraint *> constraints = GetConstraintHandles(inOp);
				if (ESimOp(inOp.mKind) == ESimOp::RemoveConstraints)
					mSystem.RemoveConstraints(constraints.data(), int(constraints.size()));
				else
					mSystem.AddConstraints(constraints.data(), int(constraints.size()));
				return GetNumConstraints();
			}

		default:
			break;
		}

		TwoBodyConstraint *c = GetConstraint(inOp.mIndex);
		if (c == nullptr)
		{
			Trace("SimulationReference: operation %u on constraint handle %u that does not exist", inOp.mKind, inOp.mIndex);
			abort();
		}

		switch (ESimOp(inOp.mKind))
		{
		// ConstraintOpRange.generic
		case ESimOp::SetConstraintEnabled:			c->SetEnabled(i[0] != 0); break;
		case ESimOp::SetConstraintPriority:			c->SetConstraintPriority(i[0]); break;
		case ESimOp::SetConstraintStepsOverride:	c->SetNumVelocityStepsOverride(i[0]); c->SetNumPositionStepsOverride(i[1]); break;
		case ESimOp::SetConstraintUserData:
			{
				// The whole value read back: the ConstraintState bits and the state filter (bit 0) do not show it
				c->SetUserData(uint64(i[0]) | (uint64(i[1]) << 32));
				uint64 user_data = c->GetUserData();
				return SimHashBytes(2166136261u, &user_data, sizeof(user_data));
			}
		case ESimOp::NotifyConstraintShapeChanged:	c->NotifyShapeChanged(GetHandle(inOp.mBody), SimLoad3(f)); break;
		case ESimOp::ResetConstraintWarmStart:		c->ResetWarmStart(); break;
		case ESimOp::ActivateConstraint:			mSystem.GetBodyInterface().ActivateConstraint(c); break;
		case ESimOp::ExpectConstraintState:			break; // Changes nothing, the driver checks the result

		case ESimOp::ConstraintSettingsHash:
			{
				std::string data = SimSaveSettings(*c->GetConstraintSettings());
				return SimHashBytes(2166136261u, data.data(), data.size());
			}

		// ConstraintOpRange.fixed_point
		case ESimOp::PointSetPoint1:	SimCast<PointConstraint>(*c, EConstraintSubType::Point, inOp).SetPoint1(EConstraintSpace(i[0]), SimLoadR3(inOp.mR)); break;
		case ESimOp::PointSetPoint2:	SimCast<PointConstraint>(*c, EConstraintSubType::Point, inOp).SetPoint2(EConstraintSpace(i[0]), SimLoadR3(inOp.mR)); break;

		// ConstraintOpRange.distance_pulley
		case ESimOp::DistanceSetDistance:				SimCast<DistanceConstraint>(*c, EConstraintSubType::Distance, inOp).SetDistance(f[0], f[1]); break;
		case ESimOp::DistanceSetLimitsSpringSettings:	SimCast<DistanceConstraint>(*c, EConstraintSubType::Distance, inOp).SetLimitsSpringSettings(SimLoadSpring(i[0], f)); break;
		case ESimOp::PulleySetLength:					SimCast<PulleyConstraint>(*c, EConstraintSubType::Pulley, inOp).SetLength(f[0], f[1]); break;

		// ConstraintOpRange.hinge
		case ESimOp::HingeSetMaxFrictionTorque:			SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetMaxFrictionTorque(f[0]); break;
		case ESimOp::HingeSetMotorState:				SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetMotorState(EMotorState(i[0])); break;
		case ESimOp::HingeSetTargetAngularVelocity:		SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetTargetAngularVelocity(f[0]); break;
		case ESimOp::HingeSetTargetAngle:				SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetTargetAngle(f[0]); break;
		case ESimOp::HingeSetTargetOrientationBS:		SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetTargetOrientationBS(SimLoadQuat(f)); break;
		case ESimOp::HingeSetLimits:					SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetLimits(f[0], f[1]); break;
		case ESimOp::HingeSetLimitsSpringSettings:		SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).SetLimitsSpringSettings(SimLoadSpring(i[0], f)); break;
		case ESimOp::HingeSetMotorSettings:				SimCast<HingeConstraint>(*c, EConstraintSubType::Hinge, inOp).GetMotorSettings() = SimLoadMotor(i[0], f); break;

		// ConstraintOpRange.slider
		case ESimOp::SliderSetMaxFrictionForce:			SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).SetMaxFrictionForce(f[0]); break;
		case ESimOp::SliderSetMotorState:				SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).SetMotorState(EMotorState(i[0])); break;
		case ESimOp::SliderSetTargetVelocity:			SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).SetTargetVelocity(f[0]); break;
		case ESimOp::SliderSetTargetPosition:			SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).SetTargetPosition(f[0]); break;
		case ESimOp::SliderSetLimits:					SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).SetLimits(f[0], f[1]); break;
		case ESimOp::SliderSetLimitsSpringSettings:		SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).SetLimitsSpringSettings(SimLoadSpring(i[0], f)); break;
		case ESimOp::SliderSetMotorSettings:			SimCast<SliderConstraint>(*c, EConstraintSubType::Slider, inOp).GetMotorSettings() = SimLoadMotor(i[0], f); break;

		// ConstraintOpRange.cone_swing_twist
		case ESimOp::ConeSetHalfConeAngle:					SimCast<ConeConstraint>(*c, EConstraintSubType::Cone, inOp).SetHalfConeAngle(f[0]); break;
		case ESimOp::SwingTwistSetNormalHalfConeAngle:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetNormalHalfConeAngle(f[0]); break;
		case ESimOp::SwingTwistSetPlaneHalfConeAngle:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetPlaneHalfConeAngle(f[0]); break;
		case ESimOp::SwingTwistSetTwistMinAngle:			SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTwistMinAngle(f[0]); break;
		case ESimOp::SwingTwistSetTwistMaxAngle:			SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTwistMaxAngle(f[0]); break;
		case ESimOp::SwingTwistSetMaxFrictionTorque:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetMaxFrictionTorque(f[0]); break;
		case ESimOp::SwingTwistSetSwingMotorState:			SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetSwingMotorState(EMotorState(i[0])); break;
		case ESimOp::SwingTwistSetTwistMotorState:			SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTwistMotorState(EMotorState(i[0])); break;
		case ESimOp::SwingTwistSetTargetAngularVelocityCS:	SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTargetAngularVelocityCS(SimLoad3(f)); break;
		case ESimOp::SwingTwistSetTargetAngularVelocityBS:	SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTargetAngularVelocityBS(SimLoad3(f)); break;
		case ESimOp::SwingTwistSetTargetOrientationCS:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTargetOrientationCS(SimLoadQuat(f)); break;
		case ESimOp::SwingTwistSetTargetOrientationBS:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).SetTargetOrientationBS(SimLoadQuat(f)); break;
		case ESimOp::SwingTwistSetSwingMotorSettings:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).GetSwingMotorSettings() = SimLoadMotor(i[0], f); break;
		case ESimOp::SwingTwistSetTwistMotorSettings:		SimCast<SwingTwistConstraint>(*c, EConstraintSubType::SwingTwist, inOp).GetTwistMotorSettings() = SimLoadMotor(i[0], f); break;

		// ConstraintOpRange.six_dof
		case ESimOp::SixDOFSetTranslationLimits:		SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetTranslationLimits(SimLoad3(f), SimLoad3(f + 3)); break;
		case ESimOp::SixDOFSetRotationLimits:			SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetRotationLimits(SimLoad3(f), SimLoad3(f + 3)); break;
		case ESimOp::SixDOFSetLimitsSpringSettings:		SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetLimitsSpringSettings(SixDOFConstraint::EAxis(i[0]), SimLoadSpring(i[1], f)); break;
		case ESimOp::SixDOFSetMaxFriction:				SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetMaxFriction(SixDOFConstraint::EAxis(i[0]), f[0]); break;
		case ESimOp::SixDOFSetMotorState:				SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetMotorState(SixDOFConstraint::EAxis(i[0]), EMotorState(i[1])); break;
		case ESimOp::SixDOFSetTargetVelocityCS:			SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetTargetVelocityCS(SimLoad3(f)); break;
		case ESimOp::SixDOFSetTargetAngularVelocityCS:	SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetTargetAngularVelocityCS(SimLoad3(f)); break;
		case ESimOp::SixDOFSetTargetPositionCS:			SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetTargetPositionCS(SimLoad3(f)); break;
		case ESimOp::SixDOFSetTargetOrientationCS:		SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetTargetOrientationCS(SimLoadQuat(f)); break;
		case ESimOp::SixDOFSetTargetOrientationBS:		SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).SetTargetOrientationBS(SimLoadQuat(f)); break;
		case ESimOp::SixDOFSetMotorSettings:			SimCast<SixDOFConstraint>(*c, EConstraintSubType::SixDOF, inOp).GetMotorSettings(SixDOFConstraint::EAxis(i[0])) = SimLoadMotor(i[1], f); break;

		// ConstraintOpRange.path
		case ESimOp::PathSetPath:
			{
				Ref<PathConstraintPathHermite> path = i[0] != 0? SimMakePath(mDesc.mConstraints[i[1]]) : nullptr;
				SimCast<PathConstraint>(*c, EConstraintSubType::Path, inOp).SetPath(path, f[0]);
				break;
			}
		case ESimOp::PathSetMaxFrictionForce:			SimCast<PathConstraint>(*c, EConstraintSubType::Path, inOp).SetMaxFrictionForce(f[0]); break;
		case ESimOp::PathSetPositionMotorState:			SimCast<PathConstraint>(*c, EConstraintSubType::Path, inOp).SetPositionMotorState(EMotorState(i[0])); break;
		case ESimOp::PathSetTargetVelocity:				SimCast<PathConstraint>(*c, EConstraintSubType::Path, inOp).SetTargetVelocity(f[0]); break;
		case ESimOp::PathSetTargetPathFraction:			SimCast<PathConstraint>(*c, EConstraintSubType::Path, inOp).SetTargetPathFraction(f[0]); break;
		case ESimOp::PathSetPositionMotorSettings:		SimCast<PathConstraint>(*c, EConstraintSubType::Path, inOp).GetPositionMotorSettings() = SimLoadMotor(i[0], f); break;

		// ConstraintOpRange.gear_rack_and_pinion
		case ESimOp::GearSetConstraints:				SetConstraints(SimCast<GearConstraint>(*c, EConstraintSubType::Gear, inOp), i[0], i[1]); break;
		case ESimOp::RackAndPinionSetConstraints:		SetConstraints(SimCast<RackAndPinionConstraint>(*c, EConstraintSubType::RackAndPinion, inOp), i[0], i[1]); break;

		default:
			Trace("SimulationReference: unknown constraint operation %u", inOp.mKind);
			abort();
		}

		return SimConstraintState(*c);
	}

	// -------------------------------------------------------------------------------------------------------------------
	// Phase 7: characters, ragdolls and vehicles

	// The index of a shape in mShapes (cInvalidHandle: nullptr or a shape that the scene did not create)
	uint32						GetShapeIndex(const Shape *inShape) const
	{
		for (uint32 i = 0; i < uint32(mShapes.size()); ++i)
			if (mShapes[i] == inShape)
				return i;
		return cInvalidHandle;
	}

	// The shape of a shape index (cInvalidHandle: nullptr)
	const Shape *				GetShape(uint32 inIndex) const
	{
		return inIndex != cInvalidHandle? mShapes[inIndex].GetPtr() : nullptr;
	}

	// The object of a handle (a scene that uses a handle that does not exist or that was destroyed is an error of the scene)
	template <class T>
	T *							GetObject(const Array<Ref<T>> &inObjects, const SimOp &inOp) const
	{
		if (inOp.mIndex >= inObjects.size() || inObjects[inOp.mIndex] == nullptr)
		{
			Trace("SimulationReference: operation %u on object handle %u that does not exist", inOp.mKind, inOp.mIndex);
			abort();
		}
		return inObjects[inOp.mIndex].GetPtr();
	}

	// The BodyState bits of a body (the result of the character operations)
	uint32						GetBodyState(const BodyID &inBodyID)
	{
		BodyInterface &bi = mSystem.GetBodyInterface();
		return (bi.IsActive(inBodyID)? 1 : 0) | (bi.IsAdded(inBodyID)? 2 : 0);
	}

	// The listener of a CharacterListenerMode
	CharacterContactListener *	GetCharacterListener(uint32 inMode)
	{
		switch (inMode)
		{
		case 0:		return nullptr;
		case 1:		return &mCharacterRecordListener;
		case 2:		return &mCharacterModifyListener;
		default:
			Trace("SimulationReference: unknown character listener mode %u", inMode);
			abort();
		}
	}

	// A Phase 7 operation (OpKind 400..599 in SimScenes.zig), see the ranges of Phase7OpRange
	uint32						ApplyObjectOp(const SimOp &inOp)
	{
		if (inOp.mKind < 430)
			return ApplyCharacterOp(inOp);
		if (inOp.mKind < 490)
			return ApplyCharacterVirtualOp(inOp);
		if (inOp.mKind < 520)
			return ApplyRagdollOp(inOp);
		if (inOp.mKind < cFirstSparePhase7Op)
			return ApplyVehicleOp(inOp);
		Trace("SimulationReference: unknown operation %u", inOp.mKind);
		abort();
	}

	// Phase7OpRange.character
	uint32						ApplyCharacterOp(const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;
		RVec3 r = SimLoadR3(inOp.mR);
		EActivation activation = inOp.mActivation == 0? EActivation::Activate : EActivation::DontActivate;
		bool lock = i[3] == 0;

		if (ESimOp(inOp.mKind) == ESimOp::CreateCharacter)
		{
			const SimCharacter &d = mDesc.mCharacters[inOp.mIndex];
			Ref<CharacterSettings> settings = new CharacterSettings;
			SimFillCharacterBaseSettings(d.mBase, mShapes, *settings);
			settings->mLayer = ObjectLayer(d.mLayer);
			settings->mMass = d.mMass;
			settings->mFriction = d.mFriction;
			settings->mGravityFactor = d.mGravityFactor;
			settings->mAllowedDOFs = EAllowedDOFs(d.mAllowedDOFs);
			Ref<Character> character = new Character(settings, SimLoadR3(d.mPosition), SimLoadQuat(d.mRotation), d.mUserData, &mSystem);
			mCharacters.push_back(character);
			return character->GetBodyID().GetIndexAndSequenceNumber();
		}

		Character *c = GetObject(mCharacters, inOp);
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::CharacterAddToPhysicsSystem:			c->AddToPhysicsSystem(activation, lock); break;
		case ESimOp::CharacterRemoveFromPhysicsSystem:		c->RemoveFromPhysicsSystem(lock); break;
		case ESimOp::CharacterActivate:						c->Activate(lock); break;
		case ESimOp::CharacterPostSimulation:				c->PostSimulation(f[0], lock); return uint32(c->GetGroundState());
		case ESimOp::CharacterSetLinearAndAngularVelocity:	c->SetLinearAndAngularVelocity(SimLoad3(f), SimLoad3(f + 3), lock); break;
		case ESimOp::CharacterSetLinearVelocity:			c->SetLinearVelocity(SimLoad3(f), lock); break;
		case ESimOp::CharacterAddLinearVelocity:			c->AddLinearVelocity(SimLoad3(f), lock); break;
		case ESimOp::CharacterAddImpulse:					c->AddImpulse(SimLoad3(f), lock); break;
		case ESimOp::CharacterSetPositionAndRotation:		c->SetPositionAndRotation(r, SimLoadQuat(f), activation, lock); break;
		case ESimOp::CharacterSetPosition:					c->SetPosition(r, activation, lock); break;
		case ESimOp::CharacterSetRotation:					c->SetRotation(SimLoadQuat(f), activation, lock); break;
		case ESimOp::CharacterSetLayer:						c->SetLayer(ObjectLayer(i[0]), lock); break;
		case ESimOp::CharacterSetShape:						return c->SetShape(GetShape(i[0]), f[0], lock)? 1 : 0;
		case ESimOp::CharacterSetMaxSlopeAngle:				c->SetMaxSlopeAngle(f[0]); break;
		case ESimOp::CharacterSetUp:						c->SetUp(SimLoad3(f)); break;
		case ESimOp::CharacterSetSupportingVolume:			c->SetSupportingVolume(Plane(SimLoad3(f), f[3])); break;

		case ESimOp::CharacterDestroy:
			mCharacters[inOp.mIndex] = nullptr;
			return 0;

		case ESimOp::CharacterSettingsHash:
			{
				CharacterSettings s = c->GetCharacterSettings(lock);
				SimHasher h;
				h.Vec3(s.mUp);
				h.Plane(s.mSupportingVolume);
				h.F32(s.mMaxSlopeAngle);
				h.Bool(s.mEnhancedInternalEdgeRemoval);
				h.U32(GetShapeIndex(s.mShape));
				h.U32(s.mLayer);
				h.F32(s.mMass);
				h.F32(s.mFriction);
				h.F32(s.mGravityFactor);
				h.U32(uint32(s.mAllowedDOFs));
				return h.mHash;
			}

		case ESimOp::CharacterCheckCollision:
			{
				SimCheckCollisionCollector collector;
				c->CheckCollision(r, SimLoadQuat(f), SimLoad3(f + 4), f[7], GetShape(i[0]), r, collector, lock);
				return collector.Result();
			}

		case ESimOp::CharacterExpectGroundState:			return uint32(c->GetGroundState()); // Changes nothing, the driver checks the result

		case ESimOp::CharacterBodyHash:
			{
				SimHasher h;
				h.Vec3(c->GetLinearVelocity(lock));
				RVec3 position;
				Quat rotation;
				c->GetPositionAndRotation(position, rotation, lock);
				h.RVec3(position);
				h.Quat(rotation);
				h.RVec3(c->GetPosition(lock));
				h.Quat(c->GetRotation(lock));
				h.RVec3(c->GetCenterOfMassPosition(lock));
				h.RMat44(c->GetWorldTransform(lock));
				TransformedShape shape = c->GetTransformedShape(lock);
				h.RVec3(shape.mShapePositionCOM);
				h.Quat(shape.mShapeRotation);
				h.Vec3(shape.GetShapeScale());
				h.U32(shape.mBodyID.GetIndexAndSequenceNumber());
				h.U32(GetShapeIndex(shape.mShape));
				return h.mHash;
			}

		case ESimOp::CharacterExpectBodyState:				break; // Changes nothing, the driver checks the BodyState bits

		case ESimOp::CharacterExpectValue: // Changes nothing, the driver checks the result (CharacterValue of SimScenes.zig)
			switch (i[0])
			{
			case 0:		return GetShapeIndex(c->GetShape());
			case 1:		return GetShapeIndex(mSystem.GetBodyInterface().GetShape(c->GetBodyID()).GetPtr());
			case 2:		return uint32(c->GetLayer());
			case 3:		return c->GetGroundBodyID() == GetHandle(inOp.mBody)? 1 : 0;
			default:
				Trace("SimulationReference: unknown character value %u", i[0]);
				abort();
			}

		default:
			Trace("SimulationReference: unknown character operation %u", inOp.mKind);
			abort();
		}
		return GetBodyState(c->GetBodyID());
	}

	// Phase7OpRange.character_virtual
	uint32						ApplyCharacterVirtualOp(const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;
		RVec3 r = SimLoadR3(inOp.mR);

		if (ESimOp(inOp.mKind) == ESimOp::CreateCharacterVirtual)
		{
			const SimCharacterVirtual &d = mDesc.mCharacterVirtuals[inOp.mIndex];
			Ref<CharacterVirtualSettings> settings = new CharacterVirtualSettings;
			SimFillCharacterBaseSettings(d.mBase, mShapes, *settings);
			settings->mID = CharacterID(d.mID);
			settings->mMass = d.mMass;
			settings->mMaxStrength = d.mMaxStrength;
			settings->mShapeOffset = SimLoad3(d.mShapeOffset);
			settings->mBackFaceMode = EBackFaceMode(d.mBackFaceMode);
			settings->mPredictiveContactDistance = d.mPredictiveContactDistance;
			settings->mMaxCollisionIterations = d.mMaxCollisionIterations;
			settings->mMaxConstraintIterations = d.mMaxConstraintIterations;
			settings->mMinTimeRemaining = d.mMinTimeRemaining;
			settings->mCollisionTolerance = d.mCollisionTolerance;
			settings->mCharacterPadding = d.mCharacterPadding;
			settings->mMaxNumHits = d.mMaxNumHits;
			settings->mHitReductionCosMaxAngle = d.mHitReductionCosMaxAngle;
			settings->mPenetrationRecoverySpeed = d.mPenetrationRecoverySpeed;
			settings->mInnerBodyShape = GetShape(d.mInnerBodyShape);
			settings->mInnerBodyIDOverride = BodyID(d.mInnerBodyIDOverride);
			settings->mInnerBodyLayer = ObjectLayer(d.mInnerBodyLayer);
			Ref<CharacterVirtual> character = new CharacterVirtual(settings, SimLoadR3(d.mPosition), SimLoadQuat(d.mRotation), d.mUserData, &mSystem);
			character->SetListener(GetCharacterListener(d.mListener));
			if (d.mCollisionGroup != cInvalidHandle)
			{
				mCharacterGroups[d.mCollisionGroup].Add(character);
				character->SetCharacterVsCharacterCollision(&mCharacterGroups[d.mCollisionGroup]);
			}
			mCharacterVirtuals.push_back(character);
			return character->GetInnerBodyID().GetIndexAndSequenceNumber();
		}

		CharacterVirtual *c = GetObject(mCharacterVirtuals, inOp);
		SimCharacterFilters filters(mObjectVsBroadPhaseLayerFilter, mObjectLayerPairFilter, inOp, GetHandle(inOp.mBody), c->GetInnerBodyID());
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::CVDestroy:
			for (CharacterVsCharacterCollisionSimple &g : mCharacterGroups)
				g.Remove(c);
			mCharacterVirtuals[inOp.mIndex] = nullptr;
			return 0;

		case ESimOp::CVSetLinearVelocity:					c->SetLinearVelocity(SimLoad3(f)); break;
		case ESimOp::CVSetPosition:							c->SetPosition(r); break;
		case ESimOp::CVSetRotation:							c->SetRotation(SimLoadQuat(f)); break;
		case ESimOp::CVSetUp:								c->SetUp(SimLoad3(f)); break;
		case ESimOp::CVSetMaxSlopeAngle:					c->SetMaxSlopeAngle(f[0]); break;
		case ESimOp::CVSetSupportingVolume:					c->SetSupportingVolume(Plane(SimLoad3(f), f[3])); break;
		case ESimOp::CVSetMass:								c->SetMass(f[0]); break;
		case ESimOp::CVSetMaxStrength:						c->SetMaxStrength(f[0]); break;
		case ESimOp::CVSetPenetrationRecoverySpeed:			c->SetPenetrationRecoverySpeed(f[0]); break;
		case ESimOp::CVSetEnhancedInternalEdgeRemoval:		c->SetEnhancedInternalEdgeRemoval(i[0] != 0); break;
		case ESimOp::CVSetMaxNumHits:						c->SetMaxNumHits(i[0]); break;
		case ESimOp::CVSetHitReductionCosMaxAngle:			c->SetHitReductionCosMaxAngle(f[0]); break;
		case ESimOp::CVSetShapeOffset:						c->SetShapeOffset(SimLoad3(f)); break;
		case ESimOp::CVSetUserData:							c->SetUserData(uint64(i[0]) | (uint64(i[1]) << 32)); break;
		case ESimOp::CVSetListener:							c->SetListener(GetCharacterListener(i[0])); break;
		case ESimOp::CVSetCharacterVsCharacterCollision:	c->SetCharacterVsCharacterCollision(i[0] != cInvalidHandle? &mCharacterGroups[i[0]] : nullptr); break;
		case ESimOp::CVAddToCollisionGroup:					mCharacterGroups[i[0]].Add(c); break;
		case ESimOp::CVRemoveFromCollisionGroup:			mCharacterGroups[i[0]].Remove(c); break;
		case ESimOp::CVUpdate:								c->Update(f[0], SimLoad3(f + 1), filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator); break;
		case ESimOp::CVExtendedUpdate:						c->ExtendedUpdate(f[0], SimLoad3(f + 1), SimLoadExtendedUpdateSettings(f + 4), filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator); break;
		case ESimOp::CVStickToFloor:						return c->StickToFloor(SimLoad3(f), filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator)? 1 : 0;
		case ESimOp::CVWalkStairs:							return c->WalkStairs(f[0], SimLoad3(f + 1), SimLoad3(f + 4), SimLoad3(f + 7), SimLoad3(f + 10), filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator)? 1 : 0;
		case ESimOp::CVCanWalkStairs:						return c->CanWalkStairs(SimLoad3(f))? 1 : 0;
		case ESimOp::CVRefreshContacts:						c->RefreshContacts(filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator); break;
		case ESimOp::CVUpdateGroundVelocity:				c->UpdateGroundVelocity(); break;
		case ESimOp::CVSetShape:							return c->SetShape(GetShape(i[3]), f[0], filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator)? 1 : 0;
		case ESimOp::CVSetInnerBodyShape:					c->SetInnerBodyShape(GetShape(i[0])); break;

		case ESimOp::CVCancelVelocityTowardsSteepSlopes:
			{
				SimHasher h;
				h.Vec3(c->CancelVelocityTowardsSteepSlopes(SimLoad3(f)));
				return h.mHash;
			}

		case ESimOp::CVStartTrackingContactChanges:			c->StartTrackingContactChanges(); break;
		case ESimOp::CVFinishTrackingContactChanges:		c->FinishTrackingContactChanges(); break;

		case ESimOp::CVCheckCollision:
			{
				SimCheckCollisionCollector collector;
				c->CheckCollision(r, SimLoadQuat(f), SimLoad3(f + 4), f[7], GetShape(i[3]), r, collector, filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes());
				return collector.Result();
			}

		case ESimOp::CVSettingsHash:
			{
				CharacterVirtualSettings s = c->GetCharacterVirtualSettings();
				SimHasher h;
				h.Vec3(s.mUp);
				h.Plane(s.mSupportingVolume);
				h.F32(s.mMaxSlopeAngle);
				h.Bool(s.mEnhancedInternalEdgeRemoval);
				h.U32(GetShapeIndex(s.mShape));
				h.U32(s.mID.GetValue());
				h.F32(s.mMass);
				h.F32(s.mMaxStrength);
				h.Vec3(s.mShapeOffset);
				h.U32(uint32(s.mBackFaceMode));
				h.F32(s.mPredictiveContactDistance);
				h.U32(s.mMaxCollisionIterations);
				h.U32(s.mMaxConstraintIterations);
				h.F32(s.mMinTimeRemaining);
				h.F32(s.mCollisionTolerance);
				h.F32(s.mCharacterPadding);
				h.U32(s.mMaxNumHits);
				h.F32(s.mHitReductionCosMaxAngle);
				h.F32(s.mPenetrationRecoverySpeed);
				h.U32(GetShapeIndex(s.mInnerBodyShape));
				h.U32(s.mInnerBodyIDOverride.GetIndexAndSequenceNumber());
				h.U32(s.mInnerBodyLayer);
				return h.mHash;
			}

		case ESimOp::CVHasCollidedWithBody:					return c->HasCollidedWith(GetHandle(inOp.mBody))? 1 : 0;

		case ESimOp::CVHasCollidedWithCharacter:
			{
				SimOp other_op = inOp;
				other_op.mIndex = i[0];
				const CharacterVirtual *other = GetObject(mCharacterVirtuals, other_op);
				return (c->HasCollidedWith(other->GetID())? 1 : 0) | (c->HasCollidedWith(other)? 2 : 0);
			}

		case ESimOp::CVExpectGroundState:					return uint32(c->GetGroundState()); // Changes nothing, the driver checks the result

		case ESimOp::CVSaveRestoreState:
			{
				StateRecorderImpl recorder;
				c->SaveState(recorder);
				std::string data = recorder.GetData();
				recorder.Rewind();
				recorder.SetValidating(i[0] != 0);
				c->RestoreState(recorder);
				return SimHashBytes(2166136261u, data.data(), data.size());
			}

		case ESimOp::CVExpectValue:											// Changes nothing, the driver checks the result
			switch (i[0])
			{
			case 0:		return c->GetMaxHitsExceeded()? 1 : 0;
			case 1:		return uint32(c->GetActiveContacts().size());
			case 2:
			case 3:
				{
					uint32 n = 0;
					for (const CharacterContact &contact : c->GetActiveContacts())
						if (contact.mHadCollision && (i[0] == 2 || contact.mCharacterB != nullptr))
							++n;
					return n;
				}
			case 4:		return c->GetGroundBodyID() == GetHandle(inOp.mBody)? 1 : 0;
			case 5:		return c->GetGroundVelocity().IsNearZero(1.0e-6f)? 0 : 1;
			case 6:		return GetShapeIndex(c->GetShape());
			case 7:		return c->GetInnerBodyID().IsInvalid()? 0 : 1;
			default:
				Trace("SimulationReference: unknown character virtual value %u", i[0]);
				abort();
			}

		case ESimOp::CVSetStepVelocity:
			{
				// Character::Step of CharacterVirtualTests.cpp
				Vec3 current_vertical_velocity = Vec3(0, c->GetLinearVelocity().GetY(), 0);
				Vec3 ground_velocity = c->GetGroundVelocity();
				Vec3 new_velocity;
				if (c->GetGroundState() == CharacterBase::EGroundState::OnGround // If on ground
					&& (current_vertical_velocity.GetY() - ground_velocity.GetY()) < 0.1f) // And not moving away from ground
				{
					// Assume velocity of ground when on ground
					new_velocity = ground_velocity;

					// Jump
					new_velocity += Vec3(0, f[7], 0);
				}
				else
					new_velocity = current_vertical_velocity;

				// Gravity
				new_velocity += SimLoad3(f + 3) * f[6];

				// Player input
				new_velocity += SimLoad3(f);

				c->SetLinearVelocity(new_velocity);
				break;
			}

		case ESimOp::CVPerfUpdate:
			{
				// The step of a character in CharacterVirtualScene::UpdateTest of PerformanceTest

				// Calculate new vertical velocity
				Vec3 new_velocity;
				if (c->GetGroundState() == CharacterVirtual::EGroundState::OnGround	// If on ground
					&& c->GetLinearVelocity().GetY() < 0.1f)						// And not moving away from ground
					new_velocity = Vec3::sZero();
				else
					new_velocity = c->GetLinearVelocity() * Vec3(0, 1, 0);
				new_velocity += mSystem.GetGravity() * f[0];

				// Deterministic random input
				uint64 count = inOp.mCount;
				uint64 hash = Hash<uint64> {} (count);
				int x = int(hash % 10);
				int y = int((hash / 10) % 10);
				int speed = int((hash / 100) % 10);

				// Determine target position
				RVec3 target = RVec3(Real(4.0) * x - Real(20.0), Real(5.0), Real(4.0) * y - Real(20.0)); // PerformanceTest: RVec3(4.0_r * x - 20.0_r, 5.0_r, 4.0_r * y - 20.0_r)

				// Determine new character velocity
				Vec3 direction = Vec3(target - c->GetPosition()).NormalizedOr(Vec3::sZero());
				direction.SetY(0);
				new_velocity += (5.0f + 0.5f * speed) * direction;
				c->SetLinearVelocity(new_velocity);

				// Update the character position
				CharacterVirtual::ExtendedUpdateSettings update_settings;
				c->ExtendedUpdate(f[0], mSystem.GetGravity(), update_settings, filters.BroadPhase(), filters.ObjectLayers(), filters.Bodies(), filters.Shapes(), mTempAllocator);
				break;
			}

		default:
			Trace("SimulationReference: unknown character virtual operation %u", inOp.mKind);
			abort();
		}
		return 0;
	}

	// The RagdollSettings of ragdoll description inIndex (the parts from the body descriptions of the joints, the shapes of
	// this world); outStabilize: the result of Stabilize (1 / 0, 2: not called)
	Ref<RagdollSettings>		MakeRagdollSettings(uint32 inIndex, uint32 *outStabilize = nullptr) const
	{
		const SimRagdoll &rd = mDesc.mRagdolls[inIndex];
		Array<BodyCreationSettings> parts;
		for (uint32 j = 0; j < rd.mNumJoints; ++j)
			parts.push_back(MakeBodyCreationSettings(mDesc.mBodies[mDesc.mRagdollJoints[rd.mFirstJoint + j].mBody]));
		return SimMakeRagdollSettings(mDesc, rd, parts, outStabilize);
	}

	// The SkeletonPose of pose description inPose for a ragdoll
	void						MakeRagdollPose(const Ragdoll &inRagdoll, uint32 inPose, SkeletonPose &outPose) const
	{
		SimMakePose(mDesc, inRagdoll.GetRagdollSettings()->GetSkeleton(), inPose, outPose);
	}

	// Phase7OpRange.ragdoll
	uint32						ApplyRagdollOp(const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;
		EActivation activation = inOp.mActivation == 0? EActivation::Activate : EActivation::DontActivate;
		bool lock = i[3] == 0;

		if (ESimOp(inOp.mKind) == ESimOp::CreateRagdoll)
		{
			// The settings of the description, built at its first creation and shared by its ragdolls
			uint32 d = inOp.mIndex;
			if (mRagdollSettings.size() < mDesc.mNumRagdolls)
				mRagdollSettings.resize(mDesc.mNumRagdolls);
			if (mRagdollSettings[d] == nullptr)
				mRagdollSettings[d] = MakeRagdollSettings(d);
			const SimRagdoll &rd = mDesc.mRagdolls[d];
			Ref<Ragdoll> ragdoll = mRagdollSettings[d]->CreateRagdoll(CollisionGroup::GroupID(rd.mCollisionGroup), rd.mUserData, &mSystem);
			mRagdolls.push_back(ragdoll);
			return ragdoll != nullptr? uint32(ragdoll->GetBodyCount()) : 0;
		}

		Ragdoll *rag = GetObject(mRagdolls, inOp);
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::RagdollAddToPhysicsSystem:				rag->AddToPhysicsSystem(activation, lock); break;
		case ESimOp::RagdollRemoveFromPhysicsSystem:		rag->RemoveFromPhysicsSystem(lock); break;
		case ESimOp::RagdollActivate:						rag->Activate(lock); break;
		case ESimOp::RagdollIsActive:						return rag->IsActive(lock)? 1 : 0;
		case ESimOp::RagdollSetGroupID:						rag->SetGroupID(CollisionGroup::GroupID(i[0]), lock); break;

		case ESimOp::RagdollSetPose:
			{
				SkeletonPose pose;
				MakeRagdollPose(*rag, i[0], pose);
				rag->SetPose(pose, lock);
				break;
			}

		case ESimOp::RagdollSetPoseMatrices:
			{
				Array<Mat44> matrices = SimMakePoseMatrices(mDesc, i[0], uint32(rag->GetBodyCount()));
				rag->SetPose(SimLoadR3(mDesc.mRagdollPoses[i[0]].mRootOffset), matrices.data(), lock);
				break;
			}

		case ESimOp::RagdollResetWarmStart:					rag->ResetWarmStart(); break;

		case ESimOp::RagdollDriveToPoseUsingKinematics:
			{
				SkeletonPose pose;
				MakeRagdollPose(*rag, i[0], pose);
				rag->DriveToPoseUsingKinematics(pose, f[0], lock);
				break;
			}

		case ESimOp::RagdollDriveToPoseUsingKinematicsMatrices:
			{
				Array<Mat44> matrices = SimMakePoseMatrices(mDesc, i[0], uint32(rag->GetBodyCount()));
				rag->DriveToPoseUsingKinematics(SimLoadR3(mDesc.mRagdollPoses[i[0]].mRootOffset), matrices.data(), f[0], lock);
				break;
			}

		case ESimOp::RagdollDriveToPoseUsingMotors:
			{
				SkeletonPose pose;
				MakeRagdollPose(*rag, i[0], pose);
				rag->DriveToPoseUsingMotors(pose);
				break;
			}

		case ESimOp::RagdollDriveToPoseUsingMotorsVelocity:
			{
				SkeletonPose prev_pose, pose;
				MakeRagdollPose(*rag, i[1], prev_pose);
				MakeRagdollPose(*rag, i[0], pose);
				rag->DriveToPoseUsingMotors(prev_pose, pose, f[0]);
				break;
			}

		case ESimOp::RagdollSetLinearAndAngularVelocity:	rag->SetLinearAndAngularVelocity(SimLoad3(f), SimLoad3(f + 3), lock); break;
		case ESimOp::RagdollSetLinearVelocity:				rag->SetLinearVelocity(SimLoad3(f), lock); break;
		case ESimOp::RagdollAddLinearVelocity:				rag->AddLinearVelocity(SimLoad3(f), lock); break;
		case ESimOp::RagdollAddImpulse:						rag->AddImpulse(SimLoad3(f), lock); break;

		case ESimOp::RagdollDestroy:
			mRagdolls[inOp.mIndex] = nullptr;
			return 0;

		case ESimOp::RagdollSettingsHash:
			{
				std::stringstream data;
				StreamOutWrapper stream(data);
				rag->GetRagdollSettings()->SaveBinaryState(stream, i[0] != 0, i[1] != 0);
				std::string bytes = data.str();
				return SimHashBytes(2166136261u, bytes.data(), bytes.size());
			}

		case ESimOp::RagdollGetPoseHash:
			{
				SkeletonPose pose;
				pose.SetSkeleton(rag->GetRagdollSettings()->GetSkeleton());
				rag->GetPose(pose, lock);
				SimHasher h;
				h.RVec3(pose.GetRootOffset());
				for (const Mat44 &m : pose.GetJointMatrices())
					for (int col = 0; col < 4; ++col)
					{
						Vec4 v = m.GetColumn4(col);
						float values[4] = { v.GetX(), v.GetY(), v.GetZ(), v.GetW() };
						h.Value(values);
					}
				return h.mHash;
			}

		case ESimOp::RagdollExpectActive:					break; // Changes nothing, the driver checks the result

		default:
			Trace("SimulationReference: unknown ragdoll operation %u", inOp.mKind);
			abort();
		}
		return rag->IsActive()? 1 : 0;
	}

	// The vehicle of a handle
	SimVehicleData *			GetVehicle(const SimOp &inOp) const
	{
		if (inOp.mIndex >= mVehicles.size())
		{
			Trace("SimulationReference: operation %u on vehicle handle %u that does not exist", inOp.mKind, inOp.mIndex);
			abort();
		}
		return mVehicles[inOp.mIndex];
	}

	// The wheeled controller of a vehicle (a wheeled or motorcycle controller, a scene error otherwise)
	static WheeledVehicleController &GetWheeledController(SimVehicleData &inVehicle, const SimOp &inOp)
	{
		if (inVehicle.mController == 1)
		{
			Trace("SimulationReference: wheeled operation %u on the tracked vehicle handle %u", inOp.mKind, inOp.mIndex);
			abort();
		}
		return *static_cast<WheeledVehicleController *>(inVehicle.mConstraint->GetController());
	}

	static TrackedVehicleController &GetTrackedController(SimVehicleData &inVehicle, const SimOp &inOp)
	{
		if (inVehicle.mController != 1)
		{
			Trace("SimulationReference: tracked operation %u on the vehicle handle %u", inOp.mKind, inOp.mIndex);
			abort();
		}
		return *static_cast<TrackedVehicleController *>(inVehicle.mConstraint->GetController());
	}

	static MotorcycleController &GetMotorcycleController(SimVehicleData &inVehicle, const SimOp &inOp)
	{
		if (inVehicle.mController != 2)
		{
			Trace("SimulationReference: motorcycle operation %u on the vehicle handle %u", inOp.mKind, inOp.mIndex);
			abort();
		}
		return *static_cast<MotorcycleController *>(inVehicle.mConstraint->GetController());
	}

	// The engine / transmission of the controller of a vehicle (every controller kind has them)
	static VehicleEngine &		GetEngine(SimVehicleData &inVehicle)
	{
		VehicleController *c = inVehicle.mConstraint->GetController();
		return inVehicle.mController == 1? static_cast<TrackedVehicleController *>(c)->GetEngine() : static_cast<WheeledVehicleController *>(c)->GetEngine();
	}

	static VehicleTransmission &GetTransmission(SimVehicleData &inVehicle)
	{
		VehicleController *c = inVehicle.mConstraint->GetController();
		return inVehicle.mController == 1? static_cast<TrackedVehicleController *>(c)->GetTransmission() : static_cast<WheeledVehicleController *>(c)->GetTransmission();
	}

	// The step callbacks of a vehicle (VehicleCallbacks bits of SimScenes.zig)
	void						SetVehicleCallbacks(SimVehicleData &ioVehicle, uint32 inCallbacks)
	{
		uint32 handle = ioVehicle.mHandle;
		uint32 controller = ioVehicle.mController;
		VehicleConstraint &v = *ioVehicle.mConstraint;
		if ((inCallbacks & (SimVehicleCallbacks::cPreStep | SimVehicleCallbacks::cSteer)) != 0)
			v.SetPreStepCallback([this, handle, controller, inCallbacks](VehicleConstraint &inVehicle, const PhysicsStepListenerContext &inContext) {
				if ((inCallbacks & SimVehicleCallbacks::cPreStep) != 0)
					mRecorder.Add(SimVehicleRecord(ESimRecordKind::VehiclePreStep, handle, inVehicle, inContext));
				if ((inCallbacks & SimVehicleCallbacks::cSteer) != 0 && controller != 1)
				{
					WheeledVehicleController *c = static_cast<WheeledVehicleController *>(inVehicle.GetController());
					c->SetRightInput(c->GetRightInput() > 0.0f? -0.25f : 0.5f);
				}
			});
		else
			v.SetPreStepCallback(nullptr);
		if ((inCallbacks & SimVehicleCallbacks::cPostCollide) != 0)
			v.SetPostCollideCallback([this, handle](VehicleConstraint &inVehicle, const PhysicsStepListenerContext &inContext) {
				mRecorder.Add(SimVehicleRecord(ESimRecordKind::VehiclePostCollide, handle, inVehicle, inContext));
			});
		else
			v.SetPostCollideCallback(nullptr);
		if ((inCallbacks & SimVehicleCallbacks::cPostStep) != 0)
			v.SetPostStepCallback([this, handle](VehicleConstraint &inVehicle, const PhysicsStepListenerContext &inContext) {
				mRecorder.Add(SimVehicleRecord(ESimRecordKind::VehiclePostStep, handle, inVehicle, inContext));
			});
		else
			v.SetPostStepCallback(nullptr);
	}

	// VehicleCombineFriction / VehicleTireMaxImpulse of SimScenes.zig
	static void					SetVehicleCombineFriction(SimVehicleData &ioVehicle, uint32 inMode)
	{
		ioVehicle.mConstraint->SetCombineFriction(inMode == 1? VehicleConstraint::CombineFunction(SimVehicleCombineFriction) : ioVehicle.mDefaultCombineFriction);
	}

	static void					SetVehicleTireMaxImpulse(SimVehicleData &ioVehicle, uint32 inMode, const SimOp *inOp)
	{
		if (ioVehicle.mController == 1)
		{
			if (inMode == 0 && inOp == nullptr)
				return; // A tracked vehicle has no tire callback, its description has the default
			Trace("SimulationReference: tire max impulse callback on the tracked vehicle handle %u", ioVehicle.mHandle);
			abort();
		}
		WheeledVehicleController *c = static_cast<WheeledVehicleController *>(ioVehicle.mConstraint->GetController());
		c->SetTireMaxImpulseCallback(inMode == 1? WheeledVehicleController::TireMaxImpulseCallback(SimVehicleTireMaxImpulse) : ioVehicle.mDefaultTireMaxImpulse);
	}

	// The SaveBinaryState bytes of settings, as a hash
	template <class T>
	static uint32				HashBinaryState(const T &inSettings)
	{
		std::stringstream data;
		StreamOutWrapper stream(data);
		inSettings.SaveBinaryState(stream);
		std::string bytes = data.str();
		return SimHashBytes(2166136261u, bytes.data(), bytes.size());
	}

	// Phase7OpRange.vehicle_tracked (TrackedVehicleSlot.applyVehicleOp of SimulationParity.zig)
	static void					ApplyTrackedOp(SimVehicleData &ioVehicle, const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;
		TrackedVehicleController &controller = GetTrackedController(ioVehicle, inOp);
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::TrackedSetDriverInput:					controller.SetDriverInput(f[0], f[1], f[2], f[3]); break;
		case ESimOp::TrackedSetForwardInput:				controller.SetForwardInput(f[0]); break;
		case ESimOp::TrackedSetLeftRatio:					controller.SetLeftRatio(f[0]); break;
		case ESimOp::TrackedSetRightRatio:					controller.SetRightRatio(f[0]); break;
		case ESimOp::TrackedSetBrakeInput:					controller.SetBrakeInput(f[0]); break;

		case ESimOp::TrackedSetTrack:
			{
				VehicleTrack &track = controller.GetTracks()[i[0]];
				track.mAngularVelocity = f[0];
				track.mInertia = f[1];
				track.mAngularDamping = f[2];
				track.mMaxBrakeTorque = f[3];
				track.mDifferentialRatio = f[4];
				break;
			}

		default:
			Trace("SimulationReference: unknown tracked vehicle operation %u", inOp.mKind);
			abort();
		}
	}

	// Phase7OpRange.vehicle_motorcycle (MotorcycleSlot.applyVehicleOp of SimulationParity.zig)
	static void					ApplyMotorcycleOp(SimVehicleData &ioVehicle, const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;
		MotorcycleController &controller = GetMotorcycleController(ioVehicle, inOp);
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::MotorcycleEnableLeanController:		controller.EnableLeanController(i[0] != 0); break;
		case ESimOp::MotorcycleEnableLeanSteeringLimit:		controller.EnableLeanSteeringLimit(i[0] != 0); break;
		case ESimOp::MotorcycleSetLeanSpringConstant:		controller.SetLeanSpringConstant(f[0]); break;
		case ESimOp::MotorcycleSetLeanSpringDamping:		controller.SetLeanSpringDamping(f[0]); break;
		case ESimOp::MotorcycleSetLeanSpringIntegrationCoefficient: controller.SetLeanSpringIntegrationCoefficient(f[0]); break;
		case ESimOp::MotorcycleSetLeanSpringIntegrationCoefficientDecay: controller.SetLeanSpringIntegrationCoefficientDecay(f[0]); break;
		case ESimOp::MotorcycleSetLeanSmoothingFactor:		controller.SetLeanSmoothingFactor(f[0]); break;

		default:
			Trace("SimulationReference: unknown motorcycle operation %u", inOp.mKind);
			abort();
		}
	}

	// Phase7OpRange.vehicle (and the routing of vehicle_tracked / vehicle_motorcycle: the vehicle of the handle, the
	// operation of the variant, the ConstraintState of the vehicle, like ZoltWorld.applyPhase7Op of SimulationParity.zig)
	uint32						ApplyVehicleOp(const SimOp &inOp)
	{
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;

		if (inOp.mKind >= cFirstTrackedOp)
		{
			SimVehicleData &v = *GetVehicle(inOp);
			if (inOp.mKind >= cFirstMotorcycleOp)
				ApplyMotorcycleOp(v, inOp);
			else
				ApplyTrackedOp(v, inOp);
			return SimConstraintState(*v.mConstraint);
		}

		if (ESimOp(inOp.mKind) == ESimOp::CreateVehicle)
		{
			const SimVehicle &d = mDesc.mVehicles[inOp.mIndex];
			Ref<VehicleConstraintSettings> settings = SimMakeVehicleSettings(mDesc, d);
			SimVehicleData *v = new SimVehicleData;
			v->mHandle = uint32(mVehicles.size());
			v->mController = d.mController;
			{
				BodyLockWrite lock(mSystem.GetBodyLockInterface(), GetHandle(d.mBody));
				if (!lock.Succeeded())
				{
					Trace("SimulationReference: vehicle description %u: the body handle %u does not exist", inOp.mIndex, d.mBody);
					abort();
				}
				v->mConstraint = new VehicleConstraint(lock.GetBody(), *settings);
			}
			v->mTester = SimMakeTester(d.mTester.mKind, d.mTester.mObjectLayer, SimLoad3(d.mTester.mUp), d.mTester.mMaxSlopeAngle, d.mTester.mRadius, d.mTester.mConvexRadiusFraction);
			v->mConstraint->SetVehicleCollisionTester(v->mTester);
			v->mDefaultCombineFriction = v->mConstraint->GetCombineFriction();
			if (v->mController != 1)
				v->mDefaultTireMaxImpulse = static_cast<WheeledVehicleController *>(v->mConstraint->GetController())->GetTireMaxImpulseCallback();
			SetVehicleCallbacks(*v, d.mCallbacks);
			SetVehicleCombineFriction(*v, d.mCombineFriction);
			SetVehicleTireMaxImpulse(*v, d.mTireMaxImpulse, nullptr);
			mSystem.AddConstraint(v->mConstraint);
			mSystem.AddStepListener(v->mConstraint);
			v->mAdded = true;
			mVehicles.push_back(v);
			return GetNumConstraints();
		}

		SimVehicleData &v = *GetVehicle(inOp);
		VehicleConstraint &c = *v.mConstraint;
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::VehicleRemove:
			mSystem.RemoveStepListener(&c);
			mSystem.RemoveConstraint(&c);
			v.mAdded = false;
			return GetNumConstraints();

		case ESimOp::VehicleAdd:
			mSystem.AddConstraint(&c);
			mSystem.AddStepListener(&c);
			v.mAdded = true;
			return GetNumConstraints();

		case ESimOp::VehicleSetMaxPitchRollAngle:			c.SetMaxPitchRollAngle(f[0]); break;
		case ESimOp::VehicleOverrideGravity:				c.OverrideGravity(SimLoad3(f)); break;
		case ESimOp::VehicleResetGravityOverride:			c.ResetGravityOverride(); break;
		case ESimOp::VehicleSetNumStepsBetweenCollisionTestActive: c.SetNumStepsBetweenCollisionTestActive(i[0]); break;
		case ESimOp::VehicleSetNumStepsBetweenCollisionTestInactive: c.SetNumStepsBetweenCollisionTestInactive(i[0]); break;

		case ESimOp::VehicleSetCollisionTester:
			v.mTester = SimMakeTester(i[0], i[1], SimLoad3(f), f[3], f[4], f[5]);
			c.SetVehicleCollisionTester(v.mTester);
			break;

		case ESimOp::VehicleTesterSetObjectLayer:			v.mTester->SetObjectLayer(ObjectLayer(i[0])); break;

		case ESimOp::VehicleTesterSetFilters:
			if (i[0] == 0)
			{
				v.mTester->SetBroadPhaseLayerFilter(nullptr);
				v.mTester->SetObjectLayerFilter(nullptr);
				v.mTester->SetBodyFilter(nullptr);
			}
			else
			{
				v.mBroadPhaseLayerFilter = std::make_unique<DefaultBroadPhaseLayerFilter>(mObjectVsBroadPhaseLayerFilter, ObjectLayer(i[1]));
				v.mObjectLayerFilter = std::make_unique<DefaultObjectLayerFilter>(mObjectLayerPairFilter, ObjectLayer(i[1]));
				v.mBodyFilter = std::make_unique<IgnoreMultipleBodiesFilter>();
				v.mBodyFilter->IgnoreBody(c.GetVehicleBody()->GetID());
				BodyID ignored = GetHandle(inOp.mBody);
				if (!ignored.IsInvalid())
					v.mBodyFilter->IgnoreBody(ignored);
				v.mTester->SetBroadPhaseLayerFilter(v.mBroadPhaseLayerFilter.get());
				v.mTester->SetObjectLayerFilter(v.mObjectLayerFilter.get());
				v.mTester->SetBodyFilter(v.mBodyFilter.get());
			}
			break;

		case ESimOp::VehicleSetCombineFriction:				SetVehicleCombineFriction(v, i[0]); break;
		case ESimOp::VehicleSetCallbacks:					SetVehicleCallbacks(v, i[0]); break;
		case ESimOp::VehicleWheelSetAngularVelocity:		c.GetWheel(i[0])->SetAngularVelocity(f[0]); break;
		case ESimOp::VehicleWheelSetRotationAngle:			c.GetWheel(i[0])->SetRotationAngle(f[0]); break;
		case ESimOp::VehicleWheelSetSteerAngle:				c.GetWheel(i[0])->SetSteerAngle(f[0]); break;

		case ESimOp::VehicleSetAntiRollBar:
			{
				VehicleAntiRollBar &bar = c.GetAntiRollBars()[i[0]];
				bar.mLeftWheel = int(i[1]);
				bar.mRightWheel = int(i[2]);
				bar.mStiffness = f[0];
				break;
			}

		case ESimOp::VehicleSettingsHash:					return HashBinaryState(*c.GetConstraintSettings());
		case ESimOp::VehicleControllerSettingsHash:			return HashBinaryState(*c.GetController()->GetSettings());

		case ESimOp::VehicleExpectWheelContacts:
			{
				uint32 bits = 0;
				for (uint w = 0; w < uint(c.GetWheels().size()); ++w)
					if (c.GetWheel(w)->HasContact())
						bits |= 1u << w;
				return bits; // Changes nothing, the driver checks the result
			}

		case ESimOp::VehicleEngineSetCurrentRPM:			GetEngine(v).SetCurrentRPM(f[0]); break;

		case ESimOp::VehicleEngineSetSettings:
			{
				VehicleEngine &engine = GetEngine(v);
				engine.mMaxTorque = f[0];
				engine.mMinRPM = f[1];
				engine.mMaxRPM = f[2];
				engine.mInertia = f[3];
				engine.mAngularDamping = f[4];
				break;
			}

		case ESimOp::VehicleEngineSetTorqueCurve:			SimLoadCurve(mDesc, i[0], i[1], GetEngine(v).mNormalizedTorque); break;
		case ESimOp::VehicleTransmissionSet:				GetTransmission(v).Set(int(i[0]), f[0]); break;

		case ESimOp::VehicleTransmissionSetSettings:
			{
				VehicleTransmission &transmission = GetTransmission(v);
				transmission.mMode = ETransmissionMode(i[0]);
				transmission.mSwitchTime = f[0];
				transmission.mClutchReleaseTime = f[1];
				transmission.mSwitchLatency = f[2];
				transmission.mShiftUpRPM = f[3];
				transmission.mShiftDownRPM = f[4];
				transmission.mClutchStrength = f[5];
				break;
			}

		case ESimOp::VehicleTransmissionSetGearRatios:
			{
				VehicleTransmission &transmission = GetTransmission(v);
				transmission.mGearRatios = SimLoadFloats(mDesc, i[0], i[1]);
				transmission.mReverseGearRatios = SimLoadFloats(mDesc, i[2], i[3]);
				break;
			}

		case ESimOp::WheeledSetDriverInput:					GetWheeledController(v, inOp).SetDriverInput(f[0], f[1], f[2], f[3]); break;
		case ESimOp::WheeledSetForwardInput:				GetWheeledController(v, inOp).SetForwardInput(f[0]); break;
		case ESimOp::WheeledSetRightInput:					GetWheeledController(v, inOp).SetRightInput(f[0]); break;
		case ESimOp::WheeledSetBrakeInput:					GetWheeledController(v, inOp).SetBrakeInput(f[0]); break;
		case ESimOp::WheeledSetHandBrakeInput:				GetWheeledController(v, inOp).SetHandBrakeInput(f[0]); break;

		case ESimOp::WheeledSetDifferential:
			{
				VehicleDifferentialSettings &d = GetWheeledController(v, inOp).GetDifferentials()[i[0]];
				d.mLeftWheel = int(i[1]);
				d.mRightWheel = int(i[2]);
				d.mDifferentialRatio = f[0];
				d.mLeftRightSplit = f[1];
				d.mLimitedSlipRatio = f[2];
				d.mEngineTorqueRatio = f[3];
				break;
			}

		case ESimOp::WheeledSetDifferentialLimitedSlipRatio: GetWheeledController(v, inOp).SetDifferentialLimitedSlipRatio(f[0]); break;
		case ESimOp::WheeledSetTireMaxImpulseCallback:		SetVehicleTireMaxImpulse(v, i[0], &inOp); break;

		// The Constraint interface of the VehicleConstraint (a vehicle is not a constraint handle)
		case ESimOp::VehicleSetEnabled:						c.SetEnabled(i[0] != 0); break;
		case ESimOp::VehicleResetWarmStart:					c.ResetWarmStart(); break;
		case ESimOp::VehicleSetConstraintPriority:			c.SetConstraintPriority(i[0]); break;

		case ESimOp::VehicleSetStepsOverride:
			c.SetNumVelocityStepsOverride(i[0]);
			c.SetNumPositionStepsOverride(i[1]);
			break;

		case ESimOp::VehicleSetUserData:					c.SetUserData(uint64(i[0]) | (uint64(i[1]) << 32)); break;

		case ESimOp::VehicleExpectConstraintState:
			// Changes nothing, the driver checks the result: a disabled vehicle whose body is active runs OnStep (its
			// callbacks, the wheel collision tests) but is not solved (IsActive includes mEnabled)
			return SimConstraintState(c) | (mSystem.GetBodyInterface().IsActive(c.GetVehicleBody()->GetID())? 4u : 0u);

		case ESimOp::VehicleWheelWVApplyTorque:
			GetWheeledController(v, inOp);
			static_cast<WheelWV *>(c.GetWheel(i[0]))->ApplyTorque(f[0], f[1]);
			break;

		case ESimOp::VehicleEngineApplyTorque:				GetEngine(v).ApplyTorque(f[0], f[1]); break;
		case ESimOp::VehicleEngineApplyDamping:				GetEngine(v).ApplyDamping(f[0]); break;

		case ESimOp::WheeledSetDifferentials:
			{
				WheeledVehicleController::Differentials &differentials = GetWheeledController(v, inOp).GetDifferentials();
				differentials.clear();
				for (uint32 d = 0; d < i[1]; ++d)
				{
					VehicleDifferentialSettings diff;
					SimFillDifferential(mDesc.mVehicleDifferentials[i[0] + d], diff);
					differentials.push_back(diff);
				}
				break;
			}

		default:
			Trace("SimulationReference: unknown vehicle operation %u", inOp.mKind);
			abort();
		}
		return SimConstraintState(c);
	}

	// The object state (ObjectField of SimScenes.zig): every character, character virtual, ragdoll and vehicle in handle
	// order, the destroyed ones skipped
	void						WriteObjectState(SimObjectStateWriter &ioWriter)
	{
		for (uint32 h = 0; h < uint32(mCharacters.size()); ++h)
			if (mCharacters[h] != nullptr)
				WriteCharacterState(ioWriter, h, *mCharacters[h]);
		for (uint32 h = 0; h < uint32(mCharacterVirtuals.size()); ++h)
			if (mCharacterVirtuals[h] != nullptr)
				WriteCharacterVirtualState(ioWriter, h, *mCharacterVirtuals[h]);
		for (uint32 h = 0; h < uint32(mRagdolls.size()); ++h)
			if (mRagdolls[h] != nullptr)
				WriteRagdollState(ioWriter, h, *mRagdolls[h]);
		for (SimVehicleData *v : mVehicles)
			WriteVehicleState(ioWriter, *v);
	}

	// The SaveState bytes of a character
	static void					WriteSaveState(SimObjectStateWriter &ioWriter, ESimObjectField inField, uint32 inHandle, const CharacterBase &inCharacter)
	{
		StateRecorderImpl recorder;
		inCharacter.SaveState(recorder);
		std::string data = recorder.GetData();
		ioWriter.Begin(inField, inHandle, 0);
		ioWriter.Raw(data.data(), data.size());
		ioWriter.End();
	}

	void						WriteTransformedShape(SimObjectStateWriter &ioWriter, ESimObjectField inField, uint32 inHandle, const TransformedShape &inShape) const
	{
		ioWriter.Begin(inField, inHandle, 0);
		ioWriter.RVec3(inShape.mShapePositionCOM);
		ioWriter.Quat(inShape.mShapeRotation);
		ioWriter.Vec3(inShape.GetShapeScale());
		ioWriter.U32(inShape.mBodyID.GetIndexAndSequenceNumber());
		ioWriter.U32(GetShapeIndex(inShape.mShape));
		ioWriter.End();
	}

	void						WriteCharacterState(SimObjectStateWriter &ioWriter, uint32 inHandle, const Character &inCharacter) const
	{
		WriteSaveState(ioWriter, ESimObjectField::CharacterSaveState, inHandle, inCharacter);

		ioWriter.Begin(ESimObjectField::CharacterProperties, inHandle, 0);
		ioWriter.U32(inCharacter.GetBodyID().GetIndexAndSequenceNumber());
		ioWriter.U32(inCharacter.GetLayer());
		ioWriter.U32(GetShapeIndex(inCharacter.GetShape()));
		ioWriter.Bool(inCharacter.IsSupported());
		ioWriter.U32(SimMaterialHash(inCharacter.GetGroundMaterial()));
		ioWriter.U64(inCharacter.GetGroundUserData());
		ioWriter.End();

		ioWriter.Begin(ESimObjectField::CharacterBase, inHandle, 0);
		ioWriter.Vec3(inCharacter.GetUp());
		ioWriter.F32(inCharacter.GetCosMaxSlopeAngle());
		ioWriter.Plane(inCharacter.GetSupportingVolume());
		ioWriter.End();

		// The body (a character without body: out of bodies)
		if (inCharacter.GetBodyID().IsInvalid())
			return;
		ioWriter.Begin(ESimObjectField::CharacterBody, inHandle, 0);
		ioWriter.RVec3(inCharacter.GetPosition());
		ioWriter.Quat(inCharacter.GetRotation());
		ioWriter.RVec3(inCharacter.GetCenterOfMassPosition());
		ioWriter.Vec3(inCharacter.GetLinearVelocity());
		ioWriter.RMat44(inCharacter.GetWorldTransform());
		ioWriter.End();

		WriteTransformedShape(ioWriter, ESimObjectField::CharacterTransformedShape, inHandle, inCharacter.GetTransformedShape());
	}

	void						WriteCharacterVirtualState(SimObjectStateWriter &ioWriter, uint32 inHandle, const CharacterVirtual &inCharacter) const
	{
		WriteSaveState(ioWriter, ESimObjectField::CVSaveState, inHandle, inCharacter);

		ioWriter.Begin(ESimObjectField::CVProperties, inHandle, 0);
		ioWriter.U32(inCharacter.GetID().GetValue());
		ioWriter.U64(inCharacter.GetUserData());
		ioWriter.U32(inCharacter.GetInnerBodyID().GetIndexAndSequenceNumber());
		ioWriter.U32(GetShapeIndex(inCharacter.GetShape()));
		ioWriter.F32(inCharacter.GetMass());
		ioWriter.F32(inCharacter.GetMaxStrength());
		ioWriter.F32(inCharacter.GetPenetrationRecoverySpeed());
		ioWriter.Bool(inCharacter.GetEnhancedInternalEdgeRemoval());
		ioWriter.F32(inCharacter.GetCharacterPadding());
		ioWriter.U32(inCharacter.GetMaxNumHits());
		ioWriter.F32(inCharacter.GetHitReductionCosMaxAngle());
		ioWriter.Bool(inCharacter.GetMaxHitsExceeded());
		ioWriter.Vec3(inCharacter.GetShapeOffset());
		ioWriter.End();

		ioWriter.Begin(ESimObjectField::CVBase, inHandle, 0);
		ioWriter.Vec3(inCharacter.GetUp());
		ioWriter.F32(inCharacter.GetCosMaxSlopeAngle());
		ioWriter.Plane(inCharacter.GetSupportingVolume());
		ioWriter.U32(SimMaterialHash(inCharacter.GetGroundMaterial()));
		ioWriter.U64(inCharacter.GetGroundUserData());
		ioWriter.Bool(inCharacter.IsSupported());
		ioWriter.Bool(inCharacter.CanWalkStairs(inCharacter.GetLinearVelocity()));
		ioWriter.End();

		ioWriter.Begin(ESimObjectField::CVTransforms, inHandle, 0);
		ioWriter.RVec3(inCharacter.GetCenterOfMassPosition());
		ioWriter.RMat44(inCharacter.GetWorldTransform());
		ioWriter.RMat44(inCharacter.GetCenterOfMassTransform());
		ioWriter.End();

		WriteTransformedShape(ioWriter, ESimObjectField::CVTransformedShape, inHandle, inCharacter.GetTransformedShape());

		const CharacterVirtual::ContactList &contacts = inCharacter.GetActiveContacts();
		ioWriter.Begin(ESimObjectField::CVNumActiveContacts, inHandle, 0);
		ioWriter.U32(uint32(contacts.size()));
		ioWriter.End();
		for (uint32 c = 0; c < uint32(contacts.size()); ++c)
		{
			const CharacterContact &contact = contacts[c];
			ioWriter.Begin(ESimObjectField::CVActiveContact, inHandle, c);
			ioWriter.U32(contact.mBodyB.GetIndexAndSequenceNumber());
			ioWriter.U32(contact.mCharacterIDB.GetValue());
			ioWriter.U32(contact.mSubShapeIDB.GetValue());
			ioWriter.RVec3(contact.mPosition);
			ioWriter.Vec3(contact.mLinearVelocity);
			ioWriter.Vec3(contact.mContactNormal);
			ioWriter.Vec3(contact.mSurfaceNormal);
			ioWriter.F32(contact.mDistance);
			ioWriter.F32(contact.mFraction);
			ioWriter.U32(uint32(contact.mMotionTypeB));
			ioWriter.Bool(contact.mIsSensorB);
			ioWriter.Bool(contact.mCharacterB != nullptr);
			ioWriter.U64(contact.mUserData);
			ioWriter.U32(SimMaterialHash(contact.mMaterial));
			ioWriter.Bool(contact.mHadCollision);
			ioWriter.Bool(contact.mWasDiscarded);
			ioWriter.Bool(contact.mCanPushCharacter);
			ioWriter.Bool(contact.mIsBackFacingContact);
			ioWriter.End();
		}
	}

	void						WriteRagdollState(SimObjectStateWriter &ioWriter, uint32 inHandle, Ragdoll &ioRagdoll) const
	{
		uint32 num_bodies = uint32(ioRagdoll.GetBodyCount());
		ioWriter.Begin(ESimObjectField::RagdollProperties, inHandle, 0);
		ioWriter.U32(num_bodies);
		ioWriter.U32(uint32(ioRagdoll.GetConstraintCount()));
		ioWriter.Bool(ioRagdoll.IsActive());
		ioWriter.End();

		for (uint32 b = 0; b < num_bodies; ++b)
		{
			ioWriter.Begin(ESimObjectField::RagdollBody, inHandle, b);
			ioWriter.U32(ioRagdoll.GetBodyID(int(b)).GetIndexAndSequenceNumber());
			ioWriter.End();
		}

		RVec3 position;
		Quat rotation;
		ioRagdoll.GetRootTransform(position, rotation);
		ioWriter.Begin(ESimObjectField::RagdollRootTransform, inHandle, 0);
		ioWriter.RVec3(position);
		ioWriter.Quat(rotation);
		ioWriter.End();

		RVec3 root_offset;
		Array<Mat44> joints(num_bodies);
		ioRagdoll.GetPose(root_offset, joints.data());
		ioWriter.Begin(ESimObjectField::RagdollPoseRootOffset, inHandle, 0);
		ioWriter.RVec3(root_offset);
		ioWriter.End();
		for (uint32 b = 0; b < num_bodies; ++b)
		{
			ioWriter.Begin(ESimObjectField::RagdollPoseJoint, inHandle, b);
			ioWriter.Mat44(joints[b]);
			ioWriter.End();
		}

		AABox bounds = ioRagdoll.GetWorldSpaceBounds();
		ioWriter.Begin(ESimObjectField::RagdollBounds, inHandle, 0);
		ioWriter.Vec3(bounds.mMin);
		ioWriter.Vec3(bounds.mMax);
		ioWriter.End();

		for (uint32 c = 0; c < uint32(ioRagdoll.GetConstraintCount()); ++c)
		{
			const TwoBodyConstraint *constraint = ioRagdoll.GetConstraint(int(c));
			ioWriter.Begin(ESimObjectField::RagdollConstraint, inHandle, c);
			ioWriter.U32(uint32(constraint->GetSubType()));
			ioWriter.U32(constraint->GetConstraintPriority());
			ioWriter.Bool(constraint->GetEnabled());
			ioWriter.Bool(constraint->IsActive());
			ioWriter.U32(constraint->GetBody1()->GetID().GetIndexAndSequenceNumber());
			ioWriter.U32(constraint->GetBody2()->GetID().GetIndexAndSequenceNumber());
			ioWriter.End();
		}
	}

	// The engine of a vehicle (VehicleSlot.writeEngineState of SimulationParity.zig, also for the tracked controller)
	static void					WriteEngineState(SimObjectStateWriter &ioWriter, uint32 inHandle, const VehicleEngine &inEngine)
	{
		ioWriter.Begin(ESimObjectField::VehicleEngine, inHandle, 0);
		ioWriter.F32(inEngine.GetCurrentRPM());
		ioWriter.F32(inEngine.GetAngularVelocity());
		ioWriter.F32(inEngine.GetTorque(1.0f));
		ioWriter.Bool(inEngine.AllowSleep());
		ioWriter.F32(inEngine.mMaxTorque);
		ioWriter.F32(inEngine.mMinRPM);
		ioWriter.F32(inEngine.mMaxRPM);
		ioWriter.F32(inEngine.mInertia);
		ioWriter.F32(inEngine.mAngularDamping);
		ioWriter.End();
	}

	// The transmission of a vehicle (VehicleSlot.writeTransmissionState, also for the tracked controller)
	static void					WriteTransmissionState(SimObjectStateWriter &ioWriter, uint32 inHandle, const VehicleTransmission &inTransmission)
	{
		ioWriter.Begin(ESimObjectField::VehicleTransmission, inHandle, 0);
		ioWriter.I32(inTransmission.GetCurrentGear());
		ioWriter.F32(inTransmission.GetClutchFriction());
		ioWriter.Bool(inTransmission.IsSwitchingGear());
		ioWriter.F32(inTransmission.GetCurrentRatio());
		ioWriter.Bool(inTransmission.AllowSleep());
		ioWriter.U32(uint32(inTransmission.mMode));
		ioWriter.End();
	}

	// The tracked part of the object state of a wheel (TrackedVehicleSlot.writeWheelState)
	static void					WriteTrackedWheelState(SimObjectStateWriter &ioWriter, uint32 inHandle, uint32 inIndex, const Wheel &inWheel)
	{
		const WheelTV &tv = static_cast<const WheelTV &>(inWheel);
		ioWriter.Begin(ESimObjectField::VehicleWheelTV, inHandle, inIndex);
		ioWriter.I32(tv.mTrackIndex);
		ioWriter.F32(tv.mCombinedLongitudinalFriction);
		ioWriter.F32(tv.mCombinedLateralFriction);
		ioWriter.F32(tv.mBrakeImpulse);
		ioWriter.End();
	}

	// The tracked controller part of the object state (TrackedVehicleSlot.writeControllerState): the engine and the
	// transmission (VehicleSlot's functions), the inputs, the tracks
	static void					WriteTrackedControllerState(SimObjectStateWriter &ioWriter, uint32 inHandle, const VehicleConstraint &inVehicle)
	{
		const TrackedVehicleController *tracked = static_cast<const TrackedVehicleController *>(inVehicle.GetController());
		WriteEngineState(ioWriter, inHandle, tracked->GetEngine());
		WriteTransmissionState(ioWriter, inHandle, tracked->GetTransmission());
		ioWriter.Begin(ESimObjectField::VehicleTracked, inHandle, 0);
		ioWriter.F32(tracked->GetForwardInput());
		ioWriter.F32(tracked->GetLeftRatio());
		ioWriter.F32(tracked->GetRightRatio());
		ioWriter.F32(tracked->GetBrakeInput());
		ioWriter.End();
		for (uint32 t = 0; t < uint32(ETrackSide::Num); ++t)
		{
			const VehicleTrack &track = tracked->GetTracks()[t];
			ioWriter.Begin(ESimObjectField::VehicleTrack, inHandle, t);
			ioWriter.F32(track.mAngularVelocity);
			ioWriter.U32(track.mDrivenWheel);
			ioWriter.U32(uint32(track.mWheels.size()));
			ioWriter.F32(track.mInertia);
			ioWriter.F32(track.mAngularDamping);
			ioWriter.F32(track.mMaxBrakeTorque);
			ioWriter.F32(track.mDifferentialRatio);
			ioWriter.End();
		}
	}

	// The motorcycle part of the object state (MotorcycleSlot.writeControllerState, after the wheeled fields)
	static void					WriteMotorcycleControllerState(SimObjectStateWriter &ioWriter, uint32 inHandle, const VehicleConstraint &inVehicle)
	{
		const MotorcycleController *motorcycle = static_cast<const MotorcycleController *>(inVehicle.GetController());
		ioWriter.Begin(ESimObjectField::VehicleMotorcycle, inHandle, 0);
		ioWriter.F32(motorcycle->GetWheelBase());
		ioWriter.Bool(motorcycle->IsLeanControllerEnabled());
		ioWriter.Bool(motorcycle->IsLeanSteeringLimitEnabled());
		ioWriter.F32(motorcycle->GetLeanSpringConstant());
		ioWriter.F32(motorcycle->GetLeanSpringDamping());
		ioWriter.F32(motorcycle->GetLeanSpringIntegrationCoefficient());
		ioWriter.F32(motorcycle->GetLeanSpringIntegrationCoefficientDecay());
		ioWriter.F32(motorcycle->GetLeanSmoothingFactor());
		ioWriter.End();
	}

	// The object state of a vehicle (VehicleSlot.writeState; the parts of the tracked and motorcycle controllers through
	// the functions of the vehicle variants above)
	void						WriteVehicleState(SimObjectStateWriter &ioWriter, SimVehicleData &ioVehicle) const
	{
		uint32 h = ioVehicle.mHandle;
		const VehicleConstraint &c = *ioVehicle.mConstraint;
		uint32 num_wheels = uint32(c.GetWheels().size());

		ioWriter.Begin(ESimObjectField::VehicleProperties, h, 0);
		ioWriter.Bool(c.IsActive());
		ioWriter.Bool(c.GetEnabled());
		ioWriter.U32(c.GetNumStepsBetweenCollisionTestActive());
		ioWriter.U32(c.GetNumStepsBetweenCollisionTestInactive());
		ioWriter.F32(c.GetMaxPitchRollAngle());
		ioWriter.Bool(c.IsGravityOverridden());
		ioWriter.Vec3(c.GetGravityOverride());
		ioWriter.Vec3(c.GetLocalForward());
		ioWriter.Vec3(c.GetLocalUp());
		ioWriter.Vec3(c.GetWorldUp());
		ioWriter.U32(num_wheels);
		ioWriter.U32(c.GetVehicleBody()->GetID().GetIndexAndSequenceNumber());
		ioWriter.U32(c.GetConstraintPriority());
		ioWriter.U32(c.GetNumVelocityStepsOverride());
		ioWriter.U32(c.GetNumPositionStepsOverride());
		ioWriter.U64(c.GetUserData());
		ioWriter.End();

		for (uint32 w = 0; w < num_wheels; ++w)
		{
			const Wheel *wheel = c.GetWheel(w);
			ioWriter.Begin(ESimObjectField::VehicleWheel, h, w);
			ioWriter.F32(wheel->GetAngularVelocity());
			ioWriter.F32(wheel->GetRotationAngle());
			ioWriter.F32(wheel->GetSteerAngle());
			ioWriter.Bool(wheel->HasContact());
			ioWriter.U32(wheel->GetContactBodyID().GetIndexAndSequenceNumber());
			ioWriter.U32(wheel->GetContactSubShapeID().GetValue());
			ioWriter.F32(wheel->GetSuspensionLength());
			ioWriter.Bool(wheel->HasHitHardPoint());
			ioWriter.F32(wheel->GetSuspensionLambda());
			ioWriter.F32(wheel->GetLongitudinalLambda());
			ioWriter.F32(wheel->GetLateralLambda());
			ioWriter.End();

			if (wheel->HasContact())
			{
				ioWriter.Begin(ESimObjectField::VehicleWheelContact, h, w);
				ioWriter.RVec3(wheel->GetContactPosition());
				ioWriter.Vec3(wheel->GetContactPointVelocity());
				ioWriter.Vec3(wheel->GetContactNormal());
				ioWriter.Vec3(wheel->GetContactLongitudinal());
				ioWriter.Vec3(wheel->GetContactLateral());
				ioWriter.End();
			}

			Vec3 forward, up, right;
			c.GetWheelLocalBasis(wheel, forward, up, right);
			ioWriter.Begin(ESimObjectField::VehicleWheelTransform, h, w);
			ioWriter.Vec3(forward);
			ioWriter.Vec3(up);
			ioWriter.Vec3(right);
			ioWriter.Mat44(c.GetWheelLocalTransform(w, Vec3::sAxisY(), Vec3::sAxisX()));
			ioWriter.RMat44(c.GetWheelWorldTransform(w, Vec3::sAxisY(), Vec3::sAxisX()));
			ioWriter.End();

			if (ioVehicle.mController == 1)
				WriteTrackedWheelState(ioWriter, h, w, *wheel);
			else
			{
				const WheelWV *wv = static_cast<const WheelWV *>(wheel);
				ioWriter.Begin(ESimObjectField::VehicleWheelWV, h, w);
				ioWriter.F32(wv->mLongitudinalSlip);
				ioWriter.F32(wv->mLateralSlip);
				ioWriter.F32(wv->mCombinedLongitudinalFriction);
				ioWriter.F32(wv->mCombinedLateralFriction);
				ioWriter.F32(wv->mBrakeImpulse);
				ioWriter.End();
			}
		}

		const VehicleAntiRollBars &bars = c.GetAntiRollBars();
		for (uint32 b = 0; b < uint32(bars.size()); ++b)
		{
			ioWriter.Begin(ESimObjectField::VehicleAntiRollBar, h, b);
			ioWriter.I32(bars[b].mLeftWheel);
			ioWriter.I32(bars[b].mRightWheel);
			ioWriter.F32(bars[b].mStiffness);
			ioWriter.End();
		}

		if (ioVehicle.mController == 1)
		{
			WriteTrackedControllerState(ioWriter, h, c);
			return;
		}

		const WheeledVehicleController *wheeled = static_cast<const WheeledVehicleController *>(c.GetController());
		WriteEngineState(ioWriter, h, wheeled->GetEngine());
		WriteTransmissionState(ioWriter, h, wheeled->GetTransmission());
		ioWriter.Begin(ESimObjectField::VehicleWheeled, h, 0);
		ioWriter.F32(wheeled->GetForwardInput());
		ioWriter.F32(wheeled->GetRightInput());
		ioWriter.F32(wheeled->GetBrakeInput());
		ioWriter.F32(wheeled->GetHandBrakeInput());
		ioWriter.F32(wheeled->GetDifferentialLimitedSlipRatio());
		ioWriter.F32(wheeled->GetWheelSpeedAtClutch());
		ioWriter.End();

		const WheeledVehicleController::Differentials &differentials = wheeled->GetDifferentials();
		for (uint32 d = 0; d < uint32(differentials.size()); ++d)
		{
			const VehicleDifferentialSettings &diff = differentials[d];
			float left_angular_velocity = diff.mLeftWheel != -1? c.GetWheel(diff.mLeftWheel)->GetAngularVelocity() : 0.0f;
			float right_angular_velocity = diff.mRightWheel != -1? c.GetWheel(diff.mRightWheel)->GetAngularVelocity() : 0.0f;
			float left_fraction, right_fraction;
			diff.CalculateTorqueRatio(left_angular_velocity, right_angular_velocity, left_fraction, right_fraction);
			ioWriter.Begin(ESimObjectField::VehicleDifferential, h, d);
			ioWriter.I32(diff.mLeftWheel);
			ioWriter.I32(diff.mRightWheel);
			ioWriter.F32(diff.mDifferentialRatio);
			ioWriter.F32(diff.mLeftRightSplit);
			ioWriter.F32(diff.mLimitedSlipRatio);
			ioWriter.F32(diff.mEngineTorqueRatio);
			ioWriter.F32(left_fraction);
			ioWriter.F32(right_fraction);
			ioWriter.End();
		}

		if (ioVehicle.mController == 2)
			WriteMotorcycleControllerState(ioWriter, h, c);
	}

	// The SaveState of every character and character virtual in handle order (part of snapshot / restore_cross: the
	// PhysicsSystem state does not hold them)
	void						SaveCharacterStates(StateRecorder &ioStream) const
	{
		for (const Ref<Character> &c : mCharacters)
			if (c != nullptr)
				c->SaveState(ioStream);
		for (const Ref<CharacterVirtual> &c : mCharacterVirtuals)
			if (c != nullptr)
				c->SaveState(ioStream);
	}

	// True when a character or character virtual exists (that has a state)
	bool						HasCharacters() const
	{
		for (const Ref<Character> &c : mCharacters)
			if (c != nullptr)
				return true;
		for (const Ref<CharacterVirtual> &c : mCharacterVirtuals)
			if (c != nullptr)
				return true;
		return false;
	}

	void						RestoreCharacterStates(StateRecorder &ioStream)
	{
		for (Ref<Character> &c : mCharacters)
			if (c != nullptr)
				c->RestoreState(ioStream);
		for (Ref<CharacterVirtual> &c : mCharacterVirtuals)
			if (c != nullptr)
				c->RestoreState(ioStream);
	}

	// -------------------------------------------------------------------------------------------------------------------
	// Phase 9: soft bodies

	// MakeSoftBodyCreationSettings of SimulationParity.zig
	SoftBodyCreationSettings	MakeSoftBodyCreationSettings(const SimSoftBody &inSoftBody) const
	{
		SoftBodyCreationSettings s(mSoftBodySettings[inSoftBody.mSettings], SimLoadR3(inSoftBody.mPosition), SimLoadQuat(inSoftBody.mRotation), ObjectLayer(inSoftBody.mObjectLayer));
		s.mUserData = inSoftBody.mUserData;
		s.mCollisionGroup = CollisionGroup(nullptr, inSoftBody.mCollisionGroupID, inSoftBody.mCollisionSubGroupID);
		s.mNumIterations = inSoftBody.mNumIterations;
		s.mLinearDamping = inSoftBody.mLinearDamping;
		s.mMaxLinearVelocity = inSoftBody.mMaxLinearVelocity;
		s.mRestitution = inSoftBody.mRestitution;
		s.mFriction = inSoftBody.mFriction;
		s.mPressure = inSoftBody.mPressure;
		s.mGravityFactor = inSoftBody.mGravityFactor;
		s.mVertexRadius = inSoftBody.mVertexRadius;
		s.mUpdatePosition = inSoftBody.mUpdatePosition != 0;
		s.mMakeRotationIdentity = inSoftBody.mMakeRotationIdentity != 0;
		s.mAllowSleeping = inSoftBody.mAllowSleeping != 0;
		s.mFacesDoubleSided = inSoftBody.mFacesDoubleSided != 0;
		return s;
	}

	// The result of soft_body_creation_settings_hash (OpKind of SimScenes.zig): every member of the settings, the index of
	// the shared settings among the world's settings objects and their SaveBinaryState bytes
	uint32						HashSoftBodyCreationSettings(const SoftBodyCreationSettings &inSettings) const
	{
		SimHasher h;
		h.RVec3(inSettings.mPosition);
		h.Quat(inSettings.mRotation);
		h.Value(inSettings.mUserData);
		h.U32(uint32(inSettings.mObjectLayer));
		h.U32(inSettings.mCollisionGroup.GetGroupID());
		h.U32(inSettings.mCollisionGroup.GetSubGroupID());
		h.Bool(inSettings.mCollisionGroup.GetGroupFilter() != nullptr);
		h.U32(inSettings.mNumIterations);
		h.F32(inSettings.mLinearDamping);
		h.F32(inSettings.mMaxLinearVelocity);
		h.F32(inSettings.mRestitution);
		h.F32(inSettings.mFriction);
		h.F32(inSettings.mPressure);
		h.F32(inSettings.mGravityFactor);
		h.F32(inSettings.mVertexRadius);
		h.Bool(inSettings.mUpdatePosition);
		h.Bool(inSettings.mMakeRotationIdentity);
		h.Bool(inSettings.mAllowSleeping);
		h.Bool(inSettings.mFacesDoubleSided);
		uint32 index = cInvalidHandle;
		for (uint32 i = 0; i < uint32(mSoftBodySettings.size()) && index == cInvalidHandle; ++i)
			if (mSoftBodySettings[i] == inSettings.mSettings)
				index = i;
		h.U32(index);
		std::string bytes = SimSaveSoftBodySettings(*inSettings.mSettings);
		h.Bytes(bytes.data(), bytes.size());
		return h.mHash;
	}

	// A soft body operation (OpKind 53..69 in SimScenes.zig)
	uint32						ApplySoftBodyOp(const SimOp &inOp)
	{
		BodyInterface &bi = mSystem.GetBodyInterface();
		EActivation activation = inOp.mActivation == 0? EActivation::Activate : EActivation::DontActivate;
		const uint32 *i = inOp.mI;
		const float *f = inOp.mF;

		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::CreateSoftBody:
		case ESimOp::CreateAndAddSoftBody:
		case ESimOp::CreateSoftBodyNoAdd:
			{
				SoftBodyCreationSettings settings = MakeSoftBodyCreationSettings(mDesc.mSoftBodies[inOp.mIndex]);
				BodyID new_id;
				if (ESimOp(inOp.mKind) == ESimOp::CreateAndAddSoftBody)
					new_id = bi.CreateAndAddSoftBody(settings, activation);
				else
				{
					Body *body = bi.CreateSoftBody(settings);
					if (body != nullptr)
					{
						new_id = body->GetID();
						if (ESimOp(inOp.mKind) == ESimOp::CreateSoftBody)
							bi.AddBody(new_id, activation);
					}
				}
				mHandles.push_back(new_id);
				return new_id.GetIndexAndSequenceNumber();
			}

		case ESimOp::SetSoftBodyContactListener:
			mSoftBodyListener.mMode = inOp.mIndex;
			mSystem.SetSoftBodyContactListener(inOp.mIndex == 0? nullptr : &mSoftBodyListener);
			return 0;

		default:
			break;
		}

		// The soft body of the body handle. The operations run between the updates: the no-lock interface (CustomUpdate
		// locks the bodies it collides with).
		Body *body = mSystem.GetBodyLockInterfaceNoLock().TryGetBody(GetHandle(inOp.mBody));
		if (body == nullptr || !body->IsSoftBody())
		{
			Trace("SimulationReference: soft body operation %u on body handle %u, which is not a soft body", inOp.mKind, inOp.mBody);
			abort();
		}
		SoftBodyMotionProperties *mp = static_cast<SoftBodyMotionProperties *>(body->GetMotionProperties());
		switch (ESimOp(inOp.mKind))
		{
		case ESimOp::SoftBodySetVertexPosition:					mp->GetVertex(i[0]).mPosition = SimLoad3(f); break;
		case ESimOp::SoftBodySetVertexVelocity:					mp->GetVertex(i[0]).mVelocity = SimLoad3(f); break;
		case ESimOp::SoftBodySetVertexInvMass:					mp->GetVertex(i[0]).mInvMass = f[0]; break;
		case ESimOp::SoftBodySetPressure:						mp->SetPressure(f[0]); break;
		case ESimOp::SoftBodySetNumIterations:					mp->SetNumIterations(i[0]); break;
		case ESimOp::SoftBodySetUpdatePosition:					mp->SetUpdatePosition(i[0] != 0); break;
		case ESimOp::SoftBodySetFacesDoubleSided:				mp->SetFacesDoubleSided(i[0] != 0); break;
		case ESimOp::SoftBodySetVertexRadius:					mp->SetVertexRadius(f[0]); break;
		case ESimOp::SoftBodySetEnableSkinConstraints:			mp->SetEnableSkinConstraints(i[0] != 0); break;
		case ESimOp::SoftBodySetSkinnedMaxDistanceMultiplier:	mp->SetSkinnedMaxDistanceMultiplier(f[0]); break;

		case ESimOp::SoftBodySkinVertices:
			{
				Array<Mat44> joints;
				for (uint32 j = 0; j < i[1]; ++j)
				{
					const float *m = mDesc.mFloats + i[0] + 16 * j;
					joints.push_back(Mat44(Vec4(m[0], m[1], m[2], m[3]), Vec4(m[4], m[5], m[6], m[7]), Vec4(m[8], m[9], m[10], m[11]), Vec4(m[12], m[13], m[14], m[15])));
				}
				mp->SkinVertices(body->GetCenterOfMassTransform(), joints.data(), uint(joints.size()), i[2] != 0, mTempAllocator);
				break;
			}

		case ESimOp::SoftBodyCustomUpdate:						mp->CustomUpdate(f[0], *body, mSystem); break;
		case ESimOp::SoftBodyCreationSettingsHash:				return HashSoftBodyCreationSettings(body->GetSoftBodyCreationSettings());

		default:
			Trace("SimulationReference: unknown soft body operation %u", inOp.mKind);
			abort();
		}
		return SimHashSoftBody(*body);
	}

	// Apply an operation, see OpKind in SimScenes.zig
	uint32						ApplyOp(const SimOp &inOp)
	{
		BodyInterface &bi = mSystem.GetBodyInterface();
		BodyID id = GetHandle(inOp.mBody);
		EActivation activation = inOp.mActivation == 0? EActivation::Activate : EActivation::DontActivate;
		const float *f = inOp.mF;
		RVec3 r = SimLoadR3(inOp.mR);

		if (inOp.mKind >= cFirstConstraintOp && inOp.mKind < cEndConstraintOp)
			return ApplyConstraintOp(inOp);
		if (inOp.mKind >= cFirstPhase7Op && inOp.mKind < cEndPhase7Op)
			return ApplyObjectOp(inOp);
		if (inOp.mKind >= cFirstSoftBodyOp && inOp.mKind < cEndSoftBodyOp)
			return ApplySoftBodyOp(inOp);

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

		case ESimOp::WereBodiesInContact:
			{
				BodyID id2 = GetHandle(inOp.mIndex);
				return (mSystem.WereBodiesInContact(id, id2)? 1 : 0) | (mSystem.WereBodiesInContact(id2, id)? 2 : 0);
			}

		case ESimOp::SetCollideKinematicVsNonDynamic:
			{
				BodyLockWrite lock(mSystem.GetBodyLockInterface(), id);
				if (lock.Succeeded())
					lock.GetBody().SetCollideKinematicVsNonDynamic(inOp.mCount != 0);
			}
			break;

		case ESimOp::CreateConstraint:
			mSystem.AddConstraint(CreateConstraint(inOp.mIndex));
			return GetNumConstraints();

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
	TempAllocatorImpl			mTempAllocator;
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
	Array<Ref<TwoBodyConstraint>> mConstraintHandles;			// The constraints of the constraint handles (also the ones not in the system)

	// Phase 7 (declared after mSystem: the destructor releases them while the system exists)
	SimRecorder					mCharacterRecorder;							// The CharacterContactListener records (compared in call order)
	SimCharacterContactListener	mCharacterRecordListener;					// CharacterListenerMode.record
	SimCharacterContactListener	mCharacterModifyListener;					// CharacterListenerMode.modify
	CharacterVsCharacterCollisionSimple mCharacterGroups[cNumCharacterCollisionGroups];
	Array<Ref<Character>>		mCharacters;								// The characters of the character handles (nullptr: destroyed)
	Array<Ref<CharacterVirtual>> mCharacterVirtuals;						// The character virtuals of the handles (nullptr: destroyed)
	Array<Ref<RagdollSettings>>	mRagdollSettings;							// Per ragdoll description, built at its first create_ragdoll
	Array<Ref<Ragdoll>>			mRagdolls;									// The ragdolls of the ragdoll handles (nullptr: destroyed or out of bodies)
	Array<SimVehicleData *>		mVehicles;									// The vehicles of the vehicle handles

	// Phase 9
	Array<Ref<SoftBodySharedSettings>> mSoftBodySettings;					// The shared settings of the SimSoftBodySettings
	SimSoftBodyContactListener	mSoftBodyListener;							// Installed by set_soft_body_contact_listener
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

// The binary state round trips of the Phase 7 settings (the foundation review M3)

// The result of a C function that has no bytes for its arguments
static constexpr uint32 cNoBytes = 0xffffffff;

// Must match VehicleSettingsPart in SimScenes.zig
enum class ESimVehicleSettingsPart : uint32
{
	Settings = 0,
	SettingsWithoutWheels = 1,
	Controller = 2,
	Wheel = 3,
	Engine = 4,
	Transmission = 5,
	Track = 6,
	Differential = 7,
	AntiRollBar = 8,
};

// The SaveBinaryState bytes of an object
template <class T>
static std::string SimSaveBinaryState(const T &inObject)
{
	std::stringstream data;
	StreamOutWrapper stream(data);
	inObject.SaveBinaryState(stream);
	return data.str();
}

// True when a stream was read without error and completely (the stream sets eof only on a read past the end: one more
// byte is read, which must fail at the end)
static bool SimReadCompletely(StreamIn &ioStream)
{
	bool read = !ioStream.IsFailed();
	uint8 extra;
	ioStream.Read(extra);
	return read && ioStream.IsEOF();
}

// RestoreBinaryState of an object from bytes, true when every byte was read
template <class T>
static bool SimRestoreBinaryState(T &ioObject, const uint8 *inBytes, uint32 inSize)
{
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper stream(data);
	ioObject.RestoreBinaryState(stream);
	return SimReadCompletely(stream);
}

// Copy bytes to a caller provided buffer when they fit, returns the size
static uint32 SimCopyBytes(const std::string &inData, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inData.size());
	if (size <= inCapacity && size > 0)
		memcpy(outBytes, inData.data(), size);
	return size;
}

// The bytes of a part of vehicle description inVehicle (VehicleSettingsPart, item inItem): without inRestore the
// SaveBinaryState bytes of the part as SimMakeVehicleSettings makes it, with inRestore a default object of the part's
// class restores inBytes (every byte read) and is saved again. False: the part does not exist (an item out of range,
// a part of another controller kind, the restore of Settings: Jolt's VehicleConstraintSettings::RestoreBinaryState
// dereferences null wheel settings, CharacterVehicleArchitecture E6) or the restore failed.
static bool SimVehicleSettingsPart(const SimSceneDesc &inDesc, const SimVehicle &inVehicle, uint32 inPart, uint32 inItem, bool inRestore, const uint8 *inBytes, uint32 inSize, std::string &outData)
{
	switch (ESimVehicleSettingsPart(inPart))
	{
	case ESimVehicleSettingsPart::Settings:
		if (inItem != 0 || inRestore)
			return false;
		outData = SimSaveBinaryState(*SimMakeVehicleSettings(inDesc, inVehicle));
		return true;

	case ESimVehicleSettingsPart::SettingsWithoutWheels:
		if (inItem != 0)
			return false;
		if (inRestore)
		{
			// The RTTI hash, then VehicleConstraintSettings::RestoreBinaryState (the controller from its hash)
			std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
			StreamInWrapper stream(data);
			ConstraintSettings::ConstraintResult result = ConstraintSettings::sRestoreFromBinaryState(stream);
			if (result.HasError() || !SimReadCompletely(stream))
				return false;
			outData = SimSaveBinaryState(*result.Get());
		}
		else
		{
			Ref<VehicleConstraintSettings> s = SimMakeVehicleSettings(inDesc, inVehicle);
			s->mWheels.clear();
			outData = SimSaveBinaryState(*s);
		}
		return true;

	case ESimVehicleSettingsPart::Controller:
		{
			if (inItem != 0)
				return false;
			Ref<VehicleControllerSettings> s;
			if (!inRestore)
				s = SimMakeControllerSettings(inDesc, inVehicle);
			else
				switch (inVehicle.mController)
				{
				case 0:		s = new WheeledVehicleControllerSettings; break;
				case 1:		s = new TrackedVehicleControllerSettings; break;
				default:	s = new MotorcycleControllerSettings; break;
				}
			if (inRestore && !SimRestoreBinaryState(*s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(*s);
			return true;
		}

	case ESimVehicleSettingsPart::Wheel:
		{
			if (inItem >= inVehicle.mNumWheels)
				return false;
			const SimVehicleWheel &w = inDesc.mVehicleWheels[inVehicle.mFirstWheel + inItem];
			Ref<WheelSettings> s;
			if (!inRestore)
				s = SimMakeWheelSettings(inDesc, w);
			else if (w.mKind == 0)
				s = new WheelSettingsWV;
			else
				s = new WheelSettingsTV;
			if (inRestore && !SimRestoreBinaryState(*s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(*s);
			return true;
		}

	case ESimVehicleSettingsPart::Engine:
		{
			if (inItem != 0)
				return false;
			VehicleEngineSettings s;
			if (!inRestore)
				SimFillEngine(inDesc, inVehicle.mEngine, s);
			else if (!SimRestoreBinaryState(s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(s);
			return true;
		}

	case ESimVehicleSettingsPart::Transmission:
		{
			if (inItem != 0)
				return false;
			VehicleTransmissionSettings s;
			if (!inRestore)
				SimFillTransmission(inDesc, inVehicle.mTransmission, s);
			else if (!SimRestoreBinaryState(s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(s);
			return true;
		}

	case ESimVehicleSettingsPart::Track:
		{
			if (inVehicle.mController != 1 || inItem >= 2)
				return false;
			VehicleTrackSettings s;
			s.mDrivenWheel = 0; // Uninitialized in Jolt
			if (!inRestore)
				SimFillTrack(inDesc, inVehicle.mTracks[inItem], s);
			else if (!SimRestoreBinaryState(s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(s);
			return true;
		}

	case ESimVehicleSettingsPart::Differential:
		{
			if (inVehicle.mController == 1 || inItem >= inVehicle.mNumDifferentials)
				return false;
			VehicleDifferentialSettings s;
			if (!inRestore)
				SimFillDifferential(inDesc.mVehicleDifferentials[inVehicle.mFirstDifferential + inItem], s);
			else if (!SimRestoreBinaryState(s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(s);
			return true;
		}

	case ESimVehicleSettingsPart::AntiRollBar:
		{
			if (inItem >= inVehicle.mNumAntiRollBars)
				return false;
			VehicleAntiRollBar s;
			if (!inRestore)
				SimFillAntiRollBar(inDesc.mVehicleAntiRollBars[inVehicle.mFirstAntiRollBar + inItem], s);
			else if (!SimRestoreBinaryState(s, inBytes, inSize))
				return false;
			outData = SimSaveBinaryState(s);
			return true;
		}
	}
	return false;
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
	outSizes[12] = offsetof(SimOp, mI);
	outSizes[13] = offsetof(SimConstraint, mUserData);
	outSizes[14] = offsetof(SimConstraint, mR);
	outSizes[15] = offsetof(SimConstraint, mF);
}

// The SaveBinaryState bytes of the settings of a constraint description (SimMakeConstraintSettings), returns the size
// (the bytes are copied when they fit)
uint32 jolt_sim_constraint_settings_save(const SimConstraint *inConstraint, uint8 *outBytes, uint32 inCapacity)
{
	SimEnsureFactory();
	std::string data = SimSaveSettings(*SimMakeConstraintSettings(*inConstraint));
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
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

// PhysicsSystem::SaveState(EStateRecorderState(inState), SimStateFilter), returns the size (the bytes are copied when
// they fit)
uint32 jolt_sim_save_state_filtered(void *inWorld, uint32 inState, uint8 *outBytes, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	SimStateFilter filter;
	StateRecorderImpl recorder;
	world->mSystem.SaveState(recorder, EStateRecorderState(inState), &filter);
	std::string data = recorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// PhysicsSystem::RestoreState with SimStateFilter (not validating), returns its result
uint32 jolt_sim_restore_state_filtered(void *inWorld, const uint8 *inBytes, uint32 inSize)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	SimStateFilter filter;
	StateRecorderImpl recorder;
	recorder.WriteBytes(inBytes, inSize);
	recorder.Rewind();
	return world->mSystem.RestoreState(recorder, &filter)? 1 : 0;
}

// Returns the number of records; when they fit in inCapacity they are copied and removed
uint32 jolt_sim_take_records(void *inWorld, SimRecord *outRecords, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	std::lock_guard<Mutex> lock(world->mRecorder.mMutex);
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

// The SaveBinaryState bytes of the default constructed settings of ConstraintKind inKind (SimScenes.zig; a path gets an
// empty PathConstraintPathHermite: Jolt's default has no path and SaveBinaryState dereferences it), returns the size
uint32 jolt_sim_default_constraint_settings_save(uint32 inKind, uint8 *outBytes, uint32 inCapacity)
{
	SimEnsureFactory();
	Ref<ConstraintSettings> settings;
	switch (inKind)
	{
	case 0:		settings = new FixedConstraintSettings; break;
	case 1:		settings = new PointConstraintSettings; break;
	case 2:		settings = new DistanceConstraintSettings; break;
	case 3:		settings = new HingeConstraintSettings; break;
	case 4:		settings = new SliderConstraintSettings; break;
	case 5:		settings = new ConeConstraintSettings; break;
	case 6:		settings = new SwingTwistConstraintSettings; break;
	case 7:		settings = new SixDOFConstraintSettings; break;
	case 8:
		{
			PathConstraintSettings *s = new PathConstraintSettings;
			s->mPath = new PathConstraintPathHermite;
			settings = s;
			break;
		}
	case 9:		settings = new GearConstraintSettings; break;
	case 10:	settings = new RackAndPinionConstraintSettings; break;
	case 11:	settings = new PulleyConstraintSettings; break;
	default:	abort();
	}
	std::string data = SimSaveSettings(*settings);
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// The number of constraints of the system whose IsActive() is true (as DetermineActiveConstraints sees them)
uint32 jolt_sim_num_active_constraints(void *inWorld)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	uint32 num_active = 0;
	for (const Ref<Constraint> &c : world->mSystem.GetConstraints())
		if (c->IsActive())
			++num_active;
	return num_active;
}

// For the state decoder: the EConstraintSubType of constraint inIndex of the system (0xffffffff when there is none)
uint32 jolt_sim_constraint_sub_type(void *inWorld, uint32 inIndex)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	Constraints constraints = world->mSystem.GetConstraints();
	return inIndex < constraints.size()? uint32(constraints[inIndex]->GetSubType()) : 0xffffffff;
}

// Phase 7: the sizes and some offsets of the structs shared with SimScenes.zig
void jolt_sim_phase7_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(SimCharacterBase);
	outSizes[1] = sizeof(SimCharacter);
	outSizes[2] = sizeof(SimCharacterVirtual);
	outSizes[3] = sizeof(SimRagdoll);
	outSizes[4] = sizeof(SimRagdollJoint);
	outSizes[5] = sizeof(SimRagdollAdditionalConstraint);
	outSizes[6] = sizeof(SimRagdollPose);
	outSizes[7] = sizeof(SimVehicleEngine);
	outSizes[8] = sizeof(SimVehicleTransmission);
	outSizes[9] = sizeof(SimVehicleTrack);
	outSizes[10] = sizeof(SimVehicleDifferential);
	outSizes[11] = sizeof(SimVehicleAntiRollBar);
	outSizes[12] = sizeof(SimVehicleWheel);
	outSizes[13] = sizeof(SimVehicleTester);
	outSizes[14] = sizeof(SimVehicle);
	outSizes[15] = offsetof(SimCharacter, mPosition);
	outSizes[16] = offsetof(SimCharacter, mUserData);
	outSizes[17] = offsetof(SimCharacterVirtual, mPosition);
	outSizes[18] = offsetof(SimCharacterVirtual, mUserData);
	outSizes[19] = offsetof(SimCharacterVirtual, mCollisionGroup);
	outSizes[20] = offsetof(SimRagdoll, mUserData);
	outSizes[21] = offsetof(SimVehicle, mUserData);
	outSizes[22] = offsetof(SimVehicle, mEngine);
	outSizes[23] = offsetof(SimVehicle, mTracks);
	outSizes[24] = offsetof(SimVehicle, mTester);
	outSizes[25] = offsetof(SimVehicle, mTireMaxImpulse);
	outSizes[26] = offsetof(SimVehicleWheel, mLateralFrictionTV);
	outSizes[27] = offsetof(SimSceneDesc, mCharacters);
	outSizes[28] = offsetof(SimSceneDesc, mVehicles);
	outSizes[29] = offsetof(SimSceneDesc, mUints);
	outSizes[30] = sizeof(SimSceneDesc);
	outSizes[31] = offsetof(SimRagdollPose, mData);
}

// The object state (ObjectField of SimScenes.zig), returns the size (the bytes are copied when they fit)
uint32 jolt_sim_object_state(void *inWorld, uint8 *outBytes, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	SimObjectStateWriter writer;
	world->WriteObjectState(writer);
	uint32 size = uint32(writer.mBytes.size());
	if (size <= inCapacity && size > 0)
		memcpy(outBytes, writer.mBytes.data(), size);
	return size;
}

// Returns the number of character listener records; when they fit in inCapacity they are copied (in call order) and
// removed
uint32 jolt_sim_take_character_records(void *inWorld, SimRecord *outRecords, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	std::lock_guard<Mutex> lock(world->mCharacterRecorder.mMutex);
	uint32 n = uint32(world->mCharacterRecorder.mRecords.size());
	if (n <= inCapacity)
	{
		if (n > 0)
			memcpy(outRecords, world->mCharacterRecorder.mRecords.data(), n * sizeof(SimRecord));
		world->mCharacterRecorder.mRecords.clear();
	}
	return n;
}

// The SaveState bytes of every character and character virtual in handle order, returns the size (the bytes are copied
// when they fit)
uint32 jolt_sim_save_character_states(void *inWorld, uint8 *outBytes, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	StateRecorderImpl recorder;
	world->SaveCharacterStates(recorder);
	std::string data = recorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity && size > 0)
		memcpy(outBytes, data.data(), size);
	return size;
}

// RestoreState of every character and character virtual from the bytes of jolt_sim_save_character_states (not
// validating), returns 1 when the bytes were read without error and completely (the stream sets eof only on a read
// past the end: one more byte is read, which must fail at the end; ZoltWorld.restoreCharacterStates does the same)
uint32 jolt_sim_restore_character_states(void *inWorld, const uint8 *inBytes, uint32 inSize)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	if (inSize == 0)
		return world->HasCharacters()? 0 : 1; // An empty std::stringstream fails to rewind
	StateRecorderImpl recorder;
	recorder.WriteBytes(inBytes, inSize);
	recorder.Rewind();
	world->RestoreCharacterStates(recorder);
	bool read = !recorder.IsFailed();
	uint8 extra;
	recorder.Read(extra); // Fails at the end of the bytes
	return read && recorder.IsEOF()? 1 : 0;
}

// Jolt's CharacterSettings defaults as a SimCharacter (the constructor arguments of the harness: SimScenes' defaults)
void jolt_sim_default_character(SimCharacter *outCharacter)
{
	SimEnsureFactory();
	CharacterSettings s;
	SimCharacter &c = *outCharacter;
	s.mUp.StoreFloat3(reinterpret_cast<Float3 *>(c.mBase.mUp));
	s.mSupportingVolume.GetNormal().StoreFloat3(reinterpret_cast<Float3 *>(c.mBase.mSupportingVolume));
	c.mBase.mSupportingVolume[3] = s.mSupportingVolume.GetConstant();
	c.mBase.mMaxSlopeAngle = s.mMaxSlopeAngle;
	c.mBase.mEnhancedInternalEdgeRemoval = s.mEnhancedInternalEdgeRemoval? 1 : 0;
	c.mBase.mShape = s.mShape == nullptr? cInvalidHandle : 0;
	c.mLayer = s.mLayer;
	c.mMass = s.mMass;
	c.mFriction = s.mFriction;
	c.mGravityFactor = s.mGravityFactor;
	c.mAllowedDOFs = uint32(s.mAllowedDOFs);
	c.mPosition[0] = c.mPosition[1] = c.mPosition[2] = 0;
	c.mRotation[0] = c.mRotation[1] = c.mRotation[2] = 0;
	c.mRotation[3] = 1;
	c.mUserData = 0;
}

// Jolt's CharacterVirtualSettings defaults as a SimCharacterVirtual (mID and the harness members: SimScenes' defaults)
void jolt_sim_default_character_virtual(SimCharacterVirtual *outCharacter)
{
	SimEnsureFactory();
	CharacterVirtualSettings s;
	SimCharacterVirtual &c = *outCharacter;
	s.mUp.StoreFloat3(reinterpret_cast<Float3 *>(c.mBase.mUp));
	s.mSupportingVolume.GetNormal().StoreFloat3(reinterpret_cast<Float3 *>(c.mBase.mSupportingVolume));
	c.mBase.mSupportingVolume[3] = s.mSupportingVolume.GetConstant();
	c.mBase.mMaxSlopeAngle = s.mMaxSlopeAngle;
	c.mBase.mEnhancedInternalEdgeRemoval = s.mEnhancedInternalEdgeRemoval? 1 : 0;
	c.mBase.mShape = s.mShape == nullptr? cInvalidHandle : 0;
	c.mID = cInvalidHandle;
	c.mMass = s.mMass;
	c.mMaxStrength = s.mMaxStrength;
	s.mShapeOffset.StoreFloat3(reinterpret_cast<Float3 *>(c.mShapeOffset));
	c.mBackFaceMode = uint32(s.mBackFaceMode);
	c.mPredictiveContactDistance = s.mPredictiveContactDistance;
	c.mMaxCollisionIterations = s.mMaxCollisionIterations;
	c.mMaxConstraintIterations = s.mMaxConstraintIterations;
	c.mMinTimeRemaining = s.mMinTimeRemaining;
	c.mCollisionTolerance = s.mCollisionTolerance;
	c.mCharacterPadding = s.mCharacterPadding;
	c.mMaxNumHits = s.mMaxNumHits;
	c.mHitReductionCosMaxAngle = s.mHitReductionCosMaxAngle;
	c.mPenetrationRecoverySpeed = s.mPenetrationRecoverySpeed;
	c.mInnerBodyShape = s.mInnerBodyShape == nullptr? cInvalidHandle : 0;
	c.mInnerBodyIDOverride = s.mInnerBodyIDOverride.GetIndexAndSequenceNumber();
	c.mInnerBodyLayer = s.mInnerBodyLayer;
	c.mPosition[0] = c.mPosition[1] = c.mPosition[2] = 0;
	c.mRotation[0] = c.mRotation[1] = c.mRotation[2] = 0;
	c.mRotation[3] = 1;
	c.mUserData = 0;
	c.mListener = 1;
	c.mCollisionGroup = cInvalidHandle;
}

// CharacterVirtual::ExtendedUpdateSettings defaults (f[4..16] of cv_extended_update)
void jolt_sim_default_extended_update(float *outFloats)
{
	CharacterVirtual::ExtendedUpdateSettings s;
	s.mStickToFloorStepDown.StoreFloat3(reinterpret_cast<Float3 *>(outFloats));
	s.mWalkStairsStepUp.StoreFloat3(reinterpret_cast<Float3 *>(outFloats + 3));
	outFloats[6] = s.mWalkStairsMinStepForward;
	outFloats[7] = s.mWalkStairsStepForwardTest;
	outFloats[8] = s.mWalkStairsCosAngleForwardContact;
	s.mWalkStairsStepDownExtra.StoreFloat3(reinterpret_cast<Float3 *>(outFloats + 9));
}

// The engine / transmission members of SimVehicle from Jolt's settings (the curves and ratios: cInvalidHandle, the
// constructor's)
static void SimFromEngine(const VehicleEngineSettings &inEngine, SimVehicleEngine &outEngine)
{
	outEngine.mMaxTorque = inEngine.mMaxTorque;
	outEngine.mMinRPM = inEngine.mMinRPM;
	outEngine.mMaxRPM = inEngine.mMaxRPM;
	outEngine.mTorquePoints = cInvalidHandle;
	outEngine.mNumTorquePoints = 0;
	outEngine.mInertia = inEngine.mInertia;
	outEngine.mAngularDamping = inEngine.mAngularDamping;
}

static void SimFromTransmission(const VehicleTransmissionSettings &inTransmission, SimVehicleTransmission &outTransmission)
{
	outTransmission.mMode = uint32(inTransmission.mMode);
	outTransmission.mGearRatios = cInvalidHandle;
	outTransmission.mNumGearRatios = 0;
	outTransmission.mReverseGearRatios = cInvalidHandle;
	outTransmission.mNumReverseGearRatios = 0;
	outTransmission.mSwitchTime = inTransmission.mSwitchTime;
	outTransmission.mClutchReleaseTime = inTransmission.mClutchReleaseTime;
	outTransmission.mSwitchLatency = inTransmission.mSwitchLatency;
	outTransmission.mShiftUpRPM = inTransmission.mShiftUpRPM;
	outTransmission.mShiftDownRPM = inTransmission.mShiftDownRPM;
	outTransmission.mClutchStrength = inTransmission.mClutchStrength;
}

// Jolt's VehicleConstraintSettings defaults with the controller settings defaults of VehicleControllerKind inKind as a
// SimVehicle (the members of the other kinds: the wheeled defaults; the tester and the harness members: SimScenes'
// defaults; VehicleTrackSettings::mDrivenWheel has no default in Jolt: 0)
void jolt_sim_default_vehicle(uint32 inKind, SimVehicle *outVehicle)
{
	SimEnsureFactory();
	VehicleConstraintSettings s;
	WheeledVehicleControllerSettings wheeled;
	SimVehicle &v = *outVehicle;
	v.mBody = cInvalidHandle;
	v.mEnabled = s.mEnabled? 1 : 0;
	v.mConstraintPriority = s.mConstraintPriority;
	v.mNumVelocityStepsOverride = s.mNumVelocityStepsOverride;
	v.mNumPositionStepsOverride = s.mNumPositionStepsOverride;
	v.mDrawConstraintSize = s.mDrawConstraintSize;
	v.mUserData = s.mUserData;
	s.mUp.StoreFloat3(reinterpret_cast<Float3 *>(v.mUp));
	s.mForward.StoreFloat3(reinterpret_cast<Float3 *>(v.mForward));
	v.mMaxPitchRollAngle = s.mMaxPitchRollAngle;
	v.mFirstWheel = v.mNumWheels = v.mFirstAntiRollBar = v.mNumAntiRollBars = 0;
	v.mController = inKind;
	v.mFirstDifferential = v.mNumDifferentials = 0;
	v.mDifferentialLimitedSlipRatio = wheeled.mDifferentialLimitedSlipRatio;
	MotorcycleControllerSettings motorcycle;
	v.mMaxLeanAngle = motorcycle.mMaxLeanAngle;
	v.mLeanSpringConstant = motorcycle.mLeanSpringConstant;
	v.mLeanSpringDamping = motorcycle.mLeanSpringDamping;
	v.mLeanSpringIntegrationCoefficient = motorcycle.mLeanSpringIntegrationCoefficient;
	v.mLeanSpringIntegrationCoefficientDecay = motorcycle.mLeanSpringIntegrationCoefficientDecay;
	v.mLeanSmoothingFactor = motorcycle.mLeanSmoothingFactor;
	TrackedVehicleControllerSettings tracked;
	for (int t = 0; t < 2; ++t)
	{
		const VehicleTrackSettings &track = tracked.mTracks[t];
		SimVehicleTrack &st = v.mTracks[t];
		st.mDrivenWheel = 0;
		st.mWheels = 0;
		st.mNumWheels = uint32(track.mWheels.size());
		st.mInertia = track.mInertia;
		st.mAngularDamping = track.mAngularDamping;
		st.mMaxBrakeTorque = track.mMaxBrakeTorque;
		st.mDifferentialRatio = track.mDifferentialRatio;
	}
	switch (inKind)
	{
	case 1:
		SimFromEngine(tracked.mEngine, v.mEngine);
		SimFromTransmission(tracked.mTransmission, v.mTransmission);
		break;
	case 2:
		SimFromEngine(motorcycle.mEngine, v.mEngine);
		SimFromTransmission(motorcycle.mTransmission, v.mTransmission);
		break;
	default:
		SimFromEngine(wheeled.mEngine, v.mEngine);
		SimFromTransmission(wheeled.mTransmission, v.mTransmission);
		break;
	}
	v.mTester.mKind = 0;
	v.mTester.mObjectLayer = 0;
	v.mTester.mUp[0] = 0; v.mTester.mUp[1] = 1; v.mTester.mUp[2] = 0;
	v.mTester.mMaxSlopeAngle = DegreesToRadians(80.0f);						// The default arguments of the tester constructors
	v.mTester.mRadius = 0.3f;												// The harness' choice: inRadius of the cast sphere tester has no default
	v.mTester.mConvexRadiusFraction = 0.1f;
	v.mCallbacks = 0;
	v.mCombineFriction = 0;
	v.mTireMaxImpulse = 0;
}

// Jolt's WheelSettingsWV (inKind 0) / WheelSettingsTV (inKind 1) defaults as a SimVehicleWheel (the members of the other
// class: SimScenes' defaults)
void jolt_sim_default_vehicle_wheel(uint32 inKind, SimVehicleWheel *outWheel)
{
	SimEnsureFactory();
	WheelSettingsWV wv;
	WheelSettingsTV tv;
	const WheelSettings &s = inKind == 0? static_cast<const WheelSettings &>(wv) : static_cast<const WheelSettings &>(tv);
	SimVehicleWheel &w = *outWheel;
	w.mKind = inKind;
	s.mPosition.StoreFloat3(reinterpret_cast<Float3 *>(w.mPosition));
	s.mSuspensionForcePoint.StoreFloat3(reinterpret_cast<Float3 *>(w.mSuspensionForcePoint));
	s.mSuspensionDirection.StoreFloat3(reinterpret_cast<Float3 *>(w.mSuspensionDirection));
	s.mSteeringAxis.StoreFloat3(reinterpret_cast<Float3 *>(w.mSteeringAxis));
	s.mWheelUp.StoreFloat3(reinterpret_cast<Float3 *>(w.mWheelUp));
	s.mWheelForward.StoreFloat3(reinterpret_cast<Float3 *>(w.mWheelForward));
	w.mSuspensionMinLength = s.mSuspensionMinLength;
	w.mSuspensionMaxLength = s.mSuspensionMaxLength;
	w.mSuspensionPreloadLength = s.mSuspensionPreloadLength;
	w.mSuspensionSpringMode = uint32(s.mSuspensionSpring.mMode);
	w.mSuspensionFrequency = s.mSuspensionSpring.mFrequency;
	w.mSuspensionDamping = s.mSuspensionSpring.mDamping;
	w.mRadius = s.mRadius;
	w.mWidth = s.mWidth;
	w.mEnableSuspensionForcePoint = s.mEnableSuspensionForcePoint? 1 : 0;
	w.mInertia = wv.mInertia;
	w.mAngularDamping = wv.mAngularDamping;
	w.mMaxSteerAngle = wv.mMaxSteerAngle;
	w.mLongitudinalFriction = cInvalidHandle;
	w.mNumLongitudinalFriction = 0;
	w.mLateralFriction = cInvalidHandle;
	w.mNumLateralFriction = 0;
	w.mMaxBrakeTorque = wv.mMaxBrakeTorque;
	w.mMaxHandBrakeTorque = wv.mMaxHandBrakeTorque;
	w.mLongitudinalFrictionTV = tv.mLongitudinalFriction;
	w.mLateralFrictionTV = tv.mLateralFriction;
}

// The bytes of part inPart (VehicleSettingsPart of SimScenes.zig) item inItem of vehicle description inIndex, see
// SimVehicleSettingsPart: inRestore != 0 restores inBytes into a default object of the part's class and saves it again.
// Returns the size (the bytes are copied when they fit), cNoBytes when the part does not exist or the restore failed.
uint32 jolt_sim_vehicle_settings_part(const SimSceneDesc *inDesc, uint32 inIndex, uint32 inPart, uint32 inItem, uint32 inRestore, const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity)
{
	SimEnsureFactory();
	std::string data;
	if (!SimVehicleSettingsPart(*inDesc, inDesc->mVehicles[inIndex], inPart, inItem, inRestore != 0, inBytes, inSize, data))
		return cNoBytes;
	return SimCopyBytes(data, outBytes, inCapacity);
}

// The SaveBinaryState bytes (with the shapes and the group filter) of the RagdollSettings of ragdoll description inIndex
// as the world builds them (SimWorld::MakeRagdollSettings, the shapes of the world), or with inRestore != 0
// RagdollSettings::sRestoreFromBinaryState of inBytes (every byte read) saved again. Returns the size (the bytes are
// copied when they fit), cNoBytes when the restore failed.
uint32 jolt_sim_ragdoll_settings_bytes(void *inWorld, uint32 inIndex, uint32 inRestore, const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	Ref<RagdollSettings> settings;
	if (inRestore != 0)
	{
		std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
		StreamInWrapper stream(data);
		RagdollSettings::RagdollResult result = RagdollSettings::sRestoreFromBinaryState(stream);
		if (result.HasError() || !SimReadCompletely(stream))
			return cNoBytes;
		settings = result.Get();
	}
	else
		settings = world->MakeRagdollSettings(inIndex);
	std::stringstream data;
	StreamOutWrapper stream(data);
	settings->SaveBinaryState(stream, true, true);
	return SimCopyBytes(data.str(), outBytes, inCapacity);
}

// The tables of the RagdollSettings of ragdoll description inIndex (SimWorld::MakeRagdollSettings): the result of
// Stabilize (1 / 0, 2: not called), the number of parts and GetBodyIndexToConstraintIndex, the number of constraints
// and GetConstraintIndexToBodyIdxPair (2 values each). Returns the number of values (copied when they fit).
uint32 jolt_sim_ragdoll_settings_info(void *inWorld, uint32 inIndex, uint32 *outValues, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	uint32 stabilize = 2;
	Ref<RagdollSettings> settings = world->MakeRagdollSettings(inIndex, &stabilize);
	Array<uint32> values;
	values.push_back(stabilize);
	const Array<int> &body_to_constraint = settings->GetBodyIndexToConstraintIndex();
	values.push_back(uint32(body_to_constraint.size()));
	for (int c : body_to_constraint)
		values.push_back(uint32(c));
	const Array<RagdollSettings::BodyIdxPair> &pairs = settings->GetConstraintIndexToBodyIdxPair();
	values.push_back(uint32(pairs.size()));
	for (const RagdollSettings::BodyIdxPair &p : pairs)
	{
		values.push_back(uint32(p.first));
		values.push_back(uint32(p.second));
	}
	uint32 size = uint32(values.size());
	if (size <= inCapacity)
		memcpy(outValues, values.data(), size * sizeof(uint32));
	return size;
}

// The CharacterID test (character_id_sequence_len values of SimScenes.zig; the counter is set back to 1 afterwards):
// sSetNextCharacterID(inFirstID); a default constructed CharacterVirtualSettings a (its mID, then the next ID), a copy
// b of a (mID, next), c default constructed then assigned from a (mID, next), a CharacterVirtual of a with shape 0 of
// the world at the origin (GetID, next), GetCharacterVirtualSettings of it (mID, next). "next" is the ID that
// sNextCharacterID would return (read without consuming it: sNextCharacterID, then sSetNextCharacterID back).
void jolt_sim_character_id_sequence(void *inWorld, uint32 inFirstID, uint32 *outValues)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	auto next = []() {
		uint32 value = CharacterID::sNextCharacterID().GetValue();
		CharacterID::sSetNextCharacterID(value);
		return value;
	};
	CharacterID::sSetNextCharacterID(inFirstID);
	CharacterVirtualSettings a;
	a.mShape = world->mShapes[0];
	outValues[0] = a.mID.GetValue();
	outValues[1] = next();
	CharacterVirtualSettings b(a);
	outValues[2] = b.mID.GetValue();
	outValues[3] = next();
	CharacterVirtualSettings c;
	c = a;
	outValues[4] = c.mID.GetValue();
	outValues[5] = next();
	{
		Ref<CharacterVirtual> character = new CharacterVirtual(&a, RVec3::sZero(), Quat::sIdentity(), 0, &world->mSystem);
		outValues[6] = character->GetID().GetValue();
		outValues[7] = next();
		CharacterVirtualSettings s = character->GetCharacterVirtualSettings();
		outValues[8] = s.mID.GetValue();
		outValues[9] = next();
	}
	CharacterID::sSetNextCharacterID();
}

// 1 when the allocation hooks of the parity process zero-fill (ParityHooks.h): a block that was filled and freed
// comes back as zeros from Allocate and AlignedAllocate, Reallocate keeps the old bytes and zeroes the new ones
uint32 jolt_sim_zero_filling_check()
{
	SimEnsureFactory();
	uint32 ok = 1;
	for (uint32 i = 0; i < 8; ++i)
	{
		size_t size = size_t(48) << i;
		uint8 *dirty = static_cast<uint8 *>(Allocate(size));
		memset(dirty, 0xab, size);
		Free(dirty);
		uint8 *block = static_cast<uint8 *>(Allocate(size));
		for (size_t j = 0; j < size; ++j)
			ok &= block[j] == 0? 1 : 0;
		memset(block, 0xcd, size);
		uint8 *grown = static_cast<uint8 *>(Reallocate(block, size, 2 * size));
		for (size_t j = 0; j < 2 * size; ++j)
			ok &= grown[j] == (j < size? 0xcd : 0)? 1 : 0;
		Free(grown);
		uint8 *aligned_dirty = static_cast<uint8 *>(AlignedAllocate(size, 64));
		memset(aligned_dirty, 0xef, size);
		AlignedFree(aligned_dirty);
		uint8 *aligned = static_cast<uint8 *>(AlignedAllocate(size, 64));
		for (size_t j = 0; j < size; ++j)
			ok &= aligned[j] == 0? 1 : 0;
		AlignedFree(aligned);
	}
	return ok;
}

// The number of Jolt asserts that failed in this process (ParityAssertFailed; always 0 without JPH_ENABLE_ASSERTS)
uint32 jolt_sim_reference_assert_count()
{
	return ParityAssertCount();
}

// Fails a JPH_ASSERT on purpose (the test of the assert hook): the number of failed asserts before and after it
void jolt_sim_test_assert_hook(uint32 *outBefore, uint32 *outAfter)
{
	SimEnsureFactory();
	*outBefore = jolt_sim_reference_assert_count();
	JPH_ASSERT(false, "SimulationParity: the test of the assert hook (expected)");
	*outAfter = jolt_sim_reference_assert_count();
}

// 1 when the reference library is built with JPH_ENABLE_ASSERTS (`zig build parity -Dreference_asserts=true`)
uint32 jolt_sim_reference_has_asserts()
{
#ifdef JPH_ENABLE_ASSERTS
	return 1;
#else
	return 0;
#endif
}

// Phase 9: the sizes and some offsets of the soft body structs shared with SimScenes.zig
void jolt_sim_soft_body_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(SimSoftBodySettings);
	outSizes[1] = sizeof(SimSoftBody);
	outSizes[2] = offsetof(SimSoftBody, mPosition);
	outSizes[3] = offsetof(SimSoftBody, mSettings);
	outSizes[4] = offsetof(SimSoftBody, mFacesDoubleSided);
	outSizes[5] = offsetof(SimSoftBodySettings, mBackStopRadius);
	outSizes[6] = offsetof(SimSceneDesc, mSoftBodySettings);
	outSizes[7] = sizeof(SimSceneDesc);
}

// Jolt's defaults of SoftBodyCreationSettings (SimSoftBody; mSettings is 0)
void jolt_sim_default_soft_body(SimSoftBody *outSoftBody)
{
	SoftBodyCreationSettings s;
	SimSoftBody &o = *outSoftBody;
	o.mUserData = s.mUserData;
	o.mPosition[0] = s.mPosition.GetX();
	o.mPosition[1] = s.mPosition.GetY();
	o.mPosition[2] = s.mPosition.GetZ();
	o.mRotation[0] = s.mRotation.GetX();
	o.mRotation[1] = s.mRotation.GetY();
	o.mRotation[2] = s.mRotation.GetZ();
	o.mRotation[3] = s.mRotation.GetW();
	o.mSettings = 0;
	o.mObjectLayer = s.mObjectLayer;
	o.mCollisionGroupID = s.mCollisionGroup.GetGroupID();
	o.mCollisionSubGroupID = s.mCollisionGroup.GetSubGroupID();
	o.mNumIterations = s.mNumIterations;
	o.mLinearDamping = s.mLinearDamping;
	o.mMaxLinearVelocity = s.mMaxLinearVelocity;
	o.mRestitution = s.mRestitution;
	o.mFriction = s.mFriction;
	o.mPressure = s.mPressure;
	o.mGravityFactor = s.mGravityFactor;
	o.mVertexRadius = s.mVertexRadius;
	o.mUpdatePosition = s.mUpdatePosition? 1 : 0;
	o.mMakeRotationIdentity = s.mMakeRotationIdentity? 1 : 0;
	o.mAllowSleeping = s.mAllowSleeping? 1 : 0;
	o.mFacesDoubleSided = s.mFacesDoubleSided? 1 : 0;
}

// Jolt's defaults in SimSoftBodySettings: VertexAttributes, the bend type of CreateConstraints, Skinned (the other
// members are the generator's and stay as they are)
void jolt_sim_default_soft_body_settings(SimSoftBodySettings *outSettings)
{
	SoftBodySharedSettings::VertexAttributes attributes;
	outSettings->mCompliance = attributes.mCompliance;
	outSettings->mShearCompliance = attributes.mShearCompliance;
	outSettings->mBendCompliance = attributes.mBendCompliance;
	outSettings->mLRAType = uint32(attributes.mLRAType);
	outSettings->mLRAMaxDistanceMultiplier = attributes.mLRAMaxDistanceMultiplier;
	outSettings->mBendType = uint32(SoftBodySharedSettings::EBendType::Distance); // The default argument of CreateConstraints
	SoftBodySharedSettings::Skinned skinned;
	outSettings->mSkinMaxDistance = skinned.mMaxDistance;
	outSettings->mBackStopDistance = skinned.mBackStopDistance;
	outSettings->mBackStopRadius = skinned.mBackStopRadius;
}

// The SaveBinaryState bytes of the shared settings that SimMakeSoftBodySettings generates from soft body settings
// description inIndex; returns the size (the bytes are copied when they fit)
uint32 jolt_sim_soft_body_settings_bytes(const SimSceneDesc *inDesc, uint32 inIndex, uint8 *outBytes, uint32 inCapacity)
{
	SimEnsureFactory();
	Ref<SoftBodySharedSettings> settings = SimMakeSoftBodySettings(*inDesc, inDesc->mSoftBodySettings[inIndex]);
	return SimCopyBytes(SimSaveSoftBodySettings(*settings), outBytes, inCapacity);
}

// For the state decoder: 1 when body inBodyID is a soft body, outCounts gets the sizes of the arrays that
// SoftBodyMotionProperties::SaveState writes: the vertices, the rod states (one per rod stretch shear constraint) and
// the skin states (one per vertex when there are skinned constraints), as Initialize sizes them
uint32 jolt_sim_soft_body_info(void *inWorld, uint32 inBodyID, uint32 *outCounts)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	BodyLockRead lock(world->mSystem.GetBodyLockInterfaceNoLock(), BodyID(inBodyID));
	if (!lock.Succeeded() || !lock.GetBody().IsSoftBody())
		return 0;
	const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(lock.GetBody().GetMotionProperties());
	const SoftBodySharedSettings *settings = mp->GetSettings();
	outCounts[0] = uint32(mp->GetVertices().size());
	outCounts[1] = uint32(settings->mRodStretchShearConstraints.size());
	outCounts[2] = settings->mSkinnedConstraints.empty()? 0 : uint32(mp->GetVertices().size());
	return 1;
}

// For the test of the state decoder: of soft body inBodyID per vertex the local position and velocity (6 floats), per
// rod the rotation and the angular velocity (7 floats), then the local bounds (6 floats); returns the number of floats
// (copied when they fit), 0 when the body is not a soft body
uint32 jolt_sim_soft_body_values(void *inWorld, uint32 inBodyID, float *outValues, uint32 inCapacity)
{
	SimWorld *world = static_cast<SimWorld *>(inWorld);
	BodyLockRead lock(world->mSystem.GetBodyLockInterfaceNoLock(), BodyID(inBodyID));
	if (!lock.Succeeded() || !lock.GetBody().IsSoftBody())
		return 0;
	const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(lock.GetBody().GetMotionProperties());
	Array<float> values;
	for (const SoftBodyVertex &v : mp->GetVertices())
		for (Vec3 x : { v.mPosition, v.mVelocity })
		{
			values.push_back(x.GetX());
			values.push_back(x.GetY());
			values.push_back(x.GetZ());
		}
	for (uint r = 0; r < uint(mp->GetSettings()->mRodStretchShearConstraints.size()); ++r)
	{
		Quat q = mp->GetRodRotation(r);
		Vec3 w = mp->GetRodAngularVelocity(r);
		for (float x : { q.GetX(), q.GetY(), q.GetZ(), q.GetW(), w.GetX(), w.GetY(), w.GetZ() })
			values.push_back(x);
	}
	const AABox &bounds = mp->GetLocalBounds();
	for (Vec3 x : { bounds.mMin, bounds.mMax })
	{
		values.push_back(x.GetX());
		values.push_back(x.GetY());
		values.push_back(x.GetZ());
	}
	uint32 size = uint32(values.size());
	if (size <= inCapacity)
		memcpy(outValues, values.data(), size * sizeof(float));
	return size;
}

} // extern "C"
