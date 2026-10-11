// Reference implementation for the Physics basics parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library (MassProperties, BodyID, SubShapeID / SubShapeIDCreator / SubShapeIDPair, RayCast / RRayCast,
// AABoxCast, CollideShapeResult::Reversed, the default values of the settings and result types, SortReverseAndStore /
// CountAndSortTrues, PhysicsMaterial / PhysicsMaterialSimple binary state and RTTI hashes, PhysicsSettings).
// ZoltParity/Physics/BasicsParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), Mat44 as 16 floats in column major order,
// RVec3 as 3 Reals (double with JPH_DOUBLE_PRECISION), RMat44 as 3 columns of 4 floats + an RVec3 translation.
// Booleans are passed as int (never bool, see the porting guide). Functions that produce bytes write into a caller
// provided buffer and return the size.
//
// Jolt's Array / String allocate through the Allocate / Free hooks (RegisterDefaultAllocator) and
// PhysicsMaterial::sRestoreFromBinaryState needs the Factory with the registered types, like Jolt's UnitTests main
// does. EnsureFactory creates both once for the whole parity binary (other wrapper files that need the Factory must
// use the same `Factory::sInstance == nullptr` check so that RegisterTypes() runs only once).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/RTTI.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Physics/Collision/AABoxCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/SortReverseAndStore.h>
#include <Jolt/Physics/Collision/Shape/SubShapeID.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>

#include <cstring>
#include <sstream>
#include <string>
#include <string_view>

using namespace JPH;

static void EnsureAllocator()
{
	RegisterDefaultAllocator();
}

static void EnsureFactory()
{
	EnsureAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 LoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void Store4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void StoreR3(RVec3Arg inV, Real *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Mat44 LoadMat44(const float *inM)			{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
static void StoreMat44(Mat44Arg inM, float *outM)	{ for (int c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
static RMat44 LoadRMat44(const float *inCols, const Real *inTranslation) { return RMat44(Load4(inCols), Load4(inCols + 4), Load4(inCols + 8), LoadR3(inTranslation)); }

static uint32 CopyBytes(const std::string &inString, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inString.size());
	if (size <= inCapacity)
		memcpy(outBytes, inString.data(), size);
	return size;
}

// MassProperties in the format of the C ABI, must match MP in BasicsParity.zig
struct MP
{
	float				mMass;
	float				mInertia[16];
};

static MassProperties LoadMP(const MP *inMP)
{
	MassProperties m;
	m.mMass = inMP->mMass;
	m.mInertia = LoadMat44(inMP->mInertia);
	return m;
}

static void StoreMP(const MassProperties &inMP, MP *outMP)
{
	outMP->mMass = inMP.mMass;
	StoreMat44(inMP.mInertia, outMP->mInertia);
}

// CollideShapeResult in the format of the C ABI, must match CSR in BasicsParity.zig
struct CSR
{
	float				mContactPointOn1[3];
	float				mContactPointOn2[3];
	float				mPenetrationAxis[3];
	float				mPenetrationDepth;
	uint32				mSubShapeID1;
	uint32				mSubShapeID2;
	uint32				mBodyID2;
	uint32				mFace1Count;
	float				mFace1[32][3];
	uint32				mFace2Count;
	float				mFace2[32][3];
};

static SubShapeID MakeSubShapeID(uint32 inValue)
{
	SubShapeID id;
	id.SetValue(inValue);
	return id;
}

static CollideShapeResult LoadCSR(const CSR *inCSR)
{
	CollideShapeResult r(Load3(inCSR->mContactPointOn1), Load3(inCSR->mContactPointOn2), Load3(inCSR->mPenetrationAxis), inCSR->mPenetrationDepth, MakeSubShapeID(inCSR->mSubShapeID1), MakeSubShapeID(inCSR->mSubShapeID2), BodyID(inCSR->mBodyID2));
	for (uint32 i = 0; i < inCSR->mFace1Count; ++i)
		r.mShape1Face.push_back(Load3(inCSR->mFace1[i]));
	for (uint32 i = 0; i < inCSR->mFace2Count; ++i)
		r.mShape2Face.push_back(Load3(inCSR->mFace2[i]));
	return r;
}

static void StoreCSR(const CollideShapeResult &inR, CSR *outCSR)
{
	Store3(inR.mContactPointOn1, outCSR->mContactPointOn1);
	Store3(inR.mContactPointOn2, outCSR->mContactPointOn2);
	Store3(inR.mPenetrationAxis, outCSR->mPenetrationAxis);
	outCSR->mPenetrationDepth = inR.mPenetrationDepth;
	outCSR->mSubShapeID1 = inR.mSubShapeID1.GetValue();
	outCSR->mSubShapeID2 = inR.mSubShapeID2.GetValue();
	outCSR->mBodyID2 = inR.mBodyID2.GetIndexAndSequenceNumber();
	outCSR->mFace1Count = uint32(inR.mShape1Face.size());
	for (uint32 i = 0; i < outCSR->mFace1Count; ++i)
		Store3(inR.mShape1Face[i], outCSR->mFace1[i]);
	outCSR->mFace2Count = uint32(inR.mShape2Face.size());
	for (uint32 i = 0; i < outCSR->mFace2Count; ++i)
		Store3(inR.mShape2Face[i], outCSR->mFace2[i]);
}

extern "C"
{

// Physics/Body/MassProperties.h
void jolt_mass_properties_solid_box(const float *inBoxSize, float inDensity, MP *outMP)
{
	MassProperties m;
	m.SetMassAndInertiaOfSolidBox(Load3(inBoxSize), inDensity);
	StoreMP(m, outMP);
}

void jolt_mass_properties_scale_to_mass(const MP *inMP, float inMass, MP *outMP)
{
	MassProperties m = LoadMP(inMP);
	m.ScaleToMass(inMass);
	StoreMP(m, outMP);
}

void jolt_mass_properties_equivalent_solid_box_size(float inMass, const float *inInertiaDiagonal, float *outSize)
{
	Store3(MassProperties::sGetEquivalentSolidBoxSize(inMass, Load3(inInertiaDiagonal)), outSize);
}

void jolt_mass_properties_rotate(const MP *inMP, const float *inRotation, MP *outMP)
{
	MassProperties m = LoadMP(inMP);
	m.Rotate(LoadMat44(inRotation));
	StoreMP(m, outMP);
}

void jolt_mass_properties_translate(const MP *inMP, const float *inTranslation, MP *outMP)
{
	MassProperties m = LoadMP(inMP);
	m.Translate(Load3(inTranslation));
	StoreMP(m, outMP);
}

void jolt_mass_properties_scale(const MP *inMP, const float *inScale, MP *outMP)
{
	MassProperties m = LoadMP(inMP);
	m.Scale(Load3(inScale));
	StoreMP(m, outMP);
}

// ioRotation / ioDiagonal hold sentinels on input, Jolt only writes them when the decomposition succeeds
int jolt_mass_properties_decompose(const MP *inMP, float *ioRotation, float *ioDiagonal)
{
	MassProperties m = LoadMP(inMP);
	Mat44 rotation = LoadMat44(ioRotation);
	Vec3 diagonal = Load3(ioDiagonal);
	bool ok = m.DecomposePrincipalMomentsOfInertia(rotation, diagonal);
	StoreMat44(rotation, ioRotation);
	Store3(diagonal, ioDiagonal);
	return ok? 1 : 0;
}

int jolt_mass_properties_equal(const MP *inA, const MP *inB)
{
	return LoadMP(inA) == LoadMP(inB)? 1 : 0;
}

uint32 jolt_mass_properties_save(const MP *inMP, uint8 *outBytes, uint32 inCapacity)
{
	EnsureAllocator();
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	LoadMP(inMP).SaveBinaryState(wrapper);
	return CopyBytes(stream.str(), outBytes, inCapacity);
}

// ioMP holds the values before the restore (a failed read can leave them partially overwritten)
void jolt_mass_properties_restore(const uint8 *inBytes, uint32 inSize, MP *ioMP, int *outEOF, int *outFailed)
{
	EnsureAllocator();
	std::istringstream stream(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper wrapper(stream);
	MassProperties m = LoadMP(ioMP);
	m.RestoreBinaryState(wrapper);
	StoreMP(m, ioMP);
	*outEOF = wrapper.IsEOF()? 1 : 0;
	*outFailed = wrapper.IsFailed()? 1 : 0;
}

// Physics/Body/BodyID.h
void jolt_body_id(uint32 inIndex, uint32 inSequenceNumber, uint32 *outValues, uint64 *outHash)
{
	BodyID id(inIndex, uint8(inSequenceNumber));
	outValues[0] = id.GetIndexAndSequenceNumber();
	outValues[1] = id.GetIndex();
	outValues[2] = id.GetSequenceNumber();
	outValues[3] = id.IsInvalid()? 1 : 0;
	*outHash = Hash<BodyID> { } (id);
}

void jolt_body_id_compare(uint32 inA, uint32 inB, int *outResults)
{
	BodyID a(inA), b(inB);
	outResults[0] = a == b? 1 : 0;
	outResults[1] = a != b? 1 : 0;
	outResults[2] = a < b? 1 : 0;
	outResults[3] = a > b? 1 : 0;
	outResults[4] = a.IsInvalid()? 1 : 0;
}

// Physics/Collision/Shape/SubShapeID.h: pushes inCount ids, stores the id and the number of bits written after each push
void jolt_sub_shape_id_push(const uint32 *inValues, const uint32 *inBits, int inCount, uint32 *outIDs, uint32 *outBitsWritten)
{
	SubShapeIDCreator creator;
	for (int i = 0; i < inCount; ++i)
	{
		creator = creator.PushID(inValues[i], inBits[i]);
		outIDs[i] = creator.GetID().GetValue();
		outBitsWritten[i] = creator.GetNumBitsWritten();
	}
}

uint32 jolt_sub_shape_id_pop(uint32 inValue, uint32 inBits, uint32 *outRemainder, int *outIsEmpty)
{
	SubShapeID id = MakeSubShapeID(inValue);
	*outIsEmpty = id.IsEmpty()? 1 : 0;
	SubShapeID remainder;
	uint32 result = id.PopID(inBits, remainder);
	*outRemainder = remainder.GetValue();
	return result;
}

// Physics/Collision/Shape/SubShapeIDPair.h: inA / inB are body1, sub shape 1, body2, sub shape 2
void jolt_sub_shape_id_pair(const uint32 *inA, const uint32 *inB, int *outResults, uint64 *outHash)
{
	SubShapeIDPair a(BodyID(inA[0]), MakeSubShapeID(inA[1]), BodyID(inA[2]), MakeSubShapeID(inA[3]));
	SubShapeIDPair b(BodyID(inB[0]), MakeSubShapeID(inB[1]), BodyID(inB[2]), MakeSubShapeID(inB[3]));
	outResults[0] = a == b? 1 : 0;
	outResults[1] = a < b? 1 : 0;
	outResults[2] = b < a? 1 : 0;
	*outHash = a.GetHash();
	outHash[1] = Hash<SubShapeIDPair> { } (a);
}

// Physics/Collision/RayCast.h
void jolt_ray_cast(const float *inOrigin, const float *inDirection, const float *inTransform, const float *inTranslation, float inFraction, float *outTransformed, float *outTranslated, float *outPoint)
{
	RayCast ray(Load3(inOrigin), Load3(inDirection));
	RayCast transformed = ray.Transformed(LoadMat44(inTransform));
	Store3(transformed.mOrigin, outTransformed);
	Store3(transformed.mDirection, outTransformed + 3);
	RayCast translated = ray.Translated(Load3(inTranslation));
	Store3(translated.mOrigin, outTranslated);
	Store3(translated.mDirection, outTranslated + 3);
	Store3(ray.GetPointOnRay(inFraction), outPoint);
}

void jolt_r_ray_cast(const Real *inOrigin, const float *inDirection, const float *inTransformColumns, const Real *inTransformTranslation, const Real *inTranslation, float inFraction, Real *outTransformedOrigin, float *outTransformedDirection, Real *outTranslatedOrigin, float *outTranslatedDirection, Real *outPoint, float *outRayCast)
{
	RRayCast ray(LoadR3(inOrigin), Load3(inDirection));
	RRayCast transformed = ray.Transformed(LoadRMat44(inTransformColumns, inTransformTranslation));
	StoreR3(transformed.mOrigin, outTransformedOrigin);
	Store3(transformed.mDirection, outTransformedDirection);
	RRayCast translated = ray.Translated(LoadR3(inTranslation));
	StoreR3(translated.mOrigin, outTranslatedOrigin);
	Store3(translated.mDirection, outTranslatedDirection);
	StoreR3(ray.GetPointOnRay(inFraction), outPoint);
	RayCast single = ray.operator RayCast(); // explicit operator RayCast()
	Store3(single.mOrigin, outRayCast);
	Store3(single.mDirection, outRayCast + 3);
}

void jolt_r_ray_cast_from_ray_cast(const float *inOrigin, const float *inDirection, Real *outOrigin, float *outDirection)
{
	RRayCast ray(RayCast(Load3(inOrigin), Load3(inDirection)));
	StoreR3(ray.mOrigin, outOrigin);
	Store3(ray.mDirection, outDirection);
}

void jolt_ray_cast_settings_defaults(uint32 *outValues)
{
	RayCastSettings settings;
	outValues[0] = uint32(settings.mBackFaceModeTriangles);
	outValues[1] = uint32(settings.mBackFaceModeConvex);
	outValues[2] = settings.mTreatConvexAsSolid? 1 : 0;
	settings.SetBackFaceMode(EBackFaceMode::CollideWithBackFaces);
	outValues[3] = uint32(settings.mBackFaceModeTriangles);
	outValues[4] = uint32(settings.mBackFaceModeConvex);
}

// Physics/Collision/AABoxCast.h
void jolt_aabox_cast(const float *inMin, const float *inMax, const float *inDirection, float *outValues)
{
	AABoxCast cast { AABox(Load3(inMin), Load3(inMax)), Load3(inDirection) };
	Store3(cast.mBox.mMin, outValues);
	Store3(cast.mBox.mMax, outValues + 3);
	Store3(cast.mDirection, outValues + 6);
}

// Physics/Collision/CollideShape.h
void jolt_collide_shape_result_reversed(const CSR *inCSR, CSR *outCSR, float *outEarlyOut)
{
	CollideShapeResult r = LoadCSR(inCSR);
	CollideShapeResult reversed = r.Reversed();
	StoreCSR(reversed, outCSR);
	outEarlyOut[0] = r.GetEarlyOutFraction();
	outEarlyOut[1] = reversed.GetEarlyOutFraction();
}

void jolt_collide_shape_settings_defaults(uint32 *outEnums, float *outFloats)
{
	CollideShapeSettings settings;
	outEnums[0] = uint32(settings.mActiveEdgeMode);
	outEnums[1] = uint32(settings.mCollectFacesMode);
	outEnums[2] = uint32(settings.mBackFaceMode);
	outFloats[0] = settings.mCollisionTolerance;
	outFloats[1] = settings.mPenetrationTolerance;
	Store3(settings.mActiveEdgeMovementDirection, outFloats + 2);
	outFloats[5] = settings.mMaxSeparationDistance;
	outFloats[6] = settings.mInternalEdgeRemovalVertexToleranceSq;

	CollideSettingsBase base;
	outEnums[3] = uint32(base.mActiveEdgeMode);
	outEnums[4] = uint32(base.mCollectFacesMode);
	outFloats[7] = base.mCollisionTolerance;
	outFloats[8] = base.mPenetrationTolerance;
	Store3(base.mActiveEdgeMovementDirection, outFloats + 9);
}

// Default values of the result types: ids (body ids and sub shape ids) and fractions
void jolt_result_defaults(uint32 *outIDs, float *outFractions)
{
	BroadPhaseCastResult broad_phase;
	outIDs[0] = broad_phase.mBodyID.GetIndexAndSequenceNumber();
	outFractions[0] = broad_phase.mFraction;
	outFractions[1] = broad_phase.GetEarlyOutFraction();

	RayCastResult ray;
	outIDs[1] = ray.mBodyID.GetIndexAndSequenceNumber();
	outIDs[2] = ray.mSubShapeID2.GetValue();
	outFractions[2] = ray.mFraction;
	ray.mBodyID = BodyID(5);
	ray.mFraction = 0.5f;
	ray.mSubShapeID2 = MakeSubShapeID(7);
	ray.Reset();
	outIDs[3] = ray.mBodyID.GetIndexAndSequenceNumber();
	outIDs[4] = ray.mSubShapeID2.GetValue();
	outFractions[3] = ray.mFraction;

	CollidePointResult point;
	outIDs[5] = point.mBodyID.GetIndexAndSequenceNumber();
	outIDs[6] = point.mSubShapeID2.GetValue();
	outFractions[4] = point.GetEarlyOutFraction();

	CollideShapeResult shape;
	outIDs[7] = shape.mSubShapeID1.GetValue();
	outIDs[8] = shape.mSubShapeID2.GetValue();
	outIDs[9] = shape.mBodyID2.GetIndexAndSequenceNumber();
	outIDs[10] = uint32(shape.mShape1Face.size());
	outIDs[11] = uint32(shape.mShape2Face.size());

	SubShapeIDPair pair;
	outIDs[12] = pair.GetBody1ID().GetIndexAndSequenceNumber();
	outIDs[13] = pair.GetSubShapeID1().GetValue();
	outIDs[14] = pair.GetBody2ID().GetIndexAndSequenceNumber();
	outIDs[15] = pair.GetSubShapeID2().GetValue();

	SubShapeIDCreator creator;
	outIDs[16] = creator.GetID().GetValue();
	outIDs[17] = creator.GetNumBitsWritten();
}

// Physics/Collision/ObjectLayer.h, Physics/Collision/BroadPhase/BroadPhaseLayer.h
void jolt_layer_constants(uint32 *outValues)
{
	outValues[0] = uint32(cObjectLayerInvalid);
	outValues[1] = uint32(sizeof(ObjectLayer));
	outValues[2] = uint32(cBroadPhaseLayerInvalid.GetValue());
	outValues[3] = uint32(sizeof(BroadPhaseLayer));
}

// Physics/PhysicsSettings.h
void jolt_physics_settings(float *outConstants, int32 *outIntConstants, int32 *outInts, float *outFloats, uint32 *outUints, int *outBools)
{
	outConstants[0] = cDefaultCollisionTolerance;
	outConstants[1] = cDefaultPenetrationTolerance;
	outConstants[2] = cDefaultConvexRadius;
	outConstants[3] = cCapsuleProjectionSlop;
	outConstants[4] = cDefaultInternalEdgeRemovalVertexToleranceSq;
	outIntConstants[0] = cMaxPhysicsJobs;
	outIntConstants[1] = cMaxPhysicsBarriers;

	PhysicsSettings s;
	outInts[0] = s.mMaxInFlightBodyPairs;
	outInts[1] = s.mStepListenersBatchSize;
	outInts[2] = s.mStepListenerBatchesPerJob;
	outFloats[0] = s.mBaumgarte;
	outFloats[1] = s.mSpeculativeContactDistance;
	outFloats[2] = s.mPenetrationSlop;
	outFloats[3] = s.mLinearCastThreshold;
	outFloats[4] = s.mLinearCastMaxPenetration;
	outFloats[5] = s.mManifoldTolerance;
	outFloats[6] = s.mMaxPenetrationDistance;
	outFloats[7] = s.mBodyPairCacheMaxDeltaPositionSq;
	outFloats[8] = s.mBodyPairCacheCosMaxDeltaRotationDiv2;
	outFloats[9] = s.mContactNormalCosMaxDeltaRotation;
	outFloats[10] = s.mContactPointPreserveLambdaMaxDistSq;
	outFloats[11] = s.mInternalEdgeRemovalVertexToleranceSq;
	outFloats[12] = s.mMinVelocityForRestitution;
	outFloats[13] = s.mTimeBeforeSleep;
	outFloats[14] = s.mPointVelocitySleepThreshold;
	outUints[0] = s.mNumVelocitySteps;
	outUints[1] = s.mNumPositionSteps;
	outBools[0] = s.mConstraintWarmStart? 1 : 0;
	outBools[1] = s.mUseBodyPairContactCache? 1 : 0;
	outBools[2] = s.mUseManifoldReduction? 1 : 0;
	outBools[3] = s.mUseLargeIslandSplitter? 1 : 0;
	outBools[4] = s.mAllowSleeping? 1 : 0;
	outBools[5] = s.mCheckActiveEdges? 1 : 0;
}

// Physics/Collision/SortReverseAndStore.h
int jolt_sort_reverse_and_store(const float *inValues, float inMaxValue, uint32 *ioIdentifiers, float *outValues)
{
	UVec4 identifiers = UVec4::sLoadInt4(ioIdentifiers);
	int num_results = SortReverseAndStore(Load4(inValues), inMaxValue, identifiers, outValues);
	identifiers.StoreInt4(ioIdentifiers);
	return num_results;
}

int jolt_count_and_sort_trues(const uint32 *inValue, uint32 *ioIdentifiers)
{
	UVec4 identifiers = UVec4::sLoadInt4(ioIdentifiers);
	int num_trues = CountAndSortTrues(UVec4::sLoadInt4(inValue), identifiers);
	identifiers.StoreInt4(ioIdentifiers);
	return num_trues;
}

// Physics/Collision/PhysicsMaterial.h, Physics/Collision/PhysicsMaterialSimple.h
void jolt_physics_material_rtti_hashes(uint32 *outHashes)
{
	outHashes[0] = JPH_RTTI(PhysicsMaterial)->GetHash();
	outHashes[1] = JPH_RTTI(PhysicsMaterialSimple)->GetHash();
}

// Saves a PhysicsMaterialSimple (inSimple != 0) or a PhysicsMaterial
uint32 jolt_physics_material_save(int inSimple, const char *inName, uint32 inNameLength, uint32 inColor, uint8 *outBytes, uint32 inCapacity)
{
	EnsureFactory();
	Ref<PhysicsMaterial> material;
	if (inSimple != 0)
		material = new PhysicsMaterialSimple(string_view(inName, inNameLength), Color(inColor));
	else
		material = new PhysicsMaterial();
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	material->SaveBinaryState(wrapper);
	return CopyBytes(stream.str(), outBytes, inCapacity);
}

// Restores a material: returns 1 and the RTTI hash, debug name and color of the material, or 0 and the error text
int jolt_physics_material_restore(const uint8 *inBytes, uint32 inSize, uint32 *outHash, char *outText, uint32 inTextCapacity, uint32 *outTextLength, uint32 *outColor)
{
	EnsureFactory();
	std::istringstream stream(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper wrapper(stream);
	PhysicsMaterial::PhysicsMaterialResult result = PhysicsMaterial::sRestoreFromBinaryState(wrapper);
	std::string text;
	int valid = 0;
	if (result.IsValid())
	{
		const PhysicsMaterial *material = result.Get();
		*outHash = material->GetRTTI()->GetHash();
		text = material->GetDebugName();
		*outColor = material->GetDebugColor().GetUInt32();
		valid = 1;
	}
	else
		text = result.GetError();
	*outTextLength = CopyBytes(text, reinterpret_cast<uint8 *>(outText), inTextCapacity);
	return valid;
}

// PhysicsMaterial::sDefault after RegisterTypes()
uint32 jolt_default_material(char *outName, uint32 inCapacity, uint32 *outColor, uint32 *outHash)
{
	EnsureFactory();
	const PhysicsMaterial *material = PhysicsMaterial::sDefault;
	*outColor = material->GetDebugColor().GetUInt32();
	*outHash = material->GetRTTI()->GetHash();
	return CopyBytes(material->GetDebugName(), reinterpret_cast<uint8 *>(outName), inCapacity);
}

} // extern "C"
