// Reference implementation for the soft body parity tests of group S0 (`zig build parity`): C ABI wrappers around the
// C++ Jolt library's SoftBodySharedSettings (SaveBinaryState / RestoreBinaryState) and the creation of a soft body
// (BodyInterface::CreateSoftBody: BodyManager::AllocateSoftBody, Body::ApplySoftBodyCreationSettings,
// SoftBodyMotionProperties::Initialize), also with ApplySoftBodyCreationSettings called again on the created body.
// ZoltParity/Physics/SoftBodyParity.zig calls these and checks that Zolt produces the same bits.
//
// Adapted from the delivery prototype of the Phase 9 design (commit 3759ab6). The simulation parts of the prototype
// (worlds, SaveState, listener records) come with the soft body simulation harness (group SH).
//
// The structs below must match the extern structs in SoftBodyParity.zig. Booleans are passed as uint32 (never bool,
// see the porting guide). All C functions have the prefix jolt_sb_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>

#include <atomic>
#include <cfloat>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

#include <UnitTests/Layers.h>

// The process hooks are ZoltParity/ParityHooks.h (R1): zero-filled allocations, a printing Trace and, with
// JPH_ENABLE_ASSERTS, an AssertFailed that prints and counts. Every exported function that allocates through Jolt calls
// ParityInstallHooks() first, because a filtered parity run links only the reference objects that it uses.
#include <ZoltParity/ParityHooks.h>

// Private access to the members of the motion properties that Jolt has no getter for (explicit instantiation ignores
// access checks, `auto` avoids naming the private nested types)
template <typename Tag, auto M>
struct SBRob
{
	friend auto SBGet(Tag) { return M; }
};

#define SB_ROB(Name, Member) \
	struct Name { friend auto SBGet(Name); }; \
	template struct SBRob<Name, &Member>;

SB_ROB(SBRodStates, SoftBodyMotionProperties::mRodStates)
SB_ROB(SBSkinState, SoftBodyMotionProperties::mSkinState)
SB_ROB(SBLocalPredictedBounds, SoftBodyMotionProperties::mLocalPredictedBounds)

// The settings kinds of jolt_sb_settings_bytes (must match SettingsKind in SoftBodyParity.zig)
enum class SBSettingsKind : uint32
{
	Random,					///< Random values in every array (SBRandom with the seed), the private arrays too
	Cube,					///< SoftBodySharedSettings::sCreateCube, grid size 2 to 8 and the spacing from the seed
	Cloth,					///< A cloth with fixated corners, dihedral bends and Euclidean LRA, optimized (grid size from the seed)
	Tetrahedron,			///< The 4 faces of a tetrahedron, CreateConstraints without bends, not optimized
	TetrahedronDistanceBend, ///< The same with EBendType::Distance
};

/// The random generator of the Random kind (must match SBRandom in SoftBodyParity.zig): xorshift32. Every value is a
/// separate statement, so the order of the calls is the order of the statements.
class SBRandom
{
public:
	explicit			SBRandom(uint32 inSeed) : mState(inSeed ^ 0x9e3779b9u) { if (mState == 0) mState = 1; }

	uint32				Next()
	{
		mState ^= mState << 13;
		mState ^= mState >> 17;
		mState ^= mState << 5;
		return mState;
	}

	/// A value in [0, inN)
	uint32				Int(uint32 inN)								{ return Next() % inN; }

	/// A float in [-15.625, 15.625] with 7 fraction bits (exact)
	float				Float()										{ return float(int(Next() % 4001) - 2000) / 128.0f; }

	/// An inverse mass in [0, 2] with 2 fraction bits (0 for 1 in 9 vertices: kinematic)
	float				InvMass()									{ return float(Next() % 9) / 4.0f; }

	Quat				RandomQuat()
	{
		float x = Float();
		float y = Float();
		float z = Float();
		float w = Float();
		return Quat(x, y, z, w);
	}

private:
	uint32				mState;
};

/// Settings with random values in every array. Jolt's private arrays (mSkinnedConstraintNormals, mUpdateGroups) are
/// returned in outNormals and outGroups (7 values per group).
static void SBGenerateRandom(uint32 inSeed, SoftBodySharedSettings &outSettings, Array<uint32> &outNormals, Array<uint32> &outGroups)
{
	SBRandom r(inSeed);
	uint32 nv = 1 + r.Int(8);
	for (uint32 i = 0; i < nv; ++i)
	{
		SoftBodySharedSettings::Vertex v;
		v.mPosition.x = r.Float();
		v.mPosition.y = r.Float();
		v.mPosition.z = r.Float();
		v.mVelocity.x = r.Float();
		v.mVelocity.y = r.Float();
		v.mVelocity.z = r.Float();
		v.mInvMass = r.InvMass();
		outSettings.mVertices.push_back(v);
	}
	uint32 nf = r.Int(5);
	for (uint32 i = 0; i < nf; ++i)
	{
		SoftBodySharedSettings::Face f;
		for (uint32 &vertex : f.mVertex)
			vertex = r.Int(nv);
		f.mMaterialIndex = r.Int(3);
		outSettings.mFaces.push_back(f);
	}
	uint32 ne = r.Int(5);
	for (uint32 i = 0; i < ne; ++i)
	{
		SoftBodySharedSettings::Edge e;
		for (uint32 &vertex : e.mVertex)
			vertex = r.Int(nv);
		e.mRestLength = r.Float();
		e.mCompliance = r.Float();
		outSettings.mEdgeConstraints.push_back(e);
	}
	uint32 nd = r.Int(4);
	for (uint32 i = 0; i < nd; ++i)
	{
		SoftBodySharedSettings::DihedralBend d;
		for (uint32 &vertex : d.mVertex)
			vertex = r.Int(nv);
		d.mCompliance = r.Float();
		d.mInitialAngle = r.Float();
		outSettings.mDihedralBendConstraints.push_back(d);
	}
	uint32 nvol = r.Int(4);
	for (uint32 i = 0; i < nvol; ++i)
	{
		SoftBodySharedSettings::Volume v;
		for (uint32 &vertex : v.mVertex)
			vertex = r.Int(nv);
		v.mSixRestVolume = r.Float();
		v.mCompliance = r.Float();
		outSettings.mVolumeConstraints.push_back(v);
	}
	uint32 ns = r.Int(4);
	for (uint32 i = 0; i < ns; ++i)
	{
		SoftBodySharedSettings::Skinned s;
		s.mVertex = r.Int(nv);
		for (SoftBodySharedSettings::SkinWeight &w : s.mWeights)
		{
			w.mInvBindIndex = r.Int(4);
			w.mWeight = r.Float();
		}
		s.mMaxDistance = r.Float();
		s.mBackStopDistance = r.Float();
		s.mBackStopRadius = r.Float();
		s.mNormalInfo = r.Next();
		outSettings.mSkinnedConstraints.push_back(s);
	}
	uint32 nn = r.Int(5);
	for (uint32 i = 0; i < nn; ++i)
		outNormals.push_back(r.Next());
	uint32 nl = r.Int(4);
	for (uint32 i = 0; i < nl; ++i)
	{
		SoftBodySharedSettings::LRA l;
		for (uint32 &vertex : l.mVertex)
			vertex = r.Int(nv);
		l.mMaxDistance = r.Float();
		outSettings.mLRAConstraints.push_back(l);
	}
	uint32 nr = r.Int(4);
	for (uint32 i = 0; i < nr; ++i)
	{
		SoftBodySharedSettings::RodStretchShear rod;
		for (uint32 &vertex : rod.mVertex)
			vertex = r.Int(nv);
		rod.mLength = r.Float();
		rod.mInvMass = r.Float();
		rod.mCompliance = r.Float();
		rod.mBishop = r.RandomQuat();
		outSettings.mRodStretchShearConstraints.push_back(rod);
	}
	uint32 nb = r.Int(4);
	for (uint32 i = 0; i < nb; ++i)
	{
		SoftBodySharedSettings::RodBendTwist rod;
		for (uint32 &index : rod.mRod)
			index = r.Int(nr + 1);
		rod.mCompliance = r.Float();
		rod.mOmega0 = r.RandomQuat();
		outSettings.mRodBendTwistConstraints.push_back(rod);
	}
	uint32 ni = r.Int(4);
	for (uint32 i = 0; i < ni; ++i)
	{
		SoftBodySharedSettings::InvBind b;
		b.mJointIndex = r.Next();
		Vec4 c[4];
		for (Vec4 &column : c)
		{
			float x = r.Float();
			float y = r.Float();
			float z = r.Float();
			float w = r.Float();
			column = Vec4(x, y, z, w);
		}
		b.mInvBind = Mat44(c[0], c[1], c[2], c[3]);
		outSettings.mInvBindMatrices.push_back(b);
	}
	uint32 ng = 1 + r.Int(3); // At least one update group: SoftBodyMotionProperties::Initialize needs optimized settings in Zolt (UB2)
	for (uint32 i = 0; i < 7 * ng; ++i)
		outGroups.push_back(r.Next());
}

/// The arrays that SaveBinaryState writes before the rod constraints, with the private arrays given as values
static void SBWritePrefix(StreamOut &ioStream, const SoftBodySharedSettings &inSettings, const Array<uint32> &inNormals, const Array<uint32> &inGroups)
{
	ioStream.Write(inSettings.mVertices);
	ioStream.Write(inSettings.mFaces);
	ioStream.Write(inSettings.mEdgeConstraints);
	ioStream.Write(inSettings.mDihedralBendConstraints);
	ioStream.Write(inSettings.mVolumeConstraints);
	ioStream.Write(inSettings.mSkinnedConstraints);
	ioStream.Write(inNormals);
	ioStream.Write(inSettings.mLRAConstraints);
	ioStream.Write(uint32(inGroups.size() / 7)); // Array<UpdateGroup>: the count, then the raw groups of 7 uint32
	for (uint32 v : inGroups)
		ioStream.Write(v);
}

/// The SaveBinaryState bytes of random settings. Jolt has no public access to its private arrays, so the stream of
/// SaveBinaryState is completed: the arrays up to the update groups are written again with the private arrays filled,
/// followed by the bytes that SaveBinaryState wrote for the rods and the inverse bind matrices. RestoreBinaryState of
/// that stream into new settings, then SaveBinaryState, gives the bytes (the same bytes, checked).
static bool SBRandomBytes(uint32 inSeed, std::string &outBytes)
{
	SoftBodySharedSettings settings;
	Array<uint32> normals, groups;
	SBGenerateRandom(inSeed, settings, normals, groups);

	// SaveBinaryState with empty private arrays, and the length of its prefix
	std::stringstream saved;
	{
		StreamOutWrapper out(saved);
		settings.SaveBinaryState(out);
	}
	std::stringstream empty_prefix;
	{
		StreamOutWrapper out(empty_prefix);
		SBWritePrefix(out, settings, {}, {});
	}
	std::string saved_bytes = saved.str();
	std::string empty_prefix_bytes = empty_prefix.str();
	if (saved_bytes.compare(0, empty_prefix_bytes.size(), empty_prefix_bytes) != 0)
		return false;

	// The complete stream
	std::stringstream complete;
	{
		StreamOutWrapper out(complete);
		SBWritePrefix(out, settings, normals, groups);
	}
	std::string complete_bytes = complete.str() + saved_bytes.substr(empty_prefix_bytes.size());

	// Restored and saved by Jolt
	Ref<SoftBodySharedSettings> restored = new SoftBodySharedSettings;
	{
		std::stringstream data(complete_bytes);
		StreamInWrapper in(data);
		restored->RestoreBinaryState(in);
		if (in.IsFailed())
			return false;
	}
	std::stringstream resaved;
	{
		StreamOutWrapper out(resaved);
		restored->SaveBinaryState(out);
	}
	outBytes = resaved.str();
	return outBytes == complete_bytes;
}

static Ref<SoftBodySharedSettings> SBCreateCloth(uint inGridSize, float inSpacing, SoftBodySharedSettings::EBendType inBendType, const SoftBodySharedSettings::VertexAttributes &inAttributes)
{
	// SoftBodyCreator::CreateClothWithFixatedCorners (Samples/Utils/SoftBodyCreator.cpp) without its std::function arguments
	const float offset = -0.5f * inSpacing * (inGridSize - 1);
	Ref<SoftBodySharedSettings> settings = new SoftBodySharedSettings;
	for (uint z = 0; z < inGridSize; ++z)
		for (uint x = 0; x < inGridSize; ++x)
		{
			SoftBodySharedSettings::Vertex v;
			Vec3(offset + x * inSpacing, 0.0f, offset + z * inSpacing).StoreFloat3(&v.mPosition);
			bool corner = (x == 0 || x == inGridSize - 1) && (z == 0 || z == inGridSize - 1);
			v.mInvMass = corner? 0.0f : 1.0f;
			settings->mVertices.push_back(v);
		}
	for (uint z = 0; z < inGridSize - 1; ++z)
		for (uint x = 0; x < inGridSize - 1; ++x)
		{
			SoftBodySharedSettings::Face f;
			f.mVertex[0] = x + z * inGridSize;
			f.mVertex[1] = x + (z + 1) * inGridSize;
			f.mVertex[2] = x + 1 + (z + 1) * inGridSize;
			settings->AddFace(f);
			f.mVertex[1] = x + 1 + (z + 1) * inGridSize;
			f.mVertex[2] = x + 1 + z * inGridSize;
			settings->AddFace(f);
		}
	settings->CreateConstraints(&inAttributes, 1, inBendType);
	settings->Optimize();
	return settings;
}

static Ref<SoftBodySharedSettings> SBCreateTetrahedron(SoftBodySharedSettings::EBendType inBendType)
{
	Ref<SoftBodySharedSettings> settings = new SoftBodySharedSettings;
	const float p[4][3] = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } };
	for (const float *q : p)
	{
		SoftBodySharedSettings::Vertex v;
		v.mPosition = Float3(q[0], q[1], q[2]);
		settings->mVertices.push_back(v);
	}
	const uint32 faces[4][3] = { { 0, 1, 2 }, { 0, 3, 1 }, { 0, 2, 3 }, { 1, 3, 2 } };
	for (const uint32 *f : faces)
		settings->AddFace(SoftBodySharedSettings::Face(f[0], f[1], f[2]));
	SoftBodySharedSettings::VertexAttributes attributes(0.0f, 0.0f, inBendType == SoftBodySharedSettings::EBendType::None? FLT_MAX : 0.0f);
	settings->CreateConstraints(&attributes, 1, inBendType);
	return settings;
}

static Ref<SoftBodySharedSettings> SBCreateSettings(SBSettingsKind inKind, uint32 inSeed)
{
	switch (inKind)
	{
	case SBSettingsKind::Random:
		break;
	case SBSettingsKind::Cube:
		return SoftBodySharedSettings::sCreateCube(2 + inSeed % 7, 0.05f + float(inSeed % 5) * 0.05f);
	case SBSettingsKind::Cloth:
		return SBCreateCloth(2 + inSeed % 12, 0.1f, SoftBodySharedSettings::EBendType::Dihedral, SoftBodySharedSettings::VertexAttributes(1.0e-5f, 1.0e-5f, 1.0e-5f, SoftBodySharedSettings::ELRAType::EuclideanDistance, 1.1f));
	case SBSettingsKind::Tetrahedron:
		return SBCreateTetrahedron(SoftBodySharedSettings::EBendType::None);
	case SBSettingsKind::TetrahedronDistanceBend:
		return SBCreateTetrahedron(SoftBodySharedSettings::EBendType::Distance);
	}
	return nullptr;
}

/// Copy a string into a caller buffer, returns the size or 0xFFFFFFFF if it does not fit
static uint32 SBCopyBytes(const std::string &inBytes, uint8 *outBytes, uint32 inCapacity)
{
	if (inBytes.size() > inCapacity)
		return 0xFFFFFFFF;
	memcpy(outBytes, inBytes.data(), inBytes.size());
	return uint32(inBytes.size());
}

// Must match CreateDesc in SoftBodyParity.zig (the fields of SoftBodyCreationSettings)
struct SBCreateDesc
{
	double				mPosition[3];
	uint64				mUserData;
	float				mRotation[4];
	float				mLinearDamping;
	float				mMaxLinearVelocity;
	float				mRestitution;
	float				mFriction;
	float				mPressure;
	float				mGravityFactor;
	float				mVertexRadius;
	uint32				mObjectLayer;
	uint32				mNumIterations;
	uint32				mUpdatePosition;
	uint32				mMakeRotationIdentity;
	uint32				mAllowSleeping;
	uint32				mFacesDoubleSided;
	uint32				mGroupID;
	uint32				mSubGroupID;
	uint32				mPad;
};

// Must match Step in SoftBodyParity.zig: the bytes of the shared settings (RestoreBinaryState) and the other fields of
// the creation settings of one step
struct SBStep
{
	const uint8 *		mSettingsBytes;
	uint32				mSize;
	uint32				mPad;
	SBCreateDesc		mDesc;
};

// Must match BodyResult in SoftBodyParity.zig
struct SBBodyResult
{
	double				mPosition[3];
	double				mCenterOfMassPosition[3];
	uint64				mUserData;
	float				mRotation[4];
	float				mWorldBounds[6];
	float				mLocalBounds[6];
	float				mLocalPredictedBounds[6];
	float				mInvMass;
	float				mInvInertiaDiagonal[3];
	float				mInertiaRotation[4];
	float				mLinearDamping;
	float				mAngularDamping;
	float				mMaxLinearVelocity;
	float				mMaxAngularVelocity;
	float				mGravityFactor;
	float				mFriction;
	float				mRestitution;
	float				mPressure;
	float				mVertexRadius;
	float				mVolume;
	uint32				mObjectLayer;
	uint32				mBroadPhaseLayer;
	uint32				mMotionType;
	uint32				mMotionQuality;
	uint32				mAllowSleeping;
	uint32				mIsSoftBody;
	uint32				mNumIterations;
	uint32				mUpdatePosition;
	uint32				mFacesDoubleSided;
	uint32				mGroupID;
	uint32				mSubGroupID;
	uint32				mNumVertices;
	uint32				mNumRods;					///< The rods of the shared settings of the last step
	uint32				mSettingsRefCount;			///< The reference count of the shared settings of the last step
	uint32				mShapeSubType;
	uint32				mEnableSkinConstraints;
	uint32				mNumRodStates;				///< The rod states of the motion properties (mRodStates)
	uint32				mNumSkinStates;				///< The skin states of the motion properties (mSkinState)
	uint32				mFirstSettingsRefCount;		///< The reference count of the shared settings of the first step
	uint32				mPad;
};

// Must match VertexResult in SoftBodyParity.zig (SoftBodyMotionProperties::Vertex)
struct SBVertexResult
{
	float				mPreviousPosition[3];
	float				mPosition[3];
	float				mVelocity[3];
	float				mCollisionPlane[4];
	int32				mCollidingShapeIndex;
	uint32				mHasContact;
	float				mLargestPenetration;
	float				mInvMass;
};

// Must match RodResult in SoftBodyParity.zig
struct SBRodResult
{
	float				mRotation[4];
	float				mAngularVelocity[3];
};

// Must match SkinResult in SoftBodyParity.zig (SoftBodyMotionProperties::SkinState)
struct SBSkinResult
{
	float				mPreviousPosition[3];
	float				mPosition[3];
	float				mNormal[3];
};

static void SBStore(Vec3Arg inV, float *outV)
{
	outV[0] = inV.GetX();
	outV[1] = inV.GetY();
	outV[2] = inV.GetZ();
}

static void SBStore(QuatArg inQ, float *outQ)
{
	outQ[0] = inQ.GetX();
	outQ[1] = inQ.GetY();
	outQ[2] = inQ.GetZ();
	outQ[3] = inQ.GetW();
}

static void SBStore(RVec3Arg inV, double *outV)
{
	outV[0] = double(inV.GetX());
	outV[1] = double(inV.GetY());
	outV[2] = double(inV.GetZ());
}

/// The creation settings of a step, with the shared settings that RestoreBinaryState makes of the step's bytes
static SoftBodyCreationSettings SBCreationSettings(SoftBodySharedSettings *ioShared, const SBStep &inStep)
{
	{
		std::stringstream data(std::string(reinterpret_cast<const char *>(inStep.mSettingsBytes), inStep.mSize));
		StreamInWrapper in(data);
		ioShared->RestoreBinaryState(in);
	}
	const SBCreateDesc *desc = &inStep.mDesc;
	SoftBodyCreationSettings settings(ioShared, RVec3(Real(desc->mPosition[0]), Real(desc->mPosition[1]), Real(desc->mPosition[2])), Quat(desc->mRotation[0], desc->mRotation[1], desc->mRotation[2], desc->mRotation[3]), ObjectLayer(desc->mObjectLayer));
	settings.mUserData = desc->mUserData;
	settings.mCollisionGroup = CollisionGroup(nullptr, desc->mGroupID, desc->mSubGroupID);
	settings.mNumIterations = desc->mNumIterations;
	settings.mLinearDamping = desc->mLinearDamping;
	settings.mMaxLinearVelocity = desc->mMaxLinearVelocity;
	settings.mRestitution = desc->mRestitution;
	settings.mFriction = desc->mFriction;
	settings.mPressure = desc->mPressure;
	settings.mGravityFactor = desc->mGravityFactor;
	settings.mVertexRadius = desc->mVertexRadius;
	settings.mUpdatePosition = desc->mUpdatePosition != 0;
	settings.mMakeRotationIdentity = desc->mMakeRotationIdentity != 0;
	settings.mAllowSleeping = desc->mAllowSleeping != 0;
	settings.mFacesDoubleSided = desc->mFacesDoubleSided != 0;
	return settings;
}

extern "C"
{

/// SaveBinaryState of the settings of kind inKind (SBSettingsKind) with seed inSeed. Returns the number of bytes
/// (0xFFFFFFFF if they don't fit or the random stream could not be made). outCounts receives the array sizes: vertices,
/// faces, edges, dihedral bends, volumes, skinned, inv binds, LRAs, rod stretch shear, rod bend twist and 0 (Jolt's
/// update groups are private).
uint32 jolt_sb_settings_bytes(uint32 inKind, uint32 inSeed, uint8 *outBytes, uint32 inCapacity, uint32 *outCounts)
{
	ParityInstallHooks();
	if (SBSettingsKind(inKind) == SBSettingsKind::Random)
	{
		std::string bytes;
		if (!SBRandomBytes(inSeed, bytes))
			return 0xFFFFFFFF;
		memset(outCounts, 0, 11 * sizeof(uint32));
		return SBCopyBytes(bytes, outBytes, inCapacity);
	}

	Ref<SoftBodySharedSettings> s = SBCreateSettings(SBSettingsKind(inKind), inSeed);
	const uint32 counts[] = {
		uint32(s->mVertices.size()), uint32(s->mFaces.size()), uint32(s->mEdgeConstraints.size()),
		uint32(s->mDihedralBendConstraints.size()), uint32(s->mVolumeConstraints.size()), uint32(s->mSkinnedConstraints.size()),
		uint32(s->mInvBindMatrices.size()), uint32(s->mLRAConstraints.size()), uint32(s->mRodStretchShearConstraints.size()),
		uint32(s->mRodBendTwistConstraints.size()), 0 };
	memcpy(outCounts, counts, sizeof(counts));
	std::stringstream data;
	{
		StreamOutWrapper stream(data);
		s->SaveBinaryState(stream);
	}
	return SBCopyBytes(data.str(), outBytes, inCapacity);
}

/// RestoreBinaryState of inBytes into new settings, then SaveBinaryState. outStreamState receives the state of the
/// input stream after the restore: bit 0 = IsEOF, bit 1 = IsFailed. Returns the number of bytes (0xFFFFFFFF if they
/// don't fit).
uint32 jolt_sb_resave(const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity, uint32 *outStreamState)
{
	ParityInstallHooks();
	Ref<SoftBodySharedSettings> s = new SoftBodySharedSettings;
	{
		std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
		StreamInWrapper in(data);
		s->RestoreBinaryState(in);
		*outStreamState = (in.IsEOF()? 1u : 0u) | (in.IsFailed()? 2u : 0u);
	}
	std::stringstream data;
	{
		StreamOutWrapper out(data);
		s->SaveBinaryState(out);
	}
	return SBCopyBytes(data.str(), outBytes, inCapacity);
}

/// BodyInterface::CreateSoftBody in a new PhysicsSystem (the layers of UnitTests/Layers.h) with the creation settings
/// of step 0, then Body::ApplySoftBodyCreationSettings on the created body with those of every other step. The shared
/// settings of a step are what RestoreBinaryState makes of its bytes. The results are the state after the last step,
/// then the body is destroyed. Returns the number of vertices (0xFFFFFFFF if the vertices, rod states or skin states
/// don't fit or no body was created).
uint32 jolt_sb_create_body(const SBStep *inSteps, uint32 inNumSteps, SBBodyResult *outResult, SBVertexResult *outVertices, uint32 inVertexCapacity, SBRodResult *outRods, uint32 inRodCapacity, SBSkinResult *outSkinStates, uint32 inSkinCapacity)
{
	ParityInstallHooks();

	// The creation settings of every step: only they hold a reference to their shared settings (like the Zolt side)
	Array<SoftBodySharedSettings *> shared;
	Array<SoftBodyCreationSettings> settings;
	for (uint32 s = 0; s < inNumSteps; ++s)
	{
		shared.push_back(new SoftBodySharedSettings);
		settings.push_back(SBCreationSettings(shared.back(), inSteps[s]));
	}

	BPLayerInterfaceImpl layers;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broad_phase;
	ObjectLayerPairFilterImpl pairs;
	PhysicsSystem system;
	system.Init(4, 0, 4, 4, layers, object_vs_broad_phase, pairs);
	BodyInterface &bi = system.GetBodyInterfaceNoLock();
	Body *body = bi.CreateSoftBody(settings[0]);
	if (body == nullptr)
		return 0xFFFFFFFF;
	for (uint32 s = 1; s < inNumSteps; ++s)
		body->ApplySoftBodyCreationSettings(settings[s], layers);

	const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(body->GetMotionProperties());
	const auto &rod_states = mp->*SBGet(SBRodStates());
	const auto &skin_state = mp->*SBGet(SBSkinState());
	const AABox &local_predicted_bounds = mp->*SBGet(SBLocalPredictedBounds());
	SBBodyResult &r = *outResult;
	memset(&r, 0, sizeof(r));
	SBStore(body->GetPosition(), r.mPosition);
	SBStore(body->GetCenterOfMassPosition(), r.mCenterOfMassPosition);
	r.mUserData = body->GetUserData();
	SBStore(body->GetRotation(), r.mRotation);
	SBStore(body->GetWorldSpaceBounds().mMin, r.mWorldBounds);
	SBStore(body->GetWorldSpaceBounds().mMax, r.mWorldBounds + 3);
	SBStore(mp->GetLocalBounds().mMin, r.mLocalBounds);
	SBStore(mp->GetLocalBounds().mMax, r.mLocalBounds + 3);
	SBStore(local_predicted_bounds.mMin, r.mLocalPredictedBounds);
	SBStore(local_predicted_bounds.mMax, r.mLocalPredictedBounds + 3);
	r.mInvMass = mp->GetInverseMassUnchecked();
	SBStore(mp->GetInverseInertiaDiagonal(), r.mInvInertiaDiagonal);
	SBStore(mp->GetInertiaRotation(), r.mInertiaRotation);
	r.mLinearDamping = mp->GetLinearDamping();
	r.mAngularDamping = mp->GetAngularDamping();
	r.mMaxLinearVelocity = mp->GetMaxLinearVelocity();
	r.mMaxAngularVelocity = mp->GetMaxAngularVelocity();
	r.mGravityFactor = mp->GetGravityFactor();
	r.mFriction = body->GetFriction();
	r.mRestitution = body->GetRestitution();
	r.mPressure = mp->GetPressure();
	r.mVertexRadius = mp->GetVertexRadius();
	r.mVolume = mp->GetVolume();
	r.mObjectLayer = uint32(body->GetObjectLayer());
	r.mBroadPhaseLayer = uint32(BroadPhaseLayer::Type(body->GetBroadPhaseLayer()));
	r.mMotionType = uint32(body->GetMotionType());
	r.mMotionQuality = uint32(mp->GetMotionQuality());
	r.mAllowSleeping = mp->GetAllowSleeping()? 1 : 0;
	r.mIsSoftBody = body->IsSoftBody()? 1 : 0;
	r.mNumIterations = mp->GetNumIterations();
	r.mUpdatePosition = mp->GetUpdatePosition()? 1 : 0;
	r.mFacesDoubleSided = mp->GetFacesDoubleSided()? 1 : 0;
	r.mGroupID = body->GetCollisionGroup().GetGroupID();
	r.mSubGroupID = body->GetCollisionGroup().GetSubGroupID();
	r.mNumVertices = uint32(mp->GetVertices().size());
	r.mNumRods = uint32(shared.back()->mRodStretchShearConstraints.size());
	r.mSettingsRefCount = shared.back()->GetRefCount();
	r.mShapeSubType = uint32(body->GetShape()->GetSubType());
	r.mEnableSkinConstraints = mp->GetEnableSkinConstraints()? 1 : 0;
	r.mNumRodStates = uint32(rod_states.size());
	r.mNumSkinStates = uint32(skin_state.size());
	r.mFirstSettingsRefCount = shared.front()->GetRefCount();

	uint32 result = r.mNumVertices;
	if (r.mNumVertices > inVertexCapacity || r.mNumRodStates > inRodCapacity || r.mNumSkinStates > inSkinCapacity)
		result = 0xFFFFFFFF;
	else
	{
		for (uint32 i = 0; i < r.mNumVertices; ++i)
		{
			const SoftBodyMotionProperties::Vertex &v = mp->GetVertex(i);
			SBVertexResult &o = outVertices[i];
			memset(&o, 0, sizeof(o));
			SBStore(v.mPreviousPosition, o.mPreviousPosition);
			SBStore(v.mPosition, o.mPosition);
			SBStore(v.mVelocity, o.mVelocity);
			SBStore(v.mCollisionPlane.GetNormal(), o.mCollisionPlane);
			o.mCollisionPlane[3] = v.mCollisionPlane.GetConstant();
			o.mCollidingShapeIndex = v.mCollidingShapeIndex;
			o.mHasContact = v.mHasContact? 1 : 0;
			o.mLargestPenetration = v.mLargestPenetration;
			o.mInvMass = v.mInvMass;
		}
		for (uint32 i = 0; i < r.mNumRodStates; ++i)
		{
			SBRodResult &o = outRods[i];
			SBStore(mp->GetRodRotation(i), o.mRotation);
			SBStore(mp->GetRodAngularVelocity(i), o.mAngularVelocity);
		}
		for (uint32 i = 0; i < r.mNumSkinStates; ++i)
		{
			SBSkinResult &o = outSkinStates[i];
			SBStore(skin_state[i].mPreviousPosition, o.mPreviousPosition);
			SBStore(skin_state[i].mPosition, o.mPosition);
			SBStore(skin_state[i].mNormal, o.mNormal);
		}
	}

	bi.DestroyBody(body->GetID());
	return result;
}

/// The number of Jolt asserts that failed in this process (always 0 without JPH_ENABLE_ASSERTS)
uint32 jolt_sb_reference_assert_count()
{
	return ParityAssertCount();
}

/// 1 when the reference library is built with JPH_ENABLE_ASSERTS (`zig build parity -Dreference_asserts=true`)
uint32 jolt_sb_reference_has_asserts()
{
#ifdef JPH_ENABLE_ASSERTS
	return 1;
#else
	return 0;
#endif
}

/// Fails a JPH_ASSERT on purpose (the test of the assert hook): the number of failed asserts before and after it
void jolt_sb_test_assert_hook(uint32 *outBefore, uint32 *outAfter)
{
	ParityInstallHooks();
	*outBefore = jolt_sb_reference_assert_count();
	JPH_ASSERT(false, "SoftBodyParity: the test of the assert hook (expected)");
	*outAfter = jolt_sb_reference_assert_count();
}

} // extern "C"
