// Reference implementation for the Skeleton parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library's Skeleton, SkeletalAnimation, SkeletonPose and SkeletonMapper. ZoltParity/Physics/SkeletonParity.zig calls
// these and checks that Zolt produces the same bits.
//
// A skeleton is a SkelDesc: a list of AddJoint calls (by parent name, with the default parent name or by parent index)
// and whether CalculateParentJointIndices follows. An animation is a SkelAnimDesc (animated joints with keyframes). The
// structs below must match the extern structs in SkeletonParity.zig. Booleans are passed as uint32 / uint8 (never bool,
// see the porting guide). All C functions have the prefix jolt_skel_. Functions that produce bytes write into a caller
// provided buffer and return the size.

#include <Jolt/Jolt.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/SkeletonPose.h>
#include <Jolt/Skeleton/SkeletonMapper.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

// Jolt allocates through the Allocate / Free hooks, which RegisterDefaultAllocator sets. Every function that allocates
// calls it first (like the other reference files), so the tests do not depend on which test runs first.
static void SkelEnsureAllocator()
{
	RegisterDefaultAllocator();
}

static constexpr uint32 cSkelMaxJoints = 40;
static constexpr uint32 cSkelMaxName = 8;
static constexpr uint32 cSkelMaxAnimatedJoints = 12;
static constexpr uint32 cSkelMaxKeyframes = 12;

// Must match SkelJointDesc in SkeletonParity.zig
struct SkelJointDesc
{
	uint32				mKind;							///< 0: AddJoint(name, parent name), 1: AddJoint(name) (default parent name), 2: AddJoint(name, parent index)
	int32				mParentIndex;					///< Kind 2
	uint32				mNameLength;
	uint32				mParentNameLength;
	char				mName[cSkelMaxName];
	char				mParentName[cSkelMaxName];
};

// Must match SkelDesc in SkeletonParity.zig
struct SkelDesc
{
	uint32				mNumJoints;
	uint32				mCalculateParentJointIndices;	///< Call CalculateParentJointIndices after adding the joints
	SkelJointDesc		mJoints[cSkelMaxJoints];
};

// Must match SkelName in SkeletonParity.zig
struct SkelName
{
	uint32				mLength;
	char				mName[cSkelMaxName];
};

// Must match SkelResult in SkeletonParity.zig
struct SkelResult
{
	uint32				mAddJointResults[cSkelMaxJoints];
	int32				mParentJointIndices[cSkelMaxJoints];
	uint32				mJointCount;
	uint32				mCorrectlyOrdered;
};

// Must match SkelKeyframe in SkeletonParity.zig
struct SkelKeyframe
{
	float				mTime;
	float				mRotation[4];
	float				mTranslation[3];
};

// Must match SkelAnimatedJoint in SkeletonParity.zig
struct SkelAnimatedJoint
{
	uint32				mNameLength;
	char				mName[cSkelMaxName];
	uint32				mNumKeyframes;
	SkelKeyframe		mKeyframes[cSkelMaxKeyframes];
};

// Must match SkelAnimDesc in SkeletonParity.zig
struct SkelAnimDesc
{
	uint32				mNumJoints;
	uint32				mIsLooping;
	SkelAnimatedJoint	mJoints[cSkelMaxAnimatedJoints];
};

// Must match SkelPoseResult in SkeletonParity.zig
struct SkelPoseResult
{
	float				mJointMatrices[cSkelMaxJoints][16];		///< CalculateJointMatrices
	float				mLocalMatrices[cSkelMaxJoints][16];		///< CalculateLocalSpaceJointMatrices
	float				mJointStates[cSkelMaxJoints][7];		///< CalculateJointStates of the input matrices
	float				mResizedStates[cSkelMaxJoints][7];		///< The joint states after SetSkeleton(skeleton 2)
	float				mFromMatrixStates[cSkelMaxJoints][7];	///< JointState::FromMatrix of every input matrix
	Real				mRootOffset[3];
	uint32				mJointCount;
	uint32				mResizedJointCount;
	uint32				mResizedMatrixCount;
};

// Must match SkelMapperDesc in SkeletonParity.zig
struct SkelMapperDesc
{
	SkelDesc			mSkeleton1;
	SkelDesc			mSkeleton2;
	float				mNeutralPose1[cSkelMaxJoints][16];
	float				mNeutralPose2[cSkelMaxJoints][16];
	uint32				mCanMapMode;							///< 0: the default argument (sDefaultCanMapJoint), 1: SkelCanMapJoint with mCanMapTable
	uint32				mLockMode;								///< Bit 0: LockTranslations(mLocked), bit 1: LockAllTranslations (when there are mappings), in that order
	uint32				mMap;									///< Call Map (no locked joint without a parent)
	uint8				mCanMapTable[cSkelMaxJoints * cSkelMaxJoints]; ///< [j1 * n2 + j2]: 0 no, 1 if the names are equal, 2 yes
	uint8				mLocked[cSkelMaxJoints];
	float				mPose1ModelSpace[cSkelMaxJoints][16];
	float				mPose2LocalSpace[cSkelMaxJoints][16];
	float				mPose2ModelSpace[cSkelMaxJoints][16];
	float				mSentinel[16];							///< Written to the outputs of Map / MapReverse before the call
};

// Must match SkelMapperResult in SkeletonParity.zig
struct SkelMapperResult
{
	uint32				mNumMappings;
	uint32				mNumChains;
	uint32				mNumUnmapped;
	uint32				mNumLocked;
	int32				mMappingJoints[cSkelMaxJoints][2];
	float				mMappingMatrices[cSkelMaxJoints][2][16];
	uint32				mChainLengths[cSkelMaxJoints][2];
	int32				mChainJoints1[cSkelMaxJoints][cSkelMaxJoints];
	int32				mChainJoints2[cSkelMaxJoints][cSkelMaxJoints];
	int32				mUnmapped[cSkelMaxJoints][2];
	int32				mLockedJoints[2 * cSkelMaxJoints][2];
	float				mLockedTranslations[2 * cSkelMaxJoints][3];
	int32				mMappedJointIdx[cSkelMaxJoints + 2];	///< GetMappedJointIdx(j1) for j1 = -1 .. n1
	uint32				mIsLocked[cSkelMaxJoints + 2];			///< IsJointTranslationLocked(j2) for j2 = -1 .. n2
	float				mPose2ModelSpace[cSkelMaxJoints][16];	///< Map
	float				mPose1ModelSpace[cSkelMaxJoints][16];	///< MapReverse
};

static Vec4 SkelLoad4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Mat44 SkelLoadMat44(const float *inM)			{ return Mat44(SkelLoad4(inM), SkelLoad4(inM + 4), SkelLoad4(inM + 8), SkelLoad4(inM + 12)); }
static void SkelStoreMat44(Mat44Arg inM, float *outM)	{ for (int c = 0; c < 4; ++c) inM.GetColumn4(c).StoreFloat4(reinterpret_cast<Float4 *>(outM + 4 * c)); }

static void SkelLoadState(const float *inV, SkeletalAnimation::JointState &outState)
{
	outState.mRotation = Quat(inV[0], inV[1], inV[2], inV[3]);
	outState.mTranslation = Vec3(inV[4], inV[5], inV[6]);
}

static void SkelStoreState(const SkeletalAnimation::JointState &inState, float *outV)
{
	outV[0] = inState.mRotation.GetX();
	outV[1] = inState.mRotation.GetY();
	outV[2] = inState.mRotation.GetZ();
	outV[3] = inState.mRotation.GetW();
	outV[4] = inState.mTranslation.GetX();
	outV[5] = inState.mTranslation.GetY();
	outV[6] = inState.mTranslation.GetZ();
}

// The skeleton of a description, the results of the AddJoint calls in outAddJointResults (when not null)
static Ref<Skeleton> SkelMakeSkeleton(const SkelDesc &inDesc, uint32 *outAddJointResults)
{
	Ref<Skeleton> skeleton = new Skeleton;
	for (uint32 j = 0; j < inDesc.mNumJoints; ++j)
	{
		const SkelJointDesc &d = inDesc.mJoints[j];
		string_view name(d.mName, d.mNameLength);
		uint result;
		switch (d.mKind)
		{
		case 0:		result = skeleton->AddJoint(name, string_view(d.mParentName, d.mParentNameLength)); break;
		case 1:		result = skeleton->AddJoint(name); break;
		default:	result = skeleton->AddJoint(name, d.mParentIndex); break;
		}
		if (outAddJointResults != nullptr)
			outAddJointResults[j] = result;
	}
	if (inDesc.mCalculateParentJointIndices != 0)
		skeleton->CalculateParentJointIndices();
	return skeleton;
}

static Ref<SkeletalAnimation> SkelMakeAnimation(const SkelAnimDesc &inDesc)
{
	Ref<SkeletalAnimation> animation = new SkeletalAnimation;
	animation->SetIsLooping(inDesc.mIsLooping != 0);
	for (uint32 j = 0; j < inDesc.mNumJoints; ++j)
	{
		const SkelAnimatedJoint &d = inDesc.mJoints[j];
		SkeletalAnimation::AnimatedJoint aj;
		aj.mJointName = String(d.mName, d.mNameLength);
		for (uint32 k = 0; k < d.mNumKeyframes; ++k)
		{
			const SkelKeyframe &kd = d.mKeyframes[k];
			SkeletalAnimation::Keyframe keyframe;
			keyframe.mTime = kd.mTime;
			keyframe.mRotation = Quat(kd.mRotation[0], kd.mRotation[1], kd.mRotation[2], kd.mRotation[3]);
			keyframe.mTranslation = Vec3(kd.mTranslation[0], kd.mTranslation[1], kd.mTranslation[2]);
			aj.mKeyframes.push_back(keyframe);
		}
		animation->GetAnimatedJoints().push_back(aj);
	}
	return animation;
}

// The bytes of a stream into a caller provided buffer (returns the size, copies at most inCapacity bytes)
static uint32 SkelCopyBytes(const std::string &inData, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inData.size());
	memcpy(outBytes, inData.data(), std::min(size, inCapacity));
	return size;
}

template <class T>
static std::string SkelSave(const T &inObject)
{
	std::stringstream data;
	StreamOutWrapper out(data);
	inObject.SaveBinaryState(out);
	return data.str();
}

// The can map joint function of mode 1: the table decides (0 no, 1 if the names are equal, 2 yes)
static bool SkelCanMapJoint(const SkelMapperDesc &inDesc, const Skeleton *inSkeleton1, int inIndex1, const Skeleton *inSkeleton2, int inIndex2)
{
	uint8 v = inDesc.mCanMapTable[inIndex1 * inSkeleton2->GetJointCount() + inIndex2];
	return v == 2 || (v == 1 && inSkeleton1->GetJoint(inIndex1).mName == inSkeleton2->GetJoint(inIndex2).mName);
}

extern "C"
{

// The sizes of the structs that are shared with SkeletonParity.zig
void jolt_skel_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = uint32(sizeof(SkelDesc));
	outSizes[1] = uint32(sizeof(SkelResult));
	outSizes[2] = uint32(sizeof(SkelAnimDesc));
	outSizes[3] = uint32(sizeof(SkelPoseResult));
	outSizes[4] = uint32(sizeof(SkelMapperDesc));
	outSizes[5] = uint32(sizeof(SkelMapperResult));
}

// Skeleton: the AddJoint calls of a description, CalculateParentJointIndices, GetJointIndex of the queries,
// AreJointsCorrectlyOrdered and SaveBinaryState (returns the size of the bytes)
uint32 jolt_skel_skeleton(const SkelDesc *inDesc, const SkelName *inQueries, uint32 inNumQueries, int32 *outJointIndices, SkelResult *outResult, uint8 *outBytes, uint32 inCapacity)
{
	SkelEnsureAllocator();
	Ref<Skeleton> skeleton = SkelMakeSkeleton(*inDesc, outResult->mAddJointResults);
	for (int j = 0; j < skeleton->GetJointCount(); ++j)
		outResult->mParentJointIndices[j] = skeleton->GetJoint(j).mParentJointIndex;
	outResult->mJointCount = uint32(skeleton->GetJointCount());
	outResult->mCorrectlyOrdered = skeleton->AreJointsCorrectlyOrdered()? 1 : 0;
	for (uint32 q = 0; q < inNumQueries; ++q)
		outJointIndices[q] = skeleton->GetJointIndex(string_view(inQueries[q].mName, inQueries[q].mLength));
	return SkelCopyBytes(SkelSave(*skeleton), outBytes, inCapacity);
}

// Skeleton::sRestoreFromBinaryState of the bytes, then SaveBinaryState of the result. Returns the size of the saved
// bytes, or 0xFFFFFFFF and the error text in outError.
uint32 jolt_skel_skeleton_resave(const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity, char *outError, uint32 inErrorCapacity)
{
	SkelEnsureAllocator();
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(data);
	Skeleton::SkeletonResult result = Skeleton::sRestoreFromBinaryState(in);
	if (result.HasError())
	{
		const String &error = result.GetError();
		size_t n = std::min(size_t(inErrorCapacity - 1), error.size());
		memcpy(outError, error.data(), n);
		outError[n] = 0;
		return 0xFFFFFFFF;
	}
	return SkelCopyBytes(SkelSave(*result.Get()), outBytes, inCapacity);
}

// SkeletalAnimation: a pose of the skeleton starts with inInitialStates (7 floats per joint: rotation, translation),
// then Sample at every time (outStates gets the states of all joints after each sample), ScaleJoints(inScale) and
// Sample at every time again (the second half of outStates), GetDuration before and after scaling, IsLooping and
// SaveBinaryState after scaling (returns the size of the bytes)
uint32 jolt_skel_animation(const SkelDesc *inSkeleton, const SkelAnimDesc *inAnimation, const float *inInitialStates, const float *inTimes, uint32 inNumTimes, float inScale, float *outStates, float *outDurations, uint32 *outIsLooping, uint8 *outBytes, uint32 inCapacity)
{
	SkelEnsureAllocator();
	Ref<Skeleton> skeleton = SkelMakeSkeleton(*inSkeleton, nullptr);
	Ref<SkeletalAnimation> animation = SkelMakeAnimation(*inAnimation);

	SkeletonPose pose;
	pose.SetSkeleton(skeleton);
	uint n = pose.GetJointCount();
	for (uint j = 0; j < n; ++j)
		SkelLoadState(inInitialStates + 7 * j, pose.GetJoint(j));

	float *out = outStates;
	for (int pass = 0; pass < 2; ++pass)
	{
		outDurations[pass] = animation->GetDuration();
		for (uint32 t = 0; t < inNumTimes; ++t)
		{
			animation->Sample(inTimes[t], pose);
			for (uint j = 0; j < n; ++j, out += 7)
				SkelStoreState(pose.GetJoint(j), out);
		}
		animation->ScaleJoints(inScale);
	}

	*outIsLooping = animation->IsLooping()? 1 : 0;
	return SkelCopyBytes(SkelSave(*animation), outBytes, inCapacity);
}

// SkeletalAnimation::sRestoreFromBinaryState of the bytes (never an error), then SaveBinaryState of the result
uint32 jolt_skel_animation_resave(const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity)
{
	SkelEnsureAllocator();
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(data);
	SkeletalAnimation::AnimationResult result = SkeletalAnimation::sRestoreFromBinaryState(in);
	return SkelCopyBytes(SkelSave(*result.Get()), outBytes, inCapacity);
}

// SkeletonPose: SetSkeleton(skeleton 1), the joint states inStates (7 floats per joint) and SetRootOffset, the joint
// matrices inMatrices (16 floats per joint, read by CalculateJointMatrices for parents after their children),
// CalculateJointMatrices, CalculateLocalSpaceJointMatrices, then the joint matrices inMatrices again and
// CalculateJointStates, then SetSkeleton(skeleton 2) (the states that are kept and the new default states), and
// JointState::FromMatrix of every input matrix (cSkelMaxJoints of them)
void jolt_skel_pose(const SkelDesc *inSkeleton1, const SkelDesc *inSkeleton2, const float *inStates, const float *inMatrices, const Real *inRootOffset, SkelPoseResult *outResult)
{
	SkelEnsureAllocator();
	Ref<Skeleton> skeleton1 = SkelMakeSkeleton(*inSkeleton1, nullptr);
	Ref<Skeleton> skeleton2 = SkelMakeSkeleton(*inSkeleton2, nullptr);

	SkeletonPose pose;
	pose.SetSkeleton(skeleton1);
	uint n = pose.GetJointCount();
	outResult->mJointCount = n;
	for (uint j = 0; j < n; ++j)
	{
		SkelLoadState(inStates + 7 * j, pose.GetJoint(j));
		pose.GetJointMatrix(j) = SkelLoadMat44(inMatrices + 16 * j);
	}
	pose.SetRootOffset(RVec3(inRootOffset[0], inRootOffset[1], inRootOffset[2]));

	pose.CalculateJointMatrices();
	for (uint j = 0; j < n; ++j)
		SkelStoreMat44(pose.GetJointMatrix(j), outResult->mJointMatrices[j]);

	Mat44 local[cSkelMaxJoints];
	pose.CalculateLocalSpaceJointMatrices(local);
	for (uint j = 0; j < n; ++j)
		SkelStoreMat44(local[j], outResult->mLocalMatrices[j]);

	for (uint j = 0; j < n; ++j)
		pose.GetJointMatrices()[j] = SkelLoadMat44(inMatrices + 16 * j);
	pose.CalculateJointStates();
	for (uint j = 0; j < n; ++j)
		SkelStoreState(pose.GetJoint(j), outResult->mJointStates[j]);

	RVec3 offset = pose.GetRootOffset();
	outResult->mRootOffset[0] = offset.GetX();
	outResult->mRootOffset[1] = offset.GetY();
	outResult->mRootOffset[2] = offset.GetZ();

	pose.SetSkeleton(skeleton2);
	outResult->mResizedJointCount = pose.GetJointCount();
	outResult->mResizedMatrixCount = uint32(pose.GetJointMatrices().size());
	for (uint j = 0; j < pose.GetJointCount(); ++j)
		SkelStoreState(pose.GetJoint(j), outResult->mResizedStates[j]);

	for (uint j = 0; j < cSkelMaxJoints; ++j)
	{
		SkeletalAnimation::JointState state;
		state.FromMatrix(SkelLoadMat44(inMatrices + 16 * j));
		SkelStoreState(state, outResult->mFromMatrixStates[j]);
	}
}

// SkeletonMapper: Initialize (with the default argument or with SkelCanMapJoint), LockTranslations /
// LockAllTranslations, the mappings, chains, unmapped and locked joints, GetMappedJointIdx, IsJointTranslationLocked,
// Map and MapReverse
void jolt_skel_mapper(const SkelMapperDesc *inDesc, SkelMapperResult *outResult)
{
	SkelEnsureAllocator();
	Ref<Skeleton> skeleton1 = SkelMakeSkeleton(inDesc->mSkeleton1, nullptr);
	Ref<Skeleton> skeleton2 = SkelMakeSkeleton(inDesc->mSkeleton2, nullptr);
	int n1 = skeleton1->GetJointCount();
	int n2 = skeleton2->GetJointCount();

	Mat44 neutral1[cSkelMaxJoints], neutral2[cSkelMaxJoints];
	for (int j = 0; j < n1; ++j)
		neutral1[j] = SkelLoadMat44(inDesc->mNeutralPose1[j]);
	for (int j = 0; j < n2; ++j)
		neutral2[j] = SkelLoadMat44(inDesc->mNeutralPose2[j]);

	Ref<SkeletonMapper> mapper = new SkeletonMapper;
	if (inDesc->mCanMapMode == 0)
		mapper->Initialize(skeleton1, neutral1, skeleton2, neutral2);
	else
		mapper->Initialize(skeleton1, neutral1, skeleton2, neutral2, [inDesc](const Skeleton *inSkeleton1, int inIndex1, const Skeleton *inSkeleton2, int inIndex2) { return SkelCanMapJoint(*inDesc, inSkeleton1, inIndex1, inSkeleton2, inIndex2); });

	if ((inDesc->mLockMode & 1) != 0)
	{
		bool locked[cSkelMaxJoints];
		for (int j = 0; j < n2; ++j)
			locked[j] = inDesc->mLocked[j] != 0;
		mapper->LockTranslations(skeleton2, locked, neutral2);
	}
	if ((inDesc->mLockMode & 2) != 0 && !mapper->GetMappings().empty())
		mapper->LockAllTranslations(skeleton2, neutral2);

	const SkeletonMapper::MappingVector &mappings = mapper->GetMappings();
	outResult->mNumMappings = uint32(mappings.size());
	for (size_t i = 0; i < mappings.size(); ++i)
	{
		outResult->mMappingJoints[i][0] = mappings[i].mJointIdx1;
		outResult->mMappingJoints[i][1] = mappings[i].mJointIdx2;
		SkelStoreMat44(mappings[i].mJoint1To2, outResult->mMappingMatrices[i][0]);
		SkelStoreMat44(mappings[i].mJoint2To1, outResult->mMappingMatrices[i][1]);
	}

	const SkeletonMapper::ChainVector &chains = mapper->GetChains();
	outResult->mNumChains = uint32(chains.size());
	for (size_t i = 0; i < chains.size(); ++i)
	{
		outResult->mChainLengths[i][0] = uint32(chains[i].mJointIndices1.size());
		outResult->mChainLengths[i][1] = uint32(chains[i].mJointIndices2.size());
		for (size_t j = 0; j < chains[i].mJointIndices1.size(); ++j)
			outResult->mChainJoints1[i][j] = chains[i].mJointIndices1[j];
		for (size_t j = 0; j < chains[i].mJointIndices2.size(); ++j)
			outResult->mChainJoints2[i][j] = chains[i].mJointIndices2[j];
	}

	const SkeletonMapper::UnmappedVector &unmapped = mapper->GetUnmapped();
	outResult->mNumUnmapped = uint32(unmapped.size());
	for (size_t i = 0; i < unmapped.size(); ++i)
	{
		outResult->mUnmapped[i][0] = unmapped[i].mJointIdx;
		outResult->mUnmapped[i][1] = unmapped[i].mParentJointIdx;
	}

	const SkeletonMapper::LockedVector &locked = mapper->GetLockedTranslations();
	outResult->mNumLocked = uint32(locked.size());
	for (size_t i = 0; i < locked.size(); ++i)
	{
		outResult->mLockedJoints[i][0] = locked[i].mJointIdx;
		outResult->mLockedJoints[i][1] = locked[i].mParentJointIdx;
		outResult->mLockedTranslations[i][0] = locked[i].mTranslation.GetX();
		outResult->mLockedTranslations[i][1] = locked[i].mTranslation.GetY();
		outResult->mLockedTranslations[i][2] = locked[i].mTranslation.GetZ();
	}

	for (int j1 = -1; j1 <= n1; ++j1)
		outResult->mMappedJointIdx[j1 + 1] = mapper->GetMappedJointIdx(j1);
	for (int j2 = -1; j2 <= n2; ++j2)
		outResult->mIsLocked[j2 + 1] = mapper->IsJointTranslationLocked(j2)? 1 : 0;

	Mat44 sentinel = SkelLoadMat44(inDesc->mSentinel);
	Mat44 pose1[cSkelMaxJoints], pose2_local[cSkelMaxJoints], pose2_model[cSkelMaxJoints], out2[cSkelMaxJoints], out1[cSkelMaxJoints];
	for (int j = 0; j < n1; ++j)
	{
		pose1[j] = SkelLoadMat44(inDesc->mPose1ModelSpace[j]);
		out1[j] = sentinel;
	}
	for (int j = 0; j < n2; ++j)
	{
		pose2_local[j] = SkelLoadMat44(inDesc->mPose2LocalSpace[j]);
		pose2_model[j] = SkelLoadMat44(inDesc->mPose2ModelSpace[j]);
		out2[j] = sentinel;
	}

	if (inDesc->mMap != 0)
		mapper->Map(pose1, pose2_local, out2);
	mapper->MapReverse(pose2_model, out1);

	for (int j = 0; j < n2; ++j)
		SkelStoreMat44(out2[j], outResult->mPose2ModelSpace[j]);
	for (int j = 0; j < n1; ++j)
		SkelStoreMat44(out1[j], outResult->mPose1ModelSpace[j]);
}

} // extern "C"
