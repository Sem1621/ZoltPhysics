// Reference implementation of the Ragdoll parity tests (`zig build parity`, Phase 7 Ragdoll group): thin C ABI wrappers
// around the C++ Jolt library's RagdollSettings. ZoltParity/Physics/RagdollParity.zig calls these C functions and checks
// that Zolt produces the same bits.
//
// - A test builds a RagdollSettings from a description (RPDesc): a skeleton whose joints are added with AddJoint(name,
//   parent index) (a parent after its child is set through GetJoints() afterwards), one Part per joint (its
//   BodyCreationSettings and the index of its mToParent constraint), the additional constraints. The shapes, materials,
//   group filters and constraint settings come from tables that both sides build from the same descriptions (RPTables):
//   a shape is shared by the parts that use its index (or created from new shape settings per part), a constraint
//   settings object by every part and additional constraint that use its index (the constraint descriptions are the
//   CSSettings of the constraint parity, made by CSMakeConstraintSettings of ConstraintsReference.cpp).
// - jolt_rp_run applies the operations of the description in order (Stabilize, CalculateConstraintPriorities,
//   DisableParentChildCollisions without and with joint matrices, the two Calculate* maps), writes the results
//   (RPResult: Stabilize's return values, the maps, the collision table of the group filter) and the four
//   SaveBinaryState byte strings (save shapes x save group filter).
// - jolt_rp_resave restores settings with sRestoreFromBinaryState and saves them again (the cross check: each side
//   restores the other side's bytes; truncated streams give the same error text or the same settings on both sides).
//
// The structs below must match the extern structs of RagdollParity.zig (checked by jolt_rp_struct_sizes). Booleans are
// passed as uint32 (never bool, see the porting guide). All C functions have the prefix jolt_rp_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

static void RPEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static constexpr uint32 cRPMaxJoints = 24;
static constexpr uint32 cRPMaxAdditional = 8;
static constexpr uint32 cRPMaxOps = 8;
static constexpr uint32 cRPNone = 0xffffffff;

static Vec3 RPLoad3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 RPLoad4(const float *inV)				{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Quat RPLoadQuat(const float *inV)			{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 RPLoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static Mat44 RPLoadMat44(const float *inM)			{ return Mat44(RPLoad4(inM), RPLoad4(inM + 4), RPLoad4(inM + 8), RPLoad4(inM + 12)); }

// Must match RPShapeDesc in RagdollParity.zig
struct RPShapeDesc
{
	uint32				mKind;				///< 0 sphere, 1 box, 2 capsule, 3 offset COM box, 4 rotated translated capsule, 5 static compound of a box and a sphere, 6 uniformly scaled box, 7 mesh (a quad of two triangles in the XZ plane)
	uint32				mMaterial;			///< Index in the material table or cRPNone
	float				mParams[8];			///< 0..2 half extent / radius / half height, 3 convex radius, 4..6 offset / translation / scale
	float				mRotation[4];
};

// Must match RPMaterialDesc in RagdollParity.zig
struct RPMaterialDesc
{
	char				mName[16];			///< Zero terminated
	uint32				mColor;
};

// Must match RPGroupFilterDesc in RagdollParity.zig
struct RPGroupFilterDesc
{
	uint32				mNumSubGroups;
	uint32				mNumDisabled;
	uint32				mDisabled[16];		///< Pairs of sub group IDs
};

// Must match RPTables in RagdollParity.zig
struct RPTables
{
	const RPShapeDesc *			mShapes;
	uint32						mNumShapes;
	const RPMaterialDesc *		mMaterials;
	uint32						mNumMaterials;
	const RPGroupFilterDesc *	mFilters;
	uint32						mNumFilters;
	const void *				mConstraints;		///< CSSettings of the constraint parity
	uint32						mNumConstraints;
};

// Must match RPPartDesc in RagdollParity.zig: the BodyCreationSettings of a part, its parent joint and constraint
struct RPPartDesc
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
	uint32				mShape;				///< Index in the shape table or cRPNone (no shape)
	uint32				mUseShapeSettings;	///< 1: new ShapeSettings from the description of the shape (GetShape creates the shape)
	uint32				mGroupFilter;		///< Index in the group filter table or cRPNone
	uint32				mGroupID;
	uint32				mSubGroupID;
	int32				mParent;			///< The parent joint index of the skeleton (-1: a root)
	uint32				mToParent;			///< mToParent: index in the constraint table or cRPNone
};

// Must match RPAdditionalDesc in RagdollParity.zig
struct RPAdditionalDesc
{
	int32				mBodyIdx[2];
	uint32				mConstraint;		///< Index in the constraint table
};

// Must match RPOp in RagdollParity.zig
struct RPOp
{
	uint32				mKind;				///< 0 Stabilize, 1 CalculateConstraintPriorities(mBasePriority), 2 DisableParentChildCollisions(), 3 DisableParentChildCollisions(mJointMatrices, mMinSeparationDistance), 4 CalculateBodyIndexToConstraintIndex, 5 CalculateConstraintIndexToBodyIdxPair
	uint32				mBasePriority;
	float				mMinSeparationDistance;
};

// Must match RPDesc in RagdollParity.zig
struct RPDesc
{
	uint32				mNumJoints;
	uint32				mNumAdditional;
	uint32				mNumOps;
	RPPartDesc			mParts[cRPMaxJoints];
	RPAdditionalDesc	mAdditional[cRPMaxAdditional];
	RPOp				mOps[cRPMaxOps];
	float				mJointMatrices[cRPMaxJoints][16];
};

// Must match RPResult in RagdollParity.zig (zeroed by the caller)
struct RPResult
{
	uint32				mStabilize[cRPMaxOps];					///< Per operation: Stabilize's result (0 / 1), cRPNone for the other operations
	uint32				mNumParts;
	uint32				mNumAdditional;
	uint32				mNumBodyToConstraint;
	int32				mBodyToConstraint[cRPMaxJoints];		///< GetBodyIndexToConstraintIndex
	int32				mConstraintIndexForBodyIndex[cRPMaxJoints]; ///< GetConstraintIndexForBodyIndex(i)
	uint32				mNumPairs;
	int32				mPairs[cRPMaxJoints + cRPMaxAdditional][2]; ///< GetConstraintIndexToBodyIdxPair
	int32				mPairsForConstraintIndex[cRPMaxJoints + cRPMaxAdditional][2]; ///< GetBodyIndicesForConstraintIndex(i)
	uint32				mFilterSubGroups;						///< The joint count after a DisableParentChildCollisions, else 0
	uint32				mFilterShared;							///< Every part has the group filter of part 0
	uint8				mCollisionEnabled[cRPMaxJoints][cRPMaxJoints]; ///< IsCollisionEnabled(i, j) of part 0's GroupFilterTable (i != j, after a DisableParentChildCollisions)
	uint32				mSubGroupIDs[cRPMaxJoints];				///< The sub group ID of every part
	uint32				mSaveSizes[4];							///< jolt_rp_run: the sizes of the SaveBinaryState bytes, index = save shapes << 1 | save group filter
};

// The settings of the description inSettings[inIndex] of the constraint parity (an array of CSSettings, see
// ConstraintsReference.cpp, which defines this function)
Ref<TwoBodyConstraintSettings> CSMakeConstraintSettings(const void *inSettings, uint32 inIndex);

// The shapes, materials, group filters and constraint settings of a test
class RPTableObjects
{
public:
	void						Init(const RPTables &inTables)
	{
		for (uint32 i = 0; i < inTables.mNumMaterials; ++i)
		{
			const RPMaterialDesc &m = inTables.mMaterials[i];
			mMaterials.push_back(new PhysicsMaterialSimple(m.mName, Color(m.mColor)));
		}
		for (uint32 i = 0; i < inTables.mNumFilters; ++i)
		{
			const RPGroupFilterDesc &f = inTables.mFilters[i];
			GroupFilterTable *filter = new GroupFilterTable(f.mNumSubGroups);
			for (uint32 j = 0; j < f.mNumDisabled; ++j)
				filter->DisableCollision(f.mDisabled[2 * j], f.mDisabled[2 * j + 1]);
			mFilters.push_back(filter);
		}
		mShapeDescs = inTables.mShapes;
		for (uint32 i = 0; i < inTables.mNumShapes; ++i)
			mShapes.push_back(MakeShapeSettings(inTables.mShapes[i])->Create().Get());
		for (uint32 i = 0; i < inTables.mNumConstraints; ++i)
			mConstraints.push_back(CSMakeConstraintSettings(inTables.mConstraints, i));
	}

	const PhysicsMaterial *		GetMaterial(uint32 inIndex) const
	{
		return inIndex < mMaterials.size()? mMaterials[inIndex].GetPtr() : nullptr;
	}

	Ref<ShapeSettings>			MakeBoxSettings(const RPShapeDesc &inDesc) const
	{
		return new BoxShapeSettings(RPLoad3(inDesc.mParams), inDesc.mParams[3], GetMaterial(inDesc.mMaterial));
	}

	// makeShapeSettings of RagdollParity.zig
	Ref<ShapeSettings>			MakeShapeSettings(const RPShapeDesc &inDesc) const
	{
		const float *p = inDesc.mParams;
		const PhysicsMaterial *material = GetMaterial(inDesc.mMaterial);
		switch (inDesc.mKind)
		{
		case 0:		return new SphereShapeSettings(p[0], material);
		case 1:		return MakeBoxSettings(inDesc);
		case 2:		return new CapsuleShapeSettings(p[0], p[1], material);
		case 3:		return new OffsetCenterOfMassShapeSettings(RPLoad3(p + 4), MakeBoxSettings(inDesc));
		case 4:		return new RotatedTranslatedShapeSettings(RPLoad3(p + 4), RPLoadQuat(inDesc.mRotation), new CapsuleShapeSettings(p[0], p[1], material));
		case 5:
			{
				StaticCompoundShapeSettings *compound = new StaticCompoundShapeSettings;
				compound->AddShape(RPLoad3(p + 4), RPLoadQuat(inDesc.mRotation), MakeBoxSettings(inDesc));
				compound->AddShape(-RPLoad3(p + 4), Quat::sIdentity(), new SphereShapeSettings(p[0], material));
				return compound;
			}
		case 6:		return new ScaledShapeSettings(MakeBoxSettings(inDesc), Vec3::sReplicate(p[4]));
		default:
			{
				TriangleList triangles;
				triangles.push_back(Triangle(Float3(-p[0], 0, -p[0]), Float3(-p[0], 0, p[0]), Float3(p[0], 0, p[0])));
				triangles.push_back(Triangle(Float3(-p[0], 0, -p[0]), Float3(p[0], 0, p[0]), Float3(p[0], 0, -p[0])));
				PhysicsMaterialList materials;
				if (material != nullptr)
					materials.push_back(material);
				return new MeshShapeSettings(triangles, materials);
			}
		}
	}

	// makeBodyCreationSettings of RagdollParity.zig
	BodyCreationSettings		MakeBodyCreationSettings(const RPPartDesc &inDesc) const
	{
		BodyCreationSettings s;
		s.mPosition = RPLoadR3(inDesc.mPosition);
		s.mRotation = RPLoadQuat(inDesc.mRotation);
		s.mLinearVelocity = RPLoad3(inDesc.mLinearVelocity);
		s.mAngularVelocity = RPLoad3(inDesc.mAngularVelocity);
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
		s.mMassPropertiesOverride.mInertia = RPLoadMat44(inDesc.mInertia);
		if (inDesc.mShape < mShapes.size())
		{
			if (inDesc.mUseShapeSettings != 0)
				s.SetShapeSettings(MakeShapeSettings(mShapeDescs[inDesc.mShape]));
			else
				s.SetShape(mShapes[inDesc.mShape]);
		}
		return s;
	}

	// buildSettings of RagdollParity.zig: the RagdollSettings of a description
	Ref<RagdollSettings>		MakeRagdollSettings(const RPDesc &inDesc) const
	{
		Ref<Skeleton> skeleton = new Skeleton;
		for (uint32 j = 0; j < inDesc.mNumJoints; ++j)
		{
			std::string name = "J" + std::to_string(j);
			int parent = inDesc.mParts[j].mParent;
			if (parent < int(j))
				skeleton->AddJoint(name, parent);
			else
				skeleton->AddJoint(name);
		}
		for (uint32 j = 0; j < inDesc.mNumJoints; ++j)
			skeleton->GetJoints()[j].mParentJointIndex = inDesc.mParts[j].mParent;

		Ref<RagdollSettings> settings = new RagdollSettings;
		settings->mSkeleton = skeleton;
		settings->mParts.resize(inDesc.mNumJoints);
		for (uint32 j = 0; j < inDesc.mNumJoints; ++j)
		{
			const RPPartDesc &d = inDesc.mParts[j];
			RagdollSettings::Part &part = settings->mParts[j];
			static_cast<BodyCreationSettings &>(part) = MakeBodyCreationSettings(d);
			if (d.mToParent != cRPNone)
				part.mToParent = mConstraints[d.mToParent];
		}
		for (uint32 a = 0; a < inDesc.mNumAdditional; ++a)
		{
			const RPAdditionalDesc &d = inDesc.mAdditional[a];
			settings->mAdditionalConstraints.push_back(RagdollSettings::AdditionalConstraint(d.mBodyIdx[0], d.mBodyIdx[1], mConstraints[d.mConstraint]));
		}
		return settings;
	}

	Array<RefConst<PhysicsMaterial>>	mMaterials;
	Array<Ref<GroupFilterTable>>		mFilters;
	Array<RefConst<Shape>>				mShapes;
	Array<Ref<TwoBodyConstraintSettings>> mConstraints;
	const RPShapeDesc *					mShapeDescs = nullptr;
};

// The maps of the settings (writeMaps of RagdollParity.zig)
static void RPWriteMaps(const RagdollSettings &inSettings, RPResult &outResult)
{
	outResult.mNumParts = uint32(inSettings.mParts.size());
	outResult.mNumAdditional = uint32(inSettings.mAdditionalConstraints.size());
	const Array<int> &body_to_constraint = inSettings.GetBodyIndexToConstraintIndex();
	outResult.mNumBodyToConstraint = uint32(body_to_constraint.size());
	for (uint32 i = 0; i < uint32(body_to_constraint.size()) && i < cRPMaxJoints; ++i)
	{
		outResult.mBodyToConstraint[i] = body_to_constraint[i];
		outResult.mConstraintIndexForBodyIndex[i] = inSettings.GetConstraintIndexForBodyIndex(int(i));
	}
	const Array<RagdollSettings::BodyIdxPair> &pairs = inSettings.GetConstraintIndexToBodyIdxPair();
	outResult.mNumPairs = uint32(pairs.size());
	for (uint32 i = 0; i < uint32(pairs.size()) && i < cRPMaxJoints + cRPMaxAdditional; ++i)
	{
		outResult.mPairs[i][0] = pairs[i].first;
		outResult.mPairs[i][1] = pairs[i].second;
		RagdollSettings::BodyIdxPair pair = inSettings.GetBodyIndicesForConstraintIndex(int(i));
		outResult.mPairsForConstraintIndex[i][0] = pair.first;
		outResult.mPairsForConstraintIndex[i][1] = pair.second;
	}
}

static void RPWriteGroups(const RagdollSettings &inSettings, uint32 inFilterSubGroups, RPResult &outResult)
{
	uint32 num_parts = uint32(inSettings.mParts.size());
	outResult.mFilterSubGroups = inFilterSubGroups;
	const GroupFilter *filter = num_parts > 0? inSettings.mParts[0].mCollisionGroup.GetGroupFilter() : nullptr;
	outResult.mFilterShared = 1;
	for (uint32 i = 0; i < num_parts; ++i)
	{
		const CollisionGroup &group = inSettings.mParts[i].mCollisionGroup;
		if (group.GetGroupFilter() != filter)
			outResult.mFilterShared = 0;
		outResult.mSubGroupIDs[i] = group.GetSubGroupID();
	}
	if (inFilterSubGroups > 0)
	{
		const GroupFilterTable *table = static_cast<const GroupFilterTable *>(filter);
		for (uint32 i = 0; i < inFilterSubGroups; ++i)
			for (uint32 j = 0; j < inFilterSubGroups; ++j)
				if (i != j)
					outResult.mCollisionEnabled[i][j] = table->IsCollisionEnabled(i, j)? 1 : 0;
	}
}

static uint32 RPSave(const RagdollSettings &inSettings, bool inSaveShapes, bool inSaveGroupFilter, uint8 *outBytes, uint32 inCapacity)
{
	std::stringstream data;
	StreamOutWrapper out(data);
	inSettings.SaveBinaryState(out, inSaveShapes, inSaveGroupFilter);
	std::string str = data.str();
	uint32 size = uint32(str.size());
	if (size <= inCapacity)
		memcpy(outBytes, str.data(), size);
	return size;
}

extern "C"
{

void jolt_rp_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(RPShapeDesc);
	outSizes[1] = sizeof(RPMaterialDesc);
	outSizes[2] = sizeof(RPGroupFilterDesc);
	outSizes[3] = sizeof(RPTables);
	outSizes[4] = sizeof(RPPartDesc);
	outSizes[5] = sizeof(RPAdditionalDesc);
	outSizes[6] = sizeof(RPOp);
	outSizes[7] = sizeof(RPDesc);
	outSizes[8] = sizeof(RPResult);
	outSizes[9] = offsetof(RPPartDesc, mObjectLayer);
	outSizes[10] = offsetof(RPDesc, mJointMatrices);
	outSizes[11] = offsetof(RPResult, mSaveSizes);
}

// Build the settings of the description, apply its operations and write the results; the four SaveBinaryState byte
// strings are written one after the other to outBytes (index = save shapes << 1 | save group filter, their sizes in
// outResult->mSaveSizes). Returns the total size (the bytes are only written when they fit).
uint32 jolt_rp_run(const RPTables *inTables, const RPDesc *inDesc, RPResult *outResult, uint8 *outBytes, uint32 inCapacity)
{
	RPEnsureFactory();
	RPTableObjects tables;
	tables.Init(*inTables);
	Ref<RagdollSettings> settings = tables.MakeRagdollSettings(*inDesc);

	uint32 filter_sub_groups = 0;
	for (uint32 o = 0; o < inDesc->mNumOps; ++o)
	{
		const RPOp &op = inDesc->mOps[o];
		outResult->mStabilize[o] = cRPNone;
		switch (op.mKind)
		{
		case 0:
			outResult->mStabilize[o] = settings->Stabilize()? 1 : 0;
			break;

		case 1:
			settings->CalculateConstraintPriorities(op.mBasePriority);
			break;

		case 2:
			settings->DisableParentChildCollisions();
			filter_sub_groups = inDesc->mNumJoints;
			break;

		case 3:
			{
				Mat44 matrices[cRPMaxJoints];
				for (uint32 j = 0; j < inDesc->mNumJoints; ++j)
					matrices[j] = RPLoadMat44(inDesc->mJointMatrices[j]);
				settings->DisableParentChildCollisions(matrices, op.mMinSeparationDistance);
				filter_sub_groups = inDesc->mNumJoints;
				break;
			}

		case 4:
			settings->CalculateBodyIndexToConstraintIndex();
			break;

		case 5:
			settings->CalculateConstraintIndexToBodyIdxPair();
			break;

		default:
			abort();
		}
	}

	RPWriteMaps(*settings, *outResult);
	RPWriteGroups(*settings, filter_sub_groups, *outResult);

	uint32 total = 0;
	for (uint32 k = 0; k < 4; ++k)
	{
		uint32 size = RPSave(*settings, (k & 2) != 0, (k & 1) != 0, outBytes + total, total <= inCapacity? inCapacity - total : 0);
		outResult->mSaveSizes[k] = size;
		total += size;
	}
	return total;
}

// sRestoreFromBinaryState of the bytes, the maps and groups of the restored settings, then SaveBinaryState with the given
// flags. Returns the size of the new bytes, or cRPNone and Jolt's error text when the restore failed.
uint32 jolt_rp_resave(const uint8 *inBytes, uint32 inSize, uint32 inSaveShapes, uint32 inSaveGroupFilter, RPResult *outResult, uint8 *outBytes, uint32 inCapacity, char *outError, uint32 inErrorCapacity)
{
	RPEnsureFactory();
	std::stringstream in_data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(in_data);
	RagdollSettings::RagdollResult result = RagdollSettings::sRestoreFromBinaryState(in);
	if (result.HasError())
	{
		const String &error = result.GetError();
		size_t n = std::min(size_t(inErrorCapacity - 1), error.size());
		memcpy(outError, error.data(), n);
		outError[n] = 0;
		return cRPNone;
	}
	const RagdollSettings &settings = *result.Get();
	RPWriteMaps(settings, *outResult);
	RPWriteGroups(settings, 0, *outResult);
	return RPSave(settings, inSaveShapes != 0, inSaveGroupFilter != 0, outBytes, inCapacity);
}

} // extern "C"
