// Reference implementation for the compound shape parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library for StaticCompoundShape, MutableCompoundShape and Shape::ScaleShape. A compound is described by a
// CompoundDesc: up to 4 leaves (SphereShape / BoxShape settings, shared by the sub shapes), an optional nested compound,
// up to cMaxSubShapes sub shapes (a leaf or the nested compound, as is or wrapped in a RotatedTranslatedShape /
// ScaledShape, added as a shape or as settings), a chain of 2 sub shape compounds around it (to exceed the sub shape ID
// bits) and, for a MutableCompoundShape, a list of mutations (AddShape, RemoveShape, ModifyShape(s),
// AdjustCenterOfMass) and an optional Clone. The same construction is done in ZoltParity/Physics/CompoundsParity.zig,
// which calls these wrappers and checks that Zolt produces the same bits.
//
// The queries that take a ShapeFilter run with CompoundsParityFilter: it rejects a shape sub type and a sub shape ID
// (shape 2 / the pair) and folds every call it receives (shape sub types and sub shape IDs) into a hash.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant), RVec3 results as 3 doubles. Booleans are
// passed as int (never bool, see the porting guide). The structs must match the extern structs in CompoundsParity.zig.
// Everything except the C ABI is in an anonymous namespace (other reference files have classes with the same names).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

namespace {

void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

Vec3 Load3(const float *inV)						{ return Vec3(inV[0], inV[1], inV[2]); }
Vec4 Load4(const float *inV)						{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
Quat LoadQuat(const float *inV)						{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
Mat44 LoadMat44(const float *inM)					{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
AABox LoadAABox(const float *inBox)					{ return AABox(Load3(inBox), Load3(inBox + 3)); }
void Store3(Vec3Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
void Store4(Vec4Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); outV[3] = inV.GetW(); }
void StoreQuat(QuatArg inQ, float *outV)			{ Store4(inQ.GetXYZW(), outV); }
void StoreR3(RVec3Arg inV, double *outV)			{ outV[0] = double(inV.GetX()); outV[1] = double(inV.GetY()); outV[2] = double(inV.GetZ()); }
void StoreMat44(Mat44Arg inM, float *outM)			{ for (int c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
void StoreAABox(const AABox &inBox, float *outBox)	{ Store3(inBox.mMin, outBox); Store3(inBox.mMax, outBox + 3); }
void StorePlane(const Plane &inPlane, float *outPlane) { Store3(inPlane.GetNormal(), outPlane); outPlane[3] = inPlane.GetConstant(); }
SubShapeIDCreator MakeCreator(const uint32 *inCreator) { return SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }

SubShapeID MakeSubShapeID(uint32 inValue)
{
	SubShapeID id;
	id.SetValue(inValue);
	return id;
}

void StoreFace(const Shape::SupportingFace &inFace, uint32 &outCount, float *outFace)
{
	outCount = uint32(inFace.size());
	for (uint32 i = 0; i < outCount; ++i)
		Store3(inFace[i], outFace + 3 * i);
}

// ---------------------------------------------------------------------------------------------------------------------
// Compound descriptions, must match LeafDesc / SubDesc / MutationDesc / CompoundDesc in CompoundsParity.zig

static constexpr uint32 cMaxLeaves = 4;
static constexpr uint32 cMaxNested = 6;
static constexpr uint32 cMaxSubShapes = 200;
static constexpr uint32 cMaxMutations = 8;
static constexpr uint32 cMaxBatch = 4;
static constexpr uint32 cNestedLeaf = 0xffffffff;

struct LeafDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape
	float					mConvexRadius;			// BoxShape
	float					mDensity;
	uint32					mUserData;
};

struct SubDesc
{
	uint32					mLeaf;					// Index in CompoundDesc::mLeaves, cNestedLeaf: the nested compound
	uint32					mWrap;					// 0: none, 1: RotatedTranslatedShape, 2: ScaledShape
	uint32					mAsSettings;			// 1: added as settings (the compound creates the child), 0: added as shape
	uint32					mUserData;
	float					mPosition[3];
	float					mRotation[4];
	float					mWrapVector[3];			// RotatedTranslatedShape position / ScaledShape scale
	float					mWrapRotation[4];		// RotatedTranslatedShape rotation
};

struct MutationDesc
{
	uint32					mOp;					// 0: AddShape, 1: RemoveShape, 2: ModifyShape, 3: ModifyShape with a shape, 4: ModifyShapes, 5: AdjustCenterOfMass
	uint32					mIndex;
	uint32					mCount;					// ModifyShapes
	SubDesc					mSub;					// The shape (AddShape, ModifyShape with a shape), position, rotation, user data
	float					mPositions[cMaxBatch][3]; // ModifyShapes
	float					mRotations[cMaxBatch][4];
};

struct CompoundDesc
{
	uint32					mKind;					// 0: StaticCompoundShape, 1: MutableCompoundShape
	uint32					mUserData;
	uint32					mCreateMode;			// Static: 1 creates with a TempAllocatorImpl. Mutable: 1 clones after the mutations
	uint32					mChainDepth;			// Number of 2 sub shape compounds (of the same kind) wrapped around the compound
	uint32					mNumLeaves;
	LeafDesc				mLeaves[cMaxLeaves];
	uint32					mNestedKind;			// The nested compound (0: StaticCompoundShape, 1: MutableCompoundShape)
	uint32					mNumNested;				// 0: there is no nested compound
	SubDesc					mNested[cMaxNested];	// Sub shapes of the nested compound (leaves only)
	uint32					mNumSubShapes;
	SubDesc					mSubShapes[cMaxSubShapes];
	uint32					mNumMutations;
	MutationDesc			mMutations[cMaxMutations];
};

uint64 UserData(uint32 inValue)						{ return uint64(inValue) * 0x100000001ull; }

// The stride of ModifyShapes
struct PosRot
{
	Vec3					mPosition;
	Quat					mRotation;
};

static_assert(sizeof(PosRot) == 32, "Must match PosRot in CompoundsParity.zig");

// Builds the shapes of a CompoundDesc (the same steps as CompoundBuilder in CompoundsParity.zig)
class CompoundBuilder
{
public:
	explicit				CompoundBuilder(const CompoundDesc &inDesc) : mDesc(inDesc)
	{
		for (uint32 i = 0; i < inDesc.mNumLeaves; ++i)
		{
			const LeafDesc &leaf = inDesc.mLeaves[i];
			Ref<ShapeSettings> settings;
			if (leaf.mKind == 0)
			{
				SphereShapeSettings *sphere = new SphereShapeSettings(leaf.mRadius);
				sphere->mDensity = leaf.mDensity;
				settings = sphere;
			}
			else
			{
				BoxShapeSettings *box = new BoxShapeSettings(Load3(leaf.mHalfExtent), leaf.mConvexRadius);
				box->mDensity = leaf.mDensity;
				settings = box;
			}
			settings->mUserData = UserData(leaf.mUserData);
			mLeafSettings.push_back(settings);
			Shape::ShapeResult result = settings->Create();
			mLeafShapes.push_back(result.IsValid()? result.Get() : nullptr);
		}

		if (inDesc.mNumNested > 0)
		{
			mNestedSettings = CompoundSettings(inDesc.mNestedKind);
			for (uint32 i = 0; i < inDesc.mNumNested; ++i)
				AddSubShape(*mNestedSettings, inDesc.mNested[i]);
			Shape::ShapeResult result = mNestedSettings->Create();
			mNestedShape = result.IsValid()? result.Get() : nullptr;
		}
	}

	// The compound (the settings of the desc, created, mutated, cloned, wrapped in the chain)
	Shape::ShapeResult		Create()
	{
		Ref<CompoundShapeSettings> settings = CompoundSettings(mDesc.mKind);
		settings->mUserData = UserData(mDesc.mUserData);
		for (uint32 i = 0; i < mDesc.mNumSubShapes; ++i)
			AddSubShape(*settings, mDesc.mSubShapes[i]);

		Shape::ShapeResult result;
		if (mDesc.mKind == 0 && mDesc.mCreateMode == 1)
		{
			TempAllocatorImpl temp_allocator(1024 * 1024);
			result = static_cast<StaticCompoundShapeSettings *>(settings.GetPtr())->Create(temp_allocator);
		}
		else
			result = settings->Create();
		if (!result.IsValid())
			return result;

		if (result.Get()->GetSubType() == EShapeSubType::MutableCompound)
		{
			MutableCompoundShape *mutable_shape = static_cast<MutableCompoundShape *>(result.Get().GetPtr());
			for (uint32 i = 0; i < mDesc.mNumMutations; ++i)
				Mutate(*mutable_shape, mDesc.mMutations[i]);
			if (mDesc.mCreateMode == 1)
			{
				Ref<MutableCompoundShape> clone = mutable_shape->Clone();
				result.Set(clone.GetPtr());
			}
		}

		for (uint32 i = 0; i < mDesc.mChainDepth; ++i)
		{
			Ref<CompoundShapeSettings> chain = CompoundSettings(mDesc.mKind);
			chain->AddShape(Vec3(-1, 0, 0), Quat::sIdentity(), result.Get().GetPtr());
			chain->AddShape(Vec3(1, 0, 0), Quat::sIdentity(), mLeafShapes[0]);
			Shape::ShapeResult chain_result = chain->Create();
			result = chain_result;
			if (!result.IsValid())
				return result;
		}
		return result;
	}

private:
	static Ref<CompoundShapeSettings> CompoundSettings(uint32 inKind)
	{
		if (inKind == 0)
			return new StaticCompoundShapeSettings;
		else
			return new MutableCompoundShapeSettings;
	}

	// The shape of a sub shape that is added as a shape (nullptr when the leaf is invalid)
	Ref<Shape>				SubShape(const SubDesc &inSub) const
	{
		Ref<Shape> inner = inSub.mLeaf == cNestedLeaf? mNestedShape : mLeafShapes[inSub.mLeaf];
		if (inner == nullptr)
			return nullptr;
		switch (inSub.mWrap)
		{
		case 1:		return new RotatedTranslatedShape(Load3(inSub.mWrapVector), LoadQuat(inSub.mWrapRotation), inner);
		case 2:		return new ScaledShape(inner, Load3(inSub.mWrapVector));
		default:	return inner;
		}
	}

	// The settings of a sub shape that is added as settings
	Ref<ShapeSettings>		SubSettings(const SubDesc &inSub) const
	{
		Ref<ShapeSettings> inner = inSub.mLeaf == cNestedLeaf? Ref<ShapeSettings>(mNestedSettings.GetPtr()) : mLeafSettings[inSub.mLeaf];
		switch (inSub.mWrap)
		{
		case 1:		return new RotatedTranslatedShapeSettings(Load3(inSub.mWrapVector), LoadQuat(inSub.mWrapRotation), inner);
		case 2:		return new ScaledShapeSettings(inner, Load3(inSub.mWrapVector));
		default:	return inner;
		}
	}

	void					AddSubShape(CompoundShapeSettings &ioSettings, const SubDesc &inSub) const
	{
		Ref<Shape> shape = inSub.mAsSettings != 0? nullptr : SubShape(inSub);
		if (shape != nullptr)
			ioSettings.AddShape(Load3(inSub.mPosition), LoadQuat(inSub.mRotation), shape, inSub.mUserData);
		else
			ioSettings.AddShape(Load3(inSub.mPosition), LoadQuat(inSub.mRotation), SubSettings(inSub), inSub.mUserData);
	}

	void					Mutate(MutableCompoundShape &ioShape, const MutationDesc &inMutation) const
	{
		uint num_sub_shapes = ioShape.GetNumSubShapes();
		const SubDesc &sub = inMutation.mSub;
		switch (inMutation.mOp)
		{
		case 0:
			{
				Ref<Shape> shape = SubShape(sub);
				if (shape != nullptr)
					ioShape.AddShape(Load3(sub.mPosition), LoadQuat(sub.mRotation), shape, sub.mUserData, inMutation.mIndex);
				break;
			}

		case 1:
			if (inMutation.mIndex < num_sub_shapes)
				ioShape.RemoveShape(inMutation.mIndex);
			break;

		case 2:
			if (inMutation.mIndex < num_sub_shapes)
				ioShape.ModifyShape(inMutation.mIndex, Load3(sub.mPosition), LoadQuat(sub.mRotation));
			break;

		case 3:
			{
				Ref<Shape> shape = SubShape(sub);
				if (inMutation.mIndex < num_sub_shapes && shape != nullptr)
					ioShape.ModifyShape(inMutation.mIndex, Load3(sub.mPosition), LoadQuat(sub.mRotation), shape);
				break;
			}

		case 4:
			if (uint64(inMutation.mIndex) + inMutation.mCount <= num_sub_shapes)
			{
				PosRot pos_rot[cMaxBatch];
				for (uint32 i = 0; i < cMaxBatch; ++i)
				{
					pos_rot[i].mPosition = Load3(inMutation.mPositions[i]);
					pos_rot[i].mRotation = LoadQuat(inMutation.mRotations[i]);
				}
				ioShape.ModifyShapes(inMutation.mIndex, inMutation.mCount, &pos_rot[0].mPosition, &pos_rot[0].mRotation, sizeof(PosRot), sizeof(PosRot));
			}
			break;

		default:
			ioShape.AdjustCenterOfMass();
			break;
		}
	}

	const CompoundDesc &	mDesc;
	Array<Ref<ShapeSettings>> mLeafSettings;
	Array<Ref<Shape>>		mLeafShapes;
	Ref<CompoundShapeSettings> mNestedSettings;
	Ref<Shape>				mNestedShape;
};

Ref<Shape> CreateShape(const CompoundDesc &inDesc)
{
	CompoundBuilder builder(inDesc);
	Shape::ShapeResult result = builder.Create();
	return result.IsValid()? result.Get() : nullptr;
}

// ---------------------------------------------------------------------------------------------------------------------
// A filter that rejects a shape sub type and a sub shape ID and hashes its calls, must match CompoundsParityFilter in
// CompoundsParity.zig

struct FilterLog
{
	uint32					mCalls = 0;
	uint32					mHash = 0x811c9dc5;

	void					Add(uint32 inValue)
	{
		mHash = (mHash ^ inValue) * 0x01000193;
	}
};

class CompoundsParityFilter : public ShapeFilter
{
public:
	explicit				CompoundsParityFilter(uint32 inRejectSubType, uint32 inRejectID) : mRejectSubType(inRejectSubType), mRejectID(inRejectID) { }

	virtual bool			ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		++mLog.mCalls;
		mLog.Add(uint32(inShape2->GetSubType()));
		mLog.Add(inSubShapeIDOfShape2.GetValue());
		return uint32(inShape2->GetSubType()) != mRejectSubType && inSubShapeIDOfShape2.GetValue() != mRejectID;
	}

	virtual bool			ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		++mLog.mCalls;
		mLog.Add(uint32(inShape1->GetSubType()));
		mLog.Add(inSubShapeIDOfShape1.GetValue());
		mLog.Add(uint32(inShape2->GetSubType()));
		mLog.Add(inSubShapeIDOfShape2.GetValue());
		return uint32(inShape2->GetSubType()) != mRejectSubType && inSubShapeIDOfShape1.GetValue() != mRejectID && inSubShapeIDOfShape2.GetValue() != mRejectID;
	}

	uint32					mRejectSubType;
	uint32					mRejectID;
	mutable FilterLog		mLog;
};

// ---------------------------------------------------------------------------------------------------------------------
// Inputs and outputs, must match the extern structs in CompoundsParity.zig

static constexpr uint32 cMaxHits = 48;
static constexpr uint32 cMaxShapeHits = 12;

struct TSOutput
{
	double					mPosition[3];
	float					mRotation[4];
	float					mScale[3];
	uint32					mSubType;
	uint32					mUserData;
	uint32					mBodyID;
	uint32					mCreatorID;
	uint32					mCreatorBits;
};

struct SubShapeOutput
{
	float					mPositionCOM[3];
	float					mRotation[4];
	float					mTransformScale[3];
	uint32					mUserData;
	uint32					mSubType;
	int						mIsValidScale;
};

struct PropertiesInput
{
	float					mScale[3];				// A valid scale
	float					mAnyScale[3];			// IsValidScale / MakeScaleValid only
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
	float					mPoint[3];				// GetSurfaceNormal
	float					mDirection[3];			// GetSupportingFace
	float					mPositionCOM[3];		// GetSubShapeTransformedShape
	float					mRotation[4];
	uint32					mLeafID;				// A valid sub shape ID: GetSurfaceNormal, GetSupportingFace, GetSubShapeTransformedShape, GetMaterial
	int						mHasLeaf;				// 0: the shape has no leaf (an empty compound), the functions that need mLeafID are not called
	uint32					mAnyID;					// Any sub shape ID: IsSubShapeIDValid (GetLeafShape / GetSubShapeIndexFromID assert on an invalid ID)
	float					mBox[6];				// GetIntersectingSubShapes
	float					mOrientedBox[16 + 3];	// GetIntersectingSubShapes: orientation, half extents
	uint32					mMaxIndices;
};

struct PropertiesOutput
{
	float					mLocalBounds[6];
	float					mWorldBounds[6];
	float					mWorldBoundsD[6];
	float					mCenterOfMass[3];
	float					mInnerRadius;
	float					mMass;
	float					mInertia[16];
	float					mVolume;
	uint32					mNumTrianglesRecursive;
	uint32					mSubShapeIDBits;
	int						mMustBeStatic;
	int						mIsValidScale;
	float					mScaleValid[3];
	int						mIsValidAnyScale;
	float					mAnyScaleValid[3];
	float					mSurfaceNormal[3];
	uint32					mFaceCount;
	float					mFace[32 * 3];
	uint32					mLeafSubType;			// GetLeafShape(mLeafID)
	uint32					mLeafUserData;
	uint32					mLeafRemainder;
	uint32					mSubShapeUserData[2];	// GetSubShapeUserData(mLeafID) (low, high)
	int						mMaterialIsDefault;
	uint32					mNumSubShapeState;		// SaveSubShapeState
	TSOutput				mChild;					// GetSubShapeTransformedShape(mLeafID)
	uint32					mChildRemainder;
	// Compound only
	uint32					mNumSubShapes;
	uint32					mCompoundBits;			// GetSubShapeIDBits
	int						mIsSubShapeIDValid;		// mAnyID
	uint32					mSubShapeIndex;			// GetSubShapeIndexFromID(mLeafID)
	uint32					mSubShapeIndexRemainder;
	uint32					mNumBlocks;				// Static: the number of nodes, mutable: the number of bounds blocks (from GetStats)
	uint32					mNumIntersecting;		// GetIntersectingSubShapes(AABox)
	uint32					mIntersecting[cMaxSubShapes];
	uint32					mNumIntersectingOriented; // GetIntersectingSubShapes(OrientedBox)
	uint32					mIntersectingOriented[cMaxSubShapes];
	SubShapeOutput			mSubShapes[cMaxSubShapes];
};

struct RayInput
{
	float					mOrigin[3];
	float					mDirection[3];
	uint32					mCreator[2];			// Sub shape ID creator: value pushed, number of bits
	float					mFraction;				// Initial fraction of the single hit version
	int						mBackFaceMode;			// 1: collide with back faces (convex)
	int						mTreatConvexAsSolid;
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
	float					mEarlyOut;				// Early out fraction of the collector (when < the initial one)
	uint32					mBodyID;				// Body ID of the collector context
	uint32					mRejectSubType;			// Shape sub type that the filter rejects (~0: none)
	uint32					mRejectID;				// Sub shape ID that the filter rejects
};

struct RayHit
{
	float					mFraction;
	uint32					mBodyID;
	uint32					mSubShapeID;
};

struct RayOutput
{
	int						mHit;
	float					mFraction;
	uint32					mSubShapeID;
	uint32					mNumHits;
	RayHit					mHits[cMaxHits];
	uint32					mFilterCalls;
	uint32					mFilterHash;
};

struct PointInput
{
	float					mPoint[3];
	uint32					mCreator[2];
	uint32					mBodyID;
	uint32					mRejectSubType;
	uint32					mRejectID;
	int						mAnyHit;				// 1: AnyHitCollisionCollector
};

struct PointOutput
{
	uint32					mNumHits;
	uint32					mHits[cMaxHits];		// Sub shape IDs in order
	uint32					mBodyID;				// Of the last hit
	uint32					mFilterCalls;
	uint32					mFilterHash;
};

struct CollectInput
{
	float					mBox[6];
	float					mPositionCOM[3];
	float					mRotation[4];
	float					mScale[3];
	uint32					mCreator[2];
	uint32					mBodyID;
	uint32					mRejectSubType;
	uint32					mRejectID;
	int						mAnyHit;				// 1: AnyHitCollisionCollector
	float					mTransform[16];			// TransformShape (may contain a scale)
};

struct CollectOutput
{
	uint32					mNumCollected;
	uint32					mFilterCalls;
	uint32					mFilterHash;
	TSOutput				mCollected[cMaxHits];
	uint32					mNumTransformed;
	TSOutput				mTransformed[cMaxHits];
};

// The other shape of a collision / cast: a leaf (sphere or box) or a compound
struct OtherDesc
{
	uint32					mIsCompound;
	LeafDesc				mLeaf;
};

struct CollideInput
{
	uint32					mCompoundIsShape1;		// 0: the compound is shape 2, 1: shape 1, 2: both (the same compound)
	OtherDesc				mOther;
	float					mScale1[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mMaxSeparationDistance;
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	int						mCollectFaces;
	int						mBackFaceMode;			// 1: collide with back faces
	int						mActiveEdgeMode;		// 1: collide with all edges
	float					mActiveEdgeMovementDirection[3];
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
	float					mEarlyOut;				// Early out fraction of the collector (when < FLT_MAX)
	uint32					mBodyID;
	uint32					mRejectSubType;
	uint32					mRejectID;
};

struct CastInput
{
	uint32					mCompoundIsShape1;		// 0: the compound is shape 2, 1: shape 1 (the cast shape), 2: both
	OtherDesc				mOther;
	float					mScale1[3];
	float					mStart[16];
	float					mDirection[3];
	float					mScale2[3];
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	float					mExtraConvexRadius;
	int						mBackFaceModeTriangles;
	int						mBackFaceModeConvex;
	int						mActiveEdgeMode;
	int						mUseShrunkenShape;
	int						mReturnDeepestPoint;
	int						mCollectFaces;
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
	float					mEarlyOut;				// Early out fraction of the collector (when < 1 + FLT_EPSILON)
	uint32					mBodyID;
	uint32					mRejectSubType;
	uint32					mRejectID;
};

struct HitOutput
{
	float					mFraction;				// Cast only
	int						mBackFace;				// Cast only
	float					mPoint1[3];
	float					mPoint2[3];
	float					mAxis[3];
	float					mDepth;
	uint32					mID1;
	uint32					mID2;
	uint32					mBodyID;
	uint32					mFace1Count;
	uint32					mFace2Count;
	float					mFace1[32 * 3];
	float					mFace2[32 * 3];
};

struct HitsOutput
{
	uint32					mNumHits;
	uint32					mFilterCalls;
	uint32					mFilterHash;
	HitOutput				mHits[cMaxShapeHits];
};

void StoreTS(const TransformedShape &inTS, TSOutput &outTS)
{
	StoreR3(inTS.mShapePositionCOM, outTS.mPosition);
	StoreQuat(inTS.mShapeRotation, outTS.mRotation);
	Store3(inTS.GetShapeScale(), outTS.mScale);
	outTS.mSubType = inTS.mShape != nullptr? uint32(inTS.mShape->GetSubType()) : ~uint32(0);
	outTS.mUserData = inTS.mShape != nullptr? uint32(inTS.mShape->GetUserData()) : 0;
	outTS.mBodyID = inTS.mBodyID.GetIndexAndSequenceNumber();
	outTS.mCreatorID = inTS.mSubShapeIDCreator.GetID().GetValue();
	outTS.mCreatorBits = inTS.mSubShapeIDCreator.GetNumBitsWritten();
}

void StoreCollideHit(const CollideShapeResult &inResult, HitOutput &outHit)
{
	Store3(inResult.mContactPointOn1, outHit.mPoint1);
	Store3(inResult.mContactPointOn2, outHit.mPoint2);
	Store3(inResult.mPenetrationAxis, outHit.mAxis);
	outHit.mDepth = inResult.mPenetrationDepth;
	outHit.mID1 = inResult.mSubShapeID1.GetValue();
	outHit.mID2 = inResult.mSubShapeID2.GetValue();
	outHit.mBodyID = inResult.mBodyID2.GetIndexAndSequenceNumber();
	StoreFace(inResult.mShape1Face, outHit.mFace1Count, outHit.mFace1);
	StoreFace(inResult.mShape2Face, outHit.mFace2Count, outHit.mFace2);
}

Ref<Shape> CreateOther(const OtherDesc &inOther, const CompoundDesc &inCompound)
{
	if (inOther.mIsCompound != 0)
		return CreateShape(inCompound);
	Ref<ShapeSettings> settings;
	if (inOther.mLeaf.mKind == 0)
		settings = new SphereShapeSettings(inOther.mLeaf.mRadius);
	else
		settings = new BoxShapeSettings(Load3(inOther.mLeaf.mHalfExtent), inOther.mLeaf.mConvexRadius);
	settings->mUserData = UserData(inOther.mLeaf.mUserData);
	return settings->Create().Get();
}

std::string SaveShape(const Shape *inShape)
{
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	inShape->SaveBinaryState(stream_out);
	return data.str();
}

std::string SaveShapeWithChildren(const Shape *inShape)
{
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	Shape::ShapeToIDMap shape_map;
	Shape::MaterialToIDMap material_map;
	inShape->SaveWithChildren(stream_out, shape_map, material_map);
	return data.str();
}

uint32 CopyBytes(const std::string &inBytes, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inBytes.size());
	if (size <= inCapacity)
		memcpy(outBytes, inBytes.data(), size);
	return size;
}

// The number of nodes of a StaticCompoundShape / bounds blocks of a MutableCompoundShape, from GetStats (the arrays are private)
uint32 NumBlocks(const Shape *inShape)
{
	const CompoundShape *compound = static_cast<const CompoundShape *>(inShape);
	size_t size = inShape->GetStats().mSizeBytes - compound->GetNumSubShapes() * sizeof(CompoundShape::SubShape);
	if (inShape->GetSubType() == EShapeSubType::StaticCompound)
		return uint32((size - sizeof(StaticCompoundShape)) / 64);
	else
		return uint32((size - sizeof(MutableCompoundShape)) / (6 * sizeof(Vec4)));
}

// Store the hits of a collide / cast collector
template <class Collector, class Store>
void StoreHits(const Collector &inCollector, Store inStore)
{
	for (const typename Collector::ResultType &r : inCollector.mHits)
		inStore(r);
}

} // namespace

extern "C" {

// Create the compound from its description: returns 1 if valid, otherwise 0 and the error text in outError (128 bytes,
// 0 terminated). outInfo receives the sub type, the low 32 bits of the user data, the number of sub shapes (of a
// compound) and the sub shape ID bits of the shape.
int jolt_compounds_create(const CompoundDesc *inDesc, char *outError, uint32 *outInfo)
{
	EnsureFactory();
	memset(outError, 0, 128);
	outInfo[0] = outInfo[1] = outInfo[2] = outInfo[3] = 0;
	CompoundBuilder builder(*inDesc);
	Shape::ShapeResult result = builder.Create();
	if (result.IsValid())
	{
		const Shape *shape = result.Get();
		outInfo[0] = uint32(shape->GetSubType());
		outInfo[1] = uint32(shape->GetUserData());
		outInfo[2] = shape->GetType() == EShapeType::Compound? static_cast<const CompoundShape *>(shape)->GetNumSubShapes() : 0;
		outInfo[3] = shape->GetSubShapeIDBitsRecursive();
		return 1;
	}
	strncpy(outError, result.GetError().c_str(), 127);
	return 0;
}

// Bounds, mass properties, volume, scales, surface normal, supporting face, leaf shape, sub shapes, the compound's
// accessors and GetIntersectingSubShapes
void jolt_compounds_properties(const CompoundDesc *inDesc, const PropertiesInput *inInput, PropertiesOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Vec3 scale = Load3(inInput->mScale);
	Vec3 any_scale = Load3(inInput->mAnyScale);
	Mat44 transform = LoadMat44(inInput->mTransform);
	PropertiesOutput &o = *outOutput;
	StoreAABox(shape->GetLocalBounds(), o.mLocalBounds);
	StoreAABox(shape->GetWorldSpaceBounds(transform, scale), o.mWorldBounds);
	StoreAABox(shape->GetWorldSpaceBounds(DMat44(transform, DVec3(inInput->mTranslation[0], inInput->mTranslation[1], inInput->mTranslation[2])), scale), o.mWorldBoundsD);
	Store3(shape->GetCenterOfMass(), o.mCenterOfMass);
	o.mInnerRadius = shape->GetInnerRadius();
	MassProperties p = shape->GetMassProperties();
	o.mMass = p.mMass;
	StoreMat44(p.mInertia, o.mInertia);
	o.mVolume = shape->GetVolume();
	Shape::VisitedShapes visited;
	o.mNumTrianglesRecursive = shape->GetStatsRecursive(visited).mNumTriangles;
	o.mSubShapeIDBits = shape->GetSubShapeIDBitsRecursive();
	o.mMustBeStatic = shape->MustBeStatic()? 1 : 0;
	o.mIsValidScale = shape->IsValidScale(scale)? 1 : 0;
	Store3(shape->MakeScaleValid(scale), o.mScaleValid);
	o.mIsValidAnyScale = shape->IsValidScale(any_scale)? 1 : 0;
	Store3(shape->MakeScaleValid(any_scale), o.mAnyScaleValid);

	SubShapeID leaf_id = MakeSubShapeID(inInput->mLeafID);
	if (inInput->mHasLeaf != 0)
	{
		Store3(shape->GetSurfaceNormal(leaf_id, Load3(inInput->mPoint)), o.mSurfaceNormal);
		Shape::SupportingFace face;
		shape->GetSupportingFace(leaf_id, Load3(inInput->mDirection), scale, transform, face);
		StoreFace(face, o.mFaceCount, o.mFace);
	}

	SubShapeID any_id = MakeSubShapeID(inInput->mAnyID);
	ShapeList sub_shapes;
	shape->SaveSubShapeState(sub_shapes);
	o.mNumSubShapeState = uint32(sub_shapes.size());
	if (inInput->mHasLeaf != 0)
	{
		SubShapeID remainder;
		const Shape *leaf = shape->GetLeafShape(leaf_id, remainder);
		o.mLeafSubType = leaf != nullptr? uint32(leaf->GetSubType()) : ~uint32(0);
		o.mLeafUserData = leaf != nullptr? uint32(leaf->GetUserData()) : 0;
		o.mLeafRemainder = remainder.GetValue();
		uint64 user_data = shape->GetSubShapeUserData(leaf_id);
		o.mSubShapeUserData[0] = uint32(user_data);
		o.mSubShapeUserData[1] = uint32(user_data >> 32);
		o.mMaterialIsDefault = shape->GetMaterial(leaf_id) == PhysicsMaterial::sDefault? 1 : 0;

		SubShapeID child_remainder;
		TransformedShape child = shape->GetSubShapeTransformedShape(leaf_id, Load3(inInput->mPositionCOM), LoadQuat(inInput->mRotation), scale, child_remainder);
		StoreTS(child, o.mChild);
		o.mChildRemainder = child_remainder.GetValue();
	}

	if (shape->GetType() == EShapeType::Compound)
	{
		const CompoundShape *compound = static_cast<const CompoundShape *>(shape.GetPtr());
		o.mNumSubShapes = compound->GetNumSubShapes();
		o.mCompoundBits = compound->GetSubShapeIDFromIndex(0, SubShapeIDCreator()).GetNumBitsWritten(); // GetSubShapeIDBits is protected
		o.mIsSubShapeIDValid = compound->IsSubShapeIDValid(any_id)? 1 : 0;
		if (inInput->mHasLeaf != 0)
		{
			SubShapeID index_remainder;
			o.mSubShapeIndex = compound->GetSubShapeIndexFromID(leaf_id, index_remainder);
			o.mSubShapeIndexRemainder = index_remainder.GetValue();
		}
		o.mNumBlocks = NumBlocks(compound);
		uint max_indices = min(inInput->mMaxIndices, cMaxSubShapes);
		o.mNumIntersecting = uint32(compound->GetIntersectingSubShapes(LoadAABox(inInput->mBox), o.mIntersecting, int(max_indices)));
		OrientedBox oriented_box(LoadMat44(inInput->mOrientedBox), Load3(inInput->mOrientedBox + 16));
		o.mNumIntersectingOriented = uint32(compound->GetIntersectingSubShapes(oriented_box, o.mIntersectingOriented, int(max_indices)));
		for (uint i = 0; i < o.mNumSubShapes && i < cMaxSubShapes; ++i)
		{
			const CompoundShape::SubShape &s = compound->GetSubShape(i);
			SubShapeOutput &so = o.mSubShapes[i];
			Store3(s.GetPositionCOM(), so.mPositionCOM);
			StoreQuat(s.GetRotation(), so.mRotation);
			so.mIsValidScale = s.IsValidScale(scale)? 1 : 0;
			Store3(s.TransformScale(scale), so.mTransformScale);
			so.mUserData = compound->GetCompoundUserData(i);
			so.mSubType = uint32(s.mShape->GetSubType());
		}
	}
}

// CastRay (the single hit version) and CastRay with a collector and CompoundsParityFilter
void jolt_compounds_cast_ray(const CompoundDesc *inDesc, const RayInput *inInput, RayOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	RayCast ray { Load3(inInput->mOrigin), Load3(inInput->mDirection) };
	SubShapeIDCreator creator = MakeCreator(inInput->mCreator);

	RayCastResult hit;
	hit.mFraction = inInput->mFraction;
	outOutput->mHit = shape->CastRay(ray, creator, hit)? 1 : 0;
	outOutput->mFraction = hit.mFraction;
	outOutput->mSubShapeID = hit.mSubShapeID2.GetValue();

	RayCastSettings settings;
	settings.mBackFaceModeConvex = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mTreatConvexAsSolid = inInput->mTreatConvexAsSolid != 0;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	CompoundsParityFilter filter(inInput->mRejectSubType, inInput->mRejectID);
	outOutput->mNumHits = 0;
	auto store = [outOutput](const RayCastResult &inHit) {
		if (outOutput->mNumHits < cMaxHits)
		{
			RayHit &h = outOutput->mHits[outOutput->mNumHits];
			h.mFraction = inHit.mFraction;
			h.mBodyID = inHit.mBodyID.GetIndexAndSequenceNumber();
			h.mSubShapeID = inHit.mSubShapeID2.GetValue();
		}
		outOutput->mNumHits++;
	};
	switch (inInput->mCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector, filter);
			for (const RayCastResult &h : collector.mHits)
				store(h);
			break;
		}

	case 1:
		{
			AnyHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector, filter);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}

	default:
		{
			ClosestHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector, filter);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}
	}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// CollidePoint with CompoundsParityFilter (all hits in order or the any hit collector)
void jolt_compounds_collide_point(const CompoundDesc *inDesc, const PointInput *inInput, PointOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	CompoundsParityFilter filter(inInput->mRejectSubType, inInput->mRejectID);
	outOutput->mNumHits = 0;
	outOutput->mBodyID = 0;
	auto store = [outOutput](const CollidePointResult &inHit) {
		if (outOutput->mNumHits < cMaxHits)
			outOutput->mHits[outOutput->mNumHits] = inHit.mSubShapeID2.GetValue();
		outOutput->mNumHits++;
		outOutput->mBodyID = inHit.mBodyID.GetIndexAndSequenceNumber();
	};
	if (inInput->mAnyHit != 0)
	{
		AnyHitCollisionCollector<CollidePointCollector> collector;
		collector.SetContext(&context);
		shape->CollidePoint(Load3(inInput->mPoint), MakeCreator(inInput->mCreator), collector, filter);
		if (collector.HadHit())
			store(collector.mHit);
	}
	else
	{
		AllHitCollisionCollector<CollidePointCollector> collector;
		collector.SetContext(&context);
		shape->CollidePoint(Load3(inInput->mPoint), MakeCreator(inInput->mCreator), collector, filter);
		for (const CollidePointResult &h : collector.mHits)
			store(h);
	}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// CollectTransformedShapes (with CompoundsParityFilter, all hits in order or the any hit collector) and TransformShape
void jolt_compounds_collect(const CompoundDesc *inDesc, const CollectInput *inInput, CollectOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	CollectOutput &o = *outOutput;

	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	CompoundsParityFilter filter(inInput->mRejectSubType, inInput->mRejectID);
	o.mNumCollected = 0;
	auto store = [&o](const TransformedShape &inTS) {
		if (o.mNumCollected < cMaxHits)
			StoreTS(inTS, o.mCollected[o.mNumCollected]);
		o.mNumCollected++;
	};
	if (inInput->mAnyHit != 0)
	{
		AnyHitCollisionCollector<TransformedShapeCollector> collector;
		collector.SetContext(&context);
		shape->CollectTransformedShapes(LoadAABox(inInput->mBox), Load3(inInput->mPositionCOM), LoadQuat(inInput->mRotation), Load3(inInput->mScale), MakeCreator(inInput->mCreator), collector, filter);
		if (collector.HadHit())
			store(collector.mHit);
	}
	else
	{
		AllHitCollisionCollector<TransformedShapeCollector> collector;
		collector.SetContext(&context);
		shape->CollectTransformedShapes(LoadAABox(inInput->mBox), Load3(inInput->mPositionCOM), LoadQuat(inInput->mRotation), Load3(inInput->mScale), MakeCreator(inInput->mCreator), collector, filter);
		for (const TransformedShape &ts : collector.mHits)
			store(ts);
	}
	o.mFilterCalls = filter.mLog.mCalls;
	o.mFilterHash = filter.mLog.mHash;

	AllHitCollisionCollector<TransformedShapeCollector> transformed;
	shape->TransformShape(LoadMat44(inInput->mTransform), transformed);
	o.mNumTransformed = 0;
	for (const TransformedShape &ts : transformed.mHits)
	{
		if (o.mNumTransformed < cMaxHits)
			StoreTS(ts, o.mTransformed[o.mNumTransformed]);
		o.mNumTransformed++;
	}
}

// CollisionDispatch::sCollideShapeVsShape (all hits in order / any hit / closest hit) with CompoundsParityFilter
void jolt_compounds_collide(const CompoundDesc *inDesc, const CollideInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> compound = CreateShape(*inDesc);
	Ref<Shape> other = inInput->mCompoundIsShape1 == 2? compound : CreateOther(inInput->mOther, *inDesc);
	Ref<Shape> shape1 = inInput->mCompoundIsShape1 != 0? compound : other;
	Ref<Shape> shape2 = inInput->mCompoundIsShape1 != 0? other : compound;
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mBackFaceMode = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	CompoundsParityFilter filter(inInput->mRejectSubType, inInput->mRejectID);
	outOutput->mNumHits = 0;
	auto store = [outOutput](const CollideShapeResult &inResult) {
		if (outOutput->mNumHits < cMaxShapeHits)
		{
			HitOutput &h = outOutput->mHits[outOutput->mNumHits];
			h.mFraction = 0.0f;
			h.mBackFace = 0;
			StoreCollideHit(inResult, h);
		}
		outOutput->mNumHits++;
	};
	auto run = [&](CollideShapeCollector &ioCollector) {
		ioCollector.SetContext(&context);
		if (inInput->mEarlyOut < ioCollector.GetEarlyOutFraction())
			ioCollector.UpdateEarlyOutFraction(inInput->mEarlyOut);
		CollisionDispatch::sCollideShapeVsShape(shape1, shape2, Load3(inInput->mScale1), Load3(inInput->mScale2), LoadMat44(inInput->mTransform1), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), settings, ioCollector, filter);
	};
	switch (inInput->mCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CollideShapeCollector> collector;
			run(collector);
			StoreHits(collector, store);
			break;
		}

	case 1:
		{
			AnyHitCollisionCollector<CollideShapeCollector> collector;
			run(collector);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}

	default:
		{
			ClosestHitCollisionCollector<CollideShapeCollector> collector;
			run(collector);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}
	}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace (all hits in order / any hit / closest hit) with CompoundsParityFilter
void jolt_compounds_cast(const CompoundDesc *inDesc, const CastInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> compound = CreateShape(*inDesc);
	Ref<Shape> other = inInput->mCompoundIsShape1 == 2? compound : CreateOther(inInput->mOther, *inDesc);
	Ref<Shape> shape1 = inInput->mCompoundIsShape1 != 0? compound : other;
	Ref<Shape> shape2 = inInput->mCompoundIsShape1 != 0? other : compound;
	ShapeCastSettings settings;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mBackFaceModeTriangles = inInput->mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mBackFaceModeConvex = inInput->mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mUseShrunkenShapeAndConvexRadius = inInput->mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = inInput->mReturnDeepestPoint != 0;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	ShapeCast shape_cast(shape1, Load3(inInput->mScale1), LoadMat44(inInput->mStart), Load3(inInput->mDirection));
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	CompoundsParityFilter filter(inInput->mRejectSubType, inInput->mRejectID);
	outOutput->mNumHits = 0;
	auto store = [outOutput](const ShapeCastResult &inResult) {
		if (outOutput->mNumHits < cMaxShapeHits)
		{
			HitOutput &h = outOutput->mHits[outOutput->mNumHits];
			h.mFraction = inResult.mFraction;
			h.mBackFace = inResult.mIsBackFaceHit? 1 : 0;
			StoreCollideHit(inResult, h);
		}
		outOutput->mNumHits++;
	};
	auto run = [&](CastShapeCollector &ioCollector) {
		ioCollector.SetContext(&context);
		if (inInput->mEarlyOut < ioCollector.GetEarlyOutFraction())
			ioCollector.UpdateEarlyOutFraction(inInput->mEarlyOut);
		CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, shape2, Load3(inInput->mScale2), filter, LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), ioCollector);
	};
	switch (inInput->mCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CastShapeCollector> collector;
			run(collector);
			StoreHits(collector, store);
			break;
		}

	case 1:
		{
			AnyHitCollisionCollector<CastShapeCollector> collector;
			run(collector);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}

	default:
		{
			ClosestHitCollisionCollector<CastShapeCollector> collector;
			run(collector);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}
	}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// GetSubmergedVolume: total volume, submerged volume, center of buoyancy
void jolt_compounds_submerged_volume(const CompoundDesc *inDesc, const float *inTransform, const float *inScale, const float *inPlane, float *outValues)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	float total_volume, submerged_volume;
	Vec3 center_of_buoyancy;
	shape->GetSubmergedVolume(LoadMat44(inTransform), Load3(inScale), Plane(Load4(inPlane)), total_volume, submerged_volume, center_of_buoyancy);
	outValues[0] = total_volume;
	outValues[1] = submerged_volume;
	Store3(center_of_buoyancy, outValues + 2);
}

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), ioPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_compounds_soft_body(const CompoundDesc *inDesc, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Array<Vec3> positions;
	Array<Plane> planes;
	for (int i = 0; i < inNumVertices; ++i)
	{
		positions.push_back(Load3(inPositions + 3 * i));
		planes.push_back(Plane(Load4(ioPlanes + 4 * i)));
	}
	CollideSoftBodyVertexIterator vertices(StridedPtr<const Vec3>(positions.data()), StridedPtr<const float>(inInvMasses), StridedPtr<Plane>(planes.data()), StridedPtr<float>(ioPenetrations), StridedPtr<int>(ioIndices));
	shape->CollideSoftBodyVertices(LoadMat44(inTransform), Load3(inScale), vertices, uint(inNumVertices), inCollidingShapeIndex);
	for (int i = 0; i < inNumVertices; ++i)
		StorePlane(planes[i], ioPlanes + 4 * i);
}

// Binary state of the shape (outSizes[0] bytes in outBytes), the same after sRestoreFromBinaryState +
// RestoreSubShapeState with the sub shapes of the original (outSizes[1], 0 when the restore fails), SaveWithChildren
// (outSizes[2]) and SaveWithChildren of the shape that sRestoreWithChildren restores from those bytes (outSizes[3], 0
// when the restore fails). Each buffer has inCapacity bytes.
void jolt_compounds_binary_state(const CompoundDesc *inDesc, uint8 *outBytes, uint8 *outRestoredBytes, uint8 *outChildrenBytes, uint8 *outRestoredChildrenBytes, uint32 inCapacity, uint32 *outSizes)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);

	std::string bytes = SaveShape(shape);
	outSizes[0] = CopyBytes(bytes, outBytes, inCapacity);

	outSizes[1] = 0;
	{
		std::stringstream in_data(bytes);
		StreamInWrapper stream_in(in_data);
		Shape::ShapeResult result = Shape::sRestoreFromBinaryState(stream_in);
		if (result.IsValid())
		{
			ShapeList sub_shapes;
			shape->SaveSubShapeState(sub_shapes);
			result.Get()->RestoreSubShapeState(sub_shapes.data(), uint(sub_shapes.size()));
			outSizes[1] = CopyBytes(SaveShape(result.Get()), outRestoredBytes, inCapacity);
		}
	}

	std::string children = SaveShapeWithChildren(shape);
	outSizes[2] = CopyBytes(children, outChildrenBytes, inCapacity);

	outSizes[3] = 0;
	{
		std::stringstream in_data(children);
		StreamInWrapper stream_in(in_data);
		Shape::IDToShapeMap shape_map;
		Shape::IDToMaterialMap material_map;
		Shape::ShapeResult result = Shape::sRestoreWithChildren(stream_in, shape_map, material_map);
		if (result.IsValid())
			outSizes[3] = CopyBytes(SaveShapeWithChildren(result.Get()), outRestoredChildrenBytes, inCapacity);
	}
}

// Shape::ScaleShape of the compound: returns 1 if valid (then outSizes[0] receives the size of SaveWithChildren of the
// result in outBytes, outInfo its sub type, number of sub shapes (of a compound) and its local bounds as 6 floats),
// otherwise 0 and the error text in outError
int jolt_compounds_scale_shape(const CompoundDesc *inDesc, const float *inScale, char *outError, uint32 *outInfo, float *outBounds, uint8 *outBytes, uint32 inCapacity, uint32 *outSize)
{
	EnsureFactory();
	memset(outError, 0, 128);
	outInfo[0] = outInfo[1] = 0;
	*outSize = 0;
	Ref<Shape> shape = CreateShape(*inDesc);
	Shape::ShapeResult result = shape->ScaleShape(Load3(inScale));
	if (!result.IsValid())
	{
		strncpy(outError, result.GetError().c_str(), 127);
		return 0;
	}
	const Shape *scaled = result.Get();
	outInfo[0] = uint32(scaled->GetSubType());
	outInfo[1] = scaled->GetType() == EShapeType::Compound? static_cast<const CompoundShape *>(scaled)->GetNumSubShapes() : 0;
	StoreAABox(scaled->GetLocalBounds(), outBounds);
	*outSize = CopyBytes(SaveShapeWithChildren(scaled), outBytes, inCapacity);
	return 1;
}

} // extern "C"
