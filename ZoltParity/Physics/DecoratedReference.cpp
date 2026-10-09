// Reference implementation for the decorated shape parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library for ScaledShape, RotatedTranslatedShape and OffsetCenterOfMassShape. A shape is described by a leaf
// (SphereShape or BoxShape, created from its settings) wrapped in up to 3 decorators (ShapeDesc); each decorator is built
// with its constructor, from settings that hold the inner shape, from settings that hold the inner settings (nested
// settings create their children) or from settings without an inner shape (an error). The same construction is done in
// ZoltParity/Physics/DecoratedParity.zig, which calls these wrappers and checks that Zolt produces the same bits.
//
// The queries that take a ShapeFilter run with DecoratedParityFilter: it rejects a shape sub type (on either side) and
// folds every call it receives (shape sub types and sub shape IDs) into a hash, so the tests check that the decorated
// shapes pass the filter on to their inner shapes and to CollisionDispatch with the right arguments.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant), RVec3 results as 3 doubles. Booleans are
// passed as int (never bool, see the porting guide). The structs must match the extern structs in DecoratedParity.zig.
// Everything except the C ABI is in an anonymous namespace (other reference files have classes with the same names).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/RegisterTypes.h>
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
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

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
// Shape descriptions, must match LeafDesc / DecoratorDesc / ShapeDesc in DecoratedParity.zig

struct LeafDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape
	float					mConvexRadius;			// BoxShape
	float					mDensity;
	uint32					mUserData;
};

struct DecoratorDesc
{
	uint32					mKind;					// 0: ScaledShape, 1: RotatedTranslatedShape, 2: OffsetCenterOfMassShape
	uint32					mMode;					// 0: constructor, 1: settings with the inner shape, 2: settings with the inner settings, 3: settings without inner shape
	float					mVector[3];				// Scale / position / offset
	float					mRotation[4];			// RotatedTranslatedShape
	uint32					mUserData;
};

struct ShapeDesc
{
	LeafDesc				mLeaf;
	uint32					mNumDecorators;			// mDecorators[0] wraps the leaf, mDecorators[1] wraps that, ...
	DecoratorDesc			mDecorators[3];
};

uint64 UserData(uint32 inValue)						{ return uint64(inValue) * 0x100000001ull; }

Ref<ShapeSettings> LeafSettings(const LeafDesc &inLeaf)
{
	Ref<ShapeSettings> settings;
	if (inLeaf.mKind == 0)
	{
		SphereShapeSettings *sphere = new SphereShapeSettings(inLeaf.mRadius);
		sphere->mDensity = inLeaf.mDensity;
		settings = sphere;
	}
	else
	{
		BoxShapeSettings *box = new BoxShapeSettings(Load3(inLeaf.mHalfExtent), inLeaf.mConvexRadius);
		box->mDensity = inLeaf.mDensity;
		settings = box;
	}
	settings->mUserData = UserData(inLeaf.mUserData);
	return settings;
}

// Settings of a decorator with an inner shape (const Shape *) or inner settings (const ShapeSettings *)
template <class T>
Ref<ShapeSettings> DecoratorSettings(const DecoratorDesc &inDesc, T inInner)
{
	Ref<ShapeSettings> settings;
	switch (inDesc.mKind)
	{
	case 0:		settings = new ScaledShapeSettings(inInner, Load3(inDesc.mVector)); break;
	case 1:		settings = new RotatedTranslatedShapeSettings(Load3(inDesc.mVector), LoadQuat(inDesc.mRotation), inInner); break;
	default:	settings = new OffsetCenterOfMassShapeSettings(Load3(inDesc.mVector), inInner); break;
	}
	settings->mUserData = UserData(inDesc.mUserData);
	return settings;
}

Shape::ShapeResult BuildShape(const ShapeDesc &inDesc, int inLevel);

// The settings of level inLevel (-1 is the leaf)
Ref<ShapeSettings> BuildSettings(const ShapeDesc &inDesc, int inLevel)
{
	if (inLevel < 0)
		return LeafSettings(inDesc.mLeaf);

	const DecoratorDesc &d = inDesc.mDecorators[inLevel];
	switch (d.mMode)
	{
	case 2:
		return DecoratorSettings(d, BuildSettings(inDesc, inLevel - 1).GetPtr());

	case 3:
		return DecoratorSettings(d, (const ShapeSettings *)nullptr);

	default:
		{
			Shape::ShapeResult inner = BuildShape(inDesc, inLevel - 1);
			return DecoratorSettings(d, inner.IsValid()? inner.Get().GetPtr() : (const Shape *)nullptr);
		}
	}
}

// The shape of level inLevel (-1 is the leaf)
Shape::ShapeResult BuildShape(const ShapeDesc &inDesc, int inLevel)
{
	if (inLevel < 0)
		return LeafSettings(inDesc.mLeaf)->Create();

	const DecoratorDesc &d = inDesc.mDecorators[inLevel];
	switch (d.mMode)
	{
	case 0:
		{
			Shape::ShapeResult inner = BuildShape(inDesc, inLevel - 1);
			if (inner.HasError())
				return inner;
			Ref<Shape> shape;
			switch (d.mKind)
			{
			case 0:		shape = new ScaledShape(inner.Get(), Load3(d.mVector)); break;
			case 1:		shape = new RotatedTranslatedShape(Load3(d.mVector), LoadQuat(d.mRotation), inner.Get()); break;
			default:	shape = new OffsetCenterOfMassShape(inner.Get(), Load3(d.mVector)); break;
			}
			shape->SetUserData(UserData(d.mUserData));
			Shape::ShapeResult result;
			result.Set(shape);
			return result;
		}

	case 1:
		{
			Shape::ShapeResult inner = BuildShape(inDesc, inLevel - 1);
			if (inner.HasError())
				return inner;
			return DecoratorSettings(d, inner.Get().GetPtr())->Create();
		}

	default:
		return BuildSettings(inDesc, inLevel)->Create();
	}
}

Shape::ShapeResult CreateShapeResult(const ShapeDesc &inDesc)
{
	return BuildShape(inDesc, int(inDesc.mNumDecorators) - 1);
}

Ref<Shape> CreateShape(const ShapeDesc &inDesc)
{
	Shape::ShapeResult result = CreateShapeResult(inDesc);
	return result.IsValid()? result.Get() : nullptr;
}

// ---------------------------------------------------------------------------------------------------------------------
// A filter that rejects a shape sub type and hashes its calls, must match DecoratedParityFilter in DecoratedParity.zig

struct FilterLog
{
	uint32					mCalls = 0;
	uint32					mHash = 0x811c9dc5;

	void					Add(uint32 inValue)
	{
		mHash = (mHash ^ inValue) * 0x01000193;
	}
};

class DecoratedParityFilter : public ShapeFilter
{
public:
	explicit				DecoratedParityFilter(uint32 inRejectSubType1, uint32 inRejectSubType2) : mRejectSubType1(inRejectSubType1), mRejectSubType2(inRejectSubType2) { }

	virtual bool			ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		++mLog.mCalls;
		mLog.Add(uint32(inShape2->GetSubType()));
		mLog.Add(inSubShapeIDOfShape2.GetValue());
		return uint32(inShape2->GetSubType()) != mRejectSubType2;
	}

	virtual bool			ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		++mLog.mCalls;
		mLog.Add(uint32(inShape1->GetSubType()));
		mLog.Add(inSubShapeIDOfShape1.GetValue());
		mLog.Add(uint32(inShape2->GetSubType()));
		mLog.Add(inSubShapeIDOfShape2.GetValue());
		return uint32(inShape1->GetSubType()) != mRejectSubType1 && uint32(inShape2->GetSubType()) != mRejectSubType2;
	}

	uint32					mRejectSubType1;
	uint32					mRejectSubType2;
	mutable FilterLog		mLog;
};

// ---------------------------------------------------------------------------------------------------------------------
// Inputs and outputs, must match the extern structs in DecoratedParity.zig

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
	uint32					mSubShapeID;			// GetLeafShape, GetSubShapeTransformedShape, GetSubShapeUserData
};

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
	uint32					mNumTriangles;
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
	uint32					mLeafSubType;			// GetLeafShape
	uint32					mLeafUserData;
	uint32					mLeafRemainder;
	uint32					mSubShapeUserData[2];	// GetSubShapeUserData (low, high)
	int						mMaterialIsDefault;
	uint32					mNumSubShapes;			// SaveSubShapeState
	uint32					mSubShapeSubType;
	TSOutput				mChild;					// GetSubShapeTransformedShape
	uint32					mChildRemainder;
	float					mDecorator[13];			// The top decorator: ScaledShape::GetScale / RotatedTranslatedShape::GetPosition, GetRotation, TransformScale(scale), TransformScale(any scale) / OffsetCenterOfMassShape::GetOffset
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
	RayHit					mHits[4];
	uint32					mFilterCalls;
	uint32					mFilterHash;
};

struct PointOutput
{
	uint32					mNumHits;
	uint32					mBodyID;				// Of the last hit
	uint32					mSubShapeID;
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
	float					mTransform[16];			// TransformShape (may contain a scale)
	double					mBaseOffset[3];			// TransformedShape::GetTrianglesStart
	int						mMaxTrianglesRequested;
};

static constexpr int cMaxTriangleCalls = 16;
static constexpr int cMaxTriangleVertices = 384;

struct CollectOutput
{
	uint32					mNumCollected;
	uint32					mFilterCalls;
	uint32					mFilterHash;
	TSOutput				mCollected[2];
	uint32					mNumTransformed;
	TSOutput				mTransformed[2];
	int						mTriangleCalls;
	int						mTriangleCounts[cMaxTriangleCalls];
	float					mTriangleVertices[cMaxTriangleVertices * 3];
	int						mDefaultMaterial[cMaxTriangleVertices / 3];
};

struct CollideInput
{
	ShapeDesc				mShape1;
	ShapeDesc				mShape2;
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
	float					mEarlyOut;				// Early out fraction of the collector (when < FLT_MAX)
	uint32					mBodyID;
	uint32					mRejectSubType1;
	uint32					mRejectSubType2;
};

struct CastInput
{
	ShapeDesc				mShape1;
	ShapeDesc				mShape2;
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
	float					mEarlyOut;				// Early out fraction of the collector (when < 1 + FLT_EPSILON)
	uint32					mBodyID;
	uint32					mRejectSubType1;
	uint32					mRejectSubType2;
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
	HitOutput				mHits[2];
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

} // namespace

extern "C" {

// Create the shape from its description: returns 1 if valid, otherwise 0 and the error text in outError (128 bytes,
// 0 terminated). outInfo receives the sub type and the low 32 bits of the user data of the shape.
int jolt_decorated_create(const ShapeDesc *inDesc, char *outError, uint32 *outInfo)
{
	EnsureFactory();
	memset(outError, 0, 128);
	outInfo[0] = outInfo[1] = 0;
	Shape::ShapeResult result = CreateShapeResult(*inDesc);
	if (result.IsValid())
	{
		outInfo[0] = uint32(result.Get()->GetSubType());
		outInfo[1] = uint32(result.Get()->GetUserData());
		return 1;
	}
	strncpy(outError, result.GetError().c_str(), 127);
	return 0;
}

// Bounds, mass properties, volume, scales, surface normal, supporting face, leaf shape, sub shapes and the accessors of
// the top decorator
void jolt_decorated_properties(const ShapeDesc *inDesc, const PropertiesInput *inInput, PropertiesOutput *outOutput)
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
	o.mNumTriangles = shape->GetStats().mNumTriangles;
	Shape::VisitedShapes visited;
	o.mNumTrianglesRecursive = shape->GetStatsRecursive(visited).mNumTriangles;
	o.mSubShapeIDBits = shape->GetSubShapeIDBitsRecursive();
	o.mMustBeStatic = shape->MustBeStatic()? 1 : 0;
	o.mIsValidScale = shape->IsValidScale(scale)? 1 : 0;
	Store3(shape->MakeScaleValid(scale), o.mScaleValid);
	o.mIsValidAnyScale = shape->IsValidScale(any_scale)? 1 : 0;
	Store3(shape->MakeScaleValid(any_scale), o.mAnyScaleValid);
	Store3(shape->GetSurfaceNormal(SubShapeID(), Load3(inInput->mPoint)), o.mSurfaceNormal);
	Shape::SupportingFace face;
	shape->GetSupportingFace(SubShapeID(), Load3(inInput->mDirection), scale, transform, face);
	StoreFace(face, o.mFaceCount, o.mFace);

	SubShapeID id = MakeSubShapeID(inInput->mSubShapeID);
	SubShapeID remainder;
	const Shape *leaf = shape->GetLeafShape(id, remainder);
	o.mLeafSubType = leaf != nullptr? uint32(leaf->GetSubType()) : ~uint32(0);
	o.mLeafUserData = leaf != nullptr? uint32(leaf->GetUserData()) : 0;
	o.mLeafRemainder = remainder.GetValue();
	uint64 user_data = shape->GetSubShapeUserData(id);
	o.mSubShapeUserData[0] = uint32(user_data);
	o.mSubShapeUserData[1] = uint32(user_data >> 32);
	o.mMaterialIsDefault = shape->GetMaterial(SubShapeID()) == PhysicsMaterial::sDefault? 1 : 0;
	ShapeList sub_shapes;
	shape->SaveSubShapeState(sub_shapes);
	o.mNumSubShapes = uint32(sub_shapes.size());
	o.mSubShapeSubType = sub_shapes.empty() || sub_shapes[0] == nullptr? ~uint32(0) : uint32(sub_shapes[0]->GetSubType());

	SubShapeID child_remainder;
	TransformedShape child = shape->GetSubShapeTransformedShape(id, Load3(inInput->mPositionCOM), LoadQuat(inInput->mRotation), scale, child_remainder);
	StoreTS(child, o.mChild);
	o.mChildRemainder = child_remainder.GetValue();

	memset(o.mDecorator, 0, sizeof(o.mDecorator));
	switch (shape->GetSubType())
	{
	case EShapeSubType::Scaled:
		Store3(static_cast<const ScaledShape *>(shape.GetPtr())->GetScale(), o.mDecorator);
		break;

	case EShapeSubType::RotatedTranslated:
		{
			const RotatedTranslatedShape *rt = static_cast<const RotatedTranslatedShape *>(shape.GetPtr());
			Store3(rt->GetPosition(), o.mDecorator);
			StoreQuat(rt->GetRotation(), o.mDecorator + 3);
			Store3(rt->TransformScale(scale), o.mDecorator + 7);
			Store3(rt->TransformScale(any_scale), o.mDecorator + 10);
			break;
		}

	case EShapeSubType::OffsetCenterOfMass:
		Store3(static_cast<const OffsetCenterOfMassShape *>(shape.GetPtr())->GetOffset(), o.mDecorator);
		break;

	default:
		break;
	}
}

// CastRay (the single hit version) and CastRay with a collector and DecoratedParityFilter
void jolt_decorated_cast_ray(const ShapeDesc *inDesc, const RayInput *inInput, RayOutput *outOutput)
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
	DecoratedParityFilter filter(~uint32(0), inInput->mRejectSubType);
	outOutput->mNumHits = 0;
	auto store = [outOutput](const RayCastResult &inHit) {
		RayHit &h = outOutput->mHits[outOutput->mNumHits++];
		h.mFraction = inHit.mFraction;
		h.mBodyID = inHit.mBodyID.GetIndexAndSequenceNumber();
		h.mSubShapeID = inHit.mSubShapeID2.GetValue();
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

// CollidePoint with DecoratedParityFilter
void jolt_decorated_collide_point(const ShapeDesc *inDesc, const float *inPoint, const uint32 *inCreator, uint32 inBodyID, uint32 inRejectSubType, PointOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	AllHitCollisionCollector<CollidePointCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inBodyID));
	collector.SetContext(&context);
	DecoratedParityFilter filter(~uint32(0), inRejectSubType);
	shape->CollidePoint(Load3(inPoint), MakeCreator(inCreator), collector, filter);
	outOutput->mNumHits = uint32(collector.mHits.size());
	outOutput->mBodyID = outOutput->mSubShapeID = 0;
	for (const CollidePointResult &h : collector.mHits)
	{
		outOutput->mBodyID = h.mBodyID.GetIndexAndSequenceNumber();
		outOutput->mSubShapeID = h.mSubShapeID2.GetValue();
	}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// CollectTransformedShapes (with DecoratedParityFilter), TransformShape and the triangles of the collected leaf shapes
// (TransformedShape::GetTrianglesStart / Next)
void jolt_decorated_collect(const ShapeDesc *inDesc, const CollectInput *inInput, CollectOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	CollectOutput &o = *outOutput;

	AllHitCollisionCollector<TransformedShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	DecoratedParityFilter filter(~uint32(0), inInput->mRejectSubType);
	shape->CollectTransformedShapes(LoadAABox(inInput->mBox), Load3(inInput->mPositionCOM), LoadQuat(inInput->mRotation), Load3(inInput->mScale), MakeCreator(inInput->mCreator), collector, filter);
	o.mNumCollected = 0;
	for (const TransformedShape &ts : collector.mHits)
		if (o.mNumCollected < 2)
			StoreTS(ts, o.mCollected[o.mNumCollected++]);
	o.mFilterCalls = filter.mLog.mCalls;
	o.mFilterHash = filter.mLog.mHash;

	AllHitCollisionCollector<TransformedShapeCollector> transformed;
	shape->TransformShape(LoadMat44(inInput->mTransform), transformed);
	o.mNumTransformed = 0;
	for (const TransformedShape &ts : transformed.mHits)
		if (o.mNumTransformed < 2)
			StoreTS(ts, o.mTransformed[o.mNumTransformed++]);

	// The triangles of the first collected leaf
	o.mTriangleCalls = 0;
	if (!collector.mHits.empty())
	{
		const TransformedShape &ts = collector.mHits[0];
		Shape::GetTrianglesContext triangle_context;
		ts.GetTrianglesStart(triangle_context, AABox::sBiggest(), RVec3(Real(inInput->mBaseOffset[0]), Real(inInput->mBaseOffset[1]), Real(inInput->mBaseOffset[2])));
		int max_requested = inInput->mMaxTrianglesRequested;
		Array<Float3> triangles(max_requested * 3);
		Array<const PhysicsMaterial *> materials(max_requested);
		float *out = o.mTriangleVertices;
		int *out_material = o.mDefaultMaterial;
		int num_vertices = 0;
		for (;;)
		{
			int count = ts.GetTrianglesNext(triangle_context, max_requested, triangles.data(), materials.data());
			o.mTriangleCounts[o.mTriangleCalls++] = count;
			for (int i = 0; i < 3 * count && num_vertices < cMaxTriangleVertices; ++i, ++num_vertices, out += 3)
			{
				out[0] = triangles[i].x; out[1] = triangles[i].y; out[2] = triangles[i].z;
			}
			for (int i = 0; i < count && out_material < o.mDefaultMaterial + cMaxTriangleVertices / 3; ++i)
				*out_material++ = materials[i] == PhysicsMaterial::sDefault? 1 : 0;
			if (count == 0 || o.mTriangleCalls == cMaxTriangleCalls)
				break;
		}
	}
}

// CollisionDispatch::sCollideShapeVsShape (all hits in order) with DecoratedParityFilter
void jolt_decorated_collide(const CollideInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mBackFaceMode = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	AllHitCollisionCollector<CollideShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	DecoratedParityFilter filter(inInput->mRejectSubType1, inInput->mRejectSubType2);
	CollisionDispatch::sCollideShapeVsShape(shape1, shape2, Load3(inInput->mScale1), Load3(inInput->mScale2), LoadMat44(inInput->mTransform1), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), settings, collector, filter);
	outOutput->mNumHits = 0;
	for (const CollideShapeResult &r : collector.mHits)
		if (outOutput->mNumHits < 2)
		{
			HitOutput &h = outOutput->mHits[outOutput->mNumHits++];
			h.mFraction = 0.0f;
			h.mBackFace = 0;
			StoreCollideHit(r, h);
		}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace (all hits in order) with DecoratedParityFilter
void jolt_decorated_cast(const CastInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
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
	AllHitCollisionCollector<CastShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	DecoratedParityFilter filter(inInput->mRejectSubType1, inInput->mRejectSubType2);
	CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, shape2, Load3(inInput->mScale2), filter, LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), collector);
	outOutput->mNumHits = 0;
	for (const ShapeCastResult &r : collector.mHits)
		if (outOutput->mNumHits < 2)
		{
			HitOutput &h = outOutput->mHits[outOutput->mNumHits++];
			h.mFraction = r.mFraction;
			h.mBackFace = r.mIsBackFaceHit? 1 : 0;
			StoreCollideHit(r, h);
		}
	outOutput->mFilterCalls = filter.mLog.mCalls;
	outOutput->mFilterHash = filter.mLog.mHash;
}

// GetSubmergedVolume: total volume, submerged volume, center of buoyancy
void jolt_decorated_submerged_volume(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, const float *inPlane, float *outValues)
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
void jolt_decorated_soft_body(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
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

// Binary state of the top shape (outSizes[0] bytes in outBytes), the same after sRestoreFromBinaryState +
// RestoreSubShapeState with the sub shapes of the original (outSizes[1], 0 when the restore fails), SaveWithChildren
// (outSizes[2]) and SaveWithChildren of the shape that sRestoreWithChildren restores from those bytes (outSizes[3], 0
// when the restore fails). Each buffer has inCapacity bytes.
void jolt_decorated_binary_state(const ShapeDesc *inDesc, uint8 *outBytes, uint8 *outRestoredBytes, uint8 *outChildrenBytes, uint8 *outRestoredChildrenBytes, uint32 inCapacity, uint32 *outSizes)
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

} // extern "C"
