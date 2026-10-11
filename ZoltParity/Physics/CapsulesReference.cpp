// Reference implementation for the capsule parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library for CapsuleShape and TaperedCapsuleShape. The shapes are built from their settings on both sides (ShapeDesc),
// so the Create() logic that turns a capsule without height into a SphereShape and a tapered capsule whose spheres
// contain each other into a (RotatedTranslatedShape with a) SphereShape is compared too. Spheres and boxes are the
// collision partners. Capsules can have a PhysicsMaterialSimple (ShapeDesc::mMaterial), CollidePoint runs with
// CapsulesParityFilter. ZoltParity/Physics/CapsulesParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see
// the porting guide). The structs must match the extern structs in CapsulesParity.zig.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/ShapeFilter.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>

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
void Store3(Vec3Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
void Store4(Vec4Arg inV, float *outV)				{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
void StoreMat44(Mat44Arg inM, float *outM)			{ for (int c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
void StoreAABox(const AABox &inBox, float *outBox)	{ Store3(inBox.mMin, outBox); Store3(inBox.mMax, outBox + 3); }
void StorePlane(const Plane &inPlane, float *outPlane) { Store3(inPlane.GetNormal(), outPlane); outPlane[3] = inPlane.GetConstant(); }
SubShapeIDCreator MakeCreator(const uint32 *inCreator) { return SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }

void StoreFace(const Shape::SupportingFace &inFace, uint32 &outCount, float *outFace)
{
	outCount = uint32(inFace.size());
	for (uint32 i = 0; i < outCount; ++i)
		Store3(inFace[i], outFace + 3 * i);
}

// Shape description, must match ShapeDesc in CapsulesParity.zig
struct ShapeDesc
{
	uint32					mKind;					// 0: CapsuleShapeSettings::Create, 1: TaperedCapsuleShapeSettings::Create, 2: SphereShapeSettings, 3: BoxShapeSettings, 4: the CapsuleShape constructor, 5: the TaperedCapsuleShape constructor
	float					mHalfHeight;			// Capsule: half height of the cylinder, tapered capsule: half height of the tapered cylinder
	float					mRadius;				// Capsule, sphere, top radius of the tapered capsule
	float					mBottomRadius;			// Tapered capsule
	float					mHalfExtent[3];			// Box
	float					mConvexRadius;			// Box
	float					mDensity;
	uint32					mUserData;
	uint32					mMaterial;				// Capsule, tapered capsule: 0 for none, otherwise the color of a PhysicsMaterialSimple named cMaterialName
};

// Name of the PhysicsMaterialSimple of a ShapeDesc, must match material_name in CapsulesParity.zig
const char *const cMaterialName = "CapsulesParity";

// 1 for the default material, otherwise the debug color (the colors of ShapeDesc::mMaterial have a non zero alpha), must
// match materialCode in CapsulesParity.zig
uint32 MaterialCode(const PhysicsMaterial *inMaterial)
{
	return inMaterial == PhysicsMaterial::sDefault? 1 : inMaterial->GetDebugColor().GetUInt32();
}

// Build the shape from its settings (kinds 4 and 5 call the constructor that takes the settings directly, bypassing the
// sphere logic of Create), null when the settings are invalid
Ref<Shape> CreateShape(const ShapeDesc &inDesc, String *outError = nullptr)
{
	RefConst<PhysicsMaterial> material = inDesc.mMaterial != 0? new PhysicsMaterialSimple(cMaterialName, Color(inDesc.mMaterial)) : nullptr;
	Shape::ShapeResult result;
	switch (inDesc.mKind)
	{
	case 0:
	case 4:
		{
			CapsuleShapeSettings settings(inDesc.mHalfHeight, inDesc.mRadius, material);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			settings.mUserData = inDesc.mUserData;
			if (inDesc.mKind == 0)
				result = settings.Create();
			else
				Ref<Shape> shape = new CapsuleShape(settings, result);
			break;
		}

	case 1:
	case 5:
		{
			TaperedCapsuleShapeSettings settings(inDesc.mHalfHeight, inDesc.mRadius, inDesc.mBottomRadius, material);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			settings.mUserData = inDesc.mUserData;
			if (inDesc.mKind == 1)
				result = settings.Create();
			else
				Ref<Shape> shape = new TaperedCapsuleShape(settings, result);
			break;
		}

	case 2:
		{
			SphereShapeSettings settings(inDesc.mRadius);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			settings.mUserData = inDesc.mUserData;
			result = settings.Create();
			break;
		}

	default:
		{
			BoxShapeSettings settings(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			settings.mUserData = inDesc.mUserData;
			result = settings.Create();
			break;
		}
	}

	if (result.HasError())
	{
		if (outError != nullptr)
			*outError = result.GetError();
		return nullptr;
	}
	return result.Get();
}

// The properties of a shape, must match PropertiesInput / PropertiesOutput in CapsulesParity.zig
struct PropertiesInput
{
	float					mScale[3];
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
	float					mPoint[3];				// GetSurfaceNormal
	float					mDirection[3];			// GetSupportingFace
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
	uint32					mSubShapeIDBits;
	int						mIsValidScale;
	float					mScaleValid[3];
	float					mSurfaceNormal[3];
	uint32					mLeafSubType;
	uint32					mLeafRemainder;
	uint32					mSubShapeUserData;
	uint32					mMaterial;				// MaterialCode of GetMaterial
	uint32					mFaceCount;
	float					mFace[32 * 3];
};

// The input of a ray cast, must match RayInput / RayOutput in CapsulesParity.zig
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
};

// A collision between 2 shapes, must match CollideInput / CollideOutput in CapsulesParity.zig
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
	int						mActiveEdgeMode;		// 1: collide with all
	float					mEarlyOut;				// Early out fraction of the collector (when < FLT_MAX)
	uint32					mBodyID;
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
	HitOutput				mHits[2];
};

// A shape cast, must match CastInput in CapsulesParity.zig
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
	int						mBackFaceModeConvex;
	int						mUseShrunkenShape;
	int						mReturnDeepestPoint;
	int						mCollectFaces;
	float					mEarlyOut;				// Early out fraction of the collector (when < 1 + FLT_EPSILON)
	uint32					mBodyID;
};

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

// The output of CollidePoint, must match PointOutput in CapsulesParity.zig
struct PointOutput
{
	uint32					mNumHits;
	uint32					mBodyID;				// Body ID and sub shape ID of the last hit
	uint32					mSubShapeID;
	uint32					mFilterCalls;
	uint32					mFilterHash;
};

// A filter that rejects a shape sub type and hashes its calls, must match CapsulesParityFilter in CapsulesParity.zig
struct FilterLog
{
	uint32					mCalls = 0;
	uint32					mHash = 0x811c9dc5;

	void					Add(uint32 inValue)
	{
		mHash = (mHash ^ inValue) * 0x01000193;
	}
};

class CapsulesParityFilter : public ShapeFilter
{
public:
	explicit				CapsulesParityFilter(uint32 inRejectSubType) : mRejectSubType(inRejectSubType) { }

	virtual bool			ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		++mLog.mCalls;
		mLog.Add(uint32(inShape2->GetSubType()));
		mLog.Add(inSubShapeIDOfShape2.GetValue());
		return uint32(inShape2->GetSubType()) != mRejectSubType;
	}

	uint32					mRejectSubType;
	mutable FilterLog		mLog;
};

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

// Create the shape from its settings: returns 1 if valid, otherwise 0 and the error text in outError (128 bytes, 0
// terminated). outInfo receives IsValid() / IsSphere() of the settings (capsules only), the sub type, the low 32 bits of
// the user data, the density bits of the shape (0 when it is not convex) and the sub type of the inner shape of a
// decorated shape (0 otherwise).
int jolt_capsules_create(const ShapeDesc *inDesc, char *outError, uint32 *outInfo)
{
	EnsureFactory();
	memset(outError, 0, 128);
	for (int i = 0; i < 6; ++i)
		outInfo[i] = 0;
	if (inDesc->mKind == 0 || inDesc->mKind == 4)
	{
		CapsuleShapeSettings settings(inDesc->mHalfHeight, inDesc->mRadius);
		outInfo[0] = settings.IsValid()? 1 : 0;
		outInfo[1] = settings.IsSphere()? 1 : 0;
	}
	else if (inDesc->mKind == 1 || inDesc->mKind == 5)
	{
		TaperedCapsuleShapeSettings settings(inDesc->mHalfHeight, inDesc->mRadius, inDesc->mBottomRadius);
		outInfo[0] = settings.IsValid()? 1 : 0;
		outInfo[1] = settings.IsSphere()? 1 : 0;
	}
	String error;
	Ref<Shape> shape = CreateShape(*inDesc, &error);
	if (shape == nullptr)
	{
		strncpy(outError, error.c_str(), 127);
		return 0;
	}
	outInfo[2] = uint32(shape->GetSubType());
	outInfo[3] = uint32(shape->GetUserData());
	if (shape->GetType() == EShapeType::Convex)
	{
		float density = static_cast<const ConvexShape *>(shape.GetPtr())->GetDensity();
		memcpy(&outInfo[4], &density, sizeof(float));
	}
	if (shape->GetType() == EShapeType::Decorated)
		outInfo[5] = uint32(static_cast<const DecoratedShape *>(shape.GetPtr())->GetInnerShape()->GetSubType());
	return 1;
}

// IsValidScale and MakeScaleValid on any scale
int jolt_capsules_scale(const ShapeDesc *inDesc, const float *inScale, float *outScaleValid)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Store3(shape->MakeScaleValid(Load3(inScale)), outScaleValid);
	return shape->IsValidScale(Load3(inScale))? 1 : 0;
}

// Bounds, mass properties, volume, scales, surface normal, leaf shape and supporting face
void jolt_capsules_properties(const ShapeDesc *inDesc, const PropertiesInput *inInput, PropertiesOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Vec3 scale = Load3(inInput->mScale);
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
	o.mSubShapeIDBits = shape->GetSubShapeIDBitsRecursive();
	o.mIsValidScale = shape->IsValidScale(scale)? 1 : 0;
	Store3(shape->MakeScaleValid(scale), o.mScaleValid);
	Store3(shape->GetSurfaceNormal(SubShapeID(), Load3(inInput->mPoint)), o.mSurfaceNormal);
	SubShapeID remainder;
	const Shape *leaf = shape->GetLeafShape(SubShapeID(), remainder);
	o.mLeafSubType = leaf != nullptr? uint32(leaf->GetSubType()) : 0xffffffff;
	o.mLeafRemainder = remainder.GetValue();
	o.mSubShapeUserData = uint32(shape->GetSubShapeUserData(SubShapeID()));
	o.mMaterial = MaterialCode(shape->GetMaterial(SubShapeID()));
	Shape::SupportingFace face;
	shape->GetSupportingFace(SubShapeID(), Load3(inInput->mDirection), scale, transform, face);
	StoreFace(face, o.mFaceCount, o.mFace);
}

// GetSupportFunction(inMode) and GetSupport for inNumDirections directions: returns 1 if the shape is convex (and the
// convex radius in outConvexRadius), 0 otherwise
int jolt_capsules_support(const ShapeDesc *inDesc, int inMode, const float *inScale, const float *inDirections, int inNumDirections, float *outPoints, float *outConvexRadius)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	if (shape->GetType() != EShapeType::Convex)
		return 0;
	ConvexShape::SupportBuffer buffer;
	const ConvexShape::Support *support = static_cast<const ConvexShape *>(shape.GetPtr())->GetSupportFunction(ConvexShape::ESupportMode(inMode), buffer, Load3(inScale));
	for (int i = 0; i < inNumDirections; ++i)
		Store3(support->GetSupport(Load3(inDirections + 3 * i)), outPoints + 3 * i);
	*outConvexRadius = support->GetConvexRadius();
	return 1;
}

// CastRay (the single hit version) and CastRay with a collector
void jolt_capsules_cast_ray(const ShapeDesc *inDesc, const RayInput *inInput, RayOutput *outOutput)
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
			shape->CastRay(ray, settings, creator, collector);
			for (const RayCastResult &h : collector.mHits)
				if (outOutput->mNumHits < 4)
					store(h);
			break;
		}

	case 1:
		{
			AnyHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector);
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
			shape->CastRay(ray, settings, creator, collector);
			if (collector.HadHit())
				store(collector.mHit);
			break;
		}
	}
}

// CollidePoint with CapsulesParityFilter (rejecting sub type inRejectSubType, ~0: none)
void jolt_capsules_collide_point(const ShapeDesc *inDesc, const float *inPoint, const uint32 *inCreator, uint32 inBodyID, uint32 inRejectSubType, PointOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	AllHitCollisionCollector<CollidePointCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inBodyID));
	collector.SetContext(&context);
	CapsulesParityFilter filter(inRejectSubType);
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

// CollisionDispatch::sCollideShapeVsShape (all hits in order)
void jolt_capsules_collide(const CollideInput *inInput, HitsOutput *outOutput)
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
	AllHitCollisionCollector<CollideShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	CollisionDispatch::sCollideShapeVsShape(shape1, shape2, Load3(inInput->mScale1), Load3(inInput->mScale2), LoadMat44(inInput->mTransform1), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), settings, collector);
	outOutput->mNumHits = 0;
	for (const CollideShapeResult &r : collector.mHits)
		if (outOutput->mNumHits < 2)
		{
			HitOutput &h = outOutput->mHits[outOutput->mNumHits++];
			h.mFraction = 0.0f;
			h.mBackFace = 0;
			StoreCollideHit(r, h);
		}
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace (all hits in order)
void jolt_capsules_cast(const CastInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	ShapeCastSettings settings;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mBackFaceModeConvex = inInput->mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = inInput->mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = inInput->mReturnDeepestPoint != 0;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	ShapeCast shape_cast(shape1, Load3(inInput->mScale1), LoadMat44(inInput->mStart), Load3(inInput->mDirection));
	AllHitCollisionCollector<CastShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, shape2, Load3(inInput->mScale2), ShapeFilter(), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), collector);
	outOutput->mNumHits = 0;
	for (const ShapeCastResult &r : collector.mHits)
		if (outOutput->mNumHits < 2)
		{
			HitOutput &h = outOutput->mHits[outOutput->mNumHits++];
			h.mFraction = r.mFraction;
			h.mBackFace = r.mIsBackFaceHit? 1 : 0;
			StoreCollideHit(r, h);
		}
}

// GetSubmergedVolume: total volume, submerged volume, center of buoyancy
void jolt_capsules_submerged_volume(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, const float *inPlane, float *outValues)
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

// GetTrianglesStart, then GetTrianglesNext with inMaxTrianglesRequested until it returns 0: the counts per call (at most
// 64 calls), the vertices (at most 480) and the MaterialCode of each triangle, returns the number of calls
int jolt_capsules_triangles(const ShapeDesc *inDesc, const float *inPosition, const float *inRotation, const float *inScale, int inMaxTrianglesRequested, int *outCounts, float *outVertices, uint32 *outMaterials)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, AABox::sBiggest(), Load3(inPosition), LoadQuat(inRotation), Load3(inScale));
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(inMaxTrianglesRequested);
	int calls = 0;
	float *out = outVertices;
	uint32 *out_material = outMaterials;
	for (;;)
	{
		int count = shape->GetTrianglesNext(context, inMaxTrianglesRequested, triangles.data(), materials.data());
		outCounts[calls++] = count;
		for (int i = 0; i < 3 * count; ++i, out += 3)
		{
			out[0] = triangles[i].x; out[1] = triangles[i].y; out[2] = triangles[i].z;
		}
		for (int i = 0; i < count; ++i)
			*out_material++ = MaterialCode(materials[i]);
		if (count == 0 || calls == 64)
			break;
	}
	return calls;
}

// SaveBinaryState, sRestoreFromBinaryState of those bytes (with the sub shapes of the original) and SaveBinaryState of
// the restored shape, SaveWithChildren and sRestoreWithChildren of those bytes and SaveWithChildren of the restored shape.
// outSizes receives the 4 sizes (0 when a restore fails).
void jolt_capsules_binary_state(const ShapeDesc *inDesc, uint8 *outBytes, uint8 *outRestoredBytes, uint8 *outChildrenBytes, uint8 *outRestoredChildrenBytes, uint32 inCapacity, uint32 *outSizes)
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

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), outPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_capsules_soft_body(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
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

} // extern "C"
