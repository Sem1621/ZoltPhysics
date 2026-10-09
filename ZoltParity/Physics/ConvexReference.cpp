// Reference implementation for the convex shape parity tests (`zig build parity`): thin C ABI wrappers around the C++
// Jolt library for ConvexShape, SphereShape and BoxShape (and PolyhedronSubmergedVolumeCalculator through
// ConvexShape::GetSubmergedVolume). The shapes are built from settings on both sides (ShapeDesc); ParityConvexShape is a
// convex shape (UserConvex1) that only provides a support function (a rounded box), the same class as in
// ConvexParity.zig, so that ConvexShape's GJK based fallbacks (CastRay, CollidePoint, GetTrianglesStart / Next) are
// compared too. ZoltParity/Physics/ConvexParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see
// the porting guide). The structs must match the extern structs in ConvexParity.zig.

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
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
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

// A convex shape that only provides a support function: a box with rounded edges (the box shrunk by the convex radius,
// plus the convex radius). Must match ParityConvexShape in ConvexParity.zig. CastRay, CollidePoint, GetTrianglesStart /
// Next and GetSubmergedVolume are ConvexShape's.
class ParityConvexShape final : public ConvexShape
{
public:
							ParityConvexShape(Vec3Arg inHalfExtent, float inConvexRadius) : ConvexShape(EShapeSubType::UserConvex1), mHalfExtent(inHalfExtent), mConvexRadius(inConvexRadius) { }

	virtual AABox			GetLocalBounds() const override								{ return AABox(-mHalfExtent, mHalfExtent); }
	virtual float			GetInnerRadius() const override								{ return mHalfExtent.ReduceMin(); }

	virtual MassProperties	GetMassProperties() const override
	{
		MassProperties p;
		p.SetMassAndInertiaOfSolidBox(2.0f * mHalfExtent, GetDensity());
		return p;
	}

	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { return inLocalSurfacePosition.NormalizedOr(Vec3::sAxisY()); }

	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override
	{
		Vec3 scaled_half_extent = inScale.Abs() * mHalfExtent;
		AABox(-scaled_half_extent, scaled_half_extent).GetSupportingFace(inDirection, outVertices);
		for (Vec3 &v : outVertices)
			v = inCenterOfMassTransform * v;
	}

	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override
	{
		Vec3 scaled_half_extent = inScale.Abs() * mHalfExtent;
		float convex_radius = ScaleHelpers::ScaleConvexRadius(mConvexRadius, inScale);
		Vec3 reduced_half_extent = scaled_half_extent - Vec3::sReplicate(convex_radius);
		AABox box(-reduced_half_extent, reduced_half_extent);
		switch (inMode)
		{
		case ESupportMode::IncludeConvexRadius:
			return new (&inBuffer) RoundedBox(box, convex_radius);

		case ESupportMode::ExcludeConvexRadius:
		case ESupportMode::Default:
		default:
			return new (&inBuffer) ShrunkBox(box, convex_radius);
		}
	}

	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override { }
	virtual Stats			GetStats() const override									{ return Stats(sizeof(*this), 0); }
	virtual float			GetVolume() const override									{ return GetLocalBounds().GetVolume(); }

private:
	// The box including the convex radius (rounded edges)
	class RoundedBox final : public Support
	{
	public:
							RoundedBox(const AABox &inBox, float inRadius) : mBox(inBox), mRadius(inRadius) { }

		virtual Vec3		GetSupport(Vec3Arg inDirection) const override
		{
			float len = inDirection.Length();
			Vec3 p = mBox.GetSupport(inDirection);
			return len > 0.0f? p + (mRadius / len) * inDirection : p;
		}

		virtual float		GetConvexRadius() const override							{ return 0.0f; }

	private:
		AABox				mBox;
		float				mRadius;
	};

	// The box excluding the convex radius
	class ShrunkBox final : public Support
	{
	public:
							ShrunkBox(const AABox &inBox, float inRadius) : mBox(inBox), mRadius(inRadius) { }

		virtual Vec3		GetSupport(Vec3Arg inDirection) const override				{ return mBox.GetSupport(inDirection); }
		virtual float		GetConvexRadius() const override							{ return mRadius; }

	private:
		AABox				mBox;
		float				mRadius;
	};

	Vec3					mHalfExtent;
	float					mConvexRadius;
};

// Exposes the protected ConvexShape::sUnitSphereTriangles
class ExposeConvexShape : public ConvexShape
{
public:
	static const StaticArray<Vec3, 384> &	sGetUnitSphereTriangles()					{ return sUnitSphereTriangles; }
};

// Shape description, must match ShapeDesc in ConvexParity.zig
struct ShapeDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape, 2: ParityConvexShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape, ParityConvexShape
	float					mConvexRadius;			// BoxShape, ParityConvexShape
	float					mDensity;
};

// Build the shape (spheres and boxes from their settings), null when the settings are invalid
Ref<Shape> CreateShape(const ShapeDesc &inDesc, String *outError = nullptr)
{
	Shape::ShapeResult result;
	switch (inDesc.mKind)
	{
	case 0:
		{
			SphereShapeSettings settings(inDesc.mRadius);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			result = settings.Create();
			break;
		}

	case 1:
		{
			BoxShapeSettings settings(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			result = settings.Create();
			break;
		}

	default:
		{
			ParityConvexShape *shape = new ParityConvexShape(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
			shape->SetDensity(inDesc.mDensity);
			return Ref<Shape>(shape);
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

// The properties of a shape, must match PropertiesInput / PropertiesOutput in ConvexParity.zig
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
	uint32					mFaceCount;
	float					mFace[32 * 3];
};

// The input of a ray cast, must match RayInput / RayOutput in ConvexParity.zig
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

// A collision between 2 shapes, must match CollideInput / CollideOutput in ConvexParity.zig
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

// A shape cast, must match CastInput in ConvexParity.zig
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

} // namespace

extern "C" {

// ConvexShape::sUnitSphereTriangles (384 vertices)
void jolt_convex_unit_sphere(float *outVertices)
{
	EnsureFactory();
	const StaticArray<Vec3, 384> &triangles = ExposeConvexShape::sGetUnitSphereTriangles();
	for (uint i = 0; i < triangles.size(); ++i)
		Store3(triangles[i], outVertices + 3 * i);
}

// Create the shape from its settings: returns 1 if valid, otherwise 0 and the error text in outError (128 bytes, 0 terminated)
int jolt_convex_settings(const ShapeDesc *inDesc, char *outError)
{
	EnsureFactory();
	memset(outError, 0, 128);
	String error;
	Ref<Shape> shape = CreateShape(*inDesc, &error);
	if (shape != nullptr)
		return 1;
	strncpy(outError, error.c_str(), 127);
	return 0;
}

// Bounds, mass properties, volume, scales, surface normal and supporting face
void jolt_convex_properties(const ShapeDesc *inDesc, const PropertiesInput *inInput, PropertiesOutput *outOutput)
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
	Shape::SupportingFace face;
	shape->GetSupportingFace(SubShapeID(), Load3(inInput->mDirection), scale, transform, face);
	StoreFace(face, o.mFaceCount, o.mFace);
}

// GetSupportFunction(inMode) and GetSupport for inNumDirections directions
float jolt_convex_support(const ShapeDesc *inDesc, int inMode, const float *inScale, const float *inDirections, int inNumDirections, float *outPoints)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	ConvexShape::SupportBuffer buffer;
	const ConvexShape::Support *support = static_cast<const ConvexShape *>(shape.GetPtr())->GetSupportFunction(ConvexShape::ESupportMode(inMode), buffer, Load3(inScale));
	for (int i = 0; i < inNumDirections; ++i)
		Store3(support->GetSupport(Load3(inDirections + 3 * i)), outPoints + 3 * i);
	return support->GetConvexRadius();
}

// CastRay (the single hit version) and CastRay with a collector
void jolt_convex_cast_ray(const ShapeDesc *inDesc, const RayInput *inInput, RayOutput *outOutput)
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

// CollidePoint: number of hits, body ID and sub shape ID of the last hit
uint32 jolt_convex_collide_point(const ShapeDesc *inDesc, const float *inPoint, const uint32 *inCreator, uint32 inBodyID, uint32 *outIDs)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	AllHitCollisionCollector<CollidePointCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inBodyID));
	collector.SetContext(&context);
	shape->CollidePoint(Load3(inPoint), MakeCreator(inCreator), collector);
	outIDs[0] = outIDs[1] = 0;
	for (const CollidePointResult &h : collector.mHits)
	{
		outIDs[0] = h.mBodyID.GetIndexAndSequenceNumber();
		outIDs[1] = h.mSubShapeID2.GetValue();
	}
	return uint32(collector.mHits.size());
}

// CollisionDispatch::sCollideShapeVsShape (all hits in order)
void jolt_convex_collide(const CollideInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
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
void jolt_convex_cast(const CastInput *inInput, HitsOutput *outOutput)
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
void jolt_convex_submerged_volume(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, const float *inPlane, float *outValues)
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
// 64 calls), the vertices (at most 384) and a flag per triangle (1 if the material is the default material), returns the
// number of calls
int jolt_convex_triangles(const ShapeDesc *inDesc, const float *inPosition, const float *inRotation, const float *inScale, int inMaxTrianglesRequested, int *outCounts, float *outVertices, int *outDefaultMaterial)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, AABox::sBiggest(), Load3(inPosition), LoadQuat(inRotation), Load3(inScale));
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(inMaxTrianglesRequested);
	int calls = 0;
	float *out = outVertices;
	int *out_material = outDefaultMaterial;
	for (;;)
	{
		int count = shape->GetTrianglesNext(context, inMaxTrianglesRequested, triangles.data(), materials.data());
		outCounts[calls++] = count;
		for (int i = 0; i < 3 * count; ++i, out += 3)
		{
			out[0] = triangles[i].x; out[1] = triangles[i].y; out[2] = triangles[i].z;
		}
		for (int i = 0; i < count; ++i)
			*out_material++ = materials[i] == PhysicsMaterial::sDefault? 1 : 0;
		if (count == 0 || calls == 64)
			break;
	}
	return calls;
}

// SaveBinaryState (with user data inUserData), then sRestoreFromBinaryState of those bytes and SaveBinaryState of the
// restored shape (outRestoredSize = 0 when the restore fails). Returns the number of bytes.
uint32 jolt_convex_binary_state(const ShapeDesc *inDesc, uint64 inUserData, uint8 *outBytes, uint32 inCapacity, uint8 *outRestoredBytes, uint32 *outRestoredSize)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	shape->SetUserData(inUserData);
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	shape->SaveBinaryState(stream_out);
	std::string bytes = data.str();
	uint32 size = uint32(bytes.size());
	if (size <= inCapacity)
		memcpy(outBytes, bytes.data(), size);

	*outRestoredSize = 0;
	std::stringstream in_data(bytes);
	StreamInWrapper stream_in(in_data);
	Shape::ShapeResult result = Shape::sRestoreFromBinaryState(stream_in);
	if (result.IsValid())
	{
		std::stringstream restored;
		StreamOutWrapper restored_out(restored);
		result.Get()->SaveBinaryState(restored_out);
		std::string restored_bytes = restored.str();
		*outRestoredSize = uint32(restored_bytes.size());
		if (*outRestoredSize <= inCapacity)
			memcpy(outRestoredBytes, restored_bytes.data(), *outRestoredSize);
	}
	return size;
}

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), outPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_convex_soft_body(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
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
