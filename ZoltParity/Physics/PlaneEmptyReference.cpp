// Reference implementation for the plane and empty shape parity tests (`zig build parity`): thin C ABI wrappers around
// the C++ Jolt library for PlaneShape and EmptyShape, with SphereShape and BoxShape as the convex shapes that collide
// with them. The shapes are built on both sides from a ShapeDesc (from their settings, or with the shape constructor
// that takes the plane / center of mass directly). ZoltParity/Physics/PlaneEmptyParity.zig calls these and checks that
// Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see
// the porting guide). The structs must match the extern structs in PlaneEmptyParity.zig. All C functions are prefixed
// with jolt_plane_empty_ (all wrapper files link into one binary).

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
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
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
Plane LoadPlane(const float *inPlane)				{ return Plane(Load3(inPlane), inPlane[3]); }
void Store3(Vec3Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
void StoreR3(RVec3Arg inV, Real *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
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

// The material of the planes that have one (the Zolt side creates an identical PhysicsMaterialSimple per shape)
const PhysicsMaterial *GetPlaneMaterial()
{
	static RefConst<PhysicsMaterial> sMaterial = new PhysicsMaterialSimple("PlaneMaterial", Color::sRed);
	return sMaterial;
}

// Shape description, must match ShapeDesc in PlaneEmptyParity.zig
// TODO(CapsuleShape): add CapsuleShape once it is ported, so that sCollideConvexVsPlane sees a convex shape with both
// a non zero support point and a non zero convex radius in ESupportMode::Default (see Gen.convex in PlaneEmptyParity.zig)
struct ShapeDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape, 2: PlaneShape, 3: EmptyShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape (PlaneShape: mHalfExtent[0] is the half extent)
	float					mConvexRadius;			// BoxShape
	float					mPlane[4];				// PlaneShape: normal and constant
	float					mCenterOfMass[3];		// EmptyShape
	int						mMaterial;				// PlaneShape: 1 = with GetPlaneMaterial()
	int						mDirect;				// PlaneShape / EmptyShape: 1 = the shape constructor instead of the settings
};

// Build the shape (from its settings unless mDirect), null when the settings are invalid
Ref<Shape> CreateShape(const ShapeDesc &inDesc, String *outError = nullptr)
{
	Shape::ShapeResult result;
	switch (inDesc.mKind)
	{
	case 0:
		{
			SphereShapeSettings settings(inDesc.mRadius);
			settings.SetEmbedded();
			result = settings.Create();
			break;
		}

	case 1:
		{
			BoxShapeSettings settings(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
			settings.SetEmbedded();
			result = settings.Create();
			break;
		}

	case 2:
		{
			const PhysicsMaterial *material = inDesc.mMaterial != 0? GetPlaneMaterial() : nullptr;
			if (inDesc.mDirect != 0)
				return new PlaneShape(LoadPlane(inDesc.mPlane), material, inDesc.mHalfExtent[0]);
			PlaneShapeSettings settings(LoadPlane(inDesc.mPlane), material, inDesc.mHalfExtent[0]);
			settings.SetEmbedded();
			result = settings.Create();
			break;
		}

	default:
		{
			if (inDesc.mDirect != 0)
				return new EmptyShape(Load3(inDesc.mCenterOfMass));
			EmptyShapeSettings settings(Load3(inDesc.mCenterOfMass));
			settings.SetEmbedded();
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

// The properties of a shape, must match PropertiesInput / PropertiesOutput in PlaneEmptyParity.zig
struct PropertiesInput
{
	float					mScale[3];
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
	float					mPoint[3];				// GetSurfaceNormal
	float					mDirection[3];			// GetSupportingFace
	uint32					mSubShapeID;			// GetLeafShape, GetSubShapeUserData, GetSubShapeTransformedShape
	float					mPosition[3];			// GetSubShapeTransformedShape
	float					mRotation[4];
	uint64					mUserData;
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
	int						mMustBeStatic;
	int						mIsValidScale;
	float					mScaleValid[3];
	float					mSurfaceNormal[3];
	int						mMaterialIsDefault;
	uint32					mFaceCount;
	float					mFace[32 * 3];
	int						mLeafIsSelf;
	uint32					mLeafRemainder;
	uint64					mSubShapeUserData;
	Real					mTSPosition[3];
	float					mTSRotation[4];
	float					mTSScale[3];
	uint32					mTSRemainder;
	uint32					mTSBodyID;
	uint32					mTSSubShapeID;
	int						mTSIsSelf;
};

// The input of a ray cast, must match RayInput / RayOutput in PlaneEmptyParity.zig
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

// A collision between 2 shapes, must match CollideInput / CollideOutput in PlaneEmptyParity.zig
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

// A shape cast, must match CastInput in PlaneEmptyParity.zig
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
	float					mExtraConvexRadius;
	int						mBackFaceModeConvex;
	int						mUseShrunkenShape;
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

// Save the binary state of the shape to a string
std::string SaveState(const Shape &inShape)
{
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	inShape.SaveBinaryState(stream_out);
	return data.str();
}

// Copy the bytes to the output buffer (when they fit), returns the size
uint32 CopyBytes(const std::string &inBytes, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inBytes.size());
	if (size <= inCapacity)
		memcpy(outBytes, inBytes.data(), size);
	return size;
}

} // namespace

extern "C" {

// Create the shape from its settings: returns 1 if valid, otherwise 0 and the error text in outError (128 bytes, 0 terminated)
int jolt_plane_empty_settings(const ShapeDesc *inDesc, char *outError)
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

// Bounds, mass properties, volume, scales, surface normal, material, supporting face, leaf shape, sub shape user data and
// GetSubShapeTransformedShape
void jolt_plane_empty_properties(const ShapeDesc *inDesc, const PropertiesInput *inInput, PropertiesOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	shape->SetUserData(inInput->mUserData);
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
	o.mMustBeStatic = shape->MustBeStatic()? 1 : 0;
	o.mIsValidScale = shape->IsValidScale(scale)? 1 : 0;
	Store3(shape->MakeScaleValid(scale), o.mScaleValid);
	Store3(shape->GetSurfaceNormal(SubShapeID(), Load3(inInput->mPoint)), o.mSurfaceNormal);
	o.mMaterialIsDefault = shape->GetMaterial(SubShapeID()) == PhysicsMaterial::sDefault? 1 : 0;
	Shape::SupportingFace face;
	shape->GetSupportingFace(SubShapeID(), Load3(inInput->mDirection), scale, transform, face);
	StoreFace(face, o.mFaceCount, o.mFace);

	SubShapeID id;
	id.SetValue(inInput->mSubShapeID);
	SubShapeID leaf_remainder;
	o.mLeafIsSelf = shape->GetLeafShape(id, leaf_remainder) == shape.GetPtr()? 1 : 0;
	o.mLeafRemainder = leaf_remainder.GetValue();
	o.mSubShapeUserData = shape->GetSubShapeUserData(id);
	SubShapeID ts_remainder;
	TransformedShape ts = shape->GetSubShapeTransformedShape(id, Load3(inInput->mPosition), LoadQuat(inInput->mRotation), scale, ts_remainder);
	StoreR3(ts.mShapePositionCOM, o.mTSPosition);
	Store4(ts.mShapeRotation.GetXYZW(), o.mTSRotation);
	Store3(ts.GetShapeScale(), o.mTSScale);
	o.mTSRemainder = ts_remainder.GetValue();
	o.mTSBodyID = ts.mBodyID.GetIndexAndSequenceNumber();
	o.mTSSubShapeID = ts.mSubShapeIDCreator.GetID().GetValue();
	o.mTSIsSelf = ts.mShape == shape? 1 : 0;
}

// CastRay (the single hit version) and CastRay with a collector
void jolt_plane_empty_cast_ray(const ShapeDesc *inDesc, const RayInput *inInput, RayOutput *outOutput)
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
uint32 jolt_plane_empty_collide_point(const ShapeDesc *inDesc, const float *inPoint, const uint32 *inCreator, uint32 inBodyID, uint32 *outIDs)
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
void jolt_plane_empty_collide(const CollideInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
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
void jolt_plane_empty_cast(const CastInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	ShapeCastSettings settings;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mBackFaceModeConvex = inInput->mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = inInput->mUseShrunkenShape != 0;
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

// GetSubmergedVolume: total volume, submerged volume, center of buoyancy (EmptyShape only, PlaneShape asserts)
void jolt_plane_empty_submerged_volume(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, const float *inPlane, float *outValues)
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
// 4 calls), the vertices (at most 6) and a flag per triangle (1 if the material is the default material), returns the
// number of calls
int jolt_plane_empty_triangles(const ShapeDesc *inDesc, const float *inPosition, const float *inRotation, const float *inScale, int inMaxTrianglesRequested, int *outCounts, float *outVertices, int *outDefaultMaterial)
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
		if (count == 0 || calls == 4)
			break;
	}
	return calls;
}

// SaveBinaryState (with user data inUserData), then sRestoreFromBinaryState of those bytes and SaveBinaryState of the
// restored shape (outRestoredSize = 0 when the restore fails). Returns the number of bytes.
uint32 jolt_plane_empty_binary_state(const ShapeDesc *inDesc, uint64 inUserData, uint8 *outBytes, uint32 inCapacity, uint8 *outRestoredBytes, uint32 *outRestoredSize)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	shape->SetUserData(inUserData);
	std::string bytes = SaveState(*shape);
	uint32 size = CopyBytes(bytes, outBytes, inCapacity);

	*outRestoredSize = 0;
	std::stringstream in_data(bytes);
	StreamInWrapper stream_in(in_data);
	Shape::ShapeResult result = Shape::sRestoreFromBinaryState(stream_in);
	if (result.IsValid())
		*outRestoredSize = CopyBytes(SaveState(*result.Get()), outRestoredBytes, inCapacity);
	return size;
}

// SaveWithChildren (the shape and its material), then sRestoreWithChildren of those bytes and SaveWithChildren of the
// restored shape (outRestoredSize = 0 when the restore fails), outMaterialIsDefault = 1 when the restored shape uses the
// default material. Returns the number of bytes.
uint32 jolt_plane_empty_save_with_children(const ShapeDesc *inDesc, uint8 *outBytes, uint32 inCapacity, uint8 *outRestoredBytes, uint32 *outRestoredSize, int *outMaterialIsDefault)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	Shape::ShapeToIDMap shape_map;
	Shape::MaterialToIDMap material_map;
	shape->SaveWithChildren(stream_out, shape_map, material_map);
	std::string bytes = data.str();
	uint32 size = CopyBytes(bytes, outBytes, inCapacity);

	*outRestoredSize = 0;
	*outMaterialIsDefault = 0;
	std::stringstream in_data(bytes);
	StreamInWrapper stream_in(in_data);
	Shape::IDToShapeMap id_to_shape;
	Shape::IDToMaterialMap id_to_material;
	Shape::ShapeResult result = Shape::sRestoreWithChildren(stream_in, id_to_shape, id_to_material);
	if (result.IsValid())
	{
		*outMaterialIsDefault = result.Get()->GetMaterial(SubShapeID()) == PhysicsMaterial::sDefault? 1 : 0;
		std::stringstream restored;
		StreamOutWrapper restored_out(restored);
		Shape::ShapeToIDMap restored_shape_map;
		Shape::MaterialToIDMap restored_material_map;
		result.Get()->SaveWithChildren(restored_out, restored_shape_map, restored_material_map);
		*outRestoredSize = CopyBytes(restored.str(), outRestoredBytes, inCapacity);
	}
	return size;
}

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), ioPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_plane_empty_soft_body(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
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
