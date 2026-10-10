// Reference implementation for the triangle shape parity tests (`zig build parity`): thin C ABI wrappers around the C++
// Jolt library for TriangleShape (and the collision functions it registers in CollisionDispatch: convex vs triangle,
// sphere vs triangle and the reversed versions). The shapes (triangles, spheres and boxes) are built from their settings
// on both sides (ShapeDesc). ZoltParity/Physics/TriangleShapeParity.zig calls these and checks that Zolt produces the
// same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see
// the porting guide). The structs must match the extern structs in TriangleShapeParity.zig.

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
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>

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
EBackFaceMode BackFaceMode(int inMode)				{ return inMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces; }
EActiveEdgeMode ActiveEdgeMode(int inMode)			{ return inMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive; }

void StoreFace(const Shape::SupportingFace &inFace, uint32 &outCount, float *outFace)
{
	outCount = uint32(inFace.size());
	for (uint32 i = 0; i < outCount; ++i)
		Store3(inFace[i], outFace + 3 * i);
}

// Shape description, must match ShapeDesc in TriangleShapeParity.zig
struct ShapeDesc
{
	uint32					mKind;					// 0: TriangleShape, 1: SphereShape, 2: BoxShape
	float					mV1[3];					// TriangleShape
	float					mV2[3];
	float					mV3[3];
	float					mConvexRadius;			// TriangleShape, BoxShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape
	float					mDensity;
	int						mMaterial;				// TriangleShape: 1 to use a material (otherwise the default material)
};

// Build the shape from its settings, null when the settings are invalid
Ref<Shape> CreateShape(const ShapeDesc &inDesc, String *outError = nullptr)
{
	Shape::ShapeResult result;
	switch (inDesc.mKind)
	{
	case 0:
		{
			TriangleShapeSettings settings(Load3(inDesc.mV1), Load3(inDesc.mV2), Load3(inDesc.mV3), inDesc.mConvexRadius, inDesc.mMaterial != 0? new PhysicsMaterialSimple("Parity", Color::sRed) : nullptr);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			result = settings.Create();
			break;
		}

	case 1:
		{
			SphereShapeSettings settings(inDesc.mRadius);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
			result = settings.Create();
			break;
		}

	default:
		{
			BoxShapeSettings settings(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
			settings.SetEmbedded();
			settings.mDensity = inDesc.mDensity;
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

// The properties of a shape, must match PropertiesInput / PropertiesOutput in TriangleShapeParity.zig
struct PropertiesInput
{
	float					mScale[3];				// A valid scale (GetWorldSpaceBounds asserts)
	float					mAnyScale[3];			// IsValidScale / MakeScaleValid
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
	float					mPoint[3];				// GetSurfaceNormal
	float					mDirection[3];			// GetSupportingFace
	float					mPlane[4];				// GetSubmergedVolume
	uint32					mSubShapeID;			// GetLeafShape, GetSubShapeUserData
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
	int						mIsValidScale;
	float					mScaleValid[3];
	int						mIsAnyScaleValid;
	float					mAnyScaleValid[3];
	float					mSurfaceNormal[3];
	uint32					mFaceCount;
	float					mFace[32 * 3];
	float					mSubmerged[5];			// Total volume, submerged volume, center of buoyancy
	int						mLeafIsSelf;
	uint32					mLeafRemainder;
	uint64					mSubShapeUserData;
	int						mMaterialIsDefault;
	int						mMustBeStatic;
};

// The input of a ray cast, must match RayInput / RayOutput in TriangleShapeParity.zig
struct RayInput
{
	float					mOrigin[3];
	float					mDirection[3];
	uint32					mCreator[2];			// Sub shape ID creator: value pushed, number of bits
	float					mFraction;				// Initial fraction of the single hit version
	int						mBackFaceModeTriangles;	// 1: collide with back faces
	int						mBackFaceModeConvex;	// 1: collide with back faces
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

// A collision between 2 shapes, must match CollideInput / CollideOutput in TriangleShapeParity.zig
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
	float					mActiveEdgeMovementDirection[3];
	int						mActiveEdgeMode;		// 1: collide with all edges
	int						mBackFaceMode;			// 1: collide with back faces
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
	float					mEarlyOut;				// The early out fraction of the collector after the query
	HitOutput				mHits[2];
};

// A shape cast, must match CastInput in TriangleShapeParity.zig
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
	float					mActiveEdgeMovementDirection[3];
	int						mActiveEdgeMode;		// 1: collide with all edges
	int						mBackFaceModeTriangles;	// 1: collide with back faces
	int						mBackFaceModeConvex;	// 1: collide with back faces
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

// Create the shape from its settings: returns 1 if valid, otherwise 0 and the error text in outError (128 bytes, 0 terminated)
int jolt_triangle_shape_settings(const ShapeDesc *inDesc, char *outError)
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

// Bounds, mass properties, volume, scales, surface normal, supporting face, submerged volume, leaf shape, user data, material
void jolt_triangle_shape_properties(const ShapeDesc *inDesc, const PropertiesInput *inInput, PropertiesOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	shape->SetUserData(inInput->mUserData);
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
	o.mSubShapeIDBits = shape->GetSubShapeIDBitsRecursive();
	o.mIsValidScale = shape->IsValidScale(scale)? 1 : 0;
	Store3(shape->MakeScaleValid(scale), o.mScaleValid);
	o.mIsAnyScaleValid = shape->IsValidScale(any_scale)? 1 : 0;
	Store3(shape->MakeScaleValid(any_scale), o.mAnyScaleValid);
	Store3(shape->GetSurfaceNormal(SubShapeID(), Load3(inInput->mPoint)), o.mSurfaceNormal);
	Shape::SupportingFace face;
	shape->GetSupportingFace(SubShapeID(), Load3(inInput->mDirection), scale, transform, face);
	StoreFace(face, o.mFaceCount, o.mFace);
	float total_volume, submerged_volume;
	Vec3 center_of_buoyancy;
	shape->GetSubmergedVolume(transform, scale, Plane(Load4(inInput->mPlane)), total_volume, submerged_volume, center_of_buoyancy);
	o.mSubmerged[0] = total_volume;
	o.mSubmerged[1] = submerged_volume;
	Store3(center_of_buoyancy, o.mSubmerged + 2);
	SubShapeID id;
	id.SetValue(inInput->mSubShapeID);
	SubShapeID remainder;
	o.mLeafIsSelf = shape->GetLeafShape(id, remainder) == shape.GetPtr()? 1 : 0;
	o.mLeafRemainder = remainder.GetValue();
	o.mSubShapeUserData = shape->GetSubShapeUserData(id);
	o.mMaterialIsDefault = shape->GetMaterial(SubShapeID()) == PhysicsMaterial::sDefault? 1 : 0; // ConvexShape::GetMaterial asserts an empty ID
	o.mMustBeStatic = shape->MustBeStatic()? 1 : 0;
}

// GetSupportFunction(inMode) and GetSupport for inNumDirections directions
float jolt_triangle_shape_support(const ShapeDesc *inDesc, int inMode, const float *inScale, const float *inDirections, int inNumDirections, float *outPoints)
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
void jolt_triangle_shape_cast_ray(const ShapeDesc *inDesc, const RayInput *inInput, RayOutput *outOutput)
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
	settings.mBackFaceModeTriangles = BackFaceMode(inInput->mBackFaceModeTriangles);
	settings.mBackFaceModeConvex = BackFaceMode(inInput->mBackFaceModeConvex);
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

// CollidePoint: number of hits
uint32 jolt_triangle_shape_collide_point(const ShapeDesc *inDesc, const float *inPoint, const uint32 *inCreator)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	AllHitCollisionCollector<CollidePointCollector> collector;
	shape->CollidePoint(Load3(inPoint), MakeCreator(inCreator), collector);
	return uint32(collector.mHits.size());
}

// CollisionDispatch::sCollideShapeVsShape (all hits in order)
void jolt_triangle_shape_collide(const CollideInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	settings.mActiveEdgeMode = ActiveEdgeMode(inInput->mActiveEdgeMode);
	settings.mBackFaceMode = BackFaceMode(inInput->mBackFaceMode);
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	AllHitCollisionCollector<CollideShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	CollisionDispatch::sCollideShapeVsShape(shape1, shape2, Load3(inInput->mScale1), Load3(inInput->mScale2), LoadMat44(inInput->mTransform1), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), settings, collector);
	outOutput->mNumHits = uint32(collector.mHits.size());
	outOutput->mEarlyOut = collector.GetEarlyOutFraction();
	for (uint32 i = 0; i < outOutput->mNumHits && i < 2; ++i)
	{
		HitOutput &h = outOutput->mHits[i];
		h.mFraction = 0.0f;
		h.mBackFace = 0;
		StoreCollideHit(collector.mHits[i], h);
	}
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace (all hits in order)
void jolt_triangle_shape_cast(const CastInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	Ref<Shape> shape1 = CreateShape(inInput->mShape1);
	Ref<Shape> shape2 = CreateShape(inInput->mShape2);
	ShapeCastSettings settings;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	settings.mActiveEdgeMode = ActiveEdgeMode(inInput->mActiveEdgeMode);
	settings.mBackFaceModeTriangles = BackFaceMode(inInput->mBackFaceModeTriangles);
	settings.mBackFaceModeConvex = BackFaceMode(inInput->mBackFaceModeConvex);
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
	outOutput->mNumHits = uint32(collector.mHits.size());
	outOutput->mEarlyOut = collector.GetEarlyOutFraction();
	for (uint32 i = 0; i < outOutput->mNumHits && i < 2; ++i)
	{
		const ShapeCastResult &r = collector.mHits[i];
		HitOutput &h = outOutput->mHits[i];
		h.mFraction = r.mFraction;
		h.mBackFace = r.mIsBackFaceHit? 1 : 0;
		StoreCollideHit(r, h);
	}
}

// GetTrianglesStart, then GetTrianglesNext with inMaxTrianglesRequested until it returns 0: the counts per call (at most
// 4 calls), the vertices of the first 4 triangles and a flag per triangle (1 if the material is the default material, 0
// for another material, 2 when no materials were requested), returns the number of calls
int jolt_triangle_shape_triangles(const ShapeDesc *inDesc, const float *inPosition, const float *inRotation, const float *inScale, int inMaxTrianglesRequested, int inWithMaterials, int *outCounts, float *outVertices, int *outDefaultMaterial)
{
	EnsureFactory();
	Ref<Shape> shape = CreateShape(*inDesc);
	Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, AABox::sBiggest(), Load3(inPosition), LoadQuat(inRotation), Load3(inScale));
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(inMaxTrianglesRequested, nullptr);
	int calls = 0;
	int num_triangles = 0;
	for (;;)
	{
		int count = shape->GetTrianglesNext(context, inMaxTrianglesRequested, triangles.data(), inWithMaterials != 0? materials.data() : nullptr);
		outCounts[calls++] = count;
		for (int t = 0; t < count; ++t, ++num_triangles)
			if (num_triangles < 4)
			{
				for (int v = 0; v < 3; ++v)
				{
					const Float3 &f = triangles[3 * t + v];
					float *out = outVertices + 9 * num_triangles + 3 * v;
					out[0] = f.x; out[1] = f.y; out[2] = f.z;
				}
				outDefaultMaterial[num_triangles] = inWithMaterials == 0? 2 : (materials[t] == PhysicsMaterial::sDefault? 1 : 0);
			}
		if (count == 0 || calls == 4)
			break;
	}
	return calls;
}

// SaveBinaryState (with user data inUserData), then sRestoreFromBinaryState of those bytes and SaveBinaryState of the
// restored shape (outRestoredSize = 0 when the restore fails). Returns the number of bytes.
uint32 jolt_triangle_shape_binary_state(const ShapeDesc *inDesc, uint64 inUserData, uint8 *outBytes, uint32 inCapacity, uint8 *outRestoredBytes, uint32 *outRestoredSize)
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

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), ioPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_triangle_shape_soft_body(const ShapeDesc *inDesc, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
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
