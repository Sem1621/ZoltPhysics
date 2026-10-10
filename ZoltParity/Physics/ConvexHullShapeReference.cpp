// Reference implementation for the convex hull shape parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library for ConvexHullShape (and the support classes, PolyhedronSubmergedVolumeCalculator through
// GetSubmergedVolume, and the convex vs convex collision functions through CollisionDispatch).
// ZoltParity/Physics/ConvexHullShapeParity.zig calls these and checks that Zolt produces the same bits.
//
// Shapes are created once from their settings (jolt_chs_create_hull / _sphere / _box) and passed to the queries as an
// opaque handle (a Shape with one reference, freed with jolt_chs_release), so that the hull is not built again for every
// query.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see
// the porting guide). The structs must match the extern structs in ConvexHullShapeParity.zig.

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
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
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

const Shape *ToShape(const void *inShape)			{ return static_cast<const Shape *>(inShape); }

// Hull description, must match HullDesc in ConvexHullShapeParity.zig
struct HullDesc
{
	const float *			mPoints;				// 3 floats per point
	uint32					mNumPoints;
	float					mMaxConvexRadius;
	float					mMaxErrorConvexRadius;
	float					mHullTolerance;
	float					mDensity;
};

// Everything the public accessors of a hull return, must match AccessorsOutput in ConvexHullShapeParity.zig
struct AccessorsOutput
{
	float					mConvexRadius;
	uint32					mNumPoints;
	float					mPoints[256 * 3];
	uint32					mNumFaces;
	uint32					mNumPlanes;
	float					mPlanes[512 * 4];
	uint32					mNumVerticesInFace[512];
	uint32					mNumVerticesReturned[512];	// GetFaceVertices with inMaxVertices = 3
	uint32					mFirstVertices[512 * 3];
	uint32					mNumVertexIndices;
	uint32					mVertexIndices[2048];		// GetFaceVertices of all faces, concatenated
};

// The properties of a shape, must match PropertiesInput / PropertiesOutput in ConvexHullShapeParity.zig
struct PropertiesInput
{
	float					mScale[3];
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
	float					mPoint[3];				// GetSurfaceNormal
	float					mDirection[3];			// GetSupportingFace
	uint32					mSubShapeID;			// GetLeafShape, GetSubShapeUserData
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
	int						mLeafIsSelf;
	uint32					mLeafRemainder;
	uint64					mSubShapeUserData;
	int						mMaterialIsDefault;
	int						mMustBeStatic;
	float					mSurfaceNormal[3];
	uint32					mFaceCount;
	float					mFace[32 * 3];
};

// The input of a ray cast, must match RayInput / RayOutput in ConvexHullShapeParity.zig
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

// A collision between 2 shapes, must match CollideInput / CollideOutput in ConvexHullShapeParity.zig
struct CollideInput
{
	float					mScale1[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mMaxSeparationDistance;
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	int						mActiveEdgeMode;		// 0: collide only with active, 1: collide with all
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
	HitOutput				mHits[2];
};

// A shape cast, must match CastInput in ConvexHullShapeParity.zig
struct CastInput
{
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
	int						mActiveEdgeMode;		// 0: collide only with active, 1: collide with all
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

// Keeps a reference to the shape for the caller
void *Keep(const ShapeSettings::ShapeResult &inResult)
{
	Shape *shape = inResult.Get().GetPtr();
	shape->AddRef();
	return shape;
}

} // namespace

extern "C" {

// Create a hull from its settings: the shape (with one reference, see jolt_chs_release) or null and the error text in
// outError (128 bytes, 0 terminated)
void *jolt_chs_create_hull(const HullDesc *inDesc, char *outError)
{
	EnsureFactory();
	memset(outError, 0, 128);
	Array<Vec3> points;
	for (uint32 i = 0; i < inDesc->mNumPoints; ++i)
		points.push_back(Load3(inDesc->mPoints + 3 * i));
	ConvexHullShapeSettings settings(points, inDesc->mMaxConvexRadius);
	settings.SetEmbedded();
	settings.mMaxErrorConvexRadius = inDesc->mMaxErrorConvexRadius;
	settings.mHullTolerance = inDesc->mHullTolerance;
	settings.mDensity = inDesc->mDensity;
	ShapeSettings::ShapeResult result = settings.Create();
	if (result.HasError())
	{
		strncpy(outError, result.GetError().c_str(), 127);
		return nullptr;
	}
	return Keep(result);
}

// Create a sphere / box from its settings (the settings must be valid)
void *jolt_chs_create_sphere(float inRadius, float inDensity)
{
	EnsureFactory();
	SphereShapeSettings settings(inRadius);
	settings.SetEmbedded();
	settings.mDensity = inDensity;
	return Keep(settings.Create());
}

void *jolt_chs_create_box(const float *inHalfExtent, float inConvexRadius, float inDensity)
{
	EnsureFactory();
	BoxShapeSettings settings(Load3(inHalfExtent), inConvexRadius);
	settings.SetEmbedded();
	settings.mDensity = inDensity;
	return Keep(settings.Create());
}

// Release the reference of a shape created by jolt_chs_create_*
void jolt_chs_release(void *inShape)
{
	static_cast<Shape *>(inShape)->Release();
}

// Everything the public accessors of the hull return
void jolt_chs_accessors(const void *inShape, AccessorsOutput *outOutput)
{
	const ConvexHullShape *hull = static_cast<const ConvexHullShape *>(ToShape(inShape));
	AccessorsOutput &o = *outOutput;
	o.mConvexRadius = hull->GetConvexRadius();
	o.mNumPoints = hull->GetNumPoints();
	for (uint i = 0; i < o.mNumPoints && i < 256; ++i)
		Store3(hull->GetPoint(i), o.mPoints + 3 * i);
	o.mNumFaces = hull->GetNumFaces();
	const Array<Plane> &planes = hull->GetPlanes();
	o.mNumPlanes = uint32(planes.size());
	for (uint i = 0; i < o.mNumPlanes && i < 512; ++i)
		StorePlane(planes[i], o.mPlanes + 4 * i);
	o.mNumVertexIndices = 0;
	for (uint f = 0; f < o.mNumFaces && f < 512; ++f)
	{
		o.mNumVerticesInFace[f] = hull->GetNumVerticesInFace(f);
		o.mNumVerticesReturned[f] = hull->GetFaceVertices(f, 3, o.mFirstVertices + 3 * f);
		uint indices[256];
		uint num = hull->GetFaceVertices(f, 256, indices);
		for (uint i = 0; i < num && o.mNumVertexIndices < 2048; ++i)
			o.mVertexIndices[o.mNumVertexIndices++] = indices[i];
	}
}

// Bounds, mass properties, volume, scales, leaf shape, surface normal and supporting face
void jolt_chs_properties(const void *inShape, const PropertiesInput *inInput, PropertiesOutput *outOutput)
{
	const Shape *shape = ToShape(inShape);
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
	SubShapeID sub_shape_id;
	sub_shape_id.SetValue(inInput->mSubShapeID);
	SubShapeID remainder;
	o.mLeafIsSelf = shape->GetLeafShape(sub_shape_id, remainder) == shape? 1 : 0;
	o.mLeafRemainder = remainder.GetValue();
	o.mSubShapeUserData = shape->GetSubShapeUserData(sub_shape_id);
	o.mMaterialIsDefault = shape->GetMaterial(SubShapeID()) == PhysicsMaterial::sDefault? 1 : 0;
	o.mMustBeStatic = shape->MustBeStatic()? 1 : 0;
	Store3(shape->GetSurfaceNormal(SubShapeID(), Load3(inInput->mPoint)), o.mSurfaceNormal);
	Shape::SupportingFace face;
	shape->GetSupportingFace(SubShapeID(), Load3(inInput->mDirection), scale, transform, face);
	StoreFace(face, o.mFaceCount, o.mFace);
}

// GetSupportFunction(inMode) and GetSupport for inNumDirections directions, returns the convex radius of the support
// function
float jolt_chs_support(const void *inShape, int inMode, const float *inScale, const float *inDirections, int inNumDirections, float *outPoints)
{
	ConvexShape::SupportBuffer buffer;
	const ConvexShape::Support *support = static_cast<const ConvexShape *>(ToShape(inShape))->GetSupportFunction(ConvexShape::ESupportMode(inMode), buffer, Load3(inScale));
	for (int i = 0; i < inNumDirections; ++i)
		Store3(support->GetSupport(Load3(inDirections + 3 * i)), outPoints + 3 * i);
	return support->GetConvexRadius();
}

// CastRay (the single hit version) and CastRay with a collector
void jolt_chs_cast_ray(const void *inShape, const RayInput *inInput, RayOutput *outOutput)
{
	const Shape *shape = ToShape(inShape);
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
uint32 jolt_chs_collide_point(const void *inShape, const float *inPoint, const uint32 *inCreator, uint32 inBodyID, uint32 *outIDs)
{
	AllHitCollisionCollector<CollidePointCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inBodyID));
	collector.SetContext(&context);
	ToShape(inShape)->CollidePoint(Load3(inPoint), MakeCreator(inCreator), collector);
	outIDs[0] = outIDs[1] = 0;
	for (const CollidePointResult &h : collector.mHits)
	{
		outIDs[0] = h.mBodyID.GetIndexAndSequenceNumber();
		outIDs[1] = h.mSubShapeID2.GetValue();
	}
	return uint32(collector.mHits.size());
}

// CollisionDispatch::sCollideShapeVsShape (the first 2 hits in order)
void jolt_chs_collide(const void *inShape1, const void *inShape2, const CollideInput *inInput, HitsOutput *outOutput)
{
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mBackFaceMode = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	AllHitCollisionCollector<CollideShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	CollisionDispatch::sCollideShapeVsShape(ToShape(inShape1), ToShape(inShape2), Load3(inInput->mScale1), Load3(inInput->mScale2), LoadMat44(inInput->mTransform1), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), settings, collector);
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

// CollisionDispatch::sCastShapeVsShapeWorldSpace (the first 2 hits in order)
void jolt_chs_cast(const void *inShape1, const void *inShape2, const CastInput *inInput, HitsOutput *outOutput)
{
	ShapeCastSettings settings;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mBackFaceModeConvex = inInput->mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = inInput->mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = inInput->mReturnDeepestPoint != 0;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	ShapeCast shape_cast(ToShape(inShape1), Load3(inInput->mScale1), LoadMat44(inInput->mStart), Load3(inInput->mDirection));
	AllHitCollisionCollector<CastShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	collector.SetContext(&context);
	if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
	CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, ToShape(inShape2), Load3(inInput->mScale2), ShapeFilter(), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), collector);
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
void jolt_chs_submerged_volume(const void *inShape, const float *inTransform, const float *inScale, const float *inPlane, float *outValues)
{
	float total_volume, submerged_volume;
	Vec3 center_of_buoyancy;
	ToShape(inShape)->GetSubmergedVolume(LoadMat44(inTransform), Load3(inScale), Plane(Load4(inPlane)), total_volume, submerged_volume, center_of_buoyancy);
	outValues[0] = total_volume;
	outValues[1] = submerged_volume;
	Store3(center_of_buoyancy, outValues + 2);
}

// GetTrianglesStart, then GetTrianglesNext with inMaxTrianglesRequested until it returns 0: the counts per call (at most
// 64 calls), the vertices (at most 512 triangles) and a flag per triangle (1 if the material is the default material),
// returns the number of calls
int jolt_chs_triangles(const void *inShape, const float *inPosition, const float *inRotation, const float *inScale, int inMaxTrianglesRequested, int *outCounts, float *outVertices, int *outDefaultMaterial)
{
	const Shape *shape = ToShape(inShape);
	Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, AABox::sBiggest(), Load3(inPosition), LoadQuat(inRotation), Load3(inScale));
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(inMaxTrianglesRequested);
	int calls = 0;
	int total = 0;
	for (;;)
	{
		int count = shape->GetTrianglesNext(context, inMaxTrianglesRequested, triangles.data(), materials.data());
		outCounts[calls++] = count;
		for (int i = 0; i < count && total < 512; ++i, ++total)
		{
			for (int v = 0; v < 3; ++v)
			{
				const Float3 &t = triangles[3 * i + v];
				float *out = outVertices + 9 * total + 3 * v;
				out[0] = t.x; out[1] = t.y; out[2] = t.z;
			}
			outDefaultMaterial[total] = materials[i] == PhysicsMaterial::sDefault? 1 : 0;
		}
		if (count == 0 || calls == 64)
			break;
	}
	return calls;
}

// SaveBinaryState (with user data inUserData), then sRestoreFromBinaryState of those bytes and SaveBinaryState of the
// restored shape (outRestoredSize = 0 when the restore fails). Returns the number of bytes. inTruncate > 0 restores only
// the first size - inTruncate bytes.
uint32 jolt_chs_binary_state(void *inShape, uint64 inUserData, uint32 inTruncate, uint8 *outBytes, uint32 inCapacity, uint8 *outRestoredBytes, uint32 *outRestoredSize, char *outError)
{
	Shape *shape = static_cast<Shape *>(inShape);
	shape->SetUserData(inUserData);
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	shape->SaveBinaryState(stream_out);
	std::string bytes = data.str();
	uint32 size = uint32(bytes.size());
	if (size <= inCapacity)
		memcpy(outBytes, bytes.data(), size);

	*outRestoredSize = 0;
	memset(outError, 0, 128);
	std::stringstream in_data(bytes.substr(0, size - inTruncate));
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
	else
		strncpy(outError, result.GetError().c_str(), 127);
	return size;
}

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), ioPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_chs_soft_body(const void *inShape, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
{
	Array<Vec3> positions;
	Array<Plane> planes;
	for (int i = 0; i < inNumVertices; ++i)
	{
		positions.push_back(Load3(inPositions + 3 * i));
		planes.push_back(Plane(Load4(ioPlanes + 4 * i)));
	}
	CollideSoftBodyVertexIterator vertices(StridedPtr<const Vec3>(positions.data()), StridedPtr<const float>(inInvMasses), StridedPtr<Plane>(planes.data()), StridedPtr<float>(ioPenetrations), StridedPtr<int>(ioIndices));
	ToShape(inShape)->CollideSoftBodyVertices(LoadMat44(inTransform), Load3(inScale), vertices, uint(inNumVertices), inCollidingShapeIndex);
	for (int i = 0; i < inNumVertices; ++i)
		StorePlane(planes[i], ioPlanes + 4 * i);
}

} // extern "C"
