// Reference implementation for the MeshShape parity tests (`zig build parity`): C ABI wrappers around the C++ Jolt
// library for MeshShapeSettings (the constructors, Sanitize) and MeshShape (construction with its error texts, the tree,
// the properties, the per triangle queries, CastRay, CollidePoint, collide and cast through CollisionDispatch and
// InternalEdgeRemovingCollector, GetTrianglesStart / Next, CollideSoftBodyVertices, GetSubmergedVolume and the binary
// state). ZoltParity/Physics/MeshShapeParity.zig does the same with Zolt and checks that both produce the same bits.
//
// Conventions: every function writes its results into a stream of uint32 (floats as their bits, strings and byte
// arrays as their length followed by one uint32 per character / byte, materials as described at WriteMaterial) and
// returns the number of values in the stream (only the first inCapacity values are written). The stream layout must
// match MeshShapeParity.zig. Vectors are passed as float arrays, Mat44 as 16 floats in column major order, booleans as
// int (never bool, see the porting guide). A mesh is created once (jolt_mesh_create returns a handle that holds a
// reference) and queried by handle. All functions are prefixed with jolt_mesh_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/InternalEdgeRemovingCollector.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

static_assert(sizeof(IndexedTriangle) == 20, "Zolt passes IndexedTriangle as raw memory");
static_assert(sizeof(Triangle) == 44, "Zolt passes Triangle as raw memory");
static_assert(sizeof(Float3) == 12, "Zolt passes Float3 as raw memory");

namespace {

// Number of materials that the tests can use, must match num_parity_materials in MeshShapeParity.zig
constexpr uint cNumMaterials = 40;

// Section markers in the streams, must match MeshShapeParity.zig
enum : uint32
{
	cMarkerSettings = 0xB0000001,
	cMarkerResult = 0xB0000002,
	cMarkerProperties = 0xB0000003,
	cMarkerSubShape = 0xB0000004,
	cMarkerRay = 0xB0000005,
	cMarkerRayCollector = 0xB0000006,
	cMarkerPoint = 0xB0000007,
	cMarkerCollide = 0xB0000008,
	cMarkerCast = 0xB0000009,
	cMarkerTriangles = 0xB000000A,
	cMarkerSoftBody = 0xB000000B,
	cMarkerBinaryState = 0xB000000C,
	cMarkerSubmerged = 0xB000000D,
	cMarkerHit = 0xB000000E,
};

void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

// The materials of the tests: "Material i" with Color::sGetDistinctColor(i)
const Array<RefConst<PhysicsMaterial>> &GetMaterials()
{
	static Array<RefConst<PhysicsMaterial>> sMaterials;
	if (sMaterials.empty())
		for (uint i = 0; i < cNumMaterials; ++i)
			sMaterials.push_back(new PhysicsMaterialSimple("Material " + std::to_string(i), Color::sGetDistinctColor(int(i))));
	return sMaterials;
}

Vec3 Load3(const float *inV)						{ return Vec3(inV[0], inV[1], inV[2]); }
Vec4 Load4(const float *inV)						{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
Quat LoadQuat(const float *inV)						{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
Mat44 LoadMat44(const float *inM)					{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
SubShapeIDCreator MakeCreator(const uint32 *inCreator) { return SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }
SubShapeID MakeID(uint32 inValue)					{ SubShapeID id; id.SetValue(inValue); return id; }

// Output stream, must match Stream in MeshShapeParity.zig
class OutStream
{
public:
						OutStream(uint32 *inData, uint32 inCapacity) : mData(inData), mCapacity(inCapacity) { }

	void				U(uint32 inValue)			{ if (mSize < mCapacity) mData[mSize] = inValue; ++mSize; }
	void				I(int inValue)				{ U(uint32(inValue)); }
	void				B(bool inValue)				{ U(inValue? 1 : 0); }
	void				F(float inValue)			{ uint32 u; memcpy(&u, &inValue, sizeof(u)); U(u); }
	void				D(double inValue)			{ uint64 u; memcpy(&u, &inValue, sizeof(u)); U(uint32(u)); U(uint32(u >> 32)); }
	void				V3(Vec3Arg inValue)			{ F(inValue.GetX()); F(inValue.GetY()); F(inValue.GetZ()); }
	void				F3(const Float3 &inValue)	{ F(inValue.x); F(inValue.y); F(inValue.z); }
	void				R3(RVec3Arg inValue)		{ D(double(inValue.GetX())); D(double(inValue.GetY())); D(double(inValue.GetZ())); }
	void				Q(QuatArg inValue)			{ F(inValue.GetX()); F(inValue.GetY()); F(inValue.GetZ()); F(inValue.GetW()); }
	void				M(Mat44Arg inValue)			{ for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) F(inValue(r, c)); }
	void				Box(const AABox &inBox)		{ V3(inBox.mMin); V3(inBox.mMax); }
	void				P(const Plane &inPlane)		{ V3(inPlane.GetNormal()); F(inPlane.GetConstant()); }
	void				Str(const char *inString)	{ uint32 len = uint32(strlen(inString)); U(len); for (uint32 i = 0; i < len; ++i) U(uint8(inString[i])); }
	void				Bytes(const std::string &inBytes) { U(uint32(inBytes.size())); for (char c : inBytes) U(uint8(c)); }
	void				Face(const Shape::SupportingFace &inFace) { U(uint32(inFace.size())); for (Vec3 v : inFace) V3(v); }

	uint32 *			mData;
	uint32				mCapacity;
	uint32				mSize = 0;
};

// A material: the index in GetMaterials(), 0xffffffff for the default material, otherwise 0xfffffffe, the name and the color
void WriteMaterial(OutStream &ioStream, const PhysicsMaterial *inMaterial)
{
	if (inMaterial == PhysicsMaterial::sDefault)
	{
		ioStream.U(0xffffffff);
		return;
	}
	const Array<RefConst<PhysicsMaterial>> &materials = GetMaterials();
	for (uint i = 0; i < materials.size(); ++i)
		if (materials[i] == inMaterial)
		{
			ioStream.U(i);
			return;
		}
	ioStream.U(0xfffffffe);
	ioStream.Str(inMaterial->GetDebugName());
	ioStream.U(inMaterial->GetDebugColor().GetUInt32());
}

void WriteCollideHit(OutStream &ioStream, const CollideShapeResult &inResult)
{
	ioStream.U(cMarkerHit);
	ioStream.V3(inResult.mContactPointOn1);
	ioStream.V3(inResult.mContactPointOn2);
	ioStream.V3(inResult.mPenetrationAxis);
	ioStream.F(inResult.mPenetrationDepth);
	ioStream.U(inResult.mSubShapeID1.GetValue());
	ioStream.U(inResult.mSubShapeID2.GetValue());
	ioStream.U(inResult.mBodyID2.GetIndexAndSequenceNumber());
	ioStream.Face(inResult.mShape1Face);
	ioStream.Face(inResult.mShape2Face);
}

// Mesh description, must match MeshInput in MeshShapeParity.zig
struct MeshInput
{
	const Float3 *			mVertices;
	const IndexedTriangle *	mTriangles;
	const Triangle *		mTriangleList;
	uint64					mUserData;
	uint32					mNumVertices;
	uint32					mNumTriangles;
	uint32					mNumTriangleList;
	uint32					mMode;					// 0: arrays set directly, 1: MeshShapeSettings(VertexList, IndexedTriangleList, PhysicsMaterialList), 2: MeshShapeSettings(TriangleList, PhysicsMaterialList)
	uint32					mNumMaterials;			// The first mNumMaterials of GetMaterials() (modulo cNumMaterials)
	uint32					mMaxTrianglesPerLeaf;
	float					mActiveEdgeCosThresholdAngle;
	int						mPerTriangleUserData;
	uint32					mBuildQuality;
	uint32					mPadding;
};

// The convex shape of a query, must match ConvexDesc in MeshShapeParity.zig
struct ConvexDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape
	float					mConvexRadius;			// BoxShape
};

Ref<Shape> CreateConvex(const ConvexDesc &inDesc)
{
	Shape::ShapeResult result;
	if (inDesc.mKind == 0)
	{
		SphereShapeSettings settings(inDesc.mRadius);
		settings.SetEmbedded();
		result = settings.Create();
	}
	else
	{
		BoxShapeSettings settings(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
		settings.SetEmbedded();
		result = settings.Create();
	}
	return result.Get();
}

// Properties input, must match PropertiesInput in MeshShapeParity.zig
struct PropertiesInput
{
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
	float					mScale[3];
	float					mSurfacePlane[4];		// GetSubmergedVolume
	int						mSubmerged;				// Call GetSubmergedVolume (Jolt asserts, only when asserts are off in Zolt)
};

// Sub shape input, must match SubShapeInput in MeshShapeParity.zig
struct SubShapeInput
{
	float					mTransform[16];
	float					mScale[3];
	float					mDirection[3];
	float					mPosition[3];
	float					mRotation[4];
	float					mPoint[3];
};

// Ray input, must match RayInput in MeshShapeParity.zig
struct RayInput
{
	float					mOrigin[3];
	float					mDirection[3];
	uint32					mCreator[2];			// Sub shape ID creator: value pushed, number of bits
	float					mFraction;				// Initial fraction of the single hit version
	int						mBackFaceMode;			// 1: collide with back faces (triangles)
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
	float					mEarlyOut;				// Early out fraction of the collector (when < the initial one)
	uint32					mBodyID;				// Body ID of the collector context
	int						mRejectAll;				// Use a shape filter that rejects everything
};

// Collide input, must match CollideInput in MeshShapeParity.zig
struct CollideInput
{
	ConvexDesc				mConvex;
	int						mMeshFirst;				// 1: the mesh is shape 1 (reversed)
	float					mScaleConvex[3];
	float					mScaleMesh[3];
	float					mTransformConvex[16];
	float					mTransformMesh[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mMaxSeparationDistance;
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	int						mBackFaceMode;			// 1: CollideWithBackFaces
	int						mActiveEdgeMode;		// 1: CollideWithAll
	int						mCollectFaces;
	float					mActiveEdgeMovementDirection[3];
	float					mEarlyOut;				// Early out fraction of the collector (when < FLT_MAX)
	uint32					mBodyID;
	int						mInternalEdgeRemoval;	// Use InternalEdgeRemovingCollector::sCollideShapeVsShape
	float					mVertexToleranceSq;		// mInternalEdgeRemovalVertexToleranceSq
};

// Cast input, must match CastInput in MeshShapeParity.zig
struct CastInput
{
	ConvexDesc				mConvex;
	int						mMeshCast;				// 1: the mesh is the cast shape, the convex shape the target
	float					mScaleCast[3];
	float					mStart[16];
	float					mDirection[3];
	float					mScaleTarget[3];
	float					mTransformTarget[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	float					mExtraConvexRadius;
	int						mBackFaceModeTriangles;
	int						mBackFaceModeConvex;
	int						mUseShrunkenShape;
	int						mReturnDeepestPoint;
	int						mCollectFaces;
	int						mActiveEdgeMode;		// 1: CollideWithAll
	float					mActiveEdgeMovementDirection[3];
	float					mEarlyOut;				// Early out fraction of the collector (when < 1 + FLT_EPSILON)
	uint32					mBodyID;
};

// GetTriangles input, must match TrianglesInput in MeshShapeParity.zig
struct TrianglesInput
{
	float					mBox[6];
	float					mPosition[3];
	float					mRotation[4];
	float					mScale[3];
	int						mMaxTrianglesRequested;
	int						mMaterials;				// Request the materials
};

class RejectAllFilter : public ShapeFilter
{
public:
	using ShapeFilter::ShouldCollide;

	virtual bool			ShouldCollide([[maybe_unused]] const Shape *inShape2, [[maybe_unused]] const SubShapeID &inSubShapeIDOfShape2) const override { return false; }
};

// Write SaveBinaryState of a shape
std::string SaveBinaryState(const Shape *inShape)
{
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	inShape->SaveBinaryState(stream_out);
	return data.str();
}

// Write SaveWithChildren of a shape
std::string SaveWithChildren(const Shape *inShape)
{
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	Shape::ShapeToIDMap shape_map;
	Shape::MaterialToIDMap material_map;
	inShape->SaveWithChildren(stream_out, shape_map, material_map);
	return data.str();
}

} // namespace

extern "C" {

// Create the mesh: the settings after the constructor (Sanitize), the result (error text) and the properties of the
// shape. Returns a handle (a reference to the shape) or null when the settings are invalid.
void *jolt_mesh_create(const MeshInput *inInput, uint32 *outStream, uint32 inCapacity, uint32 *outSize)
{
	EnsureFactory();
	OutStream s(outStream, inCapacity);
	const MeshInput &in = *inInput;

	PhysicsMaterialList materials;
	for (uint i = 0; i < in.mNumMaterials; ++i)
		materials.push_back(GetMaterials()[i % cNumMaterials]);

	Ref<MeshShapeSettings> settings;
	switch (in.mMode)
	{
	case 0:
		settings = new MeshShapeSettings();
		settings->mTriangleVertices.assign(in.mVertices, in.mVertices + in.mNumVertices);
		settings->mIndexedTriangles.assign(in.mTriangles, in.mTriangles + in.mNumTriangles);
		settings->mMaterials = materials;
		break;

	case 1:
		settings = new MeshShapeSettings(VertexList(in.mVertices, in.mVertices + in.mNumVertices), IndexedTriangleList(in.mTriangles, in.mTriangles + in.mNumTriangles), materials);
		break;

	default:
		settings = new MeshShapeSettings(TriangleList(in.mTriangleList, in.mTriangleList + in.mNumTriangleList), materials);
		break;
	}
	settings->mMaxTrianglesPerLeaf = in.mMaxTrianglesPerLeaf;
	settings->mActiveEdgeCosThresholdAngle = in.mActiveEdgeCosThresholdAngle;
	settings->mPerTriangleUserData = in.mPerTriangleUserData != 0;
	settings->mBuildQuality = MeshShapeSettings::EBuildQuality(in.mBuildQuality);
	settings->mUserData = in.mUserData;

	// The settings after the constructor
	s.U(cMarkerSettings);
	s.U(uint32(settings->mTriangleVertices.size()));
	for (const Float3 &v : settings->mTriangleVertices)
		s.F3(v);
	s.U(uint32(settings->mIndexedTriangles.size()));
	for (const IndexedTriangle &t : settings->mIndexedTriangles)
	{
		s.U(t.mIdx[0]); s.U(t.mIdx[1]); s.U(t.mIdx[2]);
		s.U(t.mMaterialIndex);
		s.U(t.mUserData);
	}
	s.U(uint32(settings->mMaterials.size()));

	// The result
	Shape::ShapeResult result = settings->Create();
	s.U(cMarkerResult);
	if (result.HasError())
	{
		s.U(0);
		s.Str(result.GetError().c_str());
		*outSize = s.mSize;
		return nullptr;
	}
	s.U(1);
	const Shape *shape = result.Get();
	const MeshShape *mesh = static_cast<const MeshShape *>(shape);

	// The properties that don't depend on an input
	s.U(cMarkerProperties);
	s.U(uint32(shape->GetType()));
	s.U(uint32(shape->GetSubType()));
	s.B(shape->MustBeStatic());
	s.Box(shape->GetLocalBounds());
	s.V3(shape->GetCenterOfMass());
	s.U(shape->GetSubShapeIDBitsRecursive());
	s.F(shape->GetInnerRadius());
	s.F(shape->GetVolume());
	MassProperties p = shape->GetMassProperties();
	s.F(p.mMass);
	s.M(p.mInertia);
	Shape::Stats stats = shape->GetStats();
	s.U(stats.mNumTriangles);
	s.U(uint32(stats.mSizeBytes - sizeof(MeshShape)));
	s.U(uint32(mesh->GetMaterialList().size()));
	for (const PhysicsMaterialRefC &m : mesh->GetMaterialList())
		WriteMaterial(s, m);
	s.U(uint32(shape->GetUserData()));
	s.U(uint32(shape->GetUserData() >> 32));

	// The binary state contains the tree
	s.Bytes(SaveBinaryState(shape));

	*outSize = s.mSize;
	shape->AddRef();
	return const_cast<Shape *>(shape);
}

// Release the reference of jolt_mesh_create
void jolt_mesh_destroy(void *inHandle)
{
	static_cast<Shape *>(inHandle)->Release();
}

// GetWorldSpaceBounds (Mat44 and DMat44), IsValidScale, MakeScaleValid and GetSubmergedVolume
uint32 jolt_mesh_properties(void *inHandle, const PropertiesInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	const Shape *shape = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	Mat44 transform = LoadMat44(inInput->mTransform);
	Vec3 scale = Load3(inInput->mScale);
	s.U(cMarkerProperties);
	s.Box(shape->GetWorldSpaceBounds(transform, scale));
	s.Box(shape->GetWorldSpaceBounds(DMat44(transform, DVec3(inInput->mTranslation[0], inInput->mTranslation[1], inInput->mTranslation[2])), scale));
	s.B(shape->IsValidScale(scale));
	s.V3(shape->MakeScaleValid(scale));
	if (inInput->mSubmerged != 0)
	{
		// Not supported: Jolt asserts and leaves the out parameters untouched
		float total_volume = 0.0f, submerged_volume = 0.0f;
		Vec3 center_of_buoyancy = Vec3::sZero();
		shape->GetSubmergedVolume(transform, scale, Plane(Load4(inInput->mSurfacePlane)), total_volume, submerged_volume, center_of_buoyancy);
		s.U(cMarkerSubmerged);
		s.F(total_volume);
		s.F(submerged_volume);
		s.V3(center_of_buoyancy);
	}
	return s.mSize;
}

// The queries for one sub shape ID each: material, material index, user data, surface normal, supporting face,
// leaf shape, sub shape user data, sub shape transformed shape
uint32 jolt_mesh_sub_shapes(void *inHandle, const uint32 *inIDs, uint32 inNumIDs, const SubShapeInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	const Shape *shape = static_cast<const Shape *>(inHandle);
	const MeshShape *mesh = static_cast<const MeshShape *>(shape);
	OutStream s(outStream, inCapacity);
	Mat44 transform = LoadMat44(inInput->mTransform);
	Vec3 scale = Load3(inInput->mScale);
	for (uint32 i = 0; i < inNumIDs; ++i)
	{
		SubShapeID id = MakeID(inIDs[i]);
		s.U(cMarkerSubShape);
		WriteMaterial(s, shape->GetMaterial(id));
		s.U(mesh->GetMaterialIndex(id));
		s.U(mesh->GetTriangleUserData(id));
		s.V3(shape->GetSurfaceNormal(id, Load3(inInput->mPoint)));
		Shape::SupportingFace face;
		shape->GetSupportingFace(id, Load3(inInput->mDirection), scale, transform, face);
		s.Face(face);
		SubShapeID remainder;
		const Shape *leaf = shape->GetLeafShape(id, remainder);
		s.B(leaf == shape);
		s.U(remainder.GetValue());
		s.U(uint32(shape->GetSubShapeUserData(id)));
		TransformedShape ts = shape->GetSubShapeTransformedShape(id, Load3(inInput->mPosition), LoadQuat(inInput->mRotation), scale, remainder);
		s.B(ts.mShape == shape);
		s.R3(ts.mShapePositionCOM);
		s.Q(ts.mShapeRotation);
		s.V3(ts.GetShapeScale());
		s.U(ts.mBodyID.GetIndexAndSequenceNumber());
		s.U(ts.mSubShapeIDCreator.GetID().GetValue());
		s.U(remainder.GetValue());
	}
	return s.mSize;
}

// CastRay: the single hit version and the collector version
uint32 jolt_mesh_cast_ray(void *inHandle, const RayInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	const Shape *shape = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	RayCast ray { Load3(inInput->mOrigin), Load3(inInput->mDirection) };
	SubShapeIDCreator creator = MakeCreator(inInput->mCreator);

	RayCastResult hit;
	hit.mFraction = inInput->mFraction;
	s.U(cMarkerRay);
	s.B(shape->CastRay(ray, creator, hit));
	s.F(hit.mFraction);
	s.U(hit.mSubShapeID2.GetValue());

	RayCastSettings settings;
	settings.mBackFaceModeTriangles = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	RejectAllFilter reject_all;
	ShapeFilter default_filter;
	const ShapeFilter &filter = inInput->mRejectAll != 0? static_cast<const ShapeFilter &>(reject_all) : default_filter;
	auto write = [&s](const RayCastResult &inHit) {
		s.F(inHit.mFraction);
		s.U(inHit.mBodyID.GetIndexAndSequenceNumber());
		s.U(inHit.mSubShapeID2.GetValue());
	};
	s.U(cMarkerRayCollector);
	switch (inInput->mCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector, filter);
			s.U(uint32(collector.mHits.size()));
			for (const RayCastResult &h : collector.mHits)
				write(h);
			s.F(collector.GetEarlyOutFraction());
			break;
		}

	case 1:
		{
			AnyHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector, filter);
			s.U(collector.HadHit()? 1 : 0);
			if (collector.HadHit())
				write(collector.mHit);
			s.F(collector.GetEarlyOutFraction());
			break;
		}

	default:
		{
			ClosestHitCollisionCollector<CastRayCollector> collector;
			collector.SetContext(&context);
			if (inInput->mEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inInput->mEarlyOut);
			shape->CastRay(ray, settings, creator, collector, filter);
			s.U(collector.HadHit()? 1 : 0);
			if (collector.HadHit())
				write(collector.mHit);
			s.F(collector.GetEarlyOutFraction());
			break;
		}
	}
	return s.mSize;
}

// CollidePoint: all hits
uint32 jolt_mesh_collide_point(void *inHandle, const float *inPoint, const uint32 *inCreator, uint32 inBodyID, int inRejectAll, uint32 *outStream, uint32 inCapacity)
{
	const Shape *shape = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	AllHitCollisionCollector<CollidePointCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inBodyID));
	collector.SetContext(&context);
	RejectAllFilter reject_all;
	ShapeFilter default_filter;
	shape->CollidePoint(Load3(inPoint), MakeCreator(inCreator), collector, inRejectAll != 0? static_cast<const ShapeFilter &>(reject_all) : default_filter);
	s.U(cMarkerPoint);
	s.U(uint32(collector.mHits.size()));
	for (const CollidePointResult &h : collector.mHits)
	{
		s.U(h.mBodyID.GetIndexAndSequenceNumber());
		s.U(h.mSubShapeID2.GetValue());
	}
	return s.mSize;
}

// CollisionDispatch::sCollideShapeVsShape (or InternalEdgeRemovingCollector::sCollideShapeVsShape) of a convex shape vs
// the mesh or the mesh vs a convex shape: all hits in order
uint32 jolt_mesh_collide(void *inHandle, const CollideInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	EnsureFactory();
	const Shape *mesh = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	const CollideInput &in = *inInput;
	Ref<Shape> convex = CreateConvex(in.mConvex);
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = in.mMaxSeparationDistance;
	settings.mCollisionTolerance = in.mCollisionTolerance;
	settings.mPenetrationTolerance = in.mPenetrationTolerance;
	settings.mBackFaceMode = in.mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mActiveEdgeMode = in.mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mCollectFacesMode = in.mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mActiveEdgeMovementDirection = Load3(in.mActiveEdgeMovementDirection);
	settings.mInternalEdgeRemovalVertexToleranceSq = in.mVertexToleranceSq;
	AllHitCollisionCollector<CollideShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(in.mBodyID));
	collector.SetContext(&context);
	if (in.mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(in.mEarlyOut);

	const Shape *shape1 = in.mMeshFirst != 0? mesh : convex.GetPtr();
	const Shape *shape2 = in.mMeshFirst != 0? convex.GetPtr() : mesh;
	Vec3 scale1 = Load3(in.mMeshFirst != 0? in.mScaleMesh : in.mScaleConvex);
	Vec3 scale2 = Load3(in.mMeshFirst != 0? in.mScaleConvex : in.mScaleMesh);
	Mat44 transform1 = LoadMat44(in.mMeshFirst != 0? in.mTransformMesh : in.mTransformConvex);
	Mat44 transform2 = LoadMat44(in.mMeshFirst != 0? in.mTransformConvex : in.mTransformMesh);
	if (in.mInternalEdgeRemoval != 0)
		InternalEdgeRemovingCollector::sCollideShapeVsShape(shape1, shape2, scale1, scale2, transform1, transform2, MakeCreator(in.mCreator1), MakeCreator(in.mCreator2), settings, collector);
	else
		CollisionDispatch::sCollideShapeVsShape(shape1, shape2, scale1, scale2, transform1, transform2, MakeCreator(in.mCreator1), MakeCreator(in.mCreator2), settings, collector);

	s.U(cMarkerCollide);
	s.U(uint32(collector.mHits.size()));
	for (const CollideShapeResult &r : collector.mHits)
		WriteCollideHit(s, r);
	s.F(collector.GetEarlyOutFraction());
	return s.mSize;
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace of a convex shape vs the mesh or the mesh vs a convex shape: all hits
// in order
uint32 jolt_mesh_cast(void *inHandle, const CastInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	EnsureFactory();
	const Shape *mesh = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	const CastInput &in = *inInput;
	Ref<Shape> convex = CreateConvex(in.mConvex);
	ShapeCastSettings settings;
	settings.mCollisionTolerance = in.mCollisionTolerance;
	settings.mPenetrationTolerance = in.mPenetrationTolerance;
	settings.mExtraConvexRadius = in.mExtraConvexRadius;
	settings.mBackFaceModeTriangles = in.mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mBackFaceModeConvex = in.mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = in.mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = in.mReturnDeepestPoint != 0;
	settings.mCollectFacesMode = in.mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mActiveEdgeMode = in.mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = Load3(in.mActiveEdgeMovementDirection);
	const Shape *cast_shape = in.mMeshCast != 0? mesh : convex.GetPtr();
	const Shape *target = in.mMeshCast != 0? convex.GetPtr() : mesh;
	ShapeCast shape_cast(cast_shape, Load3(in.mScaleCast), LoadMat44(in.mStart), Load3(in.mDirection));
	AllHitCollisionCollector<CastShapeCollector> collector;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(in.mBodyID));
	collector.SetContext(&context);
	if (in.mEarlyOut < collector.GetEarlyOutFraction())
		collector.UpdateEarlyOutFraction(in.mEarlyOut);
	CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, target, Load3(in.mScaleTarget), ShapeFilter(), LoadMat44(in.mTransformTarget), MakeCreator(in.mCreator1), MakeCreator(in.mCreator2), collector);
	s.U(cMarkerCast);
	s.Box(shape_cast.mShapeWorldBounds);
	s.U(uint32(collector.mHits.size()));
	for (const ShapeCastResult &r : collector.mHits)
	{
		s.F(r.mFraction);
		s.B(r.mIsBackFaceHit);
		WriteCollideHit(s, r);
	}
	s.F(collector.GetEarlyOutFraction());
	return s.mSize;
}

// GetTrianglesStart, then GetTrianglesNext until it returns 0 (at most 1000 calls): the count of every call, the
// vertices and the materials
uint32 jolt_mesh_triangles(void *inHandle, const TrianglesInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	const Shape *shape = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	const TrianglesInput &in = *inInput;
	Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, AABox(Load3(in.mBox), Load3(in.mBox + 3)), Load3(in.mPosition), LoadQuat(in.mRotation), Load3(in.mScale));
	Array<Float3> vertices(in.mMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(in.mMaxTrianglesRequested);
	s.U(cMarkerTriangles);
	for (int calls = 0; calls < 1000; ++calls)
	{
		int count = shape->GetTrianglesNext(context, in.mMaxTrianglesRequested, vertices.data(), in.mMaterials != 0? materials.data() : nullptr);
		s.I(count);
		for (int i = 0; i < 3 * count; ++i)
			s.F3(vertices[i]);
		if (in.mMaterials != 0)
			for (int i = 0; i < count; ++i)
				WriteMaterial(s, materials[i]);
		if (count == 0)
			break;
	}
	return s.mSize;
}

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, initial planes as 4 floats, initial
// penetrations and indices): the resulting planes, penetrations and indices
uint32 jolt_mesh_soft_body(void *inHandle, const float *inTransform, const float *inScale, uint32 inNumVertices, const float *inPositions, const float *inInvMasses, const float *inPlanes, const float *inPenetrations, const int *inIndices, int inCollidingShapeIndex, uint32 *outStream, uint32 inCapacity)
{
	const Shape *shape = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	Array<Vec3> positions;
	Array<Plane> planes;
	Array<float> penetrations(inPenetrations, inPenetrations + inNumVertices);
	Array<int> indices(inIndices, inIndices + inNumVertices);
	for (uint32 i = 0; i < inNumVertices; ++i)
	{
		positions.push_back(Load3(inPositions + 3 * i));
		planes.push_back(Plane(Load4(inPlanes + 4 * i)));
	}
	CollideSoftBodyVertexIterator vertices(StridedPtr<const Vec3>(positions.data()), StridedPtr<const float>(inInvMasses), StridedPtr<Plane>(planes.data()), StridedPtr<float>(penetrations.data()), StridedPtr<int>(indices.data()));
	shape->CollideSoftBodyVertices(LoadMat44(inTransform), Load3(inScale), vertices, inNumVertices, inCollidingShapeIndex);
	s.U(cMarkerSoftBody);
	for (uint32 i = 0; i < inNumVertices; ++i)
	{
		s.P(planes[i]);
		s.F(penetrations[i]);
		s.I(indices[i]);
	}
	return s.mSize;
}

// SaveBinaryState, sRestoreFromBinaryState + RestoreMaterialState + SaveBinaryState and the queries on the restored
// shape (a ray), SaveMaterialState, SaveWithChildren and sRestoreWithChildren + SaveWithChildren
uint32 jolt_mesh_binary_state(void *inHandle, const RayInput *inRay, uint32 *outStream, uint32 inCapacity)
{
	EnsureFactory();
	const Shape *shape = static_cast<const Shape *>(inHandle);
	OutStream s(outStream, inCapacity);
	s.U(cMarkerBinaryState);

	// Binary state
	std::string bytes = SaveBinaryState(shape);
	s.Bytes(bytes);

	// Material state
	PhysicsMaterialList materials;
	materials.push_back(PhysicsMaterial::sDefault);
	shape->SaveMaterialState(materials);
	s.U(uint32(materials.size()));
	for (const PhysicsMaterialRefC &m : materials)
		WriteMaterial(s, m);

	// Restore
	std::stringstream in_data(bytes);
	StreamInWrapper stream_in(in_data);
	Shape::ShapeResult result = Shape::sRestoreFromBinaryState(stream_in);
	s.B(result.IsValid());
	if (result.IsValid())
	{
		Ref<Shape> restored = result.Get();
		restored->RestoreMaterialState(materials.data(), uint(materials.size()));
		s.Bytes(SaveBinaryState(restored));
		RayCastResult hit;
		s.B(restored->CastRay(RayCast { Load3(inRay->mOrigin), Load3(inRay->mDirection) }, SubShapeIDCreator(), hit));
		s.F(hit.mFraction);
		s.U(hit.mSubShapeID2.GetValue());
		if (hit.mFraction < 1.0f)
			WriteMaterial(s, restored->GetMaterial(hit.mSubShapeID2));
	}

	// Truncated binary state
	std::stringstream truncated_data(bytes.substr(0, bytes.size() - 1));
	StreamInWrapper truncated_in(truncated_data);
	Shape::ShapeResult truncated = Shape::sRestoreFromBinaryState(truncated_in);
	s.B(truncated.IsValid());
	if (truncated.HasError())
		s.Str(truncated.GetError().c_str());

	// Save with children
	std::string children = SaveWithChildren(shape);
	s.Bytes(children);
	std::stringstream children_data(children);
	StreamInWrapper children_in(children_data);
	Shape::IDToShapeMap id_to_shape;
	Shape::IDToMaterialMap id_to_material;
	Shape::ShapeResult children_result = Shape::sRestoreWithChildren(children_in, id_to_shape, id_to_material);
	s.B(children_result.IsValid());
	if (children_result.IsValid())
	{
		const MeshShape *restored = static_cast<const MeshShape *>(children_result.Get().GetPtr());
		s.Bytes(SaveWithChildren(restored));
		s.U(uint32(restored->GetMaterialList().size()));
		for (const PhysicsMaterialRefC &m : restored->GetMaterialList())
			WriteMaterial(s, m);
	}
	return s.mSize;
}

} // extern "C"
