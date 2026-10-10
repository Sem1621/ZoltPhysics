// Reference implementation for the cross-shape parity sweep (`zig build parity`): C ABI wrappers around the C++ Jolt
// library that build a catalogue of shapes (every shape type of Jolt/Physics/Collision/Shape with parameter variants,
// decorators around convex shapes and meshes, nested compounds, meshes and height fields) from node descriptions and run
// CollisionDispatch (collide and cast with every collector and setting variant, InternalEdgeRemovingCollector) for pairs
// of catalogue shapes and the TransformedShape queries for single catalogue shapes. ZoltParity/Physics/PairwiseParity.zig
// builds the same catalogue with Zolt from the same descriptions and checks that both produce the same bits.
//
// Conventions: every query function writes its results into a stream of uint32 (floats as their bits, see OutStream)
// and returns the number of values in the stream (only the first inCapacity values are written). The stream layout must
// match PairwiseParity.zig. Vectors are passed as float arrays, Mat44 as 16 floats in column major order, RVec3 as 3
// doubles, booleans as int (never bool, see the porting guide). The catalogue is created once (pw_catalogue_create returns
// a handle that holds a reference to every node shape) and queried by handle. Everything except the C ABI is in an
// anonymous namespace (other reference files have classes with the same names). All functions are prefixed with pw_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Geometry/Plane.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/InternalEdgeRemovingCollector.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/TaperedCylinderShape.h>
#include <Jolt/Physics/Collision/Shape/TriangleShape.h>

#include <cstring>
#include <string>

using namespace JPH;

static_assert(sizeof(IndexedTriangle) == 20, "Zolt passes IndexedTriangle as raw memory");
static_assert(sizeof(Float3) == 12, "Zolt passes Float3 as raw memory");

namespace {

// Number of parity materials, must match num_materials in PairwiseParity.zig
constexpr uint cNumMaterials = 8;

// Section markers in the streams, must match PairwiseParity.zig
enum : uint32
{
	cMarkerProperties = 0xC0000001,
	cMarkerCollide = 0xC0000002,
	cMarkerCast = 0xC0000003,
	cMarkerHit = 0xC0000004,
	cMarkerRay = 0xC0000005,
	cMarkerRayCollector = 0xC0000006,
	cMarkerPoint = 0xC0000007,
	cMarkerTransformedShapes = 0xC0000008,
	cMarkerTriangles = 0xC0000009,
	cMarkerFace = 0xC000000A,
	cMarkerBounds = 0xC000000B,
	cMarkerFilter = 0xC000000C,
};

// Node kinds, must match NodeKind in PairwiseParity.zig
enum : uint32
{
	cSphere,
	cBox,
	cCapsule,
	cTaperedCapsule,
	cCylinder,
	cTaperedCylinder,
	cConvexHull,
	cTriangle,
	cPlane,
	cEmpty,
	cScaled,
	cRotatedTranslated,
	cOffsetCenterOfMass,
	cStaticCompound,
	cMutableCompound,
	cMesh,
	cHeightField,
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

// The materials of the tests: "Pairwise i" with Color::sGetDistinctColor(i)
const Array<RefConst<PhysicsMaterial>> &GetMaterials()
{
	static Array<RefConst<PhysicsMaterial>> sMaterials;
	if (sMaterials.empty())
		for (uint i = 0; i < cNumMaterials; ++i)
			sMaterials.push_back(new PhysicsMaterialSimple("Pairwise " + std::to_string(i), Color::sGetDistinctColor(int(i))));
	return sMaterials;
}

Vec3 Load3(const float *inV)						{ return Vec3(inV[0], inV[1], inV[2]); }
Vec4 Load4(const float *inV)						{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
Quat LoadQuat(const float *inV)						{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
Mat44 LoadMat44(const float *inM)					{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
RVec3 LoadR3(const double *inV)						{ return RVec3(Real(inV[0]), Real(inV[1]), Real(inV[2])); }
AABox LoadAABox(const float *inBox)					{ return AABox(Load3(inBox), Load3(inBox + 3)); }
SubShapeIDCreator MakeCreator(const uint32 *inCreator) { return SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }

// Output stream, must match Stream in PairwiseParity.zig
class OutStream
{
public:
						OutStream(uint32 *inData, uint32 inCapacity) : mData(inData), mCapacity(inCapacity) { }

	void				U(uint32 inValue)			{ if (mSize < mCapacity) mData[mSize] = inValue; ++mSize; }
	void				B(bool inValue)				{ U(inValue? 1 : 0); }
	void				F(float inValue)			{ uint32 u; memcpy(&u, &inValue, sizeof(u)); U(u); }
	void				D(double inValue)			{ uint64 u; memcpy(&u, &inValue, sizeof(u)); U(uint32(u)); U(uint32(u >> 32)); }
	void				V3(Vec3Arg inValue)			{ F(inValue.GetX()); F(inValue.GetY()); F(inValue.GetZ()); }
	void				F3(const Float3 &inValue)	{ F(inValue.x); F(inValue.y); F(inValue.z); }
	void				R3(RVec3Arg inValue)		{ D(double(inValue.GetX())); D(double(inValue.GetY())); D(double(inValue.GetZ())); }
	void				Q(QuatArg inValue)			{ F(inValue.GetX()); F(inValue.GetY()); F(inValue.GetZ()); F(inValue.GetW()); }
	void				M(Mat44Arg inValue)			{ for (int c = 0; c < 4; ++c) for (int r = 0; r < 4; ++r) F(inValue(r, c)); }
	void				Box(const AABox &inBox)		{ V3(inBox.mMin); V3(inBox.mMax); }
	void				Face(const Shape::SupportingFace &inFace) { U(uint32(inFace.size())); for (Vec3 v : inFace) V3(v); }

	uint32 *			mData;
	uint32				mCapacity;
	uint32				mSize = 0;
};

// A material: the index in GetMaterials(), 0xffffffff for the default material, otherwise 0xfffffffe
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

void WriteCastHit(OutStream &ioStream, const ShapeCastResult &inResult)
{
	ioStream.F(inResult.mFraction);
	ioStream.B(inResult.mIsBackFaceHit);
	WriteCollideHit(ioStream, inResult);
}

// ---------------------------------------------------------------------------------------------------------------------
// The catalogue, the descriptions must match NodeDesc / SubDesc / CatalogueInput in PairwiseParity.zig

struct NodeDesc
{
	uint32					mKind;
	uint32					mMaterial;				// 0: no material, i + 1: GetMaterials()[i]
	float					mF[16];					// Parameters of the kind (see PairwiseParity.zig)
	float					mConvexRadius;
	uint32					mChild;					// Decorators: the inner node
	uint32					mFirst;					// Compounds: first sub, hull: first float of the points, mesh: first float of the vertices, height field: first float of the samples
	uint32					mCount;					// Number of subs / points / vertices, height field: sample count
	uint32					mFirst2;				// Mesh: first triangle, height field: first byte of the material indices
	uint32					mCount2;				// Mesh: number of triangles, height field: number of material indices
	uint32					mNumMaterials;			// Mesh / height field: the first mNumMaterials of GetMaterials()
	uint32					mParam0;				// Mesh: max triangles per leaf, height field: block size
	uint32					mParam1;				// Mesh: per triangle user data, height field: bits per sample
	uint32					mUserData;
};

struct SubDesc
{
	uint32					mNode;
	uint32					mUserData;
	float					mPosition[3];
	float					mRotation[4];
};

struct CatalogueInput
{
	const NodeDesc *		mNodes;
	const SubDesc *			mSubs;
	const float *			mFloats;
	const IndexedTriangle *	mTriangles;
	const uint8 *			mBytes;
	uint32					mNumNodes;
};

class Catalogue
{
public:
	Array<RefConst<Shape>>	mShapes;
};

Shape::ShapeResult CreateNode(const CatalogueInput &inInput, const Array<RefConst<Shape>> &inShapes, uint32 inIndex)
{
	const NodeDesc &n = inInput.mNodes[inIndex];
	const float *f = n.mF;
	const PhysicsMaterial *material = n.mMaterial == 0? nullptr : GetMaterials()[n.mMaterial - 1].GetPtr();
	PhysicsMaterialList material_list;
	for (uint i = 0; i < n.mNumMaterials; ++i)
		material_list.push_back(GetMaterials()[i % cNumMaterials]);

	Ref<ShapeSettings> settings;
	switch (n.mKind)
	{
	case cSphere:
		settings = new SphereShapeSettings(f[0], material);
		break;

	case cBox:
		settings = new BoxShapeSettings(Load3(f), n.mConvexRadius, material);
		break;

	case cCapsule:
		settings = new CapsuleShapeSettings(f[0], f[1], material);
		break;

	case cTaperedCapsule:
		settings = new TaperedCapsuleShapeSettings(f[0], f[1], f[2], material);
		break;

	case cCylinder:
		settings = new CylinderShapeSettings(f[0], f[1], n.mConvexRadius, material);
		break;

	case cTaperedCylinder:
		settings = new TaperedCylinderShapeSettings(f[0], f[1], f[2], n.mConvexRadius, material);
		break;

	case cConvexHull:
		{
			Array<Vec3> points;
			for (uint32 i = 0; i < n.mCount; ++i)
				points.push_back(Load3(inInput.mFloats + n.mFirst + 3 * i));
			settings = new ConvexHullShapeSettings(points, n.mConvexRadius, material);
			break;
		}

	case cTriangle:
		settings = new TriangleShapeSettings(Load3(f), Load3(f + 3), Load3(f + 6), n.mConvexRadius, material);
		break;

	case cPlane:
		settings = new PlaneShapeSettings(Plane(Load3(f), f[3]), material, f[4]);
		break;

	case cEmpty:
		settings = new EmptyShapeSettings(Load3(f));
		break;

	case cScaled:
		settings = new ScaledShapeSettings(inShapes[n.mChild], Load3(f));
		break;

	case cRotatedTranslated:
		settings = new RotatedTranslatedShapeSettings(Load3(f), LoadQuat(f + 3), inShapes[n.mChild]);
		break;

	case cOffsetCenterOfMass:
		settings = new OffsetCenterOfMassShapeSettings(Load3(f), inShapes[n.mChild]);
		break;

	case cStaticCompound:
	case cMutableCompound:
		{
			CompoundShapeSettings *compound;
			if (n.mKind == cStaticCompound)
				compound = new StaticCompoundShapeSettings;
			else
				compound = new MutableCompoundShapeSettings;
			settings = compound;
			for (uint32 i = 0; i < n.mCount; ++i)
			{
				const SubDesc &sub = inInput.mSubs[n.mFirst + i];
				compound->AddShape(Load3(sub.mPosition), LoadQuat(sub.mRotation), inShapes[sub.mNode], sub.mUserData);
			}
			break;
		}

	case cMesh:
		{
			VertexList vertices;
			for (uint32 i = 0; i < n.mCount; ++i)
			{
				const float *v = inInput.mFloats + n.mFirst + 3 * i;
				vertices.push_back(Float3(v[0], v[1], v[2]));
			}
			IndexedTriangleList triangles(inInput.mTriangles + n.mFirst2, inInput.mTriangles + n.mFirst2 + n.mCount2);
			MeshShapeSettings *mesh = new MeshShapeSettings(vertices, triangles, material_list);
			mesh->mMaxTrianglesPerLeaf = n.mParam0;
			mesh->mActiveEdgeCosThresholdAngle = f[0];
			mesh->mPerTriangleUserData = n.mParam1 != 0;
			settings = mesh;
			break;
		}

	case cHeightField:
		{
			HeightFieldShapeSettings *hf = new HeightFieldShapeSettings(inInput.mFloats + n.mFirst, Load3(f), Load3(f + 3), n.mCount, n.mCount2 > 0? inInput.mBytes + n.mFirst2 : nullptr, material_list);
			hf->mBlockSize = n.mParam0;
			hf->mBitsPerSample = n.mParam1;
			hf->mActiveEdgeCosThresholdAngle = f[6];
			settings = hf;
			break;
		}

	default:
		JPH_ASSERT(false);
		return Shape::ShapeResult();
	}

	settings->mUserData = n.mUserData;
	return settings->Create();
}

// ---------------------------------------------------------------------------------------------------------------------
// The shape filter, must match PairwiseFilter in PairwiseParity.zig: it folds every call (shape sub types, sub shape IDs,
// the body ID) into a hash and rejects a pseudo random subset of the sub shape IDs (when mRejectModulus != 0)

struct FilterLog
{
	void					Add(uint32 inValue)			{ mHash = (mHash ^ inValue) * 0x01000193u; ++mCalls; }

	uint32					mCalls = 0;
	uint32					mHash = 0x811c9dc5u;
};

class PairwiseFilter : public ShapeFilter
{
public:
							PairwiseFilter(uint32 inRejectModulus, FilterLog *inLog) : mRejectModulus(inRejectModulus), mLog(inLog) { }

	virtual bool			ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		mLog->Add(1);
		mLog->Add(uint32(inShape2->GetSubType()));
		mLog->Add(inSubShapeIDOfShape2.GetValue());
		mLog->Add(mBodyID2.GetIndexAndSequenceNumber());
		return !Reject(inSubShapeIDOfShape2.GetValue() * 0x9e3779b1u);
	}

	virtual bool			ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		mLog->Add(2);
		mLog->Add(uint32(inShape1->GetSubType()));
		mLog->Add(inSubShapeIDOfShape1.GetValue());
		mLog->Add(uint32(inShape2->GetSubType()));
		mLog->Add(inSubShapeIDOfShape2.GetValue());
		mLog->Add(mBodyID2.GetIndexAndSequenceNumber());
		return !Reject(inSubShapeIDOfShape1.GetValue() * 0x9e3779b1u + inSubShapeIDOfShape2.GetValue() * 0x85ebca6bu);
	}

private:
	bool					Reject(uint32 inValue) const	{ return mRejectModulus != 0 && (inValue >> 16) % mRejectModulus == 0; }

	uint32					mRejectModulus;
	FilterLog *				mLog;
};

void WriteFilterLog(OutStream &ioStream, const FilterLog &inLog)
{
	ioStream.U(cMarkerFilter);
	ioStream.U(inLog.mCalls);
	ioStream.U(inLog.mHash);
}

// ---------------------------------------------------------------------------------------------------------------------
// Query inputs, must match CollideInput / CastInput / TransformedShapeInput in PairwiseParity.zig

struct CollideInput
{
	uint32					mShape1;
	uint32					mShape2;
	float					mScale1[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mMaxSeparationDistance;
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	int						mBackFaceMode;			// 1: CollideWithBackFaces
	int						mActiveEdgeMode;		// 1: CollideWithAll
	int						mCollectFaces;			// 1: CollectFaces
	float					mActiveEdgeMovementDirection[3];
	int						mCollector;				// 0: AllHit, 1: ClosestHit, 2: AnyHit
	float					mEarlyOut;				// Early out fraction of the collector (when < FLT_MAX)
	uint32					mBodyID;				// Body ID of the collector context
	uint32					mRejectModulus;			// The filter (0: rejects nothing)
	int						mInternalEdgeRemoval;	// Use InternalEdgeRemovingCollector::sCollideShapeVsShape
	float					mVertexToleranceSq;		// mInternalEdgeRemovalVertexToleranceSq
};

struct CastInput
{
	uint32					mShape1;				// The cast shape
	uint32					mShape2;				// The shape cast against
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
	int						mUseShrunkenShape;
	int						mReturnDeepestPoint;
	int						mCollectFaces;
	int						mActiveEdgeMode;		// 1: CollideWithAll
	float					mActiveEdgeMovementDirection[3];
	int						mCollector;				// 0: AllHit, 1: ClosestHit, 2: AnyHit
	float					mEarlyOut;				// Early out fraction of the collector (when < 1 + FLT_EPSILON)
	uint32					mBodyID;
	uint32					mRejectModulus;
};

struct TransformedShapeInput
{
	uint32					mShape;
	uint32					mBodyID;
	uint32					mCreator[2];
	double					mPosition[3];			// Center of mass position
	float					mRotation[4];
	float					mScale[3];
	double					mRayOrigin[3];
	float					mRayDirection[3];
	float					mRayFraction;			// Initial fraction of the single hit CastRay
	int						mBackFaceModeTriangles;
	int						mBackFaceModeConvex;
	int						mTreatConvexAsSolid;
	int						mRayCollector;			// 0: AllHit, 1: ClosestHit, 2: AnyHit
	float					mRayEarlyOut;			// Early out fraction of the ray collector (when < 1 + FLT_EPSILON)
	double					mPoint[3];				// CollidePoint
	float					mBox[6];				// CollectTransformedShapes / GetTrianglesStart (world space)
	double					mBaseOffset[3];			// GetTrianglesStart / GetSupportingFace
	int						mMaxTrianglesRequested;
	int						mMaterials;				// GetTrianglesNext with materials
	float					mFaceDirection[3];		// GetSupportingFace (world space)
	int						mFaceOfRoot;			// GetSupportingFace with the ID of the root (only for shapes without sub shapes)
	int						mTriangles;				// Call GetTrianglesStart / Next (Jolt asserts for compound and decorated shapes)
	uint32					mRejectModulus;
};

CollideShapeSettings MakeCollideSettings(const CollideInput &inInput)
{
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput.mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput.mCollisionTolerance;
	settings.mPenetrationTolerance = inInput.mPenetrationTolerance;
	settings.mBackFaceMode = inInput.mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mActiveEdgeMode = inInput.mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mCollectFacesMode = inInput.mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mActiveEdgeMovementDirection = Load3(inInput.mActiveEdgeMovementDirection);
	settings.mInternalEdgeRemovalVertexToleranceSq = inInput.mVertexToleranceSq;
	return settings;
}

ShapeCastSettings MakeCastSettings(const CastInput &inInput)
{
	ShapeCastSettings settings;
	settings.mCollisionTolerance = inInput.mCollisionTolerance;
	settings.mPenetrationTolerance = inInput.mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput.mExtraConvexRadius;
	settings.mBackFaceModeTriangles = inInput.mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mBackFaceModeConvex = inInput.mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = inInput.mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = inInput.mReturnDeepestPoint != 0;
	settings.mCollectFacesMode = inInput.mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mActiveEdgeMode = inInput.mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = Load3(inInput.mActiveEdgeMovementDirection);
	return settings;
}

// Run a query with the collector of inKind (0: AllHit, 1: ClosestHit, 2: AnyHit) and write the hits and the final early
// out fraction. inWrite writes a hit, inQuery runs the query with the collector.
template <class CollectorType, class WriteHit, class Query>
void RunWithCollector(OutStream &ioStream, int inKind, float inEarlyOut, const TransformedShape *inContext, const WriteHit &inWriteHit, const Query &inQuery)
{
	auto prepare = [&](CollectorType &ioCollector) {
		if (inContext != nullptr)
			ioCollector.SetContext(inContext);
		if (inEarlyOut < ioCollector.GetEarlyOutFraction())
			ioCollector.UpdateEarlyOutFraction(inEarlyOut);
	};

	switch (inKind)
	{
	case 0:
		{
			AllHitCollisionCollector<CollectorType> collector;
			prepare(collector);
			inQuery(collector);
			ioStream.U(uint32(collector.mHits.size()));
			for (const typename CollectorType::ResultType &hit : collector.mHits)
				inWriteHit(hit);
			ioStream.F(collector.GetEarlyOutFraction());
			break;
		}

	case 1:
		{
			ClosestHitCollisionCollector<CollectorType> collector;
			prepare(collector);
			inQuery(collector);
			ioStream.B(collector.HadHit());
			if (collector.HadHit())
				inWriteHit(collector.mHit);
			ioStream.F(collector.GetEarlyOutFraction());
			break;
		}

	default:
		{
			AnyHitCollisionCollector<CollectorType> collector;
			prepare(collector);
			inQuery(collector);
			ioStream.B(collector.HadHit());
			if (collector.HadHit())
				inWriteHit(collector.mHit);
			ioStream.F(collector.GetEarlyOutFraction());
			break;
		}
	}
}

// Private access to the dispatch tables of CollisionDispatch (explicit instantiation ignores access checks, like in
// ShapeCoreReference.cpp)
template <typename Tag, typename Tag::type M>
struct Rob
{
	friend typename Tag::type Get(Tag) { return M; }
};

struct CollideTableTag
{
	using type = CollisionDispatch::CollideShape (*)[NumSubShapeTypes][NumSubShapeTypes];
	friend type Get(CollideTableTag);
};

struct CastTableTag
{
	using type = CollisionDispatch::CastShape (*)[NumSubShapeTypes][NumSubShapeTypes];
	friend type Get(CastTableTag);
};

template struct Rob<CollideTableTag, &CollisionDispatch::sCollideShape>;
template struct Rob<CastTableTag, &CollisionDispatch::sCastShape>;

} // namespace

extern "C"
{

// Create the catalogue: every node in order (children before parents). Writes the properties of every node shape (or
// the error) into the stream. Returns the handle, nullptr when a node fails.
void *pw_catalogue_create(const CatalogueInput *inInput, uint32 *outData, uint32 inCapacity, uint32 *outSize)
{
	EnsureFactory();

	OutStream out(outData, inCapacity);
	Catalogue *catalogue = new Catalogue;
	bool valid = true;
	for (uint32 i = 0; i < inInput->mNumNodes; ++i)
	{
		Shape::ShapeResult result = CreateNode(*inInput, catalogue->mShapes, i);
		out.U(cMarkerProperties);
		if (result.HasError())
		{
			out.U(0);
			valid = false;
			catalogue->mShapes.push_back(nullptr);
			continue;
		}
		out.U(1);
		const Shape *shape = result.Get();
		catalogue->mShapes.push_back(shape);
		out.U(uint32(shape->GetType()));
		out.U(uint32(shape->GetSubType()));
		out.Box(shape->GetLocalBounds());
		out.V3(shape->GetCenterOfMass());
		out.F(shape->GetInnerRadius());
		out.U(shape->GetSubShapeIDBitsRecursive());
		out.U(uint32(shape->GetUserData()));
		out.U(shape->GetStats().mNumTriangles);
	}
	*outSize = out.mSize;
	if (!valid)
	{
		delete catalogue;
		return nullptr;
	}
	return catalogue;
}

void pw_catalogue_destroy(void *inCatalogue)
{
	delete static_cast<Catalogue *>(inCatalogue);
}

// The dispatch table entries of a pair of sub types: -1 unsupported (the function sInit installs, the entry of the User8
// pair that nothing registers), -2 the reversed function, 1 another function
void pw_dispatch(uint32 inType1, uint32 inType2, int *outCollide, int *outCast)
{
	EnsureFactory();

	CollisionDispatch::CollideShape (&collide)[NumSubShapeTypes][NumSubShapeTypes] = *Get(CollideTableTag());
	CollisionDispatch::CastShape (&cast)[NumSubShapeTypes][NumSubShapeTypes] = *Get(CastTableTag());
	const uint u8 = uint(EShapeSubType::User8);

	CollisionDispatch::CollideShape c = collide[inType1][inType2];
	*outCollide = c == collide[u8][u8]? -1 : (c == CollisionDispatch::sReversedCollideShape? -2 : 1);
	CollisionDispatch::CastShape s = cast[inType1][inType2];
	*outCast = s == cast[u8][u8]? -1 : (s == CollisionDispatch::sReversedCastShape? -2 : 1);
}

// CollisionDispatch::sCollideShapeVsShape (or InternalEdgeRemovingCollector::sCollideShapeVsShape)
uint32 pw_collide(void *inCatalogue, const CollideInput *inInput, uint32 *outData, uint32 inCapacity)
{
	const Catalogue &catalogue = *static_cast<Catalogue *>(inCatalogue);
	const CollideInput &in = *inInput;
	OutStream out(outData, inCapacity);

	const Shape *shape1 = catalogue.mShapes[in.mShape1];
	const Shape *shape2 = catalogue.mShapes[in.mShape2];
	CollideShapeSettings settings = MakeCollideSettings(in);
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(in.mBodyID));
	FilterLog log;
	PairwiseFilter filter(in.mRejectModulus, &log);

	out.U(cMarkerCollide);
	RunWithCollector<CollideShapeCollector>(out, in.mCollector, in.mEarlyOut, &context,
		[&](const CollideShapeResult &inHit) { WriteCollideHit(out, inHit); },
		[&](CollideShapeCollector &ioCollector) {
			if (in.mInternalEdgeRemoval != 0)
				InternalEdgeRemovingCollector::sCollideShapeVsShape(shape1, shape2, Load3(in.mScale1), Load3(in.mScale2), LoadMat44(in.mTransform1), LoadMat44(in.mTransform2), MakeCreator(in.mCreator1), MakeCreator(in.mCreator2), settings, ioCollector, filter);
			else
				CollisionDispatch::sCollideShapeVsShape(shape1, shape2, Load3(in.mScale1), Load3(in.mScale2), LoadMat44(in.mTransform1), LoadMat44(in.mTransform2), MakeCreator(in.mCreator1), MakeCreator(in.mCreator2), settings, ioCollector, filter);
		});
	WriteFilterLog(out, log);
	return out.mSize;
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace
uint32 pw_cast(void *inCatalogue, const CastInput *inInput, uint32 *outData, uint32 inCapacity)
{
	const Catalogue &catalogue = *static_cast<Catalogue *>(inCatalogue);
	const CastInput &in = *inInput;
	OutStream out(outData, inCapacity);

	const Shape *shape1 = catalogue.mShapes[in.mShape1];
	const Shape *shape2 = catalogue.mShapes[in.mShape2];
	ShapeCastSettings settings = MakeCastSettings(in);
	ShapeCast shape_cast(shape1, Load3(in.mScale1), LoadMat44(in.mStart), Load3(in.mDirection));
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(in.mBodyID));
	FilterLog log;
	PairwiseFilter filter(in.mRejectModulus, &log);

	out.U(cMarkerCast);
	out.Box(shape_cast.mShapeWorldBounds);
	RunWithCollector<CastShapeCollector>(out, in.mCollector, in.mEarlyOut, &context,
		[&](const ShapeCastResult &inHit) { WriteCastHit(out, inHit); },
		[&](CastShapeCollector &ioCollector) {
			CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, shape2, Load3(in.mScale2), filter, LoadMat44(in.mTransform2), MakeCreator(in.mCreator1), MakeCreator(in.mCreator2), ioCollector);
		});
	WriteFilterLog(out, log);
	return out.mSize;
}

// The TransformedShape queries of one catalogue shape
uint32 pw_transformed_shape(void *inCatalogue, const TransformedShapeInput *inInput, uint32 *outData, uint32 inCapacity)
{
	const Catalogue &catalogue = *static_cast<Catalogue *>(inCatalogue);
	const TransformedShapeInput &in = *inInput;
	OutStream out(outData, inCapacity);

	const Shape *shape = catalogue.mShapes[in.mShape];
	TransformedShape ts(LoadR3(in.mPosition), LoadQuat(in.mRotation), shape, BodyID(in.mBodyID), MakeCreator(in.mCreator));
	ts.SetShapeScale(Load3(in.mScale));
	RVec3 base_offset = LoadR3(in.mBaseOffset);

	// GetWorldSpaceBounds, IsValidScale, MakeScaleValid
	out.U(cMarkerBounds);
	out.Box(ts.GetWorldSpaceBounds());
	out.B(shape->IsValidScale(Load3(in.mScale)));
	out.V3(shape->MakeScaleValid(Load3(in.mScale)));

	// The sub shape queries of a hit: material, user data, surface normal, supporting face
	auto sub_shape_queries = [&](const SubShapeID &inID, RVec3Arg inPosition) {
		WriteMaterial(out, ts.GetMaterial(inID));
		uint64 user_data = ts.GetSubShapeUserData(inID);
		out.U(uint32(user_data));
		out.U(uint32(user_data >> 32));
		out.V3(ts.GetWorldSpaceSurfaceNormal(inID, inPosition));
		Shape::SupportingFace face;
		ts.GetSupportingFace(inID, Load3(in.mFaceDirection), base_offset, face);
		out.U(cMarkerFace);
		out.Face(face);
	};

	// CastRay (single hit)
	RRayCast ray(LoadR3(in.mRayOrigin), Load3(in.mRayDirection));
	{
		RayCastResult hit;
		hit.mFraction = in.mRayFraction;
		out.U(cMarkerRay);
		bool had_hit = ts.CastRay(ray, hit);
		out.B(had_hit);
		out.F(hit.mFraction);
		out.U(hit.mSubShapeID2.GetValue());
		out.U(hit.mBodyID.GetIndexAndSequenceNumber());
		if (had_hit)
			sub_shape_queries(hit.mSubShapeID2, ray.GetPointOnRay(hit.mFraction));
	}

	// CastRay (collector)
	{
		RayCastSettings settings;
		settings.mBackFaceModeTriangles = in.mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
		settings.mBackFaceModeConvex = in.mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
		settings.mTreatConvexAsSolid = in.mTreatConvexAsSolid != 0;
		FilterLog log;
		PairwiseFilter filter(in.mRejectModulus, &log);
		out.U(cMarkerRayCollector);
		RunWithCollector<CastRayCollector>(out, in.mRayCollector, in.mRayEarlyOut, nullptr,
			[&](const RayCastResult &inHit) {
				out.F(inHit.mFraction);
				out.U(inHit.mSubShapeID2.GetValue());
				out.U(inHit.mBodyID.GetIndexAndSequenceNumber());
				sub_shape_queries(inHit.mSubShapeID2, ray.GetPointOnRay(inHit.mFraction));
			},
			[&](CastRayCollector &ioCollector) { ts.CastRay(ray, settings, ioCollector, filter); });
		WriteFilterLog(out, log);
	}

	// CollidePoint
	{
		FilterLog log;
		PairwiseFilter filter(in.mRejectModulus, &log);
		AllHitCollisionCollector<CollidePointCollector> collector;
		ts.CollidePoint(LoadR3(in.mPoint), collector, filter);
		out.U(cMarkerPoint);
		out.U(uint32(collector.mHits.size()));
		for (const CollidePointResult &hit : collector.mHits)
		{
			out.U(hit.mBodyID.GetIndexAndSequenceNumber());
			out.U(hit.mSubShapeID2.GetValue());
		}
		WriteFilterLog(out, log);
	}

	// CollectTransformedShapes
	{
		FilterLog log;
		PairwiseFilter filter(in.mRejectModulus, &log);
		AllHitCollisionCollector<TransformedShapeCollector> collector;
		ts.CollectTransformedShapes(LoadAABox(in.mBox), collector, filter);
		out.U(cMarkerTransformedShapes);
		out.U(uint32(collector.mHits.size()));
		for (const TransformedShape &hit : collector.mHits)
		{
			// The shape as the index of the node shape (0xffffffff when it is not a node shape)
			uint32 index = 0xffffffff;
			for (uint32 i = 0; i < catalogue.mShapes.size(); ++i)
				if (catalogue.mShapes[i] == hit.mShape)
				{
					index = i;
					break;
				}
			out.U(index);
			out.U(uint32(hit.mShape->GetSubType()));
			out.R3(hit.mShapePositionCOM);
			out.Q(hit.mShapeRotation);
			out.V3(hit.GetShapeScale());
			out.U(hit.mBodyID.GetIndexAndSequenceNumber());
			out.U(hit.mSubShapeIDCreator.GetID().GetValue());
			out.U(hit.mSubShapeIDCreator.GetNumBitsWritten());
			out.Box(hit.GetWorldSpaceBounds());
		}
		WriteFilterLog(out, log);
	}

	// GetTrianglesStart / Next
	if (in.mTriangles != 0)
	{
		TransformedShape::GetTrianglesContext context;
		ts.GetTrianglesStart(context, LoadAABox(in.mBox), base_offset);
		Array<Float3> vertices(3 * in.mMaxTrianglesRequested);
		Array<const PhysicsMaterial *> materials(in.mMaxTrianglesRequested);
		out.U(cMarkerTriangles);
		for (int iteration = 0; iteration < 1000; ++iteration)
		{
			int count = ts.GetTrianglesNext(context, in.mMaxTrianglesRequested, vertices.data(), in.mMaterials != 0? materials.data() : nullptr);
			out.U(uint32(count));
			for (int i = 0; i < 3 * count; ++i)
				out.F3(vertices[i]);
			if (in.mMaterials != 0)
				for (int i = 0; i < count; ++i)
					WriteMaterial(out, materials[i]);
			if (count == 0)
				break;
		}
	}

	// GetSupportingFace with the ID of the root
	if (in.mFaceOfRoot != 0)
	{
		Shape::SupportingFace face;
		ts.GetSupportingFace(MakeCreator(in.mCreator).GetID(), Load3(in.mFaceDirection), base_offset, face);
		out.U(cMarkerFace);
		out.Face(face);
	}

	return out.mSize;
}

} // extern "C"
