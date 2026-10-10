// Reference implementation for the height field shape parity tests (`zig build parity`): thin C ABI wrappers around
// the C++ Jolt library for HeightFieldShapeSettings and HeightFieldShape. The height field is built from its settings
// on both sides (HFDesc + the sample, material index and material arrays) and kept alive between calls through an
// opaque handle (a Shape * with a reference, released by jolt_hf_release), because building a height field is the
// expensive part. ZoltParity/Physics/HeightFieldShapeParity.zig calls these and checks that Zolt produces the same bits.
//
// Materials come from a table of kNumMaterials PhysicsMaterialSimple("Material <i>", Color::sGetDistinctColor(i)) that
// both sides build the same way; materials are passed and returned as indices into that table (kDefaultMaterial for
// PhysicsMaterial::sDefault, kUnknownMaterial for anything else).
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, planes as 4 floats (normal, constant). Booleans are passed as int (never bool, see
// the porting guide). Strided arrays are passed as a base pointer, the offset of the first row and the stride (the
// stride can be negative). The structs must match the extern structs in HeightFieldShapeParity.zig. Everything except
// the C ABI is in an anonymous namespace (other reference files have helpers with the same names).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/StringTools.h>
#include <Jolt/Core/TempAllocator.h>
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
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

namespace {

constexpr uint32 kNumMaterials = 300;
constexpr uint32 kDefaultMaterial = 0xfffffffe;
constexpr uint32 kUnknownMaterial = 0xffffffff;

PhysicsMaterialList *sMaterials = nullptr;

void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
	if (sMaterials == nullptr)
	{
		sMaterials = new PhysicsMaterialList;
		for (uint32 i = 0; i < kNumMaterials; ++i)
			sMaterials->push_back(new PhysicsMaterialSimple("Material " + ConvertToString(i), Color::sGetDistinctColor(i)));
	}
}

uint32 MaterialIndex(const PhysicsMaterial *inMaterial)
{
	if (inMaterial == PhysicsMaterial::sDefault)
		return kDefaultMaterial;
	for (uint32 i = 0; i < kNumMaterials; ++i)
		if ((*sMaterials)[i] == inMaterial)
			return i;
	return kUnknownMaterial;
}

Vec3 Load3(const float *inV)						{ return Vec3(inV[0], inV[1], inV[2]); }
Vec4 Load4(const float *inV)						{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
Quat LoadQuat(const float *inV)						{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
Mat44 LoadMat44(const float *inM)					{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
AABox LoadAABox(const float *inBox)					{ return AABox(Load3(inBox), Load3(inBox + 3)); }
void Store3(Vec3Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
void Store4(Vec4Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); outV[3] = inV.GetW(); }
void StoreMat44(Mat44Arg inM, float *outM)			{ for (int c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
void StoreAABox(const AABox &inBox, float *outBox)	{ Store3(inBox.mMin, outBox); Store3(inBox.mMax, outBox + 3); }
void StorePlane(const Plane &inPlane, float *outPlane) { Store3(inPlane.GetNormal(), outPlane); outPlane[3] = inPlane.GetConstant(); }
SubShapeIDCreator MakeCreator(const uint32 *inCreator) { return inCreator[1] == 0? SubShapeIDCreator() : SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }

void StoreFace(const Shape::SupportingFace &inFace, uint32 &outCount, float *outFace)
{
	outCount = uint32(inFace.size());
	for (uint32 i = 0; i < outCount; ++i)
		Store3(inFace[i], outFace + 3 * i);
}

uint32 StoreBytes(const std::string &inBytes, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inBytes.size());
	if (size <= inCapacity)
		memcpy(outBytes, inBytes.data(), size);
	return size;
}

// Height field description, must match HFDesc in HeightFieldShapeParity.zig
struct HFDesc
{
	float					mOffset[3];
	float					mScale[3];
	uint32					mSampleCount;
	float					mMinHeightValue;
	float					mMaxHeightValue;
	uint32					mMaterialsCapacity;
	uint32					mBlockSize;
	uint32					mBitsPerSample;
	float					mActiveEdgeCosThresholdAngle;
	uint32					mNumSamples;			// Number of entries in the samples array
	uint32					mNumMaterialIndices;	// Number of entries in the material index array
	uint32					mNumMaterials;			// Number of entries in the material array (indices into the material table)
	uint64					mUserData;
};

void FillSettings(const HFDesc &inDesc, const float *inSamples, const uint8 *inMaterialIndices, const uint32 *inMaterials, HeightFieldShapeSettings &outSettings)
{
	outSettings.mOffset = Load3(inDesc.mOffset);
	outSettings.mScale = Load3(inDesc.mScale);
	outSettings.mSampleCount = inDesc.mSampleCount;
	outSettings.mMinHeightValue = inDesc.mMinHeightValue;
	outSettings.mMaxHeightValue = inDesc.mMaxHeightValue;
	outSettings.mMaterialsCapacity = inDesc.mMaterialsCapacity;
	outSettings.mBlockSize = inDesc.mBlockSize;
	outSettings.mBitsPerSample = inDesc.mBitsPerSample;
	outSettings.mActiveEdgeCosThresholdAngle = inDesc.mActiveEdgeCosThresholdAngle;
	outSettings.mHeightSamples.assign(inSamples, inSamples + inDesc.mNumSamples);
	outSettings.mMaterialIndices.assign(inMaterialIndices, inMaterialIndices + inDesc.mNumMaterialIndices);
	for (uint32 i = 0; i < inDesc.mNumMaterials; ++i)
		outSettings.mMaterials.push_back((*sMaterials)[inMaterials[i]]);
	outSettings.mUserData = inDesc.mUserData;
}

const HeightFieldShape *HF(const void *inShape)		{ return static_cast<const HeightFieldShape *>(static_cast<const Shape *>(inShape)); }
HeightFieldShape *HFMut(void *inShape)				{ return static_cast<HeightFieldShape *>(static_cast<Shape *>(inShape)); }

std::string SaveState(const Shape *inShape)
{
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	inShape->SaveBinaryState(stream_out);
	return data.str();
}

// The properties of a height field, must match PropertiesInput / PropertiesOutput in HeightFieldShapeParity.zig
struct PropertiesInput
{
	float					mScale[3];
	float					mTransform[16];
	double					mTranslation[3];		// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
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
	int						mMustBeStatic;
	float					mMinHeightValue;
	float					mMaxHeightValue;
	uint32					mSampleCount;
	uint32					mBlockSize;
	uint32					mNumMaterials;
	uint32					mMaterials[260];		// Material list (table indices)
};

// The sub shape functions for one sub shape ID, must match SubShapeInput / SubShapeOutput in HeightFieldShapeParity.zig
struct SubShapeInput
{
	uint32					mSubShapeID;
	float					mPoint[3];
	float					mDirection[3];
	float					mScale[3];
	float					mTransform[16];
};

struct SubShapeOutput
{
	float					mNormal[3];
	uint32					mMaterial;
	uint32					mX;
	uint32					mY;
	uint32					mTriangle;
	uint32					mFaceCount;
	float					mFace[32 * 3];
	uint32					mLeafRemainder;
	int						mLeafIsSelf;
	uint64					mUserData;
};

// The input of a ray cast, must match RayInput / RayOutput in HeightFieldShapeParity.zig
struct RayInput
{
	float					mOrigin[3];
	float					mDirection[3];
	uint32					mCreator[2];			// Sub shape ID creator: value pushed, number of bits (0: no push)
	float					mFraction;				// Initial fraction of the single hit version
	int						mBackFaceMode;			// 1: collide with back faces (triangles)
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
	float					mEarlyOut;				// Early out fraction of the collector (when < the initial one)
	uint32					mBodyID;				// Body ID of the collector context
	uint32					mRejectID;				// Shape filter: reject this sub shape ID (0xffffffff = default filter)
};

constexpr uint32 kMaxRayHits = 64;

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
	RayHit					mHits[kMaxRayHits];
};

// A convex shape to collide / cast with, must match ConvexDesc in HeightFieldShapeParity.zig
struct ConvexDesc
{
	uint32					mKind;					// 0: SphereShape, 1: BoxShape
	float					mRadius;				// SphereShape
	float					mHalfExtent[3];			// BoxShape
	float					mConvexRadius;			// BoxShape
};

Ref<Shape> CreateConvex(const ConvexDesc &inDesc)
{
	if (inDesc.mKind == 0)
		return new SphereShape(inDesc.mRadius);
	return new BoxShape(Load3(inDesc.mHalfExtent), inDesc.mConvexRadius);
}

// A collision between a convex shape and the height field, must match CollideInput in HeightFieldShapeParity.zig
struct CollideInput
{
	ConvexDesc				mConvex;
	int						mHeightFieldFirst;		// 1: collide the height field (shape 1) with the convex shape (shape 2)
	float					mScale1[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mMaxSeparationDistance;
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	int						mActiveEdgeMode;		// 0: CollideOnlyWithActive, 1: CollideWithAll
	int						mBackFaceMode;			// 1: collide with back faces
	int						mCollectFaces;
	float					mActiveEdgeMovementDirection[3];
	float					mEarlyOut;				// Early out fraction of the collector (when < FLT_MAX)
	uint32					mBodyID;
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
};

constexpr uint32 kMaxHits = 24;

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
	HitOutput				mHits[kMaxHits];
};

// A shape cast between a convex shape and the height field, must match CastInput in HeightFieldShapeParity.zig
struct CastInput
{
	ConvexDesc				mConvex;
	int						mHeightFieldCast;		// 1: cast the height field against the convex shape
	float					mScale1[3];				// Scale of the cast shape
	float					mStart[16];
	float					mDirection[3];
	float					mScale2[3];				// Scale of the target shape
	float					mTransform2[16];
	uint32					mCreator1[2];
	uint32					mCreator2[2];
	float					mCollisionTolerance;
	float					mPenetrationTolerance;
	float					mExtraConvexRadius;
	int						mActiveEdgeMode;
	int						mBackFaceModeTriangles;
	int						mBackFaceModeConvex;
	int						mUseShrunkenShape;
	int						mReturnDeepestPoint;
	int						mCollectFaces;
	float					mActiveEdgeMovementDirection[3];
	float					mEarlyOut;				// Early out fraction of the collector (when < 1 + FLT_EPSILON)
	uint32					mBodyID;
	int						mCollector;				// 0: AllHit, 1: AnyHit, 2: ClosestHit
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

void StoreHit(const CollideShapeResult &inResult, HitsOutput &outOutput)
{
	if (outOutput.mNumHits < kMaxHits)
	{
		HitOutput &h = outOutput.mHits[outOutput.mNumHits];
		h.mFraction = 0.0f;
		h.mBackFace = 0;
		StoreCollideHit(inResult, h);
	}
	++outOutput.mNumHits;
}

void StoreHit(const ShapeCastResult &inResult, HitsOutput &outOutput)
{
	if (outOutput.mNumHits < kMaxHits)
	{
		HitOutput &h = outOutput.mHits[outOutput.mNumHits];
		h.mFraction = inResult.mFraction;
		h.mBackFace = inResult.mIsBackFaceHit? 1 : 0;
		StoreCollideHit(inResult, h);
	}
	++outOutput.mNumHits;
}

// A shape filter that rejects one sub shape ID (of shape 2)
class RejectFilter : public ShapeFilter
{
public:
	explicit				RejectFilter(uint32 inReject) : mReject(inReject) { }

	virtual bool			ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override { return inSubShapeIDOfShape2.GetValue() != mReject; }

	uint32					mReject;
};

// Run a query with the collector selected by inCollector and store the hits
template <class CollectorType, class Query, class Store>
void RunWithCollector(int inCollector, float inEarlyOut, const TransformedShape &inContext, const Query &inQuery, const Store &inStore)
{
	switch (inCollector)
	{
	case 0:
		{
			AllHitCollisionCollector<CollectorType> collector;
			collector.SetContext(&inContext);
			if (inEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inEarlyOut);
			inQuery(collector);
			for (const typename CollectorType::ResultType &h : collector.mHits)
				inStore(h);
			break;
		}

	case 1:
		{
			AnyHitCollisionCollector<CollectorType> collector;
			collector.SetContext(&inContext);
			if (inEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inEarlyOut);
			inQuery(collector);
			if (collector.HadHit())
				inStore(collector.mHit);
			break;
		}

	default:
		{
			ClosestHitCollisionCollector<CollectorType> collector;
			collector.SetContext(&inContext);
			if (inEarlyOut < collector.GetEarlyOutFraction())
				collector.UpdateEarlyOutFraction(inEarlyOut);
			inQuery(collector);
			if (collector.HadHit())
				inStore(collector.mHit);
			break;
		}
	}
}

} // namespace

extern "C" {

// Create the height field from its settings: returns 1 and the shape (with a reference) in outShape if valid, otherwise 0
// and the error text in outError (128 bytes, 0 terminated)
int jolt_hf_create(const HFDesc *inDesc, const float *inSamples, const uint8 *inMaterialIndices, const uint32 *inMaterials, void **outShape, char *outError)
{
	EnsureFactory();
	memset(outError, 0, 128);
	*outShape = nullptr;
	HeightFieldShapeSettings settings;
	settings.SetEmbedded();
	FillSettings(*inDesc, inSamples, inMaterialIndices, inMaterials, settings);
	Shape::ShapeResult result = settings.Create();
	if (result.HasError())
	{
		strncpy(outError, result.GetError().c_str(), 127);
		return 0;
	}
	Shape *shape = result.Get();
	shape->AddRef();
	*outShape = shape;
	return 1;
}

void jolt_hf_release(void *inShape)
{
	static_cast<Shape *>(inShape)->Release();
}

// HeightFieldShapeSettings::DetermineMinAndMaxSample (3 floats) and CalculateBitsPerSampleForError(inMaxError)
uint32 jolt_hf_settings_info(const HFDesc *inDesc, const float *inSamples, float inMaxError, float *outMinMaxScale)
{
	EnsureFactory();
	HFDesc desc = *inDesc;
	desc.mNumMaterialIndices = 0;
	desc.mNumMaterials = 0;
	HeightFieldShapeSettings settings;
	settings.SetEmbedded();
	FillSettings(desc, inSamples, nullptr, nullptr, settings);
	settings.DetermineMinAndMaxSample(outMinMaxScale[0], outMinMaxScale[1], outMinMaxScale[2]);
	return settings.CalculateBitsPerSampleForError(inMaxError);
}

// SaveBinaryState of the height field
uint32 jolt_hf_binary_state(const void *inShape, uint8 *outBytes, uint32 inCapacity)
{
	return StoreBytes(SaveState(static_cast<const Shape *>(inShape)), outBytes, inCapacity);
}

// Clone, then SaveBinaryState of the clone
uint32 jolt_hf_clone_state(const void *inShape, uint8 *outBytes, uint32 inCapacity)
{
	Ref<HeightFieldShape> clone = HF(inShape)->Clone();
	return StoreBytes(SaveState(clone), outBytes, inCapacity);
}

// sRestoreFromBinaryState of inBytes, RestoreMaterialState with the materials of the table (inMaterials), then
// SaveBinaryState of the restored shape. Returns the size (0 if the restore failed).
uint32 jolt_hf_restore_state(const uint8 *inBytes, uint32 inSize, const uint32 *inMaterials, uint32 inNumMaterials, uint8 *outBytes, uint32 inCapacity, uint32 *outMaterials, uint32 *outNumMaterials)
{
	EnsureFactory();
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper stream_in(data);
	Shape::ShapeResult result = Shape::sRestoreFromBinaryState(stream_in);
	*outNumMaterials = 0;
	if (!result.IsValid())
		return 0;
	PhysicsMaterialList materials;
	for (uint32 i = 0; i < inNumMaterials; ++i)
		materials.push_back((*sMaterials)[inMaterials[i]]);
	result.Get()->RestoreMaterialState(materials.data(), uint(materials.size()));
	PhysicsMaterialList saved;
	result.Get()->SaveMaterialState(saved);
	for (const PhysicsMaterial *m : saved)
		outMaterials[(*outNumMaterials)++] = MaterialIndex(m);
	return StoreBytes(SaveState(result.Get()), outBytes, inCapacity);
}

// SaveWithChildren, then sRestoreWithChildren and SaveWithChildren of the restored shape (into outRestoredBytes).
// Returns the size of the first stream.
uint32 jolt_hf_save_with_children(const void *inShape, uint8 *outBytes, uint32 inCapacity, uint8 *outRestoredBytes, uint32 *outRestoredSize)
{
	const Shape *shape = static_cast<const Shape *>(inShape);
	Shape::ShapeToIDMap shape_map;
	Shape::MaterialToIDMap material_map;
	std::stringstream data;
	StreamOutWrapper stream_out(data);
	shape->SaveWithChildren(stream_out, shape_map, material_map);
	std::string bytes = data.str();
	uint32 size = StoreBytes(bytes, outBytes, inCapacity);

	*outRestoredSize = 0;
	std::stringstream in_data(bytes);
	StreamInWrapper stream_in(in_data);
	Shape::IDToShapeMap id_to_shape;
	Shape::IDToMaterialMap id_to_material;
	Shape::ShapeResult result = Shape::sRestoreWithChildren(stream_in, id_to_shape, id_to_material);
	if (result.IsValid())
	{
		Shape::ShapeToIDMap shape_map2;
		Shape::MaterialToIDMap material_map2;
		std::stringstream restored;
		StreamOutWrapper restored_out(restored);
		result.Get()->SaveWithChildren(restored_out, shape_map2, material_map2);
		*outRestoredSize = StoreBytes(restored.str(), outRestoredBytes, inCapacity);
	}
	return size;
}

// Bounds, mass properties, volume, scales and the simple getters
void jolt_hf_properties(const void *inShape, const PropertiesInput *inInput, PropertiesOutput *outOutput)
{
	const HeightFieldShape *shape = HF(inShape);
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
	o.mMustBeStatic = shape->MustBeStatic()? 1 : 0;
	o.mMinHeightValue = shape->GetMinHeightValue();
	o.mMaxHeightValue = shape->GetMaxHeightValue();
	o.mSampleCount = shape->GetSampleCount();
	o.mBlockSize = shape->GetBlockSize();
	const PhysicsMaterialList &materials = shape->GetMaterialList();
	o.mNumMaterials = uint32(materials.size());
	for (uint32 i = 0; i < o.mNumMaterials && i < 260; ++i)
		o.mMaterials[i] = MaterialIndex(materials[i]);
}

// GetPosition and IsNoCollision of every sample (GetSampleCount()^2), GetMaterial(x, y) of every quad ((GetSampleCount() - 1)^2)
void jolt_hf_samples(const void *inShape, float *outPositions, int *outNoCollision, uint32 *outMaterials)
{
	const HeightFieldShape *shape = HF(inShape);
	uint32 count = shape->GetSampleCount();
	for (uint32 y = 0; y < count; ++y)
		for (uint32 x = 0; x < count; ++x)
		{
			Store3(shape->GetPosition(x, y), outPositions + 3 * (y * count + x));
			outNoCollision[y * count + x] = shape->IsNoCollision(x, y)? 1 : 0;
		}
	for (uint32 y = 0; y + 1 < count; ++y)
		for (uint32 x = 0; x + 1 < count; ++x)
			outMaterials[y * (count - 1) + x] = MaterialIndex(shape->GetMaterial(x, y));
}

// ProjectOntoSurface: returns 1 if there is a surface position
int jolt_hf_project(const void *inShape, const float *inPoint, float *outPosition, uint32 *outSubShapeID)
{
	Vec3 position = Vec3::sReplicate(-1.0f);
	SubShapeID id;
	bool result = HF(inShape)->ProjectOntoSurface(Load3(inPoint), position, id);
	Store3(position, outPosition);
	*outSubShapeID = id.GetValue();
	return result? 1 : 0;
}

// GetHeights into inBase + inFirstRow with stride inStride
void jolt_hf_get_heights(const void *inShape, uint32 inX, uint32 inY, uint32 inSizeX, uint32 inSizeY, float *inBase, int64 inFirstRow, int64 inStride)
{
	HF(inShape)->GetHeights(inX, inY, inSizeX, inSizeY, inBase + inFirstRow, intptr_t(inStride));
}

// GetMaterials into inBase + inFirstRow with stride inStride
void jolt_hf_get_materials(const void *inShape, uint32 inX, uint32 inY, uint32 inSizeX, uint32 inSizeY, uint8 *inBase, int64 inFirstRow, int64 inStride)
{
	HF(inShape)->GetMaterials(inX, inY, inSizeX, inSizeY, inBase + inFirstRow, intptr_t(inStride));
}

// SetHeights from inBase + inFirstRow with stride inStride
void jolt_hf_set_heights(void *inShape, uint32 inX, uint32 inY, uint32 inSizeX, uint32 inSizeY, const float *inBase, int64 inFirstRow, int64 inStride, float inActiveEdgeCosThresholdAngle)
{
	TempAllocatorMalloc temp_allocator;
	HFMut(inShape)->SetHeights(inX, inY, inSizeX, inSizeY, inBase + inFirstRow, intptr_t(inStride), temp_allocator, inActiveEdgeCosThresholdAngle);
}

// SetMaterials from inBase + inFirstRow with stride inStride, the material list is inListCount indices into the
// material table (or the current list when inListCount < 0). Returns the result of SetMaterials.
int jolt_hf_set_materials(void *inShape, uint32 inX, uint32 inY, uint32 inSizeX, uint32 inSizeY, const uint8 *inBase, int64 inFirstRow, int64 inStride, const uint32 *inList, int inListCount)
{
	TempAllocatorMalloc temp_allocator;
	PhysicsMaterialList list;
	for (int i = 0; i < inListCount; ++i)
		list.push_back((*sMaterials)[inList[i]]);
	return HFMut(inShape)->SetMaterials(inX, inY, inSizeX, inSizeY, inBase + inFirstRow, intptr_t(inStride), inListCount >= 0? &list : nullptr, temp_allocator)? 1 : 0;
}

// GetSurfaceNormal, GetMaterial, GetSubShapeCoordinates, GetSupportingFace, GetLeafShape and GetSubShapeUserData of one sub shape ID
void jolt_hf_sub_shape(const void *inShape, const SubShapeInput *inInput, SubShapeOutput *outOutput)
{
	const HeightFieldShape *shape = HF(inShape);
	SubShapeID id;
	id.SetValue(inInput->mSubShapeID);
	SubShapeOutput &o = *outOutput;
	Store3(shape->GetSurfaceNormal(id, Load3(inInput->mPoint)), o.mNormal);
	o.mMaterial = MaterialIndex(shape->GetMaterial(id));
	shape->GetSubShapeCoordinates(id, o.mX, o.mY, o.mTriangle);
	Shape::SupportingFace face;
	shape->GetSupportingFace(id, Load3(inInput->mDirection), Load3(inInput->mScale), LoadMat44(inInput->mTransform), face);
	StoreFace(face, o.mFaceCount, o.mFace);
	SubShapeID remainder;
	const Shape *leaf = shape->GetLeafShape(id, remainder);
	o.mLeafRemainder = remainder.GetValue();
	o.mLeafIsSelf = leaf == shape? 1 : 0;
	o.mUserData = shape->GetSubShapeUserData(id);
}

// CastRay (the single hit version, starting from inInput->mFraction) and CastRay with a collector
void jolt_hf_cast_ray(const void *inShape, const RayInput *inInput, RayOutput *outOutput)
{
	const Shape *shape = static_cast<const Shape *>(inShape);
	RayCast ray { Load3(inInput->mOrigin), Load3(inInput->mDirection) };
	SubShapeIDCreator creator = MakeCreator(inInput->mCreator);

	RayCastResult hit;
	hit.mFraction = inInput->mFraction;
	outOutput->mHit = shape->CastRay(ray, creator, hit)? 1 : 0;
	outOutput->mFraction = hit.mFraction;
	outOutput->mSubShapeID = hit.mSubShapeID2.GetValue();

	RayCastSettings settings;
	settings.mBackFaceModeTriangles = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	RejectFilter reject_filter(inInput->mRejectID);
	ShapeFilter default_filter;
	const ShapeFilter &filter = inInput->mRejectID != 0xffffffff? static_cast<const ShapeFilter &>(reject_filter) : default_filter;
	outOutput->mNumHits = 0;
	RunWithCollector<CastRayCollector>(inInput->mCollector, inInput->mEarlyOut, context,
		[&](CastRayCollector &ioCollector) { shape->CastRay(ray, settings, creator, ioCollector, filter); },
		[outOutput](const RayCastResult &inHit) {
			if (outOutput->mNumHits < kMaxRayHits)
			{
				RayHit &h = outOutput->mHits[outOutput->mNumHits];
				h.mFraction = inHit.mFraction;
				h.mBodyID = inHit.mBodyID.GetIndexAndSequenceNumber();
				h.mSubShapeID = inHit.mSubShapeID2.GetValue();
			}
			++outOutput->mNumHits;
		});
}

// CollidePoint: the number of hits
uint32 jolt_hf_collide_point(const void *inShape, const float *inPoint, const uint32 *inCreator)
{
	AllHitCollisionCollector<CollidePointCollector> collector;
	static_cast<const Shape *>(inShape)->CollidePoint(Load3(inPoint), MakeCreator(inCreator), collector);
	return uint32(collector.mHits.size());
}

// CollisionDispatch::sCollideShapeVsShape between the convex shape and the height field (in either order)
void jolt_hf_collide(const void *inShape, const CollideInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	const Shape *height_field = static_cast<const Shape *>(inShape);
	Ref<Shape> convex = CreateConvex(inInput->mConvex);
	const Shape *shape1 = inInput->mHeightFieldFirst != 0? height_field : convex.GetPtr();
	const Shape *shape2 = inInput->mHeightFieldFirst != 0? convex.GetPtr() : height_field;
	CollideShapeSettings settings;
	settings.mMaxSeparationDistance = inInput->mMaxSeparationDistance;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mBackFaceMode = inInput->mBackFaceMode != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	outOutput->mNumHits = 0;
	RunWithCollector<CollideShapeCollector>(inInput->mCollector, inInput->mEarlyOut, context,
		[&](CollideShapeCollector &ioCollector) { CollisionDispatch::sCollideShapeVsShape(shape1, shape2, Load3(inInput->mScale1), Load3(inInput->mScale2), LoadMat44(inInput->mTransform1), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), settings, ioCollector); },
		[outOutput](const CollideShapeResult &inHit) { StoreHit(inHit, *outOutput); });
}

// CollisionDispatch::sCastShapeVsShapeWorldSpace of the convex shape against the height field (or the height field
// against the convex shape)
void jolt_hf_cast(const void *inShape, const CastInput *inInput, HitsOutput *outOutput)
{
	EnsureFactory();
	const Shape *height_field = static_cast<const Shape *>(inShape);
	Ref<Shape> convex = CreateConvex(inInput->mConvex);
	const Shape *cast_shape = inInput->mHeightFieldCast != 0? height_field : convex.GetPtr();
	const Shape *target = inInput->mHeightFieldCast != 0? convex.GetPtr() : height_field;
	ShapeCastSettings settings;
	settings.mCollisionTolerance = inInput->mCollisionTolerance;
	settings.mPenetrationTolerance = inInput->mPenetrationTolerance;
	settings.mExtraConvexRadius = inInput->mExtraConvexRadius;
	settings.mActiveEdgeMode = inInput->mActiveEdgeMode != 0? EActiveEdgeMode::CollideWithAll : EActiveEdgeMode::CollideOnlyWithActive;
	settings.mBackFaceModeTriangles = inInput->mBackFaceModeTriangles != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mBackFaceModeConvex = inInput->mBackFaceModeConvex != 0? EBackFaceMode::CollideWithBackFaces : EBackFaceMode::IgnoreBackFaces;
	settings.mUseShrunkenShapeAndConvexRadius = inInput->mUseShrunkenShape != 0;
	settings.mReturnDeepestPoint = inInput->mReturnDeepestPoint != 0;
	settings.mCollectFacesMode = inInput->mCollectFaces != 0? ECollectFacesMode::CollectFaces : ECollectFacesMode::NoFaces;
	settings.mActiveEdgeMovementDirection = Load3(inInput->mActiveEdgeMovementDirection);
	ShapeCast shape_cast(cast_shape, Load3(inInput->mScale1), LoadMat44(inInput->mStart), Load3(inInput->mDirection));
	TransformedShape context(RVec3::sZero(), Quat::sIdentity(), nullptr, BodyID(inInput->mBodyID));
	outOutput->mNumHits = 0;
	RunWithCollector<CastShapeCollector>(inInput->mCollector, inInput->mEarlyOut, context,
		[&](CastShapeCollector &ioCollector) { CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, settings, target, Load3(inInput->mScale2), ShapeFilter(), LoadMat44(inInput->mTransform2), MakeCreator(inInput->mCreator1), MakeCreator(inInput->mCreator2), ioCollector); },
		[outOutput](const ShapeCastResult &inHit) { StoreHit(inHit, *outOutput); });
}

// GetTrianglesStart, then GetTrianglesNext with inMaxTrianglesRequested until it returns 0 (or the output is full): the
// count of every call (at most inMaxCalls calls), the vertices (at most inMaxTriangles triangles) and the material of
// every triangle (table index; only when inWithMaterials != 0). Returns the number of calls.
int jolt_hf_triangles(const void *inShape, const float *inBox, const float *inPosition, const float *inRotation, const float *inScale, int inMaxTrianglesRequested, int inWithMaterials, int inMaxCalls, int inMaxTriangles, int *outCounts, float *outVertices, uint32 *outMaterials)
{
	const Shape *shape = static_cast<const Shape *>(inShape);
	Shape::GetTrianglesContext context;
	shape->GetTrianglesStart(context, LoadAABox(inBox), Load3(inPosition), LoadQuat(inRotation), Load3(inScale));
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(inMaxTrianglesRequested);
	int calls = 0;
	int total = 0;
	for (;;)
	{
		int count = shape->GetTrianglesNext(context, inMaxTrianglesRequested, triangles.data(), inWithMaterials != 0? materials.data() : nullptr);
		outCounts[calls++] = count;
		for (int i = 0; i < count && total < inMaxTriangles; ++i, ++total)
		{
			for (int v = 0; v < 3; ++v)
			{
				float *out = outVertices + 9 * total + 3 * v;
				out[0] = triangles[3 * i + v].x; out[1] = triangles[3 * i + v].y; out[2] = triangles[3 * i + v].z;
			}
			outMaterials[total] = inWithMaterials != 0? MaterialIndex(materials[i]) : kUnknownMaterial;
		}
		if (count == 0 || calls == inMaxCalls || total >= inMaxTriangles)
			break;
	}
	return calls;
}

// CollideSoftBodyVertices on inNumVertices vertices (positions as 3 floats, ioPenetrations in / out), ioPlanes 4 floats
// per vertex, ioIndices in / out
void jolt_hf_soft_body(const void *inShape, const float *inTransform, const float *inScale, int inNumVertices, const float *inPositions, const float *inInvMasses, float *ioPenetrations, float *ioPlanes, int *ioIndices, int inCollidingShapeIndex)
{
	const Shape *shape = static_cast<const Shape *>(inShape);
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
