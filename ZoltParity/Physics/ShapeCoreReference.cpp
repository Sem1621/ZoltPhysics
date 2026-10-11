// Reference implementation for the shape core parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library for ScaleHelpers, GetTrianglesContextVertexList / MultiVertexList and their vertex list helpers, ShapeCast /
// RShapeCast / ShapeCastResult, TransformedShape and the default implementations of Shape (with a test shape that
// derives from Shape directly, like ShapeCoreUserTypes.zig), the collision collectors on synthetic hit sequences, the
// binary state of a shape graph (Shape::SaveWithChildren), CollisionDispatch and the TransformedShape queries through
// collide / cast functions registered for the test shape (like the parity build's zolt_user_types module) and the
// contents of the CollisionDispatch / ShapeFunctions tables. ZoltParity/Physics/ShapeCoreParity.zig calls these and
// checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, RVec3 as 3 Reals (double with JPH_DOUBLE_PRECISION), RMat44 as 3 columns of 4 floats
// + an RVec3 translation. Booleans are passed as int (never bool, see the porting guide).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/PhysicsMaterialSimple.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>
#include <Jolt/Physics/Collision/Shape/GetTrianglesContext.h>
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
#include <Jolt/Physics/SoftBody/SoftBodyShape.h>

#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

static void RegisterParityShapes();

static void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}

	// The user registrations of the parity build, after RegisterTypes (which another reference file may have run: it
	// does not touch the User1 / User2 pairs, so this stays valid if it runs again)
	static bool sParityShapesRegistered = false;
	if (!sParityShapesRegistered)
	{
		RegisterParityShapes();
		sParityShapesRegistered = true;
	}
}

static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Quat LoadQuat(const float *inV)				{ return Quat(inV[0], inV[1], inV[2], inV[3]); }
static RVec3 LoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void Store4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void StoreQuat(QuatArg inQ, float *outV)		{ Store4(inQ.GetXYZW(), outV); }
static void StoreR3(RVec3Arg inV, Real *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Mat44 LoadMat44(const float *inM)			{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
static void StoreMat44(Mat44Arg inM, float *outM)	{ for (int c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
static RMat44 LoadRMat44(const float *inCols, const Real *inTranslation) { return RMat44(Load4(inCols), Load4(inCols + 4), Load4(inCols + 8), LoadR3(inTranslation)); }
static void StoreRMat44(RMat44Arg inM, float *outCols, Real *outTranslation) { for (int c = 0; c < 3; ++c) Store4(inM.GetColumn4(c), outCols + 4 * c); StoreR3(inM.GetTranslation(), outTranslation); }
static void StoreAABox(const AABox &inBox, float *outBox) { Store3(inBox.mMin, outBox); Store3(inBox.mMax, outBox + 3); }

static SubShapeID MakeSubShapeID(uint32 inValue)
{
	SubShapeID id;
	id.SetValue(inValue);
	return id;
}

static uint32 CopyBytes(const std::string &inString, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inString.size());
	if (size <= inCapacity)
		memcpy(outBytes, inString.data(), size);
	return size;
}

static void StoreCreator(const SubShapeIDCreator &inCreator, uint32 *outID)	{ outID[0] = inCreator.GetID().GetValue(); outID[1] = inCreator.GetNumBitsWritten(); }

// What a registered collide function received, must match CollideRecord in ShapeCoreUserTypes.zig
struct CollideRecord
{
	uint32					mCalls;
	uint32					mSubTypes[2];
	float					mScales[2][3];
	float					mTransforms[2][16];
	uint32					mIDs[2][2];
	float					mEarlyOut[2];
};

// What a registered cast function received, must match CastRecord in ShapeCoreUserTypes.zig
struct CastRecord
{
	uint32					mCalls;
	uint32					mSubTypes[2];
	float					mStart[16];
	float					mDirection[3];
	float					mCastScale[3];
	float					mBounds[6];
	float					mScale[3];
	float					mTransform2[16];
	uint32					mIDs[2][2];
	float					mEarlyOut[2];
};

// What the overrides of ParityShape received from the TransformedShape queries, must match QueryRecord in
// ShapeCoreUserTypes.zig
struct QueryRecord
{
	float					mTrianglesBox[6];
	float					mTrianglesPosition[3];
	float					mTrianglesRotation[4];
	float					mTrianglesScale[3];
	uint32					mFaceID;
	float					mFaceDirection[3];
	float					mFaceScale[3];
	float					mFaceTransform[16];
	float					mCollectBox[6];
	uint32					mRayCalls;
	uint32					mRayBackFaceModes[2];
};

// The state of a parity test that the shapes point to, like Record in ShapeCoreUserTypes.zig
struct Record
{
	RayCast					mLastRay { Vec3::sZero(), Vec3::sZero() };
	Vec3					mLastPoint = Vec3::sZero();
	float					mHitValues[2] = { 0.0f, 0.0f };
	bool					mPointUsingRayCast = false;
	uint32					mNumRayHits = 0;
	CollideRecord			mCollide { };
	CastRecord				mCast { };
	QueryRecord				mQueries { };
};

// A shape that derives from Shape directly (User1 or User2), must match ParityShape in ShapeCoreUserTypes.zig: a box
// around its center of mass, an optional uniform scale requirement, children and a material for the binary state of a
// graph, and it records what the queries pass to it in its Record
class ParityShape final : public Shape
{
public:
							ParityShape(Vec3Arg inHalfExtent, Vec3Arg inCenterOfMass, bool inUniformScale, Record *inRecord = nullptr, EShapeSubType inSubType = EShapeSubType::User1) : Shape(EShapeType::User1, inSubType), mHalfExtent(inHalfExtent), mCenterOfMass(inCenterOfMass), mUniformScale(inUniformScale), mRecord(inRecord != nullptr? inRecord : &mOwnRecord) { }

	virtual Vec3			GetCenterOfMass() const override									{ return mCenterOfMass; }
	virtual AABox			GetLocalBounds() const override										{ return AABox(-mHalfExtent, mHalfExtent); }
	virtual uint			GetSubShapeIDBitsRecursive() const override							{ return 0; }
	virtual float			GetInnerRadius() const override										{ return mHalfExtent.ReduceMin(); }
	virtual MassProperties	GetMassProperties() const override									{ return MassProperties(); }
	virtual const PhysicsMaterial *GetMaterial(const SubShapeID &inSubShapeID) const override	{ return mMaterial != nullptr? mMaterial.GetPtr() : PhysicsMaterial::sDefault.GetPtr(); }
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { return inLocalSurfacePosition.NormalizedOr(Vec3::sAxisY()); }
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy) const override { outTotalVolume = outSubmergedVolume = 0.0f; outCenterOfBuoyancy = Vec3::sZero(); }
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override { mRecord->mLastRay = inRay; ioHit.mFraction = 0.5f; ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID(); return true; }
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override { }
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { return 0; }
	virtual Stats			GetStats() const override											{ return Stats(sizeof(*this), 0); }
	virtual float			GetVolume() const override											{ return 1.0f; }
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override						{ Vec3 scale = Shape::MakeScaleValid(inScale); return mUniformScale? ScaleHelpers::MakeUniformScale(scale) : scale; }

	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override
	{
		QueryRecord &r = mRecord->mQueries;
		r.mFaceID = inSubShapeID.GetValue();
		Store3(inDirection, r.mFaceDirection);
		Store3(inScale, r.mFaceScale);
		StoreMat44(inCenterOfMassTransform, r.mFaceTransform);
		outVertices.push_back(inCenterOfMassTransform * (inScale * mHalfExtent));
		outVertices.push_back(inCenterOfMassTransform * inDirection);
	}

	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override
	{
		Record &record = *mRecord;
		record.mLastRay = inRay;
		++record.mQueries.mRayCalls;
		record.mQueries.mRayBackFaceModes[0] = inRayCastSettings.mBackFaceModeTriangles == EBackFaceMode::CollideWithBackFaces? 1 : 0;
		record.mQueries.mRayBackFaceModes[1] = inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces? 1 : 0;
		for (uint32 i = 0; i < record.mNumRayHits; ++i)
		{
			RayCastResult hit;
			hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
			hit.mFraction = 0.25f * float(i);
			hit.mSubShapeID2 = inSubShapeIDCreator.PushID(i, 2).GetID();
			ioCollector.AddHit(hit);
		}
	}

	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override
	{
		mRecord->mLastPoint = inPoint;
		if (mRecord->mPointUsingRayCast)
			sCollidePointUsingRayCast(*this, inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
	}

	virtual void			CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override
	{
		StoreAABox(inBox, mRecord->mQueries.mCollectBox);
		Shape::CollectTransformedShapes(inBox, inPositionCOM, inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
	}

	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override
	{
		QueryRecord &r = mRecord->mQueries;
		StoreAABox(inBox, r.mTrianglesBox);
		Store3(inPositionCOM, r.mTrianglesPosition);
		StoreQuat(inRotation, r.mTrianglesRotation);
		Store3(inScale, r.mTrianglesScale);
	}

	virtual void			SaveBinaryState(StreamOut &inStream) const override
	{
		Shape::SaveBinaryState(inStream);
		inStream.Write(mHalfExtent);
		inStream.Write(mCenterOfMass);
	}

	virtual void			SaveMaterialState(PhysicsMaterialList &outMaterials) const override	{ outMaterials.clear(); outMaterials.push_back(mMaterial); }
	virtual void			SaveSubShapeState(ShapeList &outSubShapes) const override			{ outSubShapes.assign(mChildren.begin(), mChildren.end()); }

	Vec3					mHalfExtent;
	Vec3					mCenterOfMass;
	bool					mUniformScale;
	RefConst<PhysicsMaterial> mMaterial;
	Array<RefConst<Shape>>	mChildren;
	Record *				mRecord;
	mutable Record			mOwnRecord;
};

// 0 for User1, 1 for User2 (the sub types of ParityShape)
static uint32 sSubTypeIndex(const Shape *inShape)
{
	return inShape->GetSubType() == EShapeSubType::User2? 1 : 0;
}

// Collide function of the parity shapes, must match collideParity in ShapeCoreUserTypes.zig: records its inputs, adds 2
// hits (when they pass the early out fraction) with contact points, axis and faces computed from the inputs
static void sCollideParity(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	const ParityShape *p1 = static_cast<const ParityShape *>(inShape1);
	const ParityShape *p2 = static_cast<const ParityShape *>(inShape2);
	Record &record = *p1->mRecord;
	CollideRecord &r = record.mCollide;
	++r.mCalls;
	r.mSubTypes[0] = sSubTypeIndex(inShape1);
	r.mSubTypes[1] = sSubTypeIndex(inShape2);
	Store3(inScale1, r.mScales[0]);
	Store3(inScale2, r.mScales[1]);
	StoreMat44(inCenterOfMassTransform1, r.mTransforms[0]);
	StoreMat44(inCenterOfMassTransform2, r.mTransforms[1]);
	StoreCreator(inSubShapeIDCreator1, r.mIDs[0]);
	StoreCreator(inSubShapeIDCreator2, r.mIDs[1]);

	for (int i = 0; i < 2; ++i)
	{
		float depth = record.mHitValues[i];
		if (-depth < ioCollector.GetEarlyOutFraction())
		{
			Vec3 contact1 = inCenterOfMassTransform1.GetTranslation();
			Vec3 contact2 = inCenterOfMassTransform2 * (inScale2 * p2->mHalfExtent);
			CollideShapeResult result(contact1, contact2, contact2 - contact1, depth, inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(), TransformedShape::sGetBodyID(ioCollector.GetContext()));
			result.mShape1Face.push_back(inCenterOfMassTransform1 * (inScale1 * p1->mHalfExtent));
			result.mShape1Face.push_back(inCenterOfMassTransform1 * (inScale1 * -p1->mHalfExtent));
			result.mShape2Face.push_back(inCenterOfMassTransform2 * (inScale2 * p2->mCenterOfMass));
			ioCollector.AddHit(result);
		}
		r.mEarlyOut[i] = ioCollector.GetEarlyOutFraction();
	}
}

// Cast function of the parity shapes, must match castParity in ShapeCoreUserTypes.zig: records its inputs, adds 2 hits
// (when they pass the early out fraction) with contact points, axis and faces computed from the inputs
static void sCastParity(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	const ParityShape *p1 = static_cast<const ParityShape *>(inShapeCast.mShape);
	const ParityShape *p2 = static_cast<const ParityShape *>(inShape);
	Record &record = *p1->mRecord;
	CastRecord &r = record.mCast;
	++r.mCalls;
	r.mSubTypes[0] = sSubTypeIndex(inShapeCast.mShape);
	r.mSubTypes[1] = sSubTypeIndex(inShape);
	StoreMat44(inShapeCast.mCenterOfMassStart, r.mStart);
	Store3(inShapeCast.mDirection, r.mDirection);
	Store3(inShapeCast.mScale, r.mCastScale);
	StoreAABox(inShapeCast.mShapeWorldBounds, r.mBounds);
	Store3(inScale, r.mScale);
	StoreMat44(inCenterOfMassTransform2, r.mTransform2);
	StoreCreator(inSubShapeIDCreator1, r.mIDs[0]);
	StoreCreator(inSubShapeIDCreator2, r.mIDs[1]);

	for (int i = 0; i < 2; ++i)
	{
		float fraction = record.mHitValues[i];
		if (fraction < ioCollector.GetEarlyOutFraction())
		{
			Vec3 contact1 = inCenterOfMassTransform2 * inShapeCast.GetPointOnRay(fraction);
			Vec3 contact2 = inCenterOfMassTransform2 * (inShapeCast.mCenterOfMassStart.GetTranslation() + inScale * p2->mHalfExtent);
			ShapeCastResult result(fraction, contact1, contact2, inCenterOfMassTransform2.Multiply3x3(inShapeCast.mDirection), i == 1, inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(), TransformedShape::sGetBodyID(ioCollector.GetContext()));
			result.mShape1Face.push_back(contact1);
			result.mShape1Face.push_back(Vec3::sReplicate(-0.0f)); // Its reversed copy -0 - fraction * world direction shows the sign of zero components of the world direction
			result.mShape2Face.push_back(inCenterOfMassTransform2 * p2->mCenterOfMass);
			result.mShape2Face.push_back(contact2);
			ioCollector.AddHit(result);
		}
		r.mEarlyOut[i] = ioCollector.GetEarlyOutFraction();
	}
}

// Registration of the parity shape functions, must match ParityShapeRegistration in ShapeCoreUserTypes.zig (the user
// registrations run after RegisterTypes, like Zolt's zolt_user_types registrations)
static void RegisterParityShapes()
{
	for (EShapeSubType s : { EShapeSubType::User1, EShapeSubType::User2 })
	{
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::User1, sCollideParity);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::User1, sCastParity);
	}
	CollisionDispatch::sRegisterCollideShape(EShapeSubType::User2, EShapeSubType::User2, sCollideParity);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::User2, EShapeSubType::User2, sCastParity);
	CollisionDispatch::sRegisterCollideShape(EShapeSubType::User1, EShapeSubType::User2, CollisionDispatch::sReversedCollideShape);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::User1, EShapeSubType::User2, CollisionDispatch::sReversedCastShape);
}

// TransformedShape in the format of the C ABI, must match TS in ShapeCoreParity.zig
struct TS
{
	Real					mPositionCOM[3];
	float					mRotation[4];
	float					mScale[3];
	uint32					mBodyID;
	uint32					mSubShapeID;
	uint32					mSubShapeIDBits;
};

static void StoreTS(const TransformedShape &inTS, TS *outTS)
{
	StoreR3(inTS.mShapePositionCOM, outTS->mPositionCOM);
	StoreQuat(inTS.mShapeRotation, outTS->mRotation);
	Store3(inTS.GetShapeScale(), outTS->mScale);
	outTS->mBodyID = inTS.mBodyID.GetIndexAndSequenceNumber();
	outTS->mSubShapeID = inTS.mSubShapeIDCreator.GetID().GetValue();
	outTS->mSubShapeIDBits = inTS.mSubShapeIDCreator.GetNumBitsWritten();
}

// Collects the transformed shapes (copies, like Zolt's AllHitCollisionCollector)
class TSCollector : public TransformedShapeCollector
{
public:
	virtual void			AddHit(const TransformedShape &inResult) override					{ mHits.push_back(inResult); }

	Array<TransformedShape>	mHits;
};

// Access to the private dispatch tables of CollisionDispatch (explicit instantiation ignores access checks)
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

// The function that the table test installs in every entry (CollisionDispatch::sInit's unsupported function)
static void UnsupportedCollide(const Shape *, const Shape *, Vec3Arg, Vec3Arg, Mat44Arg, Mat44Arg, const SubShapeIDCreator &, const SubShapeIDCreator &, const CollideShapeSettings &, CollideShapeCollector &, const ShapeFilter &) { }
static void UnsupportedCast(const ShapeCast &, const ShapeCastSettings &, const Shape *, Vec3Arg, const ShapeFilter &, Mat44Arg, const SubShapeIDCreator &, const SubShapeIDCreator &, CastShapeCollector &) { }

// Hit results of the collector tests, must match Hit in ShapeCoreParity.zig
struct Hit
{
	uint32					mBodyID;
	float					mFraction;
	float					mPenetrationDepth;
};

// Storage that is passed as `const Body &` to OnBody (ClosestHitPerBodyCollisionCollector ignores the body)
alignas(Body) static uint8 sBodyStorage[sizeof(Body)];

extern "C"
{

// Physics/Collision/Shape/ScaleHelpers.h: out_bools = IsNotScaled, IsUniformScale, IsUniformScaleXZ, IsInsideOut,
// IsZeroScale, CanScaleBeRotated; out_vectors = MakeNonZeroScale, MakeUniformScale, MakeUniformScaleXZ, RotateScale
float jolt_scale_helpers(const float *inScale, float inConvexRadius, const float *inRotation, int *outBools, float *outVectors)
{
	Vec3 scale = Load3(inScale);
	Quat rotation = LoadQuat(inRotation);
	outBools[0] = ScaleHelpers::IsNotScaled(scale)? 1 : 0;
	outBools[1] = ScaleHelpers::IsUniformScale(scale)? 1 : 0;
	outBools[2] = ScaleHelpers::IsUniformScaleXZ(scale)? 1 : 0;
	outBools[3] = ScaleHelpers::IsInsideOut(scale)? 1 : 0;
	outBools[4] = ScaleHelpers::IsZeroScale(scale)? 1 : 0;
	outBools[5] = ScaleHelpers::CanScaleBeRotated(rotation, scale)? 1 : 0;
	Store3(ScaleHelpers::MakeNonZeroScale(scale), outVectors);
	Store3(ScaleHelpers::MakeUniformScale(scale), outVectors + 3);
	Store3(ScaleHelpers::MakeUniformScaleXZ(scale), outVectors + 6);
	Store3(ScaleHelpers::RotateScale(rotation, scale), outVectors + 9);
	return ScaleHelpers::ScaleConvexRadius(inConvexRadius, scale);
}

// GetTrianglesContextVertexList::sCreateHalfUnitSphereTop (inWhich = 0) / Bottom (1) / sCreateUnitOpenCylinder (2),
// returns the number of vertices
uint32 jolt_triangles_vertex_list_helper(int inWhich, int inDetailLevel, float *outVertices, uint32 inCapacity)
{
	EnsureFactory();
	Array<Vec3> vertices;
	switch (inWhich)
	{
	case 0:		GetTrianglesContextVertexList::sCreateHalfUnitSphereTop(vertices, inDetailLevel); break;
	case 1:		GetTrianglesContextVertexList::sCreateHalfUnitSphereBottom(vertices, inDetailLevel); break;
	default:	GetTrianglesContextVertexList::sCreateUnitOpenCylinder(vertices, inDetailLevel); break;
	}
	uint32 count = uint32(vertices.size());
	if (count <= inCapacity)
		for (uint32 i = 0; i < count; ++i)
			Store3(vertices[i], outVertices + 3 * i);
	return count;
}

// GetTrianglesContextVertexList on inNumVertices vertices, GetTrianglesNext with inMaxTrianglesRequested until it
// returns 0: the counts per call (at most 64 calls) and the vertices, returns the number of calls
int jolt_triangles_vertex_list(const float *inPositionCOM, const float *inRotation, const float *inScale, const float *inLocalTransform, const float *inVertices, uint32 inNumVertices, int inMaxTrianglesRequested, int *outCounts, float *outVertices)
{
	EnsureFactory();
	Array<Vec3> vertices;
	for (uint32 i = 0; i < inNumVertices; ++i)
		vertices.push_back(Load3(inVertices + 3 * i));
	Shape::GetTrianglesContext context;
	GetTrianglesContextVertexList *list = new (&context) GetTrianglesContextVertexList(Load3(inPositionCOM), LoadQuat(inRotation), Load3(inScale), LoadMat44(inLocalTransform), vertices.data(), vertices.size(), PhysicsMaterial::sDefault.GetPtr());
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	Array<const PhysicsMaterial *> materials(inMaxTrianglesRequested);
	int calls = 0;
	float *out = outVertices;
	for (;;)
	{
		int count = list->GetTrianglesNext(inMaxTrianglesRequested, triangles.data(), materials.data());
		outCounts[calls++] = count;
		for (int i = 0; i < 3 * count; ++i, out += 3)
		{
			out[0] = triangles[i].x; out[1] = triangles[i].y; out[2] = triangles[i].z;
		}
		if (count == 0 || calls == 64)
			break;
	}
	return calls;
}

// GetTrianglesContextMultiVertexList with up to 3 parts (inPartSizes vertices each, transforms in inTransforms)
int jolt_triangles_multi_vertex_list(int inInsideOut, int inNumParts, const float *inTransforms, const float *inVertices, const uint32 *inPartSizes, int inMaxTrianglesRequested, int *outCounts, float *outVertices)
{
	EnsureFactory();
	Array<Vec3> vertices;
	uint32 total = 0;
	for (int p = 0; p < inNumParts; ++p)
		total += inPartSizes[p];
	for (uint32 i = 0; i < total; ++i)
		vertices.push_back(Load3(inVertices + 3 * i));
	Shape::GetTrianglesContext context;
	GetTrianglesContextMultiVertexList *list = new (&context) GetTrianglesContextMultiVertexList(inInsideOut != 0, PhysicsMaterial::sDefault.GetPtr());
	uint32 offset = 0;
	for (int p = 0; p < inNumParts; ++p)
	{
		list->AddPart(LoadMat44(inTransforms + 16 * p), vertices.data() + offset, inPartSizes[p]);
		offset += inPartSizes[p];
	}
	Array<Float3> triangles(inMaxTrianglesRequested * 3);
	int calls = 0;
	float *out = outVertices;
	for (;;)
	{
		int count = list->GetTrianglesNext(inMaxTrianglesRequested, triangles.data(), nullptr);
		outCounts[calls++] = count;
		for (int i = 0; i < 3 * count; ++i, out += 3)
		{
			out[0] = triangles[i].x; out[1] = triangles[i].y; out[2] = triangles[i].z;
		}
		if (count == 0 || calls == 64)
			break;
	}
	return calls;
}

// ShapeCast with a ParityShape: bounds of the constructor, sFromWorldTransform, PostTransformed, PostTranslated and
// GetPointOnRay. out_casts holds 4 casts (start 16, direction 3, bounds 6 floats each): constructed, from world transform,
// post transformed, post translated
void jolt_shape_cast(const float *inHalfExtent, const float *inCenterOfMass, const float *inScale, const float *inStart, const float *inDirection, const float *inTransform, const float *inTranslation, float inFraction, float *outCasts, float *outPoint)
{
	EnsureFactory();
	ParityShape shape(Load3(inHalfExtent), Load3(inCenterOfMass), false);
	shape.SetEmbedded();
	ShapeCast cast(&shape, Load3(inScale), LoadMat44(inStart), Load3(inDirection));
	ShapeCast casts[] = { cast, ShapeCast::sFromWorldTransform(&shape, Load3(inScale), LoadMat44(inStart), Load3(inDirection)), cast.PostTransformed(LoadMat44(inTransform)), cast.PostTranslated(Load3(inTranslation)) };
	for (const ShapeCast &c : casts)
	{
		StoreMat44(c.mCenterOfMassStart, outCasts);
		Store3(c.mDirection, outCasts + 16);
		StoreAABox(c.mShapeWorldBounds, outCasts + 19);
		outCasts += 25;
	}
	Store3(cast.GetPointOnRay(inFraction), outPoint);
}

// RShapeCast with a ParityShape: bounds of the constructor (the RMat44 overload of GetWorldSpaceBounds), sFromWorldTransform,
// PostTranslated, GetPointOnRay and the conversion to ShapeCast. out_columns / out_translations / out_rest hold 3 casts
// (constructed, from world transform, post translated): 12 + 3 + 9 values each
void jolt_r_shape_cast(const float *inHalfExtent, const float *inCenterOfMass, const float *inScale, const float *inStartColumns, const Real *inStartTranslation, const float *inDirection, const Real *inTranslation, float inFraction, float *outColumns, Real *outTranslations, float *outRest, Real *outPoint, float *outShapeCast)
{
	EnsureFactory();
	ParityShape shape(Load3(inHalfExtent), Load3(inCenterOfMass), false);
	shape.SetEmbedded();
	RMat44 start = LoadRMat44(inStartColumns, inStartTranslation);
	RShapeCast cast(&shape, Load3(inScale), start, Load3(inDirection));
	RShapeCast casts[] = { cast, RShapeCast::sFromWorldTransform(&shape, Load3(inScale), start, Load3(inDirection)), cast.PostTranslated(LoadR3(inTranslation)) };
	for (const RShapeCast &c : casts)
	{
		StoreRMat44(c.mCenterOfMassStart, outColumns, outTranslations);
		Store3(c.mDirection, outRest);
		StoreAABox(c.mShapeWorldBounds, outRest + 3);
		outColumns += 12;
		outTranslations += 3;
		outRest += 9;
	}
	StoreR3(cast.GetPointOnRay(inFraction), outPoint);
	ShapeCast single(cast);
	StoreMat44(single.mCenterOfMassStart, outShapeCast);
	Store3(single.mDirection, outShapeCast + 16);
	StoreAABox(single.mShapeWorldBounds, outShapeCast + 19);
}

// ShapeCastResult: the constructor (penetration depth = distance between the contact points), GetEarlyOutFraction and
// Reversed (with faces). out_values: penetration depth, early out fraction, then the reversed result: contact points,
// axis, depth, fraction, early out fraction (14 floats); out_faces: 2 faces of up to 32 vertices each
void jolt_shape_cast_result(float inFraction, const float *inContact1, const float *inContact2, const float *inAxis, int inBackFace, uint32 inID1, uint32 inID2, uint32 inBodyID, const float *inFace1, uint32 inFace1Count, const float *inFace2, uint32 inFace2Count, const float *inDirection, float *outValues, uint32 *outInts, float *outFaces)
{
	ShapeCastResult r(inFraction, Load3(inContact1), Load3(inContact2), Load3(inAxis), inBackFace != 0, MakeSubShapeID(inID1), MakeSubShapeID(inID2), BodyID(inBodyID));
	for (uint32 i = 0; i < inFace1Count; ++i)
		r.mShape1Face.push_back(Load3(inFace1 + 3 * i));
	for (uint32 i = 0; i < inFace2Count; ++i)
		r.mShape2Face.push_back(Load3(inFace2 + 3 * i));
	outValues[0] = r.mPenetrationDepth;
	outValues[1] = r.GetEarlyOutFraction();
	ShapeCastResult rev = r.Reversed(Load3(inDirection));
	Store3(rev.mContactPointOn1, outValues + 2);
	Store3(rev.mContactPointOn2, outValues + 5);
	Store3(rev.mPenetrationAxis, outValues + 8);
	outValues[11] = rev.mPenetrationDepth;
	outValues[12] = rev.mFraction;
	outValues[13] = rev.GetEarlyOutFraction();
	outInts[0] = rev.mSubShapeID1.GetValue();
	outInts[1] = rev.mSubShapeID2.GetValue();
	outInts[2] = rev.mBodyID2.GetIndexAndSequenceNumber();
	outInts[3] = rev.mIsBackFaceHit? 1 : 0;
	outInts[4] = uint32(rev.mShape1Face.size());
	outInts[5] = uint32(rev.mShape2Face.size());
	for (uint32 i = 0; i < rev.mShape1Face.size(); ++i)
		Store3(rev.mShape1Face[i], outFaces + 3 * i);
	for (uint32 i = 0; i < rev.mShape2Face.size(); ++i)
		Store3(rev.mShape2Face[i], outFaces + 96 + 3 * i);
}

// TransformedShape with a ParityShape (inUniformScale: MakeScaleValid makes the scale uniform):
// - out_matrices: GetCenterOfMassTransform, GetInverseCenterOfMassTransform, GetWorldTransform (3 x (12 floats + 3 Reals))
// - out_ts: after SetWorldTransform(position, rotation, scale), after SetWorldTransform(RMat44), GetSubShapeTransformedShape,
//   the TransformShape result (Shape::TransformShape: Decompose + MakeScaleValid), the CollectTransformedShapes result
//   (Shape::CollectTransformedShapes + the position of TransformedShape)
// - out_vectors: GetWorldSpaceBounds (6), GetWorldSpaceSurfaceNormal (3), the local ray that CastRay passed to the shape (6),
//   its fraction and the local point of CollidePoint (3)
void jolt_transformed_shape(const float *inHalfExtent, const float *inCenterOfMass, int inUniformScale, const Real *inPosition, const float *inRotation, const float *inScale, uint32 inSubShapeID, uint32 inSubShapeIDBits, const float *inWorldColumns, const Real *inWorldTranslation, const float *inShapeTransform, const Real *inRayOrigin, const float *inRayDirection, const Real *inPoint, float *outMatrixColumns, Real *outMatrixTranslations, TS *outTS, float *outVectors)
{
	EnsureFactory();
	ParityShape shape(Load3(inHalfExtent), Load3(inCenterOfMass), inUniformScale != 0);
	shape.SetEmbedded();
	SubShapeIDCreator creator = SubShapeIDCreator().PushID(inSubShapeID, inSubShapeIDBits);
	TransformedShape ts(LoadR3(inPosition), LoadQuat(inRotation), &shape, BodyID(7), creator);
	ts.SetShapeScale(Load3(inScale));

	RMat44 matrices[] = { ts.GetCenterOfMassTransform(), ts.GetInverseCenterOfMassTransform(), ts.GetWorldTransform() };
	for (int i = 0; i < 3; ++i)
		StoreRMat44(matrices[i], outMatrixColumns + 12 * i, outMatrixTranslations + 3 * i);

	StoreAABox(ts.GetWorldSpaceBounds(), outVectors);
	Store3(ts.GetWorldSpaceSurfaceNormal(creator.GetID(), LoadR3(inPoint)), outVectors + 6);

	RayCastResult hit;
	ts.CastRay(RRayCast(LoadR3(inRayOrigin), Load3(inRayDirection)), hit);
	Store3(shape.mRecord->mLastRay.mOrigin, outVectors + 9);
	Store3(shape.mRecord->mLastRay.mDirection, outVectors + 12);
	outVectors[15] = hit.mFraction;
	AnyHitCollisionCollector<CollidePointCollector> point_collector;
	ts.CollidePoint(LoadR3(inPoint), point_collector);
	Store3(shape.mRecord->mLastPoint, outVectors + 16);

	TransformedShape set1 = ts;
	set1.SetWorldTransform(LoadR3(inPosition), LoadQuat(inRotation), Load3(inScale));
	StoreTS(set1, outTS);
	TransformedShape set2 = ts;
	set2.SetWorldTransform(LoadRMat44(inWorldColumns, inWorldTranslation));
	StoreTS(set2, outTS + 1);
	SubShapeID remainder;
	StoreTS(ts.GetSubShapeTransformedShape(creator.GetID(), remainder), outTS + 2);

	TSCollector transform_collector;
	shape.TransformShape(LoadMat44(inShapeTransform), transform_collector);
	StoreTS(transform_collector.mHits[0], outTS + 3);

	TSCollector collect_collector;
	ts.CollectTransformedShapes(AABox::sBiggest(), collect_collector);
	StoreTS(collect_collector.mHits[0], outTS + 4);
}

// Inputs of the dispatch / TransformedShape query test, must match DispatchInput in ShapeCoreParity.zig
struct DispatchInput
{
	float					mHalfExtents[2][3];
	float					mCentersOfMass[2][3];
	uint32					mSubTypes[2];
	float					mHitValues[2];
	float					mScales[2][3];
	float					mTransforms[2][16];
	uint32					mCreators[2][2];
	float					mDirection[3];
	Real					mPosition[3];
	float					mRotation[4];
	float					mTSScale[3];
	float					mQueryColumns[12];
	Real					mQueryTranslation[3];
	Real					mBaseOffset[3];
	float					mBox[6];
	uint32					mFaceID[2];
	float					mFaceDirection[3];
	Real					mPoint[3];
	uint32					mNumRayHits;
};

// A collide / cast hit of a ClosestHitCollisionCollector, must match HitOut in ShapeCoreParity.zig
struct HitOut
{
	uint32					mHadHit;
	float					mContact1[3];
	float					mContact2[3];
	float					mAxis[3];
	float					mDepth;
	float					mFraction;
	uint32					mBackFace;
	uint32					mIDs[2];
	uint32					mBodyID;
	uint32					mFaceCounts[2];
	float					mFaces[2][3][3];
	float					mEarlyOut;
};

// Results of the dispatch / TransformedShape query test, must match DispatchOutput in ShapeCoreParity.zig
struct DispatchOutput
{
	CollideRecord			mCollide[2];
	HitOut					mCollideHits[2];
	CastRecord				mCast[3];
	HitOut					mCastHits[3];
	QueryRecord				mQueries;
	uint32					mFaceCount;
	float					mFace[3][3];
	uint32					mCollectCount;
	float					mPoint[3];
	float					mRay[2][3];
	uint32					mPointHits;
	uint32					mPointIDs[2];
};

static void StoreHit(bool inHadHit, const CollideShapeResult &inResult, float inFraction, bool inBackFace, float inEarlyOut, HitOut &outHit)
{
	memset(&outHit, 0, sizeof(outHit));
	outHit.mEarlyOut = inEarlyOut;
	if (!inHadHit)
		return;
	outHit.mHadHit = 1;
	Store3(inResult.mContactPointOn1, outHit.mContact1);
	Store3(inResult.mContactPointOn2, outHit.mContact2);
	Store3(inResult.mPenetrationAxis, outHit.mAxis);
	outHit.mDepth = inResult.mPenetrationDepth;
	outHit.mFraction = inFraction;
	outHit.mBackFace = inBackFace? 1 : 0;
	outHit.mIDs[0] = inResult.mSubShapeID1.GetValue();
	outHit.mIDs[1] = inResult.mSubShapeID2.GetValue();
	outHit.mBodyID = inResult.mBodyID2.GetIndexAndSequenceNumber();
	outHit.mFaceCounts[0] = uint32(inResult.mShape1Face.size());
	outHit.mFaceCounts[1] = uint32(inResult.mShape2Face.size());
	for (uint i = 0; i < min(uint(inResult.mShape1Face.size()), 3u); ++i)
		Store3(inResult.mShape1Face[i], outHit.mFaces[0][i]);
	for (uint i = 0; i < min(uint(inResult.mShape2Face.size()), 3u); ++i)
		Store3(inResult.mShape2Face[i], outHit.mFaces[1][i]);
}

static void StoreCollideHit(const ClosestHitCollisionCollector<CollideShapeCollector> &inCollector, HitOut &outHit)
{
	StoreHit(inCollector.HadHit(), inCollector.mHit, 0.0f, false, inCollector.GetEarlyOutFraction(), outHit);
}

static void StoreCastHit(const ClosestHitCollisionCollector<CastShapeCollector> &inCollector, HitOut &outHit)
{
	StoreHit(inCollector.HadHit(), inCollector.mHit, inCollector.mHit.mFraction, inCollector.mHit.mIsBackFaceHit, inCollector.GetEarlyOutFraction(), outHit);
}

// CollisionDispatch and the TransformedShape queries with two ParityShapes A and B (User1 / User2, the registered
// functions of RegisterParityShapes, User1 vs User2 through the reversed functions), like dispatchQueries in
// ShapeCoreParity.zig: sCollideShapeVsShape(A, B), TransformedShape(B)::CollideShape(A), sCastShapeVsShapeWorldSpace /
// LocalSpace(A, B), TransformedShape(B)::CastShape(A), GetTrianglesStart, GetSupportingFace, CollectTransformedShapes
// and CollidePoint through Shape::sCollidePointUsingRayCast
void jolt_dispatch_queries(const DispatchInput *inInput, DispatchOutput *outOutput)
{
	EnsureFactory();
	const DispatchInput &in = *inInput;
	DispatchOutput &out = *outOutput;
	memset(&out, 0, sizeof(out));

	Record record;
	record.mHitValues[0] = in.mHitValues[0];
	record.mHitValues[1] = in.mHitValues[1];
	ParityShape a(Load3(in.mHalfExtents[0]), Load3(in.mCentersOfMass[0]), false, &record, in.mSubTypes[0] == 0? EShapeSubType::User1 : EShapeSubType::User2);
	a.SetEmbedded();
	ParityShape b(Load3(in.mHalfExtents[1]), Load3(in.mCentersOfMass[1]), false, &record, in.mSubTypes[1] == 0? EShapeSubType::User1 : EShapeSubType::User2);
	b.SetEmbedded();
	SubShapeIDCreator creator0 = SubShapeIDCreator().PushID(in.mCreators[0][0], in.mCreators[0][1]);
	SubShapeIDCreator creator1 = SubShapeIDCreator().PushID(in.mCreators[1][0], in.mCreators[1][1]);
	Vec3 scale0 = Load3(in.mScales[0]);
	Vec3 scale1 = Load3(in.mScales[1]);
	Mat44 transform0 = LoadMat44(in.mTransforms[0]);
	Mat44 transform1 = LoadMat44(in.mTransforms[1]);
	Vec3 direction = Load3(in.mDirection);
	CollideShapeSettings collide_settings;
	ShapeCastSettings cast_settings;
	ShapeFilter filter;

	// CollisionDispatch::sCollideShapeVsShape (User1 vs User2 goes through sReversedCollideShape)
	ClosestHitCollisionCollector<CollideShapeCollector> collide0;
	CollisionDispatch::sCollideShapeVsShape(&a, &b, scale0, scale1, transform0, transform1, creator0, creator1, collide_settings, collide0, filter);
	out.mCollide[0] = record.mCollide;
	StoreCollideHit(collide0, out.mCollideHits[0]);

	// TransformedShape of B
	TransformedShape ts(LoadR3(in.mPosition), LoadQuat(in.mRotation), &b, BodyID(7), creator1);
	ts.SetShapeScale(Load3(in.mTSScale));
	RMat44 query = LoadRMat44(in.mQueryColumns, in.mQueryTranslation);
	RVec3 base_offset = LoadR3(in.mBaseOffset);

	record.mCollide = CollideRecord();
	ClosestHitCollisionCollector<CollideShapeCollector> collide1;
	ts.CollideShape(&a, scale0, query, collide_settings, base_offset, collide1);
	out.mCollide[1] = record.mCollide;
	StoreCollideHit(collide1, out.mCollideHits[1]);

	// CollisionDispatch::sCastShapeVsShapeWorldSpace / LocalSpace (User1 vs User2 goes through sReversedCastShape)
	ShapeCast cast(&a, scale0, transform0, direction);
	ClosestHitCollisionCollector<CastShapeCollector> cast0;
	CollisionDispatch::sCastShapeVsShapeWorldSpace(cast, cast_settings, &b, scale1, filter, transform1, creator0, creator1, cast0);
	out.mCast[0] = record.mCast;
	StoreCastHit(cast0, out.mCastHits[0]);

	record.mCast = CastRecord();
	ClosestHitCollisionCollector<CastShapeCollector> cast1;
	CollisionDispatch::sCastShapeVsShapeLocalSpace(cast, cast_settings, &b, scale1, filter, transform1, creator0, creator1, cast1);
	out.mCast[1] = record.mCast;
	StoreCastHit(cast1, out.mCastHits[1]);

	record.mCast = CastRecord();
	RShapeCast r_cast(&a, scale0, query, direction);
	ClosestHitCollisionCollector<CastShapeCollector> cast2;
	ts.CastShape(r_cast, cast_settings, base_offset, cast2);
	out.mCast[2] = record.mCast;
	StoreCastHit(cast2, out.mCastHits[2]);

	// TransformedShape::GetTrianglesStart, GetSupportingFace, CollectTransformedShapes
	AABox box(Load3(in.mBox), Load3(in.mBox + 3));
	Shape::GetTrianglesContext context;
	ts.GetTrianglesStart(context, box, base_offset);
	Shape::SupportingFace face;
	ts.GetSupportingFace(creator1.PushID(in.mFaceID[0], in.mFaceID[1]).GetID(), Load3(in.mFaceDirection), base_offset, face);
	out.mFaceCount = uint32(face.size());
	for (uint i = 0; i < min(uint(face.size()), 3u); ++i)
		Store3(face[i], out.mFace[i]);
	TSCollector collect;
	ts.CollectTransformedShapes(box, collect);
	out.mCollectCount = uint32(collect.mHits.size());

	// TransformedShape::CollidePoint through Shape::sCollidePointUsingRayCast
	record.mPointUsingRayCast = true;
	record.mNumRayHits = in.mNumRayHits;
	AllHitCollisionCollector<CollidePointCollector> point_collector;
	ts.CollidePoint(LoadR3(in.mPoint), point_collector);
	Store3(record.mLastPoint, out.mPoint);
	Store3(record.mLastRay.mOrigin, out.mRay[0]);
	Store3(record.mLastRay.mDirection, out.mRay[1]);
	out.mPointHits = uint32(point_collector.mHits.size());
	if (!point_collector.mHits.empty())
	{
		out.mPointIDs[0] = point_collector.mHits[0].mSubShapeID2.GetValue();
		out.mPointIDs[1] = point_collector.mHits[0].mBodyID.GetIndexAndSequenceNumber();
	}
	out.mQueries = record.mQueries;
}

// Shape::SaveWithChildren of a graph of ParityShapes. inChildren[i] holds the indices of the children of shape i (-1:
// none), inMaterials[i] the index of its material in a list of PhysicsMaterialSimple (-1: null). Shapes inRoots are
// saved in order into one stream with shared maps. Returns the size of the stream.
uint32 jolt_save_with_children(int inNumShapes, const float *inHalfExtents, const uint64 *inUserData, const int *inChildren, const int *inMaterials, int inNumMaterials, const uint32 *inMaterialColors, const int *inRoots, int inNumRoots, uint8 *outBytes, uint32 inCapacity)
{
	EnsureFactory();
	Array<RefConst<PhysicsMaterial>> materials;
	for (int m = 0; m < inNumMaterials; ++m)
		materials.push_back(new PhysicsMaterialSimple("Material" + std::to_string(m), Color(inMaterialColors[m])));
	Array<Ref<ParityShape>> shapes;
	for (int s = 0; s < inNumShapes; ++s)
	{
		Ref<ParityShape> shape = new ParityShape(Load3(inHalfExtents + 3 * s), Vec3::sZero(), false);
		shape->SetUserData(inUserData[s]);
		if (inMaterials[s] >= 0)
			shape->mMaterial = materials[inMaterials[s]];
		shapes.push_back(shape);
	}
	for (int s = 0; s < inNumShapes; ++s)
		for (int c = 0; c < 2; ++c)
			if (inChildren[2 * s + c] >= 0)
				shapes[s]->mChildren.push_back(RefConst<Shape>(shapes[inChildren[2 * s + c]].GetPtr()));
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	Shape::ShapeToIDMap shape_map;
	Shape::MaterialToIDMap material_map;
	for (int r = 0; r < inNumRoots; ++r)
		shapes[inRoots[r]]->SaveWithChildren(wrapper, shape_map, material_map);
	for (Ref<ParityShape> &s : shapes)
		s->mChildren.clear();
	return CopyBytes(stream.str(), outBytes, inCapacity);
}

} // extern "C"

// The collectors on a synthetic hit sequence: inKinds selects the collector (0 AllHit, 1 ClosestHit, 2 ClosestHitPerBody,
// 3 AnyHit) and inResultType the collector type (0 CastRay: fractions, 1 CollideShape: penetration depths, 2 CastShape:
// fraction and penetration depth). inBodies[i] is the body of hit i: a new body starts when it changes (ClosestHitPerBody
// gets OnBody / OnBodyEnd). out_early_out holds the early out fraction after every hit, out_hits the stored hits (after
// Sort for AllHit and ClosestHitPerBody). Returns the number of stored hits.
template <class Collector, class MakeResult, class StoreResult>
static int sRunCollector(Collector &ioCollector, int inNumHits, const uint32 *inBodies, MakeResult inMake, StoreResult inStore, float *outEarlyOut)
{
	const Body &body = *reinterpret_cast<const Body *>(sBodyStorage);
	for (int i = 0; i < inNumHits; ++i)
	{
		if (i == 0 || inBodies[i] != inBodies[i - 1])
		{
			if (i > 0)
				ioCollector.OnBodyEnd();
			ioCollector.OnBody(body);
		}
		ioCollector.AddHit(inMake(i));
		outEarlyOut[i] = ioCollector.GetEarlyOutFraction();
	}
	if (inNumHits > 0)
		ioCollector.OnBodyEnd();
	outEarlyOut[inNumHits] = ioCollector.GetEarlyOutFraction();
	return 0;
}

template <class CollectorType, class MakeResult, class StoreResult>
static int sCollect(int inKind, int inNumHits, const uint32 *inBodies, MakeResult inMake, StoreResult inStore, float *outEarlyOut, Hit *outHits)
{
	switch (inKind)
	{
	case 0:
		{
			AllHitCollisionCollector<CollectorType> c;
			sRunCollector(c, inNumHits, inBodies, inMake, inStore, outEarlyOut);
			c.Sort();
			for (size_t i = 0; i < c.mHits.size(); ++i)
				inStore(c.mHits[i], outHits[i]);
			return int(c.mHits.size());
		}
	case 1:
		{
			ClosestHitCollisionCollector<CollectorType> c;
			sRunCollector(c, inNumHits, inBodies, inMake, inStore, outEarlyOut);
			if (!c.HadHit())
				return 0;
			inStore(c.mHit, outHits[0]);
			return 1;
		}
	case 2:
		{
			ClosestHitPerBodyCollisionCollector<CollectorType> c;
			sRunCollector(c, inNumHits, inBodies, inMake, inStore, outEarlyOut);
			c.Sort();
			for (size_t i = 0; i < c.mHits.size(); ++i)
				inStore(c.mHits[i], outHits[i]);
			return int(c.mHits.size());
		}
	default:
		{
			AnyHitCollisionCollector<CollectorType> c;
			for (int i = 0; i < inNumHits && !c.ShouldEarlyOut(); ++i)
				c.AddHit(inMake(i));
			outEarlyOut[0] = c.GetEarlyOutFraction();
			if (!c.HadHit())
				return 0;
			inStore(c.mHit, outHits[0]);
			return 1;
		}
	}
}

extern "C"
{

int jolt_collector(int inKind, int inResultType, int inNumHits, const uint32 *inBodies, const float *inFractions, const float *inDepths, float *outEarlyOut, Hit *outHits)
{
	switch (inResultType)
	{
	case 0:
		return sCollect<CastRayCollector>(inKind, inNumHits, inBodies,
			[&](int i) { RayCastResult r; r.mBodyID = BodyID(inBodies[i]); r.mFraction = inFractions[i]; r.mSubShapeID2 = MakeSubShapeID(uint32(i)); return r; },
			[](const RayCastResult &r, Hit &h) { h.mBodyID = r.mBodyID.GetIndexAndSequenceNumber(); h.mFraction = r.mFraction; h.mPenetrationDepth = float(r.mSubShapeID2.GetValue()); },
			outEarlyOut, outHits);
	case 1:
		return sCollect<CollideShapeCollector>(inKind, inNumHits, inBodies,
			[&](int i) { return CollideShapeResult(Vec3::sZero(), Vec3::sZero(), Vec3::sAxisX(), inDepths[i], MakeSubShapeID(uint32(i)), SubShapeID(), BodyID(inBodies[i])); },
			[](const CollideShapeResult &r, Hit &h) { h.mBodyID = r.mBodyID2.GetIndexAndSequenceNumber(); h.mFraction = float(r.mSubShapeID1.GetValue()); h.mPenetrationDepth = r.mPenetrationDepth; },
			outEarlyOut, outHits);
	default:
		return sCollect<CastShapeCollector>(inKind, inNumHits, inBodies,
			[&](int i) { ShapeCastResult r(inFractions[i], Vec3::sZero(), Vec3(0, 0, inDepths[i]), Vec3::sAxisX(), false, MakeSubShapeID(uint32(i)), SubShapeID(), BodyID(inBodies[i])); return r; },
			[](const ShapeCastResult &r, Hit &h) { h.mBodyID = r.mBodyID2.GetIndexAndSequenceNumber(); h.mFraction = r.mFraction; h.mPenetrationDepth = r.mPenetrationDepth + float(r.mSubShapeID1.GetValue()) * 1000.0f; },
			outEarlyOut, outHits);
	}
}

// The CollisionDispatch tables and ShapeFunctions after registering the classes of RegisterTypesInternal whose bit is
// set in inMask (bit k = k-th sRegister call, Jolt's order), starting from sInit's unsupported functions. The function
// pointers are mapped to names: -1 unsupported, -2 CollisionDispatch::sReversedCollideShape / sReversedCastShape, other
// functions get the index of their first occurrence (row major). The original tables are restored afterwards.
void jolt_dispatch_tables(uint32 inMask, int *outCollide, int *outCast, int *outConstruct, uint32 *outColor)
{
	EnsureFactory();

	CollisionDispatch::CollideShape (&collide)[NumSubShapeTypes][NumSubShapeTypes] = *Get(CollideTableTag());
	CollisionDispatch::CastShape (&cast)[NumSubShapeTypes][NumSubShapeTypes] = *Get(CastTableTag());

	// Save the tables of RegisterTypes()
	static CollisionDispatch::CollideShape saved_collide[NumSubShapeTypes][NumSubShapeTypes];
	static CollisionDispatch::CastShape saved_cast[NumSubShapeTypes][NumSubShapeTypes];
	static ShapeFunctions saved_functions[NumSubShapeTypes];
	memcpy(saved_collide, collide, sizeof(collide));
	memcpy(saved_cast, cast, sizeof(cast));
	for (uint i = 0; i < NumSubShapeTypes; ++i)
		saved_functions[i] = ShapeFunctions::sGet(EShapeSubType(i));

	// sInit with a known unsupported function
	for (uint i = 0; i < NumSubShapeTypes; ++i)
	{
		for (uint j = 0; j < NumSubShapeTypes; ++j)
		{
			collide[i][j] = UnsupportedCollide;
			cast[i][j] = UnsupportedCast;
		}
		ShapeFunctions::sGet(EShapeSubType(i)) = ShapeFunctions();
	}

	// The registrations of RegisterTypesInternal in order
	using RegisterFunction = void (*)();
	static const RegisterFunction order[] = {
		CompoundShape::sRegister, ConvexShape::sRegister, MutableCompoundShape::sRegister, StaticCompoundShape::sRegister,
		TriangleShape::sRegister, PlaneShape::sRegister, SphereShape::sRegister, BoxShape::sRegister, CapsuleShape::sRegister,
		TaperedCapsuleShape::sRegister, CylinderShape::sRegister, TaperedCylinderShape::sRegister, MeshShape::sRegister,
		ConvexHullShape::sRegister, HeightFieldShape::sRegister, SoftBodyShape::sRegister, RotatedTranslatedShape::sRegister,
		OffsetCenterOfMassShape::sRegister, ScaledShape::sRegister, EmptyShape::sRegister };
	for (uint k = 0; k < uint(sizeof(order) / sizeof(order[0])); ++k)
		if (inMask & (1u << k))
			order[k]();

	// The user registrations of the parity build (Zolt's RegisterTypes.user_registrations)
	RegisterParityShapes();

	// Map the functions to names
	Array<CollisionDispatch::CollideShape> collide_names;
	Array<CollisionDispatch::CastShape> cast_names;
	for (uint i = 0; i < NumSubShapeTypes; ++i)
		for (uint j = 0; j < NumSubShapeTypes; ++j)
		{
			CollisionDispatch::CollideShape c = collide[i][j];
			int &collide_name = outCollide[i * NumSubShapeTypes + j];
			if (c == UnsupportedCollide)
				collide_name = -1;
			else if (c == CollisionDispatch::sReversedCollideShape)
				collide_name = -2;
			else
			{
				Array<CollisionDispatch::CollideShape>::iterator it = std::find(collide_names.begin(), collide_names.end(), c);
				if (it == collide_names.end())
				{
					collide_name = int(collide_names.size());
					collide_names.push_back(c);
				}
				else
					collide_name = int(it - collide_names.begin());
			}

			CollisionDispatch::CastShape s = cast[i][j];
			int &cast_name = outCast[i * NumSubShapeTypes + j];
			if (s == UnsupportedCast)
				cast_name = -1;
			else if (s == CollisionDispatch::sReversedCastShape)
				cast_name = -2;
			else
			{
				Array<CollisionDispatch::CastShape>::iterator it = std::find(cast_names.begin(), cast_names.end(), s);
				if (it == cast_names.end())
				{
					cast_name = int(cast_names.size());
					cast_names.push_back(s);
				}
				else
					cast_name = int(it - cast_names.begin());
			}
		}
	for (uint i = 0; i < NumSubShapeTypes; ++i)
	{
		const ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType(i));
		outConstruct[i] = f.mConstruct != nullptr? 1 : 0;
		outColor[i] = f.mColor.GetUInt32();
	}

	// Restore the tables of RegisterTypes()
	memcpy(collide, saved_collide, sizeof(collide));
	memcpy(cast, saved_cast, sizeof(cast));
	for (uint i = 0; i < NumSubShapeTypes; ++i)
		ShapeFunctions::sGet(EShapeSubType(i)) = saved_functions[i];
}

} // extern "C"
