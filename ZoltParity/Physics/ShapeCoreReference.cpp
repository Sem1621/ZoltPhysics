// Reference implementation for the shape core parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library for ScaleHelpers, GetTrianglesContextVertexList / MultiVertexList and their vertex list helpers, ShapeCast /
// RShapeCast / ShapeCastResult, TransformedShape and the default implementations of Shape (with a test shape that
// derives from Shape directly, like ShapeCoreParity.zig), the collision collectors on synthetic hit sequences, the
// binary state of a shape graph (Shape::SaveWithChildren) and the contents of the CollisionDispatch / ShapeFunctions
// tables. ZoltParity/Physics/ShapeCoreParity.zig calls these and checks that Zolt produces the same bits.
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

static void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
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

// A shape that derives from Shape directly (User1), must match ParityShape in ShapeCoreParity.zig: a box around its
// center of mass, an optional uniform scale requirement, children and a material for the binary state of a graph, and
// it records the local ray / point that TransformedShape passes to it
class ParityShape final : public Shape
{
public:
							ParityShape(Vec3Arg inHalfExtent, Vec3Arg inCenterOfMass, bool inUniformScale) : Shape(EShapeType::User1, EShapeSubType::User1), mHalfExtent(inHalfExtent), mCenterOfMass(inCenterOfMass), mUniformScale(inUniformScale) { }

	virtual Vec3			GetCenterOfMass() const override									{ return mCenterOfMass; }
	virtual AABox			GetLocalBounds() const override										{ return AABox(-mHalfExtent, mHalfExtent); }
	virtual uint			GetSubShapeIDBitsRecursive() const override							{ return 0; }
	virtual float			GetInnerRadius() const override										{ return mHalfExtent.ReduceMin(); }
	virtual MassProperties	GetMassProperties() const override									{ return MassProperties(); }
	virtual const PhysicsMaterial *GetMaterial(const SubShapeID &inSubShapeID) const override	{ return mMaterial != nullptr? mMaterial.GetPtr() : PhysicsMaterial::sDefault.GetPtr(); }
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { return inLocalSurfacePosition.NormalizedOr(Vec3::sAxisY()); }
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy) const override { outTotalVolume = outSubmergedVolume = 0.0f; outCenterOfBuoyancy = Vec3::sZero(); }
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override { mLastRay = inRay; ioHit.mFraction = 0.5f; ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID(); return true; }
	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override { mLastRay = inRay; }
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override { mLastPoint = inPoint; }
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override { }
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { }
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { return 0; }
	virtual Stats			GetStats() const override											{ return Stats(sizeof(*this), 0); }
	virtual float			GetVolume() const override											{ return 1.0f; }
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override						{ Vec3 scale = Shape::MakeScaleValid(inScale); return mUniformScale? ScaleHelpers::MakeUniformScale(scale) : scale; }

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
	mutable RayCast			mLastRay { Vec3::sZero(), Vec3::sZero() };
	mutable Vec3			mLastPoint = Vec3::sZero();
};

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
	Store3(shape.mLastRay.mOrigin, outVectors + 9);
	Store3(shape.mLastRay.mDirection, outVectors + 12);
	outVectors[15] = hit.mFraction;
	AnyHitCollisionCollector<CollidePointCollector> point_collector;
	ts.CollidePoint(LoadR3(inPoint), point_collector);
	Store3(shape.mLastPoint, outVectors + 16);

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
