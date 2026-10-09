// Reference implementation for the composite shape parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library for the abstract bases of the decorated and compound shapes: CompoundShape::SubShape (the transform
// and the compressed position / rotation), the construction of a compound (center of mass, inner radius, bounds, the
// sub shapes) and every override of CompoundShape and DecoratedShape, the visitors of CompoundShapeVisitors.h (their
// TestBounds math and the queries that walk them) and CompoundShape::sCastCompoundVsShape. Concrete compounds
// (StaticCompoundShape / MutableCompoundShape) are not ported yet (Wave B), so the tests use test classes that derive
// from the abstract bases: ParityCompoundShape (a compound that walks its sub shapes linearly), ParityDecoratedShape (a
// decorator that passes everything on) and CompositeChild (a box that records the calls it receives). The same classes
// exist in ZoltParity/Physics/CompositeParity.zig, which calls these wrappers and checks that Zolt produces the same bits.
//
// The visitors that go through CollisionDispatch (CollideCompoundVsShape, CollideShapeVsCompound, CastShape and
// sCastCompoundVsShape) collide the test shape of the shape core parity tests (ParityShape, User1, registered in the
// parity build by ShapeCoreUserTypes.zig with functions that record their inputs). That class lives in
// ShapeCoreReference.cpp, so this file has its own copy of it and of its collide / cast functions (MirrorParityShape),
// which it installs in the dispatch table for (User1, User1) during the call and restores afterwards.
//
// The queries that take a ShapeFilter (the collector ray cast, collide point, collect transformed shapes and the four
// dispatching paths) run with ParityShapeFilter (or the default filter): it rejects one sub shape ID and records every
// call it receives in a FilterLog, so the tests check that the visitors pass the filter on to the sub shapes and to
// CollisionDispatch with the right shapes and sub shape IDs.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w), Mat44 as
// 16 floats in column major order, RVec3 as 3 Reals. Booleans are passed as int / uint32 (never bool, see the porting
// guide). Everything except the C ABI is in an anonymous namespace (the shape core reference has classes with the same
// names).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Physics/Collision/CollisionDispatch.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollidePointResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/TransformedShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShapeVisitors.h>
#include <Jolt/Physics/Collision/Shape/DecoratedShape.h>
#include <Jolt/Physics/Collision/Shape/ScaleHelpers.h>

#include <cstring>
#include <sstream>
#include <string>
#include <type_traits>

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
void Store3(Vec3Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
void Store4(Vec4Arg inV, float *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); outV[3] = inV.GetW(); }
void StoreQuat(QuatArg inQ, float *outV)			{ Store4(inQ.GetXYZW(), outV); }
void StoreR3(RVec3Arg inV, Real *outV)				{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
Mat44 LoadMat44(const float *inM)					{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
void StoreMat44(Mat44Arg inM, float *outM)			{ for (int c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
void StoreAABox(const AABox &inBox, float *outBox)	{ Store3(inBox.mMin, outBox); Store3(inBox.mMax, outBox + 3); }
AABox LoadAABox(const float *inBox)					{ return AABox(Load3(inBox), Load3(inBox + 3)); }
Plane LoadPlane(const float *inPlane)				{ return Plane(Load3(inPlane), inPlane[3]); }
SubShapeIDCreator LoadCreator(const uint32 *inCreator) { return inCreator[1] == 0? SubShapeIDCreator() : SubShapeIDCreator().PushID(inCreator[0], inCreator[1]); }

SubShapeID MakeSubShapeID(uint32 inValue)
{
	SubShapeID id;
	id.SetValue(inValue);
	return id;
}

// Access to private members of Jolt (explicit instantiation ignores access checks)
template <typename Tag, typename Tag::type M>
struct Rob
{
	friend typename Tag::type Get(Tag) { return M; }
};

struct CompositeCollideTableTag
{
	using type = CollisionDispatch::CollideShape (*)[NumSubShapeTypes][NumSubShapeTypes];
	friend type Get(CompositeCollideTableTag);
};

struct CompositeCastTableTag
{
	using type = CollisionDispatch::CastShape (*)[NumSubShapeTypes][NumSubShapeTypes];
	friend type Get(CompositeCastTableTag);
};

struct CastCompoundVsShapeTag
{
	using type = CollisionDispatch::CastShape;
	friend type Get(CastCompoundVsShapeTag);
};

template struct Rob<CompositeCollideTableTag, &CollisionDispatch::sCollideShape>;
template struct Rob<CompositeCastTableTag, &CollisionDispatch::sCastShape>;
template struct Rob<CastCompoundVsShapeTag, &CompoundShape::sCastCompoundVsShape>;

// ---------------------------------------------------------------------------------------------------------------------
// The call log of CompositeChild, must match CallRecord / CallLog in CompositeParity.zig

enum ECall : uint32
{
	CallMaterial,
	CallSurfaceNormal,
	CallSupportingFace,
	CallSubmergedVolume,
	CallCastRay,
	CallCastRayCollector,
	CallCollidePoint,
	CallSoftBody,
	CallCollectTransformedShapes,
	CallTransformShape,
};

struct CallRecord
{
	uint32					mKind;
	uint32					mChild;
	uint32					mIDs[4];
	float					mValues[24];
};

constexpr uint32 cMaxCalls = 256;

struct CallLog
{
	uint32					mCount;
	CallRecord				mCalls[cMaxCalls];
};

CallLog sLog;
CallRecord sOverflow;

// Clear the log (the records after the count are compared too, so they must be zero)
void ResetLog()
{
	memset(&sLog, 0, sizeof(sLog));
}

// A new zero initialized record (the overflow record when the log is full)
CallRecord &NewCall(uint32 inKind, uint32 inChild)
{
	CallRecord &r = sLog.mCount < cMaxCalls? sLog.mCalls[sLog.mCount] : sOverflow;
	if (sLog.mCount < cMaxCalls)
		++sLog.mCount;
	memset(&r, 0, sizeof(r));
	r.mKind = inKind;
	r.mChild = inChild;
	return r;
}

void Put3(CallRecord &ioRecord, int inOffset, Vec3Arg inV)		{ Store3(inV, ioRecord.mValues + inOffset); }
void Put4(CallRecord &ioRecord, int inOffset, Vec4Arg inV)		{ Store4(inV, ioRecord.mValues + inOffset); }
void Put16(CallRecord &ioRecord, int inOffset, Mat44Arg inM)	{ StoreMat44(inM, ioRecord.mValues + inOffset); }
void PutCreator(CallRecord &ioRecord, const SubShapeIDCreator &inCreator) { ioRecord.mIDs[0] = inCreator.GetID().GetValue(); ioRecord.mIDs[1] = inCreator.GetNumBitsWritten(); }

// A child shape of the composite parity tests, must match ChildDesc in CompositeParity.zig
struct ChildDesc
{
	float					mHalfExtent[3];
	float					mCenterOfMass[3];
	float					mDensity;
	uint32					mSubShapeIDBits;
	uint32					mUniformScale;
	uint32					mMustBeStatic;
	uint32					mUserData;
};

// A box around its center of mass that records the calls it receives (User3), must match CompositeChild in
// CompositeParity.zig
class CompositeChild final : public Shape
{
public:
							CompositeChild(const ChildDesc &inDesc, uint32 inIndex) : Shape(EShapeType::User1, EShapeSubType::User3), mHalfExtent(Load3(inDesc.mHalfExtent)), mCenterOfMass(Load3(inDesc.mCenterOfMass)), mDensity(inDesc.mDensity), mSubShapeIDBits(inDesc.mSubShapeIDBits), mUniformScale(inDesc.mUniformScale != 0), mMustBeStatic(inDesc.mMustBeStatic != 0), mIndex(inIndex) { SetUserData(inDesc.mUserData); }

	virtual bool			MustBeStatic() const override										{ return mMustBeStatic; }
	virtual Vec3			GetCenterOfMass() const override									{ return mCenterOfMass; }
	virtual AABox			GetLocalBounds() const override										{ return AABox(-mHalfExtent, mHalfExtent); }
	virtual uint			GetSubShapeIDBitsRecursive() const override							{ return mSubShapeIDBits; }
	virtual float			GetInnerRadius() const override										{ return mHalfExtent.ReduceMin(); }
	virtual MassProperties	GetMassProperties() const override									{ MassProperties p; p.SetMassAndInertiaOfSolidBox(2.0f * mHalfExtent, mDensity); return p; }
	virtual Stats			GetStats() const override											{ return Stats(sizeof(*this), 3); }
	virtual float			GetVolume() const override											{ return 8.0f * mHalfExtent.GetX() * mHalfExtent.GetY() * mHalfExtent.GetZ(); }
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { return 0; }
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { }

	virtual const PhysicsMaterial *GetMaterial(const SubShapeID &inSubShapeID) const override
	{
		CallRecord &r = NewCall(CallMaterial, mIndex);
		r.mIDs[0] = inSubShapeID.GetValue();
		return PhysicsMaterial::sDefault;
	}

	// Depends on the sub shape ID, so the tests see which ID a compound / decorator passes on
	virtual uint64			GetSubShapeUserData(const SubShapeID &inSubShapeID) const override		{ return GetUserData() ^ inSubShapeID.GetValue(); }

	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override
	{
		CallRecord &r = NewCall(CallSurfaceNormal, mIndex);
		r.mIDs[0] = inSubShapeID.GetValue();
		Put3(r, 0, inLocalSurfacePosition);
		return inLocalSurfacePosition.NormalizedOr(Vec3::sAxisY());
	}

	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override
	{
		CallRecord &r = NewCall(CallSupportingFace, mIndex);
		r.mIDs[0] = inSubShapeID.GetValue();
		Put3(r, 0, inDirection);
		Put3(r, 3, inScale);
		Put16(r, 6, inCenterOfMassTransform);
		outVertices.push_back(inCenterOfMassTransform * (inScale * mHalfExtent));
		outVertices.push_back(inCenterOfMassTransform * inDirection);
	}

	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy) const override
	{
		CallRecord &r = NewCall(CallSubmergedVolume, mIndex);
		Put16(r, 0, inCenterOfMassTransform);
		Put3(r, 16, inScale);
		Put3(r, 19, inSurface.GetNormal());
		r.mValues[22] = inSurface.GetConstant();
		outTotalVolume = GetVolume() * std::abs(inScale.GetX() * inScale.GetY() * inScale.GetZ());
		Vec3 center = inCenterOfMassTransform.GetTranslation();
		float distance = inSurface.SignedDistance(center);

		// Partially submerged: the fraction of the unscaled height below the surface (as if the box stands upright), the
		// center of buoyancy moves down along the surface normal as the fraction gets smaller
		float fraction = Clamp(0.5f - distance / (2.0f * mHalfExtent.GetY()), 0.0f, 1.0f);
		r.mValues[23] = fraction;
		outSubmergedVolume = fraction * outTotalVolume;
		outCenterOfBuoyancy = center - ((1.0f - fraction) * mHalfExtent.GetY()) * inSurface.GetNormal();
	}

	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override
	{
		CallRecord &r = NewCall(CallCastRay, mIndex);
		PutCreator(r, inSubShapeIDCreator);
		Put3(r, 0, inRay.mOrigin);
		Put3(r, 3, inRay.mDirection);
		r.mValues[6] = ioHit.mFraction;
		float fraction = max(RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent), 0.0f);
		if (fraction < ioHit.mFraction)
		{
			ioHit.mFraction = fraction;
			ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
			return true;
		}
		return false;
	}

	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override
	{
		if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
			return;

		CallRecord &r = NewCall(CallCastRayCollector, mIndex);
		PutCreator(r, inSubShapeIDCreator);
		r.mIDs[2] = inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces? 1 : 0;
		Put3(r, 0, inRay.mOrigin);
		Put3(r, 3, inRay.mDirection);
		r.mValues[6] = ioCollector.GetEarlyOutFraction();

		float min_fraction, max_fraction;
		RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent, min_fraction, max_fraction);
		if (min_fraction > max_fraction || max_fraction < 0.0f)
			return;
		BodyID body_id = TransformedShape::sGetBodyID(ioCollector.GetContext());
		float front = max(min_fraction, 0.0f);
		if (front < ioCollector.GetEarlyOutFraction())
		{
			RayCastResult hit;
			hit.mBodyID = body_id;
			hit.mFraction = front;
			hit.mSubShapeID2 = inSubShapeIDCreator.GetID();
			ioCollector.AddHit(hit);
			if (ioCollector.ShouldEarlyOut())
				return;
		}
		if (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces && max_fraction <= 1.0f && max_fraction < ioCollector.GetEarlyOutFraction())
		{
			RayCastResult hit;
			hit.mBodyID = body_id;
			hit.mFraction = max_fraction;
			hit.mSubShapeID2 = inSubShapeIDCreator.GetID();
			ioCollector.AddHit(hit);
		}
	}

	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override
	{
		CallRecord &r = NewCall(CallCollidePoint, mIndex);
		PutCreator(r, inSubShapeIDCreator);
		Put3(r, 0, inPoint);
		if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
			return;
		if (AABox(-mHalfExtent, mHalfExtent).Contains(inPoint))
		{
			CollidePointResult result;
			result.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
			result.mSubShapeID2 = inSubShapeIDCreator.GetID();
			ioCollector.AddHit(result);
		}
	}

	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override
	{
		CallRecord &r = NewCall(CallSoftBody, mIndex);
		r.mIDs[0] = inNumVertices;
		r.mIDs[1] = uint32(inCollidingShapeIndex);
		Put16(r, 0, inCenterOfMassTransform);
		Put3(r, 16, inScale);
	}

	virtual void			CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override
	{
		CallRecord &r = NewCall(CallCollectTransformedShapes, mIndex);
		PutCreator(r, inSubShapeIDCreator);
		StoreAABox(inBox, r.mValues);
		Put3(r, 6, inPositionCOM);
		Put4(r, 9, inRotation.GetXYZW());
		Put3(r, 13, inScale);
		Shape::CollectTransformedShapes(inBox, inPositionCOM, inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
	}

	virtual void			TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const override
	{
		CallRecord &r = NewCall(CallTransformShape, mIndex);
		Put16(r, 0, inCenterOfMassTransform);
		Shape::TransformShape(inCenterOfMassTransform, ioCollector);
	}

	virtual bool			IsValidScale(Vec3Arg inScale) const override
	{
		return Shape::IsValidScale(inScale) && (!mUniformScale || ScaleHelpers::IsUniformScale(inScale));
	}

	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override
	{
		Vec3 scale = Shape::MakeScaleValid(inScale);
		return mUniformScale? ScaleHelpers::MakeUniformScale(scale) : scale;
	}

	Vec3					mHalfExtent;
	Vec3					mCenterOfMass;
	float					mDensity;
	uint32					mSubShapeIDBits;
	bool					mUniformScale;
	bool					mMustBeStatic;
	uint32					mIndex;
};

// Settings of CompositeChild, must match CompositeChildSettings in CompositeParity.zig
class CompositeChildSettings final : public ShapeSettings
{
public:
							CompositeChildSettings(const ChildDesc &inDesc, uint32 inIndex) : mDesc(inDesc), mIndex(inIndex) { }

	virtual ShapeResult		Create() const override
	{
		if (mCachedResult.IsEmpty())
		{
			if (Vec3::sLessOrEqual(Load3(mDesc.mHalfExtent), Vec3::sZero()).TestAnyXYZTrue())
				mCachedResult.SetError("Invalid half extent");
			else
			{
				Ref<Shape> shape = new CompositeChild(mDesc, mIndex);
				mCachedResult.Set(shape);
			}
		}
		return mCachedResult;
	}

	ChildDesc				mDesc;
	uint32					mIndex;
};

// The index of a CompositeChild / MirrorParityShape (0xffffffff for null)
uint32 ChildIndex(const Shape *inShape);

// ---------------------------------------------------------------------------------------------------------------------
// ParityCompoundShape, must match ParityCompoundShape in CompositeParity.zig: a compound (User5) that walks its sub
// shapes linearly with the visitors of CompoundShapeVisitors.h. For every sub shape it calls TestBounds with the bounds
// of the sub shape in lane 0 (the other lanes repeat it) and visits the sub shape when the result passes (like
// MutableCompoundShape::WalkSubShapes does for blocks of 4). The constructor is StaticCompoundShape's without the tree
// (any number of sub shapes).

class ParityCompoundShapeSettings final : public CompoundShapeSettings
{
public:
	virtual ShapeResult		Create() const override;
};

class ParityCompoundShape final : public CompoundShape
{
public:
							ParityCompoundShape() : CompoundShape(EShapeSubType::User5) { }

							ParityCompoundShape(const ParityCompoundShapeSettings &inSettings, ShapeResult &outResult) :
		CompoundShape(EShapeSubType::User5, inSettings, outResult)
	{
		// Keep track of total mass to calculate center of mass
		float mass = 0.0f;

		mSubShapes.resize(inSettings.mSubShapes.size());
		for (uint i = 0; i < (uint)inSettings.mSubShapes.size(); ++i)
		{
			const CompoundShapeSettings::SubShapeSettings &shape = inSettings.mSubShapes[i];
			SubShape &out_shape = mSubShapes[i];

			// Start constructing the runtime sub shape
			if (!out_shape.FromSettings(shape, outResult))
				return;

			// Calculate mass properties of child
			MassProperties child = out_shape.mShape->GetMassProperties();

			// Accumulate center of mass
			mass += child.mMass;
			mCenterOfMass += out_shape.GetPositionCOM() * child.mMass;
		}

		if (mass > 0.0f)
			mCenterOfMass /= mass;

		// Cache the inner radius as it can take a while to recursively iterate over all sub shapes
		CalculateInnerRadius();

		// Shift all shapes so that the center of mass is now at the origin and calculate bounds
		for (SubShape &shape : mSubShapes)
		{
			shape.SetPositionCOM(shape.GetPositionCOM() - mCenterOfMass);
			mLocalBounds.Encapsulate(sSubShapeBounds(shape));
		}

		// Check if we're not exceeding the amount of sub shape id bits
		if (GetSubShapeIDBitsRecursive() > SubShapeID::MaxBits)
		{
			outResult.SetError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
			return;
		}

		outResult.Set(this);
	}

	// The bounds of a sub shape in the space of the compound
	static AABox			sSubShapeBounds(const SubShape &inSubShape)
	{
		return inSubShape.mShape->GetWorldSpaceBounds(Mat44::sRotationTranslation(inSubShape.GetRotation(), inSubShape.GetPositionCOM()), Vec3::sOne());
	}

	template <class Visitor>
	static bool				sShouldVisit(const Visitor &inVisitor, UVec4Arg inResult)
	{
		return inResult.GetX() != 0;
	}

	template <class Visitor>
	static bool				sShouldVisit(const Visitor &inVisitor, Vec4Arg inResult)
	{
		float fraction;
		if constexpr (std::is_same_v<Visitor, CastRayVisitor>)
			fraction = inVisitor.mHit.mFraction;
		else
			fraction = inVisitor.mCollector.GetEarlyOutFraction();
		return inResult.GetX() < fraction;
	}

	template <class Visitor>
	void					WalkSubShapes(Visitor &ioVisitor) const
	{
		for (uint i = 0; i < (uint)mSubShapes.size(); ++i)
		{
			const SubShape &sub_shape = mSubShapes[i];
			AABox b = sSubShapeBounds(sub_shape);
			auto result = ioVisitor.TestBounds(b.mMin.SplatX(), b.mMin.SplatY(), b.mMin.SplatZ(), b.mMax.SplatX(), b.mMax.SplatY(), b.mMax.SplatZ());
			if (sShouldVisit(ioVisitor, result))
			{
				ioVisitor.VisitShape(sub_shape, i);
				if (ioVisitor.ShouldAbort())
					break;
			}
		}
	}

	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override
	{
		CastRayVisitor visitor(inRay, this, inSubShapeIDCreator, ioHit);
		WalkSubShapes(visitor);
		return visitor.mReturnValue;
	}

	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override
	{
		// Test shape filter
		if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
			return;

		CastRayVisitorCollector visitor(inRay, inRayCastSettings, this, inSubShapeIDCreator, ioCollector, inShapeFilter);
		WalkSubShapes(visitor);
	}

	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override
	{
		// Test shape filter
		if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
			return;

		CollidePointVisitor visitor(inPoint, this, inSubShapeIDCreator, ioCollector, inShapeFilter);
		WalkSubShapes(visitor);
	}

	virtual void			CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override
	{
		// Test shape filter
		if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
			return;

		CollectTransformedShapesVisitor visitor(inBox, this, inPositionCOM, inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
		WalkSubShapes(visitor);
	}

	virtual int				GetIntersectingSubShapes(const AABox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const override
	{
		GetIntersectingSubShapesVisitor<AABox> visitor(inBox, outSubShapeIndices, inMaxSubShapeIndices);
		if (!visitor.ShouldAbort())
			WalkSubShapes(visitor);
		return visitor.GetNumResults();
	}

	virtual int				GetIntersectingSubShapes(const OrientedBox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const override
	{
		GetIntersectingSubShapesVisitor<OrientedBox> visitor(inBox, outSubShapeIndices, inMaxSubShapeIndices);
		if (!visitor.ShouldAbort())
			WalkSubShapes(visitor);
		return visitor.GetNumResults();
	}

	virtual Stats			GetStats() const override											{ return Stats(sizeof(*this) + mSubShapes.size() * sizeof(SubShape), 0); }

	// The collision functions that a compound registers in CollisionDispatch (called directly)
	static void				sCollideCompoundVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
	{
		const ParityCompoundShape *shape1 = static_cast<const ParityCompoundShape *>(inShape1);
		CollideCompoundVsShapeVisitor visitor(shape1, inShape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
		shape1->WalkSubShapes(visitor);
	}

	static void				sCollideShapeVsCompound(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
	{
		const ParityCompoundShape *shape2 = static_cast<const ParityCompoundShape *>(inShape2);
		CollideShapeVsCompoundVisitor visitor(inShape1, shape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
		shape2->WalkSubShapes(visitor);
	}

	static void				sCastShapeVsCompound(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
	{
		const ParityCompoundShape *shape = static_cast<const ParityCompoundShape *>(inShape);
		CastShapeVisitor visitor(inShapeCast, inShapeCastSettings, shape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
		shape->WalkSubShapes(visitor);
	}

	// Access to the protected parts for the tests
	uint					GetSubShapeIDBitsPublic() const										{ return GetSubShapeIDBits(); }
	void					RestoreBinaryStatePublic(StreamIn &inStream)						{ RestoreBinaryState(inStream); }
	const AABox &			GetLocalBoundsMember() const										{ return mLocalBounds; }
	float					GetInnerRadiusMember() const										{ return mInnerRadius; }
	const Vec3 &			GetCenterOfMassMember() const										{ return mCenterOfMass; }

	// The TestBounds of every visitor, see jolt_composite_test_bounds
	struct BoundsInput;
	struct BoundsOutput;
	void					TestVisitorBounds(const BoundsInput &inInput, BoundsOutput &outOutput) const;
};

CompoundShapeSettings::ShapeResult ParityCompoundShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new ParityCompoundShape(*this, mCachedResult);
	return mCachedResult;
}

// ---------------------------------------------------------------------------------------------------------------------
// ParityDecoratedShape, must match ParityDecoratedShape in CompositeParity.zig: a decorator (User6) that implements the
// pure virtual functions of Shape by passing them on to the inner shape

class ParityDecoratedShapeSettings final : public DecoratedShapeSettings
{
public:
	using DecoratedShapeSettings::DecoratedShapeSettings;

	virtual ShapeResult		Create() const override;
};

class ParityDecoratedShape final : public DecoratedShape
{
public:
							ParityDecoratedShape(const ParityDecoratedShapeSettings &inSettings, ShapeResult &outResult) :
		DecoratedShape(EShapeSubType::User6, inSettings, outResult)
	{
		if (outResult.HasError())
			return;

		outResult.Set(this);
	}

	virtual AABox			GetLocalBounds() const override										{ return mInnerShape->GetLocalBounds(); }
	virtual float			GetInnerRadius() const override										{ return mInnerShape->GetInnerRadius(); }
	virtual MassProperties	GetMassProperties() const override									{ return mInnerShape->GetMassProperties(); }
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { return mInnerShape->GetSurfaceNormal(inSubShapeID, inLocalSurfacePosition); }
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy) const override { mInnerShape->GetSubmergedVolume(inCenterOfMassTransform, inScale, inSurface, outTotalVolume, outSubmergedVolume, outCenterOfBuoyancy); }
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override { return mInnerShape->CastRay(inRay, inSubShapeIDCreator, ioHit); }
	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override { mInnerShape->CastRay(inRay, inRayCastSettings, inSubShapeIDCreator, ioCollector, inShapeFilter); }
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override { mInnerShape->CollidePoint(inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter); }
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override { mInnerShape->CollideSoftBodyVertices(inCenterOfMassTransform, inScale, inVertices, inNumVertices, inCollidingShapeIndex); }
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { mInnerShape->GetTrianglesStart(ioContext, inBox, inPositionCOM, inRotation, inScale); }
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { return mInnerShape->GetTrianglesNext(ioContext, inMaxTrianglesRequested, outTriangleVertices, outMaterials); }
	virtual Stats			GetStats() const override											{ return Stats(sizeof(*this), 1); }
	virtual float			GetVolume() const override											{ return mInnerShape->GetVolume(); }
};

DecoratedShapeSettings::ShapeResult ParityDecoratedShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new ParityDecoratedShape(*this, mCachedResult);
	return mCachedResult;
}

// ---------------------------------------------------------------------------------------------------------------------
// MirrorParityShape: a copy of ParityShape of ShapeCoreReference.cpp (Zolt: ShapeCoreUserTypes.zig) with the parts
// that the compound and the visitors use, and of its registered collide / cast functions

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

// The parts of Record (ShapeCoreUserTypes.zig) that the collide / cast functions use
struct MirrorRecord
{
	float					mHitValues[2] = { 0.0f, 0.0f };
	CollideRecord			mCollide { };
	CastRecord				mCast { };
};

void StoreCreator(const SubShapeIDCreator &inCreator, uint32 *outID)	{ outID[0] = inCreator.GetID().GetValue(); outID[1] = inCreator.GetNumBitsWritten(); }

class MirrorParityShape final : public Shape
{
public:
							MirrorParityShape(Vec3Arg inHalfExtent, Vec3Arg inCenterOfMass, MirrorRecord *inRecord, uint32 inIndex) : Shape(EShapeType::User1, EShapeSubType::User1), mHalfExtent(inHalfExtent), mCenterOfMass(inCenterOfMass), mRecord(inRecord), mIndex(inIndex) { }

	virtual Vec3			GetCenterOfMass() const override									{ return mCenterOfMass; }
	virtual AABox			GetLocalBounds() const override										{ return AABox(-mHalfExtent, mHalfExtent); }
	virtual uint			GetSubShapeIDBitsRecursive() const override							{ return 0; }
	virtual float			GetInnerRadius() const override										{ return mHalfExtent.ReduceMin(); }
	virtual MassProperties	GetMassProperties() const override									{ return MassProperties(); }
	virtual const PhysicsMaterial *GetMaterial(const SubShapeID &inSubShapeID) const override	{ return PhysicsMaterial::sDefault; }
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { return inLocalSurfacePosition.NormalizedOr(Vec3::sAxisY()); }
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy) const override { outTotalVolume = outSubmergedVolume = 0.0f; outCenterOfBuoyancy = Vec3::sZero(); }
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override { return false; }
	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override { }
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override { }
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override { }
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { }
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { return 0; }
	virtual Stats			GetStats() const override											{ return Stats(sizeof(*this), 0); }
	virtual float			GetVolume() const override											{ return 1.0f; }

	Vec3					mHalfExtent;
	Vec3					mCenterOfMass;
	MirrorRecord *			mRecord;
	uint32					mIndex;
};

uint32 ChildIndex(const Shape *inShape)
{
	if (inShape == nullptr)
		return 0xffffffff;
	if (inShape->GetSubType() == EShapeSubType::User3)
		return static_cast<const CompositeChild *>(inShape)->mIndex;
	if (inShape->GetSubType() == EShapeSubType::User1)
		return static_cast<const MirrorParityShape *>(inShape)->mIndex;
	return 0xfffffffe;
}

// sCollideParity of ShapeCoreReference.cpp (collideParity in ShapeCoreUserTypes.zig) for MirrorParityShape (User1 only)
void sCollideMirror(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	const MirrorParityShape *p1 = static_cast<const MirrorParityShape *>(inShape1);
	const MirrorParityShape *p2 = static_cast<const MirrorParityShape *>(inShape2);
	MirrorRecord &record = *p1->mRecord;
	CollideRecord &r = record.mCollide;
	++r.mCalls;
	r.mSubTypes[0] = 0;
	r.mSubTypes[1] = 0;
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

// sCastParity of ShapeCoreReference.cpp (castParity in ShapeCoreUserTypes.zig) for MirrorParityShape (User1 only)
void sCastMirror(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	const MirrorParityShape *p1 = static_cast<const MirrorParityShape *>(inShapeCast.mShape);
	const MirrorParityShape *p2 = static_cast<const MirrorParityShape *>(inShape);
	MirrorRecord &record = *p1->mRecord;
	CastRecord &r = record.mCast;
	++r.mCalls;
	r.mSubTypes[0] = 0;
	r.mSubTypes[1] = 0;
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
			result.mShape1Face.push_back(Vec3::sReplicate(-0.0f));
			result.mShape2Face.push_back(inCenterOfMassTransform2 * p2->mCenterOfMass);
			result.mShape2Face.push_back(contact2);
			ioCollector.AddHit(result);
		}
		r.mEarlyOut[i] = ioCollector.GetEarlyOutFraction();
	}
}

// Installs the mirror functions for (User1, User1) in the dispatch table while it lives
class MirrorRegistration
{
public:
							MirrorRegistration()
	{
		CollisionDispatch::CollideShape (&collide)[NumSubShapeTypes][NumSubShapeTypes] = *Get(CompositeCollideTableTag());
		CollisionDispatch::CastShape (&cast)[NumSubShapeTypes][NumSubShapeTypes] = *Get(CompositeCastTableTag());
		mCollide = collide[(int)EShapeSubType::User1][(int)EShapeSubType::User1];
		mCast = cast[(int)EShapeSubType::User1][(int)EShapeSubType::User1];
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::User1, EShapeSubType::User1, sCollideMirror);
		CollisionDispatch::sRegisterCastShape(EShapeSubType::User1, EShapeSubType::User1, sCastMirror);
	}

							~MirrorRegistration()
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::User1, EShapeSubType::User1, mCollide);
		CollisionDispatch::sRegisterCastShape(EShapeSubType::User1, EShapeSubType::User1, mCast);
	}

private:
	CollisionDispatch::CollideShape mCollide;
	CollisionDispatch::CastShape mCast;
};

// ---------------------------------------------------------------------------------------------------------------------
// ParityShapeFilter, must match ParityShapeFilter in CompositeParity.zig

// The filter of a query, must match FilterDesc in CompositeParity.zig
struct FilterDesc
{
	uint32					mEnabled;			///< 0: the default filter ({ }), otherwise ParityShapeFilter
	uint32					mRejectID;			///< The sub shape ID that ParityShapeFilter rejects
};

// A call that ParityShapeFilter received, must match FilterCall in CompositeParity.zig
struct FilterCall
{
	uint32					mShapes[2];			///< ChildIndex of shape 1 / shape 2 (shape 1 is 0xffffffff for the overload without it)
	uint32					mIDs[2];			///< Sub shape ID of shape 1 / shape 2 (0 for shape 1 of the overload without it)
	uint32					mAccepted;
};

constexpr uint32 cMaxFilterCalls = 128;

// The calls that ParityShapeFilter received (the records after the count are zero)
struct FilterLog
{
	uint32					mCount;
	uint32					mRejected;
	FilterCall				mCalls[cMaxFilterCalls];
};

// A filter that rejects one sub shape ID (as shape 2, or as either shape of a pair) and records its calls
class ParityShapeFilter final : public ShapeFilter
{
public:
							ParityShapeFilter(uint32 inRejectID, FilterLog &ioLog) : mRejectID(inRejectID), mLog(ioLog) { }

	virtual bool			ShouldCollide(const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		return Record(0xffffffff, 0, ChildIndex(inShape2), inSubShapeIDOfShape2.GetValue(), inSubShapeIDOfShape2.GetValue() != mRejectID);
	}

	virtual bool			ShouldCollide(const Shape *inShape1, const SubShapeID &inSubShapeIDOfShape1, const Shape *inShape2, const SubShapeID &inSubShapeIDOfShape2) const override
	{
		return Record(ChildIndex(inShape1), inSubShapeIDOfShape1.GetValue(), ChildIndex(inShape2), inSubShapeIDOfShape2.GetValue(), inSubShapeIDOfShape1.GetValue() != mRejectID && inSubShapeIDOfShape2.GetValue() != mRejectID);
	}

private:
	bool					Record(uint32 inShape1, uint32 inID1, uint32 inShape2, uint32 inID2, bool inAccepted) const
	{
		if (mLog.mCount < cMaxFilterCalls)
		{
			FilterCall &c = mLog.mCalls[mLog.mCount];
			c.mShapes[0] = inShape1;
			c.mShapes[1] = inShape2;
			c.mIDs[0] = inID1;
			c.mIDs[1] = inID2;
			c.mAccepted = inAccepted? 1 : 0;
		}
		++mLog.mCount;
		if (!inAccepted)
			++mLog.mRejected;
		return inAccepted;
	}

	uint32					mRejectID;
	FilterLog &				mLog;
};

// ---------------------------------------------------------------------------------------------------------------------
// The C ABI structures, must match the extern structs in CompositeParity.zig

// A sub shape of a compound description
struct SubShapeDesc
{
	float					mPosition[3];
	float					mRotation[4];
	uint32					mChild;
	uint32					mUserData;
	uint32					mFromSettings;
};

// A compound of CompositeChild shapes
struct CompoundDesc
{
	uint32					mNumChildren;
	uint32					mNumSubShapes;
	ChildDesc				mChildren[4];
	SubShapeDesc			mSubShapes[16];
};

// A TransformedShape
struct TSOut
{
	Real					mPositionCOM[3];
	float					mRotation[4];
	float					mScale[3];
	uint32					mBodyID;
	uint32					mSubShapeID;
	uint32					mSubShapeIDBits;
	uint32					mChild;
};

void StoreTS(const TransformedShape &inTS, TSOut &outTS)
{
	StoreR3(inTS.mShapePositionCOM, outTS.mPositionCOM);
	StoreQuat(inTS.mShapeRotation, outTS.mRotation);
	Store3(inTS.GetShapeScale(), outTS.mScale);
	outTS.mBodyID = inTS.mBodyID.GetIndexAndSequenceNumber();
	outTS.mSubShapeID = inTS.mSubShapeIDCreator.GetID().GetValue();
	outTS.mSubShapeIDBits = inTS.mSubShapeIDCreator.GetNumBitsWritten();
	outTS.mChild = ChildIndex(inTS.mShape);
}

struct SubShapeState
{
	float					mPositionCOM[3];
	float					mRotation[3];
	uint32					mUserData;
	uint32					mIsRotationIdentity;
	uint32					mChild;
};

struct CompoundState
{
	uint32					mValid;
	char					mError[128];
	float					mCenterOfMass[3];
	float					mLocalBounds[6];
	float					mInnerRadius;
	float					mMass;
	float					mInertia[16];
	float					mVolume;
	uint32					mMustBeStatic;
	uint32					mSubShapeIDBitsRecursive;
	uint32					mSubShapeIDBits;
	uint32					mNumTriangles;
	uint32					mNumSubShapes;
	SubShapeState			mSubShapes[16];
};

// Queries of jolt_composite_compound
struct CompoundQueries
{
	float					mTransform[16];
	float					mScale[3];
	float					mTestScales[4][3];
	uint32					mIDs[4];
	uint32					mRawIDs[4];
	float					mPosition[3];
	float					mRotation[4];
	float					mLocalPosition[3];
	float					mDirection[3];
	float					mSurface[4];
	float					mRayOrigin[3];
	float					mRayDirection[3];
	uint32					mCreator[2];
	float					mHitFraction;
	uint32					mBackFaces;
	uint32					mCollectorKind;
	float					mPoint[3];
	float					mBox[6];
	float					mOrientedBox[19];
	uint32					mMaxIndices;
	FilterDesc				mFilter;
};

struct CompoundOutput
{
	CompoundState			mState;
	CompoundState			mRestored;
	uint32					mNumBytes;
	uint8					mBytes[1024];
	float					mWorldBounds[6];
	uint32					mScaleValid[4];
	float					mMadeValid[4][3];
	uint32					mRawIDValid[4];
	uint32					mIDValid[4];
	uint32					mIndex[4];
	uint32					mRemainder[4];
	uint32					mLeafChild[4];
	uint32					mLeafRemainder[4];
	uint32					mUserData[4][2];
	uint32					mMaterialIsDefault[4];
	TSOut					mSubTS[4];
	uint32					mSubTSRemainder[4];
	float					mNormal[4][3];
	uint32					mFaceCount[4];
	float					mFace[4][2][3];
	float					mSubmerged[5];
	uint32					mNumTransformed;
	TSOut					mTransformed[16];
	uint32					mRayHit;
	float					mRayFraction;
	uint32					mRayID;
	uint32					mNumRayHits;
	float					mRayHitFractions[32];
	uint32					mRayHitIDs[32];
	uint32					mNumPointHits;
	uint32					mPointHitIDs[16];
	uint32					mNumCollected;
	TSOut					mCollected[16];
	uint32					mNumIntersecting[2];
	uint32					mIntersecting[2][16];
	CallLog					mLog;
	FilterLog				mFilterLog;
};

// Collects the transformed shapes (copies, like Zolt's AllHitCollisionCollector)
class TSCollector : public TransformedShapeCollector
{
public:
	virtual void			AddHit(const TransformedShape &inResult) override					{ mHits.push_back(inResult); }

	Array<TransformedShape>	mHits;
};

void StoreState(const ParityCompoundShape &inShape, CompoundState &outState)
{
	memset(&outState, 0, sizeof(outState));
	outState.mValid = 1;
	Store3(inShape.GetCenterOfMassMember(), outState.mCenterOfMass);
	StoreAABox(inShape.GetLocalBoundsMember(), outState.mLocalBounds);
	outState.mInnerRadius = inShape.GetInnerRadiusMember();
	outState.mNumSubShapes = inShape.GetNumSubShapes();
	outState.mSubShapeIDBits = inShape.GetSubShapeIDBitsPublic();
	for (uint i = 0; i < inShape.GetNumSubShapes() && i < 16; ++i)
	{
		const CompoundShape::SubShape &s = inShape.GetSubShape(i);
		SubShapeState &o = outState.mSubShapes[i];
		memcpy(o.mPositionCOM, &s.mPositionCOM, sizeof(o.mPositionCOM));
		memcpy(o.mRotation, &s.mRotation, sizeof(o.mRotation));
		o.mUserData = s.mUserData;
		o.mIsRotationIdentity = s.mIsRotationIdentity? 1 : 0;
		o.mChild = ChildIndex(s.mShape);
	}
}

// The state that needs the child shapes
void StoreChildState(const ParityCompoundShape &inShape, CompoundState &outState)
{
	MassProperties p = inShape.GetMassProperties();
	outState.mMass = p.mMass;
	StoreMat44(p.mInertia, outState.mInertia);
	outState.mVolume = inShape.GetVolume();
	outState.mMustBeStatic = inShape.MustBeStatic()? 1 : 0;
	outState.mSubShapeIDBitsRecursive = inShape.GetSubShapeIDBitsRecursive();
	Shape::VisitedShapes visited;
	outState.mNumTriangles = inShape.GetStatsRecursive(visited).mNumTriangles;
}

void StoreError(const char *inError, CompoundState &outState)
{
	memset(&outState, 0, sizeof(outState));
	strncpy(outState.mError, inError, sizeof(outState.mError) - 1);
}

// Creates the compound described by inDesc with CompositeChild children (shapes or settings), the children get their
// index in the description
Shape::ShapeResult CreateCompound(const CompoundDesc &inDesc)
{
	Ref<ParityCompoundShapeSettings> settings = new ParityCompoundShapeSettings;
	Array<Ref<Shape>> shapes;
	Array<Ref<ShapeSettings>> child_settings;
	for (uint i = 0; i < inDesc.mNumChildren; ++i)
	{
		shapes.push_back(new CompositeChild(inDesc.mChildren[i], i));
		child_settings.push_back(new CompositeChildSettings(inDesc.mChildren[i], 100 + i));
	}
	for (uint i = 0; i < inDesc.mNumSubShapes; ++i)
	{
		const SubShapeDesc &s = inDesc.mSubShapes[i];
		if (s.mFromSettings != 0)
			settings->AddShape(Load3(s.mPosition), LoadQuat(s.mRotation), child_settings[s.mChild].GetPtr(), s.mUserData);
		else
			settings->AddShape(Load3(s.mPosition), LoadQuat(s.mRotation), shapes[s.mChild].GetPtr(), s.mUserData);
	}
	return settings->Create();
}

// Collector kinds of the queries
enum ECollector : uint32 { AllHits, ClosestHit, AnyHit };

// Runs a query with the collector kind and stores the hits with inStore (called with the hit and its index)
template <class CollectorType, class Query, class Store>
uint32 CollectHits(uint32 inKind, uint32 inMaxHits, const Query &inQuery, const Store &inStore)
{
	uint32 count = 0;
	if (inKind == ClosestHit)
	{
		ClosestHitCollisionCollector<CollectorType> collector;
		inQuery(collector);
		if (collector.HadHit())
			inStore(collector.mHit, count++);
	}
	else if (inKind == AnyHit)
	{
		AnyHitCollisionCollector<CollectorType> collector;
		inQuery(collector);
		if (collector.HadHit())
			inStore(collector.mHit, count++);
	}
	else
	{
		AllHitCollisionCollector<CollectorType> collector;
		inQuery(collector);
		for (const typename CollectorType::ResultType &hit : collector.mHits)
			if (count < inMaxHits)
				inStore(hit, count++);
	}
	return count;
}

// Inputs of jolt_composite_test_bounds
struct ParityCompoundShape::BoundsInput
{
	float					mBounds[24];
	float					mRayOrigin[3];
	float					mRayDirection[3];
	float					mPoint[3];
	float					mCastBounds[6];
	float					mCastDirection[3];
	float					mExtraConvexRadius;
	float					mScale[3];
	float					mCollectBox[6];
	float					mPosition[3];
	float					mRotation[4];
	float					mOtherHalfExtent[3];
	float					mScale2[3];
	float					mTransform1[16];
	float					mTransform2[16];
	float					mMaxSeparation;
	float					mBox[6];
	float					mOrientedBox[19];
	uint32					mNumSubShapes;
};

// Results of jolt_composite_test_bounds
struct ParityCompoundShape::BoundsOutput
{
	float					mRay[4];
	float					mRayCollector[4];
	uint32					mPoint[4];
	float					mCast[4];
	uint32					mCollect[4];
	uint32					mCompoundVsShape[4];
	uint32					mShapeVsCompound[4];
	uint32					mAABox[4];
	uint32					mOrientedBox[4];
	uint32					mSubShapeBits;
	float					mBoxCenter[3];
	float					mBoxExtent[3];
	float					mBoundsOf2[6];
	float					mBoundsOf1[6];
	float					mLocalBox[19];
};

OrientedBox LoadOrientedBox(const float *inBox) { return OrientedBox(LoadMat44(inBox), Load3(inBox + 16)); }

void ParityCompoundShape::TestVisitorBounds(const BoundsInput &inInput, BoundsOutput &outOutput) const
{
	const float *b = inInput.mBounds;
	Vec4 min_x = Load4(b), min_y = Load4(b + 4), min_z = Load4(b + 8), max_x = Load4(b + 12), max_y = Load4(b + 16), max_z = Load4(b + 20);

	ShapeFilter filter;
	RayCast ray(Load3(inInput.mRayOrigin), Load3(inInput.mRayDirection));
	RayCastResult hit;
	CastRayVisitor ray_visitor(ray, this, SubShapeIDCreator(), hit);
	Store4(ray_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z), outOutput.mRay);
	outOutput.mSubShapeBits = ray_visitor.mSubShapeBits;

	AllHitCollisionCollector<CastRayCollector> ray_collector;
	RayCastSettings ray_settings;
	CastRayVisitorCollector ray_collector_visitor(ray, ray_settings, this, SubShapeIDCreator(), ray_collector, filter);
	Store4(ray_collector_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z), outOutput.mRayCollector);

	AllHitCollisionCollector<CollidePointCollector> point_collector;
	CollidePointVisitor point_visitor(Load3(inInput.mPoint), this, SubShapeIDCreator(), point_collector, filter);
	point_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z).StoreInt4(outOutput.mPoint);

	CompositeChild other(ChildDesc { { inInput.mOtherHalfExtent[0], inInput.mOtherHalfExtent[1], inInput.mOtherHalfExtent[2] }, { 0, 0, 0 }, 1.0f, 0, 0, 0, 0 }, 99);
	other.SetEmbedded();
	ShapeCast shape_cast(&other, Vec3::sOne(), Mat44::sIdentity(), Load3(inInput.mCastDirection), LoadAABox(inInput.mCastBounds));
	ShapeCastSettings cast_settings;
	cast_settings.mExtraConvexRadius = inInput.mExtraConvexRadius;
	AllHitCollisionCollector<CastShapeCollector> cast_collector;
	CastShapeVisitor cast_visitor(shape_cast, cast_settings, this, Load3(inInput.mScale), filter, Mat44::sIdentity(), SubShapeIDCreator(), SubShapeIDCreator(), cast_collector);
	Store4(cast_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z), outOutput.mCast);
	Store3(cast_visitor.mBoxCenter, outOutput.mBoxCenter);
	Store3(cast_visitor.mBoxExtent, outOutput.mBoxExtent);

	TSCollector ts_collector;
	CollectTransformedShapesVisitor collect_visitor(LoadAABox(inInput.mCollectBox), this, Load3(inInput.mPosition), LoadQuat(inInput.mRotation), Load3(inInput.mScale), SubShapeIDCreator(), ts_collector, filter);
	collect_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z).StoreInt4(outOutput.mCollect);
	StoreMat44(collect_visitor.mLocalBox.mOrientation, outOutput.mLocalBox);
	Store3(collect_visitor.mLocalBox.mHalfExtents, outOutput.mLocalBox + 16);

	AllHitCollisionCollector<CollideShapeCollector> collide_collector;
	CollideShapeSettings collide_settings;
	collide_settings.mMaxSeparationDistance = inInput.mMaxSeparation;
	Mat44 transform1 = LoadMat44(inInput.mTransform1);
	Mat44 transform2 = LoadMat44(inInput.mTransform2);
	CollideCompoundVsShapeVisitor vs_shape(this, &other, Load3(inInput.mScale), Load3(inInput.mScale2), transform1, transform2, SubShapeIDCreator(), SubShapeIDCreator(), collide_settings, collide_collector, filter);
	vs_shape.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z).StoreInt4(outOutput.mCompoundVsShape);
	StoreAABox(vs_shape.mBoundsOf2InSpaceOf1, outOutput.mBoundsOf2);
	CollideShapeVsCompoundVisitor vs_compound(&other, this, Load3(inInput.mScale2), Load3(inInput.mScale), transform2, transform1, SubShapeIDCreator(), SubShapeIDCreator(), collide_settings, collide_collector, filter);
	vs_compound.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z).StoreInt4(outOutput.mShapeVsCompound);
	StoreAABox(vs_compound.mBoundsOf1InSpaceOf2, outOutput.mBoundsOf1);

	uint indices[4];
	GetIntersectingSubShapesVisitor<AABox> aabox_visitor(LoadAABox(inInput.mBox), indices, 4);
	aabox_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z).StoreInt4(outOutput.mAABox);
	GetIntersectingSubShapesVisitor<OrientedBox> obox_visitor(LoadOrientedBox(inInput.mOrientedBox), indices, 4);
	obox_visitor.TestBounds(min_x, min_y, min_z, max_x, max_y, max_z).StoreInt4(outOutput.mOrientedBox);
}

// Inputs of jolt_composite_sub_shape
struct SubShapeInput
{
	float					mChildCenterOfMass[3];
	float					mPosition[3];
	float					mRotation[4];
	float					mCompoundCenterOfMass[3];
	float					mScales[4][3];
	float					mRotation2[4];
	float					mPosition2[3];
};

// Results of jolt_composite_sub_shape
struct SubShapeOutput
{
	float					mStoredPosition[3];
	float					mStoredRotation[3];
	uint32					mIsRotationIdentity;
	float					mRotation[4];
	float					mPositionCOM[3];
	uint32					mValid[4];
	float					mTransformScale[4][3];
	float					mLocalTransform[4][16];
	float					mRoundTripRotation[4];
	float					mRoundTripPosition[3];
	float					mRoundTripStored[6];
};

// A ParityShape description (half extent, center of mass)
struct ParityShapeDesc
{
	float					mHalfExtent[3];
	float					mCenterOfMass[3];
};

// Inputs of jolt_composite_visitors
struct VisitorInput
{
	uint32					mNumSubShapes;
	ParityShapeDesc			mChildren[16];
	float					mPositions[16][3];
	float					mRotations[16][4];
	ParityShapeDesc			mOther;
	float					mHitValues[2];
	float					mTransform1[16];
	float					mTransform2[16];
	float					mScale1[3];
	float					mScale2[3];
	uint32					mCreators[2][2];
	float					mMaxSeparation;
	uint32					mCollectorKind;
	float					mCastStart[16];
	float					mCastDirection[3];
	FilterDesc				mFilter;
};

// A collide / cast hit
struct HitOut
{
	float					mContact1[3];
	float					mContact2[3];
	float					mAxis[3];
	float					mValue;
	uint32					mBackFace;
	uint32					mIDs[2];
	uint32					mBodyID;
	uint32					mFaceCounts[2];
	float					mFaces[2][2][3];
};

// Results of jolt_composite_visitors: 0: compound vs shape, 1: shape vs compound, 2: cast shape vs compound, 3: cast
// compound vs shape (CompoundShape::sCastCompoundVsShape)
struct VisitorOutput
{
	uint32					mNumHits[4];
	HitOut					mHits[4][16];
	CollideRecord			mCollide[2];
	CastRecord				mCast[2];
	FilterLog				mFilterLog;
};

void StoreFaces(const CollideShapeResult &inResult, HitOut &outHit)
{
	outHit.mFaceCounts[0] = inResult.mShape1Face.size();
	outHit.mFaceCounts[1] = inResult.mShape2Face.size();
	for (uint i = 0; i < inResult.mShape1Face.size() && i < 2; ++i)
		Store3(inResult.mShape1Face[i], outHit.mFaces[0][i]);
	for (uint i = 0; i < inResult.mShape2Face.size() && i < 2; ++i)
		Store3(inResult.mShape2Face[i], outHit.mFaces[1][i]);
}

void StoreCollideHit(const CollideShapeResult &inResult, HitOut &outHit)
{
	memset(&outHit, 0, sizeof(outHit));
	Store3(inResult.mContactPointOn1, outHit.mContact1);
	Store3(inResult.mContactPointOn2, outHit.mContact2);
	Store3(inResult.mPenetrationAxis, outHit.mAxis);
	outHit.mValue = inResult.mPenetrationDepth;
	outHit.mIDs[0] = inResult.mSubShapeID1.GetValue();
	outHit.mIDs[1] = inResult.mSubShapeID2.GetValue();
	outHit.mBodyID = inResult.mBodyID2.GetIndexAndSequenceNumber();
	StoreFaces(inResult, outHit);
}

void StoreCastHit(const ShapeCastResult &inResult, HitOut &outHit)
{
	StoreCollideHit(inResult, outHit);
	outHit.mValue = inResult.mFraction;
	outHit.mBackFace = inResult.mIsBackFaceHit? 1 : 0;
}

// Inputs of jolt_composite_decorated
struct DecoratedInput
{
	ChildDesc				mChild;
	uint32					mMode;
	uint32					mID;
	float					mDirection[3];
	float					mScale[3];
	float					mTransform[16];
	float					mTestScales[4][3];
	uint64					mUserData;
};

// Results of jolt_composite_decorated
struct DecoratedOutput
{
	uint32					mValid;
	char					mError[128];
	uint32					mMustBeStatic;
	float					mCenterOfMass[3];
	uint32					mSubShapeIDBits;
	uint32					mLeafChild;
	uint32					mLeafRemainder;
	uint32					mMaterialIsDefault;
	uint32					mUserData[2];
	uint32					mFaceCount;
	float					mFace[2][3];
	uint32					mScaleValid[4];
	float					mMadeValid[4][3];
	uint32					mNumTriangles;
	uint32					mNumSubShapes;
	uint32					mSubShapeChild;
	uint32					mNumBytes;
	uint8					mBytes[64];
	uint32					mNumCalls;
	CallRecord				mCalls[4];
};

} // namespace

extern "C"
{

// CompoundShape::SubShape: SetTransform (the compressed position and rotation, the identity flag), GetRotation,
// GetPositionCOM, IsValidScale / TransformScale / GetLocalTransformNoScale for 4 scales (the local transform only when
// the scale is valid, Jolt asserts otherwise) and the round trip of SetRotation / SetPositionCOM
void jolt_composite_sub_shape(const SubShapeInput *inInput, SubShapeOutput *outOutput)
{
	EnsureFactory();
	memset(outOutput, 0, sizeof(*outOutput));

	ChildDesc desc { { 1, 1, 1 }, { inInput->mChildCenterOfMass[0], inInput->mChildCenterOfMass[1], inInput->mChildCenterOfMass[2] }, 1.0f, 0, 0, 0, 0 };
	Ref<CompositeChild> child = new CompositeChild(desc, 0);
	CompoundShape::SubShape s;
	s.mShape = child;
	s.SetTransform(Load3(inInput->mPosition), LoadQuat(inInput->mRotation), Load3(inInput->mCompoundCenterOfMass));
	memcpy(outOutput->mStoredPosition, &s.mPositionCOM, sizeof(outOutput->mStoredPosition));
	memcpy(outOutput->mStoredRotation, &s.mRotation, sizeof(outOutput->mStoredRotation));
	outOutput->mIsRotationIdentity = s.mIsRotationIdentity? 1 : 0;
	StoreQuat(s.GetRotation(), outOutput->mRotation);
	Store3(s.GetPositionCOM(), outOutput->mPositionCOM);
	for (int i = 0; i < 4; ++i)
	{
		Vec3 scale = Load3(inInput->mScales[i]);
		bool valid = s.IsValidScale(scale);
		outOutput->mValid[i] = valid? 1 : 0;
		Store3(s.TransformScale(scale), outOutput->mTransformScale[i]);
		if (valid)
			StoreMat44(s.GetLocalTransformNoScale(scale), outOutput->mLocalTransform[i]);
	}
	s.SetRotation(LoadQuat(inInput->mRotation2));
	s.SetPositionCOM(Load3(inInput->mPosition2));
	StoreQuat(s.GetRotation(), outOutput->mRoundTripRotation);
	Store3(s.GetPositionCOM(), outOutput->mRoundTripPosition);
	memcpy(outOutput->mRoundTripStored, &s.mPositionCOM, 3 * sizeof(float));
	memcpy(outOutput->mRoundTripStored + 3, &s.mRotation, 3 * sizeof(float));
}

// A compound of CompositeChild shapes: the construction (state), the binary state and its restore, and the overrides of
// CompoundShape and the queries through the visitors with the inputs of inQueries
void jolt_composite_compound(const CompoundDesc *inDesc, const CompoundQueries *inQueries, CompoundOutput *outOutput)
{
	EnsureFactory();
	memset(outOutput, 0, sizeof(*outOutput));
	ResetLog();

	Shape::ShapeResult result = CreateCompound(*inDesc);
	if (result.HasError())
	{
		StoreError(result.GetError().c_str(), outOutput->mState);
		return;
	}
	const ParityCompoundShape &compound = *static_cast<const ParityCompoundShape *>(result.Get().GetPtr());
	StoreState(compound, outOutput->mState);
	StoreChildState(compound, outOutput->mState);

	// Binary state and restore (the sub shapes are restored from SaveSubShapeState)
	{
		std::stringstream data;
		StreamOutWrapper stream_out(data);
		compound.SaveBinaryState(stream_out);
		std::string bytes = data.str();
		outOutput->mNumBytes = uint32(bytes.size());
		if (bytes.size() <= sizeof(outOutput->mBytes))
			memcpy(outOutput->mBytes, bytes.data(), bytes.size());

		Ref<ParityCompoundShape> restored = new ParityCompoundShape;
		std::stringstream in_data(bytes.substr(1)); // Shape::sRestoreFromBinaryState reads the sub type
		StreamInWrapper stream_in(in_data);
		restored->RestoreBinaryStatePublic(stream_in);
		ShapeList sub_shapes;
		compound.SaveSubShapeState(sub_shapes);
		restored->RestoreSubShapeState(sub_shapes.data(), (uint)sub_shapes.size());
		StoreState(*restored, outOutput->mRestored);
		StoreChildState(*restored, outOutput->mRestored);
	}

	const CompoundQueries &q = *inQueries;
	Mat44 transform = LoadMat44(q.mTransform);
	Vec3 scale = Load3(q.mScale);
	Vec3 position = Load3(q.mPosition);
	Quat rotation = LoadQuat(q.mRotation);

	StoreAABox(compound.GetWorldSpaceBounds(transform, scale), outOutput->mWorldBounds);
	for (int i = 0; i < 4; ++i)
	{
		Vec3 s = Load3(q.mTestScales[i]);
		outOutput->mScaleValid[i] = compound.IsValidScale(s)? 1 : 0;
		Store3(compound.MakeScaleValid(s), outOutput->mMadeValid[i]);
	}

	for (int i = 0; i < 4; ++i)
		outOutput->mRawIDValid[i] = compound.IsSubShapeIDValid(MakeSubShapeID(q.mRawIDs[i]))? 1 : 0;

	for (int i = 0; i < 4 && compound.GetNumSubShapes() > 0; ++i)
	{
		SubShapeID id = MakeSubShapeID(q.mIDs[i]);
		outOutput->mIDValid[i] = compound.IsSubShapeIDValid(id)? 1 : 0;
		SubShapeID remainder;
		outOutput->mIndex[i] = compound.GetSubShapeIndexFromID(id, remainder);
		outOutput->mRemainder[i] = remainder.GetValue();
		SubShapeID leaf_remainder;
		outOutput->mLeafChild[i] = ChildIndex(compound.GetLeafShape(id, leaf_remainder));
		outOutput->mLeafRemainder[i] = leaf_remainder.GetValue();
		uint64 user_data = compound.GetSubShapeUserData(id);
		outOutput->mUserData[i][0] = uint32(user_data);
		outOutput->mUserData[i][1] = uint32(user_data >> 32);
		outOutput->mMaterialIsDefault[i] = compound.GetMaterial(id) == PhysicsMaterial::sDefault? 1 : 0;
		SubShapeID ts_remainder;
		TransformedShape ts = compound.GetSubShapeTransformedShape(id, position, rotation, scale, ts_remainder);
		StoreTS(ts, outOutput->mSubTS[i]);
		outOutput->mSubTSRemainder[i] = ts_remainder.GetValue();
		Store3(compound.GetSurfaceNormal(id, Load3(q.mLocalPosition)), outOutput->mNormal[i]);
		Shape::SupportingFace face;
		compound.GetSupportingFace(id, Load3(q.mDirection), scale, transform, face);
		outOutput->mFaceCount[i] = face.size();
		for (uint j = 0; j < face.size() && j < 2; ++j)
			Store3(face[j], outOutput->mFace[i][j]);
	}

	float total, submerged;
	Vec3 center;
	compound.GetSubmergedVolume(transform, scale, LoadPlane(q.mSurface), total, submerged, center);
	outOutput->mSubmerged[0] = total;
	outOutput->mSubmerged[1] = submerged;
	Store3(center, outOutput->mSubmerged + 2);

	CollideSoftBodyVertexIterator vertices;
	compound.CollideSoftBodyVertices(transform, scale, vertices, 7, 3);

	{
		TSCollector collector;
		compound.TransformShape(transform, collector);
		outOutput->mNumTransformed = min<uint32>(16, (uint32)collector.mHits.size());
		for (uint32 i = 0; i < outOutput->mNumTransformed; ++i)
			StoreTS(collector.mHits[i], outOutput->mTransformed[i]);
	}

	// Queries through the visitors (the ones that take a filter with ParityShapeFilter or the default filter)
	ShapeFilter default_filter;
	ParityShapeFilter parity_filter(q.mFilter.mRejectID, outOutput->mFilterLog);
	const ShapeFilter &filter = q.mFilter.mEnabled != 0? static_cast<const ShapeFilter &>(parity_filter) : default_filter;
	SubShapeIDCreator creator = LoadCreator(q.mCreator);
	RayCast ray(Load3(q.mRayOrigin), Load3(q.mRayDirection));
	{
		RayCastResult hit;
		hit.mFraction = q.mHitFraction;
		outOutput->mRayHit = compound.CastRay(ray, creator, hit)? 1 : 0;
		outOutput->mRayFraction = hit.mFraction;
		outOutput->mRayID = hit.mSubShapeID2.GetValue();
	}
	{
		RayCastSettings settings;
		if (q.mBackFaces != 0)
			settings.SetBackFaceMode(EBackFaceMode::CollideWithBackFaces);
		outOutput->mNumRayHits = CollectHits<CastRayCollector>(q.mCollectorKind, 32,
			[&](CastRayCollector &ioCollector) { compound.CastRay(ray, settings, creator, ioCollector, filter); },
			[&](const RayCastResult &inHit, uint32 inIndex) { outOutput->mRayHitFractions[inIndex] = inHit.mFraction; outOutput->mRayHitIDs[inIndex] = inHit.mSubShapeID2.GetValue(); });
	}
	outOutput->mNumPointHits = CollectHits<CollidePointCollector>(q.mCollectorKind, 16,
		[&](CollidePointCollector &ioCollector) { compound.CollidePoint(Load3(q.mPoint), creator, ioCollector, filter); },
		[&](const CollidePointResult &inHit, uint32 inIndex) { outOutput->mPointHitIDs[inIndex] = inHit.mSubShapeID2.GetValue(); });
	{
		TSCollector collector;
		compound.CollectTransformedShapes(LoadAABox(q.mBox), position, rotation, scale, creator, collector, filter);
		outOutput->mNumCollected = min<uint32>(16, (uint32)collector.mHits.size());
		for (uint32 i = 0; i < outOutput->mNumCollected; ++i)
			StoreTS(collector.mHits[i], outOutput->mCollected[i]);
	}
	outOutput->mNumIntersecting[0] = compound.GetIntersectingSubShapes(LoadAABox(q.mBox), outOutput->mIntersecting[0], int(q.mMaxIndices));
	outOutput->mNumIntersecting[1] = compound.GetIntersectingSubShapes(LoadOrientedBox(q.mOrientedBox), outOutput->mIntersecting[1], int(q.mMaxIndices));

	outOutput->mLog = sLog;
}

// The error text of a compound that cannot be created (empty when it can)
void jolt_composite_compound_error(const CompoundDesc *inDesc, char *outError)
{
	EnsureFactory();
	Shape::ShapeResult result = CreateCompound(*inDesc);
	memset(outError, 0, 128);
	if (result.HasError())
		strncpy(outError, result.GetError().c_str(), 127);
}

// The TestBounds of every visitor on 4 boxes, and the members that their constructors compute
void jolt_composite_test_bounds(const ParityCompoundShape::BoundsInput *inInput, ParityCompoundShape::BoundsOutput *outOutput)
{
	EnsureFactory();
	memset(outOutput, 0, sizeof(*outOutput));

	CompoundDesc desc;
	memset(&desc, 0, sizeof(desc));
	desc.mNumChildren = 1;
	desc.mChildren[0] = ChildDesc { { 1, 1, 1 }, { 0, 0, 0 }, 1.0f, 0, 0, 0, 0 };
	desc.mNumSubShapes = inInput->mNumSubShapes;
	for (uint i = 0; i < desc.mNumSubShapes; ++i)
		desc.mSubShapes[i].mRotation[3] = 1.0f;
	Shape::ShapeResult result = CreateCompound(desc);
	static_cast<const ParityCompoundShape *>(result.Get().GetPtr())->TestVisitorBounds(*inInput, *outOutput);
}

// The visitors that go through CollisionDispatch, with MirrorParityShape children (the parity shape of the shape core
// tests): collide compound vs shape, shape vs compound, cast shape vs compound, cast compound vs shape
void jolt_composite_visitors(const VisitorInput *inInput, VisitorOutput *outOutput)
{
	EnsureFactory();
	memset(outOutput, 0, sizeof(*outOutput));
	MirrorRegistration registration;

	MirrorRecord record;
	record.mHitValues[0] = inInput->mHitValues[0];
	record.mHitValues[1] = inInput->mHitValues[1];

	Ref<ParityCompoundShapeSettings> settings = new ParityCompoundShapeSettings;
	for (uint i = 0; i < inInput->mNumSubShapes; ++i)
	{
		const ParityShapeDesc &c = inInput->mChildren[i];
		settings->AddShape(Load3(inInput->mPositions[i]), LoadQuat(inInput->mRotations[i]), new MirrorParityShape(Load3(c.mHalfExtent), Load3(c.mCenterOfMass), &record, i));
	}
	Shape::ShapeResult result = settings->Create();
	const Shape *compound = result.Get().GetPtr();
	Ref<MirrorParityShape> other = new MirrorParityShape(Load3(inInput->mOther.mHalfExtent), Load3(inInput->mOther.mCenterOfMass), &record, 99);

	Mat44 transform1 = LoadMat44(inInput->mTransform1);
	Mat44 transform2 = LoadMat44(inInput->mTransform2);
	Vec3 scale1 = Load3(inInput->mScale1);
	Vec3 scale2 = Load3(inInput->mScale2);
	SubShapeIDCreator creator1 = LoadCreator(inInput->mCreators[0]);
	SubShapeIDCreator creator2 = LoadCreator(inInput->mCreators[1]);
	CollideShapeSettings collide_settings;
	collide_settings.mMaxSeparationDistance = inInput->mMaxSeparation;
	ShapeCastSettings cast_settings;
	ShapeFilter default_filter;
	ParityShapeFilter parity_filter(inInput->mFilter.mRejectID, outOutput->mFilterLog);
	const ShapeFilter &filter = inInput->mFilter.mEnabled != 0? static_cast<const ShapeFilter &>(parity_filter) : default_filter;

	outOutput->mNumHits[0] = CollectHits<CollideShapeCollector>(inInput->mCollectorKind, 16,
		[&](CollideShapeCollector &ioCollector) { ParityCompoundShape::sCollideCompoundVsShape(compound, other, scale1, scale2, transform1, transform2, creator1, creator2, collide_settings, ioCollector, filter); },
		[&](const CollideShapeResult &inHit, uint32 inIndex) { StoreCollideHit(inHit, outOutput->mHits[0][inIndex]); });
	outOutput->mCollide[0] = record.mCollide;
	record.mCollide = CollideRecord { };

	outOutput->mNumHits[1] = CollectHits<CollideShapeCollector>(inInput->mCollectorKind, 16,
		[&](CollideShapeCollector &ioCollector) { ParityCompoundShape::sCollideShapeVsCompound(other, compound, scale2, scale1, transform2, transform1, creator2, creator1, collide_settings, ioCollector, filter); },
		[&](const CollideShapeResult &inHit, uint32 inIndex) { StoreCollideHit(inHit, outOutput->mHits[1][inIndex]); });
	outOutput->mCollide[1] = record.mCollide;

	ShapeCast other_cast(other, scale2, LoadMat44(inInput->mCastStart), Load3(inInput->mCastDirection));
	outOutput->mNumHits[2] = CollectHits<CastShapeCollector>(inInput->mCollectorKind, 16,
		[&](CastShapeCollector &ioCollector) { ParityCompoundShape::sCastShapeVsCompound(other_cast, cast_settings, compound, scale1, filter, transform1, creator2, creator1, ioCollector); },
		[&](const ShapeCastResult &inHit, uint32 inIndex) { StoreCastHit(inHit, outOutput->mHits[2][inIndex]); });
	outOutput->mCast[0] = record.mCast;
	record.mCast = CastRecord { };

	ShapeCast compound_cast(compound, scale1, LoadMat44(inInput->mCastStart), Load3(inInput->mCastDirection));
	CollisionDispatch::CastShape cast_compound_vs_shape = Get(CastCompoundVsShapeTag());
	outOutput->mNumHits[3] = CollectHits<CastShapeCollector>(inInput->mCollectorKind, 16,
		[&](CastShapeCollector &ioCollector) { cast_compound_vs_shape(compound_cast, cast_settings, other, scale2, filter, transform2, creator1, creator2, ioCollector); },
		[&](const ShapeCastResult &inHit, uint32 inIndex) { StoreCastHit(inHit, outOutput->mHits[3][inIndex]); });
	outOutput->mCast[1] = record.mCast;
}

// A ParityDecoratedShape around a CompositeChild: inMode 0: the child shape, 1: the child settings, 2: no inner shape,
// 3: child settings that fail. The overrides of DecoratedShape, the binary state and the sub shape state.
void jolt_composite_decorated(const DecoratedInput *inInput, DecoratedOutput *outOutput)
{
	EnsureFactory();
	memset(outOutput, 0, sizeof(*outOutput));
	ResetLog();

	Ref<CompositeChild> child = new CompositeChild(inInput->mChild, 0);
	ChildDesc bad_desc = inInput->mChild;
	bad_desc.mHalfExtent[1] = -1.0f;
	Ref<CompositeChildSettings> child_settings = new CompositeChildSettings(inInput->mMode == 3? bad_desc : inInput->mChild, 1);
	Ref<ParityDecoratedShapeSettings> settings;
	switch (inInput->mMode)
	{
	case 0:		settings = new ParityDecoratedShapeSettings(static_cast<const Shape *>(child.GetPtr())); break;
	case 1:
	case 3:		settings = new ParityDecoratedShapeSettings(static_cast<const ShapeSettings *>(child_settings.GetPtr())); break;
	default:	settings = new ParityDecoratedShapeSettings(); break;
	}
	settings->mUserData = inInput->mUserData;
	Shape::ShapeResult result = settings->Create();
	if (result.HasError())
	{
		strncpy(outOutput->mError, result.GetError().c_str(), sizeof(outOutput->mError) - 1);
		return;
	}
	outOutput->mValid = 1;
	const Shape &shape = *result.Get();

	outOutput->mMustBeStatic = shape.MustBeStatic()? 1 : 0;
	Store3(shape.GetCenterOfMass(), outOutput->mCenterOfMass);
	outOutput->mSubShapeIDBits = shape.GetSubShapeIDBitsRecursive();
	SubShapeID id = MakeSubShapeID(inInput->mID);
	SubShapeID leaf_remainder;
	outOutput->mLeafChild = ChildIndex(shape.GetLeafShape(id, leaf_remainder));
	outOutput->mLeafRemainder = leaf_remainder.GetValue();
	outOutput->mMaterialIsDefault = shape.GetMaterial(id) == PhysicsMaterial::sDefault? 1 : 0;
	uint64 user_data = shape.GetSubShapeUserData(id);
	outOutput->mUserData[0] = uint32(user_data);
	outOutput->mUserData[1] = uint32(user_data >> 32);
	Shape::SupportingFace face;
	shape.GetSupportingFace(id, Load3(inInput->mDirection), Load3(inInput->mScale), LoadMat44(inInput->mTransform), face);
	outOutput->mFaceCount = face.size();
	for (uint j = 0; j < face.size() && j < 2; ++j)
		Store3(face[j], outOutput->mFace[j]);
	for (int i = 0; i < 4; ++i)
	{
		Vec3 s = Load3(inInput->mTestScales[i]);
		outOutput->mScaleValid[i] = shape.IsValidScale(s)? 1 : 0;
		Store3(shape.MakeScaleValid(s), outOutput->mMadeValid[i]);
	}
	Shape::VisitedShapes visited;
	outOutput->mNumTriangles = shape.GetStatsRecursive(visited).mNumTriangles;
	ShapeList sub_shapes;
	shape.SaveSubShapeState(sub_shapes);
	outOutput->mNumSubShapes = (uint32)sub_shapes.size();
	outOutput->mSubShapeChild = sub_shapes.empty()? 0xffffffff : ChildIndex(sub_shapes[0]);

	std::stringstream data;
	StreamOutWrapper stream_out(data);
	shape.SaveBinaryState(stream_out);
	std::string bytes = data.str();
	outOutput->mNumBytes = uint32(bytes.size());
	if (bytes.size() <= sizeof(outOutput->mBytes))
		memcpy(outOutput->mBytes, bytes.data(), bytes.size());

	outOutput->mNumCalls = sLog.mCount;
	for (uint32 i = 0; i < sLog.mCount && i < 4; ++i)
		outOutput->mCalls[i] = sLog.mCalls[i];
}

} // extern "C"
