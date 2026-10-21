// The C++ side of the draw call log of the debug renderer parity (Docs/Zolt/RemainingFeaturesArchitecture.md section 4,
// "Draw call log"). ZoltParity/Renderer/ParityRenderer.zig is the Zig side: both write the same bytes for the same
// calls, so the logs of Jolt and Zolt compare byte for byte.
//
// - ParityRenderer is a JPH::DebugRenderer that logs every virtual call with all its arguments, including what a .jor
//   file drops: the world space bounds and the LOD scale of DrawGeometry and the contents of every batch.
// - SimpleParityRenderer is a JPH::DebugRendererSimple that logs DrawLine and DrawText3D. Simple's own DrawTriangle,
//   batches and DrawGeometry (with its LOD selection) produce the lines.
// - The op stream: the Zig side encodes the calls to make (random helper calls, ...) as bytes, RunParityOps executes
//   them on any renderer. Every group adds its ops in its own range (see ParityRenderer.zig). The shape ops of D2 draw
//   the shapes of a ParityShapeContext, the ops of D3 the constraints and bodies of a ParityPhysicsContext.
// - Reference files of other groups (shapes, constraints, ...) include this header to draw into a ParityRenderer.
//
// The log format (little endian, no padding; ParityRenderer.zig decodes it for the mismatch report):
//   DrawLine           u8 1, RVec3 from, RVec3 to, Color
//   DrawTriangle       u8 2, RVec3 v1, RVec3 v2, RVec3 v3, Color, i32 cast shadow
//   CreateBatch        u8 3, u32 batch id, u32 count, count * 108 bytes (DebugRenderer::Triangle)
//   CreateBatchIndexed u8 4, u32 batch id, u32 vertex count, vertex count * 36 bytes, u32 index count, index count * u32
//   DrawGeometry       u8 5, RMat44 model, AABox world space bounds, f32 LOD scale, Color, u32 geometry id, AABox bounds,
//                      u32 LOD count, LOD count * (f32 distance, u32 batch id), i32 cull mode, i32 cast shadow, i32 draw mode
//   DrawText3D         u8 6, RVec3 position, u32 length, length bytes, Color, f32 height
//   Marker             u8 7, u32 value (written by the harness, e.g. before each frame of a playback)
// RVec3 is 3 Reals (float or double), RMat44 is 4 Vec4 columns (single precision) or 3 Vec4 columns and 3 doubles
// (double precision), AABox is 6 floats, Color is its uint32. Batch ids count from 1 in creation order (0 is a null
// batch), geometry ids count from 1 in the order of their first DrawGeometry.

#pragma once

#include <Jolt/Jolt.h>

#ifdef JPH_DEBUG_RENDERER

#include <Jolt/Renderer/DebugRenderer.h>
#include <Jolt/Renderer/DebugRendererRecorder.h>
#include <Jolt/Renderer/DebugRendererSimple.h>
#include <Jolt/Geometry/OrientedBox.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Constraints/ConstraintManager.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <ZoltParity/ParityHooks.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <functional>
#include <unordered_map>
#include <vector>

namespace ZoltParity
{

using namespace JPH;

/// The op codes of the log
enum class ELogOp : uint8
{
	DrawLine = 1,
	DrawTriangle = 2,
	CreateBatch = 3,
	CreateBatchIndexed = 4,
	DrawGeometry = 5,
	DrawText3D = 6,
	Marker = 7,
};

/// The bytes of a log
class ParityLog
{
public:
	template <class T>
	void							WriteRaw(const T &inValue)
	{
		const uint8 *p = reinterpret_cast<const uint8 *>(&inValue);
		mBytes.insert(mBytes.end(), p, p + sizeof(T));
	}

	void							WriteBytes(const void *inData, size_t inSize)
	{
		if (inSize == 0)
			return;
		const uint8 *p = static_cast<const uint8 *>(inData);
		mBytes.insert(mBytes.end(), p, p + inSize);
	}

	void							WriteOp(ELogOp inOp)						{ WriteRaw(uint8(inOp)); }
	void							WriteRVec3(RVec3Arg inV)					{ WriteRaw(Real(inV.GetX())); WriteRaw(Real(inV.GetY())); WriteRaw(Real(inV.GetZ())); }
	void							WriteVec3(Vec3Arg inV)						{ WriteRaw(inV.GetX()); WriteRaw(inV.GetY()); WriteRaw(inV.GetZ()); }
	void							WriteVec4(Vec4Arg inV)						{ WriteRaw(inV.GetX()); WriteRaw(inV.GetY()); WriteRaw(inV.GetZ()); WriteRaw(inV.GetW()); }
	void							WriteColor(ColorArg inColor)				{ WriteRaw(inColor.GetUInt32()); }
	void							WriteAABox(const AABox &inBox)				{ WriteVec3(inBox.mMin); WriteVec3(inBox.mMax); }

	void							WriteRMat44(RMat44Arg inM)
	{
	#ifdef JPH_DOUBLE_PRECISION
		for (uint i = 0; i < 3; ++i)
			WriteVec4(inM.GetColumn4(i));
		WriteRVec3(inM.GetTranslation());
	#else
		for (uint i = 0; i < 4; ++i)
			WriteVec4(inM.GetColumn4(i));
	#endif
	}

	void							WriteMarker(uint32 inValue)					{ WriteOp(ELogOp::Marker); WriteRaw(inValue); }

	std::vector<uint8>				mBytes;
};

/// The log records of the virtual functions that every parity renderer shares
inline void ParityLogLine(ParityLog &ioLog, RVec3Arg inFrom, RVec3Arg inTo, ColorArg inColor)
{
	ioLog.WriteOp(ELogOp::DrawLine);
	ioLog.WriteRVec3(inFrom);
	ioLog.WriteRVec3(inTo);
	ioLog.WriteColor(inColor);
}

inline void ParityLogText(ParityLog &ioLog, RVec3Arg inPosition, const string_view &inString, ColorArg inColor, float inHeight)
{
	ioLog.WriteOp(ELogOp::DrawText3D);
	ioLog.WriteRVec3(inPosition);
	ioLog.WriteRaw(uint32(inString.size()));
	ioLog.WriteBytes(inString.data(), inString.size());
	ioLog.WriteColor(inColor);
	ioLog.WriteRaw(inHeight);
}

/// A DebugRenderer that logs every call (see the format above)
class ParityRenderer final : public DebugRenderer
{
public:
	JPH_OVERRIDE_NEW_DELETE

	/// Constructor: Initialize logs the predefined batches (the construction log)
									ParityRenderer()							{ Initialize(); }

	virtual void					DrawLine(RVec3Arg inFrom, RVec3Arg inTo, ColorArg inColor) override
	{
		ParityLogLine(mLog, inFrom, inTo, inColor);
	}

	virtual void					DrawTriangle(RVec3Arg inV1, RVec3Arg inV2, RVec3Arg inV3, ColorArg inColor, ECastShadow inCastShadow) override
	{
		mLog.WriteOp(ELogOp::DrawTriangle);
		mLog.WriteRVec3(inV1);
		mLog.WriteRVec3(inV2);
		mLog.WriteRVec3(inV3);
		mLog.WriteColor(inColor);
		mLog.WriteRaw(int32(inCastShadow));
	}

	virtual Batch					CreateTriangleBatch(const Triangle *inTriangles, int inTriangleCount) override
	{
		uint32 id = mNextBatchID++;
		mLog.WriteOp(ELogOp::CreateBatch);
		mLog.WriteRaw(id);
		mLog.WriteRaw(uint32(inTriangleCount));
		mLog.WriteBytes(inTriangles, size_t(inTriangleCount) * sizeof(Triangle));
		return new BatchImpl(id);
	}

	virtual Batch					CreateTriangleBatch(const Vertex *inVertices, int inVertexCount, const uint32 *inIndices, int inIndexCount) override
	{
		uint32 id = mNextBatchID++;
		mLog.WriteOp(ELogOp::CreateBatchIndexed);
		mLog.WriteRaw(id);
		mLog.WriteRaw(uint32(inVertexCount));
		mLog.WriteBytes(inVertices, size_t(inVertexCount) * sizeof(Vertex));
		mLog.WriteRaw(uint32(inIndexCount));
		mLog.WriteBytes(inIndices, size_t(inIndexCount) * sizeof(uint32));
		return new BatchImpl(id);
	}

	virtual void					DrawGeometry(RMat44Arg inModelMatrix, const AABox &inWorldSpaceBounds, float inLODScaleSq, ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode, ECastShadow inCastShadow, EDrawMode inDrawMode) override
	{
		// Geometry ids in the order of the first draw (the map keeps a reference so an address is never reused)
		uint32 geometry_id;
		auto it = mGeometryIDs.find(inGeometry.GetPtr());
		if (it == mGeometryIDs.end())
		{
			geometry_id = mNextGeometryID++;
			mGeometryIDs.insert({ inGeometry.GetPtr(), geometry_id });
			mGeometries.push_back(inGeometry);
		}
		else
			geometry_id = it->second;

		mLog.WriteOp(ELogOp::DrawGeometry);
		mLog.WriteRMat44(inModelMatrix);
		mLog.WriteAABox(inWorldSpaceBounds);
		mLog.WriteRaw(inLODScaleSq);
		mLog.WriteColor(inModelColor);
		mLog.WriteRaw(geometry_id);
		mLog.WriteAABox(inGeometry->mBounds);
		mLog.WriteRaw(uint32(inGeometry->mLODs.size()));
		for (const LOD &lod : inGeometry->mLODs)
		{
			mLog.WriteRaw(lod.mDistance);
			const BatchImpl *batch = static_cast<const BatchImpl *>(lod.mTriangleBatch.GetPtr());
			mLog.WriteRaw(batch != nullptr? batch->mID : uint32(0));
		}
		mLog.WriteRaw(int32(inCullMode));
		mLog.WriteRaw(int32(inCastShadow));
		mLog.WriteRaw(int32(inDrawMode));
	}

	virtual void					DrawText3D(RVec3Arg inPosition, const string_view &inString, ColorArg inColor, float inHeight) override
	{
		ParityLogText(mLog, inPosition, inString, inColor, inHeight);
	}

	/// The log of all calls so far
	ParityLog						mLog;

	/// The id of the next batch
	uint32							mNextBatchID = 1;

private:
	/// A batch that knows its id
	class BatchImpl : public RefTargetVirtual
	{
	public:
		JPH_OVERRIDE_NEW_DELETE

		explicit					BatchImpl(uint32 inID)						: mID(inID) { }

		virtual void				AddRef() override							{ ++mRefCount; }
		virtual void				Release() override							{ if (--mRefCount == 0) delete this; }

		atomic<uint32>				mRefCount = 0;
		uint32						mID;
	};

	uint32							mNextGeometryID = 1;
	std::unordered_map<const Geometry *, uint32> mGeometryIDs;
	Array<GeometryRef>				mGeometries;
};

/// A DebugRendererSimple that logs its lines and texts (Simple's DrawTriangle, batches and DrawGeometry make the lines)
class SimpleParityRenderer final : public DebugRendererSimple
{
public:
	JPH_OVERRIDE_NEW_DELETE

	virtual void					DrawLine(RVec3Arg inFrom, RVec3Arg inTo, ColorArg inColor) override
	{
		ParityLogLine(mLog, inFrom, inTo, inColor);
	}

	virtual void					DrawText3D(RVec3Arg inPosition, const string_view &inString, ColorArg inColor, float inHeight) override
	{
		ParityLogText(mLog, inPosition, inString, inColor, inHeight);
	}

	/// The log of all calls so far
	ParityLog						mLog;
};

/// Reads the op stream of ParityRenderer.zig
class ParityOpReader
{
public:
									ParityOpReader(const uint8 *inData, size_t inSize) : mData(inData), mSize(inSize) { }

	bool							IsAtEnd() const								{ return mPos >= mSize; }

	template <class T>
	T								Read()
	{
		T value;
		memcpy(&value, mData + mPos, sizeof(T));
		mPos += sizeof(T);
		return value;
	}

	const uint8 *					ReadBytes(size_t inSize)
	{
		const uint8 *p = mData + mPos;
		mPos += inSize;
		return p;
	}

	Vec3							ReadVec3()									{ float x = Read<float>(); float y = Read<float>(); float z = Read<float>(); return Vec3(x, y, z); }
	RVec3							ReadRVec3()									{ Real x = Read<Real>(); Real y = Read<Real>(); Real z = Read<Real>(); return RVec3(x, y, z); }
	Color							ReadColor()									{ return Color(Read<uint32>()); }
	AABox							ReadAABox()									{ Vec3 min = ReadVec3(); Vec3 max = ReadVec3(); return AABox(min, max); }

	/// An affine matrix: 3 columns of 3 floats and the translation
	RMat44							ReadRMat44()
	{
		Vec3 c0 = ReadVec3();
		Vec3 c1 = ReadVec3();
		Vec3 c2 = ReadVec3();
		RVec3 t = ReadRVec3();
		return RMat44(Vec4(c0, 0), Vec4(c1, 0), Vec4(c2, 0), t);
	}

	string_view						ReadString()
	{
		uint32 length = Read<uint32>();
		return string_view(reinterpret_cast<const char *>(ReadBytes(length)), length);
	}

	/// A batch: u8 kind (0 a triangle list, 1 indexed), then the data
	DebugRenderer::Batch			ReadAndCreateBatch(DebugRenderer &ioRenderer)
	{
		uint8 kind = Read<uint8>();
		if (kind == 0)
		{
			uint32 count = Read<uint32>();
			Array<DebugRenderer::Triangle> triangles(count);
			if (count > 0)
				memcpy(triangles.data(), ReadBytes(count * sizeof(DebugRenderer::Triangle)), count * sizeof(DebugRenderer::Triangle));
			return ioRenderer.CreateTriangleBatch(triangles);
		}
		else
		{
			uint32 vertex_count = Read<uint32>();
			Array<DebugRenderer::Vertex> vertices(vertex_count);
			if (vertex_count > 0)
				memcpy(vertices.data(), ReadBytes(vertex_count * sizeof(DebugRenderer::Vertex)), vertex_count * sizeof(DebugRenderer::Vertex));
			uint32 index_count = Read<uint32>();
			Array<uint32> indices(index_count);
			if (index_count > 0)
				memcpy(indices.data(), ReadBytes(index_count * sizeof(uint32)), index_count * sizeof(uint32));
			return ioRenderer.CreateTriangleBatch(vertices, indices);
		}
	}

private:
	const uint8 *					mData;
	size_t							mSize;
	size_t							mPos = 0;
};

/// The number of geometry slots of the op stream
static constexpr uint32 cParityNumGeometrySlots = 16;

/// The support function of the convex batch ops (D1): a box with rounded edges. ParityRenderer.zig has the same one.
struct ParitySupport
{
	Vec3							GetSupport(Vec3Arg inDirection) const
	{
		Vec3 box(inDirection.GetX() < 0.0f? -mHalfExtent.GetX() : mHalfExtent.GetX(),
				 inDirection.GetY() < 0.0f? -mHalfExtent.GetY() : mHalfExtent.GetY(),
				 inDirection.GetZ() < 0.0f? -mHalfExtent.GetZ() : mHalfExtent.GetZ());
		return box + mRadius * inDirection;
	}

	Vec3							mHalfExtent;
	float							mRadius;
};

/// Read a support function of the op stream: Vec3 half extent, f32 radius
inline ParitySupport ParityReadSupport(ParityOpReader &ioReader)
{
	ParitySupport support;
	support.mHalfExtent = ioReader.ReadVec3();
	support.mRadius = ioReader.Read<float>();
	return support;
}

/// The shapes of the shape ops (D2) and their results (ShapeContext in ParityRenderer.zig)
struct ParityShapeContext
{
	const Shape *const *			mShapes;
	uint32							mNumShapes;
	std::vector<uint8>				mResults;							///< The results of the ops that compute numbers

	template <class T>
	void							WriteResult(const T &inValue)
	{
		const uint8 *p = reinterpret_cast<const uint8 *>(&inValue);
		mResults.insert(mResults.end(), p, p + sizeof(T));
	}
};

/// The shape of a shape op
inline const Shape *ParityShape(const ParityShapeContext *inShapes, uint32 inIndex)
{
	if (inShapes == nullptr || inIndex >= inShapes->mNumShapes)
	{
		Trace("RunParityOps: a shape op without its shape");
		abort();
	}
	return inShapes->mShapes[inIndex];
}

/// The bits of the op set_shape_draw_flags (ShapeDrawFlags in ParityRenderer.zig)
inline void ParitySetShapeDrawFlags(uint8 inFlags)
{
	Shape::sDrawSubmergedVolumes = (inFlags & 1) != 0;
	ConvexHullShape::sDrawFaceOutlines = (inFlags & 2) != 0;
	MeshShape::sDrawTriangleGroups = (inFlags & 4) != 0;
	MeshShape::sDrawTriangleOutlines = (inFlags & 8) != 0;
	HeightFieldShape::sDrawTriangleOutlines = (inFlags & 16) != 0;
}

/// The constraints and bodies of the D3 ops and their results (PhysicsContext in ParityRenderer.zig)
struct ParityPhysicsContext
{
	Array<Constraint *>				mConstraints;						///< The constraints that the constraint ops index
	std::function<void(uint32)>		mStepConstraint;					///< The op step_constraint: the next frame of the scenario of a constraint
	ConstraintManager *				mConstraintManager = nullptr;		///< The manager of the op draw_constraint_manager
	BodyManager *					mBodyManager = nullptr;				///< The manager of the ops draw_bodies and activate_body
	Array<Body *>					mBodies;							///< The bodies that the body ops index
	std::vector<uint8>				mResults;							///< The results of the ops that compute numbers

	template <class T>
	void							WriteResult(const T &inValue)
	{
		const uint8 *p = reinterpret_cast<const uint8 *>(&inValue);
		mResults.insert(mResults.end(), p, p + sizeof(T));
	}
};

/// The context of a D3 op
inline ParityPhysicsContext &ParityPhysics(ParityPhysicsContext *inPhysics)
{
	if (inPhysics == nullptr)
	{
		Trace("RunParityOps: a constraint or body op without its context");
		abort();
	}
	return *inPhysics;
}

/// The BodyDrawFilter of the op draw_bodies: skips the bodies whose index % 3 is 1
class ParityBodyDrawFilter final : public BodyDrawFilter
{
public:
	virtual bool					ShouldDraw(const Body &inBody) const override	{ return inBody.GetID().GetIndex() % 3 != 1; }
};

/// The DrawSettings of the op draw_bodies: the bits are the booleans in declaration order (BodyDrawBits in
/// ParityRenderer.zig)
inline BodyManager::DrawSettings ParityBodyDrawSettings(uint32 inBits, uint8 inShapeColor, uint8 inSoftBodyConstraintColor)
{
	BodyManager::DrawSettings s;
	bool *flags[] = {
		&s.mDrawGetSupportFunction, &s.mDrawSupportDirection, &s.mDrawGetSupportingFace, &s.mDrawShape, &s.mDrawShapeWireframe,
		&s.mDrawBoundingBox, &s.mDrawCenterOfMassTransform, &s.mDrawWorldTransform, &s.mDrawVelocity, &s.mDrawMassAndInertia,
		&s.mDrawSleepStats, &s.mDrawSoftBodyVertices, &s.mDrawSoftBodyVertexVelocities, &s.mDrawSoftBodyEdgeConstraints,
		&s.mDrawSoftBodyBendConstraints, &s.mDrawSoftBodyVolumeConstraints, &s.mDrawSoftBodySkinConstraints,
		&s.mDrawSoftBodyLRAConstraints, &s.mDrawSoftBodyRods, &s.mDrawSoftBodyRodStates,
		&s.mDrawSoftBodyRodBendTwistConstraints, &s.mDrawSoftBodyPredictedBounds };
	for (uint i = 0; i < std::size(flags); ++i)
		*flags[i] = (inBits & (1u << i)) != 0;
	s.mDrawShapeColor = BodyManager::EShapeColor(inShapeColor);
	s.mDrawSoftBodyConstraintColor = ESoftBodyConstraintColor(inSoftBodyConstraintColor);
	return s;
}

/// Executes the op stream of ParityRenderer.zig on inRenderer (D0's ops 1 to 15: the virtual functions and the helpers of
/// DebugRenderer that D0 found ported; D1's ops 16 to 36: the other helpers, NextFrame and the batch helpers; D2's ops 64
/// to 69: the shapes of ioShapes; D3's ops 96 to 103: the constraints and bodies of ioPhysics). ioSimple and ioRecorder
/// are the renderer as a DebugRendererSimple / DebugRendererRecorder (or null) for the ops that only they have. The shape
/// ops change the static draw flags of the shapes: the caller resets them (ParitySetShapeDrawFlags(0)).
inline void RunParityOps(DebugRenderer &ioRenderer, const uint8 *inOps, size_t inSize, DebugRendererSimple *ioSimple, DebugRendererRecorder *ioRecorder, ParityShapeContext *ioShapes = nullptr, ParityPhysicsContext *ioPhysics = nullptr)
{
	ParityOpReader r(inOps, inSize);
	DebugRenderer::GeometryRef slots[cParityNumGeometrySlots];

	while (!r.IsAtEnd())
	{
		uint8 op = r.Read<uint8>();
		switch (op)
		{
		case 1: // DrawLine
			{
				RVec3 from = r.ReadRVec3();
				RVec3 to = r.ReadRVec3();
				Color color = r.ReadColor();
				ioRenderer.DrawLine(from, to, color);
				break;
			}

		case 2: // DrawTriangle
			{
				RVec3 v1 = r.ReadRVec3();
				RVec3 v2 = r.ReadRVec3();
				RVec3 v3 = r.ReadRVec3();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				ioRenderer.DrawTriangle(v1, v2, v3, color, DebugRenderer::ECastShadow(cast_shadow));
				break;
			}

		case 3: // DrawText3D
			{
				RVec3 position = r.ReadRVec3();
				string_view string = r.ReadString();
				Color color = r.ReadColor();
				float height = r.Read<float>();
				ioRenderer.DrawText3D(position, string, color, height);
				break;
			}

		case 4: // DrawWireBox(const AABox &, ColorArg)
			{
				AABox box = r.ReadAABox();
				Color color = r.ReadColor();
				ioRenderer.DrawWireBox(box, color);
				break;
			}

		case 5: // DrawWireBox(RMat44Arg, const AABox &, ColorArg)
			{
				RMat44 matrix = r.ReadRMat44();
				AABox box = r.ReadAABox();
				Color color = r.ReadColor();
				ioRenderer.DrawWireBox(matrix, box, color);
				break;
			}

		case 6: // DrawMarker
			{
				RVec3 position = r.ReadRVec3();
				Color color = r.ReadColor();
				float size = r.Read<float>();
				ioRenderer.DrawMarker(position, color, size);
				break;
			}

		case 7: // DrawArrow
			{
				RVec3 from = r.ReadRVec3();
				RVec3 to = r.ReadRVec3();
				Color color = r.ReadColor();
				float size = r.Read<float>();
				ioRenderer.DrawArrow(from, to, color, size);
				break;
			}

		case 8: // DrawBox(const AABox &, ...)
			{
				AABox box = r.ReadAABox();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawBox(box, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 9: // DrawBox(RMat44Arg, const AABox &, ...)
			{
				RMat44 matrix = r.ReadRMat44();
				AABox box = r.ReadAABox();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawBox(matrix, box, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 10: // Create a geometry in a slot: bounds, then the LODs (distance and batch)
			{
				uint32 slot = r.Read<uint32>();
				AABox bounds = r.ReadAABox();
				DebugRenderer::GeometryRef geometry = new DebugRenderer::Geometry(bounds);
				uint32 num_lods = r.Read<uint32>();
				for (uint32 l = 0; l < num_lods; ++l)
				{
					DebugRenderer::LOD lod;
					lod.mDistance = r.Read<float>();
					lod.mTriangleBatch = r.ReadAndCreateBatch(ioRenderer);
					geometry->mLODs.push_back(lod);
				}
				slots[slot] = geometry;
				break;
			}

		case 11: // The virtual DrawGeometry (explicit world space bounds and LOD scale)
			{
				RMat44 matrix = r.ReadRMat44();
				AABox world_space_bounds = r.ReadAABox();
				float lod_scale_sq = r.Read<float>();
				Color color = r.ReadColor();
				uint32 slot = r.Read<uint32>();
				int32 cull_mode = r.Read<int32>();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawGeometry(matrix, world_space_bounds, lod_scale_sq, color, slots[slot], DebugRenderer::ECullMode(cull_mode), DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 12: // The non virtual DrawGeometry (bounds and LOD scale from the model matrix)
			{
				RMat44 matrix = r.ReadRMat44();
				Color color = r.ReadColor();
				uint32 slot = r.Read<uint32>();
				int32 cull_mode = r.Read<int32>();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawGeometry(matrix, color, slots[slot], DebugRenderer::ECullMode(cull_mode), DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 13: // End of a frame (the recorder writes it, other renderers ignore it)
			if (ioRecorder != nullptr)
				ioRecorder->EndFrame();
			break;

		case 14: // SetCameraPos (DebugRendererSimple only)
			{
				RVec3 position = r.ReadRVec3();
				if (ioSimple != nullptr)
					ioSimple->SetCameraPos(position);
				break;
			}

		case 15: // Create a batch and release it again
			{
				DebugRenderer::Batch batch = r.ReadAndCreateBatch(ioRenderer);
				break;
			}

		// D1: the other helpers of DebugRenderer

		case 16: // DrawWireBox(const OrientedBox &, ColorArg): 3 columns and the translation (Vec3), half extents
			{
				Vec3 c0 = r.ReadVec3();
				Vec3 c1 = r.ReadVec3();
				Vec3 c2 = r.ReadVec3();
				Vec3 t = r.ReadVec3();
				Vec3 half_extents = r.ReadVec3();
				Color color = r.ReadColor();
				ioRenderer.DrawWireBox(OrientedBox(Mat44(Vec4(c0, 0), Vec4(c1, 0), Vec4(c2, 0), Vec4(t, 1)), half_extents), color);
				break;
			}

		case 17: // DrawCoordinateSystem
			{
				RMat44 transform = r.ReadRMat44();
				float size = r.Read<float>();
				ioRenderer.DrawCoordinateSystem(transform, size);
				break;
			}

		case 18: // DrawPlane
			{
				RVec3 point = r.ReadRVec3();
				Vec3 normal = r.ReadVec3();
				Color color = r.ReadColor();
				float size = r.Read<float>();
				ioRenderer.DrawPlane(point, normal, color, size);
				break;
			}

		case 19: // DrawWireTriangle
			{
				RVec3 v1 = r.ReadRVec3();
				RVec3 v2 = r.ReadRVec3();
				RVec3 v3 = r.ReadRVec3();
				Color color = r.ReadColor();
				ioRenderer.DrawWireTriangle(v1, v2, v3, color);
				break;
			}

		case 20: // DrawWirePolygon: matrix, u32 count, count * Vec3, color, arrow size
			{
				RMat44 transform = r.ReadRMat44();
				uint32 count = r.Read<uint32>();
				Array<Vec3> vertices;
				for (uint32 i = 0; i < count; ++i)
					vertices.push_back(r.ReadVec3());
				Color color = r.ReadColor();
				float arrow_size = r.Read<float>();
				ioRenderer.DrawWirePolygon(transform, vertices, color, arrow_size);
				break;
			}

		case 21: // DrawWireSphere
			{
				RVec3 center = r.ReadRVec3();
				float radius = r.Read<float>();
				Color color = r.ReadColor();
				int32 level = r.Read<int32>();
				ioRenderer.DrawWireSphere(center, radius, color, level);
				break;
			}

		case 22: // DrawWireUnitSphere
			{
				RMat44 matrix = r.ReadRMat44();
				Color color = r.ReadColor();
				int32 level = r.Read<int32>();
				ioRenderer.DrawWireUnitSphere(matrix, color, level);
				break;
			}

		case 23: // DrawSphere
			{
				RVec3 center = r.ReadRVec3();
				float radius = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawSphere(center, radius, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 24: // DrawUnitSphere
			{
				RMat44 matrix = r.ReadRMat44();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawUnitSphere(matrix, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 25: // DrawCapsule
		case 26: // DrawCylinder
			{
				RMat44 matrix = r.ReadRMat44();
				float half_height = r.Read<float>();
				float radius = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				if (op == 25)
					ioRenderer.DrawCapsule(matrix, half_height, radius, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				else
					ioRenderer.DrawCylinder(matrix, half_height, radius, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 27: // DrawOpenCone
			{
				RVec3 top = r.ReadRVec3();
				Vec3 axis = r.ReadVec3();
				Vec3 perpendicular = r.ReadVec3();
				float half_angle = r.Read<float>();
				float length = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawOpenCone(top, axis, perpendicular, half_angle, length, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 28: // DrawSwingConeLimits
			{
				RMat44 matrix = r.ReadRMat44();
				float swing_y_half_angle = r.Read<float>();
				float swing_z_half_angle = r.Read<float>();
				float edge_length = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawSwingConeLimits(matrix, swing_y_half_angle, swing_z_half_angle, edge_length, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 29: // DrawSwingPyramidLimits
			{
				RMat44 matrix = r.ReadRMat44();
				float min_swing_y_angle = r.Read<float>();
				float max_swing_y_angle = r.Read<float>();
				float min_swing_z_angle = r.Read<float>();
				float max_swing_z_angle = r.Read<float>();
				float edge_length = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawSwingPyramidLimits(matrix, min_swing_y_angle, max_swing_y_angle, min_swing_z_angle, max_swing_z_angle, edge_length, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 30: // DrawPie
			{
				RVec3 center = r.ReadRVec3();
				float radius = r.Read<float>();
				Vec3 normal = r.ReadVec3();
				Vec3 axis = r.ReadVec3();
				float min_angle = r.Read<float>();
				float max_angle = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawPie(center, radius, normal, axis, min_angle, max_angle, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 31: // DrawTaperedCylinder
			{
				RMat44 matrix = r.ReadRMat44();
				float top = r.Read<float>();
				float bottom = r.Read<float>();
				float top_radius = r.Read<float>();
				float bottom_radius = r.Read<float>();
				Color color = r.ReadColor();
				int32 cast_shadow = r.Read<int32>();
				int32 draw_mode = r.Read<int32>();
				ioRenderer.DrawTaperedCylinder(matrix, top, bottom, top_radius, bottom_radius, color, DebugRenderer::ECastShadow(cast_shadow), DebugRenderer::EDrawMode(draw_mode));
				break;
			}

		case 32: // NextFrame
			ioRenderer.NextFrame();
			break;

		case 33: // CreateTriangleBatch(const VertexList &, const IndexedTriangleNoMaterialList &), then release it
			{
				uint32 num_vertices = r.Read<uint32>();
				VertexList vertices(num_vertices);
				for (Float3 &v : vertices)
				{
					v.x = r.Read<float>();
					v.y = r.Read<float>();
					v.z = r.Read<float>();
				}
				uint32 num_triangles = r.Read<uint32>();
				IndexedTriangleNoMaterialList triangles(num_triangles);
				for (IndexedTriangleNoMaterial &t : triangles)
					for (uint32 &idx : t.mIdx)
						idx = r.Read<uint32>();
				DebugRenderer::Batch batch = ioRenderer.CreateTriangleBatch(vertices, triangles);
				break;
			}

		case 34: // CreateTriangleBatchForConvex: support, i32 level, u8 with bounds, u32 slot. With bounds the slot gets
				 // new Geometry(batch, bounds), else the batch is released.
			{
				ParitySupport support = ParityReadSupport(r);
				int32 level = r.Read<int32>();
				uint8 with_bounds = r.Read<uint8>();
				uint32 slot = r.Read<uint32>();
				AABox bounds;
				DebugRenderer::Batch batch = ioRenderer.CreateTriangleBatchForConvex([&support](Vec3Arg inDirection) { return support.GetSupport(inDirection); }, level, with_bounds != 0? &bounds : nullptr);
				if (with_bounds != 0)
					slots[slot] = new DebugRenderer::Geometry(batch, bounds);
				break;
			}

		case 35: // CreateTriangleGeometryForConvex: support, u32 slot
			{
				ParitySupport support = ParityReadSupport(r);
				uint32 slot = r.Read<uint32>();
				slots[slot] = ioRenderer.CreateTriangleGeometryForConvex([&support](Vec3Arg inDirection) { return support.GetSupport(inDirection); });
				break;
			}

		case 36: // Triangles made with both Triangle constructors: u32 count, per triangle u8 kind (1: with UV origin and
				 // direction), Vec3 v1, v2, v3, Color [, Vec3 UV origin, Vec3 UV direction]. CreateTriangleBatch, then release.
			{
				uint32 count = r.Read<uint32>();
				Array<DebugRenderer::Triangle> triangles;
				for (uint32 i = 0; i < count; ++i)
				{
					uint8 kind = r.Read<uint8>();
					Vec3 v1 = r.ReadVec3();
					Vec3 v2 = r.ReadVec3();
					Vec3 v3 = r.ReadVec3();
					Color color = r.ReadColor();
					if (kind == 0)
						triangles.push_back(DebugRenderer::Triangle(v1, v2, v3, color));
					else
					{
						Vec3 uv_origin = r.ReadVec3();
						Vec3 uv_direction = r.ReadVec3();
						triangles.push_back(DebugRenderer::Triangle(v1, v2, v3, color, uv_origin, uv_direction));
					}
				}
				DebugRenderer::Batch batch = ioRenderer.CreateTriangleBatch(triangles);
				break;
			}

		// D2: the shapes of ioShapes

		case 64: // Shape::Draw: u32 shape, matrix, Vec3 scale, Color, u8 use material colors, u8 draw wireframe
			{
				const Shape *shape = ParityShape(ioShapes, r.Read<uint32>());
				RMat44 transform = r.ReadRMat44();
				Vec3 scale = r.ReadVec3();
				Color color = r.ReadColor();
				bool use_material_colors = r.Read<uint8>() != 0;
				bool draw_wireframe = r.Read<uint8>() != 0;
				shape->Draw(&ioRenderer, transform, scale, color, use_material_colors, draw_wireframe);
				break;
			}

		case 65: // Shape::DrawGetSupportFunction: u32 shape, matrix, Vec3 scale, Color, u8 draw support direction
			{
				const Shape *shape = ParityShape(ioShapes, r.Read<uint32>());
				RMat44 transform = r.ReadRMat44();
				Vec3 scale = r.ReadVec3();
				Color color = r.ReadColor();
				bool draw_support_direction = r.Read<uint8>() != 0;
				shape->DrawGetSupportFunction(&ioRenderer, transform, scale, color, draw_support_direction);
				break;
			}

		case 66: // Shape::DrawGetSupportingFace: u32 shape, matrix, Vec3 scale
			{
				const Shape *shape = ParityShape(ioShapes, r.Read<uint32>());
				RMat44 transform = r.ReadRMat44();
				Vec3 scale = r.ReadVec3();
				shape->DrawGetSupportingFace(&ioRenderer, transform, scale);
				break;
			}

		case 67: // The static draw flags of the shapes: u8 bits
			ParitySetShapeDrawFlags(r.Read<uint8>());
			break;

		case 68: // Shape::GetSubmergedVolume (draws into DebugRenderer::sInstance, which is ioRenderer): u32 shape, Vec3 columns
				 // 0..2, Vec3 translation, Vec3 scale, Vec3 plane normal, f32 plane constant, RVec3 base offset
			{
				const Shape *shape = ParityShape(ioShapes, r.Read<uint32>());
				Vec3 c0 = r.ReadVec3();
				Vec3 c1 = r.ReadVec3();
				Vec3 c2 = r.ReadVec3();
				Vec3 t = r.ReadVec3();
				Vec3 scale = r.ReadVec3();
				Vec3 normal = r.ReadVec3();
				float constant = r.Read<float>();
				RVec3 base_offset = r.ReadRVec3();
				JPH_ASSERT(DebugRenderer::sInstance == &ioRenderer);
				float total_volume, submerged_volume;
				Vec3 center_of_buoyancy;
				shape->GetSubmergedVolume(Mat44(Vec4(c0, 0), Vec4(c1, 0), Vec4(c2, 0), Vec4(t, 1)), scale, Plane(normal, constant), total_volume, submerged_volume, center_of_buoyancy, base_offset);
				ioShapes->WriteResult(total_volume);
				ioShapes->WriteResult(submerged_volume);
				ioShapes->WriteResult(center_of_buoyancy.GetX());
				ioShapes->WriteResult(center_of_buoyancy.GetY());
				ioShapes->WriteResult(center_of_buoyancy.GetZ());
				break;
			}

		case 69: // ConvexHullShape::DrawShrunkShape: u32 shape (a convex hull), matrix, Vec3 scale
			{
				const Shape *shape = ParityShape(ioShapes, r.Read<uint32>());
				RMat44 transform = r.ReadRMat44();
				Vec3 scale = r.ReadVec3();
				static_cast<const ConvexHullShape *>(shape)->DrawShrunkShape(&ioRenderer, transform, scale);
				break;
			}

		// D3: the constraints and bodies of ioPhysics

		case 96: // Constraint::DrawConstraint (0), DrawConstraintLimits (1) or DrawConstraintReferenceFrame (2): u32 constraint, u8 what
			{
				const Constraint *c = ParityPhysics(ioPhysics).mConstraints[r.Read<uint32>()];
				switch (r.Read<uint8>())
				{
				case 0:		c->DrawConstraint(&ioRenderer); break;
				case 1:		c->DrawConstraintLimits(&ioRenderer); break;
				default:	c->DrawConstraintReferenceFrame(&ioRenderer); break;
				}
				break;
			}

		case 97: // Constraint::SetDrawConstraintSize: u32 constraint, f32 size
			{
				Constraint *c = ParityPhysics(ioPhysics).mConstraints[r.Read<uint32>()];
				c->SetDrawConstraintSize(r.Read<float>());
				break;
			}

		case 98: // The next frame of the scenario of a constraint: u32 constraint
			ParityPhysics(ioPhysics).mStepConstraint(r.Read<uint32>());
			break;

		case 99: // ConstraintManager::DrawConstraints (0), DrawConstraintLimits (1) or DrawConstraintReferenceFrame (2): u8 what
			{
				const ConstraintManager *manager = ParityPhysics(ioPhysics).mConstraintManager;
				switch (r.Read<uint8>())
				{
				case 0:		manager->DrawConstraints(&ioRenderer); break;
				case 1:		manager->DrawConstraintLimits(&ioRenderer); break;
				default:	manager->DrawConstraintReferenceFrame(&ioRenderer); break;
				}
				break;
			}

		case 100: // BodyManager::Draw: u32 settings bits, u8 shape color, u8 soft body constraint color, f32 time before sleep, u8 filter
			{
				BodyManager *manager = ParityPhysics(ioPhysics).mBodyManager;
				uint32 bits = r.Read<uint32>();
				uint8 shape_color = r.Read<uint8>();
				BodyManager::DrawSettings settings = ParityBodyDrawSettings(bits, shape_color, r.Read<uint8>());
				PhysicsSettings physics_settings;
				physics_settings.mTimeBeforeSleep = r.Read<float>();
				ParityBodyDrawFilter filter;
				manager->Draw(settings, physics_settings, &ioRenderer, r.Read<uint8>() != 0? &filter : nullptr);
				break;
			}

		case 101: // Body::UpdateSleepStateInternal: u32 body, f32 delta time, f32 max movement, f32 time before sleep
			{
				Body *body = ParityPhysics(ioPhysics).mBodies[r.Read<uint32>()];
				float delta_time = r.Read<float>();
				float max_movement = r.Read<float>();
				ECanSleep result = body->UpdateSleepStateInternal(delta_time, max_movement, r.Read<float>());
				ioPhysics->WriteResult(uint32(result));
				break;
			}

		case 102: // Body::SetPositionAndRotationInternal: u32 body, RVec3 position, Quat (4 floats)
			{
				Body *body = ParityPhysics(ioPhysics).mBodies[r.Read<uint32>()];
				RVec3 position = r.ReadRVec3();
				float x = r.Read<float>();
				float y = r.Read<float>();
				float z = r.Read<float>();
				body->SetPositionAndRotationInternal(position, Quat(x, y, z, r.Read<float>()));
				break;
			}

		case 103: // BodyManager::ActivateBodies (1) or DeactivateBodies (0) of one body: u32 body, u8 activate
			{
				ParityPhysicsContext &physics = ParityPhysics(ioPhysics);
				BodyID id = physics.mBodies[r.Read<uint32>()]->GetID();
				if (r.Read<uint8>() != 0)
					physics.mBodyManager->ActivateBodies(&id, 1);
				else
					physics.mBodyManager->DeactivateBodies(&id, 1);
				break;
			}

		default:
			Trace("RunParityOps: unknown op %u", uint(op));
			abort();
		}
	}
}

} // ZoltParity

#endif // JPH_DEBUG_RENDERER
