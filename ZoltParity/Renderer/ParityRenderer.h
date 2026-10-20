// The C++ side of the draw call log of the debug renderer parity (Docs/Zolt/RemainingFeaturesArchitecture.md section 4,
// "Draw call log"). ZoltParity/Renderer/ParityRenderer.zig is the Zig side: both write the same bytes for the same
// calls, so the logs of Jolt and Zolt compare byte for byte.
//
// - ParityRenderer is a JPH::DebugRenderer that logs every virtual call with all its arguments, including what a .jor
//   file drops: the world space bounds and the LOD scale of DrawGeometry and the contents of every batch.
// - SimpleParityRenderer is a JPH::DebugRendererSimple that logs DrawLine and DrawText3D. Simple's own DrawTriangle,
//   batches and DrawGeometry (with its LOD selection) produce the lines.
// - The op stream: the Zig side encodes the calls to make (random helper calls, ...) as bytes, RunParityOps executes
//   them on any renderer. Every group adds its ops in its own range (see ParityRenderer.zig).
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
#include <ZoltParity/ParityHooks.h>

#include <atomic>
#include <cstring>
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

/// Executes the op stream of ParityRenderer.zig on inRenderer (D0's ops 1 to 15: the virtual functions and the helpers of
/// DebugRenderer that D0 found ported). ioSimple and ioRecorder are the renderer as a DebugRendererSimple /
/// DebugRendererRecorder (or null) for the ops that only they have.
inline void RunParityOps(DebugRenderer &ioRenderer, const uint8 *inOps, size_t inSize, DebugRendererSimple *ioSimple, DebugRendererRecorder *ioRecorder)
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

		default:
			Trace("RunParityOps: unknown op %u", uint(op));
			abort();
		}
	}
}

} // ZoltParity

#endif // JPH_DEBUG_RENDERER
