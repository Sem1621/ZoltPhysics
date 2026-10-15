// Reference implementation for the path parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library's PathConstraintPathHermite (GetPathMaxFraction, GetPointOnPath, GetClosestPoint, SaveBinaryState and
// PathConstraintPath::sRestoreFromBinaryState). ZoltParity/Physics/PathConstraintPathParity.zig calls these and checks
// that Zolt produces the same bits.
//
// A path is a PCPPath: the number of points, the looping flag and per point 9 floats (position, tangent, normal) that
// are added with AddPoint in order. Booleans are passed as uint32 (never bool, see the porting guide). All C functions
// have the prefix jolt_pcp_. Functions that produce bytes write into a caller provided buffer and return the size.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Constraints/PathConstraintPathHermite.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

// Must match PCPPath in PathConstraintPathParity.zig
static constexpr uint32 cPCPMaxPoints = 64;

struct PCPPath
{
	uint32				mNumPoints;
	uint32				mLooping;
	float				mPoints[cPCPMaxPoints * 9];
};

// The Factory is needed by PathConstraintPath::sRestoreFromBinaryState (the same check as the other reference files,
// so RegisterTypes() runs once for the whole parity binary)
static void PCPEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static Vec3 PCPLoad3(const float *inV)
{
	return Vec3(inV[0], inV[1], inV[2]);
}

static void PCPStore3(Vec3Arg inV, float *outV)
{
	outV[0] = inV.GetX();
	outV[1] = inV.GetY();
	outV[2] = inV.GetZ();
}

// The Hermite path of a description
static Ref<PathConstraintPathHermite> PCPMakePath(const PCPPath &inPath)
{
	Ref<PathConstraintPathHermite> path = new PathConstraintPathHermite;
	path->SetIsLooping(inPath.mLooping != 0);
	for (uint32 p = 0; p < inPath.mNumPoints; ++p)
	{
		const float *pf = inPath.mPoints + 9 * p;
		path->AddPoint(PCPLoad3(pf), PCPLoad3(pf + 3), PCPLoad3(pf + 6));
	}
	return path;
}

// The bytes of a stream into a caller provided buffer (returns the size, copies at most inCapacity bytes)
static uint32 PCPCopyBytes(const std::string &inData, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inData.size());
	memcpy(outBytes, inData.data(), std::min(size, inCapacity));
	return size;
}

static std::string PCPSave(const PathConstraintPath &inPath)
{
	std::stringstream data;
	StreamOutWrapper out(data);
	inPath.SaveBinaryState(out);
	return data.str();
}

extern "C"
{

// sizeof(PCPPath)
uint32 jolt_pcp_struct_size()
{
	return uint32(sizeof(PCPPath));
}

// GetPathMaxFraction, GetPointOnPath of every fraction (12 floats: position, tangent, normal, binormal) and
// GetClosestPoint of every query (4 floats: position, fraction hint)
void jolt_pcp_query(const PCPPath *inPath, const float *inFractions, uint32 inNumFractions, const float *inQueries, uint32 inNumQueries, float *outMaxFraction, float *outPoints, float *outClosest)
{
	PCPEnsureFactory();
	Ref<PathConstraintPathHermite> path = PCPMakePath(*inPath);

	*outMaxFraction = path->GetPathMaxFraction();

	for (uint32 f = 0; f < inNumFractions; ++f)
	{
		Vec3 position, tangent, normal, binormal;
		path->GetPointOnPath(inFractions[f], position, tangent, normal, binormal);
		float *out = outPoints + 12 * f;
		PCPStore3(position, out);
		PCPStore3(tangent, out + 3);
		PCPStore3(normal, out + 6);
		PCPStore3(binormal, out + 9);
	}

	for (uint32 q = 0; q < inNumQueries; ++q)
	{
		const float *query = inQueries + 4 * q;
		outClosest[q] = path->GetClosestPoint(PCPLoad3(query), query[3]);
	}
}

// SaveBinaryState of the path of a description
uint32 jolt_pcp_save(const PCPPath *inPath, uint8 *outBytes, uint32 inCapacity)
{
	PCPEnsureFactory();
	Ref<PathConstraintPathHermite> path = PCPMakePath(*inPath);
	return PCPCopyBytes(PCPSave(*path), outBytes, inCapacity);
}

// PathConstraintPath::sRestoreFromBinaryState of the bytes, then SaveBinaryState of the result. Returns the size of
// the saved bytes, or 0xFFFFFFFF and the error text in outError.
uint32 jolt_pcp_resave(const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity, char *outError, uint32 inErrorCapacity)
{
	PCPEnsureFactory();
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(data);
	PathConstraintPath::PathResult result = PathConstraintPath::sRestoreFromBinaryState(in);
	if (result.HasError())
	{
		const String &error = result.GetError();
		size_t n = std::min(size_t(inErrorCapacity - 1), error.size());
		memcpy(outError, error.data(), n);
		outError[n] = 0;
		return 0xFFFFFFFF;
	}
	return PCPCopyBytes(PCPSave(*result.Get()), outBytes, inCapacity);
}

} // extern "C"
