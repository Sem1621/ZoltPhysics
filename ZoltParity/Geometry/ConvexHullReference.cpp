// Reference implementation for the ConvexHullBuilder parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt ConvexHullBuilder. ZoltParity/Geometry/ConvexHullParity.zig calls these and checks that Zolt produces the
// same bits.
//
// Conventions: positions are passed as 3 floats each. A builder is created once (jolt_chb_create) and can be
// initialized several times like the C++ class. Faces are identified by their index in GetFaces(), edges by their
// position in the edge loop of their face (0 = mFirstEdge).

#include <Jolt/Jolt.h>
#include <Jolt/Geometry/ConvexHullBuilder.h>

using namespace JPH;

namespace {

// A builder together with the positions it references (ConvexHullBuilder keeps a reference to the array)
struct HullBuilder
{
	HullBuilder(const float *inPositions, int inCount)
	{
		mPositions.reserve(inCount);
		for (int i = 0; i < inCount; ++i)
			mPositions.push_back(Vec3(inPositions[3 * i], inPositions[3 * i + 1], inPositions[3 * i + 2]));
	}

	ConvexHullBuilder::Positions	mPositions;
	ConvexHullBuilder				mBuilder { mPositions };
};

const ConvexHullBuilder &GetBuilder(const void *inBuilder)
{
	return static_cast<const HullBuilder *>(inBuilder)->mBuilder;
}

// Index of inFace in GetFaces(), -1 if not found / nullptr
int GetFaceIndex(const ConvexHullBuilder &inBuilder, const ConvexHullBuilder::Face *inFace)
{
	const ConvexHullBuilder::Faces &faces = inBuilder.GetFaces();
	for (size_t i = 0; i < faces.size(); ++i)
		if (faces[i] == inFace)
			return int(i);
	return -1;
}

// Position of inEdge in the edge loop of its face, -1 if not found / nullptr
int GetEdgeIndex(const ConvexHullBuilder::Edge *inEdge)
{
	if (inEdge == nullptr || inEdge->mFace->mFirstEdge == nullptr)
		return -1;
	int idx = 0;
	const ConvexHullBuilder::Edge *e = inEdge->mFace->mFirstEdge;
	do
	{
		if (e == inEdge)
			return idx;
		++idx;
		e = e->mNextEdge;
	} while (e != inEdge->mFace->mFirstEdge);
	return -1;
}

} // namespace

extern "C" {

// Constructor: copies the positions, the builder keeps a reference to the copy
void *jolt_chb_create(const float *inPositions, int inCount)
{
	RegisterDefaultAllocator();

	return new HullBuilder(inPositions, inCount);
}

// Destructor
void jolt_chb_destroy(void *inBuilder)
{
	delete static_cast<HullBuilder *>(inBuilder);
}

// ConvexHullBuilder::Initialize: returns the result code, *ioError is only written when building fails (like Jolt)
int jolt_chb_initialize(void *inBuilder, int inMaxVertices, float inTolerance, const char **ioError)
{
	return int(static_cast<HullBuilder *>(inBuilder)->mBuilder.Initialize(inMaxVertices, inTolerance, *ioError));
}

// ConvexHullBuilder::GetNumVerticesUsed
int jolt_chb_get_num_vertices_used(const void *inBuilder)
{
	return GetBuilder(inBuilder).GetNumVerticesUsed();
}

// ConvexHullBuilder::ContainsFace
bool jolt_chb_contains_face(const void *inBuilder, const int *inIndices, int inCount)
{
	Array<int> indices(inIndices, inIndices + inCount);
	return GetBuilder(inBuilder).ContainsFace(indices);
}

// ConvexHullBuilder::GetCenterOfMassAndVolume
void jolt_chb_get_center_of_mass_and_volume(const void *inBuilder, float *outCenterOfMass, float *outVolume)
{
	Vec3 com;
	GetBuilder(inBuilder).GetCenterOfMassAndVolume(com, *outVolume);
	outCenterOfMass[0] = com.GetX();
	outCenterOfMass[1] = com.GetY();
	outCenterOfMass[2] = com.GetZ();
}

// ConvexHullBuilder::DetermineMaxError, the face is returned as index in GetFaces() (-1 for nullptr)
void jolt_chb_determine_max_error(const void *inBuilder, int *outFaceIdx, float *outMaxError, int *outMaxErrorPositionIdx, float *outCoplanarDistance)
{
	const ConvexHullBuilder &builder = GetBuilder(inBuilder);
	ConvexHullBuilder::Face *face = nullptr;
	builder.DetermineMaxError(face, *outMaxError, *outMaxErrorPositionIdx, *outCoplanarDistance);
	*outFaceIdx = GetFaceIndex(builder, face);
}

// GetFaces().size()
int jolt_chb_get_num_faces(const void *inBuilder)
{
	return int(GetBuilder(inBuilder).GetFaces().size());
}

// Everything stored in face inFaceIdx of GetFaces(): for each edge (in loop order starting at mFirstEdge) its start
// index and the face index / edge index of its neighbour edge, the plane, the conflict list, the furthest distance and
// the removed flag. Arrays are written up to inCapacity entries, the counts are the full counts.
void jolt_chb_get_face(const void *inBuilder, int inFaceIdx, int inCapacity, int *outNumEdges, int *outStartIndices, int *outNeighbourFaces, int *outNeighbourEdges, float *outNormal, float *outCentroid, int *outNumConflicts, int *outConflictList, float *outFurthestPointDistanceSq, bool *outRemoved)
{
	const ConvexHullBuilder &builder = GetBuilder(inBuilder);
	const ConvexHullBuilder::Face *face = builder.GetFaces()[inFaceIdx];

	int num_edges = 0;
	const ConvexHullBuilder::Edge *e = face->mFirstEdge;
	if (e != nullptr)
		do
		{
			if (num_edges < inCapacity)
			{
				outStartIndices[num_edges] = e->mStartIdx;
				outNeighbourFaces[num_edges] = e->mNeighbourEdge != nullptr? GetFaceIndex(builder, e->mNeighbourEdge->mFace) : -1;
				outNeighbourEdges[num_edges] = GetEdgeIndex(e->mNeighbourEdge);
			}
			++num_edges;
			e = e->mNextEdge;
		} while (e != face->mFirstEdge);
	*outNumEdges = num_edges;

	outNormal[0] = face->mNormal.GetX();
	outNormal[1] = face->mNormal.GetY();
	outNormal[2] = face->mNormal.GetZ();
	outCentroid[0] = face->mCentroid.GetX();
	outCentroid[1] = face->mCentroid.GetY();
	outCentroid[2] = face->mCentroid.GetZ();

	*outNumConflicts = int(face->mConflictList.size());
	for (size_t i = 0; i < face->mConflictList.size() && int(i) < inCapacity; ++i)
		outConflictList[i] = face->mConflictList[i];

	*outFurthestPointDistanceSq = face->mFurthestPointDistanceSq;
	*outRemoved = face->mRemoved;
}

} // extern "C"
