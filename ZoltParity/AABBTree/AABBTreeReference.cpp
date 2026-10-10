// Reference implementation for the AABBTree / TriangleSplitter parity tests (`zig build parity`): C ABI wrappers around
// the C++ Jolt library (TriangleSplitterBinning, TriangleSplitterMean, AABBTreeBuilder, AABBTreeToBuffer with
// NodeCodecQuadTreeHalfFloat and TriangleCodecIndexed8BitPackSOA4Flags). ZoltParity/AABBTree/AABBTreeParity.zig does
// the same with Zolt and checks that both produce the same bits.
//
// Conventions: every function writes its results into a stream of uint32 (floats as their bits, strings as their
// length followed by one uint32 per character) and returns the number of values in the stream. When the stream is
// longer than inCapacity only the first inCapacity values are written, the caller retries with a bigger buffer.
// Float3 and IndexedTriangle are passed as raw memory, Zolt's structs have the same layout.

#include <Jolt/Jolt.h>
#include <Jolt/AABBTree/AABBTreeBuilder.h>
#include <Jolt/AABBTree/AABBTreeToBuffer.h>
#include <Jolt/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h>
#include <Jolt/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h>
#include <Jolt/Geometry/AABox4.h>
#include <Jolt/Geometry/RayAABox.h>
#include <Jolt/Physics/Collision/SortReverseAndStore.h>
#include <Jolt/TriangleSplitter/TriangleSplitterBinning.h>
#include <Jolt/TriangleSplitter/TriangleSplitterMean.h>

#include <cstring>

using namespace JPH;

static_assert(sizeof(IndexedTriangle) == 20, "Zolt passes IndexedTriangle as raw memory");
static_assert(sizeof(Float3) == 12, "Zolt passes Float3 as raw memory");

using TriangleCodec = TriangleCodecIndexed8BitPackSOA4Flags;
using NodeCodec = NodeCodecQuadTreeHalfFloat;
using Buffer = AABBTreeToBuffer<TriangleCodec, NodeCodec>;

// Must match Input in AABBTreeParity.zig
struct ParityInput
{
	const Float3 *			mVertices;
	uint32					mNumVertices;
	const IndexedTriangle *	mTriangles;
	uint32					mNumTriangles;
	uint32					mSplitterType;			///< 0 = TriangleSplitterBinning, 1 = TriangleSplitterMean
	uint32					mMinNumBins;
	uint32					mMaxNumBins;
	uint32					mNumTrianglesPerBin;
	uint32					mMaxTrianglesPerLeaf;
	uint32					mStoreUserData;
};

// Section markers in the streams, must match AABBTreeParity.zig
enum : uint32
{
	cMarkerNode = 0xA0000001,
	cMarkerTriangles = 0xA0000002,
	cMarkerWalkDone = 0xA0000003,
	cMarkerAbort = 0xA0000004,
	cMarkerAllVisitor = 0xA0000010,
	cMarkerRayVisitor = 0xA0000011,
	cMarkerBoxVisitor = 0xA0000012,
	cMarkerNoProgress = 0xA0000013,
};

// Jolt's Array allocates through the Allocate / Free hooks, which must be set before the first allocation
static void EnsureAllocator()
{
	RegisterDefaultAllocator();
}

// Output stream of uint32 values
class Stream
{
public:
				Stream(uint32 *outValues, uint32 inCapacity) : mValues(outValues), mCapacity(inCapacity) { }

	void		Add(uint32 inValue)
	{
		if (mSize < mCapacity)
			mValues[mSize] = inValue;
		++mSize;
	}

	void		AddFloat(float inValue)
	{
		uint32 bits;
		memcpy(&bits, &inValue, sizeof(bits));
		Add(bits);
	}

	void		AddVec3(Vec3Arg inValue)
	{
		AddFloat(inValue.GetX());
		AddFloat(inValue.GetY());
		AddFloat(inValue.GetZ());
	}

	void		AddVec4(Vec4Arg inValue)
	{
		AddFloat(inValue.GetX());
		AddFloat(inValue.GetY());
		AddFloat(inValue.GetZ());
		AddFloat(inValue.GetW());
	}

	void		AddUVec4(UVec4Arg inValue)
	{
		Add(inValue.GetX());
		Add(inValue.GetY());
		Add(inValue.GetZ());
		Add(inValue.GetW());
	}

	void		AddString(const char *inString)
	{
		if (inString == nullptr)
		{
			Add(0xffffffff);
			return;
		}
		uint32 len = uint32(strlen(inString));
		Add(len);
		for (uint32 i = 0; i < len; ++i)
			Add(uint8(inString[i]));
	}

	uint32		GetSize() const			{ return mSize; }

private:
	uint32 *	mValues;
	uint32		mCapacity;
	uint32		mSize = 0;
};

// The mesh of the input as Jolt containers
struct Mesh
{
	explicit	Mesh(const ParityInput &inInput) :
		mVertices(inInput.mVertices, inInput.mVertices + inInput.mNumVertices),
		mTriangles(inInput.mTriangles, inInput.mTriangles + inInput.mNumTriangles)
	{
	}

	VertexList				mVertices;
	IndexedTriangleList		mTriangles;
};

// Holds the splitter selected by the input
struct Splitter
{
				Splitter(const ParityInput &inInput, const Mesh &inMesh)
	{
		if (inInput.mSplitterType == 0)
			mSplitter = new TriangleSplitterBinning(inMesh.mVertices, inMesh.mTriangles, inInput.mMinNumBins, inInput.mMaxNumBins, inInput.mNumTrianglesPerBin);
		else
			mSplitter = new TriangleSplitterMean(inMesh.mVertices, inMesh.mTriangles);
	}

				~Splitter()				{ delete mSplitter; }

	TriangleSplitter *		mSplitter;
};

// Builds and converts the tree of the input
struct Tree
{
				Tree(const ParityInput &inInput) :
		mMesh(inInput),
		mSplitter(inInput, mMesh),
		mBuilder(*mSplitter.mSplitter, inInput.mMaxTrianglesPerLeaf)
	{
		AABBTreeBuilderStats stats;
		mRoot = mBuilder.Build(stats);
		mSuccess = mBuffer.Convert(mBuilder.GetTriangles(), mBuilder.GetNodes(), mMesh.mVertices, mRoot, inInput.mStoreUserData != 0, mError);
	}

	Mesh					mMesh;
	Splitter				mSplitter;
	AABBTreeBuilder			mBuilder;
	const AABBTreeBuilder::Node *mRoot;
	Buffer					mBuffer;
	bool					mSuccess;
	const char *			mError = nullptr;
};

// Adds ValidationContext::IsDegenerate for every triangle of the mesh to the stream
static void AddValidation(const Mesh &inMesh, Stream &ioStream)
{
	TriangleCodec::ValidationContext validation(inMesh.mTriangles, inMesh.mVertices);
	for (const IndexedTriangle &t : inMesh.mTriangles)
		ioStream.Add(validation.IsDegenerate(t)? 1 : 0);
}

// Visits all nodes and decodes all triangles
struct AllVisitor
{
	bool		ShouldAbort() const					{ return false; }
	bool		ShouldVisitNode(int) const			{ return true; }

	int			VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
	{
		mStream.Add(cMarkerNode);
		mStream.Add(uint32(inStackTop));
		mStream.AddVec4(inBoundsMinX);
		mStream.AddVec4(inBoundsMinY);
		mStream.AddVec4(inBoundsMinZ);
		mStream.AddVec4(inBoundsMaxX);
		mStream.AddVec4(inBoundsMaxY);
		mStream.AddVec4(inBoundsMaxZ);
		mStream.AddUVec4(ioProperties);
		return 4;
	}

	void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, uint32 inTriangleBlockID)
	{
		mStream.Add(cMarkerTriangles);
		mStream.Add(inTriangleBlockID);
		mStream.Add(uint32(inNumTriangles));

		Vec3 vertices[16 * 3];
		uint8 flags[16];
		ioContext.Unpack(inTriangles, inNumTriangles, vertices, flags);
		for (int i = 0; i < 3 * inNumTriangles; ++i)
			mStream.AddVec3(vertices[i]);
		for (int i = 0; i < inNumTriangles; ++i)
			mStream.Add(flags[i]);

		for (int i = 0; i < inNumTriangles; ++i)
		{
			Vec3 v1, v2, v3;
			ioContext.GetTriangle(inTriangles, i, v1, v2, v3);
			mStream.AddVec3(v1);
			mStream.AddVec3(v2);
			mStream.AddVec3(v3);
			mStream.Add(ioContext.GetUserData(inTriangles, i));
			mStream.Add(TriangleCodec::DecodingContext::sGetFlags(inTriangles, i));
		}
	}

	Stream &	mStream;
};

// Casts a ray (like MeshShape::CastRay)
struct RayVisitor
{
	bool		ShouldAbort() const					{ return mFraction <= 0.0f; }
	bool		ShouldVisitNode(int inStackTop) const	{ return mDistanceStack[inStackTop] < mFraction; }

	int			VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
	{
		Vec4 distance = RayAABox4(mRayOrigin, mRayInvDirection, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		int num_results = SortReverseAndStore(distance, mFraction, ioProperties, &mDistanceStack[inStackTop]);

		mStream.Add(cMarkerNode);
		mStream.Add(uint32(inStackTop));
		mStream.AddVec4(distance);
		mStream.AddUVec4(ioProperties);
		mStream.Add(uint32(num_results));
		for (int i = 0; i < 4; ++i)
			mStream.AddFloat(mDistanceStack[inStackTop + i]);
		return num_results;
	}

	void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, uint32 inTriangleBlockID)
	{
		uint32 triangle_idx;
		float fraction = ioContext.TestRay(mRayOrigin, mRayDirection, inTriangles, inNumTriangles, mFraction, triangle_idx);

		mStream.Add(cMarkerTriangles);
		mStream.Add(inTriangleBlockID);
		mStream.Add(uint32(inNumTriangles));
		mStream.AddFloat(fraction);
		mStream.Add(triangle_idx);

		if (fraction < mFraction)
		{
			mFraction = fraction;
			mTriangleBlockID = inTriangleBlockID;
			mTriangleIdx = triangle_idx;
		}
	}

	Stream &	mStream;
	Vec3		mRayOrigin;
	Vec3		mRayDirection;
	RayInvDirection mRayInvDirection;
	float		mFraction;
	uint32		mTriangleBlockID = 0xffffffff;
	uint32		mTriangleIdx = 0xffffffff;
	float		mDistanceStack[NodeCodec::StackSize];
};

// Collects the triangles that overlap with a box, at most mMaxTriangles per walk (like MeshShape::GetTrianglesNext)
struct BoxVisitor
{
	bool		ShouldAbort() const					{ return mShouldAbort; }
	bool		ShouldVisitNode(int) const			{ return true; }

	int			VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
	{
		UVec4 collides = AABox4VsBox(mBox, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		int num_results = CountAndSortTrues(collides, ioProperties);

		mStream.Add(cMarkerNode);
		mStream.Add(uint32(inStackTop));
		mStream.AddUVec4(collides);
		mStream.AddUVec4(ioProperties);
		mStream.Add(uint32(num_results));
		return num_results;
	}

	void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, uint32 inTriangleBlockID)
	{
		// When the buffer is full abort the walk, the next walk continues here
		if (mNumTrianglesFound + inNumTriangles > mMaxTriangles)
		{
			mStream.Add(cMarkerAbort);
			mShouldAbort = true;
			return;
		}

		Vec3 vertices[16 * 3];
		ioContext.Unpack(inTriangles, inNumTriangles, vertices);

		mStream.Add(cMarkerTriangles);
		mStream.Add(inTriangleBlockID);
		mStream.Add(uint32(inNumTriangles));
		for (int i = 0; i < 3 * inNumTriangles; ++i)
			mStream.AddVec3(vertices[i]);

		mNumTrianglesFound += inNumTriangles;
	}

	Stream &	mStream;
	AABox		mBox;
	int			mMaxTriangles;
	int			mNumTrianglesFound = 0;
	bool		mShouldAbort = false;
};

extern "C"
{

// Runs the splitter like AABBTreeBuilder would (depth first, left first) until the ranges have at most inLeafSize
// triangles. Stream: splitter name, initial range, then per Split call: range, success, left, right (when the split
// fails, the halves that AABBTreeBuilder uses), and at the end the order of the triangles (index into the input).
uint32 jolt_aabb_splitter_run(const ParityInput *inInput, uint32 inLeafSize, uint32 *outStream, uint32 inCapacity)
{
	EnsureAllocator();

	Stream stream(outStream, inCapacity);
	Mesh mesh(*inInput);
	Splitter splitter(*inInput, mesh);
	TriangleSplitter &s = *splitter.mSplitter;

	TriangleSplitter::Stats stats;
	s.GetStats(stats);
	stream.AddString(stats.mSplitterName);
	stream.Add(uint32(stats.mLeafSize));

	TriangleSplitter::Range initial = s.GetInitialRange();
	stream.Add(initial.mBegin);
	stream.Add(initial.mEnd);

	Array<TriangleSplitter::Range> stack;
	stack.push_back(initial);
	while (!stack.empty())
	{
		TriangleSplitter::Range range = stack.back();
		stack.pop_back();
		if (range.Count() <= inLeafSize)
			continue;

		TriangleSplitter::Range left, right;
		bool success = s.Split(range, left, right);
		if (!success)
		{
			uint half = range.Count() / 2;
			left = TriangleSplitter::Range(range.mBegin, range.mBegin + half);
			right = TriangleSplitter::Range(range.mBegin + half, range.mEnd);
		}
		stream.Add(range.mBegin);
		stream.Add(range.mEnd);
		stream.Add(success? 1 : 0);
		stream.Add(left.mBegin);
		stream.Add(left.mEnd);
		stream.Add(right.mBegin);
		stream.Add(right.mEnd);

		stack.push_back(right);
		stack.push_back(left);
	}

	for (uint i = 0; i < inInput->mNumTriangles; ++i)
		stream.Add(uint32(&s.GetTriangle(i) - mesh.mTriangles.data()));

	return stream.GetSize();
}

// Builds the tree. Stream: stats, root index, nodes (bounds, triangle begin for leaves, triangle count, children,
// HasChildren), triangles, then for the first inNumQueryNodes nodes the results of the Node query functions.
uint32 jolt_aabb_tree_build(const ParityInput *inInput, uint32 inNumQueryNodes, float inCostTraversal, float inCostLeaf, uint32 *outStream, uint32 inCapacity)
{
	EnsureAllocator();

	Stream stream(outStream, inCapacity);
	Mesh mesh(*inInput);
	Splitter splitter(*inInput, mesh);
	AABBTreeBuilder builder(*splitter.mSplitter, inInput->mMaxTrianglesPerLeaf);
	AABBTreeBuilderStats stats;
	const AABBTreeBuilder::Node *root = builder.Build(stats);
	const Array<AABBTreeBuilder::Node> &nodes = builder.GetNodes();

	// Stats
	stream.AddString(stats.mSplitterStats.mSplitterName);
	stream.Add(uint32(stats.mSplitterStats.mLeafSize));
	stream.AddFloat(stats.mSAHCost);
	stream.Add(uint32(stats.mMinDepth));
	stream.Add(uint32(stats.mMaxDepth));
	stream.Add(uint32(stats.mNodeCount));
	stream.Add(uint32(stats.mLeafNodeCount));
	stream.Add(uint32(stats.mMaxTrianglesPerLeaf));
	stream.Add(uint32(stats.mTreeMinTrianglesPerLeaf));
	stream.Add(uint32(stats.mTreeMaxTrianglesPerLeaf));
	stream.AddFloat(stats.mTreeAvgTrianglesPerLeaf);

	// Nodes
	stream.Add(uint32(root - nodes.data()));
	stream.Add(uint32(nodes.size()));
	for (const AABBTreeBuilder::Node &n : nodes)
	{
		stream.AddVec3(n.mBounds.mMin);
		stream.AddVec3(n.mBounds.mMax);
		stream.Add(n.HasChildren()? 0xffffffff : n.mTrianglesBegin); // Not initialized for nodes with children
		stream.Add(n.mNumTriangles);
		stream.Add(n.mChild[0]);
		stream.Add(n.mChild[1]);
		stream.Add(n.GetTriangleCount());
		stream.Add(n.HasChildren()? 1 : 0);
	}

	// Triangles
	const Array<IndexedTriangle> &triangles = builder.GetTriangles();
	stream.Add(uint32(triangles.size()));
	for (const IndexedTriangle &t : triangles)
	{
		stream.Add(t.mIdx[0]);
		stream.Add(t.mIdx[1]);
		stream.Add(t.mIdx[2]);
		stream.Add(t.mMaterialIndex);
		stream.Add(t.mUserData);
	}

	// Node queries
	Array<const AABBTreeBuilder::Node *> children;
	for (uint i = 0; i < min(inNumQueryNodes, uint32(nodes.size())); ++i)
	{
		const AABBTreeBuilder::Node &n = nodes[i];
		stream.Add(n.GetMinDepth(nodes));
		stream.Add(n.GetMaxDepth(nodes));
		stream.Add(n.GetNodeCount(nodes));
		stream.Add(n.GetLeafNodeCount(nodes));
		stream.Add(n.GetTriangleCountInTree(nodes));
		float average;
		uint min_count, max_count;
		n.GetTriangleCountPerNode(nodes, average, min_count, max_count);
		stream.AddFloat(average);
		stream.Add(min_count);
		stream.Add(max_count);
		stream.AddFloat(n.CalculateSAHCost(nodes, 1.0f, 1.0f));
		stream.AddFloat(n.CalculateSAHCost(nodes, inCostTraversal, inCostLeaf));
		for (uint c = 0; c < 2; ++c)
		{
			const AABBTreeBuilder::Node *child = n.GetChild(c, nodes);
			stream.Add(child != nullptr? uint32(child - nodes.data()) : 0xffffffff);
		}
		for (uint num = 1; num <= 8; ++num)
		{
			children.clear();
			n.GetNChildren(nodes, num, children);
			stream.Add(uint32(children.size()));
			for (const AABBTreeBuilder::Node *c : children)
				stream.Add(uint32(c - nodes.data()));
		}
	}

	return stream.GetSize();
}

// Builds and converts the tree. Stream: ValidationContext::IsDegenerate for every triangle, success, then the error
// message or the size of the buffer and its bytes (4 per uint32, little endian, zero padded).
uint32 jolt_aabb_tree_convert(const ParityInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	EnsureAllocator();

	Stream stream(outStream, inCapacity);
	Tree tree(*inInput);

	AddValidation(tree.mMesh, stream);

	stream.Add(tree.mSuccess? 1 : 0);
	if (!tree.mSuccess)
	{
		stream.AddString(tree.mError);
		return stream.GetSize();
	}

	const ByteBuffer &buffer = tree.mBuffer.GetBuffer();
	stream.Add(uint32(buffer.size()));
	for (size_t i = 0; i < buffer.size(); i += 4)
	{
		uint32 value = 0;
		for (size_t b = 0; b < 4 && i + b < buffer.size(); ++b)
			value |= uint32(buffer[i + b]) << (8 * b);
		stream.Add(value);
	}

	// Offsets of the headers and the root
	const uint8 *start = buffer.data();
	stream.Add(uint32(reinterpret_cast<const uint8 *>(tree.mBuffer.GetNodeHeader()) - start));
	stream.Add(uint32(reinterpret_cast<const uint8 *>(tree.mBuffer.GetTriangleHeader()) - start));
	stream.Add(uint32(reinterpret_cast<const uint8 *>(tree.mBuffer.GetRoot()) - start));

	return stream.GetSize();
}

// Only ValidationContext::IsDegenerate for every triangle, without building a tree (meshes with NaN vertices trip
// the asserts of the splitters and AABBTreeToBuffer, in Jolt and in Zolt). Stream: IsDegenerate per triangle.
uint32 jolt_aabb_validate(const ParityInput *inInput, uint32 *outStream, uint32 inCapacity)
{
	EnsureAllocator();

	Stream stream(outStream, inCapacity);
	Mesh mesh(*inInput);
	AddValidation(mesh, stream);
	return stream.GetSize();
}

// Builds and converts the tree and walks it with three visitors: one that visits everything, a ray cast for each ray
// (origin, direction, initial fraction: 7 floats) and a box query for each box (min, max: 6 floats) that collects at
// most inMaxTriangles[i] triangles per walk and walks again until done.
uint32 jolt_aabb_tree_walk(const ParityInput *inInput, const float *inRays, uint32 inNumRays, const float *inBoxes, const uint32 *inMaxTriangles, uint32 inNumBoxes, uint32 *outStream, uint32 inCapacity)
{
	EnsureAllocator();

	Stream stream(outStream, inCapacity);
	Tree tree(*inInput);
	if (!tree.mSuccess)
		return stream.GetSize();

	const NodeCodec::Header *header = tree.mBuffer.GetNodeHeader();
	const uint8 *buffer_start = &tree.mBuffer.GetBuffer()[0];
	const TriangleCodec::DecodingContext triangle_ctx(tree.mBuffer.GetTriangleHeader());
	stream.Add(NodeCodec::DecodingContext::sTriangleBlockIDBits(header));

	// Visit everything
	{
		stream.Add(cMarkerAllVisitor);
		AllVisitor visitor { stream };
		NodeCodec::DecodingContext ctx(header);
		ctx.WalkTree(buffer_start, triangle_ctx, visitor);
		stream.Add(cMarkerWalkDone);
		stream.Add(ctx.IsDoneWalking()? 1 : 0);
	}

	// Ray casts
	for (uint32 r = 0; r < inNumRays; ++r)
	{
		const float *ray = inRays + 7 * r;
		stream.Add(cMarkerRayVisitor);
		RayVisitor visitor { stream };
		visitor.mRayOrigin = Vec3(ray[0], ray[1], ray[2]);
		visitor.mRayDirection = Vec3(ray[3], ray[4], ray[5]);
		visitor.mRayInvDirection.Set(visitor.mRayDirection);
		visitor.mFraction = ray[6];
		NodeCodec::DecodingContext ctx(header);
		ctx.WalkTree(buffer_start, triangle_ctx, visitor);
		stream.Add(cMarkerWalkDone);
		stream.Add(ctx.IsDoneWalking()? 1 : 0);
		stream.AddFloat(visitor.mFraction);
		stream.Add(visitor.mTriangleBlockID);
		stream.Add(visitor.mTriangleIdx);
		if (visitor.mTriangleBlockID != 0xffffffff)
		{
			// Decode the triangle that was hit
			Vec3 v1, v2, v3;
			triangle_ctx.GetTriangle(NodeCodec::DecodingContext::sGetTriangleBlockStart(buffer_start, visitor.mTriangleBlockID), visitor.mTriangleIdx, v1, v2, v3);
			stream.AddVec3(v1);
			stream.AddVec3(v2);
			stream.AddVec3(v3);
		}
	}

	// Box queries
	for (uint32 b = 0; b < inNumBoxes; ++b)
	{
		const float *box = inBoxes + 6 * b;
		stream.Add(cMarkerBoxVisitor);
		BoxVisitor visitor { stream };
		visitor.mBox = AABox(Vec3(box[0], box[1], box[2]), Vec3(box[3], box[4], box[5]));
		visitor.mMaxTriangles = int(inMaxTriangles[b]);
		NodeCodec::DecodingContext ctx(header);
		for (;;)
		{
			visitor.mNumTrianglesFound = 0;
			visitor.mShouldAbort = false;
			ctx.WalkTree(buffer_start, triangle_ctx, visitor);
			stream.Add(cMarkerWalkDone);
			stream.Add(uint32(visitor.mNumTrianglesFound));
			stream.Add(ctx.IsDoneWalking()? 1 : 0);
			if (ctx.IsDoneWalking())
				break;
			if (visitor.mNumTrianglesFound == 0)
			{
				// The budget is smaller than a triangle block
				stream.Add(cMarkerNoProgress);
				break;
			}
		}
	}

	return stream.GetSize();
}

} // extern "C"
