//! Zolt: a Zig port of Jolt Physics (https://github.com/jrouwe/JoltPhysics).
//!
//! This file is the root of the `zolt` module. It re-exports the public API in a flat namespace,
//! mirroring the C++ `JPH::` namespace (so `JPH::Vec3` is `zolt.Vec3`). Source files inside the
//! module import each other through relative paths, never through this file.
//!
//! See Docs/Zolt/PortingGuide.md for the C++ -> Zig conventions and Docs/Zolt/Roadmap.md for the plan.

const std = @import("std");

pub const getConfigurationString = @import("ConfigurationString.zig").getConfigurationString;

// AABBTree
pub const AABBTreeBuilder = @import("AABBTree/AABBTreeBuilder.zig").AABBTreeBuilder;
pub const AABBTreeBuilderStats = @import("AABBTree/AABBTreeBuilder.zig").AABBTreeBuilderStats;
pub const AABBTreeToBuffer = @import("AABBTree/AABBTreeToBuffer.zig").AABBTreeToBuffer;
pub const NodeCodecQuadTreeHalfFloat = @import("AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.zig").NodeCodecQuadTreeHalfFloat;
pub const TriangleCodecIndexed8BitPackSOA4Flags = @import("AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.zig").TriangleCodecIndexed8BitPackSOA4Flags;

// Core
pub const atomicMax = @import("Core/Atomics.zig").atomicMax;
pub const atomicMin = @import("Core/Atomics.zig").atomicMin;
pub const binaryHeapPop = @import("Core/BinaryHeap.zig").binaryHeapPop;
pub const binaryHeapPush = @import("Core/BinaryHeap.zig").binaryHeapPush;
pub const ByteBuffer = @import("Core/ByteBuffer.zig").ByteBuffer;
pub const ByteBufferVector = @import("Core/ByteBuffer.zig").ByteBufferVector;
pub const Color = @import("Core/Color.zig").Color;
pub const Core = @import("Core/Core.zig");
pub const FixedSizeFreeList = @import("Core/FixedSizeFreeList.zig").FixedSizeFreeList;
pub const FPControlWord = @import("Core/FPControlWord.zig").FPControlWord;
pub const FPFlushDenormals = @import("Core/FPFlushDenormals.zig").FPFlushDenormals;
pub const HashCombine = @import("Core/HashCombine.zig");
pub const HashTable = @import("Core/HashTable.zig").HashTable;
pub const HashTableOptions = @import("Core/HashTable.zig").HashTableOptions;
pub const insertionSort = @import("Core/InsertionSort.zig").insertionSort;
pub const JobHandle = @import("Core/JobSystem.zig").JobSystem.JobHandle;
pub const JobSystem = @import("Core/JobSystem.zig").JobSystem;
pub const JobSystemSingleThreaded = @import("Core/JobSystemSingleThreaded.zig").JobSystemSingleThreaded;
pub const JobSystemThreadPool = @import("Core/JobSystemThreadPool.zig").JobSystemThreadPool;
pub const JobSystemWithBarrier = @import("Core/JobSystemWithBarrier.zig").JobSystemWithBarrier;
pub const LinearCurve = @import("Core/LinearCurve.zig").LinearCurve;
pub const LFHMAllocator = @import("Core/LockFreeHashMap.zig").LFHMAllocator;
pub const LFHMAllocatorContext = @import("Core/LockFreeHashMap.zig").LFHMAllocatorContext;
pub const LockFreeHashMap = @import("Core/LockFreeHashMap.zig").LockFreeHashMap;
pub const Mt19937 = @import("Core/Mt19937.zig");
pub const Mutex = @import("Core/Mutex.zig").Mutex;
pub const MutexBase = @import("Core/Mutex.zig").MutexBase;
pub const SharedMutex = @import("Core/Mutex.zig").SharedMutex;
pub const SharedMutexBase = @import("Core/Mutex.zig").SharedMutexBase;
pub const MutexArray = @import("Core/MutexArray.zig").MutexArray;
pub const IDToObjectMap = @import("Core/ObjectToIDMap.zig").IDToObjectMap;
pub const ObjectToIDMap = @import("Core/ObjectToIDMap.zig").ObjectToIDMap;
pub const prefetchL1 = @import("Core/Prefetch.zig").prefetchL1;
pub const quickSort = @import("Core/QuickSort.zig").quickSort;
pub const Ref = @import("Core/Reference.zig").Ref;
pub const RefConst = @import("Core/Reference.zig").RefConst;
pub const RefCount = @import("Core/Reference.zig").RefCount;
pub const RefTargetVirtual = @import("Core/Reference.zig").RefTargetVirtual;
pub const Semaphore = @import("Core/Semaphore.zig").Semaphore;
pub const StaticArray = @import("Core/StaticArray.zig").StaticArray;
pub const STLLocalAllocator = @import("Core/STLLocalAllocator.zig").STLLocalAllocator;
pub const STLTempAllocator = @import("Core/STLTempAllocator.zig").STLTempAllocator;
pub const StreamIn = @import("Core/StreamIn.zig").StreamIn;
pub const StreamOut = @import("Core/StreamOut.zig").StreamOut;
pub const StreamInWrapper = @import("Core/StreamWrapper.zig").StreamInWrapper;
pub const StreamOutWrapper = @import("Core/StreamWrapper.zig").StreamOutWrapper;
pub const StridedPtr = @import("Core/StridedPtr.zig").StridedPtr;
pub const StridedPtrConst = @import("Core/StridedPtr.zig").StridedPtrConst;
pub const convertToString = @import("Core/StringTools.zig").convertToString;
pub const nibbleToBinary = @import("Core/StringTools.zig").nibbleToBinary;
pub const stringFormat = @import("Core/StringTools.zig").stringFormat;
pub const stringReplace = @import("Core/StringTools.zig").stringReplace;
pub const stringToVector = @import("Core/StringTools.zig").stringToVector;
pub const toLower = @import("Core/StringTools.zig").toLower;
pub const vectorToString = @import("Core/StringTools.zig").vectorToString;
pub const TempAllocator = @import("Core/TempAllocator.zig").TempAllocator;
pub const TempAllocatorImpl = @import("Core/TempAllocator.zig").TempAllocatorImpl;
pub const TempAllocatorImplWithMallocFallback = @import("Core/TempAllocator.zig").TempAllocatorImplWithMallocFallback;
pub const TempAllocatorMalloc = @import("Core/TempAllocator.zig").TempAllocatorMalloc;
pub const getProcessorTickCount = @import("Core/TickCounter.zig").getProcessorTickCount;
pub const UnorderedMap = @import("Core/UnorderedMap.zig").UnorderedMap;
pub const UnorderedMapDetail = @import("Core/UnorderedMap.zig").UnorderedMapDetail;
pub const UnorderedSet = @import("Core/UnorderedSet.zig").UnorderedSet;
pub const UnorderedSetDetail = @import("Core/UnorderedSet.zig").UnorderedSetDetail;

// Geometry
pub const AABox = @import("Geometry/AABox.zig").AABox;
pub const AABox4Bounds = @import("Geometry/AABox4.zig").AABox4Bounds;
pub const aabox4DistanceSqToPoint = @import("Geometry/AABox4.zig").aabox4DistanceSqToPoint;
pub const aabox4DistanceSqToPointVec3 = @import("Geometry/AABox4.zig").aabox4DistanceSqToPointVec3;
pub const aabox4EnlargeWithExtent = @import("Geometry/AABox4.zig").aabox4EnlargeWithExtent;
pub const AABox4OrientedBoxOptions = @import("Geometry/AABox4.zig").AABox4OrientedBoxOptions;
pub const aabox4Scale = @import("Geometry/AABox4.zig").aabox4Scale;
pub const aabox4VsBox = @import("Geometry/AABox4.zig").aabox4VsBox;
pub const aabox4VsOrientedBox = @import("Geometry/AABox4.zig").aabox4VsOrientedBox;
pub const aabox4VsOrientedBoxMat44 = @import("Geometry/AABox4.zig").aabox4VsOrientedBoxMat44;
pub const aabox4VsPoint = @import("Geometry/AABox4.zig").aabox4VsPoint;
pub const aabox4VsSphere = @import("Geometry/AABox4.zig").aabox4VsSphere;
pub const aabox4VsSphereVec3 = @import("Geometry/AABox4.zig").aabox4VsSphereVec3;
pub const clipPolyVsAABox = @import("Geometry/ClipPoly.zig").clipPolyVsAABox;
pub const clipPolyVsEdge = @import("Geometry/ClipPoly.zig").clipPolyVsEdge;
pub const clipPolyVsPlane = @import("Geometry/ClipPoly.zig").clipPolyVsPlane;
pub const clipPolyVsPoly = @import("Geometry/ClipPoly.zig").clipPolyVsPoly;
pub const ClosestPoint = @import("Geometry/ClosestPoint.zig");
pub const ConvexHullBuilder2D = @import("Geometry/ConvexHullBuilder2D.zig").ConvexHullBuilder2D;
pub const AddConvexRadius = @import("Geometry/ConvexSupport.zig").AddConvexRadius;
pub const MinkowskiDifference = @import("Geometry/ConvexSupport.zig").MinkowskiDifference;
pub const PointConvexSupport = @import("Geometry/ConvexSupport.zig").PointConvexSupport;
pub const PolygonConvexSupport = @import("Geometry/ConvexSupport.zig").PolygonConvexSupport;
pub const TransformedConvexObject = @import("Geometry/ConvexSupport.zig").TransformedConvexObject;
pub const TriangleConvexSupport = @import("Geometry/ConvexSupport.zig").TriangleConvexSupport;
pub const Ellipse = @import("Geometry/Ellipse.zig").Ellipse;
pub const IndexedTriangle = @import("Geometry/IndexedTriangle.zig").IndexedTriangle;
pub const IndexedTriangleList = @import("Geometry/IndexedTriangle.zig").IndexedTriangleList;
pub const IndexedTriangleNoMaterial = @import("Geometry/IndexedTriangle.zig").IndexedTriangleNoMaterial;
pub const IndexedTriangleNoMaterialList = @import("Geometry/IndexedTriangle.zig").IndexedTriangleNoMaterialList;
pub const deindexify = @import("Geometry/Indexify.zig").deindexify;
pub const indexify = @import("Geometry/Indexify.zig").indexify;
pub const IndexifyOptions = @import("Geometry/Indexify.zig").IndexifyOptions;
pub const MortonCode = @import("Geometry/MortonCode.zig").MortonCode;
pub const OrientedBox = @import("Geometry/OrientedBox.zig").OrientedBox;
pub const Plane = @import("Geometry/Plane.zig").Plane;
pub const rayAABox = @import("Geometry/RayAABox.zig").rayAABox;
pub const rayAABox4 = @import("Geometry/RayAABox.zig").rayAABox4;
pub const rayAABoxHits = @import("Geometry/RayAABox.zig").rayAABoxHits;
pub const rayAABoxHitsDirection = @import("Geometry/RayAABox.zig").rayAABoxHitsDirection;
pub const RayAABoxMinMax = @import("Geometry/RayAABox.zig").RayAABoxMinMax;
pub const rayAABoxMinMax = @import("Geometry/RayAABox.zig").rayAABoxMinMax;
pub const RayInvDirection = @import("Geometry/RayAABox.zig").RayInvDirection;
pub const rayCapsule = @import("Geometry/RayCapsule.zig").rayCapsule;
pub const rayCylinder = @import("Geometry/RayCylinder.zig").rayCylinder;
pub const rayInfiniteCylinder = @import("Geometry/RayCylinder.zig").rayInfiniteCylinder;
pub const raySphere = @import("Geometry/RaySphere.zig").raySphere;
pub const RaySphereMinMax = @import("Geometry/RaySphere.zig").RaySphereMinMax;
pub const raySphereMinMax = @import("Geometry/RaySphere.zig").raySphereMinMax;
pub const rayTriangle = @import("Geometry/RayTriangle.zig").rayTriangle;
pub const rayTriangle4 = @import("Geometry/RayTriangle.zig").rayTriangle4;
pub const Sphere = @import("Geometry/Sphere.zig").Sphere;
pub const Triangle = @import("Geometry/Triangle.zig").Triangle;
pub const TriangleList = @import("Geometry/Triangle.zig").TriangleList;
pub const VertexArray = @import("Geometry/VertexArray.zig");
pub const VertexArrayList = @import("Geometry/VertexArray.zig").VertexArrayList;

// Math
pub const BVec16 = @import("Math/BVec16.zig").BVec16;
pub const DMat44 = @import("Math/DMat44.zig").DMat44;
pub const Double3 = @import("Math/Double3.zig").Double3;
pub const DVec3 = @import("Math/DVec3.zig").DVec3;
pub const DynMatrix = @import("Math/DynMatrix.zig").DynMatrix;
pub const eigenValueSymmetric = @import("Math/EigenValueSymmetric.zig").eigenValueSymmetric;
pub const findRoot = @import("Math/FindRoot.zig").findRoot;
pub const FindRootResult = @import("Math/FindRoot.zig").FindRootResult;
pub const Float2 = @import("Math/Float2.zig").Float2;
pub const Float3 = @import("Math/Float3.zig").Float3;
pub const VertexList = @import("Math/Float3.zig").VertexList;
pub const Float4 = @import("Math/Float4.zig").Float4;
pub const gaussianElimination = @import("Math/GaussianElimination.zig").gaussianElimination;
pub const half_float = @import("Math/HalfFloat.zig");
pub const HalfFloat = @import("Math/HalfFloat.zig").HalfFloat;
pub const Mat44 = @import("Math/Mat44.zig").Mat44;
pub const math = @import("Math/Math.zig");
pub const Matrix = @import("Math/Matrix.zig").Matrix;
pub const Quat = @import("Math/Quat.zig").Quat;
pub const Real = @import("Math/Real.zig").Real;
pub const Real3 = @import("Math/Real.zig").Real3;
pub const RMat44 = @import("Math/Real.zig").RMat44;
pub const RVec3 = @import("Math/Real.zig").RVec3;
pub const rvector_alignment = @import("Math/Real.zig").rvector_alignment;
pub const Swizzle = @import("Math/Swizzle.zig").Swizzle;
pub const trigonometry = @import("Math/Trigonometry.zig");
pub const UVec4 = @import("Math/UVec4.zig").UVec4;
pub const Vec3 = @import("Math/Vec3.zig").Vec3;
pub const Vec4 = @import("Math/Vec4.zig").Vec4;
pub const Vector = @import("Math/Vector.zig").Vector;

// TriangleSplitter
pub const TriangleSplitter = @import("TriangleSplitter/TriangleSplitter.zig").TriangleSplitter;
pub const TriangleSplitterBinning = @import("TriangleSplitter/TriangleSplitterBinning.zig").TriangleSplitterBinning;
pub const TriangleSplitterMean = @import("TriangleSplitter/TriangleSplitterMean.zig").TriangleSplitterMean;

/// Every source file of the module. Used by the test below to make sure that all of them are
/// compiled and that their inline tests run. Add new files here when porting them.
const source_files = .{
    @import("AABBTree/AABBTreeBuilder.zig"),
    @import("AABBTree/AABBTreeToBuffer.zig"),
    @import("AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.zig"),
    @import("AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.zig"),
    @import("ConfigurationString.zig"),
    @import("Core/Atomics.zig"),
    @import("Core/BinaryHeap.zig"),
    @import("Core/ByteBuffer.zig"),
    @import("Core/Color.zig"),
    @import("Core/Core.zig"),
    @import("Core/FixedSizeFreeList.zig"),
    @import("Core/FPControlWord.zig"),
    @import("Core/FPFlushDenormals.zig"),
    @import("Core/HashCombine.zig"),
    @import("Core/HashTable.zig"),
    @import("Core/InsertionSort.zig"),
    @import("Core/JobSystem.zig"),
    @import("Core/JobSystemSingleThreaded.zig"),
    @import("Core/JobSystemThreadPool.zig"),
    @import("Core/JobSystemWithBarrier.zig"),
    @import("Core/LinearCurve.zig"),
    @import("Core/LockFreeHashMap.zig"),
    @import("Core/Mt19937.zig"),
    @import("Core/Mutex.zig"),
    @import("Core/MutexArray.zig"),
    @import("Core/ObjectToIDMap.zig"),
    @import("Core/Prefetch.zig"),
    @import("Core/QuickSort.zig"),
    @import("Core/Reference.zig"),
    @import("Core/Semaphore.zig"),
    @import("Core/StaticArray.zig"),
    @import("Core/STLLocalAllocator.zig"),
    @import("Core/STLTempAllocator.zig"),
    @import("Core/StreamIn.zig"),
    @import("Core/StreamOut.zig"),
    @import("Core/StreamWrapper.zig"),
    @import("Core/StridedPtr.zig"),
    @import("Core/StringTools.zig"),
    @import("Core/TempAllocator.zig"),
    @import("Core/TickCounter.zig"),
    @import("Core/UnorderedMap.zig"),
    @import("Core/UnorderedSet.zig"),
    @import("Geometry/AABox.zig"),
    @import("Geometry/AABox4.zig"),
    @import("Geometry/ClipPoly.zig"),
    @import("Geometry/ClosestPoint.zig"),
    @import("Geometry/ConvexHullBuilder2D.zig"),
    @import("Geometry/ConvexSupport.zig"),
    @import("Geometry/Ellipse.zig"),
    @import("Geometry/IndexedTriangle.zig"),
    @import("Geometry/Indexify.zig"),
    @import("Geometry/MortonCode.zig"),
    @import("Geometry/OrientedBox.zig"),
    @import("Geometry/Plane.zig"),
    @import("Geometry/RayAABox.zig"),
    @import("Geometry/RayCapsule.zig"),
    @import("Geometry/RayCylinder.zig"),
    @import("Geometry/RaySphere.zig"),
    @import("Geometry/RayTriangle.zig"),
    @import("Geometry/Sphere.zig"),
    @import("Geometry/Triangle.zig"),
    @import("Geometry/VertexArray.zig"),
    @import("Math/BVec16.zig"),
    @import("Math/DMat44.zig"),
    @import("Math/Double3.zig"),
    @import("Math/DVec3.zig"),
    @import("Math/DynMatrix.zig"),
    @import("Math/EigenValueSymmetric.zig"),
    @import("Math/FindRoot.zig"),
    @import("Math/Float2.zig"),
    @import("Math/Float3.zig"),
    @import("Math/Float4.zig"),
    @import("Math/GaussianElimination.zig"),
    @import("Math/HalfFloat.zig"),
    @import("Math/Mat44.zig"),
    @import("Math/Math.zig"),
    @import("Math/Matrix.zig"),
    @import("Math/Quat.zig"),
    @import("Math/Real.zig"),
    @import("Math/Swizzle.zig"),
    @import("Math/Trigonometry.zig"),
    @import("Math/UVec4.zig"),
    @import("Math/Vec3.zig"),
    @import("Math/Vec4.zig"),
    @import("Math/Vector.zig"),
    @import("TriangleSplitter/TriangleSplitter.zig"),
    @import("TriangleSplitter/TriangleSplitterBinning.zig"),
    @import("TriangleSplitter/TriangleSplitterMean.zig"),
};

test {
    // Zig only analyzes code that is referenced. Reference every public declaration (recursively
    // into nested types) so that ported functions that are not called by any test yet still get
    // type checked. Generic functions (comptime / anytype parameters) can only be checked by calling them.
    inline for (source_files) |file| {
        refAllDeclsRecursive(file);
    }
}

fn refAllDeclsRecursive(comptime T: type) void {
    @setEvalBranchQuota(100_000);
    inline for (comptime std.meta.declarations(T)) |decl| {
        const value = @field(T, decl.name);
        if (@TypeOf(value) == type) {
            switch (@typeInfo(value)) {
                .@"struct", .@"enum", .@"union", .@"opaque" => {
                    // Only recurse into types declared inside T, aliases to other types are checked by their own file
                    if (comptime std.mem.startsWith(u8, @typeName(value), @typeName(T) ++ "."))
                        refAllDeclsRecursive(value);
                },
                else => {},
            }
        }
        _ = &@field(T, decl.name);
    }
}
