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
pub const PlacementBuffer = @import("Core/PlacementBuffer.zig").PlacementBuffer;
pub const PlacementBufferOptions = @import("Core/PlacementBuffer.zig").Options;
pub const prefetchL1 = @import("Core/Prefetch.zig").prefetchL1;
pub const quickSort = @import("Core/QuickSort.zig").quickSort;
pub const Ref = @import("Core/Reference.zig").Ref;
pub const RefConst = @import("Core/Reference.zig").RefConst;
pub const RefCount = @import("Core/Reference.zig").RefCount;
pub const RefTargetVirtual = @import("Core/Reference.zig").RefTargetVirtual;
pub const ErrorString = @import("Core/Result.zig").ErrorString;
pub const Result = @import("Core/Result.zig").Result;
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
pub const virtual = @import("Core/Virtual.zig");

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
pub const ConvexHullBuilder = @import("Geometry/ConvexHullBuilder.zig").ConvexHullBuilder;
pub const ConvexHullBuilder2D = @import("Geometry/ConvexHullBuilder2D.zig").ConvexHullBuilder2D;
pub const AddConvexRadius = @import("Geometry/ConvexSupport.zig").AddConvexRadius;
pub const MinkowskiDifference = @import("Geometry/ConvexSupport.zig").MinkowskiDifference;
pub const PointConvexSupport = @import("Geometry/ConvexSupport.zig").PointConvexSupport;
pub const PolygonConvexSupport = @import("Geometry/ConvexSupport.zig").PolygonConvexSupport;
pub const TransformedConvexObject = @import("Geometry/ConvexSupport.zig").TransformedConvexObject;
pub const TriangleConvexSupport = @import("Geometry/ConvexSupport.zig").TriangleConvexSupport;
pub const Ellipse = @import("Geometry/Ellipse.zig").Ellipse;
pub const EPAConvexHullBuilder = @import("Geometry/EPAConvexHullBuilder.zig").EPAConvexHullBuilder;
pub const EPAPenetrationDepth = @import("Geometry/EPAPenetrationDepth.zig").EPAPenetrationDepth;
pub const GJKClosestPoint = @import("Geometry/GJKClosestPoint.zig").GJKClosestPoint;
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

// Physics
pub const Body = @import("Physics/Body/Body.zig").Body;
pub const BodyID = @import("Physics/Body/BodyID.zig").BodyID;
pub const MassProperties = @import("Physics/Body/MassProperties.zig").MassProperties;
pub const AABoxCast = @import("Physics/Collision/AABoxCast.zig").AABoxCast;
pub const ActiveEdgeMode = @import("Physics/Collision/ActiveEdgeMode.zig").ActiveEdgeMode;
pub const ActiveEdges = @import("Physics/Collision/ActiveEdges.zig");
pub const BackFaceMode = @import("Physics/Collision/BackFaceMode.zig").BackFaceMode;
pub const broad_phase_layer_invalid = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").broad_phase_layer_invalid;
pub const BroadPhaseLayer = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayer;
pub const BroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerFilter;
pub const BroadPhaseLayerInterface = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerInterface;
pub const DefaultBroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").DefaultBroadPhaseLayerFilter;
pub const ObjectVsBroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").ObjectVsBroadPhaseLayerFilter;
pub const SpecifiedBroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").SpecifiedBroadPhaseLayerFilter;
pub const BroadPhaseLayerInterfaceMask = @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.zig").BroadPhaseLayerInterfaceMask;
pub const BroadPhaseLayerInterfaceTable = @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
pub const ObjectVsBroadPhaseLayerFilterMask = @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.zig").ObjectVsBroadPhaseLayerFilterMask;
pub const ObjectVsBroadPhaseLayerFilterTable = @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
pub const CastConvexVsTriangles = @import("Physics/Collision/CastConvexVsTriangles.zig").CastConvexVsTriangles;
pub const BroadPhaseCastResult = @import("Physics/Collision/CastResult.zig").BroadPhaseCastResult;
pub const RayCastResult = @import("Physics/Collision/CastResult.zig").RayCastResult;
pub const CastSphereVsTriangles = @import("Physics/Collision/CastSphereVsTriangles.zig").CastSphereVsTriangles;
pub const CollectFacesMode = @import("Physics/Collision/CollectFacesMode.zig").CollectFacesMode;
pub const CollideConvexVsTriangles = @import("Physics/Collision/CollideConvexVsTriangles.zig").CollideConvexVsTriangles;
pub const CollidePointResult = @import("Physics/Collision/CollidePointResult.zig").CollidePointResult;
pub const CollideSettingsBase = @import("Physics/Collision/CollideShape.zig").CollideSettingsBase;
pub const CollideShapeResult = @import("Physics/Collision/CollideShape.zig").CollideShapeResult;
pub const CollideShapeSettings = @import("Physics/Collision/CollideShape.zig").CollideShapeSettings;
pub const collideShapeVsShapePerLeaf = @import("Physics/Collision/CollideShapeVsShapePerLeaf.zig").collideShapeVsShapePerLeaf;
pub const CollideSoftBodyVertexIterator = @import("Physics/Collision/CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
pub const CollideSoftBodyVerticesVsTriangles = @import("Physics/Collision/CollideSoftBodyVerticesVsTriangles.zig").CollideSoftBodyVerticesVsTriangles;
pub const CollideSphereVsTriangles = @import("Physics/Collision/CollideSphereVsTriangles.zig").CollideSphereVsTriangles;
pub const CollisionCollector = @import("Physics/Collision/CollisionCollector.zig").CollisionCollector;
pub const CollisionCollectorTraitsCastRay = @import("Physics/Collision/CollisionCollector.zig").CollisionCollectorTraitsCastRay;
pub const CollisionCollectorTraitsCastShape = @import("Physics/Collision/CollisionCollector.zig").CollisionCollectorTraitsCastShape;
pub const CollisionCollectorTraitsCollidePoint = @import("Physics/Collision/CollisionCollector.zig").CollisionCollectorTraitsCollidePoint;
pub const CollisionCollectorTraitsCollideShape = @import("Physics/Collision/CollisionCollector.zig").CollisionCollectorTraitsCollideShape;
pub const AllHitCollisionCollector = @import("Physics/Collision/CollisionCollectorImpl.zig").AllHitCollisionCollector;
pub const AllocationErrorLatch = @import("Physics/Collision/CollisionCollectorImpl.zig").AllocationErrorLatch;
pub const AnyHitCollisionCollector = @import("Physics/Collision/CollisionCollectorImpl.zig").AnyHitCollisionCollector;
pub const ClosestHitCollisionCollector = @import("Physics/Collision/CollisionCollectorImpl.zig").ClosestHitCollisionCollector;
pub const ClosestHitPerBodyCollisionCollector = @import("Physics/Collision/CollisionCollectorImpl.zig").ClosestHitPerBodyCollisionCollector;
pub const CollisionDispatch = @import("Physics/Collision/CollisionDispatch.zig");
pub const CollisionGroup = @import("Physics/Collision/CollisionGroup.zig").CollisionGroup;
pub const ContactListener = @import("Physics/Collision/ContactListener.zig").ContactListener;
pub const ContactManifold = @import("Physics/Collision/ContactListener.zig").ContactManifold;
pub const ContactPoints = @import("Physics/Collision/ContactListener.zig").ContactPoints;
pub const ContactSettings = @import("Physics/Collision/ContactListener.zig").ContactSettings;
pub const ValidateResult = @import("Physics/Collision/ContactListener.zig").ValidateResult;
pub const GroupFilter = @import("Physics/Collision/GroupFilter.zig").GroupFilter;
pub const GroupFilterResult = @import("Physics/Collision/GroupFilter.zig").GroupFilter.GroupFilterResult;
pub const GroupFilterTable = @import("Physics/Collision/GroupFilterTable.zig").GroupFilterTable;
pub const InternalEdgeRemovingCollector = @import("Physics/Collision/InternalEdgeRemovingCollector.zig").InternalEdgeRemovingCollector;
pub const manifoldBetweenTwoFaces = @import("Physics/Collision/ManifoldBetweenTwoFaces.zig").manifoldBetweenTwoFaces;
pub const pruneContactPoints = @import("Physics/Collision/ManifoldBetweenTwoFaces.zig").pruneContactPoints;
pub const NarrowPhaseStat = @import("Physics/Collision/NarrowPhaseStats.zig").NarrowPhaseStat;
pub const track_narrowphase_stats = @import("Physics/Collision/NarrowPhaseStats.zig").track_narrowphase_stats;
pub const TrackNarrowPhaseCollector = @import("Physics/Collision/NarrowPhaseStats.zig").TrackNarrowPhaseCollector;
pub const TrackNarrowPhaseStat = @import("Physics/Collision/NarrowPhaseStats.zig").TrackNarrowPhaseStat;
pub const DefaultObjectLayerFilter = @import("Physics/Collision/ObjectLayer.zig").DefaultObjectLayerFilter;
pub const object_layer_invalid = @import("Physics/Collision/ObjectLayer.zig").object_layer_invalid;
pub const ObjectLayer = @import("Physics/Collision/ObjectLayer.zig").ObjectLayer;
pub const ObjectLayerFilter = @import("Physics/Collision/ObjectLayer.zig").ObjectLayerFilter;
pub const ObjectLayerPairFilter = @import("Physics/Collision/ObjectLayer.zig").ObjectLayerPairFilter;
pub const SpecifiedObjectLayerFilter = @import("Physics/Collision/ObjectLayer.zig").SpecifiedObjectLayerFilter;
pub const ObjectLayerPairFilterMask = @import("Physics/Collision/ObjectLayerPairFilterMask.zig").ObjectLayerPairFilterMask;
pub const ObjectLayerPairFilterTable = @import("Physics/Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
pub const PhysicsMaterial = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
pub const PhysicsMaterialList = @import("Physics/Collision/PhysicsMaterial.zig").PhysicsMaterialList;
pub const PhysicsMaterialSimple = @import("Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
pub const RayCast = @import("Physics/Collision/RayCast.zig").RayCast;
pub const RayCastKind = @import("Physics/Collision/RayCast.zig").RayCastKind;
pub const RayCastSettings = @import("Physics/Collision/RayCast.zig").RayCastSettings;
pub const RayCastT = @import("Physics/Collision/RayCast.zig").RayCastT;
pub const RRayCast = @import("Physics/Collision/RayCast.zig").RRayCast;
pub const BoxShape = @import("Physics/Collision/Shape/BoxShape.zig").BoxShape;
pub const BoxShapeSettings = @import("Physics/Collision/Shape/BoxShape.zig").BoxShapeSettings;
pub const CapsuleShape = @import("Physics/Collision/Shape/CapsuleShape.zig").CapsuleShape;
pub const CapsuleShapeSettings = @import("Physics/Collision/Shape/CapsuleShape.zig").CapsuleShapeSettings;
pub const CompoundShape = @import("Physics/Collision/Shape/CompoundShape.zig").CompoundShape;
pub const CompoundShapeSettings = @import("Physics/Collision/Shape/CompoundShape.zig").CompoundShapeSettings;
pub const ConvexHullShape = @import("Physics/Collision/Shape/ConvexHullShape.zig").ConvexHullShape;
pub const ConvexShape = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShape;
pub const ConvexShapeSettings = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShapeSettings;
pub const CylinderShape = @import("Physics/Collision/Shape/CylinderShape.zig").CylinderShape;
pub const CylinderShapeSettings = @import("Physics/Collision/Shape/CylinderShape.zig").CylinderShapeSettings;
pub const DecoratedShape = @import("Physics/Collision/Shape/DecoratedShape.zig").DecoratedShape;
pub const DecoratedShapeSettings = @import("Physics/Collision/Shape/DecoratedShape.zig").DecoratedShapeSettings;
pub const EmptyShape = @import("Physics/Collision/Shape/EmptyShape.zig").EmptyShape;
pub const GetTrianglesContextMultiVertexList = @import("Physics/Collision/Shape/GetTrianglesContext.zig").GetTrianglesContextMultiVertexList;
pub const GetTrianglesContextVertexList = @import("Physics/Collision/Shape/GetTrianglesContext.zig").GetTrianglesContextVertexList;
pub const HeightFieldShape = @import("Physics/Collision/Shape/HeightFieldShape.zig").HeightFieldShape;
pub const HeightFieldShapeConstants = @import("Physics/Collision/Shape/HeightFieldShape.zig").HeightFieldShapeConstants;
pub const HeightFieldShapeSettings = @import("Physics/Collision/Shape/HeightFieldShape.zig").HeightFieldShapeSettings;
pub const MeshShape = @import("Physics/Collision/Shape/MeshShape.zig").MeshShape;
pub const MeshShapeSettings = @import("Physics/Collision/Shape/MeshShape.zig").MeshShapeSettings;
pub const MutableCompoundShape = @import("Physics/Collision/Shape/MutableCompoundShape.zig").MutableCompoundShape;
pub const MutableCompoundShapeSettings = @import("Physics/Collision/Shape/MutableCompoundShape.zig").MutableCompoundShapeSettings;
pub const OffsetCenterOfMassShape = @import("Physics/Collision/Shape/OffsetCenterOfMassShape.zig").OffsetCenterOfMassShape;
pub const OffsetCenterOfMassShapeSettings = @import("Physics/Collision/Shape/OffsetCenterOfMassShape.zig").OffsetCenterOfMassShapeSettings;
pub const PlaneShape = @import("Physics/Collision/Shape/PlaneShape.zig").PlaneShape;
pub const PolyhedronSubmergedVolumeCalculator = @import("Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.zig").PolyhedronSubmergedVolumeCalculator;
pub const RotatedTranslatedShape = @import("Physics/Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShape;
pub const RotatedTranslatedShapeSettings = @import("Physics/Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShapeSettings;
pub const ScaledShape = @import("Physics/Collision/Shape/ScaledShape.zig").ScaledShape;
pub const ScaledShapeSettings = @import("Physics/Collision/Shape/ScaledShape.zig").ScaledShapeSettings;
pub const ScaleHelpers = @import("Physics/Collision/Shape/ScaleHelpers.zig");
pub const all_sub_shape_types = @import("Physics/Collision/Shape/Shape.zig").all_sub_shape_types;
pub const CastRayCollector = @import("Physics/Collision/Shape/Shape.zig").CastRayCollector;
pub const CastShapeCollector = @import("Physics/Collision/Shape/Shape.zig").CastShapeCollector;
pub const CollidePointCollector = @import("Physics/Collision/Shape/Shape.zig").CollidePointCollector;
pub const CollideShapeCollector = @import("Physics/Collision/Shape/Shape.zig").CollideShapeCollector;
pub const compound_sub_shape_types = @import("Physics/Collision/Shape/Shape.zig").compound_sub_shape_types;
pub const convex_sub_shape_types = @import("Physics/Collision/Shape/Shape.zig").convex_sub_shape_types;
pub const decorator_sub_shape_types = @import("Physics/Collision/Shape/Shape.zig").decorator_sub_shape_types;
pub const num_sub_shape_types = @import("Physics/Collision/Shape/Shape.zig").num_sub_shape_types;
pub const PhysicsMaterialRefC = @import("Physics/Collision/Shape/Shape.zig").PhysicsMaterialRefC;
pub const Shape = @import("Physics/Collision/Shape/Shape.zig").Shape;
pub const ShapeFunctions = @import("Physics/Collision/Shape/Shape.zig").ShapeFunctions;
pub const ShapeList = @import("Physics/Collision/Shape/Shape.zig").ShapeList;
pub const ShapeRefC = @import("Physics/Collision/Shape/Shape.zig").ShapeRefC;
pub const ShapeResult = @import("Physics/Collision/Shape/Shape.zig").ShapeResult;
pub const ShapeSettings = @import("Physics/Collision/Shape/Shape.zig").ShapeSettings;
pub const ShapeSubType = @import("Physics/Collision/Shape/Shape.zig").ShapeSubType;
pub const ShapeType = @import("Physics/Collision/Shape/Shape.zig").ShapeType;
pub const sub_shape_type_names = @import("Physics/Collision/Shape/Shape.zig").sub_shape_type_names;
pub const TransformedShapeCollector = @import("Physics/Collision/Shape/Shape.zig").TransformedShapeCollector;
pub const SphereShape = @import("Physics/Collision/Shape/SphereShape.zig").SphereShape;
pub const SphereShapeSettings = @import("Physics/Collision/Shape/SphereShape.zig").SphereShapeSettings;
pub const StaticCompoundShape = @import("Physics/Collision/Shape/StaticCompoundShape.zig").StaticCompoundShape;
pub const StaticCompoundShapeSettings = @import("Physics/Collision/Shape/StaticCompoundShape.zig").StaticCompoundShapeSettings;
pub const SubShapeID = @import("Physics/Collision/Shape/SubShapeID.zig").SubShapeID;
pub const SubShapeIDCreator = @import("Physics/Collision/Shape/SubShapeID.zig").SubShapeIDCreator;
pub const SubShapeIDPair = @import("Physics/Collision/Shape/SubShapeIDPair.zig").SubShapeIDPair;
pub const TaperedCapsuleShape = @import("Physics/Collision/Shape/TaperedCapsuleShape.zig").TaperedCapsuleShape;
pub const TaperedCapsuleShapeSettings = @import("Physics/Collision/Shape/TaperedCapsuleShape.zig").TaperedCapsuleShapeSettings;
pub const TaperedCylinderShape = @import("Physics/Collision/Shape/TaperedCylinderShape.zig").TaperedCylinderShape;
pub const TaperedCylinderShapeSettings = @import("Physics/Collision/Shape/TaperedCylinderShape.zig").TaperedCylinderShapeSettings;
pub const TriangleShape = @import("Physics/Collision/Shape/TriangleShape.zig").TriangleShape;
pub const TriangleShapeSettings = @import("Physics/Collision/Shape/TriangleShape.zig").TriangleShapeSettings;
pub const RShapeCast = @import("Physics/Collision/ShapeCast.zig").RShapeCast;
pub const ShapeCast = @import("Physics/Collision/ShapeCast.zig").ShapeCast;
pub const ShapeCastKind = @import("Physics/Collision/ShapeCast.zig").ShapeCastKind;
pub const ShapeCastResult = @import("Physics/Collision/ShapeCast.zig").ShapeCastResult;
pub const ShapeCastSettings = @import("Physics/Collision/ShapeCast.zig").ShapeCastSettings;
pub const ShapeCastT = @import("Physics/Collision/ShapeCast.zig").ShapeCastT;
pub const ReversedShapeFilter = @import("Physics/Collision/ShapeFilter.zig").ReversedShapeFilter;
pub const ShapeFilter = @import("Physics/Collision/ShapeFilter.zig").ShapeFilter;
pub const SimShapeFilter = @import("Physics/Collision/SimShapeFilter.zig").SimShapeFilter;
pub const countAndSortTrues = @import("Physics/Collision/SortReverseAndStore.zig").countAndSortTrues;
pub const sortReverseAndStore = @import("Physics/Collision/SortReverseAndStore.zig").sortReverseAndStore;
pub const TransformedShape = @import("Physics/Collision/TransformedShape.zig").TransformedShape;
pub const physics_settings = @import("Physics/PhysicsSettings.zig");
pub const PhysicsSettings = @import("Physics/PhysicsSettings.zig").PhysicsSettings;

pub const SoftBodyShape = @import("Physics/SoftBody/SoftBodyShape.zig").SoftBodyShape;

// RegisterTypes
pub const RegisterTypes = @import("RegisterTypes.zig");

/// Test shapes of the inline tests of the shape core, registered through `zolt_user_types` by build.zig (only in test builds)
pub const test_shapes = if (@import("builtin").is_test) @import("Physics/Collision/Shape/TestShapes.zig") else struct {};

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
    @import("Core/PlacementBuffer.zig"),
    @import("Core/Prefetch.zig"),
    @import("Core/QuickSort.zig"),
    @import("Core/Reference.zig"),
    @import("Core/Result.zig"),
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
    @import("Core/Virtual.zig"),
    @import("Geometry/AABox.zig"),
    @import("Geometry/AABox4.zig"),
    @import("Geometry/ClipPoly.zig"),
    @import("Geometry/ClosestPoint.zig"),
    @import("Geometry/ConvexHullBuilder.zig"),
    @import("Geometry/ConvexHullBuilder2D.zig"),
    @import("Geometry/ConvexSupport.zig"),
    @import("Geometry/Ellipse.zig"),
    @import("Geometry/EPAConvexHullBuilder.zig"),
    @import("Geometry/EPAPenetrationDepth.zig"),
    @import("Geometry/GJKClosestPoint.zig"),
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
    @import("Physics/Body/Body.zig"),
    @import("Physics/Body/BodyID.zig"),
    @import("Physics/Body/MassProperties.zig"),
    @import("Physics/Collision/AABoxCast.zig"),
    @import("Physics/Collision/ActiveEdgeMode.zig"),
    @import("Physics/Collision/ActiveEdges.zig"),
    @import("Physics/Collision/BackFaceMode.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig"),
    @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.zig"),
    @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig"),
    @import("Physics/Collision/CastConvexVsTriangles.zig"),
    @import("Physics/Collision/CastResult.zig"),
    @import("Physics/Collision/CastSphereVsTriangles.zig"),
    @import("Physics/Collision/CollectFacesMode.zig"),
    @import("Physics/Collision/CollideConvexVsTriangles.zig"),
    @import("Physics/Collision/CollidePointResult.zig"),
    @import("Physics/Collision/CollideShape.zig"),
    @import("Physics/Collision/CollideShapeVsShapePerLeaf.zig"),
    @import("Physics/Collision/CollideSoftBodyVertexIterator.zig"),
    @import("Physics/Collision/CollideSoftBodyVerticesVsTriangles.zig"),
    @import("Physics/Collision/CollideSphereVsTriangles.zig"),
    @import("Physics/Collision/CollisionCollector.zig"),
    @import("Physics/Collision/CollisionCollectorImpl.zig"),
    @import("Physics/Collision/CollisionDispatch.zig"),
    @import("Physics/Collision/CollisionGroup.zig"),
    @import("Physics/Collision/ContactListener.zig"),
    @import("Physics/Collision/GroupFilter.zig"),
    @import("Physics/Collision/GroupFilterTable.zig"),
    @import("Physics/Collision/InternalEdgeRemovingCollector.zig"),
    @import("Physics/Collision/ManifoldBetweenTwoFaces.zig"),
    @import("Physics/Collision/NarrowPhaseStats.zig"),
    @import("Physics/Collision/ObjectLayer.zig"),
    @import("Physics/Collision/ObjectLayerPairFilterMask.zig"),
    @import("Physics/Collision/ObjectLayerPairFilterTable.zig"),
    @import("Physics/Collision/PhysicsMaterial.zig"),
    @import("Physics/Collision/PhysicsMaterialSimple.zig"),
    @import("Physics/Collision/RayCast.zig"),
    @import("Physics/Collision/Shape/BoxShape.zig"),
    @import("Physics/Collision/Shape/CapsuleShape.zig"),
    @import("Physics/Collision/Shape/CompoundShape.zig"),
    @import("Physics/Collision/Shape/CompoundShapeVisitors.zig"),
    @import("Physics/Collision/Shape/ConvexHullShape.zig"),
    @import("Physics/Collision/Shape/ConvexShape.zig"),
    @import("Physics/Collision/Shape/CylinderShape.zig"),
    @import("Physics/Collision/Shape/DecoratedShape.zig"),
    @import("Physics/Collision/Shape/EmptyShape.zig"),
    @import("Physics/Collision/Shape/GetTrianglesContext.zig"),
    @import("Physics/Collision/Shape/HeightFieldShape.zig"),
    @import("Physics/Collision/Shape/MeshShape.zig"),
    @import("Physics/Collision/Shape/MutableCompoundShape.zig"),
    @import("Physics/Collision/Shape/OffsetCenterOfMassShape.zig"),
    @import("Physics/Collision/Shape/PlaneShape.zig"),
    @import("Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.zig"),
    @import("Physics/Collision/Shape/RotatedTranslatedShape.zig"),
    @import("Physics/Collision/Shape/ScaledShape.zig"),
    @import("Physics/Collision/Shape/ScaleHelpers.zig"),
    @import("Physics/Collision/Shape/Shape.zig"),
    @import("Physics/Collision/Shape/SphereShape.zig"),
    @import("Physics/Collision/Shape/StaticCompoundShape.zig"),
    @import("Physics/Collision/Shape/SubShapeID.zig"),
    @import("Physics/Collision/Shape/SubShapeIDPair.zig"),
    @import("Physics/Collision/Shape/TaperedCapsuleShape.zig"),
    @import("Physics/Collision/Shape/TaperedCylinderShape.zig"),
    @import("Physics/Collision/Shape/TestShapes.zig"),
    @import("Physics/Collision/Shape/TriangleShape.zig"),
    @import("Physics/Collision/ShapeCast.zig"),
    @import("Physics/Collision/ShapeFilter.zig"),
    @import("Physics/Collision/SimShapeFilter.zig"),
    @import("Physics/Collision/SortReverseAndStore.zig"),
    @import("Physics/Collision/TransformedShape.zig"),
    @import("Physics/PhysicsSettings.zig"),
    @import("Physics/SoftBody/SoftBodyShape.zig"),
    @import("RegisterTypes.zig"),
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
