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

// Compute
pub const ComputeBuffer = @import("Compute/ComputeBuffer.zig").ComputeBuffer;
pub const ComputeQueue = @import("Compute/ComputeQueue.zig").ComputeQueue;
pub const ComputeShader = @import("Compute/ComputeShader.zig").ComputeShader;
pub const ComputeSystem = @import("Compute/ComputeSystem.zig").ComputeSystem;
pub const createComputeSystem = @import("Compute/ComputeSystem.zig").createComputeSystem;
pub const ComputeBufferCPU = @import("Compute/CPU/ComputeBufferCPU.zig").ComputeBufferCPU;
pub const ComputeQueueCPU = @import("Compute/CPU/ComputeQueueCPU.zig").ComputeQueueCPU;
pub const ComputeShaderCPU = @import("Compute/CPU/ComputeShaderCPU.zig").ComputeShaderCPU;
pub const ComputeSystemCPU = @import("Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
pub const HLSLToCPP = @import("Compute/CPU/HLSLToCPP.zig");
pub const ShaderWrapper = @import("Compute/CPU/ShaderWrapper.zig").ShaderWrapper;

// Core
pub const atomicMax = @import("Core/Atomics.zig").atomicMax;
pub const atomicMin = @import("Core/Atomics.zig").atomicMin;
pub const binaryHeapPop = @import("Core/BinaryHeap.zig").binaryHeapPop;
pub const binaryHeapPush = @import("Core/BinaryHeap.zig").binaryHeapPush;
pub const ByteBuffer = @import("Core/ByteBuffer.zig").ByteBuffer;
pub const ByteBufferVector = @import("Core/ByteBuffer.zig").ByteBufferVector;
pub const Color = @import("Core/Color.zig").Color;
pub const Core = @import("Core/Core.zig");
pub const Factory = @import("Core/Factory.zig").Factory;
pub const FixedSizeFreeList = @import("Core/FixedSizeFreeList.zig").FixedSizeFreeList;
pub const FPControlWord = @import("Core/FPControlWord.zig").FPControlWord;
pub const FPFlushDenormals = @import("Core/FPFlushDenormals.zig").FPFlushDenormals;
pub const HashCombine = @import("Core/HashCombine.zig");
pub const HashTable = @import("Core/HashTable.zig").HashTable;
pub const HashTableOptions = @import("Core/HashTable.zig").HashTableOptions;
pub const insertionSort = @import("Core/InsertionSort.zig").insertionSort;
pub const AssertHandler = @import("Core/IssueReporting.zig").AssertHandler;
pub const ExpectAssert = @import("Core/IssueReporting.zig").ExpectAssert;
pub const IssueReporting = @import("Core/IssueReporting.zig");
pub const FailingJobSystem = @import("Core/JobSystem.zig").FailingJobSystem;
pub const JobHandle = @import("Core/JobSystem.zig").JobSystem.JobHandle;
pub const JobSystem = @import("Core/JobSystem.zig").JobSystem;
pub const JobSystemSingleThreaded = @import("Core/JobSystemSingleThreaded.zig").JobSystemSingleThreaded;
pub const JobSystemThreadPool = @import("Core/JobSystemThreadPool.zig").JobSystemThreadPool;
pub const JobSystemWithBarrier = @import("Core/JobSystemWithBarrier.zig").JobSystemWithBarrier;
pub const Lifetime = @import("Core/Lifetime.zig");
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
pub const RTTI = @import("Core/RTTI.zig").RTTI;
pub const Semaphore = @import("Core/Semaphore.zig").Semaphore;
pub const StaticArray = @import("Core/StaticArray.zig").StaticArray;
pub const STLLocalAllocator = @import("Core/STLLocalAllocator.zig").STLLocalAllocator;
pub const STLTempAllocator = @import("Core/STLTempAllocator.zig").STLTempAllocator;
pub const StreamIn = @import("Core/StreamIn.zig").StreamIn;
pub const StreamOut = @import("Core/StreamOut.zig").StreamOut;
pub const StreamUtils = @import("Core/StreamUtils.zig");
pub const StreamInWrapper = @import("Core/StreamWrapper.zig").StreamInWrapper;
pub const StreamOutWrapper = @import("Core/StreamWrapper.zig").StreamOutWrapper;
pub const StridedPtr = @import("Core/StridedPtr.zig").StridedPtr;
pub const StridedPtrConst = @import("Core/StridedPtr.zig").StridedPtrConst;
pub const convertToString = @import("Core/StringTools.zig").convertToString;
pub const CppFixed = @import("Core/StringTools.zig").CppFixed;
pub const cppFixed = @import("Core/StringTools.zig").cppFixed;
pub const CppFloat = @import("Core/StringTools.zig").CppFloat;
pub const cppFloat = @import("Core/StringTools.zig").cppFloat;
pub const nibbleToBinary = @import("Core/StringTools.zig").nibbleToBinary;
pub const stringFormat = @import("Core/StringTools.zig").stringFormat;
pub const stringReplace = @import("Core/StringTools.zig").stringReplace;
pub const stringToVector = @import("Core/StringTools.zig").stringToVector;
pub const toLower = @import("Core/StringTools.zig").toLower;
pub const vectorToString = @import("Core/StringTools.zig").vectorToString;
pub const writeFloatFixed = @import("Core/StringTools.zig").writeFloatFixed;
pub const writeFloatGeneral = @import("Core/StringTools.zig").writeFloatGeneral;
pub const writeFloatPrecision = @import("Core/StringTools.zig").writeFloatPrecision;
pub const FailingTempAllocator = @import("Core/TempAllocator.zig").FailingTempAllocator;
pub const TempAllocator = @import("Core/TempAllocator.zig").TempAllocator;
pub const TempAllocatorImpl = @import("Core/TempAllocator.zig").TempAllocatorImpl;
pub const TempAllocatorImplWithMallocFallback = @import("Core/TempAllocator.zig").TempAllocatorImplWithMallocFallback;
pub const TempAllocatorMalloc = @import("Core/TempAllocator.zig").TempAllocatorMalloc;
pub const TempArray = @import("Core/TempAllocator.zig").TempArray;
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
pub const LibM = @import("Math/LibM.zig");
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

// ObjectStream
pub const ObjectStream = @import("ObjectStream/ObjectStream.zig");
pub const ObjectStreamIn = @import("ObjectStream/ObjectStreamIn.zig").ObjectStreamIn;
pub const ObjectStreamOut = @import("ObjectStream/ObjectStreamOut.zig").ObjectStreamOut;
pub const AttributeDecl = @import("ObjectStream/SerializableAttribute.zig").AttributeDecl;
pub const OSDataType = @import("ObjectStream/SerializableAttribute.zig").OSDataType;
pub const SerializableAttribute = @import("ObjectStream/SerializableAttribute.zig").SerializableAttribute;
pub const SerializableObject = @import("ObjectStream/SerializableObject.zig").SerializableObject;

// Physics
pub const AllowedDOFs = @import("Physics/Body/AllowedDOFs.zig").AllowedDOFs;
pub const Body = @import("Physics/Body/Body.zig").Body;
pub const BodyAccess = @import("Physics/Body/BodyAccess.zig").BodyAccess;
pub const BodyActivationListener = @import("Physics/Body/BodyActivationListener.zig").BodyActivationListener;
pub const BodyCreationSettings = @import("Physics/Body/BodyCreationSettings.zig").BodyCreationSettings;
pub const CreateBodyError = @import("Physics/Body/BodyCreationSettings.zig").CreateBodyError;
pub const OverrideMassProperties = @import("Physics/Body/BodyCreationSettings.zig").OverrideMassProperties;
pub const BodyDrawFilter = @import("Physics/Body/BodyFilter.zig").BodyDrawFilter;
pub const BodyFilter = @import("Physics/Body/BodyFilter.zig").BodyFilter;
pub const IgnoreMultipleBodiesFilter = @import("Physics/Body/BodyFilter.zig").IgnoreMultipleBodiesFilter;
pub const IgnoreSingleBodyFilter = @import("Physics/Body/BodyFilter.zig").IgnoreSingleBodyFilter;
pub const IgnoreSingleBodyFilterChained = @import("Physics/Body/BodyFilter.zig").IgnoreSingleBodyFilterChained;
pub const BodyID = @import("Physics/Body/BodyID.zig").BodyID;
pub const BodyInterface = @import("Physics/Body/BodyInterface.zig").BodyInterface;
pub const BodyLockBase = @import("Physics/Body/BodyLock.zig").BodyLockBase;
pub const BodyLockRead = @import("Physics/Body/BodyLock.zig").BodyLockRead;
pub const BodyLockWrite = @import("Physics/Body/BodyLock.zig").BodyLockWrite;
pub const BodyLockInterface = @import("Physics/Body/BodyLockInterface.zig").BodyLockInterface;
pub const BodyLockMultiBase = @import("Physics/Body/BodyLockMulti.zig").BodyLockMultiBase;
pub const BodyLockMultiRead = @import("Physics/Body/BodyLockMulti.zig").BodyLockMultiRead;
pub const BodyLockMultiWrite = @import("Physics/Body/BodyLockMulti.zig").BodyLockMultiWrite;
pub const BodyIDVector = @import("Physics/Body/BodyManager.zig").BodyIDVector;
pub const BodyManager = @import("Physics/Body/BodyManager.zig").BodyManager;
pub const BodyVector = @import("Physics/Body/BodyManager.zig").BodyVector;
pub const BodyWithMotionProperties = @import("Physics/Body/BodyManager.zig").BodyWithMotionProperties;
pub const SoftBodyConstraintColor = @import("Physics/Body/BodyManager.zig").SoftBodyConstraintColor;
pub const BodyPair = @import("Physics/Body/BodyPair.zig").BodyPair;
pub const body_type_count = @import("Physics/Body/BodyType.zig").body_type_count;
pub const BodyType = @import("Physics/Body/BodyType.zig").BodyType;
pub const MassProperties = @import("Physics/Body/MassProperties.zig").MassProperties;
pub const CanSleep = @import("Physics/Body/MotionProperties.zig").CanSleep;
pub const MotionProperties = @import("Physics/Body/MotionProperties.zig").MotionProperties;
pub const track_simulation_stats = @import("Physics/Body/MotionProperties.zig").track_simulation_stats;
pub const MotionQuality = @import("Physics/Body/MotionQuality.zig").MotionQuality;
pub const MotionType = @import("Physics/Body/MotionType.zig").MotionType;
pub const Character = @import("Physics/Character/Character.zig").Character;
pub const CharacterSettings = @import("Physics/Character/Character.zig").CharacterSettings;
pub const CharacterBase = @import("Physics/Character/CharacterBase.zig").CharacterBase;
pub const CharacterBaseSettings = @import("Physics/Character/CharacterBase.zig").CharacterBaseSettings;
pub const CharacterID = @import("Physics/Character/CharacterID.zig").CharacterID;
pub const CharacterContact = @import("Physics/Character/CharacterVirtual.zig").CharacterContact;
pub const CharacterContactKey = @import("Physics/Character/CharacterVirtual.zig").CharacterContactKey;
pub const CharacterContactListener = @import("Physics/Character/CharacterVirtual.zig").CharacterContactListener;
pub const CharacterContactSettings = @import("Physics/Character/CharacterVirtual.zig").CharacterContactSettings;
pub const CharacterVirtual = @import("Physics/Character/CharacterVirtual.zig").CharacterVirtual;
pub const CharacterVirtualSettings = @import("Physics/Character/CharacterVirtual.zig").CharacterVirtualSettings;
pub const CharacterVsCharacterCollision = @import("Physics/Character/CharacterVirtual.zig").CharacterVsCharacterCollision;
pub const CharacterVsCharacterCollisionSimple = @import("Physics/Character/CharacterVirtual.zig").CharacterVsCharacterCollisionSimple;
pub const AABoxCast = @import("Physics/Collision/AABoxCast.zig").AABoxCast;
pub const ActiveEdgeMode = @import("Physics/Collision/ActiveEdgeMode.zig").ActiveEdgeMode;
pub const ActiveEdges = @import("Physics/Collision/ActiveEdges.zig");
pub const BackFaceMode = @import("Physics/Collision/BackFaceMode.zig").BackFaceMode;
pub const BodyPairCollector = @import("Physics/Collision/BroadPhase/BroadPhase.zig").BodyPairCollector;
pub const BroadPhase = @import("Physics/Collision/BroadPhase/BroadPhase.zig").BroadPhase;
pub const BroadPhaseBruteForce = @import("Physics/Collision/BroadPhase/BroadPhaseBruteForce.zig").BroadPhaseBruteForce;
pub const broad_phase_layer_invalid = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").broad_phase_layer_invalid;
pub const BroadPhaseLayer = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayer;
pub const BroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerFilter;
pub const BroadPhaseLayerInterface = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerInterface;
pub const DefaultBroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").DefaultBroadPhaseLayerFilter;
pub const ObjectVsBroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").ObjectVsBroadPhaseLayerFilter;
pub const SpecifiedBroadPhaseLayerFilter = @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig").SpecifiedBroadPhaseLayerFilter;
pub const BroadPhaseLayerInterfaceMask = @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.zig").BroadPhaseLayerInterfaceMask;
pub const BroadPhaseLayerInterfaceTable = @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
pub const BroadPhaseQuadTree = @import("Physics/Collision/BroadPhase/BroadPhaseQuadTree.zig").BroadPhaseQuadTree;
pub const BroadPhaseQuery = @import("Physics/Collision/BroadPhase/BroadPhaseQuery.zig").BroadPhaseQuery;
pub const BroadPhaseQueryFilters = @import("Physics/Collision/BroadPhase/BroadPhaseQuery.zig").QueryFilters;
pub const CastShapeBodyCollector = @import("Physics/Collision/BroadPhase/BroadPhaseQuery.zig").CastShapeBodyCollector;
pub const CollideShapeBodyCollector = @import("Physics/Collision/BroadPhase/BroadPhaseQuery.zig").CollideShapeBodyCollector;
pub const RayCastBodyCollector = @import("Physics/Collision/BroadPhase/BroadPhaseQuery.zig").RayCastBodyCollector;
pub const ObjectVsBroadPhaseLayerFilterMask = @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.zig").ObjectVsBroadPhaseLayerFilterMask;
pub const ObjectVsBroadPhaseLayerFilterTable = @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
pub const QuadTree = @import("Physics/Collision/BroadPhase/QuadTree.zig").QuadTree;
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
pub const CollisionEstimationResult = @import("Physics/Collision/EstimateCollisionResponse.zig").CollisionEstimationResult;
pub const estimateCollisionResponse = @import("Physics/Collision/EstimateCollisionResponse.zig").estimateCollisionResponse;
pub const GroupFilter = @import("Physics/Collision/GroupFilter.zig").GroupFilter;
pub const GroupFilterResult = @import("Physics/Collision/GroupFilter.zig").GroupFilter.GroupFilterResult;
pub const GroupFilterTable = @import("Physics/Collision/GroupFilterTable.zig").GroupFilterTable;
pub const InternalEdgeRemovingCollector = @import("Physics/Collision/InternalEdgeRemovingCollector.zig").InternalEdgeRemovingCollector;
pub const manifoldBetweenTwoFaces = @import("Physics/Collision/ManifoldBetweenTwoFaces.zig").manifoldBetweenTwoFaces;
pub const pruneContactPoints = @import("Physics/Collision/ManifoldBetweenTwoFaces.zig").pruneContactPoints;
pub const NarrowPhaseQuery = @import("Physics/Collision/NarrowPhaseQuery.zig").NarrowPhaseQuery;
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
pub const ConvexHullShapeSettings = @import("Physics/Collision/Shape/ConvexHullShape.zig").ConvexHullShapeSettings;
pub const ConvexShape = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShape;
pub const ConvexShapeSettings = @import("Physics/Collision/Shape/ConvexShape.zig").ConvexShapeSettings;
pub const CylinderShape = @import("Physics/Collision/Shape/CylinderShape.zig").CylinderShape;
pub const CylinderShapeSettings = @import("Physics/Collision/Shape/CylinderShape.zig").CylinderShapeSettings;
pub const DecoratedShape = @import("Physics/Collision/Shape/DecoratedShape.zig").DecoratedShape;
pub const DecoratedShapeSettings = @import("Physics/Collision/Shape/DecoratedShape.zig").DecoratedShapeSettings;
pub const EmptyShape = @import("Physics/Collision/Shape/EmptyShape.zig").EmptyShape;
pub const EmptyShapeSettings = @import("Physics/Collision/Shape/EmptyShape.zig").EmptyShapeSettings;
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
pub const PlaneShapeSettings = @import("Physics/Collision/Shape/PlaneShape.zig").PlaneShapeSettings;
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
pub const SimShapeFilterWrapper = @import("Physics/Collision/SimShapeFilterWrapper.zig").SimShapeFilterWrapper;
pub const countAndSortTrues = @import("Physics/Collision/SortReverseAndStore.zig").countAndSortTrues;
pub const sortReverseAndStore = @import("Physics/Collision/SortReverseAndStore.zig").sortReverseAndStore;
pub const TransformedShape = @import("Physics/Collision/TransformedShape.zig").TransformedShape;
pub const CalculateSolverSteps = @import("Physics/Constraints/CalculateSolverSteps.zig").CalculateSolverSteps;
pub const DummyCalculateSolverSteps = @import("Physics/Constraints/CalculateSolverSteps.zig").DummyCalculateSolverSteps;
pub const ConeConstraint = @import("Physics/Constraints/ConeConstraint.zig").ConeConstraint;
pub const ConeConstraintSettings = @import("Physics/Constraints/ConeConstraint.zig").ConeConstraintSettings;
pub const Constraint = @import("Physics/Constraints/Constraint.zig").Constraint;
pub const ConstraintSettings = @import("Physics/Constraints/Constraint.zig").ConstraintSettings;
pub const ConstraintSpace = @import("Physics/Constraints/Constraint.zig").ConstraintSpace;
pub const ConstraintSubType = @import("Physics/Constraints/Constraint.zig").ConstraintSubType;
pub const ConstraintType = @import("Physics/Constraints/Constraint.zig").ConstraintType;
pub const ConstraintManager = @import("Physics/Constraints/ConstraintManager.zig").ConstraintManager;
pub const Constraints = @import("Physics/Constraints/ConstraintManager.zig").Constraints;
pub const AngleConstraintPart = @import("Physics/Constraints/ConstraintPart/AngleConstraintPart.zig").AngleConstraintPart;
pub const AngularFrictionConstraintPart = @import("Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.zig").AngularFrictionConstraintPart;
pub const AngularFrictionConstraintPart1 = @import("Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.zig").AngularFrictionConstraintPart1;
pub const AngularFrictionConstraintPart2 = @import("Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.zig").AngularFrictionConstraintPart2;
pub const AxisConstraintPart = @import("Physics/Constraints/ConstraintPart/AxisConstraintPart.zig").AxisConstraintPart;
pub const ContactConstraintPart = @import("Physics/Constraints/ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart;
pub const ContactConstraintPart1 = @import("Physics/Constraints/ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart1;
pub const ContactConstraintPart2 = @import("Physics/Constraints/ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart2;
pub const DualAxisConstraintPart = @import("Physics/Constraints/ConstraintPart/DualAxisConstraintPart.zig").DualAxisConstraintPart;
pub const GearConstraintPart = @import("Physics/Constraints/ConstraintPart/GearConstraintPart.zig").GearConstraintPart;
pub const HingeRotationConstraintPart = @import("Physics/Constraints/ConstraintPart/HingeRotationConstraintPart.zig").HingeRotationConstraintPart;
pub const IndependentAxisConstraintPart = @import("Physics/Constraints/ConstraintPart/IndependentAxisConstraintPart.zig").IndependentAxisConstraintPart;
pub const PointConstraintPart = @import("Physics/Constraints/ConstraintPart/PointConstraintPart.zig").PointConstraintPart;
pub const RackAndPinionConstraintPart = @import("Physics/Constraints/ConstraintPart/RackAndPinionConstraintPart.zig").RackAndPinionConstraintPart;
pub const RotationEulerConstraintPart = @import("Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.zig").RotationEulerConstraintPart;
pub const RotationQuatConstraintPart = @import("Physics/Constraints/ConstraintPart/RotationQuatConstraintPart.zig").RotationQuatConstraintPart;
pub const SpringPart = @import("Physics/Constraints/ConstraintPart/SpringPart.zig").SpringPart;
pub const SwingTwistConstraintPart = @import("Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.zig").SwingTwistConstraintPart;
pub const SwingType = @import("Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.zig").SwingType;
pub const ContactConstraintManager = @import("Physics/Constraints/ContactConstraintManager.zig").ContactConstraintManager;
pub const DistanceConstraint = @import("Physics/Constraints/DistanceConstraint.zig").DistanceConstraint;
pub const DistanceConstraintSettings = @import("Physics/Constraints/DistanceConstraint.zig").DistanceConstraintSettings;
pub const FixedConstraint = @import("Physics/Constraints/FixedConstraint.zig").FixedConstraint;
pub const FixedConstraintSettings = @import("Physics/Constraints/FixedConstraint.zig").FixedConstraintSettings;
pub const GearConstraint = @import("Physics/Constraints/GearConstraint.zig").GearConstraint;
pub const GearConstraintSettings = @import("Physics/Constraints/GearConstraint.zig").GearConstraintSettings;
pub const HingeConstraint = @import("Physics/Constraints/HingeConstraint.zig").HingeConstraint;
pub const HingeConstraintSettings = @import("Physics/Constraints/HingeConstraint.zig").HingeConstraintSettings;
pub const isPositionMotor = @import("Physics/Constraints/MotorSettings.zig").isPositionMotor;
pub const isVelocityMotor = @import("Physics/Constraints/MotorSettings.zig").isVelocityMotor;
pub const MotorSettings = @import("Physics/Constraints/MotorSettings.zig").MotorSettings;
pub const MotorState = @import("Physics/Constraints/MotorSettings.zig").MotorState;
pub const PathConstraint = @import("Physics/Constraints/PathConstraint.zig").PathConstraint;
pub const PathConstraintSettings = @import("Physics/Constraints/PathConstraint.zig").PathConstraintSettings;
pub const PathRotationConstraintType = @import("Physics/Constraints/PathConstraint.zig").PathRotationConstraintType;
pub const PathConstraintPath = @import("Physics/Constraints/PathConstraintPath.zig").PathConstraintPath;
pub const PathConstraintPathHermite = @import("Physics/Constraints/PathConstraintPathHermite.zig").PathConstraintPathHermite;
pub const PointConstraint = @import("Physics/Constraints/PointConstraint.zig").PointConstraint;
pub const PointConstraintSettings = @import("Physics/Constraints/PointConstraint.zig").PointConstraintSettings;
pub const PulleyConstraint = @import("Physics/Constraints/PulleyConstraint.zig").PulleyConstraint;
pub const PulleyConstraintSettings = @import("Physics/Constraints/PulleyConstraint.zig").PulleyConstraintSettings;
pub const RackAndPinionConstraint = @import("Physics/Constraints/RackAndPinionConstraint.zig").RackAndPinionConstraint;
pub const RackAndPinionConstraintSettings = @import("Physics/Constraints/RackAndPinionConstraint.zig").RackAndPinionConstraintSettings;
pub const SixDOFConstraint = @import("Physics/Constraints/SixDOFConstraint.zig").SixDOFConstraint;
pub const SixDOFConstraintSettings = @import("Physics/Constraints/SixDOFConstraint.zig").SixDOFConstraintSettings;
pub const SliderConstraint = @import("Physics/Constraints/SliderConstraint.zig").SliderConstraint;
pub const SliderConstraintSettings = @import("Physics/Constraints/SliderConstraint.zig").SliderConstraintSettings;
pub const SpringMode = @import("Physics/Constraints/SpringSettings.zig").SpringMode;
pub const SpringSettings = @import("Physics/Constraints/SpringSettings.zig").SpringSettings;
pub const SwingTwistConstraint = @import("Physics/Constraints/SwingTwistConstraint.zig").SwingTwistConstraint;
pub const SwingTwistConstraintSettings = @import("Physics/Constraints/SwingTwistConstraint.zig").SwingTwistConstraintSettings;
pub const TwoBodyConstraint = @import("Physics/Constraints/TwoBodyConstraint.zig").TwoBodyConstraint;
pub const TwoBodyConstraintSettings = @import("Physics/Constraints/TwoBodyConstraint.zig").TwoBodyConstraintSettings;
pub const DeterminismLog = @import("Physics/DeterminismLog.zig");
pub const Activation = @import("Physics/EActivation.zig").Activation;
pub const PhysicsUpdateError = @import("Physics/EPhysicsUpdateError.zig").PhysicsUpdateError;
pub const Hair = @import("Physics/Hair/Hair.zig").Hair;
pub const HairSettings = @import("Physics/Hair/HairSettings.zig").HairSettings;
pub const HairShaders = @import("Physics/Hair/HairShaders.zig").HairShaders;
pub const RegisterHair = @import("Physics/Hair/RegisterHair.zig");
pub const IslandBuilder = @import("Physics/IslandBuilder.zig").IslandBuilder;
pub const LargeIslandSplitter = @import("Physics/LargeIslandSplitter.zig").LargeIslandSplitter;
pub const PhysicsLock = @import("Physics/PhysicsLock.zig").PhysicsLock;
pub const PhysicsLockContext = @import("Physics/PhysicsLock.zig").PhysicsLockContext;
pub const PhysicsLockTypes = @import("Physics/PhysicsLock.zig").PhysicsLockTypes;
pub const SharedLock = @import("Physics/PhysicsLock.zig").SharedLock;
pub const UniqueLock = @import("Physics/PhysicsLock.zig").UniqueLock;
pub const PhysicsScene = @import("Physics/PhysicsScene.zig").PhysicsScene;
pub const physics_settings = @import("Physics/PhysicsSettings.zig");
pub const PhysicsSettings = @import("Physics/PhysicsSettings.zig").PhysicsSettings;
pub const PhysicsStepListener = @import("Physics/PhysicsStepListener.zig").PhysicsStepListener;
pub const PhysicsStepListenerContext = @import("Physics/PhysicsStepListener.zig").PhysicsStepListenerContext;
pub const PhysicsSystem = @import("Physics/PhysicsSystem.zig").PhysicsSystem;
pub const PhysicsUpdateContext = @import("Physics/PhysicsUpdateContext.zig").PhysicsUpdateContext;
pub const Ragdoll = @import("Physics/Ragdoll/Ragdoll.zig").Ragdoll;
pub const RagdollSettings = @import("Physics/Ragdoll/Ragdoll.zig").RagdollSettings;
pub const SoftBodyContactListener = @import("Physics/SoftBody/SoftBodyContactListener.zig").SoftBodyContactListener;
pub const SoftBodyContactSettings = @import("Physics/SoftBody/SoftBodyContactListener.zig").SoftBodyContactSettings;
pub const SoftBodyValidateResult = @import("Physics/SoftBody/SoftBodyContactListener.zig").SoftBodyValidateResult;
pub const SoftBodyCreationSettings = @import("Physics/SoftBody/SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
pub const SoftBodyManifold = @import("Physics/SoftBody/SoftBodyManifold.zig").SoftBodyManifold;
pub const SoftBodyMotionProperties = @import("Physics/SoftBody/SoftBodyMotionProperties.zig").SoftBodyMotionProperties;
pub const SoftBodySharedSettings = @import("Physics/SoftBody/SoftBodySharedSettings.zig").SoftBodySharedSettings;
pub const SoftBodyUpdateContext = @import("Physics/SoftBody/SoftBodyUpdateContext.zig").SoftBodyUpdateContext;
pub const SoftBodyVertex = @import("Physics/SoftBody/SoftBodyVertex.zig").SoftBodyVertex;
pub const StateRecorder = @import("Physics/StateRecorder.zig").StateRecorder;
pub const StateRecorderFilter = @import("Physics/StateRecorder.zig").StateRecorderFilter;
pub const StateRecorderState = @import("Physics/StateRecorder.zig").StateRecorderState;
pub const StateRecorderImpl = @import("Physics/StateRecorderImpl.zig").StateRecorderImpl;
pub const MotorcycleController = @import("Physics/Vehicle/MotorcycleController.zig").MotorcycleController;
pub const MotorcycleControllerSettings = @import("Physics/Vehicle/MotorcycleController.zig").MotorcycleControllerSettings;
pub const TrackedVehicleController = @import("Physics/Vehicle/TrackedVehicleController.zig").TrackedVehicleController;
pub const TrackedVehicleControllerSettings = @import("Physics/Vehicle/TrackedVehicleController.zig").TrackedVehicleControllerSettings;
pub const WheelSettingsTV = @import("Physics/Vehicle/TrackedVehicleController.zig").WheelSettingsTV;
pub const WheelTV = @import("Physics/Vehicle/TrackedVehicleController.zig").WheelTV;
pub const VehicleAntiRollBar = @import("Physics/Vehicle/VehicleAntiRollBar.zig").VehicleAntiRollBar;
pub const VehicleAntiRollBars = @import("Physics/Vehicle/VehicleAntiRollBar.zig").VehicleAntiRollBars;
pub const VehicleCollisionTester = @import("Physics/Vehicle/VehicleCollisionTester.zig").VehicleCollisionTester;
pub const VehicleCollisionTesterCastCylinder = @import("Physics/Vehicle/VehicleCollisionTester.zig").VehicleCollisionTesterCastCylinder;
pub const VehicleCollisionTesterCastSphere = @import("Physics/Vehicle/VehicleCollisionTester.zig").VehicleCollisionTesterCastSphere;
pub const VehicleCollisionTesterRay = @import("Physics/Vehicle/VehicleCollisionTester.zig").VehicleCollisionTesterRay;
pub const VehicleConstraint = @import("Physics/Vehicle/VehicleConstraint.zig").VehicleConstraint;
pub const VehicleConstraintSettings = @import("Physics/Vehicle/VehicleConstraint.zig").VehicleConstraintSettings;
pub const VehicleController = @import("Physics/Vehicle/VehicleController.zig").VehicleController;
pub const VehicleControllerSettings = @import("Physics/Vehicle/VehicleController.zig").VehicleControllerSettings;
pub const VehicleDifferentialSettings = @import("Physics/Vehicle/VehicleDifferential.zig").VehicleDifferentialSettings;
pub const VehicleEngine = @import("Physics/Vehicle/VehicleEngine.zig").VehicleEngine;
pub const VehicleEngineSettings = @import("Physics/Vehicle/VehicleEngine.zig").VehicleEngineSettings;
pub const TrackSide = @import("Physics/Vehicle/VehicleTrack.zig").TrackSide;
pub const VehicleTrack = @import("Physics/Vehicle/VehicleTrack.zig").VehicleTrack;
pub const VehicleTracks = @import("Physics/Vehicle/VehicleTrack.zig").VehicleTracks;
pub const VehicleTrackSettings = @import("Physics/Vehicle/VehicleTrack.zig").VehicleTrackSettings;
pub const TransmissionMode = @import("Physics/Vehicle/VehicleTransmission.zig").TransmissionMode;
pub const VehicleTransmission = @import("Physics/Vehicle/VehicleTransmission.zig").VehicleTransmission;
pub const VehicleTransmissionSettings = @import("Physics/Vehicle/VehicleTransmission.zig").VehicleTransmissionSettings;
pub const Wheel = @import("Physics/Vehicle/Wheel.zig").Wheel;
pub const Wheels = @import("Physics/Vehicle/Wheel.zig").Wheels;
pub const WheelSettings = @import("Physics/Vehicle/Wheel.zig").WheelSettings;
pub const WheeledVehicleController = @import("Physics/Vehicle/WheeledVehicleController.zig").WheeledVehicleController;
pub const WheeledVehicleControllerSettings = @import("Physics/Vehicle/WheeledVehicleController.zig").WheeledVehicleControllerSettings;
pub const WheelSettingsWV = @import("Physics/Vehicle/WheeledVehicleController.zig").WheelSettingsWV;
pub const WheelWV = @import("Physics/Vehicle/WheeledVehicleController.zig").WheelWV;

pub const SoftBodyShape = @import("Physics/SoftBody/SoftBodyShape.zig").SoftBodyShape;

// RegisterTypes
pub const RegisterTypes = @import("RegisterTypes.zig");

/// Test shapes of the inline tests of the shape core, registered through `zolt_user_types` by build.zig (only in test builds)
pub const test_shapes = if (@import("builtin").is_test) @import("Physics/Collision/Shape/TestShapes.zig") else struct {};

/// Test constraint of the inline tests and the unit tests, its settings are registered through `zolt_user_types` by
/// build.zig for the inline tests (only in test builds)
pub const test_constraints = if (@import("builtin").is_test) @import("Physics/Constraints/TestConstraints.zig") else struct {};

// Renderer
pub const DebugRenderer = @import("Renderer/DebugRenderer.zig").DebugRenderer;
pub const DebugRendererPlayback = @import("Renderer/DebugRendererPlayback.zig").DebugRendererPlayback;
pub const DebugRendererRecorder = @import("Renderer/DebugRendererRecorder.zig").DebugRendererRecorder;
pub const DebugRendererSimple = @import("Renderer/DebugRendererSimple.zig").DebugRendererSimple;

// Shaders (the namespaces of the shader files, Docs/Zolt/RemainingFeaturesArchitecture.md R12)
pub const HairApplyDeltaTransform = @import("Shaders/HairApplyDeltaTransform.zig");
pub const HairApplyGlobalPose = @import("Shaders/HairApplyGlobalPose.zig");
pub const HairCalculateCollisionPlanes = @import("Shaders/HairCalculateCollisionPlanes.zig");
pub const HairCalculateRenderPositions = @import("Shaders/HairCalculateRenderPositions.zig");
pub const HairCommon = @import("Shaders/HairCommon.zig");
pub const HairGridAccumulate = @import("Shaders/HairGridAccumulate.zig");
pub const HairGridClear = @import("Shaders/HairGridClear.zig");
pub const HairGridNormalize = @import("Shaders/HairGridNormalize.zig");
pub const HairIntegrate = @import("Shaders/HairIntegrate.zig");
pub const HairSkinRoots = @import("Shaders/HairSkinRoots.zig");
pub const HairSkinVertices = @import("Shaders/HairSkinVertices.zig");
pub const HairStructs = @import("Shaders/HairStructs.zig");
pub const HairTeleport = @import("Shaders/HairTeleport.zig");
pub const HairUpdateRoots = @import("Shaders/HairUpdateRoots.zig");
pub const HairUpdateStrands = @import("Shaders/HairUpdateStrands.zig");
pub const HairUpdateVelocity = @import("Shaders/HairUpdateVelocity.zig");
pub const HairUpdateVelocityIntegrate = @import("Shaders/HairUpdateVelocityIntegrate.zig");
pub const registerHairShaders = @import("Shaders/HairWrapper.zig").registerHairShaders;
pub const ShaderCore = @import("Shaders/ShaderCore.zig");
pub const ShaderMat44 = @import("Shaders/ShaderMat44.zig");
pub const ShaderMath = @import("Shaders/ShaderMath.zig");
pub const ShaderPlane = @import("Shaders/ShaderPlane.zig");
pub const ShaderQuat = @import("Shaders/ShaderQuat.zig");
pub const ShaderVec3 = @import("Shaders/ShaderVec3.zig");
pub const TestCompute = @import("Shaders/TestCompute.zig");
pub const TestCompute2 = @import("Shaders/TestCompute2.zig");
pub const registerTestCompute = @import("Shaders/TestComputeWrapper.zig").registerTestCompute;
pub const registerTestCompute2 = @import("Shaders/TestComputeWrapper.zig").registerTestCompute2;

// Skeleton
pub const SkeletalAnimation = @import("Skeleton/SkeletalAnimation.zig").SkeletalAnimation;
pub const Skeleton = @import("Skeleton/Skeleton.zig").Skeleton;
pub const SkeletonMapper = @import("Skeleton/SkeletonMapper.zig").SkeletonMapper;
pub const SkeletonPose = @import("Skeleton/SkeletonPose.zig").SkeletonPose;

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
    @import("Compute/ComputeBuffer.zig"),
    @import("Compute/ComputeQueue.zig"),
    @import("Compute/ComputeShader.zig"),
    @import("Compute/ComputeSystem.zig"),
    @import("Compute/CPU/ComputeBufferCPU.zig"),
    @import("Compute/CPU/ComputeQueueCPU.zig"),
    @import("Compute/CPU/ComputeShaderCPU.zig"),
    @import("Compute/CPU/ComputeSystemCPU.zig"),
    @import("Compute/CPU/HLSLToCPP.zig"),
    @import("Compute/CPU/ShaderWrapper.zig"),
    @import("ConfigurationString.zig"),
    @import("Core/Atomics.zig"),
    @import("Core/BinaryHeap.zig"),
    @import("Core/ByteBuffer.zig"),
    @import("Core/Color.zig"),
    @import("Core/Core.zig"),
    @import("Core/Factory.zig"),
    @import("Core/FixedSizeFreeList.zig"),
    @import("Core/FPControlWord.zig"),
    @import("Core/FPFlushDenormals.zig"),
    @import("Core/HashCombine.zig"),
    @import("Core/HashTable.zig"),
    @import("Core/InsertionSort.zig"),
    @import("Core/IssueReporting.zig"),
    @import("Core/JobSystem.zig"),
    @import("Core/JobSystemSingleThreaded.zig"),
    @import("Core/JobSystemThreadPool.zig"),
    @import("Core/JobSystemWithBarrier.zig"),
    @import("Core/Lifetime.zig"),
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
    @import("Core/RTTI.zig"),
    @import("Core/Semaphore.zig"),
    @import("Core/StaticArray.zig"),
    @import("Core/STLLocalAllocator.zig"),
    @import("Core/STLTempAllocator.zig"),
    @import("Core/StreamIn.zig"),
    @import("Core/StreamOut.zig"),
    @import("Core/StreamUtils.zig"),
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
    @import("Math/LibM.zig"),
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
    @import("ObjectStream/GetPrimitiveTypeOfType.zig"),
    @import("ObjectStream/ObjectStream.zig"),
    @import("ObjectStream/ObjectStreamBinaryIn.zig"),
    @import("ObjectStream/ObjectStreamBinaryOut.zig"),
    @import("ObjectStream/ObjectStreamIn.zig"),
    @import("ObjectStream/ObjectStreamOut.zig"),
    @import("ObjectStream/ObjectStreamTextIn.zig"),
    @import("ObjectStream/ObjectStreamTextOut.zig"),
    @import("ObjectStream/ObjectStreamTypes.zig"),
    @import("ObjectStream/SerializableAttribute.zig"),
    @import("ObjectStream/SerializableAttributeEnum.zig"),
    @import("ObjectStream/SerializableAttributeTyped.zig"),
    @import("ObjectStream/SerializableObject.zig"),
    @import("ObjectStream/TypeDeclarations.zig"),
    @import("Physics/Body/AllowedDOFs.zig"),
    @import("Physics/Body/Body.zig"),
    @import("Physics/Body/BodyAccess.zig"),
    @import("Physics/Body/BodyActivationListener.zig"),
    @import("Physics/Body/BodyCreationSettings.zig"),
    @import("Physics/Body/BodyFilter.zig"),
    @import("Physics/Body/BodyID.zig"),
    @import("Physics/Body/BodyInterface.zig"),
    @import("Physics/Body/BodyLock.zig"),
    @import("Physics/Body/BodyLockInterface.zig"),
    @import("Physics/Body/BodyLockMulti.zig"),
    @import("Physics/Body/BodyManager.zig"),
    @import("Physics/Body/BodyPair.zig"),
    @import("Physics/Body/BodyType.zig"),
    @import("Physics/Body/MassProperties.zig"),
    @import("Physics/Body/MotionProperties.zig"),
    @import("Physics/Body/MotionQuality.zig"),
    @import("Physics/Body/MotionType.zig"),
    @import("Physics/Character/Character.zig"),
    @import("Physics/Character/CharacterBase.zig"),
    @import("Physics/Character/CharacterID.zig"),
    @import("Physics/Character/CharacterVirtual.zig"),
    @import("Physics/Collision/AABoxCast.zig"),
    @import("Physics/Collision/ActiveEdgeMode.zig"),
    @import("Physics/Collision/ActiveEdges.zig"),
    @import("Physics/Collision/BackFaceMode.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhase.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseBruteForce.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseLayer.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseQuadTree.zig"),
    @import("Physics/Collision/BroadPhase/BroadPhaseQuery.zig"),
    @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.zig"),
    @import("Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig"),
    @import("Physics/Collision/BroadPhase/QuadTree.zig"),
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
    @import("Physics/Collision/EstimateCollisionResponse.zig"),
    @import("Physics/Collision/GroupFilter.zig"),
    @import("Physics/Collision/GroupFilterTable.zig"),
    @import("Physics/Collision/InternalEdgeRemovingCollector.zig"),
    @import("Physics/Collision/ManifoldBetweenTwoFaces.zig"),
    @import("Physics/Collision/NarrowPhaseQuery.zig"),
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
    @import("Physics/Collision/SimShapeFilterWrapper.zig"),
    @import("Physics/Collision/SortReverseAndStore.zig"),
    @import("Physics/Collision/TransformedShape.zig"),
    @import("Physics/Constraints/CalculateSolverSteps.zig"),
    @import("Physics/Constraints/ConeConstraint.zig"),
    @import("Physics/Constraints/Constraint.zig"),
    @import("Physics/Constraints/ConstraintManager.zig"),
    @import("Physics/Constraints/ConstraintPart/AngleConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/AxisConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/ConstraintPartTestContext.zig"),
    @import("Physics/Constraints/ConstraintPart/ContactConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/DualAxisConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/GearConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/HingeRotationConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/IndependentAxisConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/PointConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/RackAndPinionConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/RotationQuatConstraintPart.zig"),
    @import("Physics/Constraints/ConstraintPart/SpringPart.zig"),
    @import("Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.zig"),
    @import("Physics/Constraints/ContactConstraintManager.zig"),
    @import("Physics/Constraints/DistanceConstraint.zig"),
    @import("Physics/Constraints/FixedConstraint.zig"),
    @import("Physics/Constraints/GearConstraint.zig"),
    @import("Physics/Constraints/HingeConstraint.zig"),
    @import("Physics/Constraints/MotorSettings.zig"),
    @import("Physics/Constraints/PathConstraint.zig"),
    @import("Physics/Constraints/PathConstraintPath.zig"),
    @import("Physics/Constraints/PathConstraintPathHermite.zig"),
    @import("Physics/Constraints/PointConstraint.zig"),
    @import("Physics/Constraints/PulleyConstraint.zig"),
    @import("Physics/Constraints/RackAndPinionConstraint.zig"),
    @import("Physics/Constraints/SixDOFConstraint.zig"),
    @import("Physics/Constraints/SliderConstraint.zig"),
    @import("Physics/Constraints/SpringSettings.zig"),
    @import("Physics/Constraints/SwingTwistConstraint.zig"),
    @import("Physics/Constraints/TestConstraints.zig"),
    @import("Physics/Constraints/TwoBodyConstraint.zig"),
    @import("Physics/DeterminismLog.zig"),
    @import("Physics/EActivation.zig"),
    @import("Physics/EPhysicsUpdateError.zig"),
    @import("Physics/Hair/Hair.zig"),
    @import("Physics/Hair/HairSettings.zig"),
    @import("Physics/Hair/HairShaders.zig"),
    @import("Physics/Hair/RegisterHair.zig"),
    @import("Physics/IslandBuilder.zig"),
    @import("Physics/LargeIslandSplitter.zig"),
    @import("Physics/PhysicsLock.zig"),
    @import("Physics/PhysicsScene.zig"),
    @import("Physics/PhysicsSettings.zig"),
    @import("Physics/PhysicsStepListener.zig"),
    @import("Physics/PhysicsSystem.zig"),
    @import("Physics/PhysicsUpdateContext.zig"),
    @import("Physics/Ragdoll/Ragdoll.zig"),
    @import("Physics/SoftBody/SoftBodyContactListener.zig"),
    @import("Physics/SoftBody/SoftBodyCreationSettings.zig"),
    @import("Physics/SoftBody/SoftBodyManifold.zig"),
    @import("Physics/SoftBody/SoftBodyMotionProperties.zig"),
    @import("Physics/SoftBody/SoftBodyShape.zig"),
    @import("Physics/SoftBody/SoftBodySharedSettings.zig"),
    @import("Physics/SoftBody/SoftBodyUpdateContext.zig"),
    @import("Physics/SoftBody/SoftBodyVertex.zig"),
    @import("Physics/StateRecorder.zig"),
    @import("Physics/StateRecorderImpl.zig"),
    @import("Physics/Vehicle/MotorcycleController.zig"),
    @import("Physics/Vehicle/TrackedVehicleController.zig"),
    @import("Physics/Vehicle/VehicleAntiRollBar.zig"),
    @import("Physics/Vehicle/VehicleCollisionTester.zig"),
    @import("Physics/Vehicle/VehicleConstraint.zig"),
    @import("Physics/Vehicle/VehicleController.zig"),
    @import("Physics/Vehicle/VehicleDifferential.zig"),
    @import("Physics/Vehicle/VehicleEngine.zig"),
    @import("Physics/Vehicle/VehicleTrack.zig"),
    @import("Physics/Vehicle/VehicleTransmission.zig"),
    @import("Physics/Vehicle/Wheel.zig"),
    @import("Physics/Vehicle/WheeledVehicleController.zig"),
    @import("RegisterTypes.zig"),
    @import("Renderer/DebugRenderer.zig"),
    @import("Renderer/DebugRendererPlayback.zig"),
    @import("Renderer/DebugRendererRecorder.zig"),
    @import("Renderer/DebugRendererSimple.zig"),
    @import("Shaders/HairApplyDeltaTransform.zig"),
    @import("Shaders/HairApplyGlobalPose.zig"),
    @import("Shaders/HairCalculateCollisionPlanes.zig"),
    @import("Shaders/HairCalculateRenderPositions.zig"),
    @import("Shaders/HairCommon.zig"),
    @import("Shaders/HairGridAccumulate.zig"),
    @import("Shaders/HairGridClear.zig"),
    @import("Shaders/HairGridNormalize.zig"),
    @import("Shaders/HairIntegrate.zig"),
    @import("Shaders/HairSkinRoots.zig"),
    @import("Shaders/HairSkinVertices.zig"),
    @import("Shaders/HairStructs.zig"),
    @import("Shaders/HairTeleport.zig"),
    @import("Shaders/HairUpdateRoots.zig"),
    @import("Shaders/HairUpdateStrands.zig"),
    @import("Shaders/HairUpdateVelocity.zig"),
    @import("Shaders/HairUpdateVelocityIntegrate.zig"),
    @import("Shaders/HairWrapper.zig"),
    @import("Shaders/ShaderCore.zig"),
    @import("Shaders/ShaderMat44.zig"),
    @import("Shaders/ShaderMath.zig"),
    @import("Shaders/ShaderPlane.zig"),
    @import("Shaders/ShaderQuat.zig"),
    @import("Shaders/ShaderVec3.zig"),
    @import("Shaders/TestCompute.zig"),
    @import("Shaders/TestCompute2.zig"),
    @import("Shaders/TestComputeWrapper.zig"),
    @import("Skeleton/SkeletalAnimation.zig"),
    @import("Skeleton/Skeleton.zig"),
    @import("Skeleton/SkeletonMapper.zig"),
    @import("Skeleton/SkeletonPose.zig"),
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
