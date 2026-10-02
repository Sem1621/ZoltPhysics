//! Zolt: a Zig port of Jolt Physics (https://github.com/jrouwe/JoltPhysics).
//!
//! This file is the root of the `zolt` module. It re-exports the public API in a flat namespace,
//! mirroring the C++ `JPH::` namespace (so `JPH::Vec3` is `zolt.Vec3`). Source files inside the
//! module import each other through relative paths, never through this file.
//!
//! See Docs/Zolt/PortingGuide.md for the C++ -> Zig conventions and Docs/Zolt/Roadmap.md for the plan.

const std = @import("std");

pub const getConfigurationString = @import("ConfigurationString.zig").getConfigurationString;

// Core
pub const atomicMax = @import("Core/Atomics.zig").atomicMax;
pub const atomicMin = @import("Core/Atomics.zig").atomicMin;
pub const binaryHeapPop = @import("Core/BinaryHeap.zig").binaryHeapPop;
pub const binaryHeapPush = @import("Core/BinaryHeap.zig").binaryHeapPush;
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
pub const StreamIn = @import("Core/StreamIn.zig").StreamIn;
pub const StreamOut = @import("Core/StreamOut.zig").StreamOut;
pub const StreamInWrapper = @import("Core/StreamWrapper.zig").StreamInWrapper;
pub const StreamOutWrapper = @import("Core/StreamWrapper.zig").StreamOutWrapper;
pub const convertToString = @import("Core/StringTools.zig").convertToString;
pub const nibbleToBinary = @import("Core/StringTools.zig").nibbleToBinary;
pub const stringFormat = @import("Core/StringTools.zig").stringFormat;
pub const stringReplace = @import("Core/StringTools.zig").stringReplace;
pub const stringToVector = @import("Core/StringTools.zig").stringToVector;
pub const toLower = @import("Core/StringTools.zig").toLower;
pub const vectorToString = @import("Core/StringTools.zig").vectorToString;
pub const getProcessorTickCount = @import("Core/TickCounter.zig").getProcessorTickCount;
pub const UnorderedMap = @import("Core/UnorderedMap.zig").UnorderedMap;
pub const UnorderedMapDetail = @import("Core/UnorderedMap.zig").UnorderedMapDetail;
pub const UnorderedSet = @import("Core/UnorderedSet.zig").UnorderedSet;
pub const UnorderedSetDetail = @import("Core/UnorderedSet.zig").UnorderedSetDetail;

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

/// Every source file of the module. Used by the test below to make sure that all of them are
/// compiled and that their inline tests run. Add new files here when porting them.
const source_files = .{
    @import("ConfigurationString.zig"),
    @import("Core/Atomics.zig"),
    @import("Core/BinaryHeap.zig"),
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
    @import("Core/StreamIn.zig"),
    @import("Core/StreamOut.zig"),
    @import("Core/StreamWrapper.zig"),
    @import("Core/StringTools.zig"),
    @import("Core/TickCounter.zig"),
    @import("Core/UnorderedMap.zig"),
    @import("Core/UnorderedSet.zig"),
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
