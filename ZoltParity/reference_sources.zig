//! C++ files (relative to ZoltParity/) with the C ABI wrappers that the parity tests call.
//! Compiled into the reference library by `zig build parity`. Keep sorted (merged with git's union driver).

pub const files = [_][]const u8{
    "AABBTree/AABBTreeReference.cpp",
    "Core/ConcurrencyReference.cpp",
    "Core/ContainersReference.cpp",
    "Core/CoreReference.cpp",
    "Geometry/ConvexHullReference.cpp",
    "Geometry/GJKEPAReference.cpp",
    "Geometry/PrimitivesReference.cpp",
    "Geometry/QueriesReference.cpp",
    "Math/MathReference.cpp",
    "Physics/BasicsReference.cpp",
    "Physics/CompositeReference.cpp",
    "Physics/ConvexReference.cpp",
    "Physics/FiltersReference.cpp",
    "Physics/MeshShapeReference.cpp",
    "Physics/ShapeCoreReference.cpp",
    "Physics/TrianglesReference.cpp",
};
