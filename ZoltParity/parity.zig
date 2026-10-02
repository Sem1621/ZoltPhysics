//! Root of `zig build parity`: run Zolt and the C++ Jolt library on the same inputs and require identical bits.
//!
//! `build.zig` compiles the C++ library from `Jolt/` with Zig's C++ compiler in the configuration Zolt follows
//! (JPH_CROSS_PLATFORM_DETERMINISTIC, -ffp-contract=off, pinned CPU features) plus the C ABI wrappers listed in
//! `reference_sources.zig`, and links them into this test binary.
//!
//! Layout, mirroring Zolt/: `ZoltParity/<Dir>/<Dir>Parity.zig` (tests) + `ZoltParity/<Dir>/<Dir>Reference.cpp`
//! (C ABI wrappers around Jolt). Shared helpers are in `ParityFramework.zig`. Register new test files below and
//! new .cpp files in `reference_sources.zig` (both lists are merged with git's union driver, keep them sorted).
//!
//! Run: zig build parity [-Ddouble_precision=true] [-Dtest-filter=Vec4]

test {
    _ = @import("AABBTree/AABBTreeParity.zig");
    _ = @import("Core/ConcurrencyParity.zig");
    _ = @import("Core/ContainersParity.zig");
    _ = @import("Core/CoreParity.zig");
    _ = @import("Geometry/PrimitivesParity.zig");
    _ = @import("Geometry/QueriesParity.zig");
    _ = @import("Math/MathParity.zig");
    _ = @import("ParityFramework.zig");
}
