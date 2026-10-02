//! C++ files (relative to ZoltParity/) with the C ABI wrappers that the parity tests call.
//! Compiled into the reference library by `zig build parity`. Keep sorted (merged with git's union driver).

pub const files = [_][]const u8{
    "Core/ConcurrencyReference.cpp",
    "Core/ContainersReference.cpp",
    "Math/MathReference.cpp",
};
