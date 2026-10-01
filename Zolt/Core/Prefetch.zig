//! Port of: Jolt/Core/Prefetch.h
//! Status: complete

/// Prefetch the given address to L1 cache. Can be used to avoid cache misses, but should be used with care as it can also cause cache pollution if used incorrectly.
pub fn prefetchL1(address: anytype) void {
    @prefetch(address, .{ .rw = .read, .locality = 3, .cache = .data });
}

test prefetchL1 {
    const value: u32 = 0;
    prefetchL1(&value);
}
