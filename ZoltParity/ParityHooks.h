// The process hooks of the C++ reference of `zig build parity` (Docs/Zolt/RemainingFeaturesArchitecture.md R1): Jolt's
// allocation functions with zero-filled blocks, a Trace that prints and, with JPH_ENABLE_ASSERTS
// (`zig build parity -Dreference_asserts=true`), an AssertFailed that prints and counts.
//
// The rule for reference files:
// - A new reference file includes this header (`#include <ZoltParity/ParityHooks.h>`) and calls ParityInstallHooks() at
//   the start of every exported function that allocates through Jolt (or in its EnsureFactory function).
// - It never calls RegisterDefaultAllocator(): that installs Jolt's allocator, which does not zero-fill.
// - Why: a filtered parity run links only the reference objects that its tests use. A file that relies on the hooks of
//   another file may see Jolt's default allocator (no zero filling) or a null Allocate (a crash).
// - Including the header also installs the hooks before main (sParityHooksInstalled below), so the hooks are in place
//   before any Jolt allocation of the process.
// - The older reference files call RegisterDefaultAllocator() and keep doing so. Jolt's allocator is compatible with the
//   hooks (both free with free), so a block may be allocated by one and freed by the other. A file that includes this
//   header calls ParityInstallHooks() again before it creates a world.
//
// Allocation: Jolt's allocator (malloc / realloc / posix_memalign / free), but every allocation is zero-filled. Jolt
// leaves members uninitialized that its states hold: Wheel's mContactPosition, mContactPointVelocity, mContactNormal,
// mContactLongitudinal, mContactLateral and mAxlePlaneConstant until the first contact of the wheel (Wheel uses
// JPH_OVERRIDE_NEW_DELETE), which VehicleConstraint::SaveState writes. With the zero-filled allocations they are 0 in
// both C++ worlds, and Zolt zero-initializes them (a documented deviation, CharacterVehicleArchitecture.md "Settled in
// the foundation review"), so a vehicle that starts in the air compares too. Phase 9 relies on it too: the padding
// vertices of short hair strands and the CPU compute buffers are zero in Zolt (UB4 of R20).
//
// Trace: printed to stderr (Jolt's default does nothing in a build without asserts, so the "Trace(...); abort();"
// diagnostics of the reference files would abort silently).
//
// AssertFailed (a reference built with JPH_ENABLE_ASSERTS): printed to stderr and counted (ParityAssertCount, read by
// jolt_sim_reference_assert_count), execution continues like Jolt's build without asserts. The count is one counter for
// the whole process, whatever file hit the assert.

#pragma once

#include <Jolt/Jolt.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// The functions and the counter are inline: every reference file that includes this header shares one definition, so
// the function addresses (ParityHooksInstalled compares them) and the assert count are the same in every file.

inline void *ParityZeroAllocate(size_t inSize)
{
	return calloc(1, inSize);
}

inline void *ParityZeroReallocate(void *inBlock, size_t inOldSize, size_t inNewSize)
{
	void *block = realloc(inBlock, inNewSize);
	if (block != nullptr && inNewSize > inOldSize)
		memset(static_cast<JPH::uint8 *>(block) + inOldSize, 0, inNewSize - inOldSize);
	return block;
}

inline void ParityFree(void *inBlock)
{
	free(inBlock);
}

inline void *ParityZeroAlignedAllocate(size_t inSize, size_t inAlignment)
{
#if defined(JPH_PLATFORM_WINDOWS)
	void *block = _aligned_malloc(inSize, inAlignment);
#else
	void *block = nullptr;
	if (posix_memalign(&block, inAlignment, inSize) != 0)
		block = nullptr;
#endif
	if (block != nullptr)
		memset(block, 0, inSize);
	return block;
}

inline void ParityAlignedFree(void *inBlock)
{
#if defined(JPH_PLATFORM_WINDOWS)
	_aligned_free(inBlock);
#else
	free(inBlock);
#endif
}

inline void ParityTrace(const char *inFMT, ...)
{
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);
	fprintf(stderr, "Jolt: %s\n", buffer);
	fflush(stderr);
}

#ifdef JPH_ENABLE_ASSERTS

// The number of Jolt asserts that failed in this process (ParityAssertCount)
inline std::atomic<JPH::uint32> gParityAssertCount { 0 };

inline bool ParityAssertFailed(const char *inExpression, const char *inMessage, const char *inFile, JPH::uint inLine)
{
	++gParityAssertCount;
	fprintf(stderr, "Jolt assert failed: %s:%u: (%s) %s\n", inFile, inLine, inExpression, inMessage != nullptr? inMessage : "");
	fflush(stderr);
	return false; // No breakpoint: continue like Jolt's build without asserts, the driver fails the scene
}

#endif // JPH_ENABLE_ASSERTS

// The number of Jolt asserts that failed in this process (always 0 without JPH_ENABLE_ASSERTS)
inline JPH::uint32 ParityAssertCount()
{
#ifdef JPH_ENABLE_ASSERTS
	return gParityAssertCount.load();
#else
	return 0;
#endif
}

// True when every hook of this header is installed
inline bool ParityHooksInstalled()
{
	return JPH::Allocate == ParityZeroAllocate
		&& JPH::Reallocate == ParityZeroReallocate
		&& JPH::Free == ParityFree
		&& JPH::AlignedAllocate == ParityZeroAlignedAllocate
		&& JPH::AlignedFree == ParityAlignedFree
		&& JPH::Trace == ParityTrace
		JPH_IF_ENABLE_ASSERTS(&& JPH::AssertFailed == ParityAssertFailed);
}

// Installs the hooks. Idempotent: when they are installed already, nothing is written. Call it from the main thread,
// outside a physics update (it writes Jolt's global function pointers).
inline void ParityInstallHooks()
{
	if (ParityHooksInstalled())
		return;
	JPH::Allocate = ParityZeroAllocate;
	JPH::Reallocate = ParityZeroReallocate;
	JPH::Free = ParityFree;
	JPH::AlignedAllocate = ParityZeroAlignedAllocate;
	JPH::AlignedFree = ParityAlignedFree;
	JPH::Trace = ParityTrace;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = ParityAssertFailed;)
}

// Before main: before any Jolt allocation of the parity process. One per file that includes this header; the first one
// installs the hooks, the others find them installed.
[[maybe_unused]] static const bool sParityHooksInstalled = (ParityInstallHooks(), true);
