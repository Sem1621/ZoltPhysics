// The C ABI of ZoltParity/ParityHooks.h for ZoltParity/ParityHooks.zig (Docs/Zolt/RemainingFeaturesArchitecture.md R1,
// R8): the number of Jolt asserts that failed in this process, and two checks of the hooks themselves. A parity test that
// calls one of these functions links this file, so the hooks are installed before main.

#include <Jolt/Jolt.h>

#include <ZoltParity/ParityHooks.h>

using namespace JPH;

// Recorded during the static initialization of this file, after the one of ParityHooks.h (it is defined later in the
// same file, so it is initialized later). Read by jolt_parity_hooks_installed_before_main: later, the older reference
// files may have installed Jolt's allocator again.
static const bool sParityHooksInstalledBeforeMain = ParityHooksInstalled();

extern "C"
{

// The number of Jolt asserts that failed in this process (always 0 without JPH_ENABLE_ASSERTS)
uint32 jolt_parity_assert_count()
{
	return ParityAssertCount();
}

// 1 when the reference library is built with JPH_ENABLE_ASSERTS (`zig build parity -Dreference_asserts=true`)
uint32 jolt_parity_reference_has_asserts()
{
#ifdef JPH_ENABLE_ASSERTS
	return 1;
#else
	return 0;
#endif
}

// 1 when the hooks of ParityHooks.h were installed before main
uint32 jolt_parity_hooks_installed_before_main()
{
	return sParityHooksInstalledBeforeMain? 1 : 0;
}

// 1 when ParityInstallHooks() installs the hooks again after RegisterDefaultAllocator() (what the older reference files
// call) and a second call keeps them. The hooks stay installed.
uint32 jolt_parity_reinstall_check()
{
	RegisterDefaultAllocator();
	bool replaced = !ParityHooksInstalled();
	ParityInstallHooks();
	bool installed = ParityHooksInstalled();
	ParityInstallHooks();
	return replaced && installed && ParityHooksInstalled()? 1 : 0;
}

// Fails a JPH_ASSERT on purpose (the test of the assert hook): the number of failed asserts before and after it
void jolt_parity_test_assert_hook(uint32 *outBefore, uint32 *outAfter)
{
	ParityInstallHooks();
	*outBefore = ParityAssertCount();
	JPH_ASSERT(false, "ParityHooks: the test of the assert hook (expected)");
	*outAfter = ParityAssertCount();
}

// 1 when the allocation hooks zero-fill: a block that was filled and freed comes back as zeros from Allocate and
// AlignedAllocate, Reallocate keeps the old bytes and zeroes the new ones
uint32 jolt_parity_zero_filling_check()
{
	ParityInstallHooks();
	uint32 ok = 1;
	for (uint32 i = 0; i < 8; ++i)
	{
		size_t size = size_t(48) << i;
		uint8 *dirty = static_cast<uint8 *>(Allocate(size));
		memset(dirty, 0xab, size);
		Free(dirty);
		uint8 *block = static_cast<uint8 *>(Allocate(size));
		for (size_t j = 0; j < size; ++j)
			ok &= block[j] == 0? 1 : 0;
		memset(block, 0xcd, size);
		uint8 *grown = static_cast<uint8 *>(Reallocate(block, size, 2 * size));
		for (size_t j = 0; j < 2 * size; ++j)
			ok &= grown[j] == (j < size? 0xcd : 0)? 1 : 0;
		Free(grown);
		uint8 *aligned_dirty = static_cast<uint8 *>(AlignedAllocate(size, 64));
		memset(aligned_dirty, 0xef, size);
		AlignedFree(aligned_dirty);
		uint8 *aligned = static_cast<uint8 *>(AlignedAllocate(size, 64));
		for (size_t j = 0; j < size; ++j)
			ok &= aligned[j] == 0? 1 : 0;
		AlignedFree(aligned);
	}
	return ok;
}

} // extern "C"
