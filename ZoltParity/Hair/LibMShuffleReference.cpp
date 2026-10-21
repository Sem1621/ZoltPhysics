// Reference implementation for ZoltParity/Hair/LibMParity.zig (`zig build parity`): the C library functions and the
// standard library algorithms that Jolt's hair uses (Docs/Zolt/RemainingFeaturesArchitecture.md R13).
//
// - std::exp(float) and std::pow(float, float) as Jolt calls them (Hair.cpp, HairSettings.h). They resolve to the expf
//   and powf of LibMReference.c, the pin of the reference. Every entry point calls jolt_libm_reference_linked(), so
//   that object is linked whenever this one is.
// - std::shuffle with a default seeded std::mt19937 (HairSettings::InitRenderAndSimulationStrands) and
//   std::uniform_int_distribution: libc++'s algorithms, the standard library of Zig's C++ compiler.

#include <Jolt/Jolt.h>

#include <ZoltParity/ParityHooks.h>

#include <algorithm>
#include <cmath>
#include <random>

using namespace JPH;

extern "C" int jolt_libm_reference_linked();

namespace {

// A std::mt19937 that counts its draws. It has the result type, min and max of std::mt19937, so libc++ makes the same
// draws with it.
class CountingEngine
{
public:
	using result_type = std::mt19937::result_type;

	static constexpr result_type	min()						{ return std::mt19937::min(); }
	static constexpr result_type	max()						{ return std::mt19937::max(); }

	result_type						operator () ()				{ ++mDraws; return mEngine(); }

	std::mt19937					mEngine;
	uint64							mDraws = 0;
};

} // namespace

extern "C" {

// 1 when the pin of LibMReference.c is linked
int jolt_libm_linked()
{
	return jolt_libm_reference_linked();
}

// std::exp(float) of every input
void jolt_libm_expf(const float *inX, float *outY, size_t inCount)
{
	jolt_libm_reference_linked();
	for (size_t i = 0; i < inCount; ++i)
		outY[i] = std::exp(inX[i]);
}

// std::pow(float, float) of every pair
void jolt_libm_powf(const float *inX, const float *inY, float *outZ, size_t inCount)
{
	jolt_libm_reference_linked();
	for (size_t i = 0; i < inCount; ++i)
		outZ[i] = std::pow(inX[i], inY[i]);
}

// std::shuffle of the values 0 .. inCount - 1 with a default seeded std::mt19937, like
// HairSettings::InitRenderAndSimulationStrands. *outDraws is the number of engine calls, outNext the next 4 outputs of
// the engine after the shuffle. Returns 0, or 1 when the counting engine shuffled differently than std::mt19937.
int jolt_libcxx_shuffle(uint32 inCount, uint32 *outItems, uint64 *outDraws, uint32 *outNext)
{
	ParityInstallHooks(); // The counted copy below allocates through Jolt

	for (uint32 i = 0; i < inCount; ++i)
		outItems[i] = i;
	std::mt19937 random;
	std::shuffle(outItems, outItems + inCount, random);
	for (int i = 0; i < 4; ++i)
		outNext[i] = uint32(random());

	// The same shuffle with the counting engine
	Array<uint32> counted;
	counted.resize(inCount);
	for (uint32 i = 0; i < inCount; ++i)
		counted[i] = i;
	CountingEngine counting;
	std::shuffle(counted.begin(), counted.end(), counting);
	*outDraws = counting.mDraws;
	return std::equal(counted.begin(), counted.end(), outItems)? 0 : 1;
}

// inCount values of std::uniform_int_distribution<T>(inA, inB) with a std::mt19937 seeded with inSeed. inType: 0
// ptrdiff_t (std::shuffle's), 1 uint32_t, 2 int, 3 uint64_t. The values are written as 64 bit integers (sign extended).
// *outDraws is the number of engine calls.
void jolt_libcxx_uniform_int(int inType, int64 inA, int64 inB, uint32 inSeed, uint32 inCount, int64 *outValues, uint64 *outDraws)
{
	CountingEngine engine;
	engine.mEngine.seed(inSeed);
	for (uint32 i = 0; i < inCount; ++i)
		switch (inType)
		{
		case 0:		outValues[i] = int64(std::uniform_int_distribution<std::ptrdiff_t>(std::ptrdiff_t(inA), std::ptrdiff_t(inB))(engine)); break;
		case 1:		outValues[i] = int64(std::uniform_int_distribution<uint32_t>(uint32_t(inA), uint32_t(inB))(engine)); break;
		case 2:		outValues[i] = int64(std::uniform_int_distribution<int>(int(inA), int(inB))(engine)); break;
		default:	outValues[i] = int64(std::uniform_int_distribution<uint64_t>(uint64_t(inA), uint64_t(inB))(engine)); break;
		}
	*outDraws = engine.mDraws;
}

// The rule of ParityHooks.h for jolt_libcxx_shuffle, which allocates through Jolt: 1 when it installs the hooks again
// after an older reference file installed Jolt's allocator (RegisterDefaultAllocator). The hooks stay installed.
uint32 jolt_libcxx_shuffle_hooks_check()
{
	RegisterDefaultAllocator(); // What the older reference files call
	uint32 items[3];
	uint64 draws;
	uint32 next[4];
	jolt_libcxx_shuffle(3, items, &draws, next);
	bool installed = ParityHooksInstalled();
	ParityInstallHooks();
	return installed? 1 : 0;
}

} // extern "C"
