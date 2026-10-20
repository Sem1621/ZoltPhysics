// C ABI wrappers for ZoltParity/Renderer/DebugRendererParity.zig: Jolt's debug renderer, recorder, playback and
// DebugRendererSimple driven by the op stream of ParityRenderer.zig (Docs/Zolt/RemainingFeaturesArchitecture.md
// section 4, "Draw call log"). Every function builds its renderer, runs and destroys it again, so only one Jolt
// DebugRenderer exists at a time (Jolt's constructor asserts that DebugRenderer::sInstance is null).
//
// The functions exist in both renderer configurations. Without JPH_DEBUG_RENDERER they return null and
// jolt_dr_available() returns 0: the parity layers skip then.

#include <Jolt/Jolt.h>

#include <ZoltParity/ParityHooks.h>

#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Core/StreamWrapper.h>
	#include <Jolt/Renderer/DebugRendererPlayback.h>
	#include <ZoltParity/Renderer/ParityRenderer.h>
	#include <sstream>
#endif // JPH_DEBUG_RENDERER

#include <vector>

using namespace JPH;

/// The byte arrays that a function returns
struct JoltDRResult
{
	std::vector<std::vector<uint8>> mParts;

	void							AddU32(uint32 inValue)
	{
		std::vector<uint8> part(sizeof(inValue));
		memcpy(part.data(), &inValue, sizeof(inValue));
		mParts.push_back(std::move(part));
	}
};

#ifdef JPH_DEBUG_RENDERER

using namespace ZoltParity;

/// The bytes written to a string stream so far
static std::vector<uint8> sStreamBytes(const std::stringstream &inStream)
{
	std::string s = inStream.str();
	return std::vector<uint8>(s.begin(), s.end());
}

#endif // JPH_DEBUG_RENDERER

extern "C"
{

/// 1 when the reference is built with JPH_DEBUG_RENDERER
int jolt_dr_available()
{
#ifdef JPH_DEBUG_RENDERER
	return 1;
#else
	return 0;
#endif
}

uint32 jolt_dr_result_count(const JoltDRResult *inResult)						{ return uint32(inResult->mParts.size()); }
const uint8 *jolt_dr_result_data(const JoltDRResult *inResult, uint32 inIndex)	{ return inResult->mParts[inIndex].data(); }
uint64 jolt_dr_result_size(const JoltDRResult *inResult, uint32 inIndex)		{ return inResult->mParts[inIndex].size(); }
void jolt_dr_result_free(JoltDRResult *inResult)								{ delete inResult; }

/// Run the ops on a ParityRenderer (inKind 0) or a SimpleParityRenderer (inKind 1).
/// Parts: [0] the log of the constructor, [1] the log of the ops, [2] the next batch id after the constructor (u32,
/// ParityRenderer only, 0 for the simple renderer).
JoltDRResult *jolt_dr_run_ops(const uint8 *inOps, uint64 inSize, int inKind)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	JoltDRResult *result = new JoltDRResult;
	if (inKind == 0)
	{
		ParityRenderer renderer;
		result->mParts.push_back(renderer.mLog.mBytes);
		uint32 next_batch_id = renderer.mNextBatchID;
		renderer.mLog.mBytes.clear();
		RunParityOps(renderer, inOps, size_t(inSize), nullptr, nullptr);
		result->mParts.push_back(renderer.mLog.mBytes);
		result->AddU32(next_batch_id);
	}
	else
	{
		SimpleParityRenderer renderer;
		result->mParts.push_back(renderer.mLog.mBytes);
		renderer.mLog.mBytes.clear();
		RunParityOps(renderer, inOps, size_t(inSize), &renderer, nullptr);
		result->mParts.push_back(renderer.mLog.mBytes);
		result->AddU32(0);
	}
	return result;
#else
	(void)inOps; (void)inSize; (void)inKind;
	return nullptr;
#endif
}

/// Run the ops on a DebugRendererRecorder. Parts: [0] the bytes that the constructor writes (Initialize), [1] the bytes
/// that the ops write.
JoltDRResult *jolt_dr_record_ops(const uint8 *inOps, uint64 inSize)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	JoltDRResult *result = new JoltDRResult;
	std::stringstream stream;
	{
		StreamOutWrapper wrapper(stream);
		DebugRendererRecorder recorder(wrapper);
		std::vector<uint8> construction = sStreamBytes(stream);
		RunParityOps(recorder, inOps, size_t(inSize), nullptr, &recorder);
		std::vector<uint8> all = sStreamBytes(stream);
		result->mParts.push_back(construction);
		result->mParts.push_back(std::vector<uint8>(all.begin() + construction.size(), all.end()));
	}
	return result;
#else
	(void)inOps; (void)inSize;
	return nullptr;
#endif
}

/// Play a .jor stream back into a ParityRenderer: Parse, then DrawFrame of every frame with a marker (the frame number)
/// before each. Parts: [0] the log of the constructor, [1] the log of the playback, [2] the next batch id after the
/// constructor (u32), [3] the number of frames (u32).
JoltDRResult *jolt_dr_playback(const uint8 *inStream, uint64 inSize)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	JoltDRResult *result = new JoltDRResult;
	ParityRenderer renderer;
	result->mParts.push_back(renderer.mLog.mBytes);
	uint32 next_batch_id = renderer.mNextBatchID;
	renderer.mLog.mBytes.clear();
	uint32 num_frames;
	{
		std::stringstream data { std::string(reinterpret_cast<const char *>(inStream), size_t(inSize)) };
		StreamInWrapper wrapper(data);
		DebugRendererPlayback playback(renderer);
		playback.Parse(wrapper);
		num_frames = playback.GetNumFrames();
		for (uint32 f = 0; f < num_frames; ++f)
		{
			renderer.mLog.WriteMarker(f);
			playback.DrawFrame(f);
		}
	}
	result->mParts.push_back(renderer.mLog.mBytes);
	result->AddU32(next_batch_id);
	result->AddU32(num_frames);
	return result;
#else
	(void)inStream; (void)inSize;
	return nullptr;
#endif
}

} // extern "C"
