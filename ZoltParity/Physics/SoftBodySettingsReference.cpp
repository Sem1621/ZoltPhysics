// Reference implementation for the soft body settings parity tests of group S1 (`zig build parity`): a C ABI that runs a
// list of operations on Jolt's SoftBodySharedSettings (CreateConstraints, the Calculate* functions, Optimize, Clone,
// sCreateCube, the private CalculateClosestKinematic) and writes a transcript of the results.
// ZoltParity/Physics/SoftBodySettingsParity.zig runs the same operations on Zolt's settings, writes the same
// transcript and compares the bytes.
//
// The structs and the transcript format below must match SoftBodySettingsParity.zig. All C functions have the prefix
// jolt_sbs_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

#include <cstring>
#include <sstream>
#include <string>

// The process hooks are ZoltParity/ParityHooks.h (R1): zero-filled allocations, a printing Trace and, with
// JPH_ENABLE_ASSERTS, an AssertFailed that prints and counts. Every exported function that allocates through Jolt calls
// ParityInstallHooks() first, because a filtered parity run links only the reference objects that it uses.
#include <ZoltParity/ParityHooks.h>

using namespace JPH;

namespace {

// Private access to the closest kinematic cache and to CalculateClosestKinematic (explicit instantiation ignores access
// checks, `auto` avoids naming the private nested type)
template <typename Tag, auto M>
struct SBSRob
{
	friend auto SBSGet(Tag) { return M; }
};

#define SBS_ROB(Name, Member) \
	struct Name { friend auto SBSGet(Name); }; \
	template struct SBSRob<Name, &Member>;

SBS_ROB(SBSClosestKinematic, SoftBodySharedSettings::mClosestKinematic)
SBS_ROB(SBSCalculateClosestKinematic, SoftBodySharedSettings::CalculateClosestKinematic)

// The operations (must match OpKind in SoftBodySettingsParity.zig)
enum class SBSOpKind : uint32
{
	CreateConstraints,					///< CreateConstraints(attributes, mBendType, mValue)
	CalculateEdgeLengths,
	CalculateRodProperties,
	CalculateLRALengths,				///< CalculateLRALengths(mValue)
	CalculateBendConstraintConstants,
	CalculateVolumeConstraintVolumes,
	CalculateSkinnedConstraintNormals,
	Optimize,							///< Optimize(results): the 7 remaps go into the transcript
	OptimizeNoResults,					///< Optimize()
	CalculateClosestKinematic,			///< The private CalculateClosestKinematic
	Clone,								///< Clone(): the clone's SaveBinaryState bytes go into the transcript
	AppendExtra,						///< Append every array of the extra settings but the vertices and the private ones
	CreateConstraintsDefaults,			///< CreateConstraints(attributes) with Jolt's default arguments
};

// Must match Op in SoftBodySettingsParity.zig
struct SBSOp
{
	uint32				mKind;
	uint32				mBendType;
	float				mValue;
};

/// The transcript of a run
class SBSTranscript
{
public:
	void				Write(const void *inData, size_t inSize)	{ mBytes.append(static_cast<const char *>(inData), inSize); }
	void				WriteU32(uint32 inValue)					{ Write(&inValue, sizeof(inValue)); }
	void				WriteF32(float inValue)						{ Write(&inValue, sizeof(inValue)); }

	void				WriteRemap(const Array<uint> &inRemap)
	{
		WriteU32(uint32(inRemap.size()));
		for (uint v : inRemap)
			WriteU32(uint32(v));
	}

	/// The SaveBinaryState bytes, prefixed by their size
	void				WriteSettings(const SoftBodySharedSettings &inSettings)
	{
		std::stringstream data;
		{
			StreamOutWrapper out(data);
			inSettings.SaveBinaryState(out);
		}
		std::string bytes = data.str();
		WriteU32(uint32(bytes.size()));
		Write(bytes.data(), bytes.size());
	}

	/// The private closest kinematic array: the count, then vertex, hops and distance per element
	void				WriteClosestKinematic(const SoftBodySharedSettings &inSettings)
	{
		const auto &closest = inSettings.*SBSGet(SBSClosestKinematic());
		WriteU32(uint32(closest.size()));
		for (const auto &c : closest)
		{
			WriteU32(c.mVertex);
			WriteU32(c.mHops);
			WriteF32(c.mDistance);
		}
	}

	std::string			mBytes;
};

/// RestoreBinaryState of inBytes into new settings
Ref<SoftBodySharedSettings> SBSRestore(const uint8 *inBytes, uint32 inSize)
{
	Ref<SoftBodySharedSettings> settings = new SoftBodySharedSettings;
	std::stringstream data(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper in(data);
	settings->RestoreBinaryState(in);
	return settings;
}

template <class T>
void SBSAppend(Array<T> &ioTo, const Array<T> &inFrom)
{
	ioTo.insert(ioTo.end(), inFrom.begin(), inFrom.end());
}

} // namespace

extern "C" {

/// The sizes of the structs that the C ABI passes (VertexAttributes, SBSOp)
void jolt_sbs_struct_sizes(uint32 *outSizes)
{
	outSizes[0] = sizeof(SoftBodySharedSettings::VertexAttributes);
	outSizes[1] = sizeof(SBSOp);
}

/// Runs the operations inOps on settings and writes the transcript to outTranscript. The settings are
/// SoftBodySharedSettings::sCreateCube(inCubeGridSize, inCubeSpacing) when inCubeGridSize != 0, otherwise what
/// RestoreBinaryState makes of inSettings. inExtra are the SaveBinaryState bytes of the settings that AppendExtra
/// appends. The transcript: per operation a marker (0xABCD0000 + the operation index), the results of the operation
/// (Optimize: the 7 remaps; Clone: the clone's bytes, its number of materials and the size of its closest kinematic
/// array) and the closest kinematic array of the settings; at the end the SaveBinaryState bytes of the settings.
/// Returns the size of the transcript (0xFFFFFFFF if it doesn't fit).
uint32 jolt_sbs_run(const uint8 *inSettings, uint32 inSettingsSize, const uint8 *inExtra, uint32 inExtraSize, const SoftBodySharedSettings::VertexAttributes *inAttributes, uint32 inNumAttributes, const SBSOp *inOps, uint32 inNumOps, uint32 inCubeGridSize, float inCubeSpacing, uint8 *outTranscript, uint32 inCapacity)
{
	ParityInstallHooks();

	Ref<SoftBodySharedSettings> settings = inCubeGridSize != 0? SoftBodySharedSettings::sCreateCube(inCubeGridSize, inCubeSpacing) : SBSRestore(inSettings, inSettingsSize);
	Ref<SoftBodySharedSettings> extra = SBSRestore(inExtra, inExtraSize);
	SoftBodySharedSettings::OptimizationResults results; // Shared by every Optimize of the run
	SBSTranscript t;
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const SBSOp &op = inOps[i];
		t.WriteU32(0xABCD0000u + i);
		switch (SBSOpKind(op.mKind))
		{
		case SBSOpKind::CreateConstraints:
			settings->CreateConstraints(inAttributes, inNumAttributes, SoftBodySharedSettings::EBendType(op.mBendType), op.mValue);
			break;

		case SBSOpKind::CalculateEdgeLengths:
			settings->CalculateEdgeLengths();
			break;

		case SBSOpKind::CalculateRodProperties:
			settings->CalculateRodProperties();
			break;

		case SBSOpKind::CalculateLRALengths:
			settings->CalculateLRALengths(op.mValue);
			break;

		case SBSOpKind::CalculateBendConstraintConstants:
			settings->CalculateBendConstraintConstants();
			break;

		case SBSOpKind::CalculateVolumeConstraintVolumes:
			settings->CalculateVolumeConstraintVolumes();
			break;

		case SBSOpKind::CalculateSkinnedConstraintNormals:
			settings->CalculateSkinnedConstraintNormals();
			break;

		case SBSOpKind::Optimize:
			settings->Optimize(results);
			t.WriteRemap(results.mEdgeRemap);
			t.WriteRemap(results.mLRARemap);
			t.WriteRemap(results.mRodStretchShearConstraintRemap);
			t.WriteRemap(results.mRodBendTwistConstraintRemap);
			t.WriteRemap(results.mDihedralBendRemap);
			t.WriteRemap(results.mVolumeRemap);
			t.WriteRemap(results.mSkinnedRemap);
			break;

		case SBSOpKind::OptimizeNoResults:
			settings->Optimize();
			break;

		case SBSOpKind::CalculateClosestKinematic:
			((*settings).*SBSGet(SBSCalculateClosestKinematic()))();
			break;

		case SBSOpKind::Clone:
			{
				Ref<SoftBodySharedSettings> clone = settings->Clone();
				const SoftBodySharedSettings &c = *clone;
				t.WriteSettings(c);
				t.WriteU32(uint32(c.mMaterials.size()));
				t.WriteU32(uint32((c.*SBSGet(SBSClosestKinematic())).size()));
			}
			break;

		case SBSOpKind::AppendExtra:
			SBSAppend(settings->mFaces, extra->mFaces);
			SBSAppend(settings->mEdgeConstraints, extra->mEdgeConstraints);
			SBSAppend(settings->mDihedralBendConstraints, extra->mDihedralBendConstraints);
			SBSAppend(settings->mVolumeConstraints, extra->mVolumeConstraints);
			SBSAppend(settings->mSkinnedConstraints, extra->mSkinnedConstraints);
			SBSAppend(settings->mInvBindMatrices, extra->mInvBindMatrices);
			SBSAppend(settings->mLRAConstraints, extra->mLRAConstraints);
			SBSAppend(settings->mRodStretchShearConstraints, extra->mRodStretchShearConstraints);
			SBSAppend(settings->mRodBendTwistConstraints, extra->mRodBendTwistConstraints);
			break;

		case SBSOpKind::CreateConstraintsDefaults:
			settings->CreateConstraints(inAttributes, inNumAttributes);
			break;

		default:
			return 0xFFFFFFFF;
		}
		t.WriteClosestKinematic(*settings);
	}
	t.WriteSettings(*settings);

	if (t.mBytes.size() > inCapacity)
		return 0xFFFFFFFF;
	memcpy(outTranscript, t.mBytes.data(), t.mBytes.size());
	return uint32(t.mBytes.size());
}

} // extern "C"
