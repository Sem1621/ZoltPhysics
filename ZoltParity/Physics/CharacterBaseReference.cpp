// Reference implementation for the CharacterBase / CharacterID parity tests (`zig build parity`): thin C ABI wrappers
// around the C++ Jolt library (CharacterBaseSettings defaults, SetMaxSlopeAngle / GetCosMaxSlopeAngle, IsSlopeTooSteep,
// SaveState / RestoreState of a CharacterBase, sToString, CharacterID's ID sequence and hash).
// ZoltParity/Physics/CharacterBaseParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as 3 floats, RVec3 as 3 Reals (double with JPH_DOUBLE_PRECISION). Booleans are
// returned as uint32 (never bool, see the porting guide). Functions that produce bytes write into a caller provided
// buffer and return the size.

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Character/CharacterBase.h>
#include <Jolt/Physics/Character/CharacterID.h>
#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include <cstring>
#include <string>

using namespace JPH;

// Must match CBGround in CharacterBaseParity.zig: the ground members of a CharacterBase
struct CBGround
{
	uint32				mState;
	uint32				mBodyID;
	uint32				mSubShapeID;
	Real				mPosition[3];
	float				mNormal[3];
	float				mVelocity[3];
};

// A CharacterBase whose protected ground members can be read and written (TestCharacter of CharacterBase.zig)
class CBTestCharacter final : public CharacterBase
{
public:
	using CharacterBase::CharacterBase;

	void				SetGround(const CBGround &inGround)
	{
		mGroundState = EGroundState(inGround.mState);
		mGroundBodyID = BodyID(inGround.mBodyID);
		mGroundBodySubShapeID.SetValue(inGround.mSubShapeID);
		mGroundPosition = RVec3(inGround.mPosition[0], inGround.mPosition[1], inGround.mPosition[2]);
		mGroundNormal = Vec3(inGround.mNormal[0], inGround.mNormal[1], inGround.mNormal[2]);
		mGroundVelocity = Vec3(inGround.mVelocity[0], inGround.mVelocity[1], inGround.mVelocity[2]);
		mGroundUserData = 0x1234;									// Not part of the state: RestoreState resets it
	}

	void				GetGround(CBGround &outGround) const
	{
		outGround.mState = uint32(mGroundState);
		outGround.mBodyID = mGroundBodyID.GetIndexAndSequenceNumber();
		outGround.mSubShapeID = mGroundBodySubShapeID.GetValue();
		outGround.mPosition[0] = mGroundPosition.GetX();
		outGround.mPosition[1] = mGroundPosition.GetY();
		outGround.mPosition[2] = mGroundPosition.GetZ();
		mGroundNormal.StoreFloat3(reinterpret_cast<Float3 *>(outGround.mNormal));
		mGroundVelocity.StoreFloat3(reinterpret_cast<Float3 *>(outGround.mVelocity));
	}
};

extern "C"
{

// The CharacterBaseSettings defaults (has_shape: 1 when mShape is set)
void jolt_cb_settings_defaults(float *outUp, float *outPlane, float *outMaxSlopeAngle, uint32 *outEnhancedInternalEdgeRemoval, uint32 *outHasShape)
{
	CharacterBaseSettings s;
	s.mUp.StoreFloat3(reinterpret_cast<Float3 *>(outUp));
	s.mSupportingVolume.GetNormal().StoreFloat3(reinterpret_cast<Float3 *>(outPlane));
	outPlane[3] = s.mSupportingVolume.GetConstant();
	*outMaxSlopeAngle = s.mMaxSlopeAngle;
	*outEnhancedInternalEdgeRemoval = s.mEnhancedInternalEdgeRemoval? 1 : 0;
	*outHasShape = s.mShape != nullptr? 1 : 0;
}

// GetCosMaxSlopeAngle of a character constructed with mMaxSlopeAngle = inAngles[i] (CharacterBase's constructor calls
// SetMaxSlopeAngle)
void jolt_cb_cos_max_slope_angle(const float *inAngles, uint32 inCount, float *outCos)
{
	CharacterBaseSettings settings;
	for (uint32 i = 0; i < inCount; ++i)
	{
		settings.mMaxSlopeAngle = inAngles[i];
		CBTestCharacter character(&settings, nullptr);
		outCos[i] = character.GetCosMaxSlopeAngle();

		// SetMaxSlopeAngle gives the same value as the constructor
		character.SetMaxSlopeAngle(0.0f);
		character.SetMaxSlopeAngle(inAngles[i]);
		if (character.GetCosMaxSlopeAngle() != outCos[i] && outCos[i] == outCos[i])
			abort();
	}
}

// IsSlopeTooSteep(inNormals[i]) of a character with up vector inUp and max slope angle inMaxSlopeAngle
void jolt_cb_is_slope_too_steep(const float *inUp, float inMaxSlopeAngle, const float *inNormals, uint32 inCount, uint32 *outResults)
{
	CharacterBaseSettings settings;
	settings.mUp = Vec3(inUp[0], inUp[1], inUp[2]);
	settings.mMaxSlopeAngle = inMaxSlopeAngle;
	CBTestCharacter character(&settings, nullptr);
	for (uint32 i = 0; i < inCount; ++i)
		outResults[i] = character.IsSlopeTooSteep(Vec3(inNormals[3 * i], inNormals[3 * i + 1], inNormals[3 * i + 2]))? 1 : 0;
}

// The SaveState bytes of a character with the ground members inGround, returns the size (copied when it fits)
uint32 jolt_cb_save_state(const CBGround *inGround, uint8 *outBytes, uint32 inCapacity)
{
	CharacterBaseSettings settings;
	CBTestCharacter character(&settings, nullptr);
	character.SetGround(*inGround);
	StateRecorderImpl recorder;
	character.SaveState(recorder);
	std::string data = recorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// RestoreState of a character from inBytes (not validating): the ground members afterwards, outReset is 1 when the
// ground user data and material were reset (0 and PhysicsMaterial::sDefault); returns 1 when the bytes were read
// without error and completely
uint32 jolt_cb_restore_state(const uint8 *inBytes, uint32 inSize, CBGround *outGround, uint32 *outReset)
{
	CharacterBaseSettings settings;
	CBTestCharacter character(&settings, nullptr);
	CBGround other = { 3, 7, 9, { 1, 2, 3 }, { 0, 1, 0 }, { 4, 5, 6 } };
	character.SetGround(other);
	StateRecorderImpl recorder;
	recorder.WriteBytes(inBytes, inSize);
	recorder.Rewind();
	character.RestoreState(recorder);
	character.GetGround(*outGround);
	*outReset = character.GetGroundUserData() == 0 && character.GetGroundMaterial() == PhysicsMaterial::sDefault? 1 : 0;
	bool read = !recorder.IsFailed();
	uint8 extra;
	recorder.Read(extra); // Fails at the end of the bytes
	return read && recorder.IsEOF()? 1 : 0;
}

// CharacterBase::sToString(EGroundState(inState)) (valid states only, Jolt asserts on others), returns the length
uint32 jolt_cb_to_string(uint32 inState, char *outText, uint32 inCapacity)
{
	const char *text = CharacterBase::sToString(CharacterBase::EGroundState(inState));
	uint32 length = uint32(strlen(text));
	if (length <= inCapacity)
		memcpy(outText, text, length);
	return length;
}

// CharacterID::sSetNextCharacterID(inNextValue), then inCount times sNextCharacterID; the counter is set back to 1
void jolt_cid_sequence(uint32 inNextValue, uint32 inCount, uint32 *outIDs)
{
	CharacterID::sSetNextCharacterID(inNextValue);
	for (uint32 i = 0; i < inCount; ++i)
		outIDs[i] = CharacterID::sNextCharacterID().GetValue();
	CharacterID::sSetNextCharacterID();
}

// CharacterID::GetHash of the IDs
void jolt_cid_hash(const uint32 *inIDs, uint32 inCount, uint64 *outHashes)
{
	for (uint32 i = 0; i < inCount; ++i)
		outHashes[i] = CharacterID(inIDs[i]).GetHash();
}

} // extern "C"
