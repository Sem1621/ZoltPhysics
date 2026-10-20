// Reference implementation for ZoltParity/Hair/HairSettingsParity.zig (`zig build parity`): Jolt's HairSettings
// (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, row "HairSettings").
//
// The Zig side builds the inputs (the scalp of Assets/face.bin or none, the materials, the grooms) and hands them over as
// bytes: the settings as their SaveBinaryState bytes (restored with RestoreBinaryState), the groom as raw
// HairSettings::SVertex / SStrand arrays. This file runs Jolt's functions on them and keeps the bytes of the results in
// result slots (jolt_hair_result) until the next call.
//
// The hooks of ParityHooks.h are installed: every allocation is zero filled, so the padding vertices of InitCompute's
// buffers are 0 like in Zolt (UB4 of R20).

#include <Jolt/Jolt.h>

#include <ZoltParity/ParityHooks.h>

#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Compute/ComputeSystem.h>
#include <Jolt/Compute/CPU/ComputeSystemCPU.h>
#include <Jolt/Physics/Hair/HairSettings.h>

#include <sstream>
#include <string>

using namespace JPH;

extern "C" int jolt_libm_reference_linked();

namespace {

// The bytes of the results of the last call
std::string sResults[32];

// The compute buffers of InitCompute, in the order of the parity
Ref<ComputeBuffer> HairSettings::*const sBuffers[] = {
	&HairSettings::mScalpVerticesCB,
	&HairSettings::mScalpTrianglesCB,
	&HairSettings::mScalpSkinWeightsCB,
	&HairSettings::mSkinPointsCB,
	&HairSettings::mVerticesFixedCB,
	&HairSettings::mVerticesPositionCB,
	&HairSettings::mVerticesBishopCB,
	&HairSettings::mVerticesOmega0CB,
	&HairSettings::mVerticesLengthCB,
	&HairSettings::mVerticesStrandFractionCB,
	&HairSettings::mStrandVertexCountsCB,
	&HairSettings::mStrandMaterialIndexCB,
	&HairSettings::mNeutralDensityCB,
	&HairSettings::mSVertexInfluencesCB,
};

constexpr int cNumBuffers = int(std::size(sBuffers));

std::string SaveState(const HairSettings &inSettings)
{
	std::stringstream data;
	StreamOutWrapper out(data);
	inSettings.SaveBinaryState(out);
	return data.str();
}

Ref<HairSettings> RestoreState(const uint8 *inState, size_t inStateSize)
{
	Ref<HairSettings> settings = new HairSettings;
	std::stringstream data(std::string((const char *)inState, inStateSize));
	StreamInWrapper in(data);
	settings->RestoreBinaryState(in);
	return settings;
}

} // namespace

extern "C" {

// The bytes of result slot inIndex (valid until the next call)
const uint8 *jolt_hair_result(int inIndex, size_t *outSize)
{
	*outSize = sResults[inIndex].size();
	return (const uint8 *)sResults[inIndex].data();
}

// Runs a groom through HairSettings:
// 1. RestoreBinaryState(inState): the settings before the groom (scalp, materials, fields);
// 2. if inResample > 0: HairSettings::sResample(groom, inResample), the resampled groom in slot 16 (vertices) and 17
//    (strands);
// 3. unless inFlags bit 2: InitRenderAndSimulationStrands(groom), twice with inFlags bit 0; the state in slot 0;
// 4. Init: the state in slot 1 and *outMaxDistSq; with inFlags bit 1 Init runs twice (settings without a scalp only);
// 5. InitCompute with Jolt's CPU compute system: buffer i in slot 2 + i, outBufferPresent[i] = 1 when it exists.
// Returns 0.
int jolt_hair_settings_run(const uint8 *inState, size_t inStateSize, const HairSettings::SVertex *inVertices, uint32 inNumVertices, const HairSettings::SStrand *inStrands, uint32 inNumStrands, uint32 inResample, uint32 inFlags, float *outMaxDistSq, int *outBufferPresent)
{
	ParityInstallHooks();
	jolt_libm_reference_linked();
	for (std::string &s : sResults)
		s.clear();

	Ref<HairSettings> settings = RestoreState(inState, inStateSize);

	Array<HairSettings::SVertex> vertices(inVertices, inVertices + inNumVertices);
	Array<HairSettings::SStrand> strands(inStrands, inStrands + inNumStrands);
	if (inResample > 0)
	{
		HairSettings::sResample(vertices, strands, inResample);
		sResults[16] = std::string((const char *)vertices.data(), vertices.size() * sizeof(HairSettings::SVertex));
		sResults[17] = std::string((const char *)strands.data(), strands.size() * sizeof(HairSettings::SStrand));
	}

	if ((inFlags & 4) == 0)
	{
		settings->InitRenderAndSimulationStrands(vertices, strands);
		if (inFlags & 1)
			settings->InitRenderAndSimulationStrands(vertices, strands);
	}
	sResults[0] = SaveState(*settings);

	settings->Init(*outMaxDistSq);
	if (inFlags & 2)
		settings->Init(*outMaxDistSq);
	sResults[1] = SaveState(*settings);

	Ref<ComputeSystem> system = CreateComputeSystemCPU().Get();
	settings->InitCompute(system);
	for (int i = 0; i < cNumBuffers; ++i)
	{
		ComputeBuffer *buffer = settings->*sBuffers[i];
		outBufferPresent[i] = buffer != nullptr? 1 : 0;
		if (buffer != nullptr)
		{
			const void *data = buffer->Map(ComputeBuffer::EMode::Read);
			sResults[2 + i] = std::string((const char *)data, size_t(buffer->GetSize()) * buffer->GetStride());
			buffer->Unmap();
		}
	}
	return 0;
}

// SkinScalpVertices and PrepareForScalpSkinning of the restored settings, with the matrices as 16 floats each (column
// major): the skinned vertices (x, y, z each) in slot 0, the prepared joint matrices in slot 1
int jolt_hair_skin_scalp(const uint8 *inState, size_t inStateSize, const float *inJointToHair, const float *inJointMatrices, uint32 inNumJoints)
{
	ParityInstallHooks();
	for (std::string &s : sResults)
		s.clear();

	Ref<HairSettings> settings = RestoreState(inState, inStateSize);
	Mat44 joint_to_hair = Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(inJointToHair));
	Array<Mat44> joints;
	for (uint32 i = 0; i < inNumJoints; ++i)
		joints.push_back(Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(inJointMatrices + 16 * i)));

	Array<Vec3> vertices;
	settings->SkinScalpVertices(joint_to_hair, joints.data(), vertices);
	for (Vec3 v : vertices)
	{
		Float3 f;
		v.StoreFloat3(&f);
		sResults[0].append((const char *)&f, sizeof(f));
	}

	Array<Mat44> prepared;
	prepared.resize(settings->mScalpInverseBindPose.size());
	settings->PrepareForScalpSkinning(joint_to_hair, joints.data(), prepared.data());
	for (const Mat44 &m : prepared)
	{
		Float4 columns[4];
		m.StoreFloat4x4(columns);
		sResults[1].append((const char *)columns, sizeof(columns));
	}
	return 0;
}

// The small functions of HairSettings.h for one input: inGradient is (min, max, min fraction, max fraction).
// outValues: Gradient::MakeStepDependent(inTimeRatio) (4), GradientSampler::ToFloat4 (4), GradientSampler::Sample of
// inFraction (1), GradientSampler::Sample of vertex inVertex of the strand [inStart, inEnd) (1),
// Material::GetBendCompliance(inFraction) with inMultiplier and inCompliance (1), Material::NeedsGrid of the gradient as
// the grid velocity factor with inDensityForce (1, 0 or 1), Material::GlobalPoseOnly of the gradient as the global
// pose with inEnableCollision (1, 0 or 1)
void jolt_hair_gradient(const float *inGradient, float inTimeRatio, float inFraction, uint32 inStart, uint32 inEnd, uint32 inVertex, const float *inMultiplier, float inCompliance, float inDensityForce, int inEnableCollision, float *outValues)
{
	jolt_libm_reference_linked();
	HairSettings::Gradient gradient(inGradient[0], inGradient[1], inGradient[2], inGradient[3]);
	HairSettings::Gradient step = gradient.MakeStepDependent(inTimeRatio);
	outValues[0] = step.mMin;
	outValues[1] = step.mMax;
	outValues[2] = step.mMinFraction;
	outValues[3] = step.mMaxFraction;
	HairSettings::GradientSampler sampler(gradient);
	Float4 f = sampler.ToFloat4();
	outValues[4] = f.x;
	outValues[5] = f.y;
	outValues[6] = f.z;
	outValues[7] = f.w;
	outValues[8] = sampler.Sample(inFraction);
	outValues[9] = sampler.Sample(HairSettings::SStrand(inStart, inEnd, 0), inVertex);
	HairSettings::Material material;
	material.mBendComplianceMultiplier = Float4(inMultiplier[0], inMultiplier[1], inMultiplier[2], inMultiplier[3]);
	material.mBendCompliance = inCompliance;
	outValues[10] = material.GetBendCompliance(inFraction);
	material.mGridVelocityFactor = gradient;
	material.mGridDensityForceFactor = inDensityForce;
	outValues[11] = material.NeedsGrid()? 1.0f : 0.0f;
	material.mGlobalPose = gradient;
	material.mEnableCollision = inEnableCollision != 0;
	outValues[12] = material.GlobalPoseOnly()? 1.0f : 0.0f;
}

} // extern "C"
