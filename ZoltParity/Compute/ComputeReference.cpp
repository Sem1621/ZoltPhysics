// Reference implementation for ZoltParity/Compute/ComputeParity.zig: a generic C ABI over Jolt's CPU compute system
// (JPH_USE_CPU_COMPUTE, Docs/Zolt/RemainingFeaturesArchitecture.md section 4, row "Compute dispatch").
//
// One ComputeSystemCPU for the process, with the shaders of TestComputeWrapper.cpp (registered like
// UnitTests/Compute/ComputeTests.cpp) and of HairWrapper.cpp (HairRegisterShaders). jolt_compute_dispatch runs one
// dispatch of a shader by name: it creates a buffer per binding from the given bytes, binds them by name, dispatches and
// copies the RW buffers back. The Zig side runs the same calls on Zolt's ComputeSystemCPU and compares the bytes.
//
// The hooks: Jolt's ComputeBufferCPU leaves a buffer without data uninitialized (UB4 of R20), Zolt zero-fills it. So
// this file uses the zero-filling hooks of ZoltParity/ParityHooks.h (R1), installed before main and again in every
// exported function that allocates. A filtered parity run links only the reference objects that it uses, so the file
// cannot rely on another file's hooks.
//
// Conventions: no bool parameters (porting guide section 10). ComputeBindingDesc must match the extern struct of
// ComputeParity.zig. All functions are prefixed with jolt_compute_.

#include <Jolt/Jolt.h>
#include <Jolt/Core/RTTI.h>
#include <Jolt/Compute/ComputeSystem.h>
#include <Jolt/Compute/CPU/ComputeSystemCPU.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Shaders/HairWrapper.h>
#include <Jolt/Shaders/HairStructs.h>
#include <Jolt/Shaders/TestComputeBindings.h>
#include <Jolt/Shaders/TestCompute2Bindings.h>
#include <ZoltParity/ParityHooks.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>

JPH_DECLARE_REGISTER_SHADER(TestCompute)
JPH_DECLARE_REGISTER_SHADER(TestCompute2)

using namespace JPH;


// The compute system of the process (never released)
static ComputeSystemCPU *sGetSystem()
{
	ParityInstallHooks();

	static ComputeSystemCPU *sSystem = []() {
		ComputeSystemResult result = CreateComputeSystemCPU();
		ComputeSystemCPU *system = StaticCast<ComputeSystemCPU>(result.Get().GetPtr());
		system->AddRef();
		JPH_REGISTER_SHADER(system, TestCompute);
		JPH_REGISTER_SHADER(system, TestCompute2);
		HairRegisterShaders(system);
		return system;
	}();
	return sSystem;
}

extern "C" {

/// One buffer of a dispatch. Must match ComputeBindingDesc in ComputeParity.zig.
struct ComputeBindingDesc
{
	const char *	mName;			///< Name of the binding in the shader (e.g. "gContext")
	uint8 *			mData;			///< Initial contents (mSize * mStride bytes); an RW buffer receives its contents after the dispatch
	uint64			mSize;			///< Number of elements
	uint32			mStride;		///< Size of an element in bytes
	uint32			mKind;			///< 0 = SetConstantBuffer, 1 = SetBuffer, 2 = SetRWBuffer
	uint32			mType;			///< ComputeBuffer::EType of the buffer
	uint32			mFlags;			///< 1 = bind a null buffer (the queue ignores it), 2 = create the buffer without data
};

/// The IsKindOf checks of TestComputeCPU: ComputeSystem, ComputeSystemCPU, ShapeSettings (1 = true)
void jolt_compute_is_kind_of(uint32 *outValues)
{
	ParityInstallHooks();
	ComputeSystemResult result = CreateComputeSystemCPU();
	Ref<ComputeSystem> system = result.Get();
	outValues[0] = IsKindOf(system.GetPtr(), JPH_RTTI(ComputeSystem))? 1 : 0;
	outValues[1] = IsKindOf(system.GetPtr(), JPH_RTTI(ComputeSystemCPU))? 1 : 0;
	outValues[2] = IsKindOf(system.GetPtr(), JPH_RTTI(ShapeSettings))? 1 : 0;
}

/// Create the shader inName with the group sizes inGroupSize, create and bind the buffers, dispatch inNumGroups groups
/// and copy the RW buffers back. Returns 0 on success, 1 when the shader does not exist ("Compute shader not found").
uint32 jolt_compute_dispatch(const char *inName, const uint32 *inGroupSize, const uint32 *inNumGroups, ComputeBindingDesc *ioBindings, uint32 inNumBindings)
{
	ComputeSystemCPU *system = sGetSystem();

	ComputeShaderResult shader_result = system->CreateComputeShader(inName, inGroupSize[0], inGroupSize[1], inGroupSize[2]);
	if (shader_result.HasError())
		return 1;
	Ref<ComputeShader> shader = shader_result.Get();

	Array<Ref<ComputeBuffer>> buffers(inNumBindings);
	for (uint32 i = 0; i < inNumBindings; ++i)
	{
		const ComputeBindingDesc &b = ioBindings[i];
		if ((b.mFlags & 1) == 0)
			buffers[i] = system->CreateComputeBuffer(ComputeBuffer::EType(b.mType), b.mSize, b.mStride, (b.mFlags & 2) != 0? nullptr : b.mData).Get();
	}

	Ref<ComputeQueue> queue = system->CreateComputeQueue().Get();
	queue->SetShader(shader);
	for (uint32 i = 0; i < inNumBindings; ++i)
		switch (ioBindings[i].mKind)
		{
		case 0:		queue->SetConstantBuffer(ioBindings[i].mName, buffers[i]);	break;
		case 1:		queue->SetBuffer(ioBindings[i].mName, buffers[i]);			break;
		default:	queue->SetRWBuffer(ioBindings[i].mName, buffers[i]);		break;
		}

	// The queue holds the buffers now (like ComputeTests, which releases its references after binding)
	for (Ref<ComputeBuffer> &b : buffers)
		if (b != nullptr && b->GetType() != ComputeBuffer::EType::RWBuffer)
			b = nullptr;

	queue->Dispatch(inNumGroups[0], inNumGroups[1], inNumGroups[2]);
	queue->ExecuteAndWait();

	// Copy the RW buffers back (on the CPU backend the read back buffer is the buffer itself)
	for (uint32 i = 0; i < inNumBindings; ++i)
		if (ioBindings[i].mKind == 2 && buffers[i] != nullptr)
		{
			Ref<ComputeBuffer> readback = buffers[i]->CreateReadBackBuffer().Get();
			memcpy(ioBindings[i].mData, readback->Map(ComputeBuffer::EMode::Read), size_t(ioBindings[i].mSize) * ioBindings[i].mStride);
			readback->Unmap();
		}
	return 0;
}

/// The sizes and member offsets of the structs that C++ code and the shaders share (ShaderCore.h's C++ types), in the
/// order of ComputeParity.zig: for every struct its size, then the offset of every member. Returns the number of
/// values written (at most inMaxValues).
uint32 jolt_compute_layouts(uint32 *outValues, uint32 inMaxValues)
{
	uint32 count = 0;
	auto add = [outValues, inMaxValues, &count](size_t inValue) { if (count < inMaxValues) outValues[count] = uint32(inValue); ++count; };

#define ZOLT_SIZE(type)				add(sizeof(type));
#define ZOLT_OFFSET(type, member)	add(offsetof(type, member));

	ZOLT_SIZE(TestComputeContext)
	ZOLT_OFFSET(TestComputeContext, cFloat3Value)
	ZOLT_OFFSET(TestComputeContext, cUIntValue)
	ZOLT_OFFSET(TestComputeContext, cFloat3Value2)
	ZOLT_OFFSET(TestComputeContext, cUIntValue2)
	ZOLT_OFFSET(TestComputeContext, cNumElements)

	ZOLT_SIZE(TestCompute2Input)
	ZOLT_OFFSET(TestCompute2Input, mMat44Value)
	ZOLT_OFFSET(TestCompute2Input, mMat44MulValue)
	ZOLT_OFFSET(TestCompute2Input, mCompressedVec3)
	ZOLT_OFFSET(TestCompute2Input, mCompressedQuat)

	ZOLT_SIZE(TestCompute2Output)
	ZOLT_OFFSET(TestCompute2Output, mMul3x4Output)
	ZOLT_OFFSET(TestCompute2Output, mMul3x3Output)
	ZOLT_OFFSET(TestCompute2Output, mDecompressedVec3)
	ZOLT_OFFSET(TestCompute2Output, mDecompressedQuat)

	ZOLT_SIZE(JPH_HairSkinWeight)
	ZOLT_OFFSET(JPH_HairSkinWeight, mJointIdx)
	ZOLT_OFFSET(JPH_HairSkinWeight, mWeight)

	ZOLT_SIZE(JPH_HairSkinPoint)
	ZOLT_OFFSET(JPH_HairSkinPoint, mTriangleIndex)
	ZOLT_OFFSET(JPH_HairSkinPoint, mU)
	ZOLT_OFFSET(JPH_HairSkinPoint, mV)
	ZOLT_OFFSET(JPH_HairSkinPoint, mToBishop)

	ZOLT_SIZE(JPH_HairGlobalPoseTransform)
	ZOLT_OFFSET(JPH_HairGlobalPoseTransform, mPosition)
	ZOLT_OFFSET(JPH_HairGlobalPoseTransform, mRotation)

	ZOLT_SIZE(JPH_HairSVertexInfluence)
	ZOLT_OFFSET(JPH_HairSVertexInfluence, mVertexIndex)
	ZOLT_OFFSET(JPH_HairSVertexInfluence, mRelativePosition)
	ZOLT_OFFSET(JPH_HairSVertexInfluence, mWeight)

	ZOLT_SIZE(JPH_HairPosition)
	ZOLT_OFFSET(JPH_HairPosition, mPosition)
	ZOLT_OFFSET(JPH_HairPosition, mRotation)

	ZOLT_SIZE(JPH_HairVelocity)
	ZOLT_OFFSET(JPH_HairVelocity, mVelocity)
	ZOLT_OFFSET(JPH_HairVelocity, mAngularVelocity)

	ZOLT_SIZE(JPH_HairMaterial)
	ZOLT_OFFSET(JPH_HairMaterial, mWorldTransformInfluence)
	ZOLT_OFFSET(JPH_HairMaterial, mGlobalPose)
	ZOLT_OFFSET(JPH_HairMaterial, mSkinGlobalPose)
	ZOLT_OFFSET(JPH_HairMaterial, mGravityFactor)
	ZOLT_OFFSET(JPH_HairMaterial, mHairRadius)
	ZOLT_OFFSET(JPH_HairMaterial, mBendComplianceMultiplier)
	ZOLT_OFFSET(JPH_HairMaterial, mGridVelocityFactor)
	ZOLT_OFFSET(JPH_HairMaterial, mEnableCollision)
	ZOLT_OFFSET(JPH_HairMaterial, mEnableLRA)
	ZOLT_OFFSET(JPH_HairMaterial, mEnableGrid)
	ZOLT_OFFSET(JPH_HairMaterial, mFriction)
	ZOLT_OFFSET(JPH_HairMaterial, mExpLinearDampingDeltaTime)
	ZOLT_OFFSET(JPH_HairMaterial, mExpAngularDampingDeltaTime)
	ZOLT_OFFSET(JPH_HairMaterial, mBendComplianceInvDeltaTimeSq)
	ZOLT_OFFSET(JPH_HairMaterial, mStretchComplianceInvDeltaTimeSq)
	ZOLT_OFFSET(JPH_HairMaterial, mGridDensityForceFactor)
	ZOLT_OFFSET(JPH_HairMaterial, mInertiaMultiplier)
	ZOLT_OFFSET(JPH_HairMaterial, mMaxLinearVelocitySq)
	ZOLT_OFFSET(JPH_HairMaterial, mMaxAngularVelocitySq)

	ZOLT_SIZE(JPH_HairCollisionPlane)
	ZOLT_OFFSET(JPH_HairCollisionPlane, mPlane)
	ZOLT_OFFSET(JPH_HairCollisionPlane, mShapeIndex)

	ZOLT_SIZE(JPH_HairCollisionShape)
	ZOLT_OFFSET(JPH_HairCollisionShape, mCenterOfMass)
	ZOLT_OFFSET(JPH_HairCollisionShape, mLinearVelocity)
	ZOLT_OFFSET(JPH_HairCollisionShape, mAngularVelocity)

	ZOLT_SIZE(JPH_HairUpdateContext)
	ZOLT_OFFSET(JPH_HairUpdateContext, cNumStrands)
	ZOLT_OFFSET(JPH_HairUpdateContext, cNumVertices)
	ZOLT_OFFSET(JPH_HairUpdateContext, cNumGridPoints)
	ZOLT_OFFSET(JPH_HairUpdateContext, cNumRenderVertices)
	ZOLT_OFFSET(JPH_HairUpdateContext, cGridSizeMin2)
	ZOLT_OFFSET(JPH_HairUpdateContext, cTwoDivDeltaTime)
	ZOLT_OFFSET(JPH_HairUpdateContext, cGridSizeMin1)
	ZOLT_OFFSET(JPH_HairUpdateContext, cDeltaTime)
	ZOLT_OFFSET(JPH_HairUpdateContext, cGridOffset)
	ZOLT_OFFSET(JPH_HairUpdateContext, cHalfDeltaTime)
	ZOLT_OFFSET(JPH_HairUpdateContext, cGridScale)
	ZOLT_OFFSET(JPH_HairUpdateContext, cInvDeltaTimeSq)
	ZOLT_OFFSET(JPH_HairUpdateContext, cSubStepGravity)
	ZOLT_OFFSET(JPH_HairUpdateContext, cNumSkinVertices)
	ZOLT_OFFSET(JPH_HairUpdateContext, cGridStride)
	ZOLT_OFFSET(JPH_HairUpdateContext, cNumSkinWeightsPerVertex)
	ZOLT_OFFSET(JPH_HairUpdateContext, cDeltaTransform)
	ZOLT_OFFSET(JPH_HairUpdateContext, cScalpToHead)
	ZOLT_OFFSET(JPH_HairUpdateContext, cDeltaTransformQuat)

	ZOLT_SIZE(JPH_HairIterationContext)
	ZOLT_OFFSET(JPH_HairIterationContext, cAccumulatedDeltaTime)
	ZOLT_OFFSET(JPH_HairIterationContext, cIterationFraction)

#undef ZOLT_SIZE
#undef ZOLT_OFFSET

	return count;
}

/// The shader constants of the bindings headers: cTestComputeGroupSize, cTestCompute2GroupSize, cHairPerVertexBatch,
/// cHairPerGridCellBatch, cHairPerStrandBatch, cHairPerRenderVertexBatch, cHairNumSVertexInfluences
void jolt_compute_constants(int *outValues)
{
	outValues[0] = cTestComputeGroupSize;
	outValues[1] = cTestCompute2GroupSize;
	outValues[2] = cHairPerVertexBatch;
	outValues[3] = cHairPerGridCellBatch;
	outValues[4] = cHairPerStrandBatch;
	outValues[5] = cHairPerRenderVertexBatch;
	outValues[6] = cHairNumSVertexInfluences;
}

} // extern "C"
