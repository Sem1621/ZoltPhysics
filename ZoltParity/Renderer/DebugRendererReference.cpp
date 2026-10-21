// C ABI wrappers for ZoltParity/Renderer/DebugRendererParity.zig: Jolt's debug renderer, recorder, playback and
// DebugRendererSimple driven by the op stream of ParityRenderer.zig (Docs/Zolt/RemainingFeaturesArchitecture.md
// section 4, "Draw call log"). Every function builds its renderer, runs and destroys it again, so only one Jolt
// DebugRenderer exists at a time (Jolt's constructor asserts that DebugRenderer::sInstance is null).
//
// Group D3 (PhysicsDrawParity.zig): jolt_dr_body_world_* builds a BodyManager with bodies of catalogue shapes
// (DRBodyDesc must match the extern struct of PhysicsDrawParity.zig), jolt_dr_run_physics_ops runs the ops on the
// constraints of a constraint draw world of ConstraintsReference.cpp (jolt_cs_draw_world_*) or on the bodies of a body
// world.
//
// The functions exist in both renderer configurations. Without JPH_DEBUG_RENDERER they return null and
// jolt_dr_available() returns 0: the parity layers skip then.

#include <Jolt/Jolt.h>

#include <ZoltParity/ParityHooks.h>

#ifdef JPH_DEBUG_RENDERER
	#include <Jolt/Core/StreamWrapper.h>
	#include <Jolt/Physics/Body/Body.h>
	#include <Jolt/Physics/Body/BodyCreationSettings.h>
	#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
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

/// A body of the body world (defined below with JPH_DEBUG_RENDERER, only a pointer type without it)
struct DRBodyDesc;

#ifdef JPH_DEBUG_RENDERER

using namespace ZoltParity;

/// The shape of a catalogue of PairwiseReference.cpp
extern "C" const Shape *pw_catalogue_shape(void *inCatalogue, uint32 inIndex);

/// The constraint draw world of ConstraintsReference.cpp (group D3)
extern "C" Constraint *jolt_cs_draw_world_constraint(void *inWorld, uint32 inIndex);
extern "C" ConstraintManager *jolt_cs_draw_world_constraint_manager(void *inWorld);
extern "C" BodyManager *jolt_cs_draw_world_body_manager(void *inWorld);
extern "C" void jolt_cs_draw_world_step(void *inWorld, uint32 inIndex);

/// A body of the body world of group D3: a catalogue shape. Must match DRBodyDesc in PhysicsDrawParity.zig.
struct DRBodyDesc
{
	Real							mPosition[3];
	float							mRotation[4];
	float							mLinearVelocity[3];
	float							mAngularVelocity[3];
	float							mMass;						///< The mass of MassAndInertiaProvided / CalculateInertia (inertia: identity)
	uint32							mShape;						///< Index into the catalogue
	uint32							mMotionType;				///< EMotionType
	uint32							mOverrideMassProperties;	///< EOverrideMassProperties
	uint32							mAllowedDOFs;				///< EAllowedDOFs
	uint32							mIsSensor;
	uint32							mAllowSleeping;
	uint32							mActivate;					///< ActivateBodies after adding the body (not static)
};

/// One broad phase layer
class DRLayers final : public BroadPhaseLayerInterface
{
public:
	virtual uint					GetNumBroadPhaseLayers() const override						{ return 1; }
	virtual BroadPhaseLayer			GetBroadPhaseLayer(ObjectLayer) const override				{ return BroadPhaseLayer(0); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

/// The body world of group D3
struct DRBodyWorld
{
	DRLayers						mLayers;
	BodyManager						mBodyManager;
	Array<Body *>					mBodies;
};

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

/// Run the ops with the shapes of a PairwiseReference.cpp catalogue (D2's shape ops) on a ParityRenderer (inKind 0), a
/// SimpleParityRenderer (inKind 1) or a DebugRendererRecorder (inKind 2). Parts: [0] the log of the ops (the .jor bytes
/// after the construction for the recorder), [1] the results of the shape ops. The static draw flags of the shapes are
/// reset afterwards.
JoltDRResult *jolt_dr_run_shape_ops(void *inCatalogue, uint32 inNumShapes, const uint8 *inOps, uint64 inSize, int inKind)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	Array<const Shape *> shapes;
	for (uint32 i = 0; i < inNumShapes; ++i)
		shapes.push_back(pw_catalogue_shape(inCatalogue, i));
	ParityShapeContext context { shapes.data(), inNumShapes, { } };

	JoltDRResult *result = new JoltDRResult;
	ParitySetShapeDrawFlags(0);
	if (inKind == 0)
	{
		ParityRenderer renderer;
		renderer.mLog.mBytes.clear();
		RunParityOps(renderer, inOps, size_t(inSize), nullptr, nullptr, &context);
		result->mParts.push_back(renderer.mLog.mBytes);
	}
	else if (inKind == 1)
	{
		SimpleParityRenderer renderer;
		renderer.mLog.mBytes.clear();
		RunParityOps(renderer, inOps, size_t(inSize), &renderer, nullptr, &context);
		result->mParts.push_back(renderer.mLog.mBytes);
	}
	else
	{
		std::stringstream stream;
		StreamOutWrapper wrapper(stream);
		DebugRendererRecorder recorder(wrapper);
		size_t construction_size = sStreamBytes(stream).size();
		RunParityOps(recorder, inOps, size_t(inSize), nullptr, &recorder, &context);
		std::vector<uint8> all = sStreamBytes(stream);
		result->mParts.push_back(std::vector<uint8>(all.begin() + construction_size, all.end()));
	}
	ParitySetShapeDrawFlags(0);
	result->mParts.push_back(context.mResults);
	return result;
#else
	(void)inCatalogue; (void)inNumShapes; (void)inOps; (void)inSize; (void)inKind;
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

/// Body::ApplyBuoyancyImpulse (inKind 0) or Body::GetSubmergedVolume (inKind 1) of a body of BodyReference.cpp
/// (jolt_body_ptr), drawn into a ParityRenderer with Shape::sDrawSubmergedVolumes set (group D2). inF holds the surface
/// normal and the other arguments of ApplyBuoyancyImpulse in the layout of BodyOp in BodyParity.zig. Parts: [0] the log
/// of the call, [1] its results (u32 applied, or 5 floats: total volume, submerged volume, relative center of
/// buoyancy).
JoltDRResult *jolt_dr_body_buoyancy(void *inBody, const Real *inSurfacePosition, const float *inF, int inKind)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	Body &body = *static_cast<Body *>(inBody);
	RVec3 surface_position(inSurfacePosition[0], inSurfacePosition[1], inSurfacePosition[2]);
	Vec3 surface_normal(inF[0], inF[1], inF[2]);
	JoltDRResult *result = new JoltDRResult;
	ParitySetShapeDrawFlags(1); // Shape::sDrawSubmergedVolumes
	{
		ParityRenderer renderer;
		renderer.mLog.mBytes.clear();
		std::vector<uint8> values;
		if (inKind == 0)
		{
			uint32 applied = body.ApplyBuoyancyImpulse(surface_position, surface_normal, inF[3], inF[4], inF[5], Vec3(inF[6], inF[7], inF[8]), Vec3(inF[9], inF[10], inF[11]), inF[12])? 1 : 0;
			values.resize(sizeof(applied));
			memcpy(values.data(), &applied, sizeof(applied));
		}
		else
		{
			float out[5];
			Vec3 center_of_buoyancy;
			body.GetSubmergedVolume(surface_position, surface_normal, out[0], out[1], center_of_buoyancy);
			out[2] = center_of_buoyancy.GetX();
			out[3] = center_of_buoyancy.GetY();
			out[4] = center_of_buoyancy.GetZ();
			values.resize(sizeof(out));
			memcpy(values.data(), out, sizeof(out));
		}
		result->mParts.push_back(renderer.mLog.mBytes);
		result->mParts.push_back(std::move(values));
	}
	ParitySetShapeDrawFlags(0);
	return result;
#else
	(void)inBody; (void)inSurfacePosition; (void)inF; (void)inKind;
	return nullptr;
#endif
}

/// The size of DRBodyDesc (checked by PhysicsDrawParity.zig)
uint32 jolt_dr_body_desc_size()
{
#ifdef JPH_DEBUG_RENDERER
	return sizeof(DRBodyDesc);
#else
	return 0;
#endif
}

/// The body world of group D3: a BodyManager with the bodies of inBodies in order (AllocateBody, AddBody,
/// SetInBroadPhaseInternal, ActivateBodies), their shapes from the catalogue of PairwiseReference.cpp
void *jolt_dr_body_world_create(void *inCatalogue, const DRBodyDesc *inBodies, uint32 inCount)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	DRBodyWorld *world = new DRBodyWorld;
	world->mBodyManager.Init(inCount, 0, world->mLayers);
	for (const DRBodyDesc *d = inBodies; d < inBodies + inCount; ++d)
	{
		BodyCreationSettings settings(pw_catalogue_shape(inCatalogue, d->mShape), RVec3(d->mPosition[0], d->mPosition[1], d->mPosition[2]), Quat(d->mRotation[0], d->mRotation[1], d->mRotation[2], d->mRotation[3]), EMotionType(d->mMotionType), 0);
		settings.mLinearVelocity = Vec3(d->mLinearVelocity[0], d->mLinearVelocity[1], d->mLinearVelocity[2]);
		settings.mAngularVelocity = Vec3(d->mAngularVelocity[0], d->mAngularVelocity[1], d->mAngularVelocity[2]);
		settings.mOverrideMassProperties = EOverrideMassProperties(d->mOverrideMassProperties);
		settings.mMassPropertiesOverride.mMass = d->mMass;
		settings.mMassPropertiesOverride.mInertia = Mat44::sIdentity();
		settings.mAllowedDOFs = EAllowedDOFs(d->mAllowedDOFs);
		settings.mIsSensor = d->mIsSensor != 0;
		settings.mAllowSleeping = d->mAllowSleeping != 0;
		Body *body = world->mBodyManager.AllocateBody(settings);
		world->mBodyManager.AddBody(body);
		body->SetInBroadPhaseInternal(true);
		if (d->mActivate != 0 && !body->IsStatic())
		{
			BodyID id = body->GetID();
			world->mBodyManager.ActivateBodies(&id, 1);
		}
		world->mBodies.push_back(body);
	}
	return world;
#else
	(void)inCatalogue; (void)inBodies; (void)inCount;
	return nullptr;
#endif
}

void jolt_dr_body_world_destroy(void *inWorld)
{
#ifdef JPH_DEBUG_RENDERER
	DRBodyWorld *world = static_cast<DRBodyWorld *>(inWorld);
	for (size_t i = world->mBodies.size(); i > 0; --i)
	{
		Body *body = world->mBodies[i - 1];
		BodyID id = body->GetID();
		world->mBodyManager.DeactivateBodies(&id, 1);
		body->SetInBroadPhaseInternal(false);
		world->mBodyManager.DestroyBodies(&id, 1);
	}
	delete world;
#else
	(void)inWorld;
#endif
}

/// Run the ops of D3 on the constraints of a constraint draw world of ConstraintsReference.cpp (inConstraintWorld, its
/// first inNumConstraints constraints, its ConstraintManager and BodyManager) or on the bodies of a body world
/// (inBodyWorld), on a ParityRenderer (inKind 0), a SimpleParityRenderer (inKind 1) or a DebugRendererRecorder (inKind
/// 2). Parts: [0] the log of the ops (the .jor bytes after the construction for the recorder), [1] the results.
JoltDRResult *jolt_dr_run_physics_ops(void *inConstraintWorld, uint32 inNumConstraints, void *inBodyWorld, const uint8 *inOps, uint64 inSize, int inKind)
{
#ifdef JPH_DEBUG_RENDERER
	ParityInstallHooks();

	ParityPhysicsContext context;
	if (inConstraintWorld != nullptr)
	{
		for (uint32 i = 0; i < inNumConstraints; ++i)
			context.mConstraints.push_back(jolt_cs_draw_world_constraint(inConstraintWorld, i));
		context.mStepConstraint = [inConstraintWorld](uint32 inIndex) { jolt_cs_draw_world_step(inConstraintWorld, inIndex); };
		context.mConstraintManager = jolt_cs_draw_world_constraint_manager(inConstraintWorld);
		context.mBodyManager = jolt_cs_draw_world_body_manager(inConstraintWorld);
	}
	if (inBodyWorld != nullptr)
	{
		DRBodyWorld *world = static_cast<DRBodyWorld *>(inBodyWorld);
		context.mBodyManager = &world->mBodyManager;
		context.mBodies = world->mBodies;
	}

	JoltDRResult *result = new JoltDRResult;
	if (inKind == 0)
	{
		ParityRenderer renderer;
		renderer.mLog.mBytes.clear();
		RunParityOps(renderer, inOps, size_t(inSize), nullptr, nullptr, nullptr, &context);
		result->mParts.push_back(renderer.mLog.mBytes);
	}
	else if (inKind == 1)
	{
		SimpleParityRenderer renderer;
		renderer.mLog.mBytes.clear();
		RunParityOps(renderer, inOps, size_t(inSize), &renderer, nullptr, nullptr, &context);
		result->mParts.push_back(renderer.mLog.mBytes);
	}
	else
	{
		std::stringstream stream;
		StreamOutWrapper wrapper(stream);
		DebugRendererRecorder recorder(wrapper);
		size_t construction_size = sStreamBytes(stream).size();
		RunParityOps(recorder, inOps, size_t(inSize), nullptr, &recorder, nullptr, &context);
		std::vector<uint8> all = sStreamBytes(stream);
		result->mParts.push_back(std::vector<uint8>(all.begin() + construction_size, all.end()));
	}
	result->mParts.push_back(context.mResults);
	return result;
#else
	(void)inConstraintWorld; (void)inNumConstraints; (void)inBodyWorld; (void)inOps; (void)inSize; (void)inKind;
	return nullptr;
#endif
}

} // extern "C"
