// Reference implementation of the parity test perf_ragdoll (`zig build parity`, SerializationArchitecture.md P10,
// milestone M9): PerformanceTest's RagdollScene, run by the C++ library. ZoltParity/Physics/PerfRagdollParity.zig runs
// ZoltPerformanceTest's port of the scene and compares.
//
// - The scene is PerformanceTest/RagdollScene.h itself. Its header and PerformanceTest/Layers.h are included in the
//   namespace PerfRagdoll: Layers.h defines the same class names as UnitTests/Layers.h (ObjectLayerPairFilterImpl,
//   BPLayerInterfaceImpl, ...), which other references of the parity library include. In one namespace they would break
//   the one definition rule.
// - The test reads the private members of the scene (the loaded objects and the ragdolls). So `private` is defined as
//   `public` while RagdollScene.h is included. All Jolt headers are included before that.
// - Stage 1 (jolt_pr_load, jolt_pr_result): Load of the scene, then the SaveBinaryState bytes of the RagdollSettings
//   (with shapes and group filters), of its two tables, of the animation, of the sampled pose and of the terrain.
// - Stage 2 (jolt_pr_world_*): one thread configuration of PerformanceTest.cpp's main: the PhysicsSystem of the scene,
//   StartTest, OptimizeBroadPhase, then Update one step at a time, with the SaveState bytes and the hash after each.
//
// All C functions have the prefix jolt_pr_.

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>
#include <Jolt/Skeleton/SkeletonPose.h>

JPH_SUPPRESS_WARNINGS_STD_BEGIN
#include <iostream>
#include <sstream>
#include <string>
JPH_SUPPRESS_WARNINGS_STD_END

using namespace JPH;
using namespace JPH::literals;
using namespace std;

JPH_SUPPRESS_WARNINGS

namespace PerfRagdoll
{
#include "PerformanceTest/Layers.h"
#include "PerformanceTest/PerformanceTestScene.h"
#define private public
#include "PerformanceTest/RagdollScene.h"
#undef private
} // PerfRagdoll

static void PREnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

// The scene of the last jolt_pr_load and the byte strings of stage 1
static PerfRagdoll::RagdollScene *sPRScene = nullptr;
static constexpr int cPRNumResults = 6;
static std::string sPRResults[cPRNumResults];

// The bytes that a function writes to a StreamOut
template <class F>
static std::string PRSave(const F &inWrite)
{
	std::stringstream data;
	StreamOutWrapper stream(data);
	inWrite(stream);
	return data.str();
}

extern "C" {

// Stage 1: create RagdollScene(inNumPilesPerAxis, inPileSize, inVerticalSeparation) and Load it from inAssetPath (with a
// trailing separator). Returns 1 when Load succeeded. The results (jolt_pr_result) are:
// 0: RagdollSettings::SaveBinaryState(stream, true, true)
// 1: the tables: the size and the entries of GetBodyIndexToConstraintIndex, then of GetConstraintIndexToBodyIdxPair
// 2: SkeletalAnimation::SaveBinaryState of the animation
// 3: the sampled pose: the root offset, then the rotation and translation of every joint
// 4: PhysicsScene::SaveBinaryState(stream, true, true) of the terrain
// 5: the parent joint index of every joint of the skeleton
uint32 jolt_pr_load(const char *inAssetPath, uint32 inNumPilesPerAxis, uint32 inPileSize, float inVerticalSeparation)
{
	PREnsureFactory();
	delete sPRScene;
	sPRScene = new PerfRagdoll::RagdollScene(int(inNumPilesPerAxis), int(inPileSize), inVerticalSeparation);
	for (std::string &r : sPRResults)
		r.clear();
	if (!sPRScene->Load(String(inAssetPath)))
		return 0;

	const RagdollSettings *settings = sPRScene->mRagdollSettings;
	sPRResults[0] = PRSave([settings](StreamOut &ioStream) { settings->SaveBinaryState(ioStream, true, true); });
	sPRResults[1] = PRSave([settings](StreamOut &ioStream) {
		ioStream.Write(uint32(settings->GetBodyIndexToConstraintIndex().size()));
		for (int i : settings->GetBodyIndexToConstraintIndex())
			ioStream.Write(i);
		ioStream.Write(uint32(settings->GetConstraintIndexToBodyIdxPair().size()));
		for (const RagdollSettings::BodyIdxPair &p : settings->GetConstraintIndexToBodyIdxPair())
		{
			ioStream.Write(p.first);
			ioStream.Write(p.second);
		}
	});
	const SkeletalAnimation *animation = sPRScene->mAnimation;
	sPRResults[2] = PRSave([animation](StreamOut &ioStream) { animation->SaveBinaryState(ioStream); });
	const SkeletonPose &pose = sPRScene->mPose;
	sPRResults[3] = PRSave([&pose](StreamOut &ioStream) {
		ioStream.Write(pose.GetRootOffset());
		for (const SkeletonPose::JointState &j : pose.GetJoints())
		{
			ioStream.Write(j.mRotation);
			ioStream.Write(j.mTranslation);
		}
	});
	const PhysicsScene *background = sPRScene->mBackground;
	sPRResults[4] = PRSave([background](StreamOut &ioStream) { background->SaveBinaryState(ioStream, true, true); });
	sPRResults[5] = PRSave([settings](StreamOut &ioStream) {
		for (const Skeleton::Joint &j : settings->GetSkeleton()->GetJoints())
			ioStream.Write(j.mParentJointIndex);
	});
	return 1;
}

// Byte string inIndex of the last jolt_pr_load (see there)
const uint8 *jolt_pr_result(uint32 inIndex, uint64 *outSize)
{
	const std::string &r = sPRResults[inIndex];
	*outSize = r.size();
	return reinterpret_cast<const uint8 *>(r.data());
}

// Stage 2: one thread configuration of PerformanceTest.cpp's main for the scene of the last jolt_pr_load
struct PRWorld
{
	explicit					PRWorld(uint32 inNumThreads) :
		mTempAllocator(sPRScene->GetTempAllocatorSizeMB() * 1024 * 1024),
		mJobSystem(cMaxPhysicsJobs, cMaxPhysicsBarriers, int(inNumThreads))
	{
	}

	TempAllocatorImpl			mTempAllocator;
	JobSystemThreadPool			mJobSystem;
	PerfRagdoll::BPLayerInterfaceImpl mBroadPhaseLayerInterface;
	PerfRagdoll::ObjectVsBroadPhaseLayerFilterImpl mObjectVsBroadPhaseLayerFilter;
	PerfRagdoll::ObjectLayerPairFilterImpl mObjectVsObjectLayerFilter;
	PhysicsSystem				mSystem;
	std::string					mState;
};

// Create the physics system with a JobSystemThreadPool with inNumThreads worker threads (PerformanceTest's -t=<n> is n - 1), StartTest with motion quality inMotionQuality (0
// Discrete, 1 LinearCast) and OptimizeBroadPhase
void *jolt_pr_world_create(uint32 inNumThreads, uint32 inMotionQuality)
{
	PRWorld *world = new PRWorld(inNumThreads);
	world->mSystem.Init(sPRScene->GetMaxBodies(), 0, sPRScene->GetMaxBodyPairs(), sPRScene->GetMaxContactConstraints(), world->mBroadPhaseLayerInterface, world->mObjectVsBroadPhaseLayerFilter, world->mObjectVsObjectLayerFilter);
	sPRScene->StartTest(world->mSystem, inMotionQuality == 0? EMotionQuality::Discrete : EMotionQuality::LinearCast);
	world->mSystem.OptimizeBroadPhase();
	return world;
}

// The number of ragdolls that StartTest created
uint32 jolt_pr_world_num_ragdolls(void *inWorld)
{
	return uint32(sPRScene->mRagdolls.size());
}

// One step of PerformanceTest.cpp's main: UpdateTest, then Update. Returns the EPhysicsUpdateError bits.
uint32 jolt_pr_world_update(void *inWorld)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	constexpr float cDeltaTime = 1.0f / 60.0f;
	sPRScene->UpdateTest(world->mSystem, world->mTempAllocator, cDeltaTime);
	return uint32(world->mSystem.Update(cDeltaTime, 1, &world->mTempAllocator, &world->mJobSystem));
}

// PhysicsSystem::SaveState(EStateRecorderState::All), valid until the next call
const uint8 *jolt_pr_world_state(void *inWorld, uint64 *outSize)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	StateRecorderImpl recorder;
	world->mSystem.SaveState(recorder);
	world->mState = recorder.GetData();
	*outSize = world->mState.size();
	return reinterpret_cast<const uint8 *>(world->mState.data());
}

// The hash of PerformanceTest.cpp's main: the positions and rotations of all bodies, then UpdateHash
uint64 jolt_pr_world_hash(void *inWorld)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	uint64 hash = HashBytes(nullptr, 0); // Ensure we start with the proper seed
	BodyInterface &bi = world->mSystem.GetBodyInterfaceNoLock();
	BodyIDVector body_ids;
	world->mSystem.GetBodies(body_ids);
	for (BodyID id : body_ids)
	{
		RVec3 pos = bi.GetPosition(id);
		hash = HashBytes(&pos, 3 * sizeof(Real), hash);
		Quat rot = bi.GetRotation(id);
		hash = HashBytes(&rot, sizeof(Quat), hash);
	}
	sPRScene->UpdateHash(hash);
	return hash;
}

// StopTest, then destroy the world
void jolt_pr_world_destroy(void *inWorld)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	sPRScene->StopTest(world->mSystem);
	delete world;
}

// For the state decoder of SimulationParity.zig: does body inBodyID have motion properties
uint32 jolt_pr_body_has_motion_properties(void *inWorld, uint32 inBodyID)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	BodyLockRead lock(world->mSystem.GetBodyLockInterfaceNoLock(), BodyID(inBodyID));
	return lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked() != nullptr? 1 : 0;
}

// For the state decoder: the size of the SaveState of constraint inIndex
uint32 jolt_pr_constraint_state_size(void *inWorld, uint32 inIndex)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	Constraints constraints = world->mSystem.GetConstraints();
	if (inIndex >= constraints.size())
		return 0;
	StateRecorderImpl recorder;
	constraints[inIndex]->SaveState(recorder);
	return uint32(recorder.GetData().size());
}

// For the state decoder: the subtype of constraint inIndex (0xffffffff: none)
uint32 jolt_pr_constraint_sub_type(void *inWorld, uint32 inIndex)
{
	PRWorld *world = static_cast<PRWorld *>(inWorld);
	Constraints constraints = world->mSystem.GetConstraints();
	return inIndex < constraints.size()? uint32(constraints[inIndex]->GetSubType()) : 0xffffffff;
}

// Destroy the scene of the last jolt_pr_load
void jolt_pr_unload()
{
	delete sPRScene;
	sPRScene = nullptr;
	for (std::string &r : sPRResults)
		r = std::string();
}

} // extern "C"
