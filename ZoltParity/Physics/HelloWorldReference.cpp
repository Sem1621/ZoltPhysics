// Reference implementation for ZoltParity/Physics/HelloWorldParity.zig (milestone M5): the scene of
// HelloWorld/HelloWorld.cpp behind a C ABI, so that the Zig test can step it in lockstep with Zolt.
//
// The layer classes are copied from HelloWorld.cpp. Its listeners print a line per callback; here they record the
// callback with its arguments (the SimRecord layout of SimScenes.zig, like SimulationReference.cpp), and
// jolt_hello_world_step_line formats HelloWorld's "Step ..." line with a std::stringstream exactly like HelloWorld
// streams it to cout. The system parameters, the bodies and the update are HelloWorld's: 1024 bodies, 0 body mutexes,
// 1024 body pairs, 1024 contact constraints, a 10 MB TempAllocatorImpl, a JobSystemThreadPool, 1 / 60 s and 1 collision
// step per update.

#include <Jolt/Jolt.h>

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Core/Mutex.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;
using namespace JPH::literals;

namespace {

// Layer that objects can be in, determines which other objects it can collide with (HelloWorld.cpp)
namespace Layers
{
	static constexpr ObjectLayer NON_MOVING = 0;
	static constexpr ObjectLayer MOVING = 1;
	static constexpr ObjectLayer NUM_LAYERS = 2;
};

/// Class that determines if two object layers can collide (HelloWorld.cpp)
class ObjectLayerPairFilterImpl : public ObjectLayerPairFilter
{
public:
	virtual bool					ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
	{
		switch (inObject1)
		{
		case Layers::NON_MOVING:
			return inObject2 == Layers::MOVING; // Non moving only collides with moving
		case Layers::MOVING:
			return true; // Moving collides with everything
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// Broadphase layers (HelloWorld.cpp)
namespace BroadPhaseLayers
{
	static constexpr BroadPhaseLayer NON_MOVING(0);
	static constexpr BroadPhaseLayer MOVING(1);
	static constexpr uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation (HelloWorld.cpp)
class BPLayerInterfaceImpl final : public BroadPhaseLayerInterface
{
public:
									BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual uint					GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual BroadPhaseLayer			GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		switch ((BroadPhaseLayer::Type)inLayer)
		{
		case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
		case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
		default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	BroadPhaseLayer					mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer (HelloWorld.cpp)
class ObjectVsBroadPhaseLayerFilterImpl : public ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool				ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1)
		{
		case Layers::NON_MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING;
		case Layers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// Must match SimRecord in SimScenes.zig
struct HelloWorldRecord
{
	uint32				mKind;
	uint32				mIDs[4];
	uint32				mData[8];
};

// Must match RecordKind in SimScenes.zig
enum class ERecordKind : uint32
{
	BodyActivated = 0,
	BodyDeactivated = 1,
	ContactValidate = 3,
	ContactAdded = 4,
	ContactPersisted = 5,
	ContactRemoved = 6,
};

static uint32 FloatBits(float inValue)
{
	uint32 bits;
	memcpy(&bits, &inValue, sizeof(bits));
	return bits;
}

// FNV-1a (32 bit), hashBytes of SimScenes.zig
static uint32 HashBytes(uint32 inHash, const void *inData, size_t inSize)
{
	uint32 hash = inHash;
	const uint8 *data = static_cast<const uint8 *>(inData);
	for (size_t i = 0; i < inSize; ++i)
	{
		hash ^= data[i];
		hash *= 16777619u;
	}
	return hash;
}

// The hash of the base offset and the contact points of a manifold (hashManifold of HelloWorldParity.zig)
static uint32 HashManifold(const ContactManifold &inManifold)
{
	uint32 hash = 2166136261u;
	Real offset[3] = { inManifold.mBaseOffset.GetX(), inManifold.mBaseOffset.GetY(), inManifold.mBaseOffset.GetZ() };
	hash = HashBytes(hash, offset, sizeof(offset));
	for (const ContactPoints *points : { &inManifold.mRelativeContactPointsOn1, &inManifold.mRelativeContactPointsOn2 })
		for (Vec3 p : *points)
		{
			float v[3] = { p.GetX(), p.GetY(), p.GetZ() };
			hash = HashBytes(hash, v, sizeof(v));
		}
	return hash;
}

// The callbacks of the listeners, called from the job system's threads
class Recorder
{
public:
	void						Add(const HelloWorldRecord &inRecord)
	{
		UniqueLock lock(mMutex);
		mRecords.push_back(inRecord);
	}

	Mutex						mMutex;
	Array<HelloWorldRecord>		mRecords;
};

// HelloWorld's MyContactListener, recording instead of printing
class MyContactListener : public ContactListener
{
public:
	// See: ContactListener
	virtual ValidateResult	OnContactValidate(const Body &inBody1, const Body &inBody2, RVec3Arg inBaseOffset, const CollideShapeResult &inCollisionResult) override
	{
		// cout << "Contact validate callback" << endl;
		HelloWorldRecord r = { };
		r.mKind = uint32(ERecordKind::ContactValidate);
		r.mIDs[0] = inBody1.GetID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inBody2.GetID().GetIndexAndSequenceNumber();
		r.mIDs[2] = inCollisionResult.mSubShapeID1.GetValue();
		r.mIDs[3] = inCollisionResult.mSubShapeID2.GetValue();
		r.mData[0] = FloatBits(inCollisionResult.mPenetrationDepth);
		r.mData[1] = FloatBits(inCollisionResult.mPenetrationAxis.GetX());
		r.mData[2] = FloatBits(inCollisionResult.mPenetrationAxis.GetY());
		r.mData[3] = FloatBits(inCollisionResult.mPenetrationAxis.GetZ());
		mRecorder->Add(r);

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void			OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		// cout << "A contact was added" << endl;
		Record(ERecordKind::ContactAdded, inBody1, inBody2, inManifold, ioSettings);
	}

	virtual void			OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override
	{
		// cout << "A contact was persisted" << endl;
		Record(ERecordKind::ContactPersisted, inBody1, inBody2, inManifold, ioSettings);
	}

	virtual void			OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{
		// cout << "A contact was removed" << endl;
		HelloWorldRecord r = { };
		r.mKind = uint32(ERecordKind::ContactRemoved);
		r.mIDs[0] = inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inSubShapePair.GetSubShapeID1().GetValue();
		r.mIDs[2] = inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber();
		r.mIDs[3] = inSubShapePair.GetSubShapeID2().GetValue();
		mRecorder->Add(r);
	}

	Recorder *				mRecorder = nullptr;

private:
	void					Record(ERecordKind inKind, const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, const ContactSettings &inSettings)
	{
		HelloWorldRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inBody1.GetID().GetIndexAndSequenceNumber();
		r.mIDs[1] = inBody2.GetID().GetIndexAndSequenceNumber();
		r.mIDs[2] = inManifold.mSubShapeID1.GetValue();
		r.mIDs[3] = inManifold.mSubShapeID2.GetValue();
		r.mData[0] = FloatBits(inManifold.mWorldSpaceNormal.GetX());
		r.mData[1] = FloatBits(inManifold.mWorldSpaceNormal.GetY());
		r.mData[2] = FloatBits(inManifold.mWorldSpaceNormal.GetZ());
		r.mData[3] = FloatBits(inManifold.mPenetrationDepth);
		r.mData[4] = uint32(inManifold.mRelativeContactPointsOn1.size()) | ((inSettings.mIsSensor? 1u : 0u) << 8);
		r.mData[5] = FloatBits(inSettings.mCombinedFriction);
		r.mData[6] = FloatBits(inSettings.mCombinedRestitution);
		r.mData[7] = HashManifold(inManifold);
		mRecorder->Add(r);
	}
};

// HelloWorld's MyBodyActivationListener, recording instead of printing
class MyBodyActivationListener : public BodyActivationListener
{
public:
	virtual void		OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		// cout << "A body got activated" << endl;
		Record(ERecordKind::BodyActivated, inBodyID, inBodyUserData);
	}

	virtual void		OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		// cout << "A body went to sleep" << endl;
		Record(ERecordKind::BodyDeactivated, inBodyID, inBodyUserData);
	}

	Recorder *			mRecorder = nullptr;

private:
	void				Record(ERecordKind inKind, const BodyID &inBodyID, uint64 inBodyUserData)
	{
		HelloWorldRecord r = { };
		r.mKind = uint32(inKind);
		r.mIDs[0] = inBodyID.GetIndexAndSequenceNumber();
		r.mData[0] = uint32(inBodyUserData);
		r.mData[1] = uint32(inBodyUserData >> 32);
		mRecorder->Add(r);
	}
};

// The objects of HelloWorld's main(), in the order it creates them
class HelloWorld
{
public:
	// We need a temp allocator for temporary allocations during the physics update (10 MB)
	TempAllocatorImpl					mTempAllocator { 10 * 1024 * 1024 };

	// We need a job system that will execute physics jobs on multiple threads
	JobSystemThreadPool					mJobSystem;

	BPLayerInterfaceImpl				mBroadPhaseLayerInterface;
	ObjectVsBroadPhaseLayerFilterImpl	mObjectVsBroadPhaseLayerFilter;
	ObjectLayerPairFilterImpl			mObjectVsObjectLayerFilter;
	PhysicsSystem						mPhysicsSystem;
	Recorder							mRecorder;
	MyBodyActivationListener			mBodyActivationListener;
	MyContactListener					mContactListener;
	BodyID								mFloorID;
	BodyID								mSphereID;

	// We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
	static constexpr float				cDeltaTime = 1.0f / 60.0f;

	// inNumThreads: the worker threads of the job system (-1: thread::hardware_concurrency() - 1 like HelloWorld)
	explicit							HelloWorld(int inNumThreads) :
		mJobSystem(cMaxPhysicsJobs, cMaxPhysicsBarriers, inNumThreads)
	{
		// HelloWorld's constants
		const uint cMaxBodies = 1024;
		const uint cNumBodyMutexes = 0;
		const uint cMaxBodyPairs = 1024;
		const uint cMaxContactConstraints = 1024;

		// Now we can create the actual physics system.
		mPhysicsSystem.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, mBroadPhaseLayerInterface, mObjectVsBroadPhaseLayerFilter, mObjectVsObjectLayerFilter);

		// The listeners
		mBodyActivationListener.mRecorder = &mRecorder;
		mPhysicsSystem.SetBodyActivationListener(&mBodyActivationListener);
		mContactListener.mRecorder = &mRecorder;
		mPhysicsSystem.SetContactListener(&mContactListener);

		BodyInterface &body_interface = mPhysicsSystem.GetBodyInterface();

		// Next we can create a rigid body to serve as the floor, we make a large box
		BoxShapeSettings floor_shape_settings(Vec3(100.0f, 1.0f, 100.0f));
		floor_shape_settings.SetEmbedded();
		ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
		ShapeRefC floor_shape = floor_shape_result.Get();
		BodyCreationSettings floor_settings(floor_shape, RVec3(0.0_r, -1.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Static, Layers::NON_MOVING);
		Body *floor = body_interface.CreateBody(floor_settings);
		body_interface.AddBody(floor->GetID(), EActivation::DontActivate);
		mFloorID = floor->GetID();

		// Now create a dynamic body to bounce on the floor
		BodyCreationSettings sphere_settings(new SphereShape(0.5f), RVec3(0.0_r, 2.0_r, 0.0_r), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
		mSphereID = body_interface.CreateAndAddBody(sphere_settings, EActivation::Activate);

		// Now you can interact with the dynamic body, in this case we're going to give it a velocity.
		body_interface.SetLinearVelocity(mSphereID, Vec3(0.0f, -5.0f, 0.0f));

		// Optional step: Before starting the physics simulation you can optimize the broad phase.
		mPhysicsSystem.OptimizeBroadPhase();
	}

										~HelloWorld()
	{
		BodyInterface &body_interface = mPhysicsSystem.GetBodyInterface();

		// Remove the sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
		body_interface.RemoveBody(mSphereID);

		// Destroy the sphere. After this the sphere ID is no longer valid.
		body_interface.DestroyBody(mSphereID);

		// Remove and destroy the floor
		body_interface.RemoveBody(mFloorID);
		body_interface.DestroyBody(mFloorID);
	}
};

// HelloWorld's RegisterDefaultAllocator, Factory and RegisterTypes (same guard as BasicsReference.cpp, RegisterTypes()
// runs once for the whole parity binary)
static void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

} // namespace

extern "C"
{

// Create the HelloWorld scene with a job system of inNumThreads worker threads (-1: hardware_concurrency() - 1)
void *jolt_hello_world_create(int inNumThreads)
{
	EnsureFactory();
	return new HelloWorld(inNumThreads);
}

// HelloWorld's cleanup (remove and destroy the bodies), then the destructors of its objects
void jolt_hello_world_destroy(void *inWorld)
{
	delete static_cast<HelloWorld *>(inWorld);
}

// body_interface.IsActive(sphere_id): the condition of HelloWorld's loop
uint32 jolt_hello_world_is_active(void *inWorld)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	return world->mPhysicsSystem.GetBodyInterface().IsActive(world->mSphereID)? 1 : 0;
}

// The body IDs of the floor and the sphere
void jolt_hello_world_body_ids(void *inWorld, uint32 *outFloorID, uint32 *outSphereID)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	*outFloorID = world->mFloorID.GetIndexAndSequenceNumber();
	*outSphereID = world->mSphereID.GetIndexAndSequenceNumber();
}

// The position and velocity of the sphere that HelloWorld prints before every step
void jolt_hello_world_sphere_state(void *inWorld, Real *outPosition, float *outVelocity)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	BodyInterface &body_interface = world->mPhysicsSystem.GetBodyInterface();
	RVec3 position = body_interface.GetCenterOfMassPosition(world->mSphereID);
	Vec3 velocity = body_interface.GetLinearVelocity(world->mSphereID);
	outPosition[0] = position.GetX();
	outPosition[1] = position.GetY();
	outPosition[2] = position.GetZ();
	outVelocity[0] = velocity.GetX();
	outVelocity[1] = velocity.GetY();
	outVelocity[2] = velocity.GetZ();
}

// HelloWorld's line for step inStep, streamed like HelloWorld streams it to cout (without the endl)
uint32 jolt_hello_world_step_line(void *inWorld, uint32 inStep, char *outChars, uint32 inCapacity)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	BodyInterface &body_interface = world->mPhysicsSystem.GetBodyInterface();
	uint step = inStep;
	RVec3 position = body_interface.GetCenterOfMassPosition(world->mSphereID);
	Vec3 velocity = body_interface.GetLinearVelocity(world->mSphereID);
	std::stringstream cout;
	cout << "Step " << step << ": Position = (" << position.GetX() << ", " << position.GetY() << ", " << position.GetZ() << "), Velocity = (" << velocity.GetX() << ", " << velocity.GetY() << ", " << velocity.GetZ() << ")";
	std::string line = cout.str();
	uint32 size = uint32(line.size());
	if (size <= inCapacity)
		memcpy(outChars, line.data(), size);
	return size;
}

// physics_system.Update(cDeltaTime, cCollisionSteps, &temp_allocator, &job_system), returns the EPhysicsUpdateError bits
uint32 jolt_hello_world_update(void *inWorld)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	const int cCollisionSteps = 1;
	return uint32(world->mPhysicsSystem.Update(HelloWorld::cDeltaTime, cCollisionSteps, &world->mTempAllocator, &world->mJobSystem));
}

// PhysicsSystem::SaveState(EStateRecorderState::All), returns the size (the bytes are written when they fit)
uint32 jolt_hello_world_save_state(void *inWorld, uint8 *outBytes, uint32 inCapacity)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	StateRecorderImpl recorder;
	world->mPhysicsSystem.SaveState(recorder);
	std::string data = recorder.GetData();
	uint32 size = uint32(data.size());
	if (size <= inCapacity)
		memcpy(outBytes, data.data(), size);
	return size;
}

// The callback records since the last call (all of them are taken when they fit, otherwise none; returns the count)
uint32 jolt_hello_world_take_records(void *inWorld, HelloWorldRecord *outRecords, uint32 inCapacity)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	UniqueLock lock(world->mRecorder.mMutex);
	uint32 count = uint32(world->mRecorder.mRecords.size());
	if (count <= inCapacity)
	{
		if (count > 0)
			memcpy(outRecords, world->mRecorder.mRecords.data(), count * sizeof(HelloWorldRecord));
		world->mRecorder.mRecords.clear();
	}
	return count;
}

// Whether a body has motion properties (for the state decoder of SimulationParity.zig)
uint32 jolt_hello_world_body_has_motion_properties(void *inWorld, uint32 inBodyID)
{
	HelloWorld *world = static_cast<HelloWorld *>(inWorld);
	BodyLockRead lock(world->mPhysicsSystem.GetBodyLockInterfaceNoLock(), BodyID(inBodyID));
	return lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked() != nullptr? 1 : 0;
}

} // extern "C"
