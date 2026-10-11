// Reference implementation for the LargeIslandSplitter parity tests (`zig build parity`): C ABI wrappers that drive
// Jolt's LargeIslandSplitter like PhysicsSystem's solver jobs do. ZoltParity/Physics/LargeIslandSplitterParity.zig
// builds the same scenes with Zolt and compares the logs written here word by word.
//
// A scene (LISSceneDesc) is a BodyManager with box bodies (AllocateBody + AddBody, marked as being in the broad phase,
// activated, like BodyReference.cpp), constraints (LISTestConstraint below: a TwoBodyConstraint whose BuildIslandSplits
// is TwoBodyConstraint's, or AssignToNonParallelSplit(mBody1) like VehicleConstraint, mirroring
// zolt.test_constraints.TestConstraint and the parity file's NonParallelTestConstraint) and contacts given as body
// pairs. The contacts are stored the way ContactConstraintManager::GetAffectedBodies reads them: the mBody1 / mBody2
// pointers at the start of a record at offset i * cLISContactStride in mConstraints (set through the explicit
// instantiation trick below, like ConstraintPartsReference.cpp). An update links the islands like Jolt (BuildIslands of
// the constraints, then the contacts like ContactConstraintManager::CreateConstraint), finalizes them, prepares the
// splitter, converts the contact indices to offsets and splits every island in island order (one SolveVelocityConstraints
// job), then fetches and marks the batches of the velocity steps and, after PrepareForSolvePositions, of the position
// steps. The order of fetches and marks follows a decision list (several batches can be out at the same time, they are
// marked in any order), the same on both sides. Everything is logged as uint32 words. Booleans are passed as uint32
// (never bool, see the porting guide).

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/LargeIslandSplitter.h>
#include <Jolt/Physics/IslandBuilder.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Body/BodyManager.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/CalculateSolverSteps.h>
#include <Jolt/Physics/Constraints/ContactConstraintManager.h>

#include <cstddef>
#include <cstring>
#include <vector>

using namespace JPH;

namespace
{

// Must match LISBodyDesc in LargeIslandSplitterParity.zig
struct LISBodyDesc
{
	uint32				mMotionType;		///< EMotionType
	uint32				mActivate;
	uint32				mVelocitySteps;
	uint32				mPositionSteps;
};

// Must match LISConstraintDesc in LargeIslandSplitterParity.zig
struct LISConstraintDesc
{
	uint32				mBody1;
	uint32				mBody2;
	uint32				mVelocitySteps;
	uint32				mPositionSteps;
	uint32				mNonParallel;
};

// Must match LISContactDesc in LargeIslandSplitterParity.zig
struct LISContactDesc
{
	uint32				mBody1;
	uint32				mBody2;
};

// Must match LISSceneDesc in LargeIslandSplitterParity.zig
struct LISSceneDesc
{
	const LISBodyDesc *	mBodies;
	const LISConstraintDesc *mConstraints;
	const LISContactDesc *mContacts;
	const uint32 *		mDecisions;
	uint32				mNumBodies;
	uint32				mNumConstraints;
	uint32				mNumContacts;
	uint32				mNumDecisions;
	uint32				mNumVelocitySteps;
	uint32				mNumPositionSteps;
	uint32				mMaxOutstanding;
	uint32				mNumUpdates;
};

// Distance between the contact records in the constraint buffer of the contact manager
static constexpr uint32 cLISContactStride = 24;

// Private access to the members of the splitter and the contact manager (explicit instantiation ignores access checks)
template <typename Tag, typename Tag::type M>
struct LISRob
{
	friend typename Tag::type LISGet(Tag) { return M; }
};

#define LIS_ROB(Name, Class, Type, Member) \
	struct Name { using type = Type Class::*; friend type LISGet(Name); }; \
	template struct LISRob<Name, &Class::Member>;

LIS_ROB(LISNumActiveBodies, LargeIslandSplitter, uint32, mNumActiveBodies)
LIS_ROB(LISSplitMasks, LargeIslandSplitter, uint32 *, mSplitMasks)
LIS_ROB(LISSplitIdx, LargeIslandSplitter, uint32 *, mContactAndConstraintsSplitIdx)
LIS_ROB(LISIndices, LargeIslandSplitter, uint32 *, mContactAndConstraintIndices)
LIS_ROB(LISSize, LargeIslandSplitter, uint, mContactAndConstraintsSize)
LIS_ROB(LISNextFree, LargeIslandSplitter, atomic<uint>, mContactAndConstraintsNextFree)
LIS_ROB(LISNumSplitIslands, LargeIslandSplitter, uint, mNumSplitIslands)
LIS_ROB(LISSplitIslands, LargeIslandSplitter, LargeIslandSplitter::Splits *, mSplitIslands)
LIS_ROB(LISNextSplitIsland, LargeIslandSplitter, atomic<uint>, mNextSplitIsland)
LIS_ROB(LISContactConstraints, ContactConstraintManager, uint8 *, mConstraints)

// Broad phase layer = object layer % 2
class LISBPLayerInterface final : public BroadPhaseLayerInterface
{
public:
	virtual uint				GetNumBroadPhaseLayers() const override						{ return 2; }
	virtual BroadPhaseLayer		GetBroadPhaseLayer(ObjectLayer inLayer) const override		{ return BroadPhaseLayer(BroadPhaseLayer::Type(inLayer % 2)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *		GetBroadPhaseLayerName(BroadPhaseLayer) const override		{ return "Layer"; }
#endif
};

// A two body constraint that only takes part in the island building and splitting
class LISTestConstraint final : public TwoBodyConstraint
{
public:
								LISTestConstraint(Body &inBody1, Body &inBody2, const TwoBodyConstraintSettings &inSettings, bool inNonParallel) : TwoBodyConstraint(inBody1, inBody2, inSettings), mNonParallel(inNonParallel) { }

	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::User1; }
	virtual void				NotifyShapeChanged(const BodyID &, Vec3Arg) override		{ }
	virtual void				ResetWarmStart() override									{ }
	virtual void				SetupVelocityConstraint(float) override						{ }
	virtual void				WarmStartVelocityConstraint(float) override					{ }
	virtual bool				SolveVelocityConstraint(float) override						{ return false; }
	virtual bool				SolvePositionConstraint(float, float) override				{ return false; }
#ifdef JPH_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *) const override				{ }
#endif
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override					{ return nullptr; }
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sIdentity(); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sIdentity(); }

	// Like VehicleConstraint when mNonParallel is set
	virtual uint				BuildIslandSplits(LargeIslandSplitter &ioSplitter) const override
	{
		if (mNonParallel)
			return ioSplitter.AssignToNonParallelSplit(mBody1);
		return TwoBodyConstraint::BuildIslandSplits(ioSplitter);
	}

private:
	bool						mNonParallel;
};

// Log of uint32 words, stops writing at the capacity but keeps counting
class LISLog
{
public:
						LISLog(uint32 *outWords, uint32 inCapacity) : mWords(outWords), mCapacity(inCapacity) { }

	void				W(uint32 inValue)						{ if (mSize < mCapacity) mWords[mSize] = inValue; ++mSize; }
	void				W64(uint64 inValue)						{ W(uint32(inValue)); W(uint32(inValue >> 32)); }
	void				I(int inValue)							{ W(uint32(inValue)); }
	void				B(bool inValue)							{ W(inValue? 1 : 0); }
	uint32				GetSize() const							{ return mSize; }

private:
	uint32 *			mWords;
	uint32				mCapacity;
	uint32				mSize = 0;
};

static void LISEnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

// Log a split island
static void LISLogSplits(LISLog &ioLog, const LargeIslandSplitter::Splits &inSplits)
{
	ioLog.W(inSplits.mIslandIndex);
	ioLog.W(inSplits.mNumSplits);
	ioLog.I(inSplits.mNumIterations);
	ioLog.I(inSplits.mNumVelocitySteps);
	ioLog.I(inSplits.mNumPositionSteps);
	ioLog.W64(inSplits.mStatus.load(memory_order_relaxed));
	ioLog.W(inSplits.mItemsProcessed.load(memory_order_relaxed));
	for (uint s = 0; s < LargeIslandSplitter::cNumSplits; ++s)
	{
		// The parallel splits, then the non parallel split
		if (s >= inSplits.mNumSplits)
			s = LargeIslandSplitter::cNonParallelSplitIdx;

		const LargeIslandSplitter::Split &split = inSplits.mSplits[s];
		ioLog.W(split.mContactBufferBegin);
		ioLog.W(split.mContactBufferEnd);
		ioLog.W(split.mConstraintBufferBegin);
		ioLog.W(split.mConstraintBufferEnd);
		ioLog.W(split.GetNumContacts());
		ioLog.W(split.GetNumConstraints());
		ioLog.W(split.GetNumItems());

		uint32 begin, end;
		inSplits.GetConstraintsInSplit(s, begin, end);
		ioLog.W(begin);
		ioLog.W(end);
		inSplits.GetContactsInSplit(s, begin, end);
		ioLog.W(begin);
		ioLog.W(end);
	}
}

// Log the state of the splitter
static void LISLogSplitter(LISLog &ioLog, LargeIslandSplitter &inSplitter, bool inWithBuffers)
{
	uint size = inSplitter.*LISGet(LISSize());
	uint num_split_islands = inSplitter.*LISGet(LISNumSplitIslands());
	ioLog.W(0x5717E000);
	ioLog.W(inSplitter.*LISGet(LISNumActiveBodies()));
	ioLog.W(size);
	ioLog.W((inSplitter.*LISGet(LISNextFree())).load(memory_order_relaxed));
	ioLog.W(num_split_islands);
	ioLog.W((inSplitter.*LISGet(LISNextSplitIsland())).load(memory_order_relaxed));
	if (inWithBuffers && size > 0)
	{
		const uint32 *split_idx = inSplitter.*LISGet(LISSplitIdx());
		const uint32 *indices = inSplitter.*LISGet(LISIndices());
		for (uint i = 0; i < size; ++i)
			ioLog.W(split_idx[i]);
		for (uint i = 0; i < size; ++i)
			ioLog.W(indices[i]);
	}
	const LargeIslandSplitter::Splits *split_islands = inSplitter.*LISGet(LISSplitIslands());
	uint num_created = (inSplitter.*LISGet(LISNextSplitIsland())).load(memory_order_relaxed);
	for (uint i = 0; i < num_created; ++i)
	{
		LISLogSplits(ioLog, split_islands[i]);
		ioLog.W(inSplitter.GetIslandIndex(i));
	}
}

// A scene in a BodyManager with the island builder, the splitter and a contact manager that only resolves contacts
class LISWorld
{
public:
						LISWorld(const LISSceneDesc &inDesc) :
		mDesc(inDesc),
		mContactManager(mSettings)
	{
		mSettings.mNumVelocitySteps = inDesc.mNumVelocitySteps;
		mSettings.mNumPositionSteps = inDesc.mNumPositionSteps;

		mBodyManager.Init(inDesc.mNumBodies + 1, 0, mLayers);
		mIslandBuilder.Init(inDesc.mNumBodies + 1);

		// Bodies
		RefConst<Shape> box = new BoxShape(Vec3::sReplicate(0.5f));
		for (uint32 i = 0; i < inDesc.mNumBodies; ++i)
		{
			const LISBodyDesc &b = inDesc.mBodies[i];
			EMotionType motion_type = EMotionType(b.mMotionType);
			BodyCreationSettings settings(box, RVec3::sZero(), Quat::sIdentity(), motion_type, motion_type == EMotionType::Static? 0 : 1);
			settings.mNumVelocityStepsOverride = b.mVelocitySteps;
			settings.mNumPositionStepsOverride = b.mPositionSteps;
			Body *body = mBodyManager.AllocateBody(settings);
			mBodyManager.AddBody(body);
			body->SetInBroadPhaseInternal(true);
			BodyID id = body->GetID();
			if (b.mActivate != 0 && !body->IsStatic())
				mBodyManager.ActivateBodies(&id, 1);
			mBodies.push_back(body);
		}

		// Constraints
		for (uint32 i = 0; i < inDesc.mNumConstraints; ++i)
		{
			const LISConstraintDesc &c = inDesc.mConstraints[i];
			Ref<Constraint> constraint = new LISTestConstraint(*mBodies[c.mBody1], *mBodies[c.mBody2], mConstraintSettings, c.mNonParallel != 0);
			constraint->SetNumVelocityStepsOverride(c.mVelocitySteps);
			constraint->SetNumPositionStepsOverride(c.mPositionSteps);
			mActiveConstraints.push_back(constraint.GetPtr());
			mConstraints.push_back(constraint);
		}

		// Contact records: the bodies at the start of each record, read by GetAffectedBodies
		mContactBuffer.resize(size_t(inDesc.mNumContacts) * cLISContactStride / sizeof(uint64) + 64, 0);
		uint8 *records = reinterpret_cast<uint8 *>(mContactBuffer.data());
		for (uint32 i = 0; i < inDesc.mNumContacts; ++i)
		{
			Body *bodies[2] = { mBodies[inDesc.mContacts[i].mBody1], mBodies[inDesc.mContacts[i].mBody2] };
			memcpy(records + size_t(i) * cLISContactStride, bodies, sizeof(bodies));
		}
		mContactManager.*LISGet(LISContactConstraints()) = records;
	}

						~LISWorld()
	{
		mContactManager.*LISGet(LISContactConstraints()) = nullptr;
		mConstraints.clear();
	}

	// One physics update of the islands and the splitter
	void				Update(LISLog &ioLog)
	{
		TempAllocator *temp = &mTempAllocator;
		uint32 num_contacts = mDesc.mNumContacts;

		// Build the islands
		mIslandBuilder.PrepareContactConstraints(num_contacts, temp);
		mIslandBuilder.PrepareNonContactConstraints(uint32(mActiveConstraints.size()), temp);
		for (uint32 i = 0; i < uint32(mActiveConstraints.size()); ++i)
			mActiveConstraints[i]->BuildIslands(i, mIslandBuilder, mBodyManager);
		for (uint32 i = 0; i < num_contacts; ++i)
		{
			// Like ContactConstraintManager::CreateConstraint
			Body &body1 = *mBodies[mDesc.mContacts[i].mBody1];
			Body &body2 = *mBodies[mDesc.mContacts[i].mBody2];
			bool body1_dynamic = body1.IsDynamic();
			bool body2_dynamic = body2.IsDynamic();
			BodyID body_ids[2];
			int num_bodies = 0;
			if (body1_dynamic && !body1.IsActive())
				body_ids[num_bodies++] = body1.GetID();
			if (body2_dynamic && !body2.IsActive())
				body_ids[num_bodies++] = body2.GetID();
			if (num_bodies > 0)
				mBodyManager.ActivateBodies(body_ids, num_bodies);
			if (body1_dynamic && body2_dynamic)
				mIslandBuilder.LinkBodies(body1.GetIndexInActiveBodiesInternal(), body2.GetIndexInActiveBodiesInternal());
			if (body1_dynamic)
				mIslandBuilder.LinkContact(i, body1.GetIndexInActiveBodiesInternal());
			else
				mIslandBuilder.LinkContact(i, body2.GetIndexInActiveBodiesInternal());
		}
		mIslandBuilder.Finalize(mBodyManager.GetActiveBodiesUnsafe(EBodyType::RigidBody), mBodyManager.GetNumActiveBodies(EBodyType::RigidBody), num_contacts, temp);

		// Prepare the splitter
		mSplitter.Prepare(mIslandBuilder, mBodyManager.GetNumActiveBodies(EBodyType::RigidBody), temp);
		ioLog.W(0x9E9A0000);
		ioLog.W(uint32(mTempAllocator.GetUsage()));
		LISLogSplitter(ioLog, mSplitter, false); // The buffers are not initialized yet

		// The islands, with the contacts converted to offsets (ConstraintIdxToConstraintOffset), then split them in order
		uint32 num_islands = mIslandBuilder.GetNumIslands();
		ioLog.W(0x15A4D000);
		ioLog.W(num_islands);
		for (uint32 island = 0; island < num_islands; ++island)
		{
			BodyID *bodies_begin, *bodies_end;
			mIslandBuilder.GetBodiesInIsland(island, bodies_begin, bodies_end);
			ioLog.W(uint32(bodies_end - bodies_begin));
			for (BodyID *b = bodies_begin; b < bodies_end; ++b)
				ioLog.W(b->GetIndexAndSequenceNumber());

			uint32 *contacts_begin, *contacts_end, *constraints_begin, *constraints_end;
			if (!mIslandBuilder.GetContactsInIsland(island, contacts_begin, contacts_end))
				contacts_begin = contacts_end = nullptr;
			if (!mIslandBuilder.GetConstraintsInIsland(island, constraints_begin, constraints_end))
				constraints_begin = constraints_end = nullptr;
			ioLog.W(uint32(contacts_end - contacts_begin));
			for (uint32 *c = contacts_begin; c < contacts_end; ++c)
			{
				*c *= cLISContactStride;
				ioLog.W(*c);
			}
			ioLog.W(uint32(constraints_end - constraints_begin));
			for (uint32 *c = constraints_begin; c < constraints_end; ++c)
				ioLog.W(*c);

			CalculateSolverSteps steps_calculator(mSettings);
			ioLog.B(mSplitter.SplitIsland(island, mIslandBuilder, mBodyManager, mContactManager, mActiveConstraints.data(), steps_calculator));
		}
		LISLogSplitter(ioLog, mSplitter, true);

		// Velocity steps
		ioLog.W(0x7E10C000);
		RunBatches(ioLog);
		LISLogSplitter(ioLog, mSplitter, true);

		// Position steps
		mSplitter.PrepareForSolvePositions();
		ioLog.W(0x9051710A);
		LISLogSplitter(ioLog, mSplitter, true);
		RunBatches(ioLog);
		LISLogSplitter(ioLog, mSplitter, true);

		// The end of the update
		mSplitter.Reset(temp);
		mIslandBuilder.ResetIslands(temp);
		ioLog.W(0xE4D00000);
		ioLog.W(uint32(mTempAllocator.GetUsage()));
		LISLogSplitter(ioLog, mSplitter, true);
	}

private:
	struct Batch
	{
		uint			mSplitIslandIndex;
		uint32 *		mConstraintsBegin;
		uint32 *		mConstraintsEnd;
		uint32 *		mContactsBegin;
		uint32 *		mContactsEnd;
	};

	// Fetch and mark batches in the order of the decision list until all batches are done
	void				RunBatches(LISLog &ioLog)
	{
		const uint32 *indices = mSplitter.*LISGet(LISIndices());
		std::vector<Batch> outstanding;
		bool force_mark = false;
		uint32 decision = 0;
		for (uint32 step = 0; step < 10000000; ++step)
		{
			uint32 d = mDesc.mDecisions[decision++ % mDesc.mNumDecisions];
			bool fetch = outstanding.empty() || (!force_mark && outstanding.size() < mDesc.mMaxOutstanding && (d & 1) != 0);
			if (fetch)
			{
				Batch batch;
				bool first_iteration = false;
				LargeIslandSplitter::EStatus status = mSplitter.FetchNextBatch(batch.mSplitIslandIndex, batch.mConstraintsBegin, batch.mConstraintsEnd, batch.mContactsBegin, batch.mContactsEnd, first_iteration);
				ioLog.W(0xF0000000 | uint32(status));
				if (status == LargeIslandSplitter::EStatus::BatchRetrieved)
				{
					ioLog.W(batch.mSplitIslandIndex);
					ioLog.W(uint32(batch.mConstraintsBegin - indices));
					ioLog.W(uint32(batch.mConstraintsEnd - indices));
					ioLog.W(uint32(batch.mContactsBegin - indices));
					ioLog.W(uint32(batch.mContactsEnd - indices));
					ioLog.B(first_iteration);
					ioLog.W(mSplitter.GetIslandIndex(batch.mSplitIslandIndex));
					outstanding.push_back(batch);
				}
				else
				{
					if (outstanding.empty())
						return;
					force_mark = true;
				}
			}
			else
			{
				// Mark one of the outstanding batches as processed
				size_t which = (d >> 1) % outstanding.size();
				Batch batch = outstanding[which];
				outstanding.erase(outstanding.begin() + which);
				bool last_iteration = false, final_batch = false;
				mSplitter.MarkBatchProcessed(batch.mSplitIslandIndex, batch.mConstraintsBegin, batch.mConstraintsEnd, batch.mContactsBegin, batch.mContactsEnd, last_iteration, final_batch);
				ioLog.W(0xA0000000);
				ioLog.W(batch.mSplitIslandIndex);
				ioLog.B(last_iteration);
				ioLog.B(final_batch);
				force_mark = false;
			}
		}
		ioLog.W(0xDEADDEAD); // Did not finish
	}

	const LISSceneDesc &	mDesc;
	PhysicsSettings			mSettings;
	LISBPLayerInterface		mLayers;
	BodyManager				mBodyManager;
	IslandBuilder			mIslandBuilder;
	LargeIslandSplitter		mSplitter;
	ContactConstraintManager mContactManager;
	TempAllocatorImpl		mTempAllocator { 64 * 1024 * 1024 };
	FixedConstraintSettings	mConstraintSettings;
	std::vector<Body *>		mBodies;
	std::vector<Ref<Constraint>> mConstraints;
	std::vector<Constraint *> mActiveConstraints;
	std::vector<uint64>		mContactBuffer;
};

} // namespace

extern "C" {

// Run the updates of a scene, returns the number of words of the log (which may exceed the capacity)
uint32 jolt_lis_run(const LISSceneDesc *inDesc, uint32 *outLog, uint32 inCapacity)
{
	LISEnsureFactory();
	LISLog log(outLog, inCapacity);
	{
		LISWorld world(*inDesc);
		for (uint32 u = 0; u < inDesc->mNumUpdates; ++u)
			world.Update(log);
	}
	return log.GetSize();
}

// AssignSplit / AssignToNonParallelSplit on given split masks: inOps holds (kind, body1, body2) triplets (kind 0:
// AssignSplit, 1: AssignToNonParallelSplit(body1)), the log gets the result and all masks after each call
uint32 jolt_lis_assign(const LISBodyDesc *inBodies, uint32 inNumBodies, const uint32 *inMasks, const uint32 *inOps, uint32 inNumOps, uint32 *outLog, uint32 inCapacity)
{
	LISEnsureFactory();
	LISLog log(outLog, inCapacity);

	// A body manager with the bodies
	LISBPLayerInterface layers;
	BodyManager body_manager;
	body_manager.Init(inNumBodies + 1, 0, layers);
	RefConst<Shape> box = new BoxShape(Vec3::sReplicate(0.5f));
	std::vector<Body *> bodies;
	for (uint32 i = 0; i < inNumBodies; ++i)
	{
		EMotionType motion_type = EMotionType(inBodies[i].mMotionType);
		BodyCreationSettings settings(box, RVec3::sZero(), Quat::sIdentity(), motion_type, motion_type == EMotionType::Static? 0 : 1);
		Body *body = body_manager.AllocateBody(settings);
		body_manager.AddBody(body);
		body->SetInBroadPhaseInternal(true);
		BodyID id = body->GetID();
		if (inBodies[i].mActivate != 0 && !body->IsStatic())
			body_manager.ActivateBodies(&id, 1);
		bodies.push_back(body);
	}

	// The masks of the active bodies (Prepare would allocate them)
	uint32 num_active = body_manager.GetNumActiveBodies(EBodyType::RigidBody);
	std::vector<uint32> masks(inMasks, inMasks + num_active);
	LargeIslandSplitter splitter;
	splitter.*LISGet(LISNumActiveBodies()) = num_active;
	splitter.*LISGet(LISSplitMasks()) = masks.data();
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const uint32 *op = inOps + 3 * i;
		uint split = op[0] == 0? splitter.AssignSplit(bodies[op[1]], bodies[op[2]]) : splitter.AssignToNonParallelSplit(bodies[op[1]]);
		log.W(split);
		for (uint32 m : masks)
			log.W(m);
	}
	splitter.*LISGet(LISSplitMasks()) = nullptr;
	splitter.*LISGet(LISNumActiveBodies()) = 0;
	return log.GetSize();
}

// Layout of LargeIslandSplitter::Splits: size, alignment, offsets of mStatus and mItemsProcessed, size of Split
void jolt_lis_layout(uint32 *outValues)
{
	outValues[0] = uint32(sizeof(LargeIslandSplitter::Splits));
	outValues[1] = uint32(alignof(LargeIslandSplitter::Splits));
	outValues[2] = uint32(offsetof(LargeIslandSplitter::Splits, mStatus));
	outValues[3] = uint32(offsetof(LargeIslandSplitter::Splits, mItemsProcessed));
	outValues[4] = uint32(sizeof(LargeIslandSplitter::Split));
	outValues[5] = LargeIslandSplitter::cNumSplits;
	outValues[6] = LargeIslandSplitter::cNonParallelSplitIdx;
	outValues[7] = LargeIslandSplitter::cLargeIslandTreshold;
}

} // extern "C"
