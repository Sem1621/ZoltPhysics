// Reference implementation for the Jolt/Core concurrency parity tests (`zig build parity`): thin C ABI wrappers around
// FixedSizeFreeList, LockFreeHashMap, MutexArray and Semaphore. ZoltParity/Core/ConcurrencyParity.zig runs the same
// single threaded operation sequences on these and on the Zolt ports and checks that both return the same results
// (object indices, handles, iteration order, values).
//
// Conventions: every container is created on the heap and passed around as an opaque pointer. Keys and values of the
// hash maps are passed as uint64 and truncated to the key / value type of the map variant.

#include <Jolt/Jolt.h>
#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Core/LockFreeHashMap.h>
#include <Jolt/Core/Mutex.h>
#include <Jolt/Core/MutexArray.h>
#include <Jolt/Core/Semaphore.h>

#include <memory>

using namespace JPH;

namespace {

/// Object stored in the free list, not trivially destructible so that DestructObjectBatch walks the batch
struct FreeListObject
{
	explicit			FreeListObject(uint32 inValue) : mValue(inValue), mCheck(~inValue) { }
						~FreeListObject() { mValue = 0xdeadbeef; mCheck = 0xdeadbeef; }

	uint32				mValue;
	uint32				mCheck;
};

/// A free list and the batch of objects that is being built for it
struct FreeList
{
	FixedSizeFreeList<FreeListObject> mList;
	FixedSizeFreeList<FreeListObject>::Batch mBatch;
};

/// Type erased lock free hash map + allocator + a number of allocator contexts (as if each context is a thread)
class HashMapBase
{
public:
	virtual				~HashMapBase() = default;
	virtual void		SetNumBuckets(uint32 inNumBuckets) = 0;
	virtual void		Clear(bool inClearAllocator) = 0;
	virtual void		ResetContexts() = 0;
	virtual uint32		Insert(uint32 inContext, uint64 inKey, uint64 inValue, int inExtraBytes) = 0;
	virtual uint32		Find(uint64 inKey, uint64 *outValue) const = 0;
	virtual uint32		Iterate(uint64 *outKeys, uint64 *outValues, uint32 *outHandles, uint32 inMaxCount) = 0;
	virtual uint32		GetAll(uint32 *outHandles, uint32 inMaxCount) const = 0;
};

static constexpr uint32 cMaxContexts = 8;

template <class Key, class Value>
class HashMap final : public HashMapBase
{
public:
	using Map = LockFreeHashMap<Key, Value>;

						HashMap(uint32 inObjectStoreSize, uint32 inMaxBuckets, const uint32 *inBlockSizes, uint32 inNumContexts) :
		mMap(mAllocator),
		mNumContexts(inNumContexts)
	{
		JPH_ASSERT(inNumContexts <= cMaxContexts);
		mAllocator.Init(inObjectStoreSize);
		mMap.Init(inMaxBuckets);
		for (uint32 i = 0; i < inNumContexts; ++i)
			mBlockSizes[i] = inBlockSizes[i];
		ResetContexts();
	}

	virtual void		SetNumBuckets(uint32 inNumBuckets) override
	{
		mMap.SetNumBuckets(inNumBuckets);
	}

	virtual void		Clear(bool inClearAllocator) override
	{
		mMap.Clear();
		if (inClearAllocator)
		{
			mAllocator.Clear();
			ResetContexts();
		}
	}

	virtual void		ResetContexts() override
	{
		for (uint32 i = 0; i < mNumContexts; ++i)
			mContexts[i].reset(new LFHMAllocatorContext(mAllocator, mBlockSizes[i]));
	}

	virtual uint32		Insert(uint32 inContext, uint64 inKey, uint64 inValue, int inExtraBytes) override
	{
		Key key = Key(inKey);
		typename Map::KeyValue *kv = mMap.Create(*mContexts[inContext], key, Hash<Key> { } (key), inExtraBytes, Value(inValue));
		return kv != nullptr? mMap.ToHandle(kv) : Map::cInvalidHandle;
	}

	virtual uint32		Find(uint64 inKey, uint64 *outValue) const override
	{
		Key key = Key(inKey);
		const typename Map::KeyValue *kv = mMap.Find(key, Hash<Key> { } (key));
		if (kv == nullptr)
			return Map::cInvalidHandle;
		*outValue = kv->GetValue();
		return mMap.ToHandle(kv);
	}

	virtual uint32		Iterate(uint64 *outKeys, uint64 *outValues, uint32 *outHandles, uint32 inMaxCount) override
	{
		uint32 count = 0;
		for (typename Map::Iterator it = mMap.begin(); it != mMap.end(); ++it)
		{
			if (count < inMaxCount)
			{
				typename Map::KeyValue &kv = *it;
				outKeys[count] = kv.GetKey();
				outValues[count] = kv.GetValue();
				outHandles[count] = mMap.ToHandle(&kv);
			}
			++count;
		}
		return count;
	}

	virtual uint32		GetAll(uint32 *outHandles, uint32 inMaxCount) const override
	{
		Array<const typename Map::KeyValue *> all;
		mMap.GetAllKeyValues(all);
		for (uint32 i = 0; i < min(uint32(all.size()), inMaxCount); ++i)
			outHandles[i] = mMap.ToHandle(all[i]);
		return uint32(all.size());
	}

private:
	LFHMAllocator		mAllocator;
	Map					mMap;
	uint32				mNumContexts;
	uint32				mBlockSizes[cMaxContexts];
	std::unique_ptr<LFHMAllocatorContext> mContexts[cMaxContexts];
};

} // namespace

extern "C"
{

// Core/FixedSizeFreeList.h
void *jolt_fsfl_create(uint32 inMaxObjects, uint32 inPageSize)
{
	RegisterDefaultAllocator();
	FreeList *list = new FreeList;
	list->mList.Init(inMaxObjects, inPageSize);
	return list;
}
void jolt_fsfl_destroy(void *inList)					{ delete static_cast<FreeList *>(inList); }
uint32 jolt_fsfl_construct(void *inList, uint32 inValue) { return static_cast<FreeList *>(inList)->mList.ConstructObject(inValue); }
void jolt_fsfl_destruct(void *inList, uint32 inIndex)	{ static_cast<FreeList *>(inList)->mList.DestructObject(inIndex); }
void jolt_fsfl_destruct_ptr(void *inList, uint32 inIndex)
{
	FreeList *list = static_cast<FreeList *>(inList);
	list->mList.DestructObject(&list->mList.Get(inIndex));
}
void jolt_fsfl_add_to_batch(void *inList, uint32 inIndex)
{
	FreeList *list = static_cast<FreeList *>(inList);
	list->mList.AddObjectToBatch(list->mBatch, inIndex);
}
void jolt_fsfl_destruct_batch(void *inList)
{
	FreeList *list = static_cast<FreeList *>(inList);
	list->mList.DestructObjectBatch(list->mBatch);
	list->mBatch = FixedSizeFreeList<FreeListObject>::Batch();
}
uint32 jolt_fsfl_get(void *inList, uint32 inIndex)		{ return static_cast<FreeList *>(inList)->mList.Get(inIndex).mValue; }
int jolt_fsfl_object_storage_size()						{ return FixedSizeFreeList<FreeListObject>::ObjectStorageSize; }

// Core/LockFreeHashMap.h, inVariant 0: LockFreeHashMap<uint32, uint32>, 1: LockFreeHashMap<uint64, uint64>
void *jolt_lfhm_create(int inVariant, uint32 inObjectStoreSize, uint32 inMaxBuckets, const uint32 *inBlockSizes, uint32 inNumContexts)
{
	RegisterDefaultAllocator();
	if (inVariant == 0)
		return static_cast<HashMapBase *>(new HashMap<uint32, uint32>(inObjectStoreSize, inMaxBuckets, inBlockSizes, inNumContexts));
	else
		return static_cast<HashMapBase *>(new HashMap<uint64, uint64>(inObjectStoreSize, inMaxBuckets, inBlockSizes, inNumContexts));
}
void jolt_lfhm_destroy(void *inMap)						{ delete static_cast<HashMapBase *>(inMap); }
void jolt_lfhm_set_num_buckets(void *inMap, uint32 inNumBuckets) { static_cast<HashMapBase *>(inMap)->SetNumBuckets(inNumBuckets); }
void jolt_lfhm_clear(void *inMap, int inClearAllocator) { static_cast<HashMapBase *>(inMap)->Clear(inClearAllocator != 0); }
void jolt_lfhm_reset_contexts(void *inMap)				{ static_cast<HashMapBase *>(inMap)->ResetContexts(); }
uint32 jolt_lfhm_insert(void *inMap, uint32 inContext, uint64 inKey, uint64 inValue, int inExtraBytes) { return static_cast<HashMapBase *>(inMap)->Insert(inContext, inKey, inValue, inExtraBytes); }
uint32 jolt_lfhm_find(void *inMap, uint64 inKey, uint64 *outValue) { return static_cast<HashMapBase *>(inMap)->Find(inKey, outValue); }
uint32 jolt_lfhm_iterate(void *inMap, uint64 *outKeys, uint64 *outValues, uint32 *outHandles, uint32 inMaxCount) { return static_cast<HashMapBase *>(inMap)->Iterate(outKeys, outValues, outHandles, inMaxCount); }
uint32 jolt_lfhm_get_all(void *inMap, uint32 *outHandles, uint32 inMaxCount) { return static_cast<HashMapBase *>(inMap)->GetAll(outHandles, inMaxCount); }

// Core/MutexArray.h
void *jolt_mutex_array_create(uint32 inNumMutexes)
{
	RegisterDefaultAllocator();
	return new MutexArray<Mutex>(inNumMutexes);
}
void jolt_mutex_array_destroy(void *inArray)			{ delete static_cast<MutexArray<Mutex> *>(inArray); }
uint32 jolt_mutex_array_get_mutex_index(void *inArray, uint32 inObjectIndex) { return static_cast<MutexArray<Mutex> *>(inArray)->GetMutexIndex(inObjectIndex); }

// Core/Semaphore.h
void *jolt_semaphore_create()							{ return new Semaphore; }
void jolt_semaphore_destroy(void *inSemaphore)			{ delete static_cast<Semaphore *>(inSemaphore); }
void jolt_semaphore_release(void *inSemaphore, uint32 inNumber) { static_cast<Semaphore *>(inSemaphore)->Release(inNumber); }
void jolt_semaphore_acquire(void *inSemaphore, uint32 inNumber) { static_cast<Semaphore *>(inSemaphore)->Acquire(inNumber); }
int jolt_semaphore_get_value(void *inSemaphore)		{ return static_cast<Semaphore *>(inSemaphore)->GetValue(); }

} // extern "C"
