// Reference implementation for the Jolt/Core container parity tests (`zig build parity`): C ABI wrappers around
// JPH::UnorderedMap / JPH::UnorderedSet. ZoltParity/Core/ContainersParity.zig runs the same operation scripts on the
// Zolt containers and checks that the results (including bucket indices and iteration order) are identical.
//
// A script is three arrays of length inNumOps: op codes, keys and values (keys / values are truncated to the key /
// value type of the container). Every op appends its results to outResults, see the op codes below. The functions
// return the number of results written, or 0xffffffff if outResults was too small.

#include <Jolt/Jolt.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/UnorderedSet.h>

using namespace JPH;

namespace {

// Op codes, keep in sync with ContainersParity.zig
enum EOp : uint8
{
	OpInsert,				// insert(key [, value]):			inserted, index [, value at index]
	OpIndex,				// map[key] (set: insert):			old value (Value() when new), then assigns value
	OpTryEmplace,			// try_emplace(key, value) (set: insert):	inserted, index [, value at index]
	OpFind,					// find(key):						not_found or index [, value]
	OpEraseKey,				// erase(key):						number of erased elements
	OpEraseIterator,		// erase(find(key)):				not_found or index of the erased element
	OpClear,				// clear()
	OpClearAndKeepMemory,	// ClearAndKeepMemory()
	OpReserve,				// reserve(uint32(key))
	OpRehash,				// rehash(0)
	OpCopySnapshot,			// snapshot of a copy of the container (the copy is discarded)
	OpSnapshot,				// snapshot: size, bucket_count, empty, then index, key [, value] per element in iteration order, then snapshot_end
	OpCopyReplace,			// replace the container by a copy of itself (copy constructor + swap)
	OpEraseFirst,			// erase(begin()):					not_found (empty iteration) or index of the erased element
};

constexpr uint64 cNotFound = ~uint64(0);
constexpr uint64 cSnapshotEnd = ~uint64(1);

// Appends results to the output buffer
class Output
{
public:
					Output(uint64 *inResults, uint32 inMaxResults) : mResults(inResults), mMaxResults(inMaxResults) { }

	void			Add(uint64 inValue)
	{
		if (mCount < mMaxResults)
			mResults[mCount] = inValue;
		else
			mOverflow = true;
		++mCount;
	}

	uint32			GetResult() const { return mOverflow? ~uint32(0) : mCount; }

private:
	uint64 *		mResults;
	uint32			mMaxResults;
	uint32			mCount = 0;
	bool			mOverflow = false;
};

// A hash function that puts 4 consecutive keys in the same bucket and only uses 4 different control values
class ClusteredHash
{
public:
	uint64			operator () (uint32 inValue) const
	{
		return (uint64(inValue >> 2) << 7) | (inValue & 3);
	}
};

template <class Container, class Key, class Value, bool IsMap>
void Snapshot(const Container &inContainer, Output &ioOutput)
{
	ioOutput.Add(inContainer.size());
	ioOutput.Add(inContainer.bucket_count());
	ioOutput.Add(inContainer.empty()? 1 : 0);
	for (typename Container::const_iterator it = inContainer.begin(); it != inContainer.end(); ++it)
	{
		ioOutput.Add(it.mIndex);
		if constexpr (IsMap)
		{
			ioOutput.Add(it->first);
			ioOutput.Add(it->second);
		}
		else
			ioOutput.Add(*it);
	}
	ioOutput.Add(cSnapshotEnd);
}

template <class Container, class Key, class Value, bool IsMap>
uint32 RunScript(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	// The containers allocate through Jolt's allocation hooks
	RegisterDefaultAllocator();

	Output output(outResults, inMaxResults);
	Container container;

	for (uint32 i = 0; i < inNumOps; ++i)
	{
		Key key = Key(inKeys[i]);
		Value value = Value(inValues[i]);

		uint8 op = inOps[i];
		if constexpr (!IsMap)
			if (op == OpIndex || op == OpTryEmplace)
				op = OpInsert;

		switch (op)
		{
		case OpInsert:
		case OpTryEmplace:
			if constexpr (IsMap)
			{
				std::pair<typename Container::iterator, bool> result = op == OpInsert? container.insert({ key, value }) : container.try_emplace(key, value);
				output.Add(result.second? 1 : 0);
				output.Add(result.first.mIndex);
				output.Add(result.first->second);
			}
			else
			{
				std::pair<typename Container::iterator, bool> result = container.insert(key);
				output.Add(result.second? 1 : 0);
				output.Add(result.first.mIndex);
			}
			break;

		case OpIndex:
			if constexpr (IsMap)
			{
				Value &v = container[key];
				output.Add(v);
				v = value;
			}
			break;

		case OpFind:
			{
				typename Container::const_iterator it = static_cast<const Container &>(container).find(key);
				if (it == container.cend())
					output.Add(cNotFound);
				else
				{
					output.Add(it.mIndex);
					if constexpr (IsMap)
						output.Add(it->second);
				}
				break;
			}

		case OpEraseKey:
			output.Add(container.erase(key));
			break;

		case OpEraseIterator:
			{
				typename Container::const_iterator it = static_cast<const Container &>(container).find(key);
				if (it == container.cend())
					output.Add(cNotFound);
				else
				{
					output.Add(it.mIndex);
					container.erase(it);
				}
				break;
			}

		case OpClear:
			container.clear();
			break;

		case OpClearAndKeepMemory:
			container.ClearAndKeepMemory();
			break;

		case OpReserve:
			container.reserve(uint32(inKeys[i]));
			break;

		case OpRehash:
			container.rehash(0);
			break;

		case OpCopySnapshot:
			{
				Container copy(container);
				Snapshot<Container, Key, Value, IsMap>(copy, output);
				break;
			}

		case OpSnapshot:
			Snapshot<Container, Key, Value, IsMap>(container, output);
			break;

		case OpCopyReplace:
			{
				Container copy(container);
				container.swap(copy);
				break;
			}

		case OpEraseFirst:
			{
				typename Container::const_iterator it = container.cbegin();
				if (it == container.cend())
					output.Add(cNotFound);
				else
				{
					output.Add(it.mIndex);
					container.erase(it);
				}
				break;
			}

		default:
			JPH_ASSERT(false);
			break;
		}
	}

	return output.GetResult();
}

} // namespace

extern "C"
{

// Core/UnorderedSet.h
uint32 jolt_unordered_set_u32_run(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	return RunScript<UnorderedSet<uint32>, uint32, uint32, false>(inOps, inKeys, inValues, inNumOps, outResults, inMaxResults);
}

uint32 jolt_unordered_set_u64_run(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	return RunScript<UnorderedSet<uint64>, uint64, uint64, false>(inOps, inKeys, inValues, inNumOps, outResults, inMaxResults);
}

uint32 jolt_unordered_set_u32_clustered_run(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	return RunScript<UnorderedSet<uint32, ClusteredHash>, uint32, uint32, false>(inOps, inKeys, inValues, inNumOps, outResults, inMaxResults);
}

// Core/UnorderedMap.h
uint32 jolt_unordered_map_u32_run(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	return RunScript<UnorderedMap<uint32, uint32>, uint32, uint32, true>(inOps, inKeys, inValues, inNumOps, outResults, inMaxResults);
}

uint32 jolt_unordered_map_u64_run(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	return RunScript<UnorderedMap<uint64, uint64>, uint64, uint64, true>(inOps, inKeys, inValues, inNumOps, outResults, inMaxResults);
}

uint32 jolt_unordered_map_u32_clustered_run(const uint8 *inOps, const uint64 *inKeys, const uint64 *inValues, uint32 inNumOps, uint64 *outResults, uint32 inMaxResults)
{
	return RunScript<UnorderedMap<uint32, uint32, ClusteredHash>, uint32, uint32, true>(inOps, inKeys, inValues, inNumOps, outResults, inMaxResults);
}

} // extern "C"
