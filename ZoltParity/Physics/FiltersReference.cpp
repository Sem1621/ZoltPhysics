// Reference implementation for the filter parity tests (`zig build parity`): thin C ABI wrappers around the C++ Jolt
// library (ObjectLayerPairFilterMask / Table, BroadPhaseLayerInterfaceMask / Table, ObjectVsBroadPhaseLayerFilterMask
// / Table, GroupFilterTable, CollisionGroup, the binary state of both, ContactManifold, ContactSettings,
// ValidateResult and ActiveEdges). ZoltParity/Physics/FiltersParity.zig calls these and checks that Zolt produces the
// same bits.
//
// Conventions: vectors are passed as 3 floats, RVec3 as 3 Reals (double with JPH_DOUBLE_PRECISION), object layers as
// uint32 (converted to ObjectLayer, 16 or 32 bits), broadphase layers as uint32. Booleans are passed as int (never
// bool, see the porting guide) and returned as uint8 arrays (0 / 1). Configurations are sequences of operations:
// - layer pair table: triples (layer1, layer2, enable), applied in order with EnableCollision / DisableCollision
// - broadphase layer table: pairs (object layer, broadphase layer) for MapObjectToBroadPhaseLayer
// - broadphase layer mask: triples (broadphase layer, groups to include, groups to exclude) for ConfigureLayer
// - group filter table: triples (sub group 1, sub group 2, enable)
// Functions that produce bytes write into a caller provided buffer and return the size.
//
// Jolt's Array allocates through the Allocate / Free hooks (RegisterDefaultAllocator) and
// GroupFilter::sRestoreFromBinaryState needs the Factory with the registered types, like Jolt's UnitTests main does.
// EnsureFactory uses the same `Factory::sInstance == nullptr` check as BasicsReference.cpp, so RegisterTypes() runs
// only once for the whole parity binary.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/RTTI.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Physics/Collision/ActiveEdges.h>
#include <Jolt/Physics/Collision/CollisionGroup.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterMask.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>

#include <cstring>
#include <sstream>
#include <string>

using namespace JPH;

static void EnsureAllocator()
{
	RegisterDefaultAllocator();
}

static void EnsureFactory()
{
	EnsureAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}
}

static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static RVec3 LoadR3(const Real *inV)				{ return RVec3(inV[0], inV[1], inV[2]); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void StoreR3(RVec3Arg inV, Real *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }

static uint32 CopyBytes(const std::string &inString, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inString.size());
	if (size <= inCapacity)
		memcpy(outBytes, inString.data(), size);
	return size;
}

static SubShapeID MakeSubShapeID(uint32 inValue)
{
	SubShapeID id;
	id.SetValue(inValue);
	return id;
}

static void ConfigureLayerPairTable(ObjectLayerPairFilterTable &ioTable, const uint32 *inOps, uint32 inNumOps)
{
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const uint32 *op = inOps + 3 * i;
		if (op[2] != 0)
			ioTable.EnableCollision(ObjectLayer(op[0]), ObjectLayer(op[1]));
		else
			ioTable.DisableCollision(ObjectLayer(op[0]), ObjectLayer(op[1]));
	}
}

static void ConfigureBroadPhaseLayerTable(BroadPhaseLayerInterfaceTable &ioTable, const uint32 *inOps, uint32 inNumOps)
{
	for (uint32 i = 0; i < inNumOps; ++i)
		ioTable.MapObjectToBroadPhaseLayer(ObjectLayer(inOps[2 * i]), BroadPhaseLayer(BroadPhaseLayer::Type(inOps[2 * i + 1])));
}

static void ConfigureBroadPhaseLayerMask(BroadPhaseLayerInterfaceMask &ioMask, const uint32 *inOps, uint32 inNumOps)
{
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const uint32 *op = inOps + 3 * i;
		ioMask.ConfigureLayer(BroadPhaseLayer(BroadPhaseLayer::Type(op[0])), op[1], op[2]);
	}
}

static void ConfigureGroupFilterTable(GroupFilterTable &ioTable, const uint32 *inOps, uint32 inNumOps)
{
	for (uint32 i = 0; i < inNumOps; ++i)
	{
		const uint32 *op = inOps + 3 * i;
		if (op[2] != 0)
			ioTable.EnableCollision(op[0], op[1]);
		else
			ioTable.DisableCollision(op[0], op[1]);
	}
}

// ContactManifold in the format of the C ABI, must match CM in FiltersParity.zig
struct CM
{
	Real				mBaseOffset[3];
	float				mWorldSpaceNormal[3];
	float				mPenetrationDepth;
	uint32				mSubShapeID1;
	uint32				mSubShapeID2;
	uint32				mCount1;
	float				mPoints1[64][3];
	uint32				mCount2;
	float				mPoints2[64][3];
};

static ContactManifold LoadCM(const CM *inCM)
{
	ContactManifold m;
	m.mBaseOffset = LoadR3(inCM->mBaseOffset);
	m.mWorldSpaceNormal = Load3(inCM->mWorldSpaceNormal);
	m.mPenetrationDepth = inCM->mPenetrationDepth;
	m.mSubShapeID1 = MakeSubShapeID(inCM->mSubShapeID1);
	m.mSubShapeID2 = MakeSubShapeID(inCM->mSubShapeID2);
	for (uint32 i = 0; i < inCM->mCount1; ++i)
		m.mRelativeContactPointsOn1.push_back(Load3(inCM->mPoints1[i]));
	for (uint32 i = 0; i < inCM->mCount2; ++i)
		m.mRelativeContactPointsOn2.push_back(Load3(inCM->mPoints2[i]));
	return m;
}

static void StoreCM(const ContactManifold &inM, CM *outCM)
{
	memset(outCM, 0, sizeof(CM));
	StoreR3(inM.mBaseOffset, outCM->mBaseOffset);
	Store3(inM.mWorldSpaceNormal, outCM->mWorldSpaceNormal);
	outCM->mPenetrationDepth = inM.mPenetrationDepth;
	outCM->mSubShapeID1 = inM.mSubShapeID1.GetValue();
	outCM->mSubShapeID2 = inM.mSubShapeID2.GetValue();
	outCM->mCount1 = uint32(inM.mRelativeContactPointsOn1.size());
	for (uint32 i = 0; i < outCM->mCount1; ++i)
		Store3(inM.mRelativeContactPointsOn1[i], outCM->mPoints1[i]);
	outCM->mCount2 = uint32(inM.mRelativeContactPointsOn2.size());
	for (uint32 i = 0; i < outCM->mCount2; ++i)
		Store3(inM.mRelativeContactPointsOn2[i], outCM->mPoints2[i]);
}

extern "C" {

// Physics/Collision/ObjectLayerPairFilterMask.h
void jolt_object_layer_mask_constants(uint32 *outNumBits, uint32 *outMask, uint32 *outObjectLayerBits)
{
	*outNumBits = ObjectLayerPairFilterMask::cNumBits;
	*outMask = ObjectLayerPairFilterMask::cMask;
	*outObjectLayerBits = uint32(8 * sizeof(ObjectLayer));
}

// sGetObjectLayer(group, mask) (inUseDefaultMask != 0: sGetObjectLayer(group)), the group and mask of the result
uint32 jolt_object_layer_mask_get_object_layer(uint32 inGroup, uint32 inMask, int inUseDefaultMask, uint32 *outGroup, uint32 *outMask)
{
	ObjectLayer layer = inUseDefaultMask != 0? ObjectLayerPairFilterMask::sGetObjectLayer(inGroup) : ObjectLayerPairFilterMask::sGetObjectLayer(inGroup, inMask);
	*outGroup = ObjectLayerPairFilterMask::sGetGroup(layer);
	*outMask = ObjectLayerPairFilterMask::sGetMask(layer);
	return uint32(layer);
}

// sGetGroup / sGetMask of any layers and ShouldCollide (through the base class) of every pair (inLayers1[i], inLayers2[i])
void jolt_object_layer_pair_filter_mask(const uint32 *inLayers1, const uint32 *inLayers2, uint32 inCount, uint32 *outGroups, uint32 *outMasks, uint8 *outShouldCollide)
{
	ObjectLayerPairFilterMask filter;
	const ObjectLayerPairFilter &base = filter;
	for (uint32 i = 0; i < inCount; ++i)
	{
		ObjectLayer l1 = ObjectLayer(inLayers1[i]), l2 = ObjectLayer(inLayers2[i]);
		outGroups[i] = ObjectLayerPairFilterMask::sGetGroup(l1);
		outMasks[i] = ObjectLayerPairFilterMask::sGetMask(l1);
		outShouldCollide[i] = base.ShouldCollide(l1, l2)? 1 : 0;
	}
}

// Physics/Collision/ObjectLayerPairFilterTable.h: ShouldCollide of every pair (inNumLayers^2 values, row major), returns GetNumObjectLayers
uint32 jolt_object_layer_pair_filter_table(uint32 inNumLayers, const uint32 *inOps, uint32 inNumOps, uint8 *outShouldCollide)
{
	EnsureAllocator();
	ObjectLayerPairFilterTable table(inNumLayers);
	ConfigureLayerPairTable(table, inOps, inNumOps);
	const ObjectLayerPairFilter &base = table;
	for (uint32 i = 0; i < inNumLayers; ++i)
		for (uint32 j = 0; j < inNumLayers; ++j)
			outShouldCollide[i * inNumLayers + j] = base.ShouldCollide(ObjectLayer(i), ObjectLayer(j))? 1 : 0;
	return table.GetNumObjectLayers();
}

// Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h: GetBroadPhaseLayer of every object layer, returns GetNumBroadPhaseLayers
uint32 jolt_broad_phase_layer_interface_table(uint32 inNumObjectLayers, uint32 inNumBroadPhaseLayers, const uint32 *inOps, uint32 inNumOps, uint32 *outBroadPhaseLayers)
{
	EnsureAllocator();
	BroadPhaseLayerInterfaceTable table(inNumObjectLayers, inNumBroadPhaseLayers);
	ConfigureBroadPhaseLayerTable(table, inOps, inNumOps);
	const BroadPhaseLayerInterface &base = table;
	for (uint32 i = 0; i < inNumObjectLayers; ++i)
		outBroadPhaseLayers[i] = (BroadPhaseLayer::Type)base.GetBroadPhaseLayer(ObjectLayer(i));
	return base.GetNumBroadPhaseLayers();
}

// Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h + ObjectVsBroadPhaseLayerFilterMask.h: GetBroadPhaseLayer
// of the given object layers and ShouldCollide (of the interface and through the filter) of each of them with every
// broadphase layer (inNumLayers * inNumBroadPhaseLayers values, row major), returns GetNumBroadPhaseLayers
uint32 jolt_broad_phase_layer_interface_mask(uint32 inNumBroadPhaseLayers, const uint32 *inOps, uint32 inNumOps, const uint32 *inLayers, uint32 inNumLayers, uint32 *outBroadPhaseLayers, uint8 *outShouldCollide, uint8 *outFilterShouldCollide)
{
	EnsureAllocator();
	BroadPhaseLayerInterfaceMask mask(inNumBroadPhaseLayers);
	ConfigureBroadPhaseLayerMask(mask, inOps, inNumOps);
	ObjectVsBroadPhaseLayerFilterMask filter(mask);
	const BroadPhaseLayerInterface &base = mask;
	const ObjectVsBroadPhaseLayerFilter &filter_base = filter;
	for (uint32 i = 0; i < inNumLayers; ++i)
	{
		ObjectLayer layer = ObjectLayer(inLayers[i]);
		outBroadPhaseLayers[i] = (BroadPhaseLayer::Type)base.GetBroadPhaseLayer(layer);
		for (uint32 b = 0; b < inNumBroadPhaseLayers; ++b)
		{
			BroadPhaseLayer bp_layer = BroadPhaseLayer(BroadPhaseLayer::Type(b));
			outShouldCollide[i * inNumBroadPhaseLayers + b] = mask.ShouldCollide(layer, bp_layer)? 1 : 0;
			outFilterShouldCollide[i * inNumBroadPhaseLayers + b] = filter_base.ShouldCollide(layer, bp_layer)? 1 : 0;
		}
	}
	return base.GetNumBroadPhaseLayers();
}

// Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h built from a BroadPhaseLayerInterfaceTable and an
// ObjectLayerPairFilterTable (inFromMasks == 0) or from a BroadPhaseLayerInterfaceMask (inBroadPhaseOps are
// ConfigureLayer triples, inNumBroadPhaseLayers layers) and an ObjectLayerPairFilterMask (inFromMasks != 0).
// ShouldCollide of every (object layer, broadphase layer) pair (inNumObjectLayers * inNumBroadPhaseLayers values, row major).
void jolt_object_vs_broad_phase_layer_filter_table(int inFromMasks, uint32 inNumObjectLayers, uint32 inNumBroadPhaseLayers, const uint32 *inPairOps, uint32 inNumPairOps, const uint32 *inBroadPhaseOps, uint32 inNumBroadPhaseOps, uint8 *outShouldCollide)
{
	EnsureAllocator();
	auto query = [&](const ObjectVsBroadPhaseLayerFilterTable &inFilter)
	{
		const ObjectVsBroadPhaseLayerFilter &base = inFilter;
		for (uint32 o = 0; o < inNumObjectLayers; ++o)
			for (uint32 b = 0; b < inNumBroadPhaseLayers; ++b)
				outShouldCollide[o * inNumBroadPhaseLayers + b] = base.ShouldCollide(ObjectLayer(o), BroadPhaseLayer(BroadPhaseLayer::Type(b)))? 1 : 0;
	};
	if (inFromMasks != 0)
	{
		BroadPhaseLayerInterfaceMask bp_interface(inNumBroadPhaseLayers);
		ConfigureBroadPhaseLayerMask(bp_interface, inBroadPhaseOps, inNumBroadPhaseOps);
		ObjectLayerPairFilterMask pair_filter;
		ObjectVsBroadPhaseLayerFilterTable filter(bp_interface, inNumBroadPhaseLayers, pair_filter, inNumObjectLayers);
		query(filter);
	}
	else
	{
		BroadPhaseLayerInterfaceTable bp_interface(inNumObjectLayers, inNumBroadPhaseLayers);
		ConfigureBroadPhaseLayerTable(bp_interface, inBroadPhaseOps, inNumBroadPhaseOps);
		ObjectLayerPairFilterTable pair_filter(inNumObjectLayers);
		ConfigureLayerPairTable(pair_filter, inPairOps, inNumPairOps);
		ObjectVsBroadPhaseLayerFilterTable filter(bp_interface, inNumBroadPhaseLayers, pair_filter, inNumObjectLayers);
		query(filter);
	}
}

// A user group filter that is not symmetric (CanCollide(g1, g2) != CanCollide(g2, g1)), so CollisionGroup::CanCollide
// must pass the groups in the right order. Must match OrderedGroupFilter in FiltersParity.zig.
class OrderedGroupFilter : public GroupFilter
{
public:
	virtual bool		CanCollide(const CollisionGroup &inGroup1, const CollisionGroup &inGroup2) const override
	{
		return inGroup1.GetSubGroupID() < inGroup2.GetSubGroupID();
	}
};

// Physics/Collision/GroupFilterTable.h + CollisionGroup.h: two group filter tables A and B, then for groups described by
// triples (filter: 0 = none, 1 = A, 2 = B, 3 = OrderedGroupFilter; group ID; sub group ID):
// - outEnabledA / outEnabledB: IsCollisionEnabled(i, j) for i != j (row major, 0 on the diagonal)
// - outCanCollide: CollisionGroup::CanCollide of every pair of groups (row major)
// - outEqual: CollisionGroup::operator == of every pair of groups (row major)
void jolt_collision_groups(uint32 inNumSubGroupsA, const uint32 *inOpsA, uint32 inNumOpsA, uint32 inNumSubGroupsB, const uint32 *inOpsB, uint32 inNumOpsB, const uint32 *inGroups, uint32 inNumGroups, uint8 *outEnabledA, uint8 *outEnabledB, uint8 *outCanCollide, uint8 *outEqual)
{
	EnsureAllocator();
	Ref<GroupFilterTable> a = new GroupFilterTable(inNumSubGroupsA);
	ConfigureGroupFilterTable(*a, inOpsA, inNumOpsA);
	Ref<GroupFilterTable> b = new GroupFilterTable(inNumSubGroupsB);
	ConfigureGroupFilterTable(*b, inOpsB, inNumOpsB);
	Ref<OrderedGroupFilter> ordered = new OrderedGroupFilter;

	for (uint32 i = 0; i < inNumSubGroupsA; ++i)
		for (uint32 j = 0; j < inNumSubGroupsA; ++j)
			outEnabledA[i * inNumSubGroupsA + j] = i != j && a->IsCollisionEnabled(i, j)? 1 : 0;
	for (uint32 i = 0; i < inNumSubGroupsB; ++i)
		for (uint32 j = 0; j < inNumSubGroupsB; ++j)
			outEnabledB[i * inNumSubGroupsB + j] = i != j && b->IsCollisionEnabled(i, j)? 1 : 0;

	Array<CollisionGroup> groups;
	for (uint32 i = 0; i < inNumGroups; ++i)
	{
		const uint32 *g = inGroups + 3 * i;
		const GroupFilter *filter = nullptr;
		if (g[0] == 1)
			filter = a.GetPtr();
		else if (g[0] == 2)
			filter = b.GetPtr();
		else if (g[0] == 3)
			filter = ordered.GetPtr();
		groups.push_back(CollisionGroup(filter, g[1], g[2]));
	}
	for (uint32 i = 0; i < inNumGroups; ++i)
		for (uint32 j = 0; j < inNumGroups; ++j)
		{
			outCanCollide[i * inNumGroups + j] = groups[i].CanCollide(groups[j])? 1 : 0;
			outEqual[i * inNumGroups + j] = groups[i] == groups[j]? 1 : 0;
		}
}

// CollisionGroup::SaveBinaryState
uint32 jolt_collision_group_save(uint32 inGroupID, uint32 inSubGroupID, uint8 *outBytes, uint32 inCapacity)
{
	CollisionGroup group(nullptr, inGroupID, inSubGroupID);
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	group.SaveBinaryState(wrapper);
	return CopyBytes(stream.str(), outBytes, inCapacity);
}

// CollisionGroup::RestoreBinaryState into a group with the given values
void jolt_collision_group_restore(const uint8 *inBytes, uint32 inSize, uint32 *ioGroupID, uint32 *ioSubGroupID, int *outEOF, int *outFailed)
{
	CollisionGroup group(nullptr, *ioGroupID, *ioSubGroupID);
	std::istringstream stream(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper wrapper(stream);
	group.RestoreBinaryState(wrapper);
	*ioGroupID = group.GetGroupID();
	*ioSubGroupID = group.GetSubGroupID();
	*outEOF = wrapper.IsEOF()? 1 : 0;
	*outFailed = wrapper.IsFailed()? 1 : 0;
}

// The RTTI hashes of GroupFilter and GroupFilterTable
void jolt_group_filter_rtti_hashes(uint32 *outHashes)
{
	outHashes[0] = JPH_RTTI(GroupFilter)->GetHash();
	outHashes[1] = JPH_RTTI(GroupFilterTable)->GetHash();
}

// GroupFilterTable::SaveBinaryState of a configured table (inCopy != 0: of a copy constructed table)
uint32 jolt_group_filter_table_save(uint32 inNumSubGroups, const uint32 *inOps, uint32 inNumOps, int inCopy, uint8 *outBytes, uint32 inCapacity)
{
	EnsureAllocator();
	Ref<GroupFilterTable> table = new GroupFilterTable(inNumSubGroups);
	ConfigureGroupFilterTable(*table, inOps, inNumOps);
	Ref<GroupFilter> filter = inCopy != 0? new GroupFilterTable(*table) : table.GetPtr();
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	filter->SaveBinaryState(wrapper);
	return CopyBytes(stream.str(), outBytes, inCapacity);
}

// GroupFilter::sRestoreFromBinaryState: returns 1 and the binary state of the restored filter (saved again), or 0 and the error text
int jolt_group_filter_restore(const uint8 *inBytes, uint32 inSize, uint8 *outBytes, uint32 inCapacity, uint32 *outSize)
{
	EnsureFactory();
	std::istringstream stream(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper wrapper(stream);
	GroupFilter::GroupFilterResult result = GroupFilter::sRestoreFromBinaryState(wrapper);
	if (result.IsValid())
	{
		std::ostringstream out;
		StreamOutWrapper out_wrapper(out);
		result.Get()->SaveBinaryState(out_wrapper);
		*outSize = CopyBytes(out.str(), outBytes, inCapacity);
		return 1;
	}
	*outSize = CopyBytes(std::string(result.GetError()), outBytes, inCapacity);
	return 0;
}

// Physics/Collision/ContactListener.h: ContactManifold::SwapShapes and the world space contact points of the input
// manifold (outWorld1 for every point on 1, outWorld2 for every point on 2)
void jolt_contact_manifold(const CM *inManifold, CM *outSwapped, Real *outWorld1, Real *outWorld2)
{
	ContactManifold m = LoadCM(inManifold);
	StoreCM(m.SwapShapes(), outSwapped);
	for (uint32 i = 0; i < inManifold->mCount1; ++i)
		StoreR3(m.GetWorldSpaceContactPointOn1(i), outWorld1 + 3 * i);
	for (uint32 i = 0; i < inManifold->mCount2; ++i)
		StoreR3(m.GetWorldSpaceContactPointOn2(i), outWorld2 + 3 * i);
}

// The members of ContactSettings that have a default value (4 scales, 2 velocities) and the values of ValidateResult
void jolt_contact_settings_defaults(float *outFloats, int *outValidateResults)
{
	ContactSettings settings;
	outFloats[0] = settings.mInvMassScale1;
	outFloats[1] = settings.mInvInertiaScale1;
	outFloats[2] = settings.mInvMassScale2;
	outFloats[3] = settings.mInvInertiaScale2;
	Store3(settings.mRelativeLinearSurfaceVelocity, outFloats + 4);
	Store3(settings.mRelativeAngularSurfaceVelocity, outFloats + 7);
	outValidateResults[0] = int(ValidateResult::AcceptAllContactsForThisBodyPair);
	outValidateResults[1] = int(ValidateResult::AcceptContact);
	outValidateResults[2] = int(ValidateResult::RejectContact);
	outValidateResults[3] = int(ValidateResult::RejectAllContactsForThisBodyPair);
}

// Physics/Collision/ActiveEdges.h
int jolt_is_edge_active(const float *inNormal1, const float *inNormal2, const float *inEdgeDirection, float inCosThresholdAngle)
{
	return ActiveEdges::IsEdgeActive(Load3(inNormal1), Load3(inNormal2), Load3(inEdgeDirection), inCosThresholdAngle)? 1 : 0;
}

void jolt_fix_normal(const float *inV0, const float *inV1, const float *inV2, const float *inTriangleNormal, uint32 inActiveEdges, const float *inPoint, const float *inNormal, const float *inMovementDirection, float *outNormal)
{
	Store3(ActiveEdges::FixNormal(Load3(inV0), Load3(inV1), Load3(inV2), Load3(inTriangleNormal), uint8(inActiveEdges), Load3(inPoint), Load3(inNormal), Load3(inMovementDirection)), outNormal);
}

} // extern "C"
