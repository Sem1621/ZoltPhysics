// Reference implementation for ZoltParity/ObjectStream/ObjectStreamParity.zig (`zig build parity`): thin C ABI wrappers
// around Jolt's RTTI, Factory, ObjectStreamOut and ObjectStreamIn (requires JPH_OBJECT_STREAM, see build.zig).
//
// The harness knows classes only through Jolt's RTTI, by name. Nothing below is written for one class, except P7's
// SaveBinaryState (a function of the asset's class) and the classes of UnitTests/ObjectStream/ObjectStreamTest.cpp (P8).
//
// Results that are bytes stay in a result slot (jolt_os_result) until the next call. Return values: 0 or more success,
// -1 read failed (Jolt's sReadObject returns false), -2 write failed, -4 unknown class or not creatable.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/ObjectStream/ObjectStreamBinaryIn.h>
#include <Jolt/ObjectStream/ObjectStreamTextIn.h>
#include <Jolt/ObjectStream/ObjectStreamTextOut.h>
#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>
#include <Jolt/Physics/PhysicsScene.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Skeleton/SkeletalAnimation.h>

#include <algorithm>
#include <cerrno>
#include <cfloat>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace JPH;

// The classes of UnitTests/ObjectStream/ObjectStreamTest.cpp (copied verbatim, also ported in
// ZoltTests/ObjectStream/ObjectStreamTest.zig): all primitives, an enum, Array, StaticArray, C arrays (of arrays),
// nested arrays, a raw pointer, Ref / RefConst and two base classes (the second at a non-zero offset)

enum TestEnum
{
	A,
	B,
	C
};

class TestSerializableBase : public RefTarget<TestSerializableBase>
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL_BASE(JPH_NO_EXPORT, TestSerializableBase)

public:
	virtual						~TestSerializableBase() = default;

	uint8						mUInt8 = 0;
	uint16						mUInt16 = 0;
	int							mInt = 0;
	uint32						mUInt32 = 0;
	uint64						mUInt64 = 0;
	float						mFloat = 0;
	double						mDouble = 0;
	bool						mBool = false;
	Float3						mFloat3 = { };
	Float4						mFloat4 = { };
	Double3						mDouble3 = { };
	Quat						mQuat = Quat::sIdentity();
	Vec3						mVec3 = Vec3::sZero();
	DVec3						mDVec3 = DVec3::sZero();
	Vec4						mVec4 = Vec4::sZero();
	UVec4						mUVec4 = UVec4::sZero();
	Mat44						mMat44 = Mat44::sIdentity();
	DMat44						mDMat44 = DMat44::sIdentity();
	JPH::String					mString;
};

class TestSerializableBase2
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL_BASE(JPH_NO_EXPORT, TestSerializableBase2)

public:
	virtual						~TestSerializableBase2() = default;

	uint32						mBase2 = 0;
};

class TestSerializable : public TestSerializableBase, public TestSerializableBase2
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_NO_EXPORT, TestSerializable)

public:
	TestEnum					mEnum = A;
	Array<int>					mIntVector;
	StaticArray<bool, 10>		mBoolVector;
	float						mFloatVector[3] = { 0, 0, 0 };
	Array<float>				mArrayOfVector[3];
	Array<Array<int>>			mVectorOfVector;
	TestSerializable *			mPointer = nullptr;
	Ref<TestSerializable>		mReference;
	RefConst<TestSerializable>	mReferenceConst;
};

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestSerializableBase)
{
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mUInt8)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mUInt16)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mInt)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mUInt32)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mUInt64)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mFloat)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mDouble)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mBool)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mFloat3)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mFloat4)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mDouble3)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mQuat)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mVec3)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mDVec3)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mVec4)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mUVec4)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mMat44)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mDMat44)
	JPH_ADD_ATTRIBUTE(TestSerializableBase, mString)
}

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestSerializableBase2)
{
	JPH_ADD_ATTRIBUTE(TestSerializableBase2, mBase2)
}

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestSerializable)
{
	JPH_ADD_BASE_CLASS(TestSerializable, TestSerializableBase)
	JPH_ADD_BASE_CLASS(TestSerializable, TestSerializableBase2)

	JPH_ADD_ENUM_ATTRIBUTE(TestSerializable, mEnum)
	JPH_ADD_ATTRIBUTE(TestSerializable, mIntVector)
	JPH_ADD_ATTRIBUTE(TestSerializable, mBoolVector)
	JPH_ADD_ATTRIBUTE(TestSerializable, mFloatVector)
	JPH_ADD_ATTRIBUTE(TestSerializable, mArrayOfVector)
	JPH_ADD_ATTRIBUTE(TestSerializable, mVectorOfVector)
	JPH_ADD_ATTRIBUTE(TestSerializable, mPointer)
	JPH_ADD_ATTRIBUTE(TestSerializable, mReference)
	JPH_ADD_ATTRIBUTE(TestSerializable, mReferenceConst)
}

// CreateTestObject of ObjectStreamTest.cpp
static TestSerializable *CreateTestObject()
{
	TestSerializable *test = new TestSerializable();
	test->mUInt8 = 0xff;
	test->mUInt16 = 0xffff;
	test->mInt = -1;
	test->mUInt32 = 0xf1f2f3f4;
	test->mUInt64 = 0xf5f6f7f8f9fafbfc;
	test->mFloat = 0.12345f;
	test->mDouble = DBL_EPSILON;
	test->mBool = true;
	test->mFloat3 = Float3(9, 10, 11);
	test->mFloat4 = Float4(11, 9, 10, 12);
	test->mDouble3 = Double3(10, 11, 12);
	test->mVec3 = Vec3(6, 7, 8);
	test->mDVec3 = DVec3(7, 8, 9);
	test->mVec4 = Vec4(9, 10, 11, 12);
	test->mUVec4 = UVec4(12, 11, 9, 10);
	test->mQuat = Quat::sRotation(Vec3::sAxisX(), 0.1234f);
	test->mMat44 = Mat44::sRotationTranslation(Quat::sRotation(Vec3::sAxisY(), 0.4567f), Vec3(13, 14, 15));
	test->mDMat44 = DMat44::sRotationTranslation(Quat::sRotation(Vec3::sAxisY(), 0.789f), DVec3(20, 21, 22));
	test->mString = "\"test string\"";
	test->mEnum = B;
	test->mIntVector = { 1, 2, 3, 4, 5 };
	test->mBoolVector.push_back(true);
	test->mBoolVector.push_back(false);
	test->mBoolVector.push_back(true);
	test->mFloatVector[0] = 1.0f;
	test->mFloatVector[1] = 2.0f;
	test->mFloatVector[2] = 3.0f;
	test->mArrayOfVector[0] = { 1, 2, 3 };
	test->mArrayOfVector[1] = { 4, 5 };
	test->mArrayOfVector[2] = { 6, 7, 8, 9 };
	test->mVectorOfVector = { { 10, 11 }, { 12, 13, 14 }, { 15, 16, 17, 18 }};
	test->mBase2 = 0x9876;

	TestSerializable *test2 = new TestSerializable();
	test2->mFloat = 4.5f;
	test->mPointer = test2;
	test->mReference = test2;
	test->mReferenceConst = test2;

	return test;
}

// The global Factory: RegisterTypes() and the classes that it does not reach (the unit test classes, AABox, Triangle and
// the primitives that no registered class uses). Schemas (P1) are looked up here, so the extra classes can be compared.
static void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}

	// Register returns early for a class that is already known (another reference file may have made the Factory)
	const RTTI *extra[] = { JPH_RTTI(TestSerializable), JPH_RTTI(AABox), JPH_RTTI(Triangle), JPH_RTTI(uint8), JPH_RTTI(uint16),
		JPH_RTTI(int), JPH_RTTI(uint32), JPH_RTTI(uint64), JPH_RTTI(float), JPH_RTTI(double), JPH_RTTI(bool), JPH_RTTI(String),
		JPH_RTTI(Float3), JPH_RTTI(Float4), JPH_RTTI(Double3), JPH_RTTI(Vec3), JPH_RTTI(DVec3), JPH_RTTI(Vec4), JPH_RTTI(UVec4),
		JPH_RTTI(Quat), JPH_RTTI(Mat44), JPH_RTTI(DMat44), JPH_RTTI(SpringSettings), JPH_RTTI(MotorSettings),
		JPH_RTTI(IndexedTriangle), JPH_RTTI(Plane), JPH_RTTI(Color) };
	Factory::sInstance->Register(extra, uint(std::size(extra)));
}

// "The C++ Factory after RegisterTypes" (SerializationArchitecture.md section 5): a Factory of its own, filled by
// RegisterTypes() only. Every read of P2 to P7 uses it as Factory::sInstance, like Zolt's RegisterTypes.factory.
// Running RegisterTypes() again is harmless: CollisionDispatch::sInit only fills empty entries, the shapes register the
// same functions again (the user registrations of the other reference files stay) and the default material exists.
static Factory *sRegisterTypesFactory = nullptr;
static Array<const RTTI *> *sRegisterTypesClasses = nullptr;

static Factory *GetRegisterTypesFactory()
{
	EnsureFactory();
	if (sRegisterTypesFactory == nullptr)
	{
		Factory *saved = Factory::sInstance;
		sRegisterTypesFactory = new Factory();
		Factory::sInstance = sRegisterTypesFactory;
		RegisterTypes();
		Factory::sInstance = saved;

		// The classes sorted by name (GetAllClasses returns its hash map order)
		sRegisterTypesClasses = new Array<const RTTI *>(sRegisterTypesFactory->GetAllClasses());
		std::sort(sRegisterTypesClasses->begin(), sRegisterTypesClasses->end(), [](const RTTI *inLHS, const RTTI *inRHS) { return strcmp(inLHS->GetName(), inRHS->GetName()) < 0; });
	}
	return sRegisterTypesFactory;
}

// Makes the RegisterTypes Factory Factory::sInstance for the lifetime of the scope
class UseRegisterTypesFactory
{
public:
								UseRegisterTypesFactory()				: mSaved((GetRegisterTypesFactory(), Factory::sInstance)) { Factory::sInstance = sRegisterTypesFactory; }
								~UseRegisterTypesFactory()				{ Factory::sInstance = mSaved; }

private:
	Factory *					mSaved;
};

// The allocation hooks of P2 to P5 and P7: every block is zero filled, so that members that C++ leaves indeterminate are
// 0 as in Zolt (D9), and frees are deferred until the scope ends (a quarantine), so that Jolt's use after free paths on
// odd streams stay harmless. The six outside classes and the primitives use the global new, which the hooks do not see.
class Quarantine
{
public:
								Quarantine()
	{
		mSavedAllocate = Allocate;
		mSavedReallocate = Reallocate;
		mSavedFree = Free;
		mSavedAlignedAllocate = AlignedAllocate;
		mSavedAlignedFree = AlignedFree;
		Allocate = sAllocate;
		Reallocate = sReallocate;
		Free = sFree;
		AlignedAllocate = sAlignedAllocate;
		AlignedFree = sAlignedFree;
	}

								~Quarantine()
	{
		Allocate = mSavedAllocate;
		Reallocate = mSavedReallocate;
		Free = mSavedFree;
		AlignedAllocate = mSavedAlignedAllocate;
		AlignedFree = mSavedAlignedFree;

		// Both malloc and posix_memalign blocks are freed with free (and Jolt's default allocator frees with free too)
		for (void *block : sBlocks)
			free(block);
		sBlocks.clear();
	}

private:
	static void *				sAllocate(size_t inSize)
	{
		void *block = malloc(inSize > 0? inSize : 1);
		if (block != nullptr)
			memset(block, 0, inSize);
		return block;
	}

	static void *				sReallocate(void *inBlock, size_t inOldSize, size_t inNewSize)
	{
		void *block = sAllocate(inNewSize);
		if (block != nullptr && inBlock != nullptr)
		{
			memcpy(block, inBlock, inOldSize < inNewSize? inOldSize : inNewSize);
			sFree(inBlock);
		}
		return block;
	}

	static void					sFree(void *inBlock)
	{
		if (inBlock != nullptr)
			sBlocks.push_back(inBlock);
	}

	static void *				sAlignedAllocate(size_t inSize, size_t inAlignment)
	{
		void *block = nullptr;
		if (posix_memalign(&block, inAlignment < sizeof(void *)? sizeof(void *) : inAlignment, inSize > 0? inSize : 1) != 0)
			return nullptr;
		memset(block, 0, inSize);
		return block;
	}

	static void					sAlignedFree(void *inBlock)
	{
		sFree(inBlock);
	}

	// std::vector allocates with the global new, outside of the hooks
	static inline std::vector<void *> sBlocks;

	AllocateFunction			mSavedAllocate;
	ReallocateFunction			mSavedReallocate;
	FreeFunction				mSavedFree;
	AlignedAllocateFunction		mSavedAlignedAllocate;
	AlignedFreeFunction			mSavedAlignedFree;
};

// Access to the protected static constructors of the streams (sReadObject / sWriteObject are templates)
class ObjectStreamInAccess : public ObjectStreamIn
{
public:
	using ObjectStreamIn::Open;
};

class ObjectStreamOutAccess : public ObjectStreamOut
{
public:
	using ObjectStreamOut::Open;
};

// The byte results of the last call (jolt_os_result)
static std::string sResults[3];

static ObjectStream::EStreamType StreamType(int inBinary)
{
	return inBinary != 0? ObjectStream::EStreamType::Binary : ObjectStream::EStreamType::Text;
}

// sReadObject by RTTI: the main object (the first object of the stream, kind of inRTTI) or nullptr.
// ObjectStreamTextIn::ReadIdentifier tests errno without clearing it, so an ERANGE of an earlier read (a subnormal float)
// makes the next identifier fail (D7 of SerializationArchitecture.md). Every read starts with a clear errno.
static void *ReadObject(const std::string &inBytes, const RTTI *inRTTI)
{
	errno = 0;
	std::stringstream in(inBytes);
	ObjectStreamIn *stream = ObjectStreamInAccess::Open(in);
	if (stream == nullptr)
		return nullptr;
	void *object = stream->Read(inRTTI);
	delete stream;
	return object;
}

// sWriteObject by RTTI: inObject is written as an object of class inRTTI
static bool WriteObject(std::string &outBytes, const void *inObject, const RTTI *inRTTI, int inBinary)
{
	std::stringstream out;
	ObjectStreamOut *stream = ObjectStreamOutAccess::Open(StreamType(inBinary), out);
	if (stream == nullptr)
		return false;
	bool ok = stream->Write(inObject, inRTTI);
	delete stream;
	outBytes = out.str();
	return ok;
}

static const RTTI *FindCreatable(const char *inName)
{
	const RTTI *rtti = GetRegisterTypesFactory()->Find(inName);
	return rtti != nullptr && !rtti->IsAbstract()? rtti : nullptr;
}

static void AppendUInt32(std::string &ioBytes, uint32 inValue)
{
	ioBytes.append((const char *)&inValue, sizeof(inValue));
}

// The binary size of one element of a primitive type (ObjectStreamBinaryOut), 0 for the others
static size_t BinarySize(EOSDataType inType)
{
	switch (inType)
	{
	case EOSDataType::T_uint8:		return 1;
	case EOSDataType::T_uint16:		return 2;
	case EOSDataType::T_int:		return 4;
	case EOSDataType::T_uint32:		return 4;
	case EOSDataType::T_uint64:		return 8;
	case EOSDataType::T_float:		return 4;
	case EOSDataType::T_double:		return 8;
	case EOSDataType::T_bool:		return 1;
	case EOSDataType::T_String:		return 4; // The empty string: length 0
	case EOSDataType::T_Float3:		return 12;
	case EOSDataType::T_Float4:		return 16;
	case EOSDataType::T_Double3:	return 24;
	case EOSDataType::T_Vec3:		return 12;
	case EOSDataType::T_DVec3:		return 24;
	case EOSDataType::T_Vec4:		return 16;
	case EOSDataType::T_UVec4:		return 16;
	case EOSDataType::T_Quat:		return 16;
	case EOSDataType::T_Mat44:		return 64;
	case EOSDataType::T_DMat44:		return 72;
	case EOSDataType::Pointer:		return 4; // An identifier
	default:						return 0;
	}
}

// Binary data of an attribute: inDepth nested arrays of inCount elements each, every element inElementSize zero bytes
static void AppendProbeData(std::string &ioBytes, int inDepth, uint32 inCount, size_t inElementSize)
{
	if (inDepth == 0)
	{
		ioBytes.append(inElementSize, '\0');
		return;
	}
	AppendUInt32(ioBytes, inCount);
	for (uint32 i = 0; i < inCount; ++i)
		AppendProbeData(ioBytes, inDepth - 1, inCount, inElementSize);
}

// Reads inData into attribute inAttribute of a new object of class inRTTI (SerializableAttribute::ReadData with an
// ObjectStreamBinaryIn). Before the data the stream declares inClassName (if not null) with no attributes, so that its
// class data is empty. True when the read succeeds and uses every byte.
static bool ProbeRead(const RTTI *inRTTI, const SerializableAttribute &inAttribute, const char *inClassName, const std::string &inData)
{
	std::string bytes;
	if (inClassName != nullptr)
	{
		uint32 length = uint32(strlen(inClassName));
		AppendUInt32(bytes, length);
		bytes.append(inClassName, length);
		AppendUInt32(bytes, 0);
	}
	bytes += inData;

	errno = 0;
	std::stringstream stream(bytes);
	ObjectStreamBinaryIn in(stream);
	void *object = inRTTI->CreateObject();
	bool ok = (inClassName == nullptr || in.ReadRTTI()) && inAttribute.ReadData(in, object);
	if (ok)
	{
		std::streampos position = stream.tellg();
		ok = position != std::streampos(-1) && size_t(position) == bytes.size();
	}
	inRTTI->DestructObject(object);
	return ok;
}

extern "C" {

// The number of classes of the Factory after RegisterTypes
int jolt_os_class_count()
{
	GetRegisterTypesFactory();
	return int(sRegisterTypesClasses->size());
}

// The name of class inIndex of the Factory after RegisterTypes (sorted by name), in result slot 0
int jolt_os_class_name(int inIndex)
{
	GetRegisterTypesFactory();
	if (inIndex < 0 || inIndex >= int(sRegisterTypesClasses->size()))
		return -4;
	sResults[0] = (*sRegisterTypesClasses)[inIndex]->GetName();
	return 0;
}

// The bytes of result slot inIndex (valid until the next call)
const uint8 *jolt_os_result(int inIndex, size_t *outSize)
{
	*outSize = sResults[inIndex].size();
	return (const uint8 *)sResults[inIndex].data();
}

// One line describing class inName as the global Factory knows it, in result slot 0: "name hash abstract|creatable
// bases=[...] attributes=[name:type(primitive type);...]", the type written by ObjectStreamTextOut::WriteDataType
int jolt_os_class_schema(const char *inName)
{
	EnsureFactory();
	const RTTI *rtti = Factory::sInstance->Find(inName);
	if (rtti == nullptr)
		return -4;

	std::stringstream line;
	char hash[16];
	snprintf(hash, sizeof(hash), "%08x", rtti->GetHash());
	line << rtti->GetName() << " " << hash << " " << (rtti->IsAbstract()? "abstract" : "creatable") << " bases=[";
	for (int i = 0; i < rtti->GetBaseClassCount(); ++i)
		line << (i > 0? "," : "") << rtti->GetBaseClass(i)->GetName();
	line << "] attributes=[";
	for (int i = 0; i < rtti->GetAttributeCount(); ++i)
	{
		const SerializableAttribute &attr = rtti->GetAttribute(i);
		std::stringstream type;
		ObjectStreamTextOut out(type);
		type.str(""); // Drop the header
		attr.WriteDataType(out);
		const RTTI *primitive = attr.GetMemberPrimitiveType();
		line << (i > 0? ";" : "") << attr.GetName() << ":" << type.str() << "(" << (primitive != nullptr? primitive->GetName() : "-") << ")";
	}
	line << "]";
	sResults[0] = line.str();
	return 0;
}

// IsKindOf of two classes the global Factory knows (-4 if one is unknown)
int jolt_os_is_kind_of(const char *inName, const char *inBaseName)
{
	EnsureFactory();
	const RTTI *rtti = Factory::sInstance->Find(inName);
	const RTTI *base = Factory::sInstance->Find(inBaseName);
	if (rtti == nullptr || base == nullptr)
		return -4;
	return rtti->IsKindOf(base)? 1 : 0;
}

// How attribute inIndex of the creatable class inName stores its data, found by reading crafted binary data into a new
// object. *outElement: 1 a primitive, an enum or a raw pointer (an identifier); 2 an `instance` whose data is an
// identifier (Ref / RefConst); 3 an `instance` whose data is class data (by value); 0 unknown. *outCounts (arrays only):
// bit c is set when an array of c elements (at every level) can be read: all counts for Array, up to N for
// StaticArray<T, N>, only N for T[N]. Returns -4 for an unknown or abstract class.
int jolt_os_probe_attribute(const char *inName, int inIndex, int *outElement, uint64 *outCounts)
{
	UseRegisterTypesFactory use_factory;
	Quarantine quarantine;
	const RTTI *rtti = FindCreatable(inName);
	if (rtti == nullptr || inIndex < 0 || inIndex >= rtti->GetAttributeCount())
		return -4;
	const SerializableAttribute &attr = rtti->GetAttribute(inIndex);

	// The declared type (array depth, element type) from the text that WriteDataType writes
	std::stringstream type_text;
	ObjectStreamTextOut out(type_text);
	type_text.str("");
	attr.WriteDataType(out);
	std::string type = type_text.str();
	int depth = 0;
	while (type.compare(0, 6, "array ") == 0)
	{
		++depth;
		type = type.substr(6);
	}
	bool is_instance = type.compare(0, 9, "instance ") == 0;
	const char *class_name = is_instance? attr.GetMemberPrimitiveType()->GetName() : nullptr;
	size_t element_size = 0;
	if (!is_instance)
	{
		ObjectStreamTextIn type_in(type_text);
		EOSDataType data_type = EOSDataType::Invalid;
		type_text.clear();
		type_text.str(type);
		type_in.ReadDataType(data_type);
		element_size = BinarySize(data_type);
	}

	// The counts that can be read, for an element of inSize bytes
	auto probe_counts = [&](size_t inSize) {
		uint64 counts = 0;
		uint32 max_count = depth == 1? 63 : 16;
		for (uint32 c = 0; c <= max_count; ++c)
		{
			std::string data;
			AppendProbeData(data, depth, c, inSize);
			if (ProbeRead(rtti, attr, class_name, data))
				counts |= uint64(1) << c;
		}
		return counts;
	};

	*outCounts = 0;
	if (!is_instance)
	{
		*outElement = element_size > 0? 1 : 0;
		if (depth > 0)
			*outCounts = probe_counts(element_size);
		return 0;
	}

	// An instance: an identifier (4 bytes) or class data (nothing, the class is declared without attributes)
	if (depth == 0)
	{
		std::string identifier;
		AppendUInt32(identifier, 0);
		bool reference = ProbeRead(rtti, attr, class_name, identifier);
		bool value = ProbeRead(rtti, attr, class_name, std::string());
		*outElement = reference && !value? 2 : (value && !reference? 3 : 0);
		return 0;
	}
	uint64 reference_counts = probe_counts(4);
	uint64 value_counts = probe_counts(0);
	bool reference = (reference_counts & ~uint64(1)) != 0;
	bool value = (value_counts & ~uint64(1)) != 0;
	*outElement = reference && !value? 2 : (value && !reference? 3 : 0);
	*outCounts = reference? reference_counts : value_counts;
	return 0;
}

// P2: CreateObject of class inName, written in text (result slot 0) and binary (slot 1)
int jolt_os_default_write(const char *inName)
{
	UseRegisterTypesFactory use_factory;
	Quarantine quarantine;
	const RTTI *rtti = FindCreatable(inName);
	if (rtti == nullptr)
		return -4;
	void *object = rtti->CreateObject();
	bool ok = WriteObject(sResults[0], object, rtti, 0) && WriteObject(sResults[1], object, rtti, 1);
	rtti->DestructObject(object);
	return ok? 0 : -2;
}

// P3 to P5, P7: sReadObject of inData as an object of class inName (the main object must be exactly of that class),
// then sWriteObject of the main object in text (result slot 0) and binary (slot 1)
int jolt_os_resave(const uint8 *inData, size_t inSize, const char *inName)
{
	UseRegisterTypesFactory use_factory;
	Quarantine quarantine;
	const RTTI *rtti = GetRegisterTypesFactory()->Find(inName);
	if (rtti == nullptr)
		return -4;
	void *object = ReadObject(std::string((const char *)inData, inSize), rtti);
	if (object == nullptr)
		return -1;
	bool ok = WriteObject(sResults[0], object, rtti, 0) && WriteObject(sResults[1], object, rtti, 1);
	rtti->DestructObject(object);
	return ok? 0 : -2;
}

// P6: one word read as type inType by ObjectStreamTextIn (IStringStream >> T, strtoul for identifiers): 0 uint8,
// 1 uint16, 2 int, 3 uint32, 4 uint64, 5 float, 6 double, 7 bool, 8 identifier, 9 count. Returns 1 and the value bits
// in *outBits when the word is read, 0 when not.
int jolt_os_read_word(const uint8 *inWord, size_t inSize, int inType, uint64 *outBits)
{
	EnsureFactory(); // Jolt's allocator (String allocates through it)
	errno = 0;
	std::stringstream stream(std::string((const char *)inWord, inSize));
	ObjectStreamTextIn in(stream);
	*outBits = 0;
	switch (inType)
	{
	case 0:		{ uint8 v = 0; if (!in.ReadPrimitiveData(v)) return 0; *outBits = v; return 1; }
	case 1:		{ uint16 v = 0; if (!in.ReadPrimitiveData(v)) return 0; *outBits = v; return 1; }
	case 2:		{ int v = 0; if (!in.ReadPrimitiveData(v)) return 0; *outBits = uint32(v); return 1; }
	case 3:		{ uint32 v = 0; if (!in.ReadPrimitiveData(v)) return 0; *outBits = v; return 1; }
	case 4:		{ uint64 v = 0; if (!in.ReadPrimitiveData(v)) return 0; *outBits = v; return 1; }
	case 5:		{ float v = 0; if (!in.ReadPrimitiveData(v)) return 0; uint32 b; memcpy(&b, &v, 4); *outBits = b; return 1; }
	case 6:		{ double v = 0; if (!in.ReadPrimitiveData(v)) return 0; memcpy(outBits, &v, 8); return 1; }
	case 7:		{ bool v = false; if (!in.ReadPrimitiveData(v)) return 0; *outBits = v? 1 : 0; return 1; }
	case 8:		{ uint32 v = 0; if (!in.ReadIdentifier(v)) return 0; *outBits = v; return 1; }
	case 9:		{ uint32 v = 0; if (!in.ReadCount(v)) return 0; *outBits = v; return 1; }
	default:	return -4;
	}
}

// P7: sReadObject of an asset as class inName, then the SaveBinaryState bytes of the loaded object (result slot 2)
int jolt_os_asset_binary_state(const uint8 *inData, size_t inSize, const char *inName)
{
	UseRegisterTypesFactory use_factory;
	Quarantine quarantine;
	const RTTI *rtti = GetRegisterTypesFactory()->Find(inName);
	if (rtti == nullptr)
		return -4;
	void *object = ReadObject(std::string((const char *)inData, inSize), rtti);
	if (object == nullptr)
		return -1;
	std::stringstream data;
	StreamOutWrapper stream(data);
	int result = 0;
	if (rtti == JPH_RTTI(RagdollSettings))
		static_cast<RagdollSettings *>(object)->SaveBinaryState(stream, true, true);
	else if (rtti == JPH_RTTI(SkeletalAnimation))
		static_cast<SkeletalAnimation *>(object)->SaveBinaryState(stream);
	else if (rtti == JPH_RTTI(PhysicsScene))
		static_cast<PhysicsScene *>(object)->SaveBinaryState(stream, true, true);
	else
		result = -4;
	sResults[2] = data.str();
	rtti->DestructObject(object);
	return result;
}

// P8, TestObjectStreamLoadSave*: writes the test object of ObjectStreamTest.cpp (result slot 0)
int jolt_os_unit_test_write(int inBinary)
{
	EnsureFactory();
	TestSerializable *test = CreateTestObject();
	std::stringstream out;
	bool ok = ObjectStreamOut::sWriteObject(out, StreamType(inBinary), *test);
	delete test;
	sResults[0] = out.str();
	return ok? 0 : -2;
}

// P8: reads a TestSerializable and writes it again (result slot 0); *outRefCount is the reference count of mReference
// (2 in the unit test), *outCastsOk tells if DynamicCast gives the static_cast addresses for both bases
int jolt_os_unit_test_resave(const uint8 *inData, size_t inSize, int inBinary, uint32 *outRefCount, int *outCastsOk)
{
	EnsureFactory();
	errno = 0;
	std::stringstream in(std::string((const char *)inData, inSize));
	TestSerializable *test = nullptr;
	if (!ObjectStreamIn::sReadObject(in, test))
		return -1;
	*outRefCount = test->mReference != nullptr? test->mReference->GetRefCount() : 0;
	*outCastsOk = DynamicCast<TestSerializable>(test) == test
		&& DynamicCast<TestSerializableBase>(test) == static_cast<TestSerializableBase *>(test)
		&& DynamicCast<TestSerializableBase2>(test) == static_cast<TestSerializableBase2 *>(test);
	std::stringstream out;
	bool ok = ObjectStreamOut::sWriteObject(out, StreamType(inBinary), *test);
	delete test;
	sResults[0] = out.str();
	return ok? 0 : -2;
}

} // extern "C"
