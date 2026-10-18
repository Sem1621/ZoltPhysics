// Reference implementation for ZoltParity/ObjectStream/ObjectStreamParity.zig (`zig build parity`): thin C ABI wrappers
// around Jolt's RTTI, Factory, ObjectStreamOut and ObjectStreamIn (requires JPH_OBJECT_STREAM, see build.zig).
//
// Functions that produce bytes write into a caller provided buffer and return the size, or a negative value:
// -1 read failed (sReadObject returned false), -2 write failed, -3 buffer too small, -4 unknown class.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/ObjectStream/ObjectStreamOut.h>
#include <Jolt/ObjectStream/ObjectStreamTextOut.h>
#include <Jolt/ObjectStream/SerializableObject.h>
#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Physics/Constraints/MotorSettings.h>
#include <Jolt/Physics/Constraints/SpringSettings.h>

#include <cerrno>
#include <cfloat>
#include <cstring>
#include <sstream>
#include <string>

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

// RegisterTypes() and the classes that it does not reach (the unit test classes, AABox, Triangle and the primitives
// that no registered class uses), so that Factory::Find knows every class the parity test compares
static void EnsureFactory()
{
	RegisterDefaultAllocator();
	if (Factory::sInstance == nullptr)
	{
		Factory::sInstance = new Factory();
		RegisterTypes();
	}

	// Register returns early for a class that is already known (another reference file may have made the Factory)
	const RTTI *extra[] = { JPH_RTTI(TestSerializable), JPH_RTTI(AABox), JPH_RTTI(Triangle), JPH_RTTI(uint16), JPH_RTTI(double),
		JPH_RTTI(Double3), JPH_RTTI(DVec3), JPH_RTTI(DMat44), JPH_RTTI(Float4), JPH_RTTI(UVec4), JPH_RTTI(SpringSettings),
		JPH_RTTI(MotorSettings), JPH_RTTI(IndexedTriangle), JPH_RTTI(Plane), JPH_RTTI(Color) };
	Factory::sInstance->Register(extra, uint(std::size(extra)));
}

// ObjectStreamTextIn::ReadIdentifier tests errno without clearing it, so an ERANGE of an earlier read (a subnormal float)
// makes the next identifier fail (D7 of SerializationArchitecture.md). Every read starts with a clear errno.
static void ClearErrno()
{
	errno = 0;
}

static int CopyOut(const std::string &inBytes, uint8 *outBuffer, int inCapacity)
{
	if ((int)inBytes.size() > inCapacity)
		return -3;
	memcpy(outBuffer, inBytes.data(), inBytes.size());
	return (int)inBytes.size();
}

static ObjectStream::EStreamType StreamType(int inBinary)
{
	return inBinary != 0? ObjectStream::EStreamType::Binary : ObjectStream::EStreamType::Text;
}

// MotorSettings as plain data (ObjectStreamParity.zig: MotorDesc)
struct MotorDesc
{
	uint32						mSpringMode;
	float						mFrequency;
	float						mDamping;
	float						mMinForceLimit;
	float						mMaxForceLimit;
	float						mMinTorqueLimit;
	float						mMaxTorqueLimit;
};

static MotorSettings ToMotor(const MotorDesc &inDesc)
{
	MotorSettings m;
	m.mSpringSettings.mMode = ESpringMode(inDesc.mSpringMode);
	m.mSpringSettings.mFrequency = inDesc.mFrequency;
	m.mSpringSettings.mDamping = inDesc.mDamping;
	m.mMinForceLimit = inDesc.mMinForceLimit;
	m.mMaxForceLimit = inDesc.mMaxForceLimit;
	m.mMinTorqueLimit = inDesc.mMinTorqueLimit;
	m.mMaxTorqueLimit = inDesc.mMaxTorqueLimit;
	return m;
}

static MotorDesc FromMotor(const MotorSettings &inMotor)
{
	return { uint32(inMotor.mSpringSettings.mMode), inMotor.mSpringSettings.mFrequency, inMotor.mSpringSettings.mDamping,
		inMotor.mMinForceLimit, inMotor.mMaxForceLimit, inMotor.mMinTorqueLimit, inMotor.mMaxTorqueLimit };
}

extern "C" {

// One line describing class `inName` as the Factory knows it: "name hash abstract bases=[...] attributes=[name:type
// (primitive type)...]", the type written by ObjectStreamTextOut::WriteDataType. -4 if the class is unknown.
int jolt_os_class_schema(const char *inName, uint8 *outBuffer, int inCapacity)
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
	return CopyOut(line.str(), outBuffer, inCapacity);
}

// IsKindOf of two classes the Factory knows (-4 if one is unknown)
int jolt_os_is_kind_of(const char *inName, const char *inBaseName)
{
	EnsureFactory();
	const RTTI *rtti = Factory::sInstance->Find(inName);
	const RTTI *base = Factory::sInstance->Find(inBaseName);
	if (rtti == nullptr || base == nullptr)
		return -4;
	return rtti->IsKindOf(base)? 1 : 0;
}

// sWriteObject of a MotorSettings
int jolt_os_motor_write(const MotorDesc *inDesc, int inBinary, uint8 *outBuffer, int inCapacity)
{
	EnsureFactory();
	MotorSettings motor = ToMotor(*inDesc);
	std::stringstream out;
	if (!ObjectStreamOut::sWriteObject(out, StreamType(inBinary), motor))
		return -2;
	return CopyOut(out.str(), outBuffer, inCapacity);
}

// sReadObject of a MotorSettings, then sWriteObject in the given format; *outDesc is the object that was read
int jolt_os_motor_resave(const uint8 *inData, int inSize, int inBinary, uint8 *outBuffer, int inCapacity, MotorDesc *outDesc)
{
	EnsureFactory();
	ClearErrno();
	std::stringstream in(std::string((const char *)inData, (size_t)inSize));
	MotorSettings *motor = nullptr;
	if (!ObjectStreamIn::sReadObject(in, motor))
		return -1;
	*outDesc = FromMotor(*motor);
	std::stringstream out;
	bool ok = ObjectStreamOut::sWriteObject(out, StreamType(inBinary), *motor);
	delete motor;
	return ok? CopyOut(out.str(), outBuffer, inCapacity) : -2;
}

// TestObjectStreamLoadSave*: writes the test object of ObjectStreamTest.cpp
int jolt_os_unit_test_write(int inBinary, uint8 *outBuffer, int inCapacity)
{
	EnsureFactory();
	TestSerializable *test = CreateTestObject();
	std::stringstream out;
	bool ok = ObjectStreamOut::sWriteObject(out, StreamType(inBinary), *test);
	delete test;
	return ok? CopyOut(out.str(), outBuffer, inCapacity) : -2;
}

// Reads a TestSerializable and writes it again; *outRefCount is the reference count of mReference (2 in the unit test),
// *outCastsOk tells if DynamicCast gives the static_cast addresses for both bases
int jolt_os_unit_test_resave(const uint8 *inData, int inSize, int inBinary, uint8 *outBuffer, int inCapacity, uint32 *outRefCount, int *outCastsOk)
{
	EnsureFactory();
	ClearErrno();
	std::stringstream in(std::string((const char *)inData, (size_t)inSize));
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
	return ok? CopyOut(out.str(), outBuffer, inCapacity) : -2;
}

} // extern "C"
