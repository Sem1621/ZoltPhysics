// Reference implementation for the Jolt/Core parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library. ZoltParity/Core/CoreParity.zig calls these and checks that Zolt produces the same results.
//
// Conventions: arrays are passed as pointer + count, strings as pointer + length (not zero terminated unless noted),
// LinearCurve points as (x, y) float pairs. Functions that produce variable sized output write into a caller
// provided buffer and return the size. Functions that allocate through Jolt's Array / String first make sure the
// default allocator is registered.

#include <Jolt/Jolt.h>
#include <Jolt/Core/BinaryHeap.h>
#include <Jolt/Core/HashCombine.h>
#include <Jolt/Core/InsertionSort.h>
#include <Jolt/Core/LinearCurve.h>
#include <Jolt/Core/QuickSort.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/StringTools.h>

#include <cstring>
#include <random>
#include <sstream>
#include <string>
#include <string_view>

using namespace JPH;

// Element with a sort key and its original position, sorted on the key only so that the order of equal keys is observable
struct SortElem
{
	uint32				mKey;
	uint32				mIndex;
};

// Types of the values in jolt_hash_combine_sequence, must match HashType in CoreParity.zig
enum class EHashType : uint32
{
	Float,
	Double,
	Int,
	UInt32,
	UInt64,
	Char,
};

// Enum that is streamed as raw bytes
enum class EStreamEnum : uint16
{
	A,
	B,
};

// All types that StreamIn / StreamOut handle, must match StreamSample in CoreParity.zig
struct StreamSample
{
	uint8				mU8;
	uint16				mU16;
	uint32				mU32;
	uint64				mU64;
	int32				mI32;
	float				mFloat;
	double				mDouble;
	bool				mBool;
	uint16				mEnum;
	float				mVec3[3];
	float				mVec4[4];
	float				mQuat[4];
	float				mMat44[16];
	float				mFloat3[3];
	double				mDVec3[3];
	float				mDMat44Cols[12];
	double				mDMat44T[3];
	uint32				mFloatArrayLen;
	float				mFloatArray[8];
	uint32				mVec3ArrayLen;
	float				mVec3Array[4][3];
	uint32				mDVec3ArrayLen;
	double				mDVec3Array[3][3];
	uint32				mDMat44ArrayLen;
	float				mDMat44ArrayCols[2][12];
	double				mDMat44ArrayT[2][3];
	uint32				mStringLen;
	char				mString[16];
	uint32				mPointArrayLen;
	float				mPointArray[4][2];
	uint32				mCurveLen;
	float				mCurve[4][2];
};

// Number of fields in StreamSample that are read separately, see jolt_stream_read_sample
static constexpr uint cNumStreamFields = 23;

static void EnsureAllocator()
{
	RegisterDefaultAllocator();
}

static LinearCurve MakeCurve(const float *inPoints, uint32 inCount)
{
	LinearCurve curve;
	for (uint32 i = 0; i < inCount; ++i)
		curve.AddPoint(inPoints[2 * i], inPoints[2 * i + 1]);
	return curve;
}

static void StoreCurve(const LinearCurve &inCurve, float *outPoints)
{
	for (size_t i = 0; i < inCurve.mPoints.size(); ++i)
	{
		outPoints[2 * i] = inCurve.mPoints[i].mX;
		outPoints[2 * i + 1] = inCurve.mPoints[i].mY;
	}
}

static uint32 CopyString(const std::string &inString, uint8 *outBytes, uint32 inCapacity)
{
	uint32 size = uint32(inString.size());
	if (size <= inCapacity)
		memcpy(outBytes, inString.data(), size);
	return size;
}

static uint32 CopyString(const String &inString, char *outChars, uint32 inCapacity)
{
	uint32 size = uint32(inString.size());
	if (size <= inCapacity)
		memcpy(outChars, inString.data(), size);
	return size;
}

static Vec3 LoadVec3(const float *inV)				{ return Vec3(inV[0], inV[1], inV[2]); }
static Vec4 LoadVec4(const float *inV)				{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static DVec3 LoadDVec3(const double *inV)			{ return DVec3(inV[0], inV[1], inV[2]); }
static DMat44 LoadDMat44(const float *inCols, const double *inT) { return DMat44(LoadVec4(inCols), LoadVec4(inCols + 4), LoadVec4(inCols + 8), LoadDVec3(inT)); }
static void StoreVec3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void StoreVec4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void StoreDVec3(DVec3Arg inV, double *outV)	{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static void StoreDMat44(DMat44Arg inM, float *outCols, double *outT)
{
	for (uint c = 0; c < 3; ++c)
		StoreVec4(inM.GetColumn4(c), outCols + 4 * c);
	StoreDVec3(inM.GetTranslation(), outT);
}

static bool StreamOK(const StreamIn &inStream)		{ return !inStream.IsEOF() && !inStream.IsFailed(); }

extern "C"
{

// Core/QuickSort.h, Core/InsertionSort.h
void jolt_quick_sort_elems(SortElem *ioItems, uint32 inCount)
{
	QuickSort(ioItems, ioItems + inCount, [](const SortElem &inLHS, const SortElem &inRHS) { return inLHS.mKey < inRHS.mKey; });
}

void jolt_insertion_sort_elems(SortElem *ioItems, uint32 inCount)
{
	InsertionSort(ioItems, ioItems + inCount, [](const SortElem &inLHS, const SortElem &inRHS) { return inLHS.mKey < inRHS.mKey; });
}

void jolt_quick_sort_floats(float *ioItems, uint32 inCount)		{ QuickSort(ioItems, ioItems + inCount); }
void jolt_insertion_sort_floats(float *ioItems, uint32 inCount)	{ InsertionSort(ioItems, ioItems + inCount); }

// Core/BinaryHeap.h
void jolt_binary_heap_push_less(SortElem *ioItems, uint32 inCount)
{
	BinaryHeapPush(ioItems, ioItems + inCount, [](const SortElem &inLHS, const SortElem &inRHS) { return inLHS.mKey < inRHS.mKey; });
}

void jolt_binary_heap_pop_less(SortElem *ioItems, uint32 inCount)
{
	BinaryHeapPop(ioItems, ioItems + inCount, [](const SortElem &inLHS, const SortElem &inRHS) { return inLHS.mKey < inRHS.mKey; });
}

void jolt_binary_heap_push_less_equal(SortElem *ioItems, uint32 inCount)
{
	BinaryHeapPush(ioItems, ioItems + inCount, [](const SortElem &inLHS, const SortElem &inRHS) { return inLHS.mKey <= inRHS.mKey; });
}

void jolt_binary_heap_pop_less_equal(SortElem *ioItems, uint32 inCount)
{
	BinaryHeapPop(ioItems, ioItems + inCount, [](const SortElem &inLHS, const SortElem &inRHS) { return inLHS.mKey <= inRHS.mKey; });
}

// Core/HashCombine.h
uint64 jolt_hash_bytes(const void *inData, uint32 inSize, uint64 inSeed)	{ return HashBytes(inData, inSize, inSeed); }
uint64 jolt_hash_bytes_default_seed(const void *inData, uint32 inSize)	{ return HashBytes(inData, inSize); }
uint64 jolt_hash_string(const char *inString)							{ return HashString(inString); }
uint64 jolt_hash_c_string(const char *inString)							{ return Hash<const char *> { } (inString); }
uint64 jolt_hash_string_view(const char *inData, uint32 inSize)			{ return Hash<std::string_view> { } (std::string_view(inData, inSize)); }
uint64 jolt_hash64(uint64 inValue)										{ return Hash64(inValue); }
uint64 jolt_hash_float(float inValue)									{ return Hash<float> { } (inValue); }
uint64 jolt_hash_double(double inValue)									{ return Hash<double> { } (inValue); }
uint64 jolt_hash_int(int inValue)										{ return Hash<int> { } (inValue); }
uint64 jolt_hash_uint32(uint32 inValue)									{ return Hash<uint32> { } (inValue); }
uint64 jolt_hash_uint64(uint64 inValue)									{ return Hash<uint64> { } (inValue); }
uint64 jolt_hash_char(uint8 inValue)									{ return Hash<char> { } (char(inValue)); }

uint64 jolt_hash_string_jolt_string(const char *inData, uint32 inSize)
{
	EnsureAllocator();
	return Hash<String> { } (String(inData, inSize));
}

uint64 jolt_hash_combine_sequence(uint64 inSeed, const uint32 *inTypes, const uint64 *inValues, uint32 inCount)
{
	uint64 seed = inSeed;
	for (uint32 i = 0; i < inCount; ++i)
	{
		uint64 v = inValues[i];
		switch (EHashType(inTypes[i]))
		{
		case EHashType::Float:	{ uint32 bits = uint32(v); float f; memcpy(&f, &bits, sizeof(f)); HashCombine(seed, f); break; }
		case EHashType::Double:	{ double d; memcpy(&d, &v, sizeof(d)); HashCombine(seed, d); break; }
		case EHashType::Int:	HashCombine(seed, int(uint32(v))); break;
		case EHashType::UInt32:	HashCombine(seed, uint32(v)); break;
		case EHashType::UInt64:	HashCombine(seed, v); break;
		case EHashType::Char:	HashCombine(seed, char(uint8(v))); break;
		}
	}
	return seed;
}

uint64 jolt_hash_combine_args(float inA, uint32 inB, int inC, uint64 inD, double inE)
{
	return HashCombineArgs(inA, inB, inC, inD, inE);
}

// std::mt19937 (replaced by Core/Mt19937.zig)
void jolt_mt19937(uint32 inSeed, uint32 *outValues, uint32 inCount)
{
	std::mt19937 random(inSeed);
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = uint32(random());
}

void jolt_mt19937_default(uint32 *outValues, uint32 inCount)
{
	std::mt19937 random;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = uint32(random());
}

// Core/LinearCurve.h
void jolt_linear_curve_sort(float *ioPoints, uint32 inCount)
{
	EnsureAllocator();
	LinearCurve curve = MakeCurve(ioPoints, inCount);
	curve.Sort();
	StoreCurve(curve, ioPoints);
}

void jolt_linear_curve_get_values(const float *inPoints, uint32 inCount, const float *inX, float *outY, uint32 inNumX)
{
	EnsureAllocator();
	LinearCurve curve = MakeCurve(inPoints, inCount);
	for (uint32 i = 0; i < inNumX; ++i)
		outY[i] = curve.GetValue(inX[i]);
}

void jolt_linear_curve_min_max(const float *inPoints, uint32 inCount, float *outMinMax)
{
	EnsureAllocator();
	LinearCurve curve = MakeCurve(inPoints, inCount);
	outMinMax[0] = curve.GetMinX();
	outMinMax[1] = curve.GetMaxX();
}

uint32 jolt_linear_curve_save(const float *inPoints, uint32 inCount, uint8 *outBytes, uint32 inCapacity)
{
	EnsureAllocator();
	LinearCurve curve = MakeCurve(inPoints, inCount);
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	curve.SaveBinaryState(wrapper);
	return CopyString(stream.str(), outBytes, inCapacity);
}

// Restores a curve, returns the number of points (writes at most inCapacity points) and the stream state
uint32 jolt_linear_curve_restore(const uint8 *inBytes, uint32 inSize, float *outPoints, uint32 inCapacity, bool *outEOF, bool *outFailed)
{
	EnsureAllocator();
	std::istringstream stream(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper wrapper(stream);
	LinearCurve curve;
	curve.RestoreBinaryState(wrapper);
	*outEOF = wrapper.IsEOF();
	*outFailed = wrapper.IsFailed();
	uint32 count = uint32(curve.mPoints.size());
	if (count <= inCapacity)
		StoreCurve(curve, outPoints);
	return count;
}

// Core/StreamIn.h, Core/StreamOut.h, Core/StreamWrapper.h: write all fields of the sample, returns the number of bytes
uint32 jolt_stream_write_sample(const StreamSample *inSample, uint8 *outBytes, uint32 inCapacity)
{
	EnsureAllocator();
	std::ostringstream stream;
	StreamOutWrapper wrapper(stream);
	StreamOut &out = wrapper;

	out.Write(inSample->mU8);
	out.Write(inSample->mU16);
	out.Write(inSample->mU32);
	out.Write(inSample->mU64);
	out.Write(inSample->mI32);
	out.Write(inSample->mFloat);
	out.Write(inSample->mDouble);
	out.Write(inSample->mBool);
	out.Write(EStreamEnum(inSample->mEnum));
	out.Write(LoadVec3(inSample->mVec3));
	out.Write(LoadVec4(inSample->mVec4));
	out.Write(Quat(LoadVec4(inSample->mQuat)));
	out.Write(Mat44(LoadVec4(inSample->mMat44), LoadVec4(inSample->mMat44 + 4), LoadVec4(inSample->mMat44 + 8), LoadVec4(inSample->mMat44 + 12)));
	out.Write(Float3(inSample->mFloat3[0], inSample->mFloat3[1], inSample->mFloat3[2]));
	out.Write(LoadDVec3(inSample->mDVec3));
	out.Write(LoadDMat44(inSample->mDMat44Cols, inSample->mDMat44T));

	Array<float> float_array(inSample->mFloatArray, inSample->mFloatArray + inSample->mFloatArrayLen);
	out.Write(float_array);

	Array<Vec3> vec3_array;
	for (uint32 i = 0; i < inSample->mVec3ArrayLen; ++i)
		vec3_array.push_back(LoadVec3(inSample->mVec3Array[i]));
	out.Write(vec3_array);

	Array<DVec3> dvec3_array;
	for (uint32 i = 0; i < inSample->mDVec3ArrayLen; ++i)
		dvec3_array.push_back(LoadDVec3(inSample->mDVec3Array[i]));
	out.Write(dvec3_array);

	Array<DMat44> dmat44_array;
	for (uint32 i = 0; i < inSample->mDMat44ArrayLen; ++i)
		dmat44_array.push_back(LoadDMat44(inSample->mDMat44ArrayCols[i], inSample->mDMat44ArrayT[i]));
	out.Write(dmat44_array);

	out.Write(String(inSample->mString, inSample->mStringLen));

	Array<LinearCurve::Point> point_array;
	for (uint32 i = 0; i < inSample->mPointArrayLen; ++i)
		point_array.push_back({ inSample->mPointArray[i][0], inSample->mPointArray[i][1] });
	out.Write(point_array, [](const LinearCurve::Point &inElement, StreamOut &inS) {
		inS.Write(inElement.mY);
		inS.Write(inElement.mX);
	});

	MakeCurve(&inSample->mCurve[0][0], inSample->mCurveLen).SaveBinaryState(out);

	return CopyString(stream.str(), outBytes, inCapacity);
}

// Read all fields of the sample in the same order as jolt_stream_write_sample. The destination values start at zero
// (empty arrays). After each field outStatus[field] = IsEOF() | IsFailed() << 1.
void jolt_stream_read_sample(const uint8 *inBytes, uint32 inSize, StreamSample *outSample, uint8 *outStatus)
{
	EnsureAllocator();
	std::istringstream stream(std::string(reinterpret_cast<const char *>(inBytes), inSize));
	StreamInWrapper wrapper(stream);
	StreamIn &in = wrapper;

	memset(outSample, 0, sizeof(StreamSample));
	uint field = 0;
	auto status = [&]() { outStatus[field++] = uint8(in.IsEOF()) | (uint8(in.IsFailed()) << 1); };

	in.Read(outSample->mU8); status();
	in.Read(outSample->mU16); status();
	in.Read(outSample->mU32); status();
	in.Read(outSample->mU64); status();
	in.Read(outSample->mI32); status();
	in.Read(outSample->mFloat); status();
	in.Read(outSample->mDouble); status();
	in.Read(outSample->mBool); status();

	EStreamEnum e = EStreamEnum::A;
	in.Read(e); status();
	outSample->mEnum = uint16(e);

	Vec3 vec3 = Vec3::sZero();
	in.Read(vec3); status();
	StoreVec3(vec3, outSample->mVec3);

	Vec4 vec4 = Vec4::sZero();
	in.Read(vec4); status();
	StoreVec4(vec4, outSample->mVec4);

	Quat quat(0, 0, 0, 0);
	in.Read(quat); status();
	StoreVec4(quat.GetXYZW(), outSample->mQuat);

	Mat44 mat44 = Mat44::sZero();
	in.Read(mat44); status();
	for (uint c = 0; c < 4; ++c)
		StoreVec4(mat44.GetColumn4(c), outSample->mMat44 + 4 * c);

	Float3 float3(0, 0, 0);
	in.Read(float3); status();
	outSample->mFloat3[0] = float3.x;
	outSample->mFloat3[1] = float3.y;
	outSample->mFloat3[2] = float3.z;

	DVec3 dvec3 = DVec3::sZero();
	in.Read(dvec3); status();
	StoreDVec3(dvec3, outSample->mDVec3);

	DMat44 dmat44 = DMat44::sZero();
	in.Read(dmat44); status();
	StoreDMat44(dmat44, outSample->mDMat44Cols, outSample->mDMat44T);

	Array<float> float_array;
	in.Read(float_array); status();
	outSample->mFloatArrayLen = uint32(float_array.size());
	if (StreamOK(in) && float_array.size() <= 8)
		memcpy(outSample->mFloatArray, float_array.data(), float_array.size() * sizeof(float));

	Array<Vec3> vec3_array;
	in.Read(vec3_array); status();
	outSample->mVec3ArrayLen = uint32(vec3_array.size());
	if (StreamOK(in) && vec3_array.size() <= 4)
		for (size_t i = 0; i < vec3_array.size(); ++i)
			StoreVec3(vec3_array[i], outSample->mVec3Array[i]);

	Array<DVec3> dvec3_array;
	in.Read(dvec3_array); status();
	outSample->mDVec3ArrayLen = uint32(dvec3_array.size());
	if (StreamOK(in) && dvec3_array.size() <= 3)
		for (size_t i = 0; i < dvec3_array.size(); ++i)
			StoreDVec3(dvec3_array[i], outSample->mDVec3Array[i]);

	Array<DMat44> dmat44_array;
	in.Read(dmat44_array); status();
	outSample->mDMat44ArrayLen = uint32(dmat44_array.size());
	if (StreamOK(in) && dmat44_array.size() <= 2)
		for (size_t i = 0; i < dmat44_array.size(); ++i)
			StoreDMat44(dmat44_array[i], outSample->mDMat44ArrayCols[i], outSample->mDMat44ArrayT[i]);

	String string;
	in.Read(string); status();
	outSample->mStringLen = CopyString(string, outSample->mString, 16);

	Array<LinearCurve::Point> point_array;
	in.Read(point_array, [](StreamIn &inS, LinearCurve::Point &outElement) {
		inS.Read(outElement.mY);
		inS.Read(outElement.mX);
	});
	status();
	outSample->mPointArrayLen = uint32(point_array.size());
	if (point_array.size() <= 4)
		for (size_t i = 0; i < point_array.size(); ++i)
		{
			outSample->mPointArray[i][0] = point_array[i].mX;
			outSample->mPointArray[i][1] = point_array[i].mY;
		}

	LinearCurve curve;
	curve.RestoreBinaryState(in); status();
	outSample->mCurveLen = uint32(curve.mPoints.size());
	if (curve.mPoints.size() <= 4)
		StoreCurve(curve, &outSample->mCurve[0][0]);

	JPH_ASSERT(field == cNumStreamFields);
}

// Core/StringTools.h
uint32 jolt_convert_to_string_float(float inValue, char *outChars, uint32 inCapacity)	{ EnsureAllocator(); return CopyString(ConvertToString(inValue), outChars, inCapacity); }
uint32 jolt_convert_to_string_double(double inValue, char *outChars, uint32 inCapacity)	{ EnsureAllocator(); return CopyString(ConvertToString(inValue), outChars, inCapacity); }
uint32 jolt_convert_to_string_int(int inValue, char *outChars, uint32 inCapacity)		{ EnsureAllocator(); return CopyString(ConvertToString(inValue), outChars, inCapacity); }
uint32 jolt_convert_to_string_uint64(uint64 inValue, char *outChars, uint32 inCapacity)	{ EnsureAllocator(); return CopyString(ConvertToString(inValue), outChars, inCapacity); }

uint32 jolt_to_lower(const char *inChars, uint32 inSize, char *outChars, uint32 inCapacity)
{
	EnsureAllocator();
	return CopyString(ToLower(string_view(inChars, inSize)), outChars, inCapacity);
}

uint32 jolt_string_replace(const char *inChars, uint32 inSize, const char *inSearch, uint32 inSearchSize, const char *inReplace, uint32 inReplaceSize, char *outChars, uint32 inCapacity)
{
	EnsureAllocator();
	String s(inChars, inSize);
	StringReplace(s, string_view(inSearch, inSearchSize), string_view(inReplace, inReplaceSize));
	return CopyString(s, outChars, inCapacity);
}

// Splits inChars, appending to a vector that initially holds inNumInitial copies of "x" (cleared first if inClearVector).
// Returns the number of strings, outLengths / outChars receive the lengths and the concatenated characters.
uint32 jolt_string_to_vector(const char *inChars, uint32 inSize, const char *inDelimiter, uint32 inDelimiterSize, bool inClearVector, uint32 inNumInitial, uint32 *outLengths, uint32 inMaxStrings, char *outChars, uint32 inCapacity)
{
	EnsureAllocator();
	Array<String> vector(inNumInitial, String("x"));
	StringToVector(string_view(inChars, inSize), vector, string_view(inDelimiter, inDelimiterSize), inClearVector);
	uint32 num_chars = 0;
	for (size_t i = 0; i < vector.size() && i < inMaxStrings; ++i)
	{
		outLengths[i] = uint32(vector[i].size());
		if (num_chars + vector[i].size() <= inCapacity)
			memcpy(outChars + num_chars, vector[i].data(), vector[i].size());
		num_chars += uint32(vector[i].size());
	}
	return uint32(vector.size());
}

// Joins inNumStrings strings (lengths in inLengths, characters concatenated in inChars)
uint32 jolt_vector_to_string(const uint32 *inLengths, uint32 inNumStrings, const char *inChars, const char *inDelimiter, uint32 inDelimiterSize, char *outChars, uint32 inCapacity)
{
	EnsureAllocator();
	Array<String> vector;
	uint32 offset = 0;
	for (uint32 i = 0; i < inNumStrings; ++i)
	{
		vector.push_back(String(inChars + offset, inLengths[i]));
		offset += inLengths[i];
	}
	String s = "garbage"; // VectorToString clears the output first
	VectorToString(vector, s, string_view(inDelimiter, inDelimiterSize));
	return CopyString(s, outChars, inCapacity);
}

} // extern "C"
