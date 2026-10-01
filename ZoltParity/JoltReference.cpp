// Reference implementation for the parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library. ZoltParity/parity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w),
// matrices as 16 floats in column major order. Add a wrapper here for every Jolt function that gets a parity test.

#include <Jolt/Jolt.h>
#include <Jolt/Math/Trigonometry.h>

using namespace JPH;

static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static void Store4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }

extern "C"
{

// Math/Trigonometry.h
float jolt_acos_approximate(float inX)				{ return ACosApproximate(inX); }

// Math/Vec4.h
void jolt_vec4_sincos(const float *inV, float *outSin, float *outCos)
{
	Vec4 s, c;
	Load4(inV).SinCos(s, c);
	Store4(s, outSin);
	Store4(c, outCos);
}
void jolt_vec4_tan(const float *inV, float *outV)	{ Store4(Load4(inV).Tan(), outV); }
void jolt_vec4_asin(const float *inV, float *outV)	{ Store4(Load4(inV).ASin(), outV); }
void jolt_vec4_acos(const float *inV, float *outV)	{ Store4(Load4(inV).ACos(), outV); }
void jolt_vec4_atan(const float *inV, float *outV)	{ Store4(Load4(inV).ATan(), outV); }
void jolt_vec4_atan2(const float *inY, const float *inX, float *outV) { Store4(Vec4::sATan2(Load4(inY), Load4(inX)), outV); }
void jolt_vec4_normalized(const float *inV, float *outV) { Store4(Load4(inV).Normalized(), outV); }
float jolt_vec4_dot(const float *inA, const float *inB) { return Load4(inA).Dot(Load4(inB)); }
float jolt_vec4_length(const float *inV)			{ return Load4(inV).Length(); }
float jolt_vec4_reduce_min(const float *inV)		{ return Load4(inV).ReduceMin(); }
float jolt_vec4_reduce_max(const float *inV)		{ return Load4(inV).ReduceMax(); }
uint32 jolt_vec4_compress_unit_vector(const float *inV) { return Load4(inV).CompressUnitVector(); }
void jolt_vec4_decompress_unit_vector(uint32 inValue, float *outV) { Store4(Vec4::sDecompressUnitVector(inValue), outV); }

// Math/Vec3.h
void jolt_vec3_normalized(const float *inV, float *outV) { Store3(Load3(inV).Normalized(), outV); }
void jolt_vec3_cross(const float *inA, const float *inB, float *outV) { Store3(Load3(inA).Cross(Load3(inB)), outV); }
float jolt_vec3_dot(const float *inA, const float *inB) { return Load3(inA).Dot(Load3(inB)); }
float jolt_vec3_length(const float *inV)			{ return Load3(inV).Length(); }
void jolt_vec3_normalized_perpendicular(const float *inV, float *outV) { Store3(Load3(inV).GetNormalizedPerpendicular(), outV); }
void jolt_vec3_unit_spherical(float inTheta, float inPhi, float *outV) { Store3(Vec3::sUnitSpherical(inTheta, inPhi), outV); }
uint32 jolt_vec3_compress_unit_vector(const float *inV) { return Load3(inV).CompressUnitVector(); }
void jolt_vec3_decompress_unit_vector(uint32 inValue, float *outV) { Store3(Vec3::sDecompressUnitVector(inValue), outV); }

} // extern "C"
