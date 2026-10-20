// Reference implementation for ZoltParity/Compute/ShaderHelpersParity.zig and HLSLToCPPParity.zig: the shader helper
// functions and the operators of HLSLToCPP.h, compiled like Jolt's CPU compute backend compiles a shader
// (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, R11, R12).
//
// - The helpers: a shader wrapper class built with Jolt's own macro headers (WrapShaderBegin.h, WrapShaderBindings.h,
//   WrapShaderEnd.h) around HairStructs.h and HairCommon.h. HairCommon.h includes ShaderMath.h, ShaderMat44.h,
//   ShaderQuat.h, ShaderPlane.h and ShaderVec3.h, so every helper is a member function of the class, exactly as in a hair
//   shader. The grid helpers read the constants of gContext, which jolt_shader_grid binds with the wrapper's Bind.
// - The operators: every operator and function of HLSLToCPP.h, called in the namespace context of a shader wrapper
//   (JPH::ShaderWrappers uses `using namespace HLSLToCPP`), by an operation code. The names of the operations are
//   exported, so the Zig side checks that its enums match.
//
// Nothing here allocates through Jolt, so the file needs no allocation hooks.
// Note: HLSLToCPP.h defines the swizzle macros (`#define xyz swizzle_xyz()`) for the rest of this file, so no identifier
// below may be xy, yx, xyz, xzy, yxz, yzx, zxy, zyx, xywz, xwyz or wxyz.
// Conventions: no bool parameters (porting guide section 10). All functions are prefixed with jolt_shader_ or jolt_hlsl_.

#include <Jolt/Jolt.h>

#include <cstring>
#include <type_traits>

// The helpers as members of a shader wrapper class (what HairWrapper.cpp does for every hair shader)
#define JPH_SHADER_NAME ZoltShaderHelpers
#include <Jolt/Compute/CPU/WrapShaderBegin.h>
#include <Jolt/Shaders/HairStructs.h>
#include <Jolt/Shaders/HairCommon.h>

JPH_SHADER_FUNCTION_BEGIN(void, main, 1, 1, 1)
	JPH_SHADER_PARAM_THREAD_ID(tid)
JPH_SHADER_FUNCTION_END
{
	(void)tid;
}

#include <Jolt/Compute/CPU/WrapShaderBindings.h>
#include <Jolt/Shaders/HairStructs.h>
#include <Jolt/Compute/CPU/WrapShaderEnd.h>

JPH_NAMESPACE_BEGIN

namespace ZoltHLSLReference {

using namespace HLSLToCPP;
using Helpers = ShaderWrappers::ZoltShaderHelpersShaderWrapper;

//////////////////////////////////////////////////////////////////////////////////////////
// The operations of HLSLToCPP.h. Must match the enums of HLSLToCPPParity.zig (it compares the names).
//////////////////////////////////////////////////////////////////////////////////////////

#define ZOLT_FLOAT_VECTOR_OPS(X) X(add) X(sub) X(mul) X(div) X(mul_scalar) X(scalar_mul) X(div_scalar) X(negate) X(add_assign) X(sub_assign) X(mul_assign) X(div_assign) X(mul_assign_scalar) X(div_assign_scalar) X(eq) X(ne) X(dot) X(min) X(max) X(length) X(normalize) X(round) X(clamp) X(splat) X(get) X(set)
#define ZOLT_INT_VECTOR_OPS(X) X(add) X(sub) X(mul) X(div) X(mul_scalar) X(scalar_mul) X(div_scalar) X(negate) X(add_assign) X(sub_assign) X(mul_assign) X(div_assign) X(mul_assign_scalar) X(div_assign_scalar) X(eq) X(ne) X(dot) X(min) X(max) X(clamp) X(splat) X(get) X(set)
#define ZOLT_MISC_OPS(X) \
	X(f2_swizzle_xy) X(f2_swizzle_yx) \
	X(f3_swizzle_xy) X(f3_swizzle_yx) X(f3_swizzle_xyz) X(f3_swizzle_xzy) X(f3_swizzle_yxz) X(f3_swizzle_yzx) X(f3_swizzle_zxy) X(f3_swizzle_zyx) \
	X(f4_swizzle_xy) X(f4_swizzle_yx) X(f4_swizzle_xyz) X(f4_swizzle_xzy) X(f4_swizzle_yxz) X(f4_swizzle_yzx) X(f4_swizzle_zxy) X(f4_swizzle_zyx) X(f4_swizzle_xywz) X(f4_swizzle_xwyz) X(f4_swizzle_wxyz) \
	X(u3_swizzle_xyz) X(u3_swizzle_xzy) X(u3_swizzle_yxz) X(u3_swizzle_yzx) X(u3_swizzle_zxy) X(u3_swizzle_zyx) \
	X(u4_swizzle_xyz) X(u4_swizzle_xzy) X(u4_swizzle_yxz) X(u4_swizzle_yzx) X(u4_swizzle_zxy) X(u4_swizzle_zyx) X(u4_swizzle_xywz) X(u4_swizzle_xwyz) X(u4_swizzle_wxyz) \
	X(i3_swizzle_xyz) X(i3_swizzle_xzy) X(i3_swizzle_yxz) X(i3_swizzle_yzx) X(i3_swizzle_zxy) X(i3_swizzle_zyx) \
	X(i4_swizzle_xyz) X(i4_swizzle_xzy) X(i4_swizzle_yxz) X(i4_swizzle_yzx) X(i4_swizzle_zxy) X(i4_swizzle_zyx) X(i4_swizzle_xywz) X(i4_swizzle_xwyz) X(i4_swizzle_wxyz) \
	X(f3_from_float2) X(f3_from_uint3) X(f4_from_float3) X(f4_from_int4) X(f4_asint) X(f3_cross) \
	X(u3_from_float3) X(u4_from_uint3) X(i3_from_float3) X(i4_from_int3) X(i4_from_float4) X(mat44_column) \
	X(float_min) X(float_max) X(float_clamp) X(float_sqrt) X(float_round) \
	X(uint_min) X(uint_max) X(uint_clamp) X(int_min) X(int_max) X(int_clamp) \
	X(to_int) X(to_uint) X(float_from_uint) X(float_from_int) X(atomic_add_int) X(atomic_add_uint)

#define ZOLT_ENUM_ENTRY(name)	name,
#define ZOLT_NAME_ENTRY(name)	#name,

enum class EFloatVectorOp : uint32 { ZOLT_FLOAT_VECTOR_OPS(ZOLT_ENUM_ENTRY) Count };
enum class EIntVectorOp : uint32 { ZOLT_INT_VECTOR_OPS(ZOLT_ENUM_ENTRY) Count };
enum class EMiscOp : uint32 { ZOLT_MISC_OPS(ZOLT_ENUM_ENTRY) Count };

static const char *sFloatVectorOpNames[] = { ZOLT_FLOAT_VECTOR_OPS(ZOLT_NAME_ENTRY) };
static const char *sIntVectorOpNames[] = { ZOLT_INT_VECTOR_OPS(ZOLT_NAME_ENTRY) };
static const char *sMiscOpNames[] = { ZOLT_MISC_OPS(ZOLT_NAME_ENTRY) };

/// The operands of one operation: three float4, three int4 and three uint4 (an operation reads what it needs, a scalar
/// is component x of the second operand). Must match HLSLOpInput in HLSLToCPPParity.zig.
struct HLSLOpInput
{
	float			mF[3][4];
	int				mI[3][4];
	uint32			mU[3][4];
};

/// The results of one operation, pre-filled by the caller. Must match HLSLOpOutput in HLSLToCPPParity.zig.
struct HLSLOpOutput
{
	float			mF[4];
	int				mI[4];
	uint32			mU[4];
};

static_assert(sizeof(HLSLOpInput) == 144 && sizeof(HLSLOpOutput) == 48);

/// Number of components of a vector type
template <class T> constexpr uint cDim = 0;
template <> constexpr uint cDim<float2> = 2;
template <> constexpr uint cDim<float3> = 3;
template <> constexpr uint cDim<float4> = 4;
template <> constexpr uint cDim<uint3> = 3;
template <> constexpr uint cDim<uint4> = 4;
template <> constexpr uint cDim<int3> = 3;
template <> constexpr uint cDim<int4> = 4;

template <class T, class S>
static T Load(const S *inValues)
{
	T v;
	for (uint i = 0; i < cDim<T>; ++i)
		v[i] = inValues[i];
	return v;
}

template <class T, class S>
static void Store(const T &inV, S *outValues)
{
	for (uint i = 0; i < cDim<T>; ++i)
		outValues[i] = inV[i];
}

template <class T>
static void FloatVectorOp(EFloatVectorOp inOp, const HLSLOpInput &inIn, HLSLOpOutput &outOut)
{
	const T a = Load<T>(inIn.mF[0]);
	const T b = Load<T>(inIn.mF[1]);
	const T c = Load<T>(inIn.mF[2]);
	const float s = inIn.mF[1][0];
	const uint index = inIn.mU[0][0] % cDim<T>;
	T r = a;

	switch (inOp)
	{
	case EFloatVectorOp::add:				Store(a + b, outOut.mF); break;
	case EFloatVectorOp::sub:				Store(a - b, outOut.mF); break;
	case EFloatVectorOp::mul:				Store(a * b, outOut.mF); break;
	case EFloatVectorOp::div:				Store(a / b, outOut.mF); break;
	case EFloatVectorOp::mul_scalar:		Store(a * s, outOut.mF); break;
	case EFloatVectorOp::scalar_mul:		Store(s * a, outOut.mF); break;
	case EFloatVectorOp::div_scalar:		Store(a / s, outOut.mF); break;
	case EFloatVectorOp::negate:			Store(-a, outOut.mF); break;
	case EFloatVectorOp::add_assign:		r += b; Store(r, outOut.mF); break;
	case EFloatVectorOp::sub_assign:		r -= b; Store(r, outOut.mF); break;
	case EFloatVectorOp::mul_assign:		r *= b; Store(r, outOut.mF); break;
	case EFloatVectorOp::div_assign:		r /= b; Store(r, outOut.mF); break;
	case EFloatVectorOp::mul_assign_scalar:	r *= s; Store(r, outOut.mF); break;
	case EFloatVectorOp::div_assign_scalar:	r /= s; Store(r, outOut.mF); break;
	case EFloatVectorOp::eq:				outOut.mU[0] = a == b? 1 : 0; break;
	case EFloatVectorOp::ne:				outOut.mU[0] = a != b? 1 : 0; break;
	case EFloatVectorOp::dot:				outOut.mF[0] = dot(a, b); break;
	case EFloatVectorOp::min:				Store(min(a, b), outOut.mF); break;
	case EFloatVectorOp::max:				Store(max(a, b), outOut.mF); break;
	case EFloatVectorOp::length:			outOut.mF[0] = length(a); break;
	case EFloatVectorOp::normalize:			Store(normalize(a), outOut.mF); break;
	case EFloatVectorOp::round:				Store(round(a), outOut.mF); break;
	case EFloatVectorOp::clamp:				Store(clamp(a, b, c), outOut.mF); break;
	case EFloatVectorOp::splat:				Store(T(s), outOut.mF); break;
	case EFloatVectorOp::get:				outOut.mF[0] = a[index]; break;
	case EFloatVectorOp::set:				r[index] = s; Store(r, outOut.mF); break;
	case EFloatVectorOp::Count:				break;
	}
}

template <class T, class S>
static void IntVectorOp(EIntVectorOp inOp, const S (&inV)[3][4], S *outV, uint32 &outBool, uint32 inIndex)
{
	const T a = Load<T>(inV[0]);
	const T b = Load<T>(inV[1]);
	const T c = Load<T>(inV[2]);
	const S s = inV[1][0];
	const uint index = inIndex % cDim<T>;
	T r = a;

	switch (inOp)
	{
	case EIntVectorOp::add:					Store(a + b, outV); break;
	case EIntVectorOp::sub:					Store(a - b, outV); break;
	case EIntVectorOp::mul:					Store(a * b, outV); break;
	case EIntVectorOp::div:					Store(a / b, outV); break;
	case EIntVectorOp::mul_scalar:			Store(a * s, outV); break;
	case EIntVectorOp::scalar_mul:			Store(s * a, outV); break;
	case EIntVectorOp::div_scalar:			Store(a / s, outV); break;
	case EIntVectorOp::negate:				if constexpr (std::is_same_v<T, int3> || std::is_same_v<T, int4>) Store(-a, outV); break; // uint3 / uint4 have no unary minus
	case EIntVectorOp::add_assign:			r += b; Store(r, outV); break;
	case EIntVectorOp::sub_assign:			r -= b; Store(r, outV); break;
	case EIntVectorOp::mul_assign:			r *= b; Store(r, outV); break;
	case EIntVectorOp::div_assign:			r /= b; Store(r, outV); break;
	case EIntVectorOp::mul_assign_scalar:	r *= s; Store(r, outV); break;
	case EIntVectorOp::div_assign_scalar:	r /= s; Store(r, outV); break;
	case EIntVectorOp::eq:					outBool = a == b? 1 : 0; break;
	case EIntVectorOp::ne:					outBool = a != b? 1 : 0; break;
	case EIntVectorOp::dot:					outV[0] = dot(a, b); break;
	case EIntVectorOp::min:					Store(min(a, b), outV); break;
	case EIntVectorOp::max:					Store(max(a, b), outV); break;
	case EIntVectorOp::clamp:				Store(clamp(a, b, c), outV); break;
	case EIntVectorOp::splat:				if constexpr (std::is_same_v<T, uint4> || std::is_same_v<T, int4>) Store(T(s), outV); break; // uint3 / int3 have no such constructor
	case EIntVectorOp::get:					outV[0] = a[index]; break;
	case EIntVectorOp::set:					r[index] = s; Store(r, outV); break;
	case EIntVectorOp::Count:				break;
	}
}

static void MiscOp(EMiscOp inOp, const HLSLOpInput &inIn, HLSLOpOutput &outOut)
{
	const float2 f2 = Load<float2>(inIn.mF[0]);
	const float3 f3 = Load<float3>(inIn.mF[0]);
	const float4 f4 = Load<float4>(inIn.mF[0]);
	const float3 f3b = Load<float3>(inIn.mF[1]);
	const uint3 u3 = Load<uint3>(inIn.mU[0]);
	const uint4 u4 = Load<uint4>(inIn.mU[0]);
	const int3 i3 = Load<int3>(inIn.mI[0]);
	const int4 i4 = Load<int4>(inIn.mI[0]);
	const float fa = inIn.mF[0][0], fb = inIn.mF[1][0], fc = inIn.mF[2][0];
	const uint32 ua = inIn.mU[0][0], ub = inIn.mU[1][0], uc = inIn.mU[2][0];
	const int ia = inIn.mI[0][0], ib = inIn.mI[1][0], ic = inIn.mI[2][0];

	switch (inOp)
	{
	case EMiscOp::f2_swizzle_xy:	Store(f2.xy, outOut.mF); break;
	case EMiscOp::f2_swizzle_yx:	Store(f2.yx, outOut.mF); break;

	case EMiscOp::f3_swizzle_xy:	Store(f3.xy, outOut.mF); break;
	case EMiscOp::f3_swizzle_yx:	Store(f3.yx, outOut.mF); break;
	case EMiscOp::f3_swizzle_xyz:	Store(f3.xyz, outOut.mF); break;
	case EMiscOp::f3_swizzle_xzy:	Store(f3.xzy, outOut.mF); break;
	case EMiscOp::f3_swizzle_yxz:	Store(f3.yxz, outOut.mF); break;
	case EMiscOp::f3_swizzle_yzx:	Store(f3.yzx, outOut.mF); break;
	case EMiscOp::f3_swizzle_zxy:	Store(f3.zxy, outOut.mF); break;
	case EMiscOp::f3_swizzle_zyx:	Store(f3.zyx, outOut.mF); break;

	case EMiscOp::f4_swizzle_xy:	Store(f4.xy, outOut.mF); break;
	case EMiscOp::f4_swizzle_yx:	Store(f4.yx, outOut.mF); break;
	case EMiscOp::f4_swizzle_xyz:	Store(f4.xyz, outOut.mF); break;
	case EMiscOp::f4_swizzle_xzy:	Store(f4.xzy, outOut.mF); break;
	case EMiscOp::f4_swizzle_yxz:	Store(f4.yxz, outOut.mF); break;
	case EMiscOp::f4_swizzle_yzx:	Store(f4.yzx, outOut.mF); break;
	case EMiscOp::f4_swizzle_zxy:	Store(f4.zxy, outOut.mF); break;
	case EMiscOp::f4_swizzle_zyx:	Store(f4.zyx, outOut.mF); break;
	case EMiscOp::f4_swizzle_xywz:	Store(f4.xywz, outOut.mF); break;
	case EMiscOp::f4_swizzle_xwyz:	Store(f4.xwyz, outOut.mF); break;
	case EMiscOp::f4_swizzle_wxyz:	Store(f4.wxyz, outOut.mF); break;

	case EMiscOp::u3_swizzle_xyz:	Store(u3.xyz, outOut.mU); break;
	case EMiscOp::u3_swizzle_xzy:	Store(u3.xzy, outOut.mU); break;
	case EMiscOp::u3_swizzle_yxz:	Store(u3.yxz, outOut.mU); break;
	case EMiscOp::u3_swizzle_yzx:	Store(u3.yzx, outOut.mU); break;
	case EMiscOp::u3_swizzle_zxy:	Store(u3.zxy, outOut.mU); break;
	case EMiscOp::u3_swizzle_zyx:	Store(u3.zyx, outOut.mU); break;

	case EMiscOp::u4_swizzle_xyz:	Store(u4.xyz, outOut.mU); break;
	case EMiscOp::u4_swizzle_xzy:	Store(u4.xzy, outOut.mU); break;
	case EMiscOp::u4_swizzle_yxz:	Store(u4.yxz, outOut.mU); break;
	case EMiscOp::u4_swizzle_yzx:	Store(u4.yzx, outOut.mU); break;
	case EMiscOp::u4_swizzle_zxy:	Store(u4.zxy, outOut.mU); break;
	case EMiscOp::u4_swizzle_zyx:	Store(u4.zyx, outOut.mU); break;
	case EMiscOp::u4_swizzle_xywz:	Store(u4.xywz, outOut.mU); break;
	case EMiscOp::u4_swizzle_xwyz:	Store(u4.xwyz, outOut.mU); break;
	case EMiscOp::u4_swizzle_wxyz:	Store(u4.wxyz, outOut.mU); break;

	case EMiscOp::i3_swizzle_xyz:	Store(i3.xyz, outOut.mI); break;
	case EMiscOp::i3_swizzle_xzy:	Store(i3.xzy, outOut.mI); break;
	case EMiscOp::i3_swizzle_yxz:	Store(i3.yxz, outOut.mI); break;
	case EMiscOp::i3_swizzle_yzx:	Store(i3.yzx, outOut.mI); break;
	case EMiscOp::i3_swizzle_zxy:	Store(i3.zxy, outOut.mI); break;
	case EMiscOp::i3_swizzle_zyx:	Store(i3.zyx, outOut.mI); break;

	case EMiscOp::i4_swizzle_xyz:	Store(i4.xyz, outOut.mI); break;
	case EMiscOp::i4_swizzle_xzy:	Store(i4.xzy, outOut.mI); break;
	case EMiscOp::i4_swizzle_yxz:	Store(i4.yxz, outOut.mI); break;
	case EMiscOp::i4_swizzle_yzx:	Store(i4.yzx, outOut.mI); break;
	case EMiscOp::i4_swizzle_zxy:	Store(i4.zxy, outOut.mI); break;
	case EMiscOp::i4_swizzle_zyx:	Store(i4.zyx, outOut.mI); break;
	case EMiscOp::i4_swizzle_xywz:	Store(i4.xywz, outOut.mI); break;
	case EMiscOp::i4_swizzle_xwyz:	Store(i4.xwyz, outOut.mI); break;
	case EMiscOp::i4_swizzle_wxyz:	Store(i4.wxyz, outOut.mI); break;

	case EMiscOp::f3_from_float2:	Store(float3(f2, fb), outOut.mF); break;
	case EMiscOp::f3_from_uint3:	Store(float3(u3), outOut.mF); break;
	case EMiscOp::f4_from_float3:	Store(float4(f3, fb), outOut.mF); break;
	case EMiscOp::f4_from_int4:		Store(float4(i4), outOut.mF); break;
	case EMiscOp::f4_asint:			Store(asint(f4), outOut.mI); break;
	case EMiscOp::f3_cross:			Store(cross(f3, f3b), outOut.mF); break;
	case EMiscOp::u3_from_float3:	Store(uint3(f3), outOut.mU); break;
	case EMiscOp::u4_from_uint3:	Store(uint4(u3, ub), outOut.mU); break;
	case EMiscOp::i3_from_float3:	Store(int3(f3), outOut.mI); break;
	case EMiscOp::i4_from_int3:		Store(int4(i3, ib), outOut.mI); break;
	case EMiscOp::i4_from_float4:	Store(int4(f4), outOut.mI); break;
	case EMiscOp::mat44_column:
		{
			const HLSLToCPP::Mat44 m(f4, Load<float4>(inIn.mF[1]), Load<float4>(inIn.mF[2]), float4(fa, fb, fc, 1.0f));
			Store(m[ua % 4], outOut.mF);
			break;
		}

	case EMiscOp::float_min:		outOut.mF[0] = min(fa, fb); break;
	case EMiscOp::float_max:		outOut.mF[0] = max(fa, fb); break;
	case EMiscOp::float_clamp:		outOut.mF[0] = clamp(fa, fb, fc); break;
	case EMiscOp::float_sqrt:		outOut.mF[0] = sqrt(fa); break;
	case EMiscOp::float_round:		outOut.mF[0] = round(fa); break;
	case EMiscOp::uint_min:			outOut.mU[0] = min(ua, ub); break;
	case EMiscOp::uint_max:			outOut.mU[0] = max(ua, ub); break;
	case EMiscOp::uint_clamp:		outOut.mU[0] = clamp(ua, ub, uc); break;
	case EMiscOp::int_min:			outOut.mI[0] = min(ia, ib); break;
	case EMiscOp::int_max:			outOut.mI[0] = max(ia, ib); break;
	case EMiscOp::int_clamp:		outOut.mI[0] = clamp(ia, ib, ic); break;

	case EMiscOp::to_int:			outOut.mI[0] = int(fa); break;
	case EMiscOp::to_uint:			outOut.mU[0] = uint32(fa); break;
	case EMiscOp::float_from_uint:	outOut.mF[0] = float(ua); break;
	case EMiscOp::float_from_int:	outOut.mF[0] = float(ia); break;
	case EMiscOp::atomic_add_int:
		{
			int value = ia;
			outOut.mI[0] = JPH_AtomicAdd(value, ib);
			outOut.mI[1] = value;
			break;
		}
	case EMiscOp::atomic_add_uint:
		{
			uint32 value = ua;
			outOut.mU[0] = JPH_AtomicAdd(value, ub);
			outOut.mU[1] = value;
			break;
		}

	case EMiscOp::Count:			break;
	}
}

} // ZoltHLSLReference

JPH_NAMESPACE_END

using namespace JPH;
using namespace JPH::ZoltHLSLReference;

extern "C" {

//////////////////////////////////////////////////////////////////////////////////////////
// HLSLToCPP.h
//////////////////////////////////////////////////////////////////////////////////////////

/// The name of operation inOp of a family (0 = float vector, 1 = int vector, 2 = misc), nullptr past the last one
const char *jolt_hlsl_op_name(uint32 inFamily, uint32 inOp)
{
	switch (inFamily)
	{
	case 0:		return inOp < uint32(EFloatVectorOp::Count)? sFloatVectorOpNames[inOp] : nullptr;
	case 1:		return inOp < uint32(EIntVectorOp::Count)? sIntVectorOpNames[inOp] : nullptr;
	default:	return inOp < uint32(EMiscOp::Count)? sMiscOpNames[inOp] : nullptr;
	}
}

/// Run a float vector operation on float2 (inDim 2), float3 (3) or float4 (4) for inCount inputs
void jolt_hlsl_float_vector_op(uint32 inDim, uint32 inOp, const HLSLOpInput *inIn, HLSLOpOutput *ioOut, uint32 inCount)
{
	for (uint32 i = 0; i < inCount; ++i)
		switch (inDim)
		{
		case 2:		FloatVectorOp<float2>(EFloatVectorOp(inOp), inIn[i], ioOut[i]); break;
		case 3:		FloatVectorOp<float3>(EFloatVectorOp(inOp), inIn[i], ioOut[i]); break;
		default:	FloatVectorOp<float4>(EFloatVectorOp(inOp), inIn[i], ioOut[i]); break;
		}
}

/// Run an integer vector operation on uint3 (inType 0), uint4 (1), int3 (2) or int4 (3) for inCount inputs
void jolt_hlsl_int_vector_op(uint32 inType, uint32 inOp, const HLSLOpInput *inIn, HLSLOpOutput *ioOut, uint32 inCount)
{
	for (uint32 i = 0; i < inCount; ++i)
	{
		const HLSLOpInput &in = inIn[i];
		HLSLOpOutput &out = ioOut[i];
		switch (inType)
		{
		case 0:		IntVectorOp<uint3>(EIntVectorOp(inOp), in.mU, out.mU, out.mU[3], in.mU[2][3]); break;
		case 1:		IntVectorOp<uint4>(EIntVectorOp(inOp), in.mU, out.mU, out.mU[3], in.mU[2][3]); break;
		case 2:		IntVectorOp<int3>(EIntVectorOp(inOp), in.mI, out.mI, out.mU[3], in.mU[2][3]); break;
		default:	IntVectorOp<int4>(EIntVectorOp(inOp), in.mI, out.mI, out.mU[3], in.mU[2][3]); break;
		}
	}
}

/// Run one of the other operations for inCount inputs
void jolt_hlsl_misc_op(uint32 inOp, const HLSLOpInput *inIn, HLSLOpOutput *ioOut, uint32 inCount)
{
	for (uint32 i = 0; i < inCount; ++i)
		MiscOp(EMiscOp(inOp), inIn[i], ioOut[i]);
}

//////////////////////////////////////////////////////////////////////////////////////////
// The shader helpers (ShaderMath.h, ShaderMat44.h, ShaderQuat.h, ShaderPlane.h, ShaderVec3.h, HairCommon.h)
//////////////////////////////////////////////////////////////////////////////////////////

void jolt_shader_square(const float *inValues, float *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_Square(inValues[i]);
}

void jolt_shader_get_closest_point_on_line(const HLSLToCPP::float3 *inA, const HLSLToCPP::float3 *inAB, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_GetClosestPointOnLine(inA[i], inAB[i]);
}

void jolt_shader_mat44_mul3x4_vec3(const HLSLToCPP::Mat44 *inM, const HLSLToCPP::float3 *inV, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_Mat44Mul3x4Vec3(inM[i], inV[i]);
}

void jolt_shader_mat44_mul3x3_vec3(const HLSLToCPP::Mat44 *inM, const HLSLToCPP::float3 *inV, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_Mat44Mul3x3Vec3(inM[i], inV[i]);
}

void jolt_shader_quat_mul_vec3(const HLSLToCPP::float4 *inQ, const HLSLToCPP::float3 *inV, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatMulVec3(inQ[i], inV[i]);
}

void jolt_shader_quat_mul_quat(const HLSLToCPP::float4 *inA, const HLSLToCPP::float4 *inB, HLSLToCPP::float4 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatMulQuat(inA[i], inB[i]);
}

void jolt_shader_quat_imaginary_mul_quat(const HLSLToCPP::float3 *inA, const HLSLToCPP::float4 *inB, HLSLToCPP::float4 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatImaginaryMulQuat(inA[i], inB[i]);
}

void jolt_shader_quat_rotate_axis_z(const HLSLToCPP::float4 *inQ, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatRotateAxisZ(inQ[i]);
}

void jolt_shader_quat_conjugate(const HLSLToCPP::float4 *inQ, HLSLToCPP::float4 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatConjugate(inQ[i]);
}

void jolt_shader_quat_decompress(const uint32 *inValues, HLSLToCPP::float4 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatDecompress(inValues[i]);
}

void jolt_shader_quat_from_mat33(const HLSLToCPP::float3 *inCol0, const HLSLToCPP::float3 *inCol1, const HLSLToCPP::float3 *inCol2, HLSLToCPP::float4 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_QuatFromMat33(inCol0[i], inCol1[i], inCol2[i]);
}

void jolt_shader_plane_from_point_and_normal(const HLSLToCPP::float3 *inPoint, const HLSLToCPP::float3 *inNormal, HLSLToCPP::float4 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_PlaneFromPointAndNormal(inPoint[i], inNormal[i]);
}

void jolt_shader_plane_get_normal(const HLSLToCPP::float4 *inPlane, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_PlaneGetNormal(inPlane[i]);
}

void jolt_shader_plane_signed_distance(const HLSLToCPP::float4 *inPlane, const HLSLToCPP::float3 *inPoint, float *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_PlaneSignedDistance(inPlane[i], inPoint[i]);
}

void jolt_shader_vec3_decompress_unit(const uint32 *inValues, HLSLToCPP::float3 *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.JPH_Vec3DecompressUnit(inValues[i]);
}

/// The packed bit and byte readers of HairCommon.h on one buffer: IsVertexFixed, GetVertexInvMass,
/// GetVertexStrandFraction, GetStrandVertexCount and GetStrandMaterialIndex of index inIndices[i]
void jolt_shader_hair_packed(const uint32 *inWords, const uint32 *inIndices, uint32 *outFixed, float *outInvMass, float *outFraction, uint32 *outVertexCount, uint32 *outMaterialIndex, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
	{
		outFixed[i] = h.IsVertexFixed(inWords, inIndices[i])? 1 : 0;
		outInvMass[i] = h.GetVertexInvMass(inWords, inIndices[i]);
		outFraction[i] = h.GetVertexStrandFraction(inWords, inIndices[i]);
		outVertexCount[i] = h.GetStrandVertexCount(inWords, inIndices[i]);
		outMaterialIndex[i] = h.GetStrandMaterialIndex(inWords, inIndices[i]);
	}
}

void jolt_shader_gradient_sampler_sample(const HLSLToCPP::float4 *inSampler, const float *inFraction, float *outValues, uint32 inCount)
{
	Helpers h;
	for (uint32 i = 0; i < inCount; ++i)
		outValues[i] = h.GradientSamplerSample(inSampler[i], inFraction[i]);
}

/// GridPositionToIndexAndFraction of inPositions[i] and GridIndexToBufferIndex of the index it returns and of
/// inIndices[i], with the constants of the JPH_HairUpdateContext inContext (256 bytes) bound as gContext
void jolt_shader_grid(const void *inContext, const HLSLToCPP::float3 *inPositions, const HLSLToCPP::uint3 *inIndices, HLSLToCPP::uint3 *outIndex, HLSLToCPP::float3 *outFraction, uint32 *outBufferIndex, uint32 *outBufferIndex2, uint32 inCount)
{
	Helpers h;
	alignas(16) uint8 context[256];
	memcpy(context, inContext, sizeof(context));
	h.Bind("gContext", context, sizeof(context));

	for (uint32 i = 0; i < inCount; ++i)
	{
		HLSLToCPP::uint3 index;
		HLSLToCPP::float3 fraction;
		h.GridPositionToIndexAndFraction(inPositions[i], index, fraction);
		outIndex[i] = index;
		outFraction[i] = fraction;
		outBufferIndex[i] = h.GridIndexToBufferIndex(index);
		outBufferIndex2[i] = h.GridIndexToBufferIndex(inIndices[i]);
	}
}

/// The constants of HairCommon.h: cFloatToFixed, cFixedToFloat
void jolt_shader_hair_common_constants(int *outFloatToFixed, float *outFixedToFloat)
{
	*outFloatToFixed = Helpers::cFloatToFixed;
	*outFixedToFloat = Helpers::cFixedToFloat;
}

} // extern "C"
