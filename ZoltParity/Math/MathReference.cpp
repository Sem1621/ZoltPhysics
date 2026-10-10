// Reference implementation for the Jolt/Math parity tests (`zig build parity`): thin C ABI wrappers around the
// C++ Jolt library. ZoltParity/Math/MathParity.zig calls these and checks that Zolt produces the same bits.
//
// Conventions: vectors are passed as float arrays (3 or 4 components), quaternions as 4 floats (x, y, z, w),
// matrices as 16 floats in column major order. DVec3 is passed as 3 doubles, DMat44 as 12 floats (the 3 rotation
// columns, 4 floats each) plus 3 doubles (the translation). Add a wrapper here for every Jolt function that gets a parity test.

#include <Jolt/Jolt.h>
#include <Jolt/Math/Trigonometry.h>
#include <Jolt/Math/HalfFloat.h>

using namespace JPH;

static Vec4 Load4(const float *inV)					{ return Vec4(inV[0], inV[1], inV[2], inV[3]); }
static Vec3 Load3(const float *inV)					{ return Vec3(inV[0], inV[1], inV[2]); }
static void Store4(Vec4Arg inV, float *outV)		{ inV.StoreFloat4(reinterpret_cast<Float4 *>(outV)); }
static void Store3(Vec3Arg inV, float *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static Quat LoadQuat(const float *inQ)				{ return Quat(inQ[0], inQ[1], inQ[2], inQ[3]); }
static void StoreQuat(QuatArg inQ, float *outQ)		{ Store4(inQ.GetXYZW(), outQ); }
static Mat44 LoadMat44(const float *inM)			{ return Mat44(Load4(inM), Load4(inM + 4), Load4(inM + 8), Load4(inM + 12)); }
static void StoreMat44(Mat44Arg inM, float *outM)	{ for (uint c = 0; c < 4; ++c) Store4(inM.GetColumn4(c), outM + 4 * c); }
static DVec3 LoadD3(const double *inV)				{ return DVec3(inV[0], inV[1], inV[2]); }
static void StoreD3(DVec3Arg inV, double *outV)		{ outV[0] = inV.GetX(); outV[1] = inV.GetY(); outV[2] = inV.GetZ(); }
static DMat44 LoadDMat44(const float *inCols, const double *inT) { return DMat44(Load4(inCols), Load4(inCols + 4), Load4(inCols + 8), LoadD3(inT)); }
static void StoreDMat44(DMat44Arg inM, float *outCols, double *outT)
{
	for (uint c = 0; c < 3; ++c)
		Store4(inM.GetColumn4(c), outCols + 4 * c);
	StoreD3(inM.GetTranslation(), outT);
}

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
void jolt_vec4_abs(const float *inV, float *outV)	{ Store4(Load4(inV).Abs(), outV); }
void jolt_vec3_abs(const float *inV, float *outV)	{ Store3(Load3(inV).Abs(), outV); }
uint32 jolt_vec4_compress_unit_vector(const float *inV) { return Load4(inV).CompressUnitVector(); }
void jolt_vec4_decompress_unit_vector(uint32 inValue, float *outV) { Store4(Vec4::sDecompressUnitVector(inValue), outV); }
void jolt_vec4_to_int(const float *inV, uint32 *outV) { Load4(inV).ToInt().StoreInt4(outV); }
void jolt_uvec4_to_float(const uint32 *inV, float *outV) { Store4(UVec4::sLoadInt4(inV).ToFloat(), outV); }

// Math/Vec3.h
void jolt_vec3_normalized(const float *inV, float *outV) { Store3(Load3(inV).Normalized(), outV); }
void jolt_vec3_cross(const float *inA, const float *inB, float *outV) { Store3(Load3(inA).Cross(Load3(inB)), outV); }
float jolt_vec3_dot(const float *inA, const float *inB) { return Load3(inA).Dot(Load3(inB)); }
float jolt_vec3_length(const float *inV)			{ return Load3(inV).Length(); }
void jolt_vec3_normalized_perpendicular(const float *inV, float *outV) { Store3(Load3(inV).GetNormalizedPerpendicular(), outV); }
void jolt_vec3_unit_spherical(float inTheta, float inPhi, float *outV) { Store3(Vec3::sUnitSpherical(inTheta, inPhi), outV); }
uint32 jolt_vec3_compress_unit_vector(const float *inV) { return Load3(inV).CompressUnitVector(); }
void jolt_vec3_decompress_unit_vector(uint32 inValue, float *outV) { Store3(Vec3::sDecompressUnitVector(inValue), outV); }
void jolt_vec3_to_int(const float *inV, uint32 *outV) { UVec4 v = Load3(inV).ToInt(); outV[0] = v.GetX(); outV[1] = v.GetY(); outV[2] = v.GetZ(); }

// Vec3::sUnitSphere (1026 vertices, 3 floats each), returns the number of vertices
uint32 jolt_vec3_unit_sphere(float *outVertices)
{
	for (size_t i = 0; i < Vec3::sUnitSphere.size(); ++i)
		Store3(Vec3::sUnitSphere[i], outVertices + 3 * i);
	return (uint32)Vec3::sUnitSphere.size();
}

// Math/Quat.h
void jolt_quat_rotation(const float *inAxis, float inAngle, float *outQ) { StoreQuat(Quat::sRotation(Load3(inAxis), inAngle), outQ); }
void jolt_quat_mul(const float *inA, const float *inB, float *outQ) { StoreQuat(LoadQuat(inA) * LoadQuat(inB), outQ); }
void jolt_quat_mul_vec3(const float *inQ, const float *inV, float *outV) { Store3(LoadQuat(inQ) * Load3(inV), outV); }
void jolt_quat_inverse_rotate(const float *inQ, const float *inV, float *outV) { Store3(LoadQuat(inQ).InverseRotate(Load3(inV)), outV); }
void jolt_quat_multiply_imaginary(const float *inV, const float *inQ, float *outQ) { StoreQuat(Quat::sMultiplyImaginary(Load3(inV), LoadQuat(inQ)), outQ); }
void jolt_quat_rotate_axis_x(const float *inQ, float *outV) { Store3(LoadQuat(inQ).RotateAxisX(), outV); }
void jolt_quat_rotate_axis_y(const float *inQ, float *outV) { Store3(LoadQuat(inQ).RotateAxisY(), outV); }
void jolt_quat_rotate_axis_z(const float *inQ, float *outV) { Store3(LoadQuat(inQ).RotateAxisZ(), outV); }
void jolt_quat_inversed(const float *inQ, float *outQ) { StoreQuat(LoadQuat(inQ).Inversed(), outQ); }
void jolt_quat_conjugated(const float *inQ, float *outQ) { StoreQuat(LoadQuat(inQ).Conjugated(), outQ); }
void jolt_quat_normalized(const float *inQ, float *outQ) { StoreQuat(LoadQuat(inQ).Normalized(), outQ); }
void jolt_quat_ensure_w_positive(const float *inQ, float *outQ) { StoreQuat(LoadQuat(inQ).EnsureWPositive(), outQ); }
void jolt_quat_get_axis_angle(const float *inQ, float *outAxis, float *outAngle)
{
	Vec3 axis;
	LoadQuat(inQ).GetAxisAngle(axis, *outAngle);
	Store3(axis, outAxis);
}
void jolt_quat_get_angular_velocity(const float *inQ, float inDeltaTime, float *outV) { Store3(LoadQuat(inQ).GetAngularVelocity(inDeltaTime), outV); }
void jolt_quat_get_swing_twist(const float *inQ, float *outSwing, float *outTwist)
{
	Quat swing, twist;
	LoadQuat(inQ).GetSwingTwist(swing, twist);
	StoreQuat(swing, outSwing);
	StoreQuat(twist, outTwist);
}
void jolt_quat_get_twist(const float *inQ, const float *inAxis, float *outQ) { StoreQuat(LoadQuat(inQ).GetTwist(Load3(inAxis)), outQ); }
float jolt_quat_get_rotation_angle(const float *inQ, const float *inAxis) { return LoadQuat(inQ).GetRotationAngle(Load3(inAxis)); }
void jolt_quat_get_euler_angles(const float *inQ, float *outV) { Store3(LoadQuat(inQ).GetEulerAngles(), outV); }
void jolt_quat_euler_angles(const float *inAngles, float *outQ) { StoreQuat(Quat::sEulerAngles(Load3(inAngles)), outQ); }
void jolt_quat_from_to(const float *inFrom, const float *inTo, float *outQ) { StoreQuat(Quat::sFromTo(Load3(inFrom), Load3(inTo)), outQ); }
void jolt_quat_lerp(const float *inA, const float *inB, float inFraction, float *outQ) { StoreQuat(LoadQuat(inA).LERP(LoadQuat(inB), inFraction), outQ); }
void jolt_quat_slerp(const float *inA, const float *inB, float inFraction, float *outQ) { StoreQuat(LoadQuat(inA).SLERP(LoadQuat(inB), inFraction), outQ); }

// Math/Mat44.h
void jolt_mat44_rotation_x(float inAngle, float *outM)	{ StoreMat44(Mat44::sRotationX(inAngle), outM); }
void jolt_mat44_rotation_y(float inAngle, float *outM)	{ StoreMat44(Mat44::sRotationY(inAngle), outM); }
void jolt_mat44_rotation_z(float inAngle, float *outM)	{ StoreMat44(Mat44::sRotationZ(inAngle), outM); }
void jolt_mat44_rotation(const float *inAxis, float inAngle, float *outM) { StoreMat44(Mat44::sRotation(Load3(inAxis), inAngle), outM); }
void jolt_mat44_rotation_quat(const float *inQ, float *outM) { StoreMat44(Mat44::sRotation(LoadQuat(inQ)), outM); }
void jolt_mat44_inverse_rotation_translation(const float *inQ, const float *inT, float *outM) { StoreMat44(Mat44::sInverseRotationTranslation(LoadQuat(inQ), Load3(inT)), outM); }
void jolt_mat44_outer_product(const float *inA, const float *inB, float *outM) { StoreMat44(Mat44::sOuterProduct(Load3(inA), Load3(inB)), outM); }
void jolt_mat44_cross_product(const float *inV, float *outM) { StoreMat44(Mat44::sCrossProduct(Load3(inV)), outM); }
void jolt_mat44_quat_left_multiply(const float *inQ, float *outM) { StoreMat44(Mat44::sQuatLeftMultiply(LoadQuat(inQ)), outM); }
void jolt_mat44_quat_right_multiply(const float *inQ, float *outM) { StoreMat44(Mat44::sQuatRightMultiply(LoadQuat(inQ)), outM); }
void jolt_mat44_look_at(const float *inPos, const float *inTarget, const float *inUp, float *outM) { StoreMat44(Mat44::sLookAt(Load3(inPos), Load3(inTarget), Load3(inUp)), outM); }
void jolt_mat44_perspective(float inFovY, float inAspect, float inNear, float inFar, float *outM) { StoreMat44(Mat44::sPerspective(inFovY, inAspect, inNear, inFar), outM); }
void jolt_mat44_mul(const float *inA, const float *inB, float *outM) { StoreMat44(LoadMat44(inA) * LoadMat44(inB), outM); }
void jolt_mat44_mul_vec3(const float *inM, const float *inV, float *outV) { Store3(LoadMat44(inM) * Load3(inV), outV); }
void jolt_mat44_mul_vec4(const float *inM, const float *inV, float *outV) { Store4(LoadMat44(inM) * Load4(inV), outV); }
void jolt_mat44_multiply3x3(const float *inM, const float *inV, float *outV) { Store3(LoadMat44(inM).Multiply3x3(Load3(inV)), outV); }
void jolt_mat44_multiply3x3_transposed(const float *inM, const float *inV, float *outV) { Store3(LoadMat44(inM).Multiply3x3Transposed(Load3(inV)), outV); }
void jolt_mat44_multiply3x3_mat44(const float *inA, const float *inB, float *outM) { StoreMat44(LoadMat44(inA).Multiply3x3(LoadMat44(inB)), outM); }
void jolt_mat44_multiply3x3_left_transposed(const float *inA, const float *inB, float *outM) { StoreMat44(LoadMat44(inA).Multiply3x3LeftTransposed(LoadMat44(inB)), outM); }
void jolt_mat44_multiply3x3_right_transposed(const float *inA, const float *inB, float *outM) { StoreMat44(LoadMat44(inA).Multiply3x3RightTransposed(LoadMat44(inB)), outM); }
void jolt_mat44_mul_scalar(const float *inM, float inV, float *outM) { StoreMat44(LoadMat44(inM) * inV, outM); }
void jolt_mat44_add(const float *inA, const float *inB, float *outM) { StoreMat44(LoadMat44(inA) + LoadMat44(inB), outM); }
void jolt_mat44_sub(const float *inA, const float *inB, float *outM) { StoreMat44(LoadMat44(inA) - LoadMat44(inB), outM); }
void jolt_mat44_negate(const float *inM, float *outM) { StoreMat44(-LoadMat44(inM), outM); }
void jolt_mat44_transposed(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).Transposed(), outM); }
void jolt_mat44_transposed3x3(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).Transposed3x3(), outM); }
void jolt_mat44_inversed(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).Inversed(), outM); }
void jolt_mat44_inversed_rotation_translation(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).InversedRotationTranslation(), outM); }
float jolt_mat44_get_determinant3x3(const float *inM) { return LoadMat44(inM).GetDeterminant3x3(); }
void jolt_mat44_adjointed3x3(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).Adjointed3x3(), outM); }
void jolt_mat44_inversed3x3(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).Inversed3x3(), outM); }
bool jolt_mat44_set_inversed3x3(const float *inM, float *ioM)
{
	Mat44 m = LoadMat44(ioM);
	bool result = m.SetInversed3x3(LoadMat44(inM));
	StoreMat44(m, ioM);
	return result;
}
void jolt_mat44_get_quaternion(const float *inM, float *outQ) { StoreQuat(LoadMat44(inM).GetQuaternion(), outQ); }
void jolt_mat44_get_direction_preserving_matrix(const float *inM, float *outM) { StoreMat44(LoadMat44(inM).GetDirectionPreservingMatrix(), outM); }
void jolt_mat44_pre_translated(const float *inM, const float *inT, float *outM) { StoreMat44(LoadMat44(inM).PreTranslated(Load3(inT)), outM); }
void jolt_mat44_post_translated(const float *inM, const float *inT, float *outM) { StoreMat44(LoadMat44(inM).PostTranslated(Load3(inT)), outM); }
void jolt_mat44_pre_scaled(const float *inM, const float *inS, float *outM) { StoreMat44(LoadMat44(inM).PreScaled(Load3(inS)), outM); }
void jolt_mat44_post_scaled(const float *inM, const float *inS, float *outM) { StoreMat44(LoadMat44(inM).PostScaled(Load3(inS)), outM); }
void jolt_mat44_decompose(const float *inM, float *outM, float *outScale)
{
	Vec3 scale;
	StoreMat44(LoadMat44(inM).Decompose(scale), outM);
	Store3(scale, outScale);
}

// Math/DVec3.h
void jolt_dvec3_add(const double *inA, const double *inB, double *outV) { StoreD3(LoadD3(inA) + LoadD3(inB), outV); }
void jolt_dvec3_sub(const double *inA, const double *inB, double *outV) { StoreD3(LoadD3(inA) - LoadD3(inB), outV); }
void jolt_dvec3_mul(const double *inA, const double *inB, double *outV) { StoreD3(LoadD3(inA) * LoadD3(inB), outV); }
void jolt_dvec3_div(const double *inA, const double *inB, double *outV) { StoreD3(LoadD3(inA) / LoadD3(inB), outV); }
void jolt_dvec3_add_vec3(const double *inA, const float *inB, double *outV) { StoreD3(LoadD3(inA) + Load3(inB), outV); }
void jolt_dvec3_sub_vec3(const double *inA, const float *inB, double *outV) { StoreD3(LoadD3(inA) - Load3(inB), outV); }
void jolt_dvec3_mul_scalar(const double *inA, double inB, double *outV) { StoreD3(LoadD3(inA) * inB, outV); }
void jolt_dvec3_div_scalar(const double *inA, double inB, double *outV) { StoreD3(LoadD3(inA) / inB, outV); }
void jolt_dvec3_negate(const double *inA, double *outV) { StoreD3(-LoadD3(inA), outV); }
void jolt_dvec3_abs(const double *inA, double *outV) { StoreD3(LoadD3(inA).Abs(), outV); }
void jolt_dvec3_reciprocal(const double *inA, double *outV) { StoreD3(LoadD3(inA).Reciprocal(), outV); }
void jolt_dvec3_sqrt(const double *inA, double *outV) { StoreD3(LoadD3(inA).Sqrt(), outV); }
void jolt_dvec3_get_sign(const double *inA, double *outV) { StoreD3(LoadD3(inA).GetSign(), outV); }
void jolt_dvec3_min(const double *inA, const double *inB, double *outV) { StoreD3(DVec3::sMin(LoadD3(inA), LoadD3(inB)), outV); }
void jolt_dvec3_max(const double *inA, const double *inB, double *outV) { StoreD3(DVec3::sMax(LoadD3(inA), LoadD3(inB)), outV); }
void jolt_dvec3_cross(const double *inA, const double *inB, double *outV) { StoreD3(LoadD3(inA).Cross(LoadD3(inB)), outV); }
double jolt_dvec3_dot(const double *inA, const double *inB) { return LoadD3(inA).Dot(LoadD3(inB)); }
double jolt_dvec3_length_sq(const double *inA) { return LoadD3(inA).LengthSq(); }
double jolt_dvec3_length(const double *inA) { return LoadD3(inA).Length(); }
void jolt_dvec3_normalized(const double *inA, double *outV) { StoreD3(LoadD3(inA).Normalized(), outV); }
void jolt_dvec3_to_vec3(const double *inA, float *outV) { Store3(Vec3(LoadD3(inA)), outV); }
void jolt_dvec3_prepare_round_to_zero(const double *inA, double *outV) { StoreD3(LoadD3(inA).PrepareRoundToZero(), outV); }
void jolt_dvec3_prepare_round_to_inf(const double *inA, double *outV) { StoreD3(LoadD3(inA).PrepareRoundToInf(), outV); }
void jolt_dvec3_to_vec3_round_down(const double *inA, float *outV) { Store3(LoadD3(inA).ToVec3RoundDown(), outV); }
void jolt_dvec3_to_vec3_round_up(const double *inA, float *outV) { Store3(LoadD3(inA).ToVec3RoundUp(), outV); }

// Math/DMat44.h
void jolt_dmat44_mul(const float *inACols, const double *inAT, const float *inBCols, const double *inBT, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inACols, inAT) * LoadDMat44(inBCols, inBT), outCols, outT); }
void jolt_dmat44_mul_mat44(const float *inACols, const double *inAT, const float *inB, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inACols, inAT) * LoadMat44(inB), outCols, outT); }
void jolt_dmat44_mul_vec3(const float *inCols, const double *inT, const float *inV, double *outV) { StoreD3(LoadDMat44(inCols, inT) * Load3(inV), outV); }
void jolt_dmat44_mul_dvec3(const float *inCols, const double *inT, const double *inV, double *outV) { StoreD3(LoadDMat44(inCols, inT) * LoadD3(inV), outV); }
void jolt_dmat44_multiply3x3(const float *inCols, const double *inT, const float *inV, float *outV) { Store3(LoadDMat44(inCols, inT).Multiply3x3(Load3(inV)), outV); }
void jolt_dmat44_multiply3x3_dvec3(const float *inCols, const double *inT, const double *inV, double *outV) { StoreD3(LoadDMat44(inCols, inT).Multiply3x3(LoadD3(inV)), outV); }
void jolt_dmat44_multiply3x3_transposed(const float *inCols, const double *inT, const float *inV, float *outV) { Store3(LoadDMat44(inCols, inT).Multiply3x3Transposed(Load3(inV)), outV); }
void jolt_dmat44_inversed(const float *inCols, const double *inT, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).Inversed(), outCols, outT); }
void jolt_dmat44_inversed_rotation_translation(const float *inCols, const double *inT, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).InversedRotationTranslation(), outCols, outT); }
void jolt_dmat44_rotation_translation(const float *inQ, const double *inT, float *outCols, double *outT) { StoreDMat44(DMat44::sRotationTranslation(LoadQuat(inQ), LoadD3(inT)), outCols, outT); }
void jolt_dmat44_inverse_rotation_translation(const float *inQ, const double *inT, float *outCols, double *outT) { StoreDMat44(DMat44::sInverseRotationTranslation(LoadQuat(inQ), LoadD3(inT)), outCols, outT); }
void jolt_dmat44_pre_translated(const float *inCols, const double *inT, const float *inV, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).PreTranslated(Load3(inV)), outCols, outT); }
void jolt_dmat44_pre_translated_dvec3(const float *inCols, const double *inT, const double *inV, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).PreTranslated(LoadD3(inV)), outCols, outT); }
void jolt_dmat44_post_translated(const float *inCols, const double *inT, const float *inV, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).PostTranslated(Load3(inV)), outCols, outT); }
void jolt_dmat44_post_translated_dvec3(const float *inCols, const double *inT, const double *inV, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).PostTranslated(LoadD3(inV)), outCols, outT); }
void jolt_dmat44_pre_scaled(const float *inCols, const double *inT, const float *inS, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).PreScaled(Load3(inS)), outCols, outT); }
void jolt_dmat44_post_scaled(const float *inCols, const double *inT, const float *inS, float *outCols, double *outT) { StoreDMat44(LoadDMat44(inCols, inT).PostScaled(Load3(inS)), outCols, outT); }
void jolt_dmat44_decompose(const float *inCols, const double *inT, float *outCols, double *outT, float *outScale)
{
	Vec3 scale;
	StoreDMat44(LoadDMat44(inCols, inT).Decompose(scale), outCols, outT);
	Store3(scale, outScale);
}
void jolt_dmat44_to_mat44(const float *inCols, const double *inT, float *outM) { StoreMat44(LoadDMat44(inCols, inT).ToMat44(), outM); }

// Math/HalfFloat.h. FromFloat / ToFloat use the F16C instructions when the reference is compiled for a CPU that
// has them (Zolt always uses the fallback), so both the fast path and the fallback are compared.
uint16 jolt_half_from_float(float inV, int inRoundingMode)
{
	switch (inRoundingMode)
	{
	case HalfFloatConversion::ROUND_TO_NEG_INF:	return HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEG_INF>(inV);
	case HalfFloatConversion::ROUND_TO_POS_INF:	return HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_POS_INF>(inV);
	default:									return HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEAREST>(inV);
	}
}
uint16 jolt_half_from_float_fallback(float inV, int inRoundingMode)
{
	switch (inRoundingMode)
	{
	case HalfFloatConversion::ROUND_TO_NEG_INF:	return HalfFloatConversion::FromFloatFallback<HalfFloatConversion::ROUND_TO_NEG_INF>(inV);
	case HalfFloatConversion::ROUND_TO_POS_INF:	return HalfFloatConversion::FromFloatFallback<HalfFloatConversion::ROUND_TO_POS_INF>(inV);
	default:									return HalfFloatConversion::FromFloatFallback<HalfFloatConversion::ROUND_TO_NEAREST>(inV);
	}
}
void jolt_half_to_float(const uint16 *inV, float *outV)
{
	UVec4 v(uint32(inV[0]) | (uint32(inV[1]) << 16), uint32(inV[2]) | (uint32(inV[3]) << 16), 0, 0);
	Store4(HalfFloatConversion::ToFloat(v), outV);
}
void jolt_half_to_float_fallback(const uint16 *inV, float *outV)
{
	UVec4 v(uint32(inV[0]) | (uint32(inV[1]) << 16), uint32(inV[2]) | (uint32(inV[3]) << 16), 0, 0);
	Store4(HalfFloatConversion::ToFloatFallback(v), outV);
}

} // extern "C"
