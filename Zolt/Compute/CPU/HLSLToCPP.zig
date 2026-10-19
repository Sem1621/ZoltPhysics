//! Port of: Jolt/Compute/CPU/HLSLToCPP.h
//! Status: complete
//!
//! Emulates HLSL vector types and operations, like Jolt's HLSLToCPP (Docs/Zolt/RemainingFeaturesArchitecture.md R11).
//! Jolt compiles its HLSL compute shaders as C++ with these types for the CPU compute backend. The Zig shaders of
//! Zolt/Shaders use this file (imported as `hlsl`), so they give the bits of Jolt's CPU backend.
//! - The types keep HLSL's names: `float2`, `float3`, `float4`, `uint3`, `uint4`, `int3`, `int4`; `Quat` and `Plane` are
//!   `float4`; `Mat44` is 4 `float4` columns. A shader port is reviewed line by line against the HLSL text.
//! - They are extern structs of scalars, not `@Vector`. Buffer layouts equal ShaderCore.h's (`float3` is 12 bytes).
//! - One Zig function per C++ operator, in its operation order. Operators are methods (porting guide section 4):
//!   `a + b` is `a.add(b)`, `a * s` and `s * a` are `a.mulScalar(s)` (HLSLToCPP defines `s * a` as `a * s`), `-a` is
//!   `a.negate()`, `a += b` is `a = a.add(b)`, `a == b` is `a.eql(b)`, `a[i]` is `a.getComponent(i)` /
//!   `a.setComponent(i, v)`. HLSL intrinsics are functions: `hlsl.dot(a, b)`, `hlsl.min(a, b)`, `hlsl.clamp(v, lo, hi)`.
//! - The swizzle macros (`#define xyz swizzle_xyz()`) are methods: `v.xyz()`.
//! - The constructors are `init` (all components), `splat` (`explicit float3(float inS)`) and `fromX` for the others
//!   (`float3(const float2 &, float)` is `fromFloat2`, `explicit float3(const uint3 &)` is `fromUint3`).
//!
//! Semantics that differ from Zolt's Vec3 (each pinned by a test at the end of this file):
//! - `dot` is left to right: `a.x * b.x + a.y * b.y + a.z * b.z` (Vec3 sums `(x + y) + (z + 0)`);
//! - `normalize(v)` divides each component by `length(v)` (no reciprocal); `length` is `@sqrt(dot(v, v))`;
//! - `min` / `max` are `std::min` / `std::max` per component (`math.min` / `math.max`), not Vec3's `minps` semantics;
//! - `clamp(v, lo, hi)` is `min(max(v, lo), hi)`;
//! - unary minus flips the sign bit per component (Vec3's negate is `0 - v`);
//! - `round` is `@round` (half away from zero, like `std::round`); `asint` is `@bitCast`;
//! - `uint(f)` is the 64-bit truncation of `cvttss2si` cut to 32 bits; `int(f)` is the 32-bit `cvttss2si` (porting guide
//!   section 8, rule 12): `toUint` and `toInt`. `float(uint)` and `float(int)` are correctly rounded (`@floatFromInt`);
//! - `uint3(float3)` is not three scalar `uint(f)`: the reference build converts x and y as a vector pair, which gives
//!   0x80000000 for NaN and for values of 2^32 and above (`toUintVectorized`), and z with the scalar conversion
//!   (`toUint`). The parity found it (HLSLToCPPParity and the grid helper of HairCommon); R11 of the design only lists
//!   the scalar conversion. The other conversions are the same for scalars and vectors;
//! - integer arithmetic wraps (`+%`, `*%`), like C++ unsigned arithmetic. Signed overflow is undefined in C++: Zolt
//!   wraps too;
//! - `JPH_AtomicAdd` on `int` is `@atomicRmw(.Add, .seq_cst)` plus the operand (`atomicAdd`).
//!
//! The binding types of WrapShaderBegin.h are here too, because the shaders declare their bindings with them (R12):
//! `Constants(T)` is JPH_SHADER_CONSTANTS_BEGIN, `Buffer(T)` is JPH_SHADER_BIND_BUFFER, `RWBuffer(T)` is
//! JPH_SHADER_BIND_RW_BUFFER. Compute/CPU/ShaderWrapper.zig binds them by name.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");

//////////////////////////////////////////////////////////////////////////////////////////
// float2
//////////////////////////////////////////////////////////////////////////////////////////

pub const float2 = extern struct {
    x: f32,
    y: f32,

    // Constructors

    /// float2(float inX, float inY)
    pub fn init(x: f32, y: f32) float2 {
        return .{ .x = x, .y = y };
    }

    /// explicit float2(float inS)
    pub fn splat(s: f32) float2 {
        return .{ .x = s, .y = s };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`)

    /// operator + (const float2 &inA, const float2 &inB)
    pub fn add(a: float2, b: float2) float2 {
        return .{ .x = a.x + b.x, .y = a.y + b.y };
    }

    /// operator - (const float2 &inA, const float2 &inB)
    pub fn sub(a: float2, b: float2) float2 {
        return .{ .x = a.x - b.x, .y = a.y - b.y };
    }

    /// operator * (const float2 &inA, const float2 &inB)
    pub fn mul(a: float2, b: float2) float2 {
        return .{ .x = a.x * b.x, .y = a.y * b.y };
    }

    /// operator / (const float2 &inA, const float2 &inB)
    pub fn div(a: float2, b: float2) float2 {
        return .{ .x = a.x / b.x, .y = a.y / b.y };
    }

    /// operator * (const float2 &inA, float inS), also operator * (float inS, const float2 &inA), which is `inA * inS`
    pub fn mulScalar(a: float2, s: f32) float2 {
        return .{ .x = a.x * s, .y = a.y * s };
    }

    /// operator / (const float2 &inA, float inS)
    pub fn divScalar(a: float2, s: f32) float2 {
        return .{ .x = a.x / s, .y = a.y / s };
    }

    /// operator - (const float2 &inA): flips the sign of each component
    pub fn negate(a: float2) float2 {
        return .{ .x = -a.x, .y = -a.y };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: float2, b: float2) bool {
        return a.x == b.x and a.y == b.y;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: float2, index: u32) f32 {
        const components: [2]f32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *float2, index: u32, value: f32) void {
        const components: *[2]f32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xy(v: float2) float2 {
        return .{ .x = v.x, .y = v.y };
    }

    pub fn yx(v: float2) float2 {
        return .{ .x = v.y, .y = v.x };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// float3
//////////////////////////////////////////////////////////////////////////////////////////

pub const float3 = extern struct {
    x: f32,
    y: f32,
    z: f32,

    // Constructors

    /// float3(float inX, float inY, float inZ)
    pub fn init(x: f32, y: f32, z: f32) float3 {
        return .{ .x = x, .y = y, .z = z };
    }

    /// float3(const float2 &inV, float inZ)
    pub fn fromFloat2(v: float2, z: f32) float3 {
        return .{ .x = v.x, .y = v.y, .z = z };
    }

    /// explicit float3(float inS)
    pub fn splat(s: f32) float3 {
        return .{ .x = s, .y = s, .z = s };
    }

    /// explicit float3(const uint3 &inV): float(uint32) per component
    pub fn fromUint3(v: uint3) float3 {
        return .{ .x = @floatFromInt(v.x), .y = @floatFromInt(v.y), .z = @floatFromInt(v.z) };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`)

    /// operator + (const float3 &inA, const float3 &inB)
    pub fn add(a: float3, b: float3) float3 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    /// operator - (const float3 &inA, const float3 &inB)
    pub fn sub(a: float3, b: float3) float3 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    /// operator * (const float3 &inA, const float3 &inB)
    pub fn mul(a: float3, b: float3) float3 {
        return .{ .x = a.x * b.x, .y = a.y * b.y, .z = a.z * b.z };
    }

    /// operator / (const float3 &inA, const float3 &inB)
    pub fn div(a: float3, b: float3) float3 {
        return .{ .x = a.x / b.x, .y = a.y / b.y, .z = a.z / b.z };
    }

    /// operator * (const float3 &inA, float inS), also operator * (float inS, const float3 &inA), which is `inA * inS`
    pub fn mulScalar(a: float3, s: f32) float3 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
    }

    /// operator / (const float3 &inA, float inS)
    pub fn divScalar(a: float3, s: f32) float3 {
        return .{ .x = a.x / s, .y = a.y / s, .z = a.z / s };
    }

    /// operator - (const float3 &inA): flips the sign of each component
    pub fn negate(a: float3) float3 {
        return .{ .x = -a.x, .y = -a.y, .z = -a.z };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: float3, b: float3) bool {
        return a.x == b.x and a.y == b.y and a.z == b.z;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: float3, index: u32) f32 {
        const components: [3]f32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *float3, index: u32, value: f32) void {
        const components: *[3]f32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xy(v: float3) float2 {
        return .{ .x = v.x, .y = v.y };
    }

    pub fn yx(v: float3) float2 {
        return .{ .x = v.y, .y = v.x };
    }

    pub fn xyz(v: float3) float3 {
        return .{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn xzy(v: float3) float3 {
        return .{ .x = v.x, .y = v.z, .z = v.y };
    }

    pub fn yxz(v: float3) float3 {
        return .{ .x = v.y, .y = v.x, .z = v.z };
    }

    pub fn yzx(v: float3) float3 {
        return .{ .x = v.y, .y = v.z, .z = v.x };
    }

    pub fn zxy(v: float3) float3 {
        return .{ .x = v.z, .y = v.x, .z = v.y };
    }

    pub fn zyx(v: float3) float3 {
        return .{ .x = v.z, .y = v.y, .z = v.x };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// float4
//////////////////////////////////////////////////////////////////////////////////////////

pub const float4 = extern struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,

    // Constructors

    /// float4(float inX, float inY, float inZ, float inW)
    pub fn init(x: f32, y: f32, z: f32, w: f32) float4 {
        return .{ .x = x, .y = y, .z = z, .w = w };
    }

    /// float4(const float3 &inV, float inW)
    pub fn fromFloat3(v: float3, w: f32) float4 {
        return .{ .x = v.x, .y = v.y, .z = v.z, .w = w };
    }

    /// explicit float4(float inS)
    pub fn splat(s: f32) float4 {
        return .{ .x = s, .y = s, .z = s, .w = s };
    }

    /// explicit float4(const int4 &inV): float(int) per component
    pub fn fromInt4(v: int4) float4 {
        return .{ .x = @floatFromInt(v.x), .y = @floatFromInt(v.y), .z = @floatFromInt(v.z), .w = @floatFromInt(v.w) };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`)

    /// operator + (const float4 &inA, const float4 &inB)
    pub fn add(a: float4, b: float4) float4 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z, .w = a.w + b.w };
    }

    /// operator - (const float4 &inA, const float4 &inB)
    pub fn sub(a: float4, b: float4) float4 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z, .w = a.w - b.w };
    }

    /// operator * (const float4 &inA, const float4 &inB)
    pub fn mul(a: float4, b: float4) float4 {
        return .{ .x = a.x * b.x, .y = a.y * b.y, .z = a.z * b.z, .w = a.w * b.w };
    }

    /// operator / (const float4 &inA, const float4 &inB)
    pub fn div(a: float4, b: float4) float4 {
        return .{ .x = a.x / b.x, .y = a.y / b.y, .z = a.z / b.z, .w = a.w / b.w };
    }

    /// operator * (const float4 &inA, float inS), also operator * (float inS, const float4 &inA), which is `inA * inS`
    pub fn mulScalar(a: float4, s: f32) float4 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s, .w = a.w * s };
    }

    /// operator / (const float4 &inA, float inS)
    pub fn divScalar(a: float4, s: f32) float4 {
        return .{ .x = a.x / s, .y = a.y / s, .z = a.z / s, .w = a.w / s };
    }

    /// operator - (const float4 &inA): flips the sign of each component
    pub fn negate(a: float4) float4 {
        return .{ .x = -a.x, .y = -a.y, .z = -a.z, .w = -a.w };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: float4, b: float4) bool {
        return a.x == b.x and a.y == b.y and a.z == b.z and a.w == b.w;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: float4, index: u32) f32 {
        const components: [4]f32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *float4, index: u32, value: f32) void {
        const components: *[4]f32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xy(v: float4) float2 {
        return .{ .x = v.x, .y = v.y };
    }

    pub fn yx(v: float4) float2 {
        return .{ .x = v.y, .y = v.x };
    }

    pub fn xyz(v: float4) float3 {
        return .{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn xzy(v: float4) float3 {
        return .{ .x = v.x, .y = v.z, .z = v.y };
    }

    pub fn yxz(v: float4) float3 {
        return .{ .x = v.y, .y = v.x, .z = v.z };
    }

    pub fn yzx(v: float4) float3 {
        return .{ .x = v.y, .y = v.z, .z = v.x };
    }

    pub fn zxy(v: float4) float3 {
        return .{ .x = v.z, .y = v.x, .z = v.y };
    }

    pub fn zyx(v: float4) float3 {
        return .{ .x = v.z, .y = v.y, .z = v.x };
    }

    pub fn xywz(v: float4) float4 {
        return .{ .x = v.x, .y = v.y, .z = v.w, .w = v.z };
    }

    pub fn xwyz(v: float4) float4 {
        return .{ .x = v.x, .y = v.w, .z = v.y, .w = v.z };
    }

    pub fn wxyz(v: float4) float4 {
        return .{ .x = v.w, .y = v.x, .z = v.y, .w = v.z };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// uint3
//////////////////////////////////////////////////////////////////////////////////////////

pub const uint3 = extern struct {
    x: u32,
    y: u32,
    z: u32,

    // Constructors

    /// uint3(uint32 inX, uint32 inY, uint32 inZ)
    pub fn init(x: u32, y: u32, z: u32) uint3 {
        return .{ .x = x, .y = y, .z = z };
    }

    /// explicit uint3(const float3 &inV): uint32(float) per component. The reference build (clang, x86-64-v3)
    /// converts x and y together as a vector pair (`toUintVectorized`) and z alone (`toUint`). They differ for NaN, for
    /// values of 2^32 and above and for values of -2^31 and below; in range both give C++'s value.
    pub fn fromFloat3(v: float3) uint3 {
        return .{ .x = toUintVectorized(v.x), .y = toUintVectorized(v.y), .z = toUint(v.z) };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`). The arithmetic
    // wraps like C++ unsigned arithmetic. A division by zero is undefined in C++ and panics in safe builds.

    /// operator + (const uint3 &inA, const uint3 &inB)
    pub fn add(a: uint3, b: uint3) uint3 {
        return .{ .x = a.x +% b.x, .y = a.y +% b.y, .z = a.z +% b.z };
    }

    /// operator - (const uint3 &inA, const uint3 &inB)
    pub fn sub(a: uint3, b: uint3) uint3 {
        return .{ .x = a.x -% b.x, .y = a.y -% b.y, .z = a.z -% b.z };
    }

    /// operator * (const uint3 &inA, const uint3 &inB)
    pub fn mul(a: uint3, b: uint3) uint3 {
        return .{ .x = a.x *% b.x, .y = a.y *% b.y, .z = a.z *% b.z };
    }

    /// operator / (const uint3 &inA, const uint3 &inB)
    pub fn div(a: uint3, b: uint3) uint3 {
        return .{ .x = a.x / b.x, .y = a.y / b.y, .z = a.z / b.z };
    }

    /// operator * (const uint3 &inA, uint32 inS), also operator * (uint32 inS, const uint3 &inA), which is `inA * inS`
    pub fn mulScalar(a: uint3, s: u32) uint3 {
        return .{ .x = a.x *% s, .y = a.y *% s, .z = a.z *% s };
    }

    /// operator / (const uint3 &inA, uint32 inS)
    pub fn divScalar(a: uint3, s: u32) uint3 {
        return .{ .x = a.x / s, .y = a.y / s, .z = a.z / s };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: uint3, b: uint3) bool {
        return a.x == b.x and a.y == b.y and a.z == b.z;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: uint3, index: u32) u32 {
        const components: [3]u32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *uint3, index: u32, value: u32) void {
        const components: *[3]u32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xyz(v: uint3) uint3 {
        return .{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn xzy(v: uint3) uint3 {
        return .{ .x = v.x, .y = v.z, .z = v.y };
    }

    pub fn yxz(v: uint3) uint3 {
        return .{ .x = v.y, .y = v.x, .z = v.z };
    }

    pub fn yzx(v: uint3) uint3 {
        return .{ .x = v.y, .y = v.z, .z = v.x };
    }

    pub fn zxy(v: uint3) uint3 {
        return .{ .x = v.z, .y = v.x, .z = v.y };
    }

    pub fn zyx(v: uint3) uint3 {
        return .{ .x = v.z, .y = v.y, .z = v.x };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// uint4
//////////////////////////////////////////////////////////////////////////////////////////

pub const uint4 = extern struct {
    x: u32,
    y: u32,
    z: u32,
    w: u32,

    // Constructors

    /// uint4(uint32 inX, uint32 inY, uint32 inZ, uint32 inW)
    pub fn init(x: u32, y: u32, z: u32, w: u32) uint4 {
        return .{ .x = x, .y = y, .z = z, .w = w };
    }

    /// uint4(const uint3 &inV, uint32 inW)
    pub fn fromUint3(v: uint3, w: u32) uint4 {
        return .{ .x = v.x, .y = v.y, .z = v.z, .w = w };
    }

    /// explicit uint4(uint32 inS)
    pub fn splat(s: u32) uint4 {
        return .{ .x = s, .y = s, .z = s, .w = s };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`). The arithmetic
    // wraps like C++ unsigned arithmetic. A division by zero is undefined in C++ and panics in safe builds.

    /// operator + (const uint4 &inA, const uint4 &inB)
    pub fn add(a: uint4, b: uint4) uint4 {
        return .{ .x = a.x +% b.x, .y = a.y +% b.y, .z = a.z +% b.z, .w = a.w +% b.w };
    }

    /// operator - (const uint4 &inA, const uint4 &inB)
    pub fn sub(a: uint4, b: uint4) uint4 {
        return .{ .x = a.x -% b.x, .y = a.y -% b.y, .z = a.z -% b.z, .w = a.w -% b.w };
    }

    /// operator * (const uint4 &inA, const uint4 &inB)
    pub fn mul(a: uint4, b: uint4) uint4 {
        return .{ .x = a.x *% b.x, .y = a.y *% b.y, .z = a.z *% b.z, .w = a.w *% b.w };
    }

    /// operator / (const uint4 &inA, const uint4 &inB)
    pub fn div(a: uint4, b: uint4) uint4 {
        return .{ .x = a.x / b.x, .y = a.y / b.y, .z = a.z / b.z, .w = a.w / b.w };
    }

    /// operator * (const uint4 &inA, uint32 inS), also operator * (uint32 inS, const uint4 &inA), which is `inA * inS`
    pub fn mulScalar(a: uint4, s: u32) uint4 {
        return .{ .x = a.x *% s, .y = a.y *% s, .z = a.z *% s, .w = a.w *% s };
    }

    /// operator / (const uint4 &inA, uint32 inS)
    pub fn divScalar(a: uint4, s: u32) uint4 {
        return .{ .x = a.x / s, .y = a.y / s, .z = a.z / s, .w = a.w / s };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: uint4, b: uint4) bool {
        return a.x == b.x and a.y == b.y and a.z == b.z and a.w == b.w;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: uint4, index: u32) u32 {
        const components: [4]u32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *uint4, index: u32, value: u32) void {
        const components: *[4]u32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xyz(v: uint4) uint3 {
        return .{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn xzy(v: uint4) uint3 {
        return .{ .x = v.x, .y = v.z, .z = v.y };
    }

    pub fn yxz(v: uint4) uint3 {
        return .{ .x = v.y, .y = v.x, .z = v.z };
    }

    pub fn yzx(v: uint4) uint3 {
        return .{ .x = v.y, .y = v.z, .z = v.x };
    }

    pub fn zxy(v: uint4) uint3 {
        return .{ .x = v.z, .y = v.x, .z = v.y };
    }

    pub fn zyx(v: uint4) uint3 {
        return .{ .x = v.z, .y = v.y, .z = v.x };
    }

    pub fn xywz(v: uint4) uint4 {
        return .{ .x = v.x, .y = v.y, .z = v.w, .w = v.z };
    }

    pub fn xwyz(v: uint4) uint4 {
        return .{ .x = v.x, .y = v.w, .z = v.y, .w = v.z };
    }

    pub fn wxyz(v: uint4) uint4 {
        return .{ .x = v.w, .y = v.x, .z = v.y, .w = v.z };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// int3
//////////////////////////////////////////////////////////////////////////////////////////

pub const int3 = extern struct {
    x: i32,
    y: i32,
    z: i32,

    // Constructors

    /// int3(int inX, int inY, int inZ)
    pub fn init(x: i32, y: i32, z: i32) int3 {
        return .{ .x = x, .y = y, .z = z };
    }

    /// explicit int3(const float3 &inV): int(float) per component (`toInt`)
    pub fn fromFloat3(v: float3) int3 {
        return .{ .x = toInt(v.x), .y = toInt(v.y), .z = toInt(v.z) };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`). Signed overflow
    // is undefined in C++, Zolt wraps. A division by zero (or of the minimum by -1) panics in safe builds.

    /// operator + (const int3 &inA, const int3 &inB)
    pub fn add(a: int3, b: int3) int3 {
        return .{ .x = a.x +% b.x, .y = a.y +% b.y, .z = a.z +% b.z };
    }

    /// operator - (const int3 &inA, const int3 &inB)
    pub fn sub(a: int3, b: int3) int3 {
        return .{ .x = a.x -% b.x, .y = a.y -% b.y, .z = a.z -% b.z };
    }

    /// operator * (const int3 &inA, const int3 &inB)
    pub fn mul(a: int3, b: int3) int3 {
        return .{ .x = a.x *% b.x, .y = a.y *% b.y, .z = a.z *% b.z };
    }

    /// operator / (const int3 &inA, const int3 &inB): truncates toward zero like C++
    pub fn div(a: int3, b: int3) int3 {
        return .{ .x = @divTrunc(a.x, b.x), .y = @divTrunc(a.y, b.y), .z = @divTrunc(a.z, b.z) };
    }

    /// operator * (const int3 &inA, int inS), also operator * (int inS, const int3 &inA), which is `inA * inS`
    pub fn mulScalar(a: int3, s: i32) int3 {
        return .{ .x = a.x *% s, .y = a.y *% s, .z = a.z *% s };
    }

    /// operator / (const int3 &inA, int inS): truncates toward zero like C++
    pub fn divScalar(a: int3, s: i32) int3 {
        return .{ .x = @divTrunc(a.x, s), .y = @divTrunc(a.y, s), .z = @divTrunc(a.z, s) };
    }

    /// operator - (const int3 &inA)
    pub fn negate(a: int3) int3 {
        return .{ .x = -%a.x, .y = -%a.y, .z = -%a.z };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: int3, b: int3) bool {
        return a.x == b.x and a.y == b.y and a.z == b.z;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: int3, index: u32) i32 {
        const components: [3]i32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *int3, index: u32, value: i32) void {
        const components: *[3]i32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xyz(v: int3) int3 {
        return .{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn xzy(v: int3) int3 {
        return .{ .x = v.x, .y = v.z, .z = v.y };
    }

    pub fn yxz(v: int3) int3 {
        return .{ .x = v.y, .y = v.x, .z = v.z };
    }

    pub fn yzx(v: int3) int3 {
        return .{ .x = v.y, .y = v.z, .z = v.x };
    }

    pub fn zxy(v: int3) int3 {
        return .{ .x = v.z, .y = v.x, .z = v.y };
    }

    pub fn zyx(v: int3) int3 {
        return .{ .x = v.z, .y = v.y, .z = v.x };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// int4
//////////////////////////////////////////////////////////////////////////////////////////

pub const int4 = extern struct {
    x: i32,
    y: i32,
    z: i32,
    w: i32,

    // Constructors

    /// int4(int inX, int inY, int inZ, int inW)
    pub fn init(x: i32, y: i32, z: i32, w: i32) int4 {
        return .{ .x = x, .y = y, .z = z, .w = w };
    }

    /// int4(const int3 &inV, int inW)
    pub fn fromInt3(v: int3, w: i32) int4 {
        return .{ .x = v.x, .y = v.y, .z = v.z, .w = w };
    }

    /// explicit int4(int inS)
    pub fn splat(s: i32) int4 {
        return .{ .x = s, .y = s, .z = s, .w = s };
    }

    /// explicit int4(const float4 &inV): int(float) per component (`toInt`), the `(int4)` cast of the shaders
    pub fn fromFloat4(v: float4) int4 {
        return .{ .x = toInt(v.x), .y = toInt(v.y), .z = toInt(v.z), .w = toInt(v.w) };
    }

    // Operators (the compound operators `+=`, `-=`, `*=` and `/=` compute the same: `a = a.add(b)`). Signed overflow
    // is undefined in C++, Zolt wraps. A division by zero (or of the minimum by -1) panics in safe builds.

    /// operator + (const int4 &inA, const int4 &inB)
    pub fn add(a: int4, b: int4) int4 {
        return .{ .x = a.x +% b.x, .y = a.y +% b.y, .z = a.z +% b.z, .w = a.w +% b.w };
    }

    /// operator - (const int4 &inA, const int4 &inB)
    pub fn sub(a: int4, b: int4) int4 {
        return .{ .x = a.x -% b.x, .y = a.y -% b.y, .z = a.z -% b.z, .w = a.w -% b.w };
    }

    /// operator * (const int4 &inA, const int4 &inB)
    pub fn mul(a: int4, b: int4) int4 {
        return .{ .x = a.x *% b.x, .y = a.y *% b.y, .z = a.z *% b.z, .w = a.w *% b.w };
    }

    /// operator / (const int4 &inA, const int4 &inB): truncates toward zero like C++
    pub fn div(a: int4, b: int4) int4 {
        return .{ .x = @divTrunc(a.x, b.x), .y = @divTrunc(a.y, b.y), .z = @divTrunc(a.z, b.z), .w = @divTrunc(a.w, b.w) };
    }

    /// operator * (const int4 &inA, int inS), also operator * (int inS, const int4 &inA), which is `inA * inS`
    pub fn mulScalar(a: int4, s: i32) int4 {
        return .{ .x = a.x *% s, .y = a.y *% s, .z = a.z *% s, .w = a.w *% s };
    }

    /// operator / (const int4 &inA, int inS): truncates toward zero like C++
    pub fn divScalar(a: int4, s: i32) int4 {
        return .{ .x = @divTrunc(a.x, s), .y = @divTrunc(a.y, s), .z = @divTrunc(a.z, s), .w = @divTrunc(a.w, s) };
    }

    /// operator - (const int4 &inA)
    pub fn negate(a: int4) int4 {
        return .{ .x = -%a.x, .y = -%a.y, .z = -%a.z, .w = -%a.w };
    }

    // Equality

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(a: int4, b: int4) bool {
        return a.x == b.x and a.y == b.y and a.z == b.z and a.w == b.w;
    }

    // Component access

    /// operator [] (uint inIndex) const
    pub fn getComponent(self: int4, index: u32) i32 {
        const components: [4]i32 = @bitCast(self);
        return components[index];
    }

    /// operator [] (uint inIndex)
    pub fn setComponent(self: *int4, index: u32, value: i32) void {
        const components: *[4]i32 = @ptrCast(self);
        components[index] = value;
    }

    // Swizzling

    pub fn xyz(v: int4) int3 {
        return .{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn xzy(v: int4) int3 {
        return .{ .x = v.x, .y = v.z, .z = v.y };
    }

    pub fn yxz(v: int4) int3 {
        return .{ .x = v.y, .y = v.x, .z = v.z };
    }

    pub fn yzx(v: int4) int3 {
        return .{ .x = v.y, .y = v.z, .z = v.x };
    }

    pub fn zxy(v: int4) int3 {
        return .{ .x = v.z, .y = v.x, .z = v.y };
    }

    pub fn zyx(v: int4) int3 {
        return .{ .x = v.z, .y = v.y, .z = v.x };
    }

    pub fn xywz(v: int4) int4 {
        return .{ .x = v.x, .y = v.y, .z = v.w, .w = v.z };
    }

    pub fn xwyz(v: int4) int4 {
        return .{ .x = v.x, .y = v.w, .z = v.y, .w = v.z };
    }

    pub fn wxyz(v: int4) int4 {
        return .{ .x = v.w, .y = v.x, .z = v.y, .w = v.z };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// Mat44
//////////////////////////////////////////////////////////////////////////////////////////

/// A 4x4 matrix of 4 float4 columns (the C++ type of JPH_Mat44 in the shaders)
pub const Mat44 = extern struct {
    /// The columns, `operator [] (uint inIndex)` (private in Jolt)
    c: [4]float4,

    /// Mat44(const float4 &inC0, const float4 &inC1, const float4 &inC2, const float4 &inC3)
    pub fn init(c0: float4, c1: float4, c2: float4, c3: float4) Mat44 {
        return .{ .c = .{ c0, c1, c2, c3 } };
    }
};

//////////////////////////////////////////////////////////////////////////////////////////
// Other types
//////////////////////////////////////////////////////////////////////////////////////////

/// using Quat = float4: xyz = imaginary part, w = real part
pub const Quat = float4;

/// using Plane = float4: xyz = normal, w = constant
pub const Plane = float4;

//////////////////////////////////////////////////////////////////////////////////////////
// Functions (HLSL intrinsics). Each overload of a C++ function is a prong of the switch on the argument type.
//////////////////////////////////////////////////////////////////////////////////////////

fn unsupported(comptime T: type) noreturn {
    @compileError("HLSLToCPP has no overload for " ++ @typeName(T));
}

/// The scalar type of a vector type (the result of `dot`)
pub fn Scalar(comptime T: type) type {
    return switch (T) {
        float2, float3, float4 => f32,
        uint3, uint4 => u32,
        int3, int4 => i32,
        else => unsupported(T),
    };
}

/// Dot product: left to right, `inA.x * inB.x + inA.y * inB.y + ...` (integer overloads wrap)
pub fn dot(a: anytype, b: @TypeOf(a)) Scalar(@TypeOf(a)) {
    return switch (@TypeOf(a)) {
        float2 => a.x * b.x + a.y * b.y,
        float3 => a.x * b.x + a.y * b.y + a.z * b.z,
        float4 => a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w,
        uint3, int3 => a.x *% b.x +% a.y *% b.y +% a.z *% b.z,
        uint4, int4 => a.x *% b.x +% a.y *% b.y +% a.z *% b.z +% a.w *% b.w,
        else => unsupported(@TypeOf(a)),
    };
}

/// Min value: `std::min` for scalars (`(b < a)? b : a`, `using std::min`), per component for vectors
pub fn min(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
    return switch (@TypeOf(a)) {
        f32, u32, i32 => math.min(a, b),
        float2 => .{ .x = math.min(a.x, b.x), .y = math.min(a.y, b.y) },
        float3, uint3, int3 => .{ .x = math.min(a.x, b.x), .y = math.min(a.y, b.y), .z = math.min(a.z, b.z) },
        float4, uint4, int4 => .{ .x = math.min(a.x, b.x), .y = math.min(a.y, b.y), .z = math.min(a.z, b.z), .w = math.min(a.w, b.w) },
        else => unsupported(@TypeOf(a)),
    };
}

/// Max value: `std::max` for scalars (`(a < b)? b : a`, `using std::max`), per component for vectors
pub fn max(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
    return switch (@TypeOf(a)) {
        f32, u32, i32 => math.max(a, b),
        float2 => .{ .x = math.max(a.x, b.x), .y = math.max(a.y, b.y) },
        float3, uint3, int3 => .{ .x = math.max(a.x, b.x), .y = math.max(a.y, b.y), .z = math.max(a.z, b.z) },
        float4, uint4, int4 => .{ .x = math.max(a.x, b.x), .y = math.max(a.y, b.y), .z = math.max(a.z, b.z), .w = math.max(a.w, b.w) },
        else => unsupported(@TypeOf(a)),
    };
}

/// Clamp value: `min(max(inValue, inMinValue), inMaxValue)` (HLSLToCPP's template, for scalars and vectors)
pub fn clamp(value: anytype, min_value: @TypeOf(value), max_value: @TypeOf(value)) @TypeOf(value) {
    return min(max(value, min_value), max_value);
}

/// Length: `sqrt(dot(inV, inV))`
pub fn length(v: anytype) f32 {
    return switch (@TypeOf(v)) {
        float2, float3, float4 => @sqrt(dot(v, v)),
        else => unsupported(@TypeOf(v)),
    };
}

/// Normalization: `inV / length(inV)`, each component divided by the length
pub fn normalize(v: anytype) @TypeOf(v) {
    return switch (@TypeOf(v)) {
        float2, float3, float4 => v.divScalar(length(v)),
        else => unsupported(@TypeOf(v)),
    };
}

/// Rounding to int: `std::round` per component (half away from zero), a float result
pub fn round(v: anytype) @TypeOf(v) {
    return switch (@TypeOf(v)) {
        f32 => @round(v),
        float2 => .{ .x = @round(v.x), .y = @round(v.y) },
        float3 => .{ .x = @round(v.x), .y = @round(v.y), .z = @round(v.z) },
        float4 => .{ .x = @round(v.x), .y = @round(v.y), .z = @round(v.z), .w = @round(v.w) },
        else => unsupported(@TypeOf(v)),
    };
}

/// `sqrt` of a float (`using std::sqrt`), correctly rounded
pub fn sqrt(v: f32) f32 {
    return @sqrt(v);
}

/// Cross product
pub fn cross(a: float3, b: float3) float3 {
    return .{ .x = a.y * b.z - a.z * b.y, .y = a.z * b.x - a.x * b.z, .z = a.x * b.y - a.y * b.x };
}

/// Bitcast float4 to int4
pub fn asint(v: float4) int4 {
    return .{ .x = @bitCast(v.x), .y = @bitCast(v.y), .z = @bitCast(v.z), .w = @bitCast(v.w) };
}

/// The type that `ptr` points to
fn Pointee(comptime P: type) type {
    return @typeInfo(P).pointer.child;
}

/// Atomic add (JPH_AtomicAdd, HLSL's InterlockedAdd): `std::atomic<T>::fetch_add(inValue) + inValue`. The fetch_add is
/// sequentially consistent, both additions wrap.
pub fn atomicAdd(ptr: anytype, value: Pointee(@TypeOf(ptr))) Pointee(@TypeOf(ptr)) {
    const T = Pointee(@TypeOf(ptr));
    return @atomicRmw(T, ptr, .Add, value, .seq_cst) +% value;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Scalar conversions (C++ casts that HLSLToCPP relies on)
//////////////////////////////////////////////////////////////////////////////////////////

/// C++ `int(f)` as Jolt's x86-64 build computes it: a 32-bit `cvttss2si`, which truncates toward zero and gives
/// 0x80000000 for NaN and values outside the range of an int (porting guide section 8, rule 12)
pub fn toInt(f: f32) i32 {
    if (f >= -0x1p31 and f < 0x1p31) // False for NaN
        return @intFromFloat(f);
    return std.math.minInt(i32);
}

/// C++ `uint32(f)` as Jolt's x86-64 build computes it: a 64-bit `cvttss2si` (truncation toward zero) of which the low
/// 32 bits are used. NaN and values outside the range of an int64 give 0, negative values wrap (`uint32(-1.0f)` is
/// 0xffffffff).
pub fn toUint(f: f32) u32 {
    if (f >= -0x1p63 and f < 0x1p63) // False for NaN
        return @truncate(@as(u64, @bitCast(@as(i64, @intFromFloat(f)))));
    return 0;
}

/// C++ `uint32(f)` of a vector component as Jolt's x86-64 build computes it when the compiler converts a whole vector
/// (`uint3(const float3 &)`). x86-64-v3 has no unsigned vector conversion, so LLVM uses the signed one twice:
/// `small = cvttps2dq(f)`, `big = cvttps2dq(f - 2^31)`, result `small | (big & (small >> 31))` (arithmetic shift).
/// Values in [0, 2^32) and in (-2^31, 0] give the same as `toUint`; NaN, values of 2^32 and above and values of -2^31
/// and below give 0x80000000.
pub fn toUintVectorized(f: f32) u32 {
    const small: u32 = @bitCast(toInt(f));
    const big: u32 = @bitCast(toInt(f - 0x1p31));
    const overflown: u32 = @bitCast(@as(i32, @bitCast(small)) >> 31);
    return small | (big & overflown);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Bindings of a shader (WrapShaderBegin.h, RemainingFeaturesArchitecture.md R12). A shader's `Bindings` struct has one
// field per binding, named like the HLSL binding. ShaderWrapper.Of(Shader) calls `bind` on the field whose name is
// bound.
//////////////////////////////////////////////////////////////////////////////////////////

/// JPH_SHADER_CONSTANTS_BEGIN(type, name): a constant block of the shader. Binding a constant buffer copies its bytes
/// into `value` (Jolt: `memcpy(&name + 1, inData, inSize)`), so the queue may release the buffer afterwards.
/// `value` starts zeroed: Jolt's `new Wrapper()` value-initializes the members of the wrapper.
pub fn Constants(comptime T: type) type {
    return struct {
        const Self = @This();

        /// The constants (Jolt: the members `cName` that follow the block's dummy member)
        value: T = std.mem.zeroes(T),

        /// Copy a bound constant buffer into the block.
        /// Zolt (UB5 of RemainingFeaturesArchitecture.md R20): Jolt copies the whole buffer, so a buffer larger than the
        /// block overwrites the members after it. Zolt asserts and copies the block size.
        pub fn bind(self: *Self, data: []align(16) const u8) void {
            if (Core.enable_asserts) std.debug.assert(data.len <= @sizeOf(T));
            const size = @min(data.len, @sizeOf(T));
            @memcpy(std.mem.asBytes(&self.value)[0..size], data[0..size]);
        }
    };
}

/// JPH_SHADER_BIND_BUFFER(type, name): a read only buffer (Jolt: `const type *name = nullptr`). It is a slice over the
/// buffer memory, so a read out of bounds panics in safe builds where Jolt reads past the allocation (UB3).
pub fn Buffer(comptime T: type) type {
    return struct {
        const Self = @This();

        /// The bound elements, empty until bound
        data: []const T = &.{},

        /// Bind the memory of a buffer (Jolt: `name = (const type *)inData`)
        pub fn bind(self: *Self, data: []align(16) const u8) void {
            const elements: [*]const T = @ptrCast(@alignCast(data.ptr));
            self.data = elements[0 .. data.len / @sizeOf(T)];
        }

        /// `name[index]`
        pub fn get(self: Self, index: usize) T {
            return self.data[index];
        }

        /// `&name[index]`
        pub fn at(self: Self, index: usize) *const T {
            return &self.data[index];
        }
    };
}

/// JPH_SHADER_BIND_RW_BUFFER(type, name): a read / write buffer (Jolt: `type *name = nullptr`). It is a slice over the
/// buffer memory, so an access out of bounds panics in safe builds.
pub fn RWBuffer(comptime T: type) type {
    return struct {
        const Self = @This();

        /// The bound elements, empty until bound
        data: []T = &.{},

        /// Bind the memory of a buffer (Jolt: `name = (type *)inData`)
        pub fn bind(self: *Self, data: []align(16) u8) void {
            const elements: [*]T = @ptrCast(@alignCast(data.ptr));
            self.data = elements[0 .. data.len / @sizeOf(T)];
        }

        /// `name[index]` (read)
        pub fn get(self: Self, index: usize) T {
            return self.data[index];
        }

        /// `name[index] = value`
        pub fn set(self: Self, index: usize, value: T) void {
            self.data[index] = value;
        }

        /// `&name[index]`, for a read-modify-write or an atomic (`JPH_AtomicAdd(name[index].x, v)`)
        pub fn at(self: Self, index: usize) *T {
            return &self.data[index];
        }
    };
}

comptime {
    // The layouts of HLSLToCPP (and of ShaderCore.h's C++ types: Float3, Float4, uint32[3], Float4[4])
    std.debug.assert(@sizeOf(float2) == 8);
    std.debug.assert(@sizeOf(float3) == 12 and @alignOf(float3) == 4);
    std.debug.assert(@sizeOf(float4) == 16 and @alignOf(float4) == 4);
    std.debug.assert(@sizeOf(uint3) == 12 and @sizeOf(uint4) == 16);
    std.debug.assert(@sizeOf(int3) == 12 and @sizeOf(int4) == 16);
    std.debug.assert(@sizeOf(Mat44) == 64 and @alignOf(Mat44) == 4);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Tests: the semantics of R11 that differ from Zolt's Vec3
//////////////////////////////////////////////////////////////////////////////////////////

const testing = std.testing;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;

fn bits(f: f32) u32 {
    return @bitCast(f);
}

test "HLSLToCPP: dot is left to right" {
    // HLSL: (x * x' + y * y') + z * z'. Vec3: (x * x' + y * y') + (z * z' + 0). They differ for a sum of -0.
    const m0 = float3.init(-0.0, -0.0, -0.0);
    const one = float3.splat(1.0);
    try testing.expectEqual(bits(-0.0), bits(dot(m0, one)));
    try testing.expectEqual(bits(0.0), bits(Vec3.init(-0.0, -0.0, -0.0).dot(Vec3.one())));

    // Rounding: (1e8 + -1e8) + 1 = 1 left to right
    try testing.expectEqual(@as(f32, 1.0), dot(float3.init(1.0e8, -1.0e8, 1.0), one));
    try testing.expectEqual(@as(f32, 1.0), dot(float4.init(1.0e8, -1.0e8, 1.0, 0.0), float4.splat(1.0)));
    try testing.expectEqual(@as(f32, 0.0), dot(float2.init(1.0e8, -1.0e8), float2.splat(1.0)));

    // The integer overloads wrap
    try testing.expectEqual(@as(u32, 1), dot(uint3.init(0xffffffff, 1, 1), uint3.init(1, 1, 1)));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), dot(int4.init(std.math.maxInt(i32), 1, 0, 0), int4.init(1, 1, 0, 0)));
}

test "HLSLToCPP: normalize divides by the length" {
    // inV / length(inV): each component divided, no multiplication with a reciprocal
    const v = float3.init(1.0, 2.0, 3.0);
    const len = @sqrt(@as(f32, 14.0));
    try testing.expectEqual(len, length(v));
    const n = normalize(v);
    try testing.expectEqual(bits(1.0 / len), bits(n.x));
    try testing.expectEqual(bits(2.0 / len), bits(n.y));
    try testing.expectEqual(bits(3.0 / len), bits(n.z));

    // A component where x / len differs from x * (1 / len)
    var found = false;
    var x: f32 = 1.0;
    while (x < 100.0) : (x += 1.0) {
        const w = float3.init(x, 1.0, 0.0);
        const l = length(w);
        if (normalize(w).x != x * (1.0 / l)) {
            try testing.expectEqual(bits(x / l), bits(normalize(w).x));
            found = true;
        }
    }
    try testing.expect(found);
}

test "HLSLToCPP: min / max are std::min / std::max per component" {
    const nan = std.math.nan(f32);

    // std::min(a, b) is (b < a)? b : a: a NaN in the second argument gives the first
    try testing.expectEqual(@as(f32, 1.0), min(@as(f32, 1.0), nan));
    try testing.expect(std.math.isNan(min(nan, @as(f32, 1.0))));
    try testing.expectEqual(@as(f32, 1.0), max(@as(f32, 1.0), nan));
    try testing.expect(std.math.isNan(max(nan, @as(f32, 1.0))));

    // -0 and +0 are equal: the first argument wins
    try testing.expectEqual(bits(-0.0), bits(min(@as(f32, -0.0), 0.0)));
    try testing.expectEqual(bits(0.0), bits(min(@as(f32, 0.0), -0.0)));
    try testing.expectEqual(bits(-0.0), bits(max(@as(f32, -0.0), 0.0)));

    // Per component on vectors, and Vec3's min (minps) differs: it returns the second operand for NaN
    const a = float3.init(nan, 2.0, -0.0);
    const b = float3.init(1.0, nan, 0.0);
    const m = min(a, b);
    try testing.expect(std.math.isNan(m.x));
    try testing.expectEqual(@as(f32, 2.0), m.y);
    try testing.expectEqual(bits(-0.0), bits(m.z));
    try testing.expectEqual(@as(f32, 1.0), Vec3.min(Vec3.init(nan, 2.0, 0.0), Vec3.init(1.0, nan, 0.0)).getX());

    // Integer vectors
    try testing.expect(min(uint3.init(1, 5, 3), uint3.init(2, 4, 3)).eql(uint3.init(1, 4, 3)));
    try testing.expect(max(int4.init(-1, 5, 3, 0), int4.init(2, -4, 3, 0)).eql(int4.init(2, 5, 3, 0)));
}

test "HLSLToCPP: clamp is min(max(v, lo), hi)" {
    // With lo > hi the result is hi (std::clamp would be undefined)
    try testing.expectEqual(@as(f32, 0.0), clamp(@as(f32, 0.5), 1.0, 0.0));
    try testing.expectEqual(@as(f32, 0.25), clamp(@as(f32, 0.25), 0.0, 1.0));
    // A NaN value: max(NaN, lo) is NaN, min(NaN, hi) is NaN
    try testing.expect(std.math.isNan(clamp(std.math.nan(f32), 0.0, 1.0)));
    try testing.expect(clamp(float3.init(-1.0, 0.5, 2.0), float3.splat(0.0), float3.splat(1.0)).eql(float3.init(0.0, 0.5, 1.0)));
    try testing.expect(clamp(uint3.init(0, 5, 9), uint3.init(1, 1, 1), uint3.init(8, 8, 8)).eql(uint3.init(1, 5, 8)));
}

test "HLSLToCPP: unary minus flips the sign" {
    // -v flips the sign bit, Vec3's negate is 0 - v (+0 for +0)
    const v = float3.init(0.0, -0.0, 1.0);
    const n = v.negate();
    try testing.expectEqual(bits(-0.0), bits(n.x));
    try testing.expectEqual(bits(0.0), bits(n.y));
    try testing.expectEqual(bits(-1.0), bits(n.z));
    try testing.expectEqual(bits(0.0), bits(Vec3.init(0.0, -0.0, 1.0).negate().getX()));
    try testing.expectEqual(bits(-0.0), bits(float4.init(0.0, 0.0, 0.0, 0.0).negate().w));
    try testing.expectEqual(bits(-0.0), bits(float2.init(0.0, 0.0).negate().y));

    // Integer negate wraps
    try testing.expect(int3.init(std.math.minInt(i32), 1, -1).negate().eql(int3.init(std.math.minInt(i32), -1, 1)));
}

test "HLSLToCPP: round is std::round, asint is a bitcast" {
    // Half away from zero
    try testing.expect(round(float4.init(0.5, -0.5, 1.5, 2.5)).eql(float4.init(1.0, -1.0, 2.0, 3.0)));
    try testing.expectEqual(bits(-0.0), bits(round(@as(f32, -0.4))));
    try testing.expect(std.math.isNan(round(float3.splat(std.math.nan(f32))).x));
    try testing.expect(asint(float4.init(1.0, -0.0, 0.0, -2.0)).eql(int4.init(0x3f800000, std.math.minInt(i32), 0, @bitCast(@as(u32, 0xc0000000)))));
}

test "HLSLToCPP: float to int conversions follow the x86-64 build" {
    // int(f): 32-bit cvttss2si
    try testing.expectEqual(@as(i32, 13), toInt(13.9));
    try testing.expectEqual(@as(i32, -13), toInt(-13.9));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), toInt(-0x1p31));
    try testing.expectEqual(@as(i32, std.math.maxInt(i32) - 127), toInt(0x1.fffffep30));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), toInt(0x1p31));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), toInt(3.0e9));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), toInt(std.math.nan(f32)));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), toInt(-std.math.inf(f32)));

    // uint32(f): 64-bit cvttss2si, the low 32 bits
    try testing.expectEqual(@as(u32, 3), toUint(3.99));
    try testing.expectEqual(@as(u32, 0xffffffff), toUint(-1.0));
    try testing.expectEqual(@as(u32, 0xfffffffd), toUint(-3.5));
    try testing.expectEqual(@as(u32, 3000000000), toUint(3.0e9));
    try testing.expectEqual(@as(u32, 0), toUint(0x1p32)); // 2^32 truncated to 32 bits
    try testing.expectEqual(@as(u32, 0), toUint(0x1p63)); // Out of the int64 range
    try testing.expectEqual(@as(u32, 0), toUint(-0x1p63)); // -2^63 is in range, its low 32 bits are 0
    try testing.expectEqual(@as(u32, 0), toUint(std.math.nan(f32)));
    try testing.expectEqual(@as(u32, 0), toUint(std.math.inf(f32)));

    // uint3(float3): x and y use the vectorized conversion (0x80000000 for NaN and for values of 2^32 and above), z the
    // scalar one
    try testing.expectEqual(@as(u32, 3000000000), toUintVectorized(3.0e9));
    try testing.expectEqual(@as(u32, 0xffffffff), toUintVectorized(-1.0));
    try testing.expectEqual(@as(u32, 0), toUintVectorized(-0.5));
    try testing.expectEqual(@as(u32, 0x80000000), toUintVectorized(0x1p32));
    try testing.expectEqual(@as(u32, 0x80000000), toUintVectorized(std.math.nan(f32)));
    try testing.expectEqual(@as(u32, 0x80000000), toUintVectorized(-0x1p31));
    try testing.expectEqual(@as(u32, 0x80000000), toUintVectorized(-std.math.inf(f32)));
    try testing.expect(uint3.fromFloat3(float3.init(-1.0, 2.5, std.math.nan(f32))).eql(uint3.init(0xffffffff, 2, 0)));
    try testing.expect(uint3.fromFloat3(float3.init(std.math.nan(f32), 0x1p33, 0x1.000002p32)).eql(uint3.init(0x80000000, 0x80000000, 512)));

    // int4(float4) uses the signed conversion, which is the same for scalars and vectors
    try testing.expect(int4.fromFloat4(float4.init(-1.5, 2.5, 3.0e9, 0.0)).eql(int4.init(-1, 2, std.math.minInt(i32), 0)));

    // float(uint) and float(int) are correctly rounded
    try testing.expect(float3.fromUint3(uint3.init(0xffffffff, 16777217, 1)).eql(float3.init(4294967296.0, 16777216.0, 1.0)));
    try testing.expect(float4.fromInt4(int4.init(-16777217, 16777219, std.math.minInt(i32), 0)).eql(float4.init(-16777216.0, 16777220.0, -0x1p31, 0.0)));
}

test "HLSLToCPP: integer arithmetic wraps" {
    try testing.expect(uint3.init(0xffffffff, 0, 1).add(uint3.init(1, 0, 1)).eql(uint3.init(0, 0, 2)));
    try testing.expect(uint3.init(0, 0, 0).sub(uint3.init(1, 0, 0)).eql(uint3.init(0xffffffff, 0, 0)));
    try testing.expect(uint4.init(0x80000000, 3, 0, 0).mulScalar(2).eql(uint4.init(0, 6, 0, 0)));
    try testing.expect(int4.init(std.math.maxInt(i32), 0, 0, 0).add(int4.init(1, 0, 0, 0)).eql(int4.init(std.math.minInt(i32), 0, 0, 0)));
    // Division truncates toward zero like C++
    try testing.expect(int3.init(-7, 7, -7).div(int3.init(2, -2, -2)).eql(int3.init(-3, -3, 3)));
    try testing.expect(int4.init(-7, 7, -8, 8).divScalar(3).eql(int4.init(-2, 2, -2, 2)));
}

test "HLSLToCPP: atomicAdd returns the new value" {
    var i: i32 = std.math.maxInt(i32);
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), atomicAdd(&i, 1));
    try testing.expectEqual(@as(i32, std.math.minInt(i32)), i);
    var u: u32 = 5;
    try testing.expectEqual(@as(u32, 12), atomicAdd(&u, 7));
    try testing.expectEqual(@as(u32, 12), u);
}

test "HLSLToCPP: swizzles, component access, cross" {
    const v = float4.init(1, 2, 3, 4);
    try testing.expect(v.wxyz().eql(float4.init(4, 1, 2, 3)));
    try testing.expect(v.xwyz().eql(float4.init(1, 4, 2, 3)));
    try testing.expect(v.xywz().eql(float4.init(1, 2, 4, 3)));
    try testing.expect(v.zxy().eql(float3.init(3, 1, 2)));
    try testing.expect(v.xyz().yzx().eql(float3.init(2, 3, 1)));
    try testing.expect(v.yx().eql(float2.init(2, 1)));
    try testing.expectEqual(@as(f32, 3), v.getComponent(2));
    var w = v;
    w.setComponent(3, 7);
    try testing.expectEqual(@as(f32, 7), w.w);
    var u = uint4.init(1, 2, 3, 4);
    u.setComponent(0, 9);
    try testing.expect(u.wxyz().eql(uint4.init(4, 9, 2, 3)));
    try testing.expect(cross(float3.init(1, 0, 0), float3.init(0, 1, 0)).eql(float3.init(0, 0, 1)));

    // float * float3 is float3 * float
    try testing.expect(float3.init(1, 2, 3).mulScalar(2).eql(float3.init(2, 4, 6)));
}

test "HLSLToCPP: binding types" {
    const Context = extern struct { a: u32, b: f32, c: u32 };

    // A constant block starts zeroed and copies the bound bytes
    var constants: Constants(Context) = .{};
    try testing.expectEqual(@as(u32, 0), constants.value.c);
    const data: [3]u32 align(16) = .{ 7, @bitCast(@as(f32, 1.5)), 9 };
    constants.bind(std.mem.sliceAsBytes(&data));
    try testing.expectEqual(Context{ .a = 7, .b = 1.5, .c = 9 }, constants.value);

    // A shorter buffer copies its bytes only
    const short: [1]u32 align(16) = .{11};
    constants.bind(std.mem.sliceAsBytes(&short));
    try testing.expectEqual(Context{ .a = 11, .b = 1.5, .c = 9 }, constants.value);

    // A larger buffer: Zolt asserts (UB5), without asserts it copies the block size
    if (!Core.enable_asserts) {
        const long: [4]u32 align(16) = .{ 1, 2, 3, 4 };
        constants.bind(std.mem.sliceAsBytes(&long));
        try testing.expectEqual(@as(u32, 3), constants.value.c);
    }

    // Buffers are slices over the bound memory
    var memory: [4]u32 align(16) = .{ 1, 2, 3, 4 };
    var buffer: Buffer(u32) = .{};
    try testing.expectEqual(@as(usize, 0), buffer.data.len);
    buffer.bind(std.mem.sliceAsBytes(&memory));
    try testing.expectEqual(@as(u32, 3), buffer.get(2));
    var rw: RWBuffer(int4) = .{};
    rw.bind(std.mem.sliceAsBytes(&memory));
    try testing.expectEqual(@as(usize, 1), rw.data.len);
    rw.set(0, rw.get(0).add(int4.splat(1)));
    _ = atomicAdd(&rw.at(0).w, 10);
    try testing.expectEqualSlices(u32, &.{ 2, 3, 4, 15 }, &memory);
}
