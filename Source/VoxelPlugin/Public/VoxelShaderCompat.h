// VoxelShaderCompat.h - The small subset of HLSL types and intrinsics the shared density
// code (VoxelDensityCore.ush) uses, so the same file compiles as C++ for the CPU backend.
//
// Include this, then the .ush, inside namespace VoxelShader (see VoxelDensity.cpp): the
// names below (float3, dot, lerp, ...) stay out of the global namespace.
//
// The land code (VoxelLandSample.ush, VoxelLandPlates.ush) is included inside a C++ struct
// whose members carry the names of the shader's buffers and constants (VoxelLandBake.cpp,
// VoxelDensity.cpp), so it must not declare static const data of its own (use #define).
//
// Subset rules for shared code: scalar/float2/float3/int3/uint3 only (float4 as a value holder); members .x/.y/.z or
// [axis] (no swizzles); no out/inout parameters in shared functions (return structs; the
// Interlocked* intrinsics are the exception); 32-bit int/uint arithmetic (wraps on both
// sides); float literals with the f suffix; VOXEL_PRECISE on every float operation whose exact
// rounding matters (double-float arithmetic). Compile the C++ side without fused
// multiply-add contraction (MSVC /fp:precise, the UE default).

#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

// Shared functions are plain inline functions in C++ (HLSL inlines everything anyway).
#define VOXEL_FUNC inline
// HLSL's precise (no reassociation, no fusing); plain float operations in C++.
#define VOXEL_PRECISE
// HLSL's [loop] (don't unroll); nothing in C++.
#define VOXEL_LOOP

namespace VoxelShader
{
    typedef uint32_t uint;

    struct float2
    {
        float x, y;
        float2() : x(0.f), y(0.f) {}
        float2(float InX, float InY) : x(InX), y(InY) {}
    };

    struct float3
    {
        float x, y, z;
        float3() : x(0.f), y(0.f), z(0.f) {}
        explicit float3(float s) : x(s), y(s), z(s) {}
        float3(float InX, float InY, float InZ) : x(InX), y(InY), z(InZ) {}
        float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
        float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    };

    inline float3 operator+(float3 a, float3 b) { return float3(a.x + b.x, a.y + b.y, a.z + b.z); }
    inline float3 operator-(float3 a, float3 b) { return float3(a.x - b.x, a.y - b.y, a.z - b.z); }
    inline float3 operator*(float3 a, float3 b) { return float3(a.x * b.x, a.y * b.y, a.z * b.z); }
    inline float3 operator/(float3 a, float3 b) { return float3(a.x / b.x, a.y / b.y, a.z / b.z); }
    inline float3 operator+(float3 a, float s) { return float3(a.x + s, a.y + s, a.z + s); }
    inline float3 operator-(float3 a, float s) { return float3(a.x - s, a.y - s, a.z - s); }
    inline float3 operator*(float3 a, float s) { return float3(a.x * s, a.y * s, a.z * s); }
    inline float3 operator/(float3 a, float s) { return float3(a.x / s, a.y / s, a.z / s); }
    inline float3 operator+(float s, float3 a) { return float3(s + a.x, s + a.y, s + a.z); }
    inline float3 operator-(float s, float3 a) { return float3(s - a.x, s - a.y, s - a.z); }
    inline float3 operator*(float s, float3 a) { return float3(s * a.x, s * a.y, s * a.z); }
    inline float3 operator-(float3 a) { return float3(-a.x, -a.y, -a.z); }

    // float4: members only (the ocean's table texels; VoxelOceanSeaState.ush).
    struct float4
    {
        float x, y, z, w;
        float4() : x(0.f), y(0.f), z(0.f), w(0.f) {}
        float4(float InX, float InY, float InZ, float InW) : x(InX), y(InY), z(InZ), w(InW) {}
    };

    struct uint3
    {
        uint x, y, z;
        uint3() : x(0u), y(0u), z(0u) {}
        uint3(uint InX, uint InY, uint InZ) : x(InX), y(InY), z(InZ) {}
    };

    inline uint3 operator+(uint3 a, uint3 b) { return uint3(a.x + b.x, a.y + b.y, a.z + b.z); }

    struct int2
    {
        int x, y;
        int2() : x(0), y(0) {}
        int2(int InX, int InY) : x(InX), y(InY) {}
    };

    struct int3
    {
        int x, y, z;
        int3() : x(0), y(0), z(0) {}
        int3(int InX, int InY, int InZ) : x(InX), y(InY), z(InZ) {}
        int3(int2 XY, int InZ) : x(XY.x), y(XY.y), z(InZ) {}
        int& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
        int operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    };

    inline int3 operator+(int3 a, int3 b) { return int3(a.x + b.x, a.y + b.y, a.z + b.z); }
    inline int3 operator-(int3 a, int3 b) { return int3(a.x - b.x, a.y - b.y, a.z - b.z); }

    inline float floor(float a) { return std::floor(a); }
    inline uint asuint(int a) { return (uint)a; }
    inline uint asuint(float a) { uint u; std::memcpy(&u, &a, 4); return u; }
    inline float asfloat(uint a) { float f; std::memcpy(&f, &a, 4); return f; }
    inline int asint(float a) { int i; std::memcpy(&i, &a, 4); return i; }
    inline float ldexp(float a, int e) { return std::ldexp(a, e); }
    inline float atan2(float y, float x) { return std::atan2(y, x); }
    inline float atan(float a) { return std::atan(a); }
    inline float acos(float a) { return std::acos(a); }
    inline float tan(float a) { return std::tan(a); }
    inline float exp(float a) { return std::exp(a); }
    inline float cos(float a) { return std::cos(a); }
    inline float sin(float a) { return std::sin(a); }
    inline int min(int a, int b) { return a < b ? a : b; }
    inline int max(int a, int b) { return a > b ? a : b; }
    inline uint min(uint a, uint b) { return a < b ? a : b; }
    inline uint max(uint a, uint b) { return a > b ? a : b; }
    inline int clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
    inline float log2(float a) { return std::log2(a); }
    inline float exp2(float a) { return std::exp2(a); }
    inline float pow(float a, float b) { return std::pow(a, b); }
    /** HLSL's f16tof32: the IEEE half in the low 16 bits as a float (exact). */
    inline float f16tof32(uint h)
    {
        const uint Sign = (h >> 15) & 1u;
        const uint Exponent = (h >> 10) & 0x1fu;
        const uint Mantissa = h & 0x3ffu;
        float Value;
        if (Exponent == 0u) Value = std::ldexp((float)Mantissa, -24);
        else if (Exponent == 31u) Value = Mantissa != 0u ? NAN : INFINITY;
        else Value = std::ldexp((float)(Mantissa | 0x400u), (int)Exponent - 25);
        return Sign != 0u ? -Value : Value;
    }

    /** HLSL's f32tof16: the nearest IEEE half (ties to even; overflow to infinity, NaN kept),
     *  in the low 16 bits. */
    inline uint f32tof16(float f)
    {
        uint x;
        std::memcpy(&x, &f, 4);
        const uint Sign = (x >> 16) & 0x8000u;
        const uint Abs = x & 0x7fffffffu;
        if (Abs >= 0x7f800000u) return Sign | 0x7c00u | (Abs > 0x7f800000u ? 0x200u : 0u);
        if (Abs >= 0x477ff000u) return Sign | 0x7c00u;          // rounds past the largest half
        if (Abs < 0x38800000u)
        {
            // A subnormal half (or zero): the value in units of 2^-24, rounded to even.
            if (Abs < 0x33000000u) return Sign;
            const uint Exponent = Abs >> 23;
            const uint Mantissa = (Abs & 0x7fffffu) | 0x800000u;
            const uint Shift = 126u - Exponent;                 // 14 .. 24
            const uint Half = Mantissa >> Shift;
            const uint Rest = Mantissa & ((1u << Shift) - 1u);
            const uint Mid = 1u << (Shift - 1u);
            return Sign | (Half + ((Rest > Mid || (Rest == Mid && (Half & 1u))) ? 1u : 0u));
        }
        const uint Rebased = Abs - 0x38000000u;                 // the exponent rebased to the half's
        const uint Half = Rebased >> 13;
        const uint Rest = Rebased & 0x1fffu;
        return Sign | (Half + ((Rest > 0x1000u || (Rest == 0x1000u && (Half & 1u))) ? 1u : 0u));
    }

#if defined(__cpp_lib_atomic_ref)
    // Atomics on buffer elements (uint lvalues), with HLSL's signatures (C++ tests of the
    // GPU passes).
    inline void InterlockedAdd(uint& Dest, uint Value, uint& Original)
    {
        Original = std::atomic_ref<uint>(Dest).fetch_add(Value);
    }
    inline void InterlockedCompareExchange(uint& Dest, uint Compare, uint Value, uint& Original)
    {
        uint Expected = Compare;
        std::atomic_ref<uint>(Dest).compare_exchange_strong(Expected, Value);
        Original = Expected;
    }
    inline void InterlockedOr(uint& Dest, uint Value)
    {
        std::atomic_ref<uint>(Dest).fetch_or(Value);
    }
    inline void InterlockedOr(uint& Dest, uint Value, uint& Original)
    {
        Original = std::atomic_ref<uint>(Dest).fetch_or(Value);
    }
    inline void InterlockedAdd(uint& Dest, uint Value)
    {
        std::atomic_ref<uint>(Dest).fetch_add(Value);
    }
    inline void InterlockedMin(uint& Dest, uint Value)
    {
        std::atomic_ref<uint> Ref(Dest);
        uint Old = Ref.load();
        while (Value < Old && !Ref.compare_exchange_weak(Old, Value)) {}
    }
    inline void InterlockedMax(uint& Dest, uint Value)
    {
        std::atomic_ref<uint> Ref(Dest);
        uint Old = Ref.load();
        while (Value > Old && !Ref.compare_exchange_weak(Old, Value)) {}
    }
#endif

    inline float dot(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float length(float3 a) { return std::sqrt(dot(a, a)); }
    inline float3 normalize(float3 a) { return a * (1.f / std::sqrt(dot(a, a))); }
    inline float3 cross(float3 a, float3 b) { return float3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
    inline float abs(float a) { return std::fabs(a); }
    inline float sqrt(float a) { return std::sqrt(a); }
    inline float min(float a, float b) { return a < b ? a : b; }
    inline float max(float a, float b) { return a > b ? a : b; }
    inline float clamp(float v, float lo, float hi) { return min(max(v, lo), hi); }
    inline float saturate(float v) { return clamp(v, 0.f, 1.f); }
    inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
    inline float3 lerp(float3 a, float3 b, float t) { return a + (b - a) * t; }
    inline float smoothstep(float e0, float e1, float v)
    {
        const float t = saturate((v - e0) / (e1 - e0));
        return t * t * (3.f - 2.f * t);
    }
}
