// VoxelShaderCompat.h - The small subset of HLSL types and intrinsics the shared density
// code (VoxelDensityCore.ush) uses, so the same file compiles as C++ for the CPU backend.
//
// Include this, then the .ush, inside namespace VoxelShader (see VoxelDensity.cpp): the
// names below (float3, dot, lerp, ...) stay out of the global namespace.
//
// Subset rules for shared code: scalar/float3/uint3 only; members .x/.y/.z (no swizzles);
// no out/inout parameters (return structs); 32-bit uint arithmetic (wraps on both sides);
// float literals with the f suffix.

#pragma once

#include <cmath>
#include <cstdint>

// Shared functions are plain inline functions in C++ (HLSL inlines everything anyway).
#define VOXEL_FUNC inline

namespace VoxelShader
{
    typedef uint32_t uint;

    struct float3
    {
        float x, y, z;
        float3() : x(0.f), y(0.f), z(0.f) {}
        explicit float3(float s) : x(s), y(s), z(s) {}
        float3(float InX, float InY, float InZ) : x(InX), y(InY), z(InZ) {}
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

    struct uint3
    {
        uint x, y, z;
        uint3() : x(0u), y(0u), z(0u) {}
        uint3(uint InX, uint InY, uint InZ) : x(InX), y(InY), z(InZ) {}
    };

    inline uint3 operator+(uint3 a, uint3 b) { return uint3(a.x + b.x, a.y + b.y, a.z + b.z); }

    inline float dot(float3 a, float3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float abs(float a) { return std::fabs(a); }
    inline float sqrt(float a) { return std::sqrt(a); }
    inline float min(float a, float b) { return a < b ? a : b; }
    inline float max(float a, float b) { return a > b ? a : b; }
    inline float clamp(float v, float lo, float hi) { return min(max(v, lo), hi); }
    inline float saturate(float v) { return clamp(v, 0.f, 1.f); }
    inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
    inline float smoothstep(float e0, float e1, float v)
    {
        const float t = saturate((v - e0) / (e1 - e0));
        return t * t * (3.f - 2.f * t);
    }
}
