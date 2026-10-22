// CoreMinimal.h - A stand-in for the engine types VoxelOceanClipmap.h uses (stage W1), so
// clip_check.cpp compiles the shipped header unchanged. Not the engine's behaviour beyond that.
#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

typedef int32_t int32; typedef uint32_t uint32; typedef int64_t int64; typedef uint64_t uint64;
#define UE_SQRT_2 1.41421356237309504880
#define MIN_int32 INT32_MIN

struct FMath
{
    template<class T> static T Max(T a, T b) { return a > b ? a : b; }
    template<class T> static T Min(T a, T b) { return a < b ? a : b; }
    template<class T> static T Clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
    static double Abs(double v) { return std::fabs(v); }
    static int32 Abs(int32 v) { return v < 0 ? -v : v; }
    static double Sqrt(double v) { return std::sqrt(v); }
    static double Cos(double v) { return std::cos(v); }
    static double Tan(double v) { return std::tan(v); }
    static double Log2(double v) { return std::log2(v); }
    static double Pow(double a, double b) { return std::pow(a, b); }
    static double Fmod(double a, double b) { return std::fmod(a, b); }
    static double FloorToDouble(double v) { return std::floor(v); }
    static double CeilToDouble(double v) { return std::ceil(v); }
    static int32 RoundToInt(double v) { return (int32)std::floor(v + 0.5); }
    static double DegreesToRadians(double v) { return v * 3.14159265358979323846 / 180.0; }
};

struct FVector
{
    double X = 0, Y = 0, Z = 0;
    FVector() = default;
    FVector(double x, double y, double z) : X(x), Y(y), Z(z) {}
    FVector operator+(const FVector& o) const { return { X + o.X, Y + o.Y, Z + o.Z }; }
    FVector operator-(const FVector& o) const { return { X - o.X, Y - o.Y, Z - o.Z }; }
    FVector operator*(double s) const { return { X * s, Y * s, Z * s }; }
    FVector operator/(double s) const { return { X / s, Y / s, Z / s }; }
    double operator|(const FVector& o) const { return X * o.X + Y * o.Y + Z * o.Z; }
    FVector operator^(const FVector& o) const { return { Y * o.Z - Z * o.Y, Z * o.X - X * o.Z, X * o.Y - Y * o.X }; }
    bool operator==(const FVector& o) const { return X == o.X && Y == o.Y && Z == o.Z; }
    double& operator[](int i) { return i == 0 ? X : (i == 1 ? Y : Z); }
    double operator[](int i) const { return i == 0 ? X : (i == 1 ? Y : Z); }
    double Size() const { return std::sqrt(X * X + Y * Y + Z * Z); }
    bool IsNearlyZero() const { return Size() < 1e-8; }
    FVector GetSafeNormal() const { const double L = Size(); return L > 1e-30 ? *this / L : FVector(); }
    static const FVector ZeroVector;
};
inline const FVector FVector::ZeroVector(0, 0, 0);

struct FVector2D
{
    double X = 0, Y = 0;
    FVector2D() = default;
    FVector2D(double x, double y) : X(x), Y(y) {}
    FVector2D operator/(double s) const { return { X / s, Y / s }; }
    FVector2D operator+(const FVector2D& o) const { return { X + o.X, Y + o.Y }; }
    double Size() const { return std::sqrt(X * X + Y * Y); }
    static const FVector2D ZeroVector;
};
inline const FVector2D FVector2D::ZeroVector(0, 0);

struct FIntPoint
{
    int32 X = 0, Y = 0;
    FIntPoint() = default;
    FIntPoint(int32 x, int32 y) : X(x), Y(y) {}
    bool operator==(const FIntPoint& o) const { return X == o.X && Y == o.Y; }
};

struct FVector4f
{
    float X = 0, Y = 0, Z = 0, W = 0;
    FVector4f() = default;
    FVector4f(float x, float y, float z, float w) : X(x), Y(y), Z(z), W(w) {}
};
