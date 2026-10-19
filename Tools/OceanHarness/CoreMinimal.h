// Minimal stand-ins for the UE types VoxelOceanTiles.h uses, so the harness compiles the shipped
// header unchanged. Double FVector, float FVector3f, TArray / TSet over std containers.
#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <functional>

using int32 = int32_t; using int64 = int64_t; using uint32 = uint32_t; using uint8 = uint8_t; using uint64 = uint64_t;
#define UE_DOUBLE_PI 3.141592653589793238462643383279502884197169399

inline uint32 GetTypeHash(int32 V) { return (uint32)V; }
inline uint32 GetTypeHash(uint32 V) { return V; }
inline uint32 HashCombine(uint32 A, uint32 C) { uint32 B = 0x9e3779b9; A += B; A -= B; A -= C; A ^= (C >> 13); B -= C; B -= A; B ^= (A << 8); C -= A; C -= B; C ^= (B >> 13); return C ^ (A * 31u); }

struct FVector
{
    double X = 0, Y = 0, Z = 0;
    FVector() = default;
    FVector(double x, double y, double z) : X(x), Y(y), Z(z) {}
    explicit FVector(double v) : X(v), Y(v), Z(v) {}
    FVector operator+(const FVector& o) const { return { X + o.X, Y + o.Y, Z + o.Z }; }
    FVector operator-(const FVector& o) const { return { X - o.X, Y - o.Y, Z - o.Z }; }
    FVector operator-() const { return { -X, -Y, -Z }; }
    FVector operator*(double s) const { return { X * s, Y * s, Z * s }; }
    FVector operator/(double s) const { return { X / s, Y / s, Z / s }; }
    double operator|(const FVector& o) const { return X * o.X + Y * o.Y + Z * o.Z; }
    FVector operator^(const FVector& o) const { return { Y * o.Z - Z * o.Y, Z * o.X - X * o.Z, X * o.Y - Y * o.X }; }
    double Size() const { return std::sqrt(X * X + Y * Y + Z * Z); }
    double SizeSquared() const { return X * X + Y * Y + Z * Z; }
    static double DistSquared(const FVector& a, const FVector& b) { return (a - b).SizeSquared(); }
    FVector GetSafeNormal() const { double s = SizeSquared(); if (s < 1e-8) return { 0, 0, 0 }; double inv = 1.0 / std::sqrt(s); return { X * inv, Y * inv, Z * inv }; }
};
inline FVector operator*(double s, const FVector& v) { return v * s; }
struct FVector3f;

struct FVector3f
{
    float X = 0, Y = 0, Z = 0;
    FVector3f() = default;
    FVector3f(float x, float y, float z) : X(x), Y(y), Z(z) {}
    explicit FVector3f(const FVector& v) : X((float)v.X), Y((float)v.Y), Z((float)v.Z) {}
};
inline FVector ToDouble(const FVector3f& v) { return FVector(v.X, v.Y, v.Z); }

struct FMath
{
    static double Tan(double v) { return std::tan(v); }
    static double Atan(double v) { return std::atan(v); }
    static double Atan2(double y, double x) { return std::atan2(y, x); }
    static double Acos(double v) { return std::acos(std::max(-1.0, std::min(1.0, v))); }
    static double Sin(double v) { return std::sin(v); }
    static double Sqrt(double v) { return std::sqrt(v); }
    static double Abs(double v) { return std::fabs(v); }
    static double Log2(double v) { return std::log2(v); }
    static double Exp2(double v) { return std::exp2(v); }
    static double FloorToDouble(double v) { return std::floor(v); }
    static int64 CeilToInt(double v) { return (int64)std::ceil(v); }
    template<class T> static T Square(T v) { return v * v; }
    template<class T> static T Max(T a, T b) { return a > b ? a : b; }
    template<class T> static T Min(T a, T b) { return a < b ? a : b; }
    template<class T> static T Clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
    static uint32 FloorLog2(uint32 v) { uint32 r = 0; while (v >>= 1) r++; return r; }
};

template<class T> struct TArray
{
    std::vector<T> V;
    void Add(const T& x) { V.push_back(x); }
    int32 Num() const { return (int32)V.size(); }
    T& operator[](int32 i) { return V[i]; }
    const T& operator[](int32 i) const { return V[i]; }
    void Reset(int32 n = 0) { V.clear(); V.reserve(n); }
    void Reserve(int32 n) { V.reserve(n); }
    T Pop() { T x = V.back(); V.pop_back(); return x; }
    typename std::vector<T>::iterator begin() { return V.begin(); }
    typename std::vector<T>::iterator end() { return V.end(); }
    typename std::vector<T>::const_iterator begin() const { return V.begin(); }
    typename std::vector<T>::const_iterator end() const { return V.end(); }
};

template<class T> struct TSetHash { size_t operator()(const T& k) const { return GetTypeHash(k); } };
template<class T> struct TSet
{
    std::unordered_set<T, TSetHash<T>> S;
    void Add(const T& x) { S.insert(x); }
    bool Contains(const T& x) const { return S.count(x) != 0; }
    int32 Num() const { return (int32)S.size(); }
    void Reset() { S.clear(); }
    auto begin() const { return S.begin(); }
    auto end() const { return S.end(); }
};

template<class T> struct TConstArrayView
{
    const T* Data = nullptr; int32 Count = 0;
    TConstArrayView() = default;
    TConstArrayView(const TArray<T>& A) : Data(A.V.data()), Count(A.Num()) {}
    const T& operator[](int32 i) const { return Data[i]; }
    int32 Num() const { return Count; }
};
