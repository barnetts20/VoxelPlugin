// CoreMinimal.h - A stand-in for the few engine types the FFT wave model uses
// (VoxelOceanWaves.h, VoxelOceanWavesFFT.h/.cpp), so hook_check.cpp compiles the model's own
// source outside the engine. Not the engine's behaviour beyond what the model relies on.
#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <vector>

typedef uint8_t uint8; typedef int32_t int32; typedef uint32_t uint32; typedef int64_t int64; typedef uint64_t uint64;
#define VOXELPLUGIN_API
#define USTRUCT(...)
#define UCLASS(...)
#define UPROPERTY(...)
#define UFUNCTION(...)
#define GENERATED_BODY()
#define check(x) assert(x)
#define ENUM_CLASS_FLAGS(E) \
    inline E operator|(E a, E b) { return E(uint32(a) | uint32(b)); } \
    inline E operator&(E a, E b) { return E(uint32(a) & uint32(b)); }
template<class E> bool EnumHasAnyFlags(E a, E b) { return (uint32(a) & uint32(b)) != 0; }
#define UE_DOUBLE_PI 3.14159265358979323846

struct FMath
{
    template<class T> static T Max(T a, T b) { return a > b ? a : b; }
    template<class T> static T Min(T a, T b) { return a < b ? a : b; }
    template<class T> static T Clamp(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
    template<class T> static T Square(T v) { return v * v; }
    static double Sqrt(double v) { return std::sqrt(v); }
    static double Sin(double v) { return std::sin(v); }
    static double Cos(double v) { return std::cos(v); }
    static double FloorToDouble(double v) { return std::floor(v); }
    static double Abs(double v) { return std::fabs(v); }
};

struct FVector
{
    double X = 0, Y = 0, Z = 0;
    FVector() = default;
    FVector(double x, double y, double z) : X(x), Y(y), Z(z) {}
    explicit FVector(double s) : X(s), Y(s), Z(s) {}
    FVector operator+(const FVector& o) const { return { X + o.X, Y + o.Y, Z + o.Z }; }
    FVector operator-(const FVector& o) const { return { X - o.X, Y - o.Y, Z - o.Z }; }
    FVector operator-() const { return { -X, -Y, -Z }; }
    FVector operator*(double s) const { return { X * s, Y * s, Z * s }; }
    FVector operator/(double s) const { return { X / s, Y / s, Z / s }; }
    FVector& operator+=(const FVector& o) { X += o.X; Y += o.Y; Z += o.Z; return *this; }
    FVector& operator-=(const FVector& o) { X -= o.X; Y -= o.Y; Z -= o.Z; return *this; }
    double operator|(const FVector& o) const { return X * o.X + Y * o.Y + Z * o.Z; }
    FVector operator^(const FVector& o) const { return { Y * o.Z - Z * o.Y, Z * o.X - X * o.Z, X * o.Y - Y * o.X }; }
    double& operator[](int i) { return i == 0 ? X : (i == 1 ? Y : Z); }
    double operator[](int i) const { return i == 0 ? X : (i == 1 ? Y : Z); }
    double Size() const { return std::sqrt(X * X + Y * Y + Z * Z); }
    FVector GetSafeNormal() const { const double L = Size(); return L > 1e-30 ? *this / L : FVector(); }
    static const FVector ZeroVector;
    static const FVector UpVector;
};
inline const FVector FVector::ZeroVector(0, 0, 0);
inline const FVector FVector::UpVector(0, 0, 1);
inline FVector operator*(double s, const FVector& v) { return v * s; }
typedef FVector FVector3d;

struct FVector4f
{
    float X = 0, Y = 0, Z = 0, W = 0;
    FVector4f() = default;
    FVector4f(float x, float y, float z, float w) : X(x), Y(y), Z(z), W(w) {}
};

struct FQuat
{
    double X = 0, Y = 0, Z = 0, W = 1;
    FQuat() = default;
    FQuat(double x, double y, double z, double w) : X(x), Y(y), Z(z), W(w) {}
    FVector RotateVector(const FVector& V) const
    {
        const FVector Q(X, Y, Z);
        const FVector T = (Q ^ V) * 2.0;
        return V + T * W + (Q ^ T);
    }
    FVector UnrotateVector(const FVector& V) const { return FQuat(-X, -Y, -Z, W).RotateVector(V); }
};

struct FTransform
{
    FQuat Rotation;
    FVector Translation;
    FTransform() = default;
    FTransform(const FQuat& R, const FVector& T) : Rotation(R), Translation(T) {}
    FQuat GetRotation() const { return Rotation; }
    FVector GetLocation() const { return Translation; }
    FVector TransformVectorNoScale(const FVector& V) const { return Rotation.RotateVector(V); }
    FVector TransformPositionNoScale(const FVector& V) const { return Rotation.RotateVector(V) + Translation; }
    FVector InverseTransformPositionNoScale(const FVector& V) const { return Rotation.UnrotateVector(V - Translation); }
    static const FTransform Identity;
};
inline const FTransform FTransform::Identity;

template<class T> struct TArray
{
    std::vector<T> V;
    TArray() = default;
    int32 Num() const { return int32(V.size()); }
    void Add(const T& x) { V.push_back(x); }
    void RemoveAt(int32 i) { V.erase(V.begin() + i); }
    void SetNum(int32 n) { V.resize(size_t(n)); }
    void SetNumZeroed(int32 n) { V.assign(size_t(n), T()); }
    void Reset() { V.clear(); }
    void Empty() { V.clear(); }
    void Reserve(int32 n) { V.reserve(size_t(n)); }
    bool IsEmpty() const { return V.empty(); }
    T& Last() { return V.back(); }
    const T& Last() const { return V.back(); }
    T& operator[](int32 i) { return V[size_t(i)]; }
    const T& operator[](int32 i) const { return V[size_t(i)]; }
    T* GetData() { return V.data(); }
    const T* GetData() const { return V.data(); }
    auto begin() { return V.begin(); } auto end() { return V.end(); }
    auto begin() const { return V.begin(); } auto end() const { return V.end(); }
};

template<class T> struct TArrayView
{
    T* Data = nullptr; int32 Count = 0;
    TArrayView() = default;
    TArrayView(T* D, int32 N) : Data(D), Count(N) {}
    template<class U> TArrayView(TArray<U>& A) : Data(A.GetData()), Count(A.Num()) {}
    template<class U> TArrayView(const TArray<U>& A) : Data(A.GetData()), Count(A.Num()) {}
    int32 Num() const { return Count; }
    T& operator[](int32 i) const { return Data[i]; }
    T* GetData() const { return Data; }
};
template<class T> using TConstArrayView = TArrayView<const T>;

enum class ESPMode { NotThreadSafe, ThreadSafe };
template<class T, ESPMode M = ESPMode::ThreadSafe> struct TSharedPtr
{
    std::shared_ptr<T> P;
    TSharedPtr() = default;
    TSharedPtr(std::shared_ptr<T> p) : P(std::move(p)) {}
    template<class U, ESPMode N> TSharedPtr(const TSharedPtr<U, N>& o) : P(o.P) {}
    bool IsValid() const { return bool(P); }
    T* operator->() const { return P.get(); }
    T& operator*() const { return *P; }
    T* Get() const { return P.get(); }
    void Reset() { P.reset(); }
};
template<class T, ESPMode M = ESPMode::ThreadSafe, class... A> TSharedPtr<T, M> MakeShared(A&&... a) { return TSharedPtr<T, M>(std::make_shared<T>(std::forward<A>(a)...)); }

struct FCriticalSection { std::mutex M; };
struct FScopeLock
{
    FCriticalSection* C;
    explicit FScopeLock(FCriticalSection* c) : C(c) { C->M.lock(); }
    ~FScopeLock() { C->M.unlock(); }
};
