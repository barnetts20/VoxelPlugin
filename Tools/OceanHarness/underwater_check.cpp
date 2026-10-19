// VoxelUnderwater.ush's float math (heights from the selection camera, the stable sphere height,
// the exit root and its Newton refinement against the test waves) against double references.
#include <cmath>
#include <cstdio>
#include <random>
#include <algorithm>
struct D3 { double x, y, z; };
static D3 operator+(D3 a, D3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static D3 operator-(D3 a, D3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static D3 operator*(D3 a, double s) { return { a.x * s, a.y * s, a.z * s }; }
static double dot(D3 a, D3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static D3 nrm(D3 a) { return a * (1.0 / std::sqrt(dot(a, a))); }
struct F3 { float x, y, z; };
static F3 operator+(F3 a, F3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static F3 operator-(F3 a, F3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static F3 operator*(F3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
static float dotf(F3 a, F3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static float lenf(F3 a) { return std::sqrt(dotf(a, a)); }
static F3 nrmf(F3 a) { return a * (1.0f / lenf(a)); }
static F3 toF(D3 a) { return { (float)a.x, (float)a.y, (float)a.z }; }

const double R = 1.075e8, Amp = 100.0;
D3 K[3]; double Phase0[3];
// Shader float functions.
static float SphereHeightF(F3 D, float Rf, float H, F3 Up)
{
    const float A = Rf + H, Mu = dotf(Up, D);
    const float Num = H * (2.0f * Rf + H) + 2.0f * A * Mu + dotf(D, D);
    return Num / std::max(lenf(Up * A + D) + Rf, 1.0f);
}
struct FWaves { F3 k[3]; float ph[3]; float amp; };
static float WaveHF(F3 P, const FWaves& W) { float h = 0; for (int i = 0; i < 3; i++) h += std::sin(dotf(W.k[i], P) + W.ph[i]); return W.amp * h; }
static F3 WaveGF(F3 P, const FWaves& W) { F3 g{ 0, 0, 0 }; for (int i = 0; i < 3; i++) g = g + W.k[i] * (W.amp * std::cos(dotf(W.k[i], P) + W.ph[i])); return g; }
static float HeightAtF(F3 D, float Rf, float H, F3 Up, F3 C, const FWaves& W)
{
    const float h = SphereHeightF(D, Rf, H, Up);
    if (W.amp <= 0) return h;
    const F3 Foot = D - nrmf(D - C) * h;
    return h - WaveHF(Foot, W);
}
static float ShellExitF(float B, float H, float Rf, float Off)
{
    const float C = (H - Off) * (2.0f * Rf + H + Off);
    const float Root = std::sqrt(std::max(B * B - C, 0.0f));
    return B > 0 ? -C / std::max(B + Root, 1e-6f) : Root - B;
}
static float ExitF(F3 S, F3 Dir, float Rf, float H, F3 Up, F3 C, const FWaves& W)
{
    const float h = SphereHeightF(S, Rf, H, Up);
    const float B = dotf(S - C, Dir);
    if (W.amp <= 0) return ShellExitF(B, h, Rf, 0.0f);
    const float MaxH = 3.0f * W.amp + 1.0f;
    if (h < -MaxH && B < 0) return 3.0e38f;
    const float Lower = h < -MaxH ? ShellExitF(B, h, Rf, -MaxH) : 0.0f;
    float Upper = std::max(ShellExitF(B, h, Rf, MaxH), Lower);
    // Heading down from inside the shell: the search ends where the ray dives below it.
    if (h >= -MaxH && B < 0)
    {
        const float Cin = (h + MaxH) * (2.0f * Rf + h - MaxH);
        const float Disc = B * B - Cin;
        if (Disc >= 0) Upper = std::min(Upper, Cin / std::max(-B + std::sqrt(Disc), 1e-6f));
    }
    float Below = Lower, Above = Upper; bool bFound = false;
    for (int s = 1; s <= 16; s++)
    {
        const float X = s / 16.0f, T = Lower + (Upper - Lower) * X * X;
        if (HeightAtF(S + Dir * T, Rf, H, Up, C, W) > 0) { Above = T; bFound = true; break; }
        Below = T;
    }
    if (!bFound) return 3.0e38f;
    for (int i = 0; i < 6; i++) { const float M = 0.5f * (Below + Above); (HeightAtF(S + Dir * M, Rf, H, Up, C, W) > 0 ? Above : Below) = M; }
    return 0.5f * (Below + Above);
}
// Double references (planet frame, absolute).
static double HeightAtD(D3 P, bool bWaves)
{
    const double r = std::sqrt(dot(P, P)), h = r - R;
    if (!bWaves) return h;
    const D3 Foot = P * (R / r);
    double w = 0; for (int i = 0; i < 3; i++) w += Amp * std::sin(dot(K[i], Foot) + Phase0[i]);
    return h - w;
}
static double ExitD(D3 S, D3 Dir, bool bWaves, double TMax)
{
    // March and bisect the first sign change.
    double Prev = 0, Step = 1.0;
    for (double t = 1.0; t < TMax; t += Step, Step = std::min(Step * 1.05, 10.0))
    {
        if (HeightAtD(S + Dir * t, bWaves) > 0)
        {
            double lo = Prev, hi = t;
            for (int i = 0; i < 60; i++) { const double m = 0.5 * (lo + hi); (HeightAtD(S + Dir * m, bWaves) > 0 ? hi : lo) = m; }
            return 0.5 * (lo + hi);
        }
        Prev = t;
    }
    return -1;
}
int main()
{
    const D3 Dirs[3] = { nrm({ 0.97, 0.29, -0.19 }), nrm({ -0.36, 0.89, 0.27 }), nrm({ 0.18, -0.53, 0.83 }) };
    const double Scales[3] = { 1.0, 0.61, 0.37 };
    for (int i = 0; i < 3; i++) { K[i] = Dirs[i] * (2.0 * M_PI / (3000.0 * Scales[i])); Phase0[i] = 0.4 * i; }
    std::mt19937_64 Rng(9);
    std::normal_distribution<double> G;
    std::uniform_real_distribution<double> U01(0, 1);
    for (int bWaves = 0; bWaves < 2; bWaves++)
    {
        double MaxNearErr = 0, MaxExitErrRel = 0, MaxExitErrAbs = 0, MaxSteep = 0;
        long FalseExits = 0, Mismatch = 0, Samples = 0, Exits = 0, MissedExits = 0, Within10 = 0, Steep = 0, LaterRoot = 0, Window = 0, WindowOff = 0; double MaxWindow = 0;
        for (int n = 0; n < 200000; n++)
        {
            const D3 Dir0 = nrm({ G(Rng), G(Rng), G(Rng) });
            // Selection camera within +-30 m of the sea, the view's camera within 2 m of it.
            const D3 Sel = Dir0 * (R + (U01(Rng) * 2 - 1) * 3000.0);
            const D3 Cam = Sel;   // O3b: the reference point is the view camera itself
            const D3 Ray = nrm({ G(Rng), G(Rng), G(Rng) });
            // Near point 10-15 cm along the ray.
            const D3 Near = Cam + Ray * (10.0 + 5.0 * U01(Rng));
            // Float inputs as the material builds them.
            const F3 Cf = toF(D3{ 0, 0, 0 } - Sel);
            const float Hsel = (float)(std::sqrt(dot(Sel, Sel)) - R);
            const F3 Upf = nrmf(Cf * -1.0f);
            FWaves W; W.amp = bWaves ? (float)Amp : 0.0f;
            for (int i = 0; i < 3; i++) { W.k[i] = toF(K[i]); W.ph[i] = (float)std::fmod(dot(K[i], Sel) + Phase0[i], 2 * M_PI); }
            const F3 NearF = toF(Cam - Sel) + toF(Ray) * (float)dot(Near - Cam, Ray);
            const float hF = HeightAtF(NearF, (float)R, Hsel, Upf, Cf, W);
            const double hD = HeightAtD(Near, bWaves);
            MaxNearErr = std::max(MaxNearErr, std::fabs(hF - hD));
            Samples++;
            if (std::fabs(hD) > 0.01 && ((hF > 0) != (hD > 0))) Mismatch++;
            // Exit from below for underwater near points, up to 2 km.
            if (hD < 0)
            {
                const double TD = ExitD(Near, Ray, bWaves, 2.0e5);
                const float TF = ExitF(NearF, toF(Ray), (float)R, Hsel, Upf, Cf, W);
                if (!(TD > 0) && TF < 2.0e5f) FalseExits++;
                if (TD > 0 && TD < 2.0e5)
                {
                    Exits++;
                    if (TF < 0 || TF > 1e30f) { MissedExits++; continue; }
                    // Compare where the exit lands: its height in double should be ~0.
                    const double Err = std::fabs(TF - TD);
                    MaxExitErrAbs = std::max(MaxExitErrAbs, Err);
                    MaxExitErrRel = std::max(MaxExitErrRel, Err / TD);
                    Within10 += Err < 10.0;
                    LaterRoot += TF > TD + 10.0;
                    // Steep: more than 20 degrees above the horizon.
                    if (dot(Ray, nrm(Near)) > 0.66) { Window++; WindowOff += Err > 10.0; MaxWindow = std::max(MaxWindow, Err); }
                    if (dot(Ray, nrm(Near)) > 0.34) { Steep++;  MaxSteep = std::max(MaxSteep, Err); }
                }
            }
        }
        std::printf("%s: %ld near-plane points: max height error %.4f mm, %ld waterline sign mismatches beyond 0.1 mm\n"
            "  %ld exits within 2 km: %.2f%% within 10 cm of the first crossing, %ld at a later crossing (grazing rays), max error %.3g cm; %ld steep rays (> 20 deg up): max error %.3f cm; %ld in Snell's window (> 41 deg up): %ld off by more than 10 cm, max %.2f cm\n  %ld rays with no exit within 2 km reported as exiting (want 0), %ld exits missed\n",
            bWaves ? "with 1 m test waves" : "sphere", Samples, MaxNearErr * 10.0, Mismatch, Exits, 100.0 * Within10 / std::max(1L, Exits), LaterRoot, MaxExitErrAbs, Steep, MaxSteep, Window, WindowOff, MaxWindow, FalseExits, MissedExits);
    }
}
