// O4: the wave hook's maths, checked in a harness.
// - The CPU test model (VoxelOceanWaves.cpp) in double: normal and Jacobian against finite
//   differences of the displaced sphere; velocity against a time difference.
// - The CPU query (VoxelOceanQuery::Run): convergence, and that the surface point it finds lies on
//   the query point's radial.
// - The shader's float VoxelOceanHeightAt (VoxelOceanWaves.ush) from a reference point near the
//   camera, against the CPU query: the waterline the underwater pass draws vs what a query says.
// - The underwater exit (VoxelUnderwater.ush) in float with choppy waves against a careful double
//   march, in Snell's window, and no false exits.
#include <cmath>
#include <cstdio>
#include <random>
#include <algorithm>
#include <vector>

template <typename T> struct V3 { T x, y, z; };
template <typename T> V3<T> operator+(V3<T> a, V3<T> b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
template <typename T> V3<T> operator-(V3<T> a, V3<T> b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
template <typename T> V3<T> operator*(V3<T> a, T s) { return { a.x * s, a.y * s, a.z * s }; }
template <typename T> T dot(V3<T> a, V3<T> b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
template <typename T> T len(V3<T> a) { return std::sqrt(dot(a, a)); }
template <typename T> V3<T> nrm(V3<T> a) { return a * (T(1) / len(a)); }
template <typename T> V3<T> cross(V3<T> a, V3<T> b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
typedef V3<double> D3; typedef V3<float> F3;
static F3 toF(D3 a) { return { (float)a.x, (float)a.y, (float)a.z }; }
static D3 toD(F3 a) { return { a.x, a.y, a.z }; }

static const double R = 107500000.0;
static const double Grav = 980.665;

// ---- CPU test model (double), as FVoxelOceanWavesTest ---------------------------------------
struct Model { D3 K[3]; double Om[3]; double A, C; };
static Model MakeModel(double A, double L, double Chop)
{
    const D3 Dirs[3] = { { 0.97, 0.29, -0.19 }, { -0.36, 0.89, 0.27 }, { 0.18, -0.53, 0.83 } };
    const double Sc[3] = { 1.0, 0.61, 0.37 };
    Model M; M.A = A; double SumK = 0;
    for (int w = 0; w < 3; w++) { double k = 2 * M_PI / (L * Sc[w]); M.K[w] = nrm(Dirs[w]) * k; M.Om[w] = std::sqrt(Grav * k); SumK += k; }
    M.C = std::clamp(std::min(Chop, 0.95 / (A * SumK)), 0.0, 1.0);
    return M;
}
struct Res { D3 Disp; double H; D3 N; D3 Vel; double Jac; D3 Grad[3]; };
static Res Eval(const Model& M, D3 P, double t)
{
    Res r{}; const D3 U = nrm(P); r.N = U; double h = 0; D3 Hz{0,0,0}, G{0,0,0}, T[3]; double Ws[3];
    for (int b = 0; b < 3; b++) r.Grad[b] = { 0, 0, 0 };
    r.Vel = { 0, 0, 0 };
    for (int w = 0; w < 3; w++)
    {
        D3 Kt3 = M.K[w] - U * dot(U, M.K[w]); double Kt = len(Kt3); T[w] = Kt > 1e-12 ? Kt3 * (1 / Kt) : D3{0,0,0};
        double th = dot(M.K[w], P) + M.Om[w] * t, S = std::sin(th), C = std::cos(th);
        h += M.A * S; Hz = Hz + T[w] * (M.C * M.A * C); G = G + Kt3 * (M.A * C); Ws[w] = M.C * M.A * Kt * S;
        r.Vel = r.Vel + U * (M.A * M.Om[w] * C) - T[w] * (M.C * M.A * M.Om[w] * S);
        double tb[3] = { T[w].x, T[w].y, T[w].z };
        for (int b = 0; b < 3; b++) r.Grad[b] = r.Grad[b] - T[w] * (Ws[w] * tb[b]);
    }
    r.H = h; r.Disp = U * h + Hz;
    double Tr = Ws[0] + Ws[1] + Ws[2], Tr2 = 0;
    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) { double d = dot(T[a], T[b]); Tr2 += Ws[a] * Ws[b] * d * d; }
    r.Jac = 1 - Tr + 0.5 * (Tr * Tr - Tr2);
    D3 SG = T[0] * (Ws[0] * dot(T[0], G)) + T[1] * (Ws[1] * dot(T[1], G)) + T[2] * (Ws[2] * dot(T[2], G));
    r.N = nrm(U * r.Jac - (G * (1 - Tr) + SG));
    return r;
}
// ---- CPU query (double), as VoxelOceanQuery::Run ----------------------------------------------
struct QRes { double Height; D3 X; Res W; bool Converged; int Steps; };
static void Basis(D3 U, D3& E1, D3& E2) { D3 a = std::fabs(U.x) < 0.9 ? D3{1,0,0} : D3{0,1,0}; E1 = nrm(cross(U, a)); E2 = cross(U, E1); }
static QRes Query(const Model& M, D3 Q, double t)
{
    QRes q{}; const D3 U = nrm(Q); const D3 Foot = U * R; D3 X = Foot; Res W = Eval(M, X, t); q.Converged = false;
    for (int s = 0; s < 8; s++)
    {
        D3 Hd = W.Disp - nrm(X) * W.H; D3 Rr = X + Hd - Foot; Rr = Rr - U * dot(U, Rr);
        if (len(Rr) < 1e-3) { q.Converged = true; q.Steps = s; break; }
        D3 E1, E2; Basis(U, E1, E2);
        auto Ap = [&](D3 v) { return D3{ dot(W.Grad[0], v), dot(W.Grad[1], v), dot(W.Grad[2], v) }; };
        D3 JE1 = Ap(E1), JE2 = Ap(E2);
        double A11 = 1 + dot(E1, JE1), A12 = dot(E1, JE2), A21 = dot(E2, JE1), A22 = 1 + dot(E2, JE2), Det = std::max(A11 * A22 - A12 * A21, 0.05);
        double R1 = dot(E1, Rr), R2 = dot(E2, Rr);
        D3 Dl = E1 * ((A22 * R1 - A12 * R2) / Det) + E2 * ((A11 * R2 - A21 * R1) / Det);
        X = nrm(X - Dl) * R; W = Eval(M, X, t); q.Steps = s + 1;
    }
    q.X = X; q.W = W; q.Height = len(Q) - R - W.H; return q;
}
// ---- Shader float (VoxelOceanWaves.ush) ------------------------------------------------------
struct FW { F3 k[3]; float ph[3]; float A, C; F3 Center; };
struct FOut { F3 Disp; float H; F3 N; float Jac; };
static float SphereHeightF(F3 D, float Rf, float Hc, F3 Up)
{
    const float A = Rf + Hc, Mu = dot(Up, D);
    const float Num = Hc * (2.0f * Rf + Hc) + 2.0f * A * Mu + dot(D, D);
    return Num / std::max(len(Up * A + D) + Rf, 1.0f);
}
static FOut WavesF(F3 P, const FW& W, F3& UpOut)
{
    FOut o{}; const F3 Up = nrm(P - W.Center); UpOut = Up; o.N = Up; o.Jac = 1; o.Disp = {0,0,0};
    if (W.A <= 0) return o;
    float h = 0; F3 Hz{0,0,0}, G{0,0,0}, T[3]; float Ws[3];
    for (int i = 0; i < 3; i++)
    {
        F3 Kt3 = W.k[i] - Up * dot(Up, W.k[i]); float Kt = len(Kt3); T[i] = Kt > 1e-12f ? Kt3 * (1.0f / Kt) : F3{0,0,0};
        float th = dot(W.k[i], P) + W.ph[i], S = std::sin(th), C = std::cos(th);
        h += W.A * S; Hz = Hz + T[i] * (W.C * W.A * C); G = G + Kt3 * (W.A * C); Ws[i] = W.C * W.A * Kt * S;
    }
    o.H = h; o.Disp = Up * h + Hz;
    float Tr = Ws[0] + Ws[1] + Ws[2], Tr2 = 0;
    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) { float d = dot(T[a], T[b]); Tr2 += Ws[a] * Ws[b] * d * d; }
    o.Jac = 1 - Tr + 0.5f * (Tr * Tr - Tr2);
    F3 SG = T[0] * (Ws[0] * dot(T[0], G)) + T[1] * (Ws[1] * dot(T[1], G)) + T[2] * (Ws[2] * dot(T[2], G));
    o.N = nrm(Up * o.Jac - (G * (1 - Tr) + SG));
    return o;
}
static float HeightAtF(F3 Q, float Rf, float Hc, const FW& W, FOut& Surf)
{
    const float Height = SphereHeightF(Q, Rf, Hc, nrm(W.Center * -1.0f));
    const F3 Up = nrm(Q - W.Center); const F3 Foot = Q - Up * Height; F3 X = Foot; F3 InUp;
    Surf = WavesF(X, W, InUp);
    if (W.A <= 0 || W.C <= 0) return Height - Surf.H;
    for (int s = 0; s < 3; s++)
    {
        F3 Hz = Surf.Disp - InUp * Surf.H; F3 Rr = X + Hz - Foot; Rr = Rr - Up * dot(Up, Rr);
        float Tr = 0; F3 SR{0,0,0};
        for (int i = 0; i < 3; i++)
        {
            F3 Kt3 = W.k[i] - InUp * dot(InUp, W.k[i]); float Kt = len(Kt3); F3 T = Kt > 1e-12f ? Kt3 * (1.0f / Kt) : F3{0,0,0};
            float Ws = W.C * W.A * Kt * std::sin(dot(W.k[i], X) + W.ph[i]); Tr += Ws; SR = SR + T * (Ws * dot(T, Rr));
        }
        const float Det = std::max(Surf.Jac, 0.05f);
        F3 Dl = (Rr * (1 - Tr) + SR) * (1.0f / Det);
        X = X - Dl; Surf = WavesF(X, W, InUp);
    }
    return Height - Surf.H;
}
static FW MakeFW(const Model& M, D3 Ref, double t)
{
    FW W; W.A = (float)M.A; W.C = (float)M.C; W.Center = toF(Ref * -1.0);
    for (int i = 0; i < 3; i++) { W.k[i] = toF(M.K[i]); W.ph[i] = (float)std::fmod(dot(M.K[i], Ref) + M.Om[i] * t, 2 * M_PI); }
    return W;
}
// ---- Shader float exit (VoxelUnderwater.ush) -------------------------------------------------
static float ShellExitF(float B, float H, float Rf, float Off)
{
    const float C = (H - Off) * (2.0f * Rf + H + Off); const float Root = std::sqrt(std::max(B * B - C, 0.0f));
    return B > 0.0f ? -C / std::max(B + Root, 1e-6f) : Root - B;
}
static float ExitF(F3 S, F3 Dir, float Rf, float Hc, const FW& W)
{
    FOut Surf; const float H = SphereHeightF(S, Rf, Hc, nrm(W.Center * -1.0f)); const float B = dot(S - W.Center, Dir);
    const float Shell = 3.0f * W.A + 1.0f;
    if (H < -Shell && B < 0) return 3e38f;
    const float Lower = H < -Shell ? ShellExitF(B, H, Rf, -Shell) : 0.0f;
    float Upper = std::max(ShellExitF(B, H, Rf, Shell), Lower);
    if (H >= -Shell && B < 0) { const float In = (H + Shell) * (2.0f * Rf + H - Shell); const float Disc = B * B - In; if (Disc >= 0) Upper = std::min(Upper, In / std::max(std::sqrt(Disc) - B, 1e-6f)); }
    float Below = Lower, Above = Upper; bool f = false;
    for (int s = 1; s <= 16; s++) { float X = s / 16.0f, T = Lower + (Upper - Lower) * X * X; if (HeightAtF(S + Dir * T, Rf, Hc, W, Surf) > 0) { Above = T; f = true; break; } Below = T; }
    if (!f) return 3e38f;
    for (int i = 0; i < 6; i++) { float Mid = 0.5f * (Below + Above); (HeightAtF(S + Dir * Mid, Rf, Hc, W, Surf) > 0 ? Above : Below) = Mid; }
    return 0.5f * (Below + Above);
}

int main()
{
    std::mt19937_64 Rng(11); std::normal_distribution<double> G; std::uniform_real_distribution<double> U01(0, 1);
    const double Chops[2] = { 0.5, 1.0 };
    for (double Chop : Chops)
    {
        const Model M = MakeModel(100.0, 3000.0, Chop);
        printf("== test model: A 100 cm, L 30 m, choppiness %.2f (after the clamp %.3f), max height %.0f cm, max horizontal %.0f cm\n", Chop, M.C, 3 * M.A, 3 * M.C * M.A);
        // 1. Normal, Jacobian, velocity against finite differences.
        double MaxNormalDeg = 0, MaxJacErr = 0, MaxVelErr = 0, MinJac = 1e9;
        for (int n = 0; n < 20000; n++)
        {
            const D3 P = nrm(D3{ G(Rng), G(Rng), G(Rng) }) * R; const double t = U01(Rng) * 1000;
            const Res r = Eval(M, P, t);
            D3 E1, E2; Basis(nrm(P), E1, E2); const double e = 0.5;
            auto Disp = [&](D3 p) { D3 q = nrm(p) * R; return q + Eval(M, q, t).Disp; };
            const D3 d1 = (Disp(P + E1 * e) - Disp(P - E1 * e)) * (1 / (2 * e)), d2 = (Disp(P + E2 * e) - Disp(P - E2 * e)) * (1 / (2 * e));
            const D3 Nn = nrm(cross(d1, d2));
            MaxNormalDeg = std::max(MaxNormalDeg, std::acos(std::min(1.0, dot(Nn, r.N))) * 180 / M_PI);
            const double JacN = dot(cross(d1, d2), nrm(P));
            MaxJacErr = std::max(MaxJacErr, std::fabs(JacN - r.Jac)); MinJac = std::min(MinJac, r.Jac);
            const double dt = 1e-4; const D3 v = (Eval(M, P, t + dt).Disp - Eval(M, P, t - dt).Disp) * (1 / (2 * dt));
            MaxVelErr = std::max(MaxVelErr, len(v - r.Vel));
        }
        printf("  normal vs finite differences: %.4f deg max; Jacobian %.2e max error (min Jacobian %.3f, no folds: %s); velocity %.2e cm/s max error\n",
            MaxNormalDeg, MaxJacErr, MinJac, MinJac > 0 ? "yes" : "NO", MaxVelErr);
        // 2. CPU query: convergence and the surface point on the radial.
        int NotConv = 0, MaxSteps = 0; double MaxRadialErr = 0, MaxLiftErr = 0;
        for (int n = 0; n < 50000; n++)
        {
            const D3 Dir = nrm(D3{ G(Rng), G(Rng), G(Rng) }); const D3 Q = Dir * (R + (U01(Rng) * 2 - 1) * 1000); const double t = U01(Rng) * 1000;
            const QRes q = Query(M, Q, t);
            if (!q.Converged) NotConv++; MaxSteps = std::max(MaxSteps, q.Steps);
            const D3 S = q.X + q.W.Disp;
            MaxRadialErr = std::max(MaxRadialErr, len(S - Dir * dot(S, Dir)));
            // Height against the true distance along the radial to the displaced point.
            MaxLiftErr = std::max(MaxLiftErr, std::fabs((len(Q) - len(S)) - q.Height));
        }
        printf("  query: %d of 50000 not settled, at most %d Newton steps; surface point off the radial by %.2e cm max; height vs |Q| - |S| %.2e cm max\n",
            NotConv, MaxSteps, MaxRadialErr, MaxLiftErr);
        // 3. Shader float height near a reference camera vs the CPU query.
        double MaxHErr = 0; long Mism = 0, Samples = 0;
        for (int n = 0; n < 100000; n++)
        {
            const D3 Dir0 = nrm(D3{ G(Rng), G(Rng), G(Rng) }); const D3 Ref = Dir0 * (R + (U01(Rng) * 2 - 1) * 3000.0); const double t = U01(Rng) * 1e5;
            const FW W = MakeFW(M, Ref, t); const float Hc = (float)(len(Ref) - R);
            const D3 Q = Ref + nrm(D3{ G(Rng), G(Rng), G(Rng) }) * (U01(Rng) * 10000.0);
            FOut Surf; const float hF = HeightAtF(toF(Q - Ref), (float)R, Hc, W, Surf); const QRes q = Query(M, Q, t);
            const double e = std::fabs(hF - q.Height); MaxHErr = std::max(MaxHErr, e); Samples++;
            if ((hF > 0) != (q.Height > 0) && std::fabs(q.Height) > 0.01) Mism++;
        }
        printf("  shader float height (points within 100 m of the reference) vs the CPU query: %.4f cm max error; %ld of %ld waterline decisions differ beyond 0.1 mm\n", MaxHErr, Mism, Samples);
        // 4. Underwater exit through the choppy surface, float vs a careful double march.
        long Window = 0, WindowOff = 0, False = 0, NoExit = 0; double MaxWin = 0;
        for (int n = 0; n < 20000; n++)
        {
            const D3 Dir0 = nrm(D3{ G(Rng), G(Rng), G(Rng) }); const double t = U01(Rng) * 1e4;
            D3 Cam = Dir0 * R; const QRes c0 = Query(M, Cam, t);
            Cam = Dir0 * (R + c0.W.H - 30 - U01(Rng) * 500);   // 0.3-5.3 m under the local surface
            const D3 Up = nrm(Cam); D3 E1, E2; Basis(Up, E1, E2);
            const double Elev = (41 + U01(Rng) * 49) * M_PI / 180, Az = U01(Rng) * 2 * M_PI;   // inside Snell's window, upward
            const D3 Ray = nrm(Up * std::sin(Elev) + (E1 * std::cos(Az) + E2 * std::sin(Az)) * std::cos(Elev));
            // double truth: march with steps bounded by the height (slope bound 1 + max slope)
            double T = 0, Truth = -1;
            for (int s = 0; s < 200000 && T < 200000; s++)
            {
                const double h = Query(M, Cam + Ray * T, t).Height;
                if (h > 0) { double lo = T - std::max(0.5, 0.0), hi = T; lo = std::max(0.0, T - 1.0);
                    for (int b = 0; b < 40; b++) { double mid = 0.5 * (lo + hi); (Query(M, Cam + Ray * mid, t).Height > 0 ? hi : lo) = mid; } Truth = hi; break; }
                T += std::max(0.5, -h / 2.5);
            }
            const FW W = MakeFW(M, Cam, t); const float Hc = (float)(len(Cam) - R);
            const float Ex = ExitF(F3{0,0,0}, toF(Ray), (float)R, Hc, W);
            if (Truth < 0) { if (Ex < 1e30f) False++; continue; }
            if (Ex > 1e30f) { NoExit++; continue; }
            Window++; const double e = std::fabs(Ex - Truth); MaxWin = std::max(MaxWin, e); if (e > 10) WindowOff++;
        }
        printf("  underwater exits in Snell's window: %ld rays, %ld off by more than 10 cm, max error %.2f cm; %ld missed exits; %ld false exits\n", Window, WindowOff, MaxWin, NoExit, False);
    }
}
