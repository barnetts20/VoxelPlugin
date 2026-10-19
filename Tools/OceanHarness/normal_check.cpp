// The pixel normal of VoxelOceanPixel (test waves, all resolved) against finite differences of the
// displaced surface S(p) = p + U(p) h(p) on the sphere, in double; and the float evaluation the
// shader does (sample point relative to the selection camera) against the double one.
#include <cmath>
#include <cstdio>
#include <random>
struct V { double x, y, z; };
static V operator+(V a, V b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static V operator-(V a, V b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static V operator*(V a, double s) { return { a.x * s, a.y * s, a.z * s }; }
static double dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V cross(V a, V b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static V norm(V a) { return a * (1.0 / std::sqrt(dot(a, a))); }
struct VF { float x, y, z; };
int main()
{
    const double R = 1.075e8, A = 100.0, L = 3000.0;
    const V Dirs[3] = { norm({ 0.97, 0.29, -0.19 }), norm({ -0.36, 0.89, 0.27 }), norm({ 0.18, -0.53, 0.83 }) };
    const double Scales[3] = { 1.0, 0.61, 0.37 };
    V K[3]; for (int i = 0; i < 3; i++) K[i] = Dirs[i] * (2.0 * M_PI / (L * Scales[i]));
    auto H = [&](V p) { double h = 0; for (int i = 0; i < 3; i++) h += A * std::sin(dot(K[i], p) + 0.3 * i); return h; };
    auto Grad = [&](V p) { V g{ 0, 0, 0 }; for (int i = 0; i < 3; i++) g = g + K[i] * (A * std::cos(dot(K[i], p) + 0.3 * i)); return g; };
    auto S = [&](V p) { return p + norm(p) * H(p); };
    std::mt19937_64 Rng(5);
    std::normal_distribution<double> G;
    double MaxDeg = 0, MaxFloatNear = 0, MaxFloatFar = 0, MaxBeyond = 0;
    for (int n = 0; n < 20000; n++)
    {
        const V Dir = norm({ G(Rng), G(Rng), G(Rng) });
        const V Cam = Dir * (R + 200.0 + 1e5 * std::fabs(G(Rng)));
        // A sample point on the sphere 1 m to 1000 km from the camera's ground point.
        V t = norm(cross(Dir, { G(Rng), G(Rng), G(Rng) }));
        const double Dist = std::pow(10.0, 2.0 + 6.0 * (n % 1000) / 1000.0);
        const V P = norm(Dir * R + t * Dist) * R;
        // Finite differences along two tangents, staying on the sphere.
        const V U = norm(P);
        const V T1 = norm(cross(U, { 0.3, 0.7, -0.2 })), T2 = cross(U, T1);
        const double E = 0.5;
        const V D1 = S(norm(P + T1 * E) * R) - S(norm(P - T1 * E) * R);
        const V D2 = S(norm(P + T2 * E) * R) - S(norm(P - T2 * E) * R);
        V NFd = norm(cross(D1, D2)); if (dot(NFd, U) < 0) NFd = NFd * -1.0;
        const V Gr = Grad(P);
        const V NAn = norm(U - (Gr - U * dot(U, Gr)));
        MaxDeg = std::fmax(MaxDeg, std::acos(std::fmin(1.0, dot(NFd, NAn))) * 180.0 / M_PI);
        // Float, as the shader: P and the centre relative to the camera, phases at the camera.
        const V Rel = P - Cam, Cen = V{ 0, 0, 0 } - Cam;
        float px = (float)Rel.x, py = (float)Rel.y, pz = (float)Rel.z;
        float cx = (float)Cen.x, cy = (float)Cen.y, cz = (float)Cen.z;
        float ux = px - cx, uy = py - cy, uz = pz - cz; float ul = std::sqrt(ux * ux + uy * uy + uz * uz); ux /= ul; uy /= ul; uz /= ul;
        float gx = 0, gy = 0, gz = 0;
        for (int i = 0; i < 3; i++)
        {
            const float kx = (float)K[i].x, ky = (float)K[i].y, kz = (float)K[i].z;
            const float ph = (float)std::fmod(dot(K[i], Cam) + 0.3 * i, 2.0 * M_PI);
            const float c = (float)A * std::cos(kx * px + ky * py + kz * pz + ph);
            gx += kx * c; gy += ky * c; gz += kz * c;
        }
        const float ug = ux * gx + uy * gy + uz * gz;
        float nx = ux - (gx - ux * ug), ny = uy - (gy - uy * ug), nz = uz - (gz - uz * ug);
        const float nl = std::sqrt(nx * nx + ny * ny + nz * nz); nx /= nl; ny /= nl; nz /= nl;
        const double Deg = std::acos(std::fmin(1.0, nx * NAn.x + ny * NAn.y + nz * NAn.z)) * 180.0 / M_PI;
        if (Dist < 1e4) MaxFloatNear = std::fmax(MaxFloatNear, Deg); else if (Dist < 5e5) MaxFloatFar = std::fmax(MaxFloatFar, Deg); else MaxBeyond = std::fmax(MaxBeyond, Deg);
    }
    std::printf("Analytic vs finite-difference normal: max %.4f deg\n", MaxDeg);
    std::printf("Float (shader) vs double normal: max %.4f deg within 100 m, %.4f deg 100 m - 5 km, %.3f deg 5 - 1000 km (waves faded there)\n", MaxFloatNear, MaxFloatFar, MaxBeyond);
}
