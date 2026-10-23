"""foam_tile.py - The ocean's foam texture set (stage W3g; Design/OceanWaves.md section 32): a tiling
2D set of four channels in the noise generator's .vxnt v3 layout (VoxelNoiseTiles.h), the plugin's
stand-in until the noise generator makes its own (Design/NoiseTiles2D_Addendum5.md).

THE IDEA. Foam isn't a pattern laid on the water: it's bubbles carried by the surface's own flow,
and the flow sorts them. Under a breaking sea the surface is turbulent and compressible: water wells
up in some places and sinks in others, and the floating bubbles gather along the lines where the
surface converges, while the upwellings open holes. Bubbles also burst, so the lines that last are
the ones the flow keeps feeding. The result is the lace: a network of strands of every weight round
holes of many sizes, stretched by the flow's shear. So each lace channel here is a particle
simulation of exactly that, on a periodic square:
  - particles (bubbles) are born anywhere and live an exponentially distributed time;
  - they move with a flow u = -grad(phi) + swirl * curl(psi) (+ a small random walk), phi and psi
    periodic random fields with a power-law spectrum between two cell sizes whose Fourier phases
    drift (the turbulence evolves while the foam gathers);
  - a bubble's weight grows with its age (a fresh bubble is sparse; foam forms as it gathers);
  - the channel is their density at the end, splatted and blurred by under a texel, as
    log(1 + density / median).
A strand's weight, its holes and its fraying all come out of the flow, not out of a cell lattice.

THE CHANNELS (each with its decode range, mean and deviation in the header; kinds 10 to 13):
  R foam_lace      the lace: a large-scale network (holes 48 to 1024 texels) with a fine one (16
                   to 80) inside its strands, and the bubbles' grain added (so the threshold's
                   edges break up into bubbles, the interior unchanged)
  G foam_grain     the bubbles: band-passed noise about 3 texels across (relief, edge breakup)
  B foam_whitecap  the whitecap: billowy turbulence (four octaves of |band noise| from 48 texels),
                   with the grain
  A foam_streak    the streaks: the lace's simulation with its flow stretched 4x along x (the
                   downwind axis in the plugin's chart frame), with the grain
The plugin thresholds each against its own quantile for the foam's density (a mean-preserving
cover) and reads its relief from the grain.

Usage: python3 -I foam_tile.py OUT.vxnt [N=1024] [SEED=1] [--previews DIR]
Needs numpy (and Pillow for the previews). N 1024 takes about 40 minutes; 256 a couple.
"""
import os
import struct
import sys

import numpy as np

KIND_FOAM_LACE = 10
KIND_FOAM_GRAIN = 11
KIND_FOAM_WHITECAP = 12
KIND_FOAM_STREAK = 13


# --- Periodic fields --------------------------------------------------------------------------

def wavenumbers(N, aniso=1.0):
    k = np.fft.fftfreq(N) * N
    KX, KY = np.meshgrid(k, k)
    return np.sqrt((KX * aniso) ** 2 + KY ** 2)


def evolving(N, rng, cell_min, cell_max, slope, aniso):
    """A periodic field (cells cell_min .. cell_max texels, power ~ k^slope) whose phases drift:
    returns f(t), unit RMS at t = 0."""
    K = wavenumbers(N, aniso)
    kmin, kmax = N / cell_max, N / cell_min
    amp = np.where((K >= kmin) & (K <= kmax), np.power(np.maximum(K, 1e-9), slope * 0.5), 0.0)
    ph = rng.uniform(0, 2 * np.pi, (N, N))
    omega = rng.normal(0, 1, (N, N))
    norm = np.real(np.fft.ifft2(amp * np.exp(1j * ph))).std() + 1e-12

    def at(t):
        return np.real(np.fft.ifft2(amp * np.exp(1j * (ph + omega * t)))) / norm
    return at


def grad(f):
    N = f.shape[0]
    k = np.fft.fftfreq(N) * 2 * np.pi
    KX, KY = np.meshgrid(k, k)
    F = np.fft.fft2(f)
    return np.real(np.fft.ifft2(1j * KX * F)), np.real(np.fft.ifft2(1j * KY * F))


def blur(f, sigma):
    N = f.shape[0]
    k = np.fft.fftfreq(N) * 2 * np.pi
    KX, KY = np.meshgrid(k, k)
    return np.real(np.fft.ifft2(np.fft.fft2(f) * np.exp(-0.5 * sigma * sigma * (KX * KX + KY * KY))))


def band_noise(N, rng, cell):
    """Periodic noise around one cell size (texels), unit RMS."""
    K = wavenumbers(N)
    k0 = N / cell
    amp = np.exp(-0.5 * ((K - k0) / (0.5 * k0)) ** 2)
    f = np.real(np.fft.ifft2(amp * np.exp(1j * rng.uniform(0, 2 * np.pi, (N, N)))))
    return f / f.std()


def billow(N, rng, cell, octaves=4, gain=0.55):
    out = np.zeros((N, N))
    a = 1.0
    for o in range(octaves):
        out += a * np.abs(band_noise(N, rng, cell / 2 ** o))
        a *= gain
    return (out - out.mean()) / out.std()


# --- The foam's simulation ----------------------------------------------------------------------

def bilinear(f, x, y):
    N = f.shape[0]
    x0 = np.floor(x).astype(np.int64)
    y0 = np.floor(y).astype(np.int64)
    fx = x - x0
    fy = y - y0
    x0 %= N
    y0 %= N
    x1 = (x0 + 1) % N
    y1 = (y0 + 1) % N
    return (f[y0, x0] * (1 - fx) + f[y0, x1] * fx) * (1 - fy) + (f[y1, x0] * (1 - fx) + f[y1, x1] * fx) * fy


def splat(N, x, y, w):
    g = np.zeros(N * N)
    x0 = np.floor(x).astype(np.int64)
    y0 = np.floor(y).astype(np.int64)
    fx = x - x0
    fy = y - y0
    x0 %= N
    y0 %= N
    x1 = (x0 + 1) % N
    y1 = (y0 + 1) % N
    for yy, xx, ww in ((y0, x0, (1 - fx) * (1 - fy)), (y0, x1, fx * (1 - fy)), (y1, x0, (1 - fx) * fy), (y1, x1, fx * fy)):
        g += np.bincount(yy * N + xx, weights=w * ww, minlength=N * N)
    return g.reshape(N, N)


def gather(N, seed, cell_min, cell_max, aniso=1.0, swirl=1.0, life=160.0, drift=0.05, slope=-3.0, per_texel=3, jitter=0.25, width=0.7):
    """The bubbles' density after they've gathered on the flow's convergence lines."""
    rng = np.random.default_rng(seed)
    phi = evolving(N, rng, cell_min, cell_max, slope, aniso)
    psi = evolving(N, rng, cell_min, cell_max, slope, aniso)
    speed = 2.4 * cell_min / life
    M = per_texel * N * N
    x = rng.uniform(0, N, M)
    y = rng.uniform(0, N, M)
    age = rng.uniform(0, life, M)
    lives = rng.exponential(life, M)
    steps = int(3 * life)
    acc = np.zeros((N, N))
    count = 0
    for s in range(steps):
        if s % 2 == 0:
            t = s * drift
            gx, gy = grad(phi(t))
            hx, hy = grad(psi(t))
            g = np.sqrt((gx ** 2 + gy ** 2).mean()) + 1e-12
            h = np.sqrt((hx ** 2 + hy ** 2).mean()) + 1e-12
            ux = (-gx / g + swirl * hy / h) * speed
            uy = (-gy / g - swirl * hx / h) * speed
        vx = bilinear(ux, x, y)
        vy = bilinear(uy, x, y)
        mx = x + 0.5 * vx
        my = y + 0.5 * vy
        x = (x + bilinear(ux, mx, my) + rng.normal(0, jitter, M)) % N
        y = (y + bilinear(uy, mx, my) + rng.normal(0, jitter, M)) % N
        age += 1.0
        dead = age > lives
        k = int(dead.sum())
        if k:
            x[dead] = rng.uniform(0, N, k)
            y[dead] = rng.uniform(0, N, k)
            age[dead] = 0.0
            lives[dead] = rng.exponential(life, k)
        if s >= steps - 4:
            acc += splat(N, x, y, 1.0 - np.exp(-age / (0.3 * life)))
            count += 1
    d = np.maximum(blur(acc / count / per_texel, width), 0.0)
    v = np.log1p(d / np.median(d))
    return (v - v.mean()) / v.std()


def generate(N, seed, log=print):
    """The four channels (float, unit deviation) at size N. The cell sizes are given at 1024."""
    s = N / 1024.0
    rng = np.random.default_rng(seed * 7919 + 13)
    log("grain")
    grain = band_noise(N, rng, max(3.0 * s, 2.0))
    log("lace, large")
    big = gather(N, seed * 4 + 0, 48 * s, 1024 * s, life=160 * s, slope=-3.0)
    log("lace, fine")
    fine = gather(N, seed * 4 + 1, max(16 * s, 6), 80 * s, life=60 * s, slope=-1.0, swirl=1.2, drift=0.08)
    lace = big + 0.6 * fine * np.sqrt(np.clip((big - big.min()) / (big.max() - big.min()), 0, 1))
    lace = (lace - lace.mean()) / lace.std() + 0.35 * grain
    log("streaks")
    streak = gather(N, seed * 4 + 2, 32 * s, 512 * s, aniso=4.0, swirl=0.6, life=160 * s, slope=-2.0) + 0.35 * grain
    log("whitecap")
    cap = billow(N, rng, 48 * s) + 0.35 * grain
    chans = [lace, grain, cap, streak]
    return [(c - c.mean()) / c.std() for c in chans]


# --- The .vxnt v3 file (VoxelNoiseTiles.h's reader) ----------------------------------------------

def mip_chain(f):
    out = [f]
    while out[-1].shape[0] > 1:
        a = out[-1]
        out.append(0.25 * (a[0::2, 0::2] + a[1::2, 0::2] + a[0::2, 1::2] + a[1::2, 1::2]))
    return out


def write_vxnt(path, N, seed, chans, name="foam", look="lace from gathered bubbles"):
    mips = [mip_chain(c.astype(np.float64)) for c in chans]
    M = len(mips[0])
    ranges = [(float(c.min()), float(c.max())) for c in chans]
    header = 128 + 4 * 64 + 16 * M
    out = bytearray(header)
    out[0:4] = b"VXNT"
    struct.pack_into("<IIIIIII", out, 4, 3, N, M, 4, 0, seed, header)
    out[32:32 + len(name)] = name.encode()
    out[64:64 + len(look)] = look.encode()
    kinds = [KIND_FOAM_LACE, KIND_FOAM_GRAIN, KIND_FOAM_WHITECAP, KIND_FOAM_STREAK]
    for c in range(4):
        lo, hi = ranges[c]
        struct.pack_into("<IIIIffffiffff", out, 128 + 64 * c, kinds[c], seed * 4 + c, 1, 1, 2.0, 0.5, 0.0, 0.0, -1,
                         lo, hi, float(chans[c].mean()), float(chans[c].std()))
    offset = header
    for m in range(M):
        struct.pack_into("<IIQ", out, 128 + 256 + 16 * m, N >> m, 0, offset)
        offset += (N >> m) ** 2 * 8
    body = []
    for m in range(M):
        q = [np.clip(np.round((mips[c][m] - ranges[c][0]) / (ranges[c][1] - ranges[c][0]) * 65535.0), 0, 65535).astype("<u2") for c in range(4)]
        body.append(np.stack(q, axis=-1).tobytes())
    with open(path, "wb") as f:
        f.write(bytes(out))
        for b in body:
            f.write(b)


def previews(dirname, chans):
    from PIL import Image
    os.makedirs(dirname, exist_ok=True)
    for c, name in enumerate(["lace", "grain", "whitecap", "streak"]):
        v = chans[c]
        lo, hi = np.quantile(v, 0.005), np.quantile(v, 0.995)
        Image.fromarray((np.clip((v - lo) / (hi - lo), 0, 1) * 255).astype(np.uint8)).save(os.path.join(dirname, f"foam_{name}.png"))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = args[0]
    N = int(args[1]) if len(args) > 1 else 1024
    seed = int(args[2]) if len(args) > 2 else 1
    chans = generate(N, seed, log=lambda s: print(s, flush=True))
    write_vxnt(out, N, seed, chans)
    if "--previews" in sys.argv:
        previews(sys.argv[sys.argv.index("--previews") + 1], chans)
    print("wrote", out)
