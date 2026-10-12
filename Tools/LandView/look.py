"""Look test renders and measures (Design/SurfaceLook.md, T0) for bakes dumped by
Tools/LandSolveHarness/look_harness.cpp.

    python look.py <tag> [--regions regions.json] [--pick]

Writes <tag>_planet.png (equirectangular, hill-shaded, drainage and lakes), <tag>_<region>.png
(native bake cells of a face window), and prints the measures:
  - channels: of the cells draining more than 64 cells, how often a cell's receiver continues
    in the same of the 8 directions (straight grooves near 1; a branching, wandering network
    near 0.4), and the mean straight run length (cells); of the cells draining more than 256,
    the sinuosity of their next 16 steps, and the share of straight runs of 6 steps or more
    (walk_measures); channels are picked by the D8 area from the receivers (d8_area), since
    the dumped drainage is spread over hillslopes from T1b;
  - lakes: count, the largest (km^2), and their share of the land;
  - summits: local maxima over 2,000 m within 7 x 7 cells, their height spread (p10, p50, p90,
    and p90 - p10 over p50), and the share of high ground (over 2,500 m) that is nearly flat
    (slope under 0.08): plateaus.
--pick chooses the windows from this bake and writes them to the regions file (default
regions.json); otherwise the file's windows are used, so variants are compared on the same
ground."""
import json, sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

FACE_U = [1, 2, 2, 0, 0, 1]
FACE_V = [2, 1, 0, 2, 1, 0]
NONE = 0xffffffff

_land = LinearSegmentedColormap.from_list("land", [(0, (0.33, 0.52, 0.30)), (0.12, (0.47, 0.60, 0.36)), (0.28, (0.70, 0.66, 0.45)),
                                                   (0.48, (0.58, 0.46, 0.33)), (0.7, (0.52, 0.5, 0.48)), (1, (0.96, 0.96, 0.97))])
_sea = LinearSegmentedColormap.from_list("sea", [(0, (0.03, 0.08, 0.25)), (0.6, (0.1, 0.25, 0.5)), (0.97, (0.25, 0.5, 0.7)), (1, (0.45, 0.7, 0.8))])


def load(tag):
    meta = json.load(open(tag + ".json"))
    n, pad = meta["n"], meta["pad"]
    side = n + 2 * pad
    shape = (6, side, side)
    f = {k: np.fromfile(f"{tag}_{k}.f32", np.float32).reshape(shape) for k in ("height", "flow", "water", "plates", "up")}
    f["rec"] = np.fromfile(f"{tag}_rec.u32", np.uint32).reshape(shape)
    f["attr"] = np.fromfile(f"{tag}_attr.u32", np.uint32).reshape(shape)
    return meta, f


def colour(h):
    return np.where((h > 0)[..., None], _land(np.clip(h / 6000, 0, 1))[..., :3], _sea(np.clip(1 + h / 6000, 0, 1))[..., :3])


def shade(h, px, z=1.0):
    gy, gx = np.gradient(np.maximum(h, 0), px)
    slope = np.arctan(z * np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    s = 0
    for az, w in ((315, 0.6), (225, 0.2), (45, 0.2)):
        s = s + w * np.clip(np.sin(np.radians(40)) * np.cos(slope) + np.cos(np.radians(40)) * np.sin(slope) * np.cos(np.radians(az) - aspect), 0, 1)
    return s


def overlay(img, h, flow, water, cell_area_m2):
    out = img.copy()
    area_cells = np.exp2(flow) / cell_area_m2
    land = h > 0
    for cells, col, a in ((64, (0.45, 0.65, 0.95), 0.5), (512, (0.2, 0.45, 0.95), 0.8), (4096, (0.05, 0.25, 0.85), 1.0)):
        m = land & (area_cells > cells)
        out[m] = out[m] * (1 - a) + np.array(col) * a
    out[water > -1e30] = (0.0, 0.8, 0.9)
    return out


def sample(field, n, pad, d):
    ax = np.abs(d)
    axis = np.argmax(ax, axis=-1)
    comp = np.take_along_axis(d, axis[..., None], -1)[..., 0]
    face = axis * 2 + (comp < 0)
    out = np.zeros(d.shape[:-1], field.dtype)
    for fc in range(6):
        m = face == fc
        if not m.any():
            continue
        den = np.abs(comp[m])
        s = np.arctan(d[m][:, FACE_U[fc]] / den) * 4 / np.pi
        t = np.arctan(d[m][:, FACE_V[fc]] / den) * 4 / np.pi
        i = np.clip(np.floor((s + 1) * 0.5 * n).astype(int), 0, n - 1) + pad
        j = np.clip(np.floor((t + 1) * 0.5 * n).astype(int), 0, n - 1) + pad
        out[m] = field[fc, j, i]
    return out


def d8_area(meta, f):
    """The D8 drainage area (cells) from the receivers, whatever the dumped drainage field is
    (spread over hillslopes since T1b): donors before receivers by the filled level."""
    rec = f["rec"].reshape(-1).astype(np.int64)
    h = f["height"].reshape(-1).astype(np.float64)
    w = f["water"].reshape(-1)
    level = np.maximum(h, np.where(w > -1e30, w, -1e30))
    area = np.ones(len(rec))
    for k in np.lexsort((-np.arange(len(rec)), -level)):
        r = rec[k]
        if r != k and r != NONE:
            area[r] += area[k]
    return area.reshape(f["height"].shape)


def walk_measures(f, side, area_cells, land, pad, steps=16):
    """Of the channel cells draining more than 256 cells: the mean sinuosity of their paths
    16 receivers down (path length over the ends' distance; 1 straight), and the share of the
    channel cells (over 64) whose next 6 steps all go the same way (straight runs; the long
    straight diagonals across filled flats before T1b), with the longest such run."""
    flat_rec = f["rec"].astype(np.int64).reshape(-1)
    ss = side * side
    interior = np.zeros(land.shape, bool)
    interior[:, pad:-pad, pad:-pad] = True
    start = np.flatnonzero((land & interior & (area_cells > 256)).reshape(-1))
    face = start // ss
    cur = start.copy()
    alive = np.ones(len(start), bool)
    length = np.zeros(len(start))
    for _ in range(steps):
        nxt = flat_rec[cur]
        ok = alive & (nxt != cur) & (nxt != NONE) & ((nxt // ss) == face)
        nxt = np.where(ok, nxt, cur)
        length += np.hypot((nxt % ss) // side - (cur % ss) // side, (nxt % ss) % side - (cur % ss) % side) * ok
        alive = ok
        cur = nxt
    s0, e = start[alive], cur[alive]
    chord = np.hypot((e % ss) // side - (s0 % ss) // side, (e % ss) % side - (s0 % ss) % side)
    good = chord > 0
    sinuosity = float(np.mean(length[alive][good] / chord[good])) if good.any() else 1.0
    # Straight runs.
    idx = np.arange(len(flat_rec))
    ch = (land & interior & (area_cells > 64)).reshape(-1) & (flat_rec != idx) & (flat_rec != NONE)
    ch &= (flat_rec // ss) == (idx // ss)
    step = np.where(ch, flat_rec - idx, 0)
    cur = np.flatnonzero(ch)
    first = step[cur]
    run = np.ones(len(cur), np.int32)
    alive = np.ones(len(cur), bool)
    for _ in range(64):
        nxt = flat_rec[cur]
        ok = alive & ch[nxt] & (step[nxt] == first)
        run += ok
        cur = np.where(ok, nxt, cur)
        alive = ok
        if not alive.any():
            break
    share = float(np.mean(run >= 6)) if len(run) else 0.0
    return sinuosity, share, int(run.max()) if len(run) else 0


def measures(meta, f):
    n, pad = meta["n"], meta["pad"]
    side = n + 2 * pad
    cell_m = np.pi / 2 * meta["radius_m"] / n
    cell_area = cell_m * cell_m
    h = f["height"]
    inner = (slice(None), slice(pad, pad + n), slice(pad, pad + n))
    rec = f["rec"]
    idx = np.arange(6 * side * side, dtype=np.int64).reshape(6, side, side)
    area_cells = d8_area(meta, f)
    land = (h > 0)
    chan = land & (area_cells > 64)
    chan[:, :pad, :] = False; chan[:, -pad:, :] = False; chan[:, :, :pad] = False; chan[:, :, -pad:] = False
    r = rec.astype(np.int64)
    flat_rec = r.reshape(-1)
    c = idx[chan]
    rc = flat_rec[c]
    ok = (rc != c) & (rc != NONE)
    c, rc = c[ok], rc[ok]
    rrc = flat_rec[rc]
    same_face = (c // (side * side)) == (rc // (side * side))
    ok2 = (rrc != rc) & (rrc != NONE) & same_face & ((rc // (side * side)) == (rrc // (side * side)))
    c, rc, rrc = c[ok2], rc[ok2], rrc[ok2]
    d1 = rc - c
    d2 = rrc - rc
    same = float(np.mean(d1 == d2)) if len(d1) else 0.0
    run = 1.0 / max(1e-6, 1.0 - same)
    # Sinuosity and straight runs (walk_measures).
    sin_mean, straight_share, straight_longest = walk_measures(f, side, area_cells, land, pad)
    # Lakes.
    w = f["water"][inner]
    lake = w > -1e30
    lake_area_km2 = lake.sum() * cell_area * 1e-6
    land_area = land[inner].sum() * cell_area * 1e-6
    from scipy import ndimage
    labels = 0
    largest = 0.0
    for fc in range(6):
        lab, k = ndimage.label(lake[fc])
        labels += k
        if k:
            sizes = ndimage.sum(np.ones_like(lab), lab, range(1, k + 1))
            largest = max(largest, sizes.max() * cell_area * 1e-6)
    # Summits.
    peaks = []
    plateau_n = 0
    high_n = 0
    for fc in range(6):
        hh = h[fc, pad:pad + n, pad:pad + n]
        mx = ndimage.maximum_filter(hh, size=7)
        pk = (hh == mx) & (hh > 2000)
        peaks.extend(hh[pk].tolist())
        gy, gx = np.gradient(hh, cell_m)
        sl = np.hypot(gx, gy)
        hi = hh > 2500
        high_n += hi.sum()
        plateau_n += (hi & (sl < 0.08)).sum()
    peaks = np.array(peaks) if peaks else np.zeros(1)
    p10, p50, p90 = np.percentile(peaks, [10, 50, 90])
    return {
        "channel_same_direction": round(same, 3), "channel_run_cells": round(run, 2), "channel_cells": int(len(d1)),
        "channel_sinuosity": round(sin_mean, 4), "channel_straight_6": round(straight_share, 4), "channel_straight_longest": straight_longest,
        "lakes": int(labels), "largest_lake_km2": round(largest, 0), "lake_share_of_land": round(lake_area_km2 / max(land_area, 1), 4),
        "summits": int(len(peaks)), "summit_p10_p50_p90_m": [round(p10), round(p50), round(p90)], "summit_spread": round((p90 - p10) / max(p50, 1), 3),
        "plateau_share_of_high_ground": round(plateau_n / max(high_n, 1), 3),
        "max_height_m": round(float(h[inner].max())),
    }


def pick_regions(meta, f, size):
    n, pad = meta["n"], meta["pad"]
    h = f["height"][:, pad:pad + n, pad:pad + n]
    w = f["water"][:, pad:pad + n, pad:pad + n]
    from scipy import ndimage
    best = {}
    k = size
    for fc in range(6):
        hh = h[fc]
        land = (hh > 0).astype(float)
        mean = ndimage.uniform_filter(hh, k)
        landf = ndimage.uniform_filter(land, k)
        var = ndimage.uniform_filter(hh * hh, k) - mean * mean
        lake = ndimage.uniform_filter((w[fc] > -1e30).astype(float), k)
        half = k // 2
        sl = (slice(half, n - half), slice(half, n - half))
        def take(name, score):
            s = score[sl]
            j, i = np.unravel_index(np.argmax(s), s.shape)
            v = s[j, i]
            if name not in best or v > best[name][0]:
                best[name] = (float(v), fc, int(j + half), int(i + half))
        take("range", mean)
        take("lakes", lake + 0.5 * landf)
        take("plains", np.where(landf > 0.98, -np.sqrt(np.maximum(var, 0)) + 0.001 * mean, -1e9))
        take("foothills", np.where((landf > 0.95) & (mean > 600) & (mean < 1800), np.sqrt(np.maximum(var, 0)), -1e9))
    return {name: {"face": v[1], "j": v[2], "i": v[3], "size": size} for name, v in best.items()}


def render(tag, meta, f, regions):
    n, pad = meta["n"], meta["pad"]
    cell_m = np.pi / 2 * meta["radius_m"] / n
    W, H = 2048, 1024
    lat = np.pi / 2 - (np.arange(H) + 0.5) / H * np.pi
    lon = (np.arange(W) + 0.5) / W * 2 * np.pi - np.pi
    LON, LAT = np.meshgrid(lon, lat)
    d = np.stack([np.cos(LAT) * np.cos(LON), np.cos(LAT) * np.sin(LON), np.sin(LAT)], -1)
    hp = sample(f["height"], n, pad, d).astype(np.float64)
    px = 2 * np.pi * meta["radius_m"] / W
    img = np.clip(colour(hp) * (0.35 + 0.75 * shade(hp, px, 4.0)[..., None]), 0, 1)
    img = overlay(img, hp, sample(f["flow"], n, pad, d), sample(f["water"], n, pad, d), cell_m * cell_m)
    plt.imsave(tag + "_planet.png", img)
    wrote = [tag + "_planet.png"]
    for name, r in regions.items():
        fc, j, i, s = r["face"], r["j"], r["i"], r["size"]
        sl = (fc, slice(pad + j - s // 2, pad + j + s // 2), slice(pad + i - s // 2, pad + i + s // 2))
        hh = f["height"][sl].astype(np.float64)
        img = np.clip(colour(hh) * (0.3 + 0.8 * shade(hh, cell_m, 1.5)[..., None]), 0, 1)
        img = overlay(img, hh, f["flow"][sl], f["water"][sl], cell_m * cell_m)
        img = np.repeat(np.repeat(img, 3, 0), 3, 1)[::-1]
        plt.imsave(f"{tag}_{name}.png", img)
        wrote.append(f"{tag}_{name}.png")
    return wrote


def main():
    tag = sys.argv[1]
    reg_path = "regions.json"
    if "--regions" in sys.argv:
        reg_path = sys.argv[sys.argv.index("--regions") + 1]
    meta, f = load(tag)
    if "--pick" in sys.argv or not os.path.exists(reg_path):
        regions = pick_regions(meta, f, max(64, meta["n"] // 4))
        json.dump(regions, open(reg_path, "w"), indent=1)
    regions = json.load(open(reg_path))
    m = measures(meta, f)
    json.dump(m, open(tag + "_measures.json", "w"), indent=1)
    print(tag, json.dumps(m))
    if "--no-render" not in sys.argv:
        print("wrote", *render(tag, meta, f, regions))


if __name__ == "__main__":
    main()
