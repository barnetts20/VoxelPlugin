"""Renders a terrestrial bake exported with voxel.Land.ExportBake (Saved/VoxelLand/Export_*):
an equirectangular, hill-shaded height map, the boundary classes, and for an eroded bake the
rivers (drainage above 1,000 km^2) and kept lakes over the map.

    python view_bake.py <Saved/VoxelLand/Export_<planet>_<key>.json> [width]

Needs numpy and matplotlib. Samples the nearest cell of the face each direction falls on (the
engine's B-spline and face blend are smoother; this is a preview)."""
import json, sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# Face f: normal axis f // 2 (negative for odd f), U and V axes (VoxelCubeSphere.h).
FACE_U = [1, 2, 2, 0, 0, 1]
FACE_V = [2, 1, 0, 2, 1, 0]


def load(path):
    meta = json.load(open(path))
    base = path[:-len(".json")]
    n, pad = meta["n"], meta["pad"]
    side = n + 2 * pad
    h = np.fromfile(base + "_height.f32", np.float32).reshape(6, side, side)
    a = np.fromfile(base + "_attr.u16", np.uint16).reshape(6, side, side)
    flow = water = None
    if os.path.exists(base + "_flow.f32"):
        flow = np.fromfile(base + "_flow.f32", np.float32).reshape(6, side, side)
        water = np.fromfile(base + "_water.f32", np.float32).reshape(6, side, side)
    return meta, h, a, flow, water


def sample(field, n, pad, d):
    ax = np.abs(d)
    axis = np.argmax(ax, axis=-1)
    comp = np.take_along_axis(d, axis[..., None], -1)[..., 0]
    face = axis * 2 + (comp < 0)
    out = np.zeros(d.shape[:-1], field.dtype)
    for f in range(6):
        m = face == f
        if not m.any():
            continue
        den = np.abs(comp[m])
        s = np.arctan(d[m][:, FACE_U[f]] / den) * 4 / np.pi
        t = np.arctan(d[m][:, FACE_V[f]] / den) * 4 / np.pi
        i = np.clip(np.floor((s + 1) * 0.5 * n).astype(int), 0, n - 1) + pad
        j = np.clip(np.floor((t + 1) * 0.5 * n).astype(int), 0, n - 1) + pad
        out[m] = field[f, j, i]
    return out


def main():
    path = sys.argv[1]
    width = int(sys.argv[2]) if len(sys.argv) > 2 else 2048
    meta, h, a, flow, water = load(path)
    n, pad = meta["n"], meta["pad"]
    W, H = width, width // 2
    lat = np.pi / 2 - (np.arange(H) + 0.5) / H * np.pi
    lon = (np.arange(W) + 0.5) / W * 2 * np.pi - np.pi
    LON, LAT = np.meshgrid(lon, lat)
    d = np.stack([np.cos(LAT) * np.cos(LON), np.cos(LAT) * np.sin(LON), np.sin(LAT)], -1)
    img = sample(h, n, pad, d).astype(np.float64)
    cls = (sample(a, n, pad, d) >> 8) & 7

    land = LinearSegmentedColormap.from_list("land", [(0, (0.33, 0.52, 0.30)), (0.12, (0.47, 0.60, 0.36)), (0.28, (0.70, 0.66, 0.45)),
                                                      (0.48, (0.58, 0.46, 0.33)), (0.7, (0.52, 0.5, 0.48)), (1, (0.96, 0.96, 0.97))])
    sea = LinearSegmentedColormap.from_list("sea", [(0, (0.03, 0.08, 0.25)), (0.6, (0.1, 0.25, 0.5)), (0.97, (0.25, 0.5, 0.7)), (1, (0.45, 0.7, 0.8))])
    colour = np.where((img > 0)[..., None], land(np.clip(img / 6000, 0, 1))[..., :3], sea(np.clip(1 + img / 6000, 0, 1))[..., :3])
    px = 2 * np.pi * meta["sea_radius_cm"] * 0.01 / W
    gy, gx = np.gradient(np.maximum(img, 0), px)
    slope = np.arctan(6 * np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    shade = np.clip(np.sin(np.radians(40)) * np.cos(slope) + np.cos(np.radians(40)) * np.sin(slope) * np.cos(np.radians(315) - aspect), 0, 1)
    out = os.path.splitext(path)[0]
    plt.imsave(out + "_map.png", np.clip(colour * (0.35 + 0.75 * shade[..., None]), 0, 1))
    pal = np.array([[.5, .5, .5], [.85, .2, .2], [.95, .55, .1], [.2, .2, .6], [.6, .3, .8], [.9, .9, .2], [.2, .7, .9], [0, 0, 0]])
    plt.imsave(out + "_classes.png", np.where((img > 0)[..., None], pal[cls] * 0.75 + 0.25, pal[cls] * 0.5))
    wrote = [out + "_map.png", out + "_classes.png"]
    if flow is not None:
        rivers = np.clip(colour * (0.35 + 0.75 * shade[..., None]), 0, 1)
        f = sample(flow, n, pad, d)
        w = sample(water, n, pad, d)
        rivers[(img > 0) & (f > np.log2(1.0e9))] = [0.1, 0.35, 0.9]
        rivers[w > -1e30] = [0.0, 0.8, 0.9]
        plt.imsave(out + "_rivers.png", rivers)
        wrote.append(out + "_rivers.png")
    print("wrote", *wrote, "heights", img.min(), img.max(), "land", (img > 0).mean(), "lakes", len(meta.get("lakes", [])))


if __name__ == "__main__":
    main()
