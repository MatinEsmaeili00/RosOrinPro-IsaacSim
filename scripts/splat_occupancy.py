"""Occupancy grid, Nav2 map and collision-proxy rectangles from 3D Gaussian Splat centres.

Pure numpy + scipy (+ Pillow for the preview), so it runs inside Isaac Sim (scripts/build_splat_scene.py)
or any Python with those packages.

Idea: a Gaussian splat has no surfaces, but its Gaussians sit on the surfaces it reconstructs.
  * free     = cells where the floor was reconstructed (Gaussians in a thin band around z = 0), closed and
               opened morphologically, largest connected region only
  * occupied = cells with enough opaque Gaussians in the robot's height band, plus a ring around the free
               region (so the robot treats everything outside the scanned floor as a wall)
  * unknown  = the rest
The same occupied cells become the collision/lidar proxy boxes, so the map, the physics and the lidar agree.
"""

import math

import numpy as np
from scipy import ndimage


def disk(radius):
    r = int(radius)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return (yy * yy + xx * xx) <= r * r


def gaussian_arrays(positions, raw_opacities, log_scales):
    """Sigmoid opacity and largest linear scale per Gaussian (3DGS PLY stores logit / log values)."""
    opacity = 1.0 / (1.0 + np.exp(-raw_opacities.astype(np.float32)))
    max_scale = np.exp(log_scales.astype(np.float32).max(axis=1))
    return positions.astype(np.float32), opacity, max_scale


def crop_mask(positions, crop_min, crop_max):
    lo, hi = np.asarray(crop_min, np.float32), np.asarray(crop_max, np.float32)
    return np.all((positions > lo) & (positions < hi), axis=1)


def occupancy(positions, opacity, max_scale, cfg, xy_min, xy_max):
    """Return dict(free, occupied, origin, resolution, stats). Grids are [row = y, col = x], row 0 = min y."""
    res = float(cfg["resolution"])
    origin = np.asarray(xy_min, np.float64)
    W = int(math.ceil((xy_max[0] - xy_min[0]) / res))
    H = int(math.ceil((xy_max[1] - xy_min[1]) / res))
    good = (opacity > cfg["min_opacity"]) & (max_scale < cfg["max_scale"])
    p, op = positions[good], opacity[good]
    ix = np.clip(((p[:, 0] - origin[0]) / res).astype(np.int64), 0, W - 1)
    iy = np.clip(((p[:, 1] - origin[1]) / res).astype(np.int64), 0, H - 1)
    z = p[:, 2]

    def accumulate(mask, weights=None):
        g = np.zeros(H * W, np.float64)
        np.add.at(g, iy[mask] * W + ix[mask], 1.0 if weights is None else weights[mask])
        return g.reshape(H, W)

    fb, ob = cfg["floor_band"], cfg["obstacle_band"]
    floor_count = accumulate((z > fb[0]) & (z < fb[1]))
    obstacle_weight = accumulate((z > ob[0]) & (z < ob[1]), op)

    seen = floor_count >= cfg["floor_min_count"]
    free = ndimage.binary_closing(seen, structure=disk(cfg["close_radius"]), border_value=0)
    free = ndimage.binary_opening(free, structure=disk(cfg["open_radius"]), border_value=0)
    obstacle = obstacle_weight >= cfg["obstacle_min_weight"]
    # drop isolated single obstacle cells (noise): keep cells with at least one 8-neighbour
    neighbours = ndimage.convolve(obstacle.astype(np.int32), np.ones((3, 3), np.int32), mode="constant") - obstacle
    obstacle &= neighbours >= 1
    labels, n = ndimage.label(free & ~obstacle)
    if n == 0:
        raise RuntimeError("no free floor found: check floor_band / floor_min_count")
    sizes = np.bincount(labels.ravel())[1:]
    free_main = labels == (int(np.argmax(sizes)) + 1)
    ring = ndimage.binary_dilation(free_main, structure=disk(int(cfg["boundary_cells"]))) & ~free_main
    near = ndimage.binary_dilation(free_main, structure=disk(4))
    occupied = (obstacle & near) | ring
    free_main &= ~occupied
    stats = {
        "gaussians_used": int(good.sum()),
        "grid_cells": [W, H],
        "free_m2": round(float(free_main.sum()) * res * res, 1),
        "occupied_cells": int(occupied.sum()),
        "free_components": int(n),
    }
    return {"free": free_main, "occupied": occupied, "origin": origin, "resolution": res, "stats": stats}


def clearance(grid):
    """Distance (m) from every free cell to the nearest non-free cell."""
    return ndimage.distance_transform_edt(grid["free"]) * grid["resolution"]


def world_to_cell(grid, x, y):
    res, o = grid["resolution"], grid["origin"]
    return int((y - o[1]) / res), int((x - o[0]) / res)


def check_places(grid, places, start, min_clearance):
    """Clearance at each place and whether the robot (radius min_clearance) can reach it from start."""
    clear = clearance(grid)
    passable = clear >= min_clearance
    labels, _ = ndimage.label(passable)
    sr, sc = world_to_cell(grid, start[0], start[1])
    start_label = labels[sr, sc] if 0 <= sr < labels.shape[0] and 0 <= sc < labels.shape[1] else 0
    out = {}
    for name, pose in places.items():
        r, c = world_to_cell(grid, pose[0], pose[1])
        inside = 0 <= r < labels.shape[0] and 0 <= c < labels.shape[1]
        out[name] = {
            "clearance_m": round(float(clear[r, c]), 2) if inside else 0.0,
            "reachable": bool(inside and start_label and labels[r, c] == start_label),
        }
    return out


def write_nav2_map(grid, out_dir, name, note="scripts/build_splat_scene.py from the Gaussian splat"):
    """ROS map_server files: <name>.pgm (254 free, 0 occupied, 205 unknown) + <name>.yaml (trinary)."""
    import os

    img = np.full(grid["free"].shape, 205, np.uint8)
    img[grid["free"]] = 254
    img[grid["occupied"]] = 0
    img = img[::-1]  # image row 0 = max y
    h, w = img.shape
    with open(os.path.join(out_dir, name + ".pgm"), "wb") as f:
        f.write(b"P5\n# ROSOrin Pro Nav2 map %s\n%d %d\n255\n" % (name.encode(), w, h))
        f.write(img.tobytes())
    with open(os.path.join(out_dir, name + ".yaml"), "w", encoding="utf-8", newline="\n") as f:
        f.write("# Generated by scripts/build_splat_scene.py from the Gaussian splat. World (scene) coordinates.\n")
        f.write("image: %s.pgm\nmode: trinary\nresolution: %.3f\n" % (name, grid["resolution"]))
        f.write("origin: [%.3f, %.3f, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
                % (grid["origin"][0], grid["origin"][1]))
    return img


def rectangles(mask):
    """Greedy merge of occupied cells into axis-aligned rectangles: list of (row0, col0, row1, col1) exclusive."""
    mask = mask.copy()
    H, W = mask.shape
    rects = []
    for r in range(H):
        c = 0
        while c < W:
            if not mask[r, c]:
                c += 1
                continue
            c1 = c
            while c1 < W and mask[r, c1]:
                c1 += 1
            r1 = r + 1
            while r1 < H and mask[r1, c:c1].all():
                r1 += 1
            mask[r:r1, c:c1] = False
            rects.append((r, c, r1, c1))
            c = c1
    return rects


def preview(img, grid, places, path, spawn=None):
    """Colour preview of the map with the named places."""
    from PIL import Image, ImageDraw

    rgb = np.stack([img] * 3, axis=-1)
    rgb[img == 205] = (190, 196, 204)
    rgb[img == 0] = (30, 30, 30)
    scale = 2
    im = Image.fromarray(rgb).resize((rgb.shape[1] * scale, rgb.shape[0] * scale), Image.NEAREST)
    dr = ImageDraw.Draw(im)
    H = img.shape[0]

    def px(x, y):
        r, c = world_to_cell(grid, x, y)
        return (c + 0.5) * scale, (H - 1 - r + 0.5) * scale

    for name, pose in places.items():
        u, v = px(pose[0], pose[1])
        dr.ellipse([u - 6, v - 6, u + 6, v + 6], outline=(214, 40, 40), width=3)
        dr.text((u + 9, v - 7), name, fill=(150, 20, 20))
    if spawn is not None:
        u, v = px(spawn[0], spawn[1])
        a = math.radians(spawn[2])
        dr.line([u, v, u + 22 * math.cos(a), v - 22 * math.sin(a)], fill=(20, 110, 220), width=4)
    # 1 m scale bar
    dr.line([12, im.height - 14, 12 + scale / grid["resolution"], im.height - 14], fill=(0, 0, 0), width=4)
    dr.text((14, im.height - 32), "1 m", fill=(0, 0, 0))
    im.save(path)
