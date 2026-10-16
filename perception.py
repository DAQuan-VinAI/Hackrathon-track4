"""Virtual overhead camera and cube detection (owner: B).

    python perception.py                      # evaluate seeds 0-19 and 100-104 against engine ground truth
    python perception.py --seeds 100 --save   # also write debug/<seed>_{rgb,mask,contours}.png

Pipeline: RGB -> HSV -> inRange per color -> open -> contours -> centroid (moments)
-> median depth around the centroid -> inv(P @ V) -> world xyz of the top face center.
Each detection is checked (area, aspect ratio, height) and flagged `suspicious` instead
of being guessed: the robot skips it and asks the operator.

detect() only reads the RGB and depth images. The segmentation image and the engine
poses are ground truth for evaluation, never used for detection.
"""
import argparse
import os

import cv2
import numpy as np
import pybullet as p

from common import CAM, COLORS, CUBE, SEEDS_FAILURE, SEEDS_NORMAL, TABLE_Z, Detection

# HSV thresholds (OpenCV scale: H 0-180, S and V 0-255), measured on seed 0:
# cube faces S 219-228, plane S <= 71; hue red 0, green 62, blue 114.
# S >= 120 leaves margin for the noise mode (brightness x0.7-1.3, Gaussian sigma 8).
HSV_RANGES = {
    "red": [((0, 120, 50), (10, 255, 255)), ((170, 120, 50), (180, 255, 255))],   # red wraps around hue 0
    "green": [((45, 120, 50), (85, 255, 255))],
    "blue": [((100, 120, 50), (130, 255, 255))],
}
MIN_AREA = 300            # px; one cube is ~1500-1700 px, anything much smaller is noise
DEPTH_WIN = 5             # fallback: median depth over a 5x5 window around the centroid avoids cube edges
TOP_TOL = 0.003           # m; pixels within 3 mm of the closest one form the top face
KERNEL = np.ones((3, 3), np.uint8)

# Confidence checks (first one that fails sets `reason`)
A_REF = 1538              # px; median single-cube area on seeds 0-4 (min 1343, max 1661), `--calibrate`
AREA_RATIO = 1.6          # area > 1.6 x A_REF: probably two cubes merged into one blob
ASPECT_MAX = 1.5          # long side / short side of minAreaRect: a cube seen from above is square
Z_TOL = 0.01              # top face must be at TABLE_Z + CUBE within 1 cm (not stacked or tilted)


def _matrices():
    view = p.computeViewMatrix(CAM["eye"], CAM["target"], CAM["up"])
    proj = p.computeProjectionMatrixFOV(CAM["fov"], CAM["W"] / CAM["H"], CAM["near"], CAM["far"])
    return view, proj


def capture():
    """Returns (rgb uint8 HxWx3, depth float HxW in [0, 1], seg int HxW)."""
    view, proj = _matrices()
    H, W = CAM["H"], CAM["W"]
    _, _, rgba, depth, seg = p.getCameraImage(W, H, view, proj,
                                              renderer=p.ER_TINY_RENDERER)   # same renderer in GUI and DIRECT
    rgb = np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8)              # RGB, not BGR
    return rgb, np.reshape(depth, (H, W)), np.reshape(seg, (H, W))


def pixel_to_world(u, v, d, view, proj, W, H):
    """Pixel (u, v) with depth-buffer value d in [0, 1] -> world (x, y, z)."""
    # +0.5: use the pixel center, not its corner
    ndc = np.array([(2 * (u + 0.5) - W) / W, (H - 2 * (v + 0.5)) / H, 2 * d - 1, 1.0])
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    pw = np.linalg.inv(P @ V) @ ndc
    return pw[:3] / pw[3]


def color_masks(rgb):
    """{color: binary mask} after HSV thresholding and a morphological open."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    masks = {}
    for color in COLORS:
        m = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES[color]:
            m |= cv2.inRange(hsv, lo, hi)
        masks[color] = cv2.morphologyEx(m, cv2.MORPH_OPEN, KERNEL)
    return masks


def _median_depth(depth, u, v):
    r = DEPTH_WIN // 2
    H, W = depth.shape
    win = depth[max(v - r, 0):min(v + r + 1, H), max(u - r, 0):min(u + r + 1, W)]
    return float(np.median(win))


def _linear_depth(d):
    """Depth-buffer value in [0, 1] -> distance along the camera axis (m)."""
    n, f = CAM["near"], CAM["far"]
    return f * n / (f - (f - n) * d)


def _top_face(contour, depth):
    """Pixels of the blob that belong to its top face.

    The camera looks straight down, so the top face is parallel to the image plane and has
    a constant depth; side faces (visible because of perspective) are farther away. Using
    only the top face removes the perspective bias of the centroid.
    """
    blob = np.zeros(depth.shape, np.uint8)
    cv2.drawContours(blob, [contour], -1, 255, -1)
    lin = _linear_depth(depth)
    inside = blob > 0
    top = inside & (lin <= lin[inside].min() + TOP_TOL)
    return top


def _world_yaw(rect, d, view, proj):
    """Yaw of the cube in the world frame, folded into [-pi/4, pi/4) (a square has 90 deg symmetry)."""
    corners = cv2.boxPoints(rect)
    a, b = (pixel_to_world(*c, d, view, proj, CAM["W"], CAM["H"]) for c in corners[:2])
    yaw = np.arctan2(b[1] - a[1], b[0] - a[0])
    return float((yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4)


def _check(area, rect, z):
    """Returns (suspicious, reason). Order matters: the first failed check is reported."""
    if area > AREA_RATIO * A_REF:
        return True, "area"
    w, h = rect[1]
    if min(w, h) > 0 and max(w, h) / min(w, h) > ASPECT_MAX:
        return True, "aspect"
    if abs(z - (TABLE_Z + CUBE)) > Z_TOL:
        return True, "depth"
    return False, ""


def detect(rgb=None, depth=None):
    """RGB + depth -> list[Detection]. Captures a new frame if no images are given."""
    if rgb is None or depth is None:
        rgb, depth, _ = capture()
    view, proj = _matrices()
    W, H = CAM["W"], CAM["H"]
    dets = []
    for color, mask in color_masks(rgb).items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            area = M["m00"]
            if area < MIN_AREA:
                continue
            top = _top_face(c, depth)
            vs, us = np.nonzero(top)
            if len(us) >= DEPTH_WIN ** 2:
                uf, vf = us.mean(), vs.mean()                    # sub-pixel centroid of the top face
                d = float(np.median(depth[top]))
                rect = cv2.minAreaRect(np.column_stack([us, vs]).astype(np.float32))
            else:                                                # fallback: whole blob
                uf, vf = M["m10"] / area, M["m01"] / area
                d = _median_depth(depth, int(round(uf)), int(round(vf)))
                rect = cv2.minAreaRect(c)
            u, v = int(round(uf)), int(round(vf))
            xyz = pixel_to_world(uf, vf, d, view, proj, W, H)
            suspicious, reason = _check(area, rect, xyz[2])
            dets.append(Detection(color, (u, v), tuple(float(x) for x in xyz),
                                  _world_yaw(rect, d, view, proj), int(area), suspicious, reason))
    return dets


def draw_detections(rgb, dets):
    """BGR image with boxes and labels, ready for cv2.imshow (for the HUD in main.py)."""
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for d in dets:
        col = (0, 0, 255) if d.suspicious else (0, 255, 0)
        u, v = d.uv
        cv2.circle(img, (u, v), 3, col, -1)
        cv2.rectangle(img, (u - 22, v - 22), (u + 22, v + 22), col, 2 if d.suspicious else 1)
        label = f"{d.color} ({d.xyz[0]:.3f},{d.xyz[1]:.3f})"
        if d.suspicious:
            label = f"? {d.color} {d.reason}"
        cv2.putText(img, label, (u - 22, v - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
    return img


# ---------------------------------------------------------------- 3D overlay in the PyBullet window

_ITEMS = []               # debug item ids drawn by show_in_3d, removed on the next call
RGB3D = {"red": (0.95, 0.15, 0.15), "green": (0.1, 0.75, 0.2), "blue": (0.15, 0.35, 1.0)}
SUSPICIOUS_RGB = (1.0, 0.55, 0.0)


def _ground_footprint():
    """World corners of the image on the ground plane (z = TABLE_Z): what the camera can see."""
    view, proj = _matrices()
    W, H = CAM["W"], CAM["H"]
    eye = np.array(CAM["eye"], float)
    corners = []
    for u, v in ((0, 0), (W - 1, 0), (W - 1, H - 1), (0, H - 1)):
        far = pixel_to_world(u, v, 1.0, view, proj, W, H)     # a point on the pixel's ray
        t = (eye[2] - TABLE_Z) / (eye[2] - far[2])
        corners.append(eye + t * (far - eye))
    return eye, corners


def show_in_3d(dets):
    """Draw what perception sees into the PyBullet GUI: the camera frustum, one ray per detection
    from the camera to the 3D point, and a labeled marker (orange = suspicious). GUI only."""
    if p.getConnectionInfo().get("connectionMethod") != p.GUI:
        return
    for i in _ITEMS:
        p.removeUserDebugItem(i)
    _ITEMS.clear()
    line = lambda a, b, rgb, w=1.5: _ITEMS.append(p.addUserDebugLine(a, b, rgb, lineWidth=w))
    text = lambda s, pos, rgb, size=1.1: _ITEMS.append(p.addUserDebugText(s, pos, rgb, textSize=size))

    eye, corners = _ground_footprint()
    gray = (0.35, 0.35, 0.45)
    for i, c in enumerate(corners):
        line(eye, c, gray, 1)
        line(c, corners[(i + 1) % 4], gray, 2)
    text("overhead camera", eye + (0, 0, 0.03), (0.2, 0.2, 0.3), 1.2)

    for d in dets:
        rgb = SUSPICIOUS_RGB if d.suspicious else RGB3D.get(d.color, (1, 1, 1))
        xyz = np.array(d.xyz)
        line(eye, xyz, rgb, 1)                                 # pixel -> ray -> 3D point (inv(P V) + depth)
        s = CUBE * 0.7                                         # cross on the top face
        line(xyz - (s, 0, 0), xyz + (s, 0, 0), rgb, 3)
        line(xyz - (0, s, 0), xyz + (0, s, 0), rgb, 3)
        label = f"? {d.reason}" if d.suspicious else f"{d.color} ({d.xyz[0]:.3f}, {d.xyz[1]:.3f})"
        text(label, xyz + (0, 0, 0.05), rgb)


# ---------------------------------------------------------------- evaluation (ground truth only here)

def _match(dets, gt, radius=0.02):
    """Greedy match: same color, nearest in XY, within `radius`. Returns [(det, gt_xyz)], unmatched dets."""
    pairs, used = [], set()
    for d in dets:
        cands = [(np.hypot(d.xyz[0] - g[0], d.xyz[1] - g[1]), i) for i, (_, col, g) in enumerate(gt)
                 if col == d.color and i not in used]
        if cands and min(cands)[0] < radius:
            i = min(cands)[1]
            used.add(i)
            pairs.append((d, gt[i][2]))
    matched = {id(d) for d, _ in pairs}
    return pairs, [d for d in dets if id(d) not in matched]


def _save_debug(seed, rgb, dets):
    os.makedirs("debug", exist_ok=True)
    masks = color_masks(rgb)
    union = np.max(np.stack(list(masks.values())), axis=0)
    cont = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for m in masks.values():
        cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cont, cs, -1, (0, 255, 255), 2)
    cv2.imwrite(f"debug/{seed}_rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite(f"debug/{seed}_mask.png", union)
    cv2.imwrite(f"debug/{seed}_contours.png", cont)
    cv2.imwrite(f"debug/{seed}_detections.png", draw_detections(rgb, dets))


if __name__ == "__main__":
    from controller import go_home
    from scene import build_scene, ground_truth

    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=SEEDS_NORMAL + SEEDS_FAILURE)
    ap.add_argument("--save", action="store_true", help="write debug images per seed")
    ap.add_argument("--calibrate", action="store_true", help="print median single-cube area of seeds 0-4")
    ap.add_argument("--noise", action="store_true", help="D's noise mode: brightness x[0.7, 1.3], Gaussian sigma 8")
    args = ap.parse_args()

    if args.calibrate:
        areas = []
        for s in range(5):
            go_home(build_scene(s, gui=False)["robot"])
            areas += [d.area for d in detect()]
        print(f"A_REF = median area = {np.median(areas):.0f} px (min {min(areas)}, max {max(areas)}, n={len(areas)})")
        raise SystemExit

    xy_err, xy_err_sus, z_err, n_gt, n_fp, flags = [], [], [], 0, 0, {"normal": 0, "failure": 0}
    for s in args.seeds:
        sc = build_scene(s, gui=False)
        go_home(sc["robot"])                       # arm out of the camera view before capturing
        rgb, depth, _ = capture()
        if args.noise:
            rng = np.random.default_rng(s)         # fixed noise seed: reruns give the same numbers
            rgb = np.clip(rgb * rng.uniform(0.7, 1.3) + rng.normal(0, 8, rgb.shape), 0, 255).astype(np.uint8)
        dets = detect(rgb, depth)
        gt = ground_truth(sc)
        pairs, fps = _match(dets, gt)
        n_gt += len(gt)
        n_fp += len(fps)
        errs = [np.hypot(d.xyz[0] - g[0], d.xyz[1] - g[1]) * 1000 for d, g in pairs]
        xy_err += [e for e, (d, _) in zip(errs, pairs) if not d.suspicious]
        xy_err_sus += [e for e, (d, _) in zip(errs, pairs) if d.suspicious]
        z_err += [(d.xyz[2] - (TABLE_Z + CUBE)) * 1000 for d, _ in pairs]
        sus = [d for d in dets if d.suspicious]
        flags["failure" if s in SEEDS_FAILURE else "normal"] += bool(sus)
        print(f"seed {s:3d}: {len(dets)} det / {len(gt)} gt, matched {len(pairs)}, fp {len(fps)}, "
              f"xy max {max(errs, default=0):.1f} mm, suspicious {[(d.color, d.reason, d.area) for d in sus]}")
        if args.save:
            _save_debug(s, rgb, dets)

    print(f"\nrecall {len(xy_err) + len(xy_err_sus)}/{n_gt}, false positives {n_fp}")
    if xy_err:
        print(f"XY error mm (trusted detections): mean {np.mean(xy_err):.2f}, "
              f"median {np.median(xy_err):.2f}, max {np.max(xy_err):.2f}")
    if xy_err_sus:
        print(f"XY error mm (suspicious, skipped by the robot): max {np.max(xy_err_sus):.2f}")
        print(f"Z error mm (top face vs TABLE_Z + CUBE): mean {np.mean(z_err):.2f}, max |.| {np.max(np.abs(z_err)):.2f}")
    n_norm = sum(s not in SEEDS_FAILURE for s in args.seeds)
    n_fail = len(args.seeds) - n_norm
    print(f"suspicious flag: failure seeds {flags['failure']}/{n_fail} flagged, "
          f"normal seeds {flags['normal']}/{n_norm} false alarms")
