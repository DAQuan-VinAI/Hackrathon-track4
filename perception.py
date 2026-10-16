"""[B] Virtual overhead camera: RGB+depth -> HSV mask -> contour -> 3D, with confidence checks."""
import cv2
import numpy as np
import pybullet as p

from common import CAM, CUBE, TABLE_Z, Detection

# HSV ranges (OpenCV hue 0-180). Red wraps around 0.
HSV = {
    "red": [((0, 120, 70), (10, 255, 255)), ((170, 120, 70), (180, 255, 255))],
    "green": [((40, 120, 70), (80, 255, 255))],
    "blue": [((100, 120, 70), (130, 255, 255))],
}
MIN_AREA = 300          # px; one cube ~ 1200 px at this camera height
A_REF = 1510.0          # median single-cube area (px), measured by calibrate_area() on seeds 0-4
AREA_K, ASPECT_K, DEPTH_TOL = 1.6, 1.5, 0.01


def matrices():
    view = p.computeViewMatrix(CAM["eye"], CAM["target"], CAM["up"])
    proj = p.computeProjectionMatrixFOV(CAM["fov"], CAM["W"] / CAM["H"], CAM["near"], CAM["far"])
    return view, proj


def capture():
    view, proj = matrices()
    _, _, rgba, depth, seg = p.getCameraImage(CAM["W"], CAM["H"], view, proj,
                                              renderer=p.ER_TINY_RENDERER)  # same in GUI and DIRECT
    H, W = CAM["H"], CAM["W"]
    rgb = np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8)  # RGB, not BGR
    return rgb, np.reshape(depth, (H, W)), np.reshape(seg, (H, W))


def pixel_to_world(u, v, d, view, proj, W, H):
    ndc = np.array([(2 * (u + 0.5) - W) / W, (H - 2 * (v + 0.5)) / H, 2 * d - 1, 1.0])
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    pw = np.linalg.inv(P @ V) @ ndc
    return pw[:3] / pw[3]


def masks(rgb):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    kernel = np.ones((3, 3), np.uint8)
    out = {}
    for color, ranges in HSV.items():
        m = sum(cv2.inRange(hsv, np.array(lo), np.array(hi)) for lo, hi in ranges)
        out[color] = cv2.morphologyEx(np.clip(m, 0, 255).astype(np.uint8), cv2.MORPH_OPEN, kernel)
    return out


def detect(rgb, depth, a_ref=None):
    a_ref = a_ref or A_REF
    view, proj = matrices()
    W, H = CAM["W"], CAM["H"]
    dets = []
    for color, m in masks(rgb).items():
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = int(cv2.contourArea(c))
            if area < MIN_AREA:
                continue
            mo = cv2.moments(c)
            u, v = mo["m10"] / mo["m00"], mo["m01"] / mo["m00"]
            ui, vi = int(round(u)), int(round(v))
            patch = depth[max(vi - 2, 0):vi + 3, max(ui - 2, 0):ui + 3]
            d = float(np.median(patch))  # median: avoid edge pixels
            xyz = pixel_to_world(u, v, d, view, proj, W, H)
            (_, _), (rw, rh), ang = cv2.minAreaRect(c)
            det = Detection(color, (u, v), tuple(float(t) for t in xyz), float(np.deg2rad(ang % 90)), area)
            if a_ref and area > AREA_K * a_ref:
                det.suspicious, det.reason = True, "area"
            elif min(rw, rh) > 0 and max(rw, rh) / min(rw, rh) > ASPECT_K:
                det.suspicious, det.reason = True, "aspect"
            elif abs(xyz[2] - (TABLE_Z + CUBE)) > DEPTH_TOL:
                det.suspicious, det.reason = True, "depth"
            dets.append(det)
    return dets


def calibrate_area(seeds=range(5)):
    """Median single-cube pixel area over normal seeds (run once, then frozen in A_REF)."""
    from scene import build_scene
    areas = []
    for s in seeds:
        build_scene(s, gui=False)
        rgb, depth, _ = capture()
        areas += [d.area for d in detect(rgb, depth, a_ref=0)]
    return float(np.median(areas))


def draw(rgb, dets):
    """BGR debug image for the HUD."""
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    for d in dets:
        col = (0, 0, 255) if d.suspicious else (255, 255, 255)
        u, v = int(d.uv[0]), int(d.uv[1])
        cv2.circle(img, (u, v), 4, col, -1)
        cv2.rectangle(img, (u - 22, v - 22), (u + 22, v + 22), col, 2)
        label = f"{d.color} {d.xyz[0]:.3f},{d.xyz[1]:.3f}" + (f" ?{d.reason}" if d.suspicious else "")
        cv2.putText(img, label, (u - 40, v - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
    return img


if __name__ == "__main__":
    import os
    from scene import build_scene
    os.makedirs("debug", exist_ok=True)
    s = build_scene(0, gui=False)
    rgb, depth, _ = capture()
    cv2.imwrite("debug/cap.png", draw(rgb, detect(rgb, depth)))
    # failure-case slide: RGB / mask / merged contour (seed 100)
    build_scene(100, gui=False)
    rgb, depth, _ = capture()
    dets = detect(rgb, depth)
    bad = next(d for d in dets if d.suspicious)
    m = masks(rgb)[bad.color]
    cv2.imwrite("debug/fail_rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    cv2.imwrite("debug/fail_mask.png", m)
    cv2.imwrite("debug/fail_contour.png", draw(rgb, dets))
    print("failure seed 100:", bad.color, bad.area, "px, reason", bad.reason)
    print("A_REF (median area seeds 0-4):", calibrate_area())
