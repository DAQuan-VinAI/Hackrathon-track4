"""Virtual overhead camera and cube detection (owner: B).

STUB: detect() returns oracle detections from scene.CURRENT (engine ground truth)
so A and D can integrate before the real HSV detector lands. B replaces it.
"""
import numpy as np
import pybullet as p

from common import CAM, CUBE, Detection


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


def _world_to_pixel(xyz):
    view, proj = _matrices()
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    clip = P @ V @ np.array([*xyz, 1.0])
    x, y = clip[:2] / clip[3]
    return int((x + 1) * CAM["W"] / 2), int((1 - y) * CAM["H"] / 2)


def detect(rgb=None, depth=None):
    """STUB (oracle): ignores the images, reads cube poses from the engine."""
    from scene import CURRENT, ground_truth

    if not CURRENT:
        return []
    dets = []
    for _, color, (x, y, z) in ground_truth(CURRENT):
        top = (x, y, z + CUBE / 2)                                  # top face center
        dets.append(Detection(color, _world_to_pixel(top), top, 0.0, 0))
    return dets


if __name__ == "__main__":
    from scene import build_scene

    build_scene(0, gui=False)
    rgb, depth, seg = capture()
    print("rgb", rgb.shape, "depth", depth.shape)
    for d in detect(rgb, depth):
        print(d)
