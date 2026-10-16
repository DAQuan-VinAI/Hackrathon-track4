"""Stand-ins for B's and C's functions, so everyone can import and run before 1:15 (owner: A).

main.py uses these only when perception.py / teleop.py are missing or incomplete.
- capture(): real overhead render (same code as the plan), B may copy it.
- detect(): ORACLE, reads object poses from the engine. Never use it for benchmark numbers.
- KeyboardTracker + teleop_step(): minimal keyboard teleop with the same signatures as C's.
"""
import time

import numpy as np
import pybullet as p

from common import CAM, CUBE, WORKSPACE, Detection

STUB_AREA = 1300                      # rough pixel area of one cube top face at 1.1 mm/pixel


def _matrices():
    view = p.computeViewMatrix(CAM["eye"], CAM["target"], CAM["up"])
    proj = p.computeProjectionMatrixFOV(CAM["fov"], CAM["W"] / CAM["H"], CAM["near"], CAM["far"])
    return view, proj


def capture():
    view, proj = _matrices()
    _, _, rgba, depth, seg = p.getCameraImage(CAM["W"], CAM["H"], view, proj,
                                              renderer=p.ER_TINY_RENDERER)   # same renderer in GUI and DIRECT
    H, W = CAM["H"], CAM["W"]
    rgb = np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8)              # RGB, not BGR
    return rgb, np.reshape(depth, (H, W)), np.reshape(seg, (H, W))


def _world_to_pixel(xyz):
    view, proj = _matrices()
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    clip = P @ V @ np.array([*xyz, 1.0])
    ndc = clip[:3] / clip[3]
    W, H = CAM["W"], CAM["H"]
    return (ndc[0] + 1) / 2 * W - 0.5, (1 - ndc[1]) / 2 * H - 0.5


def detect(rgb=None, depth=None):
    """ORACLE detect from engine ground truth. Flags same-color cubes closer than 1.5 cube edges."""
    from scene import CURRENT
    dets = []
    for o in CURRENT.get("objects", []):
        pos, orn = p.getBasePositionAndOrientation(o["id"])
        top = (pos[0], pos[1], pos[2] + CUBE / 2)
        u, v = _world_to_pixel(top)
        if not (0 <= u < CAM["W"] and 0 <= v < CAM["H"]) or pos[2] > 0.1:
            continue                  # out of view, or held by the gripper
        dets.append(Detection(o["color"], (int(u), int(v)), top, p.getEulerFromQuaternion(orn)[2], STUB_AREA))
    for a in dets:
        for b in dets:
            if a is not b and a.color == b.color and np.hypot(a.xyz[0] - b.xyz[0], a.xyz[1] - b.xyz[1]) < 1.5 * CUBE:
                a.suspicious, a.reason = True, "area"
    return dets


MOVE_KEYS = {ord("i"): (0.01, 0, 0), ord("k"): (-0.01, 0, 0),   # i/k: away from / toward the robot
             ord("j"): (0, 0.01, 0), ord("l"): (0, -0.01, 0),   # j/l: screen left / right (view from behind)
             ord("u"): (0, 0, 0.01), ord("o"): (0, 0, -0.01)}   # u/o: up / down
MAX_STEP = 0.015                                                 # m per frame, robot never jumps to the target


def clamp_workspace(xyz):
    return np.array([np.clip(xyz[i], *WORKSPACE[k]) for i, k in enumerate("xyz")])


class KeyboardTracker:
    def __init__(self, start=(0.55, 0.0, 0.20)):
        self.pos = np.array(start, dtype=float)
        self.grip = False

    def read(self, key=-1):
        t0 = time.perf_counter()
        if key in MOVE_KEYS:
            self.pos = clamp_workspace(self.pos + MOVE_KEYS[key])
        elif key == ord(" "):
            self.grip = not self.grip
        return self.pos.copy(), self.grip, None, (time.perf_counter() - t0) * 1000


def teleop_step(robot, tracker, object_ids, state):
    from controller import ee_pos, grasp, release, servo_ee
    if state.get("target") is None:
        state["target"] = ee_pos(robot)
    xyz, grip, frame, latency_ms = tracker.read(state.get("key", -1))
    if xyz is not None:                                          # no hand -> hold position and the cube
        goal = clamp_workspace(xyz)
        delta = goal - state["target"]
        n = np.linalg.norm(delta)
        state["target"] = state["target"] + (delta if n <= MAX_STEP else delta / n * MAX_STEP)
        if grip and state.get("cid") is None:
            state["cid"] = grasp(robot, object_ids)
            state["msg"] = "grasped" if state["cid"] is not None else "nothing within 4 cm"
        elif not grip and state.get("cid") is not None:
            release(state["cid"])
            state["cid"], state["msg"] = None, "released"
    servo_ee(robot, state["target"])
    state["frame"], state["latency_ms"] = frame, latency_ms
    return state
