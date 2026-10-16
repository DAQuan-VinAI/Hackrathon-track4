"""[A] PyBullet world: Panda, colored cubes, bins outside the camera view."""
import argparse
import random

import numpy as np
import pybullet as p
import pybullet_data

from common import BIN_HALF, BINS, COLORS, CUBE, RGBA, SPAWN, TABLE_Z

FINGERS = (9, 10)
_cube_shapes = {}


def connect(gui=True):
    if not p.isConnected():
        p.connect(p.GUI if gui else p.DIRECT)
        if gui:
            # Keyboard shortcuts off (w/a/s/g... clash with our keys). Mouse camera still works:
            # Ctrl+left drag rotate, middle drag pan, wheel zoom, left drag = move a part.
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
            reset_view()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())


HOME_VIEW = (1.3, -90, -40, (0.45, 0, 0.1))  # distance, yaw, pitch, target: behind the robot


def reset_view():
    p.resetDebugVisualizerCamera(*HOME_VIEW)


def camera_yaw():
    """Current GUI camera yaw (deg). DIRECT mode / no GUI: the home view yaw."""
    try:
        return float(p.getDebugVisualizerCamera()[8])
    except Exception:
        return HOME_VIEW[1]


def nudge_view(dyaw=0.0, dpitch=0.0, zoom=1.0):
    cam = p.getDebugVisualizerCamera()
    dist, yaw, pitch, target = cam[10], cam[8], cam[9], cam[11]
    p.resetDebugVisualizerCamera(float(np.clip(dist * zoom, 0.4, 4.0)), yaw + dyaw,
                                 float(np.clip(pitch + dpitch, -89, -5)), target)


def add_cube(color, x, y, yaw=0.0, z=None):
    if color not in _cube_shapes:
        h = CUBE / 2
        _cube_shapes[color] = (p.createCollisionShape(p.GEOM_BOX, halfExtents=[h] * 3),
                               p.createVisualShape(p.GEOM_BOX, halfExtents=[h] * 3, rgbaColor=RGBA[color]))
    col, vis = _cube_shapes[color]
    z = TABLE_Z + CUBE / 2 if z is None else z
    return p.createMultiBody(0.05, col, vis, [x, y, z], p.getQuaternionFromEuler([0, 0, yaw]))


def _add_bin(color, xy):
    wall, h = 0.005, 0.03
    for dx, dy, ex, ey in [(BIN_HALF, 0, wall, BIN_HALF), (-BIN_HALF, 0, wall, BIN_HALF),
                           (0, BIN_HALF, BIN_HALF, wall), (0, -BIN_HALF, BIN_HALF, wall)]:
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[ex, ey, h])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[ex, ey, h], rgbaColor=RGBA[color][:3] + (0.6,))
        p.createMultiBody(0, col, vis, [xy[0] + dx, xy[1] + dy, h])


def _layout(rng, n, min_dist=0.08):
    pts = []
    while len(pts) < n:
        q = (rng.uniform(*SPAWN["x"]), rng.uniform(*SPAWN["y"]))
        if all(np.hypot(q[0] - a, q[1] - b) >= min_dist for a, b in pts):
            pts.append(q)
    return pts


def random_free_xy(rng, object_ids, min_dist=0.08):
    taken = [p.getBasePositionAndOrientation(o)[0][:2] for o in object_ids]
    for _ in range(200):
        q = (rng.uniform(*SPAWN["x"]), rng.uniform(*SPAWN["y"]))
        if all(np.hypot(q[0] - a, q[1] - b) >= min_dist for a, b in taken):
            return q
    return None


def settle(steps=240):
    for _ in range(steps):
        p.stepSimulation()


def build_scene(seed, n_objects=6, gui=True):
    """Same seed -> same layout. Seeds >= 100: two same-color cubes touching (failure case)."""
    import controller  # local import: controller imports scene helpers only lazily

    connect(gui)
    p.resetSimulation()
    _cube_shapes.clear()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
    for j in FINGERS:
        p.resetJointState(robot, j, 0.04)  # open fingers so they never push a part
    controller.reset_home(robot)
    rng = random.Random(seed)
    colors = [COLORS[i % 3] for i in range(n_objects)]
    rng.shuffle(colors)
    objects = []
    if seed >= 100:
        # failure case: same color, 1 mm gap, placed at rest (not dropped) so they stay together
        c = colors[0]
        x, y = rng.uniform(0.48, 0.60), rng.uniform(-0.12, 0.12)
        for dy in (-(CUBE + 0.001) / 2, (CUBE + 0.001) / 2):
            objects.append({"id": add_cube(c, x, y + dy), "color": c})
        taken = [(x, y - 0.03), (x, y + 0.03)]
        rest = colors[2:]
        while len(objects) < n_objects:
            q = (rng.uniform(*SPAWN["x"]), rng.uniform(*SPAWN["y"]))
            if all(np.hypot(q[0] - a, q[1] - b) >= 0.09 for a, b in taken):
                taken.append(q)
                c = rest[len(objects) - 2]
                objects.append({"id": add_cube(c, *q, yaw=rng.uniform(-0.7, 0.7)), "color": c})
    else:
        for c, (x, y) in zip(colors, _layout(rng, n_objects)):
            objects.append({"id": add_cube(c, x, y, yaw=rng.uniform(-0.7, 0.7)), "color": c})
    for c, xy in BINS.items():
        _add_bin(c, xy)
    settle()
    return {"robot": robot, "objects": objects, "bins": BINS, "rng": rng}


if __name__ == "__main__":
    import time
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    s = build_scene(ap.parse_args().seed, gui=True)
    print("objects:", s["objects"])
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1 / 240)
