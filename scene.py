"""Simulated world: plane, Panda arm, colored cubes, three bins (owner: A).

    python scene.py --seed 0          # open GUI, cubes settle, print ground truth
    python scene.py --seed 100        # failure case: two same-color cubes touching
"""
import argparse
import time

import numpy as np
import pybullet as p
import pybullet_data

from common import BIN_SIZE, BINS, COLORS, CUBE, SEEDS_FAILURE, SIM_HZ, TABLE_Z, WORKSPACE

RGBA = {"red": (0.85, 0.10, 0.10, 1), "green": (0.10, 0.70, 0.15, 1), "blue": (0.10, 0.25, 0.90, 1)}
CUBE_MASS = 0.05
MIN_SPACING = 0.08
EDGE_MARGIN = 0.03                    # keep cubes away from the WORKSPACE border
PANDA_REST = [0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.785]   # nominal arm pose, joints 0..6
FINGER_JOINTS = [9, 10]
VIEW_YAW = -90                        # GUI camera behind the robot: screen right == world -y

CURRENT = {}                          # last scene built in this process (used by the oracle stub)


def _connect(gui):
    if p.isConnected():
        p.resetSimulation()
    else:
        p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1 / SIM_HZ)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)            # GUI only; the TINY camera has no shadows
        try:
            p.configureDebugVisualizer(rgbBackground=[0.93, 0.95, 0.98])
        except TypeError:                                               # older pybullet builds
            pass
        # look from behind the robot so screen left/right matches the operator's hand
        p.resetDebugVisualizerCamera(cameraDistance=0.95, cameraYaw=VIEW_YAW, cameraPitch=-55,
                                     cameraTargetPosition=[0.5, -0.1, 0.0])


def camera_yaw():
    """Current GUI camera yaw (deg), so teleop stays screen-relative after the view is rotated.
    DIRECT mode reports 0 for every camera field: use the default view instead."""
    if p.getConnectionInfo().get("connectionMethod") != p.GUI:
        return VIEW_YAW
    return float(p.getDebugVisualizerCamera()[8])


def _make_ground():
    """Plain gray ground. plane.urdf has a blue checker texture that leaks into the blue HSV mask."""
    col = p.createCollisionShape(p.GEOM_PLANE)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[2, 2, 0.001], rgbaColor=[0.82, 0.82, 0.82, 1],
                              visualFramePosition=[0, 0, -0.001])
    ground = p.createMultiBody(0, col, vis, [0, 0, TABLE_Z])
    p.changeDynamics(ground, -1, lateralFriction=1.0)
    return ground


def _visual_box(half, pos, rgba):
    """Decoration only: no collision shape, so physics and grasping are unchanged."""
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    return p.createMultiBody(0, -1, vis, pos)


def _decorate(gui):
    """Work mat, safety border, robot base plate; bin labels in the GUI.

    Built in GUI and DIRECT alike, so the overhead camera sees the same scene in the demo and in
    the benchmark. Low saturation (mat) or hue outside every HSV range (yellow, H ~25) keeps the
    detector unaffected; tops sit 0.5 mm above the ground to avoid z-fighting."""
    (x0, x1), (y0, y1) = WORKSPACE["x"], WORKSPACE["y"]
    m, t, z = 0.03, 0.006, TABLE_Z + 0.0005
    cx, cy, hx, hy = (x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2 + m, (y1 - y0) / 2 + m
    _visual_box([hx, hy, 0.0005], [cx, cy, z - 0.0005], [0.62, 0.64, 0.67, 1])      # work mat
    yellow = [1.0, 0.85, 0.0, 1]
    for sx in (-1, 1):                                                                # safety border
        _visual_box([t, hy + t, 0.0006], [cx + sx * (hx + t), cy, z - 0.0004], yellow)
    for sy in (-1, 1):
        _visual_box([hx + t, t, 0.0006], [cx, cy + sy * (hy + t), z - 0.0004], yellow)
    _visual_box([0.13, 0.13, 0.0005], [0, 0, z - 0.0005], [0.25, 0.27, 0.3, 1])      # robot base plate
    if gui:
        for color, (bx, by) in BINS.items():
            p.addUserDebugText(color.upper(), [bx - 0.03, by, 0.16], RGBA[color][:3], textSize=1.4)
        p.addUserDebugText("VinSort", [0.0, 0.22, 0.02], [0.2, 0.25, 0.35], textSize=1.6)


def _load_robot():
    robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
    for j, q in enumerate(PANDA_REST):
        p.resetJointState(robot, j, q)
    for j in FINGER_JOINTS:
        p.resetJointState(robot, j, 0.04)
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=0.04, force=20)
    return robot


def _make_bin(color):
    """Low gray tray with a colored floor, centered on BINS[color]."""
    bx, by = BINS[color]
    s, h, t = BIN_SIZE / 2, 0.08, 0.005               # inner half-width, wall height, thickness
    # 8 cm walls: with 4 cm walls a cube dropped from RELEASE_Z bounced off another cube and out
    w = s + t                                          # outer half-width
    parts = [([w, w, t / 2], [0, 0, t / 2]),           # (halfExtents, center): floor
             ([t / 2, w, h / 2], [s + t / 2, 0, h / 2]), ([t / 2, w, h / 2], [-s - t / 2, 0, h / 2]),
             ([w, t / 2, h / 2], [0, s + t / 2, h / 2]), ([w, t / 2, h / 2], [0, -s - t / 2, h / 2])]
    col = p.createCollisionShapeArray([p.GEOM_BOX] * 5, halfExtents=[h for h, _ in parts],
                                      collisionFramePositions=[c for _, c in parts])
    floor_rgba = [0.5 + 0.5 * c for c in RGBA[color][:3]] + [1]
    vis = p.createVisualShapeArray([p.GEOM_BOX] * 5, halfExtents=[h for h, _ in parts],
                                   visualFramePositions=[c for _, c in parts],
                                   rgbaColors=[floor_rgba] + [[0.45, 0.45, 0.45, 1]] * 4)
    return p.createMultiBody(0, col, vis, [bx, by, TABLE_Z])


def _spawn_cube(color, x, y, yaw):
    half = [CUBE / 2] * 3
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=RGBA[color])
    oid = p.createMultiBody(CUBE_MASS, col, vis, [x, y, TABLE_Z + CUBE / 2 + 0.002],
                            p.getQuaternionFromEuler([0, 0, yaw]))
    p.changeDynamics(oid, -1, lateralFriction=0.8)
    return oid


def _sample_free_xy(rng, taken, tries=500):
    (x0, x1), (y0, y1) = WORKSPACE["x"], WORKSPACE["y"]
    for _ in range(tries):
        xy = np.array([rng.uniform(x0 + EDGE_MARGIN, x1 - EDGE_MARGIN),
                       rng.uniform(y0 + EDGE_MARGIN, y1 - EDGE_MARGIN)])
        if all(np.linalg.norm(xy - t) >= MIN_SPACING for t in taken):
            return xy
    return None


def settle(steps=SIM_HZ):
    for _ in range(steps):
        p.stepSimulation()


def build_scene(seed, n_objects=6, gui=True, settle_steps=SIM_HZ):
    """Same seed -> same layout. Seeds in SEEDS_FAILURE place two same-color cubes touching.

    Returns {"robot", "objects": [{"id", "color"}], "bins": {color: body_id}, "seed", "rng"}.
    Ground truth for D: p.getBasePositionAndOrientation(obj["id"]).
    """
    _connect(gui)
    _make_ground()
    _decorate(gui)
    robot = _load_robot()
    bins = {c: _make_bin(c) for c in COLORS}

    rng = np.random.default_rng(seed)
    objects, taken = [], []
    if seed in SEEDS_FAILURE:
        color = COLORS[seed % len(COLORS)]
        xy = _sample_free_xy(rng, taken)
        yaw = rng.uniform(-np.pi / 4, np.pi / 4)
        offset = (CUBE + 0.001) * np.array([np.cos(yaw), np.sin(yaw)])     # side by side, 1 mm gap
        for xy_i in (xy, xy + offset):
            objects.append({"id": _spawn_cube(color, *xy_i, yaw), "color": color})
        taken += [xy, xy + offset]

    while len(objects) < n_objects:
        xy = _sample_free_xy(rng, taken)
        if xy is None:
            break
        color = COLORS[len(objects) % len(COLORS)]       # balanced colors
        objects.append({"id": _spawn_cube(color, *xy, rng.uniform(-np.pi / 4, np.pi / 4)),
                        "color": color})
        taken.append(xy)

    settle(settle_steps)
    scene = {"robot": robot, "objects": objects, "bins": bins, "seed": seed,
             "rng": np.random.default_rng(seed + 10_000)}
    CURRENT.clear()
    CURRENT.update(scene)
    return scene


def add_random_object(scene, color=None):
    """Drop one cube at a free spot inside WORKSPACE (key `n` in main.py). Returns its id or None."""
    rng = scene["rng"]
    taken = [np.array(p.getBasePositionAndOrientation(o["id"])[0][:2]) for o in scene["objects"]]
    xy = _sample_free_xy(rng, taken)
    if xy is None:
        return None
    color = color or COLORS[int(rng.integers(len(COLORS)))]
    oid = _spawn_cube(color, *xy, rng.uniform(-np.pi / 4, np.pi / 4))
    scene["objects"].append({"id": oid, "color": color})
    return oid


def in_bin(obj_id, color):
    """True if the object rests inside the bin of `color` (used by D for 'correct bin' rate)."""
    x, y, z = p.getBasePositionAndOrientation(obj_id)[0]
    bx, by = BINS[color]
    return abs(x - bx) < BIN_SIZE / 2 and abs(y - by) < BIN_SIZE / 2 and z < 0.15   # stacked cubes count


def ground_truth(scene):
    """[(id, color, (x, y, z) of the cube center)] for every object in the scene."""
    return [(o["id"], o["color"], p.getBasePositionAndOrientation(o["id"])[0])
            for o in scene["objects"]]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--direct", action="store_true", help="no GUI, print and exit")
    args = ap.parse_args()

    sc = build_scene(args.seed, args.n, gui=not args.direct)
    for oid, color, pos in ground_truth(sc):
        print(f"id={oid:2d} {color:5s} xyz=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    if not args.direct:
        print("Ctrl+C to quit. Drag cubes with the left mouse button.")
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(1 / SIM_HZ)
        except KeyboardInterrupt:
            pass
