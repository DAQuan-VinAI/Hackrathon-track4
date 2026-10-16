"""Conveyor mode for the demo: parts ride in on a belt, it stops under the camera, the robot sorts.

    python main.py --conveyor            # belt feeding parts; every 5th spawn is a touching pair

The belt runs along -y at x = BELT_X, from outside the camera view (y = 0.80) to the end of the
workspace. It stops when the leading loose part reaches STOP_Y (center of the view); main.py then
runs normal AUTO cycles (HOME -> capture -> detect -> pick) on the parts in PICK_ZONE_Y and starts
the belt again once none are left. A suspicious blob (the touching pair) hands off to TELEOP as usual.
Default scenes, the benchmark and the tests do not use this file.
"""
import numpy as np
import pybullet as p

from common import COLORS, CUBE, TABLE_Z
from scene import _spawn_cube

BELT_X = 0.55
BELT_HALF_W = 0.07
BELT_Y0, BELT_Y1 = 0.80, -0.22      # start (outside the camera view, y > 0.36) and end
SPEED = 0.15                        # m/s
STOP_Y = 0.0                        # belt stops when the leading part reaches this y
PICK_ZONE_Y = 0.25                  # main.py only picks parts with y below this (fully in view)
SPACING = 0.16                      # m between spawned parts
PAIR_EVERY = 5                      # every Nth spawn is two same-color cubes touching (failure case)
MAX_ON_BELT = 6
N_BARS = 14                         # moving cross bars: make the belt motion visible


def _visual_box(half, pos, rgba):
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
    return p.createMultiBody(0, -1, vis, pos)


class Conveyor:
    def __init__(self, scene, seed=0):
        self.scene = scene
        self.rng = np.random.default_rng(seed + 20_000)
        self.running = False
        self.spawned = 0
        self.offset = 0.0               # bar animation phase (m)
        length = BELT_Y0 - BELT_Y1
        cy = (BELT_Y0 + BELT_Y1) / 2
        z = TABLE_Z + 0.0008            # above the work mat (0.5 mm), visual only: parts rest on the ground
        # dark low-saturation surfaces: invisible to the HSV masks
        _visual_box([BELT_HALF_W, length / 2, 0.0008], [BELT_X, cy, z - 0.0008], [0.22, 0.23, 0.25, 1])
        for sx in (-1, 1):
            _visual_box([0.008, length / 2, 0.008], [BELT_X + sx * (BELT_HALF_W + 0.008), cy, 0.008],
                        [0.55, 0.56, 0.58, 1])
        _visual_box([BELT_HALF_W + 0.016, 0.01, 0.015], [BELT_X, BELT_Y1 - 0.01, 0.015], [0.55, 0.56, 0.58, 1])
        self.bar_pitch = length / N_BARS
        self.bars = [_visual_box([BELT_HALF_W - 0.004, 0.004, 0.0004], [BELT_X, BELT_Y0, z + 0.0004],
                                 [0.32, 0.33, 0.35, 1]) for _ in range(N_BARS)]
        self._place_bars()
        if p.getConnectionInfo().get("connectionMethod") == p.GUI:
            p.addUserDebugText("CONVEYOR", [BELT_X + 0.10, BELT_Y0 - 0.05, 0.02], [0.3, 0.3, 0.35], textSize=1.3)
            p.addUserDebugLine([BELT_X, BELT_Y0 - 0.12, 0.03], [BELT_X, BELT_Y0 - 0.30, 0.03], [0.3, 0.3, 0.35], 3)

    # ---------- parts ----------
    def _on_belt(self, pos):
        return abs(pos[0] - BELT_X) < BELT_HALF_W + 0.01 and pos[2] < CUBE + 0.01 and BELT_Y1 - 0.02 < pos[1] < BELT_Y0 + 0.05

    def loose_parts(self):
        """[(id, pos)] of parts resting on the belt (not held, not in a bin)."""
        out = []
        for o in self.scene["objects"]:
            pos = p.getBasePositionAndOrientation(o["id"])[0]
            if self._on_belt(pos):
                out.append((o["id"], pos))
        return out

    def waiting(self):
        """True when a part has reached the stop line: the robot should scan and pick."""
        return any(pos[1] <= STOP_Y for _, pos in self.loose_parts())

    def spawn(self):
        """Drop the next part (or a touching pair) at the belt start if there is room."""
        parts = self.loose_parts()
        if len(parts) >= MAX_ON_BELT or any(pos[1] > BELT_Y0 - SPACING for _, pos in parts):
            return []
        self.spawned += 1
        color = COLORS[int(self.rng.integers(len(COLORS)))]
        if self.spawned % PAIR_EVERY == 0:
            xs = [BELT_X - (CUBE + 0.001) / 2, BELT_X + (CUBE + 0.001) / 2]     # side by side, 1 mm gap
            yaw = 0.0
        else:
            xs = [BELT_X + self.rng.uniform(-0.02, 0.02)]
            yaw = self.rng.uniform(-np.pi / 4, np.pi / 4)
        ids = []
        for x in xs:
            oid = _spawn_cube(color, x, BELT_Y0, yaw)
            self.scene["objects"].append({"id": oid, "color": color})
            ids.append(oid)
        return ids

    # ---------- belt ----------
    def _place_bars(self):
        for i, b in enumerate(self.bars):
            y = BELT_Y0 - ((i * self.bar_pitch + self.offset) % (BELT_Y0 - BELT_Y1))
            p.resetBasePositionAndOrientation(b, [BELT_X, y, TABLE_Z + 0.0012], [0, 0, 0, 1])

    def run(self, n_steps, sim_hz):
        """Advance n_steps with the belt moving. Stops (and freezes the parts) once a part waits."""
        self.running = True
        new = self.spawn()
        for _ in range(n_steps):
            for oid, pos in self.loose_parts():
                if pos[1] > BELT_Y1 + 0.03:                 # the end stop holds the last part
                    v = p.getBaseVelocity(oid)[0]
                    p.resetBaseVelocity(oid, [0, -SPEED, v[2]], [0, 0, 0])
            p.stepSimulation()
        self.offset += SPEED * n_steps / sim_hz
        self._place_bars()
        if self.waiting():
            self.stop()
        return new

    def stop(self):
        self.running = False
        for oid, _ in self.loose_parts():
            p.resetBaseVelocity(oid, [0, 0, 0], [0, 0, 0])
