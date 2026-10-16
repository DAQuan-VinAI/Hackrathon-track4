"""Self-check, headless (p.DIRECT, no webcam). Run: python test_vinsort.py"""
import numpy as np
import pybullet as p

import controller
import perception
import teleop
from common import BINS, CUBE, WORKSPACE
from main import Orchestrator
from scene import build_scene


class FakeTracker:
    """Scripted hand: list of (xyz or None, grip)."""
    def __init__(self, script):
        self.script, self.i, self.t0 = script, 0, 0.0

    def read(self, key=-1):
        import time
        self.t0 = time.perf_counter()
        xyz, grip = self.script[min(self.i, len(self.script) - 1)]
        self.i += 1
        return (None if xyz is None else np.array(xyz)), grip, None, 0.0

    def close(self):
        pass


def test_pure_helpers():
    assert teleop.GripHysteresis().update(0.3) is False          # between thresholds: no change
    g = teleop.GripHysteresis()
    assert g.update(0.2) and g.update(0.3) and not g.update(0.45)  # hysteresis
    np.testing.assert_allclose(teleop.rate_limit((0, 0, 0), (1, 0, 0)), (teleop.STEP_LIMIT, 0, 0))
    lo = teleop.map_hand((0.0, 0.0), 0.0)
    hi = teleop.map_hand((1.0, 1.0), 1.0)
    assert lo[1] == WORKSPACE["y"][1] and hi[1] == WORKSPACE["y"][0]   # screen right -> -y
    assert lo[0] == WORKSPACE["x"][1] and lo[2] == WORKSPACE["z"][1]   # screen top -> far; small hand -> high
    c0, c1 = teleop.map_hand((0.5, 0.5), 0.21), teleop.map_hand((0.55, 0.5), 0.21)
    span = WORKSPACE["y"][1] - WORKSPACE["y"][0]
    assert abs(abs(c1[1] - c0[1]) - 0.05 * teleop.GAIN * span) < 1e-9  # hand move x GAIN
    W, H = 640, 480
    view, proj = perception.matrices()
    c = perception.pixel_to_world(W / 2 - 0.5, H / 2 - 0.5, 0.5, view, proj, W, H)
    assert abs(c[0] - 0.55) < 1e-6 and abs(c[1]) < 1e-6              # image center -> camera axis


def test_detect_accuracy():
    s = build_scene(0, gui=False)
    rgb, depth, _ = perception.capture()
    dets = perception.detect(rgb, depth)
    assert len(dets) == 6 and not any(d.suspicious for d in dets)
    for d in dets:
        gt = min((p.getBasePositionAndOrientation(o["id"])[0] for o in s["objects"] if o["color"] == d.color),
                 key=lambda g: np.hypot(g[0] - d.xyz[0], g[1] - d.xyz[1]))
        assert np.hypot(gt[0] - d.xyz[0], gt[1] - d.xyz[1]) < 0.005
        assert abs(d.xyz[2] - CUBE) < 0.003


def test_auto_e2e_and_hybrid_handoff():
    o = Orchestrator(100, FakeTracker([(None, False)]), realtime=False, show=False)
    for _ in range(12):
        o.tick()
        if o.mode != "AUTO":
            break
    assert o.mode == "TELEOP" and o.alert, (o.mode, o.msg)
    assert "area" in o.msg or "aspect" in o.msg
    assert o.in_correct_bin() == 4 and o.sorted == 4          # 4 normal parts sorted, 2 touching left


def test_full_hybrid_story():
    """Failure seed: AUTO sorts the sure parts, hands off, operator separates the touching pair
    (hand lost mid-way -> hold + keep part), 'a' -> AUTO sorts the rest. Bins are outside the
    teleop safe box by design: the operator only fixes the ambiguity, the robot does the sorting."""
    import random
    from scene import random_free_xy
    o = Orchestrator(100, FakeTracker([(None, False)]), realtime=False, show=False)
    while o.mode == "AUTO":
        o.tick()
    assert o.mode == "TELEOP"
    pair = [oid for oid in o.ids if not o.in_bin(oid)]
    assert len(pair) == 2
    x, y, _ = p.getBasePositionAndOrientation(pair[0])[0]
    others = [q for q in o.ids if q != pair[0]]
    fx, fy = random_free_xy(random.Random(0), others, min_dist=0.10)
    hover, low = (x, y, 0.20), (x, y, CUBE + 0.01)
    dst_hi, dst_lo = (fx, fy, 0.20), (fx, fy, CUBE + 0.015)
    script = [(hover, False)] * 70 + [(low, False)] * 40 + [(low, True)] * 3 + [(hover, True)] * 30
    script += [(None, True)] * 20                               # hand lost: hold and keep the part
    script += [(dst_hi, True)] * 50 + [(dst_lo, True)] * 30 + [(dst_lo, False)] * 3 + [(dst_hi, False)] * 30
    o.tracker = FakeTracker(script)
    held = None
    for i in range(len(script)):
        o.tick()
        if i == 142:
            held = np.array(controller.ee_pos(o.robot))
            assert o.tstate["cid"] is not None, "part should be grasped"
        if 143 <= i < 162:
            assert not o.tstate["hand"] and o.tstate["cid"] is not None
            assert np.linalg.norm(np.array(controller.ee_pos(o.robot)) - held) < 0.01, "must hold still"
    assert o.tstate["cid"] is None
    nx, ny, _ = p.getBasePositionAndOrientation(pair[0])[0]
    assert np.hypot(nx - x, ny - y) > 0.06, "operator separated the pair"
    o.tick(ord("a"))
    while o.mode == "AUTO":
        o.tick()
    assert o.mode == "DONE" and o.in_correct_bin() == 6, (o.mode, o.in_correct_bin())


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("all tests passed")
