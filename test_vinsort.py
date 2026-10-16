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
    g = teleop.GripHysteresis(stable_frames=3)
    assert not g.update(closed=True) and not g.update(closed=True) and g.update(closed=True)
    assert g.update(opened=True) and g.update(opened=True) and not g.update(opened=True)
    # latch: after GRAB, nothing but RELEASE frees it (no gesture / other gestures / lost hand keep it)
    for _ in range(3): g.update(closed=True)
    assert all(g.update() for _ in range(20))
    for _ in range(3): g.update(opened=True)
    assert not any(g.update() for _ in range(20))
    g.update(opened=True); g.update(closed=True); g.update(opened=True)  # flicker never confirms
    assert not g.update(closed=True)
    g.force(True)                                                 # Space = instant GRAB, latched
    assert all(g.update() for _ in range(10))
    assert g.update(opened=True) and g.update(opened=True) and not g.update(opened=True)  # V releases
    fist = [(0.0, 0.0)] * 21
    for pip in (6, 10, 14, 18): fist[pip] = (0.0, 1.0)
    closed, opened, count, fingers = teleop.finger_pose(fist)
    assert closed and not opened and count == 0 and not any(fingers)
    np.testing.assert_allclose(teleop.rate_limit((0, 0, 0), (1, 0, 0)), (teleop.STEP_LIMIT, 0, 0))
    # joystick: center = stop; right = world -y (home view); up = world +x; quadratic, capped
    assert not np.any(teleop.joystick((0.5 + teleop.DEAD * 0.9, 0.5)))
    vr = teleop.joystick((0.9, 0.5))
    assert vr[1] < 0 and abs(vr[0]) < 1e-12
    assert teleop.joystick((0.5, 0.1))[0] > 0
    assert abs(np.linalg.norm(teleop.joystick((1.0, 0.5))) - teleop.V_MAX) < 1e-12
    assert np.linalg.norm(teleop.joystick((0.7, 0.5))) < np.linalg.norm(vr)
    # home view (yaw -90, behind robot): hand right -> world -y, hand up -> world +x (far)
    c = teleop.map_hand((0.5, 0.5), 0.2, -90)
    r = teleop.map_hand((0.6, 0.5), 0.2, -90)
    u = teleop.map_hand((0.5, 0.4), 0.2, -90)
    assert r[1] < c[1] and abs(r[0] - c[0]) < 1e-9
    assert u[0] > c[0] and abs(u[1] - c[1]) < 1e-9
    # free POV: camera yaw 0 -> screen right is world +x
    r0 = teleop.map_hand((0.6, 0.5), 0.2, 0)
    assert r0[0] > c[0] and abs(r0[1] - c[1]) < 1e-9
    # gain: 5% hand move -> 5% * GAIN * 2 * REACH
    assert abs((c[1] - r[1]) - 0.1 * teleop.GAIN * 2 * teleop.REACH) < 1e-9 or r[1] == WORKSPACE["y"][0]
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


def test_teleop_reaches_bin():
    s = build_scene(0, gui=False)
    robot = s["robot"]
    ids = [o["id"] for o in s["objects"]]
    target = s["objects"][0]
    x, y, _ = p.getBasePositionAndOrientation(target["id"])[0]
    bx, by = BINS[target["color"]]
    assert WORKSPACE["y"][0] < by < WORKSPACE["y"][1]
    # Pick a real cube, carry it above the bin wall, then descend and release.
    script = [((x, y, 0.20), False)] * 70 + [((x, y, CUBE + 0.01), False)] * 40
    script += [((x, y, CUBE + 0.01), True)] * 3
    script += [((bx, by, 0.35), True)] * 110
    script += [((bx, by, 0.20), True)] * 40
    script += [((bx, by, 0.20), False)] * 3
    state, tracker = {}, FakeTracker(script)
    for _ in script:
        state = teleop.teleop_step(robot, tracker, ids, state)
    assert state["cid"] is None
    for _ in range(240):
        p.stepSimulation()
    pos = p.getBasePositionAndOrientation(target["id"])[0]
    assert abs(pos[0] - bx) < 0.07 and abs(pos[1] - by) < 0.07, pos


def test_full_hybrid_story():
    """Failure seed: AUTO sorts sure parts, operator separates the touching pair,
    lost hand keeps grip, 'a' resumes AUTO to sort the remaining objects."""
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
