"""Headless self-check (p.DIRECT, no webcam, no windows). Run: python test_vinsort.py"""
import time

import numpy as np
import pybullet as p

import controller
import perception
import teleop
from common import CAM, CUBE, WORKSPACE
from scene import build_scene, in_bin


class FakeTracker:
    """Scripted hand: list of (xyz or None, grip)."""

    def __init__(self, script):
        self.script, self.i, self.t0 = script, 0, 0.0

    def read(self, key=-1):
        self.t0 = time.perf_counter()
        xyz, grip = self.script[min(self.i, len(self.script) - 1)]
        self.i += 1
        return (None if xyz is None else np.array(xyz)), grip, None, 0.0


def auto_until_handoff(sc):
    """main.py AUTO rule: pick the nearest confident cube; stop when only suspicious ones remain."""
    robot, ids = sc["robot"], [o["id"] for o in sc["objects"]]
    for _ in range(2 * len(ids)):
        controller.go_home(robot)
        dets = perception.detect()
        ok = [d for d in dets if not d.suspicious]
        if not ok:
            return [d for d in dets if d.suspicious]
        det = min(ok, key=lambda d: np.hypot(d.xyz[0], d.xyz[1]))
        assert controller.pick_and_place(robot, det, ids), det
    raise AssertionError("AUTO did not converge")


def free_xy(sc, exclude, min_dist=0.10):
    """A spot inside WORKSPACE at least `min_dist` from every cube except `exclude`."""
    taken = [p.getBasePositionAndOrientation(o["id"])[0][:2] for o in sc["objects"] if o["id"] != exclude]
    for x in np.linspace(WORKSPACE["x"][0] + 0.03, WORKSPACE["x"][1] - 0.03, 9):
        for y in np.linspace(WORKSPACE["y"][0] + 0.03, WORKSPACE["y"][1] - 0.03, 9):
            if all(np.hypot(x - tx, y - ty) >= min_dist for tx, ty in taken):
                return x, y
    raise AssertionError("no free spot")


def test_pure_helpers():
    assert teleop.GripHysteresis().update(0.3) is False            # between thresholds: no change
    g, n = teleop.GripHysteresis(), teleop.GRIP_FRAMES
    assert [g.update(0.2) for _ in range(n)][-1]                     # pinch held n frames: grab
    assert g.update(0.3) and g.update(closed=False, opened=True)    # 1 frame of open palm: not yet
    assert [g.update(opened=True) for _ in range(n)][-1] is False   # open palm held: release
    g.update(closed=True); g.update(0.45)                           # a lone fist frame is dropped
    assert not g.grip and g.candidate is None
    np.testing.assert_allclose(teleop.rate_limit((0, 0, 0), (1, 0, 0)), (teleop.STEP_LIMIT, 0, 0))
    assert not np.any(teleop.joystick((0.5, 0.5)))                  # dead zone: stop
    right, up = teleop.joystick((1.0, 0.5)), teleop.joystick((0.5, 0.0))
    assert right[1] < 0 and abs(right[0]) < 1e-9                    # default view: screen right -> -y
    assert up[0] > 0 and abs(up[1]) < 1e-9                          # screen top -> away from the robot (+x)
    W, H = CAM["W"], CAM["H"]
    view, proj = perception._matrices()
    c = perception.pixel_to_world(W / 2 - 0.5, H / 2 - 0.5, 0.5, view, proj, W, H)
    assert abs(c[0] - CAM["target"][0]) < 1e-6 and abs(c[1]) < 1e-6   # image center -> camera axis


def test_detect_accuracy():
    sc = build_scene(0, gui=False)
    controller.go_home(sc["robot"])
    dets = perception.detect()
    assert len(dets) == 6 and not any(d.suspicious for d in dets)
    for d in dets:
        gt = min((p.getBasePositionAndOrientation(o["id"])[0] for o in sc["objects"] if o["color"] == d.color),
                 key=lambda g: np.hypot(g[0] - d.xyz[0], g[1] - d.xyz[1]))
        assert np.hypot(gt[0] - d.xyz[0], gt[1] - d.xyz[1]) < 0.005
        assert abs(d.xyz[2] - CUBE) < 0.003


def test_keyboard_key_comes_from_state():
    """main.py passes keys via state["key"]; i must move the target away from the robot (+x)."""
    sc = build_scene(0, gui=False)
    tr = teleop.KeyboardTracker()
    st = teleop.teleop_step(sc["robot"], tr, [], {"key": -1})
    x0 = tr.pos[0]
    st["key"] = ord("i")
    teleop.teleop_step(sc["robot"], tr, [], st)
    assert tr.pos[0] > x0


def test_reentry_does_not_grab():
    """Leaving TELEOP with the grip toggled on must not grab a cube on the next entry."""
    sc = build_scene(0, gui=False)
    robot, ids = sc["robot"], [o["id"] for o in sc["objects"]]
    x, y, _ = p.getBasePositionAndOrientation(ids[0])[0]
    tr = teleop.KeyboardTracker()
    st = teleop.teleop_step(robot, tr, ids, {"key": -1})
    tr.grip = True                                                  # operator left with space toggled on
    for _ in range(60):
        controller.servo_ee(robot, (x, y, CUBE + 0.008))            # arm parked right above a cube
    st = teleop.teleop_step(robot, tr, ids, {"key": -1})            # fresh state = new TELEOP entry
    assert st["cid"] is None and not tr.grip


def test_hybrid_story():
    """Failure seed: AUTO sorts the sure cubes and hands off; the operator separates the touching
    pair (hand lost mid-way -> hold + keep the cube); back to AUTO, everything ends in its bin."""
    sc = build_scene(100, gui=False)
    robot = sc["robot"]
    left = auto_until_handoff(sc)
    assert left and left[0].reason in ("area", "aspect"), left
    pair = [o["id"] for o in sc["objects"] if not in_bin(o["id"], o["color"])]
    assert len(pair) == 2 and sum(in_bin(o["id"], o["color"]) for o in sc["objects"]) == 4

    x, y, _ = p.getBasePositionAndOrientation(pair[0])[0]
    fx, fy = free_xy(sc, pair[0])
    hover, low = (x, y, 0.20), (x, y, CUBE + 0.008)
    dst_hi, dst_lo = (fx, fy, 0.20), (fx, fy, CUBE + 0.015)
    script = [(hover, False)] * 70 + [(low, False)] * 40 + [(low, True)] * 3 + [(hover, True)] * 30
    script += [(None, True)] * 20                                   # hand lost: hold and keep the cube
    script += [(dst_hi, True)] * 50 + [(dst_lo, True)] * 30 + [(dst_lo, False)] * 3 + [(dst_hi, False)] * 30
    tr, st = FakeTracker(script), {"key": -1}
    held = None
    for i in range(len(script)):
        st = teleop.teleop_step(robot, tr, [o["id"] for o in sc["objects"]], st)
        if i == 142:
            held = controller.ee_pos(robot)
            assert st["cid"] is not None, "cube should be grasped"
        if 143 <= i < 162:
            assert not st["hand"] and st["cid"] is not None
            assert np.linalg.norm(controller.ee_pos(robot) - held) < 0.01, "must hold still"
    assert st["cid"] is None
    nx, ny, _ = p.getBasePositionAndOrientation(pair[0])[0]
    assert np.hypot(nx - x, ny - y) > 0.06, "operator separated the pair"

    assert auto_until_handoff(sc) == []
    assert all(in_bin(o["id"], o["color"]) for o in sc["objects"])


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("all tests passed")
