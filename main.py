"""[A] Orchestrator: AUTO <-> TELEOP + HUD (OpenCV window).

Keys (OpenCV HUD window OR PyBullet window):
  a AUTO (home, rescan) | t TELEOP | r rescan | n add a random part | q quit
  TELEOP keyboard fallback: i/k j/l u/o move, space grip
  Free POV: 1/2 rotate left/right | 3/4 tilt | 5/6 zoom in/out | 0 reset view
  v toggle 4-camera view (side / gripper / front / top); enabled at launch
  Mouse (PyBullet): Ctrl+left drag rotate, middle drag pan, wheel zoom, left drag = move a part.
Teleop controls follow the current camera yaw: hand right is always screen right.
"""
import argparse
import os
import random
import time

import cv2
import numpy as np
import pybullet as p

from common import BIN_HALF, BINS, COLORS
import controller
import multiview
import perception
import scene
import teleop

FAIL_RADIUS = 0.02  # detections within 2 cm of a failed grasp are flagged "grasp_fail"


class Orchestrator:
    def __init__(self, seed, tracker, realtime=True, show=True):
        self.s = scene.build_scene(seed, gui=show)
        self.robot = self.s["robot"]
        self.ids = [o["id"] for o in self.s["objects"]]
        self.colors = {o["id"]: o["color"] for o in self.s["objects"]}
        self.tracker, self.realtime, self.show = tracker, realtime, show
        self.rng = random.Random(seed + 7)
        self.mode, self.msg, self.alert = "AUTO", "", False
        self.failed = []          # xy of failed grasps (P0 fix #1: never retry the same spot forever)
        self.sorted = 0
        self.dets, self.rgb = [], None
        self.tstate = {}
        self.log = []             # mode transitions, for the smoke test
        self.mv = None
        if show and os.environ.get("VINSORT_MV", "1") == "1":
            self.mv = multiview.MultiView(self.robot)  # show all four angles immediately
            controller.ON_STEP = self._refresh_views

    def _refresh_views(self):
        if self.mode != "TELEOP":
            self.mv.update()  # AUTO: synchronous four POVs; no camera input to delay

    # -- perception -----------------------------------------------------------------
    def scan(self):
        self.rgb, depth, _ = perception.capture()
        self.dets = perception.detect(self.rgb, depth)
        for d in self.dets:
            if not d.suspicious and any(np.hypot(d.xyz[0] - x, d.xyz[1] - y) < FAIL_RADIUS for x, y in self.failed):
                d.suspicious, d.reason = True, "grasp_fail"
        return self.dets

    # -- modes ----------------------------------------------------------------------
    def set_mode(self, mode, msg="", alert=False):
        if mode == "TELEOP" and self.mode != "TELEOP":
            self.tstate = {}
        if mode == "AUTO" and self.mode == "TELEOP":
            controller.release(self.tstate.get("cid"))
            self.failed.clear()  # operator intervened: allow one fresh attempt
            controller.go_home(self.robot, self.realtime)
            self.scan()
        self.mode, self.msg, self.alert = mode, msg, alert
        self.log.append(mode)

    def auto_step(self):
        self.scan()
        ok = [d for d in self.dets if not d.suspicious]
        if ok:
            d = min(ok, key=lambda d: np.hypot(d.xyz[0], d.xyz[1]))  # nearest to the robot base
            self.msg, self.alert = f"PICK {d.color} at {d.xyz[0]:.3f},{d.xyz[1]:.3f}", False
            self.render()
            if controller.pick_and_place(self.robot, d, self.ids, realtime=self.realtime):
                self.sorted += 1
            else:
                self.failed.append(d.xyz[:2])
            self.scan()
        elif self.dets:
            reasons = ",".join(sorted({d.reason for d in self.dets}))
            self.set_mode("TELEOP", f"CAN NGUOI VAN HANH: {reasons}", alert=True)
        else:
            self.mode, self.msg, self.alert = "DONE", "XONG - r: quet lai, n: them vat", False
            self.log.append("DONE")

    def handle_key(self, key):
        views = {ord("1"): (-15, 0, 1), ord("2"): (15, 0, 1), ord("3"): (0, 5, 1),
                 ord("4"): (0, -5, 1), ord("5"): (0, 0, 0.85), ord("6"): (0, 0, 1.18)}
        if self.show and key in views:
            scene.nudge_view(*views[key])
            return
        if self.show and key == ord("0"):
            scene.reset_view()
            return
        if self.mv and key == ord("v"):
            self.msg = f"CAMERA: {self.mv.toggle()}"
            return
        if key == ord("a"):
            self.set_mode("AUTO")
        elif key == ord("t"):
            self.set_mode("TELEOP", "TELEOP (operator)")
        elif key == ord("r"):
            self.scan()
            if self.mode == "DONE":
                self.set_mode("AUTO")
        elif key == ord("n"):
            xy = scene.random_free_xy(self.rng, self.ids)
            if xy:
                c = self.rng.choice(COLORS)
                oid = scene.add_cube(c, *xy, yaw=self.rng.uniform(-0.7, 0.7), z=0.1)
                self.ids.append(oid)
                self.colors[oid] = c
                scene.settle(120)
                if self.mode == "DONE":
                    self.set_mode("AUTO")

    def tick(self, key=-1):
        if key != -1:
            self.handle_key(key)
        if self.mode == "AUTO":
            self.auto_step()
        elif self.mode == "TELEOP":
            self.tstate = teleop.teleop_step(self.robot, self.tracker, self.ids, self.tstate,
                                             key=key, realtime=self.realtime)
            if self.mv:
                self.mv.update(teleop=True)  # render after servo, not before reading the next webcam frame
        else:  # DONE: keep stepping so judges can drag parts with the mouse
            for _ in range(8):
                p.stepSimulation()
                if self.realtime:
                    time.sleep(1 / 240)

    def in_bin(self, oid):
        x, y, _ = p.getBasePositionAndOrientation(oid)[0]
        bx, by = BINS[self.colors[oid]]
        return abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF

    def in_correct_bin(self):
        return sum(self.in_bin(oid) for oid in self.ids)

    # -- HUD ------------------------------------------------------------------------
    def render(self):
        if not self.show:
            return
        if self.rgb is None:
            self.scan()
        img = perception.draw(self.rgb, self.dets)
        bar = np.zeros((70, img.shape[1], 3), np.uint8)
        if self.alert:
            bar[:] = (0, 0, 180)
        lat = self.tstate.get("latency_ms")
        hand = self.tstate.get("hand")
        line1 = f"MODE {self.mode} | sorted {self.sorted} | in-bin {self.in_correct_bin()}/{len(self.ids)}"
        if self.mode == "TELEOP":
            line1 += f" | hand {'OK' if hand else 'LOST-HOLD'}" + (f" | {lat:.0f} ms" if lat else "")
            tgt = self.tstate.get("target")
            if tgt is not None:
                line1 += f" | z {tgt[2]:.2f}"
        cv2.putText(bar, line1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(bar, self.msg, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow("VinSort HUD", np.vstack([bar, img]))
        if self.mv and self.mode != "TELEOP":
            self.mv.update()
        frame = self.tstate.get("frame")
        if self.mode == "TELEOP" and frame is not None:
            cv2.imshow("Operator", frame)


def read_key(wait_ms=1):
    """P0 fix #2: keys from the OpenCV HUD OR the PyBullet window (whichever has focus)."""
    k = cv2.waitKey(wait_ms) & 0xFF
    if k != 0xFF:
        return k
    for code, state in p.getKeyboardEvents().items():
        if state & p.KEY_WAS_TRIGGERED and 0 < code < 256:
            return code
    return -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--keyboard", action="store_true", help="teleop with keyboard instead of webcam")
    ap.add_argument("--device", type=int, default=0)
    args = ap.parse_args()
    tracker = None
    if not args.keyboard:
        try:
            tracker = teleop.HandTracker(args.device)
        except Exception as e:  # no webcam: keyboard fallback, demo continues
            print("webcam unavailable, keyboard teleop:", e)
    tracker = tracker or teleop.KeyboardTracker()
    o = Orchestrator(args.seed, tracker)
    try:
        while p.isConnected():
            o.render()
            key = read_key()
            if key == ord("q"):
                break
            o.tick(key)
    finally:
        tracker.close()
        cv2.destroyAllWindows()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
