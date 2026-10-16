"""VinSort demo: AUTO sorting with hand-off to gesture TELEOP when unsure (owner: A).

    python main.py --seed 0           # normal layout
    python main.py --seed 100         # failure case: two same-color cubes touching
    python main.py --keyboard         # force keyboard teleop (no webcam)

Keys (focus the HUD or the PyBullet window): a AUTO · t TELEOP · r rescan · n add cube · q quit
Keyboard teleop: i/k forward/back · j/l left/right · u/o up/down · space grip
"""
import argparse
import time

import cv2
import numpy as np
import pybullet as p

import controller
import perception
import stubs
import teleop
from common import CAM, SIM_HZ
from controller import go_home, pick_and_place, release
from perception import capture, detect
from scene import add_random_object, build_scene, in_bin

PERCEPTION = "STUB (oracle)" if getattr(perception, "STUB", False) else "perception.py"

FPS = 30
STEPS_PER_FRAME = SIM_HZ // FPS
CAM_REFRESH = 6                   # re-render the overhead view every N frames outside AUTO (TINY renderer is slow)
MAX_PICK_FAILS = 2                # consecutive failed grasps before asking the operator
WINDOW = "VinSort HUD"
DRAW = {"red": (40, 40, 230), "green": (60, 190, 60), "blue": (230, 90, 40)}   # BGR


def read_key():
    """One key code or -1, from the HUD (cv2) or the PyBullet window.

    On macOS the PyBullet GUI pumps the Cocoa event loop itself and swallows key events,
    so cv2.waitKey alone often never sees them; p.getKeyboardEvents() does.
    """
    k = cv2.waitKey(1)
    if k != -1:
        return k & 0xFF
    events = p.getKeyboardEvents()
    for code, state in events.items():
        if state & p.KEY_WAS_TRIGGERED and code < 256:
            return code
    for code, state in events.items():                # hold a move key to keep moving
        if state & p.KEY_IS_DOWN and code in stubs.MOVE_KEYS:
            return code
    return -1


def load_teleop(force_keyboard):
    """Returns (tracker, teleop_step, name). Prefers C's HandTracker, falls back to keyboard."""
    step = getattr(teleop, "teleop_step", stubs.teleop_step)
    if not force_keyboard and hasattr(teleop, "HandTracker"):
        try:
            return teleop.HandTracker(), step, "hand"
        except Exception as e:                # no webcam, no permission, mediapipe missing...
            print(f"[main] HandTracker failed ({e}), using keyboard")
    return getattr(teleop, "KeyboardTracker", stubs.KeyboardTracker)(), step, "keyboard"


class App:
    def __init__(self, args):
        self.args = args
        self.scene = build_scene(args.seed, args.n, gui=True)
        self.robot = self.scene["robot"]
        self.mode = "IDLE"            # IDLE | AUTO | DONE | TELEOP
        self.alert = ""               # red banner text, set when the robot hands over to the operator
        self.msg = "press a to start"
        self.rgb = np.zeros((CAM["H"], CAM["W"], 3), np.uint8)
        self.dets = []
        self.pending_key = -1
        self.quit = False
        self.frame_i = 0
        self.pick_fails = 0
        self.tracker = self.teleop_step = None
        self.tstate = {}
        go_home(self.robot)
        self.refresh_camera()

    @property
    def object_ids(self):
        return [o["id"] for o in self.scene["objects"]]

    # ---------- perception ----------
    def refresh_camera(self, run_detect=True):
        if run_detect:                        # TINY renderer (~0.5 s), same images as D's benchmark
            self.rgb, depth, _ = capture()
            self.dets = detect(self.rgb, depth)
        else:                                 # display only: GUI OpenGL renderer at half size, much faster
            view, proj = stubs._matrices()
            w, h = CAM["W"] // 2, CAM["H"] // 2
            _, _, rgba, _, _ = p.getCameraImage(w, h, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                                flags=p.ER_NO_SEGMENTATION_MASK)
            small = np.reshape(rgba, (h, w, 4))[:, :, :3].astype(np.uint8)
            self.rgb = cv2.resize(small, (CAM["W"], CAM["H"]))

    # ---------- HUD ----------
    def sorted_count(self):
        return sum(in_bin(o["id"], o["color"]) for o in self.scene["objects"])

    def draw_hud(self):
        img = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
        for d in self.dets:
            r = max(8, int(np.sqrt(max(d.area, 1)) / 2) + 4)
            u, v = int(d.uv[0]), int(d.uv[1])
            box_color = (0, 165, 255) if d.suspicious else DRAW.get(d.color, (255, 255, 255))
            cv2.rectangle(img, (u - r, v - r), (u + r, v + r), box_color, 3 if d.suspicious else 2)
            label = f"{d.color} ({d.xyz[0]:.2f},{d.xyz[1]:.2f})" + (f" ? {d.reason}" if d.suspicious else "")
            cv2.putText(img, label, (u - r, v - r - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1, cv2.LINE_AA)

        bar_h = 70
        hud = np.full((bar_h + img.shape[0], img.shape[1], 3), 30, np.uint8)
        hud[bar_h:] = img
        mode_color = {"AUTO": (80, 200, 80), "TELEOP": (0, 200, 255), "DONE": (200, 200, 200)}.get(self.mode, (180, 180, 180))
        cv2.putText(hud, self.mode, (10, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, mode_color, 2, cv2.LINE_AA)
        phase = controller.phase if self.mode == "AUTO" else ""
        info = f"sorted {self.sorted_count()}/{len(self.scene['objects'])}  seed {self.args.seed}  {phase}"
        cv2.putText(hud, info, (190, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA)
        status = self.msg
        if self.mode == "TELEOP" and self.tracker is not None:
            lat = self.tstate.get("latency_ms")
            status = f"{self.tracker_name} teleop" + (f"  latency {lat:.0f} ms" if lat is not None else "") \
                     + (f"  {self.tstate['msg']}" if self.tstate.get("msg") else "")
        cv2.putText(hud, status, (190, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(hud, f"perception: {PERCEPTION}", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        if self.alert:
            y0 = bar_h + 8
            cv2.rectangle(hud, (8, y0), (hud.shape[1] - 8, y0 + 40), (0, 0, 200), -1)
            cv2.putText(hud, self.alert, (18, y0 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        frame = self.tstate.get("frame") if self.mode == "TELEOP" else None
        if frame is not None:                 # webcam preview from C, bottom-right corner
            h, w = 150, int(150 * frame.shape[1] / frame.shape[0])
            hud[-h - 8:-8, -w - 8:-8] = cv2.resize(frame, (w, h))

        cv2.putText(hud, "a AUTO  t TELEOP  r rescan  n add  q quit", (10, hud.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW, hud)

    def pump(self):
        """Called every sim step inside blocking robot moves: keep the HUD alive, buffer keys."""
        self.frame_i += 1
        if self.frame_i % STEPS_PER_FRAME:
            return
        self.draw_hud()
        k = read_key()
        if k != -1:
            self.pending_key = k

    # ---------- modes ----------
    def enter_auto(self, msg="scanning"):
        if self.tstate.get("cid") is not None:
            release(self.tstate["cid"])
        self.tstate = {}
        self.alert, self.msg, self.pick_fails = "", msg, 0
        self.mode = "AUTO"

    def enter_teleop(self, alert=""):
        if self.tracker is None:
            self.tracker, self.teleop_step, self.tracker_name = load_teleop(self.args.keyboard)
        self.tstate = {"cid": None, "key": -1, "target": None}
        self.alert = alert
        self.mode = "TELEOP"

    def auto_tick(self):
        """One AUTO cycle: HOME -> capture -> detect -> pick the nearest confident cube."""
        realtime = not self.args.fast
        go_home(self.robot, realtime=realtime, on_step=self.pump)
        self.refresh_camera()
        ok = [d for d in self.dets if not d.suspicious]
        unsure = [d for d in self.dets if d.suspicious]
        if ok:
            det = min(ok, key=lambda d: np.hypot(d.xyz[0], d.xyz[1]))   # nearest to the robot base
            self.msg = f"picking {det.color} at ({det.xyz[0]:.3f}, {det.xyz[1]:.3f})"
            if pick_and_place(self.robot, det, self.object_ids, realtime=realtime, on_step=self.pump):
                self.pick_fails = 0
            else:
                self.pick_fails += 1
                self.msg = f"grasp failed ({self.pick_fails}/{MAX_PICK_FAILS})"
                if self.pick_fails >= MAX_PICK_FAILS:
                    self.enter_teleop("CAN NGUOI VAN HANH: grasp failed")
        elif unsure:
            self.enter_teleop(f"CAN NGUOI VAN HANH: {unsure[0].reason} ({len(unsure)} uncertain)")
        else:
            self.mode, self.msg = "DONE", "XONG - r rescan, n add cube"

    def handle_key(self, k):
        if k == ord("q"):
            self.quit = True
        elif k == ord("a"):
            self.enter_auto()
        elif k == ord("r"):
            self.enter_auto("rescan")
        elif k == ord("t") and self.mode != "TELEOP":
            self.enter_teleop()
        elif k == ord("n"):
            oid = add_random_object(self.scene)
            self.msg = "added a cube" if oid is not None else "no free spot"

    def run(self):
        next_t = time.perf_counter()
        while not self.quit:
            k, self.pending_key = self.pending_key, -1
            if k == -1:
                k = read_key()
            if k in (ord("q"), ord("a"), ord("r"), ord("t"), ord("n")):
                self.handle_key(k)
                k = -1
            if self.quit:
                break

            if self.mode == "AUTO":
                self.auto_tick()
                next_t = time.perf_counter()
            else:
                if self.mode == "TELEOP":
                    self.tstate["key"] = k
                    self.tstate = self.teleop_step(self.robot, self.tracker, self.object_ids, self.tstate)
                else:                                     # IDLE / DONE: keep stepping so mouse drag works
                    for _ in range(STEPS_PER_FRAME):
                        p.stepSimulation()
                self.frame_i += 1
                if self.frame_i % CAM_REFRESH == 0:
                    self.refresh_camera(run_detect=False)    # boxes stay from the last AUTO scan
                next_t += 1 / FPS
                time.sleep(max(0.0, next_t - time.perf_counter()))
                if time.perf_counter() - next_t > 0.2:
                    next_t = time.perf_counter()
            self.draw_hud()

    def close(self):
        if self.tracker is not None and hasattr(self.tracker, "close"):
            self.tracker.close()
        cv2.destroyAllWindows()
        if p.isConnected():
            p.disconnect()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--keyboard", action="store_true", help="keyboard teleop instead of the webcam")
    ap.add_argument("--fast", action="store_true", help="no real-time pacing in AUTO (debug)")
    ap.add_argument("--auto", action="store_true", help="start in AUTO instead of IDLE")
    args = ap.parse_args()

    app = App(args)
    print(f"[main] perception: {PERCEPTION}. Focus the '{WINDOW}' or PyBullet window for keys.")
    if args.auto:
        app.enter_auto()
    try:
        app.run()
    except p.error as e:                      # PyBullet window closed
        print(f"[main] pybullet stopped: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    main()
