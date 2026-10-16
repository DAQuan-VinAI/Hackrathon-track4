"""VinSort demo: AUTO sorting with hand-off to gesture TELEOP when unsure (owner: A).

    python main.py --seed 0           # normal layout
    python main.py --seed 100         # failure case: two same-color cubes touching
    python main.py --keyboard         # force keyboard teleop (no webcam)
    python main.py --conveyor --auto  # parts arrive on a belt, every 5th is a touching pair

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
COLORS_ORDER = ["red", "green", "blue"]

# Dashboard layout (px)
HEADER, FOOTER, PANEL_W = 64, 30, 320
BG, CARD, TEXT, MUTED = (28, 24, 22), (48, 42, 38), (235, 235, 235), (150, 150, 150)
MODE_BGR = {"AUTO": (90, 200, 90), "TELEOP": (0, 190, 255), "DONE": (200, 200, 200), "IDLE": (150, 150, 150)}
TRAIL_LIFE = 4.0                  # s, EE trail in the PyBullet window
FONT = cv2.FONT_HERSHEY_SIMPLEX


def put(img, text, org, scale=0.5, color=TEXT, thick=1):
    cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)


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
        self.conv = None
        if getattr(args, "conveyor", False):  # parts arrive on a belt instead of a fixed layout
            from conveyor import Conveyor
            self.scene = build_scene(args.seed, 0, gui=True)
            self.conv = Conveyor(self.scene, args.seed)
        else:
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
        self.t0 = time.perf_counter()
        self.stats = {"attempts": 0, "ok": 0, "cycles": [], "handoffs": 0, "t_auto": None}
        self.events = []              # (elapsed s, text, BGR) for the dashboard log
        self.last_tmsg = ""
        self.trail_prev, self.trail_t = None, 0.0
        go_home(self.robot)
        self.refresh_camera()
        self.log("scene ready, seed %d, %d parts" % (args.seed, len(self.scene["objects"])), MUTED)

    @property
    def object_ids(self):
        return [o["id"] for o in self.scene["objects"]]

    # ---------- perception ----------
    def log(self, text, color=TEXT):
        self.events.append((time.perf_counter() - self.t0, text, color))
        del self.events[:-9]

    def trail(self):
        """Fading EE path in the PyBullet window (color = mode)."""
        now = time.perf_counter()
        if now - self.trail_t < 0.05:
            return
        ee = controller.ee_pos(self.robot)
        if self.trail_prev is not None and np.linalg.norm(ee - self.trail_prev) > 0.004:
            rgb = (0.0, 0.75, 1.0) if self.mode == "TELEOP" else (0.2, 0.8, 0.3)
            p.addUserDebugLine(self.trail_prev, ee, rgb, lineWidth=3, lifeTime=TRAIL_LIFE)
        self.trail_prev, self.trail_t = ee, now

    def refresh_camera(self, run_detect=True):
        if run_detect:                        # TINY renderer (~0.5 s), same images as D's benchmark
            self.rgb, depth, _ = capture()
            self.dets = detect(self.rgb, depth)
            perception.show_in_3d(self.dets)  # camera frustum, rays and 3D markers in the PyBullet window
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

    def camera_panel(self):
        """Overhead camera with detection boxes, alert banner and webcam inset (640 x 480)."""
        img = cv2.cvtColor(self.rgb, cv2.COLOR_RGB2BGR)
        for d in self.dets:
            r = max(8, int(np.sqrt(max(d.area, 1)) / 2) + 4)
            u, v = int(d.uv[0]), int(d.uv[1])
            box_color = (0, 165, 255) if d.suspicious else DRAW.get(d.color, (255, 255, 255))
            cv2.rectangle(img, (u - r, v - r), (u + r, v + r), box_color, 3 if d.suspicious else 2)
            label = f"{d.color} ({d.xyz[0]:.2f},{d.xyz[1]:.2f})" + (f" ? {d.reason}" if d.suspicious else "")
            (tw, th), _ = cv2.getTextSize(label, FONT, 0.4, 1)
            cv2.rectangle(img, (u - r, v - r - th - 8), (u - r + tw + 6, v - r - 2), (30, 30, 30), -1)
            put(img, label, (u - r + 3, v - r - 6), 0.4, box_color)
        put(img, "OVERHEAD CAMERA", (10, 22), 0.5, (60, 60, 60), 1)
        if self.alert:
            cv2.rectangle(img, (8, 32), (img.shape[1] - 8, 72), (0, 0, 200), -1)
            put(img, self.alert, (18, 60), 0.7, (255, 255, 255), 2)
        frame = self.tstate.get("frame") if self.mode == "TELEOP" else None
        if frame is not None:                 # webcam preview from C, bottom-right corner
            h, w = 170, int(170 * frame.shape[1] / frame.shape[0])
            img[-h - 8:-8, -w - 8:-8] = cv2.resize(frame, (w, h))
        return img

    def stats_panel(self, h):
        """Right column outside TELEOP: totals, per-color bars, KPIs, event log."""
        pnl = np.full((h, PANEL_W, 3), BG, np.uint8)
        objs = self.scene["objects"]
        done = self.sorted_count()
        cv2.rectangle(pnl, (10, 10), (PANEL_W - 10, 92), CARD, -1)
        put(pnl, "SORTED", (22, 34), 0.5, MUTED)
        put(pnl, f"{done}/{len(objs)}", (22, 78), 1.4, TEXT, 3)
        frac = done / max(len(objs), 1)
        cv2.rectangle(pnl, (160, 60), (PANEL_W - 22, 72), (70, 70, 70), -1)
        cv2.rectangle(pnl, (160, 60), (160 + int((PANEL_W - 182) * frac), 72), (90, 200, 90), -1)
        put(pnl, f"{frac * 100:.0f}%", (160, 50), 0.5, MUTED)

        y = 120
        for c in COLORS_ORDER:                # per-color progress
            mine = [o for o in objs if o["color"] == c]
            n_in = sum(in_bin(o["id"], c) for o in mine)
            cv2.rectangle(pnl, (14, y - 12), (28, y + 2), DRAW[c], -1)
            put(pnl, f"{c.upper():5s} {n_in}/{len(mine)}", (36, y), 0.5)
            cv2.rectangle(pnl, (150, y - 9), (PANEL_W - 22, y - 1), (70, 70, 70), -1)
            if mine:
                cv2.rectangle(pnl, (150, y - 9), (150 + int((PANEL_W - 172) * n_in / len(mine)), y - 1), DRAW[c], -1)
            y += 26

        s = self.stats
        rate = f"{100 * s['ok'] / s['attempts']:.0f}%" if s["attempts"] else "-"
        mins = (time.perf_counter() - s["t_auto"]) / 60 if s["t_auto"] else 0
        thr = f"{done / mins:.1f}" if mins > 0.05 else "-"
        cyc = f"{np.mean(s['cycles']):.1f} s" if s["cycles"] else "-"
        kpis = [("pick success", rate), ("parts / min", thr), ("cycle (sim)", cyc), ("operator hand-offs", str(s["handoffs"])),
                ("last scan", f"{len(self.dets)} ({sum(d.suspicious for d in self.dets)} unsure)")]
        y += 6
        for i, (k, v) in enumerate(kpis):
            put(pnl, k, (14, y), 0.45, MUTED)
            put(pnl, v, (190, y), 0.5, TEXT, 1)
            y += 22

        y += 10
        put(pnl, "EVENTS", (14, y), 0.45, MUTED)
        y += 8
        for t, text, color in reversed(self.events):
            y += 20
            if y > h - 8:
                break
            put(pnl, f"{int(t) // 60:02d}:{int(t) % 60:02d}", (14, y), 0.4, MUTED)
            put(pnl, text[:30], (62, y), 0.42, color)
        return pnl

    def draw_hud(self):
        if self.mode == "TELEOP":             # log the operator's grasp / release
            m = self.tstate.get("msg", "")
            if m != self.last_tmsg and m.startswith(("grasped", "released", "auto-grab failed")):
                self.log(f"operator: {m}", MODE_BGR["TELEOP"])
            self.last_tmsg = m
        cam = self.camera_panel()
        h = cam.shape[0]
        views = self.tstate.get("views") if self.mode == "TELEOP" else None
        if views is not None:                 # C's 2x2 views (top, side, front, 3D) replace the stats in TELEOP
            right = cv2.resize(views, (int(h * views.shape[1] / views.shape[0]), h))
        else:
            right = self.stats_panel(h)
        body = np.hstack([cam, right])
        W = body.shape[1]

        head = np.full((HEADER, W, 3), BG, np.uint8)
        put(head, "VinSort", (14, 34), 1.0, TEXT, 2)
        put(head, "Vin Robotics x VinFast  |  vision sorting cell", (14, 54), 0.42, MUTED)
        mc = MODE_BGR.get(self.mode, MUTED)
        x0 = 330
        cv2.rectangle(head, (x0, 14), (x0 + 120, 48), mc, -1)
        put(head, self.mode, (x0 + 12, 40), 0.75, (20, 20, 20), 2)
        status = self.msg
        if self.mode == "AUTO":
            status = f"{controller.phase}  |  {self.msg}"
        elif self.mode == "TELEOP" and self.tracker is not None:
            lat = self.tstate.get("latency_ms")
            status = f"{self.tracker_name} teleop" + (f"  {lat:.0f} ms" if lat is not None else "") \
                     + (f"  |  {self.tstate['msg']}" if self.tstate.get("msg") else "")
        put(head, status[:70], (x0 + 136, 30), 0.5, TEXT)
        t = int(time.perf_counter() - self.t0)
        put(head, f"seed {self.args.seed}   {t // 60:02d}:{t % 60:02d}   perception: {PERCEPTION}",
            (x0 + 136, 52), 0.42, MUTED)

        foot = np.full((FOOTER, W, 3), BG, np.uint8)
        put(foot, "a AUTO   t TELEOP   r rescan   n add part   q quit", (14, 20), 0.48, MUTED)
        cv2.imshow(WINDOW, np.vstack([head, body, foot]))

    def pump(self):
        """Called every sim step inside blocking robot moves: keep the HUD alive, buffer keys."""
        self.frame_i += 1
        if self.frame_i % STEPS_PER_FRAME:
            return
        self.trail()
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
        if self.stats["t_auto"] is None:
            self.stats["t_auto"] = time.perf_counter()
        self.log(f"AUTO: {msg}", MODE_BGR["AUTO"])

    def enter_teleop(self, alert=""):
        if self.tracker is None:
            self.tracker, self.teleop_step, self.tracker_name = load_teleop(self.args.keyboard)
        self.tstate = {"cid": None, "key": -1, "target": None}
        self.alert = alert
        self.mode = "TELEOP"
        if alert:
            self.stats["handoffs"] += 1
            self.log("hand-off: " + alert.split(": ", 1)[-1], (0, 120, 255))
        else:
            self.log(f"TELEOP ({self.tracker_name})", MODE_BGR["TELEOP"])

    def auto_tick(self):
        """One AUTO cycle: HOME -> capture -> detect -> pick the nearest confident cube."""
        realtime = not self.args.fast
        go_home(self.robot, realtime=realtime, on_step=self.pump)
        self.refresh_camera()
        dets = self.dets
        if self.conv:                         # parts still entering the view (cut by the image edge) wait
            from conveyor import PICK_ZONE_Y
            dets = [d for d in dets if d.xyz[1] < PICK_ZONE_Y]
        ok = [d for d in dets if not d.suspicious]
        unsure = [d for d in dets if d.suspicious]
        if ok:
            det = min(ok, key=lambda d: np.hypot(d.xyz[0], d.xyz[1]))   # nearest to the robot base
            self.msg = f"picking {det.color} at ({det.xyz[0]:.3f}, {det.xyz[1]:.3f})"
            self.stats["attempts"] += 1
            steps0 = self.frame_i
            if pick_and_place(self.robot, det, self.object_ids, realtime=realtime, on_step=self.pump):
                self.pick_fails = 0
                self.stats["ok"] += 1
                self.stats["cycles"].append((self.frame_i - steps0) / SIM_HZ)   # sim seconds, see benchmark
                self.log(f"picked {det.color} -> {det.color} bin", DRAW[det.color])
            else:
                self.pick_fails += 1
                self.msg = f"grasp failed ({self.pick_fails}/{MAX_PICK_FAILS})"
                self.log(f"grasp failed ({det.color})", (0, 0, 230))
                if self.pick_fails >= MAX_PICK_FAILS:
                    self.enter_teleop("CAN NGUOI VAN HANH: grasp failed")
        elif unsure:
            self.enter_teleop(f"CAN NGUOI VAN HANH: {unsure[0].reason} ({len(unsure)} uncertain)")
        elif self.conv:                       # nothing left under the camera: start the belt again
            self.msg = "belt running"
        else:
            self.mode, self.msg = "DONE", "XONG - r rescan, n add cube"
            self.log("all confident parts sorted", MUTED)

    def belt_frame(self):
        """One frame of the conveyor (AUTO, nothing waiting under the camera)."""
        n_before = len(self.scene["objects"])
        new = self.conv.run(STEPS_PER_FRAME, SIM_HZ)
        if new:
            c = self.scene["objects"][-1]["color"]
            pair = len(self.scene["objects"]) - n_before > 1
            self.log(f"belt: new {'PAIR ' if pair else ''}{c} part", DRAW[c])
        self.msg = "belt running" if self.conv.running else "part arrived, scanning"

    def handle_key(self, k):
        if k == ord("q"):
            self.quit = True
        elif k == ord("a"):
            self.enter_auto()
        elif k == ord("r"):
            self.enter_auto("rescan")
        elif k == ord("t") and self.mode != "TELEOP":
            self.enter_teleop()
        elif k == ord("n") and self.conv:
            self.msg = "part added on the belt" if self.conv.spawn() else "belt start is busy"
        elif k == ord("n"):
            oid = add_random_object(self.scene)
            self.msg = "added a cube" if oid is not None else "no free spot"
            if oid is not None:
                self.log(f"new part: {self.scene['objects'][-1]['color']}", MUTED)

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

            belt = self.mode == "AUTO" and self.conv is not None and not self.conv.waiting()
            if self.mode == "AUTO" and not belt:
                self.auto_tick()
                next_t = time.perf_counter()
            else:
                if belt:
                    self.belt_frame()
                elif self.mode == "TELEOP":
                    self.tstate["key"] = k
                    self.tstate = self.teleop_step(self.robot, self.tracker, self.object_ids, self.tstate)
                    self.trail()
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
    ap.add_argument("--conveyor", action="store_true", help="parts arrive on a belt (see conveyor.py)")
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
