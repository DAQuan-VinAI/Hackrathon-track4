"""Operator hand -> robot: webcam + MediaPipe Hands, keyboard fallback, fail-safe hold (owner: C).

    python teleop.py                  # webcam only: skeleton, gesture, FPS, latency (no robot)

Hand = joystick: palm in the center box = stop, palm toward an edge = move that way (farther = faster).
1 finger (index) = UP | 2 fingers (V) = DOWN | fist = GRAB | open palm = RELEASE.
Controls are screen-relative: they follow the PyBullet camera yaw (scene.camera_yaw), so hand right
is always screen right even after the operator rotates the view. Default view: screen right == world -y.
Keyboard fallback uses i/j/k/l (xy), u/o (z), space (grip): a/t/r/n/q are reserved by main.py.
Contract (common.py): tracker.read(key) -> (xyz or None, grip, frame_bgr or None, latency_ms);
teleop_step(robot, tracker, object_ids, state) -> state, with the key in state["key"].
"""
import time

import numpy as np
import pybullet as p

from common import BIN_SIZE, BINS, CUBE, SIM_HZ, WORKSPACE

# Speeds are per frame at NOMINAL_FPS and scaled by the real frame time, so the arm moves at
# the same speed in m/s when webcam + MediaPipe + rendering drop the loop to 10-20 FPS.
NOMINAL_FPS = 30
MAX_DT = 0.1            # longer frames (a hiccup) are not caught up, the arm would lurch
STEP_LIMIT = 0.025      # max EE target change per frame (m, ~0.75 m/s): no jump when entering TELEOP.
                        # PHAN_CONG said 1.5 cm; raised because the operators found it too slow
EMA = 0.3               # weight of the new palm sample: smoother but adds lag
DEAD = 0.08             # joystick dead-zone half-size, fraction of the frame
V_MAX = 0.02            # max XY speed (m/frame, ~0.6 m/s) at the frame edge
Z_STEP = 0.018          # index only: raise; V sign: lower (m/frame, ~0.54 m/s)
START_Z = 0.20
GRIP_ON, GRIP_OFF = 0.25, 0.40    # pinch ratio thresholds, fallback when the hand is neither fist nor open
GRIP_FRAMES = 3                   # a grab/release must be seen this many frames in a row
KEY_STEP = 0.02                   # keyboard: one key event moves 2 cm (main.read_key repeats held keys)
CENTER = ((WORKSPACE["x"][0] + WORKSPACE["x"][1]) / 2, (WORKSPACE["y"][0] + WORKSPACE["y"][1]) / 2)

# Assisted grab (PHAN_CONG 6C "Nếu trễ thì cắt": grab gesture -> the arm descends, grabs, lifts).
# grasp() needs the EE within 4 cm of the cube center, which is hard to hit by hand from one view.
ASSIST_XY = 0.08        # grab gesture with a loose cube this close in XY: the arm finishes the grab.
                        # = the minimum cube spacing; from behind the robot a 6-8 cm depth offset
                        # still looks "right above the cube", so 6 cm missed too often
GRASP_DZ = 0.008        # stop this far above the top face (same as controller.pick_and_place)
CARRY_Z = 0.15          # lift to this height after an assisted grab
ASSIST_FRAMES = 120     # give up an assisted grab after ~4 s

# Corridor over the bins (outside WORKSPACE): x along the bin row, EE at least 14 cm high
# (held cube bottom ~5 cm below the EE + 8 cm walls).
BIN_ZONE = dict(x=(min(b[0] for b in BINS.values()) - 0.07, max(b[0] for b in BINS.values()) + 0.07),
                y=(min(b[1] for b in BINS.values()) - 0.05, WORKSPACE["y"][0]), z_min=0.14)

# Four views for the HUD (GUI only), a 2x2 grid in state["views"]. One view is re-rendered per
# RENDER_EVERY frames (round robin), so a frame costs at most one small render.
VIEWS = [
    # top and front are widened to y = -0.6 so the bins are in view too
    ("TOP (right = robot right)", dict(eye=(0.50, -0.17, 1.15), target=(0.50, -0.17, 0.0), up=(1, 0, 0), fov=45)),
    ("SIDE (robot on right)", dict(eye=(0.55, 1.05, 0.20), target=(0.55, 0.0, 0.18), up=(0, 0, 1), fov=32)),
    ("FRONT (facing robot)", dict(eye=(1.40, -0.17, 0.30), target=(0.45, -0.17, 0.10), up=(0, 0, 1), fov=48)),
    ("3D", dict(eye=(1.15, 0.75, 0.65), target=(0.50, -0.05, 0.05), up=(0, 0, 1), fov=45)),
]
VIEW_PX = 240           # each tile is VIEW_PX x VIEW_PX (main.py scales the grid to the HUD height)
RENDER_EVERY = 2        # re-render one tile every N frames: rendering competes with MediaPipe for frame time
FOOTER = 62             # status bar under the grid: height, FPS and hint


def clamp_ws(xyz):
    """Teleop safe region: WORKSPACE, plus a corridor over the bin row so the operator can drop a
    part in its bin (PHAN_CONG 6C). Over the bins the EE stays high enough for a held cube to clear
    the 8 cm walls."""
    x, y, z = xyz
    y = np.clip(y, BIN_ZONE["y"][0], WORKSPACE["y"][1])
    if y < WORKSPACE["y"][0]:
        return np.array([np.clip(x, *BIN_ZONE["x"]), y, np.clip(z, BIN_ZONE["z_min"], WORKSPACE["z"][1])])
    return np.array([np.clip(x, *WORKSPACE["x"]), y, np.clip(z, *WORKSPACE["z"])])


def bin_under(xyz):
    """Color of the bin right below xyz, or None."""
    for color, (bx, by) in BINS.items():
        if abs(xyz[0] - bx) < BIN_SIZE / 2 and abs(xyz[1] - by) < BIN_SIZE / 2:
            return color
    return None


def screen_axes(yaw_deg):
    """World xy of screen-right and screen-forward for a PyBullet camera yaw."""
    a = np.deg2rad(yaw_deg)
    return np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])


def joystick(palm_uv, yaw_deg=-90.0):
    """Palm position (0..1, mirrored) -> XY velocity in world (m/frame), screen-relative.
    Inside the dead zone: 0. Farther from the center: faster (quadratic for fine control)."""
    def axis(off):
        mag = max(abs(off) - DEAD, 0.0) / (0.5 - DEAD)
        return np.sign(off) * min(mag, 1.0) ** 2 * V_MAX
    right, fwd = screen_axes(yaw_deg)
    return right * axis(palm_uv[0] - 0.5) + fwd * axis(0.5 - palm_uv[1])


def palm_center(lm):
    """Wrist + finger bases: does not move when the fingers curl (fist/V), unlike landmark 9 alone."""
    return np.mean([lm[i] for i in (0, 5, 9, 13, 17)], axis=0)


def finger_pose(lm):
    """Returns (fist, open_palm, [index, middle, ring, pinky] extended)."""
    wrist = np.array(lm[0])
    fingers = [bool(np.linalg.norm(np.subtract(lm[tip], wrist)) > 1.12 * np.linalg.norm(np.subtract(lm[pip], wrist)))
               for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18))]
    return sum(fingers) <= 1, sum(fingers) >= 3, fingers


class GripHysteresis:
    """Fist = grab, open palm = release, pinch ratio as fallback; holds the state when unclear.
    A change must be seen GRIP_FRAMES frames in a row (no flicker)."""

    def __init__(self, stable_frames=GRIP_FRAMES):
        self.grip = False
        self.candidate = None
        self.frames = 0
        self.stable_frames = stable_frames

    def update(self, ratio=None, closed=False, opened=False):
        if closed or opened:
            wanted = bool(closed)
        elif ratio is not None and ratio < GRIP_ON:
            wanted = True
        elif ratio is not None and ratio > GRIP_OFF:
            wanted = False
        else:
            wanted = None
        if wanted is None or wanted == self.grip:
            self.candidate, self.frames = None, 0
        elif wanted == self.candidate:
            self.frames += 1
            if self.frames >= self.stable_frames:
                self.grip, self.candidate, self.frames = wanted, None, 0
        else:
            self.candidate, self.frames = wanted, 1
            if self.stable_frames <= 1:
                self.grip, self.candidate, self.frames = wanted, None, 0
        return self.grip

    def reset(self):
        self.grip, self.candidate, self.frames = False, None, 0


def rate_limit(current, target, limit=STEP_LIMIT):
    d = np.subtract(target, current)
    n = np.linalg.norm(d)
    return np.array(target, float) if n <= limit else np.asarray(current, float) + d * (limit / n)


class HandTracker:
    """read(key) -> (xyz or None, grip, frame_bgr, latency_ms). Frames are never written to disk."""

    def __init__(self, device=0):
        import cv2
        import mediapipe as mp
        self.cv2 = cv2
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise RuntimeError("webcam not available")
        self.hands = mp.solutions.hands.Hands(max_num_hands=1, model_complexity=0,
                                              min_detection_confidence=0.6, min_tracking_confidence=0.5)
        self.draw = mp.solutions.drawing_utils
        self.conn = mp.solutions.hands.HAND_CONNECTIONS
        self.grip = GripHysteresis()
        self.pos = np.array([*CENTER, START_Z])   # integrated target; teleop_step resets it to the EE
        self.palm = None                          # smoothed palm center (image 0..1)
        self.yaw = -90.0                          # camera yaw, set by teleop_step every frame
        self.scale = 1.0                          # real frame time / nominal, set by teleop_step every frame
        self.t0 = time.perf_counter()

    def read(self, key=-1):
        ok, frame = self.cap.read()
        self.t0 = time.perf_counter()                     # latency starts when the frame is available
        if not ok:
            self.palm = None
            return None, self.grip.grip, None, 0.0
        frame = self.cv2.flip(frame, 1)                   # mirror: operator's right == screen right
        res = self.hands.process(self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB))
        if not res.multi_hand_landmarks:
            self.palm = None                              # fail-safe: caller holds position and keeps the part
            return None, self.grip.grip, frame, (time.perf_counter() - self.t0) * 1000
        hl = res.multi_hand_landmarks[0]
        self.draw.draw_landmarks(frame, hl, self.conn)
        lm = [(q.x, q.y) for q in hl.landmark]
        hand = max(float(np.hypot(lm[0][0] - lm[9][0], lm[0][1] - lm[9][1])), 1e-6)   # scale reference
        fist, open_palm, fingers = finger_pose(lm)
        up = fingers[0] and not any(fingers[1:])
        down = fingers[0] and fingers[1] and not any(fingers[2:])
        # pinch fallback is off during the height gestures: index-only looks like an open pinch
        ratio = None if up or down else float(np.hypot(lm[4][0] - lm[8][0], lm[4][1] - lm[8][1])) / hand
        grip = self.grip.update(ratio, closed=fist and not up, opened=open_palm)
        palm = palm_center(lm)
        self.palm = palm if self.palm is None else (1 - EMA) * self.palm + EMA * palm
        # freeze XY while changing height or confirming a grab/release: the hand-shape change
        # would otherwise drag the arm sideways
        frozen = up or down or self.grip.candidate is not None
        vel = np.zeros(2) if frozen else joystick(self.palm, self.yaw) * self.scale
        self.pos = clamp_ws((*(self.pos[:2] + vel), self.pos[2] + Z_STEP * self.scale * (up - down)))
        action = "UP" if up else "DOWN" if down else "GRAB" if grip else "MOVE"
        self._overlay(frame, action, vel)
        return self.pos.copy(), grip, frame, (time.perf_counter() - self.t0) * 1000

    def _overlay(self, frame, action, vel):
        cv2 = self.cv2
        h, w = frame.shape[:2]
        x0, x1 = int(w * (0.5 - DEAD)), int(w * (0.5 + DEAD))
        y0, y1 = int(h * (0.5 - DEAD)), int(h * (0.5 + DEAD))
        cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 255), 2)       # dead zone: hand here = stop
        cv2.putText(frame, "DUNG", (x0 + 4, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        pu, pv = int(self.palm[0] * w), int(self.palm[1] * h)
        cv2.circle(frame, (pu, pv), 9, (0, 255, 255), -1)
        if np.any(vel):
            cv2.arrowedLine(frame, (w // 2, h // 2), (pu, pv), (0, 255, 255), 3, tipLength=0.2)
        color = {"UP": (255, 200, 0), "DOWN": (255, 0, 255), "GRAB": (0, 0, 255), "MOVE": (0, 255, 0)}[action]
        cv2.putText(frame, f"{action}  z={self.pos[2]:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(frame, "Tay ra ngoai o = di chuyen | 1 ngon: LEN | V: XUONG | nam: GAP | xoe: THA",
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    def reset(self, xyz):
        """Start from the current EE pose with the grip open (called when entering TELEOP)."""
        self.pos, self.palm = clamp_ws(xyz), None
        self.grip.reset()

    def close(self):
        self.hands.close()
        self.cap.release()


class KeyboardTracker:
    """Fallback with the same read() signature. Keys come from main.py (single key source),
    screen-relative like the hand: i/k forward/back, j/l left/right, u/o up/down, space grip."""

    def __init__(self, start=(0.55, 0.0, START_Z)):
        self.pos = np.array(start, float)
        self.grip = False
        self.yaw = -90.0
        self.t0 = time.perf_counter()

    def read(self, key=-1):
        self.t0 = time.perf_counter()
        right, fwd = screen_axes(self.yaw)
        moves = {ord("i"): (*fwd, 0), ord("k"): (*-fwd, 0), ord("j"): (*-right, 0),
                 ord("l"): (*right, 0), ord("u"): (0, 0, 1), ord("o"): (0, 0, -1)}
        if key in moves:
            self.pos = clamp_ws(self.pos + KEY_STEP * np.array(moves[key]))
        elif key == ord(" "):
            self.grip = not self.grip
        return self.pos.copy(), self.grip, None, 0.0

    def reset(self, xyz):
        self.pos, self.grip = clamp_ws(xyz), False

    def close(self):
        pass


def teleop_step(robot, tracker, object_ids, state):
    """One frame of teleop. state: {key, target, cid, grip, hand, frame, latency_ms, msg}. Returns state."""
    from controller import ee_pos, grasp, release
    from scene import camera_yaw
    if state.get("target") is None:
        state.update(target=np.array(ee_pos(robot)), cid=None, grip=False, latency_ms=0.0)
        if hasattr(tracker, "reset"):
            tracker.reset(state["target"])                # start where the arm is, grip open: no jump, no stale grab
    _frame_timing(state)
    if hasattr(tracker, "yaw"):
        tracker.yaw = camera_yaw()                        # controls follow the view the operator chose
    if hasattr(tracker, "scale"):
        tracker.scale = state["scale"]
    xyz, grip, frame, _ = tracker.read(state.get("key", -1))
    state["frame"] = frame
    state["hand"] = xyz is not None
    near = nearest_cube(ee_pos(robot), object_ids)
    if xyz is None:
        _servo(robot, state)                              # hold position, keep the part (assist paused too)
        state["msg"] = "no hand: holding" + (" + part" if state.get("cid") is not None else "")
    elif state.get("assist") is not None:
        if grip:
            _assist_step(robot, tracker, state)
        else:                                             # open palm cancels the assisted grab
            state["assist"], state["msg"] = None, "auto-grab cancelled"
            _sync_tracker(tracker, state["target"])
        state["grip"] = grip
    else:
        state["target"] = rate_limit(state["target"], clamp_ws(xyz), STEP_LIMIT * state["scale"])
        _servo(robot, state)
        if grip and state.get("cid") is None:             # suction stays on while the fist is held
            state["cid"] = grasp(robot, object_ids)
            if state["cid"] is not None:
                state["msg"] = "grasped"
            elif near is not None and near[0] < ASSIST_XY:
                state["assist"] = {"id": near[1], "frames": 0, "phase": "align"}
                state["msg"] = "auto-grab"
            else:                                         # every frame, so the distance updates as the hand moves
                dist = "no cube" if near is None else f"nearest cube {near[0] * 100:.0f} cm"
                state["msg"] = f"move closer ({dist}, need < {ASSIST_XY * 100:.0f} cm)"
        elif not grip and state["grip"]:
            release(state["cid"])
            state["cid"], state["msg"] = None, "released"
        state["grip"] = grip
    state["latency_ms"] = (time.perf_counter() - tracker.t0) * 1000
    _draw_aids(robot, state, near)
    if frame is not None and state.get("msg"):            # status where the operator is looking
        import cv2
        cv2.putText(frame, state["msg"], (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
        cv2.putText(frame, state["msg"], (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return state


def _frame_timing(state):
    """Real time since the last teleop frame -> state["scale"] (vs NOMINAL_FPS) and state["fps"].
    Never below 1: when the loop runs faster than nominal, behave exactly as at NOMINAL_FPS."""
    now = time.perf_counter()
    dt = now - state.get("t_prev", now - 1 / NOMINAL_FPS)
    state["t_prev"] = now
    state["fps"] = 0.8 * state.get("fps", NOMINAL_FPS) + 0.2 / max(dt, 1 / 60)   # display only
    state["scale"] = float(np.clip(dt * NOMINAL_FPS, 1.0, MAX_DT * NOMINAL_FPS))


def _servo(robot, state):
    """Advance the simulation by the real frame time (not a fixed 1/30 s): at 15 FPS a fixed
    step count ran the arm in slow motion."""
    from controller import servo_ee
    servo_ee(robot, state["target"], n_steps=int(round(SIM_HZ / NOMINAL_FPS * state["scale"])))


def nearest_cube(ee, object_ids):
    """(xy distance, id, center xyz) of the closest cube resting in the workspace, or None.
    Cubes already in a bin (outside WORKSPACE) or lifted are ignored."""
    best = None
    for o in object_ids:
        pos = p.getBasePositionAndOrientation(o)[0]
        if pos[2] > 2 * CUBE or pos[1] < WORKSPACE["y"][0] - 0.03:
            continue
        d = float(np.hypot(pos[0] - ee[0], pos[1] - ee[1]))
        if best is None or d < best[0]:
            best = (d, o, pos)
    return best


def _sync_tracker(tracker, xyz):
    """After the arm moved on its own, continue hand/keyboard control from where it is."""
    if hasattr(tracker, "pos"):
        tracker.pos = clamp_ws(xyz)


def _assist_step(robot, tracker, state):
    """Assisted grab: align over the cube at the current height, descend, attach, lift to CARRY_Z."""
    from controller import ee_pos, grasp
    a = state["assist"]
    a["frames"] += 1
    x, y, z = p.getBasePositionAndOrientation(a["id"])[0]
    ee = ee_pos(robot)
    if a["phase"] == "align":
        goal = (x, y, state["target"][2])
        if np.hypot(ee[0] - x, ee[1] - y) < 0.005:
            a["phase"] = "descend"
    elif a["phase"] == "descend":
        goal = (x, y, z + CUBE / 2 + GRASP_DZ)
        if np.linalg.norm(ee - goal) < 0.01:
            state["cid"] = grasp(robot, [a["id"]])
            if state["cid"] is not None:
                a["phase"], state["msg"] = "lift", "grasped (auto)"
    else:
        goal = (state["target"][0], state["target"][1], max(CARRY_Z, state["target"][2]))
        if abs(ee[2] - goal[2]) < 0.01:
            state["assist"] = None
            _sync_tracker(tracker, goal)
    state["target"] = rate_limit(state["target"], clamp_ws(goal), STEP_LIMIT * state["scale"])
    _servo(robot, state)
    if state.get("assist") is not None and a["frames"] > ASSIST_FRAMES:
        state["assist"], state["msg"] = None, "auto-grab failed"
        _sync_tracker(tracker, state["target"])


# ---------------------------------------------------------------- visual aids (GUI only)

_LINE = {"id": None, "t": 0.0}


def _gui():
    return p.getConnectionInfo().get("connectionMethod") == p.GUI


def _project(pt, view, proj, W, H):
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    clip = P @ V @ np.array([*pt, 1.0])
    ndc = clip[:3] / clip[3]
    return int((ndc[0] + 1) / 2 * W), int((1 - ndc[1]) / 2 * H)


def _render_view(cv2, idx, ee, near, holding, bgr):
    """One tile: render view `idx`, draw the EE, its drop line to the floor and the nearest cube."""
    title, cam = VIEWS[idx]
    n = VIEW_PX
    view = p.computeViewMatrix(cam["eye"], cam["target"], cam["up"])
    proj = p.computeProjectionMatrixFOV(cam["fov"], 1.0, 0.1, 3.0)
    _, _, rgba, _, _ = p.getCameraImage(n, n, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                        flags=p.ER_NO_SEGMENTATION_MASK)
    img = cv2.cvtColor(np.reshape(rgba, (n, n, 4))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
    top, floor = _project(ee, view, proj, n, n), _project((ee[0], ee[1], 0.0), view, proj, n, n)
    if idx == 0:                                          # top view: grab radius around the EE on the floor
        ring = [_project((ee[0] + ASSIST_XY * np.cos(a), ee[1] + ASSIST_XY * np.sin(a), 0.0), view, proj, n, n)
                for a in np.linspace(0, 2 * np.pi, 33)]
        cv2.polylines(img, [np.array(ring, np.int32)], True, bgr, 2)
    cv2.line(img, top, floor, bgr, 2)
    cv2.circle(img, top, 6, bgr, 2)
    cv2.circle(img, floor, 3, bgr, -1)
    if near is not None and not holding:
        cv2.circle(img, _project(near[2], view, proj, n, n), 12, (0, 220, 255), 2)
    cv2.rectangle(img, (0, 0), (n, 22), (255, 255, 255), -1)
    cv2.putText(img, title, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
    cv2.rectangle(img, (0, 0), (n - 1, n - 1), (90, 90, 90), 1)
    return img


def _draw_aids(robot, state, near):
    """Drop line from the EE to the floor in the PyBullet window (green = a cube can be grabbed)
    and four views as a 2x2 grid in state["views"] (BGR) for the HUD. No-op in DIRECT mode."""
    if not _gui():
        return
    import cv2
    from controller import ee_pos
    ee = ee_pos(robot)
    holding = state.get("cid") is not None
    ok = holding or (near is not None and near[0] < ASSIST_XY)
    rgb = (0.1, 0.8, 0.1) if ok else (0.9, 0.1, 0.1)
    bgr = tuple(int(255 * c) for c in rgb[::-1])

    now = time.perf_counter()                             # lifeTime clears the line once TELEOP ends
    kw = dict(lineColorRGB=rgb, lineWidth=3, lifeTime=0.5)
    if _LINE["id"] is not None and now - _LINE["t"] < 0.4:
        kw["replaceItemUniqueId"] = _LINE["id"]
    _LINE["id"], _LINE["t"] = p.addUserDebugLine(ee, (ee[0], ee[1], 0.0), **kw), now

    tiles = state.setdefault("tiles", [None] * len(VIEWS))
    state["aid_i"] = state.get("aid_i", -1) + 1
    if state["aid_i"] % RENDER_EVERY == 0:                # one tile every RENDER_EVERY frames
        i = state["view_i"] = (state.get("view_i", -1) + 1) % len(VIEWS)
        tiles[i] = _render_view(cv2, i, ee, near, holding, bgr)
    blank = np.full((VIEW_PX, VIEW_PX, 3), 60, np.uint8)
    t = [x if x is not None else blank for x in tiles]
    grid = np.vstack([np.hstack(t[0:2]), np.hstack(t[2:4])])

    over = bin_under(ee)
    if holding and over:
        hint = f"over {over.upper()} bin - open palm to drop"
    elif holding:
        hint = "HOLDING - move to its bin (right of the table)"
    elif state.get("assist") is not None:
        hint = "AUTO-GRAB..."
    elif near is not None:
        hint = f"cube {near[0] * 100:.0f} cm away" + (" - FIST to grab" if ok else f" - get < {ASSIST_XY * 100:.0f} cm")
    else:
        hint = "no cube in the workspace"
    bar = np.full((FOOTER, grid.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, f"z = {ee[2]:.2f} m    {state.get('fps', 0):.0f} FPS", (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (230, 230, 230), 2, cv2.LINE_AA)
    cv2.putText(bar, hint, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2, cv2.LINE_AA)
    state["views"] = np.vstack([grid, bar])


if __name__ == "__main__":
    # standalone: show skeleton, gesture, FPS (needs a webcam)
    import cv2
    tr = HandTracker()
    last = time.perf_counter()
    while True:
        xyz, grip, frame, lat = tr.read()
        now = time.perf_counter()
        fps, last = 1 / max(now - last, 1e-6), now
        if frame is not None:
            txt = f"fps {fps:.0f} lat {lat:.0f}ms grip {grip} xyz {None if xyz is None else np.round(xyz, 3)}"
            cv2.putText(frame, txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("teleop", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    tr.close()
