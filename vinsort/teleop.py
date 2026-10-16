"""[C] Operator hand -> robot. Webcam + MediaPipe Hands, keyboard fallback, fail-safe hold.

Controls are screen-relative: they follow the current PyBullet camera yaw (free POV).
Hand = joystick: palm in the center box = stop, palm toward an edge = move that way (farther = faster).
1 finger (index) = UP | 2 fingers (V) = DOWN | fist = GRAB | open palm = RELEASE.
Keyboard fallback uses i/j/k/l (xy), u/o (z), space (grip): a/q/t/r/n are taken by main.py.
"""
import time

import numpy as np

from common import WORKSPACE

GAIN = 3.0              # hand motion -> robot motion amplification around the frame center
# Joystick mode: hand offset from the frame center = velocity (dead zone in the middle = stop).
DEAD = 0.08             # dead-zone half-size, fraction of the frame
V_MAX = 0.012           # max XY speed (m/frame, ~0.36 m/s at 30 FPS) at the frame edge
STEP_LIMIT = 0.045      # max EE target change per frame (m): no jump when entering TELEOP
EMA = 0.3               # weight of the new sample
GRIP_ON, GRIP_OFF = 0.25, 0.40   # pinch ratio thresholds (two thresholds: no flicker)
HAND_NEAR, HAND_FAR = 0.30, 0.12 # retained for compatibility; z now uses explicit gestures
Z_STEP = 0.012          # index-only: raise; V sign: lower (m/frame before rate limit)


def clamp_ws(xyz):
    return np.array([np.clip(xyz[0], *WORKSPACE["x"]), np.clip(xyz[1], *WORKSPACE["y"]),
                     np.clip(xyz[2], *WORKSPACE["z"])])


def _amp(t):
    """0..1 -> 0..1 with GAIN around 0.5 (clamped): a small hand move covers the whole range."""
    return float(np.clip(0.5 + (t - 0.5) * GAIN, 0, 1))


CENTER = ((WORKSPACE["x"][0] + WORKSPACE["x"][1]) / 2, (WORKSPACE["y"][0] + WORKSPACE["y"][1]) / 2)
REACH = 0.25            # half-width covered by the hand around CENTER (m)


def screen_axes(yaw_deg):
    """World xy of screen-right and screen-forward for a PyBullet camera yaw (free POV)."""
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


def map_hand(lm9_xy, hand_size, yaw_deg=-90.0):
    """Image (mirrored, 0..1) -> world xyz. Screen-relative: follows the current camera POV,
    so hand right is always robot-right on screen, hand up is always 'into the screen'."""
    right, fwd = screen_axes(yaw_deg)
    du = (_amp(lm9_xy[0]) - 0.5) * 2 * REACH
    dv = (0.5 - _amp(lm9_xy[1])) * 2 * REACH
    x, y = np.array(CENTER) + right * du + fwd * dv
    t = _amp((hand_size - HAND_FAR) / (HAND_NEAR - HAND_FAR))
    z = WORKSPACE["z"][1] - t * (WORKSPACE["z"][1] - WORKSPACE["z"][0])   # big hand -> low
    return clamp_ws((x, y, z))


class GripHysteresis:
    """Nắm tay để gắp, xòe tay để thả; giữ trạng thái khi cử chỉ chưa rõ."""
    def __init__(self, stable_frames=3):
        self.grip = False
        self.candidate = None
        self.frames = 0
        self.stable_frames = stable_frames

    def update(self, closed=None, opened=None, ratio=None):
        # Pinch kept as fallback; fist/open-palm is easier and preferred.
        wanted = True if closed else False if opened else (
            True if ratio is not None and ratio < GRIP_ON else
            False if ratio is not None and ratio > GRIP_OFF else None)
        if wanted is None or wanted == self.grip:
            self.candidate, self.frames = None, 0
        elif wanted == self.candidate:
            self.frames += 1
            if self.frames >= self.stable_frames:
                self.grip = wanted
                self.candidate, self.frames = None, 0
        else:
            self.candidate, self.frames = wanted, 1
        return self.grip


def finger_pose(lm):
    """Return (fist, open palm, extended count, finger booleans) for non-thumb fingers."""
    wrist = np.array(lm[0])
    fingers = []
    for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18)):
        tip_d = np.linalg.norm(np.array(lm[tip]) - wrist)
        pip_d = np.linalg.norm(np.array(lm[pip]) - wrist)
        fingers.append(bool(tip_d > pip_d * 1.12))
    extended = sum(fingers)
    return extended <= 1, extended >= 3, extended, fingers


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
        self.pos = None
        self.z = None           # height is held between frames; only UP/DOWN gestures change it
        self.palm = None        # smoothed palm center (image 0..1)
        self.yaw = -90.0        # camera yaw, set by teleop_step each frame (free POV)
        self.t0 = time.perf_counter()

    def read(self, key=-1):
        ok, frame = self.cap.read()
        self.t0 = time.perf_counter()
        if not ok:
            self.pos = None
            return None, self.grip.grip, None, 0.0
        frame = self.cv2.flip(frame, 1)  # mirror: operator's right == screen right
        res = self.hands.process(self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB))
        if not res.multi_hand_landmarks:
            self.palm = None  # fail-safe: caller holds position and keeps the part
            return None, self.grip.grip, frame, (time.perf_counter() - self.t0) * 1000
        hl = res.multi_hand_landmarks[0]
        self.draw.draw_landmarks(frame, hl, self.conn)
        lm = [(q.x, q.y) for q in hl.landmark]
        dist = lambda a, b: float(np.hypot(lm[a][0] - lm[b][0], lm[a][1] - lm[b][1]))
        hand = max(dist(0, 9), 1e-6)
        fist, open_palm, extended, fingers = finger_pose(lm)
        index_only = fingers[0] and not any(fingers[1:])
        v_sign = fingers[0] and fingers[1] and not any(fingers[2:])
        # Fist/open palm controls grip. Pinch fallback is disabled during explicit height gestures.
        ratio = None if index_only or v_sign else dist(4, 8) / hand
        grip = self.grip.update(fist and not index_only, open_palm, ratio)
        if self.z is None:
            self.z = self.start_z
        if self.pos is None:
            self.pos = np.array([*CENTER, self.z])
        # Height: held still unless an explicit gesture is shown (no more guessing from hand size).
        self.z = float(np.clip(self.z + Z_STEP * (index_only - v_sign), *WORKSPACE["z"]))
        palm = palm_center(lm)
        self.palm = palm if self.palm is None else (1 - EMA) * self.palm + EMA * palm
        # Freeze XY while changing height or while a grab/release is being confirmed:
        # the hand-shape change would otherwise drag the arm sideways.
        frozen = index_only or v_sign or self.grip.candidate is not None
        vel = np.zeros(2) if frozen else joystick(self.palm, self.yaw)
        self.pos = clamp_ws((*(self.pos[:2] + vel), self.z))
        action = "UP" if index_only else "DOWN" if v_sign else "GRAB" if grip else "MOVE"
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
        cv2.putText(frame, f"{action}  z={self.z:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(frame, "Tay ra ngoai o = di chuyen | 1 ngon: LEN | V: XUONG | nam: GAP | xoe: THA",
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    start_z = 0.20

    def close(self):
        self.hands.close()
        self.cap.release()


class KeyboardTracker:
    """Fallback with the same read() signature. Keys come from main.py (single waitKey source)."""
    STEP = 0.03

    def __init__(self, start=(0.55, 0.0, 0.20)):
        self.pos = np.array(start, float)
        self.grip = False
        self.yaw = -90.0
        self.t0 = time.perf_counter()

    def read(self, key=-1):
        self.t0 = time.perf_counter()
        right, fwd = screen_axes(self.yaw)  # screen-relative, like the hand
        moves = {ord("i"): (*fwd, 0), ord("k"): (*-fwd, 0), ord("j"): (*-right, 0),
                 ord("l"): (*right, 0), ord("u"): (0, 0, 1), ord("o"): (0, 0, -1)}
        if key in moves:
            self.pos = clamp_ws(self.pos + self.STEP * np.array(moves[key]))
        elif key == ord(" "):
            self.grip = not self.grip
        return self.pos.copy(), self.grip, None, 0.0

    def close(self):
        pass


def teleop_step(robot, tracker, object_ids, state, key=-1, realtime=False):
    """One frame of teleop. state: {target, cid, grip, latency_ms, hand}. Returns state."""
    from controller import ee_pos, grasp, release, servo_ee
    from scene import camera_yaw
    if state.get("target") is None:
        state.update(target=np.array(ee_pos(robot)), cid=None, grip=False, latency_ms=0.0)
        # start from where the arm is now: no jump when entering TELEOP
        tracker.pos = clamp_ws(state["target"])
        if hasattr(tracker, "z"):
            tracker.z = float(tracker.pos[2])
    if hasattr(tracker, "yaw"):
        tracker.yaw = camera_yaw()  # free POV: controls follow whatever view the operator chose
    xyz, grip, frame, _ = tracker.read(key)
    state["frame"] = frame
    state["hand"] = xyz is not None
    if xyz is None:
        servo_ee(robot, state["target"], realtime=realtime)  # hold position, keep the part
        return state
    state["target"] = rate_limit(state["target"], clamp_ws(xyz))
    servo_ee(robot, state["target"], realtime=realtime)
    if grip and not state["grip"]:
        state["cid"] = grasp(robot, object_ids)
    elif not grip and state["grip"]:
        release(state["cid"])
        state["cid"] = None
    state["grip"] = grip
    state["latency_ms"] = (time.perf_counter() - tracker.t0) * 1000
    return state


if __name__ == "__main__":
    # standalone: show skeleton, pinch state, FPS (needs a webcam)
    import cv2
    tr = HandTracker()
    last = time.perf_counter()
    while True:
        xyz, grip, frame, lat = tr.read()
        now = time.perf_counter()
        fps, last = 1 / max(now - last, 1e-6), now
        if frame is not None:
            txt = f"fps {fps:.0f} lat {lat:.0f}ms grip {grip} xyz {None if xyz is None else np.round(xyz, 3)}"
            cv2.putText(frame, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("teleop", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    tr.close()
