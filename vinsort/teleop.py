"""[C] Operator hand -> robot. Webcam + MediaPipe Hands, keyboard fallback, fail-safe hold.

Axis convention (GUI camera sits behind the robot): screen right == world -y, screen up == world +x.
Keyboard fallback uses i/j/k/l (xy), u/o (z), space (grip): a/q/t/r/n are taken by main.py.
"""
import time

import numpy as np

from common import WORKSPACE

GAIN = 3.0              # hand motion -> robot motion amplification around the frame center
STEP_LIMIT = 0.045      # max EE target change per frame (m): no jump when entering TELEOP
EMA = 0.3               # weight of the new sample
GRIP_ON, GRIP_OFF = 0.25, 0.40   # pinch ratio thresholds (two thresholds: no flicker)
HAND_NEAR, HAND_FAR = 0.30, 0.12 # wrist->middle-MCP length in image units: near -> low z


def clamp_ws(xyz):
    return np.array([np.clip(xyz[0], *WORKSPACE["x"]), np.clip(xyz[1], *WORKSPACE["y"]),
                     np.clip(xyz[2], *WORKSPACE["z"])])


def _amp(t):
    """0..1 -> 0..1 with GAIN around 0.5 (clamped): a small hand move covers the whole range."""
    return float(np.clip(0.5 + (t - 0.5) * GAIN, 0, 1))


def map_hand(lm9_xy, hand_size):
    """Image (mirrored, 0..1) -> world xyz in WORKSPACE."""
    u, v = _amp(lm9_xy[0]), _amp(lm9_xy[1])
    x = WORKSPACE["x"][1] - v * (WORKSPACE["x"][1] - WORKSPACE["x"][0])   # up on screen -> farther
    y = WORKSPACE["y"][1] - u * (WORKSPACE["y"][1] - WORKSPACE["y"][0])   # right on screen -> -y
    t = _amp((hand_size - HAND_FAR) / (HAND_NEAR - HAND_FAR))
    z = WORKSPACE["z"][1] - t * (WORKSPACE["z"][1] - WORKSPACE["z"][0])   # big hand -> low
    return clamp_ws((x, y, z))


class GripHysteresis:
    def __init__(self):
        self.grip = False

    def update(self, ratio):
        if self.grip and ratio > GRIP_OFF:
            self.grip = False
        elif not self.grip and ratio < GRIP_ON:
            self.grip = True
        return self.grip


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
            self.pos = None  # fail-safe: caller holds position and keeps the part
            return None, self.grip.grip, frame, (time.perf_counter() - self.t0) * 1000
        hl = res.multi_hand_landmarks[0]
        self.draw.draw_landmarks(frame, hl, self.conn)
        lm = [(q.x, q.y) for q in hl.landmark]
        dist = lambda a, b: float(np.hypot(lm[a][0] - lm[b][0], lm[a][1] - lm[b][1]))
        hand = max(dist(0, 9), 1e-6)
        grip = self.grip.update(dist(4, 8) / hand)
        raw = map_hand(lm[9], hand)
        self.pos = raw if self.pos is None else (1 - EMA) * self.pos + EMA * raw
        return self.pos, grip, frame, (time.perf_counter() - self.t0) * 1000

    def close(self):
        self.hands.close()
        self.cap.release()


class KeyboardTracker:
    """Fallback with the same read() signature. Keys come from main.py (single waitKey source)."""
    STEP = 0.03

    def __init__(self, start=(0.55, 0.0, 0.20)):
        self.pos = np.array(start, float)
        self.grip = False
        self.t0 = time.perf_counter()

    def read(self, key=-1):
        self.t0 = time.perf_counter()
        moves = {ord("i"): (1, 0, 0), ord("k"): (-1, 0, 0), ord("j"): (0, 1, 0),
                 ord("l"): (0, -1, 0), ord("u"): (0, 0, 1), ord("o"): (0, 0, -1)}
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
    if state.get("target") is None:
        state.update(target=np.array(ee_pos(robot)), cid=None, grip=False, latency_ms=0.0)
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
