"""Four simultaneous camera tiles. Press v to hide/show. Display only; no webcam files saved."""
import time

import cv2
import numpy as np
import pybullet as p

from common import EE_LINK

W, H = 128, 96
# ponytail: four synchronized low-resolution frames cost ~40 ms on RTX 3060 (~20 FPS).
# Raising to 160x120 costs ~61 ms (~15 FPS); a separate renderer is needed for crisp 30 FPS.
MIN_PERIOD = 0.05
VIEWS = (
    ("BEN (side, do cao)", (0.50, 0.0, 0.15), 1.30, 0, -5),
    ("TAY GAP (gripper)", "ee", 0.40, -60, -35),
    ("TRUOC (front)", (0.40, 0.0, 0.15), 1.35, 90, -20),
    ("TREN (top)", (0.45, -0.10, 0.0), 1.45, -90, -89),
)


class MultiView:
    def __init__(self, robot, window="VinSort Multi-View"):
        self.robot, self.window = robot, window
        self.enabled, self.last, self.next = True, 0.0, 0
        self.tiles = [np.zeros((H, W, 3), np.uint8) for _ in VIEWS]
        self.proj = p.computeProjectionMatrixFOV(50, W / H, 0.02, 5.0)
        self.renderer = (p.ER_BULLET_HARDWARE_OPENGL if
                         p.getConnectionInfo()["connectionMethod"] == p.GUI else p.ER_TINY_RENDERER)
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 640, 480)
        self.update(force=True)  # four views synchronized from startup

    def _tile(self, i):
        name, target, dist, yaw, pitch = VIEWS[i]
        if target == "ee":
            target = p.getLinkState(self.robot, EE_LINK)[0]
        view = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2)
        rgba = p.getCameraImage(W, H, view, self.proj, renderer=self.renderer,
                                shadow=0, flags=p.ER_NO_SEGMENTATION_MASK)[2]
        img = cv2.cvtColor(np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
        cv2.rectangle(img, (0, 0), (W, 16), (30, 30, 30), -1)
        cv2.putText(img, name, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 255, 255), 1)
        return img

    def update(self, force=False, teleop=False):
        if not self.enabled:
            return
        now = time.perf_counter()
        if not force and now - self.last < MIN_PERIOD:
            return
        self.last = now
        # AUTO: all four synchronized. TELEOP: one tile per update, preserving camera responsiveness.
        indices = range(len(VIEWS)) if force or not teleop else (self.next,)
        for i in indices:
            self.tiles[i] = self._tile(i)
        if teleop:
            self.next = (self.next + 1) % len(VIEWS)
        cv2.imshow(self.window, np.vstack((np.hstack(self.tiles[:2]), np.hstack(self.tiles[2:]))))

    def toggle(self):
        self.enabled = not self.enabled
        if self.enabled:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, 640, 480)
            self.update(force=True)
        else:
            cv2.destroyWindow(self.window)
        return "4 POV" if self.enabled else "TAT"
