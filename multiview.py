"""[A] Multi-angle view: several virtual cameras rendered once into one OpenCV window (2x2 grid).

Helps the operator judge height (side view) and grasp alignment (gripper close-up) without
rotating the main PyBullet camera. Display only: never used for perception or saved to disk.
"""
import time

import cv2
import numpy as np
import pybullet as p

from common import EE_LINK

W, H = 240, 180          # window is resizable; OpenCV upscales
# Measured on Intel UHD (PyBullet GUI, hardware OpenGL): ~19 ms per 240x180 view, ~290 ms for 4 at 400x300.
# So render ONE view per update (round-robin) at most every MIN_PERIOD: control loop stays smooth.
# ponytail: each view refreshes ~2x/s; add a separate render process if a faster GPU view is needed.
MIN_PERIOD = 0.12

# name: (target or "ee", distance, yaw, pitch). Robot base at origin, parts around x=0.55.
VIEWS = {
    "TRUOC (front)": ((0.40, 0.0, 0.15), 1.35, 90, -20),
    "BEN (side, do cao)": ((0.50, 0.0, 0.15), 1.30, 0, -5),
    "TREN (top)": ((0.45, -0.10, 0.0), 1.45, -90, -89),
    "TAY GAP (gripper)": ("ee", 0.40, -60, -35),
}


class MultiView:
    def __init__(self, robot, window="VinSort Multi-View"):
        self.robot, self.window = robot, window
        self.enabled, self.last = True, 0.0
        self.tiles = [np.zeros((H, W, 3), np.uint8) for _ in VIEWS]
        self.next = 0
        self.proj = p.computeProjectionMatrixFOV(50, W / H, 0.02, 5.0)
        import os
        hw = os.environ.get("VINSORT_MV_HW", "1") == "1" and p.getConnectionInfo()["connectionMethod"] == p.GUI
        self.renderer = p.ER_BULLET_HARDWARE_OPENGL if hw else p.ER_TINY_RENDERER
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window, 640, 480)
        self.update(force=True)  # all 4 views once at startup

    def _render(self, name, target, dist, yaw, pitch):
        if target == "ee":
            target = p.getLinkState(self.robot, EE_LINK)[0]
        view = p.computeViewMatrixFromYawPitchRoll(target, dist, yaw, pitch, 0, 2)
        rgba = p.getCameraImage(W, H, view, self.proj, renderer=self.renderer)[2]
        img = cv2.cvtColor(np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8), cv2.COLOR_RGB2BGR)
        cv2.rectangle(img, (0, 0), (W, 20), (30, 30, 30), -1)
        cv2.putText(img, name, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0, 0), (W - 1, H - 1), (90, 90, 90), 1)
        return img

    def grid(self, all_views=False):
        items = list(VIEWS.items())
        todo = range(len(items)) if all_views else [self.next]
        for i in todo:
            self.tiles[i] = self._render(items[i][0], *items[i][1])
        self.next = (self.next + 1) % len(items)
        t = self.tiles
        return np.vstack([np.hstack(t[:2]), np.hstack(t[2:])])

    def update(self, force=False):
        """Refresh one view (round-robin) if enabled and not throttled. Safe to call every frame."""
        now = time.perf_counter()
        if not self.enabled or (not force and now - self.last < MIN_PERIOD):
            return
        self.last = now
        cv2.imshow(self.window, self.grid(all_views=force))

    def toggle(self):
        self.enabled = not self.enabled
        if not self.enabled:
            cv2.destroyWindow(self.window)
        else:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window, 640, 480)
            self.update(force=True)
