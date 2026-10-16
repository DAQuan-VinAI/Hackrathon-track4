"""Shared contract for the whole team. Only A edits this file.

Units: meters, radians, PyBullet world frame (x away from the robot, z up,
robot base at the origin). Images are RGB everywhere; convert to BGR only
right before cv2.imshow.
"""
from dataclasses import dataclass

COLORS = ["red", "green", "blue"]                 # 3 part types, one bin per color
CUBE = 0.04                                       # cube edge length (m)
TABLE_Z = 0.0                                     # objects rest on the ground plane, no table.urdf
WORKSPACE = dict(x=(0.35, 0.75), y=(-0.25, 0.25), z=(0.03, 0.35))  # spawn area and teleop safe box
BINS = {"red": (0.25, -0.50), "green": (0.40, -0.50), "blue": (0.55, -0.50)}  # xy, outside camera view
BIN_SIZE = 0.14                                   # inner width of a bin (m)
RELEASE_Z = 0.20                                  # drop height above a bin
EE_LINK = 11                                      # panda_grasptarget
HOME_XYZ = (0.15, 0.40, 0.45)                     # end-effector pose that keeps the arm out of the camera view

# Overhead camera: B tunes it, then frozen at 0:30
CAM = dict(eye=(0.55, 0.0, 1.0), target=(0.55, 0.0, 0.0), up=(1, 0, 0),
           fov=30, W=640, H=480, near=0.1, far=2.0)

SEEDS_NORMAL = list(range(0, 20))                 # random layout, min spacing 8 cm
SEEDS_FAILURE = list(range(100, 105))             # two same-color cubes touching

SIM_HZ = 240                                      # PyBullet default time step is 1/240 s


@dataclass
class Detection:
    color: str            # "red" | "green" | "blue"
    uv: tuple             # pixel centroid (u, v)
    xyz: tuple            # world coordinate of the top face center (m)
    yaw: float            # rotation (rad), optional for a suction gripper
    area: int             # pixel area
    suspicious: bool = False
    reason: str = ""      # "area" | "aspect" | "depth" | ""


# Teleop contract (C implements, main.py calls once per frame):
#   tracker.read(key=-1) -> (xyz or None, grip: bool, frame_bgr or None, latency_ms)
#       HandTracker ignores `key`; KeyboardTracker uses it.
#   teleop_step(robot, tracker, object_ids, state) -> state
#       main.py fills state["key"] (cv2.waitKey code, -1 if none) before each call.
#       teleop_step keeps state["cid"] (grasp constraint or None) and may set
#       state["frame"] (BGR webcam preview), state["latency_ms"], state["msg"].
#   Keys a / t / r / n / q are reserved by main.py. Keyboard teleop uses
#   i / j / k / l (move), u / o (up / down), space (grip toggle).
