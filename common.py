# common.py  (contract shared by every module; only A edits)
from dataclasses import dataclass

COLORS = ["red", "green", "blue"]                 # 3 part types, one bin per color
RGBA = {"red": (1, 0, 0, 1), "green": (0, 1, 0, 1), "blue": (0, 0, 1, 1)}
CUBE = 0.04                                       # cube edge length (m)
TABLE_Z = 0.0                                     # objects rest on the ground plane, no table.urdf
WORKSPACE = dict(x=(0.35, 0.75), y=(-0.25, 0.25), z=(0.03, 0.35))  # teleop safe box
SPAWN = dict(x=(0.40, 0.68), y=(-0.22, 0.22))     # spawn area (inside camera view and reach)
BINS = {"red": (0.25, -0.50), "green": (0.40, -0.50), "blue": (0.55, -0.50)}  # xy, outside camera view
BIN_HALF = 0.07                                   # inner half-size of a bin (m)
RELEASE_Z = 0.20                                  # drop height above a bin
EE_LINK = 11                                      # panda_grasptarget
HOME = (0.15, 0.40, 0.45)                         # EE home, outside camera view
APPROACH_DZ = 0.10                                # approach height above the part top

# Overhead camera: frozen
CAM = dict(eye=(0.55, 0.0, 1.0), target=(0.55, 0.0, 0.0), up=(1, 0, 0),
           fov=30, W=640, H=480, near=0.1, far=2.0)

SEEDS_NORMAL = list(range(0, 20))                 # random layout, min spacing 8 cm
SEEDS_FAILURE = list(range(100, 105))             # two same-color cubes touching


@dataclass
class Detection:
    color: str            # "red" | "green" | "blue"
    uv: tuple             # pixel centroid (u, v)
    xyz: tuple            # world coordinate of the top face center (m)
    yaw: float            # rotation (rad)
    area: int             # pixel area
    suspicious: bool = False
    reason: str = ""      # "area" | "aspect" | "depth" | "grasp_fail" | ""
