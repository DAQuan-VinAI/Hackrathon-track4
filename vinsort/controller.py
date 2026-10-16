"""[A] Arm control: IK at link 11, suction-style grasp via constraint, pick-and-place FSM."""
import time

import numpy as np
import pybullet as p

from common import APPROACH_DZ, BINS, CUBE, EE_LINK, HOME, RELEASE_Z, TABLE_Z

ARM = list(range(7))
FINGERS = (9, 10)
REST = [0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.8, 0.04, 0.04]
LOWER = [-2.9, -1.76, -2.9, -3.07, -2.9, -0.02, -2.9, 0.0, 0.0]
UPPER = [2.9, 1.76, 2.9, -0.07, 2.9, 3.75, 2.9, 0.04, 0.04]
RANGES = [u - l for l, u in zip(LOWER, UPPER)]
STEPS = 0  # simulation steps since start; cycle time in sim = steps / 240 (team-chosen, not real robot speed)


def _step(realtime):
    global STEPS
    p.stepSimulation()
    STEPS += 1
    if realtime:
        time.sleep(1 / 240)


def solve_ik(robot, xyz, yaw=0.0):
    orn = p.getQuaternionFromEuler([np.pi, 0, yaw])  # gripper pointing down
    q = p.calculateInverseKinematics(robot, EE_LINK, xyz, orn, LOWER, UPPER, RANGES, REST,
                                     maxNumIterations=100, residualThreshold=1e-4)
    return q[:7]


def _command(robot, q):
    p.setJointMotorControlArray(robot, ARM, p.POSITION_CONTROL, targetPositions=q, forces=[87] * 7)
    p.setJointMotorControlArray(robot, list(FINGERS), p.POSITION_CONTROL, targetPositions=[0.04, 0.04])


def ee_pos(robot):
    return p.getLinkState(robot, EE_LINK)[0]


def reset_home(robot):
    for j, v in enumerate(REST[:7]):
        p.resetJointState(robot, j, v)
    q = solve_ik(robot, HOME)
    for j, v in enumerate(q):
        p.resetJointState(robot, j, v)
    _command(robot, q)


def move_ee(robot, xyz, yaw=0.0, max_steps=240, tol=0.005, realtime=False, seg=0.02, steps_per_seg=4):
    """Straight Cartesian line to xyz (joint-space jumps sweep low and knock parts).
    False if not reached within max_steps: the caller decides."""
    start = np.array(ee_pos(robot))
    goal = np.array(xyz, dtype=float)
    n = max(1, int(np.ceil(np.linalg.norm(goal - start) / seg)))
    used = 0
    for k in range(1, n + 1):
        _command(robot, solve_ik(robot, start + (goal - start) * k / n, yaw))
        for _ in range(steps_per_seg):
            _step(realtime)
            used += 1
    for i in range(max(max_steps - used, 60)):
        if i % 10 == 0:  # refine IK as the arm moves (IK is local)
            _command(robot, solve_ik(robot, goal, yaw))
        _step(realtime)
        if np.linalg.norm(np.subtract(ee_pos(robot), goal)) < tol:
            return True
    return False


def servo_ee(robot, xyz, n_steps=8, realtime=False):
    """Non-blocking: set the IK target, run n_steps, return (teleop calls this every frame)."""
    _command(robot, solve_ik(robot, xyz))
    for _ in range(n_steps):
        _step(realtime)


def grasp(robot, object_ids, radius=0.04):
    """Attach the part nearest to link 11 (distance between centers, not contact query)."""
    if not object_ids:
        return None
    e_pos, e_orn = p.getLinkState(robot, EE_LINK)[:2]
    dist = lambda o: np.linalg.norm(np.subtract(p.getBasePositionAndOrientation(o)[0], e_pos))
    obj = min(object_ids, key=dist)
    if dist(obj) > radius:
        return None
    o_pos, o_orn = p.getBasePositionAndOrientation(obj)
    inv_pos, inv_orn = p.invertTransform(e_pos, e_orn)
    rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, o_pos, o_orn)  # keep pose: no jump
    cid = p.createConstraint(robot, EE_LINK, obj, -1, p.JOINT_FIXED, [0, 0, 0], rel_pos, [0, 0, 0], rel_orn)
    p.changeConstraint(cid, maxForce=100)
    return cid


def release(cid):
    if cid is not None:
        p.removeConstraint(cid)


def go_home(robot, realtime=False):
    x, y, _ = ee_pos(robot)
    move_ee(robot, (x, y, HOME[2]), max_steps=240, realtime=realtime)  # rise first: never sweep low
    return move_ee(robot, HOME, max_steps=480, realtime=realtime)


def pick_and_place(robot, det, object_ids, realtime=False):
    """HOME -> APPROACH -> DESCEND -> GRASP -> LIFT -> MOVE_BIN -> RELEASE -> HOME."""
    x, y, top = det.xyz
    top = TABLE_Z + CUBE if not np.isfinite(top) else top
    above = (x, y, top + APPROACH_DZ)
    hx, hy, hz = ee_pos(robot)
    move_ee(robot, (x, y, max(hz, RELEASE_Z + 0.1)), det.yaw, max_steps=480, realtime=realtime)  # travel high
    if not move_ee(robot, above, det.yaw, realtime=realtime):
        return False  # approach failed: nothing held, abort
    if not move_ee(robot, (x, y, top + 0.01), det.yaw, realtime=realtime):
        go_home(robot, realtime)
        return False
    cid = grasp(robot, object_ids)
    if cid is None:
        go_home(robot, realtime)
        return False
    bx, by = BINS[det.color]
    move_ee(robot, (x, y, RELEASE_Z + 0.1), det.yaw, realtime=realtime)  # LIFT: keep going even if slow
    move_ee(robot, (bx, by, RELEASE_Z + 0.1), max_steps=480, realtime=realtime)
    move_ee(robot, (bx, by, RELEASE_Z), realtime=realtime)
    release(cid)
    for _ in range(60):
        _step(realtime)
    go_home(robot, realtime)
    return True
