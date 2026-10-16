"""Arm control: IK, suction-like grasp, pick-and-place state machine (owner: A).

    python controller.py --seed 0     # pick every cube using engine ground truth (no perception)
"""
import argparse
import time

import numpy as np
import pybullet as p

from common import BINS, CUBE, EE_LINK, HOME_XYZ, RELEASE_Z, SIM_HZ, Detection

ARM_JOINTS = list(range(7))
ARM_FORCES = [87] * 4 + [12] * 3              # Panda joint torque limits (Nm)
MAX_JOINT_VEL = 2.0                           # rad/s, keeps the motion readable in the demo
IK_REFRESH = 20                               # re-solve IK every N steps, the arm moved since the last solve
APPROACH_DZ = 0.10                            # hover height above the top face before descending
GRASP_DZ = 0.008                              # stop this far above the top face (suction cup contact)
LIFT_DZ = 0.15

DROP_OFFSETS = [(-0.03, -0.03), (0.03, 0.03), (-0.03, 0.03), (0.03, -0.03), (0.0, 0.0)]

PHASES = ["HOME", "APPROACH", "DESCEND", "GRASP", "LIFT", "MOVE_BIN", "RELEASE"]
phase = "HOME"                                # current phase, shown on the HUD


class _Pacer:
    """Sleep so that simulated time keeps up with wall-clock time."""

    def __init__(self):
        self.t = time.perf_counter()

    def tick(self):
        self.t += 1 / SIM_HZ
        dt = self.t - time.perf_counter()
        if dt > 0:
            time.sleep(dt)
        elif dt < -0.1:                       # fell far behind (slow frame), resync instead of rushing
            self.t = time.perf_counter()


def ee_pos(robot):
    return np.array(p.getLinkState(robot, EE_LINK)[0])


def solve_ik(robot, xyz, yaw=0.0):
    # Plain damped least squares seeded from the current joint state. The null-space
    # variant (restPoses + limits) was tested: 9-23 mm error at the workspace edge vs 0.5 mm here.
    orn = p.getQuaternionFromEuler([np.pi, 0, yaw])            # gripper pointing down
    q = p.calculateInverseKinematics(robot, EE_LINK, xyz, orn,
                                     maxNumIterations=200, residualThreshold=1e-5)
    return q[:7]       # IK returns all movable joints (7 arm + 2 fingers); keep the arm only


def _set_arm_targets(robot, q):
    for j, qj, f in zip(ARM_JOINTS, q, ARM_FORCES):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=qj, force=f,
                                maxVelocity=MAX_JOINT_VEL)


def move_ee(robot, xyz, yaw=0.0, max_steps=480, tol=0.005, realtime=False, on_step=None):
    """Drive the end effector to xyz. Returns False if not within `tol` after max_steps."""
    pacer = _Pacer() if realtime else None
    for i in range(max_steps):
        if i % IK_REFRESH == 0:
            _set_arm_targets(robot, solve_ik(robot, xyz, yaw))
        p.stepSimulation()
        if pacer:
            pacer.tick()
        if on_step:
            on_step()
        if np.linalg.norm(ee_pos(robot) - xyz) < tol:
            return True
    return False       # unreachable or too slow: caller decides, never assume success


def servo_ee(robot, xyz, n_steps=8, yaw=0.0):
    """Non-blocking: set the IK target and advance n_steps (call once per teleop frame)."""
    _set_arm_targets(robot, solve_ik(robot, xyz, yaw))
    for _ in range(n_steps):
        p.stepSimulation()
    return ee_pos(robot)


def grasp(robot, object_ids, radius=0.04):
    """Attach the object closest to the EE (within `radius`). Returns a constraint id or None."""
    if not object_ids:
        return None
    ee, ee_orn = p.getLinkState(robot, EE_LINK)[:2]
    dist = lambda o: np.linalg.norm(np.subtract(p.getBasePositionAndOrientation(o)[0], ee))
    obj = min(object_ids, key=dist)
    if dist(obj) > radius:
        return None
    # keep the current relative pose so the object does not jump into the gripper
    obj_pos, obj_orn = p.getBasePositionAndOrientation(obj)
    inv_pos, inv_orn = p.invertTransform(ee, ee_orn)
    rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, obj_pos, obj_orn)
    cid = p.createConstraint(robot, EE_LINK, obj, -1, p.JOINT_FIXED, [0, 0, 0],
                             rel_pos, [0, 0, 0], rel_orn)
    p.changeConstraint(cid, maxForce=100)
    return cid


def release(cid):
    if cid is not None:
        p.removeConstraint(cid)


def go_home(robot, realtime=False, on_step=None):
    global phase
    phase = "HOME"
    return move_ee(robot, HOME_XYZ, tol=0.02, realtime=realtime, on_step=on_step)


def _wait(steps, realtime, on_step):
    pacer = _Pacer() if realtime else None
    for _ in range(steps):
        p.stepSimulation()
        if pacer:
            pacer.tick()
        if on_step:
            on_step()


def pick_and_place(robot, det: Detection, object_ids, realtime=False, on_step=None):
    """HOME -> APPROACH -> DESCEND -> GRASP -> LIFT -> MOVE_BIN -> RELEASE -> HOME.

    det.xyz is the top face center. Returns True if an object was grasped and dropped
    over BINS[det.color]. Whether it landed inside is checked by scene.in_bin().
    """
    global phase
    x, y, z_top = det.xyz
    kw = dict(realtime=realtime, on_step=on_step)

    phase = "APPROACH"
    if not move_ee(robot, (x, y, z_top + APPROACH_DZ), tol=0.01, **kw):
        go_home(robot, **kw)
        return False
    phase = "DESCEND"
    move_ee(robot, (x, y, z_top + GRASP_DZ), max_steps=240, **kw)   # may stop short; grasp() checks distance
    phase = "GRASP"
    cid = grasp(robot, object_ids)
    if cid is None:
        move_ee(robot, (x, y, z_top + APPROACH_DZ), tol=0.01, **kw)
        go_home(robot, **kw)
        return False
    phase = "LIFT"
    move_ee(robot, (x, y, z_top + LIFT_DZ), tol=0.01, **kw)
    phase = "MOVE_BIN"
    bx, by = BINS[det.color]
    n_in_bin = sum(np.hypot(*np.subtract(p.getBasePositionAndOrientation(o)[0][:2], (bx, by))) < 0.1
                   for o in object_ids)       # cubes already in this bin; the held one is still over the workspace
    dx, dy = DROP_OFFSETS[n_in_bin % len(DROP_OFFSETS)]     # spread cubes so they do not stack
    move_ee(robot, (bx + dx, by + dy, RELEASE_Z), tol=0.01, **kw)
    phase = "RELEASE"
    release(cid)
    _wait(SIM_HZ // 4, **kw)                  # let the cube fall into the bin
    go_home(robot, **kw)
    return True


if __name__ == "__main__":
    from scene import build_scene, ground_truth, in_bin

    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--direct", action="store_true")
    args = ap.parse_args()

    sc = build_scene(args.seed, gui=not args.direct)
    robot, ids = sc["robot"], [o["id"] for o in sc["objects"]]
    go_home(robot, realtime=not args.direct)
    print(f"HOME error: {np.linalg.norm(ee_pos(robot) - HOME_XYZ) * 1000:.1f} mm")
    ok = 0
    for oid, color, (x, y, z) in ground_truth(sc):
        det = Detection(color, (0, 0), (x, y, z + CUBE / 2), 0.0, 0)   # oracle: engine pose, no perception
        t0 = time.perf_counter()
        picked = pick_and_place(robot, det, ids, realtime=not args.direct)
        landed = in_bin(oid, color)
        ok += landed
        print(f"id={oid} {color:5s} picked={picked} in_bin={landed} ({time.perf_counter() - t0:.1f}s)")
    print(f"{ok}/{len(ids)} in the correct bin")
