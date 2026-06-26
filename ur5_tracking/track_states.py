"""Hover-tracking state machine.

update() is called every control tick and writes to the robot via
TrackController.

Active state
------------
  TRACK : hover the flange at a fixed height above the object and follow it
          laterally in XY.  If the object is outside the configured workspace
          the arm retreats to the perch pose and waits.

Stubbed states (gripper not available in current lab setup)
-----------------------------------------------------------
  APPROACH, DESCEND, GRASP, LIFT, CARRY, PLACE, RELEASE, RETREAT

These states are preserved here as documented intent for when a Robotiq 2F-85
gripper becomes available.  They are never entered — update() only dispatches
to TRACK.
"""
from __future__ import annotations

import mujoco
import numpy as np

TRACK, APPROACH, DESCEND, GRASP, LIFT, CARRY, PLACE, RELEASE, RETREAT = (
    "TRACK", "APPROACH", "DESCEND", "GRASP", "LIFT", "CARRY", "PLACE", "RELEASE", "RETREAT")

_DOWN = np.array([0.0, 0.0, -1.0])


def _in_workspace(pos: np.ndarray, ws: dict) -> bool:
    """Return True if pos is inside the axis-aligned workspace box."""
    for axis, idx in (("x", 0), ("y", 1), ("z", 2)):
        bounds = ws.get(axis)
        if bounds is not None:
            if pos[idx] < bounds[0] or pos[idx] > bounds[1]:
                return False
    return True


class RetrieveFSM:
    def __init__(self, ctl, cfg):
        self.ctl = ctl
        self.state = TRACK

        self.perch = cfg.arr("tracking", "perch_joints")
        self.hover_height = float(cfg.get("tracking", "hover_height", default=0.20))
        self.workspace = cfg.get("tracking", "workspace") or {}

        # Pick-and-place parameters kept for future use when a gripper is available.
        self.hover    = float(cfg.get("pick", "hover_height",    default=0.14))
        self.tol      = float(cfg.get("pick", "reach_tol",       default=0.015))
        self.grip_settle = float(cfg.get("pick", "gripper_settle_s", default=0.6))
        self.coarse_tol  = max(self.tol * 3.0, 0.03)
        self.stall_time  = 0.8
        home_cfg = cfg.get("home_return") or {}
        self.home = np.asarray(home_cfg.get("position", [0.0, 0.0, 0.0]), dtype=float)

        # Perch pinch position (used to hold the arm when object is out of workspace)
        sd = ctl.scratch
        sd.qpos[:] = 0.0
        sd.qpos[ctl.qadr] = self.perch
        mujoco.mj_kinematics(ctl.m, sd)
        self.perch_pinch = sd.site_xpos[ctl.pinch].copy()

        self._phase_t0 = 0.0
        self._best_dist = np.inf
        self._improve_t = 0.0

    # -- helpers --------------------------------------------------------------
    def _reached(self, target):
        return np.linalg.norm(self.ctl.pinch_pos() - target) < self.tol

    def _enter(self, state):
        self.state = state
        self._phase_t0 = self.ctl.get_time()
        self._best_dist = np.inf
        self._improve_t = self.ctl.get_time()

    def _timeout(self, limit=8.0):
        return (self.ctl.get_time() - self._phase_t0) > limit

    # -- main dispatch --------------------------------------------------------
    def update(self):
        # Only TRACK is active. The pick-and-place states below are stubs
        # preserved for future gripper integration.
        return self._track()

    # -- active phase ---------------------------------------------------------
    def _track(self):
        ctl = self.ctl
        obj = ctl.object_pos()

        if self.workspace and not _in_workspace(obj, self.workspace):
            # Object outside reachable workspace — hold the perch pose.
            q_des = ctl.solve_ik(self.perch_pinch, _DOWN, ctl.arm_q(),
                                 pos_weight=1.0, ori_weight=0.3, iters=60)
            ctl.command_arm(q_des)
            return self.state

        # Object is in workspace — hover directly above it.
        target = np.array([obj[0], obj[1], obj[2] + self.hover_height])
        q_des = ctl.solve_ik(target, _DOWN, ctl.arm_q(),
                             pos_weight=1.0, ori_weight=0.3, iters=60)
        ctl.command_arm(q_des)
        return self.state

    # -- stubbed pick-and-place phases (gripper not available) ----------------

    def _goto(self, target, closed, after, on_done=None):
        """Move to Cartesian target. Stub — requires gripper."""
        raise NotImplementedError("Gripper not available; pick-and-place disabled.")

    def _grip(self, close, weld, after):
        """Open/close gripper and toggle weld. Stub — requires gripper."""
        raise NotImplementedError("Gripper not available; pick-and-place disabled.")

    def _reset_track(self):
        self.state = TRACK
