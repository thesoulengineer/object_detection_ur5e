"""SimInterface: RobotInterface backed by a live MuJoCo MjData object."""
from __future__ import annotations

import mujoco
import numpy as np

from .config_loader import Config
from .robot_interface import RobotInterface



class SimInterface(RobotInterface):
    """Reads state from and writes commands to a MuJoCo MjData instance."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, cfg: Config):
        self.m = model
        self.d = data

        nid = lambda t, n: mujoco.mj_name2id(model, t, n)
        joint_names = cfg.joint_names
        self.qadr = np.array([model.jnt_qposadr[nid(mujoco.mjtObj.mjOBJ_JOINT, j)]
                               for j in joint_names])
        self.lo = cfg.arr("robot", "joint_limits_lower")
        self.hi = cfg.arr("robot", "joint_limits_upper")

        self.pinch      = nid(mujoco.mjtObj.mjOBJ_SITE,  "ee_cam_site")
        self.obj_qadr   = model.jnt_qposadr[nid(mujoco.mjtObj.mjOBJ_JOINT, "object_free")]
        self.obj_dofadr = model.jnt_dofadr [nid(mujoco.mjtObj.mjOBJ_JOINT, "object_free")]
        self.arm_act    = list(range(6))

    # -- RobotInterface -------------------------------------------------------
    def get_pinch_pos(self) -> np.ndarray:
        return self.d.site_xpos[self.pinch].copy()

    def get_arm_q(self) -> np.ndarray:
        return self.d.qpos[self.qadr].copy()

    def get_object_pos(self) -> np.ndarray:
        return self.d.qpos[self.obj_qadr:self.obj_qadr + 3].copy()

    def get_object_speed(self) -> float:
        return float(np.linalg.norm(self.d.qvel[self.obj_dofadr:self.obj_dofadr + 3]))

    def command_arm(self, q_des: np.ndarray) -> None:
        self.d.ctrl[self.arm_act] = np.clip(q_des, self.lo, self.hi)

    def get_time(self) -> float:
        return self.d.time

    # -- sim-only (not in RobotInterface) ------------------------------------
    def set_object_pose(self, pos, quat=None, vel=None) -> None:
        """Kinematically place the object (used by AutoObjectDriver and setup)."""
        a = self.obj_qadr
        self.d.qpos[a:a + 3] = pos
        if quat is not None:
            self.d.qpos[a + 3:a + 7] = quat
        v = self.obj_dofadr
        self.d.qvel[v:v + 6] = 0.0 if vel is None else np.concatenate([vel, np.zeros(3)])
