"""HardwareInterface: RobotInterface backed by real UR5e hardware.

Arm:    ur-rtde  (RTDEControlInterface + RTDEReceiveInterface)
Vision: ArucoDetector (D435i RGB stream)

Note: Robotiq 2F-85 gripper support is stubbed out — no gripper is available
in the current lab setup.  The command_gripper / set_grasp methods are removed;
re-add them when a gripper is available.

Install dependencies:
    pip install ur-rtde pyrealsense2 opencv-contrib-python

Usage (from track_and_retrieve.py --hardware):
    iface = HardwareInterface(model, cfg)
    iface.connect()
    ...
    iface.disconnect()
"""
from __future__ import annotations

import time
from typing import Optional

import mujoco
import numpy as np

from .config_loader import Config
from .robot_interface import RobotInterface


class HardwareInterface(RobotInterface):
    """Live hardware backend — wraps ur-rtde, pyrobotiqgripper, and ArUco."""

    def __init__(self, model: mujoco.MjModel, cfg: Config) -> None:
        self.m = cfg  # keep ref for connect()
        self._model = model
        self._cfg = cfg
        self._hw = cfg.get("hardware") or {}

        # Indices into the MuJoCo model (for FK only — no simulation stepping)
        joint_names = cfg.joint_names
        nid = lambda t, n: mujoco.mj_name2id(model, t, n)
        self._qadr = np.array([
            model.jnt_qposadr[nid(mujoco.mjtObj.mjOBJ_JOINT, j)]
            for j in joint_names
        ])
        self._pinch_id = nid(mujoco.mjtObj.mjOBJ_SITE, "ee_cam_site")
        self._lo = cfg.arr("robot", "joint_limits_lower")
        self._hi = cfg.arr("robot", "joint_limits_upper")

        # Scratch MjData used for FK (never stepped, never written to sim)
        self._fk_data = mujoco.MjData(model)

        # Hardware objects (populated by connect())
        self._ctrl  = None   # RTDEControlInterface
        self._recv  = None   # RTDEReceiveInterface

        # ArUco detector (populated by connect())
        self._detector = None

        # servoJ parameters
        self._dt            = 1.0 / float(self._hw.get("control_hz", 10))
        self._arm_speed     = float(self._hw.get("arm_speed",  0.5))
        self._arm_accel     = float(self._hw.get("arm_accel",  0.3))
        self._use_servoj    = bool(self._hw.get("use_servoj",  False))

        # Fallback object position if vision drops out
        self._last_obj_pos: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open connections to arm, gripper and camera. Call before the loop."""
        arm_ip = self._hw.get("arm_ip", "192.168.1.100")

        try:
            import rtde_control
            import rtde_receive
        except ImportError as e:
            raise ImportError(
                "ur-rtde not installed. Run: pip install ur-rtde"
            ) from e

        print(f"[HW] Connecting to UR5e at {arm_ip}...")
        self._ctrl = rtde_control.RTDEControlInterface(arm_ip)
        self._recv = rtde_receive.RTDEReceiveInterface(arm_ip)
        print("[HW] Arm connected.")

        try:
            from .aruco_detector import ArucoDetector
            self._detector = ArucoDetector(self._cfg)
            self._detector.start()
            print("[HW] ArUco detector started.")
        except Exception as e:
            raise RuntimeError(f"[HW] Could not start ArUco detector: {e}") from e

    def disconnect(self) -> None:
        """Cleanly shut down all hardware connections."""
        if self._detector:
            self._detector.stop()
        if self._ctrl:
            try:
                self._ctrl.stopScript()
            except Exception:
                pass
        print("[HW] Disconnected.")

    # ------------------------------------------------------------------
    # RobotInterface implementation
    # ------------------------------------------------------------------

    def get_arm_q(self) -> np.ndarray:
        return np.array(self._recv.getActualQ(), dtype=float)

    def get_pinch_pos(self) -> np.ndarray:
        """Forward-kinematics estimate of the 2F-85 pinch site."""
        q = self.get_arm_q()
        self._fk_data.qpos[self._qadr] = q
        mujoco.mj_kinematics(self._model, self._fk_data)
        return self._fk_data.site_xpos[self._pinch_id].copy()

    def get_object_pos(self) -> np.ndarray:
        pos = self._detector.get_object_pos() if self._detector else None
        if pos is not None:
            self._last_obj_pos = pos
            return pos
        if self._last_obj_pos is not None:
            return self._last_obj_pos
        raise RuntimeError("[HW] Object not detected and no previous position available.")

    def get_object_speed(self) -> float:
        return self._detector.get_object_speed() if self._detector else 0.0

    def command_arm(self, q_des: np.ndarray) -> None:
        q = np.clip(q_des, self._lo, self._hi).tolist()
        if self._use_servoj:
            # High-rate streaming (tracking phase)
            self._ctrl.servoJ(q, self._arm_speed, self._arm_accel,
                              self._dt, lookahead_time=0.1, gain=300)
        else:
            # Blocking move (transit phases — APPROACH, CARRY, RETREAT)
            self._ctrl.moveJ(q, self._arm_speed, self._arm_accel, asynchronous=True)

    def get_time(self) -> float:
        return time.monotonic()
