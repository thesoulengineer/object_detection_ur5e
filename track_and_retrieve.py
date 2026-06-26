#!/usr/bin/env python3
"""UR5e hover tracking — simulation and hardware.

The arm's flange hovers at a fixed height above a movable object and follows
it laterally in XY.  When the object leaves the configured workspace the arm
retreats to its perch pose and waits; it resumes tracking once the object
re-enters the workspace.

Pick-and-place (APPROACH → GRASP → CARRY → PLACE) is stubbed in track_states.py
for future use when a Robotiq 2F-85 gripper becomes available.

Simulation modes:
  python track_and_retrieve.py                              # interactive (drag the object)
  python track_and_retrieve.py --auto                       # object moves automatically
  python track_and_retrieve.py --headless --seconds 30      # headless verification
  python track_and_retrieve.py --headless --seconds 30 --record out.mp4

Hardware mode (requires ur-rtde, pyrealsense2, opencv-contrib-python):
  python track_and_retrieve.py --hardware
"""
from __future__ import annotations

import argparse
import sys
import time

import mujoco
import numpy as np

from ur5_tracking.config_loader import load_config
from ur5_tracking.object_scene import build_model
from ur5_tracking.sim_interface import SimInterface
from ur5_tracking.hardware_interface import HardwareInterface
from ur5_tracking.track_control import TrackController
from ur5_tracking.track_states import RetrieveFSM
from ur5_tracking.auto_object import AutoObjectDriver

CONTROL_EVERY = 5   # run the controller every N physics steps


def setup_sim(cfg):
    model, _ = build_model(cfg)
    data = mujoco.MjData(model)
    iface = SimInterface(model, data, cfg)
    ctl = TrackController(model, cfg, iface)
    perch = cfg.arr("tracking", "perch_joints")
    data.qpos[iface.qadr] = perch
    data.ctrl[iface.arm_act] = perch
    iface.set_object_pose(cfg.arr("object", "spawn"))
    mujoco.mj_forward(model, data)
    return model, data, ctl, RetrieveFSM(ctl, cfg)


def run_headless(cfg, seconds, record=None):
    model, data, ctl, tracker = setup_sim(cfg)
    driver = AutoObjectDriver(ctl, cfg)
    dt = model.opt.timestep
    renderer = mujoco.Renderer(model, height=480, width=640) if record else None
    frames = []

    for i in range(int(seconds / dt)):
        if i % CONTROL_EVERY == 0:
            tracker.update()
            driver.update(tracker.state, ctl.get_time(), dt * CONTROL_EVERY)
        mujoco.mj_step(model, data)
        if renderer and i % 20 == 0:
            renderer.update_scene(data, camera="gripper_cam")
            frames.append(renderer.render().copy())

    print(f"Run complete ({seconds}s). Final object pos: {ctl.object_pos().round(3)}")
    if record and frames:
        try:
            import imageio
            imageio.mimsave(record, frames, fps=30)
            print("Saved video:", record)
        except ImportError:
            print("(imageio not installed; skipping video)")


def run_viewer(cfg, auto):
    import mujoco.viewer
    model, data, ctl, tracker = setup_sim(cfg)
    driver = AutoObjectDriver(ctl, cfg) if auto else None
    dt = model.opt.timestep
    with mujoco.viewer.launch_passive(model, data) as v:
        i = 0
        while v.is_running():
            t0 = time.time()
            if i % CONTROL_EVERY == 0:
                tracker.update()
                if driver:
                    driver.update(tracker.state, ctl.get_time(), dt * CONTROL_EVERY)
            mujoco.mj_step(model, data)
            v.sync()
            i += 1
            lag = dt - (time.time() - t0)
            if lag > 0:
                time.sleep(lag)


def run_hardware(cfg) -> None:
    """Hardware control loop — connects to real UR5e and D435i."""
    model, _ = build_model(cfg)
    iface = HardwareInterface(model, cfg)
    iface.connect()
    ctl = TrackController(model, cfg, iface)
    tracker = RetrieveFSM(ctl, cfg)
    dt = 1.0 / float(cfg.get("hardware", "control_hz", default=10))
    print("[HW] Starting hover-tracking loop. Ctrl-C to stop.")
    try:
        while True:
            t0 = time.monotonic()
            tracker.update()
            elapsed = time.monotonic() - t0
            time.sleep(max(0.0, dt - elapsed))
    except KeyboardInterrupt:
        print("\n[HW] Interrupted.")
    finally:
        iface.disconnect()


def main():
    ap = argparse.ArgumentParser(description="UR5e hover tracking")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--hardware", action="store_true",
                    help="run on real hardware (requires ur-rtde, pyrealsense2)")
    ap.add_argument("--auto", action="store_true", help="(sim) move the object automatically")
    ap.add_argument("--headless", action="store_true", help="(sim) run without a display")
    ap.add_argument("--seconds", type=float, default=25.0, help="(sim) headless run duration")
    ap.add_argument("--record", default=None, help="(sim) save gripper-camera video (mp4)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.hardware:
        run_hardware(cfg)
    elif args.headless:
        run_headless(cfg, args.seconds, args.record)
    else:
        run_viewer(cfg, auto=args.auto)
    return 0


if __name__ == "__main__":
    sys.exit(main())
