# UR5e Object Tracking & Retrieval (MuJoCo)

A UR5e arm with a Robotiq 2F-85 gripper **visually tracks a movable object** —
the gripper-mounted camera follows it like a snake's head. When the object is
**left stationary**, the arm **picks it up and returns it to its original
position**, then resumes tracking.

This is a **closed-loop simulation**: every control step reads the object pose
from the simulator's ground truth. A RealSense D435i is modelled on the gripper
for hand-eye calibration and ArUco-based detection workflows.

## Repository layout

```
object_detection_ur5e/
├─ config.yaml                   # single source of truth (robot, env, object, behaviour)
├─ track_and_retrieve.py         # main entry point (interactive / --auto / --headless)
├─ calibrate_camera.py           # intrinsic camera calibration helper
├─ calibrate_handeye.py          # hand-eye calibration (robot ↔ camera)
├─ requirements.txt
├─ mujoco_menagerie/
│  ├─ universal_robots_ur5e/     # UR5e MJCF model (bundled)
│  ├─ robotiq_2f85/              # 2F-85 gripper model (bundled)
│  ├─ realsense_d435i/           # D435i camera model (bundled)
│  └─ picknik_ur_realsense_adapter/  # camera mount adapter (bundled)
└─ ur5_tracking/                 # the core package
   ├─ config_loader.py           # load + validate config.yaml
   ├─ object_scene.py            # build scene: UR5e + gripper + camera + object
   ├─ track_control.py           # state readouts, gaze/reach IK, grasp weld
   ├─ track_states.py            # finite state machine: track → pick → return
   ├─ auto_object.py             # scripted object motion for --auto mode
   ├─ aruco_detector.py          # ArUco marker detection via the D435i
   ├─ sim_interface.py           # MuJoCo simulation interface
   ├─ robot_interface.py         # robot command interface (sim + real)
   └─ hardware_interface.py      # real-hardware UR5e communication layer
```

## Installation

```bash
python -m venv .venv
# Linux / macOS
source .venv/bin/activate
# Windows PowerShell
.\.venv\Scripts\Activate.ps1

pip install -r requirements.txt
```

Required packages: `mujoco`, `numpy`, `pyyaml`.  
Optional: `imageio` (for `--record`), `opencv-python` (for ArUco detection).

## Running the simulation

```bash
# Interactive — drag the blue object with Ctrl + right-mouse-drag in the viewer
python track_and_retrieve.py

# Scripted motion: object moves on its own, arm retrieves it; repeats
python track_and_retrieve.py --auto

# Headless (no display): prints state transitions + placement accuracy
python track_and_retrieve.py --headless --seconds 30

# Headless + record the gripper camera to a video file
python track_and_retrieve.py --headless --seconds 30 --record run.mp4
```

In the interactive viewer, drag the object around — the arm aims at it. Release
it; once stationary for ~1.5 s and away from home, the arm picks it up and
places it back on the green home marker.

## Cameras & viewer controls

The scene includes a skybox, checkered floor, wooden work table, and soft
shadows. Two named cameras are available — press **Tab** (or `[` / `]`) to cycle:

| Camera | Description |
|--------|-------------|
| `overview` | Auto-frames the object; stays pointed at it as it moves |
| `gripper_cam` | Gripper's-eye view, looking along the tool approach axis |

The free camera remains available for orbiting, panning, and zooming with the
mouse at any time.

## Calibration utilities

```bash
# Collect images and compute D435i intrinsics
python calibrate_camera.py

# Run hand-eye calibration (robot poses ↔ ArUco detections)
python calibrate_handeye.py
```

## State machine

`ur5_tracking/track_states.py` implements the following cycle:

```
TRACK → APPROACH → DESCEND → GRASP → LIFT → CARRY → PLACE → RELEASE → RETREAT → TRACK
```

| State | Behaviour |
|-------|-----------|
| **TRACK** | Holds the perch pose; re-orients so the camera points at the object. Starts retrieve when object is idle for `tracking.idle_time` and is not at home. |
| **APPROACH / DESCEND** | IK-based motion toward the grasp pose. |
| **GRASP / RELEASE** | Closes/opens the gripper and enables/disables the weld constraint. |
| **LIFT / CARRY / PLACE** | Moves the object from grasp position to the home marker. |
| **RETREAT** | Returns to the perch pose before resuming tracking. |

Each phase advances when the target is reached, when progress stalls, or after
an 8 s safety timeout — the arm never deadlocks.

## Configuration

All tunable parameters live in `config.yaml`:

- `robot` — joint names, limits, end-effector site
- `gripper` — open/close control values
- `camera` — D435i and adapter paths
- `platform`, `floor_z`, `work_surface` — environment geometry
- `object` — initial pose, size, colour
- `tracking` — idle speed/time thresholds, gaze gain
- `pick` — approach offset, lift height, grasp tolerance
- `home_return` — home position and placement tolerance

## Design notes

1. **Weld-based grasp.** The 2F-85 visibly closes, but the hold is guaranteed by
   a MuJoCo weld equality constraint enabled at grasp time. Pure contact grasping
   is fragile in simulation (slip, closing force, friction tuning); the weld keeps
   the demo reliable. To experiment with contact grasping, disable the weld in
   `track_control.set_grasp` and tune gripper friction/force.

2. **Work surface.** The main platform under the base is too narrow in +y for the
   object to rest on. A static surface in front of the robot (within reach) is
   added as the resting area. Edit `work_surface` in `config.yaml` to match your
   real setup.

3. **Gaze control.** The arm keeps its perch position and only changes orientation
   to keep the camera pointed at the object — snake-like head tracking while the
   body stays roughly put.

4. **Hardware layer.** `hardware_interface.py` and `robot_interface.py` provide a
   path toward deploying on a physical UR5e. The simulation interface
   (`sim_interface.py`) exposes the same API so the control logic is robot-agnostic.

## Verification

`--headless` reports state transitions, completed retrieve cycles, and placement
error at each release. In a 30 s `--auto` run, four cycles typically complete
with placement errors of ~2–5 mm (mean ~3 mm).

Preview images (`preview_*.png`) in the repo root show the rendered scene at key
states (TRACK, DESCEND, CARRY).
