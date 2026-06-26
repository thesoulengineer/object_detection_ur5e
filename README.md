# UR5e Object Hover Tracking (MuJoCo)

A UR5e arm with a RealSense D435i camera (mounted directly on the flange) **hovers
above a movable object and follows it laterally in XY** at a configurable height.
When the object leaves the configured workspace the arm retreats to its perch pose
and resumes tracking once the object returns.

This is a **closed-loop simulation**: every control step reads the object pose from
the simulator's ground truth. The hardware path uses an ArUco marker detected by
the D435i for real-world object localisation.

> **Note — pick-and-place:** The state constants `APPROACH → GRASP → CARRY → PLACE`
> are preserved as stubs in `ur5_tracking/track_states.py` for future use once a
> Robotiq 2F-85 gripper becomes available. Only `TRACK` is active.

## Repository layout

```
object_detection_ur5e/
├─ config.yaml                   # single source of truth (robot, env, object, tracking)
├─ track_and_retrieve.py         # main entry point (interactive / --auto / --headless / --hardware)
├─ calibrate_camera.py           # D435i intrinsics calibration (checkerboard)
├─ calibrate_handeye.py          # hand-eye calibration (robot poses ↔ ArUco)
├─ requirements.txt
├─ mujoco_menagerie/
│  ├─ universal_robots_ur5e/     # UR5e MJCF model (bundled)
│  ├─ realsense_d435i/           # D435i camera model (bundled)
│  └─ picknik_ur_realsense_adapter/  # camera mount adapter (bundled)
└─ ur5_tracking/                 # core package
   ├─ config_loader.py           # load + validate config.yaml
   ├─ object_scene.py            # build scene: UR5e + D435i + object + cameras
   ├─ track_control.py           # Jacobian IK solver, state readouts
   ├─ track_states.py            # hover-tracking loop + pick-and-place stubs
   ├─ auto_object.py             # scripted object motion for --auto mode
   ├─ aruco_detector.py          # threaded ArUco pose estimator (D435i)
   ├─ sim_interface.py           # MuJoCo simulation backend
   ├─ robot_interface.py         # abstract robot interface (sim + hardware)
   └─ hardware_interface.py      # real UR5e via ur-rtde + ArUco vision
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

Required: `mujoco`, `numpy`, `pyyaml`.  
Optional: `imageio` (for `--record`), `opencv-contrib-python` + `pyrealsense2` (for hardware).

## Running the simulation

```bash
# Interactive — drag the blue object with Ctrl + right-mouse-drag in the viewer
python track_and_retrieve.py

# Scripted object motion; arm hovers above as object moves
python track_and_retrieve.py --auto

# Headless (no display): prints final object position
python track_and_retrieve.py --headless --seconds 30

# Headless + record the flange camera to a video file
python track_and_retrieve.py --headless --seconds 30 --record run.mp4
```

Drag the object inside the workspace — the flange follows it at `hover_height` above.
Move it outside the workspace bounds and the arm retreats to its perch pose. Bring
it back and tracking resumes.

## Cameras & viewer controls

Two named cameras — press **Tab** (or `[` / `]`) to cycle:

| Camera | Description |
|--------|-------------|
| `overview` | Auto-frames the object; tracks it as it moves |
| `gripper_cam` | Flange-mounted D435i view, looking along the tool axis |

## Tracking behaviour

The arm runs a single continuous control loop (`TRACK`) on every tick:

- **Object inside workspace** → flange IK target = `[obj_x, obj_y, obj_z + hover_height]`  
  with a downward tool orientation. The arm chases the object laterally in XY.
- **Object outside workspace** → arm holds the perch pose (`tracking.perch_joints`)
  and waits.

No state transitions occur in normal operation. The workspace is an axis-aligned
box defined in `config.yaml` under `tracking.workspace`.

## Configuration

All tunable parameters live in `config.yaml`:

| Section | Key parameters |
|---------|---------------|
| `robot` | Joint names, limits, end-effector site |
| `camera` | D435i and adapter model paths |
| `platform`, `floor_z`, `work_surface` | Environment geometry |
| `object` | Spawn position, size, mass, colour |
| `tracking` | `perch_joints`, `hover_height`, `move_speed`, `workspace` bounds |
| `hardware` | `arm_ip`, `control_hz`, ArUco params, calibration matrices *(commented — fill in before hardware run)* |

## Hardware deployment

### 1. Install hardware dependencies
```bash
pip install ur-rtde opencv-contrib-python pyrealsense2
```

### 2. Uncomment the `hardware:` block in `config.yaml`
Set `arm_ip` to the UR5e controller's IP address.

### 3. Run calibration (in order)
```bash
# Camera intrinsics — print a 9×6 checkerboard, capture 20-30 frames
python calibrate_camera.py

# Hand-eye calibration — arm moves through 15 poses with ArUco marker in workspace
python calibrate_handeye.py
```
Both scripts write their results directly into `config.yaml`.

### 4. Verify ArUco detection
```bash
python -m ur5_tracking.aruco_detector --config config.yaml
```

### 5. Launch
```bash
python track_and_retrieve.py --hardware
```

## Design notes

1. **Hover-only, no gripper.** The Robotiq 2F-85 is not used in the current setup.
   Pick-and-place state stubs (`APPROACH`, `DESCEND`, `GRASP`, `LIFT`, `CARRY`,
   `PLACE`, `RELEASE`, `RETREAT`) are kept in `track_states.py` as documented intent
   for when a gripper becomes available.

2. **Jacobian IK.** `track_control.solve_ik()` uses a damped pseudo-inverse with
   separate position and orientation weights. The hover loop uses `pos_weight=1.0,
   ori_weight=0.3` — enough orientation bias to keep the tool pointing roughly
   downward and prevent wrist flip, but loose enough to let the arm move freely
   across the workspace.

3. **Work surface.** A static surface in front of the robot gives the object
   somewhere to rest within reach. Edit `work_surface` in `config.yaml` to match
   your physical table geometry.

4. **Eye-in-hand.** The D435i is mounted directly on the UR5e flange. The
   `ee_cam_site` MuJoCo site (12 cm along the tool axis) is used as the IK
   control point, approximating where the camera optical centre sits above the object.

## Verification

```bash
python track_and_retrieve.py --headless --seconds 30 --auto
```

Prints final object position. In a 30 s `--auto` run the object moves across the
workspace several times; the arm tracks continuously with no state transitions.
Preview images (`preview_TRACK.png`, `preview_overview.png`) show the rendered scene.
