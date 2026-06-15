# lerobot_sdk

A small, task-oriented Python SDK for the **LeRobot SO-ARM100 / SO-101 follower
arm** (the official LeRobot hardware kit: Feetech **STS3215** bus servos driven
through the **Waveshare bus-servo adapter**).

It is a thin wrapper over the official [`lerobot`](../lerobot) package
(`SOFollower` robot + `RobotKinematics`) that exposes exactly the operations you
usually want when scripting the arm:

| Function | Description |
| --- | --- |
| `get_joints()` | Read current joint angles (degrees) + gripper opening (0–100) |
| `get_tcp()` | Read current end-effector (tool-center-point) Cartesian pose |
| `move_to_joints(...)` | Move in joint space (blocking or non-blocking) |
| `move_to_pose(...)` | Move in Cartesian space via inverse kinematics |
| `save_pose(name)` | Capture the current pose (joints + TCP) into a named library |
| `move_to_saved_pose(name)` | Replay a saved pose exactly (joint-space) |

> Contributors: see [`DEVELOPER.md`](./DEVELOPER.md) for a per-module deep dive
> into the SDK internals and the logic behind each script.

## Install

The SDK depends on the `lerobot` package. If `lerobot` is not already
installed, the SDK automatically adds the in-repo source tree
(`lerobot/src`) to `sys.path`, so it works out of the box from this repo.
For a proper install:

```bash
pip install -e ./lerobot          # the official lerobot package
pip install -r lerobot_sdk/requirements.txt
# kinematics (get_tcp / move_to_pose) need placo:
pip install placo
```

---

## Running the real robot — step by step

The arm is the LeRobot SO-ARM100/SO-101 follower: 6 Feetech **STS3215** bus
servos connected in a daisy chain to the **Waveshare bus-servo adapter**, which
plugs into your computer over USB and into the arm's power supply.

> Safety first: the arm moves under power. Keep the workspace clear, keep a hand
> near the power switch, and start with small step sizes.

### 0. Wire it up

1. Power the Waveshare adapter from the included 5 V / 12 V supply (match your
   servos; STS3215 are 12 V).
2. Connect the adapter to your PC via USB.
3. Daisy-chain all 6 servos to the adapter.

### 1. Find the serial port

```bash
# Linux
ls /dev/ttyACM* /dev/ttyUSB*
# or use lerobot's helper (unplug/replug when prompted):
lerobot-find-port
```

On Linux you may need permission for the port (then log out/in):

```bash
sudo usermod -aG dialout $USER
```

Use that port as `--port` below (e.g. `/dev/ttyACM0`). Pick a `--id` (any name,
e.g. `my_arm`); the same id is reused for calibration.

### 2. Assign motor IDs (once per arm)

Every STS3215 ships with the **same default ID (1)**, so each servo must be
given a unique ID matching its joint:

```
shoulder_pan=1  shoulder_lift=2  elbow_flex=3  wrist_flex=4  wrist_roll=5  gripper=6
```

Run the guided setup and connect **one motor at a time** when prompted:

```bash
python -m lerobot_sdk.setup_motors --port /dev/ttyACM0 --id my_arm
```

(This is equivalent to the official
`lerobot-setup-motors --robot.type=so101_follower --robot.port=/dev/ttyACM0`.)

### 3. Calibrate (once per arm)

```bash
python -m lerobot_sdk.calibrate --port /dev/ttyACM0 --id my_arm
```

You'll move the arm to the middle of its range, then sweep each joint through
its full travel. The result is saved to `<calibration_dir>/my_arm.json` and is
loaded automatically on every connect. After this, **0 deg on every joint =
the middle of its range** (the calibration/home pose).

### 4. First motion test — go to the all-zero pose

```bash
python -m lerobot_sdk.go_to_zero --port /dev/ttyACM0 --id my_arm
# gentler approach with a per-step clamp:
python -m lerobot_sdk.go_to_zero --port /dev/ttyACM0 --id my_arm --max-step 5
```

This confirms IDs, wiring, and calibration are correct.

### 5. Drive joints by hand — interactive controller

```bash
python -m lerobot_sdk.joint_controller --port /dev/ttyACM0 --id my_arm
```

Live keys: `1`–`6` select a joint, `+`/`-` (or arrow up/down) nudge it,
`[`/`]` change the step size, `z` go to zero, `r` read state, `p` print TCP
pose, `q` quit.

### 6. Script it

Use the Python API (below) for your own programs.

## Quick start

```python
from lerobot_sdk import LeRobotArm, TCPPose

with LeRobotArm(port="/dev/ttyACM0", robot_id="my_arm") as arm:
    # --- read state ---
    print(arm.get_joints())          # {'shoulder_pan': 1.2, ..., 'gripper': 40.0}
    print(arm.get_tcp())             # TCPPose(x=..., y=..., z=..., rx=..., ...)

    # --- save / replay named poses ---
    arm.save_pose("home")
    print(arm.list_poses())          # ['home']

    # --- move in joint space ---
    arm.move_to_joints({"shoulder_pan": 15.0, "elbow_flex": -20.0})
    arm.set_gripper(80)              # open gripper

    # --- move in Cartesian space (inverse kinematics) ---
    tcp = arm.get_tcp()
    target = TCPPose.from_position_rotvec(
        [tcp.x, tcp.y, tcp.z + 0.03],  # +3 cm in z
        tcp.rotvec,
    )
    arm.move_to_pose(target)

    # --- go back to a saved pose ---
    arm.move_to_saved_pose("home")
```

## Concepts

### Joints

The arm has five revolute joints plus a gripper, in this order:

```
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
```

Arm joints are reported/commanded in **degrees**. The gripper is a normalised
**0–100** opening value (0 = closed, 100 = open).

`move_to_joints` accepts either:
- a dict (partial allowed): `{"shoulder_pan": 10.0}`
- a sequence of 5 (arm only) or 6 (arm + gripper) values.

### Poses (TCP)

`TCPPose` stores position in **metres** and orientation as a **rotation vector**
(axis-angle, radians). Convenience constructors/accessors are provided:

```python
TCPPose.from_position_rotvec(pos, rotvec)
TCPPose.from_position_quat(pos, [x, y, z, w])
TCPPose.from_position_euler(pos, [roll, pitch, yaw], degrees=True)
TCPPose.from_matrix(T_4x4)

pose.as_quat()                # [x, y, z, w]
pose.as_euler(degrees=True)   # intrinsic XYZ
pose.to_matrix()              # 4x4 homogeneous transform
```

`move_to_pose` also accepts a raw `[x, y, z, rx, ry, rz]` list or a 4×4 matrix.

### Kinematics

Forward/inverse kinematics use `lerobot`'s `RobotKinematics` (the `placo`
solver) with the SO-101 URDF shipped at
`lerobot_integration/SO-ARM100/Simulation/SO101/so101_new_calib.urdf`.
Pass a different `urdf_path=` (e.g. the SO-100 URDF) to the constructor if
needed. Joint-space features work even without `placo` installed.

### Pose library

Saved poses are persisted as JSON (default: `~/.lerobot_sdk/poses.json`). Each
entry records both the joint configuration (used for exact replay) and the TCP
pose (for inspection). Override the location with
`LeRobotArm(..., pose_library_path=...)`.

## Calibration

This SDK reuses `lerobot`'s standard calibration. On first `connect()` an
uncalibrated arm will be guided through calibration, and the result is stored
under `<calibration_dir>/<robot_id>.json`. Use the same `robot_id` you used when
calibrating with the official `lerobot` tooling.

## Command-line tools

| Command | Purpose |
| --- | --- |
| `python -m lerobot_sdk.setup_motors --port ... --id ...` | Assign motor IDs (run once) |
| `python -m lerobot_sdk.calibrate --port ... --id ...` | Calibrate the arm (run once) |
| `python -m lerobot_sdk.go_to_zero --port ... --id ...` | Move to the all-zero calibration pose |
| `python -m lerobot_sdk.joint_controller --port ... --id ...` | Interactive keyboard joint control |
| `python -m lerobot_sdk.joint_gui --port ... --id ...` | Slider GUI joint control (Tkinter) |
| `python -m lerobot_sdk.example --port ... --id ...` | Full end-to-end demo |
| `python -m lerobot_sdk.example --dry-run` | Pose math only, no hardware |

The high-level helper `arm.move_to_zero()` is also available from the Python API
for moving to the calibration pose programmatically.
