"""Simple Tkinter slider GUI to drive the LeRobot arm's joints.

One horizontal slider per joint (5 arm joints in degrees + gripper 0-100).
Dragging a slider streams the target to the arm (throttled, non-blocking).
A telemetry line shows the measured joint angles and TCP pose.

Usage:
    python -m lerobot_sdk.joint_gui --port /dev/ttyACM0 --id my_arm

Safety:
    On launch the sliders are synced to the arm's CURRENT position and nothing is
    sent until you move a slider, so the arm won't jump. Use --max-step to clamp
    how far each command may move a joint per update.
"""

from __future__ import annotations

import argparse

from lerobot_sdk import ARM_JOINTS, GRIPPER_JOINT, LeRobotArm

JOINTS = [*ARM_JOINTS, GRIPPER_JOINT]

# Slider ranges.
ARM_MIN, ARM_MAX = -180.0, 180.0
GRIPPER_MIN, GRIPPER_MAX = 0.0, 100.0

# Timing (milliseconds).
SEND_PERIOD_MS = 60      # how often a changed target is streamed to the arm
TELEMETRY_PERIOD_MS = 400  # how often measured state is read back


def _limits(joint: str) -> tuple[float, float]:
    return (GRIPPER_MIN, GRIPPER_MAX) if joint == GRIPPER_JOINT else (ARM_MIN, ARM_MAX)


class JointGUI:
    def __init__(self, arm: LeRobotArm, *, title: str = "LeRobot Joint Control"):
        import tkinter as tk

        self.tk = tk
        self.arm = arm

        self._suppress = False  # ignore slider callbacks during programmatic sets
        self._dirty = False     # a target changed and needs streaming
        self._target: dict[str, float] = {}
        self._vars: dict[str, "tk.DoubleVar"] = {}
        self._value_labels: dict[str, "tk.Label"] = {}

        self.root = tk.Tk()
        self.root.title(title)
        self.root.minsize(560, 460)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_ui()
        self._sync_sliders_to_arm()

        # Start periodic loops.
        self.root.after(SEND_PERIOD_MS, self._tick_send)
        self.root.after(TELEMETRY_PERIOD_MS, self._tick_telemetry)

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        tk = self.tk

        header = tk.Frame(self.root, padx=12, pady=8)
        header.pack(fill="x")
        tk.Label(
            header,
            text=f"{self.arm.robot_type}  ·  id={self.arm.robot_id}  ·  port={self.arm.robot.bus.port}",
            font=("TkDefaultFont", 10, "bold"),
        ).pack(side="left")

        self.send_enabled = tk.BooleanVar(value=True)
        tk.Checkbutton(header, text="Send", variable=self.send_enabled).pack(side="right")

        body = tk.Frame(self.root, padx=12, pady=4)
        body.pack(fill="both", expand=True)
        body.columnconfigure(1, weight=1)

        for row, joint in enumerate(JOINTS):
            lo, hi = _limits(joint)
            var = tk.DoubleVar(value=0.0)
            self._vars[joint] = var
            self._target[joint] = 0.0

            unit = "" if joint == GRIPPER_JOINT else "°"
            tk.Label(body, text=joint, width=14, anchor="w").grid(
                row=row, column=0, sticky="w", pady=2
            )
            scale = tk.Scale(
                body,
                from_=lo,
                to=hi,
                orient="horizontal",
                resolution=0.5,
                showvalue=False,
                variable=var,
                command=lambda _v, j=joint: self._on_slider(j),
            )
            scale.grid(row=row, column=1, sticky="ew", padx=8, pady=2)
            value_lbl = tk.Label(body, text=f"0.0{unit}", width=8, anchor="e")
            value_lbl.grid(row=row, column=2, sticky="e")
            self._value_labels[joint] = value_lbl

        # Buttons row.
        btns = tk.Frame(self.root, padx=12, pady=8)
        btns.pack(fill="x")
        tk.Button(btns, text="Read current", command=self._sync_sliders_to_arm).pack(side="left")
        tk.Button(btns, text="Go to zero", command=self._go_zero).pack(side="left", padx=6)
        tk.Button(btns, text="Stop (hold)", command=self._stop_here).pack(side="left")

        # Save-pose row.
        save = tk.Frame(self.root, padx=12, pady=4)
        save.pack(fill="x")
        tk.Label(save, text="Save pose as:").pack(side="left")
        self.pose_name = tk.Entry(save, width=20)
        self.pose_name.pack(side="left", padx=6)
        tk.Button(save, text="Save", command=self._save_pose).pack(side="left")

        # Telemetry / status line.
        self.status = tk.Label(
            self.root, text="", anchor="w", justify="left", padx=12, pady=8, fg="#333"
        )
        self.status.pack(fill="x")

    # ------------------------------------------------------------------
    # Slider <-> arm wiring
    # ------------------------------------------------------------------
    def _on_slider(self, joint: str) -> None:
        if self._suppress:
            return
        value = float(self._vars[joint].get())
        self._target[joint] = value
        unit = "" if joint == GRIPPER_JOINT else "°"
        self._value_labels[joint].config(text=f"{value:.1f}{unit}")
        self._dirty = True

    def _set_sliders(self, values: dict[str, float]) -> None:
        """Programmatically move sliders without triggering a send."""
        self._suppress = True
        try:
            for joint, val in values.items():
                if joint not in self._vars:
                    continue
                self._vars[joint].set(val)
                self._target[joint] = float(val)
                unit = "" if joint == GRIPPER_JOINT else "°"
                self._value_labels[joint].config(text=f"{float(val):.1f}{unit}")
        finally:
            self._suppress = False

    def _sync_sliders_to_arm(self) -> None:
        try:
            current = self.arm.get_joints(include_gripper=True)
        except Exception as e:
            self._set_status(f"Read failed: {e}")
            return
        self._set_sliders(current)
        self._dirty = False  # don't re-send what we just read

    # ------------------------------------------------------------------
    # Buttons
    # ------------------------------------------------------------------
    def _go_zero(self) -> None:
        self._set_sliders({j: 0.0 for j in JOINTS})
        self._dirty = True

    def _stop_here(self) -> None:
        """Freeze: set the target to the measured position to halt motion."""
        self._sync_sliders_to_arm()

    def _save_pose(self) -> None:
        name = self.pose_name.get().strip()
        if not name:
            self._set_status("Enter a pose name before saving.")
            return
        try:
            self.arm.save_pose(name)
            self._set_status(f"Saved pose '{name}'. Library: {self.arm.list_poses()}")
        except Exception as e:
            self._set_status(f"Save failed: {e}")

    # ------------------------------------------------------------------
    # Periodic loops
    # ------------------------------------------------------------------
    def _tick_send(self) -> None:
        if self._dirty and self.send_enabled.get():
            try:
                self.arm.move_to_joints(dict(self._target), wait=False)
                self._dirty = False
            except Exception as e:
                self._set_status(f"Send failed: {e}")
        self.root.after(SEND_PERIOD_MS, self._tick_send)

    def _tick_telemetry(self) -> None:
        try:
            measured = self.arm.get_joints(include_gripper=True)
            text = "measured: " + "  ".join(
                f"{j}={measured.get(j, float('nan')):.1f}" for j in JOINTS
            )
            try:
                tcp = self.arm.get_tcp()
                text += (
                    f"\nTCP: x={tcp.x:+.3f} y={tcp.y:+.3f} z={tcp.z:+.3f}  "
                    f"rx={tcp.rx:+.2f} ry={tcp.ry:+.2f} rz={tcp.rz:+.2f}"
                )
            except Exception:
                text += "\nTCP: n/a (kinematics unavailable)"
            self._set_status(text)
        except Exception as e:
            self._set_status(f"Telemetry error: {e}")
        self.root.after(TELEMETRY_PERIOD_MS, self._tick_telemetry)

    def _set_status(self, text: str) -> None:
        self.status.config(text=text)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def _on_close(self) -> None:
        try:
            self.arm.disconnect()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Slider GUI joint controller for the LeRobot arm.")
    parser.add_argument("--port", required=True,default="/dev/ttyACM0" ,help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", dest="robot_id", default="so101_arm", help="Calibration id")
    parser.add_argument("--type", dest="robot_type", default="so101_follower")
    parser.add_argument(
        "--max-step",
        type=float,
        default=None,
        help="Per-command joint clamp (deg) for safety, e.g. 8",
    )
    args = parser.parse_args()

    try:
        import tkinter  # noqa: F401
    except ImportError:
        raise SystemExit(
            "tkinter is required for the GUI. Install it, e.g.:\n"
            "  sudo apt-get install python3-tk"
        )

    arm = LeRobotArm(
        port=args.port,
        robot_id=args.robot_id,
        robot_type=args.robot_type,
        max_relative_target=args.max_step,
    )
    arm.connect()
    try:
        JointGUI(arm).run()
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
