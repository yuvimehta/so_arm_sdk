"""Interactive joint controller for the LeRobot arm.

Directly drive each joint from the keyboard in real time. Works in two modes:

* **Live mode** (a real terminal): single-keypress control, no ENTER needed.
* **REPL fallback** (piped/non-tty stdin): type commands followed by ENTER.

Usage:
    python -m lerobot_sdk.joint_controller --port /dev/ttyACM0 --id my_arm

Live keys:
    1..6        select active joint (1-5 = arm, 6 = gripper)
    + / =       increase active joint by one step
    - / _       decrease active joint by one step
    w / s       same as + / -
    [ / ]       decrease / increase the step size
    z           move all joints to 0 (calibration pose)
    r           re-read and print current joint state
    p           print current TCP pose (needs kinematics)
    ? / h       show help
    q / Ctrl-C  quit

REPL commands (one per line):
    <joint> <value>     set a joint, e.g. `shoulder_pan 15`  or  `3 -20`
    g <value>           set gripper opening (0-100)
    z                   go to zero
    r                   read state
    p                   print TCP pose
    q                   quit
"""

from __future__ import annotations

import argparse
import sys

from so_arm_sdk import ARM_JOINTS, GRIPPER_JOINT, LeRobotArm

JOINTS = [*ARM_JOINTS, GRIPPER_JOINT]
ARM_MIN, ARM_MAX = -180.0, 180.0
GRIPPER_MIN, GRIPPER_MAX = 0.0, 100.0


def _clamp(joint: str, value: float) -> float:
    if joint == GRIPPER_JOINT:
        return max(GRIPPER_MIN, min(GRIPPER_MAX, value))
    return max(ARM_MIN, min(ARM_MAX, value))


class JointController:
    def __init__(self, arm: LeRobotArm, step: float = 5.0):
        self.arm = arm
        self.step = step
        self.active = 0  # index into JOINTS
        self.target = arm.get_joints(include_gripper=True)
        # Ensure every joint has an entry.
        for j in JOINTS:
            self.target.setdefault(j, 0.0)

    # -- rendering ----------------------------------------------------------
    def _status_line(self) -> str:
        parts = []
        for i, j in enumerate(JOINTS):
            marker = ">" if i == self.active else " "
            parts.append(f"{marker}{i + 1}:{j}={self.target[j]:7.1f}")
        return " | ".join(parts) + f"   [step={self.step:g}]"

    def print_state(self) -> None:
        current = self.arm.get_joints(include_gripper=True)
        print("\n  measured:", {k: round(v, 1) for k, v in current.items()})

    def print_tcp(self) -> None:
        try:
            print("\n  tcp:", self.arm.get_tcp())
        except Exception as e:  # kinematics may be unavailable
            print("\n  tcp unavailable:", e)

    # -- actions ------------------------------------------------------------
    def _send(self) -> None:
        self.arm.move_to_joints(dict(self.target), wait=False)

    def nudge(self, delta: float) -> None:
        j = JOINTS[self.active]
        self.target[j] = _clamp(j, self.target[j] + delta)
        self._send()

    def set_joint(self, joint: str, value: float) -> None:
        self.target[joint] = _clamp(joint, value)
        self._send()

    def go_zero(self) -> None:
        for j in JOINTS:
            self.target[j] = 0.0
        self._send()

    # -- live (single-keypress) loop ---------------------------------------
    def run_live(self) -> None:
        import termios
        import tty

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        _print_help()
        print(self._status_line(), end="\r", flush=True)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch in ("q", "\x03"):  # q or Ctrl-C
                    break
                self._handle_key(ch)
                print(self._status_line() + "   ", end="\r", flush=True)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            print()

    def _handle_key(self, ch: str) -> None:
        if ch in "123456":
            self.active = int(ch) - 1
        elif ch in ("+", "=", "w"):
            self.nudge(self.step)
        elif ch in ("-", "_", "s"):
            self.nudge(-self.step)
        elif ch == "[":
            self.step = max(0.5, self.step / 2)
        elif ch == "]":
            self.step = min(45.0, self.step * 2)
        elif ch == "z":
            self.go_zero()
        elif ch == "r":
            self.print_state()
        elif ch == "p":
            self.print_tcp()
        elif ch in ("?", "h"):
            _print_help()
        elif ch == "\x1b":  # arrow-key escape sequence: ESC [ A/B
            seq = sys.stdin.read(2)
            if seq == "[A":
                self.nudge(self.step)
            elif seq == "[B":
                self.nudge(-self.step)

    # -- REPL fallback ------------------------------------------------------
    def run_repl(self) -> None:
        print("REPL mode. Type `?` for help, `q` to quit.")
        for raw in sys.stdin:
            line = raw.strip()
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            self._handle_command(line)

    def _handle_command(self, line: str) -> None:
        tokens = line.split()
        head = tokens[0].lower()
        if head in ("?", "h", "help"):
            print(__doc__)
            return
        if head == "z":
            self.go_zero()
            print("-> zero")
            return
        if head == "r":
            self.print_state()
            return
        if head == "p":
            self.print_tcp()
            return
        if head == "g" and len(tokens) == 2:
            self.set_joint(GRIPPER_JOINT, float(tokens[1]))
            print(f"-> gripper={self.target[GRIPPER_JOINT]:.1f}")
            return
        if len(tokens) == 2:
            joint = self._resolve_joint(tokens[0])
            if joint is None:
                print(f"Unknown joint: {tokens[0]}")
                return
            try:
                value = float(tokens[1])
            except ValueError:
                print(f"Invalid value: {tokens[1]}")
                return
            self.set_joint(joint, value)
            print(f"-> {joint}={self.target[joint]:.1f}")
            return
        print("Could not parse. Type `?` for help.")

    @staticmethod
    def _resolve_joint(token: str) -> str | None:
        if token.isdigit():
            idx = int(token) - 1
            return JOINTS[idx] if 0 <= idx < len(JOINTS) else None
        return token if token in JOINTS else None


def _print_help() -> None:
    print(
        "\nJoint controller (live). Keys: 1-6 select | +/- nudge | [ ] step | "
        "z zero | r read | p tcp | q quit\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive joint controller.")
    parser.add_argument("--port", required=True,defualt="/dev/ttyACM0" ,help="Serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--id", dest="robot_id", default="my_arm", help="Calibration id")
    parser.add_argument("--type", dest="robot_type", default="so101_follower")
    parser.add_argument("--step", type=float, default=5.0, help="Initial step size (deg)")
    args = parser.parse_args()

    arm = LeRobotArm(port=args.port, robot_id=args.robot_id, robot_type=args.robot_type)
    with arm:
        controller = JointController(arm, step=args.step)
        if sys.stdin.isatty():
            controller.run_live()
        else:
            controller.run_repl()
    print("Disconnected.")


if __name__ == "__main__":
    main()
