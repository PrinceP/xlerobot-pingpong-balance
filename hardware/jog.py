"""Move joints slowly to a target, in degrees (gripper in % open). Torque is turned on holding the current pose first; it stays on.

    python -m hardware.jog --arm right wrist_flex=-12             # absolute
    python -m hardware.jog --arm right --rel wrist_roll=+90         # relative to now
    python -m hardware.jog --arm right                              # only print the joint readings

Stops early and holds where it is if a joint falls behind its command by more than --max-lag degrees (blocked by something).
"""
import argparse
import time

import numpy as np
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

from hardware.xlerobot import add_arm_arg, apply_arm

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("targets", nargs="*", help="joint=value pairs")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", default="my_follower_arm")
    ap.add_argument("--rel", action="store_true", help="values are offsets from the current reading")
    ap.add_argument("--speed", type=float, default=20.0, help="degrees per second")
    ap.add_argument("--max-lag", type=float, default=15.0)
    add_arm_arg(ap)
    a = apply_arm(ap.parse_args(), ap)

    robot = SOFollower(SOFollowerRobotConfig(port=a.port, id=a.id, disable_torque_on_disconnect=False))
    bus = robot.bus
    bus.connect()
    read = lambda: bus.sync_read("Present_Position")
    try:
        q0 = read()
        print("now   " + "  ".join(f"{j} {q0[j]:+.1f}" for j in JOINTS))
        if not a.targets:
            return
        tgt = dict(q0)
        for item in a.targets:
            j, v = item.split("=")
            if j not in JOINTS:
                raise SystemExit(f"unknown joint {j}; one of {JOINTS}")
            tgt[j] = q0[j] + float(v) if a.rel else float(v)
        bus.sync_write("Goal_Position", {j: float(q0[j]) for j in JOINTS})  # goal = pose: enabling torque does not jump
        bus.enable_torque(num_retry=5)
        dist = max(abs(tgt[j] - q0[j]) for j in JOINTS)
        steps = max(1, int(dist / a.speed * 30))
        for s in np.linspace(0, 1, steps + 1)[1:]:
            cmd = {j: float(q0[j] + s * (tgt[j] - q0[j])) for j in JOINTS}
            bus.sync_write("Goal_Position", cmd)
            time.sleep(1 / 30)
            q = read()
            lag = {j: cmd[j] - q[j] for j in JOINTS if j != "gripper"}
            worst = max(lag, key=lambda j: abs(lag[j]))
            if abs(lag[worst]) > a.max_lag:
                bus.sync_write("Goal_Position", {j: float(q[j]) for j in JOINTS})
                print(f"STOPPED: {worst} is {lag[worst]:+.1f} deg behind its command (blocked?). Holding here.")
                break
        time.sleep(0.5)
        q = read()
        print("now   " + "  ".join(f"{j} {q[j]:+.1f}" for j in JOINTS))
    finally:
        bus.disconnect(False)  # torque stays on


if __name__ == "__main__":
    main()
