"""How far do the joints really move for small commands? Commands +-deg steps from home (open loop, like the controller) and reads
the joint after --wait seconds, for each servo P gain. Torque stays on, the arm ends at home.

    python -m hardware.step_response --arm right --kp 32,48,64

The controller sends 1-2 deg per step: a joint that moves far less than commanded cannot balance (the arm sags and ignores small commands).
"""
import argparse
import json
import time

import numpy as np
from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

from hardware.xlerobot import add_arm_arg, apply_arm

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0"); ap.add_argument("--id", default="my_follower_arm")
    ap.add_argument("--home", default="hardware/home.json")
    ap.add_argument("--kp", default="32,48,64")
    ap.add_argument("--joints", default="shoulder_lift,elbow_flex,wrist_flex,wrist_roll")
    ap.add_argument("--deg", type=float, default=3.0)
    ap.add_argument("--wait", type=float, default=0.6)
    add_arm_arg(ap)
    a = apply_arm(ap.parse_args(), ap)
    home = np.array([json.load(open(a.home))[f"{j}.pos"] for j in JOINTS])
    rows = []
    for kp in [int(k) for k in a.kp.split(",")]:
        robot = SOFollower(SOFollowerRobotConfig(port=a.port, id=a.id, disable_torque_on_disconnect=False, position_p_coefficient=kp))
        bus = robot.bus
        bus.connect()
        read = lambda: np.array([bus.sync_read("Present_Position")[j] for j in JOINTS])
        send = lambda q: bus.sync_write("Goal_Position", {j: float(v) for j, v in zip(JOINTS, q)})
        try:
            send(read())
            robot.configure()  # P gain, torque on (as run_real)
            goal = home.copy()  # settle at home with sag compensation, as run_real
            for _ in range(60):
                goal = home + np.clip(goal - home + 0.25 * (home - read()), -12, 12); send(goal); time.sleep(1 / 30)
            bias = goal - home
            for jn in a.joints.split(","):
                ji = JOINTS.index(jn)
                for d in (a.deg, -a.deg):
                    off = np.zeros(5); send(home + bias); time.sleep(0.5); q0 = read()[ji]
                    off[ji] = d; send(home + bias + off); time.sleep(a.wait); q1 = read()[ji]
                    rows.append((kp, jn, d, q1 - q0))
                send(home + bias); time.sleep(0.4)
        finally:
            bus.disconnect(False)
    print(f"\ncommanded +-{a.deg:g} deg, measured after {a.wait:g} s:")
    for kp, jn, d, moved in rows:
        print(f"  P={kp:3d}  {jn:13s} {d:+.0f} -> moved {moved:+5.1f} deg ({100 * moved / d:4.0f}%)")


if __name__ == "__main__":
    main()
