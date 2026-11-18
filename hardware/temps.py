"""Read the motor temperatures and status flags of one or both XLeRobot arms. Nothing moves; torque is not touched.

    python -m hardware.temps                       # both arms, once
    python -m hardware.temps --until 50            # poll every 30 s until every motor is below 50 C (max --timeout min)

Status flag 4 = overheat cut-out (the servo drops its torque until it cools). The STS3215 trips at about 70 C.
"""
import argparse
import time

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower

from hardware.xlerobot import ARMS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=sorted(ARMS), default=None, help="default: both")
    ap.add_argument("--until", type=float, default=None, help="poll until all motors are below this many C")
    ap.add_argument("--every", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=25.0, help="minutes")
    a = ap.parse_args()
    buses = {}
    for arm in [a.arm] if a.arm else sorted(ARMS):
        p = ARMS[arm]
        bus = SOFollower(SOFollowerRobotConfig(port=p["port"], id=p["id"], disable_torque_on_disconnect=False)).bus
        bus.connect()
        buses[arm] = bus
    t0 = time.time()
    try:
        while True:
            hottest = 0
            parts = []
            for arm, bus in buses.items():
                t = bus.sync_read("Present_Temperature", normalize=False)
                st = bus.sync_read("Status", normalize=False)
                hottest = max(hottest, max(t.values()))
                flags = {m: s for m, s in st.items() if s}
                parts.append(f"{arm}: shoulder_lift {t['shoulder_lift']} elbow {t['elbow_flex']} max {max(t.values())} C"
                             + (f" STATUS {flags}" if flags else ""))
            print(f"{(time.time() - t0) / 60:4.1f} min | " + " | ".join(parts), flush=True)
            if a.until is None or hottest < a.until or time.time() - t0 > a.timeout * 60:
                break
            time.sleep(a.every)
    finally:
        for bus in buses.values():
            bus.disconnect(False)


if __name__ == "__main__":
    main()
