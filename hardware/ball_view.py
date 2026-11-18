"""Check the ball detector on live wrist-camera frames, without a display (works over ssh). Nothing moves.

    python -m hardware.ball_view --arm right --ball white            # 3 s of detections, then logs/ball_view_right.jpg
    python -m hardware.ball_view --arm left --ball orange --seconds 10

Saves one image: the camera frame (128x96, as the pipeline sees it) with the detection circled, next to the detector mask.
The ball on an edge-on blade should show up near image x = +0.2 (run_real only accepts |x - 0.2| < 0.25).
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np

from hardware.camera import open_camera
from hardware.xlerobot import add_arm_arg, apply_arm
from perception import detector

ROOT = Path(__file__).parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", default="/dev/video4"); ap.add_argument("--cam-size", default="640x480")
    ap.add_argument("--ball", choices=sorted(detector.COLORS), default="orange")
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--out", default=None)
    add_arm_arg(ap)
    a = apply_arm(ap.parse_args(), ap)
    detector.set_color(a.ball)
    cap = open_camera(a.cam, a.cam_size)
    n = seen = 0
    t0 = time.perf_counter()
    img = None
    while time.perf_counter() - t0 < a.seconds:
        ok, f = cap.read()
        if not ok:
            continue
        img = cv2.cvtColor(cv2.resize(f, (128, 96), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
        det = detector.detect(img)
        n += 1; seen += int(det[3] > 0)
        if n % 10 == 0:
            print(f"{'BALL' if det[3] else '----'} x {det[0]:+.2f} y {det[1]:+.2f} r {det[2]:.2f}", flush=True)
    cap.release()
    if img is None:
        raise SystemExit(f"no frames from {a.cam}")
    print(f"{n} frames in {a.seconds:.0f} s ({n / a.seconds:.0f} fps), ball seen in {seen} ({100 * seen / max(n, 1):.0f}%)")
    det = detector.detect(img)
    vis = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if det[3]:
        c = (int((det[0] + 1) * 64), int((det[1] + 1) * 48))
        cv2.circle(vis, c, max(2, int(det[2] * 10)), (0, 0, 255), 1)
    cv2.line(vis, (int(1.2 * 64), 0), (int(1.2 * 64), 95), (0, 255, 0), 1)  # expected ball column (x = +0.2)
    m = cv2.cvtColor(detector.mask(img), cv2.COLOR_GRAY2BGR)
    out = Path(a.out or ROOT / f"logs/ball_view_{a.arm or 'cam'}.jpg")
    out.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(out), cv2.resize(np.hstack([vis, m]), (512, 192), interpolation=cv2.INTER_NEAREST))
    print(f"saved {out}")


if __name__ == "__main__":
    main()
