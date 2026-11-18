"""XLeRobot support: per-arm presets (port, calibration id, wrist camera, home file) and the calibration split.

XLeRobot keeps one calibration file for both arms, the head and the wheels (`left_arm_*`, `right_arm_*`, ...). The balancing scripts use
LeRobot's SOFollower (one arm, motor IDs 1-6 on one port), so split the file once, after every XLeRobot recalibration:

    python -m hardware.xlerobot split-calib            # -> so_follower/xlerobot_left.json and xlerobot_right.json

Every hardware script takes `--arm left|right`, which fills in --port, --id, --cam and the home file. Explicit flags still win.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

ROOT = Path(__file__).parent.parent
CALIB_ROOT = Path(os.getenv("HF_LEROBOT_CALIBRATION", "~/.cache/huggingface/lerobot/calibration")).expanduser() / "robots"
XLEROBOT_CALIB = CALIB_ROOT / "xlerobot" / "xlerobot_3wheel.json"
ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Wrist cameras by USB port: all three Innomaker cameras report the same serial number, so /dev/v4l/by-id cannot tell them apart,
# and /dev/videoN can change between boots.
ARMS = {
    "left": {"port": "/dev/ttyACM0", "id": "xlerobot_left",
             "cam": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4:1.0-video-index0"},
    "right": {"port": "/dev/ttyACM1", "id": "xlerobot_right",
              "cam": "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2:1.0-video-index0"},
}
CAM_SIZE = (640, 480)  # MJPG. 4:3 like the original; these cameras have no 320x240 mode (OpenCV silently falls back to 640x360, 16:9)


def home_path(arm: str) -> Path:
    return ROOT / f"hardware/home_{arm}.json"


def add_arm_arg(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--arm", choices=sorted(ARMS), default=None,
                    help="XLeRobot arm: sets --port, --id, --cam and the home file unless you pass them")


def apply_arm(a: argparse.Namespace, ap: argparse.ArgumentParser) -> argparse.Namespace:
    """Fill in the arm preset for every option the user left at its default."""
    if not getattr(a, "arm", None):
        return a
    preset = dict(ARMS[a.arm])
    preset["home"] = str(home_path(a.arm))
    preset["save"] = str(home_path(a.arm))
    for k, v in preset.items():
        if hasattr(a, k) and getattr(a, k) == ap.get_default(k):
            setattr(a, k, v)
    if hasattr(a, "cam_size") and a.cam_size == ap.get_default("cam_size"):
        a.cam_size = "%dx%d" % CAM_SIZE
    return a


def split_calib(src: Path = XLEROBOT_CALIB, out_dir: Path = CALIB_ROOT / "so_follower") -> list[Path]:
    """Write one SOFollower calibration file per arm from the combined XLeRobot file. The values are copied unchanged, so they match
    what is already written in the motors and SOFollower does not ask to recalibrate."""
    full = json.loads(Path(src).read_text())
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []
    for arm, preset in ARMS.items():
        part = {}
        for m in ARM_MOTORS:
            c = dict(full[f"{arm}_arm_{m}"])
            if c["range_max"] - c["range_min"] > 4000 and m not in ("wrist_roll",):
                raise SystemExit(f"{arm}_arm_{m} spans the full encoder circle ({c['range_min']}..{c['range_max']}): recalibrate first")
            part[m] = c
        if [part[m]["id"] for m in ARM_MOTORS] != [1, 2, 3, 4, 5, 6]:
            raise SystemExit(f"{arm} arm motor IDs are not 1-6: {[part[m]['id'] for m in ARM_MOTORS]}")
        dst = out_dir / f"{preset['id']}.json"
        dst.write_text(json.dumps(part, indent=4))
        written.append(dst)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["split-calib"])
    ap.add_argument("--src", default=str(XLEROBOT_CALIB))
    a = ap.parse_args()
    for p in split_calib(Path(a.src)):
        print(f"wrote {p}")
        for m, c in json.loads(p.read_text()).items():
            print(f"  {m:14s} id {c['id']}  offset {c['homing_offset']:6d}  range {c['range_min']:5d} .. {c['range_max']:5d}")


if __name__ == "__main__":
    main()
