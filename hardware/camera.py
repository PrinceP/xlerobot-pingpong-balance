"""Open the wrist camera (default 640x480; XLeRobot uses 320x240), 30 fps. `cam` is a device path (Linux: /dev/video4) or an index ("0", "1", ...)."""
import shutil
import subprocess
import sys

import cv2


def open_camera(cam: str, size: str = "640x480") -> cv2.VideoCapture:
    w, h = (int(v) for v in size.lower().split("x"))
    src = int(cam) if str(cam).isdigit() else cam
    if sys.platform.startswith("linux") and shutil.which("v4l2-ctl"):
        # Keep 30 fps: with dynamic framerate on, auto exposure lengthens frames in room light (XLeRobot wrist cams dropped to 13 fps).
        dev = f"/dev/video{cam}" if str(cam).isdigit() else cam
        subprocess.run(["v4l2-ctl", "-d", dev, "-c", "exposure_dynamic_framerate=0"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if sys.platform.startswith("linux") else cv2.VideoCapture(src)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")); cap.set(cv2.CAP_PROP_FRAME_WIDTH, w); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h); cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)  # 1 halves the frame rate on the XLeRobot (Innomaker) cameras; the grab thread keeps the newest frame anyway
    if not cap.isOpened():
        raise SystemExit(f"cannot open camera {cam} (wrong --cam, or another program is using it?)")
    return cap
